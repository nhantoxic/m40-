#!/usr/bin/env python3
"""PC client for the ESP32-S2 Dreame bridge: CLI, raw terminal and MCP server.

The bridge firmware defines ONE command language, used identically by the web
app, this CLI, AI agents (MCP) and USB. Any command the firmware knows is
passed through unchanged:

    python bridge_tool.py help                          # list firmware commands
    python bridge_tool.py status
    python bridge_tool.py wifi.scan
    python bridge_tool.py wifi.set "My Wi-Fi" "password123"
    python bridge_tool.py uart.baud 115200
    python bridge_tool.py uart.xfer 800 "info -a\\r\\n"   # send, return the reply
    python bridge_tool.py uart.send "ver -t\\r\\n"
    python bridge_tool.py uart.read 500
    python bridge_tool.py uart.sendhex "3C 00 01 3E"
    python bridge_tool.py swd ID
    python bridge_tool.py swd READ 0x08000000 64

Local commands (run on the PC):

    discover     list bridges on the LAN (UDP 2326)
    term         interactive raw terminal on tcp/2324
    app          open the web app in a browser
    mcp          run as an MCP server on stdio (for AI agents)

Output is one JSON object per command (--pretty to indent, --text to print only
the received UART text). Exit status: 0 when "ok" is true, 1 otherwise.

Bridge address: --host, else $BRIDGE_HOST, else the last bridge used, else
dreame-bridge.local, else UDP discovery, else the setup AP (192.168.4.1).
--usb COM8 sends the same commands over the USB cable instead (needs
pyserial; handy for the first Wi-Fi setup).
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Callable, Optional

VERSION = "2.0"
DISCOVERY_PORT = 2326
DISCOVERY_QUERY = b"DREAME_BRIDGE_DISCOVER"
MDNS_NAME = "dreame-bridge.local"
SETUP_AP_IP = "192.168.4.1"
UART_PORT = 2324
CMD_TIMEOUT = 45.0
CACHE_FILE = Path.home() / ".dreame_bridge_host"

# Commands whose last argument is raw data (escapes like \r\n are interpreted
# by the firmware), so backslashes must be passed through untouched.
DATA_COMMANDS = {"uart.send", "uart.sendhex", "uart.xfer", "swd"}


class BridgeError(Exception):
    pass


# --------------------------------------------------------------------------- command line building

def quote_token(arg: str) -> str:
    """Quote for the firmware tokenizer (wifi.set etc.): \\ and " escaped."""
    if arg and not any(c in arg for c in ' \t"\\'):
        return arg
    return '"' + arg.replace("\\", "\\\\").replace('"', '\\"') + '"'


def quote_data(arg: str) -> str:
    """Quote a data argument: keep \\r \\n \\xHH escapes for the firmware."""
    if arg and not any(c in arg for c in ' \t"') and not (arg.startswith('"') and arg.endswith('"')):
        return arg
    return '"' + arg.replace('"', '\\"') + '"'


def build_line(argv: list[str]) -> str:
    """['wifi.set', 'My Wi-Fi', 'pw'] -> 'wifi.set "My Wi-Fi" pw'."""
    if not argv:
        raise BridgeError("empty command")
    name, args = argv[0], argv[1:]
    if name in DATA_COMMANDS:
        if name == "uart.xfer" and args:
            return " ".join([name, args[0]] + ([quote_data(" ".join(args[1:]))] if args[1:] else []))
        if name == "swd":
            return " ".join([name] + args)
        return " ".join([name] + ([quote_data(" ".join(args))] if args else []))
    return " ".join([name] + [quote_token(a) for a in args])


# --------------------------------------------------------------------------- discovery

def discover(timeout: float = 1.5, target: str = "255.255.255.255") -> list[dict]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.2)
    found: dict[str, dict] = {}
    try:
        sock.sendto(DISCOVERY_QUERY, (target, DISCOVERY_PORT))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                info = json.loads(data.decode("utf-8", "replace"))
            except ValueError:
                continue
            if info.get("ip") in (None, "", "0.0.0.0"):
                info["ip"] = addr[0]
            found[info.get("mac", addr[0])] = info
    except OSError:
        pass
    finally:
        sock.close()
    return list(found.values())


def _reachable(host: str, timeout: float = 0.6) -> bool:
    try:
        with socket.create_connection((host, 80), timeout=timeout):
            return True
    except OSError:
        return False


def resolve_host(host: Optional[str], log: Callable[[str], None] = lambda m: None) -> str:
    if host:
        return host
    env = os.environ.get("BRIDGE_HOST")
    if env:
        return env
    try:
        cached = CACHE_FILE.read_text().strip()
        if cached and _reachable(cached):
            return cached
    except OSError:
        pass
    try:
        ip = socket.gethostbyname(MDNS_NAME)
        if _reachable(ip):
            log(f"{MDNS_NAME} -> {ip}")
            return ip
    except OSError:
        pass
    bridges = discover(1.2)
    if bridges:
        log(f"discovered {bridges[0].get('name', '?')} at {bridges[0]['ip']}")
        return bridges[0]["ip"]
    if _reachable(SETUP_AP_IP, 1.0):
        log(f"using the setup AP at {SETUP_AP_IP}")
        return SETUP_AP_IP
    raise BridgeError("no bridge found (tried $BRIDGE_HOST, cache, mDNS, UDP discovery, "
                      "setup AP 192.168.4.1); pass --host or connect to the DreameBridge-XXXX Wi-Fi")


def remember_host(host: str) -> None:
    try:
        CACHE_FILE.write_text(host)
    except OSError:
        pass


# --------------------------------------------------------------------------- transports

class HttpTransport:
    def __init__(self, host: str):
        self.host = host

    def run(self, line: str, timeout: float = CMD_TIMEOUT) -> dict:
        req = urllib.request.Request(
            f"http://{self.host}/api/cmd", data=line.encode("utf-8"), method="POST",
            headers={"X-Bridge": "1", "Content-Type": "text/plain"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                body = resp.read()
        except urllib.error.HTTPError as exc:
            body = exc.read()
            if not body.startswith(b"{"):
                raise BridgeError(f"HTTP {exc.code}: {body.decode(errors='replace')}")
        except (urllib.error.URLError, OSError) as exc:
            raise BridgeError(f"{self.host}: {getattr(exc, 'reason', exc)}")
        try:
            return json.loads(body)
        except ValueError:
            raise BridgeError(f"bad reply: {body[:200]!r}")


class UsbTransport:
    """Same commands over the USB CDC console: "@CMD <line>" -> "@CMD <json>"."""

    def __init__(self, port: str):
        try:
            import serial  # type: ignore
        except ImportError:
            raise BridgeError("--usb needs pyserial: pip install pyserial")
        # ESP32-S2 (ROM USB CDC, 303A:0002) only transmits with DTR=0, RTS=1,
        # and closing the port (RTS falling) reboots it; settings are kept.
        # ESP32-S3 (USB-Serial-JTAG, 303A:1001) resets while RTS is asserted,
        # so both lines stay low there.
        is_s3 = False
        try:
            from serial.tools import list_ports  # type: ignore
            for info in list_ports.comports():
                if info.device == port and info.vid == 0x303A and info.pid == 0x1001:
                    is_s3 = True
        except Exception:
            pass
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.2
        self.ser.dtr = False
        self.ser.rts = not is_s3
        self.ser.open()
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def run(self, line: str, timeout: float = CMD_TIMEOUT) -> dict:
        self.ser.write(b"@CMD " + line.encode("utf-8") + b"\n")
        deadline = time.monotonic() + timeout
        buf = b""
        while time.monotonic() < deadline:
            buf += self.ser.read(4096)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                raw = raw.strip()
                if raw.startswith(b"@CMD "):
                    return json.loads(raw[5:])
        raise BridgeError("no reply on USB (is this the bridge's port?)")


def make_transport(args) -> HttpTransport | UsbTransport:
    if getattr(args, "usb", None):
        return UsbTransport(args.usb)
    host = resolve_host(args.host, lambda m: print(f"# {m}", file=sys.stderr) if args.verbose else None)
    return HttpTransport(host)


# --------------------------------------------------------------------------- output

def print_result(res: dict, args) -> int:
    if args.text and "text" in res:
        text = res["text"]
        sys.stdout.write(text)
        if text and not text.endswith("\n"):
            sys.stdout.write("\n")
    elif args.pretty:
        print(json.dumps(res, indent=2, ensure_ascii=False))
    else:
        print(json.dumps(res, ensure_ascii=False))
    return 0 if res.get("ok") else 1


# --------------------------------------------------------------------------- local commands

def cmd_discover(args) -> int:
    bridges = discover(args.timeout)
    print(json.dumps({"ok": bool(bridges), "bridges": bridges}, indent=2 if args.pretty else None))
    return 0 if bridges else 1


def cmd_term(args) -> int:
    host = resolve_host(args.host)
    sock = socket.create_connection((host, args.port), timeout=5)
    sock.settimeout(None)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    eol = {"none": b"", "lf": b"\n", "cr": b"\r", "crlf": b"\r\n"}[args.eol]
    log = open(args.log, "ab") if args.log else None
    stop = threading.Event()

    def reader():
        try:
            while True:
                data = sock.recv(4096)
                if not data:
                    break
                if log:
                    log.write(data)
                    log.flush()
                sys.stdout.write(data.hex(" ") + "\n" if args.hex_out else data.decode("utf-8", "replace"))
                sys.stdout.flush()
        except OSError:
            pass
        stop.set()
        print("\n# disconnected", file=sys.stderr)

    threading.Thread(target=reader, daemon=True).start()
    print(f"# {host}:{args.port} raw terminal (eol={args.eol}; ':hex 41 42' sends hex; ':q' quits)",
          file=sys.stderr)
    try:
        for line in sys.stdin:
            if stop.is_set():
                break
            line = line.rstrip("\r\n")
            if line == ":q":
                break
            if line.startswith(":hex "):
                digits = "".join(c for c in line[5:].replace("0x", "") if c in "0123456789abcdefABCDEF")
                sock.sendall(bytes.fromhex(digits))
            else:
                sock.sendall(line.encode("utf-8").decode("unicode_escape").encode("latin-1") + eol)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if log:
            log.close()
    return 0


def cmd_app(args) -> int:
    import webbrowser
    host = resolve_host(args.host)
    url = f"http://{host}/"
    print(url)
    webbrowser.open(url)
    return 0


# --------------------------------------------------------------------------- MCP server

MCP_TOOLS = [
    {
        "name": "bridge_command",
        "description": (
            "Run one command on the Dreame robot bridge (ESP32-S2, Wi-Fi <-> robot UART/SWD) and "
            "return its JSON reply. Same language as the web app and the bridge_tool.py CLI. "
            "Commands: help | status | wifi.scan | wifi.set <ssid> <password> | wifi.forget | "
            "wifi.ap on|off | uart.baud [rate] | uart.send <data> | uart.sendhex <hex> | "
            "uart.read [wait_ms] | uart.xfer <wait_ms> <data> | swd <cmd> (PING ID DPID PID CTRL "
            "HALT RESUME STEP REGREAD n REGWRITE n v READ addr len ...) | reboot. "
            "Data escapes: \\r \\n \\t \\0 \\\\ \\\" \\xHH; quote values containing spaces. "
            "UART replies come back as 'text' (JSON string) and 'hex'."),
        "inputSchema": {
            "type": "object",
            "properties": {"command": {"type": "string",
                                       "description": 'e.g. uart.xfer 800 "info -a\\r\\n"'}},
            "required": ["command"],
        },
    },
    {
        "name": "uart_xfer",
        "description": ("Send data to the robot UART and return the reply "
                        "(firmware command: uart.xfer <wait_ms> <data>). The reply ends after 100 ms "
                        "of silence or wait_ms. Remember the line ending, e.g. 'info -a\\r\\n'."),
        "inputSchema": {
            "type": "object",
            "properties": {
                "data": {"type": "string", "description": "text with escapes \\r \\n \\xHH"},
                "wait_ms": {"type": "integer", "default": 1000, "minimum": 0, "maximum": 30000},
            },
            "required": ["data"],
        },
    },
    {
        "name": "uart_read",
        "description": "Return robot UART bytes received since the last read/xfer (firmware: uart.read [wait_ms]).",
        "inputSchema": {"type": "object",
                        "properties": {"wait_ms": {"type": "integer", "default": 0, "minimum": 0,
                                                   "maximum": 30000}}},
    },
    {
        "name": "bridge_status",
        "description": "Bridge status: Wi-Fi, UART baud and error counters, SWD ports (firmware: status).",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "bridge_discover",
        "description": "Find bridges on the local network via UDP broadcast.",
        "inputSchema": {"type": "object", "properties": {}},
    },
]


def mcp_serve(args) -> int:
    transport: list = [None]

    def get_transport():
        if transport[0] is None:
            transport[0] = make_transport(args)
            if isinstance(transport[0], HttpTransport):
                remember_host(transport[0].host)
        return transport[0]

    def run_line(line: str) -> dict:
        return get_transport().run(line)

    def call_tool(name: str, a: dict) -> dict:
        if name == "bridge_command":
            return run_line(str(a.get("command", "")))
        if name == "uart_xfer":
            wait = int(a.get("wait_ms", 1000))
            return run_line(f"uart.xfer {wait} {quote_data(str(a.get('data', '')))}")
        if name == "uart_read":
            return run_line(f"uart.read {int(a.get('wait_ms', 0))}")
        if name == "bridge_status":
            return run_line("status")
        if name == "bridge_discover":
            b = discover(1.5)
            return {"ok": bool(b), "bridges": b}
        raise BridgeError(f"unknown tool {name}")

    def reply(msg_id, result=None, error=None):
        out = {"jsonrpc": "2.0", "id": msg_id}
        if error is not None:
            out["error"] = error
        else:
            out["result"] = result
        sys.stdout.write(json.dumps(out, ensure_ascii=False) + "\n")
        sys.stdout.flush()

    for raw in sys.stdin:
        raw = raw.strip()
        if not raw:
            continue
        try:
            msg = json.loads(raw)
        except ValueError:
            reply(None, error={"code": -32700, "message": "parse error"})
            continue
        method, msg_id = msg.get("method"), msg.get("id")
        if msg_id is None:
            continue  # notification (e.g. notifications/initialized)
        if method == "initialize":
            reply(msg_id, {
                "protocolVersion": msg.get("params", {}).get("protocolVersion", "2025-06-18"),
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "dreame-bridge", "version": VERSION},
            })
        elif method == "ping":
            reply(msg_id, {})
        elif method == "tools/list":
            reply(msg_id, {"tools": MCP_TOOLS})
        elif method == "tools/call":
            params = msg.get("params", {})
            try:
                res = call_tool(params.get("name", ""), params.get("arguments") or {})
                reply(msg_id, {"content": [{"type": "text", "text": json.dumps(res, ensure_ascii=False)}],
                               "isError": not res.get("ok", False)})
            except (BridgeError, OSError, ValueError) as exc:
                transport[0] = None  # re-resolve the bridge next time
                reply(msg_id, {"content": [{"type": "text", "text": f"error: {exc}"}], "isError": True})
        else:
            reply(msg_id, error={"code": -32601, "message": f"method not found: {method}"})
    return 0


# --------------------------------------------------------------------------- main

LOCAL = {"discover", "term", "app", "mcp"}


def main(argv: Optional[list[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    p = argparse.ArgumentParser(
        prog="bridge_tool.py", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", help="bridge IP or hostname")
    p.add_argument("--usb", metavar="PORT", help="use the USB cable (e.g. COM8, /dev/ttyACM0)")
    p.add_argument("--pretty", action="store_true", help="indent JSON output")
    p.add_argument("--text", action="store_true", help="print only the UART 'text' of the reply")
    p.add_argument("-v", "--verbose", action="store_true")
    p.add_argument("--timeout", type=float, default=1.5, help="discover: seconds to wait")
    p.add_argument("--port", type=int, default=UART_PORT, help="term: raw UART TCP port")
    p.add_argument("--eol", choices=["none", "lf", "cr", "crlf"], default="crlf", help="term: line ending")
    p.add_argument("--hex-out", action="store_true", help="term: show received bytes as hex")
    p.add_argument("--log", help="term: append received bytes to this file")
    p.add_argument("command", nargs=argparse.REMAINDER,
                   help="a firmware command (see 'help') or discover/term/app/mcp")

    # Options must come before the command; everything after it belongs to the command.
    args = p.parse_args(argv)
    if not args.command:
        p.print_help()
        return 2
    name = args.command[0]

    try:
        if name == "discover":
            return cmd_discover(args)
        if name == "term":
            return cmd_term(args)
        if name == "app":
            return cmd_app(args)
        if name == "mcp":
            return mcp_serve(args)
        line = build_line(args.command)
        transport = make_transport(args)
        res = transport.run(line)
        if isinstance(transport, HttpTransport):
            remember_host(transport.host)
        return print_result(res, args)
    except BridgeError as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1


if __name__ == "__main__":
    sys.exit(main())
