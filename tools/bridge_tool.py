#!/usr/bin/env python3
"""PC-side CLI + GUI for the ESP32-S2 esp_uart_bridge firmware.

Standard library only (Python 3.8+). The GUI needs tkinter, which ships with
the python.org Windows installer.

    python bridge_tool.py discover                  # find bridges on the LAN
    python bridge_tool.py status                    # counters of one bridge
    python bridge_tool.py term                      # interactive raw UART terminal
    python bridge_tool.py send "info -a" --wait 1   # one command, print reply
    python bridge_tool.py send "3C 00 01 3E" --hex
    python bridge_tool.py swd PING ID "READ 0x08000000 64"
    python bridge_tool.py gui                       # graphical console

--host is optional everywhere: without it the tool tries dreame-bridge.local,
then a UDP broadcast discovery.
"""

from __future__ import annotations

import argparse
import json
import queue
import re
import socket
import sys
import threading
import time
from typing import Callable, Optional

DISCOVERY_PORT = 2326
DISCOVERY_QUERY = b"DREAME_BRIDGE_DISCOVER"
DEFAULT_HOSTNAME = "dreame-bridge.local"
DEFAULT_UART_PORT = 2324
DEFAULT_SWD_PORT = 2325

EOLS = {"none": b"", "lf": b"\n", "cr": b"\r", "crlf": b"\r\n"}


# --------------------------------------------------------------------------- helpers

def parse_hex(text: str) -> bytes:
    """'3C 00 0x01,3e' -> b'<\\x00\\x01>'"""
    cleaned = text.replace("0x", "").replace("0X", "")
    digits = "".join(c for c in cleaned if c in "0123456789abcdefABCDEF")
    if len(digits) % 2:
        raise ValueError("odd number of hex digits")
    return bytes.fromhex(digits)


def unescape(text: str) -> bytes:
    r"""Text with \n \r \t \xHH \\ escapes -> bytes (UTF-8 for other characters)."""
    out = bytearray()
    i = 0
    while i < len(text):
        c = text[i]
        if c == "\\" and i + 1 < len(text):
            n = text[i + 1]
            if n in "nrt\\":
                out += {"n": b"\n", "r": b"\r", "t": b"\t", "\\": b"\\"}[n]
                i += 2
                continue
            if n == "x" and re.fullmatch(r"[0-9a-fA-F]{2}", text[i + 2:i + 4]):
                out.append(int(text[i + 2:i + 4], 16))
                i += 4
                continue
        out += c.encode("utf-8")
        i += 1
    return bytes(out)


def hexdump(data: bytes, base: int = 0) -> str:
    lines = []
    for off in range(0, len(data), 16):
        row = data[off:off + 16]
        hx = " ".join(f"{b:02x}" for b in row)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        lines.append(f"{base + off:08x}  {hx:<47}  {asc}")
    return "\n".join(lines)


def discover(timeout: float = 1.5, target: str = "255.255.255.255") -> list[dict]:
    """Broadcast (or unicast to `target`) the discovery query; return replies."""
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
            try:
                info = json.loads(data.decode("utf-8", "replace"))
            except ValueError:
                continue
            info.setdefault("ip", addr[0])
            found[info.get("mac", addr[0])] = info
    finally:
        sock.close()
    return list(found.values())


def resolve_host(host: Optional[str], quiet: bool = False) -> str:
    if host:
        return host
    try:
        ip = socket.gethostbyname(DEFAULT_HOSTNAME)
        if not quiet:
            print(f"# using {DEFAULT_HOSTNAME} -> {ip}", file=sys.stderr)
        return ip
    except OSError:
        pass
    bridges = discover()
    if not bridges:
        raise SystemExit("no bridge found (mDNS and UDP discovery failed); pass --host")
    if len(bridges) > 1 and not quiet:
        print("# several bridges found, using the first:", file=sys.stderr)
    ip = bridges[0]["ip"]
    if not quiet:
        print(f"# discovered {bridges[0].get('name', '?')} at {ip}", file=sys.stderr)
    return ip


# --------------------------------------------------------------------------- UART link

class UartLink:
    """Raw TCP connection to the bridge's UART port with a reader thread."""

    def __init__(self, host: str, port: int = DEFAULT_UART_PORT,
                 on_data: Optional[Callable[[bytes], None]] = None,
                 on_close: Optional[Callable[[str], None]] = None):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(None)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.on_data = on_data
        self.on_close = on_close
        self.closed = False
        self.rx_queue: "queue.Queue[bytes]" = queue.Queue()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self) -> None:
        reason = "closed by bridge"
        try:
            while True:
                data = self.sock.recv(4096)
                if not data:
                    break
                if self.on_data:
                    self.on_data(data)
                else:
                    self.rx_queue.put(data)
        except OSError as exc:
            reason = str(exc)
        if not self.closed:
            self.closed = True
            if self.on_close:
                self.on_close(reason)

    def send(self, data: bytes) -> None:
        self.sock.sendall(data)

    def read_for(self, seconds: float) -> bytes:
        """Collect whatever arrives within `seconds` (only without on_data)."""
        out = bytearray()
        deadline = time.monotonic() + seconds
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                break
            try:
                out += self.rx_queue.get(timeout=left)
            except queue.Empty:
                break
        return bytes(out)

    def close(self) -> None:
        self.closed = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


# --------------------------------------------------------------------------- SWD text API

class SwdClient:
    """Line protocol on tcp/2325. Several commands share one connection."""

    def __init__(self, host: str, port: int = DEFAULT_SWD_PORT, timeout: float = 10):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b""

    def _read_line(self) -> str:
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("SWD connection closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode("utf-8", "replace").rstrip("\r")

    def _read_exact(self, n: int) -> bytes:
        while len(self.buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("SWD connection closed mid-transfer")
            self.buf += chunk
        data, self.buf = self.buf[:n], self.buf[n:]
        return data

    def command(self, line: str) -> tuple[str, Optional[bytes]]:
        """Returns (status line, binary body for READ/DUMP)."""
        verb = line.split()[0].upper() if line.split() else ""
        if verb in ("WRITE", "MWRITE"):
            raise ValueError(f"{verb} needs a binary payload; not supported from this tool")
        self.sock.sendall(line.encode() + b"\n")
        status = self._read_line()
        if verb in ("READ", "DUMP") and status.startswith("OK "):
            try:
                length = int(status.split()[1])
            except (IndexError, ValueError):
                return status, None
            return status, self._read_exact(length)
        return status, None

    def close(self) -> None:
        self.sock.close()


# --------------------------------------------------------------------------- CLI commands

def cmd_discover(args) -> int:
    bridges = discover(args.timeout)
    if args.json:
        print(json.dumps(bridges, indent=2))
        return 0 if bridges else 1
    if not bridges:
        print("no bridge answered on UDP", DISCOVERY_PORT)
        return 1
    for b in bridges:
        st = b.get("uart_stats", {})
        print(f"{b.get('name', '?'):16} {b.get('ip', '?'):15} {b.get('mac', '?')}  "
              f"uart tcp/{b.get('uart_port')} {b.get('uart_baud')} {b.get('uart_mode')}  "
              f"swd tcp/{b.get('swd_port')}  rx={st.get('rx_bytes')} tx={st.get('tx_bytes')}")
    return 0


def cmd_status(args) -> int:
    host = resolve_host(args.host, quiet=True)
    replies = discover(args.timeout, target=host)
    if not replies:
        print(f"{host}: no reply on UDP {DISCOVERY_PORT}")
        return 1
    print(json.dumps(replies[0], indent=2))
    st = replies[0].get("uart_stats", {})
    if st.get("frame_err", 0) or st.get("parity_err", 0):
        print("# frame/parity errors: check baud rate and parity", file=sys.stderr)
    if st.get("fifo_ovf", 0) or st.get("buf_full", 0):
        print("# overflow: robot sends faster than the bridge forwards", file=sys.stderr)
    return 0


def cmd_send(args) -> int:
    host = resolve_host(args.host)
    payload = parse_hex(args.data) if args.hex else unescape(args.data) + EOLS[args.eol]
    link = UartLink(host, args.port)
    try:
        link.send(payload)
        reply = link.read_for(args.wait)
    finally:
        link.close()
    if args.hex_out:
        print(hexdump(reply))
    else:
        sys.stdout.write(reply.decode("utf-8", "replace"))
        if reply and not reply.endswith(b"\n"):
            print()
    return 0


def cmd_term(args) -> int:
    host = resolve_host(args.host)
    log = open(args.log, "ab") if args.log else None
    out = sys.stdout

    def on_data(data: bytes) -> None:
        if log:
            log.write(data)
            log.flush()
        if args.hex_out:
            out.write(hexdump(data) + "\n")
        else:
            out.write(data.decode("utf-8", "replace"))
        out.flush()

    def on_close(reason: str) -> None:
        out.write(f"\n# disconnected: {reason}\n")
        out.flush()

    link = UartLink(host, args.port, on_data, on_close)
    print(f"# connected to {host}:{args.port}  (eol={args.eol}; "
          f"lines starting with ':hex ' are sent as hex; Ctrl+C or ':q' quits)",
          file=sys.stderr)
    try:
        for line in sys.stdin:
            if link.closed:
                break
            line = line.rstrip("\r\n")
            if line == ":q":
                break
            if line.startswith(":hex "):
                data = parse_hex(line[5:])
            else:
                data = unescape(line) + EOLS[args.eol]
            if log:
                log.write(b"\n>>> " + data + b"\n")
            link.send(data)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        if log:
            log.close()
    return 0


def cmd_swd(args) -> int:
    host = resolve_host(args.host)
    client = SwdClient(host, args.port)
    rc = 0
    try:
        for line in args.commands:
            status, body = client.command(line)
            print(f"> {line}\n{status}")
            if not status.startswith("OK"):
                rc = 1
            if body is not None:
                if args.out:
                    with open(args.out, "wb") as f:
                        f.write(body)
                    print(f"# {len(body)} bytes -> {args.out}")
                else:
                    base = 0
                    parts = line.split()
                    if len(parts) > 1:
                        try:
                            base = int(parts[1], 0)
                        except ValueError:
                            pass
                    print(hexdump(body, base))
    finally:
        client.close()
    return rc


# --------------------------------------------------------------------------- GUI

def cmd_gui(args) -> int:
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
    except ImportError:
        raise SystemExit("tkinter is not available in this Python installation")

    root = tk.Tk()
    root.title("Dreame bridge console")
    root.geometry("1100x700")

    events: "queue.Queue[tuple[str, object]]" = queue.Queue()
    state = {"link": None, "last_stats": None, "history": [], "hist_pos": 0}

    # --- top bar
    top = ttk.Frame(root, padding=6)
    top.pack(fill="x")
    ttk.Label(top, text="Host").pack(side="left")
    host_var = tk.StringVar(value=args.host or "")
    host_box = ttk.Combobox(top, textvariable=host_var, width=22)
    host_box.pack(side="left", padx=4)
    ttk.Label(top, text="UART port").pack(side="left")
    port_var = tk.StringVar(value=str(args.port))
    ttk.Entry(top, textvariable=port_var, width=6).pack(side="left", padx=4)
    btn_scan = ttk.Button(top, text="Scan")
    btn_scan.pack(side="left", padx=2)
    btn_conn = ttk.Button(top, text="Connect")
    btn_conn.pack(side="left", padx=2)
    conn_label = ttk.Label(top, text="● offline", foreground="#c0392b")
    conn_label.pack(side="left", padx=10)
    stats_label = ttk.Label(top, text="")
    stats_label.pack(side="right")

    paned = ttk.PanedWindow(root, orient="horizontal")
    paned.pack(fill="both", expand=True, padx=6, pady=(0, 6))

    # --- terminal
    left = ttk.Frame(paned)
    paned.add(left, weight=3)
    term = tk.Text(left, wrap="char", bg="#0f172a", fg="#e2e8f0", insertbackground="#e2e8f0",
                   font=("Consolas", 10), state="disabled")
    term.tag_configure("tx", foreground="#93c5fd")
    term.tag_configure("sys", foreground="#fbbf24")
    sb = ttk.Scrollbar(left, command=term.yview)
    term.configure(yscrollcommand=sb.set)
    sb.pack(side="right", fill="y")
    term.pack(fill="both", expand=True)

    view_var = tk.StringVar(value="text")
    echo_var = tk.BooleanVar(value=True)
    ts_var = tk.BooleanVar(value=False)
    auto_var = tk.BooleanVar(value=True)

    send_row = ttk.Frame(left, padding=(0, 6, 0, 0))
    send_row.pack(fill="x")
    entry = ttk.Entry(send_row)
    entry.pack(side="left", fill="x", expand=True)
    eol_var = tk.StringVar(value=args.eol)
    ttk.Combobox(send_row, textvariable=eol_var, values=list(EOLS), width=6,
                 state="readonly").pack(side="left", padx=4)
    hex_in_var = tk.BooleanVar(value=False)
    ttk.Checkbutton(send_row, text="HEX", variable=hex_in_var).pack(side="left")
    btn_send = ttk.Button(send_row, text="Send")
    btn_send.pack(side="left", padx=4)

    opt_row = ttk.Frame(left, padding=(0, 4, 0, 0))
    opt_row.pack(fill="x")
    ttk.Label(opt_row, text="View").pack(side="left")
    ttk.Combobox(opt_row, textvariable=view_var, values=["text", "hex"], width=5,
                 state="readonly").pack(side="left", padx=4)
    for text, var in (("Echo TX", echo_var), ("Timestamp", ts_var), ("Auto-scroll", auto_var)):
        ttk.Checkbutton(opt_row, text=text, variable=var).pack(side="left", padx=4)
    btn_clear = ttk.Button(opt_row, text="Clear")
    btn_clear.pack(side="right")
    btn_save = ttk.Button(opt_row, text="Save log")
    btn_save.pack(side="right", padx=4)

    # --- right side: macros, SWD, stats
    right = ttk.Frame(paned, padding=(6, 0, 0, 0))
    paned.add(right, weight=1)

    mac_frame = ttk.LabelFrame(right, text="Macros (name = command, \\r \\n \\xHH ok)", padding=6)
    mac_frame.pack(fill="x")
    mac_buttons = ttk.Frame(mac_frame)
    mac_buttons.pack(fill="x")
    mac_text = tk.Text(mac_frame, height=6, font=("Consolas", 9))
    mac_text.insert("1.0", "\n".join(args.macro or ["info = info -a", "version = ver -t", "help = help"]))
    mac_text.pack(fill="x", pady=(6, 0))
    btn_mac_apply = ttk.Button(mac_frame, text="Apply macros")
    btn_mac_apply.pack(anchor="e", pady=(4, 0))

    swd_frame = ttk.LabelFrame(right, text=f"SWD (tcp/{DEFAULT_SWD_PORT})", padding=6)
    swd_frame.pack(fill="x", pady=6)
    swd_btns = ttk.Frame(swd_frame)
    swd_btns.pack(fill="x")
    swd_entry = ttk.Entry(swd_frame)
    swd_entry.pack(fill="x", pady=(6, 0))
    swd_entry.insert(0, "READ 0x08000000 64")

    stat_frame = ttk.LabelFrame(right, text="Bridge status (UDP discovery, 2 s)", padding=6)
    stat_frame.pack(fill="both", expand=True)
    stat_text = tk.Text(stat_frame, height=14, font=("Consolas", 9), state="disabled")
    stat_text.pack(fill="both", expand=True)

    # --- terminal output
    line_open = {"v": False}

    def term_write(text: str, tag: str = "") -> None:
        term.configure(state="normal")
        if ts_var.get() and not line_open["v"]:
            term.insert("end", time.strftime("%H:%M:%S "), "sys")
        term.insert("end", text, tag)
        line_open["v"] = not text.endswith("\n")
        lines = int(term.index("end-1c").split(".")[0])
        if lines > 5000:
            term.delete("1.0", f"{lines - 5000}.0")
        term.configure(state="disabled")
        if auto_var.get():
            term.see("end")

    def sys_msg(text: str) -> None:
        if line_open["v"]:
            term_write("\n")
        term_write(f"# {text}\n", "sys")

    def show_rx(data: bytes) -> None:
        if view_var.get() == "hex":
            if line_open["v"]:
                term_write("\n")
            term_write(hexdump(data) + "\n")
        else:
            term_write(data.decode("utf-8", "replace").replace("\r\n", "\n").replace("\r", "\n"))

    # --- actions
    def set_connected(ok: bool) -> None:
        conn_label.configure(text="● connected" if ok else "● offline",
                             foreground="#16a34a" if ok else "#c0392b")
        btn_conn.configure(text="Disconnect" if ok else "Connect")

    def connect_toggle() -> None:
        link = state["link"]
        if link:
            link.close()
            state["link"] = None
            set_connected(False)
            sys_msg("disconnected")
            return
        host = host_var.get().strip() or DEFAULT_HOSTNAME
        try:
            port = int(port_var.get())
        except ValueError:
            messagebox.showerror("Port", "invalid port")
            return

        def worker():
            try:
                lk = UartLink(host, port,
                              on_data=lambda d: events.put(("rx", d)),
                              on_close=lambda r: events.put(("closed", r)))
                events.put(("connected", (lk, host, port)))
            except OSError as exc:
                events.put(("error", f"connect {host}:{port} failed: {exc}"))
        sys_msg(f"connecting to {host}:{port} ...")
        threading.Thread(target=worker, daemon=True).start()

    def send_bytes(data: bytes, label: Optional[str] = None) -> None:
        link = state["link"]
        if not link:
            sys_msg("not connected")
            return
        try:
            link.send(data)
        except OSError as exc:
            sys_msg(f"send failed: {exc}")
            return
        if echo_var.get():
            if line_open["v"]:
                term_write("\n")
            term_write(f"> {label if label is not None else data.hex(' ')}\n", "tx")

    def send_entry(_event=None) -> None:
        text = entry.get()
        try:
            if hex_in_var.get():
                send_bytes(parse_hex(text))
            else:
                send_bytes(unescape(text) + EOLS[eol_var.get()], text)
        except ValueError as exc:
            sys_msg(f"bad input: {exc}")
            return
        if text and (not state["history"] or state["history"][-1] != text):
            state["history"].append(text)
        state["hist_pos"] = len(state["history"])
        entry.delete(0, "end")

    def history(step: int) -> str:
        h = state["history"]
        if not h:
            return "break"
        state["hist_pos"] = max(0, min(len(h), state["hist_pos"] + step))
        entry.delete(0, "end")
        if state["hist_pos"] < len(h):
            entry.insert(0, h[state["hist_pos"]])
        return "break"

    def apply_macros() -> None:
        for w in mac_buttons.winfo_children():
            w.destroy()
        for i, raw in enumerate(mac_text.get("1.0", "end").splitlines()):
            if "=" not in raw:
                continue
            name, cmd = (p.strip() for p in raw.split("=", 1))
            if not name:
                continue
            b = ttk.Button(mac_buttons, text=name,
                           command=lambda c=cmd, n=name: send_bytes(unescape(c) + EOLS[eol_var.get()],
                                                                    f"{n}: {c}"))
            b.grid(row=i // 3, column=i % 3, sticky="ew", padx=2, pady=2)

    def scan() -> None:
        def worker():
            events.put(("scan", discover(1.5)))
        threading.Thread(target=worker, daemon=True).start()

    def swd_run(line: str) -> None:
        host = host_var.get().strip() or DEFAULT_HOSTNAME

        def worker():
            try:
                c = SwdClient(host)
                try:
                    status, body = c.command(line)
                finally:
                    c.close()
                msg = f"SWD> {line}\n{status}"
                if body is not None:
                    try:
                        base = int(line.split()[1], 0)
                    except (IndexError, ValueError):
                        base = 0
                    msg += "\n" + hexdump(body[:1024], base)
                    if len(body) > 1024:
                        msg += f"\n... ({len(body)} bytes, use the CLI with --out to save)"
                events.put(("swd", msg))
            except (OSError, ValueError, ConnectionError) as exc:
                events.put(("swd", f"SWD> {line}\nerror: {exc}"))
        threading.Thread(target=worker, daemon=True).start()

    for i, cmd in enumerate(("PING", "ID", "DPID", "HALT", "RESUME", "STEP")):
        ttk.Button(swd_btns, text=cmd, command=lambda c=cmd: swd_run(c)).grid(
            row=i // 3, column=i % 3, sticky="ew", padx=2, pady=2)
    swd_entry.bind("<Return>", lambda e: swd_run(swd_entry.get().strip()))

    def poll_status() -> None:
        host = host_var.get().strip()

        def worker():
            target = host or DEFAULT_HOSTNAME
            try:
                target = socket.gethostbyname(target)
            except OSError:
                events.put(("stats", None))
                return
            r = discover(0.8, target=target)
            events.put(("stats", r[0] if r else None))
        threading.Thread(target=worker, daemon=True).start()
        root.after(2000, poll_status)

    def show_stats(info: Optional[dict]) -> None:
        stat_text.configure(state="normal")
        stat_text.delete("1.0", "end")
        if not info:
            stat_text.insert("end", "no discovery reply")
            stats_label.configure(text="")
        else:
            st = info.get("uart_stats", {})
            now = time.monotonic()
            rate = ""
            last = state["last_stats"]
            if last and now > last[0]:
                dt = now - last[0]
                rate = (f"↓{(st.get('rx_bytes', 0) - last[1]) / dt:.0f} B/s  "
                        f"↑{(st.get('tx_bytes', 0) - last[2]) / dt:.0f} B/s")
            state["last_stats"] = (now, st.get("rx_bytes", 0), st.get("tx_bytes", 0))
            stats_label.configure(text=f"{info.get('name')} {info.get('ip')}  {rate}")
            rows = [("name", info.get("name")), ("ip", info.get("ip")), ("mac", info.get("mac")),
                    ("uart", f"tcp/{info.get('uart_port')} {info.get('uart_baud')} {info.get('uart_mode')}"),
                    ("gpio tx/rx", f"{info.get('tx_gpio')}/{info.get('rx_gpio')} "
                                   f"level {info.get('uart_tx_level')}/{info.get('uart_rx_level')}"),
                    ("swd", f"tcp/{info.get('swd_port')}  bitbang tcp/{info.get('bitbang_port')}")]
            rows += [(k, v) for k, v in st.items()]
            for k, v in rows:
                stat_text.insert("end", f"{k:>12}: {v}\n")
        stat_text.configure(state="disabled")

    def pump_events() -> None:
        try:
            while True:
                kind, payload = events.get_nowait()
                if kind == "rx":
                    show_rx(payload)  # type: ignore[arg-type]
                elif kind == "connected":
                    lk, host, port = payload  # type: ignore[misc]
                    state["link"] = lk
                    set_connected(True)
                    sys_msg(f"connected to {host}:{port}")
                elif kind == "closed":
                    if state["link"]:
                        state["link"] = None
                        set_connected(False)
                        sys_msg(f"connection lost: {payload}")
                elif kind == "error":
                    sys_msg(str(payload))
                elif kind == "scan":
                    found = payload or []
                    host_box.configure(values=[b["ip"] for b in found])  # type: ignore[index]
                    if found and not host_var.get():
                        host_var.set(found[0]["ip"])  # type: ignore[index]
                    sys_msg(f"scan: {len(found)} bridge(s) " +  # type: ignore[arg-type]
                            ", ".join(f"{b.get('name')}@{b.get('ip')}" for b in found))  # type: ignore[union-attr]
                elif kind == "swd":
                    sys_msg(str(payload).replace("\n", "\n# "))
                elif kind == "stats":
                    show_stats(payload)  # type: ignore[arg-type]
        except queue.Empty:
            pass
        root.after(30, pump_events)

    def save_log() -> None:
        path = filedialog.asksaveasfilename(defaultextension=".txt")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(term.get("1.0", "end"))

    def clear() -> None:
        term.configure(state="normal")
        term.delete("1.0", "end")
        term.configure(state="disabled")
        line_open["v"] = False

    btn_conn.configure(command=connect_toggle)
    btn_scan.configure(command=scan)
    btn_send.configure(command=send_entry)
    btn_mac_apply.configure(command=apply_macros)
    btn_clear.configure(command=clear)
    btn_save.configure(command=save_log)
    entry.bind("<Return>", send_entry)
    entry.bind("<Up>", lambda e: history(-1))
    entry.bind("<Down>", lambda e: history(1))

    apply_macros()
    pump_events()
    poll_status()
    if not args.host:
        scan()
    entry.focus_set()
    root.mainloop()
    if state["link"]:
        state["link"].close()
    return 0


# --------------------------------------------------------------------------- main

def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_host(sp, port_default):
        sp.add_argument("--host", help=f"bridge IP/hostname (default: {DEFAULT_HOSTNAME} or discovery)")
        sp.add_argument("--port", type=int, default=port_default)

    sp = sub.add_parser("discover", help="find bridges via UDP broadcast")
    sp.add_argument("--timeout", type=float, default=1.5)
    sp.add_argument("--json", action="store_true")
    sp.set_defaults(func=cmd_discover)

    sp = sub.add_parser("status", help="show discovery JSON (UART counters) of one bridge")
    sp.add_argument("--host")
    sp.add_argument("--timeout", type=float, default=1.5)
    sp.set_defaults(func=cmd_status)

    sp = sub.add_parser("term", help="interactive raw UART terminal")
    add_host(sp, DEFAULT_UART_PORT)
    sp.add_argument("--eol", choices=list(EOLS), default="crlf")
    sp.add_argument("--hex-out", action="store_true", help="show received bytes as hex dump")
    sp.add_argument("--log", help="append everything received to this file")
    sp.set_defaults(func=cmd_term)

    sp = sub.add_parser("send", help="send one command and print the reply")
    add_host(sp, DEFAULT_UART_PORT)
    sp.add_argument("data")
    sp.add_argument("--hex", action="store_true", help="data is hex bytes, no EOL added")
    sp.add_argument("--eol", choices=list(EOLS), default="crlf")
    sp.add_argument("--wait", type=float, default=1.0, help="seconds to collect the reply")
    sp.add_argument("--hex-out", action="store_true")
    sp.set_defaults(func=cmd_send)

    sp = sub.add_parser("swd", help="run SWD text commands over one connection")
    add_host(sp, DEFAULT_SWD_PORT)
    sp.add_argument("commands", nargs="+", help='e.g. PING ID "READ 0x08000000 256"')
    sp.add_argument("--out", help="write the READ/DUMP body to this file")
    sp.set_defaults(func=cmd_swd)

    sp = sub.add_parser("gui", help="graphical console")
    add_host(sp, DEFAULT_UART_PORT)
    sp.add_argument("--eol", choices=list(EOLS), default="crlf")
    sp.add_argument("--macro", action="append", help='"name = command", repeatable')
    sp.set_defaults(func=cmd_gui)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
