#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One command language for every client: the web app, the PC tool, AI agents
 * (via the PC tool / MCP server), HTTP (POST /api/cmd) and USB ("@CMD <line>").
 * A command is one text line; the reply is one line of JSON that always has
 * "ok" and, on failure, "error". Run "help" for the list. */

/* usb_reader: also read "@CMD" lines from the USB console (when the SWD
 * task, which normally owns the console, is not running). */
void cmd_start(bool usb_reader);

/* Executes one command line. Returns a malloc'd JSON string (caller frees). */
char *cmd_exec(const char *line);

/* USB console hook: runs `line` and prints "@CMD <json>". */
void cmd_usb_line(const char *line);

#ifdef __cplusplus
}
#endif
