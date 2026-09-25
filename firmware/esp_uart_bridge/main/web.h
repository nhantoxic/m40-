#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HTTP server on port 80: the web app, POST /api/cmd and the /ws terminal. */
void web_start(void);

/* False if the HTTP server could not start (e.g. out of memory). */
bool web_running(void);

#ifdef __cplusplus
}
#endif
