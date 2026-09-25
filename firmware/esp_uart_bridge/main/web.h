#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* HTTP server on port 80: the web app, POST /api/cmd and the /ws terminal. */
void web_start(void);

#ifdef __cplusplus
}
#endif
