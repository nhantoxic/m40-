#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Start the UDP responder used by RobotMonitor to find this bridge on LAN. */
void discovery_start(void);

#ifdef __cplusplus
}
#endif
