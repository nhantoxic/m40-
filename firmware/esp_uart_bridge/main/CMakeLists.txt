idf_component_register(
    SRCS "main.c"
         "crashlog.c"
         "settings.c"
         "wifi_mgr.c"
         "jbuf.c"
         "cmd.c"
         "web.c"
         "discovery.c"
         "uart_tcp_bridge.c"
         "swd_bridge.c"
    INCLUDE_DIRS ".")
# The web app is compiled in from web_index.h (run ../gen_web_index.py after
# editing web/index.html). No EMBED_TXTFILES: PlatformIO does not build it.
