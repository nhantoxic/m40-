idf_component_register(
    SRCS "main.c"
         "settings.c"
         "wifi_mgr.c"
         "jbuf.c"
         "cmd.c"
         "web.c"
         "discovery.c"
         "uart_tcp_bridge.c"
         "swd_bridge.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "web/index.html")
