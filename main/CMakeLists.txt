set(srcs "main.c" "fan_common.c" "key.c" "battery.c" "oled.c"
        "fan_wifi_ap.c" "fan_wifi_ota.c" "fan_http_server.c")

idf_component_register(SRCS ${srcs}
        EMBED_FILES "static/favicon.ico" "static/ota.html" "static/index.html"
        INCLUDE_DIRS ".")
