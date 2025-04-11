idf_component_register(SRCS "main.c"
                       "wifi_handler.c"
                       "mqtt_handler.c"
                       "uart_handler.c"
                       "led_handler.c"
                    INCLUDE_DIRS "."
                    )

# Specify required ESP-IDF components
set(REQUIRES freertos esp_log nvs_flash esp_netif esp_event esp_wifi esp-mqtt driver cjson)