idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES MCP4725 LAB_LINK esp_wifi nvs_flash esp_event esp_netif esp_driver_i2c esp_timer)
