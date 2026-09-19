idf_component_register(SRCS "main.c"
                       REQUIRES wifi_manager mqtt_manager sntp_manager
                                config_store web_server nvs_flash esp_wifi esp_event rotor)
