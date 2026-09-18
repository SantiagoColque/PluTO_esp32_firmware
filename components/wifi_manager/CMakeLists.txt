idf_component_register(SRCS "wifi_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_event esp_netif config_store
                       PRIV_REQUIRES nvs_flash esp_timer esp_hw_support)
