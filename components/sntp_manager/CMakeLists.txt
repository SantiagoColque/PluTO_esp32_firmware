idf_component_register(SRCS "sntp_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES lwip
                       PRIV_REQUIRES config_store esp_netif)
