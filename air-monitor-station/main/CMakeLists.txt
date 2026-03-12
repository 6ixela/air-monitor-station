idf_component_register(SRCS "app_main.c"
                    PRIV_REQUIRES mqtt esp_partition nvs_flash esp_netif app_update
                    INCLUDE_DIRS "."
                    REQUIRES json aht21 ens160)
