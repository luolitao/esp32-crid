idf_component_register(SRCS crid_scan_main.c
                            crid_sniffer.c
                            crid_parser.c
                            crid_tracker.c
                            crid_display.c
                            crid_json.c
                       INCLUDE_DIRS ""
                       PRIV_REQUIRES opendroneid
                       REQUIRES nvs_flash esp_wifi esp_netif esp_event esp_driver_uart)