idf_component_register(
    SRCS
    "main.c"
    "scenes.c"
    "wifi.c"
    "ota.c"
    "dali/dali.c"
    "dali/dali_bus.c"
    INCLUDE_DIRS "."
    REQUIRES ieee802154 openthread nvs_flash esp_adc esp_timer esp_pm app_update esp_http_client esp_wifi esp_netif mbedtls
    EMBED_TXTFILES ${project_dir}/letsencrypt.pem
)
