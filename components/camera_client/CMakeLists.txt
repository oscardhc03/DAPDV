idf_component_register(
    SRCS "camera_client.c"
    INCLUDE_DIRS "include"
    REQUIRES "nvs_flash" "esp_wifi" "esp_netif" "esp_http_client" "esp_event" "esp-tls" "json"
)