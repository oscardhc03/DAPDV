idf_component_register(
    SRCS "power_save.c"
    INCLUDE_DIRS "include"
    REQUIRES "freertos" "esp_hw_support" "driver"
)
