idf_component_register(
    SRCS "hmi.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver" "freertos" "power_save"
)
