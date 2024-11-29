idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES "tf_mini_parser" "hmi" "imu" "power_save" "battery_monitor" "camera_client"
)
