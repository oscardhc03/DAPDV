idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES "tf_mini_parser" "hmi" "imu"
)
