idf_component_register(
    SRCS "vl53l0x.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_timer esp_driver_i2c
)
