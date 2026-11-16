idf_component_register(SRCS "encoder_angle.c" "as5600.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c)
