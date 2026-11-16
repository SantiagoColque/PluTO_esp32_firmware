idf_component_register(SRCS "servo_pulse.c" "servo_driver.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_ledc)
