#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>

#include "esp_err.h"
#include "servo_pulse.h"

/*
 * The pan and tilt DS3218 on the ESP32's LEDC: one 50 Hz timer with a 16-bit
 * duty and one channel per servo.
 *
 * It takes angles in servo space, centidegrees from 0 to 18000, already
 * limited by pan_tilt, and turns them into pulses with servo_pulse.h. Pins and
 * calibration come in through servo_driver_init() and the driver never reads
 * Kconfig itself, so whoever starts it decides where they come from: Kconfig
 * today, NVS once the calibration is stored there.
 *
 * Not thread-safe: every call must come from the same task, the rotor task.
 */

typedef struct {
    int gpio_num;                    /* signal pin; must be safe to drive from boot on (see servo_driver.c) */
    servo_calibration_t calibration; /* must pass servo_calibration_valid() */
} servo_axis_config_t;

typedef struct {
    servo_axis_config_t pan;
    servo_axis_config_t tilt;
} servo_driver_config_t;

/**
 * @brief Configure the timer and both channels.
 *
 * No pulse leaves either pin until the first servo_driver_set(): both lines
 * stay low, so the servos do not lurch to some default angle at boot.
 * Returns ESP_ERR_INVALID_ARG for an invalid calibration or a pin that is
 * unsafe or shared by both servos. Call it once.
 */
esp_err_t servo_driver_init(const servo_driver_config_t *config);

/**
 * @brief Command both servos, in centidegrees of servo space.
 *
 * A channel whose duty is unchanged is left alone, so calling this with the
 * same pose again is cheap. After servo_driver_release() it resumes the
 * pulses, even for the pose the servos had before. Fails only with
 * ESP_ERR_INVALID_STATE, when servo_driver_init() did not succeed.
 */
esp_err_t servo_driver_set(int32_t pan_cdeg, int32_t tilt_cdeg);

/**
 * @brief Stop the pulses on both servos and hold the lines low.
 *
 * The pulse in flight finishes first, so no short pulse leaves the pin. Meant
 * for the rest state of HU-35: a servo without pulses usually stops holding
 * its position and draws far less, although whether the DS3218 does is still
 * to be seen on the bench. The next servo_driver_set() resumes them. Fails
 * only like servo_driver_set().
 */
esp_err_t servo_driver_release(void);

#endif
