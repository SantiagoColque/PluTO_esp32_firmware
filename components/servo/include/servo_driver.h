#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>

#include "esp_err.h"
#include "servo_pulse.h"

/* Both DS3218 on one 50 Hz LEDC timer, a channel each. Pins and calibration
 * come from the caller. Not thread-safe: rotor task only. */

typedef struct {
    int gpio_num;                    /* must be safe to drive from boot on (see servo_driver.c) */
    servo_calibration_t calibration; /* must pass servo_calibration_valid() */
} servo_axis_config_t;

typedef struct {
    servo_axis_config_t pan;
    servo_axis_config_t tilt;
} servo_driver_config_t;

/**
 * @brief Configure the timer and both channels.
 *
 * Lines stay low until the first servo_driver_set(), so the servos don't lurch
 * at boot. ESP_ERR_INVALID_ARG on a bad calibration or an unsafe or shared pin.
 */
esp_err_t servo_driver_init(const servo_driver_config_t *config);

/**
 * @brief Command both servos, in centidegrees of servo space.
 *
 * Unchanged channels are left alone. Also resumes the pulses after a release.
 */
esp_err_t servo_driver_set(int32_t pan_cdeg, int32_t tilt_cdeg);

/**
 * @brief Stop the pulses on both servos and hold the lines low.
 *
 * Rest state (HU-35), once the pulse in flight ends. Whether the DS3218 then
 * draws less is still to check on the bench.
 */
esp_err_t servo_driver_release(void);

#endif
