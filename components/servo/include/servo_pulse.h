#ifndef SERVO_PULSE_H
#define SERVO_PULSE_H

#include <stdbool.h>
#include <stdint.h>

/* DS3218 angle (cdeg, 0..18000) to pulse width and LEDC duty. Two points per
 * servo cover trim, scale, reversed mounting and the 270-degree model. */

/* DS3218 input range, never exceeded whatever the calibration says: past it
 * the servo runs into its end stop. */
#define SERVO_PULSE_MIN_US 500
#define SERVO_PULSE_MAX_US 2500

#define SERVO_SPAN_CDEG 18000

/* 16 bits at 50 Hz is 0.305 us a count, ~10 counts across the 3 us dead band;
 * 13 bits would make one count as wide as the dead band. */
#define SERVO_PWM_FREQ_HZ 50
#define SERVO_DUTY_RESOLUTION_BITS 16

typedef struct {
    int32_t us_at_0;   /* pulse at 0 deg */
    int32_t us_at_180; /* pulse at 180 deg */
} servo_calibration_t;

/* Nominal until measured. The 270-degree model spends 500-2500 us on 270
 * degrees, so 180 is 833.3..2166.7, rounded inward to never overshoot. */
#define SERVO_CALIBRATION_DS3218_180 ((servo_calibration_t){.us_at_0 = 500, .us_at_180 = 2500})
#define SERVO_CALIBRATION_DS3218_270 ((servo_calibration_t){.us_at_0 = 834, .us_at_180 = 2166})

/**
 * @brief Whether a calibration can drive a servo.
 *
 * Both points within SERVO_PULSE_MIN_US..SERVO_PULSE_MAX_US and different.
 */
bool servo_calibration_valid(const servo_calibration_t *cal);

/**
 * @brief Pulse width, in microseconds, that puts the servo at cdeg.
 *
 * Clamps the angle to the span and the pulse to the input range. Rounds halves
 * away from zero so a reversed servo mirrors a normal one exactly.
 */
int32_t servo_pulse_us(const servo_calibration_t *cal, int32_t cdeg);

/**
 * @brief LEDC duty counts for a pulse width.
 *
 * Clamped to the input range like servo_pulse_us().
 */
uint32_t servo_pulse_duty(int32_t pulse_us);

#endif
