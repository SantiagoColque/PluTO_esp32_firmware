#ifndef SERVO_PULSE_H
#define SERVO_PULSE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Servo angle to pulse width, and pulse width to LEDC duty, for the two DS3218
 * of the pan-tilt. Angles are in the servo's own space, the way pan_tilt hands
 * them over: centidegrees from 0 to 18000.
 *
 * Each servo is calibrated by two points: the pulse that puts it at 0 degrees
 * and the one that puts it at 180. The straight line through them absorbs an
 * offset (the trim), a scale error, a servo mounted the other way round (the
 * pulse at 0 above the one at 180) and the 270-degree version of the DS3218,
 * with no parameter for each.
 *
 * No ESP-IDF dependencies, so it runs as a host test (test_host/). The LEDC
 * side lives in servo_driver.h.
 */

/* The DS3218's input range. No pulse outside it leaves the board, whatever the
 * calibration says: past it the servo runs into its end stop. */
#define SERVO_PULSE_MIN_US 500
#define SERVO_PULSE_MAX_US 2500

/* The span a calibration describes: 0 to 180 degrees of the servo. */
#define SERVO_SPAN_CDEG 18000

/* 50 Hz frames with a 16-bit duty. One count is 0.305 us, so the servo's 3 us
 * dead band spans about ten of them and the resolution is never what limits
 * the pointing. 13 bits would make a count as wide as the dead band. */
#define SERVO_PWM_FREQ_HZ 50
#define SERVO_DUTY_RESOLUTION_BITS 16

typedef struct {
    int32_t us_at_0;   /* pulse width that puts the servo at 0 degrees */
    int32_t us_at_180; /* pulse width that puts it at 180 degrees */
} servo_calibration_t;

/*
 * Nominal calibration of each DS3218 version, until the servo is measured.
 * The 180-degree version spends its whole 500-2500 us input on 180 degrees.
 * The 270-degree one spends it on 270, so its 180 degrees are 1333 us centred
 * on 1500 (833.3 to 2166.7), rounded inward so an unsure value never turns the
 * servo further than asked.
 */
#define SERVO_CALIBRATION_DS3218_180 ((servo_calibration_t){.us_at_0 = 500, .us_at_180 = 2500})
#define SERVO_CALIBRATION_DS3218_270 ((servo_calibration_t){.us_at_0 = 834, .us_at_180 = 2166})

/**
 * @brief Whether a calibration can drive a servo.
 *
 * Both points must lie within SERVO_PULSE_MIN_US..SERVO_PULSE_MAX_US and be
 * different; two equal points would hold the servo still whatever the angle.
 */
bool servo_calibration_valid(const servo_calibration_t *cal);

/**
 * @brief Pulse width, in microseconds, that puts the servo at cdeg.
 *
 * The angle is clamped to 0..SERVO_SPAN_CDEG first, so the servo never turns
 * past the two calibrated points, and the result is clamped to the DS3218's
 * input range in case the calibration itself lies outside it. Rounds to the
 * nearest microsecond, halves away from zero, so a reversed servo mirrors a
 * normal one exactly.
 */
int32_t servo_pulse_us(const servo_calibration_t *cal, int32_t cdeg);

/**
 * @brief LEDC duty, in counts of SERVO_DUTY_RESOLUTION_BITS at
 * SERVO_PWM_FREQ_HZ, for a pulse width.
 *
 * Rounds to the nearest count and clamps the pulse to the DS3218's input
 * range like servo_pulse_us() does.
 */
uint32_t servo_pulse_duty(int32_t pulse_us);

#endif
