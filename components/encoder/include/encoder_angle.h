#ifndef ENCODER_ANGLE_H
#define ENCODER_ANGLE_H

#include <stdint.h>

/* Encoder raw count to servo angle, in the cdeg servo space of pan_tilt.h. */

/* AS5600: 12 bits a turn. */
#define ENCODER_COUNTS 4096

typedef struct {
    int32_t zero_raw; /* count at servo 0 degrees */
    int32_t dir;      /* +1 if the count grows with the angle, -1 if it shrinks */
} encoder_calibration_t;

/**
 * @brief Servo angle for a raw count, in -9000..26999 centidegrees.
 *
 * The circle splits at mid-travel, so a little past either end reads just below
 * 0 or above 18000, never as the far side.
 */
int32_t encoder_angle_cdeg(const encoder_calibration_t *cal, uint16_t raw);

#endif
