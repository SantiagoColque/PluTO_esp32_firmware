#ifndef ENCODER_ANGLE_H
#define ENCODER_ANGLE_H

#include <stdint.h>

/*
 * A magnetic encoder's raw count to the servo angle it reads, in the servo
 * space of pan_tilt.h: centidegrees, 0 to 18000 over the mount's travel.
 *
 * No ESP-IDF dependencies, so it runs as a host test (test_host/). The I2C
 * side lives in as5600.h.
 */

/* 12 bits a turn, as the AS5600 reads it. */
#define ENCODER_COUNTS 4096

typedef struct {
    int32_t zero_raw; /* count read with the servo at 0 degrees */
    int32_t dir;      /* +1 if the count grows with the servo angle, -1 if it shrinks */
} encoder_calibration_t;

/**
 * @brief Servo angle for a raw count, in -9000..26999 centidegrees.
 *
 * The encoder turns a full circle while the servo uses half of one, so the
 * circle is split around the middle of the travel: a little past either end
 * reads as just below 0 or just above 18000, never as the far side.
 */
int32_t encoder_angle_cdeg(const encoder_calibration_t *cal, uint16_t raw);

#endif
