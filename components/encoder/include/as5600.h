#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/* One AS5600 per I2C controller, its address 0x36 is fixed. New i2c_master
 * driver only: the legacy driver/i2c.h aborts at boot if both are linked.
 * Never writes the OTP registers; the zero lives in encoder_angle.h. Chip not
 * confirmed yet, registers per the datasheet. Not thread-safe: rotor task only. */

typedef struct {
    const char *name;            /* "pan" or "tilt", for the log */
    i2c_master_dev_handle_t dev; /* NULL until as5600_init() succeeds */
    const char *issue;           /* last problem logged, NULL while fine */
} as5600_t;

/**
 * @brief Start I2C controller port (0 or 1) with one AS5600.
 *
 * Sends nothing, so a missing chip shows on the first read.
 */
esp_err_t as5600_init(as5600_t *encoder, const char *name, int port, int sda_gpio, int scl_gpio);

/**
 * @brief Read the raw angle, 0..4095.
 *
 * ESP_ERR_NOT_FOUND without a magnet. Problems are logged when they appear and
 * clear, not on every read.
 */
esp_err_t as5600_read(as5600_t *encoder, uint16_t *raw);

#endif
