#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/*
 * An AS5600 magnetic encoder alone on an I2C bus of its own: its address,
 * 0x36, is fixed, so the pan and the tilt each take one of the ESP32's two
 * controllers. Only the new i2c_master driver: the legacy driver/i2c.h aborts
 * at boot if both are linked.
 *
 * It reads the raw angle and the magnet status, nothing else, and never
 * writes the chip's one-time-programmable registers: the zero lives in the
 * firmware (encoder_angle.h). The chip is not confirmed yet, so the registers
 * are the AS5600 datasheet's.
 *
 * Not thread-safe: read each encoder from one task, the rotor task.
 */

typedef struct {
    const char *name;            /* for the log, "pan" or "tilt" */
    i2c_master_dev_handle_t dev; /* NULL until as5600_init() succeeds */
    const char *issue;           /* the last problem logged, NULL while reading fine */
} as5600_t;

/**
 * @brief Start I2C controller port (0 or 1) on its pins with one AS5600.
 *
 * Nothing is sent yet, so a missing chip only shows on the first read.
 */
esp_err_t as5600_init(as5600_t *encoder, const char *name, int port, int sda_gpio, int scl_gpio);

/**
 * @brief Read the raw angle, 0..4095.
 *
 * Fails with ESP_ERR_NOT_FOUND when the chip sees no magnet, with the I2C
 * error when the chip does not answer, and with ESP_ERR_INVALID_STATE after a
 * failed init. A magnet too close or too far still reads, less accurately.
 * Every problem is logged when it appears and when it clears, not on each
 * read, so a missing encoder does not flood the log.
 */
esp_err_t as5600_read(as5600_t *encoder, uint16_t *raw);

#endif
