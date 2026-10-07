#ifndef LOG_RING_H
#define LOG_RING_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "log_ring_core.h"

/**
 * @brief Start copying this firmware's log lines into the ring.
 *
 * Wraps the vprintf that ESP_LOGx ends up in, so the serial monitor keeps
 * printing exactly what it does today. Call it first thing in app_main: lines
 * logged before it, including everything the bootloader prints, are not
 * captured.
 */
void log_ring_install(void);

/**
 * @brief Copy up to `max` of the newest lines into dst, oldest first.
 *
 * Returns ESP_ERR_TIMEOUT if a logging task held the ring for too long, and
 * ESP_ERR_INVALID_STATE before log_ring_install().
 */
esp_err_t log_ring_snapshot(log_ring_entry_t *dst, size_t max, size_t *count);

/**
 * @brief Lines that went to the serial port without being looked at.
 *
 * The ring is skipped rather than waited for when another task holds it, so
 * this counts lines of any tag, ours or not.
 */
uint32_t log_ring_dropped(void);

#endif
