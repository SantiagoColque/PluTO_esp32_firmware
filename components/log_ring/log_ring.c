#include "log_ring.h"

#include <stdarg.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/* With log version 1, ESP-IDF hands the vprintf hook one complete line per
 * call (components/log/src/log.c). Version 2 assembles a line from several
 * calls, which would land in the ring as fragments. */
#if CONFIG_LOG_VERSION != 1
#error "log_ring needs CONFIG_LOG_VERSION=1: version 2 calls the vprintf hook several times per line"
#endif

/* The parser expects the "I (52125) tag: " prefix of RTOS timestamps. With
 * system time the prefix becomes "I (12:34:56.789) tag: ", no line would parse,
 * and the page would stay empty without saying why. */
#if !CONFIG_LOG_TIMESTAMP_SOURCE_RTOS
#error "log_ring needs CONFIG_LOG_TIMESTAMP_SOURCE_RTOS: it parses the millisecond prefix"
#endif

/* The rendered line is the message plus a prefix of at most ~40 bytes. */
#define RENDER_LEN (LOG_RING_MSG_LEN + LOG_RING_TAG_LEN + 24)
#define SNAPSHOT_WAIT_MS 100

static log_ring_t s_ring;
static SemaphoreHandle_t s_lock;
static uint32_t s_dropped;

/* Static rather than on the stack: the hook runs on the stack of whichever
 * task logs, and sys_evt has 2.3 KB. Both are only touched with s_lock held.
 * The line is parsed here instead of straight into the ring, because a line
 * that turns out not to be ours would otherwise overwrite the oldest entry
 * while count still counts it. */
static char s_render[RENDER_LEN];
static log_ring_entry_t s_scratch;

/* Initialised to the default so a line logged between the swap in
 * log_ring_install() and the assignment of its result still reaches the UART. */
static vprintf_like_t s_next = vprintf;

static bool can_take_lock(void)
{
    return s_lock != NULL && !xPortInIsrContext() && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

/*
 * Nothing in here may log: the call would come straight back into this hook.
 *
 * ESP-IDF calls it without holding any lock of its own, from any task, so the
 * ring has its own mutex. It is taken without waiting: when the HTTP handler
 * holds it and logs, its own line is skipped instead of deadlocking, and a busy
 * ring never delays a line on its way to the UART.
 */
static int log_ring_vprintf(const char *format, va_list args)
{
    if (can_take_lock() && xSemaphoreTake(s_lock, 0) == pdTRUE) {
        va_list copy;

        /* args is consumed once, by the UART below. */
        va_copy(copy, args);
        vsnprintf(s_render, sizeof(s_render), format, copy);
        va_end(copy);

        if (log_ring_parse(s_render, &s_scratch) && log_ring_tag_is_ours(s_scratch.tag)) {
            log_ring_redact(&s_scratch);
            log_ring_push(&s_ring, &s_scratch);
        }
        xSemaphoreGive(s_lock);
    } else if (s_lock != NULL) {
        __atomic_fetch_add(&s_dropped, 1, __ATOMIC_RELAXED);
    }

    return s_next(format, args);
}

void log_ring_install(void)
{
    if (s_lock != NULL) {
        return;
    }

    log_ring_reset(&s_ring);
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        /* The page stays empty; the serial port is unaffected. */
        return;
    }

    s_next = esp_log_set_vprintf(log_ring_vprintf);
}

esp_err_t log_ring_snapshot(log_ring_entry_t *dst, size_t max, size_t *count)
{
    *count = 0;

    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(SNAPSHOT_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *count = log_ring_copy(&s_ring, dst, max);
    xSemaphoreGive(s_lock);

    return ESP_OK;
}

uint32_t log_ring_dropped(void)
{
    return __atomic_load_n(&s_dropped, __ATOMIC_RELAXED);
}
