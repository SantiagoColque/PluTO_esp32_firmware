#ifndef LOG_RING_CORE_H
#define LOG_RING_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * A fixed ring of the most recent log lines, for the /logs page of the
 * dashboard. When it is full, each new line evicts the oldest one.
 *
 * This half is pure: no ESP-IDF, no locking, so the eviction and the parsing
 * run as a host test (test_host/). The hook that feeds it and the mutex that
 * guards it live in log_ring.c.
 */

/* Set from CONFIG_PLUTO_LOG_RING_LINES by the component's CMakeLists. The
 * fallback is what the host test builds against. */
#ifndef LOG_RING_CAPACITY
#define LOG_RING_CAPACITY 20
#endif

#define LOG_RING_TAG_LEN 24
#define LOG_RING_MSG_LEN 144

typedef struct {
    char level;     /* 'E', 'W', 'I', 'D', 'V', or '?' when the line had no prefix */
    uint32_t t_ms;  /* milliseconds since boot, as the monitor prints it */
    char tag[LOG_RING_TAG_LEN];
    char msg[LOG_RING_MSG_LEN];
} log_ring_entry_t;

typedef struct {
    log_ring_entry_t entries[LOG_RING_CAPACITY];
    size_t head;  /* next slot to write */
    size_t count; /* saturates at LOG_RING_CAPACITY */
} log_ring_t;

void log_ring_reset(log_ring_t *ring);

/**
 * @brief Split a rendered line such as "I (52125) pluto_main: text\n".
 *
 * Returns false when the line does not have that shape; `out` then holds level
 * '?', an empty tag and the whole line as the message. Control characters
 * become spaces and the trailing newline is dropped, so the message is safe to
 * put in JSON and in the page as is.
 */
bool log_ring_parse(const char *rendered, log_ring_entry_t *out);

/** Whether the tag belongs to this firmware rather than to ESP-IDF. */
bool log_ring_tag_is_ours(const char *tag);

/** Blank out anything that must not leave the board over HTTP. */
void log_ring_redact(log_ring_entry_t *entry);

/** Append a line, evicting the oldest one when the ring is full. */
void log_ring_push(log_ring_t *ring, const log_ring_entry_t *entry);

/**
 * @brief Copy up to `max` of the newest lines into dst, oldest first.
 *
 * Returns how many were copied.
 */
size_t log_ring_copy(const log_ring_t *ring, log_ring_entry_t *dst, size_t max);

#endif
