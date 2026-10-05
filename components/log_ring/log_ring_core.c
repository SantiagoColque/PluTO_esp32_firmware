#include "log_ring_core.h"

#include <stdio.h>
#include <string.h>

/*
 * The tags this firmware logs under. A component added later with a tag of its
 * own does not show on the /logs page until it is listed here.
 */
static const char *const s_our_tags[] = {
    "pluto_main",
    "mqtt_manager",
    "wifi_manager",
    "web_server",
    "web_api",
    "config_store",
    "sntp_manager",
    "dns_hijack",
};

/* config_store prints the dashboard password at boot so the owner can read the
 * MAC-derived default off the serial port. Served over HTTP it would be readable
 * by anyone on the provisioning access point, which skips authentication. */
#define REDACT_TAG "config_store"
#define REDACT_PREFIX "Dashboard login:"

void log_ring_reset(log_ring_t *ring)
{
    memset(ring, 0, sizeof(*ring));
}

/* Copies text into dst, turning control characters into spaces and dropping
 * the newline the log macros append. */
static void copy_message(char *dst, size_t dst_len, const char *src)
{
    size_t len = 0;

    while (src[len] != '\0' && len < dst_len - 1) {
        unsigned char c = (unsigned char)src[len];

        dst[len] = c < 0x20 || c == 0x7f ? ' ' : (char)c;
        len++;
    }
    while (len > 0 && dst[len - 1] == ' ') {
        len--;
    }
    dst[len] = '\0';
}

static bool is_level(char c)
{
    return c != '\0' && strchr("EWIDV", c) != NULL;
}

bool log_ring_parse(const char *rendered, log_ring_entry_t *out)
{
    const char *p = rendered;
    const char *tag_end;
    uint32_t t_ms = 0;
    int digits = 0;
    size_t tag_len;

    memset(out, 0, sizeof(*out));

    if (!is_level(p[0]) || p[1] != ' ' || p[2] != '(') {
        goto unparsed;
    }
    p += 3;
    /* Ten digits is all a uint32_t can hold. */
    while (*p >= '0' && *p <= '9' && digits < 10) {
        t_ms = t_ms * 10 + (uint32_t)(*p - '0');
        p++;
        digits++;
    }
    if (digits == 0 || p[0] != ')' || p[1] != ' ') {
        goto unparsed;
    }
    p += 2;

    tag_end = strstr(p, ": ");
    if (tag_end == NULL || tag_end == p) {
        goto unparsed;
    }
    tag_len = (size_t)(tag_end - p);
    if (tag_len >= LOG_RING_TAG_LEN) {
        goto unparsed;
    }

    out->level = rendered[0];
    out->t_ms = t_ms;
    memcpy(out->tag, p, tag_len);
    out->tag[tag_len] = '\0';
    copy_message(out->msg, sizeof(out->msg), tag_end + 2);
    return true;

unparsed:
    out->level = '?';
    copy_message(out->msg, sizeof(out->msg), rendered);
    return false;
}

bool log_ring_tag_is_ours(const char *tag)
{
    for (size_t i = 0; i < sizeof(s_our_tags) / sizeof(s_our_tags[0]); i++) {
        if (strcmp(tag, s_our_tags[i]) == 0) {
            return true;
        }
    }
    return false;
}

void log_ring_redact(log_ring_entry_t *entry)
{
    if (strcmp(entry->tag, REDACT_TAG) == 0 &&
        strncmp(entry->msg, REDACT_PREFIX, sizeof(REDACT_PREFIX) - 1) == 0) {
        snprintf(entry->msg, sizeof(entry->msg), "%s <redacted>", REDACT_PREFIX);
    }
}

void log_ring_push(log_ring_t *ring, const log_ring_entry_t *entry)
{
    ring->entries[ring->head] = *entry;
    ring->head = (ring->head + 1) % LOG_RING_CAPACITY;
    if (ring->count < LOG_RING_CAPACITY) {
        ring->count++;
    }
}

size_t log_ring_copy(const log_ring_t *ring, log_ring_entry_t *dst, size_t max)
{
    size_t n = ring->count < max ? ring->count : max;
    /* head is one past the newest line, so the oldest of the n newest sits n
     * slots behind it. */
    size_t start = (ring->head + LOG_RING_CAPACITY - n) % LOG_RING_CAPACITY;

    for (size_t i = 0; i < n; i++) {
        dst[i] = ring->entries[(start + i) % LOG_RING_CAPACITY];
    }
    return n;
}
