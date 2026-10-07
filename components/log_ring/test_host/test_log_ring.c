#include <stdio.h>
#include <string.h>

#include "log_ring_core.h"

static int s_failures;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            s_failures++;                                                        \
        }                                                                        \
    } while (0)

/* An entry whose message is "line <n>" and whose timestamp is n. */
static log_ring_entry_t numbered(unsigned n)
{
    log_ring_entry_t entry = {.level = 'I', .t_ms = n};

    strcpy(entry.tag, "pluto_main");
    snprintf(entry.msg, sizeof(entry.msg), "line %u", n);
    return entry;
}

static void test_partial_ring_keeps_order(void)
{
    log_ring_t ring;
    log_ring_entry_t out[LOG_RING_CAPACITY];
    size_t n;

    log_ring_reset(&ring);
    CHECK(log_ring_copy(&ring, out, LOG_RING_CAPACITY) == 0);

    for (unsigned i = 1; i <= 5; i++) {
        log_ring_entry_t entry = numbered(i);
        log_ring_push(&ring, &entry);
    }
    n = log_ring_copy(&ring, out, LOG_RING_CAPACITY);
    CHECK(n == 5);
    for (size_t i = 0; i < n; i++) {
        CHECK(out[i].t_ms == i + 1);
    }
}

/* The behaviour the page exists for: line 21 pushes line 1 out. */
static void test_line_21_evicts_line_1(void)
{
    log_ring_t ring;
    log_ring_entry_t out[LOG_RING_CAPACITY];
    size_t n;

    log_ring_reset(&ring);
    for (unsigned i = 1; i <= LOG_RING_CAPACITY + 1; i++) {
        log_ring_entry_t entry = numbered(i);
        log_ring_push(&ring, &entry);
    }

    n = log_ring_copy(&ring, out, LOG_RING_CAPACITY);
    CHECK(n == LOG_RING_CAPACITY);
    CHECK(out[0].t_ms == 2);
    CHECK(strcmp(out[0].msg, "line 2") == 0);
    CHECK(out[n - 1].t_ms == LOG_RING_CAPACITY + 1);
    for (size_t i = 1; i < n; i++) {
        CHECK(out[i].t_ms == out[i - 1].t_ms + 1);
    }
}

/* Several laps around the ring, so head wraps more than once. */
static void test_many_laps(void)
{
    log_ring_t ring;
    log_ring_entry_t out[LOG_RING_CAPACITY];
    const unsigned total = LOG_RING_CAPACITY * 3 + 7;

    log_ring_reset(&ring);
    for (unsigned i = 1; i <= total; i++) {
        log_ring_entry_t entry = numbered(i);
        log_ring_push(&ring, &entry);
    }

    CHECK(log_ring_copy(&ring, out, LOG_RING_CAPACITY) == LOG_RING_CAPACITY);
    CHECK(out[0].t_ms == total - LOG_RING_CAPACITY + 1);
    CHECK(out[LOG_RING_CAPACITY - 1].t_ms == total);
}

/* Asking for fewer than are held returns the newest ones, still oldest first. */
static void test_copy_fewer_than_held(void)
{
    log_ring_t ring;
    log_ring_entry_t out[3];

    log_ring_reset(&ring);
    for (unsigned i = 1; i <= 10; i++) {
        log_ring_entry_t entry = numbered(i);
        log_ring_push(&ring, &entry);
    }

    CHECK(log_ring_copy(&ring, out, 3) == 3);
    CHECK(out[0].t_ms == 8);
    CHECK(out[2].t_ms == 10);
}

static void test_parses_monitor_lines(void)
{
    log_ring_entry_t e;

    CHECK(log_ring_parse("I (52125) pluto_main: Batch accepted (59 bytes), mode tracking\n", &e));
    CHECK(e.level == 'I');
    CHECK(e.t_ms == 52125);
    CHECK(strcmp(e.tag, "pluto_main") == 0);
    CHECK(strcmp(e.msg, "Batch accepted (59 bytes), mode tracking") == 0);

    CHECK(log_ring_parse("W (7) wifi_manager: Reconnecting in 2.0 s (attempt 3)\n", &e));
    CHECK(e.level == 'W' && e.t_ms == 7);

    CHECK(log_ring_parse("E (4294967295) mqtt_manager: MQTT error event received\n", &e));
    CHECK(e.level == 'E' && e.t_ms == 4294967295u);

    /* The debug dump in main.c indents its lines; the indent is message. */
    CHECK(log_ring_parse("I (52125) pluto_main:   [0] t+488 ms  az=124.00 el=45.30\n", &e));
    CHECK(strcmp(e.msg, "  [0] t+488 ms  az=124.00 el=45.30") == 0);

    /* A colon inside the message does not move the tag boundary. */
    CHECK(log_ring_parse("I (1) web_server: listening: port 80\n", &e));
    CHECK(strcmp(e.tag, "web_server") == 0);
    CHECK(strcmp(e.msg, "listening: port 80") == 0);
}

static void test_rejects_lines_without_prefix(void)
{
    log_ring_entry_t e;
    char long_tag[64];

    CHECK(!log_ring_parse("plain text from printf\n", &e));
    CHECK(e.level == '?' && e.tag[0] == '\0');
    CHECK(strcmp(e.msg, "plain text from printf") == 0);

    CHECK(!log_ring_parse("", &e));
    CHECK(!log_ring_parse("I", &e));
    CHECK(!log_ring_parse("X (12) tag: unknown level\n", &e));
    CHECK(!log_ring_parse("I () tag: no digits\n", &e));
    CHECK(!log_ring_parse("I (12)tag: no space\n", &e));
    CHECK(!log_ring_parse("I (12) : empty tag\n", &e));
    CHECK(!log_ring_parse("I (12) no separator at all\n", &e));
    /* System-time prefix, which the component refuses to build with. */
    CHECK(!log_ring_parse("I (12:34:56.789) pluto_main: text\n", &e));

    memset(long_tag, 'x', sizeof(long_tag));
    memcpy(long_tag, "I (1) ", 6);
    strcpy(long_tag + 6 + LOG_RING_TAG_LEN, ": text");
    CHECK(!log_ring_parse(long_tag, &e));
}

static void test_truncates_long_messages(void)
{
    char line[LOG_RING_MSG_LEN * 2 + 32];
    log_ring_entry_t e;

    strcpy(line, "I (1) pluto_main: ");
    memset(line + strlen(line), 'a', LOG_RING_MSG_LEN * 2);
    line[sizeof(line) - 1] = '\0';

    CHECK(log_ring_parse(line, &e));
    CHECK(strlen(e.msg) == LOG_RING_MSG_LEN - 1);
}

static void test_control_characters_become_spaces(void)
{
    log_ring_entry_t e;

    CHECK(log_ring_parse("I (1) pluto_main: a\tb\x1b[0mc\r\n", &e));
    CHECK(strcmp(e.msg, "a b [0mc") == 0);
}

static void test_only_our_tags(void)
{
    const char *ours[] = {"pluto_main", "mqtt_manager", "wifi_manager", "web_server",
                          "web_api", "config_store", "sntp_manager", "dns_hijack"};
    const char *theirs[] = {"wifi", "esp_netif_handlers", "mqtt_client", "httpd_uri", "", "pluto"};

    for (size_t i = 0; i < sizeof(ours) / sizeof(ours[0]); i++) {
        CHECK(log_ring_tag_is_ours(ours[i]));
    }
    for (size_t i = 0; i < sizeof(theirs) / sizeof(theirs[0]); i++) {
        CHECK(!log_ring_tag_is_ours(theirs[i]));
    }
}

static void test_redacts_the_dashboard_password(void)
{
    log_ring_entry_t e;

    CHECK(log_ring_parse("I (520) config_store: Dashboard login: admin / pluto-699190\n", &e));
    log_ring_redact(&e);
    CHECK(strcmp(e.msg, "Dashboard login: <redacted>") == 0);
    CHECK(strstr(e.msg, "pluto-699190") == NULL);

    /* Other config_store lines and the same text under another tag are kept. */
    CHECK(log_ring_parse("I (520) config_store: Configuration loaded: provisioned=1\n", &e));
    log_ring_redact(&e);
    CHECK(strcmp(e.msg, "Configuration loaded: provisioned=1") == 0);

    CHECK(log_ring_parse("I (520) pluto_main: Dashboard login: is a phrase\n", &e));
    log_ring_redact(&e);
    CHECK(strcmp(e.msg, "Dashboard login: is a phrase") == 0);
}

int main(void)
{
    test_partial_ring_keeps_order();
    test_line_21_evicts_line_1();
    test_many_laps();
    test_copy_fewer_than_held();
    test_parses_monitor_lines();
    test_rejects_lines_without_prefix();
    test_truncates_long_messages();
    test_control_characters_become_spaces();
    test_only_our_tags();
    test_redacts_the_dashboard_password();

    if (s_failures) {
        fprintf(stderr, "log_ring: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("log_ring: all tests passed (capacity %d)\n", LOG_RING_CAPACITY);
    return 0;
}
