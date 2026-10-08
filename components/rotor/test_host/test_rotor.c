#include <stdio.h>
#include <string.h>

#include "rotor.h"

static int s_failures;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            s_failures++;                                                        \
        }                                                                        \
    } while (0)

#define T0 1789763400000LL
#define TOLERANCE_MS 500

typedef struct {
    uint32_t dt_ms;
    uint16_t az;
    int16_t el;
} point_spec_t;

/* Packs a payload the way the server's encoder does. */
static size_t pack(uint8_t *buf, uint8_t flags, int64_t t0_ms, int64_t t_sent_ms, const point_spec_t *points, uint8_t count)
{
    pluto_dto_header_t header = {PLUTO_DTO_MAGIC, flags, count, t0_ms, t_sent_ms};

    memcpy(buf, &header, sizeof(header));
    for (uint8_t i = 0; i < count; i++) {
        pluto_dto_point_t point = {points[i].dt_ms, points[i].az, points[i].el};
        memcpy(buf + PLUTO_DTO_HEADER_LEN + (size_t)i * PLUTO_DTO_POINT_LEN, &point, sizeof(point));
    }
    return PLUTO_DTO_HEADER_LEN + (size_t)count * PLUTO_DTO_POINT_LEN;
}

static const point_spec_t PASS[] = {{0, 4500, 1000}, {1000, 5000, 2000}, {2000, 5500, 3000}};

static void new_rotor(rotor_t *rotor)
{
    pan_tilt_config_t mount = PAN_TILT_CONFIG_DEFAULT;
    rotor_init(rotor, &mount, TOLERANCE_MS);
}

static void test_a_fresh_rotor_reports_idle(void)
{
    rotor_t rotor;
    char json[ROTOR_STATE_JSON_MAX];

    new_rotor(&rotor);
    CHECK(rotor_state_json(&rotor, 1000, true, json, sizeof(json)) > 0);
    CHECK(strcmp(json,
                 "{\"v\":1,\"ts_ms\":1000,\"clock_synced\":true,\"mode\":\"idle\","
                 "\"az_cdeg\":null,\"el_cdeg\":null,\"pan_mode\":null,\"batch\":null,"
                 "\"rejected_total\":0}") == 0);
}

static void test_an_accepted_batch_starts_tracking(void)
{
    rotor_t rotor;
    uint8_t buf[64];
    char json[ROTOR_STATE_JSON_MAX];
    size_t len = pack(buf, 0, T0, T0 - 2000, PASS, 3);

    new_rotor(&rotor);
    CHECK(rotor_handle_payload(&rotor, buf, len, T0 - 1860, true));
    CHECK(rotor.mode == ROTOR_MODE_TRACKING);
    CHECK(rotor.has_pose && rotor.pose.pan_cdeg == 4500 && rotor.pose.tilt_cdeg == 1000);
    CHECK(rotor.last_batch.accepted && rotor.last_batch.latency_ms == 140);

    CHECK(rotor_state_json(&rotor, T0 - 1860, true, json, sizeof(json)) > 0);
    CHECK(strcmp(json,
                 "{\"v\":1,\"ts_ms\":1789763398140,\"clock_synced\":true,\"mode\":\"tracking\","
                 "\"az_cdeg\":4500,\"el_cdeg\":1000,\"pan_mode\":\"normal\","
                 "\"batch\":{\"t_sent_ms\":1789763398000,\"received_ms\":1789763398140,"
                 "\"latency_ms\":140,\"accepted\":true,\"error\":null},\"rejected_total\":0}") == 0);
}

static void test_ticks_follow_the_trajectory_and_then_hold(void)
{
    rotor_t rotor;
    uint8_t buf[64];
    size_t len = pack(buf, 0, T0, T0 - 2000, PASS, 3);

    new_rotor(&rotor);
    rotor_handle_payload(&rotor, buf, len, T0 - 1000, true);

    CHECK(rotor_tick(&rotor, T0 + 500));
    CHECK(rotor.pose.pan_cdeg == 4750 && rotor.pose.tilt_cdeg == 1500);
    CHECK(!rotor_tick(&rotor, T0 + 500)); /* same instant, nothing to move */

    rotor_tick(&rotor, T0 + 5000);
    CHECK(rotor.mode == ROTOR_MODE_HOLDING);
    CHECK(rotor.pose.pan_cdeg == 5500 && rotor.pose.tilt_cdeg == 3000);
}

static void test_the_back_of_the_sky_uses_the_flipped_mode(void)
{
    rotor_t rotor;
    uint8_t buf[64];
    char json[ROTOR_STATE_JSON_MAX];
    const point_spec_t west[] = {{0, 27000, 3000}};
    size_t len = pack(buf, 0, T0, T0 - 100, west, 1);

    new_rotor(&rotor);
    CHECK(rotor_handle_payload(&rotor, buf, len, T0 - 50, true));
    CHECK(rotor.pose.mode == PAN_TILT_FLIPPED && rotor.pose.pan_cdeg == 9000 && rotor.pose.tilt_cdeg == 15000);

    /* The state reports it back in geographic terms. */
    rotor_state_json(&rotor, T0, true, json, sizeof(json));
    CHECK(strstr(json, "\"az_cdeg\":27000,\"el_cdeg\":3000,\"pan_mode\":\"flipped\"") != NULL);
}

static void test_a_decode_error_is_reported_without_a_header(void)
{
    rotor_t rotor;
    uint8_t buf[64];
    char json[ROTOR_STATE_JSON_MAX];
    size_t len = pack(buf, 0, T0, T0 - 2000, PASS, 3);

    buf[0] = 0x51;
    new_rotor(&rotor);
    CHECK(!rotor_handle_payload(&rotor, buf, len, T0, true));
    CHECK(rotor.rejected_total == 1);
    CHECK(rotor.mode == ROTOR_MODE_IDLE);

    rotor_state_json(&rotor, T0, true, json, sizeof(json));
    CHECK(strstr(json, "\"batch\":{\"t_sent_ms\":null,\"received_ms\":1789763400000,\"latency_ms\":null,"
                       "\"accepted\":false,\"error\":\"bad_magic\"},\"rejected_total\":1") != NULL);
}

static void test_the_time_based_rejections(void)
{
    rotor_t rotor;
    uint8_t buf[64];
    char json[ROTOR_STATE_JSON_MAX];
    const point_spec_t below_horizon[] = {{0, 4500, 1000}, {1000, 4600, -200}};
    size_t len;

    new_rotor(&rotor);

    len = pack(buf, 0, T0, T0 - 2000, PASS, 3);
    CHECK(!rotor_handle_payload(&rotor, buf, len, T0 - 1900, false));
    CHECK(strcmp(rotor.last_batch.error, "clock_not_synced") == 0);
    CHECK(!rotor.last_batch.latency_valid); /* no clock, no latency */
    rotor_state_json(&rotor, T0, false, json, sizeof(json));
    CHECK(strstr(json, "\"t_sent_ms\":1789763398000,\"received_ms\":1789763398100,\"latency_ms\":null") != NULL);

    CHECK(!rotor_handle_payload(&rotor, buf, len, T0 + 60000, true));
    CHECK(strcmp(rotor.last_batch.error, "all_expired") == 0);

    len = pack(buf, 0, T0, T0 - 2000, below_horizon, 2);
    CHECK(!rotor_handle_payload(&rotor, buf, len, T0 - 1900, true));
    CHECK(strcmp(rotor.last_batch.error, "unreachable") == 0);

    CHECK(rotor.rejected_total == 3);
    CHECK(rotor.mode == ROTOR_MODE_IDLE);
}

static void test_a_rejected_batch_keeps_the_current_trajectory(void)
{
    rotor_t rotor;
    uint8_t buf[64];
    const point_spec_t below_horizon[] = {{0, 4500, -100}};
    size_t len = pack(buf, 0, T0, T0 - 2000, PASS, 3);

    new_rotor(&rotor);
    rotor_handle_payload(&rotor, buf, len, T0 - 1000, true);
    len = pack(buf, 0, T0, T0 - 1000, below_horizon, 1);
    CHECK(!rotor_handle_payload(&rotor, buf, len, T0 - 900, true));

    rotor_tick(&rotor, T0 + 1000);
    CHECK(rotor.mode == ROTOR_MODE_TRACKING);
    CHECK(rotor.pose.pan_cdeg == 5000 && rotor.pose.tilt_cdeg == 2000);
}

static void test_hold_stops_even_without_a_clock(void)
{
    rotor_t rotor;
    uint8_t buf[64];
    size_t len = pack(buf, 0, T0, T0 - 2000, PASS, 3);

    new_rotor(&rotor);
    rotor_handle_payload(&rotor, buf, len, T0 - 1000, true);
    rotor_tick(&rotor, T0 + 500);

    len = pack(buf, PLUTO_DTO_FLAG_HOLD, T0, T0, NULL, 0);
    CHECK(rotor_handle_payload(&rotor, buf, len, T0 + 600, false));
    CHECK(rotor.mode == ROTOR_MODE_HOLDING);

    /* It stays where it was when the HOLD arrived. */
    CHECK(!rotor_tick(&rotor, T0 + 2000));
    CHECK(rotor.pose.pan_cdeg == 4750 && rotor.pose.tilt_cdeg == 1500);
}

static void test_the_json_refuses_a_short_buffer(void)
{
    rotor_t rotor;
    char json[32];

    new_rotor(&rotor);
    CHECK(rotor_state_json(&rotor, T0, true, json, sizeof(json)) == -1);
}

static void test_the_longest_state_fits(void)
{
    rotor_t rotor;
    char json[ROTOR_STATE_JSON_MAX];

    new_rotor(&rotor);
    rotor.mode = ROTOR_MODE_TRACKING;
    rotor.has_pose = true;
    rotor.pose = (pan_tilt_pose_t){17999, 18000, PAN_TILT_FLIPPED};
    rotor.rejected_total = UINT32_MAX;
    rotor.last_batch = (rotor_batch_report_t){true, true, INT64_MIN, INT64_MIN, true, INT64_MIN, false, "clock_not_synced"};

    CHECK(rotor_state_json(&rotor, INT64_MIN, false, json, sizeof(json)) > 0);
}

int main(void)
{
    test_a_fresh_rotor_reports_idle();
    test_an_accepted_batch_starts_tracking();
    test_ticks_follow_the_trajectory_and_then_hold();
    test_the_back_of_the_sky_uses_the_flipped_mode();
    test_a_decode_error_is_reported_without_a_header();
    test_the_time_based_rejections();
    test_a_rejected_batch_keeps_the_current_trajectory();
    test_hold_stops_even_without_a_clock();
    test_the_json_refuses_a_short_buffer();
    test_the_longest_state_fits();

    if (s_failures) {
        fprintf(stderr, "rotor: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("rotor: all tests passed\n");
    return 0;
}
