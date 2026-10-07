#include <math.h>
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
#define TICK_MS 50

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

/* Without a motion profile the servos are commanded the target as is, which
 * keeps the tests of the pointing rules free of ramps. */
static void new_rotor(rotor_t *rotor)
{
    pan_tilt_config_t mount = PAN_TILT_CONFIG_DEFAULT;
    rotor_init(rotor, &mount, TOLERANCE_MS, &MOTION_PROFILE_NO_LIMITS);
}

/* The defaults of the motion profile's Kconfig: 60 deg/s, 120 deg/s^2, 5 s. */
static const motion_profile_config_t LIMITS = {6000, 12000, 5000};

static void new_limited_rotor(rotor_t *rotor, const motion_profile_config_t *limits)
{
    pan_tilt_config_t mount = PAN_TILT_CONFIG_DEFAULT;
    rotor_init(rotor, &mount, TOLERANCE_MS, limits);
}

/* Accepts a one-point batch and checks it went in. */
static void send_point(rotor_t *rotor, uint16_t az, int16_t el, int64_t t_ms, int64_t now_ms)
{
    uint8_t buf[64];
    const point_spec_t point[] = {{0, az, el}};
    size_t len = pack(buf, 0, t_ms, now_ms, point, 1);

    CHECK(rotor_handle_payload(rotor, buf, len, now_ms, true));
}

static void send_hold(rotor_t *rotor, int64_t now_ms)
{
    uint8_t buf[64];
    size_t len = pack(buf, PLUTO_DTO_FLAG_HOLD, now_ms, now_ms, NULL, 0);

    CHECK(rotor_handle_payload(rotor, buf, len, now_ms, false));
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

    CHECK(rotor_tick(&rotor, T0 + 500, TICK_MS));
    CHECK(rotor.pose.pan_cdeg == 4750 && rotor.pose.tilt_cdeg == 1500);
    CHECK(!rotor_tick(&rotor, T0 + 500, TICK_MS)); /* same instant, nothing to move */

    rotor_tick(&rotor, T0 + 5000, TICK_MS);
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

    rotor_tick(&rotor, T0 + 1000, TICK_MS);
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
    rotor_tick(&rotor, T0 + 500, TICK_MS);

    len = pack(buf, PLUTO_DTO_FLAG_HOLD, T0, T0, NULL, 0);
    CHECK(rotor_handle_payload(&rotor, buf, len, T0 + 600, false));
    CHECK(rotor.mode == ROTOR_MODE_HOLDING);

    /* It stays where it was when the HOLD arrived. */
    CHECK(!rotor_tick(&rotor, T0 + 2000, TICK_MS));
    CHECK(rotor.pose.pan_cdeg == 4750 && rotor.pose.tilt_cdeg == 1500);
}

static void test_the_mount_decides_what_is_reachable(void)
{
    /* What start_rotor hands in from a stored calibration: a mount facing east
     * whose tilt cannot lean past the zenith. */
    pan_tilt_config_t mount = {.az_ref_cdeg = 9000,
                               .pan_min_cdeg = 0,
                               .pan_max_cdeg = 18000,
                               .tilt_min_cdeg = 0,
                               .tilt_max_cdeg = 9000};
    const point_spec_t south_east[] = {{0, 13500, 2000}};
    const point_spec_t north_east[] = {{0, 4500, 3000}};
    rotor_t rotor;
    uint8_t buf[64];
    size_t len;

    rotor_init(&rotor, &mount, TOLERANCE_MS, &MOTION_PROFILE_NO_LIMITS);

    len = pack(buf, 0, T0, T0 - 100, south_east, 1);
    CHECK(rotor_handle_payload(&rotor, buf, len, T0 - 50, true));
    CHECK(rotor.pose.mode == PAN_TILT_NORMAL && rotor.pose.pan_cdeg == 4500 && rotor.pose.tilt_cdeg == 2000);

    /* Behind the mount it would take the flipped mode, which the tilt limit
     * rules out, although the default mount reaches it. */
    len = pack(buf, 0, T0, T0 - 100, north_east, 1);
    CHECK(!rotor_handle_payload(&rotor, buf, len, T0 - 50, true));
    CHECK(rotor.last_batch.error != NULL && strcmp(rotor.last_batch.error, "unreachable") == 0);
    CHECK(rotor.pose.pan_cdeg == 4500 && rotor.pose.tilt_cdeg == 2000); /* did not move */
}

static void test_a_payload_only_moves_the_target(void)
{
    rotor_t rotor;
    char json[ROTOR_STATE_JSON_MAX];
    char commanded[32];

    new_limited_rotor(&rotor, &LIMITS);
    send_point(&rotor, 4500, 1000, T0, T0 - 1000);
    /* The first pose is taken as is: there is no position to ramp from. */
    CHECK(rotor.pose.pan_cdeg == 4500 && rotor.pose.tilt_cdeg == 1000);

    /* The MQTT task hands in batches between ticks; moving there as well
     * would add steps the profile never accounted for. */
    send_point(&rotor, 9000, 1000, T0, T0 - 900);
    CHECK(rotor.target.pan_cdeg == 9000);
    CHECK(rotor.pose.pan_cdeg == 4500);

    /* The next tick starts the ramp: from rest, a_max * dt^2 = 30 at most. */
    CHECK(rotor_tick(&rotor, T0 - 850, TICK_MS));
    CHECK(rotor.pose.pan_cdeg > 4500 && rotor.pose.pan_cdeg <= 4530);
    CHECK(rotor.mode == ROTOR_MODE_TRACKING);

    /* The state reports what the servos were commanded, not the target. */
    snprintf(commanded, sizeof(commanded), "\"az_cdeg\":%d,", (int)rotor.pose.pan_cdeg);
    CHECK(rotor_state_json(&rotor, T0 - 850, true, json, sizeof(json)) > 0);
    CHECK(strstr(json, commanded) != NULL);
}

static void test_a_hold_brakes_along_a_ramp(void)
{
    rotor_t rotor;
    int64_t now = T0;
    int32_t last_step = 0;
    int ticks = 0;

    new_limited_rotor(&rotor, &LIMITS);
    send_point(&rotor, 1000, 1000, T0, now);
    send_point(&rotor, 17000, 1000, T0, now);
    for (int i = 0; i < 30; i++) {
        now += TICK_MS;
        rotor_tick(&rotor, now, TICK_MS);
    }
    CHECK(rotor.pose.pan_cdeg > 1000 && rotor.pose.pan_cdeg < 17000);

    send_hold(&rotor, now);
    CHECK(rotor.mode == ROTOR_MODE_HOLDING);

    /* It keeps going the same way, each step shorter than the one before by
     * a_max * dt^2 = 30 at most, until it stops: never a full-speed halt in a
     * single tick. The slack of 2 is the rounding of two positions. */
    last_step = 6000 * TICK_MS / 1000;
    while (ticks < 20) {
        int32_t before = rotor.pose.pan_cdeg;
        int32_t step;

        now += TICK_MS;
        if (!rotor_tick(&rotor, now, TICK_MS)) {
            break;
        }
        step = rotor.pose.pan_cdeg - before;
        CHECK(step > 0 && step <= last_step + 2);
        CHECK(last_step - step <= 30 + 2);
        last_step = step;
        ticks++;
    }
    CHECK(ticks >= 5 && ticks < 20);
    CHECK(rotor.pose.pan_cdeg < 17000);
    CHECK(rotor.mode == ROTOR_MODE_HOLDING);

    /* A HOLD dropped the batch, so a stopped rotor stays stopped. */
    CHECK(!rotor_tick(&rotor, now + 10000, TICK_MS));
}

static void test_holding_waits_for_the_servos_to_arrive(void)
{
    rotor_t rotor;
    int64_t now = T0;

    new_limited_rotor(&rotor, &LIMITS);
    send_point(&rotor, 1000, 1000, T0, now);
    send_point(&rotor, 17000, 1000, T0, now);

    /* Past the only point, but still on the way to it. */
    now += TICK_MS;
    rotor_tick(&rotor, now, TICK_MS);
    CHECK(rotor.mode == ROTOR_MODE_TRACKING);

    for (int i = 0; i < 200 && rotor.mode == ROTOR_MODE_TRACKING; i++) {
        now += TICK_MS;
        rotor_tick(&rotor, now, TICK_MS);
    }
    CHECK(rotor.mode == ROTOR_MODE_HOLDING);
    CHECK(rotor.pose.pan_cdeg == 17000);
}

static void test_the_hysteresis_follows_the_target_not_the_servos(void)
{
    rotor_t rotor;
    int64_t now = T0;

    new_limited_rotor(&rotor, &LIMITS);
    send_point(&rotor, 4500, 3000, T0, now);
    /* Due west at 30 degrees: a flip, pan 45 to 90 and tilt 30 to 150. */
    send_point(&rotor, 27000, 3000, T0, now);
    CHECK(rotor.target.mode == PAN_TILT_FLIPPED);
    for (int i = 0; i < 10; i++) {
        now += TICK_MS;
        rotor_tick(&rotor, now, TICK_MS);
    }
    /* Halfway: the tilt has not reached the zenith yet. */
    CHECK(rotor.pose.tilt_cdeg < 9000);
    CHECK(rotor.pose.mode == PAN_TILT_NORMAL);

    /* North at 30 degrees is on the border, reachable in both modes. The
     * flip under way decides, not where the servos happen to be. */
    send_point(&rotor, 0, 3000, now, now);
    CHECK(rotor.target.mode == PAN_TILT_FLIPPED);
    CHECK(rotor.target.pan_cdeg == 18000 && rotor.target.tilt_cdeg == 15000);
}

static void test_a_flip_reports_where_the_servos_point(void)
{
    rotor_t rotor;
    char json[ROTOR_STATE_JSON_MAX];
    int64_t now = T0;
    bool saw_flipped_report = false;

    new_limited_rotor(&rotor, &LIMITS);
    send_point(&rotor, 4500, 3000, T0, now);
    send_point(&rotor, 27000, 3000, T0, now);

    for (int i = 0; i < 100; i++) {
        int32_t az;
        int32_t el;

        now += TICK_MS;
        rotor_tick(&rotor, now, TICK_MS);
        CHECK(rotor.pose.mode == pan_tilt_pose_mode(&rotor.pose));
        pan_tilt_to_azel(&rotor.mount, &rotor.pose, &az, &el);
        CHECK(el >= 0 && el <= 9000);

        rotor_state_json(&rotor, now, true, json, sizeof(json));
        if (rotor.pose.tilt_cdeg > 9000) {
            saw_flipped_report = saw_flipped_report || strstr(json, "\"pan_mode\":\"flipped\"") != NULL;
        } else if (rotor.pose.tilt_cdeg < 9000) {
            CHECK(strstr(json, "\"pan_mode\":\"normal\"") != NULL);
        }
    }
    CHECK(saw_flipped_report);
    CHECK(rotor.pose.pan_cdeg == 9000 && rotor.pose.tilt_cdeg == 15000);
}

static void test_the_outputs_rest_and_wake(void)
{
    rotor_t rotor;
    int64_t now = T0;

    new_limited_rotor(&rotor, &LIMITS);
    /* Nothing commanded yet, so nothing to hold. */
    CHECK(rotor_outputs_resting(&rotor));

    send_point(&rotor, 4500, 3000, T0, now);
    CHECK(!rotor_outputs_resting(&rotor));

    for (int i = 0; i < 5000 / TICK_MS - 1; i++) {
        now += TICK_MS;
        rotor_tick(&rotor, now, TICK_MS);
    }
    CHECK(!rotor_outputs_resting(&rotor));
    now += TICK_MS;
    rotor_tick(&rotor, now, TICK_MS);
    CHECK(rotor_outputs_resting(&rotor));

    /* A batch somewhere else wakes them as it arrives, before the next tick. */
    send_point(&rotor, 4600, 3000, now + 1000, now);
    CHECK(!rotor_outputs_resting(&rotor));

    /* A rotor without a rest timeout never rests once it has moved. */
    new_rotor(&rotor);
    send_point(&rotor, 4500, 3000, T0, T0);
    for (int i = 0; i < 1000; i++) {
        rotor_tick(&rotor, T0 + i * TICK_MS, TICK_MS);
    }
    CHECK(!rotor_outputs_resting(&rotor));
}

/* What main's drive_servos() relies on to release and resume the outputs: a
 * batch that only repeats where the axes rest moves nothing and keeps them at
 * rest, while after a HOLD any target, even the spot they stopped on, wakes
 * them with the commanded pose unchanged. */
static void test_only_something_to_do_wakes_the_outputs(void)
{
    rotor_t rotor;
    int64_t now = T0;
    pan_tilt_pose_t rested;
    int32_t az;
    int32_t el;

    new_limited_rotor(&rotor, &LIMITS);
    send_point(&rotor, 4500, 3000, T0, now);
    for (int i = 0; i < 5000 / TICK_MS; i++) {
        now += TICK_MS;
        rotor_tick(&rotor, now, TICK_MS);
    }
    CHECK(rotor_outputs_resting(&rotor));

    send_point(&rotor, 4500, 3000, now, now);
    CHECK(rotor_outputs_resting(&rotor));
    now += TICK_MS;
    CHECK(!rotor_tick(&rotor, now, TICK_MS));
    CHECK(rotor_outputs_resting(&rotor));

    send_hold(&rotor, now);
    for (int i = 0; i < 5000 / TICK_MS; i++) {
        now += TICK_MS;
        rotor_tick(&rotor, now, TICK_MS);
    }
    CHECK(rotor_outputs_resting(&rotor));

    rested = rotor.pose;
    pan_tilt_to_azel(&rotor.mount, &rested, &az, &el);
    send_point(&rotor, (uint16_t)az, (int16_t)el, now, now);
    CHECK(!rotor_outputs_resting(&rotor));
    now += TICK_MS;
    CHECK(!rotor_tick(&rotor, now, TICK_MS));
    CHECK(rotor.pose.pan_cdeg == rested.pan_cdeg && rotor.pose.tilt_cdeg == rested.tilt_cdeg);
    CHECK(!rotor_outputs_resting(&rotor));
}

/*
 * A pass culminating at 88 degrees, from a 500 km orbit, crossing the sky
 * from north to south east of the station. The pan follows the azimuth, which
 * turns at about 25 deg/s at the top: the fastest the profile has to keep up
 * with during a pass. Flat-earth geometry is plenty for the rates.
 */
static void high_pass_point(int32_t t_s, uint16_t *az, int16_t *el)
{
    const double height_km = 500.0;
    const double east_km = height_km / tan(88.0 * M_PI / 180.0);
    const double north_km = -7.6 * t_s;
    double az_deg = atan2(east_km, north_km) * 180.0 / M_PI;
    double el_deg = atan2(height_km, hypot(east_km, north_km)) * 180.0 / M_PI;

    *az = (uint16_t)lround(az_deg * 100.0);
    *el = (int16_t)lround(el_deg * 100.0);
}

/* Angle between two directions given as az/el in centidegrees, in degrees. */
static double pointing_error_deg(int32_t az1, int32_t el1, int32_t az2, int32_t el2)
{
    double a1 = az1 * M_PI / 18000.0;
    double e1 = el1 * M_PI / 18000.0;
    double a2 = az2 * M_PI / 18000.0;
    double e2 = el2 * M_PI / 18000.0;
    double h = pow(sin((e2 - e1) / 2.0), 2) + cos(e1) * cos(e2) * pow(sin((a2 - a1) / 2.0), 2);

    return 2.0 * asin(sqrt(h)) * 180.0 / M_PI;
}

/* Follows the high pass and returns the worst pointing error against the
 * target, in degrees. */
static double worst_error_on_a_high_pass(const motion_profile_config_t *limits)
{
    rotor_t rotor;
    uint8_t buf[256];
    point_spec_t points[16];
    double worst = 0.0;

    /* 16 points a second apart, culminating at the ninth. */
    for (int i = 0; i < 16; i++) {
        points[i].dt_ms = (uint32_t)i * 1000;
        high_pass_point(i - 8, &points[i].az, &points[i].el);
    }

    new_limited_rotor(&rotor, limits);
    CHECK(rotor_handle_payload(&rotor, buf, pack(buf, 0, T0, T0 - 100, points, 16), T0 - 100, true));

    for (int64_t now = T0; now <= T0 + 15000; now += TICK_MS) {
        int32_t target_az;
        int32_t target_el;
        int32_t az;
        int32_t el;

        rotor_tick(&rotor, now, TICK_MS);
        CHECK(rotor.pose.mode == PAN_TILT_NORMAL); /* this pass never needs a flip */
        trajectory_target(&rotor.trajectory, now, &target_az, &target_el);
        pan_tilt_to_azel(&rotor.mount, &rotor.pose, &az, &el);
        worst = fmax(worst, pointing_error_deg(az, el, target_az, target_el));
    }
    return worst;
}

static void test_a_high_pass_keeps_up(void)
{
    const motion_profile_config_t too_slow = {1000, 12000, 5000};

    /* The pan lags a couple of degrees at the top, but that close to the
     * zenith an azimuth error barely moves the beam. */
    CHECK(worst_error_on_a_high_pass(&LIMITS) < 0.5);

    /* At 10 deg/s it falls tens of degrees behind and misses by more as the
     * pass comes down: v_max cannot go that low. */
    CHECK(worst_error_on_a_high_pass(&too_slow) > 1.0);
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
    test_the_mount_decides_what_is_reachable();
    test_a_payload_only_moves_the_target();
    test_a_hold_brakes_along_a_ramp();
    test_holding_waits_for_the_servos_to_arrive();
    test_the_hysteresis_follows_the_target_not_the_servos();
    test_a_flip_reports_where_the_servos_point();
    test_the_outputs_rest_and_wake();
    test_only_something_to_do_wakes_the_outputs();
    test_a_high_pass_keeps_up();
    test_the_json_refuses_a_short_buffer();
    test_the_longest_state_fits();

    if (s_failures) {
        fprintf(stderr, "rotor: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("rotor: all tests passed\n");
    return 0;
}
