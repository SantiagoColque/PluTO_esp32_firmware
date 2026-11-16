#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "motion_profile.h"

static int s_failures;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            s_failures++;                                                        \
        }                                                                        \
    } while (0)

#define TICK_MS 50
#define V_MAX 6000  /* 60 degrees per second */
#define A_MAX 12000 /* 120 degrees per second squared */
#define REST_MS 5000

/* Float rounding allowed in a measured speed, in centidegrees per second. */
#define SPEED_SLACK 1.0f

static const motion_profile_config_t LIMITED = {V_MAX, A_MAX, REST_MS};

/*
 * What the servo would see: speeds and accelerations measured from the
 * successive positions, not read from the profile's own state.
 */
typedef struct {
    float last_pos;
    float last_vel;
    float max_speed;
    float max_accel;
    float min_pos;
    float max_pos;
} trace_t;

static void trace_start(trace_t *trace, const motion_profile_t *axis)
{
    trace->last_pos = axis->pos_cdeg;
    trace->last_vel = 0.0f;
    trace->max_speed = 0.0f;
    trace->max_accel = 0.0f;
    trace->min_pos = axis->pos_cdeg;
    trace->max_pos = axis->pos_cdeg;
}

static void trace_record(trace_t *trace, const motion_profile_t *axis, int32_t dt_ms)
{
    float dt_s = (float)dt_ms / 1000.0f;
    float vel = (axis->pos_cdeg - trace->last_pos) / dt_s;
    float accel = fabsf(vel - trace->last_vel) / dt_s;

    trace->max_speed = fmaxf(trace->max_speed, fabsf(vel));
    trace->max_accel = fmaxf(trace->max_accel, accel);
    trace->min_pos = fminf(trace->min_pos, axis->pos_cdeg);
    trace->max_pos = fmaxf(trace->max_pos, axis->pos_cdeg);
    trace->last_pos = axis->pos_cdeg;
    trace->last_vel = vel;
}

static bool within_limits(const trace_t *trace, const motion_profile_config_t *cfg, int32_t dt_ms)
{
    float dt_s = (float)dt_ms / 1000.0f;

    return trace->max_speed <= cfg->v_max_cdeg_s + SPEED_SLACK &&
           trace->max_accel <= cfg->a_max_cdeg_s2 + SPEED_SLACK / dt_s;
}

/* Steps toward a fixed target until settled, recording the trace. Returns the
 * number of steps, or -1 if it never settled. */
static int run_to(motion_profile_t *axis, int32_t target, int32_t dt_ms, trace_t *trace)
{
    for (int i = 1; i <= 2000; i++) {
        motion_profile_step(axis, target, dt_ms);
        trace_record(trace, axis, dt_ms);
        if (motion_profile_settled(axis)) {
            return i;
        }
    }
    return -1;
}

/* Steps toward a target for a while, to get the axis moving. */
static void move_for(motion_profile_t *axis, int32_t target, int steps, trace_t *trace)
{
    for (int i = 0; i < steps; i++) {
        motion_profile_step(axis, target, TICK_MS);
        trace_record(trace, axis, TICK_MS);
    }
}

static void test_the_first_target_is_taken_as_is(void)
{
    motion_profile_t axis;

    motion_profile_init(&axis, &LIMITED);
    CHECK(!motion_profile_has_position(&axis));

    /* Nothing to ramp from: a servo without a sensor gives no position. */
    CHECK(motion_profile_step(&axis, 12345, TICK_MS) == 12345);
    CHECK(motion_profile_has_position(&axis));
    CHECK(motion_profile_settled(&axis));
}

static void test_a_reset_starts_from_a_known_position_at_rest(void)
{
    motion_profile_t axis;

    motion_profile_init(&axis, &LIMITED);
    motion_profile_reset(&axis, 5000);
    CHECK(motion_profile_has_position(&axis));
    CHECK(motion_profile_settled(&axis));
    CHECK(motion_profile_position(&axis) == 5000);

    /* From rest the first step covers at most a_max * dt^2: 30 centidegrees. */
    CHECK(motion_profile_step(&axis, 6000, TICK_MS) <= 5030);
    CHECK(motion_profile_position(&axis) > 5000);
}

static void test_it_brakes_onto_the_target_without_overshoot(void)
{
    const int32_t distances[] = {1, 7, 30, 599, 600, 601, 2500, 7777};
    const int32_t dts[] = {20, 37, 50};

    for (size_t d = 0; d < sizeof(distances) / sizeof(distances[0]); d++) {
        for (size_t t = 0; t < sizeof(dts) / sizeof(dts[0]); t++) {
            for (int sign = -1; sign <= 1; sign += 2) {
                motion_profile_t axis;
                trace_t trace;
                int32_t target = 9000 + sign * distances[d];

                motion_profile_init(&axis, &LIMITED);
                motion_profile_reset(&axis, 9000);
                trace_start(&trace, &axis);

                CHECK(run_to(&axis, target, dts[t], &trace) > 0);
                CHECK(within_limits(&trace, &LIMITED, dts[t]));
                CHECK(motion_profile_position(&axis) == target);
                if (sign > 0) {
                    CHECK(trace.max_pos <= (float)target && trace.min_pos >= 9000.0f);
                } else {
                    CHECK(trace.min_pos >= (float)target && trace.max_pos <= 9000.0f);
                }
            }
        }
    }
}

static void test_slow_limits_still_arrive(void)
{
    /* A fraction of a centidegree per step: whole centidegrees would round
     * every step to nothing and never get there. */
    const motion_profile_config_t slow = {10, 20, 0};
    motion_profile_t axis;
    trace_t trace;

    motion_profile_init(&axis, &slow);
    motion_profile_reset(&axis, 0);
    trace_start(&trace, &axis);

    CHECK(run_to(&axis, 100, TICK_MS, &trace) > 0);
    CHECK(motion_profile_position(&axis) == 100);
    CHECK(within_limits(&trace, &slow, TICK_MS));
}

static void test_a_slow_target_is_followed_without_lag(void)
{
    motion_profile_t axis;
    trace_t trace;

    motion_profile_init(&axis, &LIMITED);
    motion_profile_reset(&axis, 1000);
    trace_start(&trace, &axis);

    /* The elevation of a low-orbit pass moves about 1 degree per second. */
    for (int i = 1; i <= 200; i++) {
        int32_t target = 1000 + i * 100 * TICK_MS / 1000;

        motion_profile_step(&axis, target, TICK_MS);
        trace_record(&trace, &axis, TICK_MS);
        CHECK(motion_profile_position(&axis) == target);
    }
    CHECK(within_limits(&trace, &LIMITED, TICK_MS));
}

static void test_a_target_that_turns_around_keeps_the_acceleration_limit(void)
{
    motion_profile_t axis;
    trace_t trace;
    int32_t target = 5000;

    motion_profile_init(&axis, &LIMITED);
    motion_profile_reset(&axis, target);
    trace_start(&trace, &axis);

    /* Slow enough that the axis lands on the target every step, which must
     * not make it forget the speed it is moving at. */
    for (int i = 0; i < 20; i++) {
        target += 25;
        motion_profile_step(&axis, target, TICK_MS);
        trace_record(&trace, &axis, TICK_MS);
    }
    for (int i = 0; i < 40; i++) {
        target -= 25;
        motion_profile_step(&axis, target, TICK_MS);
        trace_record(&trace, &axis, TICK_MS);
    }
    CHECK(within_limits(&trace, &LIMITED, TICK_MS));
    CHECK(abs(motion_profile_position(&axis) - target) <= 25);
}

static void test_a_zero_or_negative_dt_moves_nothing(void)
{
    motion_profile_t axis;
    trace_t trace;
    int32_t before;

    motion_profile_init(&axis, &LIMITED);
    motion_profile_reset(&axis, 0);
    CHECK(motion_profile_step(&axis, 5000, 0) == 0);
    CHECK(motion_profile_step(&axis, 5000, -50) == 0);

    /* Nor while it is moving. */
    trace_start(&trace, &axis);
    move_for(&axis, 5000, 5, &trace);
    before = motion_profile_position(&axis);
    CHECK(before > 0);
    CHECK(motion_profile_step(&axis, 5000, 0) == before);
    CHECK(motion_profile_step(&axis, 5000, -1000) == before);
    CHECK(motion_profile_brake(&axis, 0) == before);
}

static void test_a_huge_dt_counts_as_the_longest_step(void)
{
    motion_profile_t huge;
    motion_profile_t longest;

    motion_profile_init(&huge, &LIMITED);
    motion_profile_init(&longest, &LIMITED);
    motion_profile_reset(&huge, 0);
    motion_profile_reset(&longest, 0);

    CHECK(motion_profile_step(&huge, 18000, INT32_MAX) == motion_profile_step(&longest, 18000, MOTION_PROFILE_MAX_DT_MS));
    for (int i = 0; i < 10; i++) {
        motion_profile_step(&huge, 18000, 3600000);
        motion_profile_step(&longest, 18000, MOTION_PROFILE_MAX_DT_MS);
    }
    CHECK(huge.pos_cdeg == longest.pos_cdeg);
    CHECK(motion_profile_position(&huge) < 18000);
}

static void test_no_limits_follow_the_target_as_is(void)
{
    motion_profile_t axis;

    motion_profile_init(&axis, &MOTION_PROFILE_NO_LIMITS);
    CHECK(motion_profile_step(&axis, 1000, TICK_MS) == 1000);
    CHECK(motion_profile_step(&axis, 17000, TICK_MS) == 17000);
    /* Even without any time passing: there is no profile to wait for. */
    CHECK(motion_profile_step(&axis, 250, 0) == 250);
    CHECK(motion_profile_step(&axis, 9000, -3) == 9000);
    CHECK(motion_profile_settled(&axis));
    CHECK(motion_profile_brake(&axis, 0) == 9000);
    CHECK(!motion_profile_resting(&axis)); /* rest_after_ms 0: never */
}

static void test_a_flip_is_a_ramp_within_the_range(void)
{
    /* Crossing north at 30 degrees of elevation (az_ref 0): 359.90 to 0.10
     * swings the pan from 179.90 to 0.10 and the tilt from 150 to 30. */
    motion_profile_t pan;
    motion_profile_t tilt;
    trace_t pan_trace;
    trace_t tilt_trace;
    int steps = 0;

    motion_profile_init(&pan, &LIMITED);
    motion_profile_init(&tilt, &LIMITED);
    motion_profile_step(&pan, 17990, TICK_MS);
    motion_profile_step(&tilt, 15000, TICK_MS);
    trace_start(&pan_trace, &pan);
    trace_start(&tilt_trace, &tilt);

    do {
        motion_profile_step(&pan, 10, TICK_MS);
        motion_profile_step(&tilt, 3000, TICK_MS);
        trace_record(&pan_trace, &pan, TICK_MS);
        trace_record(&tilt_trace, &tilt, TICK_MS);
        steps++;
    } while (!(motion_profile_settled(&pan) && motion_profile_settled(&tilt)) && steps < 1000);

    CHECK(motion_profile_position(&pan) == 10 && motion_profile_position(&tilt) == 3000);
    CHECK(pan_trace.min_pos >= 10.0f && pan_trace.max_pos <= 17990.0f);
    CHECK(tilt_trace.min_pos >= 3000.0f && tilt_trace.max_pos <= 15000.0f);
    CHECK(within_limits(&pan_trace, &LIMITED, TICK_MS));
    CHECK(within_limits(&tilt_trace, &LIMITED, TICK_MS));
    /* No slower than it needs to be: the pan reaches the top speed and the flip
     * takes about d / v_max + v_max / a_max = 3.5 s, the time off target that
     * coordinates-dto.md §9 warns about. */
    CHECK(pan_trace.max_speed >= V_MAX - SPEED_SLACK);
    CHECK(steps > 1 && steps <= 3600 / TICK_MS);
}

static void test_a_target_moved_back_is_overshot_short_of_the_old_one(void)
{
    motion_profile_t axis;
    trace_t trace;
    int32_t closer;

    motion_profile_init(&axis, &LIMITED);
    motion_profile_reset(&axis, 0);
    trace_start(&trace, &axis);
    move_for(&axis, 18000, 20, &trace); /* at full speed by now */

    /* Too close to stop in time: it goes past, comes back and settles. */
    closer = motion_profile_position(&axis) + 100;
    CHECK(run_to(&axis, closer, TICK_MS, &trace) > 0);
    CHECK(motion_profile_position(&axis) == closer);
    CHECK(trace.max_pos > (float)closer);
    CHECK(trace.max_pos <= 18000.0f);
    CHECK(within_limits(&trace, &LIMITED, TICK_MS));
}

static void test_a_brake_slows_down_at_the_acceleration_limit(void)
{
    motion_profile_t axis;
    trace_t trace;
    float braking_from;
    int steps = 0;

    motion_profile_init(&axis, &LIMITED);
    motion_profile_reset(&axis, 0);
    trace_start(&trace, &axis);
    move_for(&axis, 18000, 30, &trace);
    CHECK(trace.last_vel >= V_MAX - SPEED_SLACK);

    braking_from = axis.pos_cdeg;
    while (!motion_profile_settled(&axis) && steps < 100) {
        float before = axis.pos_cdeg;

        motion_profile_brake(&axis, TICK_MS);
        trace_record(&trace, &axis, TICK_MS);
        CHECK(axis.pos_cdeg >= before); /* keeps going the same way while it slows */
        steps++;
    }

    /* v / a = 0.5 s, and about v^2 / 2a = 15 degrees further on. */
    CHECK(steps <= 11);
    CHECK(fabsf(axis.pos_cdeg - braking_from - 1500.0f) <= 300.0f);
    CHECK(within_limits(&trace, &LIMITED, TICK_MS));

    /* Once stopped it stays put. */
    braking_from = axis.pos_cdeg;
    motion_profile_brake(&axis, TICK_MS);
    CHECK(axis.pos_cdeg == braking_from);
}

static void test_it_rests_after_the_timeout_at_the_same_target(void)
{
    motion_profile_t axis;

    motion_profile_init(&axis, &LIMITED);
    motion_profile_step(&axis, 5000, TICK_MS);
    CHECK(!motion_profile_resting(&axis));

    for (int i = 0; i < REST_MS / TICK_MS - 1; i++) {
        motion_profile_step(&axis, 5000, TICK_MS);
    }
    CHECK(!motion_profile_resting(&axis));
    motion_profile_step(&axis, 5000, TICK_MS);
    CHECK(motion_profile_resting(&axis));

    /* And it stays at rest for as long as nothing changes. */
    for (int i = 0; i < 1000; i++) {
        motion_profile_step(&axis, 5000, TICK_MS);
    }
    CHECK(motion_profile_resting(&axis));
    CHECK(motion_profile_position(&axis) == 5000);
}

static void test_a_new_target_ends_the_rest_at_once(void)
{
    motion_profile_t axis;

    motion_profile_init(&axis, &LIMITED);
    for (int i = 0; i <= REST_MS / TICK_MS; i++) {
        motion_profile_step(&axis, 5000, TICK_MS);
    }
    CHECK(motion_profile_resting(&axis));

    /* Before any time passes, so the outputs are back before the first move. */
    CHECK(motion_profile_step(&axis, 5100, 0) == 5000);
    CHECK(!motion_profile_resting(&axis));
}

static void test_a_moving_axis_does_not_rest(void)
{
    const motion_profile_config_t quick_rest = {V_MAX, A_MAX, 1000};
    motion_profile_t axis;
    int steps = 0;

    motion_profile_init(&axis, &quick_rest);
    motion_profile_reset(&axis, 0);

    /* The same target all along, but 3.5 s on the way to it. */
    do {
        motion_profile_step(&axis, 18000, TICK_MS);
        CHECK(!motion_profile_resting(&axis));
        steps++;
    } while (!motion_profile_settled(&axis) && steps < 1000);
    CHECK(steps > 1000 / TICK_MS);

    for (int i = 0; i < 1000 / TICK_MS; i++) {
        motion_profile_step(&axis, 18000, TICK_MS);
    }
    CHECK(motion_profile_resting(&axis));
}

static void test_an_axis_braked_to_a_stop_rests(void)
{
    motion_profile_t axis;
    trace_t trace;

    motion_profile_init(&axis, &LIMITED);
    motion_profile_reset(&axis, 0);
    trace_start(&trace, &axis);
    move_for(&axis, 18000, 30, &trace);

    for (int i = 0; i < 20; i++) {
        motion_profile_brake(&axis, TICK_MS);
    }
    CHECK(motion_profile_settled(&axis));
    CHECK(!motion_profile_resting(&axis));

    for (int i = 0; i < REST_MS / TICK_MS; i++) {
        motion_profile_brake(&axis, TICK_MS);
    }
    CHECK(motion_profile_resting(&axis));

    /* Any target after a stop wakes it, even the one it had before. */
    motion_profile_step(&axis, 18000, 0);
    CHECK(!motion_profile_resting(&axis));
}

int main(void)
{
    test_the_first_target_is_taken_as_is();
    test_a_reset_starts_from_a_known_position_at_rest();
    test_it_brakes_onto_the_target_without_overshoot();
    test_slow_limits_still_arrive();
    test_a_slow_target_is_followed_without_lag();
    test_a_target_that_turns_around_keeps_the_acceleration_limit();
    test_a_zero_or_negative_dt_moves_nothing();
    test_a_huge_dt_counts_as_the_longest_step();
    test_no_limits_follow_the_target_as_is();
    test_a_flip_is_a_ramp_within_the_range();
    test_a_target_moved_back_is_overshot_short_of_the_old_one();
    test_a_brake_slows_down_at_the_acceleration_limit();
    test_it_rests_after_the_timeout_at_the_same_target();
    test_a_new_target_ends_the_rest_at_once();
    test_a_moving_axis_does_not_rest();
    test_an_axis_braked_to_a_stop_rests();

    if (s_failures) {
        fprintf(stderr, "motion_profile: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("motion_profile: all tests passed\n");
    return 0;
}
