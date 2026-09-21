#include <stdio.h>

#include "trajectory.h"

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

/* Asks for the target at `now` and checks it. */
static void expect_target(const trajectory_t *traj, int64_t now, int32_t az, int32_t el, int line)
{
    int32_t got_az = -1;
    int32_t got_el = -1;

    if (!trajectory_target(traj, now, &got_az, &got_el) || got_az != az || got_el != el) {
        fprintf(stderr, "%s:%d: at t0%+lld ms expected (%d, %d), got (%d, %d)\n",
                __FILE__, line, (long long)(now - T0), (int)az, (int)el, (int)got_az, (int)got_el);
        s_failures++;
    }
}

#define EXPECT_TARGET(traj, now, az, el) expect_target(traj, now, az, el, __LINE__)

static const trajectory_point_t THREE_POINTS[] = {
    {T0, 10000, 2000},
    {T0 + 1000, 11000, 3000},
    {T0 + 2000, 12000, 2000},
};

static void test_nothing_to_follow_after_init(void)
{
    trajectory_t traj;
    int32_t az;
    int32_t el;

    trajectory_init(&traj);
    CHECK(!trajectory_target(&traj, T0, &az, &el));
}

static void test_follows_the_points_in_time(void)
{
    trajectory_t traj;

    trajectory_init(&traj);
    CHECK(trajectory_load(&traj, THREE_POINTS, 3, T0 - 2000, TOLERANCE_MS) == 3);

    EXPECT_TARGET(&traj, T0 - 2000, 10000, 2000); /* before the first: the first */
    EXPECT_TARGET(&traj, T0, 10000, 2000);
    EXPECT_TARGET(&traj, T0 + 500, 10500, 2500); /* halfway: interpolated */
    EXPECT_TARGET(&traj, T0 + 250, 10250, 2250);
    EXPECT_TARGET(&traj, T0 + 1000, 11000, 3000);
    EXPECT_TARGET(&traj, T0 + 1500, 11500, 2500);
    EXPECT_TARGET(&traj, T0 + 2000, 12000, 2000);
    EXPECT_TARGET(&traj, T0 + 60000, 12000, 2000); /* long after the last: stays there */
}

static void test_azimuth_crosses_north_the_short_way(void)
{
    trajectory_t traj;
    const trajectory_point_t east_to_west[] = {{T0, 35900, 1000}, {T0 + 2000, 100, 1000}};
    const trajectory_point_t west_to_east[] = {{T0, 100, 1000}, {T0 + 2000, 35900, 1000}};

    trajectory_init(&traj);
    trajectory_load(&traj, east_to_west, 2, T0, TOLERANCE_MS);
    EXPECT_TARGET(&traj, T0 + 500, 35950, 1000);
    EXPECT_TARGET(&traj, T0 + 1000, 0, 1000);
    EXPECT_TARGET(&traj, T0 + 1500, 50, 1000);

    trajectory_load(&traj, west_to_east, 2, T0, TOLERANCE_MS);
    EXPECT_TARGET(&traj, T0 + 1000, 0, 1000);
    EXPECT_TARGET(&traj, T0 + 1500, 35950, 1000);
}

static void test_a_new_batch_replaces_the_pending_points(void)
{
    trajectory_t traj;
    const trajectory_point_t newer[] = {{T0 + 500, 20000, 4000}, {T0 + 1500, 21000, 4000}};

    trajectory_init(&traj);
    trajectory_load(&traj, THREE_POINTS, 3, T0, TOLERANCE_MS);
    CHECK(trajectory_load(&traj, newer, 2, T0, TOLERANCE_MS) == 2);

    EXPECT_TARGET(&traj, T0 + 1000, 20500, 4000);
    EXPECT_TARGET(&traj, T0 + 1900, 21000, 4000); /* the old third point is gone */
}

static void test_expired_points_are_dropped(void)
{
    trajectory_t traj;

    trajectory_init(&traj);
    /* At t0+1200 with 500 ms of tolerance, only the first point is older than that. */
    CHECK(trajectory_load(&traj, THREE_POINTS, 3, T0 + 1200, TOLERANCE_MS) == 2);
    EXPECT_TARGET(&traj, T0 + 1200, 11200, 2800); /* 20% of the way from the second point to the third */

    /* Within the tolerance a late point is still kept. */
    CHECK(trajectory_load(&traj, THREE_POINTS, 3, T0 + TOLERANCE_MS, TOLERANCE_MS) == 3);
}

static void test_a_fully_expired_batch_keeps_the_current_trajectory(void)
{
    trajectory_t traj;
    const trajectory_point_t stale[] = {{T0 - 10000, 0, 0}, {T0 - 9000, 100, 100}};

    trajectory_init(&traj);
    trajectory_load(&traj, THREE_POINTS, 3, T0, TOLERANCE_MS);
    CHECK(trajectory_load(&traj, stale, 2, T0, TOLERANCE_MS) == 0);

    EXPECT_TARGET(&traj, T0 + 1000, 11000, 3000);
}

static void test_invalid_batches_are_discarded(void)
{
    trajectory_t traj;
    trajectory_point_t too_many[TRAJECTORY_MAX_POINTS + 1];
    const trajectory_point_t out_of_order[] = {{T0 + 1000, 0, 0}, {T0 + 1000, 100, 100}};

    for (size_t i = 0; i < TRAJECTORY_MAX_POINTS + 1; i++) {
        too_many[i] = (trajectory_point_t){T0 + (int64_t)i * 1000, 0, 0};
    }

    trajectory_init(&traj);
    trajectory_load(&traj, THREE_POINTS, 3, T0, TOLERANCE_MS);
    CHECK(trajectory_load(&traj, too_many, TRAJECTORY_MAX_POINTS + 1, T0, TOLERANCE_MS) == 0);
    CHECK(trajectory_load(&traj, out_of_order, 2, T0, TOLERANCE_MS) == 0);
    CHECK(trajectory_load(&traj, THREE_POINTS, 0, T0, TOLERANCE_MS) == 0);

    EXPECT_TARGET(&traj, T0 + 1000, 11000, 3000);
}

static void test_hold_drops_everything(void)
{
    trajectory_t traj;
    int32_t az;
    int32_t el;

    trajectory_init(&traj);
    trajectory_load(&traj, THREE_POINTS, 3, T0, TOLERANCE_MS);
    trajectory_clear(&traj);

    CHECK(!trajectory_target(&traj, T0 + 1000, &az, &el));
}

static void test_negative_elevation_interpolates(void)
{
    trajectory_t traj;
    const trajectory_point_t dipping[] = {{T0, 5000, 300}, {T0 + 1000, 5000, -300}};

    trajectory_init(&traj);
    trajectory_load(&traj, dipping, 2, T0, TOLERANCE_MS);
    EXPECT_TARGET(&traj, T0 + 750, 5000, -150);
}

int main(void)
{
    test_nothing_to_follow_after_init();
    test_follows_the_points_in_time();
    test_azimuth_crosses_north_the_short_way();
    test_a_new_batch_replaces_the_pending_points();
    test_expired_points_are_dropped();
    test_a_fully_expired_batch_keeps_the_current_trajectory();
    test_invalid_batches_are_discarded();
    test_hold_drops_everything();
    test_negative_elevation_interpolates();

    if (s_failures) {
        fprintf(stderr, "trajectory: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("trajectory: all tests passed\n");
    return 0;
}
