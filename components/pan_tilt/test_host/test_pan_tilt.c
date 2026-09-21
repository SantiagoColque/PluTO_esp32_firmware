#include <stdio.h>

#include "pan_tilt.h"

static int s_failures;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            s_failures++;                                                        \
        }                                                                        \
    } while (0)

static void expect_pose(const pan_tilt_config_t *cfg, pan_tilt_mode_t current, int32_t az, int32_t el,
                        pan_tilt_mode_t mode, int32_t pan, int32_t tilt, int line)
{
    pan_tilt_pose_t pose = {0};

    if (!pan_tilt_from_azel(cfg, current, az, el, &pose) ||
        pose.mode != mode || pose.pan_cdeg != pan || pose.tilt_cdeg != tilt) {
        fprintf(stderr, "%s:%d: az=%d el=%d expected mode %d (%d, %d), got mode %d (%d, %d)\n",
                __FILE__, line, (int)az, (int)el, (int)mode, (int)pan, (int)tilt,
                (int)pose.mode, (int)pose.pan_cdeg, (int)pose.tilt_cdeg);
        s_failures++;
    }
}

#define EXPECT_POSE(cfg, current, az, el, mode, pan, tilt) \
    expect_pose(cfg, current, az, el, mode, pan, tilt, __LINE__)

static void test_front_half_uses_the_normal_mode(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    EXPECT_POSE(&cfg, PAN_TILT_NORMAL, 4500, 3000, PAN_TILT_NORMAL, 4500, 3000);
    EXPECT_POSE(&cfg, PAN_TILT_FLIPPED, 4500, 3000, PAN_TILT_NORMAL, 4500, 3000);
}

static void test_back_half_points_past_the_zenith(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    /* Due west at 30 degrees: pan 90 and tilt 150, i.e. 30 degrees above the horizon behind. */
    EXPECT_POSE(&cfg, PAN_TILT_NORMAL, 27000, 3000, PAN_TILT_FLIPPED, 9000, 15000);
}

static void test_mount_heading_shifts_the_pan(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    cfg.az_ref_cdeg = 9000; /* the pan faces east at 0 */
    EXPECT_POSE(&cfg, PAN_TILT_NORMAL, 13500, 2000, PAN_TILT_NORMAL, 4500, 2000);
    EXPECT_POSE(&cfg, PAN_TILT_NORMAL, 4500, 2000, PAN_TILT_FLIPPED, 13500, 16000);
}

static void test_keeps_the_current_mode_on_the_border(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    /* Right on the border both modes reach it, so there is no reason to flip. */
    EXPECT_POSE(&cfg, PAN_TILT_NORMAL, 0, 3000, PAN_TILT_NORMAL, 0, 3000);
    EXPECT_POSE(&cfg, PAN_TILT_FLIPPED, 0, 3000, PAN_TILT_FLIPPED, 18000, 15000);
    EXPECT_POSE(&cfg, PAN_TILT_NORMAL, 18000, 3000, PAN_TILT_NORMAL, 18000, 3000);
    EXPECT_POSE(&cfg, PAN_TILT_FLIPPED, 18000, 3000, PAN_TILT_FLIPPED, 0, 15000);
}

static void test_zenith_is_reached_in_both_modes(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    EXPECT_POSE(&cfg, PAN_TILT_NORMAL, 9000, 9000, PAN_TILT_NORMAL, 9000, 9000);
    EXPECT_POSE(&cfg, PAN_TILT_FLIPPED, 27000, 9000, PAN_TILT_FLIPPED, 9000, 9000);
}

static void test_below_the_horizon_is_unreachable(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;
    pan_tilt_pose_t pose = {.pan_cdeg = 1234, .tilt_cdeg = 5678, .mode = PAN_TILT_NORMAL};

    CHECK(!pan_tilt_from_azel(&cfg, PAN_TILT_NORMAL, 4500, -100, &pose));
    CHECK(pose.pan_cdeg == 1234 && pose.tilt_cdeg == 5678); /* untouched */
    CHECK(!pan_tilt_reachable(&cfg, 27000, -1));
    CHECK(pan_tilt_reachable(&cfg, 27000, 0));
}

static void test_narrowed_servo_ranges_are_enforced(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    cfg.pan_min_cdeg = 1000;
    cfg.pan_max_cdeg = 17000;
    /* Within 10 degrees of the mount's axis neither mode's pan fits. */
    CHECK(!pan_tilt_reachable(&cfg, 500, 3000));
    CHECK(!pan_tilt_reachable(&cfg, 18500, 3000));
    CHECK(pan_tilt_reachable(&cfg, 9000, 3000));

    cfg = PAN_TILT_CONFIG_DEFAULT;
    cfg.tilt_max_cdeg = 12000; /* cannot lean back further than 30 degrees past the zenith */
    CHECK(pan_tilt_reachable(&cfg, 27000, 7000));  /* flipped tilt 110 */
    CHECK(!pan_tilt_reachable(&cfg, 27000, 5000)); /* flipped tilt 130 */
}

static void test_the_default_config_covers_the_whole_sky(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;
    int unreachable = 0;

    for (int32_t az = 0; az < 36000; az += 100) {
        for (int32_t el = 0; el <= 9000; el += 250) {
            if (!pan_tilt_reachable(&cfg, az, el)) {
                unreachable++;
            }
        }
    }
    CHECK(unreachable == 0);
}

static void test_the_inverse_recovers_the_target(void)
{
    const int32_t refs[] = {0, 9000, 21550};
    const int32_t elevations[] = {0, 1, 4500, 8999, 9000};

    for (size_t r = 0; r < sizeof(refs) / sizeof(refs[0]); r++) {
        pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;
        cfg.az_ref_cdeg = refs[r];

        for (int32_t az = 0; az < 36000; az += 250) {
            for (size_t e = 0; e < sizeof(elevations) / sizeof(elevations[0]); e++) {
                for (int m = 0; m < 2; m++) {
                    pan_tilt_pose_t pose;
                    int32_t back_az;
                    int32_t back_el;

                    CHECK(pan_tilt_from_azel(&cfg, (pan_tilt_mode_t)m, az, elevations[e], &pose));
                    pan_tilt_to_azel(&cfg, &pose, &back_az, &back_el);
                    CHECK(back_az == az && back_el == elevations[e]);
                }
            }
        }
    }
}

int main(void)
{
    test_front_half_uses_the_normal_mode();
    test_back_half_points_past_the_zenith();
    test_mount_heading_shifts_the_pan();
    test_keeps_the_current_mode_on_the_border();
    test_zenith_is_reached_in_both_modes();
    test_below_the_horizon_is_unreachable();
    test_narrowed_servo_ranges_are_enforced();
    test_the_default_config_covers_the_whole_sky();
    test_the_inverse_recovers_the_target();

    if (s_failures) {
        fprintf(stderr, "pan_tilt: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("pan_tilt: all tests passed\n");
    return 0;
}
