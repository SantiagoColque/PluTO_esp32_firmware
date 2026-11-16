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

    /* Due west at 30 degrees: pan 90, tilt 150. */
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
    cfg.tilt_max_cdeg = 12000; /* 30 degrees past the zenith */
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

static void test_the_default_config_is_valid(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    CHECK(pan_tilt_config_valid(&cfg));
}

static void test_a_narrowed_and_turned_mount_is_valid(void)
{
    pan_tilt_config_t cfg = {.az_ref_cdeg = 21550,
                             .pan_min_cdeg = 1000,
                             .pan_max_cdeg = 17000,
                             .tilt_min_cdeg = 500,
                             .tilt_max_cdeg = 12000};

    CHECK(pan_tilt_config_valid(&cfg));
}

static void test_az_ref_must_be_a_heading(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    cfg.az_ref_cdeg = 35999;
    CHECK(pan_tilt_config_valid(&cfg));
    /* 360 is north again; only 0 is accepted, so a stored value has one meaning. */
    cfg.az_ref_cdeg = 36000;
    CHECK(!pan_tilt_config_valid(&cfg));
    cfg.az_ref_cdeg = -1;
    CHECK(!pan_tilt_config_valid(&cfg));
    cfg.az_ref_cdeg = INT32_MIN;
    CHECK(!pan_tilt_config_valid(&cfg));
}

static void test_limits_must_fit_a_180_degree_servo(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    cfg.pan_min_cdeg = -1;
    CHECK(!pan_tilt_config_valid(&cfg));

    cfg = PAN_TILT_CONFIG_DEFAULT;
    cfg.pan_max_cdeg = 18001;
    CHECK(!pan_tilt_config_valid(&cfg));

    cfg = PAN_TILT_CONFIG_DEFAULT;
    cfg.tilt_min_cdeg = -1;
    CHECK(!pan_tilt_config_valid(&cfg));

    cfg = PAN_TILT_CONFIG_DEFAULT;
    cfg.tilt_max_cdeg = 18001;
    CHECK(!pan_tilt_config_valid(&cfg));

    cfg = PAN_TILT_CONFIG_DEFAULT;
    cfg.pan_max_cdeg = INT32_MAX;
    cfg.tilt_min_cdeg = INT32_MIN;
    CHECK(!pan_tilt_config_valid(&cfg));
}

static void test_each_range_must_be_open(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;

    /* An empty range makes every batch unreachable, which looks like a server bug. */
    cfg.pan_min_cdeg = 9000;
    cfg.pan_max_cdeg = 9000;
    CHECK(!pan_tilt_config_valid(&cfg));
    cfg.pan_min_cdeg = 9001;
    CHECK(!pan_tilt_config_valid(&cfg));

    cfg = PAN_TILT_CONFIG_DEFAULT;
    cfg.tilt_min_cdeg = 9000;
    cfg.tilt_max_cdeg = 9000;
    CHECK(!pan_tilt_config_valid(&cfg));
    cfg.tilt_min_cdeg = 9001;
    CHECK(!pan_tilt_config_valid(&cfg));

    cfg = PAN_TILT_CONFIG_DEFAULT;
    cfg.pan_min_cdeg = 17999;
    cfg.tilt_max_cdeg = 1;
    CHECK(pan_tilt_config_valid(&cfg));
}

static void test_a_missing_config_is_invalid(void)
{
    CHECK(!pan_tilt_config_valid(NULL));
}

static void test_a_pose_between_modes_reports_where_it_points(void)
{
    pan_tilt_config_t cfg = PAN_TILT_CONFIG_DEFAULT;
    /* Mid-flip to due west: the tilt is past the zenith, the label still normal. */
    pan_tilt_pose_t pose = {.pan_cdeg = 9000, .tilt_cdeg = 12000, .mode = PAN_TILT_NORMAL};
    int32_t az;
    int32_t el;

    CHECK(pan_tilt_pose_mode(&pose) == PAN_TILT_FLIPPED);
    pan_tilt_to_azel(&cfg, &pose, &az, &el);
    CHECK(az == 27000 && el == 6000);

    pose = (pan_tilt_pose_t){.pan_cdeg = 9000, .tilt_cdeg = 6000, .mode = PAN_TILT_FLIPPED};
    CHECK(pan_tilt_pose_mode(&pose) == PAN_TILT_NORMAL);
    pan_tilt_to_azel(&cfg, &pose, &az, &el);
    CHECK(az == 9000 && el == 6000);
}

static void test_on_the_zenith_the_label_decides(void)
{
    pan_tilt_pose_t pose = {.pan_cdeg = 4500, .tilt_cdeg = 9000, .mode = PAN_TILT_NORMAL};

    CHECK(pan_tilt_pose_mode(&pose) == PAN_TILT_NORMAL);
    pose.mode = PAN_TILT_FLIPPED;
    CHECK(pan_tilt_pose_mode(&pose) == PAN_TILT_FLIPPED);
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
    test_the_default_config_is_valid();
    test_a_narrowed_and_turned_mount_is_valid();
    test_az_ref_must_be_a_heading();
    test_limits_must_fit_a_180_degree_servo();
    test_each_range_must_be_open();
    test_a_missing_config_is_invalid();
    test_a_pose_between_modes_reports_where_it_points();
    test_on_the_zenith_the_label_decides();

    if (s_failures) {
        fprintf(stderr, "pan_tilt: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("pan_tilt: all tests passed\n");
    return 0;
}
