#include <stdio.h>

#include "encoder_angle.h"

static int s_failures;

static void expect_angle(encoder_calibration_t cal, uint16_t raw, int32_t cdeg, int line)
{
    int32_t got = encoder_angle_cdeg(&cal, raw);

    if (got != cdeg) {
        fprintf(stderr, "%s:%d: zero %d dir %d at raw %u: expected %d cdeg, got %d cdeg\n",
                __FILE__, line, (int)cal.zero_raw, (int)cal.dir, (unsigned)raw, (int)cdeg, (int)got);
        s_failures++;
    }
}

#define EXPECT_ANGLE(cal, raw, cdeg) expect_angle(cal, raw, cdeg, __LINE__)

static const encoder_calibration_t PLAIN = {.zero_raw = 0, .dir = 1};

static void test_counts_scale_to_centidegrees(void)
{
    EXPECT_ANGLE(PLAIN, 0, 0);
    EXPECT_ANGLE(PLAIN, 1024, 9000);
    EXPECT_ANGLE(PLAIN, 2048, 18000);
    /* A count is 8.79 cdeg, rounded to the nearest. */
    EXPECT_ANGLE(PLAIN, 1, 9);
    EXPECT_ANGLE(PLAIN, 3, 26);
}

static void test_the_zero_is_an_offset_that_wraps(void)
{
    const encoder_calibration_t zero = {.zero_raw = 4000, .dir = 1};

    EXPECT_ANGLE(zero, 4000, 0);
    EXPECT_ANGLE(zero, 4095, 835);
    EXPECT_ANGLE(zero, 100, 1723); /* past the chip's own 0 */
}

static void test_a_reversed_encoder_counts_down(void)
{
    const encoder_calibration_t reversed = {.zero_raw = 3000, .dir = -1};

    EXPECT_ANGLE(reversed, 3000, 0);
    EXPECT_ANGLE(reversed, 1976, 9000);
    EXPECT_ANGLE(reversed, 952, 18000);
}

/* The servo uses half a turn of the encoder's whole one: a little past either
 * end of the travel reads as just outside 0..18000, never as the far side. */
static void test_the_turn_is_split_around_the_travel(void)
{
    const encoder_calibration_t zero = {.zero_raw = 100, .dir = 1};

    EXPECT_ANGLE(zero, 90, -88);
    EXPECT_ANGLE(zero, 2148 + 10, 18088);
    EXPECT_ANGLE(PLAIN, 3071, 26991);
    EXPECT_ANGLE(PLAIN, 3072, -9000);
    EXPECT_ANGLE(PLAIN, 4095, -9);
}

int main(void)
{
    test_counts_scale_to_centidegrees();
    test_the_zero_is_an_offset_that_wraps();
    test_a_reversed_encoder_counts_down();
    test_the_turn_is_split_around_the_travel();

    if (s_failures) {
        fprintf(stderr, "encoder_angle: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("encoder_angle: all tests passed\n");
    return 0;
}
