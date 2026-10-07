#include <stdio.h>

#include "servo_pulse.h"

static int s_failures;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            s_failures++;                                                        \
        }                                                                        \
    } while (0)

static void expect_pulse(servo_calibration_t cal, int32_t cdeg, int32_t us, int line)
{
    int32_t got = servo_pulse_us(&cal, cdeg);

    if (got != us) {
        fprintf(stderr, "%s:%d: calibration %d..%d at %d cdeg: expected %d us, got %d us\n",
                __FILE__, line, (int)cal.us_at_0, (int)cal.us_at_180, (int)cdeg, (int)us, (int)got);
        s_failures++;
    }
}

#define EXPECT_PULSE(cal, cdeg, us) expect_pulse(cal, cdeg, us, __LINE__)

static void test_the_calibrated_points_and_the_middle(void)
{
    servo_calibration_t cal = SERVO_CALIBRATION_DS3218_180;

    EXPECT_PULSE(cal, 0, 500);
    EXPECT_PULSE(cal, 9000, 1500);
    EXPECT_PULSE(cal, 18000, 2500);
    EXPECT_PULSE(cal, 4500, 1000);
}

static void test_the_270_degree_version_spends_less_pulse_on_180_degrees(void)
{
    servo_calibration_t cal = SERVO_CALIBRATION_DS3218_270;

    EXPECT_PULSE(cal, 0, 834);
    EXPECT_PULSE(cal, 9000, 1500);
    EXPECT_PULSE(cal, 18000, 2166);
}

static void test_a_trimmed_servo_follows_its_own_line(void)
{
    /* Measured 0 and 180 degrees at 560 and 2440 us: offset and scale both off. */
    servo_calibration_t cal = {.us_at_0 = 560, .us_at_180 = 2440};

    EXPECT_PULSE(cal, 0, 560);
    EXPECT_PULSE(cal, 9000, 1500);
    EXPECT_PULSE(cal, 18000, 2440);
}

static void test_rounds_to_the_nearest_microsecond(void)
{
    servo_calibration_t cal = SERVO_CALIBRATION_DS3218_180; /* 0.111 us per cdeg */
    servo_calibration_t narrow = {.us_at_0 = 1000, .us_at_180 = 2000};

    EXPECT_PULSE(cal, 4, 500); /* 500.44 */
    EXPECT_PULSE(cal, 5, 501); /* 500.56 */
    EXPECT_PULSE(cal, 17995, 2499); /* 2499.44 */
    EXPECT_PULSE(narrow, 9, 1001); /* exactly 1000.5: halves go away from the 0-degree point */
}

static void test_a_reversed_servo_mirrors_a_normal_one(void)
{
    servo_calibration_t cal = {.us_at_0 = 2500, .us_at_180 = 500};
    servo_calibration_t narrow = {.us_at_0 = 2000, .us_at_180 = 1000};

    EXPECT_PULSE(cal, 0, 2500);
    EXPECT_PULSE(cal, 4500, 2000);
    EXPECT_PULSE(cal, 18000, 500);
    EXPECT_PULSE(cal, 5, 2499);
    EXPECT_PULSE(narrow, 9, 1999); /* the mirror of 1001 above */
}

static void test_never_turns_past_the_calibrated_points(void)
{
    servo_calibration_t cal = SERVO_CALIBRATION_DS3218_270;
    servo_calibration_t reversed = {.us_at_0 = 2166, .us_at_180 = 834};

    EXPECT_PULSE(cal, -1, 834);
    EXPECT_PULSE(cal, -36000, 834);
    EXPECT_PULSE(cal, 18001, 2166);
    EXPECT_PULSE(cal, INT32_MAX, 2166);
    EXPECT_PULSE(cal, INT32_MIN, 834);
    EXPECT_PULSE(reversed, 20000, 834);
}

static void test_clamps_to_the_servo_input_even_if_the_calibration_does_not(void)
{
    servo_calibration_t wide = {.us_at_0 = 300, .us_at_180 = 2700};
    servo_calibration_t reversed = {.us_at_0 = 2700, .us_at_180 = 300};
    servo_calibration_t extreme = {.us_at_0 = INT32_MIN, .us_at_180 = INT32_MAX};

    EXPECT_PULSE(wide, 0, 500);
    EXPECT_PULSE(wide, 9000, 1500);
    EXPECT_PULSE(wide, 18000, 2500);
    EXPECT_PULSE(reversed, 0, 2500);
    EXPECT_PULSE(reversed, 18000, 500);
    /* No overflow on the way, whatever the struct holds. */
    EXPECT_PULSE(extreme, 0, 500);
    EXPECT_PULSE(extreme, 9000, 500); /* the middle of INT32_MIN..INT32_MAX is about 0 us */
    EXPECT_PULSE(extreme, 18000, 2500);
}

static void test_duty_at_16_bits_and_50_hz(void)
{
    CHECK(SERVO_PWM_FREQ_HZ == 50);
    CHECK(SERVO_DUTY_RESOLUTION_BITS == 16);

    CHECK(servo_pulse_duty(500) == 1638);  /* 1638.4 */
    CHECK(servo_pulse_duty(1500) == 4915); /* 4915.2 */
    CHECK(servo_pulse_duty(2500) == 8192); /* exact */
    CHECK(servo_pulse_duty(834) == 2733);  /* 2732.85 */
    CHECK(servo_pulse_duty(2166) == 7098); /* 7097.55 */
}

static void test_duty_clamps_the_pulse_too(void)
{
    CHECK(servo_pulse_duty(499) == 1638);
    CHECK(servo_pulse_duty(0) == 1638);
    CHECK(servo_pulse_duty(-1000) == 1638);
    CHECK(servo_pulse_duty(INT32_MIN) == 1638);
    CHECK(servo_pulse_duty(2501) == 8192);
    CHECK(servo_pulse_duty(INT32_MAX) == 8192);
}

static void test_which_calibrations_are_valid(void)
{
    servo_calibration_t nominal_180 = SERVO_CALIBRATION_DS3218_180;
    servo_calibration_t nominal_270 = SERVO_CALIBRATION_DS3218_270;
    servo_calibration_t reversed = {.us_at_0 = 2500, .us_at_180 = 500};
    servo_calibration_t unset = {0};
    servo_calibration_t flat = {.us_at_0 = 1500, .us_at_180 = 1500};
    servo_calibration_t below = {.us_at_0 = 499, .us_at_180 = 2500};
    servo_calibration_t above = {.us_at_0 = 500, .us_at_180 = 2501};

    CHECK(servo_calibration_valid(&nominal_180));
    CHECK(servo_calibration_valid(&nominal_270));
    CHECK(servo_calibration_valid(&reversed));
    CHECK(!servo_calibration_valid(&unset));
    CHECK(!servo_calibration_valid(&flat));
    CHECK(!servo_calibration_valid(&below));
    CHECK(!servo_calibration_valid(&above));
}

int main(void)
{
    test_the_calibrated_points_and_the_middle();
    test_the_270_degree_version_spends_less_pulse_on_180_degrees();
    test_a_trimmed_servo_follows_its_own_line();
    test_rounds_to_the_nearest_microsecond();
    test_a_reversed_servo_mirrors_a_normal_one();
    test_never_turns_past_the_calibrated_points();
    test_clamps_to_the_servo_input_even_if_the_calibration_does_not();
    test_duty_at_16_bits_and_50_hz();
    test_duty_clamps_the_pulse_too();
    test_which_calibrations_are_valid();

    if (s_failures) {
        fprintf(stderr, "servo: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("servo: all tests passed\n");
    return 0;
}
