#include "servo_pulse.h"

#define US_PER_SECOND 1000000

static int64_t clamp64(int64_t value, int64_t min, int64_t max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

bool servo_calibration_valid(const servo_calibration_t *cal)
{
    return cal->us_at_0 >= SERVO_PULSE_MIN_US && cal->us_at_0 <= SERVO_PULSE_MAX_US &&
           cal->us_at_180 >= SERVO_PULSE_MIN_US && cal->us_at_180 <= SERVO_PULSE_MAX_US &&
           cal->us_at_0 != cal->us_at_180;
}

int32_t servo_pulse_us(const servo_calibration_t *cal, int32_t cdeg)
{
    /* 64 bits throughout: the span of two arbitrary int32 points times 18000
     * does not fit in 32, and the struct may come from NVS one day. */
    int64_t span = (int64_t)cal->us_at_180 - cal->us_at_0;
    int64_t scaled = span * clamp64(cdeg, 0, SERVO_SPAN_CDEG);
    int64_t offset = scaled / SERVO_SPAN_CDEG;
    int64_t remainder = scaled % SERVO_SPAN_CDEG;

    /* C division truncates toward zero, so rounding the remainder by its
     * magnitude rounds halves away from zero on both sides of the line. */
    if (2 * (remainder < 0 ? -remainder : remainder) >= SERVO_SPAN_CDEG) {
        offset += scaled < 0 ? -1 : 1;
    }

    return (int32_t)clamp64(cal->us_at_0 + offset, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
}

uint32_t servo_pulse_duty(int32_t pulse_us)
{
    uint64_t us = (uint64_t)clamp64(pulse_us, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    uint64_t counts_per_frame = (uint64_t)1 << SERVO_DUTY_RESOLUTION_BITS;

    /* duty = pulse / period * 2^bits, with the period as 1 s / frequency. */
    return (uint32_t)((us * SERVO_PWM_FREQ_HZ * counts_per_frame + US_PER_SECOND / 2) / US_PER_SECOND);
}
