#include "encoder_angle.h"

#define FULL_TURN_CDEG 36000
/* Half a turn past the middle of the 0..18000 travel. */
#define SPLIT_CDEG 27000

int32_t encoder_angle_cdeg(const encoder_calibration_t *cal, uint16_t raw)
{
    /* zero_raw comes from NVS: reduced first so no stored value can overflow. */
    int32_t counts = ((int32_t)raw - cal->zero_raw % ENCODER_COUNTS) % ENCODER_COUNTS;
    int32_t cdeg;

    if (cal->dir < 0) {
        counts = -counts;
    }
    if (counts < 0) {
        counts += ENCODER_COUNTS;
    }
    /* Rounded to the nearest centidegree; a count is 8.79 of them. */
    cdeg = (counts * FULL_TURN_CDEG + ENCODER_COUNTS / 2) / ENCODER_COUNTS;
    return cdeg >= SPLIT_CDEG ? cdeg - FULL_TURN_CDEG : cdeg;
}
