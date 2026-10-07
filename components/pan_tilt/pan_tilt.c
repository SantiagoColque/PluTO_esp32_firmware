#include "pan_tilt.h"

#include <stddef.h>

#define FULL_TURN_CDEG 36000
#define HALF_TURN_CDEG 18000

static int32_t wrap_azimuth(int32_t cdeg)
{
    int32_t wrapped = cdeg % FULL_TURN_CDEG;

    if (wrapped < 0) {
        wrapped += FULL_TURN_CDEG;
    }
    return wrapped;
}

static pan_tilt_pose_t pose_in_mode(const pan_tilt_config_t *cfg,
                                    pan_tilt_mode_t mode,
                                    int32_t az_cdeg,
                                    int32_t el_cdeg)
{
    int32_t relative = wrap_azimuth(az_cdeg - cfg->az_ref_cdeg);
    pan_tilt_pose_t pose = {.mode = mode};

    if (mode == PAN_TILT_NORMAL) {
        pose.pan_cdeg = relative;
        pose.tilt_cdeg = el_cdeg;
    } else {
        pose.pan_cdeg = wrap_azimuth(relative - HALF_TURN_CDEG);
        pose.tilt_cdeg = HALF_TURN_CDEG - el_cdeg;
    }
    return pose;
}

static bool within_limits(const pan_tilt_config_t *cfg, const pan_tilt_pose_t *pose)
{
    return pose->pan_cdeg >= cfg->pan_min_cdeg && pose->pan_cdeg <= cfg->pan_max_cdeg &&
           pose->tilt_cdeg >= cfg->tilt_min_cdeg && pose->tilt_cdeg <= cfg->tilt_max_cdeg;
}

bool pan_tilt_from_azel(const pan_tilt_config_t *cfg,
                        pan_tilt_mode_t current,
                        int32_t az_cdeg,
                        int32_t el_cdeg,
                        pan_tilt_pose_t *out)
{
    pan_tilt_mode_t other = current == PAN_TILT_NORMAL ? PAN_TILT_FLIPPED : PAN_TILT_NORMAL;
    pan_tilt_pose_t pose = pose_in_mode(cfg, current, az_cdeg, el_cdeg);

    if (!within_limits(cfg, &pose)) {
        pose = pose_in_mode(cfg, other, az_cdeg, el_cdeg);
        if (!within_limits(cfg, &pose)) {
            return false;
        }
    }

    *out = pose;
    return true;
}

bool pan_tilt_reachable(const pan_tilt_config_t *cfg, int32_t az_cdeg, int32_t el_cdeg)
{
    pan_tilt_pose_t pose;

    return pan_tilt_from_azel(cfg, PAN_TILT_NORMAL, az_cdeg, el_cdeg, &pose);
}

void pan_tilt_to_azel(const pan_tilt_config_t *cfg,
                      const pan_tilt_pose_t *pose,
                      int32_t *az_cdeg,
                      int32_t *el_cdeg)
{
    if (pose->mode == PAN_TILT_NORMAL) {
        *az_cdeg = wrap_azimuth(cfg->az_ref_cdeg + pose->pan_cdeg);
        *el_cdeg = pose->tilt_cdeg;
    } else {
        *az_cdeg = wrap_azimuth(cfg->az_ref_cdeg + pose->pan_cdeg + HALF_TURN_CDEG);
        *el_cdeg = HALF_TURN_CDEG - pose->tilt_cdeg;
    }
}

/* Comparisons only, no arithmetic, so a corrupted stored value near the int32
 * limits cannot overflow on its way to being rejected. */
static bool range_valid(int32_t min_cdeg, int32_t max_cdeg)
{
    return min_cdeg >= 0 && min_cdeg < max_cdeg && max_cdeg <= HALF_TURN_CDEG;
}

bool pan_tilt_config_valid(const pan_tilt_config_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    return cfg->az_ref_cdeg >= 0 && cfg->az_ref_cdeg < FULL_TURN_CDEG &&
           range_valid(cfg->pan_min_cdeg, cfg->pan_max_cdeg) &&
           range_valid(cfg->tilt_min_cdeg, cfg->tilt_max_cdeg);
}
