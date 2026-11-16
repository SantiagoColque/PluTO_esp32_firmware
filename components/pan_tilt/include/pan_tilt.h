#ifndef PAN_TILT_H
#define PAN_TILT_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Geographic az/el to a two-servo 180-degree pan-tilt (coordinates-dto.md §9),
 * in centidegrees.
 *
 *   mode      pan                 tilt
 *   normal    az - az_ref         el
 *   flipped   az - az_ref - 180   180 - el
 *
 * Tilt runs 0 (horizon ahead), 90 (zenith), 180 (horizon behind), so the two
 * modes together cover the whole sky.
 */

typedef enum {
    PAN_TILT_NORMAL = 0,
    PAN_TILT_FLIPPED,
} pan_tilt_mode_t;

typedef struct {
    int32_t az_ref_cdeg;   /* heading the pan faces at 0 */
    int32_t pan_min_cdeg;  /* usable range, within 0..18000 */
    int32_t pan_max_cdeg;
    int32_t tilt_min_cdeg; /* usable range, within 0..18000 */
    int32_t tilt_max_cdeg;
} pan_tilt_config_t;

typedef struct {
    int32_t pan_cdeg;
    int32_t tilt_cdeg;
    pan_tilt_mode_t mode;
} pan_tilt_pose_t;

/* Mount facing north, both servos using their full 180 degrees. */
#define PAN_TILT_CONFIG_DEFAULT \
    ((pan_tilt_config_t){.az_ref_cdeg = 0, .pan_min_cdeg = 0, .pan_max_cdeg = 18000, .tilt_min_cdeg = 0, .tilt_max_cdeg = 18000})

/**
 * @brief Whether a config describes a mount this mapping can drive.
 *
 * Heading in 0..35999, each axis min < max within 0..18000. The calibration
 * comes from NVS, so check it before the rotor gets it.
 */
bool pan_tilt_config_valid(const pan_tilt_config_t *cfg);

/**
 * @brief Servo angles that point at a geographic az/el.
 *
 * Keeps the current mode while it reaches the target, so the pan does not flip
 * back and forth near the border. False if no mode reaches it: don't move.
 */
bool pan_tilt_from_azel(const pan_tilt_config_t *cfg,
                        pan_tilt_mode_t current,
                        int32_t az_cdeg,
                        int32_t el_cdeg,
                        pan_tilt_pose_t *out);

/**
 * @brief Whether some mode reaches a geographic az/el.
 *
 * A batch with an unreachable point is rejected whole.
 */
bool pan_tilt_reachable(const pan_tilt_config_t *cfg, int32_t az_cdeg, int32_t el_cdeg);

/**
 * @brief The mode a pose is physically in.
 *
 * From the side of the zenith the tilt is on; mid-flip it can differ from the
 * label. On the zenith itself the label decides.
 */
pan_tilt_mode_t pan_tilt_pose_mode(const pan_tilt_pose_t *pose);

/**
 * @brief The geographic az/el a pose points at.
 *
 * Inverse of pan_tilt_from_azel(), by pan_tilt_pose_mode(), so a pose mid-flip
 * still reports where it points with el within ±90.
 */
void pan_tilt_to_azel(const pan_tilt_config_t *cfg,
                      const pan_tilt_pose_t *pose,
                      int32_t *az_cdeg,
                      int32_t *el_cdeg);

#endif
