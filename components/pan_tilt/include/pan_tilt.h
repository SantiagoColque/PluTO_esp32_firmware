#ifndef PAN_TILT_H
#define PAN_TILT_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Geographic azimuth/elevation to the angles of a pan-tilt made of two
 * 180-degree servos, as defined in PluTO's docs/contracts/coordinates-dto.md §9.
 *
 *   mode      pan                 tilt
 *   normal    az - az_ref         el
 *   flipped   az - az_ref - 180   180 - el
 *
 * Tilt runs from 0 (horizon ahead) through 90 (zenith) to 180 (horizon
 * behind), so the flipped mode points past the zenith. Between them the two
 * modes cover the whole sky above the horizon. All angles are in centidegrees
 * (0.01 degrees), the unit of the wire format.
 *
 * This is kinematics only: servo trims and the angle-to-PWM conversion belong
 * to the servo driver. No ESP-IDF dependencies, so it runs as a host test
 * (test_host/).
 */

typedef enum {
    PAN_TILT_NORMAL = 0,
    PAN_TILT_FLIPPED,
} pan_tilt_mode_t;

typedef struct {
    int32_t az_ref_cdeg;   /* geographic azimuth the pan faces at 0: the mount's heading */
    int32_t pan_min_cdeg;  /* usable pan range; 0..18000 for a full 180-degree servo */
    int32_t pan_max_cdeg;
    int32_t tilt_min_cdeg; /* usable tilt range; 0..18000 for a full 180-degree servo */
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
 * @brief Servo angles that point at a geographic az/el.
 *
 * Keeps the current mode while it reaches the target and switches only when it
 * cannot, so a target near the border between modes does not flip the pan back
 * and forth. Returns false when neither mode reaches it, for instance below the
 * horizon or outside a narrowed servo range. In that case the servos must not
 * move.
 */
bool pan_tilt_from_azel(const pan_tilt_config_t *cfg,
                        pan_tilt_mode_t current,
                        int32_t az_cdeg,
                        int32_t el_cdeg,
                        pan_tilt_pose_t *out);

/**
 * @brief Whether some mode reaches a geographic az/el.
 *
 * A batch with an unreachable point is rejected whole with `unreachable`
 * (coordinates-dto.md §9, device-state.md §5).
 */
bool pan_tilt_reachable(const pan_tilt_config_t *cfg, int32_t az_cdeg, int32_t el_cdeg);

/**
 * @brief The geographic az/el a pose points at.
 *
 * The inverse of pan_tilt_from_azel(), used to report the position on the
 * state channel (device-state.md §4).
 */
void pan_tilt_to_azel(const pan_tilt_config_t *cfg,
                      const pan_tilt_pose_t *pose,
                      int32_t *az_cdeg,
                      int32_t *el_cdeg);

#endif
