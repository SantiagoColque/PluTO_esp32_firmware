#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

/* Speed/acceleration limits (HU-36) and rest detection (HU-35) for one servo
 * axis, in servo space. dt must be monotonic: SNTP steps the wall clock. */

/* A longer gap counts as this much: the axis falls behind instead of jumping. */
#define MOTION_PROFILE_MAX_DT_MS 100

/* Both limits positive, or both 0 for no ramp at all. */
typedef struct {
    int32_t v_max_cdeg_s;
    int32_t a_max_cdeg_s2; /* also the deceleration */
    int32_t rest_after_ms; /* 0 never rests */
} motion_profile_config_t;

/* No ramp: the target as is. */
#define MOTION_PROFILE_NO_LIMITS ((motion_profile_config_t){0})

typedef struct {
    motion_profile_config_t cfg;
    bool has_position; /* nothing to ramp from until the first target or reset */
    bool has_target;   /* false after a brake */
    float pos_cdeg;    /* float so a slow axis still advances by sub-cdeg steps */
    float vel_cdeg_s;
    int32_t target_cdeg;
    int32_t still_ms; /* capped at rest_after_ms */
} motion_profile_t;

void motion_profile_init(motion_profile_t *axis, const motion_profile_config_t *cfg);

/**
 * @brief Start from a known position, at rest.
 *
 * E.g. an encoder reading. Without it, the first target is taken as is.
 */
void motion_profile_reset(motion_profile_t *axis, int32_t pos_cdeg);

/**
 * @brief Move toward target_cdeg for dt_ms and return the position to command.
 *
 * dt_ms <= 0 only updates the target; above MOTION_PROFILE_MAX_DT_MS counts as
 * that. A long tick while braking can overshoot slightly, so rotor clamps.
 */
int32_t motion_profile_step(motion_profile_t *axis, int32_t target_cdeg, int32_t dt_ms);

/** @brief HOLD: brake at the acceleration limit and stop wherever that lands. */
int32_t motion_profile_brake(motion_profile_t *axis, int32_t dt_ms);

bool motion_profile_has_position(const motion_profile_t *axis);

/** @brief The position to command, rounded to whole centidegrees. */
int32_t motion_profile_position(const motion_profile_t *axis);

/** @brief Stopped, and on the target unless it was braking. */
bool motion_profile_settled(const motion_profile_t *axis);

/** @brief Settled at one target for rest_after_ms: the output may be released. */
bool motion_profile_resting(const motion_profile_t *axis);

#endif
