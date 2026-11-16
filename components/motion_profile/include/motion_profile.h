#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Speed and acceleration limits for one servo axis (HU-36), and when that axis
 * has stood still long enough for its output to be released (HU-35).
 *
 * It works in servo space, in centidegrees, after pan_tilt has chosen the mode,
 * so the 180-degree swing of a flip is a ramp like any other. With a steady
 * tick a fixed target is reached without overshoot; a longer tick while
 * braking can pass it slightly, so rotor clamps the result to the mount.
 *
 * Elapsed time is an argument and must come from a monotonic clock: SNTP steps
 * the wall clock.
 */

/*
 * Longest step taken at once. A longer gap, such as a task held off for a
 * while, is taken as this much: the axis falls behind instead of jumping.
 */
#define MOTION_PROFILE_MAX_DT_MS 100

/* Both limits positive, or both 0 for no ramp at all. */
typedef struct {
    int32_t v_max_cdeg_s;  /* top speed */
    int32_t a_max_cdeg_s2; /* top acceleration and deceleration */
    int32_t rest_after_ms; /* still this long at the same target to rest; 0 never rests */
} motion_profile_config_t;

/* The target as is, for tests that want no ramp. */
#define MOTION_PROFILE_NO_LIMITS ((motion_profile_config_t){0})

typedef struct {
    motion_profile_config_t cfg;
    bool has_position; /* false until the first target or reset: nothing to ramp from */
    bool has_target;   /* false after a brake, which stops wherever it can */
    float pos_cdeg;    /* float so a slow axis, a fraction of a centidegree per step, still advances */
    float vel_cdeg_s;
    int32_t target_cdeg;
    int32_t still_ms; /* time settled at an unchanged target, capped at rest_after_ms */
} motion_profile_t;

void motion_profile_init(motion_profile_t *axis, const motion_profile_config_t *cfg);

/**
 * @brief Start from a known position, at rest.
 *
 * Without it the first target is taken as is, since a servo without a sensor
 * gives no position to ramp from. An encoder reading goes here.
 */
void motion_profile_reset(motion_profile_t *axis, int32_t pos_cdeg);

/**
 * @brief Move toward target_cdeg for dt_ms and return the position to command.
 *
 * dt_ms is the time since the previous step or brake. 0 or negative moves
 * nothing, which lets a caller update the target between steps; anything
 * above MOTION_PROFILE_MAX_DT_MS counts as that. A changed target ends a rest
 * right away, before any movement.
 */
int32_t motion_profile_step(motion_profile_t *axis, int32_t target_cdeg, int32_t dt_ms);

/**
 * @brief Slow down at the acceleration limit, wherever that stops the axis,
 * and return the position to command.
 *
 * What a HOLD does: there is no target to reach, only the motion to end.
 */
int32_t motion_profile_brake(motion_profile_t *axis, int32_t dt_ms);

bool motion_profile_has_position(const motion_profile_t *axis);

/** @brief The position to command, rounded to whole centidegrees. */
int32_t motion_profile_position(const motion_profile_t *axis);

/** @brief Stopped, and on the target unless it was braking. */
bool motion_profile_settled(const motion_profile_t *axis);

/**
 * @brief Settled at the same target for rest_after_ms: the output can be
 * released until the target changes.
 */
bool motion_profile_resting(const motion_profile_t *axis);

#endif
