#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Speed and acceleration limits for one servo axis (HU-36).
 *
 * It works in servo space, in centidegrees, after pan_tilt has chosen the mode.
 * There each axis lives in its own interval with no wraparound, so the 180
 * degree swing of a flip is a ramp like any other and the axis never leaves
 * the range on the way: a fixed target is approached along a braking curve
 * that ends on it, and a target that moves back toward the axis is overshot
 * by at most the distance needed to stop, which stays short of the previous
 * target.
 *
 * Elapsed time is an argument, never read here, so the logic runs as a host
 * test (test_host/). It has to come from a monotonic clock: the wall clock is
 * stepped by SNTP, and a jump would read as a long step or a negative one.
 */

/*
 * Longest step taken at once. A longer gap, such as a task held off for a
 * while, is taken as this much: the axis falls behind instead of jumping.
 */
#define MOTION_PROFILE_MAX_DT_MS 100

typedef struct {
    int32_t v_max_cdeg_s;  /* top speed; 0 for no speed limit */
    int32_t a_max_cdeg_s2; /* top acceleration and deceleration; 0 for no limit */
} motion_profile_config_t;

/* With both limits at 0 the axis is commanded the target as is. */
#define MOTION_PROFILE_NO_LIMITS ((motion_profile_config_t){0})

typedef struct {
    motion_profile_config_t cfg;
    bool has_position; /* false until the first target or reset: nothing to ramp from */
    bool has_target;   /* false after a brake, which stops wherever it can */
    float pos_cdeg;    /* float so a slow axis, a fraction of a centidegree per step, still advances */
    float vel_cdeg_s;
    int32_t target_cdeg;
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
 * above MOTION_PROFILE_MAX_DT_MS counts as that.
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

#endif
