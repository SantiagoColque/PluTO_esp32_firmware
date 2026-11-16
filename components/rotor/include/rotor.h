#ifndef ROTOR_H
#define ROTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "coord_dto.h"
#include "motion_profile.h"
#include "pan_tilt.h"
#include "trajectory.h"

/*
 * Board side of coordinates-dto.md and device-state.md: takes payloads, follows
 * the trajectory within the motion limits, reports state. Time comes in as
 * arguments, so it runs as a host test.
 */

/* Longest state document, with margin. */
#define ROTOR_STATE_JSON_MAX 384

typedef enum {
    ROTOR_MODE_IDLE = 0, /* no trajectory accepted since boot */
    ROTOR_MODE_TRACKING, /* heading to or following a trajectory */
    ROTOR_MODE_HOLDING,  /* after a HOLD, or arrived past the last point */
} rotor_mode_t;

typedef struct {
    bool present;       /* false until the first payload */
    bool header_valid;
    int64_t t_sent_ms;
    int64_t received_ms;
    bool latency_valid; /* needs a decoded header and a synced clock */
    int64_t latency_ms;
    bool accepted;
    const char *error;  /* reject code, NULL if accepted */
} rotor_batch_report_t;

typedef struct {
    pan_tilt_config_t mount;
    int64_t tolerance_ms;
    trajectory_t trajectory;
    motion_profile_t pan_axis;
    motion_profile_t tilt_axis;
    bool received_any;
    bool has_target;
    pan_tilt_pose_t target; /* its mode is the hysteresis state */
    bool has_pose;
    pan_tilt_pose_t pose; /* last commanded, after the profile; its mode is physical */
    bool has_measured;
    pan_tilt_pose_t measured; /* from the encoders */
    rotor_mode_t mode;
    rotor_batch_report_t last_batch;
    uint32_t rejected_total;
} rotor_t;

/**
 * @brief Start idle, with no pose.
 *
 * MOTION_PROFILE_NO_LIMITS commands the target as is.
 */
void rotor_init(rotor_t *rotor,
                const pan_tilt_config_t *mount,
                int64_t tolerance_ms,
                const motion_profile_config_t *motion);

/**
 * @brief Handle one payload from device/<device_id>/coordinates/polar.
 *
 * True if accepted. A HOLD applies even unsynced. Sets the target only; the
 * first accepted batch also sets pose, since there is nothing to ramp from.
 */
bool rotor_handle_payload(rotor_t *rotor,
                          const uint8_t *payload,
                          size_t payload_len,
                          int64_t now_ms,
                          bool clock_synced);

/**
 * @brief Advance to now_ms: recompute target, servo angles and mode.
 *
 * now_ms is the wall clock; dt_ms must be monotonic since SNTP steps the wall
 * clock. True when the commanded angles changed.
 */
bool rotor_tick(rotor_t *rotor, int64_t now_ms, int32_t dt_ms);

/**
 * @brief Hand in the servo angles the encoders read this tick.
 *
 * Telemetry only; valid=false reports meas and err_cdeg as null.
 */
void rotor_set_measured(rotor_t *rotor, bool valid, int32_t pan_cdeg, int32_t tilt_cdeg);

/**
 * @brief Whether the servo outputs can be released (HU-35).
 *
 * Before the first pose, or once both axes sat at one target for rest_after_ms.
 * A new target, or any after a HOLD, clears it.
 */
bool rotor_outputs_resting(const rotor_t *rotor);

/**
 * @brief Write the state document (device-state.md §3) into buf.
 *
 * Returns the length written, or -1 if it does not fit.
 */
int rotor_state_json(const rotor_t *rotor, int64_t now_ms, bool clock_synced, char *buf, size_t buf_len);

const char *rotor_mode_name(rotor_mode_t mode);

#endif
