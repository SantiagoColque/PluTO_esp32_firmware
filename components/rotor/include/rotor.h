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
 * The board's side of both contracts in PluTO's docs/contracts: it takes the
 * payloads of coordinates-dto.md, applies the rules that need the time and the
 * mount (clock, expired points, reachability), follows the trajectory through
 * the speed and acceleration limits of motion_profile, and describes all of it
 * in the JSON of device-state.md.
 *
 * Pure like its dependencies: time and clock sync come in as arguments, the
 * FreeRTOS task and the MQTT publishing live in main, and the whole thing runs
 * as a host test (test_host/).
 */

/* Room for the longest state document, with margin. */
#define ROTOR_STATE_JSON_MAX 384

typedef enum {
    ROTOR_MODE_IDLE = 0, /* no trajectory accepted since boot */
    ROTOR_MODE_TRACKING, /* heading to or following a trajectory */
    ROTOR_MODE_HOLDING,  /* after a HOLD, or past the last point and arrived */
} rotor_mode_t;

typedef struct {
    bool present;       /* false until the first payload arrives */
    bool header_valid;  /* false when the header could not be decoded */
    int64_t t_sent_ms;
    int64_t received_ms;
    bool latency_valid; /* false without a decoded header or a synced clock */
    int64_t latency_ms;
    bool accepted;
    const char *error;  /* reject code of device-state.md §5, NULL if accepted */
} rotor_batch_report_t;

typedef struct {
    pan_tilt_config_t mount;
    int64_t tolerance_ms;
    trajectory_t trajectory;
    motion_profile_t pan_axis;
    motion_profile_t tilt_axis;
    bool received_any;
    bool has_target;
    pan_tilt_pose_t target; /* where the trajectory points now; its mode is the hysteresis state */
    bool has_pose;
    pan_tilt_pose_t pose; /* last servo angles commanded, after the profile; its mode is physical */
    rotor_mode_t mode;
    rotor_batch_report_t last_batch;
    uint32_t rejected_total;
} rotor_t;

/**
 * @brief Start idle, with no pose. motion holds the limits for both axes;
 * MOTION_PROFILE_NO_LIMITS commands the target as is.
 */
void rotor_init(rotor_t *rotor,
                const pan_tilt_config_t *mount,
                int64_t tolerance_ms,
                const motion_profile_config_t *motion);

/**
 * @brief Handle one payload from device/<device_id>/coordinates/polar.
 *
 * Returns true if it was accepted. Either way last_batch and rejected_total
 * describe the outcome for the state channel. A HOLD is applied even without a
 * synced clock, since stopping never depends on the time.
 *
 * It updates the target but does not move: it runs on the MQTT task between
 * ticks, and a step here would come on top of the ones rotor_tick() takes. The
 * exception is an axis with no position yet, or a profile without limits:
 * either takes the target as is, so the first accepted batch sets pose right
 * away.
 */
bool rotor_handle_payload(rotor_t *rotor,
                          const uint8_t *payload,
                          size_t payload_len,
                          int64_t now_ms,
                          bool clock_synced);

/**
 * @brief Advance to now_ms: recompute the target, the servo angles and the mode.
 *
 * now_ms is the wall clock of the trajectory; dt_ms comes from a monotonic
 * clock, because SNTP steps the wall clock. After a HOLD the servos keep
 * braking along a ramp until they stop.
 *
 * Returns true when this call changed the commanded angles.
 */
bool rotor_tick(rotor_t *rotor, int64_t now_ms, int32_t dt_ms);

/**
 * @brief Whether the servo outputs can be released (HU-35).
 *
 * True before the first pose, and once both axes have stood still at the
 * same target for the profile's rest_after_ms. A new target turns it false as
 * soon as it arrives, possibly before the commanded angles change, so the
 * driver has to be given the pose again when this goes false even if
 * rotor_tick() reports no change. Which outputs to actually cut is the
 * driver's call: a released servo may sag under the antenna.
 */
bool rotor_outputs_resting(const rotor_t *rotor);

/**
 * @brief Write the state document of device-state.md §3 into buf.
 *
 * Returns its length, or -1 if it does not fit.
 */
int rotor_state_json(const rotor_t *rotor, int64_t now_ms, bool clock_synced, char *buf, size_t buf_len);

const char *rotor_mode_name(rotor_mode_t mode);

#endif
