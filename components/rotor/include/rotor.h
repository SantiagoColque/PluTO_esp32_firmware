#ifndef ROTOR_H
#define ROTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "coord_dto.h"
#include "pan_tilt.h"
#include "trajectory.h"

/*
 * The board's side of both contracts in PluTO's docs/contracts: it takes the
 * payloads of coordinates-dto.md, applies the rules that need the time and the
 * mount (clock, expired points, reachability), follows the trajectory, and
 * describes all of it in the JSON of device-state.md.
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
    ROTOR_MODE_HOLDING,  /* stopped after a HOLD or past the last point */
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
    bool received_any;
    bool has_pose;
    pan_tilt_pose_t pose; /* last servo angles commanded */
    rotor_mode_t mode;
    rotor_batch_report_t last_batch;
    uint32_t rejected_total;
} rotor_t;

void rotor_init(rotor_t *rotor, const pan_tilt_config_t *mount, int64_t tolerance_ms);

/**
 * @brief Handle one payload from device/<device_id>/coordinates/polar.
 *
 * Returns true if it was accepted. Either way last_batch and rejected_total
 * describe the outcome for the state channel. A HOLD is applied even without a
 * synced clock, since stopping never depends on the time.
 */
bool rotor_handle_payload(rotor_t *rotor,
                          const uint8_t *payload,
                          size_t payload_len,
                          int64_t now_ms,
                          bool clock_synced);

/**
 * @brief Advance to now_ms: recompute the target, the servo angles and the mode.
 *
 * Returns true when the commanded angles changed, which is when the servo
 * driver has to move.
 */
bool rotor_tick(rotor_t *rotor, int64_t now_ms);

/**
 * @brief Write the state document of device-state.md §3 into buf.
 *
 * Returns its length, or -1 if it does not fit.
 */
int rotor_state_json(const rotor_t *rotor, int64_t now_ms, bool clock_synced, char *buf, size_t buf_len);

const char *rotor_mode_name(rotor_mode_t mode);

#endif
