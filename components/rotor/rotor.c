#include "rotor.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

void rotor_init(rotor_t *rotor,
                const pan_tilt_config_t *mount,
                int64_t tolerance_ms,
                const motion_profile_config_t *motion)
{
    memset(rotor, 0, sizeof(*rotor));
    rotor->mount = *mount;
    rotor->tolerance_ms = tolerance_ms;
    rotor->mode = ROTOR_MODE_IDLE;
    trajectory_init(&rotor->trajectory);
    motion_profile_init(&rotor->pan_axis, motion);
    motion_profile_init(&rotor->tilt_axis, motion);
}

/* Returns the reject code, or NULL when the batch was loaded. */
static const char *load_batch(rotor_t *rotor, const coord_dto_msg_t *msg, int64_t now_ms, bool clock_synced)
{
    trajectory_point_t points[PLUTO_DTO_MAX_POINTS];

    if (coord_dto_is_hold(msg)) {
        trajectory_clear(&rotor->trajectory);
        return NULL;
    }

    /* Without a synced clock the target instants mean nothing (§6.4). */
    if (!clock_synced) {
        return "clock_not_synced";
    }

    for (uint8_t i = 0; i < msg->count; i++) {
        points[i].t_ms = msg->t0_ms + (int64_t)msg->points[i].dt_ms;
        points[i].az_cdeg = msg->points[i].az_cdeg;
        points[i].el_cdeg = msg->points[i].el_cdeg;

        /* Rejected whole rather than clipped (§9). */
        if (!pan_tilt_reachable(&rotor->mount, points[i].az_cdeg, points[i].el_cdeg)) {
            return "unreachable";
        }
    }

    if (trajectory_load(&rotor->trajectory, points, msg->count, now_ms, rotor->tolerance_ms) == 0) {
        return "all_expired";
    }
    return NULL;
}

bool rotor_handle_payload(rotor_t *rotor,
                          const uint8_t *payload,
                          size_t payload_len,
                          int64_t now_ms,
                          bool clock_synced)
{
    coord_dto_msg_t msg;
    coord_dto_err_t err = coord_dto_decode(payload, payload_len, &msg);
    rotor_batch_report_t *report = &rotor->last_batch;

    memset(report, 0, sizeof(*report));
    report->present = true;
    report->received_ms = now_ms;

    if (err != COORD_DTO_OK) {
        report->error = coord_dto_err_code(err);
    } else {
        report->header_valid = true;
        report->t_sent_ms = msg.t_sent_ms;
        report->latency_valid = clock_synced;
        report->latency_ms = now_ms - msg.t_sent_ms;
        report->error = load_batch(rotor, &msg, now_ms, clock_synced);
    }

    report->accepted = report->error == NULL;
    if (!report->accepted) {
        rotor->rejected_total++;
        return false;
    }

    rotor->received_any = true;
    rotor_tick(rotor, now_ms, 0);
    return true;
}

static int32_t clamp(int32_t value, int32_t min, int32_t max)
{
    return value < min ? min : value > max ? max : value;
}

/* Copies the profiled angles into the commanded pose. Returns true if they changed. */
static bool update_pose(rotor_t *rotor)
{
    pan_tilt_pose_t pose;
    bool changed;

    if (!motion_profile_has_position(&rotor->pan_axis) || !motion_profile_has_position(&rotor->tilt_axis)) {
        return false;
    }

    /* A late tick can carry an axis a little past its target; never past the mount. */
    pose.pan_cdeg = clamp(motion_profile_position(&rotor->pan_axis), rotor->mount.pan_min_cdeg,
                          rotor->mount.pan_max_cdeg);
    pose.tilt_cdeg = clamp(motion_profile_position(&rotor->tilt_axis), rotor->mount.tilt_min_cdeg,
                           rotor->mount.tilt_max_cdeg);
    /* Halfway through a flip the tilt can still be on the old side of the
     * zenith: the pose takes the mode it is physically in, with the target's
     * as the tie-break on the zenith itself. */
    pose.mode = rotor->target.mode;
    pose.mode = pan_tilt_pose_mode(&pose);

    changed = !rotor->has_pose || pose.pan_cdeg != rotor->pose.pan_cdeg ||
              pose.tilt_cdeg != rotor->pose.tilt_cdeg || pose.mode != rotor->pose.mode;
    rotor->pose = pose;
    rotor->has_pose = true;
    return changed;
}

bool rotor_tick(rotor_t *rotor, int64_t now_ms, int32_t dt_ms)
{
    int32_t az_cdeg;
    int32_t el_cdeg;
    pan_tilt_pose_t target;
    bool following = trajectory_target(&rotor->trajectory, now_ms, &az_cdeg, &el_cdeg);
    bool changed;

    /* An unreachable intermediate target keeps the last one: never move
     * toward something the mount cannot point at. The hysteresis goes by the
     * target's mode, not by the servos': halfway through a flip they are
     * between modes, and going by them could turn the flip back. */
    if (following && pan_tilt_from_azel(&rotor->mount,
                                        rotor->has_target ? rotor->target.mode : PAN_TILT_NORMAL,
                                        az_cdeg,
                                        el_cdeg,
                                        &target)) {
        rotor->target = target;
        rotor->has_target = true;
    }

    if (following && rotor->has_target) {
        motion_profile_step(&rotor->pan_axis, rotor->target.pan_cdeg, dt_ms);
        motion_profile_step(&rotor->tilt_axis, rotor->target.tilt_cdeg, dt_ms);
    } else {
        /* After a HOLD, or before any target: whatever is moving slows down
         * at the acceleration limit instead of stopping dead (§6.6). */
        motion_profile_brake(&rotor->pan_axis, dt_ms);
        motion_profile_brake(&rotor->tilt_axis, dt_ms);
    }
    changed = update_pose(rotor);

    if (!following) {
        rotor->mode = rotor->received_any ? ROTOR_MODE_HOLDING : ROTOR_MODE_IDLE;
    } else if (now_ms < rotor->trajectory.points[rotor->trajectory.count - 1].t_ms ||
               !motion_profile_settled(&rotor->pan_axis) || !motion_profile_settled(&rotor->tilt_axis)) {
        /* Past the last point but still on the way to it is not holding yet:
         * device-state.md describes holding as standing still. */
        rotor->mode = ROTOR_MODE_TRACKING;
    } else {
        rotor->mode = ROTOR_MODE_HOLDING;
    }
    return changed;
}

bool rotor_outputs_resting(const rotor_t *rotor)
{
    return !rotor->has_pose ||
           (motion_profile_resting(&rotor->pan_axis) && motion_profile_resting(&rotor->tilt_axis));
}

const char *rotor_mode_name(rotor_mode_t mode)
{
    switch (mode) {
    case ROTOR_MODE_IDLE:
        return "idle";
    case ROTOR_MODE_TRACKING:
        return "tracking";
    case ROTOR_MODE_HOLDING:
        return "holding";
    }
    return "unknown";
}

static int write_batch_json(const rotor_batch_report_t *report, char *buf, size_t buf_len)
{
    char t_sent[24] = "null";
    char latency[24] = "null";
    char error[40] = "null";

    if (report->header_valid) {
        snprintf(t_sent, sizeof(t_sent), "%" PRId64, report->t_sent_ms);
    }
    if (report->latency_valid) {
        snprintf(latency, sizeof(latency), "%" PRId64, report->latency_ms);
    }
    if (report->error != NULL) {
        snprintf(error, sizeof(error), "\"%s\"", report->error);
    }

    return snprintf(buf,
                    buf_len,
                    "{\"t_sent_ms\":%s,\"received_ms\":%" PRId64 ",\"latency_ms\":%s,"
                    "\"accepted\":%s,\"error\":%s}",
                    t_sent,
                    report->received_ms,
                    latency,
                    report->accepted ? "true" : "false",
                    error);
}

int rotor_state_json(const rotor_t *rotor, int64_t now_ms, bool clock_synced, char *buf, size_t buf_len)
{
    char az[16] = "null";
    char el[16] = "null";
    const char *pan_mode = "null";
    char batch[192] = "null";
    int written;

    if (rotor->has_pose) {
        int32_t az_cdeg;
        int32_t el_cdeg;

        pan_tilt_to_azel(&rotor->mount, &rotor->pose, &az_cdeg, &el_cdeg);
        snprintf(az, sizeof(az), "%" PRId32, az_cdeg);
        snprintf(el, sizeof(el), "%" PRId32, el_cdeg);
        pan_mode = rotor->pose.mode == PAN_TILT_FLIPPED ? "\"flipped\"" : "\"normal\"";
    }
    if (rotor->last_batch.present) {
        written = write_batch_json(&rotor->last_batch, batch, sizeof(batch));
        if (written < 0 || (size_t)written >= sizeof(batch)) {
            return -1;
        }
    }

    written = snprintf(buf,
                       buf_len,
                       "{\"v\":1,\"ts_ms\":%" PRId64 ",\"clock_synced\":%s,\"mode\":\"%s\","
                       "\"az_cdeg\":%s,\"el_cdeg\":%s,\"pan_mode\":%s,\"batch\":%s,"
                       "\"rejected_total\":%" PRIu32 "}",
                       now_ms,
                       clock_synced ? "true" : "false",
                       rotor_mode_name(rotor->mode),
                       az,
                       el,
                       pan_mode,
                       batch,
                       rotor->rejected_total);
    if (written < 0 || (size_t)written >= buf_len) {
        return -1;
    }
    return written;
}
