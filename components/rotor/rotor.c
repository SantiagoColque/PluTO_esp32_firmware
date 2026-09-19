#include "rotor.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

void rotor_init(rotor_t *rotor, const pan_tilt_config_t *mount, int64_t tolerance_ms)
{
    memset(rotor, 0, sizeof(*rotor));
    rotor->mount = *mount;
    rotor->tolerance_ms = tolerance_ms;
    rotor->mode = ROTOR_MODE_IDLE;
    trajectory_init(&rotor->trajectory);
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
    rotor_tick(rotor, now_ms);
    return true;
}

bool rotor_tick(rotor_t *rotor, int64_t now_ms)
{
    int32_t az_cdeg;
    int32_t el_cdeg;
    pan_tilt_pose_t pose;
    bool changed = false;

    if (!trajectory_target(&rotor->trajectory, now_ms, &az_cdeg, &el_cdeg)) {
        rotor->mode = rotor->received_any ? ROTOR_MODE_HOLDING : ROTOR_MODE_IDLE;
        return false;
    }

    /* An unreachable intermediate target keeps the last angles: never move
     * toward something the mount cannot point at. */
    if (pan_tilt_from_azel(&rotor->mount,
                           rotor->has_pose ? rotor->pose.mode : PAN_TILT_NORMAL,
                           az_cdeg,
                           el_cdeg,
                           &pose)) {
        changed = !rotor->has_pose || pose.pan_cdeg != rotor->pose.pan_cdeg ||
                  pose.tilt_cdeg != rotor->pose.tilt_cdeg || pose.mode != rotor->pose.mode;
        rotor->pose = pose;
        rotor->has_pose = true;
    }

    rotor->mode = now_ms < rotor->trajectory.points[rotor->trajectory.count - 1].t_ms
                      ? ROTOR_MODE_TRACKING
                      : ROTOR_MODE_HOLDING;
    return changed;
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
