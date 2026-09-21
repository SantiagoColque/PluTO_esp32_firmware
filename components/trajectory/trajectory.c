#include "trajectory.h"

#include <string.h>

#define FULL_TURN_CDEG 36000
#define HALF_TURN_CDEG 18000

static int32_t wrap_azimuth(int64_t az_cdeg)
{
    int64_t wrapped = az_cdeg % FULL_TURN_CDEG;

    if (wrapped < 0) {
        wrapped += FULL_TURN_CDEG;
    }
    return (int32_t)wrapped;
}

/* Signed azimuth change from `from` to `to` along the shorter way around, so
 * crossing north goes 359.50 -> 0.00 -> 0.50 instead of all the way back. */
static int32_t shortest_azimuth_delta(int32_t from, int32_t to)
{
    int32_t delta = to - from;

    if (delta > HALF_TURN_CDEG) {
        delta -= FULL_TURN_CDEG;
    } else if (delta < -HALF_TURN_CDEG) {
        delta += FULL_TURN_CDEG;
    }
    return delta;
}

void trajectory_init(trajectory_t *traj)
{
    traj->count = 0;
}

size_t trajectory_load(trajectory_t *traj,
                       const trajectory_point_t *points,
                       size_t count,
                       int64_t now_ms,
                       int64_t tolerance_ms)
{
    size_t first_kept = 0;

    if (count == 0 || count > TRAJECTORY_MAX_POINTS) {
        return 0;
    }
    for (size_t i = 1; i < count; i++) {
        if (points[i].t_ms <= points[i - 1].t_ms) {
            return 0;
        }
    }

    /* Points are in time order, so the expired ones are a prefix. */
    while (first_kept < count && points[first_kept].t_ms < now_ms - tolerance_ms) {
        first_kept++;
    }
    if (first_kept == count) {
        return 0;
    }

    traj->count = count - first_kept;
    memcpy(traj->points, points + first_kept, traj->count * sizeof(traj->points[0]));
    return traj->count;
}

void trajectory_clear(trajectory_t *traj)
{
    traj->count = 0;
}

bool trajectory_target(const trajectory_t *traj, int64_t now_ms, int32_t *az_cdeg, int32_t *el_cdeg)
{
    const trajectory_point_t *last;

    if (traj->count == 0) {
        return false;
    }

    if (now_ms <= traj->points[0].t_ms) {
        *az_cdeg = traj->points[0].az_cdeg;
        *el_cdeg = traj->points[0].el_cdeg;
        return true;
    }

    for (size_t i = 1; i < traj->count; i++) {
        const trajectory_point_t *a = &traj->points[i - 1];
        const trajectory_point_t *b = &traj->points[i];

        if (now_ms <= b->t_ms) {
            int64_t span = b->t_ms - a->t_ms;
            int64_t elapsed = now_ms - a->t_ms;
            int64_t az_delta = shortest_azimuth_delta(a->az_cdeg, b->az_cdeg);
            int64_t el_delta = (int64_t)b->el_cdeg - a->el_cdeg;

            *az_cdeg = wrap_azimuth(a->az_cdeg + az_delta * elapsed / span);
            *el_cdeg = (int32_t)(a->el_cdeg + el_delta * elapsed / span);
            return true;
        }
    }

    /* Past the last point: stay there rather than extrapolate (§6.7). */
    last = &traj->points[traj->count - 1];
    *az_cdeg = last->az_cdeg;
    *el_cdeg = last->el_cdeg;
    return true;
}
