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
    size_t old_kept = 0;
    size_t new_count;

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

    /* The batch arrives about 2 s before its first point (§10), so what is
     * pending until then stays, or the target would jump ahead. Strictly
     * before, so no two points share an instant. */
    while (old_kept < traj->count && traj->points[old_kept].t_ms < points[0].t_ms) {
        old_kept++;
    }
    new_count = count - first_kept;

    /* Out of room, the oldest go first: those are the ones already passed. */
    if (old_kept + new_count > TRAJECTORY_MAX_POINTS) {
        size_t dropped = old_kept + new_count - TRAJECTORY_MAX_POINTS;

        old_kept -= dropped;
        memmove(traj->points, traj->points + dropped, old_kept * sizeof(traj->points[0]));
    }

    memcpy(traj->points + old_kept, points + first_kept, new_count * sizeof(traj->points[0]));
    traj->count = old_kept + new_count;
    return new_count;
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
