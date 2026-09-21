#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * What the board does between messages, as defined in PluTO's
 * docs/contracts/coordinates-dto.md §6.3, §6.6 and §6.7:
 *
 * - a new batch replaces the pending points instead of merging with them;
 * - before the first point the target is the first point;
 * - between two points the target is interpolated linearly in time, with the
 *   azimuth taking the shortest way around;
 * - after the last point the target stays there;
 * - a HOLD drops everything that is pending.
 *
 * Time is always an argument, never read here, so the logic runs as a host
 * test (test_host/). Points use absolute instants and plain integers so this
 * does not depend on the wire format.
 */

#define TRAJECTORY_MAX_POINTS 16

typedef struct {
    int64_t t_ms;    /* target instant, epoch UTC milliseconds */
    int32_t az_cdeg; /* geographic azimuth, 0..35999 */
    int32_t el_cdeg; /* elevation, -9000..9000 */
} trajectory_point_t;

typedef struct {
    trajectory_point_t points[TRAJECTORY_MAX_POINTS];
    size_t count; /* 0: nothing to follow, stay where you are */
} trajectory_t;

void trajectory_init(trajectory_t *traj);

/**
 * @brief Replace whatever is pending with a new batch (§6.7).
 *
 * Points more than tolerance_ms in the past are dropped (§6.3). Returns how
 * many points were kept. 0 means the batch is discarded and the current
 * trajectory is left untouched. That happens when every point had expired,
 * when there are more than TRAJECTORY_MAX_POINTS, or when the points are not
 * in strictly increasing time order. The decoder already guarantees the
 * order (§6.5); checking it again here keeps the interpolation from ever
 * dividing by zero.
 */
size_t trajectory_load(trajectory_t *traj,
                       const trajectory_point_t *points,
                       size_t count,
                       int64_t now_ms,
                       int64_t tolerance_ms);

/** @brief HOLD: drop every pending point (§6.6). */
void trajectory_clear(trajectory_t *traj);

/**
 * @brief Where the antenna should point at now_ms.
 *
 * Returns false when there is nothing to follow (after init or a HOLD), which
 * means staying where it is.
 */
bool trajectory_target(const trajectory_t *traj, int64_t now_ms, int32_t *az_cdeg, int32_t *el_cdeg);

#endif
