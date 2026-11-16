#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Target between messages (coordinates-dto.md §6): linear in time between
 * points with the azimuth the short way round, held at both ends. Time is
 * always an argument, so it runs as a host test.
 */

#define TRAJECTORY_MAX_POINTS 16

typedef struct {
    int64_t t_ms;    /* target instant, epoch UTC milliseconds */
    int32_t az_cdeg; /* geographic azimuth, 0..35999 */
    int32_t el_cdeg; /* elevation, -9000..9000 */
} trajectory_point_t;

typedef struct {
    trajectory_point_t points[TRAJECTORY_MAX_POINTS];
    size_t count; /* 0: nothing to follow, stay put */
} trajectory_t;

void trajectory_init(trajectory_t *traj);

/**
 * @brief Load a new batch.
 *
 * Replaces pending points from the batch's first instant on, like the server's
 * merge_trajectory; when full, the oldest pending go first. Drops points older
 * than tolerance_ms. Returns how many were kept; 0 (all expired, too many, out
 * of order) leaves traj untouched. Order is rechecked so it never divides by 0.
 */
size_t trajectory_load(trajectory_t *traj,
                       const trajectory_point_t *points,
                       size_t count,
                       int64_t now_ms,
                       int64_t tolerance_ms);

/** @brief HOLD: drop every pending point. */
void trajectory_clear(trajectory_t *traj);

/**
 * @brief Where the antenna should point at now_ms.
 *
 * False when there is nothing to follow: stay where you are.
 */
bool trajectory_target(const trajectory_t *traj, int64_t now_ms, int32_t *az_cdeg, int32_t *el_cdeg);

#endif
