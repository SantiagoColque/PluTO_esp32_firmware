#include "motion_profile.h"

#include <math.h>
#include <string.h>

/*
 * A step that ends this close to the target ends on it. Float addition can
 * land a few ulps off (about 0.002 centidegrees at 180 degrees), and without
 * this the axis would never count as settled.
 */
#define LANDING_SLACK_CDEG 0.05f

static int32_t clamp_dt(int32_t dt_ms)
{
    if (dt_ms < 0) {
        return 0;
    }
    return dt_ms > MOTION_PROFILE_MAX_DT_MS ? MOTION_PROFILE_MAX_DT_MS : dt_ms;
}

/*
 * The fastest speed for this step from which the axis can still stop within
 * dist_cdeg, losing dv_cdeg_s of speed every later step of dt_s.
 *
 * Worked out step by step rather than with the continuous sqrt(2 a d): the
 * axis moves in whole steps, and the continuous curve underestimates the
 * stopping distance enough to overshoot.
 *
 * At speed v this step and then v - dv, v - 2 dv, ... the axis covers
 * dt * ((n + 1) v - n (n + 1) dv / 2), with n = floor(v / dv). In units of
 * dv * dt that is linear in v for each n, so the speed that covers exactly
 * dist comes out in closed form once n is known.
 */
static float braking_speed(float dist_cdeg, float dv_cdeg_s, float dt_s)
{
    float steps = dist_cdeg / (dv_cdeg_s * dt_s);
    float n = floorf((sqrtf(8.0f * steps + 1.0f) - 1.0f) / 2.0f);

    /* The square root can round across a boundary; the curve is continuous
     * there, so nudging n back is all it takes. */
    if ((n + 1.0f) * (n + 2.0f) / 2.0f <= steps) {
        n += 1.0f;
    } else if (n > 0.0f && n * (n + 1.0f) / 2.0f > steps) {
        n -= 1.0f;
    }
    return (steps / (n + 1.0f) + n / 2.0f) * dv_cdeg_s;
}

static void advance(motion_profile_t *axis, float target_cdeg, float dt_s)
{
    float error = target_cdeg - axis->pos_cdeg;
    float dv = (float)axis->cfg.a_max_cdeg_s2 * dt_s;
    float speed = fminf(braking_speed(fabsf(error), dv, dt_s), (float)axis->cfg.v_max_cdeg_s);
    /* The braking curve wants to slow down faster than allowed only when the
     * target moved back toward the axis, or after a longer tick. Then it goes
     * a little past and comes back. */
    float vel = fminf(fmaxf(copysignf(speed, error), axis->vel_cdeg_s - dv), axis->vel_cdeg_s + dv);

    axis->pos_cdeg += vel * dt_s;
    axis->vel_cdeg_s = vel;

    /* Only the position is snapped: the speed it landed at is the one it
     * really had, and it drops to zero on the next step, within the limit. */
    if (fabsf(target_cdeg - axis->pos_cdeg) <= LANDING_SLACK_CDEG) {
        axis->pos_cdeg = target_cdeg;
    }
}

static void update_rest(motion_profile_t *axis, bool woken, int32_t dt_ms)
{
    int32_t left = axis->cfg.rest_after_ms - axis->still_ms;

    if (woken || !motion_profile_settled(axis)) {
        axis->still_ms = 0;
    } else if (axis->cfg.rest_after_ms > 0) {
        /* Capped rather than summed, so a board left alone for weeks cannot
         * overflow it. */
        axis->still_ms = dt_ms >= left ? axis->cfg.rest_after_ms : axis->still_ms + dt_ms;
    }
}

void motion_profile_init(motion_profile_t *axis, const motion_profile_config_t *cfg)
{
    memset(axis, 0, sizeof(*axis));
    axis->cfg = *cfg;
}

void motion_profile_reset(motion_profile_t *axis, int32_t pos_cdeg)
{
    axis->has_position = true;
    axis->has_target = false;
    axis->pos_cdeg = (float)pos_cdeg;
    axis->vel_cdeg_s = 0.0f;
    axis->still_ms = 0;
}

int32_t motion_profile_step(motion_profile_t *axis, int32_t target_cdeg, int32_t dt_ms)
{
    bool woken = !axis->has_target || target_cdeg != axis->target_cdeg;
    int32_t dt = clamp_dt(dt_ms);

    axis->has_target = true;
    axis->target_cdeg = target_cdeg;

    if (!axis->has_position || (axis->cfg.v_max_cdeg_s == 0 && axis->cfg.a_max_cdeg_s2 == 0)) {
        axis->pos_cdeg = (float)target_cdeg;
        axis->vel_cdeg_s = 0.0f;
        axis->has_position = true;
    } else if (dt > 0) {
        advance(axis, (float)target_cdeg, (float)dt / 1000.0f);
    }

    update_rest(axis, woken, dt);
    return motion_profile_position(axis);
}

int32_t motion_profile_brake(motion_profile_t *axis, int32_t dt_ms)
{
    int32_t dt = clamp_dt(dt_ms);

    axis->has_target = false;

    /* MOTION_PROFILE_NO_LIMITS never builds up a speed: no case of its own. */
    if (dt > 0) {
        float dt_s = (float)dt / 1000.0f;
        float dv = (float)axis->cfg.a_max_cdeg_s2 * dt_s;

        if (fabsf(axis->vel_cdeg_s) <= dv) {
            axis->vel_cdeg_s = 0.0f;
        } else {
            axis->vel_cdeg_s -= copysignf(dv, axis->vel_cdeg_s);
            axis->pos_cdeg += axis->vel_cdeg_s * dt_s;
        }
    }

    update_rest(axis, false, dt);
    return motion_profile_position(axis);
}

bool motion_profile_has_position(const motion_profile_t *axis)
{
    return axis->has_position;
}

int32_t motion_profile_position(const motion_profile_t *axis)
{
    return (int32_t)lroundf(axis->pos_cdeg);
}

bool motion_profile_settled(const motion_profile_t *axis)
{
    return axis->vel_cdeg_s == 0.0f && (!axis->has_target || axis->pos_cdeg == (float)axis->target_cdeg);
}

bool motion_profile_resting(const motion_profile_t *axis)
{
    return axis->cfg.rest_after_ms > 0 && axis->still_ms >= axis->cfg.rest_after_ms;
}
