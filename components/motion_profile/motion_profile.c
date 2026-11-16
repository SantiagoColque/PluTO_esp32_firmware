#include "motion_profile.h"

#include <math.h>
#include <string.h>

/* Float steps land a few ulps off the target; without this the axis never settles. */
#define LANDING_SLACK_CDEG 0.05f

static int32_t clamp_dt(int32_t dt_ms)
{
    if (dt_ms < 0) {
        return 0;
    }
    return dt_ms > MOTION_PROFILE_MAX_DT_MS ? MOTION_PROFILE_MAX_DT_MS : dt_ms;
}

/* Fastest speed this step that still stops within dist_cdeg, losing dv per step.
 * Summed over whole steps, not sqrt(2 a d): the continuous curve underestimates
 * the stopping distance and overshoots. n is the number of later braking steps. */
static float braking_speed(float dist_cdeg, float dv_cdeg_s, float dt_s)
{
    float steps = dist_cdeg / (dv_cdeg_s * dt_s);
    float n = floorf((sqrtf(8.0f * steps + 1.0f) - 1.0f) / 2.0f);

    /* sqrtf can round n across a boundary; nudging it back is enough. */
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
    /* Braking harder than allowed only happens when the target moved back or
     * after a long tick; the axis then overshoots a little and returns. */
    float vel = fminf(fmaxf(copysignf(speed, error), axis->vel_cdeg_s - dv), axis->vel_cdeg_s + dv);

    axis->pos_cdeg += vel * dt_s;
    axis->vel_cdeg_s = vel;

    /* Only the position snaps; the speed drops to zero next step, within the limit. */
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
        /* Capped, so weeks of idling cannot overflow it. */
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
