/**
  ******************************************************************************
  * @file    mission_logic.c
  * @brief   Pure launch/float pressure detection (STAB-06/#153, STAB-07/#154)
  ******************************************************************************
  * No HAL, no statics, no logging — host-testable policy. mission_state.c
  * owns persistence (DR3) and logging.
  ******************************************************************************
  */

#include "mission_logic.h"

void LaunchDetector_Reset(LaunchDetector_t *d)
{
    d->ref_max_hpa = 0.0f;
    d->ref_set_s = 0;
    d->have_ref = false;
}

bool LaunchDetector_Update(LaunchDetector_t *d, float pressure_hpa,
                           bool pressure_valid, uint32_t now_s)
{
    if (!pressure_valid || pressure_hpa <= 0.0f) {
        return false;   /* stale/invalid proves nothing; window untouched */
    }

    /* RV-06 (#162): backward time step (LSE->LSI failover restarts the RTC
     * counter near zero) — re-seed, never evaluate a wrapped delta (the
     * Deadman_Check pattern). */
    if (d->have_ref && now_s < d->ref_set_s) {
        d->ref_max_hpa = pressure_hpa;
        d->ref_set_s = now_s;
        return false;
    }

    /* STAB-06 (#153): the reference AGES OUT. An old maximum (high-pressure
     * weather system hours ago) can no longer serve as the launch reference;
     * reseed from the current reading and keep watching. */
    if (!d->have_ref || (now_s - d->ref_set_s) > MISSION_LAUNCH_REF_WINDOW_S) {
        d->ref_max_hpa = pressure_hpa;
        d->ref_set_s = now_s;
        d->have_ref = true;
        return false;
    }

    if (pressure_hpa > d->ref_max_hpa) {
        d->ref_max_hpa = pressure_hpa;
        d->ref_set_s = now_s;
        return false;
    }

    return (d->ref_max_hpa - pressure_hpa) >= MISSION_LAUNCH_DP_HPA;
}

bool LaunchDetector_HasRef(const LaunchDetector_t *d)
{
    return d->have_ref;
}

float LaunchDetector_RefHpa(const LaunchDetector_t *d)
{
    return d->ref_max_hpa;
}

void FloatDetector_Reset(FloatDetector_t *d)
{
    d->win_min_hpa = 0.0f;
    d->win_max_hpa = 0.0f;
    d->win_first_hpa = 0.0f;
    d->win_start_s = 0;
    d->samples = 0;
    d->active = false;
}

bool FloatDetector_Update(FloatDetector_t *d, float pressure_hpa,
                          bool pressure_valid, uint32_t now_s,
                          bool min_ascent_met)
{
    if (!pressure_valid || pressure_hpa <= 0.0f) {
        d->active = false;  /* stale/invalid cannot prove float (DDR-0019) */
        return false;
    }

    /* RV-06 (#162): backward time step (RTC restart) — re-seed the window at
     * the current sample; (now - start) must never wrap into an instant latch
     * of the terminal state. */
    if (d->active && now_s < d->win_start_s) {
        d->active = false;
    }

    if (!d->active) {
        d->win_min_hpa = pressure_hpa;
        d->win_max_hpa = pressure_hpa;
        d->win_first_hpa = pressure_hpa;
        d->win_start_s = now_s;
        d->samples = 1;
        d->active = true;
        return false;
    }

    d->samples++;
    if (pressure_hpa < d->win_min_hpa) d->win_min_hpa = pressure_hpa;
    if (pressure_hpa > d->win_max_hpa) d->win_max_hpa = pressure_hpa;

    float range = d->win_max_hpa - d->win_min_hpa;
    if (range > MISSION_FLOAT_RANGE_PCT * pressure_hpa) {
        /* Climbing (or descending): restart the window at the current sample */
        d->win_min_hpa = pressure_hpa;
        d->win_max_hpa = pressure_hpa;
        d->win_first_hpa = pressure_hpa;
        d->win_start_s = now_s;
        d->samples = 1;
        return false;
    }

    if ((now_s - d->win_start_s) >= MISSION_FLOAT_WINDOW_S) {
        /* STAB-07 (#154), second guard: NET displacement over the window must
         * be small, not just the range. A slow climb (0.5-1 m/s) can stay
         * inside the wide range band at altitude while steadily ascending —
         * its net change over the window is a large fraction of the band.
         * A real float wobbles around a mean: net ~= 0. */
        float net = (pressure_hpa > d->win_first_hpa)
                    ? (pressure_hpa - d->win_first_hpa)
                    : (d->win_first_hpa - pressure_hpa);
        /* RV-07 (#163): a degenerate window must not latch — require a minimum
         * sample count, so two readings an hour apart (SURVIVAL cadence) can
         * never satisfy "sustained window". Too few samples but otherwise
         * flat+still: KEEP ACCUMULATING (do not restart — that would discard
         * the samples and never latch at long cadence). At 10 s ascent
         * cadence the count is always met long before the time window. */
        if (min_ascent_met &&
            net <= 0.25f * MISSION_FLOAT_RANGE_PCT * pressure_hpa) {
            if (d->samples >= 4) {
                return true;  /* genuine float: flat, still, measured, above launch */
            }
            return false;
        }
        /* Guard not met (no accumulated ascent, still moving, or too few
         * samples): restart the window — stay in ASCENT (cadence cost beats
         * a false terminal latch). */
        d->win_min_hpa = pressure_hpa;
        d->win_max_hpa = pressure_hpa;
        d->win_first_hpa = pressure_hpa;
        d->win_start_s = now_s;
        d->samples = 1;
    }

    return false;
}
