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

    if (!d->active) {
        d->win_min_hpa = pressure_hpa;
        d->win_max_hpa = pressure_hpa;
        d->win_first_hpa = pressure_hpa;
        d->win_start_s = now_s;
        d->active = true;
        return false;
    }

    if (pressure_hpa < d->win_min_hpa) d->win_min_hpa = pressure_hpa;
    if (pressure_hpa > d->win_max_hpa) d->win_max_hpa = pressure_hpa;

    float range = d->win_max_hpa - d->win_min_hpa;
    if (range > MISSION_FLOAT_RANGE_PCT * pressure_hpa) {
        /* Climbing (or descending): restart the window at the current sample */
        d->win_min_hpa = pressure_hpa;
        d->win_max_hpa = pressure_hpa;
        d->win_first_hpa = pressure_hpa;
        d->win_start_s = now_s;
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
        if (min_ascent_met &&
            net <= 0.25f * MISSION_FLOAT_RANGE_PCT * pressure_hpa) {
            return true;    /* genuine float: flat, still, AND well above launch */
        }
        /* Guard not met (no accumulated ascent, or still moving): restart the
         * window — stay in ASCENT (cadence cost beats a false terminal latch). */
        d->win_min_hpa = pressure_hpa;
        d->win_max_hpa = pressure_hpa;
        d->win_first_hpa = pressure_hpa;
        d->win_start_s = now_s;
    }

    return false;
}
