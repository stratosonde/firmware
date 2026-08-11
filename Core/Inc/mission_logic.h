/**
  ******************************************************************************
  * @file    mission_logic.h
  * @brief   Pure launch/float pressure detection (STAB-06/#153, STAB-07/#154)
  ******************************************************************************
  * Extracted from mission_state.c so the policy is host-testable (R47/R49
  * precedent: mission_state.c stays the HAL/persistence wrapper). No HAL, no
  * statics — all state is caller-owned, which also makes reset semantics
  * explicit at the call site.
  ******************************************************************************
  */

#ifndef __MISSION_LOGIC_H
#define __MISSION_LOGIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "mission_state.h"   /* the tuning knobs (MISSION_* macros) */

/* STAB-06 (#153): the launch reference maximum is valid only for this long.
 * The old all-time running max let a high-pressure WEATHER system serve as
 * the launch reference hours/days later: ordinary lower pressure 6 hPa below
 * the historical max false-armed flight. 2 h bounds the reference to recent
 * behavior; slower weather drift never accumulates into a false launch. */
#ifndef MISSION_LAUNCH_REF_WINDOW_S
#define MISSION_LAUNCH_REF_WINDOW_S (2U * 3600U)
#endif

/* STAB-07 (#154): the flat-pressure FLOAT latch requires this much accumulated
 * pressure drop below the launch reference (~170 m of climb). A bench-armed
 * unit at ground pressure, or a shallow low-altitude stall, cannot reach the
 * terminal FLOAT latch. This is ascent SINCE LAUNCH, not an absolute altitude
 * ceiling — float altitude stays payload/balloon-dependent (#142). */
#ifndef MISSION_FLOAT_MIN_ASCENT_DP_HPA
#define MISSION_FLOAT_MIN_ASCENT_DP_HPA 20.0f
#endif

/* --- Launch detector (COMMISSIONING): bounded reference window --- */
typedef struct {
    float    ref_max_hpa;   /* highest pressure seen inside the window */
    uint32_t ref_set_s;     /* when ref_max was (re)seeded */
    bool     have_ref;
    bool     pinned;        /* F1 (#167): restored launch ref never ages out */
} LaunchDetector_t;

void LaunchDetector_Reset(LaunchDetector_t *d);

/** Feed one pressure sample (call each work cycle in COMMISSIONING).
  * @retval true when a real pressure departure (>= MISSION_LAUNCH_DP_HPA
  *         below the bounded reference) indicates launch. */
bool LaunchDetector_Update(LaunchDetector_t *d, float pressure_hpa,
                           bool pressure_valid, uint32_t now_s);

/** Reference accessors for the FLOAT min-ascent guard (STAB-07). */
bool LaunchDetector_HasRef(const LaunchDetector_t *d);
float LaunchDetector_RefHpa(const LaunchDetector_t *d);

/** F1 (#167): install a launch reference restored from the backup domain
 * after a mid-ascent reset. PINNED: it is the actual launch pressure, not a
 * weather maximum — it must never age out (else FLOAT becomes unreachable
 * again ~2 h after the reset). */
void LaunchDetector_SetRef(LaunchDetector_t *d, float ref_hpa, uint32_t now_s);

/* --- Float detector (ASCENT): windowed range + min-ascent guard --- */
typedef struct {
    float    win_min_hpa;
    float    win_max_hpa;
    float    win_first_hpa;  /* window-start pressure: net-displacement guard */
    uint32_t win_start_s;
    uint16_t samples;        /* RV-07 (#163): min-sample-count guard */
    bool     active;
} FloatDetector_t;

void FloatDetector_Reset(FloatDetector_t *d);

/** Feed one pressure sample (call each work cycle in ASCENT).
  * @param min_ascent_met: STAB-07 guard — caller computes
  *        (launch_ref - pressure) >= MISSION_FLOAT_MIN_ASCENT_DP_HPA.
  * @retval true exactly once when pressure has stayed inside
  *         MISSION_FLOAT_RANGE_PCT for MISSION_FLOAT_WINDOW_S AND the
  *         min-ascent guard holds. Guard not met: the window restarts
  *         (stay in ASCENT — FLOAT is terminal, so never latch cheap). */
bool FloatDetector_Update(FloatDetector_t *d, float pressure_hpa,
                          bool pressure_valid, uint32_t now_s,
                          bool min_ascent_met);

#ifdef __cplusplus
}
#endif

#endif /* __MISSION_LOGIC_H */
