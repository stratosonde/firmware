/**
  ******************************************************************************
  * @file    mission_state.h
  * @brief   Minimal one-way mission state machine (T3 / DDR-0002)
  ******************************************************************************
  * COMMISSIONING -> FLIGHT (ASCENT -> FLOAT). All transitions one-way, never
  * toward higher power. Door anchored to the session bank (DDR-0018):
  * ambiguity resolves to FLIGHT — a mid-air reboot must never land in
  * commissioning.
  ******************************************************************************
  */

#ifndef __MISSION_STATE_H
#define __MISSION_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** @brief Mission states (also the status-byte b6-b7 codes, DDR-0003) */
typedef enum {
    MISSION_COMMISSIONING = 0,  /* Ground: joins allowed, LEDs on, GPS config allowed */
    MISSION_ASCENT        = 1,  /* Flight, fast cadence to capture the climb */
    MISSION_FLOAT         = 2   /* Flight, float cadence (final state) */
} MissionState_t;

/* Float detection (D8/#59, FR-17/#98, reworked 2026-08-10 finding #7):
 * WINDOWED RANGE, not per-sample delta. The old consecutive-sample test
 * (|ΔP|/P < 2% for 3 samples) broke twice over: at float altitude it demanded
 * |ΔP| < 0.14 hPa (below MS5607 noise, so FLOAT might never latch), and at
 * the 10 s ascent cadence a climbing balloon moves only ~0.6%/sample, so ANY
 * per-sample threshold either false-latches in the climb or never trips.
 *
 * New rule: FLOAT latches when pressure stays inside a WIDE band
 * (MISSION_FLOAT_RANGE_PCT of ambient) for a sustained window
 * (MISSION_FLOAT_WINDOW_S). A 5 m/s climb moves ~18% of ambient per 5 min at
 * ANY altitude, far outside the 5% band, so ascent can never false-latch
 * regardless of cadence. The latch itself is one-way (DESCENT descoped,
 * #126), so once FLOAT it never leaves — deliberate: latches fast, stays.
 * (MISSION-01/#142: the original 150 hPa altitude guard was removed — float
 * altitude is payload/balloon-dependent, 5-25 km.)
 *
 * These are compile-time knobs today; if ops wants them tunable per-flight,
 * move them into SystemConfig_t (config.h) — the detector reads the macros
 * in exactly one place (mission_state.c) to make that a mechanical change. */
#ifndef MISSION_FLOAT_RANGE_PCT
#define MISSION_FLOAT_RANGE_PCT 0.05f        /* (max-min)/P below this = level */
#endif
#ifndef MISSION_FLOAT_WINDOW_S
#define MISSION_FLOAT_WINDOW_S 900U          /* sustained this long = FLOAT */
#endif
/* STAB-07 (#154): 900 s (was 300) so that ascent STALLS (1/3/5/10 min in the
 * review's profile set) cannot reach the latch — combined with the
 * net-displacement guard in mission_logic.c, no expected ascent trajectory
 * permanently false-latches the terminal FLOAT state. Cadence cost: FLOAT
 * relaxes up to 10 min later than before; trivial against a multi-week float. */
/* MISSION-01 (#142), maintainer decision 2026-08-11: NO altitude guard on the
 * FLOAT latch. Design float altitude is payload/balloon-dependent (5-25 km),
 * so a fixed ceiling could block the latch for an entire multi-week float.
 * Pad-side protection comes from the state machine instead: float detection
 * only runs in ASCENT, and ASCENT is entered only by deliberate arming
 * (MissionState_EnterFlight — button; see below) or by autonomous launch
 * detection (BR-LIFE-007). FLOAT is terminal — never exits (DESCENT stays
 * descoped, #126; BR-LIFE-013/014 accepted as first-flight gaps, DDR-0002
 * amendment). */

/* BR-LIFE-007 launch detection (MISSION-01b, #142): COMMISSIONING tracks the
 * running MAX pressure; a cumulative drop of this many hPa below that maximum
 * means we are climbing (~50 m). One knob, intentionally obvious; tune per
 * balloon if bench weather false-arms. */
#ifndef MISSION_LAUNCH_DP_HPA
#define MISSION_LAUNCH_DP_HPA 6.0f
#endif
/* R3-10 (#222): two-evidence launch detection - the cumulative drop must
 * ALSO hold continuously for this long before launch latches. A momentary
 * downward spike (handling bump, sensor glitch, elevator) arms a candidate
 * that any recovery cancels. Sustained real events (driving uphill to the
 * launch site, aircraft transport) still arm - accepted residual: bounded
 * energy cost of a false ASCENT (fast cadence + GNSS) until the FLOAT
 * latch guards or recovery sort it out. */
#ifndef MISSION_LAUNCH_CONFIRM_S
#define MISSION_LAUNCH_CONFIRM_S 120U
#endif

/* DDR-0002 mission cadence (finding #7): the consumer that was missing.
 * ASCENT = fast updates to capture the climb; FLOAT = relaxed cadence.
 * Applied in SendTxData ONLY when the power model is healthy (NORMAL or
 * CONSERVATIVE) — REDUCED/RECOVERY/SURVIVAL keep their longer intervals
 * (never toward higher power). NOTE: 10 s uplinks are duty-cycle/dwell-time
 * sensitive in some regions; ascent is short and this is deliberate. */
#ifndef MISSION_ASCENT_TX_INTERVAL_MS
#define MISSION_ASCENT_TX_INTERVAL_MS 10000U    /* 10 s during ascent */
#endif
#ifndef MISSION_FLOAT_TX_INTERVAL_MS
#define MISSION_FLOAT_TX_INTERVAL_MS 300000U    /* 5 min at float */
#endif

/**
 * @brief Initialize mission state. Call after MultiRegion_Init().
 *        Door anchoring (DDR-0018): session bank decides; ambiguity -> FLIGHT.
 */
void MissionState_Init(void);

/** @brief Current mission state */
MissionState_t MissionState_Get(void);

/** @brief true if in COMMISSIONING (joins, LEDs, GPS config allowed) */
bool MissionState_IsCommissioning(void);

/** @brief One-way COMMISSIONING -> FLIGHT (ASCENT). Deliberate ground action:
 *        called by the arming input (arming_input.c, PRETEST-DEC-01 2026-08-16:
 *        PB13, active-low to GND, sharing the SPI2_SCK net — sampled only in
 *        commissioning, where the quiet watch performs no flash logging) or
 *        by the pressure-based launch detector in MissionState_Update(). The
 *        call must never live in lora_app.c (R6/#192). */
void MissionState_EnterFlight(void);

/** @brief Call each work cycle: ASCENT -> FLOAT by windowed pressure range
 *        (DDR-0002). now_s is RTC seconds (TIMER_IF_GetTime). */
void MissionState_Update(float pressure_hpa, bool pressure_valid, uint32_t now_s);

/** @brief 2-bit code for the uplink status byte (b6-b7, DDR-0003) */
uint8_t MissionState_GetStatusBits(void);

#ifdef __cplusplus
}
#endif

#endif /* __MISSION_STATE_H */
