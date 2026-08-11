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
 * (MISSION_FLOAT_WINDOW_S), and only above a minimum altitude proxy
 * (MISSION_FLOAT_MAX_PRESSURE_HPA — a pad-side or just-launched unit can
 * never latch). A 5 m/s climb moves ~18% of ambient per 5 min at ANY
 * altitude, far outside the 5% band, so ascent can never false-latch
 * regardless of cadence. The latch itself is one-way (DESCENT descoped,
 * #126), so once FLOAT it never leaves — deliberate: latches fast, stays.
 *
 * These are compile-time knobs today; if ops wants them tunable per-flight,
 * move them into SystemConfig_t (config.h) — the detector reads the macros
 * in exactly one place (mission_state.c) to make that a mechanical change. */
#ifndef MISSION_FLOAT_RANGE_PCT
#define MISSION_FLOAT_RANGE_PCT 0.05f        /* (max-min)/P below this = level */
#endif
#ifndef MISSION_FLOAT_WINDOW_S
#define MISSION_FLOAT_WINDOW_S 300U          /* sustained this long = FLOAT */
#endif
#ifndef MISSION_FLOAT_MAX_PRESSURE_HPA
#define MISSION_FLOAT_MAX_PRESSURE_HPA 150.0f /* ~13.6 km: latch allowed only above */
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

/** @brief One-way COMMISSIONING -> FLIGHT (ASCENT). Deliberate ground action. */
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
