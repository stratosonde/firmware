/**
  ******************************************************************************
  * @file    mission_state.h
  * @brief   Minimal one-way mission state machine (T3 / DDR-0008)
  ******************************************************************************
  * COMMISSIONING -> FLIGHT (ASCENT -> FLOAT). All transitions one-way, never
  * toward higher power. Door anchored to the session bank (DDR-0006):
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

/** @brief Mission states (also the status-byte b6-b7 codes, DDR-0007) */
typedef enum {
    MISSION_COMMISSIONING = 0,  /* Ground: joins allowed, LEDs on, GPS config allowed */
    MISSION_ASCENT        = 1,  /* Flight, fast cadence to capture the climb */
    MISSION_FLOAT         = 2   /* Flight, float cadence (final state) */
} MissionState_t;

/** @brief Ascent duration before settling to float cadence (timer-based, DDR-0008) */
/* D8 (#59): float detection is pressure-based, not a timer. Pressure is the
 * reliable signal at altitude and the determination is effectively
 * stateless-from-sensor — a cold-snap reset no longer stretches the ascent
 * cadence indefinitely (F-028 concern eliminated by construction). */
#ifndef MISSION_FLOAT_DP_HPA
#define MISSION_FLOAT_DP_HPA        2.0f   /* |ΔP| below this = level flight */
#endif
#ifndef MISSION_FLOAT_LEVEL_SAMPLES
#define MISSION_FLOAT_LEVEL_SAMPLES 3      /* consecutive level cycles = FLOAT */
#endif

/**
 * @brief Initialize mission state. Call after MultiRegion_Init().
 *        Door anchoring (DDR-0006): session bank decides; ambiguity -> FLIGHT.
 */
void MissionState_Init(void);

/** @brief Current mission state */
MissionState_t MissionState_Get(void);

/** @brief true if in COMMISSIONING (joins, LEDs, GPS config allowed) */
bool MissionState_IsCommissioning(void);

/** @brief One-way COMMISSIONING -> FLIGHT (ASCENT). Deliberate ground action. */
void MissionState_EnterFlight(void);

/** @brief Call each work cycle: ASCENT -> FLOAT by timer (DDR-0008). */
void MissionState_Update(float pressure_hpa, bool pressure_valid);

/** @brief 2-bit code for the uplink status byte (b6-b7, DDR-0007) */
uint8_t MissionState_GetStatusBits(void);

#ifdef __cplusplus
}
#endif

#endif /* __MISSION_STATE_H */
