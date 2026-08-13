/**
  ******************************************************************************
  * @file    mission_state.c
  * @brief   Minimal one-way mission state machine (T3 / DDR-0002)
  ******************************************************************************
  * State persists in RTC backup register DR3 (survives reset, not power loss
  * without VBAT backup — hence the door anchor, not the register, decides).
  * R01/R02: DR0-DR2 are owned by timer_if SysTime — see backup_regs.h.
  ******************************************************************************
  */

#include "main.h"
#include "mission_state.h"
#include "mission_logic.h"  /* STAB-06/07 (#153/#154): pure detectors */
#include "multiregion_context.h"
#include "backup_regs.h"
#include "SEGGER_RTT.h"
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */

extern RTC_HandleTypeDef hrtc;

/* DR3 layout: upper 16 bits = magic, lower 16 bits = MissionState_t */
#define MISSION_STATE_BKP_REG   BKP_REG_MISSION_STATE
#define MISSION_STATE_MAGIC     0xA55A0000UL
#define MISSION_STATE_MASK      0xFFFF0000UL

static MissionState_t s_state = MISSION_COMMISSIONING;
/* STAB-06/07 (#153/#154): the detection policy lives in the pure,
 * host-testable mission_logic.c; the state structs stay here (caller-owned). */
static LaunchDetector_t s_launch_det;
static FloatDetector_t  s_float_det;
/* F1 (#167): ASCENT restored without a launch reference (backup-domain wipe
 * mid-flight) — the min-ascent guard is unprovable, so FLOAT falls back to
 * the window+net+sample guards alone. That path implies a commissioned,
 * airborne unit (session bank intact); documented tradeoff. */
static bool s_no_launch_ref = false;

/* F1 (#167): DR15 layout: upper 16 bits = magic, lower 16 = launch ref
 * pressure x10 (max ~1100 hPa -> 11000 < 65536). */
#define LAUNCH_REF_MAGIC    0x5A5A0000UL
#define LAUNCH_REF_MASK     0xFFFF0000UL

static void MissionState_Persist(void)
{
    HAL_RTCEx_BKUPWrite(&hrtc, MISSION_STATE_BKP_REG,
                        MISSION_STATE_MAGIC | (uint32_t)s_state);
}

void MissionState_Init(void)
{
    /* Idempotent: gates inside LoRaWAN_Init need the state decided early */
    static bool s_inited = false;
    if (s_inited) {
        return;
    }
    s_inited = true;

    /* DR-12: explicit resets - the stated rationale for caller-owned
     * detector state ("reset semantics explicit at the call site",
     * mission_logic.h) - instead of relying on static zero-init, which is
     * only coincidentally equivalent. */
    LaunchDetector_Reset(&s_launch_det);
    FloatDetector_Reset(&s_float_det);

    uint32_t raw = HAL_RTCEx_BKUPRead(&hrtc, MISSION_STATE_BKP_REG);
    bool bkp_valid = ((raw & MISSION_STATE_MASK) == MISSION_STATE_MAGIC) &&
                     ((raw & 0xFFFFUL) <= (uint32_t)MISSION_FLOAT);
    MissionState_t persisted = (MissionState_t)(raw & 0xFFFFUL);

    /* Door anchor (DDR-0018): the session bank decides, not the lone flag.
     * FW-1: the bank is now the Tier-1 credential store — IsRegionJoined()
     * only returns true when a CRC-valid Tier-1 copy supplied the context,
     * so this anchors to Tier-1 presence even if the DR3 record is corrupt. */
    bool bank_commissioned =
        MultiRegion_IsRegionJoined(LORAMAC_REGION_US915) ||
        MultiRegion_IsRegionJoined(LORAMAC_REGION_EU868) ||
        MultiRegion_IsRegionJoined(LORAMAC_REGION_AS923) ||
        MultiRegion_IsRegionJoined(LORAMAC_REGION_AU915);

    /* MISSION-01 (#142, DDR-0002 amendment): the persisted DR3 record now wins
     * outright — including COMMISSIONING. Previously a commissioned bank
     * forced ASCENT even with a valid DR3=COMMISSIONING, so a unit entered
     * ASCENT on the bench immediately after joining and held the 10 s ascent
     * cadence until launch. Flight entry is now EXPLICIT (arming or launch
     * detection) and persisted, so DR3=COMMISSIONING is authoritative.
     * DDR-0018 preserved for the true ambiguity: bank commissioned but DR3
     * wiped (power loss without VBAT) mid-flight -> ASCENT, never
     * commissioning with its join/LED privileges. */
    if (bkp_valid) {
        s_state = persisted;
    } else if (bank_commissioned) {
        s_state = MISSION_ASCENT;
    } else {
        s_state = MISSION_COMMISSIONING;
    }

    /* F1 (#167): restore the launch reference for a persisted ASCENT, or the
     * min-ascent guard can never be proven and FLOAT is unreachable for the
     * rest of the mission (a recoverable reset mid-climb became a permanent
     * 10 s / GNSS-on energy failure). DR15 dies with the backup domain that
     * also carried DR3, so a reconstruct-from-bank ASCENT has no ref by
     * construction -> s_no_launch_ref fallback. */
    if (s_state == MISSION_ASCENT) {
        uint32_t lr = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_LAUNCH_REF);
        if ((lr & LAUNCH_REF_MASK) == LAUNCH_REF_MAGIC && (lr & 0xFFFFUL) > 0) {
            LaunchDetector_SetRef(&s_launch_det,
                                  (float)(lr & 0xFFFFUL) / 10.0f, 0);
        } else {
            s_no_launch_ref = true;
            SONDE_LOG_STR("MissionState: ASCENT without launch ref - FLOAT on window guards only\r\n");
        }
    }

    MissionState_Persist();

    SONDE_LOG("MissionState: %s (bank %s, DR3 %s)\r\n",
                      s_state == MISSION_COMMISSIONING ? "COMMISSIONING" :
                      s_state == MISSION_ASCENT ? "FLIGHT-ASCENT" : "FLIGHT-FLOAT",
                      bank_commissioned ? "commissioned" : "virgin",
                      bkp_valid ? "valid" : "invalid");
}

MissionState_t MissionState_Get(void)
{
    return s_state;
}

bool MissionState_IsCommissioning(void)
{
    return (s_state == MISSION_COMMISSIONING);
}

void MissionState_EnterFlight(void)
{
    /* One-way door: COMMISSIONING -> ASCENT, never back */
    if (s_state == MISSION_COMMISSIONING) {
        s_state = MISSION_ASCENT;
        MissionState_Persist();
        SONDE_LOG_STR("MissionState: COMMISSIONING -> FLIGHT (ASCENT)\r\n");
    }
}

void MissionState_Update(float pressure_hpa, bool pressure_valid, uint32_t now_s)
{
    /* BR-LIFE-007 / MISSION-01b (#142) + STAB-06 (#153): autonomous launch
     * detection in COMMISSIONING, bounded reference window (mission_logic.c):
     * a high-pressure weather system older than MISSION_LAUNCH_REF_WINDOW_S
     * can no longer serve as the launch baseline. A fast storm-front DROP of
     * 6 hPa inside the window can still false-arm - accepted: it only moves
     * the unit to ASCENT cadence. */
    if (s_state == MISSION_COMMISSIONING) {
        if (LaunchDetector_Update(&s_launch_det, pressure_hpa, pressure_valid, now_s)) {
            SONDE_LOG_STR("MissionState: LAUNCH detected (pressure departure) -> ASCENT\r\n");
            /* F1 (#167): persist the launch reference BEFORE entering flight —
             * a reset during ascent must be able to restore it or FLOAT is
             * unreachable for the rest of the mission. */
            uint16_t ref_x10 = (uint16_t)(LaunchDetector_RefHpa(&s_launch_det) * 10.0f);
            HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_LAUNCH_REF,
                                LAUNCH_REF_MAGIC | (uint32_t)ref_x10);
            MissionState_EnterFlight();
        }
        return;
    }

    /* Finding #7 (2026-08-10) + STAB-07 (#154): ASCENT -> FLOAT by WINDOWED
     * RANGE plus two guards (mission_logic.c): accumulated ascent since
     * launch >= MISSION_FLOAT_MIN_ASCENT_DP_HPA (a bench-armed unit at
     * ground pressure can never reach the terminal latch) and small NET
     * displacement over the window (a slow climb inside the wide range band
     * keeps moving, a float wobbles around a mean). Stale/invalid pressure
     * resets the window: it cannot prove float (DDR-0019). The latch is
     * one-way: FLOAT never leaves once latched. */
    if (s_state != MISSION_ASCENT) {
        return;
    }
    bool min_ascent_met =
        s_no_launch_ref ||
        (LaunchDetector_HasRef(&s_launch_det) &&
         (LaunchDetector_RefHpa(&s_launch_det) - pressure_hpa) >= MISSION_FLOAT_MIN_ASCENT_DP_HPA);
    if (FloatDetector_Update(&s_float_det, pressure_hpa, pressure_valid, now_s,
                             min_ascent_met)) {
        s_state = MISSION_FLOAT;
        MissionState_Persist();
        SONDE_LOG_STR("MissionState: ASCENT -> FLOAT (level + still + well above launch)\r\n");
    }
}

uint8_t MissionState_GetStatusBits(void)
{
    return ((uint8_t)s_state) & 0x03;
}
