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
/* D8 (#59) / finding #7: windowed-range float detection state (no timer) */
static float    s_win_min_hpa = 0.0f;
static float    s_win_max_hpa = 0.0f;
static uint32_t s_win_start_s = 0;
static bool     s_win_active = false;
/* MISSION-01b (#142): launch detection state (COMMISSIONING only) */
static float    s_launch_ref_max_hpa = 0.0f;
static bool     s_have_launch_ref = false;

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
    /* BR-LIFE-007 / MISSION-01b (#142): autonomous launch detection in
     * COMMISSIONING. Track the running MAX of valid pressure readings; a
     * cumulative drop of MISSION_LAUNCH_DP_HPA below that maximum means we
     * are climbing (~50 m). Tracking the max makes slow weather drift
     * self-cancelling on the upside; a fast storm-front DROP of 6 hPa could
     * still false-arm — acceptable: it only moves the unit to ASCENT cadence,
     * and the windowed FLOAT detector will re-settle it if it isn't actually
     * rising... no — FLOAT is terminal, so a false arm on the bench holds
     * ASCENT cadence until powerdown. Tune MISSION_LAUNCH_DP_HPA per site if
     * that matters; the button/arming hook is the deliberate path. */
    if (s_state == MISSION_COMMISSIONING) {
        if (pressure_valid && pressure_hpa > 0.0f) {
            if (!s_have_launch_ref) {
                s_launch_ref_max_hpa = pressure_hpa;
                s_have_launch_ref = true;
            } else if (pressure_hpa > s_launch_ref_max_hpa) {
                s_launch_ref_max_hpa = pressure_hpa;
            } else if (s_launch_ref_max_hpa - pressure_hpa >= MISSION_LAUNCH_DP_HPA) {
                SONDE_LOG_STR("MissionState: LAUNCH detected (pressure departure) -> ASCENT\r\n");
                MissionState_EnterFlight();
            }
        }
        return;
    }

    /* Finding #7 (2026-08-10): ASCENT -> FLOAT by WINDOWED RANGE.
     * Float = pressure stays inside MISSION_FLOAT_RANGE_PCT of ambient for
     * MISSION_FLOAT_WINDOW_S (no altitude guard, #142 — see mission_state.h).
     * Works at any
     * cadence, including the 10 s ascent interval where per-sample deltas
     * are meaningless. Stale/invalid pressure resets the window: it cannot
     * prove float — stay in ASCENT (the safe direction, DDR-0019). The
     * transition itself is one-way (s_state != ASCENT returns early), so
     * FLOAT never leaves once latched. */
    if (s_state != MISSION_ASCENT) {
        return;
    }
    if (!pressure_valid || pressure_hpa <= 0.0f) {
        s_win_active = false;
        return;
    }

    if (!s_win_active) {
        s_win_min_hpa = pressure_hpa;
        s_win_max_hpa = pressure_hpa;
        s_win_start_s = now_s;
        s_win_active = true;
        return;
    }

    if (pressure_hpa < s_win_min_hpa) s_win_min_hpa = pressure_hpa;
    if (pressure_hpa > s_win_max_hpa) s_win_max_hpa = pressure_hpa;

    float range = s_win_max_hpa - s_win_min_hpa;
    if (range > MISSION_FLOAT_RANGE_PCT * pressure_hpa) {
        /* Climbing: restart the window at the current sample. (MISSION-01:
         * no altitude guard — float can be 5-25 km depending on payload and
         * balloon; pad-side protection is that this code only runs in ASCENT,
         * which is entered only by arming or launch detection.) */
        s_win_min_hpa = pressure_hpa;
        s_win_max_hpa = pressure_hpa;
        s_win_start_s = now_s;
        return;
    }

    if ((now_s - s_win_start_s) >= MISSION_FLOAT_WINDOW_S) {
        s_state = MISSION_FLOAT;
        MissionState_Persist();
        SONDE_LOG_STR("MissionState: ASCENT -> FLOAT (pressure level window)\r\n");
    }
}

uint8_t MissionState_GetStatusBits(void)
{
    return ((uint8_t)s_state) & 0x03;
}
