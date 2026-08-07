/**
  ******************************************************************************
  * @file    mission_state.c
  * @brief   Minimal one-way mission state machine (T3 / DDR-0008)
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
/* D8 (#59): pressure-trend float detection state (no timer) */
static float    s_last_pressure_hpa = 0.0f;
static uint8_t  s_level_count = 0;
static bool     s_have_pressure_ref = false;

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

    /* Door anchor (DDR-0006): the session bank decides, not the lone flag.
     * FW-1: the bank is now the Tier-1 credential store — IsRegionJoined()
     * only returns true when a CRC-valid Tier-1 copy supplied the context,
     * so this anchors to Tier-1 presence even if the DR3 record is corrupt. */
    bool bank_commissioned =
        MultiRegion_IsRegionJoined(LORAMAC_REGION_US915) ||
        MultiRegion_IsRegionJoined(LORAMAC_REGION_EU868) ||
        MultiRegion_IsRegionJoined(LORAMAC_REGION_AS923) ||
        MultiRegion_IsRegionJoined(LORAMAC_REGION_AU915);

    if (bank_commissioned) {
        /* Bank shows commissioning complete -> FLIGHT, even if DR3 is corrupt */
        s_state = (bkp_valid && persisted != MISSION_COMMISSIONING)
                  ? persisted : MISSION_ASCENT;
    } else if (bkp_valid && persisted != MISSION_COMMISSIONING) {
        /* Bank virgin but state record says flight: ambiguity -> FLIGHT */
        s_state = persisted;
    } else {
        /* Virgin bank, no credible flight record -> COMMISSIONING */
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

void MissionState_Update(float pressure_hpa, bool pressure_valid)
{
    /* D8 (#59): ASCENT -> FLOAT by pressure trend, not elapsed time.
     * Float = |ΔP| below threshold for N consecutive work cycles. Stale or
     * invalid pressure cannot prove float — stay in ASCENT (more uplinks,
     * the safe direction, DDR-0002). No timer to restart on cold-snap reset. */
    if (s_state != MISSION_ASCENT) {
        return;
    }
    if (!pressure_valid) {
        s_level_count = 0;
        s_have_pressure_ref = false;
        return;
    }

    if (s_have_pressure_ref) {
        float dp = pressure_hpa - s_last_pressure_hpa;
        if (dp < 0.0f) dp = -dp;
        if (dp < MISSION_FLOAT_DP_HPA) {
            if (s_level_count < 255) s_level_count++;
        } else {
            s_level_count = 0;
        }
        if (s_level_count >= MISSION_FLOAT_LEVEL_SAMPLES) {
            s_state = MISSION_FLOAT;
            MissionState_Persist();
            SONDE_LOG_STR("MissionState: ASCENT -> FLOAT (pressure level)\r\n");
        }
    }
    s_last_pressure_hpa = pressure_hpa;
    s_have_pressure_ref = true;
}

uint8_t MissionState_GetStatusBits(void)
{
    return ((uint8_t)s_state) & 0x03;
}
