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

#include "mission_state.h"
#include "SEGGER_RTT.h"
#include "backup_regs.h"
#include "main.h"
#include "mission_logic.h"    /* STAB-06/07 (#153/#154): pure detectors */
#include "mission_manifest.h" /* H-01 (#281): durable lifecycle record */
#include "multiregion_context.h"
#include "sonde_log.h" /* R50 (#47): compile-time log gate */

extern RTC_HandleTypeDef hrtc;

/* DR3 layout: upper 16 bits = magic, lower 16 bits = MissionState_t */
#define MISSION_STATE_BKP_REG BKP_REG_MISSION_STATE
#define MISSION_STATE_MAGIC 0xA55A0000UL
#define MISSION_STATE_MASK 0xFFFF0000UL

static MissionState_t s_state = MISSION_COMMISSIONING;
/* STAB-06/07 (#153/#154): the detection policy lives in the pure,
 * host-testable mission_logic.c; the state structs stay here (caller-owned). */
static LaunchDetector_t s_launch_det;
/* H-11 (#287): the commissioning -> flight door requires this latch (in
 * addition to PROVISIONED) before it opens. RAM-only by design: a latch
 * that survives a power-cut is wrong - a reboot means reverify GNSS. */
static bool s_gnss_accepted = false;

void MissionState_MarkGnssAccepted(bool accepted) { s_gnss_accepted = accepted; }
bool MissionState_GnssAccepted(void) { return s_gnss_accepted; }
static FloatDetector_t s_float_det;
/* F1 (#167): ASCENT restored without a launch reference (backup-domain wipe
 * mid-flight) — the min-ascent guard is unprovable, so FLOAT falls back to
 * the window+net+sample guards alone. That path implies a commissioned,
 * airborne unit (session bank intact); documented tradeoff. */
static bool s_no_launch_ref = false;

/* F1 (#167): DR15 layout: upper 16 bits = magic, lower 16 = launch ref
 * pressure x10 (max ~1100 hPa -> 11000 < 65536). */
#define LAUNCH_REF_MAGIC 0x5A5A0000UL
#define LAUNCH_REF_MASK 0xFFFF0000UL

static void MissionState_Persist(void) {
  uint32_t written = MISSION_STATE_MAGIC | (uint32_t)s_state;
  HAL_RTCEx_BKUPWrite(&hrtc, MISSION_STATE_BKP_REG, written);
  /* Bench finding (2026-08-19): on a unit with a dead backup domain (VBAT
   * rail fault - LSE also fails) BKP writes silently do not stick. A
   * commissioned unit then always boots "DR3 invalid" -> bank door-anchor ->
   * ASCENT. Read back and name the failure class instead of persisting
   * silently into the void. */
  static bool s_persist_warned = false;
  if (!s_persist_warned &&
      HAL_RTCEx_BKUPRead(&hrtc, MISSION_STATE_BKP_REG) != written) {
    s_persist_warned = true;
    SONDE_LOG_STR("MissionState: WARNING DR3 persist failed - backup domain dead (VBAT?)\r\n");
  }
}

void MissionState_Init(void) {
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
   * so this anchors to Tier-1 presence even if the DR3 record is corrupt.
   * Region-set (region_set.h): one loop over the compile-time configured
   * set, so this anchor can never drift from the pre-join table; compiled-
   * out regions read as never-joined inside IsRegionJoined itself. */
  const LoRaMacRegion_t *door_regions;
  uint8_t door_region_count = MultiRegion_GetConfiguredRegions(&door_regions);
  bool bank_commissioned = false;
  for (uint8_t i = 0; i < door_region_count; i++) {
    if (MultiRegion_IsRegionJoined(door_regions[i])) {
      bank_commissioned = true;
      break;
    }
  }

  /* MISSION-01 (#142, DDR-0002 amendment): the persisted DR3 record now wins
   * outright — including COMMISSIONING. Previously a commissioned bank
   * forced ASCENT even with a valid DR3=COMMISSIONING, so a unit entered
   * ASCENT on the bench immediately after joining and held the 10 s ascent
   * cadence until launch. Flight entry is now EXPLICIT (arming or launch
   * detection) and persisted, so DR3=COMMISSIONING is authoritative.
   * DDR-0018 preserved for the true ambiguity: bank commissioned but DR3
   * wiped (power loss without VBAT) mid-flight -> ASCENT, never
   * commissioning with its join/LED privileges. */
  /* H-01 (#281): the durable manifest is consulted FIRST. Its one-way
   * FLIGHT_STARTED latch survives a full power loss without VBAT, so a
   * commissioned unit power-cycled on the ground (storage, transport,
   * battery service) reboots into COMMISSIONING and the arming/launch door
   * works as designed. DR3 remains the fast reset cache; the session bank
   * door anchor (DDR-0002) is the floor for a manifest-less unit. */
  bool manifest_flight = MissionManifest_IsFlightStarted();
  if (manifest_flight) {
    s_state = MISSION_ASCENT;
  } else if (bkp_valid) {
    s_state = persisted;
  } else {
    /* H-01 (#281): latch clear + no fast cache -> COMMISSIONING, even with a
     * commissioned bank. The durable manifest's ABSENCE of FLIGHT_STARTED
     * means the door never opened; a commissioned unit power-cycled on the
     * ground (bank written, never armed, DR3 wiped with the backup domain)
     * must not land in ASCENT. The airborne-VBAT-loss case stays protected:
     * a real flight committed the latch at the door, so a genuinely airborne
     * unit reaches here only if the manifest page itself is unreadable -
     * which the next boot's fresh COMMISSIONING + launch detector re-opens
     * within one pressure-departure window. */
    s_state = MISSION_COMMISSIONING;
  }
  if (manifest_flight && bkp_valid && persisted != MISSION_ASCENT &&
      persisted != MISSION_FLOAT) {
    /* The durable record says flight but the (faster) cache disagrees -
     * trust the manifest, repair DR3 below via Persist(). */
    SONDE_LOG_STR("MissionState: DR3 stale vs manifest FLIGHT_STARTED - repaired\r\n");
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
    } else if (MissionManifest_LaunchRefHpaX10() > 0U) {
      /* H-01 (#281): DR15 died with the backup domain, but the manifest
       * committed the launch reference at the door - restore it so the
       * FLOAT min-ascent guard survives a full power loss. */
      LaunchDetector_SetRef(&s_launch_det,
                            (float)MissionManifest_LaunchRefHpaX10() / 10.0f, 0);
    } else {
      s_no_launch_ref = true;
      SONDE_LOG_STR("MissionState: ASCENT without launch ref - FLOAT on window guards only\r\n");
    }
  }

  MissionState_Persist();

  SONDE_LOG("MissionState: %s (bank %s, DR3 %s)\r\n",
            s_state == MISSION_COMMISSIONING ? "COMMISSIONING" : s_state == MISSION_ASCENT ? "FLIGHT-ASCENT"
                                                                                           : "FLIGHT-FLOAT",
            bank_commissioned ? "commissioned" : "virgin",
            bkp_valid ? "valid" : "invalid");
}

MissionState_t MissionState_Get(void) {
  return s_state;
}

bool MissionState_IsCommissioning(void) {
  return (s_state == MISSION_COMMISSIONING);
}

void MissionState_EnterFlight(void) {
  /* One-way door: COMMISSIONING -> ASCENT, never back */
  if (s_state == MISSION_COMMISSIONING) {
    s_state = MISSION_ASCENT;
    MissionState_Persist();
    /* H-01 (#281): commit the durable FLIGHT_STARTED latch - the ONE
     * sanctioned post-commissioning internal-flash write, at the door-open
     * instant (operator's hands, ground power). The launch reference is
     * taken from the detector when set (launch-detection path) and 0 for a
     * ground arm (a ground-armed unit has no departure reference; the F1
     * no-ref fallback already tolerates that). DR15 is still written by the
     * caller (launch path) for the fast cache. */
    uint32_t ref_x10 = LaunchDetector_HasRef(&s_launch_det)
                           ? (uint32_t)(LaunchDetector_RefHpa(&s_launch_det) * 10.0f)
                           : 0U;
    if (MissionManifest_CommitFlightStarted(ref_x10, 0U) != MANIFEST_OK) {
      SONDE_LOG_STR("MissionState: WARNING manifest commit failed - flight state not durable\r\n");
    }
    SONDE_LOG_STR("MissionState: COMMISSIONING -> FLIGHT (ASCENT)\r\n");
  }
}

void MissionState_Update(float pressure_hpa, bool pressure_valid, uint32_t now_s) {
  /* SP-06 (#256, P-COMM-013): average the launch-detector input over the last
   * 3 samples so a single gust or data glitch cannot false-arm, yet 3 samples
   * = 60 s at the 20 s commissioning chunk = fast enough for ascent entry. */
  static float window[3];
  static uint8_t wc = 0, wn = 0;
  float avg = pressure_hpa;
  if (pressure_valid) {
    window[wc] = pressure_hpa;
    wc = (uint8_t)((wc + 1U) % 3U);
    if (wn < 3U)
      wn++;
    float acc = 0.0f;
    for (uint8_t i = 0; i < wn; i++)
      acc += window[i];
    avg = acc / (float)wn;
  }
  pressure_hpa = avg;

  /* BR-LIFE-007 / MISSION-01b (#142) + STAB-06 (#153): autonomous launch
   * detection in COMMISSIONING, bounded reference window (mission_logic.c):
   * a high-pressure weather system older than MISSION_LAUNCH_REF_WINDOW_S
   * can no longer serve as the launch baseline. A fast storm-front DROP of
   * 6 hPa inside the window can still false-arm - accepted: it only moves
   * the unit to ASCENT cadence. */
  if (s_state == MISSION_COMMISSIONING) {
    if (LaunchDetector_Update(&s_launch_det, pressure_hpa, pressure_valid, now_s)) {
      /* C-01 (#270, DDR-0018): the one-way flight door is gated on the
       * durable PROVISIONED latch in the Tier-1 credential bank. A
       * virgin or partially provisioned unit that sees a qualifying
       * pressure drop (real launch, drive uphill, storm front) must
       * NOT latch ASCENT: joins are commissioning-only, so it could
       * never join and the mission would be archive-only. The launch
       * detector keeps tracking its reference either way, so the door
       * can still open on a later update once provisioning completes.
       * The persisted-state restore path (Init) is untouched: an
       * already-flying unit still restores ASCENT. */
      if (!MultiRegion_IsProvisioningComplete()) {
        SONDE_LOG_STR("MissionState: LAUNCH detected but NOT provisioned - door stays shut (DDR-0018)\r\n");
        return;
      }
      /* H-11 (#287): a commissioning-time accepted GNSS fix is the third
       * latch beside PROVISIONED - the home-region fallback stays a
       * documented option, never guessed on the pad. */
      if (!MissionState_GnssAccepted()) {
        SONDE_LOG_STR("MissionState: LAUNCH detected but GNSS not yet accepted - door stays shut (H-11 #287)\r\n");
        return;
      }
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

uint8_t MissionState_GetStatusBits(void) {
  return ((uint8_t)s_state) & 0x03;
}
