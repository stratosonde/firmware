/**
 ******************************************************************************
 * @file    mission_manifest.h
 * @brief   Durable mission-lifecycle record in internal flash (H-01, #281)
 ******************************************************************************
 * H-01 (#281) + A-003 (#78): the mission lifecycle state is the ONE fact that
 * must never be lost, and it lived only in RTC backup registers (DR3/DR15)
 * which die on full power loss without a trusted VBAT rail. The manifest is a
 * CRC'd, generation-tagged record in internal flash page 127 (two slots in
 * the 2 KB page) so a commissioned unit that is power-cycled on the ground
 * (storage, transport, battery service) reboots into COMMISSIONING and the
 * arming/launch door works as designed - instead of landing in ASCENT with
 * the PROVISIONED and GNSS gates bypassed.
 *
 * Write policy (maintainer decision 2026-08-22): the manifest is written at
 * ONE sanctioned moment - the flight-door open (arm gesture or launch
 * detection, post PROVISIONED + GNSS gates), when the unit is on the ground
 * in the operator's hands. It is never written in flight. This is the only
 * sanctioned post-commissioning internal-flash write; Tier-1's
 * "no writes in FLIGHT" policy holds everywhere else.
 *
 * RTC backup registers remain the fast reset cache (standby/wake is
 * power-free); the manifest is the durable truth. Boot decision order:
 *   1. manifest.FLIGHT_STARTED  -> ASCENT (durable, matches DR3-wipe case)
 *   2. DR3 valid                -> persisted state (fast path, cross-checked)
 *   3. session bank commissioned -> ASCENT (DDR-0002 ambiguity floor)
 *   4. else                      -> COMMISSIONING
 ******************************************************************************
 */

#ifndef MISSION_MANIFEST_H
#define MISSION_MANIFEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Page 127 (0x0803F800): previously "legacy multi-region (retired)"; the
 * F-016 NVM-slot-B claim was never wired (LmHandlerNvmDataStore is dead
 * code). Reclaimed for the manifest - correct the stale map comments. */
#define MANIFEST_FLASH_ADDRESS 0x0803F800UL /* page 127, 2 KB */
#define MANIFEST_FLASH_PAGE_SIZE 2048U

/* Two slots inside the 2 KB page (ping-pong). */
#define MANIFEST_SLOT_A_OFFSET 0x000U
#define MANIFEST_SLOT_B_OFFSET 0x400U /* 1 KB */

#define MANIFEST_MAGIC 0x4D414E49UL /* 'MANI' */
#define MANIFEST_VERSION 1U

/* FLIGHT_STARTED is a one-way latch (bit 0). Once set it is never cleared by
 * firmware - flight entry is one-way (DDR-0002). */
#define MANIFEST_FLAG_FLIGHT_STARTED 0x1U

typedef struct __attribute__((aligned(8))) {
  uint32_t magic;          /* MANIFEST_MAGIC */
  uint32_t version;        /* MANIFEST_VERSION */
  uint32_t generation;     /* monotonic per successful write; newest wins */
  uint32_t flags;          /* bit0 = FLIGHT_STARTED */
  uint32_t launch_ref_x10; /* launch reference pressure hPa x10 (0 = unset) */
  uint32_t launch_epoch_s; /* epoch seconds at flight entry (0 = unset) */
  uint32_t reserved[4];    /* future; 0 */
  uint32_t crc32;          /* over all preceding bytes (CRC-32/ISO-HDLC) */
} MissionManifestRecord_t;

_Static_assert(sizeof(MissionManifestRecord_t) % 8U == 0U,
               "manifest record must be 8-byte aligned for FLASH_IF_Write");
_Static_assert(sizeof(MissionManifestRecord_t) <= 1024U,
               "manifest record must fit one manifest slot");

typedef enum {
  MANIFEST_OK = 0,
  MANIFEST_EMPTY, /* neither slot valid - fresh unit / page never written */
  MANIFEST_ERROR_PARAM,
  MANIFEST_ERROR_FLASH
} MissionManifest_StatusTypeDef;

/**
 * @brief  Load the newest valid manifest record.
 * @param  rec: output record (zeroed on EMPTY)
 * @retval MANIFEST_EMPTY when no valid slot exists (fresh unit).
 */
MissionManifest_StatusTypeDef MissionManifest_Load(MissionManifestRecord_t *rec);

/** @brief  Is the one-way FLIGHT_STARTED latch set in the durable record? */
bool MissionManifest_IsFlightStarted(void);

/**
 * @brief  Commit the one-way FLIGHT_STARTED latch (the single sanctioned
 *         post-commissioning flash write, at the door-open instant).
 * @param  launch_ref_hpa_x10: launch reference pressure x10 (0 = unknown)
 * @param  launch_epoch_s: epoch seconds at flight entry (0 = unknown)
 * @note   Idempotent: a second call re-writes the same latch (new
 *         generation) - safe on retry after a torn write.
 */
MissionManifest_StatusTypeDef MissionManifest_CommitFlightStarted(uint32_t launch_ref_hpa_x10,
                                                                  uint32_t launch_epoch_s);

/** @brief  Launch reference accessor (0 when not yet committed). */
uint32_t MissionManifest_LaunchRefHpaX10(void);

/* --- pure rules (host-testable, no FLASH_IF) ----------------------------- */

/** @brief  Is a raw slot read plausible (magic + version + CRC)? */
bool MissionManifest_RecordPlausible(const MissionManifestRecord_t *rec);

/** @brief  CRC-32/ISO-HDLC over the record with crc32 field zeroed (host-test
 *         helper: builds valid records for the generation/torn scenarios). */
uint32_t MissionManifest_ComputeCrc(const MissionManifestRecord_t *rec);

/** @brief  Wrap-aware: is candidate generation strictly newer? */
bool MissionManifest_GenerationNewer(uint32_t candidate, uint32_t incumbent);

/** @brief  Ping-pong slot for the next write: 0 = A, 1 = B. */
uint8_t MissionManifest_SlotForStore(uint32_t current_generation);

#ifdef __cplusplus
}
#endif

#endif /* MISSION_MANIFEST_H */
