/**
  ******************************************************************************
  * @file    region_policy.h
  * @brief   Pure geofence / region-selection policy (refactor stage 3)
  ******************************************************************************
  * Decision rules extracted from lora_app.c's GeofenceRestricted /
  * SelectRegionAndSession. Coordinate validation and the H3 lookup itself
  * stay at the adapter boundary (lora_app.c); this module consumes the
  * explicit validated result. No hardware, no file-scope mutable state.
  * The veto vocabulary is the existing TransmitVeto_t (transmit_plan.h) -
  * reused, not duplicated. SP-02 note: policy correctness here is proven
  * against injected RegionId values, independent of the production h3lite
  * dataset. That dataset is populated since 2026-08-15 (h3lite 5480859:
  * 53 REGION_RESTRICTED cells, Yemen + North Korea); the geo suite pins
  * the data side with a hard non-empty gate plus restricted-coordinate
  * resolve probes.
  ******************************************************************************
  */

#ifndef REGION_POLICY_H
#define REGION_POLICY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "transmit_plan.h"   /* TransmitPlan_t, TransmitVeto_t */
#include "h3lite.h"          /* RegionId, REGION_RESTRICTED, REGION_UNKNOWN */

/* DR-02 (#237): tri-state, not boolean. An implausible last-known position is
 * UNKNOWN, not "permitted" - the old `return false` let a validation failure
 * silently read as permission on the one policy with regulatory consequences.
 * Callers map UNKNOWN to the documented "no fix ever -> transmit" policy
 * EXPLICITLY (same outcome, different reasoning - F10/#175, DDR-0015). */
typedef enum {
  GEO_PERMISSION_PERMITTED = 0,
  GEO_PERMISSION_RESTRICTED,
  GEO_PERMISSION_UNKNOWN
} GeoPermission_t;

/** Tri-state geofence verdict from the validated inputs. Pure. */
GeoPermission_t RegionPolicy_GeoPermission(bool coordinates_valid, RegionId h3_region_id);

/** Pure decision output of the region/session gate. */
typedef struct {
  bool silence_restricted;   /* REGION_RESTRICTED hit -> silence, VETO_RESTRICTED_REGION */
  bool silence_unjoined;     /* detected != active AND detected has no session -> silence,
                                VETO_RF_SILENCE (FR-03/#290 fail-closed; only valid when
                                switch_allowed - the caller's stale branch takes precedence) */
  bool switch_allowed;       /* false on a stale position (F-3/#178: inhibit, never switch) */
} RegionDecision_t;

/**
  * @brief  Region/session gate decision. Pure: same inputs -> same outputs.
  * @param  h3_region_id: completed H3 lookup result (adapter's latLngToRegion)
  * @param  region_differs: detected_region != MultiRegion_GetActiveRegion()
  * @param  detected_joined: MultiRegion_IsRegionJoined(detected_region)
  * @param  gnss_stale: EnvSensors_GnssIsStale()
  */
RegionDecision_t RegionPolicy_Decide(RegionId h3_region_id,
                                     bool region_differs,
                                     bool detected_joined,
                                     bool gnss_stale);

/* DR-06 (#241): ONE silence helper for every RF-silence path, so the archive
 * always records WHY a cycle went dark (DDR-0003 §6a), not just THAT. First
 * veto wins (the existing DecideTransmitPlan rule). Previously only the
 * no-session veto reached plan.veto; the restricted-region, GPS-loss and
 * pre-launch quiet-watch paths archived as VETO_NONE - indistinguishable
 * from a normal cycle on recovery.
 * (Moved as-is from lora_app.c's static Silence(); renamed only because the
 * symbol is no longer file-local.) */
void RegionPolicy_Silence(TransmitPlan_t *plan, bool *rf_silence, TransmitVeto_t why);

#ifdef __cplusplus
}
#endif

#endif /* REGION_POLICY_H */
