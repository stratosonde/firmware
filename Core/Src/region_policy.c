/**
  ******************************************************************************
  * @file    region_policy.c
  * @brief   Pure geofence / region-selection policy (refactor stage 3)
  ******************************************************************************
  * Rules extracted condition-for-condition from lora_app.c. No I/O, no
  * globals, no time reads: same inputs -> same outputs.
  ******************************************************************************
  */

#include "region_policy.h"

GeoPermission_t RegionPolicy_GeoPermission(bool coordinates_valid, RegionId h3_region_id)
{
  if (!coordinates_valid) {
    return GEO_PERMISSION_UNKNOWN;
  }
  return (h3_region_id == REGION_RESTRICTED)
         ? GEO_PERMISSION_RESTRICTED : GEO_PERMISSION_PERMITTED;
}

RegionDecision_t RegionPolicy_Decide(RegionId h3_region_id,
                                     bool region_differs,
                                     bool detected_joined,
                                     bool gnss_stale)
{
  RegionDecision_t d;
  d.silence_restricted = (h3_region_id == REGION_RESTRICTED);
  d.silence_unjoined = region_differs && !detected_joined;
  d.switch_allowed = !gnss_stale;
  return d;
}

void RegionPolicy_Silence(TransmitPlan_t *plan, bool *rf_silence, TransmitVeto_t why)
{
  *rf_silence = true;
  if (plan->veto == VETO_NONE) {
    plan->veto = why;
  }
}
