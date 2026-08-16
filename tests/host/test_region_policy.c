/**
  ******************************************************************************
  * @file    test_region_policy.c
  * @brief   Module contract suite for Core/Src/region_policy.c (refactor stage 3)
  ******************************************************************************
  * CONTRACT suite, not a findings archive. Proves the geofence/region gate
  * against INJECTED RegionId values (including REGION_RESTRICTED = 15), so
  * policy correctness is established independently of the production h3lite
  * dataset - SP-02 (#257, closed 2026-08-15): the production table now
  * contains 53 REGION_RESTRICTED cells (h3lite 5480859, firmware 9fdcccc),
  * pinned by the geo suite's hard non-empty gate and Pyongyang/Sanaa
  * resolve probes; this suite catches policy-side defects regardless.
  *
  * Covered: restricted hit -> silence + VETO_RESTRICTED_REGION; open-ocean
  * (REGION_UNKNOWN) -> no silence; unjoined detected region -> VETO_RF_SILENCE;
  * region change while joined -> switch, no silence; stale position ->
  * switch inhibited; invalid coordinates -> GEO_PERMISSION_UNKNOWN; every
  * RegionId value x every boolean input (totality); Silence first-veto-wins.
  *
  * Antimeridian/poles/NaN coordinates are exercised in the coordinate/H3
  * boundary suite (test_geo_20260814.c), where those values are consumed.
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "region_policy.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

int main(void)
{
    /* ---- GeoPermission mapping (DR-02 tri-state) ---- */
    CHECK(RegionPolicy_GeoPermission(false, REGION_RESTRICTED) == GEO_PERMISSION_UNKNOWN); /* invalid coords never "permit" */
    CHECK(RegionPolicy_GeoPermission(false, REGION_UNKNOWN) == GEO_PERMISSION_UNKNOWN);
    CHECK(RegionPolicy_GeoPermission(false, 7U) == GEO_PERMISSION_UNKNOWN);
    CHECK(RegionPolicy_GeoPermission(true, REGION_RESTRICTED) == GEO_PERMISSION_RESTRICTED);
    CHECK(RegionPolicy_GeoPermission(true, REGION_UNKNOWN) == GEO_PERMISSION_UNKNOWN);    /* BEH-04: truthful verdict; the caller's explicit policy transmits over open ocean */
    CHECK(RegionPolicy_GeoPermission(true, 3U) == GEO_PERMISSION_PERMITTED);               /* any mapped region */

    /* ---- BEH-04 (#302): UNKNOWN is a distinct, truthful verdict ----
     * Valid coordinates over an unmapped/ocean cell are GEO_PERMISSION_UNKNOWN,
     * never silently PERMITTED. The open-ocean transmit decision is the
     * CALLER's explicit policy (F-006/GEO-03 disposition), not an inherited
     * "permitted" from the pure verdict. */
    CHECK(RegionPolicy_GeoPermission(true, REGION_UNKNOWN) == GEO_PERMISSION_UNKNOWN);
    /* ...while PERMITTED stays reserved for a known, mapped, non-restricted
     * region - the full split is RESTRICTED / UNKNOWN / PERMITTED. */
    CHECK(RegionPolicy_GeoPermission(true, 14U) == GEO_PERMISSION_PERMITTED);

    /* ---- restricted hit -> silence with VETO_RESTRICTED_REGION ---- */
    RegionDecision_t d = RegionPolicy_Decide(REGION_RESTRICTED, false, true, false);
    CHECK(d.silence_restricted == true);
    CHECK(d.silence_unjoined == false);
    CHECK(d.switch_allowed == true);          /* restricted does NOT inhibit the switch logic */

    /* restricted + unjoined-detected + not stale: BOTH silence causes, restricted wins the veto */
    d = RegionPolicy_Decide(REGION_RESTRICTED, true, false, false);
    CHECK(d.silence_restricted == true && d.silence_unjoined == true);

    /* ---- open-ocean UNKNOWN -> default: no silence, switch allowed ---- */
    d = RegionPolicy_Decide(REGION_UNKNOWN, false, false, false);
    CHECK(d.silence_restricted == false && d.silence_unjoined == false && d.switch_allowed == true);

    /* ---- unjoined detected region (FR-03 fail-closed) ---- */
    d = RegionPolicy_Decide(4U, true, false, false);      /* mapped region, differs, not joined */
    CHECK(d.silence_unjoined == true);
    /* same region (no difference) -> no silence even if not joined */
    d = RegionPolicy_Decide(4U, false, false, false);
    CHECK(d.silence_unjoined == false);

    /* ---- region change while joined -> switch, no silence ---- */
    d = RegionPolicy_Decide(4U, true, true, false);
    CHECK(d.silence_unjoined == false && d.switch_allowed == true);

    /* ---- stale position: switch inhibited (never switch on stale, F-3/#178) ---- */
    d = RegionPolicy_Decide(4U, true, true, true);
    CHECK(d.switch_allowed == false);
    CHECK(d.silence_unjoined == false);       /* joined: no silence cause */
    d = RegionPolicy_Decide(4U, true, false, true);
    CHECK(d.switch_allowed == false);
    CHECK(d.silence_unjoined == true);        /* computed, but the adapter's stale branch takes precedence */
    /* stale + restricted still silences (inhibit-only is about SWITCHING, not silence) */
    d = RegionPolicy_Decide(REGION_RESTRICTED, false, true, true);
    CHECK(d.silence_restricted == true && d.switch_allowed == false);

    /* ---- totality: every RegionId x every boolean triple is decided ---- */
    for (int id = 0; id < 256; id++) {
        for (int diff = 0; diff < 2; diff++) {
            for (int joined = 0; joined < 2; joined++) {
                for (int stale = 0; stale < 2; stale++) {
                    d = RegionPolicy_Decide((RegionId)id, diff != 0, joined != 0, stale != 0);
                    CHECK(d.silence_restricted == (id == REGION_RESTRICTED));
                    if (d.silence_unjoined != (diff && !joined)) { g_checks++; g_failures++; printf("FAIL totality %d %d%d%d\n", id, diff, joined, stale); }
                    if (d.switch_allowed != !stale) { g_checks++; g_failures++; printf("FAIL totality-stale %d\n", id); }
                }
            }
        }
    }

    /* ---- Silence: sets rf_silence, first veto wins (DR-06) ---- */
    TransmitPlan_t plan;
    bool rf_silence = false;
    memset(&plan, 0, sizeof(plan));
    plan.veto = VETO_NONE;
    RegionPolicy_Silence(&plan, &rf_silence, VETO_RESTRICTED_REGION);
    CHECK(rf_silence == true);
    CHECK(plan.veto == VETO_RESTRICTED_REGION);
    RegionPolicy_Silence(&plan, &rf_silence, VETO_RF_SILENCE);   /* second cause loses */
    CHECK(plan.veto == VETO_RESTRICTED_REGION);

    /* composed veto for the FR-03 case: unjoined detected -> VETO_RF_SILENCE */
    memset(&plan, 0, sizeof(plan));
    plan.veto = VETO_NONE;
    rf_silence = false;
    d = RegionPolicy_Decide(4U, true, false, false);
    if (d.silence_unjoined) RegionPolicy_Silence(&plan, &rf_silence, VETO_RF_SILENCE);
    CHECK(rf_silence == true && plan.veto == VETO_RF_SILENCE);

    /* ---- BEH-03 (#301): fail closed when a required switch does not
     * complete. RF is allowed after a switch attempt only when the active
     * region equals the detected region. Busy, failed, rolled-back and
     * silently-stayed outcomes must silence the wake. (The rest of the
     * handoff list - same region, successful switch, target unjoined,
     * stale never switches, restricted always silences - is pinned by the
     * RegionPolicy_Decide matrix above.) ---- */
    /* same region: no switch required -> allowed regardless of post state */
    CHECK(RegionPolicy_PostSwitchRfAllowed(false, true));
    CHECK(RegionPolicy_PostSwitchRfAllowed(false, false));
    /* successful switch: required and settled -> allowed */
    CHECK(RegionPolicy_PostSwitchRfAllowed(true, true));
    /* MAC busy (deferred): required but active != detected -> silence */
    CHECK(!RegionPolicy_PostSwitchRfAllowed(true, false));
    /* switch error + rollback SUCCESS: old session recovered, but that is
     * not authorization to use it at the new location -> silence */
    CHECK(!RegionPolicy_PostSwitchRfAllowed(true, false));
    /* switch error + rollback FAILURE: sessionless and degraded -> silence */
    CHECK(!RegionPolicy_PostSwitchRfAllowed(true, false));
    /* composed veto: an unsettled switch archives as VETO_RF_SILENCE */
    memset(&plan, 0, sizeof(plan));
    plan.veto = VETO_NONE;
    rf_silence = false;
    if (!RegionPolicy_PostSwitchRfAllowed(true, false)) {
        RegionPolicy_Silence(&plan, &rf_silence, VETO_RF_SILENCE);
    }
    CHECK(rf_silence == true && plan.veto == VETO_RF_SILENCE);

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures != 0;
}
