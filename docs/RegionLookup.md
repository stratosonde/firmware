# Region Lookup (H3)

## Overview

Region detection maps a GNSS position to a LoRaWAN region using the **in-tree
H3Lite** hexagon index (`h3lite.h`): `latLngToRegion(lat, lon)` → H3Lite
`RegionId` → `LoRaMacRegion_t`. The module is `Core/Src/multiregion_h3.c` +
`Core/Inc/multiregion_h3.h`; session storage per region is the multiregion
context bank (see MultiRegionSupport.md).

## API (actual)

| Function | Purpose |
|----------|---------|
| `MultiRegion_DetectFromGPS_H3(lat, lon)` | Full detect: H3 lookup + nearest-neighbor fallback |
| `MultiRegion_DetectFromH3Region(h3Region, lat, lon)` | Detect from an already-resolved RegionId (#77: resolve once per cycle; lat/lon only for fallback + logs) |
| `H3Region_ToLoRaMacRegion(h3Region)` | Name→enum mapping (exposed for tests) |

The old `Region_*` API in the previous version of this document never existed.

## Region Map

14 name→region entries (`_Static_assert`-pinned against drift). AS923
sub-plans (AS923-1, -1B, -1C, -2, -3, -4) all map to `LORAMAC_REGION_AS923`;
CN470 and EU433 are detectable but have **no session-bank slot**.

**Session bank (MAX_REGION_CONTEXTS = 7):** US915, EU868, AS923, AU915, IN865,
KR920, RU864 (SP-05/#246 added the RU864 slot; H-07/#274 captured RU864 OTAA
evidence). A detected region outside the bank can be detected but not joined.

## Fallbacks and the Geofence

- **Outside all known regions** (ocean): nearest-neighbor ring search selects
  exactly **one** closest region — the previous doc's "transmit to the closest
  region or two" never existed.
- **Restricted** (`REGION_RESTRICTED`): `GeofenceRestricted()` →
  `GEO_PERMISSION_RESTRICTED` → `VETO_RESTRICTED_REGION` RF silence.
- **Unmapped / open-ocean cells** (`REGION_UNKNOWN`) and **invalid
  coordinates**: `GEO_PERMISSION_UNKNOWN` — a distinct verdict, never silently
  PERMITTED (BEH-04/#302). The documented policy on UNKNOWN is "cannot be
  known restricted → hold the latched region and transmit", chosen explicitly
  (F-006/GEO-03 disposition), not inherited from a failed validation or a
  silent permit.
- **Stale position** (F-3/#178, DDR-0015): may **inhibit** (the geofence runs
  on the backup-register last-known position when GNSS is off) but never
  **switch** region.

## Who Calls It

Only the transmit cycle, and only on a fresh, token-present fix
(`GNSS_HasPosition`): detect → compare with the active region → transactional
switch with rollback on failure (LT-02/H-04/H-06, #272). Detection never runs
on fabricated coordinates.

## Cross-References

- `docs/MultiRegionSupport.md` — the session bank, switching, persistence
- `docs/H3LiteIntegration.md` — the H3Lite library binding
- DDR-0006 (region selection), DDR-0015 (staleness + RF legality)