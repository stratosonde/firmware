# Multi-Region LoRaWAN Support (As-Built)

## Overview

The sonde pre-joins every supported LoRaWAN region **on the ground** and
switches between banked sessions in flight based on the GNSS position — no
in-flight joins, ever (SI-015, DDR-0018 INV-COMM-001). Implementation:
`Core/Src/multiregion_context.c` (bank + switching + persistence),
`Core/Src/multiregion_h3.c` (H3 detection, see RegionLookup.md).

> This document describes the **as-built** system. It replaces a December 2025
> design proposal that described 4 regions, a 75-byte context, and a
> single-page store — the shipped system differs in every one of those
> numbers.

## The Session Bank

- **7 slots** (`MAX_REGION_CONTEXTS`): US915, EU868, AS923, AU915, IN865,
  KR920, RU864 (SP-05/#246 added RU864; H-07/#274 captured its OTAA evidence)
- **84-byte `MinimalRegionContext_t`** per slot: region, **per-region unique
  DevEUI (8 B)**, activation, DevAddr, AppSKey/NwkSKey, up/down frame
  counters, LastRxMic, datarate/tx_power/ADR, RX2 params, LRU timestamp, CRC16
- Bank header: magic `0xDEADBEEF`, active slot, valid count, version, CRC32
- **Bank version 5** (`MULTIREGION_VERSION`): v2 two-tier storage (FW-1/
  DDR-0018) · v3 Tier-1 monotonic generation (R8/#190) · v4 RU864 slot
  (SP-05/#246) · v5 durable PROVISIONED latch (C-01/#270). A version mismatch
  reads as virgin and the unit **re-commissions on the bench** — acceptable
  pre-first-flight; there is no in-field migration path.

## Persistence: Two Tiers (FW-1/DDR-0018)

| Tier | Content | Where | Write policy |
|------|---------|-------|--------------|
| **Tier-1** | Immutable per-region credentials (DevEUI/DevAddr/session keys) + PROVISIONED latch | Internal flash pages 120-122, **three CRC'd copies** | Written once at commissioning; **never erased in flight** |
| **Tier-2** | Dynamic frame counters only | Pages 123-124, **ping-pong slots** with erase-before-write | Every `frame_counter_save_interval` (default 10) transmissions — brownout-safe, wear-leveled (~83 h → >800 h endurance) |

Pages 126-127 are the LoRaWAN NVM context + slot B (F-016/#54). The page map
lives in `config.h` and is mirrored in `multiregion_context.c` (FR-21/#102).

Frame-counter save batching means the restore margin must follow the
configured interval — which is why `Config_Init` runs before
`MX_LoRaWAN_Init` (H-02/#273, DR-07/#239 defect class).

## Ground Pre-Join Ceremony

`MultiRegion_PreJoinAllRegions()` — **commissioning-only** (F4/T3, DDR-0002;
blocked in flight):

1. Erase **both** LoRaWAN NVM slots via the owner (FR-11/#94: a page-126-only
   erase left a valid stale slot B that restore would select)
2. Loop the 7-region table (`kPreJoinRegions`, F-R3/#73): OTAA join each with
   a bounded 5-min-per-region wait and 30 s retry cadence (R30)
3. Flight entry requires ≥ 1 successful join (R30/D6)
4. C-01 (#270): on success the durable PROVISIONED latch is written to the
   Tier-1 bank — the "flight door" that a commissioned unit never re-opens

## In-Flight Switching (transactional, #272)

Triggered only by a fresh, token-present fix (`GNSS_HasPosition`); the H3
geofence resolves **once per cycle** (#77) and the decision flows through
`MultiRegion_AutoSwitchToRegion` (policy only, no redundant lookup). A stale
position may inhibit but never switch (F-3/#178, DDR-0015).

1. Target has a joined context? No → stay. MAC busy? → defer.
2. Halt MAC, **force-save** the current context (`MultiRegion_ForceSaveCurrentContext` — bypasses batching)
3. Reconfigure for the target region, inject banked session keys/DevAddr,
   restore frame counters + LastRxMic, mark activated
4. **Verify radio params read back** and **roll back to the previous region
   on any failure** (LT-02/H-04/H-06, #272) — a half-switched stack is worse
   than a wrong-region one
5. Resume MAC; transmission proceeds on the new region

Over ocean (no region within `H3_MAX_DISTANCE_KM`): the nearest-neighbor
fallback selects one closest region; if nothing is in range the current
region is kept — never strand mid-ocean.

## API (actual — `multiregion_context.h`)

| Function | Purpose |
|----------|---------|
| `MultiRegion_Init` | Restore Tier-1/2 banks at boot (called from `MX_LoRaWAN_Init`) |
| `MultiRegion_JoinRegion` | Blocking OTAA join, 5-min bound (ground) |
| `MultiRegion_PreJoinAllRegions` | The commissioning ceremony above |
| `MultiRegion_SwitchToRegion` / `AutoSwitchForLocation` / `AutoSwitchToRegion` | Transactional switch + policy wrappers |
| `MultiRegion_GetActiveRegion` / `RegionToString` | Query + shared name map (#77) |
| `MultiRegion_SaveCurrentContext` / `ForceSaveCurrentContext` / `SaveAllContexts` / `RestoreContexts` | Persistence |
| `MultiRegion_GetStats` / `ClearAllContexts` | Diagnostics / emergency reset |
| `MultiRegion_InitializeRegionFromNetworkServer` / `DisplaySessionKeys` | **Bench ABP helpers** (ChirpStack key paste; flight is OTAA) |

## Testing

- `tests/host/test_multiregion.c` — bank semantics, version reject, H-02
  restore-margin ordering, all-7-regions read-back (C-01)
- The multiregion host gate: 95 checks green at the 2026-08-15 sweep
- ChirpStack multi-region device setup: one device per region DevEUI

## Cross-References

- `docs/RegionLookup.md` — H3 detection + geofence vocabulary
- `docs/H3LiteIntegration.md` — the H3Lite engine binding
- `docs/TransmissionModule.md` — where switching sits in the cycle
- DDR-0006 (region selection), DDR-0015 (staleness), DDR-0018 (sessions)