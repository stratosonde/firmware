# ADR-0006: Session Integrity — Two-Tier Storage and the One-Way Commissioning Door

**Status:** Accepted
**Date:** 2026-08-01
**Context:** LoRaWAN session persistence for an autonomous balloon that cannot be reset after launch. Supersedes the implicit design in `multiregion_context.c` (single `MultiRegionStorage_t` copy in one internal-flash page, keys and frame counters mixed, one CRC).

## Problem with the current design

Keys (never change) and frame counters (change every 10 TX) share **one flash page**. Every counter save erases and rewrites the page holding the immutable credentials — a brownout mid-save can destroy the keys, and there is only one copy to lose. No redundancy, no tier split, no anchor for the commissioning/flight decision.

## Decision

### All joins happen on the ground
LoRaWAN joins execute **only in COMMISSIONING state**, against the per-region gateway bench. The device flies on stored session state only. Join code remains in the single binary but is unreachable in FLIGHT — no error fallback, no corrupted-context recovery path may re-enter it.

### Two tiers, treated differently

- **Tier 1 — immutable credentials** (per region: DevAddr, NwkSKey, AppSKey, region params): written once at commissioning, stored as **three redundant copies, each independently CRC'd**, never erased or rewritten in flight. A corrupted copy is restored from a surviving copy. Commissioning **must read back and CRC-verify every copy after writing** (redundancy only defeats random isolated corruption; a systemic writer bug clones the same error into all three).
- **Tier 2 — mutable state** (frame counters): persisted separately using the existing counter-margin scheme (persist every N frames, restore +margin — verified correct, C6). Counter area corrupt → reload keys from Tier 1, resume counter from last persisted value + margin.

### Degrade ladder (flight, per region)
1. Session restore fails → retry from redundant Tier-1 copies.
2. Keys good, counter bad → reload keys, counter = last persisted + margin.
3. Region's credentials wholly unrecoverable → **RF silence in that region** (compliance: never transmit US915 outside its footprint), keep logging to flash, resume TX on entering a region with a valid session.

### Door anchoring
COMMISSIONING vs FLIGHT is decided by the session bank, not a lone flag: virgin bank (never commissioned) → COMMISSIONING; bank shows commissioning-complete marker (Tier-1 copies present) → FLIGHT, even if any separate state record is corrupt. **Ambiguity resolves to FLIGHT** — a mid-air reboot must never land in commissioning. Bench recovery from a confused state is a manual re-commissioning action.

## Known limitations (accepted)
- Frame-counter reset semantics live partly on the network server. The margin scheme avoids reuse in normal operation; for the unrecoverable-counter case the NS-side expectation is: counter jump beyond margin is accepted; do not rely on NS "reset counters" options.
- The one-way door is only as strong as its anchor; hence anchoring to the Tier-1 bank with fail-to-FLIGHT.

## Consequences
- `MultiRegionStorage_t` is split: Tier-1 bank (3 copies) and Tier-2 counter area become separate flash structures with separate write policies.
- All join call sites (`MultiRegion_JoinRegion`, `MultiRegion_PreJoinAllRegions`, and the `LmHandlerJoin` fallback in `SendTxData`) are gated on mission state (ADR-0008).
- "Transmit US915 over Europe" becomes unreachable: no valid session for the region under the balloon ⇒ silence.
