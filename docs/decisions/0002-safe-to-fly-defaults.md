# DDR-0002: Safe-to-Fly Out of the Box

**Status:** Accepted  
**Date:** 2026-07-30  
**Context:** Default build configuration philosophy  

## Decision

The default build must produce a **minimum-airtime, single compact packet, no-debug-traffic** firmware image. Debug features are deliberate opt-in, never default-on.

### Implications
- `ENABLE_DEBUG_LPP` defaults to `0`
- `ENABLE_GNSS_DETAIL_PACKET` defaults to `0`
- Debug paths must have zero side effects on flight radio state (save/restore DR, etc.)
- ADR is OFF to prevent network-driven DR changes on an unreachable device

## Consequences
- Developers must explicitly enable debug packets via compile-time defines
- LPP and GNSS detail packets are only sent when opted in
- No debug code path may mutate the data rate used by production transmissions