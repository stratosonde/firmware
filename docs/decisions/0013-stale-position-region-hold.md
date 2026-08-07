# DDR-0013: Stale-Position Region Hold Is Deliberate

**Status:** Accepted (project owner, 2026-08-07 firmware review)
**Date:** 2026-08-07
**Issue:** #64 (WO-0807 F-06)

**Context:** On GPS timeout, `SendTxData` re-issues the last-known position with `valid=true, fix_quality=GNSS_FIX_GPS` (marked stale per DDR-0007). Region selection (`latLngToRegion` / `MultiRegion_AutoSwitchForLocation` in `lora_app.c`) therefore runs on a possibly-hours-old position. Reviews periodically flag this as a bug.

## Decision

**Holding the last known region across a GPS gap is intentional and is kept as-is.**

- A missed fix does not mean the sonde teleported. Snapping from US915 to EU868 because a fix was missed five minutes ago would be the more dangerous action (wrong-region transmission is a regulatory violation and breaks the session bank).
- Going radio-silent on stale position is unacceptable: it risks the entire mission for a transient sensor gap.
- Consistent with DDR-0007's framing: the GPS-stale bit governs **science data honesty** (archive/uplink freshness marking), **not region selection**.

## Consequences

- A comment at the region-selection site in `lora_app.c` states this rationale so future reviews do not re-flag it.
- With F-15 (#72), the last-known position survives warm resets via backup registers, so the held position after a reset is a real past fix rather than the compile-time default.
- If region policy ever changes, this DDR must be revisited first.
