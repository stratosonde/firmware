# Stratosonde System Operational Assumptions

**Status:** Draft baseline assumptions elicited 2026-08-12  
**Purpose:** Document environmental and operational facts the requirements are designed around.

These are **assumptions**, not requirements. If an assumption changes, affected DDRs and requirements must be re-evaluated.

---

## A-001 — LoRaWAN coverage is intermittent
Poor or nonexistent terrestrial LoRaWAN coverage is normal, especially over oceans and remote regions.

Consequences:
- science cannot depend on current connectivity;
- local circular archive is required;
- no-link operation remains normal;
- reconnection must recover backlog without starving fresh science.

## A-002 — Long no-coverage periods are normal
Outages may last days or longer. No short communication timeout may end the mission.

## A-003 — GNSS sky visibility is usually good
The balloon generally has strong sky visibility, so GNSS availability is expected to be good under normal receiver/environment conditions.

## A-004 — GNSS can be unavailable or untrustworthy
Possible causes include receiver/antenna fault, power denial, interference, spoofing, or regional anomalies.

Consequences:
- GNSS loss is fail-soft;
- stale last-known position may remain scientifically useful;
- regulatory RF authorization eventually expires without fresh position;
- fresh valid fix restores normal RF eligibility.

The first-flight product need not implement a generalized spoof detector unless separately specified.

## A-005 — Solar generation is highly variable
Solar input varies with:
- day/night;
- latitude/season;
- payload orientation;
- spinning/tumbling;
- shading;
- solar-cell temperature;
- atmosphere/cloud conditions where relevant.

Fixed orientation cannot be assumed.

## A-006 — Extreme long nights are possible
High-latitude operation may include very long darkness or polar-night-like conditions.

Consequences:
- adapt before depletion;
- cadence may slow;
- GNSS/radio/optional work may be shed;
- critical energy may return immediately to sleep.

## A-007 — Extreme cold is a primary mission stressor
Cold affects battery internal resistance, voltage droop, usable energy, sensors, GNSS/radio load capability, timing, and startup margin.

Power policy must be based on measured cold behavior.

## A-008 — Power availability is mission-limiting
Cold-night power is expected to be a major constraint on cadence.

GNSS is expected to be a major load, but this remains a measurement hypothesis.

Ordinary short radio transmissions may be less limiting than GNSS except during backlog bursts; this too must be measured.

## A-009 — Brownout can occur but is not an operating strategy
Unexpected brownout remains physically possible. The product is designed to avoid it through admission control and cadence adaptation.

## A-010 — Hardware is effectively unrecoverable after launch
The sonde may circumnavigate and is likely eventually lost, often to water.

Hardware preservation therefore has little independent mission value.

## A-011 — The mission is open-ended
There is no planned software mission completion date or normal consumable-based endpoint.

## A-012 — Backend availability is not required for autonomy
Firmware must remain autonomous with locally provisioned configuration, identity, credentials, and mission logic.

## A-013 — Backend compute/storage is much less constrained than LoRaWAN
The backend can retain raw data, calibration history, device history, mission metadata, and richer scientific processing.

This supports preserving raw observables and performing long-term interpretation on the ground when local computation is unnecessary.

## A-014 — First-flight hardware is one known revision
Use sensible modular boundaries but no speculative multi-revision autodetection.

## A-015 — Physical bench access exists before launch
Pre-flight engineering can use ST-Link/SWD, Otii Ace, programmable supplies, cold chamber, and normal lab equipment.

## A-016 — Post-launch firmware replacement is unavailable
Executable firmware is effectively fixed after launch. Future remote adaptation is configuration-only.

---

## Assumption Review Rule

If a future design changes an assumption—for example satellite communication, heaters, OTA firmware, different battery chemistry, or active orientation—the affected DDRs and derived requirements SHALL be reviewed.
