# DDR-0008: Minimal One-Way Mission State Machine

**Status:** Accepted
**Date:** 2026-08-01
**Context:** The device needs exactly one behavioral pivot — ground vs flight — and today has none: join loops, GPS reconfiguration with flash-save, and LED/blocking-delay paths are reachable in every boot context, including mid-air brownout reboots. Explicit steer from the owner: keep it simple, don't over-model.

## Decision

Two states, one optional sub-phase, all transitions one-way and never toward higher power.

- **COMMISSIONING.** Device stays here until *every* configured region has joined and self-checks pass. LEDs used freely for ground feedback. Join allowed. GPS configure + PCAS00 flash-save allowed. Exit is a deliberate ground action (arm/release step at the airfield), never automatic drift.
- **FLIGHT.** Everything after release. LEDs permanently off (wasted power). Join unreachable. GPS reconfigure unreachable — the receiver's persisted config (saved during commissioning) is trusted.
  - Optional sub-phase **ASCENT → FLOAT**, one-way: ascent samples/transmits fast (5–10 s cadence) to capture the climb, then settles to float cadence. Transition by **timer by default** (most robust — cannot be fooled by any sensor), optionally a simple altitude check (above threshold and no longer climbing for a sustained period, with hysteresis).
- **No descent behavior.** A storm, pressure excursion, or sensor fault could fake a descent; acting on it could kill the pack on a false trigger. The device keeps flying its float profile.

### Persistence and the door
State is persisted; on reboot the door-anchoring rule from DDR-0006 applies: the Tier-1 session bank decides, and **ambiguity resolves to FLIGHT** — a mid-air reboot must never land in commissioning.

### Telemetry
Current state rides the status byte (DDR-0007, b6–b7).

## Consequences
- Single build: no `#ifdef FLIGHT_BUILD`; all gating is runtime mission state.
- Gates: all join call sites (incl. the `SendTxData` rejoin fallback), `GNSS_Configure` + PCAS00, all LED paths and flight-path blocking delays.
- COMMISSIONING is exempt from the progress deadman (a human is present to power-cycle).
