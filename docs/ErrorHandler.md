# Error Handling

## Overview

There is no centralized Error Handler module — no `ErrorRecord_t`, no error
log buffer, no severity enum, no `Error_Report` API. An earlier version of
this document described all of those; none exist in the tree. Error handling
in this firmware is **distributed and deliberate**, built from five real
mechanisms:

## 1. Fatal vs Recoverable (F-001)

Every fault path is classified at the site: recoverable faults degrade and
continue; fatal faults (no timebase, no supervision) record a fault code and
reset. `main.c` carries the fault-code table (`FAULT_CODE_*`, F6/#171 —
IWDG-init failure, RTC-init failure, RTC stalled on LSI post-failover
F4/#170). There is no "safe mode" beyond this: safe is a degraded but honest
mission, or a clean reset.

## 2. Reset Cause and Fault Persistence (reset_cause.h)

`ResetCause_CaptureBoot` classifies each reset (watchdog, brownout, software,
pin) at boot; fault codes persist across the reset in a backup register
(magic 0xF17B) with a boot-attempt counter. Post-mortem analysis reads the
backup domain — there is no RAM error log to lose.

## 3. Supervision (DDR-0020)

- **IWDG** for hard hangs.
- **Deadman** for liveness: no productive work cycle within
  `ConfigGetDeadmanTimeoutS` (max(3 h, 3 × survival cadence), S-04/#228) →
  reset. Deliberately **exempt in COMMISSIONING** — a bench unit sitting idle
  is not a failed mission.

## 4. Capability Degrade (sys_caps.h, STAB-02/#149, #104)

FLASH / GNSS / SENSORS / RADIO degrade independently. A failed subsystem's
dependents degrade; unrelated capability continues (SI-013). There is no
probation state, no reset-count escalation, no permanent give-up: recovery
succeeds → normal immediately.

## 5. Honest Degradation in Data (DDR-0003)

Failures surface in the science itself: stale bits on every sensor channel
(R28/#36, #136), veto reasons on every non-go cycle (DR-06/#241), bounded
retries at every driver. **Omission is preferable to fabrication** (SI-004) —
a reset may create a gap, never a fake completed observation.

## What Replaces the Fictional Features

| Old doc claim | Reality |
|---|---|
| Central error log in RAM + flash backup | Fault codes in backup registers (survive reset); RTT on bench |
| Severity levels (Fatal/Critical/Error/Warning/Info) | Binary: recoverable (degrade) or fatal (fault code + reset) |
| Error telemetry packets | Stale bits + veto reasons ride the normal science records |
| ABP fallback on join failure | OTAA ground-only joins (SI-015); flight never rejoins |
| "Try alternative regions" on TX failure | Region switches are position-driven only, transactional with rollback (#272) |
| Limp mode | Capability degrade (sys_caps) + honest stale data |
| Exponential backoff retries | Bounded retries, then next eligible wake (SI-013) |

## Philosophy (SI-013, SI-020)

Recovery is **aggressive, bounded, and forgetful**: attempt normal operation,
exercise deterministic recovery actions, bound them by the wake budget,
degrade only dependent capability, sleep normally, retry next wake. No
stateful escalation machinery is added merely because it is possible —
complexity must protect an explicit invariant or a measured failure mode.

## Cross-References

- `docs/SystemModule.md` — boot order, supervision, capability ledger
- `Core/Inc/reset_cause.h`, `Core/Inc/sys_caps.h`, `Core/Inc/backup_regs.h`
- DDR-0009 (fault policy), DDR-0020 (supervision)