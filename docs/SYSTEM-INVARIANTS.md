# Stratosonde System Invariants

**Status:** Cross-cutting intent contract — merged 2026-08-13
**Authority:** **Explanatory summary, not a rival source of truth.** Each
invariant names its owning DDR in `decisions/`; if this file and a DDR disagree,
the DDR wins and this file is the defect.

**Purpose:** give a reviewer, new contributor, or implementation agent the whole
product contract on one page, above individual DDR mechanics.

---

## SI-001 — There is one mission

A physical sonde has one long-duration mission. Reset, watchdog recovery,
brownout, RF loss, GNSS loss, sensor failure, or backend outage SHALL NOT create
a new mission.

There is no ordinary `MISSION_COMPLETE`, `LANDED`, or recovery state. The mission
ends observationally when hardware no longer wakes or is physically powered down.

*Owned by:* DDR-0002 (`INV-LIFE-008`), DDR-0010 (`INV-PERSIST-012`), DDR-0022.

## SI-002 — Lifecycle transitions are one-way

Runtime lifecycle is conceptually:

`PRE-COMMISSIONED → COMMISSIONED_PRE-FLIGHT → ASCENT → FLOAT`

Provisioning completion is one-way. Pre-flight → ascent is one-way. Ascent →
float is one-way. Float never returns to ascent. No software landing state
exists.

*Owned by:* DDR-0002 (`INV-LIFE-004`, `INV-LIFE-009/010`, `INV-LIFE-011`),
DDR-0018 (`INV-COMM-007`).

## SI-003 — Privacy holds until mission start

A commissioned pre-flight unit may acquire GNSS and transmit health/science
validation data, but precise horizontal X/Y is withheld.

The operator may deliberately start the mission at the launch site. If the
operator forgets, pressure-based launch detection begins ascent automatically.

*Owned by:* DDR-0002 (`INV-LIFE-002`), DDR-0003 (`INV-GNSS-007`),
DDR-0018 (`INV-COMM-006`).

## SI-004 — Reset is recovery, not continuation of a half-finished transaction

After reset, restore durable state and begin a clean wake.

A reset may create a gap. It SHALL NOT cause partially acquired science to be
represented as a completed observation.

**Omission is preferable to fabrication.**

*Owned by:* DDR-0010 (`INV-PERSIST-001`, `INV-PERSIST-011`), DDR-0012.

## SI-005 — Persistent state is classified by consequence

State must survive reset when losing it could materially break LoRaWAN
continuity, lifecycle, science provenance, calibration, long-term energy trend,
archive append continuity, durable configuration, or regulatory RF behavior.

Pure diagnostics may be lost. The exact transient next-wake timer may also be
lost when the consequence is at most one shifted observation.

*Owned by:* DDR-0010 (`INV-PERSIST-003`, `INV-PERSIST-008`, `INV-PERSIST-010`).

## SI-006 — Science is a coherent observation package

Each logical science record binds time, position plus freshness, sensor values
plus freshness, and required interpretation context.

A sensor timeout does not create a structurally partial record. After bounded
recovery, last-known-good may be substituted with stale status.

*Owned by:* DDR-0001 (`INV-WAKE-002`, `INV-WAKE-006`), DDR-0004, DDR-0009,
DDR-0023.

## SI-007 — Archived science is immutable

Once a full-resolution science record is committed, its scientific contents and
logical record ID never change. Retransmission does not create a new ID.

*Owned by:* DDR-0004 (`INV-ARCH-003`, `INV-ARCH-008`).

## SI-008 — Record identity is the gap-repair primitive

Each archived record has a unique logical ID. The backend may request any
retained ID.

The returned record's actual ID is authoritative and distinguishes exact
response, duplicate, or substitute. Linear lookup is acceptable.

*Owned by:* DDR-0004 (`INV-ARCH-007`, `BR-ARCH-017..019`), DDR-0005.

## SI-009 — Archive is a rolling history

The archive is circular. New observations win over old ones. Oldest records may
be erased even if never delivered. The beginning of the mission is not preserved
forever.

No corrupt record may wedge traversal or make later valid records unreachable.

*Owned by:* DDR-0004, DDR-0011 (`INV-STORE-011`).

## SI-010 — Storage corruption is local

A bad/torn science record is skipped when encountered. Firmware does not repair
ordinary science records in place. Validation may be lazy on read rather than a
mandatory whole-archive boot scan.

*Owned by:* DDR-0011 (`INV-STORE-011`, `BR-STORE-001..003`), DDR-0012
(`INV-BOOT-010`).

## SI-011 — Energy policy has final authority

Long-term energy sustainability may lengthen cadence.

For first flight, each scheduled wake makes one early admission decision:
**FULL CYCLE** or **SLEEP**.

Energy policy does not intentionally create a half-science mode. Subsystem
failures during an admitted cycle may still produce honest stale values.

*Owned by:* DDR-0016 (`INV-PWR-001`, `INV-PWR-022`), DDR-0001 (`INV-WAKE-012`).

## SI-012 — Cadence is start-to-start

Configured/effective observation interval is measured from scheduled observation
start to scheduled observation start, not "finish, then sleep N minutes."

Opportunistic archive work yields to live science epochs.

*Owned by:* DDR-0001 (`INV-WAKE-012`, `BR-WAKE-017`), DDR-0005 (`INV-TX-001`).

## SI-013 — Recovery is aggressive, bounded, and forgetful

For a failed sensor, GNSS receiver, radio, bus, or stack:

1. attempt normal operation;
2. exercise deterministic recovery actions;
3. keep them bounded by wake/supervision budget;
4. degrade only dependent capability if still broken;
5. sleep normally;
6. retry next eligible wake.

When recovery succeeds, return to normal immediately. No probation state, no
reset-count escalation, no permanent software give-up state.

*Owned by:* DDR-0009 (`INV-FAIL-016`, `BR-FAIL-016`), DDR-0020.

## SI-014 — GNSS failure has a regulatory tail

Temporary GNSS loss uses last valid position honestly stale.

After more than **24 hours** without a quality-valid horizontal fix, RF becomes
silent because region legality can no longer be trusted. Science, logging, RTC,
and GNSS attempts continue.

One later valid fix immediately restores normal region selection and possible RF.

Loss of fresh GNSS *time* alone never silences RF.

*Owned by:* DDR-0015 (`BR-STALE-017..020`), DDR-0013 (`INV-TIME-009`).

## SI-015 — LoRaWAN joins are ground-only

All required sessions are established during commissioning. Flight firmware never
autonomously joins/rejoins because of coverage loss, no ACKs, radio reset, stack
reset, region transition, or damaged session state.

*Owned by:* DDR-0018 (`INV-COMM-001`).

## SI-016 — Connectivity probe gates opportunistic backlog

The compact confirmed uplink proves a useful radio opportunity. No ACK ends RF
work for that wake. No same-wake probe retry and no ordinary backlog dump without
the required probe success.

*Owned by:* DDR-0019 (`INV-RADIO-007`, `BR-RADIO-010`), DDR-0005.

## SI-017 — Explicit repair requests outrank opportunistic backlog

A valid specific-record request has priority over the normal newest-to-oldest
walker.

After one response attempt, normal backfill may resume if budget remains. The
request is ephemeral, not a persistent job queue.

Energy and RF legality still outrank the request.

*Owned by:* DDR-0005 (`INV-TX-010`, `BR-TX-023..025`).

## SI-018 — Backend gaps and duplicates are acceptable; false science is not

Backend must tolerate gaps, duplicates, bounded replay-watermark rollback, and
old/new protocol versions.

It must not be asked to accept knowingly corrupt or fabricated observations as
valid.

*Owned by:* DDR-0010 (`INV-PERSIST-011`), DDR-0005, DDR-0023.

## SI-019 — Deployed protocol versions remain interpretable

A long-lived sonde may remain airborne while backend and firmware advance.

New deployments may use new formats, but backend must retain a way to identify
and decode historical formats still in flight. Protocol evolution never depends
on over-the-air firmware upgrade.

*Owned by:* DDR-0027, DDR-0025.

## SI-020 — Simplicity is a reliability feature

Do not add stateful escalation, elaborate bad-block management, retransmission
queues, dynamic failure-history logic, or heavy boot repair merely because it is
possible.

Add complexity only to protect an explicit invariant or a measured mission
failure mode.

*Owned by:* corpus-wide principle; see DDR-0009, DDR-0011, DDR-0020.

---

## Related documents

- `ARCHITECTURE-OVERVIEW.md` — how these invariants compose into a running system.
- `decisions/` — the normative records.
- `decisions/system-operational-assumptions.md` — the environmental facts these
  invariants are designed around.
- `requirements/` — requirements, traceability, and proof obligations.
