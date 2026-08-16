# Transmission Module

> **Authoritative** for transmit-cycle behavior (owner decision 2026-08-16):
> code changes that alter the transmit cycle must update this document in the
> same commit. Wire-format bytes remain authoritative in PayloadFormats.md.

## Overview

Transmission is not a standalone module — it is the **executor half** of the
transmit cycle (`SendTxData` in `LoRaWAN/App/lora_app.c`) driven by a **pure
decide half** (`Core/Src/transmit_plan.c`, R47/#44, host-testable with zero
hardware), plus the multiregion session bank (`Core/Src/multiregion_context.c`).
The decide half takes raw inputs and returns a `TransmitPlan_t`; the executor
does the doing. A skipped or degraded cycle records **why** (veto), not just
that (DDR-0003).

## Radio and Network

- **Radio**: STM32WLE5 integrated LoRa transceiver, LoRaWAN **Class A**
- **Activation**: OTAA only, **ground-only** (SI-015, DDR-0018): all sessions
  are established during commissioning. Flight firmware never autonomously
  joins/rejoins — not for coverage loss, radio/stack reset, region transition,
  or damaged session state. There is no ABP fallback.
- **Credentials**: Tier-1 DevEUI/keys live in internal-flash pages 120-122
  (three copies, FW-1/DDR-0018) — not in any "secure enclave" (the WLE5 has
  none). Per-region session state lives in the Tier-2 multiregion bank.

## The Transmit Cycle

1. **Admission**: read fresh temperature and raw battery voltage. Below the
   configured minimum (`gps_temperature_lockout`, default −55 °C;
   `battery_critical_threshold`, default/hard floor 4300 mV), or on
   stale/invalid input,
   schedule `tx_interval_survival` and return to sleep without GNSS, archive,
   probe, or live telemetry.
2. **Plan** (`DecideTransmitPlan`): power mode (with F8 hysteresis), cadence,
   GNSS timeout, and veto. Every admitted science wake enables GNSS with a
   nonzero acquisition budget; mode cadence selection does not create a
   deliberate GNSS-less science cycle. Mission cadence override (DDR-0002):
   ASCENT 10 s / FLOAT 5 min when the power model is healthy.
3. **Silences** (executor, first veto wins — `TransmitVeto_t`, archived in the
   v6 flash record's dedicated `veto_reason` byte; the record's flags byte
   carries fix-valid b0, satellite count b1-b4, power mode b5-b7):
   | # | Veto | Cause |
   |---|------|-------|
   | 0 | `VETO_NONE` | full go |
   | 1-2 | `VETO_TEMP_STALE` / `VETO_TEMP_LOCKOUT` | **DEPRECATED** (RV-08/#164, DDR-0021) — never produced |
   | 3 | `VETO_RF_SILENCE` | FLIGHT with no valid session (DDR-0018) |
   | 4 | `VETO_RESTRICTED_REGION` | regulatory RF prohibition (geofence) |
   | 5 | `VETO_GPS_LOSS` | position stale beyond the 24 h budget (`> GPS_LOSS_SILENCE_S`, strict, DDR-0015 BR-STALE-017) — science/logging/GNSS retries continue (STAB-03/#150); a same-wake accepted fix clears only this veto, before region selection and TX (H-09/#285, A6/A7) |
   | 6 | `VETO_PRELAUNCH_QUIET` | commissioned-but-not-launched quiet watch (DR-06/#241) |
4. **Probe**: one compact confirmed heartbeat (Port 10). **The ACK gates all
   further RF work this wake** (SI-016, DDR-0019): no ACK → end of RF work.
   No same-wake probe retry, no backlog dump without probe success.
5. **Burst** (only after probe ACK): Port 11 archive packets, newest→oldest
   via the one-pass recovery walker (DDR-0005; `tx_high_water` monotonic up /
   `recovery_frontier` monotonic down — never rewalks). Budget knobs (config,
   §6b): `max_bulk_packets` (20), `bulk_battery_min_mv` (5000),
   `bulk_timeout_ms` (60 s). The burst runs under a hard deadline with no
   timer re-arm (LT-07/#277). Watermarks advance at send time (no commit-on-ACK,
   no autonomous retry) and persist via deferred header sync —
   `FlashLog_DeferHeaderSync()`/`FlashLog_FlushHeaderSync()` batch the header
   write to one flush at burst end (Finding #8); the header commits only after
   a successful program (#135). Backend dedups on
   (DevEUI, sequence); gaps and duplicates are acceptable (SI-018).
6. **Region**: H3 lookup runs only on a fresh, token-present fix
   (`GNSS_HasPosition`); a region switch is transactional — radio params are
   verified and the switch rolls back on failure (LT-02/H-04/H-06, #272).
   Restricted regions inhibit RF; the geofence never auto-switches on a stale
   position. Only a fix passing the configured acceptance predicate
   (satellites, HDOP, fix quality — `GnssAcquire_FixAccepted`) may update the
   trusted last-known position, its freshness epoch, or clear GNSS staleness
   (BEH-02/#284); weaker positions remain in the wake's sample as stale/weak
   provenance, and valid GNSS date/time may discipline the RTC on its own
   validity, never as proof of position quality. A required region switch
   that does not settle with active == detected (busy, failed, rolled back,
   silently stayed) fails closed: the wake archives locally and silences RF
   (BEH-03/#301); a successful rollback recovers the old session but never
   authorizes it at the new location. Unmapped/open-ocean cells resolve to
   `GEO_PERMISSION_UNKNOWN`, a truthful verdict distinct from PERMITTED
   (BEH-04/#302); transmission over open ocean is the explicit documented
   disposition, not a silent permit.

An admitted science wake attempts GNSS and the remaining sensors, marks each
field fresh/stale honestly, and appends the current record when flash is
available (BEH-01/#300: package freshness is record-quality metadata carried
in the v6 `sensor_quality` byte — never an abort gate; one failed channel no
longer discards the other observations or the fact of the failure).
Transmission then proceeds only if the plan veto / RF silence authorize it.
A wake that fails energy admission (temperature or raw battery below floor,
or stale/invalid input) creates no record: the TX FSM parks and retries at
the survival cadence. Cached archive recovery may continue in an already-open
bulk callback path when admission passed.

## Wire Formats (authoritative: PayloadFormats.md)

| Port | Content | Status |
|------|---------|--------|
| 10 | Heartbeat v2, 11 B, little-endian (D9) — compact telemetry + status byte; LinkCheckReq rides in FOpts | current |
| 11 | Core science archive, **v6** (`0x06`): `seq u32 LE + 34 B` records (adds `sensor_quality` + `veto_reason`), n ≤ 6, packed to the runtime payload budget (`LoRaMacQueryTxPossible`) | current; v1-v5 historical decode only |
| 2, 3 | CayenneLPP debug, GNSS detail | compiled out of flight builds (`ENABLE_DEBUG_LPP=0`) |

## Error Handling

- **No ACK**: end RF work for the wake; retry next eligible wake (SI-013 —
  aggressive, bounded, forgetful; no escalation, no probation).
- **Join failure**: ground problem only — sessions are commissioned on the
  ground; flight never rejoins (SI-015).
- **Region switch failure**: transactional rollback to the previous region
  context (#272).
- **Radio/stack faults**: counted and surfaced; dependent capability degrades,
  everything else continues.

## Power Considerations

- Radio is powered down after the receive windows per Class A; the burst
  deadline caps worst-case RF-on time (LT-07).
- The SF7 archive burst is link-gated: LinkCheck margin ≥ `link_margin_threshold`
  (15 dB) and gateway count ≥ `gateway_count_threshold` (2), else the cycle
  falls back to heartbeat-only (host-pinned in `test_burst_fsm.c` T-B4b).
- Energy and RF legality outrank every backlog ambition — including explicit
  backend record requests (SI-017; request servicing not yet implemented).

## Cross-References

- `docs/PayloadFormats.md` — byte-level wire specs + decoder
- `docs/MultiRegionSupport.md` — region bank, switching, rollback
- `Core/Inc/transmit_plan.h` — the decide-half contract
- DDR-0005 (one-pass recovery), DDR-0015 (RF legality), DDR-0018 (joins),
  DDR-0019 (probe gating)
