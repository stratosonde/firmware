# GNSS Module

## Overview

The GNSS driver (`Core/Src/atgm336h.c`, API in `Core/Inc/atgm336h.h`) interfaces
with the ATGM336H-5N31 CASIC receiver over UART with a 512-byte DMA circular
buffer. It is a **handle-based, event-driven** driver: the epoch in `lora_app.c`
wakes the module, streams NMEA, parses, and gates quality — there is no blocking
"get position" call (`GNSS_GetPosition` was deleted, F-023/D12 #59).

## Hardware Interface

- **Module**: ATGM336H-5N31 (CASIC protocol, PCAS commands — **not** UBX)
- **UART**: 9600 baud, DMA circular reception (`GNSS_DMA_BUFFER_SIZE` 512)
- **Power**: PB10 power control (active low) + PB5 enable; standby ~15 µA via
  `PCAS12,0` (module keeps its backup domain → hot starts)
- **Sleep-safe pins**: PB6/PB7 (UART) are driven to the SP-09 (#249) sleep-safe
  state whenever the module is off — an AF-HIGH idle leaks through the
  depowered module's ESD clamp

## Configuration (PCAS command bodies, runtime checksums)

Bodies are defined without `$`/checksum; `GNSS_SendCommandBody` computes the
checksum at runtime (R23/#22 — three baked checksums were wrong and silently
rejected; expected checksums are pinned by host tests).

| Body | Effect |
|------|--------|
| `PCAS03,1,0,0,0,1,1,0,0` | Flight NMEA mask: GGA+RMC+VTG, GSV off (R26) |
| `PCAS03,1,0,0,1,1,1,0,0` | Commissioning/bench variant, GSV on |
| `PCAS04,5` | GPS+GLONASS constellations |
| `PCAS11,5` | **Airborne dynamic model** — defeats the 18 km CoCom limit (R24: `PCAS11,2` pedestrian deleted; the setting is single-value last-write-wins, and ,2 re-enabled the limit) |
| `PCAS02,1000` | 1 Hz update rate |
| `PCAS00` | Save configuration to module flash |
| `PCAS12,0` | Standby (~15 µA, permanent until wake char) |

## Fix Quality and the Position Gate

Three distinct predicates, in increasing strictness:

- `GNSS_IsFixValid` — NMEA validity only (can pass on a partial sentence with
  (0,0) coordinates — RMC 'A' with empty lat/lon tokens)
- `GNSS_IsFixGoodQuality` — adds satellite/HDOP/range checks (a real (0,0)
  Null-Island fix legitimately passes, R32)
- `GNSS_HasPosition` — valid fix **and** lat/lon tokens actually present
  (R2-16/#120) **and** range-checked (DR-02/#237). **This is the only gate
  allowed to persist last-known-good position** (`LastPos_Store`) and to
  discipline the RTC (F-1/#176: clock first, then stamp UTC).

`GNSS_Data_t` carries `double` latitude/longitude (R34/#57: 1e-7° container)
plus `position_present` token tracking. Extended per-constellation satellite
lists and 3D/vertical speed feed the bench GNSS detail packet
(GNSSDetailPacket.md).

## Failure and Recovery Behavior

- **Provenance before optimism** (R3-02/#216): the epoch marks GNSS stale and
  clears `hgnss.data` *before* attempting a wake — a failed wake can never
  re-use the previous cycle's fix.
- **One teardown path** (LT-04/#276): `GNSS_TeardownToOff` is the single exit
  for every failed startup — module electrically OFF, STOP mode re-enabled,
  pins in the SP-09 sleep-safe state. Earlier failure paths left the module
  powered (~25–30 mA) or PB6 driven HIGH into a dead module.
- **UART error recovery** (SP-01/#244): the vendored HAL aborts circular DMA on
  any UART error; `GNSS_UART_ErrorCallback` counts the event, drops the torn
  partial sentence, and re-arms reception (`rx_dma_active` distinguishes
  mid-stream kills from teardown).
- **Overrun accounting** (F-011/#25): absolute producer/consumer counters;
  a lapped ring is counted and resynced (`GNSS_GetDmaOverrunCount`).
- **Bounded retries**: recovery is aggressive, bounded, and forgetful
  (SI-013) — a failed acquisition is retried next eligible wake, never
  escalated.

## Staleness and RF Legality (policy lives in `lora_app.c`)

The 24 h stale-position budget is **not** in this driver. `lora_app.c`
silences RF when `(utc_now_s - ref_s) > GPS_LOSS_SILENCE_S` (strict `>`,
`GPS_LOSS_SILENCE_S` = 24 h, DDR-0015 BR-STALE-017, knob in `lora_app.h`)
while GNSS retries continue (STAB-03/#150). Known open conformance gaps:
BR-STALE-019 same-wake restore (#285) and the diagnostic-only position
predicate (#286) — see the readiness checklist. A fresh-fix acceptance
predicate is #284's deliverable.

## Error Health Metrics

`GNSS_GetDmaOverrunCount` (F-011/#25) and `GNSS_GetUartErrorCount` (SP-01/#244)
are cumulative health counters surfaced for diagnostics.

## Cross-References

- `docs/GNSSDetailPacket.md` — bench satellite-detail wire format (Port 3)
- `docs/PayloadFormats.md` — where position lands in science records
- DDR-0003 (freshness/honesty), DDR-0015 (stale-position RF legality),
  DDR-0020 (fault policy; GNSS On/Off switch descoped)