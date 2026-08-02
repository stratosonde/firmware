# ADR-0007: Data Honesty — Every Reading Carries Its Own Freshness

**Status:** Accepted
**Date:** 2026-08-01
**Context:** Sensor and GPS failures currently produce fabricated data indistinguishable from live data: SHT31 failure substitutes +18 °C / 50 %RH (`sys_sensors.c`), GPS timeout re-issues last-known position marked `valid=true, fix_quality=GNSS_FIX_GPS` (`lora_app.c`), and the uplink status byte was removed (`payload_encode.c:99`). Downstream consumers (the −55 °C GPS cold-lockout, battery-voltage compensation, the science archive) cannot tell real from fabricated.

## Decision

**A reading carries its own honesty.** Fresh vs stale is marked at the point of read and survives the entire pipeline — sensor read → flash record → uplink. Stale data still flows (last-known-good behavior) but nothing downstream can mistake it for live.

### Rules
1. Stale bits for **GPS position, temperature, humidity**: set where the read fails/times out, cleared on a good read. No fabricated defaults anywhere in the pipeline.
2. Fail safe, not fail sunny: **stale/unknown temperature is treated as COLD** by the GPS lockout (GPS held). A gap is honest; a fantasy default is not.
3. Failed record conversion ⇒ skip that record, continue with the next. Never pack a failed conversion.
4. **Status byte restored to the compact uplink as byte 11** (ADR-0005 freed it: LinkCheck rides FOpts). Layout:
   - b0 GPS stale, b1 temp stale, b2 humidity stale
   - b3–b5 reset cause, condensed from `RCC->CSR` (POR / BOR / IWDG / SW / pin / fault-breadcrumb)
   - b6–b7 mission state (COMMISSIONING / ASCENT / FLOAT / reserved — see ADR-0008)
5. Detailed fault breadcrumbs (PC, CFSR/HFSR) are too big for this byte: log to flash and/or a rare diagnostic uplink; the status byte only signals *that* a fault reset occurred.
6. Flash side: the high-res record's flags byte (`PackStatusFlags`) is extended with the stale bits (second flags byte if needed).

## Consequences
- `EncodeCompactBinaryPacket` grows to 11 bytes (exact fit at US915 DR0).
- Ground decoder must interpret byte 11; versioned with the payload format doc.
- The GPS cold-lockout and `NormalizeBatteryVoltage` consume last-known-good + stale, never constants.
