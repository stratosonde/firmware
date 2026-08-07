# Stratosonde LoRaWAN Application Protocol

**Status:** Draft v0.1  
**Date:** 2026-08-03  
**Related decisions:** DDR-0005, DDR-0007, DDR-0011, DDR-0012  
**Audience:** Firmware, network-server integration, backend decoder, and data-pipeline developers

## 1. Purpose

This document defines the Stratosonde application-level LoRaWAN ports, delivery semantics, archive-opportunity state machine, and proposed extension payloads.

It records both the existing production formats and the proposed Qwiic extension formats.

## 2. FPort allocation

| FPort | Name | Class | Status |
|---:|---|---|---|
| 2 | Cayenne LPP debug | Development | Existing, disabled by default |
| 3 | GNSS detail debug | Development | Existing, disabled by default |
| 10 | Mission heartbeat | Heartbeat | Existing |
| 11 | Core science archive | First-class | Existing |
| 12 | Extension science archive | First-class | Proposed |
| 13 | Best-effort application fragments | Best-effort | Proposed |
| 14-19 | Reserved for future Stratosonde telemetry | Reserved | Do not use |
| 20+ | Future control/downlink services | Reserved | Separate authenticated design required |

Port meaning must not be changed in place. Incompatible changes require a payload version or a new port.

## 3. Canonical encoding warning

The current firmware sends packed C structures by casting them directly to byte buffers. On STM32WL this produces little-endian multibyte fields.

The existing `PayloadFormats.md` decoder examples describe many fields as big-endian. That is a compatibility discrepancy that must be resolved before declaring a stable wire protocol.

Required release gate:

1. Generate one golden packet from actual firmware.
2. Capture the exact transmitted FRMPayload.
3. Decode it independently.
4. Make firmware, this document, `PayloadFormats.md`, and backend tests agree.
5. Replace native-structure transmission with explicit serialization in the next incompatible payload version.

For proposed FPorts 12 and 13, all multibyte values are explicitly little-endian.

## 4. Delivery model

LoRaWAN does not provide exactly-once delivery.

Stratosonde uses:

- **At-least-once** delivery for first-class archive data.
- **Best-effort** delivery for application objects.
- Stable packet, record, producer, and object identifiers for backend deduplication.
- No irreversible archive acknowledgement based only on `LmHandlerSend()` success or radio TX completion.

A confirmed uplink acknowledgement consumes no FRMPayload byte. A `LinkCheckReq` is a MAC command carried in FOpts and does consume part of the LoRaWAN frame budget, even though it is not part of application FRMPayload.

## 5. Adaptive communication state machine

### 5.1 LONG_RANGE_HEARTBEAT

- Set the region-correct data rate corresponding to the configured long-range spreading factor, initially SF10.
- Send the 11-byte heartbeat on FPort 10.
- A heartbeat designated as an opportunity probe is sent as a confirmed uplink.
- No archive opportunity is opened without the required acknowledgement.

### 5.2 FIRST_ARCHIVE_PROBE

After a confirmed long-range heartbeat is acknowledged:

1. Switch to the region-correct high-throughput data rate, initially SF7.
2. Select the oldest eligible first-class archive data.
3. Send one confirmed archive packet.
4. Attach `LinkCheckReq` to that packet.
5. Keep all included archive records pending.

The network response may contain both:

- The confirmed-uplink ACK bit.
- `LinkCheckAns` with demodulation margin and gateway count.

### 5.3 ARCHIVE_BURST

Continue the burst only if:

- The first archive packet receives the required delivery evidence.
- A valid `LinkCheckAns` is received.
- Demodulation margin meets the configured threshold.
- Gateway count meets the configured threshold.
- Region, battery survival floor, duty-cycle, dwell-time, and payload-size rules permit continuation.

During the burst:

- First-class packets are confirmed unless a later ADR explicitly changes delivery evidence.
- Core and extension science are scheduled before best-effort data.
- A periodic heartbeat deadline may preempt the burst.
- Link check may be repeated periodically, not necessarily on every packet.
- Thresholds and maximum burst length are configurable.

### 5.4 FALLBACK

End the burst immediately when:

- An archive packet lacks required acknowledgement.
- Link-check response is missing or below threshold.
- The MAC cannot schedule the packet within policy.
- Region changes.
- Battery falls below the archive floor.
- Heartbeat becomes due.
- No first-class data remains and best-effort policy does not permit continuation.

Return to LONG_RANGE_HEARTBEAT for the next opportunity.

### 5.5 Delivery commit

A first-class packet remains pending until the required network evidence is received.

If an acknowledgement is lost but the backend received the packet:

- Stratosonde retransmits it later.
- The backend deduplicates it.
- Data is duplicated, not lost.

This is intentional.

## 6. FPort 10 — mission heartbeat v2

Payload length: 11 bytes (unchanged; exact fit at US915 DR0). All multibyte fields little-endian.

v2 (current firmware, D1/D2/D4, issue #33):

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 2 | Timestamp minutes modulo 65536 | `uint16_t` LE; wraps every 45.5 days — see status bit 5 |
| 2 | 2 | Latitude | `int16_t` LE, full-range scale: deg = value × 90 / 32767 |
| 4 | 2 | Longitude | `int16_t` LE, full-range scale: deg = value × 180 / 32767 |
| 6 | 1 | Temperature code | 2 °C steps, offset +64 (unsigned byte 0-255 → −64…+63.5 °C) |
| 7 | 2 | Pressure + humidity packed | `uint16_t` LE: bits 0-10 pressure in 1 hPa (0-2046 valid; 2047 = invalid); bits 11-15 humidity in 5% steps (0-20 valid; 31 = invalid) |
| 9 | 1 | Battery code | 50 mV steps |
| 10 | 1 | Status v2 | see bit table below |

Status v2 bits:

| Bits | Meaning |
|---:|---|
| 0 | GPS stale (last-known-good position) |
| 1 | Temperature stale |
| 2 | Humidity stale |
| 3 | Pressure stale |
| 4 | RTC GNSS-disciplined this cycle (N-03) |
| 5 | `timestamp_min` has wrapped its 45.5-day range (D4) |
| 6-7 | Mission state (0=COMMISSIONING, 1=PRE_FLIGHT, 2=FLIGHT, 3=RECOVERY) |

v1 → v2 changes: bytes 7-8 were separate legacy pressure code (950 hPa base, collapsed below 950 hPa) and humidity code; status bits 3-4 were a condensed reset cause and bit 5 was pressure-stale. Reset cause is no longer on the wire (it remains in flash/bulk records). v1 and v2 share port and length; discrimination is by deployment epoch plus the golden vectors in §13 — v1 never flew.

The heartbeat is intentionally coarse. It is not the authoritative science record.

### 6.1 Probe data rate (D1)

In US915/AU915 the heartbeat is sent at SF9 (DR1): the SF10/DR0 11-byte budget is an exact fit with zero headroom there; SF9 buys 42 bytes of headroom for ~2.5 dB of link budget. Elsewhere the heartbeat uses SF10. The data rate is resolved per region from spreading-factor intent, never from a hardcoded DR integer.

## 7. FPort 11 — core science archive

The decoder branches on `payload[0]`: `0x01` = legacy 222-byte fixed (historical only), `0x02` = legacy 198-byte fixed, `0x03` = current variable-length.

### 7.1 Archive v3 (current firmware, D3, issue #33)

Variable length: `2 + 32n + 4` bytes. Header:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet type, `0x03` |
| 1 | 1 | Record count n, 1-6 |

n complete 32-byte records follow (layout below), explicitly little-endian serialized (§3's explicit-serialization requirement lands with this version). The packet ends with a 4-byte CRC32/IEEE (LE) over all preceding bytes.

The serializer queries the runtime payload budget before each packet (`LoRaMacQueryTxPossible` — current DR plus pending FOpts, §11) and packs as many oldest complete records as fit. Records cut by the budget remain pending with stable identity (§4, §7.3).

### 7.2 Archive v2 (legacy)

Payload length: 198 bytes fixed.

Header:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Packet type, `0x02` |
| 1 | 1 | Record count, 0-6 |

Records begin at offset 2. Six 32-byte slots are present. Unused slots are zero.

Record layout:

| Record offset | Size | Field | Scale |
|---:|---:|---|---|
| 0 | 4 | Timestamp | Unix seconds |
| 4 | 4 | Latitude | Stratosonde binary coordinate |
| 8 | 4 | Longitude | Stratosonde binary coordinate |
| 12 | 2 | Altitude | metres |
| 14 | 2 | Temperature | signed, 0.1 °C |
| 16 | 2 | Humidity | 0.1% |
| 18 | 2 | Pressure | 0.1 hPa |
| 20 | 2 | Battery voltage | mV |
| 22 | 2 | Solar voltage | mV |
| 24 | 2 | Voltage slope | signed mV/hour |
| 26 | 1 | Satellites | count |
| 27 | 1 | HDOP | times 10 |
| 28 | 1 | Power mode | enum |
| 29 | 1 | Flags | bitfield |
| 30 | 2 | Record CRC16 | CRC16/MODBUS over bytes 0-29 |

Packet trailer:

| Offset | Size | Field |
|---:|---:|---|
| 194 | 4 | CRC32/IEEE | Over bytes 0-193 |

Legacy packet type `0x01` used 222 bytes and is retained only for backend compatibility.

### 7.3 Core archive identity requirement

The backend and firmware need a stable identity for every durable record. Timestamp alone is not sufficient under resets, duplicate timestamps, or clock correction.

A future version must carry either:

- A persistent 32-bit archive record ID, or
- A mission ID plus monotonic record sequence.

Until that exists, delivery commit and backend deduplication are weaker than intended.

## 8. FPort 12 — proposed extension science archive v1

FPort 12 carries first-class records published by Qwiic applications.

Packet header:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Version, `0x01` |
| 1 | 1 | Record count |
| 2 | 2 | Packet flags |
| 4 | 4 | Archive packet sequence |
| 8 | 2 | Bytes occupied by records |
| 10 | 2 | Header CRC16 |

One or more complete records follow.

Each record:

| Size | Field |
|---:|---|
| 2 | Record length including header and CRC |
| 4 | Stable Stratosonde archive record ID |
| 2 | Producer ID |
| 2 | Schema ID |
| 1 | Schema version |
| 1 | Record flags |
| 4 | Timestamp, Unix seconds |
| 4 | Producer sequence |
| 2 | Data length |
| N | Schema-defined data |
| 2 | Record CRC16/MODBUS |

Packet ends with a 4-byte CRC32/IEEE.

Rules:

- No record is split across LoRaWAN packets in v1.
- The serializer packs as many oldest complete records as fit the current regional maximum.
- A record too large for any supported archive data rate is rejected at Qwiic publication time.
- Records remain pending until the confirmed packet is acknowledged.
- Retransmission uses the same record IDs.
- The backend deduplicates by archive record ID.

## 9. FPort 13 — proposed best-effort application fragment v1

FPort 13 carries opaque fragments, such as JPEG data.

Header:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Version, `0x01` |
| 1 | 1 | Flags |
| 2 | 2 | Producer ID |
| 4 | 4 | Object ID |
| 8 | 4 | Total object length |
| 12 | 4 | Fragment offset |
| 16 | 1 | Content type |
| 17 | 1 | Fragment sequence modulo 256 |
| 18 | 2 | Fragment data length |
| 20 | N | Fragment data |
| 20+N | 2 | Fragment CRC16/MODBUS |

Flags:

| Bit | Meaning |
|---:|---|
| 0 | START |
| 1 | END |
| 2 | OBJECT_LENGTH_VALID |
| 3 | OBJECT_CRC_PRESENT |
| 4 | PARTIAL_OBJECT_ALLOWED |
| 5-7 | Reserved |

If `OBJECT_CRC_PRESENT`, the START fragment metadata includes the full object CRC32 before fragment data; the exact header length is indicated by data offset in a future minor revision or fixed in implementation.

Best-effort packets are normally unconfirmed. The backend must accept:

- Missing fragments.
- Duplicate fragments.
- Reordered fragments.
- Objects that never receive END.
- Eviction before transmission.

For images, progressive JPEG, thumbnails, independently decodable tiles, or application-level erasure coding may provide more value than raw sequential fragmentation.

## 10. Scheduling priority

Within an archive opportunity:

1. Due heartbeat.
2. Core science archive, FPort 11.
3. Extension science archive, FPort 12.
4. Best-effort fragments, FPort 13.

A configurable fairness ratio may alternate core and extension first-class packets. Best-effort data is sent only when first-class policy permits.

## 11. Regional payload sizing

No serializer may assume that SF7 always permits the same application payload in every region.

Before each packet:

- Resolve region-specific data rate.
- Query or calculate the legal maximum application payload after FOpts.
- Include the current FOpts length, including `LinkCheckReq`.
- Pack only complete records or a bounded best-effort fragment that fits.
- Respect dwell-time and duty-cycle restrictions.

Compile-time worst-case checks remain required for fixed formats.

## 12. Backend responsibilities

The backend must:

- Branch by FPort and payload version.
- Validate length before reading fields.
- Validate record and packet CRCs.
- Deduplicate first-class records.
- Reassemble best-effort objects sparsely.
- Preserve raw payload bytes for forensic reprocessing.
- Track missing, duplicate, and corrupt records explicitly.
- Never invent missing science values.
- Support legacy port 11 packet type `0x01` only while historical data requires it.

## 13. Golden vectors

Each payload version must include:

- One encoder test vector.
- One decoder test vector.
- Minimum and maximum field values.
- Invalid length and invalid CRC examples.
- Actual captured bytes from firmware.
- Tests in firmware and backend repositories using identical vectors.

## 14. Implementation deltas from current firmware

The desired strategy differs from the current implementation in several important respects:

- Current code requests `LinkCheckReq` on the SF10 heartbeat itself.
- Current heartbeat send is unconfirmed.
- Current bulk packets are unconfirmed.
- Current flash delivery marking occurs after successful radio TX callback, not confirmed network acknowledgement.
- Current queue handling and count-based record marking require transactional repair.

Resolved by the heartbeat v2 / archive v3 rework (#33):

- ~~Current FPort 10 pressure representation is not useful at stratospheric pressure~~ — v2 packs 1 hPa over 0-2046 hPa.
- ~~Existing byte-order documentation conflicts with direct packed-structure transmission~~ — LE is wire truth (D9); archive v3 records are explicitly LE-serialized; heartbeat v2 documented LE.

These are implementation tasks, not merely documentation changes.

## 15. Required tests

- Confirmed heartbeat ACK opens exactly one archive probe.
- No ACK leaves the data rate in long-range mode.
- First SF7 packet includes LinkCheckReq and remains pending without response.
- Good ACK plus LinkCheckAns enters burst.
- Poor margin or gateway count returns immediately to heartbeat.
- Lost ACK causes duplicate retransmission and backend deduplication.
- Region change aborts burst.
- FOpts reduces payload packing correctly.
- Port 12 never splits a first-class record.
- Port 13 accepts partial, duplicate, and reordered fragments.
- Actual firmware bytes match golden vectors.
