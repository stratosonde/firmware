# Stratosonde Application Services Protocol

**Status:** Draft v0.2  
**Date:** 2026-08-03 (revised 2026-08-09)  
**Transport:** `QwiicTransportProtocol.md`  
**Related decisions:** DDR-0017  
**Audience:** Application payload developers

> **v0.2 revision (DDR-0017):** decision references updated to the V2 design corpus. The service set, data-class contract, and deadline behavior below already match DDR-0017; note that first-class extension records are full equals of core archive records (§16), and Application Controller sessions are energy-gated — a session may simply not be granted on a given wake when energy policy does not admit the rail load.

## 1. Purpose

This document defines the services available to an intelligent Application Controller while it owns a powered Qwiic session.

The intended developer experience is:

1. Wake when Stratosonde applies Qwiic power.
2. Claim a bounded session.
3. Perform application work.
4. Publish first-class records and/or best-effort objects.
5. Call `COMPLETE_SESSION`.
6. Stop activity and tolerate power removal.

The application does not manage Stratosonde flash, LoRaWAN, data rates, mission state, or sleep.

## 2. Service command map

| Command | Name | Session required | Side effect |
|---:|---|---|---|
| `0x01` | GET_PROTOCOL_INFO | No | No |
| `0x02` | CLAIM_SESSION | No | Yes |
| `0x03` | GET_SESSION_STATUS | Yes | No |
| `0x04` | COMPLETE_SESSION | Yes | Yes |
| `0x05` | ABORT_SESSION | Yes | Yes |
| `0x10` | GET_MISSION_TIME | Yes | No |
| `0x11` | GET_NAVIGATION | Yes | No |
| `0x20` | PUBLISH_MISSION_RECORD | Yes | Yes |
| `0x30` | BEGIN_BEST_EFFORT_OBJECT | Yes | Yes |
| `0x31` | WRITE_BEST_EFFORT_CHUNK | Yes | Yes |
| `0x32` | END_BEST_EFFORT_OBJECT | Yes | Yes |
| `0x33` | CANCEL_BEST_EFFORT_OBJECT | Yes | Yes |

Responses use command OR `0x80`. Their payload begins with the transport status code.

## 3. Common service fields

All multibyte integers are little-endian.

### 3.1 Session token

Every session-required request begins with the 32-bit session token granted by `CLAIM_SESSION`.

### 3.2 Producer ID

A 16-bit producer ID identifies the hardware/application producer. IDs are assigned by the Stratosonde project. `0x0000` is reserved for Stratosonde core firmware.

### 3.3 Producer sequence

A producer-maintained 32-bit monotonic sequence supports deduplication. The sequence should survive application resets when practical.

Stratosonde combines producer ID and producer sequence when detecting duplicate first-class records.

## 4. GET_PROTOCOL_INFO (`0x01`)

Request payload is empty.

Successful response after status:

| Size | Field |
|---:|---|
| 1 | Transport major |
| 1 | Transport minor |
| 1 | Application service major |
| 1 | Application service minor |
| 4 | Service capability flags |
| 2 | Maximum frame payload |
| 2 | Reserved |

## 5. CLAIM_SESSION (`0x02`)

The transport document defines the request and response.

An accepted claim does not guarantee that every service will be accepted. Remaining time, power, storage, and mission policy are evaluated per request.

## 6. GET_SESSION_STATUS (`0x03`)

Request:

| Size | Field |
|---:|---|
| 4 | Session token |

Response after status:

| Size | Field |
|---:|---|
| 4 | Milliseconds to soft deadline |
| 4 | Milliseconds to hard deadline |
| 2 | Remaining first-class byte budget |
| 4 | Remaining best-effort spool budget |
| 2 | Active object handle, zero if none |
| 4 | Session flags |

Applications should stop starting new work before the soft deadline.

## 7. COMPLETE_SESSION (`0x04`)

Request:

| Size | Field |
|---:|---|
| 4 | Session token |
| 2 | Application result code |
| 2 | Published first-class record count |
| 2 | Published best-effort object count |
| 2 | Reserved |

Response after status:

| Size | Field |
|---:|---|
| 4 | Accepted first-class record count |
| 4 | Accepted best-effort byte count |
| 4 | Suggested milliseconds before power removal |

After a successful response, the application must perform no further I2C transactions.

Stratosonde may remove power earlier than the suggestion if the hard deadline or safety policy requires it.

## 8. ABORT_SESSION (`0x05`)

This reports that application work failed or was intentionally abandoned.

Request:

| Size | Field |
|---:|---|
| 4 | Session token |
| 2 | Application error code |
| 2 | Reserved |

Open best-effort objects are cancelled. Successfully accepted first-class records remain valid.

## 9. GET_MISSION_TIME (`0x10`)

Request:

| Size | Field |
|---:|---|
| 4 | Session token |

Response after status:

| Size | Field |
|---:|---|
| 4 | Unix seconds |
| 2 | Milliseconds |
| 1 | Time quality |
| 1 | Flags |

Time quality:

| Value | Meaning |
|---:|---|
| 0 | Unknown |
| 1 | RTC estimate |
| 2 | Previously GNSS-disciplined RTC |
| 3 | Current GNSS time |

Applications may submit timestamp zero and ask Stratosonde to assign mission time.

## 10. GET_NAVIGATION (`0x11`)

Request:

| Size | Field |
|---:|---|
| 4 | Session token |

Response after status:

| Size | Field |
|---:|---|
| 4 | Latitude, signed binary Stratosonde format |
| 4 | Longitude, signed binary Stratosonde format |
| 4 | Altitude in centimetres, signed |
| 4 | Fix timestamp, Unix seconds |
| 1 | Satellites |
| 1 | HDOP times 10 |
| 1 | Fix quality |
| 1 | Flags |

Flags identify valid versus last-known-good/stale data. Applications must not reinterpret stale data as a current fix.

## 11. PUBLISH_MISSION_RECORD (`0x20`)

Use this for first-class scientific data that must enter the durable archive.

Request:

| Size | Field |
|---:|---|
| 4 | Session token |
| 2 | Producer ID |
| 2 | Schema ID |
| 1 | Schema version |
| 1 | Record flags |
| 4 | Producer sequence |
| 4 | Timestamp, Unix seconds; zero requests Stratosonde assignment |
| 2 | Data length |
| N | Schema-defined data |

Record flags:

| Bit | Meaning |
|---:|---|
| 0 | Application timestamp is valid |
| 1 | Data is calibrated |
| 2 | Data contains stale values |
| 3 | Data contains a validity bitmap |
| 4-7 | Reserved |

Successful response after status:

| Size | Field |
|---:|---|
| 4 | Stratosonde archive record ID |
| 4 | Assigned timestamp |
| 1 | Storage class, `1` durable |
| 1 | Deduplicated, `1` if record already existed |
| 2 | Reserved |

### 11.1 Acceptance guarantee

`OK` or `ACCEPTED` means the complete record has been copied into Stratosonde-controlled durable storage and is eligible for archive recovery.

It does not mean the record has reached the backend.

A record is not accepted if:

- The schema or version is unsupported.
- Data length is invalid.
- Durable storage cannot commit it.
- The session deadline cannot accommodate the commit.
- The same producer sequence is reused with different data.
- Mission policy rejects the producer.

### 11.2 Schema rules

Each schema specification defines:

- Units and scaling.
- Byte order.
- Required validity fields.
- Calibration identity.
- Legal ranges.
- How missing values are represented.
- Golden encode/decode vectors.

Opaque unversioned bytes are not first-class mission data.

## 12. BEGIN_BEST_EFFORT_OBJECT (`0x30`)

Use this for images and other opaque data whose loss must not affect core mission success.

Request:

| Size | Field |
|---:|---|
| 4 | Session token |
| 2 | Producer ID |
| 1 | Content type |
| 1 | Object flags |
| 4 | Object ID |
| 4 | Total length |
| 4 | Object CRC32, zero if unknown until end |
| 2 | Preferred chunk length |
| 2 | Metadata length |
| M | Optional metadata |

Content types:

| Value | Meaning |
|---:|---|
| 0 | Opaque binary |
| 1 | JPEG |
| 2 | Progressive JPEG |
| 3 | PNG |
| 4 | UTF-8 text |
| 5 | CBOR |
| 6 | Application-defined |
| 7-255 | Reserved |

Successful response after status:

| Size | Field |
|---:|---|
| 2 | Object handle |
| 2 | Maximum chunk data length |
| 4 | Bytes reserved or immediately available |
| 1 | Storage mode |
| 1 | Reserved |

Storage mode:

| Value | Meaning |
|---:|---|
| 0 | Volatile only |
| 1 | Low-priority flash spool |
| 2 | Streaming opportunity only |

The application must inspect the response. A smaller reservation or `PARTIAL` status is allowed.

## 13. WRITE_BEST_EFFORT_CHUNK (`0x31`)

Request:

| Size | Field |
|---:|---|
| 4 | Session token |
| 2 | Object handle |
| 4 | Offset |
| 2 | Data length |
| N | Data |

Response after status:

| Size | Field |
|---:|---|
| 4 | Accepted through offset |
| 4 | Total accepted bytes |
| 4 | Remaining object budget |

A repeated identical chunk is idempotent. Overlapping chunks with different bytes are rejected.

## 14. END_BEST_EFFORT_OBJECT (`0x32`)

Request:

| Size | Field |
|---:|---|
| 4 | Session token |
| 2 | Object handle |
| 2 | Final flags |
| 4 | Final length |
| 4 | CRC32/IEEE over complete object |

Response after status:

| Size | Field |
|---:|---|
| 4 | Accepted object ID |
| 4 | Accepted byte count |
| 1 | Complete locally |
| 1 | Eligible for radio |
| 2 | Reserved |

`Complete locally` does not imply durable retention. The spool may later evict the object.

If the object is incomplete, Stratosonde may accept a partial object only when the object flags permit partial delivery.

## 15. CANCEL_BEST_EFFORT_OBJECT (`0x33`)

Request contains session token and object handle. Accepted bytes may be reclaimed.

## 16. Data-class scheduling contract

Stratosonde schedules data in this order:

1. Mission heartbeat when due.
2. Core first-class scientific archive.
3. Extension first-class mission records.
4. Best-effort application objects.

A fairness policy may interleave core and extension first-class records. Best-effort data never blocks a due heartbeat or a first-class commit.

## 17. Camera example

An ESP32 camera application typically:

1. Claims ten seconds.
2. Gets mission time and optionally position.
3. Captures and compresses a JPEG.
4. Calls `BEGIN_BEST_EFFORT_OBJECT`.
5. Writes chunks until the reservation, soft deadline, or image end.
6. Ends the object with length and CRC32.
7. Completes the session.
8. Stops and waits for power removal.

For best recovery, use a thumbnail or progressive JPEG first. A fully transmitted thumbnail is often more valuable than the first arbitrary bytes of a large baseline JPEG.

## 18. Extension science example

A smart external instrument can publish:

- Producer ID `0x0102`
- Schema ID `0x0201`
- Schema version `1`
- Producer sequence `87`
- Timestamp zero for Stratosonde assignment
- Data containing RTD temperature, capacitive humidity, validity flags, and calibration identifier

If accepted, Stratosonde returns a stable archive record ID and retains the record under first-class archive policy.

## 19. Error handling

- Retry `ERR_BUSY` only while sufficient hard-deadline margin remains.
- Treat `ERR_DEADLINE` as a command to complete immediately.
- Treat `ERR_STORAGE` on first-class data as a failed scientific commit; do not pretend publication succeeded.
- Best-effort rejection is normal and must not prevent session completion.
- No application retry loop may be unbounded.

## 20. Required tests

- Duplicate first-class request returns one archive record.
- Same sequence with different bytes is rejected.
- Power loss during first-class commit leaves either the old state or one valid new record.
- Oversized best-effort object receives a bounded reservation or rejection.
- Partial and out-of-order chunk behavior matches flags.
- Session deadline expires during capture and during chunk write.
- Camera can complete even when zero best-effort bytes are accepted.
- First-class storage remains available under best-effort spool pressure.
