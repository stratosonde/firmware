# Building a Stratosonde Qwiic Application

**Status:** Draft  
**Date:** 2026-08-03  
**Read with:** `QwiicTransportProtocol.md` and `ApplicationServicesProtocol.md`

## 1. Choose the expansion type

There are two supported models.

### Intelligent Application Controller

Use this when the expansion has its own microcontroller, for example an ESP32 camera.

The expansion:

- Boots when Stratosonde powers the Qwiic rail.
- Becomes the I2C controller for the session.
- Claims Stratosonde at address `0x42`.
- Calls application services.
- Completes before power is removed.

### Passive sensor board

Use this when the expansion is a sensor IC, for example an AD7745-based capacitance and RTD board.

The board:

- Does not claim the bus.
- Is bound from a commissioned static profile (DDR-0017: no runtime discovery, no descriptor EEPROM).
- Allows Stratosonde to become I2C controller.
- Is read by a versioned Stratosonde sensor driver.

## 2. Decide the data class

### First-class mission record

Choose first-class when the data is part of the scientific result and must be retained until acknowledged.

You need:

- An assigned producer ID.
- A schema ID and version.
- Units, scaling, ranges, and validity rules.
- A producer sequence.
- A bounded record size.
- Golden encoding vectors.

### Best-effort object

Choose best-effort for large or experimental data whose loss is acceptable.

Examples:

- JPEG image.
- Thumbnail.
- Diagnostic capture.
- Application log excerpt.

The object can be rejected, evicted, partially transmitted, or lost.

## 3. Minimal intelligent-controller sequence

```text
Qwiic power appears
    ↓
Initialize I2C controller at 100 kHz
    ↓
Probe Stratosonde target 0x42
    ↓
GET_PROTOCOL_INFO
    ↓
CLAIM_SESSION
    ↓
Perform bounded application work
    ↓
Publish mission records and/or best-effort object
    ↓
COMPLETE_SESSION
    ↓
Stop bus activity and tolerate power removal
```

## 4. Minimal frame helper in Python

This example illustrates framing. Adapt I2C calls to the ESP32 environment.

```python
import struct

MAGIC = b"SQ"
VERSION = 1
STRATOSONDE_ADDR = 0x42

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def make_frame(message_type: int, sequence: int, payload: bytes,
               flags: int = 0) -> bytes:
    if len(payload) > 224:
        raise ValueError("payload exceeds proposed v1 frame limit")
    header = MAGIC + bytes((VERSION, message_type, flags, sequence))
    header += struct.pack("<H", len(payload))
    body = header + payload
    return body + struct.pack("<H", crc16_modbus(body))

def parse_frame(frame: bytes) -> tuple[int, int, int, bytes]:
    if len(frame) < 10 or frame[:2] != MAGIC:
        raise ValueError("bad frame")
    version, message_type, flags, sequence = frame[2:6]
    payload_length = struct.unpack_from("<H", frame, 6)[0]
    expected = 10 + payload_length
    if len(frame) != expected:
        raise ValueError("bad length")
    expected_crc = struct.unpack_from("<H", frame, 8 + payload_length)[0]
    actual_crc = crc16_modbus(frame[:8 + payload_length])
    if expected_crc != actual_crc:
        raise ValueError("bad CRC")
    return message_type, flags, sequence, frame[8:8 + payload_length]
```

Production code also needs:

- Retry with identical sequence and request bytes.
- Response status handling.
- Deadline tracking.
- Bounds on every loop.
- Immediate completion when time is low.

## 5. ESP32 camera example

### 5.1 Mission behavior

On each powered session:

1. Claim a session.
2. Capture a small thumbnail first.
3. Optionally capture a larger progressive JPEG if time and reservation permit.
4. Publish through the best-effort object service.
5. Complete the session.
6. Never wait indefinitely for Stratosonde or the camera.

A thumbnail-first policy improves scientific return. A complete 8-20 kB image is often more useful than an incomplete 200 kB image.

### 5.2 Pseudocode

```python
def run_camera_session(i2c, camera, producer_id, object_id):
    session = claim_session(i2c, requested_ms=10_000)

    # Keep a hard local guard earlier than Stratosonde's deadline.
    local_stop_ms = session.hard_deadline_ms - 250

    image = camera.capture_jpeg(
        resolution="QVGA",
        quality=35,
        progressive=True,
    )

    begin = begin_best_effort(
        i2c=i2c,
        session_token=session.token,
        producer_id=producer_id,
        content_type=2,  # progressive JPEG
        object_id=object_id,
        total_length=len(image),
        crc32=crc32(image),
        preferred_chunk=192,
    )

    accepted = 0
    while accepted < len(image):
        if milliseconds_remaining() < local_stop_ms:
            break

        end = min(accepted + begin.max_chunk, len(image))
        result = write_chunk(
            i2c,
            session.token,
            begin.handle,
            accepted,
            image[accepted:end],
        )

        if not result.accepted:
            break
        accepted = result.accepted_through

    end_best_effort(
        i2c,
        session.token,
        begin.handle,
        final_length=len(image),
        crc32=crc32(image),
        allow_partial=True,
    )

    complete_session(i2c, session.token)
```

The application must accept that zero image bytes may be retained.

## 6. Smart first-class sensor example

A controller reading an external precision instrument can publish a schema-defined record.

Example conceptual schema `0x0201` version 1:

| Offset | Size | Field | Scale |
|---:|---:|---|---|
| 0 | 2 | RTD temperature | signed 0.01 °C |
| 2 | 2 | Capacitive humidity | 0.01% RH |
| 4 | 4 | Raw capacitance | attofarads or profile-defined |
| 8 | 2 | Excitation voltage | mV |
| 10 | 1 | Validity flags | bitfield |
| 11 | 1 | Calibration revision | integer |

Publish with:

- Stable producer ID.
- Monotonic producer sequence.
- Timestamp zero if Stratosonde should assign mission time.
- `calibrated` flag only when the declared calibration was applied.

A successful response returns the durable Stratosonde archive record ID.

## 7. Passive AD7745 sensor-board example

A passive board contains:

- AD7745 at its native I2C address.
- A commissioned static profile declaring: sensor profile ID ("AD7745 RTD + capacitive humidity"), native address, hardware revision, schema ID and version, and calibration reference.

On wake:

1. The board does not drive I2C as controller.
2. The claim window expires.
3. Stratosonde becomes controller.
4. Stratosonde loads the commissioned static profile (no discovery is performed).
5. The registered profile driver configures and reads the AD7745.
6. The driver emits a first-class mission record.
7. Stratosonde removes Qwiic power.

Do not identify this board merely because an ACK appears at the AD7745 address. Address alone is not identity.

## 8. Schema checklist

Before assigning a first-class schema:

- [ ] Every field has units.
- [ ] Byte order is explicit.
- [ ] Signedness and scale are explicit.
- [ ] Minimum, maximum, and saturation are explicit.
- [ ] Invalid and stale values are explicit.
- [ ] Calibration identity is explicit.
- [ ] Record size is bounded.
- [ ] Producer sequence behavior is defined.
- [ ] Golden bytes and decoded values are supplied.
- [ ] Backend support exists before flight.

## 9. Best-effort object checklist

- [ ] Content type assigned.
- [ ] Object ID is stable and unlikely to collide.
- [ ] Total size is bounded.
- [ ] Partial delivery behavior is useful.
- [ ] Thumbnail/progressive/tiled representation considered.
- [ ] Application can complete when no storage is available.
- [ ] Capture and compression have timeouts.
- [ ] Local deadline is earlier than Stratosonde's hard deadline.

## 10. Failure philosophy

Expected application failures include:

- Controller boot takes too long.
- Camera fails to initialize.
- Sensor returns invalid data.
- Stratosonde has no best-effort quota.
- Session expires.
- I2C response is lost.
- Power is removed.

The correct behavior is bounded failure and clean return to the core mission. No optional application may trap Stratosonde awake.

## 11. Repository integration checklist

A new expansion should add:

1. Producer ID registration.
2. Schema or content-type registration.
3. Static commissioned profile entry, if passive.
4. Firmware encoder tests.
5. Backend decoder tests.
6. Golden protocol vectors.
7. Power and maximum-session measurements.
8. Fault-injection tests.
9. Documentation of whether data is first-class or best-effort.
