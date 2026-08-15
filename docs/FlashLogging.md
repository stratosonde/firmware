# Flash Logging Module

## Overview

The Flash Logging Module (`Core/Src/flash_log.c`, API in `Core/Inc/flash_log.h`)
is the circular science archive: 64-byte `FlashLog_Record_t` records in external
NOR flash, written every admitted cycle and replayed opportunistically when
LoRaWAN connectivity allows. Intent authority: DDR-0004 (archive), DDR-0010
(persistence), DDR-0011 (storage mechanics); NOR mechanics: FlashStorageNotes.md.

## Hardware Interface

- **Flash Chip**: W25Q16JV (16 Mbit / 2 MB), driver `Core/Src/w25q16jv.c`
- **Interface**: SPI2
- **Pins**: MOSI PA10 · MISO PB14 · SCK PB13 · CS PB9 (software CS, pre-initialized HIGH in `main.c` before SPI init)
- **Erase granularity**: 4 KB sectors (512 total); NOR program can only clear bits

## Memory Organization

```
+---------------------------+  0x000000
| Header A (sector 0, 4KB)  |
+---------------------------+  0x001000
| Header B (sector 1, 4KB)  |  <- separate sector: one erase can never kill both
+---------------------------+  0x002000
|                           |
|   Records (sectors 2-511) |  circular buffer, ~32,000 records
|                           |
+---------------------------+  0x200000
```

- **No wear leveling**: simple overwrite of oldest data when full (SI-009: new
  observations win over old ones)
- **Erase-ahead**: in a wrapped ring the sector ahead of the write frontier is
  erased before writing into it; `oldest_addr` advances past erased records
- **Header checkpointing**: the header is persisted every
  `HEADER_UPDATE_INTERVAL` (10) records; an unclean reboot loses at most that
  many records of bookkeeping, which boot recovery re-derives (bounded record
  loss acceptable, structural integrity inviolable)

## Record Layout (64 B, record layout v4 — 2026-08-06, D5/#35)

Authoritative definition: `Core/Inc/flash_log.h` (`FlashLog_Record_t`,
`_Static_assert` 64 bytes). All multibyte fields little-endian; CRC32 over the
first 60 bytes.

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;             // 0xFEEDDA7A
    uint32_t sequence;          // monotonic, consumed only after write success
    uint32_t timestamp;         // UTC epoch seconds (GNSS-disciplined; boot-relative before first fix)
    float    pressure;          // mbar
    float    temperature;       // degC
    float    humidity;          // %
    int32_t  latitude;          // binary (scaled 8388607/90)
    int32_t  longitude;         // binary (scaled 8388607/180)
    int32_t  altitude_gps;      // metres (v4: widened from int16)
    uint8_t  satellites;
    uint8_t  gnss_fix_quality;
    uint8_t  gnss_hdop_x10;
    uint8_t  gnss_valid;
    uint16_t battery_mv;
    uint16_t solar_mv;          // v4: was never archived before (F-025)
    int16_t  voltage_slope;     // mV/hour at write time (v4)
    uint8_t  power_mode;        // operating mode at write time (v4)
    uint8_t  flags;             // b0 press_stale, b1 temp_stale, b2 hum_stale,
                                // b3 gnss_stale, b4 batt_stale (#136),
                                // b5-b7 TransmitVeto_t at write time (2026-08-11 §6a, DDR-0003)
    uint8_t  reserved[12];
    uint32_t crc32;
} FlashLog_Record_t;
```

`altitude_bar` was deleted in v4 (never assigned; the backend computes barometric
altitude from pressure+temperature). One version bump covers F-024/F-025/R19.

A record that fails validation on read (`FlashLog_VerifyRecord`: magic + CRC32)
is skipped in place — corruption is local and never wedges traversal (SI-010);
firmware never repairs science records.

## Header (v5) and Power-Failure Recovery

Two header copies in dedicated sectors 0/1, ping-ponged by a monotonic
`sequence`/`header_generation` (T4, F-007/R12 #50): on init both are read and
the valid one with the higher generation wins. Data is written first, header
second — power loss during a record write costs at most that record; power loss
during a header write leaves the old header valid.

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;                // 0xF1A5DEAD
    uint32_t version;              // header version (current: 5)
    uint32_t write_addr;           // next write address
    uint32_t record_count;         // total records written (may exceed capacity)
    uint32_t sequence;             // header update sequence (ping-pong selection)
    uint32_t oldest_addr;          // oldest valid record
    uint32_t flags;
    uint32_t last_transmitted_seq; // v5: TX HIGH WATER — highest sequence ever
                                   // handed to the radio (monotonic up)
    uint32_t reserved[2];          // reserved[0] v5: RECOVERY FRONTIER — the
                                   // one-pass walker has visited every seq >=
                                   // frontier (monotonic down)
    uint32_t crc32;
} FlashLog_Header_t;
```

Version history: v3 moved headers to dedicated sectors (T4); v4 changed the
record layout (D5/#35, old headers fail validation → clean init, acceptable
pre-launch); v5 (R3-04/#218, DDR-0005) redefined the two watermark semantics for
the one-pass recovery walker.

## One-Pass Recovery Walker (DDR-0005)

`tx_high_water` (monotonic up) and `recovery_frontier` (monotonic down) bracket
the not-yet-replayed history exactly once: the walker serves records from
`tx_high_water - 1` downward and retires each below the frontier. It never
rewalks, never needs a persistent job queue, and tolerates gaps/duplicates
(SI-018). Explicit backend record-requests outrank the walker (SI-017 —
requested-record lookup is not yet implemented, see the conformance worklist).

`FlashLog_DeferHeaderSync()` / `FlashLog_FlushHeaderSync()` batch the watermark
persist across a bulk burst (Finding #8): `MarkRecoverySent` skips the sector
erase per packet and the caller flushes once at burst end. The
`pending_tx_committed` gate (C-01/#270) keeps a post-send reset from
double-committing the watermark ahead of the ACK.

## API (actual — `flash_log.h`)

| Function | Purpose |
|----------|---------|
| `FlashLog_Init` / `FlashLog_DeInit` | Recover state from headers (or clean-init); release |
| `FlashLog_WriteRecord` | Append one record (erase-ahead, sequence consumed only on success) |
| `FlashLog_ReadRecord` / `FlashLog_ReadRecords` | Read by address / batch |
| `FlashLog_GetRecordCount` / `FlashLog_GetAvailableRecords` / `FlashLog_HasWrapped` | Ring statistics |
| `FlashLog_SyncHeader` / `FlashLog_DeferHeaderSync` / `FlashLog_FlushHeaderSync` | Header checkpoint control |
| `FlashLog_VerifyRecord` | Magic + CRC32 validation (lazy, on read) |
| `FlashLog_GetRecoveryRecords` / `FlashLog_MarkRecoverySent` / `FlashLog_GetUnsentCount` / `FlashLog_HasUnsentData` | Recovery walker + watermarks |
| `FlashLog_GetStats` | Counters |
| `FlashLog_CRC32` | Shared CRC helper |

## Error Handling

Graceful degradation, no limp mode: write failures cost at most the current
record (sequence not consumed); read failures skip the corrupt record; header
loss falls back to the ping-pong copy. Double-header loss currently triggers a
full-archive rescan — flagged by the 2026-08-14 review as an SI-010 tension
(#289, open). Errors are counted via `FlashLog_GetStats` and surfaced to the
system module.

## Power Considerations

- Flash chip cannot be powered off in hardware; the driver uses the W25Q
  power-down command between operations
- CS is driven HIGH before SPI init so the flash never floats selected (main.c)

## Cross-References

- `docs/FlashStorageNotes.md` — NOR erase-before-write mechanics and the
  wrapped-ring frontier rule (sequence discontinuity)
- `docs/PayloadFormats.md` — the wire encoding these records leave in
  (Port 11 archive packets, v1–v6)
- `tests/host/fake_w25q.c` — NOR-faithful flash double (program only clears
  bits, 4 KB erase) making the invariants host-testable
