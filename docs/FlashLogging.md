# Flash Logging Module

## Overview

The Flash Logging Module provides persistent storage for telemetry data when LoRaWAN connectivity is unavailable. It implements a circular buffer in external flash memory, allowing the device to store high-resolution sensor readings and retrieve them later for transmission.

## Hardware Interface

- **Flash Chip**: W25Q16JV
- **Interface**: SPI
- **Pins**:
  - MOSI: PA10
  - MISO: PB14
  - CS: PB9
  - SCK: PB13
- **Capacity**: 16 Mbit (2 MB)

## Memory Organization

```
+---------------------------+
| Configuration/Calibration |
+---------------------------+
|                           |
|                           |
|      Telemetry Data       |
|     (Circular Buffer)     |
|                           |
|                           |
+---------------------------+
```

- **Configuration/Calibration Section**: Small section at the beginning of flash
- **Telemetry Section**: Majority of flash used for circular buffer of telemetry records
- **No wear leveling required**: Simple overwrite of oldest data when full

## Data Structures

### Flash record (64 B, record layout v4 — 2026-08-06, D5/#35)

Authoritative definition: `Core/Inc/flash_log.h` (`FlashLog_Record_t`, header version 4).
All multibyte fields little-endian; CRC32 over the first 60 bytes.

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
    uint8_t  flags;             // stale bits: b0 press, b1 temp, b2 hum, b3 gnss
    uint8_t  reserved[12];
    uint32_t crc32;
} FlashLog_Record_t;            // exactly 64 bytes (_Static_assert)
```

`altitude_bar` was deleted in v4 (never assigned; the backend computes barometric
altitude from pressure+temperature). One version bump covers F-024/F-025/R19.

### Wire record (32 B) — HighResTelemetryRecord_t

The 32-byte record carried in FPort 11 archive packets. Authoritative definition:
`Core/Inc/payload_format.h`; byte layout and decoder: `docs/PayloadFormats.md`
(archive v3).

## Metadata Structure

```c
typedef struct {
    uint32_t magic;               // Magic number for validation
    uint32_t write_pointer;       // Current write position
    uint32_t last_transmitted;    // Last transmitted record pointer
    uint16_t record_count;        // Number of valid records
    uint16_t crc;                 // CRC for metadata validation
} FlashMetadata_t;
```

## Key Functions

### Initialization

```c
FlashStatus_t Flash_Init(void);
```
- Initializes SPI interface
- Detects and identifies flash chip
- Reads metadata and validates integrity
- Prepares circular buffer for operation

### Writing Data

```c
FlashStatus_t Flash_WriteRecord(HighResTelemetryRecord_t *record);
```
- Writes a high-resolution telemetry record to flash
- Updates metadata with new write position
- Handles buffer wrap-around

### Reading Data

```c
FlashStatus_t Flash_ReadRecord(uint32_t index, HighResTelemetryRecord_t *record);
FlashStatus_t Flash_ReadLatestRecord(HighResTelemetryRecord_t *record);
FlashStatus_t Flash_ReadNextUnsentRecord(HighResTelemetryRecord_t *record);
```
- Retrieves records from flash based on index or status
- Validates record integrity using CRC

### Packet Conversion

```c
void Flash_ConvertToLowResPacket(HighResTelemetryRecord_t *record, LowResTelemetryPacket_t *packet);
```
- Converts high-resolution record to 11-byte LoRaWAN packet
- Compresses data to fit within size constraints
- Prioritizes critical information

### Transmission Tracking

```c
FlashStatus_t Flash_MarkAsSent(uint32_t index);
FlashStatus_t Flash_GetTransmissionStatus(uint32_t *sent, uint32_t *total);
```
- Tracks which records have been successfully transmitted
- Updates watermark or packet ID markers
- Provides statistics on transmission status

### Power Management

```c
FlashStatus_t Flash_EnterLowPowerMode(void);
FlashStatus_t Flash_ExitLowPowerMode(void);
```
- Sends low power shutdown mode command to flash chip
- Restores normal operation when needed

### Error Handling

```c
FlashStatus_t Flash_EnterLimpMode(void);
bool Flash_IsInLimpMode(void);
```
- Enters limp mode when flash errors occur
- Provides status information for system module

## Error Handling

The Flash Logging Module implements graceful failure modes:

1. **Retry Mechanism**: Attempts operations multiple times before declaring failure
2. **Limp Mode**: When flash errors occur, the system enters a limp mode where:
   - Data can still be transmitted via LoRaWAN
   - New data cannot be written to flash
   - Historical data cannot be read from flash
3. **Error Reporting**: Flash errors are reported to the Error Handler module

## Power Considerations

- Flash chip cannot be powered off via hardware
- Low power shutdown mode command is used to minimize power consumption
- SPI interface is only enabled when flash operations are needed

## Implementation Notes

- Fixed-size record format for efficient storage and retrieval
- Metadata region for tracking write position and transmission status
- Simple overwrite of oldest data when buffer is full
- No wear leveling required as specified
- CRC validation for data integrity
