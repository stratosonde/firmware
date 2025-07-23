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

### High-Resolution Telemetry Record

```c
typedef struct {
    uint32_t timestamp;           // UTC timestamp
    int32_t latitude;             // Full resolution latitude
    int32_t longitude;            // Full resolution longitude
    uint16_t altitude;            // Altitude in meters
    int16_t temperature;          // Temperature in 0.1°C
    uint16_t humidity;            // Humidity in 0.1%
    uint16_t pressure;            // Pressure in 0.1 hPa
    uint16_t packet_index;        // Sequence number (0-65535)
    uint8_t satellites;           // Number of satellites
    uint8_t hdop;                 // Horizontal dilution of precision
    uint8_t battery;              // Battery level in %
    uint8_t flags;                // Status flags
    uint16_t crc;                 // CRC for data validation
} HighResTelemetryRecord_t;
```

### Low-Resolution Telemetry Packet (11 bytes)

```c
typedef struct {
    uint8_t timestamp[2];         // Compressed timestamp (minutes since epoch)
    uint8_t latitude[2];          // 100m resolution latitude
    uint8_t longitude[2];         // 100m resolution longitude
    uint8_t altitude;             // Compressed altitude
    uint8_t temperature;          // Compressed temperature
    uint8_t pressure;             // Compressed pressure
    uint8_t status;               // Battery, satellites, flags
    uint8_t packet_index;         // LSB of sequence number (0-255)
} LowResTelemetryPacket_t;
```

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
