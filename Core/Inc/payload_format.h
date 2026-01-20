/**
  ******************************************************************************
  * @file    payload_format.h
  * @brief   Custom binary payload structures for Stratosonde telemetry
  ******************************************************************************
  * @attention
  *
  * This module defines efficient binary packet formats for LoRaWAN transmission:
  * - CompactTelemetryPacket_t: 11-byte packet for SF10 (maximum range)
  * - HighResTelemetryRecord_t: 32-byte record for flash storage  
  * - BulkTelemetryPacket_t: 222-byte packet for SF7 bulk transfer
  *
  ******************************************************************************
  */

#ifndef PAYLOAD_FORMAT_H
#define PAYLOAD_FORMAT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "lora_app.h"

/* Exported defines ----------------------------------------------------------*/

/* LoRaWAN port assignments for different packet types */
#define LORAWAN_LPP_PORT          2   // CayenneLPP (development/debug)
#define LORAWAN_GNSS_DETAIL_PORT  3   // GNSS satellite detail (development/debug) 
#define LORAWAN_COMPACT_PORT      10  // 11-byte compact binary (SF10 probe) - PRODUCTION
#define LORAWAN_BULK_PORT         11  // 222-byte bulk binary (SF7 bulk) - PRODUCTION

/* Compile-time control flags for debug packet formats */
#ifndef ENABLE_DEBUG_LPP
#define ENABLE_DEBUG_LPP           1  // 1 = Enable CayenneLPP on port 2, 0 = Disable
#endif

#ifndef ENABLE_GNSS_DETAIL_PACKET
#define ENABLE_GNSS_DETAIL_PACKET  1  // 1 = Enable GNSS detail on port 3, 0 = Disable
#endif

#ifndef DEBUG_LPP_TX_INTERVAL
#define DEBUG_LPP_TX_INTERVAL      5  // Send LPP every 5th transmission (reduce airtime)
#endif

/* Status flags bit masks for compact packet */
#define GPS_FIX_VALID_MASK    0x01  // Bit 0: GPS fix valid
#define GPS_SATS_MASK         0x1E  // Bits 1-4: Satellite count (0-15)
#define POWER_MODE_MASK       0xE0  // Bits 5-7: Power mode (0-7)

/* Helper macros for status flag extraction */
#define GET_GPS_FIX_VALID(flags)    (((flags) & GPS_FIX_VALID_MASK) != 0)
#define GET_GPS_SATELLITE_COUNT(flags) (((flags) & GPS_SATS_MASK) >> 1)
#define GET_POWER_MODE(flags)       (((flags) & POWER_MODE_MASK) >> 5)

/* Helper macros for status flag packing */
#define SET_GPS_FIX_VALID(flags, valid)     do { \
    if (valid) (flags) |= GPS_FIX_VALID_MASK; \
    else (flags) &= ~GPS_FIX_VALID_MASK; \
} while(0)

#define SET_GPS_SATELLITE_COUNT(flags, count) do { \
    (flags) = ((flags) & ~GPS_SATS_MASK) | (((count) & 0x0F) << 1); \
} while(0)

#define SET_POWER_MODE(flags, mode) do { \
    (flags) = ((flags) & ~POWER_MODE_MASK) | (((mode) & 0x07) << 5); \
} while(0)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 11-byte compact telemetry packet (SF10 maximum payload for US915)
 * @note Optimized for maximum range transmission at SF10
 * @note Includes battery voltage (required since DevStatusAns is on-demand only)
 * @note Altitude calculated on ground station from pressure + temperature
 */
typedef struct __attribute__((packed)) {
    uint16_t timestamp_min;     // Minutes since epoch (2 bytes) - 45 days range
    int16_t  latitude_100m;     // Latitude in 100m resolution (2 bytes) - ±3276.7 km
    int16_t  longitude_100m;    // Longitude in 100m resolution (2 bytes) - ±3276.7 km  
    int8_t   temperature_2deg;  // Temperature / 2°C offset +64 (1 byte) - -64°C to +63°C
    uint8_t  pressure_10hPa;    // Pressure / 10hPa from 950hPa base (1 byte) - 950-2500 hPa
    uint8_t  battery_volt_50mv; // Battery voltage / 50mV (1 byte) - 0-12.75V
    uint8_t  humidity_5pct;     // Humidity / 5% resolution (1 byte) - 0-100% in 20 steps
    uint8_t  status_flags;      // GPS fix + power mode + satellites (1 byte)
} CompactTelemetryPacket_t;  // Total: 11 bytes (maximum SF10 US915 payload)

/**
 * @brief High-resolution telemetry record for flash storage
 * @note Full precision data stored locally for later bulk transmission
 * @note 32-byte aligned size for efficient flash operations
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;         // Full precision timestamp (4 bytes)
    int32_t  latitude;          // Full GPS precision (4 bytes) - 1e-7 degree resolution
    int32_t  longitude;         // Full GPS precision (4 bytes) - 1e-7 degree resolution
    uint16_t altitude;          // Meters (2 bytes) - 0-65535m
    int16_t  temperature;       // 0.1°C resolution (2 bytes) - -3276.8 to +3276.7°C
    uint16_t humidity;          // 0.1% resolution (2 bytes) - 0-655.35%
    uint16_t pressure;          // 0.1hPa resolution (2 bytes) - 0-6553.5 hPa
    uint16_t battery_voltage;   // mV (2 bytes) - 0-65.535V
    uint16_t solar_voltage;     // mV (2 bytes) - 0-65.535V
    int16_t  voltage_slope;     // mV/hour (2 bytes) - ±32.767 V/hour
    uint8_t  satellites;        // GPS satellite count (1 byte)
    uint8_t  hdop;             // GPS HDOP * 10 (1 byte) - 0-25.5 HDOP
    uint8_t  power_mode;       // Operating mode enum (1 byte)
    uint8_t  flags;            // Status flags (1 byte)
    uint16_t crc16;            // Data integrity (2 bytes)
} HighResTelemetryRecord_t;   // Total: 32 bytes

/**
 * @brief 222-byte bulk telemetry packet for SF7 transmission
 * @note Packs multiple high-resolution records for efficient bulk transfer
 * @note LIFO order (newest records first)
 */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;           // 1 byte - Format version/type identifier
    uint8_t  record_count;          // 1 byte - Number of records in this packet
    uint32_t flash_page_addr;       // 4 bytes - Source flash page (for LIFO tracking)
    
    // High-resolution records (32 bytes each)
    // 222 - 6 header = 216 bytes available
    // 216 ÷ 32 = 6 complete records per packet
    HighResTelemetryRecord_t records[6];  // 6 × 32 = 192 bytes
    
    // Metadata/padding (222 - 198 = 24 bytes remaining)
    uint8_t  voltage_trend[10];       // 10 bytes - recent voltage samples
    uint8_t  mode_changes[10];        // 10 bytes - power mode history  
    uint32_t crc32;                   // 4 bytes - Packet integrity
    
} BulkTelemetryPacket_t;  // Total: 222 bytes (SF7/US915 maximum)

/* Note: OperatingMode_t and VoltageSlope_t are defined in lora_app.h to avoid conflicts */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Encode compact 11-byte telemetry packet
 * @param packet: Destination packet structure
 * @param sensor_data: Source sensor data
 * @param timestamp_min: Timestamp in minutes since epoch
 * @param voltage_slope: Voltage slope in mV/hour
 * @param power_mode: Current operating mode
 * @retval bool: true if encoding successful
 */
bool EncodeCompactBinaryPacket(CompactTelemetryPacket_t *packet, 
                               const void *sensor_data,
                               uint16_t timestamp_min,
                               int16_t voltage_slope,
                               OperatingMode_t power_mode);

/**
 * @brief Encode high-resolution telemetry record for flash storage
 * @param record: Destination record structure  
 * @param sensor_data: Source sensor data
 * @param timestamp: Full precision timestamp
 * @param voltage_slope: Voltage slope in mV/hour
 * @param power_mode: Current operating mode
 * @retval bool: true if encoding successful
 */
bool EncodeHighResTelemetryRecord(HighResTelemetryRecord_t *record,
                                  const void *sensor_data,
                                  uint32_t timestamp,
                                  int16_t voltage_slope,
                                  OperatingMode_t power_mode);

/**
 * @brief Encode bulk telemetry packet from flash records (LIFO order)
 * @param packet: Destination bulk packet
 * @param records: Array of high-resolution records (up to 6)
 * @param record_count: Number of records to include
 * @param flash_page_addr: Source flash page address
 * @param voltage_trend: Recent voltage samples (10 bytes)
 * @param mode_changes: Power mode history (10 bytes)
 * @retval bool: true if encoding successful
 */
bool EncodeBulkPacketFromRecords(BulkTelemetryPacket_t *packet,
                                 const HighResTelemetryRecord_t *records,
                                 uint8_t record_count,
                                 uint32_t flash_page_addr,
                                 const uint8_t *voltage_trend,
                                 const uint8_t *mode_changes);

/**
 * @brief Decode compact telemetry packet (for ground station)
 * @param packet: Source compact packet
 * @param decoded_data: Destination structure for decoded data
 * @retval bool: true if decoding successful
 */
bool DecodeCompactBinaryPacket(const CompactTelemetryPacket_t *packet,
                               void *decoded_data);

/**
 * @brief Convert FlashLog_Record_t to HighResTelemetryRecord_t format
 * @param flash_record: Source flash log record
 * @param highres_record: Destination high-res record  
 * @param voltage_slope: Voltage slope in mV/hour
 * @param power_mode: Current operating mode
 * @retval bool: true if conversion successful
 * @note Converts floating point flash data to scaled integer format
 */
bool ConvertFlashLogToHighRes(const void *flash_record,
                              HighResTelemetryRecord_t *highres_record,
                              int16_t voltage_slope,
                              OperatingMode_t power_mode);

/**
 * @brief Validate packet structure sizes at compile time
 * @note Call during initialization to verify packet sizes are correct
 * @retval bool: true if all packet sizes are valid
 */
bool PayloadFormat_ValidateSizes(void);

#ifdef __cplusplus
}
#endif

#endif /* PAYLOAD_FORMAT_H */
