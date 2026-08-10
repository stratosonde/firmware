/**
  ******************************************************************************
  * @file    payload_format.h
  * @brief   Custom binary payload structures for Stratosonde telemetry
  ******************************************************************************
  * @attention
  *
  * This module defines efficient binary packet formats for LoRaWAN transmission:
  * - CompactTelemetryPacket_t: 11-byte heartbeat packet for SF10 (maximum range w/ LinkCheck)
  * - HighResTelemetryRecord_t: 32-byte record for flash storage  
  * - BulkTelemetryPacket_t: 198-byte packet for SF7 bulk transfer (FW-20)
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
#include "power_model.h"  /* OperatingMode_t (R49: was lora_app.h — much lighter) */

/* Exported defines ----------------------------------------------------------*/

/* LoRaWAN port assignments for different packet types */
#define LORAWAN_LPP_PORT          2   // CayenneLPP (development/debug)
#define LORAWAN_GNSS_DETAIL_PORT  3   // GNSS satellite detail (development/debug) 
#define LORAWAN_COMPACT_PORT      10  // 10-byte compact binary (SF10 probe) - PRODUCTION
#define LORAWAN_BULK_PORT         11  // 198-byte bulk binary (SF7 bulk) - PRODUCTION

/* Compile-time control flags for debug packet formats */
#ifndef ENABLE_DEBUG_LPP
#define ENABLE_DEBUG_LPP           0  // 0 = Default OFF for flight builds (opt-in for debug)
#endif

#ifndef ENABLE_GNSS_DETAIL_PACKET
#define ENABLE_GNSS_DETAIL_PACKET  0  // 0 = Default OFF for flight builds (opt-in for debug)
#endif

#ifndef DEBUG_LPP_TX_INTERVAL
#define DEBUG_LPP_TX_INTERVAL      5  // Send LPP every 5th transmission (reduce airtime)
#endif

/* Status flags bit masks for high-res record flags byte */
#define GPS_FIX_VALID_MASK    0x01  // Bit 0: GPS fix valid
#define GPS_SATS_MASK         0x1E  // Bits 1-4: Satellite count (0-15)
#define POWER_MODE_MASK       0xE0  // Bits 5-7: Power mode (0-7)

/* Status byte (byte 10) bit masks for compact uplink — HEARTBEAT v2 (D2/D4, #33)
 * v2 replaces v1's condensed reset cause (b3-b4) with the GNSS-disciplined-time
 * marker (N-03) and the timestamp wrap/epoch bit (D4). Pressure staleness moves
 * b5 -> b3. Reset cause remains available in the high-res record / flash dump.
 * v1 vs v2 discrimination is by deployment + golden vectors (LoRaWANApplicationProtocol
 * §3/§13): the wire layout changed incompatibly at the same length/port. */
#define STATUS_GPS_STALE_MASK      0x01  // Bit 0: GPS position is last-known-good
#define STATUS_TEMP_STALE_MASK     0x02  // Bit 1: temperature is last-known-good
#define STATUS_HUM_STALE_MASK      0x04  // Bit 2: humidity is last-known-good
#define STATUS_PRESS_STALE_MASK    0x08  // Bit 3: pressure is last-known-good (was b5 in v1)
#define STATUS_TIME_GNSS_MASK      0x10  // Bit 4: RTC disciplined by GNSS this cycle (N-03)
#define STATUS_TS_WRAP_MASK        0x20  // Bit 5: timestamp_min has wrapped 45.5-day range (D4)
#define STATUS_MISSION_STATE_MASK  0xC0  // Bits 6-7: mission state (mission_state.h)

/* Heartbeat wire version (v2 = D2/D4 layout; A-005) */
#define HEARTBEAT_FORMAT_VERSION   2

/* Packed pressure+humidity word (bytes 7-8, little-endian) — D2 (#33):
 * bits 0-10: pressure in 1 hPa units, 0..2046 valid, 2047 = invalid sentinel.
 *            Covers 2-1100 hPa (stratospheric-useful; v1 collapsed below 950 hPa).
 * bits 11-15: humidity in 5% units, 0..20 valid, 31 = invalid sentinel. */
#define PRESS_HUM_PRESS_MASK       0x07FF
#define PRESS_HUM_PRESS_INVALID    0x07FF
#define PRESS_HUM_HUM_SHIFT        11
#define PRESS_HUM_HUM_INVALID      0x1F

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
 * @brief 11-byte compact telemetry packet — HEARTBEAT v2 (D1/D2/D4, #33)
 * @note Same 11-byte budget as v1 (exact fit at US915 DR0), fields little-endian.
 * @note v2 changes: pressure/humidity merged into one packed u16 (stratospheric-
 *       useful 1 hPa over 0-2046 hPa); status b3-b5 repurposed (see masks above).
 * @note Altitude calculated on ground station from pressure + temperature
 */
typedef struct __attribute__((packed)) {
    uint16_t timestamp_min;     // Minutes since mission epoch (2 bytes) - 45.5 day wrap, see STATUS_TS_WRAP
    int16_t  latitude_100m;     // Latitude scaled to full int16 range (2 bytes)
    int16_t  longitude_100m;    // Longitude scaled to full int16 range (2 bytes)
    int8_t   temperature_2deg;  // Temperature / 2°C offset +64 (1 byte) - -64°C to +63°C
    uint16_t press_hum;         // Packed pressure (bits 0-10, 1 hPa) + humidity (bits 11-15, 5%)
    uint8_t  battery_volt_50mv; // Battery voltage / 50mV (1 byte) - 0-12.75V
    uint8_t  status;            // Status byte v2 (1 byte) - stale bits + time markers + mission state
} CompactTelemetryPacket_t;  // Total: 11 bytes (exact fit at US915 DR0, DDR-0019)

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
 * @brief 198-byte bulk telemetry packet for SF7 transmission
 * @note Packs multiple high-resolution records for efficient bulk transfer
 * @note Records are FIFO order (oldest unsent first) since the C4 fix
 *
 * FW-20: packet_type bumped 0x01 -> 0x02 and the packet shrunk 222 -> 198 B.
 * The old v1 layout carried three permanently-zero placeholders
 * (flash_page_addr 4B, voltage_trend 10B, mode_changes 10B = 24 B of
 * SF7 airtime per packet with zero consumers) and pretended to be LIFO.
 * Record identity comes from each HighResTelemetryRecord_t's own
 * timestamp + sequence; if voltage-trend or mode-history telemetry is
 * ever wanted, implement it as a v3 packet with real producers.
 */
typedef struct __attribute__((packed)) {
    uint8_t  packet_type;           // 1 byte - 0x02 = FIFO bulk, no placeholders
    uint8_t  record_count;          // 1 byte - Number of records in this packet

    // High-resolution records (32 bytes each)
    // 198 - 6 header/crc = 192 bytes = 6 complete records per packet
    HighResTelemetryRecord_t records[6];  // 6 × 32 = 192 bytes

    uint32_t crc32;                   // 4 bytes - Packet integrity

} BulkTelemetryPacket_t;  // Total: 198 bytes (FW-20, was 222) — LEGACY, superseded by v3

/* ---- Bulk wire format v4 (D3 + DDR-0005, #33/#34): variable-length, packet_type 0x04 ----
 * Layout: [packet_type=0x04][record_count=n][base_seq u32 LE][n × 32B records][crc32]
 * Length = 6 + 32n + 4. CRC32 (same polynomial as v2) covers everything before it.
 * base_seq is the flash sequence of the FIRST record; records are contiguous FIFO,
 * so record i has identity base_seq + i (DDR-0005 stable archive record IDs —
 * backend dedups on (device, sequence)). The sender packs only complete records
 * and as many as fit the runtime payload budget (LoRaMacQueryTxPossible — current
 * DR + pending FOpts, protocol §11). Decoder branches on payload[0].
 * 0x03 (2+32n+4, no base_seq) shipped <1 day in CI only and never flew — superseded. */
#define BULK_PACKET_TYPE_LEGACY_FIXED   0x02  // BulkTelemetryPacket_t (v2, 198 B fixed)
#define BULK_PACKET_TYPE_V3_SUPERSEDED  0x03  // v3: variable, no record identity (never deployed)
#define BULK_PACKET_TYPE_VARIABLE       0x04  // v4: variable + base_seq identity (superseded by v5, FR-07)
#define BULK_V3_OVERHEAD                10    // type + count + base_seq + crc32
#define BULK_V3_MAX_RECORDS             6     // 198 B worst-case budget parity with v2

/* ---- Bulk wire format v5 (FR-07 option A, #87): per-record explicit identity ----
 * Layout: [packet_type=0x05][record_count=n][n × (seq u32 LE + 32B record)][crc32]
 * Length = 2 + 36n + 4 = 6 + 36n. CRC32 (same polynomial as v2/v4) covers
 * everything before it.
 *
 * v4 derived identity implicitly (base_seq + i), which broke whenever the
 * record array was compacted (corrupt-skip in the FIFO read, or a failed
 * ConvertFlashLogToHighRes). v5 serializes each record's own flash sequence,
 * so identity holds by construction for ANY subset of the archive, and the
 * sender's watermark commit point is simply seq_of(last packed record) + 1.
 * No deployed fleet at adoption time, so no decoder compatibility burden. */
#define BULK_PACKET_TYPE_V5_EXPLICIT    0x05  // v5: variable + per-record sequence
#define BULK_V5_OVERHEAD                6     // type + count + crc32
#define BULK_V5_RECORD_WIRE             36    // seq u32 LE + 32B record
#define BULK_V5_MAX_RECORDS             5     // 6 + 36*5 = 186 <= 198 (v2 parity)

/* Note: OperatingMode_t and VoltageSlope_t are defined in lora_app.h to avoid conflicts */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Encode compact 11-byte heartbeat (v2) telemetry packet
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
 * @brief Encode bulk telemetry packet from flash records (FIFO order)
 * @param packet: Destination bulk packet
 * @param records: Array of high-resolution records (up to 6)
 * @param record_count: Number of records to include
 * @retval bool: true if encoding successful
 * @note  FW-20: flash_page_addr/voltage_trend/mode_changes parameters
 *        removed — they were always-zero placeholders (v1 222 B layout).
 */
bool EncodeBulkPacketFromRecords(BulkTelemetryPacket_t *packet,
                                 const HighResTelemetryRecord_t *records,
                                 uint8_t record_count);

/**
 * @brief Encode a variable-length bulk packet (wire v5, packet_type 0x05) — FR-07 (#87)
 * @param buf: output buffer
 * @param buf_cap: output buffer capacity in bytes
 * @param max_payload: runtime payload budget (LoRaMacQueryTxPossible), bytes
 * @param records: candidate records, FIFO order (need NOT be contiguous in sequence)
 * @param record_seqs: parallel array — each record's own flash sequence (DDR-0005)
 * @param record_count: number of candidate records
 * @param packed_count: out — records actually encoded (<= record_count)
 * @param out_len: out — encoded packet length (6 + 36n)
 * @retval bool: true if at least one record was encoded
 * @note Identity is explicit per record, so skips (corrupt or unconvertible
 *       records) can never corrupt the ground-side (device, seq) dedup key.
 *       The watermark commit point is record_seqs[packed_count-1] + 1.
 */
bool EncodeBulkPacketV5(uint8_t *buf,
                        uint16_t buf_cap,
                        uint16_t max_payload,
                        const HighResTelemetryRecord_t *records,
                        const uint32_t *record_seqs,
                        uint8_t record_count,
                        uint8_t *packed_count,
                        uint16_t *out_len);

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
