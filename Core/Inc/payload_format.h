/**
 ******************************************************************************
 * @file    payload_format.h
 * @brief   Custom binary payload structures for Stratosonde telemetry
 ******************************************************************************
 * @attention
 *
 * This module defines efficient binary packet formats for LoRaWAN transmission:
 * - CompactTelemetryPacket_t: 11-byte heartbeat packet for SF10 (maximum range w/ LinkCheck)
 * - HighResTelemetryRecord_t: 34-byte record (v6, STAB-04/#151) for archive transfer
 * - Bulk wire format: variable-length v6 packets for SF7 archive bursts (0x06)
 *
 ******************************************************************************
 */

#ifndef PAYLOAD_FORMAT_H
#define PAYLOAD_FORMAT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "power_model.h" /* OperatingMode_t (R49: was lora_app.h — much lighter) */
#include <stdbool.h>
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

/* LoRaWAN port assignments for different packet types */
#define LORAWAN_LPP_PORT 2         // CayenneLPP (development/debug)
#define LORAWAN_GNSS_DETAIL_PORT 3 // GNSS satellite detail (development/debug)
#define LORAWAN_COMPACT_PORT 10    // 11-byte compact binary (SF10 probe) - PRODUCTION
#define LORAWAN_BULK_PORT 11       // variable bulk binary (SF7 bulk) - PRODUCTION
#define LORAWAN_VERSION_PORT 20    // version report frame (A-005/#79) - PRODUCTION

/* Compile-time control flags for debug packet formats */
#ifndef ENABLE_DEBUG_LPP
#define ENABLE_DEBUG_LPP 0 // 0 = Default OFF for flight builds (opt-in for debug)
#endif

#ifndef ENABLE_GNSS_DETAIL_PACKET
#define ENABLE_GNSS_DETAIL_PACKET 0 // 0 = Default OFF for flight builds (opt-in for debug)
#endif

#ifndef DEBUG_LPP_TX_INTERVAL
#define DEBUG_LPP_TX_INTERVAL 5 // Send LPP every 5th transmission (reduce airtime)
#endif

/* Status flags bit masks for high-res record flags byte */
#define GPS_FIX_VALID_MASK 0x01 // Bit 0: GPS fix valid
#define GPS_SATS_MASK 0x1E      // Bits 1-4: Satellite count (0-15)
#define POWER_MODE_MASK 0xE0    // Bits 5-7: Power mode (0-7)

/* Status byte (byte 10) bit masks for compact uplink — HEARTBEAT v2 (D2/D4, #33)
 * v2 replaces v1's condensed reset cause (b3-b4) with the GNSS-disciplined-time
 * marker (N-03) and the timestamp wrap/epoch bit (D4). Pressure staleness moves
 * b5 -> b3. Reset cause remains available in the high-res record / flash dump.
 * v1 vs v2 discrimination is by deployment + golden vectors (LoRaWANApplicationProtocol
 * §3/§13): the wire layout changed incompatibly at the same length/port. */
#define STATUS_GPS_STALE_MASK 0x01     // Bit 0: GPS position is last-known-good
#define STATUS_TEMP_STALE_MASK 0x02    // Bit 1: temperature is last-known-good
#define STATUS_HUM_STALE_MASK 0x04     // Bit 2: humidity is last-known-good
#define STATUS_PRESS_STALE_MASK 0x08   // Bit 3: pressure is last-known-good (was b5 in v1)
#define STATUS_TIME_GNSS_MASK 0x10     // Bit 4: RTC disciplined by GNSS this cycle (N-03)
#define STATUS_TS_WRAP_MASK 0x20       // Bit 5: timestamp_min has wrapped 45.5-day range (D4)
#define STATUS_MISSION_STATE_MASK 0xC0 // Bits 6-7: mission state (mission_state.h)

/* Heartbeat wire version (v2 = D2/D4 layout; A-005) */
#define HEARTBEAT_FORMAT_VERSION 2

/* Packed pressure+humidity word (bytes 7-8, little-endian) — D2 (#33):
 * bits 0-10: pressure in 1 hPa units, 0..2046 valid, 2047 = invalid sentinel.
 *            Covers 2-1100 hPa (stratospheric-useful; v1 collapsed below 950 hPa).
 * bits 11-15: humidity in 5% units, 0..20 valid, 31 = invalid sentinel. */
#define PRESS_HUM_PRESS_MASK 0x07FF
#define PRESS_HUM_PRESS_INVALID 0x07FF
#define PRESS_HUM_HUM_SHIFT 11
#define PRESS_HUM_HUM_INVALID 0x1F

/* Helper macros for status flag extraction */
#define GET_GPS_FIX_VALID(flags) (((flags) & GPS_FIX_VALID_MASK) != 0)
#define GET_GPS_SATELLITE_COUNT(flags) (((flags) & GPS_SATS_MASK) >> 1)
#define GET_POWER_MODE(flags) (((flags) & POWER_MODE_MASK) >> 5)

/* Helper macros for status flag packing.
 * Finding #9: explicit (uint8_t) casts on the results — ~MASK promotes to a
 * negative int and the narrowing assignment tripped 6 x -Wsign-conversion
 * warnings (benign but they mask real ones). Behavior is unchanged: the cast
 * truncates exactly the way the implicit conversion did. */
#define SET_GPS_FIX_VALID(flags, valid)                            \
  do {                                                             \
    if (valid)                                                     \
      (flags) = (uint8_t)((flags) | GPS_FIX_VALID_MASK);           \
    else                                                           \
      (flags) = (uint8_t)((flags) & (uint8_t)~GPS_FIX_VALID_MASK); \
  } while (0)

#define SET_GPS_SATELLITE_COUNT(flags, count)                                           \
  do {                                                                                  \
    (flags) = (uint8_t)(((flags) & (uint8_t)~GPS_SATS_MASK) | (((count) & 0x0F) << 1)); \
  } while (0)

#define SET_POWER_MODE(flags, mode)                                                      \
  do {                                                                                   \
    (flags) = (uint8_t)(((flags) & (uint8_t)~POWER_MODE_MASK) | (((mode) & 0x07) << 5)); \
  } while (0)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 11-byte compact telemetry packet — HEARTBEAT v2 (D1/D2/D4, #33)
 * @note Same 11-byte budget as v1 (exact fit at US915 DR0), fields little-endian.
 * @note v2 changes: pressure/humidity merged into one packed u16 (stratospheric-
 *       useful 1 hPa over 0-2046 hPa); status b3-b5 repurposed (see masks above).
 * @note Altitude calculated on ground station from pressure + temperature
 */
typedef struct __attribute__((packed)) {
  uint16_t timestamp_min;    // Minutes since mission epoch (2 bytes) - 45.5 day wrap, see STATUS_TS_WRAP
  int16_t latitude_100m;     // Latitude scaled to full int16 range (2 bytes)
  int16_t longitude_100m;    // Longitude scaled to full int16 range (2 bytes)
  uint8_t temperature_2deg;  // Temperature / 2°C offset +64 (1 byte) - unsigned 0-255 -> -64°C to +63.5°C
  uint16_t press_hum;        // Packed pressure (bits 0-10, 1 hPa) + humidity (bits 11-15, 5%)
  uint8_t battery_volt_50mv; // Battery voltage / 50mV (1 byte) - 0-12.75V
  uint8_t status;            // Status byte v2 (1 byte) - stale bits + time markers + mission state
} CompactTelemetryPacket_t;  // Total: 11 bytes (exact fit at US915 DR0, DDR-0019)

/**
 * @brief High-resolution telemetry record for flash storage
 * @note Full precision data stored locally for later bulk transmission
 * @note 32-byte aligned size for efficient flash operations
 */
typedef struct __attribute__((packed)) {
  uint32_t timestamp;       // Full precision timestamp (4 bytes)
  int32_t latitude;         // sensor_t binary format: deg = raw * 90 / 8388607  (4 bytes)
  int32_t longitude;        // sensor_t binary format: deg = raw * 180 / 8388607 (4 bytes)
  uint16_t altitude;        // Meters (2 bytes) - 0-65535m
  int16_t temperature;      // 0.1°C resolution (2 bytes) - -3276.8 to +3276.7°C
  uint16_t humidity;        // 0.1% resolution (2 bytes) - 0-655.35%
  uint16_t pressure;        // 0.1hPa resolution (2 bytes) - 0-6553.5 hPa
  uint16_t battery_voltage; // mV (2 bytes) - 0-65.535V
  uint16_t solar_voltage;   // mV (2 bytes) - 0-65.535V
  int16_t voltage_slope;    // mV/hour (2 bytes) - ±32.767 V/hour
  uint8_t satellites;       // GPS satellite count (1 byte) - AUTHORITATIVE (flags b1-b4 is a redundant copy)
  uint8_t hdop;             // GPS HDOP * 10 (1 byte) - 0-25.5 HDOP
  uint8_t power_mode;       // Operating mode enum (1 byte) - AUTHORITATIVE (flags b5-b7 is a redundant copy)
  uint8_t flags;            // Redundant copy of sats (b1-b4) + power mode (b5-b7); b0 gps_fix_valid. Prefer fields above.
  uint8_t sensor_quality;   // STAB-04 (#151): b0 press_stale, b1 temp_stale, b2 hum_stale,
                            // b3 gnss_stale, b4 batt_stale, b5-b7 reserved
  uint8_t veto_reason;      // STAB-04 (#151): TransmitVeto_t at write time (0 = none)
  uint16_t crc16;           // Data integrity (2 bytes)
} HighResTelemetryRecord_t; // Total: 34 bytes (v6, was 32)

/* ---- Historical bulk v2 (packet_type 0x02): 198 B fixed, 6 x 32B records ----
 * The v2 struct/encoder (BulkTelemetryPacket_t, EncodeBulkPacketFromRecords)
 * were deleted at the v6 record change: the layout documented here is FROZEN
 * history — [0x02][count][6 x 32B records][crc32] = 198 B. Nothing decodes or
 * encodes it anymore; it exists so old captures remain interpretable. */

/* ---- Bulk wire format v4 (D3 + DDR-0005, #33/#34): variable-length, packet_type 0x04 ----
 * Layout: [packet_type=0x04][record_count=n][base_seq u32 LE][n × 32B records][crc32]
 * Length = 6 + 32n + 4. CRC32 (same polynomial as v2) covers everything before it.
 * base_seq is the flash sequence of the FIRST record; records are contiguous FIFO,
 * so record i has identity base_seq + i (DDR-0005 stable archive record IDs —
 * backend dedups on (device, sequence)). The sender packs only complete records
 * and as many as fit the runtime payload budget (LoRaMacQueryTxPossible — current
 * DR + pending FOpts, protocol §11). Decoder branches on payload[0].
 * 0x03 (2+32n+4, no base_seq) shipped <1 day in CI only and never flew — superseded. */
#define BULK_PACKET_TYPE_LEGACY_FIXED 0x02  // BulkTelemetryPacket_t (v2, 198 B fixed)
#define BULK_PACKET_TYPE_V3_SUPERSEDED 0x03 // v3: variable, no record identity (never deployed)
#define BULK_PACKET_TYPE_VARIABLE 0x04      // v4: variable + base_seq identity (superseded by v5, FR-07)
#define BULK_V3_OVERHEAD 10                 // type + count + base_seq + crc32
#define BULK_V3_MAX_RECORDS 6               // 198 B worst-case budget parity with v2

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
 * v5 NEVER FLEW — superseded by v6 before the first real burst (#146).
 *
 * ---- Bulk wire format v6 (STAB-04/#151 + STAB-05/#152): provenance ----
 * Same envelope as v5 but the record is the 34-byte v6 layout (adds
 * sensor_quality + veto_reason): [0x06][n][n × (seq u32 LE + 34B)][crc32].
 * Length = 6 + 38n. A historical record's data-honesty bits (stale sensors,
 * transmit veto) survive the archive hop; unknown/stale can no longer arrive
 * looking fresh (DDR-0007). Type bumped so no decoder can misparse size. */
#define BULK_PACKET_TYPE_V6_PROVENANCE 0x06 // v6: variable + per-record seq + provenance
#define BULK_V6_OVERHEAD 6                  // type + count + crc32
#define BULK_V6_RECORD_WIRE 38              // seq u32 LE + 34B record
#define BULK_V6_MAX_RECORDS 5               // 6 + 38*5 = 196 <= 198 (v2 parity)

/* Note: OperatingMode_t and VoltageSlope_t are defined in lora_app.h to avoid conflicts */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief STAB-12 (#159): timestamp-wrap latch persistence hooks. The HAL side
 *        restores the latch from BKP_REG_TS_WRAP at boot and persists it when
 *        the encode path first reports wrapped.
 */
void Payload_SetTimestampWrapped(bool wrapped);

/**
 * @brief  Current UTC timestamp in wire units (minutes since Unix epoch,
 *         wrap-truncated to 16 bits - 45.5-day range, status bit 5 marks it).
 *         SP-11/SP-12 (#250): the ONLY path allowed to stamp the compact
 *         packet's timestamp; RTC-uptime seconds must not reach the wire.
 */
uint16_t Payload_TimestampMinutesNow(void);
bool Payload_IsTimestampWrapped(void);

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
 * @brief Serialize the 11-byte compact heartbeat as explicit little-endian bytes
 *        (WIRE ROBUSTNESS - replaces shipping the raw packed struct, whose
 *        endianness was MCU-native; output byte-identical on little-endian).
 * @param out: buffer of at least 11 bytes
 * @param p:   source compact packet
 * @retval number of bytes written (always 11)
 */
uint16_t SerializeCompactLE(uint8_t *out, const CompactTelemetryPacket_t *p);

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
 * @brief Encode a live high-resolution record for COMMISSIONING telemetry
 *        (COMM-TX, DDR-0002 §7 / BR-LIFE-004): full fidelity EXCEPT the
 *        horizontal position - withheld (zeroed), CRC16 re-sealed.
 * @retval bool: true if encoding successful
 */
bool EncodeCommissioningLiveRecord(HighResTelemetryRecord_t *record,
                                   const void *sensor_data,
                                   uint32_t timestamp,
                                   int16_t voltage_slope,
                                   OperatingMode_t power_mode);

/**
 * @brief Encode a variable-length bulk packet (wire v6, packet_type 0x06) — STAB-04 (#151)
 * @param buf: output buffer
 * @param buf_cap: output buffer capacity in bytes
 * @param max_payload: runtime payload budget (LoRaMacQueryTxPossible), bytes
 * @param records: candidate records, FIFO order (need NOT be contiguous in sequence)
 * @param record_seqs: parallel array — each record's own flash sequence (DDR-0005)
 * @param record_count: number of candidate records
 * @param packed_count: out — records actually encoded (<= record_count)
 * @param out_len: out — encoded packet length (6 + 38n)
 * @retval bool: true if at least one record was encoded
 * @note Identity is explicit per record, so skips (corrupt or unconvertible
 *       records) can never corrupt the ground-side (device, seq) dedup key.
 *       The watermark commit point is record_seqs[packed_count-1] + 1.
 */
bool EncodeBulkPacketV6(uint8_t *buf,
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
 * @retval bool: true if conversion successful
 * @note Converts floating point flash data to scaled integer format.
 *       STAB-05 (#152): no live-context parameters — every field of a
 *       historical record describes acquisition time (slope/mode come from
 *       the record; STAB-04 (#151): sensor_quality + veto_reason copied from
 *       the flash record's data-honesty flags).
 */
bool ConvertFlashLogToHighRes(const void *flash_record,
                              HighResTelemetryRecord_t *highres_record);

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
