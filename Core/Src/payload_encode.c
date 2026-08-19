/**
 ******************************************************************************
 * @file    payload_encode.c
 * @brief   Binary payload encoding functions for Stratosonde telemetry
 ******************************************************************************
 * @attention
 *
 * This module implements efficient binary packet encoding for LoRaWAN
 * transmission, converting sensor_t data to compact binary formats.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "SEGGER_RTT.h"
#include "flash_log.h"
#include "mission_state.h"
#include "payload_format.h"
#include "reset_cause.h"
#include "sonde_log.h"     /* R50 (#47): compile-time log gate */
#include "stm32_systime.h" /* R45: SysTimeGet for UTC-disciplined timestamps */
#include "sys_sensors.h"
#include "timer_if.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/* GPS coordinate conversion factors */
#define GPS_BINARY_TO_DEGREES (90.0f / 8388607.0f) // Convert sensor_t binary to degrees
/* DEGREES_TO_100M_RESOLUTION removed — replaced by full-range int16 scaling (see DDR-0019) */
#define LAT_SCALE_FACTOR (32767.0f / 90.0f)  // Maps ±90° to full int16 range (~300m resolution)
#define LON_SCALE_FACTOR (32767.0f / 180.0f) // Maps ±180° to full int16 range (~550m resolution at equator)

/* Compact packet scaling factors */
#define TEMPERATURE_SCALE_FACTOR 2  // 2°C resolution
#define TEMPERATURE_OFFSET 64       // Offset for signed storage in uint8_t
#define BATTERY_SCALE_FACTOR 0.050f // 50mV resolution
#define HUMIDITY_SCALE_FACTOR 5     // 5% resolution

/* Private function prototypes -----------------------------------------------*/
uint16_t Payload_TimestampMinutesNow(void);
static int16_t ConvertLatitudeToCompact(int32_t binary_latitude);
static int16_t ConvertLongitudeToCompact(int32_t binary_longitude);
static int8_t ConvertTemperatureToCompact(float temperature_c);
static uint16_t PackPressureHumidity(float pressure_mbar, float humidity_percent);
static uint8_t ConvertBatteryVoltageToCompact(float voltage_volts);
static uint8_t PackStatusFlags(bool gps_valid, uint8_t satellites, OperatingMode_t power_mode);
static uint16_t CalculateCRC16(const uint8_t *data, uint32_t length);
static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length);

/* STAB-12 (#159): the 45.5-day timestamp wrap latch, hoisted to file scope so
 * the HAL side can persist it (BKP_REG_TS_WRAP) and restore it after reset —
 * a post-wrap reset must not make time look like an earlier epoch. */
static bool s_ts_wrapped = false; /* sticky for the mission */

void Payload_SetTimestampWrapped(bool wrapped) { s_ts_wrapped = wrapped; }
bool Payload_IsTimestampWrapped(void) { return s_ts_wrapped; }

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Encode compact 10-byte telemetry packet
 */
bool EncodeCompactBinaryPacket(CompactTelemetryPacket_t *packet,
                               const void *sensor_data,
                               uint16_t timestamp_min,
                               int16_t voltage_slope,
                               OperatingMode_t power_mode) {
  if (!packet || !sensor_data) {
    return false;
  }

  const sensor_t *sensors = (const sensor_t *)sensor_data;

  SONDE_LOG_STR("Encoding 11-byte compact binary packet...\r\n");

  // Clear packet structure
  memset(packet, 0, sizeof(CompactTelemetryPacket_t));

  /* SP-11/SP-12 (#250): timestamp_min is DATA, verbatim - 0 is a legal wire
   * value (wrap boundary; status bit 5 carries the wrap story). The old
   * 0->fetch-now sentinel silently mixed clock domains: callers passing
   * now_timestamp/60 stamped RTC-uptime (nothing ever sets the RTC
   * calendar), this branch stamped SysTime UTC-epoch. One basis everywhere:
   * callers stamp with Payload_TimestampMinutesNow(). */
  packet->timestamp_min = timestamp_min;

  // Convert GPS coordinates (handle invalid GPS gracefully)
  if (sensors->gnss_valid) {
    packet->latitude_100m = ConvertLatitudeToCompact(sensors->latitude);
    packet->longitude_100m = ConvertLongitudeToCompact(sensors->longitude);
  } else {
    // Use zeros for invalid GPS data
    packet->latitude_100m = 0;
    packet->longitude_100m = 0;
  }

  // Convert environmental sensors
  packet->temperature_2deg = ConvertTemperatureToCompact(sensors->temperature);
  /* D2 (#33): pressure+humidity share one packed u16 — 1 hPa resolution over
   * 0-2046 hPa (stratospheric-useful; v1 collapsed below 950 hPa) */
  packet->press_hum = PackPressureHumidity(sensors->pressure, sensors->humidity);

  // Convert battery voltage (REQUIRED - DevStatusAns is on-demand only)
  packet->battery_volt_50mv = ConvertBatteryVoltageToCompact(sensors->battery_voltage);

  /* Heartbeat v2 status byte (D2/D4, #33):
   * b0 GPS stale, b1 temp stale, b2 humidity stale, b3 pressure stale,
   * b4 RTC GNSS-disciplined (N-03), b5 timestamp_min wrapped (D4),
   * b6-b7 mission state. (v1's condensed reset cause b3-b4 is gone from the
   * wire; it remains available in flash/bulk records.) */
  static uint16_t s_last_ts_min = 0;
  static bool s_baseline_valid = false; /* disciplined baseline established */

  const bool time_gnss_disciplined = sensors->gnss_valid && !sensors->gnss_stale;

  /* R2-14 (#118): only track wraps while GNSS-disciplined. Before the first
   * fix the clock is boot-relative; SysTimeSyncFromGnss then JUMPS it to
   * epoch, and epoch/60 truncated to u16 lands below the pre-sync value
   * ~50% of the time - a discipline transition, not a 45.5-day wrap, and
   * it latched the wrap bit for the whole mission. Re-baseline on the
   * transition to disciplined time instead. */
  if (time_gnss_disciplined) {
    if (s_baseline_valid && timestamp_min < s_last_ts_min) {
      s_ts_wrapped = true; /* real 45.5-day wrap observed */
    }
    s_last_ts_min = timestamp_min;
    s_baseline_valid = true;
  }
  packet->status = (sensors->gnss_stale ? STATUS_GPS_STALE_MASK : 0) | (sensors->temp_stale ? STATUS_TEMP_STALE_MASK : 0) | (sensors->hum_stale ? STATUS_HUM_STALE_MASK : 0) | (sensors->press_stale ? STATUS_PRESS_STALE_MASK : 0) | (time_gnss_disciplined ? STATUS_TIME_GNSS_MASK : 0) | (s_ts_wrapped ? STATUS_TS_WRAP_MASK : 0) | ((MissionState_GetStatusBits() & 0x03) << 6);

  // Debug logging with safe integer conversions
  int32_t lat_micro = sensors->gnss_valid ? (int32_t)(sensors->latitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
  int32_t lon_micro = sensors->gnss_valid ? (int32_t)(sensors->longitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
  int16_t temp_centi = (int16_t)(sensors->temperature * 100);
  int32_t pressure_centi = (int32_t)(sensors->pressure * 100); /* int16 overflowed at >327 hPa (host-test catch) */
  int16_t humidity_centi = (int16_t)(sensors->humidity * 100);
  int16_t battery_mv = (int16_t)(sensors->battery_voltage * 1000);
  (void)temp_centi;
  (void)pressure_centi;
  (void)humidity_centi;
  (void)battery_mv; /* FR-19: log-only in flight */
  (void)lat_micro;
  (void)lon_micro; /* FR-19 */

  /* FR-16 (#97): SONDE_LOG directly — an ungated snprintf still ran in flight */
  SONDE_LOG("Compact: T=%lum Lat=%ld.%06ld Lon=%ld.%06ld Temp=%d.%02dC P=%ld.%02ld H=%d.%02d%% Bat=%dmV Sats=%d Mode=%d\r\n",
            (unsigned long)timestamp_min,
            (long)(lat_micro / 1000000), (long)labs(lat_micro % 1000000),
            (long)(lon_micro / 1000000), (long)labs(lon_micro % 1000000),
            temp_centi / 100, abs(temp_centi % 100),
            (long)(pressure_centi / 100), labs(pressure_centi % 100),
            humidity_centi / 100, abs(humidity_centi % 100),
            battery_mv, sensors->satellites, power_mode);

  return true;
}

/**
 * @brief Encode high-resolution telemetry record for flash storage
 */
bool EncodeHighResTelemetryRecord(HighResTelemetryRecord_t *record,
                                  const void *sensor_data,
                                  uint32_t timestamp,
                                  int16_t voltage_slope,
                                  OperatingMode_t power_mode) {
  if (!record || !sensor_data) {
    return false;
  }

  const sensor_t *sensors = (const sensor_t *)sensor_data;

  SONDE_LOG_STR("Encoding 32-byte high-resolution record...\r\n");

  // Clear record structure
  memset(record, 0, sizeof(HighResTelemetryRecord_t));

  // If timestamp not provided, get current timestamp
  if (timestamp == 0) {
    /* R45: UTC epoch via SysTime (GPS-disciplined delta), not the
     * boot-relative RTC calendar. Falls back to boot-relative before
     * the first fix — honest and distinguishable from epoch. */
    timestamp = SysTimeGet().Seconds; // UTC epoch seconds
  }
  record->timestamp = timestamp;

  // Store GPS coordinates at full precision
  if (sensors->gnss_valid) {
    record->latitude = sensors->latitude;   // Keep original binary format
    record->longitude = sensors->longitude; // Keep original binary format
    record->altitude = (sensors->altitudeGps < 0) ? 0 : (sensors->altitudeGps > 65535) ? 65535
                                                                                       : (uint16_t)sensors->altitudeGps; /* D5/#35: clamp int32 m to wire u16 */
  } else {
    // Use zeros for invalid GPS data
    record->latitude = 0;
    record->longitude = 0;
    record->altitude = 0;
  }

  // Store environmental sensors at high precision
  record->temperature = (int16_t)(sensors->temperature * 10.0f); // 0.1°C resolution
  record->humidity = (uint16_t)(sensors->humidity * 10.0f);      // 0.1% resolution
  record->pressure = (uint16_t)(sensors->pressure * 10.0f);      // 0.1hPa resolution

  // Store voltage data
  record->battery_voltage = (uint16_t)(sensors->battery_voltage * 1000.0f); // mV
  record->solar_voltage = (uint16_t)(sensors->solar_voltage * 1000.0f);     // mV
  record->voltage_slope = voltage_slope;                                    // Already in mV/hour

  // Store GPS metadata
  record->satellites = sensors->satellites;
  record->hdop = (uint8_t)(sensors->gnss_hdop * 10.0f); // HDOP * 10

  // Store power mode
  record->power_mode = (uint8_t)power_mode;

  // Pack status flags
  record->flags = PackStatusFlags(sensors->gnss_valid,
                                  sensors->satellites,
                                  power_mode);

  /* STAB-04 (#151): live-path provenance mirrors the flash-record layout
   * (b0 press, b1 temp, b2 hum, b3 gnss, b4 batt); veto is 0 here — the
   * veto is decided at the flash-write site, not in the encoder. */
  record->sensor_quality =
      (uint8_t)((sensors->press_stale ? 0x01 : 0) |
                (sensors->temp_stale ? 0x02 : 0) |
                (sensors->hum_stale ? 0x04 : 0) |
                (sensors->gnss_stale ? 0x08 : 0) |
                (sensors->batt_stale ? 0x10 : 0));
  record->veto_reason = 0;

  // Calculate CRC16 for data integrity
  record->crc16 = CalculateCRC16((const uint8_t *)record, sizeof(HighResTelemetryRecord_t) - 2);

  // Debug logging with safe integer conversions
  int32_t lat_micro = sensors->gnss_valid ? (int32_t)(sensors->latitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
  int32_t lon_micro = sensors->gnss_valid ? (int32_t)(sensors->longitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
  (void)lat_micro;
  (void)lon_micro; /* FR-19: log-only in flight */

  SONDE_LOG("HighRes: T=%lus Lat=%ld.%06ld Lon=%ld.%06ld Alt=%dm Temp=%d.%dC P=%d.%d Bat=%dmV Solar=%dmV Slope=%+dmV/h\r\n",
            (unsigned long)timestamp,
            (long)(lat_micro / 1000000), (long)labs(lat_micro % 1000000),
            (long)(lon_micro / 1000000), (long)labs(lon_micro % 1000000),
            record->altitude,
            record->temperature / 10, abs(record->temperature % 10),
            record->pressure / 10, record->pressure % 10,
            record->battery_voltage, record->solar_voltage,
            voltage_slope);

  return true;
}

/**
 * @brief Encode a live high-resolution record for COMMISSIONING telemetry
 *        (COMM-TX, DDR-0002 §7 / BR-LIFE-004): identical to
 *        EncodeHighResTelemetryRecord but with the horizontal position
 *        WITHHELD - zeroed after encode, CRC re-sealed so the wire record
 *        is self-consistent. Flags and sensor_quality stay honest: a valid
 *        fix with a withheld position must not masquerade as no fix
 *        (SYS-GNSS-011).
 * @retval bool: true if encoding successful
 */
bool EncodeCommissioningLiveRecord(HighResTelemetryRecord_t *record,
                                   const void *sensor_data,
                                   uint32_t timestamp,
                                   int16_t voltage_slope,
                                   OperatingMode_t power_mode) {
  if (!EncodeHighResTelemetryRecord(record, sensor_data, timestamp,
                                    voltage_slope, power_mode)) {
    return false;
  }
  record->latitude = 0; /* DDR-0002 §7: commissioning X/Y never transmits */
  record->longitude = 0;
  record->crc16 =
      CalculateCRC16((const uint8_t *)record, sizeof(HighResTelemetryRecord_t) - 2);
  return true;
}
/* ---- Explicit little-endian serialization helpers (D9, #33) ----
 * Wire v3 is explicitly LE-serialized per LoRaWANApplicationProtocol.md §3 —
 * no raw struct casts on the new format. */
static void PutU16LE(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}
static void PutU32LE(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

/**
 * @brief Serialize one high-res record as 34 LE bytes (wire v6, STAB-04/#151).
 *        Field order matches HighResTelemetryRecord_t; crc16 covers bytes 0-31.
 * @retval number of bytes written (always 34)
 */
static uint16_t SerializeRecordV3LE(uint8_t *out, const HighResTelemetryRecord_t *r) {
  PutU32LE(out + 0, r->timestamp);
  PutU32LE(out + 4, (uint32_t)r->latitude);
  PutU32LE(out + 8, (uint32_t)r->longitude);
  PutU16LE(out + 12, r->altitude);
  PutU16LE(out + 14, (uint16_t)r->temperature);
  PutU16LE(out + 16, r->humidity);
  PutU16LE(out + 18, r->pressure);
  PutU16LE(out + 20, r->battery_voltage);
  PutU16LE(out + 22, r->solar_voltage);
  PutU16LE(out + 24, (uint16_t)r->voltage_slope);
  out[26] = r->satellites;
  out[27] = r->hdop;
  out[28] = r->power_mode;
  out[29] = r->flags;
  out[30] = r->sensor_quality; /* STAB-04 (#151): v6 provenance */
  out[31] = r->veto_reason;
  PutU16LE(out + 32, CalculateCRC16(out, 32));
  return 34;
}

/**
 * @brief Encode a variable-length bulk packet (wire v6, packet_type 0x06) — STAB-04 (#151)
 *
 * v5 (FR-07/#87) with the 34-byte provenance record: identity is explicit per
 * record (seq u32 LE preceding each record), so a non-contiguous candidate
 * array — corrupt-skip in the FIFO read, or a failed conversion — can no
 * longer misattribute every record after the gap, and the record's
 * sensor_quality/veto_reason provenance survives the archive hop.
 */
bool EncodeBulkPacketV6(uint8_t *buf,
                        uint16_t buf_cap,
                        uint16_t max_payload,
                        const HighResTelemetryRecord_t *records,
                        const uint32_t *record_seqs,
                        uint8_t record_count,
                        uint8_t *packed_count,
                        uint16_t *out_len) {
  if (!buf || !records || !record_seqs || !packed_count || !out_len || record_count == 0) {
    return false;
  }

  /* Whole records only; budget = min(buf_cap, max_payload) */
  uint16_t budget = (max_payload < buf_cap) ? max_payload : buf_cap;
  /* STAB-P3#8 (#243): defend the budget invariant explicitly and zero BOTH
   * outputs on refusal. Today int promotion saves the (budget - OVERHEAD)
   * subtraction from unsigned underflow, but a future type change would
   * turn it into a record count far past the buffer, and the old n==0
   * refusal left *out_len unwritten - caller-visible garbage. */
  if (budget < (uint16_t)(BULK_V6_OVERHEAD + BULK_V6_RECORD_WIRE)) {
    *packed_count = 0;
    *out_len = 0;
    return false;
  }
  uint8_t n = (uint8_t)((budget - BULK_V6_OVERHEAD) / BULK_V6_RECORD_WIRE);
  if (n > record_count)
    n = record_count;
  if (n > BULK_V6_MAX_RECORDS)
    n = BULK_V6_MAX_RECORDS;
  if (n == 0) {
    *packed_count = 0;
    *out_len = 0;
    return false;
  }

  buf[0] = BULK_PACKET_TYPE_V6_PROVENANCE;
  buf[1] = n;
  uint16_t off = 2;
  for (uint8_t i = 0; i < n; i++) {
    PutU32LE(buf + off, record_seqs[i]); /* DDR-0005: explicit identity */
    off += 4;
    off += SerializeRecordV3LE(buf + off, &records[i]);
  }
  PutU32LE(buf + off, CalculateCRC32(buf, off));
  off += 4;

  *packed_count = n;
  *out_len = off;

  SONDE_LOG("Bulk v6: Records=%d FirstSeq=%lu Len=%u\r\n",
            n, (unsigned long)record_seqs[0], off);
  return true;
}

/**
 * @brief Convert FlashLog_Record_t to HighResTelemetryRecord_t
 */
bool ConvertFlashLogToHighRes(const void *flash_record,
                              HighResTelemetryRecord_t *highres_record) {
  if (!flash_record || !highres_record) {
    return false;
  }

  const FlashLog_Record_t *flash_rec = (const FlashLog_Record_t *)flash_record;

  // Clear destination structure
  memset(highres_record, 0, sizeof(HighResTelemetryRecord_t));

  // Copy basic fields
  highres_record->timestamp = flash_rec->timestamp;
  highres_record->latitude = flash_rec->latitude;
  highres_record->longitude = flash_rec->longitude;
  highres_record->altitude = (flash_rec->altitude_gps < 0) ? 0 : (flash_rec->altitude_gps > 65535) ? 65535
                                                                                                   : (uint16_t)flash_rec->altitude_gps; /* D5/#35: clamp int32 */

  // Convert floating point to scaled integers
  highres_record->temperature = (int16_t)(flash_rec->temperature * 10.0f); // 0.1°C
  highres_record->humidity = (uint16_t)(flash_rec->humidity * 10.0f);      // 0.1%
  highres_record->pressure = (uint16_t)(flash_rec->pressure * 10.0f);      // 0.1hPa

  // Battery voltage
  highres_record->battery_voltage = flash_rec->battery_mv;
  /* D5/F-025/R19 (#35): history is self-describing — solar/slope/mode come
   * from the flash record itself (record v4), not from the current cycle's
   * live values passed as parameters. */
  highres_record->solar_voltage = flash_rec->solar_mv;
  highres_record->voltage_slope = flash_rec->voltage_slope;

  // GPS metadata
  highres_record->satellites = flash_rec->satellites;
  highres_record->hdop = flash_rec->gnss_hdop_x10; // Already scaled by 10

  // Power mode and flags (D5/#35: mode from the flash record — honest history)
  highres_record->power_mode = flash_rec->power_mode;
  /* STAB-05 (#152): the flags byte's mode field is the HISTORICAL mode too —
   * packing the live retransmission-time mode here made one record carry two
   * contradictory power states. */
  highres_record->flags = PackStatusFlags((bool)flash_rec->gnss_valid,
                                          flash_rec->satellites,
                                          (OperatingMode_t)flash_rec->power_mode);
  /* STAB-04 (#151): the flash record's data-honesty provenance survives the
   * archive hop — stale bits (b0-b4) and the transmit veto (b5-b7) are
   * decoded on the backend, not silently dropped. */
  highres_record->sensor_quality = flash_rec->flags & 0x1F;
  highres_record->veto_reason = (flash_rec->flags >> 5) & 0x07;

  // Calculate CRC16 for integrity
  highres_record->crc16 = CalculateCRC16((const uint8_t *)highres_record,
                                         sizeof(HighResTelemetryRecord_t) - 2);

  return true;
}

/**
 * @brief Calculate CRC32 for bulk packets
 */
static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFF;

  for (uint32_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x00000001) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc = crc >> 1;
      }
    }
  }

  return ~crc;
}

/**
 * @brief Validate packet structure sizes at compile time
 */
bool PayloadFormat_ValidateSizes(void) {
  bool valid = true;

  // Check packet sizes
  if (sizeof(CompactTelemetryPacket_t) != 11) {
    SONDE_LOG("ERROR: CompactTelemetryPacket_t size = %d bytes (expected 11)\r\n",
              sizeof(CompactTelemetryPacket_t));
    valid = false;
  }

  if (sizeof(HighResTelemetryRecord_t) != 34) { /* v6, STAB-04 (#151) */
    SONDE_LOG("ERROR: HighResTelemetryRecord_t size = %d bytes (expected 34)\r\n",
              sizeof(HighResTelemetryRecord_t));
    valid = false;
  }

  if (valid) {
    SONDE_LOG_STR("Payload format sizes validated: 11/34 bytes\r\n");
  }

  return valid;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Get current timestamp in minutes since Unix epoch
 */
uint16_t Payload_TimestampMinutesNow(void) {
  /* R45: UTC epoch via SysTime (GPS-disciplined), not the boot-relative
   * RTC calendar — same doctrine as the flash write site (#42). */
  uint32_t seconds = SysTimeGet().Seconds;

  /* SP-12 (#250): TIMER_IF seconds are free-running RTC uptime (nothing
   * ever sets the calendar), never UTC - they must not reach the wire.
   * 45.5-day wrap: status bit 5 carries the story. */
  uint32_t minutes = seconds / 60;
  return (uint16_t)(minutes & 0xFFFF);
}

/**
 * @brief Convert binary latitude to compact 100m resolution format
 */
static int16_t ConvertLatitudeToCompact(int32_t binary_latitude) {
  // Convert binary to degrees, then scale to full int16 range (±90°)
  // Ground decoder: lat_deg = encoded × 90 / 32767
  float lat_degrees = binary_latitude * GPS_BINARY_TO_DEGREES;
  int32_t lat_scaled = (int32_t)(lat_degrees * LAT_SCALE_FACTOR);

  // Clamp to int16_t range (maps exactly to ±90°)
  if (lat_scaled > 32767)
    lat_scaled = 32767;
  if (lat_scaled < -32768)
    lat_scaled = -32768;

  return (int16_t)lat_scaled;
}

/**
 * @brief Convert binary longitude to compact 100m resolution format
 */
static int16_t ConvertLongitudeToCompact(int32_t binary_longitude) {
  // Convert binary to degrees, then scale to full int16 range (±180°)
  // Ground decoder: lon_deg = encoded × 180 / 32767
  float lon_degrees = binary_longitude * (180.0f / 8388607.0f);
  int32_t lon_scaled = (int32_t)(lon_degrees * LON_SCALE_FACTOR);

  // Clamp to int16_t range (maps exactly to ±180°)
  if (lon_scaled > 32767)
    lon_scaled = 32767;
  if (lon_scaled < -32768)
    lon_scaled = -32768;

  return (int16_t)lon_scaled;
}

/**
 * @brief Convert temperature to compact format with 2°C resolution
 */
static int8_t ConvertTemperatureToCompact(float temperature_c) {
  /* SP-14 (#251): NaN has no defined comparison, and float->int casts on
   * out-of-range values are UB (observed toolchain-lottery: host x86 and
   * ARM disagree). No wire sentinel exists for temperature - range-bottom
   * (0 raw = -64C) and the status byte's stale bits carry the honesty. */
  if (isnan(temperature_c) || temperature_c < -128.0f) {
    return 0;
  }

  // Scale by 2°C resolution and add offset to handle negative values
  int16_t scaled = (int16_t)(temperature_c / TEMPERATURE_SCALE_FACTOR) + TEMPERATURE_OFFSET;

  // Clamp to int8_t range (0-255) = -64°C to +63°C after offset removal
  if (scaled > 255)
    scaled = 255;
  if (scaled < 0)
    scaled = 0;

  return (int8_t)scaled;
}

/**
 * @brief Pack pressure (11-bit, 1 hPa) + humidity (5-bit, 5%) — D2 (#33)
 * Pressure: 0..2046 hPa valid, sentinel 2047 when out of range/NaN.
 * Humidity: 0..20 (0-100%), sentinel 31 when out of range/NaN.
 */
static uint16_t PackPressureHumidity(float pressure_mbar, float humidity_percent) {
  uint16_t press_units;
  if (isnan(pressure_mbar) || pressure_mbar < 0.0f || pressure_mbar > 2046.0f) {
    press_units = PRESS_HUM_PRESS_INVALID;
  } else {
    press_units = (uint16_t)(pressure_mbar + 0.5f);
    if (press_units > 2046)
      press_units = 2046;
  }

  uint16_t hum_units;
  if (isnan(humidity_percent) || humidity_percent < 0.0f || humidity_percent > 100.0f) {
    hum_units = PRESS_HUM_HUM_INVALID;
  } else {
    hum_units = (uint16_t)(humidity_percent / HUMIDITY_SCALE_FACTOR + 0.5f);
    if (hum_units > 20)
      hum_units = 20;
  }

  return (uint16_t)((hum_units << PRESS_HUM_HUM_SHIFT) | press_units);
}

/**
 * @brief Convert battery voltage to compact format with 50mV resolution
 */
static uint8_t ConvertBatteryVoltageToCompact(float voltage_volts) {
  /* SP-14 (#251): guard NaN/negative BEFORE the float->int cast (UB).
   * 0 units = 0 V is the range bottom; no wire sentinel exists. */
  if (isnan(voltage_volts) || voltage_volts < 0.0f) {
    return 0;
  }

  // Convert to 50mV units
  uint16_t voltage_50mv_units = (uint16_t)(voltage_volts / BATTERY_SCALE_FACTOR);

  // Clamp to uint8_t range (0-255) = 0-12.75V
  if (voltage_50mv_units > 255)
    voltage_50mv_units = 255;

  return (uint8_t)voltage_50mv_units;
}

/**
 * @brief Pack status flags into single byte
 */
static uint8_t PackStatusFlags(bool gps_valid, uint8_t satellites, OperatingMode_t power_mode) {
  uint8_t flags = 0;

  // Set GPS fix valid bit
  SET_GPS_FIX_VALID(flags, gps_valid);

  // Set satellite count (clamp to 15)
  uint8_t sat_count = (satellites > 15) ? 15 : satellites;
  SET_GPS_SATELLITE_COUNT(flags, sat_count);

  // Set power mode (clamp to 7)
  uint8_t mode = ((uint8_t)power_mode > 7) ? 7 : (uint8_t)power_mode;
  SET_POWER_MODE(flags, mode);

  return flags;
}

/**
 * @brief Calculate CRC16 for data integrity
 */
static uint16_t CalculateCRC16(const uint8_t *data, uint32_t length) {
  uint16_t crc = 0xFFFF;

  for (uint32_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }

  return crc;
}
