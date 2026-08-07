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
#include "payload_format.h"
#include "sys_sensors.h"
#include "timer_if.h"
#include "stm32_systime.h"  /* R45: SysTimeGet for UTC-disciplined timestamps */
#include "reset_cause.h"
#include "mission_state.h"
#include "SEGGER_RTT.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "flash_log.h"

/* Private defines -----------------------------------------------------------*/

/* GPS coordinate conversion factors */
#define GPS_BINARY_TO_DEGREES       (90.0f / 8388607.0f)  // Convert sensor_t binary to degrees
/* DEGREES_TO_100M_RESOLUTION removed — replaced by full-range int16 scaling (see DDR-0005) */
#define LAT_SCALE_FACTOR  (32767.0f / 90.0f)   // Maps ±90° to full int16 range (~300m resolution)
#define LON_SCALE_FACTOR  (32767.0f / 180.0f)  // Maps ±180° to full int16 range (~550m resolution at equator)

/* Compact packet scaling factors */
#define TEMPERATURE_SCALE_FACTOR    2     // 2°C resolution
#define TEMPERATURE_OFFSET          64    // Offset for signed storage in uint8_t
#define BATTERY_SCALE_FACTOR        0.050f // 50mV resolution
#define HUMIDITY_SCALE_FACTOR       5     // 5% resolution

/* Private function prototypes -----------------------------------------------*/
static uint16_t GetTimestampMinutes(void);
static int16_t ConvertLatitudeToCompact(int32_t binary_latitude);
static int16_t ConvertLongitudeToCompact(int32_t binary_longitude);
static int8_t ConvertTemperatureToCompact(float temperature_c);
static uint16_t PackPressureHumidity(float pressure_mbar, float humidity_percent);
static uint8_t ConvertBatteryVoltageToCompact(float voltage_volts);
static uint8_t PackStatusFlags(bool gps_valid, uint8_t satellites, OperatingMode_t power_mode);
static uint16_t CalculateCRC16(const uint8_t *data, uint32_t length);
static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Encode compact 10-byte telemetry packet
 */
bool EncodeCompactBinaryPacket(CompactTelemetryPacket_t *packet,
                               const void *sensor_data,
                               uint16_t timestamp_min,
                               int16_t voltage_slope,
                               OperatingMode_t power_mode)
{
    if (!packet || !sensor_data) {
        return false;
    }
    
    const sensor_t *sensors = (const sensor_t*)sensor_data;
    
    SEGGER_RTT_WriteString(0, "Encoding 11-byte compact binary packet...\r\n");
    
    // Clear packet structure
    memset(packet, 0, sizeof(CompactTelemetryPacket_t));
    
    // If timestamp not provided, get current timestamp
    if (timestamp_min == 0) {
        timestamp_min = GetTimestampMinutes();
    }
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
    static bool s_ts_wrapped = false;      /* sticky for the mission */
    static uint16_t s_last_ts_min = 0;
    if (timestamp_min < s_last_ts_min) {
        s_ts_wrapped = true;               /* 45.5-day wrap observed */
    }
    s_last_ts_min = timestamp_min;

    const bool time_gnss_disciplined = sensors->gnss_valid && !sensors->gnss_stale;
    packet->status = (sensors->gnss_stale ? STATUS_GPS_STALE_MASK : 0)
                   | (sensors->temp_stale ? STATUS_TEMP_STALE_MASK : 0)
                   | (sensors->hum_stale  ? STATUS_HUM_STALE_MASK : 0)
                   | (sensors->press_stale ? STATUS_PRESS_STALE_MASK : 0)
                   | (time_gnss_disciplined ? STATUS_TIME_GNSS_MASK : 0)
                   | (s_ts_wrapped ? STATUS_TS_WRAP_MASK : 0)
                   | ((MissionState_GetStatusBits() & 0x03) << 6);
    
    // Debug logging with safe integer conversions
    int32_t lat_micro = sensors->gnss_valid ? (int32_t)(sensors->latitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
    int32_t lon_micro = sensors->gnss_valid ? (int32_t)(sensors->longitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
    int16_t temp_centi = (int16_t)(sensors->temperature * 100);
    int32_t pressure_centi = (int32_t)(sensors->pressure * 100);  /* int16 overflowed at >327 hPa (host-test catch) */
    int16_t humidity_centi = (int16_t)(sensors->humidity * 100);
    int16_t battery_mv = (int16_t)(sensors->battery_voltage * 1000);
    
    char debug_msg[200];
    snprintf(debug_msg, sizeof(debug_msg),
             "Compact: T=%lum Lat=%ld.%06ld Lon=%ld.%06ld Temp=%d.%02dC P=%ld.%02ld H=%d.%02d%% Bat=%dmV Sats=%d Mode=%d\r\n",
             (unsigned long)timestamp_min,
             (long)(lat_micro / 1000000), (long)labs(lat_micro % 1000000),
             (long)(lon_micro / 1000000), (long)labs(lon_micro % 1000000),
             temp_centi / 100, abs(temp_centi % 100),
             (long)(pressure_centi / 100), labs(pressure_centi % 100),
             humidity_centi / 100, abs(humidity_centi % 100),
             battery_mv, sensors->satellites, power_mode);
    SEGGER_RTT_WriteString(0, debug_msg);
    
    return true;
}

/**
 * @brief Encode high-resolution telemetry record for flash storage
 */
bool EncodeHighResTelemetryRecord(HighResTelemetryRecord_t *record,
                                  const void *sensor_data,
                                  uint32_t timestamp,
                                  int16_t voltage_slope,
                                  OperatingMode_t power_mode)
{
    if (!record || !sensor_data) {
        return false;
    }
    
    const sensor_t *sensors = (const sensor_t*)sensor_data;
    
    SEGGER_RTT_WriteString(0, "Encoding 32-byte high-resolution record...\r\n");
    
    // Clear record structure
    memset(record, 0, sizeof(HighResTelemetryRecord_t));
    
    // If timestamp not provided, get current timestamp
    if (timestamp == 0) {
        /* R45: UTC epoch via SysTime (GPS-disciplined delta), not the
         * boot-relative RTC calendar. Falls back to boot-relative before
         * the first fix — honest and distinguishable from epoch. */
        timestamp = SysTimeGet().Seconds;  // UTC epoch seconds
    }
    record->timestamp = timestamp;
    
    // Store GPS coordinates at full precision
    if (sensors->gnss_valid) {
        record->latitude = sensors->latitude;    // Keep original binary format
        record->longitude = sensors->longitude;  // Keep original binary format
        record->altitude = (sensors->altitudeGps < 0) ? 0 :
                           (sensors->altitudeGps > 65535) ? 65535 :
                           (uint16_t)sensors->altitudeGps;  /* D5/#35: clamp int32 m to wire u16 */
    } else {
        // Use zeros for invalid GPS data
        record->latitude = 0;
        record->longitude = 0;
        record->altitude = 0;
    }
    
    // Store environmental sensors at high precision
    record->temperature = (int16_t)(sensors->temperature * 10.0f);  // 0.1°C resolution
    record->humidity = (uint16_t)(sensors->humidity * 10.0f);       // 0.1% resolution
    record->pressure = (uint16_t)(sensors->pressure * 10.0f);       // 0.1hPa resolution
    
    // Store voltage data
    record->battery_voltage = (uint16_t)(sensors->battery_voltage * 1000.0f);  // mV
    record->solar_voltage = (uint16_t)(sensors->solar_voltage * 1000.0f);      // mV
    record->voltage_slope = voltage_slope;  // Already in mV/hour
    
    // Store GPS metadata
    record->satellites = sensors->satellites;
    record->hdop = (uint8_t)(sensors->gnss_hdop * 10.0f);  // HDOP * 10
    
    // Store power mode
    record->power_mode = (uint8_t)power_mode;
    
    // Pack status flags
    record->flags = PackStatusFlags(sensors->gnss_valid, 
                                    sensors->satellites, 
                                    power_mode);
    
    // Calculate CRC16 for data integrity
    record->crc16 = CalculateCRC16((const uint8_t*)record, sizeof(HighResTelemetryRecord_t) - 2);
    
    // Debug logging with safe integer conversions
    int32_t lat_micro = sensors->gnss_valid ? (int32_t)(sensors->latitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
    int32_t lon_micro = sensors->gnss_valid ? (int32_t)(sensors->longitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
    
    char debug_msg[200];
    snprintf(debug_msg, sizeof(debug_msg),
             "HighRes: T=%lus Lat=%ld.%06ld Lon=%ld.%06ld Alt=%dm Temp=%d.%dC P=%d.%d Bat=%dmV Solar=%dmV Slope=%+dmV/h\r\n",
             (unsigned long)timestamp,
             (long)(lat_micro / 1000000), (long)labs(lat_micro % 1000000),
             (long)(lon_micro / 1000000), (long)labs(lon_micro % 1000000),
             record->altitude,
             record->temperature / 10, abs(record->temperature % 10),
             record->pressure / 10, record->pressure % 10,
             record->battery_voltage, record->solar_voltage, 
             voltage_slope);
    SEGGER_RTT_WriteString(0, debug_msg);
    
    return true;
}

/**
 * @brief Encode bulk telemetry packet from flash records (FIFO order)
 * @note  FW-20: v2 layout (198 B) — the v1 222 B layout's flash_page_addr /
 *        voltage_trend / mode_changes placeholders are gone; records carry
 *        their own timestamp + sequence identity.
 */
bool EncodeBulkPacketFromRecords(BulkTelemetryPacket_t *packet,
                                 const HighResTelemetryRecord_t *records,
                                 uint8_t record_count)
{
    if (!packet || !records || record_count == 0) {
        return false;
    }
    
    SEGGER_RTT_printf(0, "Encoding 198-byte bulk packet with %d records...\r\n", record_count);
    
    // Clear packet structure
    memset(packet, 0, sizeof(BulkTelemetryPacket_t));
    
    // Set packet header
    packet->packet_type = 0x02;  // FW-20: v2 FIFO bulk, no placeholder fields
    packet->record_count = (record_count > 6) ? 6 : record_count;  // Max 6 records
    
    // Copy high-resolution records (up to 6)
    for (uint8_t i = 0; i < packet->record_count && i < 6; i++) {
        memcpy(&packet->records[i], &records[i], sizeof(HighResTelemetryRecord_t));
    }
    
    // Calculate CRC32 for packet integrity (exclude CRC32 field itself)
    packet->crc32 = CalculateCRC32((const uint8_t*)packet, sizeof(BulkTelemetryPacket_t) - 4);
    
    SEGGER_RTT_printf(0, "Bulk packet: Type=%d Records=%d CRC32=0x%08lX\r\n",
                      packet->packet_type, packet->record_count, 
                      packet->crc32);
    
    return true;
}

/* ---- Explicit little-endian serialization helpers (D9, #33) ----
 * Wire v3 is explicitly LE-serialized per LoRaWANApplicationProtocol.md §3 —
 * no raw struct casts on the new format. */
static void PutU16LE(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void PutU32LE(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
                                               p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

/**
 * @brief Serialize one high-res record as 32 LE bytes (wire v3).
 *        Field order matches HighResTelemetryRecord_t; crc16 covers bytes 0-29.
 * @retval number of bytes written (always 32)
 */
static uint16_t SerializeRecordV3LE(uint8_t *out, const HighResTelemetryRecord_t *r)
{
    PutU32LE(out + 0,  r->timestamp);
    PutU32LE(out + 4,  (uint32_t)r->latitude);
    PutU32LE(out + 8,  (uint32_t)r->longitude);
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
    PutU16LE(out + 30, CalculateCRC16(out, 30));
    return 32;
}

/**
 * @brief Encode a variable-length bulk packet (wire v4, packet_type 0x04) — D3 (#33) + DDR-0011 identity (#34)
 */
bool EncodeBulkPacketV3(uint8_t *buf,
                        uint16_t buf_cap,
                        uint16_t max_payload,
                        const HighResTelemetryRecord_t *records,
                        uint8_t record_count,
                        uint32_t base_seq,
                        uint8_t *packed_count,
                        uint16_t *out_len)
{
    if (!buf || !records || !packed_count || !out_len || record_count == 0) {
        return false;
    }

    /* Whole records only; budget = min(buf_cap, max_payload) */
    uint16_t budget = (max_payload < buf_cap) ? max_payload : buf_cap;
    uint8_t n = (uint8_t)((budget - BULK_V3_OVERHEAD) / sizeof(HighResTelemetryRecord_t));
    if (n > record_count) n = record_count;
    if (n > BULK_V3_MAX_RECORDS) n = BULK_V3_MAX_RECORDS;
    if (n == 0) {
        *packed_count = 0;
        return false;
    }

    buf[0] = BULK_PACKET_TYPE_VARIABLE;
    buf[1] = n;
    PutU32LE(buf + 2, base_seq);   /* DDR-0011: record i identity = base_seq + i */
    uint16_t off = 6;
    for (uint8_t i = 0; i < n; i++) {
        off += SerializeRecordV3LE(buf + off, &records[i]);
    }
    PutU32LE(buf + off, CalculateCRC32(buf, off));
    off += 4;

    *packed_count = n;
    *out_len = off;

    SEGGER_RTT_printf(0, "Bulk v4: Records=%d Base=%lu Len=%u\r\n", n, (unsigned long)base_seq, off);
    return true;
}

/**
 * @brief Convert FlashLog_Record_t to HighResTelemetryRecord_t
 */
bool ConvertFlashLogToHighRes(const void *flash_record,
                              HighResTelemetryRecord_t *highres_record,
                              int16_t voltage_slope,
                              OperatingMode_t power_mode)
{
    if (!flash_record || !highres_record) {
        return false;
    }
    
    const FlashLog_Record_t *flash_rec = (const FlashLog_Record_t*)flash_record;
    
    // Clear destination structure
    memset(highres_record, 0, sizeof(HighResTelemetryRecord_t));
    
    // Copy basic fields
    highres_record->timestamp = flash_rec->timestamp;
    highres_record->latitude = flash_rec->latitude;
    highres_record->longitude = flash_rec->longitude;
    highres_record->altitude = (flash_rec->altitude_gps < 0) ? 0 :
                               (flash_rec->altitude_gps > 65535) ? 65535 :
                               (uint16_t)flash_rec->altitude_gps;  /* D5/#35: clamp int32 */
    
    // Convert floating point to scaled integers
    highres_record->temperature = (int16_t)(flash_rec->temperature * 10.0f);  // 0.1°C
    highres_record->humidity = (uint16_t)(flash_rec->humidity * 10.0f);       // 0.1%
    highres_record->pressure = (uint16_t)(flash_rec->pressure * 10.0f);       // 0.1hPa
    
    // Battery voltage
    highres_record->battery_voltage = flash_rec->battery_mv;
    /* D5/F-025/R19 (#35): history is self-describing — solar/slope/mode come
     * from the flash record itself (record v4), not from the current cycle's
     * live values passed as parameters. */
    highres_record->solar_voltage = flash_rec->solar_mv;
    highres_record->voltage_slope = flash_rec->voltage_slope;
    (void)voltage_slope;  /* superseded by flash-record value (kept for API compat) */
    (void)power_mode;
    
    // GPS metadata
    highres_record->satellites = flash_rec->satellites;
    highres_record->hdop = flash_rec->gnss_hdop_x10;  // Already scaled by 10
    
    // Power mode and flags (D5/#35: mode from the flash record — honest history)
    highres_record->power_mode = flash_rec->power_mode;
    highres_record->flags = PackStatusFlags((bool)flash_rec->gnss_valid, 
                                            flash_rec->satellites, 
                                            power_mode);
    
    // Calculate CRC16 for integrity
    highres_record->crc16 = CalculateCRC16((const uint8_t*)highres_record, 
                                           sizeof(HighResTelemetryRecord_t) - 2);
    
    return true;
}

/**
 * @brief Calculate CRC32 for bulk packets
 */
static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length)
{
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
bool PayloadFormat_ValidateSizes(void)
{
    bool valid = true;
    
    // Check packet sizes
    if (sizeof(CompactTelemetryPacket_t) != 11) {
        SEGGER_RTT_printf(0, "ERROR: CompactTelemetryPacket_t size = %d bytes (expected 11)\r\n", 
                          sizeof(CompactTelemetryPacket_t));
        valid = false;
    }
    
    if (sizeof(HighResTelemetryRecord_t) != 32) {
        SEGGER_RTT_printf(0, "ERROR: HighResTelemetryRecord_t size = %d bytes (expected 32)\r\n",
                          sizeof(HighResTelemetryRecord_t));
        valid = false;
    }
    
    if (sizeof(BulkTelemetryPacket_t) != 198) {
        SEGGER_RTT_printf(0, "ERROR: BulkTelemetryPacket_t size = %d bytes (expected 198)\r\n",
                          sizeof(BulkTelemetryPacket_t));
        valid = false;
    }
    
    if (valid) {
        SEGGER_RTT_WriteString(0, "Payload format sizes validated: 11/32/198 bytes\r\n");
    }
    
    return valid;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Get current timestamp in minutes since Unix epoch
 */
static uint16_t GetTimestampMinutes(void)
{
    /* R45: UTC epoch via SysTime (GPS-disciplined), not the boot-relative
     * RTC calendar — same doctrine as the flash write site (#42). */
    uint32_t seconds = SysTimeGet().Seconds;

    // Convert to minutes (with overflow handling)
    // uint16_t gives us ~45 days of range (65535 minutes = 1092 hours = 45.5 days)
    uint32_t minutes = seconds / 60;

    // Handle overflow by wrapping (simple modulo)
    return (uint16_t)(minutes & 0xFFFF);
}

/**
 * @brief Convert binary latitude to compact 100m resolution format
 */
static int16_t ConvertLatitudeToCompact(int32_t binary_latitude)
{
    // Convert binary to degrees, then scale to full int16 range (±90°)
    // Ground decoder: lat_deg = encoded × 90 / 32767
    float lat_degrees = binary_latitude * GPS_BINARY_TO_DEGREES;
    int32_t lat_scaled = (int32_t)(lat_degrees * LAT_SCALE_FACTOR);
    
    // Clamp to int16_t range (maps exactly to ±90°)
    if (lat_scaled > 32767) lat_scaled = 32767;
    if (lat_scaled < -32768) lat_scaled = -32768;
    
    return (int16_t)lat_scaled;
}

/**
 * @brief Convert binary longitude to compact 100m resolution format
 */
static int16_t ConvertLongitudeToCompact(int32_t binary_longitude)
{
    // Convert binary to degrees, then scale to full int16 range (±180°)
    // Ground decoder: lon_deg = encoded × 180 / 32767
    float lon_degrees = binary_longitude * (180.0f / 8388607.0f);
    int32_t lon_scaled = (int32_t)(lon_degrees * LON_SCALE_FACTOR);
    
    // Clamp to int16_t range (maps exactly to ±180°)
    if (lon_scaled > 32767) lon_scaled = 32767;
    if (lon_scaled < -32768) lon_scaled = -32768;
    
    return (int16_t)lon_scaled;
}

/**
 * @brief Convert temperature to compact format with 2°C resolution
 */
static int8_t ConvertTemperatureToCompact(float temperature_c)
{
    // Scale by 2°C resolution and add offset to handle negative values
    int16_t scaled = (int16_t)(temperature_c / TEMPERATURE_SCALE_FACTOR) + TEMPERATURE_OFFSET;
    
    // Clamp to int8_t range (0-255) = -64°C to +63°C after offset removal
    if (scaled > 255) scaled = 255;
    if (scaled < 0) scaled = 0;
    
    return (int8_t)scaled;
}

/**
 * @brief Pack pressure (11-bit, 1 hPa) + humidity (5-bit, 5%) — D2 (#33)
 * Pressure: 0..2046 hPa valid, sentinel 2047 when out of range/NaN.
 * Humidity: 0..20 (0-100%), sentinel 31 when out of range/NaN.
 */
static uint16_t PackPressureHumidity(float pressure_mbar, float humidity_percent)
{
    uint16_t press_units;
    if (isnan(pressure_mbar) || pressure_mbar < 0.0f || pressure_mbar > 2046.0f) {
        press_units = PRESS_HUM_PRESS_INVALID;
    } else {
        press_units = (uint16_t)(pressure_mbar + 0.5f);
        if (press_units > 2046) press_units = 2046;
    }

    uint16_t hum_units;
    if (isnan(humidity_percent) || humidity_percent < 0.0f || humidity_percent > 100.0f) {
        hum_units = PRESS_HUM_HUM_INVALID;
    } else {
        hum_units = (uint16_t)(humidity_percent / HUMIDITY_SCALE_FACTOR + 0.5f);
        if (hum_units > 20) hum_units = 20;
    }

    return (uint16_t)((hum_units << PRESS_HUM_HUM_SHIFT) | press_units);
}

/**
 * @brief Convert battery voltage to compact format with 50mV resolution
 */
static uint8_t ConvertBatteryVoltageToCompact(float voltage_volts)
{
    // Convert to 50mV units
    uint16_t voltage_50mv_units = (uint16_t)(voltage_volts / BATTERY_SCALE_FACTOR);
    
    // Clamp to uint8_t range (0-255) = 0-12.75V
    if (voltage_50mv_units > 255) voltage_50mv_units = 255;
    
    return (uint8_t)voltage_50mv_units;
}

/**
 * @brief Pack status flags into single byte
 */
static uint8_t PackStatusFlags(bool gps_valid, uint8_t satellites, OperatingMode_t power_mode)
{
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
static uint16_t CalculateCRC16(const uint8_t *data, uint32_t length)
{
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
