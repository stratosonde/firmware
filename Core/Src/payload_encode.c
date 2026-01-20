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
#include "SEGGER_RTT.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "flash_log.h"

/* Private defines -----------------------------------------------------------*/

/* GPS coordinate conversion factors */
#define GPS_BINARY_TO_DEGREES       (90.0f / 8388607.0f)  // Convert sensor_t binary to degrees
#define DEGREES_TO_100M_RESOLUTION  (1.0f / 0.0009009f)   // ~100m at equator (0.0009009° ≈ 100m)

/* Compact packet scaling factors */
#define TEMPERATURE_SCALE_FACTOR    2     // 2°C resolution
#define TEMPERATURE_OFFSET          64    // Offset for signed storage in uint8_t
#define PRESSURE_SCALE_FACTOR       10    // 10hPa resolution  
#define PRESSURE_BASE_OFFSET        950   // Base pressure (950 hPa)
#define BATTERY_SCALE_FACTOR        0.050f // 50mV resolution
#define HUMIDITY_SCALE_FACTOR       5     // 5% resolution

/* Private function prototypes -----------------------------------------------*/
static uint16_t GetTimestampMinutes(void);
static int16_t ConvertLatitudeToCompact(int32_t binary_latitude);
static int16_t ConvertLongitudeToCompact(int32_t binary_longitude);
static int8_t ConvertTemperatureToCompact(float temperature_c);
static uint8_t ConvertPressureToCompact(float pressure_mbar);
static uint8_t ConvertBatteryVoltageToCompact(float voltage_volts);
static uint8_t ConvertHumidityToCompact(float humidity_percent);
static uint8_t PackStatusFlags(bool gps_valid, uint8_t satellites, OperatingMode_t power_mode);
static uint16_t CalculateCRC16(const uint8_t *data, uint32_t length);
static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Encode compact 11-byte telemetry packet
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
    packet->pressure_10hPa = ConvertPressureToCompact(sensors->pressure);
    packet->humidity_5pct = ConvertHumidityToCompact(sensors->humidity);
    
    // Convert battery voltage (REQUIRED - DevStatusAns is on-demand only)
    packet->battery_volt_50mv = ConvertBatteryVoltageToCompact(sensors->battery_voltage);
    
    // Pack status flags
    packet->status_flags = PackStatusFlags(sensors->gnss_valid, 
                                           sensors->satellites, 
                                           power_mode);
    
    // Debug logging with safe integer conversions
    int32_t lat_micro = sensors->gnss_valid ? (int32_t)(sensors->latitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
    int32_t lon_micro = sensors->gnss_valid ? (int32_t)(sensors->longitude * GPS_BINARY_TO_DEGREES * 1000000) : 0;
    int16_t temp_centi = (int16_t)(sensors->temperature * 100);
    int16_t pressure_centi = (int16_t)(sensors->pressure * 100);
    int16_t humidity_centi = (int16_t)(sensors->humidity * 100);
    int16_t battery_mv = (int16_t)(sensors->battery_voltage * 1000);
    
    char debug_msg[200];
    snprintf(debug_msg, sizeof(debug_msg),
             "Compact: T=%lum Lat=%ld.%06ld Lon=%ld.%06ld Temp=%d.%02dC P=%d.%02d H=%d.%02d%% Bat=%dmV Sats=%d Mode=%d\r\n",
             (unsigned long)timestamp_min,
             (long)(lat_micro / 1000000), (long)labs(lat_micro % 1000000),
             (long)(lon_micro / 1000000), (long)labs(lon_micro % 1000000),
             temp_centi / 100, abs(temp_centi % 100),
             pressure_centi / 100, abs(pressure_centi % 100),
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
        uint16_t ms_unused;
        timestamp = TIMER_IF_GetTime(&ms_unused);  // RTC seconds
    }
    record->timestamp = timestamp;
    
    // Store GPS coordinates at full precision
    if (sensors->gnss_valid) {
        record->latitude = sensors->latitude;    // Keep original binary format
        record->longitude = sensors->longitude;  // Keep original binary format
        record->altitude = (uint16_t)sensors->altitudeGps;  // Convert to uint16_t
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
 * @brief Encode bulk telemetry packet from flash records (LIFO order)
 */
bool EncodeBulkPacketFromRecords(BulkTelemetryPacket_t *packet,
                                 const HighResTelemetryRecord_t *records,
                                 uint8_t record_count,
                                 uint32_t flash_page_addr,
                                 const uint8_t *voltage_trend,
                                 const uint8_t *mode_changes)
{
    if (!packet || !records || record_count == 0) {
        return false;
    }
    
    SEGGER_RTT_printf(0, "Encoding 222-byte bulk packet with %d records...\r\n", record_count);
    
    // Clear packet structure
    memset(packet, 0, sizeof(BulkTelemetryPacket_t));
    
    // Set packet header
    packet->packet_type = 0x01;  // Version 1 bulk format
    packet->record_count = (record_count > 6) ? 6 : record_count;  // Max 6 records
    packet->flash_page_addr = flash_page_addr;
    
    // Copy high-resolution records (up to 6)
    for (uint8_t i = 0; i < packet->record_count && i < 6; i++) {
        memcpy(&packet->records[i], &records[i], sizeof(HighResTelemetryRecord_t));
    }
    
    // Add voltage trend if provided
    if (voltage_trend != NULL) {
        memcpy(packet->voltage_trend, voltage_trend, 10);
    } else {
        memset(packet->voltage_trend, 0, 10);
    }
    
    // Add mode changes if provided  
    if (mode_changes != NULL) {
        memcpy(packet->mode_changes, mode_changes, 10);
    } else {
        memset(packet->mode_changes, 0, 10);
    }
    
    // Calculate CRC32 for packet integrity (exclude CRC32 field itself)
    packet->crc32 = CalculateCRC32((const uint8_t*)packet, sizeof(BulkTelemetryPacket_t) - 4);
    
    SEGGER_RTT_printf(0, "Bulk packet: Type=%d Records=%d FlashAddr=0x%08lX CRC32=0x%08lX\r\n",
                      packet->packet_type, packet->record_count, 
                      packet->flash_page_addr, packet->crc32);
    
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
    highres_record->altitude = (uint16_t)flash_rec->altitude_gps;
    
    // Convert floating point to scaled integers
    highres_record->temperature = (int16_t)(flash_rec->temperature * 10.0f);  // 0.1°C
    highres_record->humidity = (uint16_t)(flash_rec->humidity * 10.0f);       // 0.1%
    highres_record->pressure = (uint16_t)(flash_rec->pressure * 10.0f);       // 0.1hPa
    
    // Battery voltage
    highres_record->battery_voltage = flash_rec->battery_mv;
    highres_record->solar_voltage = 0;  // Not stored in FlashLog_Record_t yet
    highres_record->voltage_slope = voltage_slope;
    
    // GPS metadata
    highres_record->satellites = flash_rec->satellites;
    highres_record->hdop = flash_rec->gnss_hdop_x10;  // Already scaled by 10
    
    // Power mode and flags
    highres_record->power_mode = (uint8_t)power_mode;
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
    
    if (sizeof(BulkTelemetryPacket_t) != 222) {
        SEGGER_RTT_printf(0, "ERROR: BulkTelemetryPacket_t size = %d bytes (expected 222)\r\n",
                          sizeof(BulkTelemetryPacket_t));
        valid = false;
    }
    
    if (valid) {
        SEGGER_RTT_WriteString(0, "Payload format sizes validated: 11/32/222 bytes\r\n");
    }
    
    return valid;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Get current timestamp in minutes since Unix epoch
 */
static uint16_t GetTimestampMinutes(void)
{
    uint16_t ms_unused;
    uint32_t seconds = TIMER_IF_GetTime(&ms_unused);  // RTC seconds since epoch
    
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
    // Convert binary to degrees
    float lat_degrees = binary_latitude * GPS_BINARY_TO_DEGREES;
    
    // Convert to 100m resolution (approximately)
    // At equator: 1 degree ≈ 111.32 km, so 0.0009009° ≈ 100m
    int32_t lat_100m_units = (int32_t)(lat_degrees * DEGREES_TO_100M_RESOLUTION);
    
    // Clamp to int16_t range (-32768 to +32767) = ±3276.7 km from equator
    if (lat_100m_units > 32767) lat_100m_units = 32767;
    if (lat_100m_units < -32768) lat_100m_units = -32768;
    
    return (int16_t)lat_100m_units;
}

/**
 * @brief Convert binary longitude to compact 100m resolution format
 */
static int16_t ConvertLongitudeToCompact(int32_t binary_longitude)
{
    // Convert binary to degrees
    float lon_degrees = binary_longitude * (180.0f / 8388607.0f);  // Full longitude range
    
    // Convert to 100m resolution
    int32_t lon_100m_units = (int32_t)(lon_degrees * DEGREES_TO_100M_RESOLUTION);
    
    // Clamp to int16_t range
    if (lon_100m_units > 32767) lon_100m_units = 32767;
    if (lon_100m_units < -32768) lon_100m_units = -32768;
    
    return (int16_t)lon_100m_units;
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
 * @brief Convert pressure to compact format with 10hPa resolution from base
 */
static uint8_t ConvertPressureToCompact(float pressure_mbar)
{
    // Subtract base pressure and scale
    int16_t relative_pressure = (int16_t)((pressure_mbar - PRESSURE_BASE_OFFSET) / PRESSURE_SCALE_FACTOR);
    
    // Clamp to uint8_t range (0-255) = 950-3500 hPa range
    if (relative_pressure > 255) relative_pressure = 255;
    if (relative_pressure < 0) relative_pressure = 0;
    
    return (uint8_t)relative_pressure;
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
 * @brief Convert humidity to compact format with 5% resolution
 */
static uint8_t ConvertHumidityToCompact(float humidity_percent)
{
    // Scale to 5% resolution
    uint8_t humidity_5pct_units = (uint8_t)(humidity_percent / HUMIDITY_SCALE_FACTOR);
    
    // Clamp to valid range (0-20) = 0-100%
    if (humidity_5pct_units > 20) humidity_5pct_units = 20;
    
    return humidity_5pct_units;
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
