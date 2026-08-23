/**
 ******************************************************************************
 * @file    version_report.c
 * @brief   Pure version-report frame encoder (version_report.h)
 ******************************************************************************
 */

#include "version_report.h"
#include <stddef.h> /* NULL */

/* CRC-16CCITT (poly 0x1021, init 0xFFFF) - same as payload_encode.c. */
static uint16_t Crc16(const uint8_t *data, uint32_t length) {
  uint16_t crc = 0xFFFFU;
  for (uint32_t i = 0; i < length; i++) {
    crc ^= (uint16_t)(data[i] << 8);
    for (uint8_t j = 0; j < 8U; j++) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

bool VersionReport_Build(uint8_t *out, uint32_t format_version,
                         VersionReport_StageTypeDef stage,
                         uint32_t mission_minutes) {
  if (out == NULL) {
    return false;
  }
  out[0] = VERSION_REPORT_MAGIC;
  out[1] = (uint8_t)SONDE_FW_MAJOR;
  out[2] = (uint8_t)SONDE_FW_MINOR;
  out[3] = (uint8_t)SONDE_FW_PATCH;
  out[4] = (uint8_t)format_version;
  out[5] = (uint8_t)stage;
  out[6] = (uint8_t)(mission_minutes & 0xFFU);
  out[7] = (uint8_t)((mission_minutes >> 8) & 0xFFU);
  out[8] = (uint8_t)((mission_minutes >> 16) & 0xFFU);
  out[9] = (uint8_t)((mission_minutes >> 24) & 0xFFU);
  uint16_t crc = Crc16(out, 10);
  out[10] = (uint8_t)(crc & 0xFFU);
  out[11] = (uint8_t)((crc >> 8) & 0xFFU);
  return true;
}
