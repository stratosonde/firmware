/**
  ******************************************************************************
  * @file    version_report.h
  * @brief   Firmware + wire-format version report frame (FPort 20)
  ******************************************************************************
  * A-005 / STAB-11 / F-09 (#79 / #158 / #266): the heartbeat's schema is
  * discriminated by the KNOWN firmware version, and firmware cannot be
  * updated in flight - so the device announces its version once (commissioning)
  * and once at first-flight admission, on a dedicated port, leaving the
  * 11-byte heartbeat budget untouched. The backend maps (fw version) ->
  * expected wire layout for all subsequent frames.
  *
  * Frame (11 bytes, little-endian, like the heartbeat):
  *   [0]  magic 0x56 'V'
  *   [1]  fw major
  *   [2]  fw minor
  *   [3]  fw patch
  *   [4]  format version (heartbeat wire version, e.g. 2)
  *   [5]  flags (bit0 = commissioning stage, bit1 = flight admission)
  *   [6-9]  mission minutes elapsed (LE u32)
  *   [10-11] CRC16 CCITT over bytes 0-9 (LE)
  ******************************************************************************
  */

#ifndef VERSION_REPORT_H
#define VERSION_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Bumped manually at each release (hardcoded config convention). */
#define SONDE_FW_MAJOR 1U
#define SONDE_FW_MINOR 0U
#define SONDE_FW_PATCH 0U

#define VERSION_REPORT_MAGIC 0x56U /* 'V' */
#define VERSION_REPORT_LEN 12U

typedef enum {
  VERSION_STAGE_COMMISSIONING = 0x01,
  VERSION_STAGE_FLIGHT = 0x02
} VersionReport_StageTypeDef;

/**
  * @brief  Build a version-report frame.
  * @param  out: buffer of at least VERSION_REPORT_LEN bytes
  * @param  format_version: HEARTBEAT_FORMAT_VERSION from payload_format.h
  * @param  stage: commissioning / flight admission flag
  * @param  mission_minutes: Payload_TimestampMinutesNow()
  * @retval true on success, false on NULL buffer
  */
bool VersionReport_Build(uint8_t *out, uint32_t format_version,
                         VersionReport_StageTypeDef stage,
                         uint32_t mission_minutes);

#ifdef __cplusplus
}
#endif

#endif /* VERSION_REPORT_H */
