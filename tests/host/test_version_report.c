/**
 ******************************************************************************
 * @file    test_version_report.c
 * @brief   Module contract for Core/Src/version_report.c (A-005/STAB-11/F-09,
 *          #79/#158/#266)
 ******************************************************************************
 * The fw/format/epoch announce frame: version discriminator by mechanism
 * (device announces once; backend maps version -> wire layout) instead of
 * deployment folklore. Golden-vector structure and CRC checked here.
 ******************************************************************************
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "payload_format.h" /* HEARTBEAT_FORMAT_VERSION */
#include "version_report.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                          \
  do {                                                       \
    g_checks++;                                              \
    if (!(cond)) {                                           \
      g_failures++;                                          \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                        \
  } while (0)

static uint16_t crc16ccitt(const uint8_t *d, uint32_t n) {
  uint16_t crc = 0xFFFFU;
  for (uint32_t i = 0; i < n; i++) {
    crc ^= (uint16_t)(d[i] << 8);
    for (uint8_t j = 0; j < 8U; j++)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                            : (uint16_t)(crc << 1U);
  }
  return crc;
}

static void test_layout_sane(void) {
  uint8_t f[VERSION_REPORT_LEN] = {0};
  CHECK(VersionReport_Build(f, HEARTBEAT_FORMAT_VERSION,
                            VERSION_STAGE_COMMISSIONING, 1234U));
  CHECK(f[0] == VERSION_REPORT_MAGIC);
  CHECK(f[1] == SONDE_FW_MAJOR && f[2] == SONDE_FW_MINOR &&
        f[3] == SONDE_FW_PATCH);
  CHECK(f[4] == 2U); /* heartbeat v2 */
  CHECK(f[5] == VERSION_STAGE_COMMISSIONING);
  uint32_t minutes = (uint32_t)f[6] | ((uint32_t)f[7] << 8) |
                     ((uint32_t)f[8] << 16) | ((uint32_t)f[9] << 24);
  CHECK(minutes == 1234U);
  uint16_t wire_crc = (uint16_t)((uint16_t)f[10] | (uint16_t)(f[11] << 8));
  CHECK(wire_crc == crc16ccitt(f, 10));
  /* NULL guard */
  CHECK(!VersionReport_Build(NULL, HEARTBEAT_FORMAT_VERSION,
                             VERSION_STAGE_FLIGHT, 0));
}

static void test_stage_flag_choice(void) {
  uint8_t f[VERSION_REPORT_LEN];
  CHECK(VersionReport_Build(f, HEARTBEAT_FORMAT_VERSION,
                            VERSION_STAGE_FLIGHT, 0));
  CHECK(f[5] == VERSION_STAGE_FLIGHT);
}

static void test_form_completed_frame_bits_shift(void) {
  /* Stage flags must not leak into the version bits or magic. */
  uint8_t f[VERSION_REPORT_LEN];
  CHECK(VersionReport_Build(f, HEARTBEAT_FORMAT_VERSION,
                            VERSION_STAGE_COMMISSIONING, 0xA5A5U));
  CHECK(f[0] == VERSION_REPORT_MAGIC);
  uint32_t fw_bits = (uint32_t)f[1] | ((uint32_t)f[2] << 8) | ((uint32_t)f[3] << 16);
  uint32_t fw_expect = SONDE_FW_MAJOR | ((uint32_t)SONDE_FW_MINOR << 8) |
                       ((uint32_t)SONDE_FW_PATCH << 16);
  CHECK(fw_bits == fw_expect);
}

int main(void) {
  printf("=== version_report contract suite (A-005/#79) ===\n\n");
  test_layout_sane();
  test_stage_flag_choice();
  test_form_completed_frame_bits_shift();
  printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}
