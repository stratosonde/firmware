/**
 ******************************************************************************
 * @file    test_mission_manifest.c
 * @brief   Module contract suite for Core/Src/mission_manifest.c (H-01, #281)
 ******************************************************************************
 * CONTRACT suite (precedent: test_nvm_slot.c): the durable mission-lifecycle
 * record. Covers the pure rules (plausibility, generation race, slot choice)
 * and the full I/O path against an in-memory flash fake.
 ******************************************************************************
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* SONDE_LOG inside mission_manifest.c compiles to no-op via the Makefile's
 * -DSONDE_FLIGHT_BUILD (same pattern as test_flightreadiness.c). */
#include "flash_if.h" /* stub: FLASH_IF_StatusTypedef for the fake below */
#include "mission_manifest.h"

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

#define CHECK_REGRESSION(cond, tag)                                       \
  do {                                                                    \
    g_checks++;                                                           \
    if (!(cond)) {                                                        \
      g_failures++;                                                       \
      printf("FAIL-REGRESSION [%s] %s:%d: %s\n", tag, __FILE__, __LINE__, \
             #cond);                                                      \
    }                                                                     \
  } while (0)

/* ---- in-memory internal-flash fake (NOR: erase->0xFF, write clears bits) - */
static uint8_t g_flash[MANIFEST_FLASH_PAGE_SIZE];

FLASH_IF_StatusTypedef FLASH_IF_Init(void *p) {
  (void)p;
  return FLASH_IF_OK;
}
FLASH_IF_StatusTypedef FLASH_IF_DeInit(void) { return FLASH_IF_OK; }

FLASH_IF_StatusTypedef FLASH_IF_Write(void *dst, const void *src, uint32_t len) {
  uint32_t addr = (uint32_t)(uintptr_t)dst;
  if (addr < MANIFEST_FLASH_ADDRESS ||
      addr + len > MANIFEST_FLASH_ADDRESS + MANIFEST_FLASH_PAGE_SIZE) {
    return FLASH_IF_ERROR;
  }
  uint32_t off = addr - MANIFEST_FLASH_ADDRESS;
  const uint8_t *s = (const uint8_t *)src;
  for (uint32_t i = 0; i < len; i++) {
    g_flash[off + i] &= s[i];
  }
  return FLASH_IF_OK;
}

FLASH_IF_StatusTypedef FLASH_IF_Read(void *dst, const void *src, uint32_t len) {
  uint32_t addr = (uint32_t)(uintptr_t)src;
  if (addr < MANIFEST_FLASH_ADDRESS ||
      addr + len > MANIFEST_FLASH_ADDRESS + MANIFEST_FLASH_PAGE_SIZE) {
    return FLASH_IF_ERROR;
  }
  memcpy(dst, g_flash + (addr - MANIFEST_FLASH_ADDRESS), len);
  return FLASH_IF_OK;
}

FLASH_IF_StatusTypedef FLASH_IF_Erase(void *start, uint32_t len) {
  uint32_t addr = (uint32_t)(uintptr_t)start;
  if (addr != MANIFEST_FLASH_ADDRESS || len > MANIFEST_FLASH_PAGE_SIZE) {
    return FLASH_IF_ERROR;
  }
  memset(g_flash, 0xFF, len);
  return FLASH_IF_OK;
}

static void flash_reset(void) { memset(g_flash, 0xFF, sizeof(g_flash)); }

static void corrupt_slot(uint8_t slot) {
  uint32_t off = (slot != 0U) ? MANIFEST_SLOT_B_OFFSET : MANIFEST_SLOT_A_OFFSET;
  g_flash[off] ^= 0xA5U;
}

static void test_pure_rules(void) {
  printf("-- manifest pure rules\n");
  CHECK(MissionManifest_GenerationNewer(5, 4));
  CHECK(!MissionManifest_GenerationNewer(4, 5));
  CHECK(!MissionManifest_GenerationNewer(5, 5));
  CHECK(MissionManifest_GenerationNewer(0, 0xFFFFFFFFUL));
  /* slot choice: next store flips parity -> gen0->B, gen1->A, gen2->B */
  CHECK(MissionManifest_SlotForStore(0) == 1);
  CHECK(MissionManifest_SlotForStore(1) == 0);
  CHECK(MissionManifest_SlotForStore(2) == 1);
  MissionManifestRecord_t r;
  memset(&r, 0, sizeof(r));
  r.magic = MANIFEST_MAGIC;
  r.version = MANIFEST_VERSION;
  CHECK(!MissionManifest_RecordPlausible(&r)); /* crc=0 wrong */
  CHECK(!MissionManifest_RecordPlausible(NULL));
  r.magic = 0xDEADBEEF;
  CHECK(!MissionManifest_RecordPlausible(&r));
}

static void test_fresh_unit_empty(void) {
  printf("-- fresh unit: no valid slot -> EMPTY, not flight-started\n");
  flash_reset();
  MissionManifestRecord_t rec;
  CHECK(MissionManifest_Load(&rec) == MANIFEST_EMPTY);
  CHECK(!MissionManifest_IsFlightStarted());
  CHECK(MissionManifest_LaunchRefHpaX10() == 0U);
}

static void test_commit_roundtrip(void) {
  printf("-- commit: latch set, round-trips, ref restores\n");
  flash_reset();
  CHECK(MissionManifest_CommitFlightStarted(10133U, 0U) == MANIFEST_OK);
  CHECK(MissionManifest_IsFlightStarted());
  CHECK_REGRESSION(MissionManifest_LaunchRefHpaX10() == 10133U, "H-01-ref");
  MissionManifestRecord_t rec;
  CHECK(MissionManifest_Load(&rec) == MANIFEST_OK);
  CHECK((rec.flags & MANIFEST_FLAG_FLIGHT_STARTED) != 0U);
  CHECK(rec.generation == 1U);
}

/* Helper: hand-build a valid record into a slot without the module's page
 * erase, so two records can coexist for the generation/torn scenarios. */
static void plant_record(uint8_t slot, uint32_t gen, uint32_t ref_x10) {
  MissionManifestRecord_t r;
  memset(&r, 0, sizeof(r));
  r.magic = MANIFEST_MAGIC;
  r.version = MANIFEST_VERSION;
  r.generation = gen;
  r.flags = MANIFEST_FLAG_FLIGHT_STARTED;
  r.launch_ref_x10 = ref_x10;
  r.crc32 = MissionManifest_ComputeCrc(&r);
  uint32_t addr = MANIFEST_FLASH_ADDRESS +
                  ((slot != 0U) ? MANIFEST_SLOT_B_OFFSET : MANIFEST_SLOT_A_OFFSET);
  memcpy(g_flash + (addr - MANIFEST_FLASH_ADDRESS), &r, sizeof(r));
}

static void test_pingpong_and_torn(void) {
  printf("-- generation race + torn write (coexisting slots)\n");
  flash_reset();
  /* Two coexisting valid records (the page survives between boots; only ONE
   * slot is live per commit, but a reset between commits leaves both). */
  plant_record(0U, 1U, 10000U); /* A: gen1 */
  plant_record(1U, 2U, 10050U); /* B: gen2 - newest */
  MissionManifestRecord_t rec;
  CHECK(MissionManifest_Load(&rec) == MANIFEST_OK);
  CHECK(rec.generation == 2U);
  CHECK(rec.launch_ref_x10 == 10050U);

  /* torn write: corrupt the NEWEST slot (B = gen2) -> older A (gen1) loads */
  corrupt_slot(1U);
  CHECK(MissionManifest_Load(&rec) == MANIFEST_OK);
  CHECK_REGRESSION(rec.generation == 1U, "H-01-torn");
  CHECK(rec.launch_ref_x10 == 10000U);

  /* both invalid -> EMPTY (safe default) */
  corrupt_slot(0U);
  CHECK(MissionManifest_Load(&rec) == MANIFEST_EMPTY);
  CHECK(!MissionManifest_IsFlightStarted());
}

static void test_ground_powercycle_commissioning(void) {
  printf("-- H-01: commissioned unit power-cycled on ground -> NOT flight\n");
  /* Session bank written, door never opened: manifest page is blank. Power
   * loss wipes DR3 but the manifest does not invent flight. */
  flash_reset();
  CHECK(!MissionManifest_IsFlightStarted());
}

static void test_airborne_vbat_dead_ascent(void) {
  printf("-- H-01: airborne unit, VBAT dead, power loss -> FLIGHT survives\n");
  flash_reset();
  CHECK(MissionManifest_CommitFlightStarted(9900U, 0U) == MANIFEST_OK);
  /* DR3/DR15 gone; the durable manifest must still say FLIGHT. */
  CHECK_REGRESSION(MissionManifest_IsFlightStarted(), "H-01-durable");
}

int main(void) {
  printf("=== mission_manifest contract suite (H-01, #281) ===\n\n");
  test_pure_rules();
  test_fresh_unit_empty();
  test_commit_roundtrip();
  test_pingpong_and_torn();
  test_ground_powercycle_commissioning();
  test_airborne_vbat_dead_ascent();
  printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}
