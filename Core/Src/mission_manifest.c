/**
 ******************************************************************************
 * @file    mission_manifest.c
 * @brief   Durable mission-lifecycle record (H-01, #281)
 ******************************************************************************
 * See mission_manifest.h for the design contract. The module follows the
 * Tier-2/nvm_slot precedent: newest valid generation wins, a torn write can
 * only kill the slot being written, the other survives. All FLASH_IF_* I/O
 * is here; the pure rules are at the bottom for host tests.
 ******************************************************************************
 */

#include "mission_manifest.h"
#include "flash_if.h"
#include "sonde_log.h"
#include <stddef.h> /* NULL */

/* CRC-32/ISO-HDLC (poly 0xEDB88320, init/xorout 0xFFFFFFFF) - same as
 * flash_log.c / payload_encode.c so the wire and storage CRCs agree. */
static uint32_t Manifest_CRC32(const uint8_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFFU;
  for (uint32_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8U; j++) {
      crc = (crc & 1U) ? (crc >> 1U) ^ 0xEDB88320U : (crc >> 1U);
    }
  }
  return ~crc;
}

static uint32_t Manifest_SlotAddr(uint8_t slot) {
  return MANIFEST_FLASH_ADDRESS + ((slot != 0U) ? MANIFEST_SLOT_B_OFFSET : MANIFEST_SLOT_A_OFFSET);
}

static bool Manifest_ReadSlot(uint8_t slot, MissionManifestRecord_t *rec) {
  if (FLASH_IF_Read(rec, (void *)(uintptr_t)Manifest_SlotAddr(slot),
                    sizeof(MissionManifestRecord_t)) != FLASH_IF_OK) {
    return false;
  }
  return MissionManifest_RecordPlausible(rec);
}

MissionManifest_StatusTypeDef MissionManifest_Load(MissionManifestRecord_t *rec) {
  if (rec == NULL) {
    return MANIFEST_ERROR_PARAM;
  }
  MissionManifestRecord_t a, b;
  bool va = Manifest_ReadSlot(0U, &a);
  bool vb = Manifest_ReadSlot(1U, &b);

  if (!va && !vb) {
    /* zero the output: FLIGHT_STARTED clear, generation 0 */
    for (uint32_t i = 0; i < sizeof(*rec); i++) {
      ((uint8_t *)rec)[i] = 0U;
    }
    return MANIFEST_EMPTY;
  }
  if (va && !vb) {
    *rec = a;
    return MANIFEST_OK;
  }
  if (!va && vb) {
    *rec = b;
    return MANIFEST_OK;
  }
  /* both valid: newest generation wins (wrap-aware) */
  *rec = MissionManifest_GenerationNewer(b.generation, a.generation) ? b : a;
  return MANIFEST_OK;
}

bool MissionManifest_IsFlightStarted(void) {
  MissionManifestRecord_t rec;
  if (MissionManifest_Load(&rec) != MANIFEST_OK) {
    return false;
  }
  return (rec.flags & MANIFEST_FLAG_FLIGHT_STARTED) != 0U;
}

uint32_t MissionManifest_LaunchRefHpaX10(void) {
  MissionManifestRecord_t rec;
  if (MissionManifest_Load(&rec) != MANIFEST_OK) {
    return 0U;
  }
  return rec.launch_ref_x10;
}

MissionManifest_StatusTypeDef MissionManifest_CommitFlightStarted(uint32_t launch_ref_hpa_x10,
                                                                  uint32_t launch_epoch_s) {
  /* Load current (EMPTY is fine: generation 0, latch clear). */
  MissionManifestRecord_t cur;
  MissionManifest_StatusTypeDef st = MissionManifest_Load(&cur);
  uint32_t cur_gen = (st == MANIFEST_OK) ? cur.generation : 0U;

  MissionManifestRecord_t rec;
  for (uint32_t i = 0; i < sizeof(rec); i++) {
    ((uint8_t *)&rec)[i] = 0U;
  }
  rec.magic = MANIFEST_MAGIC;
  rec.version = MANIFEST_VERSION;
  rec.generation = cur_gen + 1U; /* wrap-safe via GenerationNewer */
  rec.flags = MANIFEST_FLAG_FLIGHT_STARTED;
  rec.launch_ref_x10 = launch_ref_hpa_x10;
  rec.launch_epoch_s = launch_epoch_s;
  rec.crc32 = Manifest_CRC32((const uint8_t *)&rec, sizeof(rec) - sizeof(rec.crc32));

  /* Ping-pong: erase the inactive slot, write, commit by generation. A torn
   * write can only kill the slot being written; the other survives. */
  uint8_t slot = MissionManifest_SlotForStore(cur_gen);
  uint32_t addr = Manifest_SlotAddr(slot);
  if (FLASH_IF_Erase((void *)(uintptr_t)MANIFEST_FLASH_ADDRESS,
                     MANIFEST_FLASH_PAGE_SIZE) != FLASH_IF_OK) {
    SONDE_LOG_STR("Manifest: page erase FAILED\r\n");
    return MANIFEST_ERROR_FLASH;
  }
  if (FLASH_IF_Write((void *)(uintptr_t)addr, &rec, sizeof(rec)) != FLASH_IF_OK) {
    SONDE_LOG_STR("Manifest: slot write FAILED\r\n");
    return MANIFEST_ERROR_FLASH;
  }
  SONDE_LOG("Manifest: FLIGHT_STARTED committed (gen %lu, slot %d)\r\n",
            (unsigned long)rec.generation, (int)slot);
  return MANIFEST_OK;
}

/* --- pure rules ---------------------------------------------------------- */

uint32_t MissionManifest_ComputeCrc(const MissionManifestRecord_t *rec) {
  if (rec == NULL) {
    return 0U;
  }
  MissionManifestRecord_t tmp = *rec;
  tmp.crc32 = 0U;
  return Manifest_CRC32((const uint8_t *)&tmp, sizeof(tmp) - sizeof(tmp.crc32));
}

bool MissionManifest_RecordPlausible(const MissionManifestRecord_t *rec) {
  if (rec == NULL) {
    return false;
  }
  if (rec->magic != MANIFEST_MAGIC) {
    return false;
  }
  if (rec->version != MANIFEST_VERSION) {
    return false;
  }
  return MissionManifest_ComputeCrc(rec) == rec->crc32;
}

bool MissionManifest_GenerationNewer(uint32_t candidate, uint32_t incumbent) {
  return (int32_t)(candidate - incumbent) > 0;
}

uint8_t MissionManifest_SlotForStore(uint32_t current_generation) {
  /* even -> A, odd -> B (next store flips the parity). */
  return (uint8_t)((current_generation + 1U) % 2U);
}
