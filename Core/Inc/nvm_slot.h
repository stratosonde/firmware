/**
  ******************************************************************************
  * @file    nvm_slot.h
  * @brief   Pure codec/selection rules for the two-slot NVM context store
  ******************************************************************************
  * Extracted from LoRaWAN/App/lora_app.c (refactor stage 2; behaviour
  * preserving). The pure decisions (header build, plausibility, CRC match,
  * generation race, ping-pong slot choice, store admissibility) live here;
  * all FLASH_IF_* I/O, the scratch/staging buffers, and g_nvm_generation
  * remain in lora_app.c. No hardware, no file-scope mutable state.
  *
  * F-016 (#54): two-slot transactional NVM context persistence, matching the
  * proven Tier-2 ping-pong pattern (T1/FW-1). Each slot carries
  * magic/length/generation/CRC; newest valid generation wins. A torn write
  * can only kill the slot being written — the other survives.
  ******************************************************************************
  */

#ifndef NVM_SLOT_H
#define NVM_SLOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NVM_SLOT_MAGIC    0x4E564D43UL  /* "NVMC" */

typedef struct {
  uint32_t magic;
  uint32_t generation;
  uint32_t length;
  uint32_t crc32;      /* over the payload only */
} NvmSlotHeader_t;

/**
  * @brief  Flash-write length for a logical payload length (64-bit padded).
  *         FR-03 (#85): the header/CRC use the LOGICAL length so store and
  *         restore agree; only the physical flash write needs padding.
  */
uint32_t NvmSlot_PaddedLen(uint32_t logical_len);

/**
  * @brief  Store-side admission: reject bad pointer, zero size, or a padded
  *         payload + header that cannot fit one flash page.
  * @param  nvm: caller payload pointer (NULL check only; never dereferenced)
  * @param  nvm_size: size passed by the stack handler (already padded; zero rejects)
  * @param  padded_len: NvmSlot_PaddedLen(logical_len)
  * @param  page_size: flash page size in bytes
  * @retval true if the store may proceed
  */
bool NvmSlot_StoreAdmissible(const void *nvm, uint32_t nvm_size,
                             uint32_t padded_len, uint32_t page_size);

/**
  * @brief  Ping-pong slot choice for the NEXT store.
  * @param  current_generation: g_nvm_generation before the store
  * @retval 0 = slot A, 1 = slot B (current even -> A, odd -> B)
  */
uint8_t NvmSlot_SlotIndexForStore(uint32_t current_generation);

/**
  * @brief  Fill a slot header for the next store.
  * @param  next_generation: g_nvm_generation + 1 (wrap is caller-visible)
  * @param  logical_len: true object size the CRC covers (FR-03)
  * @param  crc32: CRC over the payload, computed by the caller
  */
void NvmSlot_BuildHeader(NvmSlotHeader_t *hdr, uint32_t next_generation,
                         uint32_t logical_len, uint32_t crc32);

/**
  * @brief  Restore-side header plausibility: magic, expected logical length,
  *         and payload + header fitting one page. Payload CRC is checked
  *         separately (NvmSlot_CrcMatches) after the caller stages the
  *         payload in scratch.
  */
bool NvmSlot_HeaderPlausible(const NvmSlotHeader_t *hdr, uint32_t expected_len,
                             uint32_t page_size);

/**
  * @brief  Does the staged payload's CRC match the header? A mismatch marks
  *         a torn slot: the other slot must be tried (FR-02, #282).
  */
bool NvmSlot_CrcMatches(const NvmSlotHeader_t *hdr, uint32_t payload_crc32);

/**
  * @brief  Wrap-aware generation race: is candidate strictly newer than
  *         incumbent? Signed-difference compare; a candidate that wrapped
  *         (0 after 0xFFFFFFFF) is newer than its pre-wrap incumbent.
  */
bool NvmSlot_GenerationNewer(uint32_t candidate, uint32_t incumbent);

#ifdef __cplusplus
}
#endif

#endif /* NVM_SLOT_H */
