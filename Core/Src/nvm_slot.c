/**
  ******************************************************************************
  * @file    nvm_slot.c
  * @brief   Pure codec/selection rules for the two-slot NVM context store
  ******************************************************************************
  * Decision logic extracted verbatim from lora_app.c's OnStoreContextRequest /
  * OnRestoreContextRequest (refactor stage 2). Same inputs -> same outputs;
  * no I/O, no globals. The header-alignment _Static_assert moved here with
  * the struct; the page-fit assert stays in lora_app.c because it also pins
  * the ST stack's NVM data object type, which this HAL-free module must
  * not see.
  ******************************************************************************
  */

#include "nvm_slot.h"

/* Moved with the struct from lora_app.c (refactor stage 2). */
_Static_assert(sizeof(NvmSlotHeader_t) % 8U == 0U, "NVM slot header must be 8-byte aligned");

uint32_t NvmSlot_PaddedLen(uint32_t logical_len)
{
  return (logical_len + 7U) & ~7U;
}

bool NvmSlot_StoreAdmissible(const void *nvm, uint32_t nvm_size,
                             uint32_t padded_len, uint32_t page_size)
{
  if (nvm == NULL || nvm_size == 0 ||
      padded_len + sizeof(NvmSlotHeader_t) > page_size) {
    return false;  /* honest failure, no silent drop */
  }
  return true;
}

uint8_t NvmSlot_SlotIndexForStore(uint32_t current_generation)
{
  return (uint8_t)(current_generation % 2U);
}

void NvmSlot_BuildHeader(NvmSlotHeader_t *hdr, uint32_t next_generation,
                         uint32_t logical_len, uint32_t crc32)
{
  hdr->magic = NVM_SLOT_MAGIC;
  hdr->generation = next_generation;
  hdr->length = logical_len;
  hdr->crc32 = crc32;
}

bool NvmSlot_HeaderPlausible(const NvmSlotHeader_t *hdr, uint32_t expected_len,
                             uint32_t page_size)
{
  if (hdr->magic != NVM_SLOT_MAGIC) return false;
  if (hdr->length != expected_len) return false;
  if (hdr->length + sizeof(NvmSlotHeader_t) > page_size) return false;
  return true;
}

bool NvmSlot_CrcMatches(const NvmSlotHeader_t *hdr, uint32_t payload_crc32)
{
  return payload_crc32 == hdr->crc32;
}

bool NvmSlot_GenerationNewer(uint32_t candidate, uint32_t incumbent)
{
  return (int32_t)(candidate - incumbent) > 0;
}
