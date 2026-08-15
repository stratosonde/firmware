/**
  ******************************************************************************
  * @file    test_nvm_slot.c
  * @brief   Module contract suite for Core/Src/nvm_slot.c (refactor stage 2)
  ******************************************************************************
  * CONTRACT suite, not a findings archive: documents what the NVM slot
  * codec/selection module promises, against the real linked module. The
  * restore-selection scenarios drive the same predicate composition the
  * adapter loop in lora_app.c uses (plausibility -> CRC -> generation race),
  * with CRCs injected by the test the same way the adapter injects
  * FlashLog_CRC32 results.
  *
  * Covered: padded length; store admissibility (NULL / zero / oversize);
  * ping-pong slot choice; header build; header plausibility; CRC match;
  * generation wraparound (0xFFFFFFFF -> 0); both slots invalid; both valid
  * with equal generations; corrupt payload with valid header; torn write.
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "nvm_slot.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define PAGE_SIZE 2048U
#define LOGICAL    1492U   /* the real LoRaMacNvmData_t size (FR-03) */

/* Mirror of the adapter's restore selection loop over two slots, using only
 * the module predicates. Validity inputs stand in for the flash reads. */
static int select_best(const NvmSlotHeader_t *h0, bool v0, uint32_t crc0,
                       const NvmSlotHeader_t *h1, bool v1, uint32_t crc1,
                       uint32_t *best_gen_out)
{
    const NvmSlotHeader_t *hdrs[2] = { h0, h1 };
    const bool valid[2] = { v0, v1 };
    const uint32_t crcs[2] = { crc0, crc1 };
    int best = -1;
    NvmSlotHeader_t best_hdr = {0, 0, 0, 0};
    for (int i = 0; i < 2; i++) {
        if (!valid[i]) continue;                                    /* read failed */
        if (!NvmSlot_HeaderPlausible(hdrs[i], LOGICAL, PAGE_SIZE)) continue;
        if (!NvmSlot_CrcMatches(hdrs[i], crcs[i])) continue;        /* torn slot */
        if (best >= 0 && !NvmSlot_GenerationNewer(hdrs[i]->generation,
                                                  best_hdr.generation)) continue;
        best = i;
        best_hdr = *hdrs[i];
    }
    if (best >= 0 && best_gen_out) *best_gen_out = best_hdr.generation;
    return best;
}

static NvmSlotHeader_t good_hdr(uint32_t generation, uint32_t crc)
{
    NvmSlotHeader_t h;
    NvmSlot_BuildHeader(&h, generation, LOGICAL, crc);
    return h;
}

int main(void)
{
    /* ---- padded length (FR-03 store/restore agreement) ---- */
    CHECK(NvmSlot_PaddedLen(1492U) == 1496U);     /* the real FR-03 pair */
    CHECK(NvmSlot_PaddedLen(8U) == 8U);           /* exact multiple stays */
    CHECK(NvmSlot_PaddedLen(1U) == 8U);
    CHECK(NvmSlot_PaddedLen(0U) == 0U);

    /* ---- store admissibility ---- */
    uint8_t dummy[8];
    CHECK(NvmSlot_StoreAdmissible(dummy, 1496U, 1496U, PAGE_SIZE) == true);
    CHECK(NvmSlot_StoreAdmissible(NULL, 1496U, 1496U, PAGE_SIZE) == false);
    CHECK(NvmSlot_StoreAdmissible(dummy, 0U, 1496U, PAGE_SIZE) == false);
    CHECK(NvmSlot_StoreAdmissible(dummy, 1496U, PAGE_SIZE, PAGE_SIZE) == false); /* padded+hdr > page */
    CHECK(NvmSlot_StoreAdmissible(dummy, 1496U, PAGE_SIZE - sizeof(NvmSlotHeader_t), PAGE_SIZE) == true); /* exact fit */

    /* ---- ping-pong slot choice ---- */
    CHECK(NvmSlot_SlotIndexForStore(0U) == 0U);   /* even -> A */
    CHECK(NvmSlot_SlotIndexForStore(1U) == 1U);   /* odd  -> B */
    CHECK(NvmSlot_SlotIndexForStore(0xFFFFFFFEU) == 0U);
    CHECK(NvmSlot_SlotIndexForStore(0xFFFFFFFFU) == 1U);

    /* ---- header build ---- */
    NvmSlotHeader_t h;
    memset(&h, 0xAA, sizeof(h));
    NvmSlot_BuildHeader(&h, 7U, LOGICAL, 0xDEADBEEFU);
    CHECK(h.magic == NVM_SLOT_MAGIC);
    CHECK(h.generation == 7U);
    CHECK(h.length == LOGICAL);
    CHECK(h.crc32 == 0xDEADBEEFU);

    /* ---- header plausibility ---- */
    CHECK(NvmSlot_HeaderPlausible(&h, LOGICAL, PAGE_SIZE) == true);
    CHECK(NvmSlot_HeaderPlausible(&h, 1400U, PAGE_SIZE) == false);          /* wrong length */
    h.magic = 0;
    CHECK(NvmSlot_HeaderPlausible(&h, LOGICAL, PAGE_SIZE) == false);        /* bad magic */
    h = good_hdr(1U, 0U);
    h.length = PAGE_SIZE;                                                   /* payload+hdr > page */
    CHECK(NvmSlot_HeaderPlausible(&h, PAGE_SIZE, PAGE_SIZE) == false);

    /* ---- CRC match ---- */
    h = good_hdr(1U, 1234U);
    CHECK(NvmSlot_CrcMatches(&h, 1234U) == true);
    CHECK(NvmSlot_CrcMatches(&h, 1235U) == false);

    /* ---- generation race, incl. wraparound ---- */
    CHECK(NvmSlot_GenerationNewer(2U, 1U) == true);
    CHECK(NvmSlot_GenerationNewer(1U, 1U) == false);                        /* equal loses */
    CHECK(NvmSlot_GenerationNewer(1U, 2U) == false);
    CHECK(NvmSlot_GenerationNewer(0U, 0xFFFFFFFFU) == true);                /* wrapped wins */
    CHECK(NvmSlot_GenerationNewer(0xFFFFFFFFU, 0U) == false);               /* pre-wrap loses */

    /* ---- restore-selection scenarios (adapter composition) ---- */
    uint32_t gen = 0;
    NvmSlotHeader_t a, b;

    /* both slots invalid (fresh start): no winner */
    a.magic = 0; b.magic = 0; a.length = b.length = LOGICAL;
    CHECK(select_best(&a, true, 0, &b, true, 0, &gen) == -1);

    /* both valid, equal generations: first slot wins (strictly-newer rule) */
    a = good_hdr(5U, 100U); b = good_hdr(5U, 100U);
    CHECK(select_best(&a, true, 100U, &b, true, 100U, &gen) == 0 && gen == 5U);

    /* newer generation in slot B beats older valid slot A */
    a = good_hdr(5U, 100U); b = good_hdr(6U, 200U);
    CHECK(select_best(&a, true, 100U, &b, true, 200U, &gen) == 1 && gen == 6U);

    /* corrupt payload with valid header (torn write): newest slot rejected,
     * older fully valid slot wins instead (FR-02) */
    a = good_hdr(5U, 100U); b = good_hdr(6U, 200U);
    CHECK(select_best(&a, true, 100U, &b, true, 999U, &gen) == 0 && gen == 5U);

    /* unreadable slot (flash read failed) is skipped like an invalid one */
    CHECK(select_best(&a, false, 0, &b, true, 200U, &gen) == 1 && gen == 6U);

    /* generation wrap: post-wrap generation 0 in B beats pre-wrap max in A */
    a = good_hdr(0xFFFFFFFFU, 100U); b = good_hdr(0U, 200U);
    CHECK(select_best(&a, true, 100U, &b, true, 200U, &gen) == 1 && gen == 0U);

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures != 0;
}
