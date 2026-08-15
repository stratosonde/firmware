/**
  ******************************************************************************
  * @file    fake_w25q.c
  * @brief   Host-test simulated W25Q16JV NOR flash backend (FR-10 / T-7)
  ******************************************************************************
  * Models the properties that actually matter for flash_log.c correctness:
  *
  *   - Erased state is 0xFF.
  *   - Program can only clear bits (AND), never set them. A rewrite without
  *     erase corrupts, exactly like real NOR. This is what makes FlashStorageNotes.md's
  *     erase-before-write invariant testable.
  *   - Sector erase granularity is W25Q_SECTOR_SIZE (4 KB), so an erase always
  *     destroys the whole sector — the property behind the erase-ahead slack
  *     that FR-10 is about.
  *
  * Also provides fault injection (fake_w25q_corrupt / fake_w25q_fail_next_read)
  * so torn-record and read-error paths can be exercised deterministically.
  ******************************************************************************
  */

#include "fake_w25q.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>

static uint8_t *g_mem = NULL;
static int      g_fail_reads  = 0;
static int      g_fail_writes = 0;   /* finding #4: intermittent program faults */
static int      g_fail_erases = 0;   /* finding #4: intermittent erase faults */
static uint32_t g_fail_below_addr  = 0;
static int      g_fail_below_count = 0;   /* FR-12 (#292): fail writes below an address (header sectors) */

/* Statistics — used to assert that the code under test does not, e.g.,
 * erase a sector it is about to read from. */
uint32_t fake_w25q_erase_count = 0;
uint32_t fake_w25q_write_count = 0;
uint32_t fake_w25q_read_count  = 0;

void fake_w25q_init(void)
{
    if (g_mem == NULL) {
        g_mem = (uint8_t *)malloc(W25Q_FLASH_SIZE);
        assert(g_mem != NULL);
    }
    memset(g_mem, 0xFF, W25Q_FLASH_SIZE);   /* virgin chip */
    g_fail_reads  = 0;
    g_fail_writes = 0;
    g_fail_erases = 0;
    g_fail_below_addr = 0;
    g_fail_below_count = 0;
    fake_w25q_erase_count = 0;
    fake_w25q_write_count = 0;
    fake_w25q_read_count  = 0;
}

void fake_w25q_free(void)
{
    free(g_mem);
    g_mem = NULL;
}

void fake_w25q_corrupt(uint32_t addr, uint32_t len)
{
    /* Flip bits to 0 the way a torn program would: clear the low nibble of
     * every byte. Guarantees a CRC failure without producing an all-0xFF
     * (erased-looking) run, so the record fails CRC rather than magic. */
    for (uint32_t i = 0; i < len; i++) {
        g_mem[addr + i] &= 0xF0u;
    }
}

void fake_w25q_fail_next_reads(int n) { g_fail_reads = n; }

/* Finding #4 (2026-08-10 review): the upstream fake could only fail READS,
 * which is why T-7b never exercised a failing header write. These model an
 * intermittent SPI/CS fault or a brownout that survives the erase but kills
 * the program step — the destructive case the finding reproduces. */
void fake_w25q_fail_next_writes(int n) { g_fail_writes = n; }
void fake_w25q_fail_next_erases(int n) { g_fail_erases = n; }

/* FR-12 (#292): WriteRecord's ordinary header checkpoint rides the SAME call
 * as the record's own program op, so "fail the next write" can only ever kill
 * the record, never the checkpoint. Target by address instead: the headers
 * live in the two sectors below the data region, so fail the next `n` writes
 * whose destination is below `addr_threshold`. Immune to the op-count shift
 * the fix's header-sync retries introduce. */
void fake_w25q_fail_writes_below(uint32_t addr_threshold, int n) { g_fail_below_addr = addr_threshold; g_fail_below_count = n; }

uint8_t fake_w25q_peek(uint32_t addr) { return g_mem[addr]; }

void fake_w25q_poke(uint32_t addr, const void *data, uint32_t len)
{
    /* Test-only fault injection: raw placement write that bypasses NOR
     * program-only semantics. Used by R2-03 to plant a CRC-valid record
     * carrying the WRONG sequence (the identity-mismatch fault). */
    if (g_mem == NULL) return;
    memcpy(g_mem + addr, data, len);
}

bool fake_w25q_is_erased(uint32_t addr, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        if (g_mem[addr + i] != 0xFFu) { return false; }
    }
    return true;
}

/* ---- W25Q API surface used by flash_log.c ------------------------------- */

W25Q_StatusTypeDef W25Q_Read(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                             uint8_t *data, uint32_t len)
{
    (void)hw25q;
    if (g_mem == NULL || data == NULL || len == 0) { return W25Q_ERROR; }
    if ((uint64_t)addr + len > W25Q_FLASH_SIZE)    { return W25Q_ERROR; }
    if (g_fail_reads > 0) { g_fail_reads--; return W25Q_ERROR; }
    fake_w25q_read_count++;
    memcpy(data, &g_mem[addr], len);
    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_Write(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                              const uint8_t *data, uint32_t len)
{
    (void)hw25q;
    if (g_mem == NULL || data == NULL || len == 0) { return W25Q_ERROR; }
    if ((uint64_t)addr + len > W25Q_FLASH_SIZE)    { return W25Q_ERROR; }
    if (g_fail_below_count > 0 && addr < g_fail_below_addr) { g_fail_below_count--; return W25Q_ERROR; }
    if (g_fail_writes > 0) { g_fail_writes--; return W25Q_ERROR; }
    fake_w25q_write_count++;
    /* NOR semantics: program can only clear bits. */
    for (uint32_t i = 0; i < len; i++) {
        g_mem[addr + i] &= data[i];
    }
    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_EraseSector(W25Q_HandleTypeDef *hw25q, uint32_t addr)
{
    (void)hw25q;
    if (g_mem == NULL)                     { return W25Q_ERROR; }
    if (addr >= W25Q_FLASH_SIZE)           { return W25Q_ERROR; }
    if (g_fail_erases > 0) { g_fail_erases--; return W25Q_ERROR; }
    fake_w25q_erase_count++;
    uint32_t base = (addr / W25Q_SECTOR_SIZE) * W25Q_SECTOR_SIZE;
    memset(&g_mem[base], 0xFF, W25Q_SECTOR_SIZE);
    return W25Q_OK;
}
