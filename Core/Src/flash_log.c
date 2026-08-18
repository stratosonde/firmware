/**
 ******************************************************************************
 * @file    flash_log.c
 * @brief   Power-Safe Flash Data Logging Implementation
 * @details Production-grade logging system with power-failure recovery
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "flash_log.h"
#include "sonde_log.h" /* R3-07 (#221): reconstruction is loud */
#include <math.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
/* T4 FIX (FlashStorageNotes.md): the two ping-pong headers MUST live in DIFFERENT sectors.
 * Previously A=0x0000 and B=0x0100 were both inside sector 0 — one sector
 * erase killed both copies, and every header rewrite after the first two was
 * a non-erased rewrite (NOR flash cannot flip 0->1 without erase), so the
 * ping-pong was fake. Now: sector 0 = header A, sector 1 = header B,
 * data starts at sector 2 (FLASH_LOG_DATA_START in flash_log.h). */
#define HEADER_A_ADDR 0x0000           /* Sector 0 */
#define HEADER_B_ADDR W25Q_SECTOR_SIZE /* Sector 1 */
#define HEADER_UPDATE_INTERVAL 10      /* Update header every N records */

/* CRC32 polynomial (IEEE 802.3) */
#define CRC32_POLYNOMIAL 0xEDB88320

/* Private variables ---------------------------------------------------------*/
static uint32_t crc32_table[256];
static bool crc32_table_initialized = false;

/* Private function prototypes -----------------------------------------------*/
static void FlashLog_InitCRC32Table(void);
static FlashLog_StatusTypeDef FlashLog_ReadHeader(FlashLog_HandleTypeDef *hlog, uint32_t addr, FlashLog_Header_t *header);
static FlashLog_StatusTypeDef FlashLog_WriteHeader(FlashLog_HandleTypeDef *hlog);
static bool FlashLog_ValidateHeader(const FlashLog_HandleTypeDef *hlog, const FlashLog_Header_t *header);
static uint32_t FlashLog_GetRecordAddress(FlashLog_HandleTypeDef *hlog, uint32_t record_index);
static FlashLog_StatusTypeDef FlashLog_EraseSectorIfNeeded(FlashLog_HandleTypeDef *hlog, uint32_t addr);
static FlashLog_StatusTypeDef FlashLog_FrontierScan(FlashLog_HandleTypeDef *hlog);

/* CRC32 Implementation ------------------------------------------------------*/

/**
 * @brief  Initialize CRC32 lookup table (called once)
 */
static void FlashLog_InitCRC32Table(void) {
  uint32_t i, j, crc;

  if (crc32_table_initialized) {
    return;
  }

  for (i = 0; i < 256; i++) {
    crc = i;
    for (j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
      } else {
        crc >>= 1;
      }
    }
    crc32_table[i] = crc;
  }

  crc32_table_initialized = true;
}

uint32_t FlashLog_CRC32(const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFF;
  uint32_t i;

  FlashLog_InitCRC32Table();

  for (i = 0; i < len; i++) {
    crc = (crc >> 8) ^ crc32_table[(crc ^ data[i]) & 0xFF];
  }

  return ~crc;
}

/* Private Functions ---------------------------------------------------------*/

/**
 * @brief  Read header from flash and validate
 */
static FlashLog_StatusTypeDef FlashLog_ReadHeader(FlashLog_HandleTypeDef *hlog,
                                                  uint32_t addr,
                                                  FlashLog_Header_t *header) {
  W25Q_StatusTypeDef status;

  status = W25Q_Read(hlog->hw25q, addr, (uint8_t *)header, sizeof(FlashLog_Header_t));
  if (status != W25Q_OK) {
    return FLASH_LOG_ERROR_FLASH;
  }

  return FLASH_LOG_OK;
}

/* R3-04 (#218): oldest sequence still retained (wrap-aware). */
static uint32_t FlashLog_OldestSequence(FlashLog_HandleTypeDef *hlog) {
  uint32_t available = FlashLog_GetAvailableRecords(hlog);
  return (hlog->next_sequence > available) ? (hlog->next_sequence - available) : 0;
}

bool FlashLog_HasUnsentData(FlashLog_HandleTypeDef *hlog) {
  if (hlog == NULL || !hlog->initialized) {
    return false;
  }

  /* v5 one-pass (R3-04/#218) + BEH-05 (#286): sendable = pending-drain
   * ([tx_high_water, pending_frontier)) UNION walker-eligible (oldest <=
   * seq < recovery_frontier). Invariant: F <= H <= B <= next_sequence. */
  if (hlog->pending_frontier > hlog->tx_high_water) {
    return true; /* pending-live */
  }
  return (hlog->recovery_frontier > FlashLog_OldestSequence(hlog));
}

/* R3-04 (#218, DDR-0005 BR-TX-008..011): one-pass recovery read.
 * Supersedes FlashLog_GetUnsentRecordsFIFO (C4/F15).
 *
 * Exact O(1)-state model (invariants F <= H <= B <= next):
 *   pending_frontier B  - BEH-05 (#286): pending-live = [H, B), drained
 *                         DESCENDING (newest-first, BR-TX-008) and marked
 *                         per record (B := seq). A fresh outage therefore
 *                         begins with the NEWEST missed record, not the
 *                         oldest. B is persisted in header reserved[1].
 *   tx_high_water H     - bottom edge of the pending drain; rises only when
 *                         the drain edge meets it (the whole pending range
 *                         is then sent/retired) or on the legacy bottom-
 *                         edge live-send lift.
 *   recovery_frontier F - the BACKLOG walker: unsent = [oldest_seq, F).
 *                         Read DESCENDING and marked per record (F := seq).
 * The pending drain leads the batch (the common case: the current cycle's
 * record goes out in its own opportunity, BR-TX-005); the deep archive
 * drains newest-first via F.
 * Corrupt/torn records are skipped and counted; a contiguous corrupt run at
 * a leading edge is retired once after the loop (T4/RV-01/S-G anti-wedge
 * discipline); holes past a good record retire when a later send marks
 * past them. */
FlashLog_StatusTypeDef FlashLog_GetRecoveryRecords(FlashLog_HandleTypeDef *hlog,
                                                   FlashLog_Record_t *records,
                                                   uint32_t max_count,
                                                   uint32_t *actual_count,
                                                   uint32_t *skipped_count) {
  FlashLog_StatusTypeDef status;

/* R13 (#51): bound the scan — a long corrupt run must not spin thousands
 * of SPI reads in one call. The watermarks persist, so the next call
 * resumes exactly where this one stopped. */
#define FLASH_LOG_MAX_PROBES_PER_CALL 256U
  uint32_t probes = 0;

  if (hlog == NULL || !hlog->initialized || records == NULL || actual_count == NULL) {
    return FLASH_LOG_ERROR_PARAM;
  }

  *actual_count = 0;
  if (skipped_count != NULL) {
    *skipped_count = 0;
  }

  if (!FlashLog_HasUnsentData(hlog)) {
    return FLASH_LOG_ERROR_EMPTY;
  }

  /* Defensive clamps (persistence corruption, downgrade-reupgrade): the
   * watermarks must stay inside [0, next_sequence] and satisfy F <= H. */
  if (hlog->tx_high_water > hlog->next_sequence) {
    hlog->tx_high_water = hlog->next_sequence;
  }
  if (hlog->recovery_frontier > hlog->tx_high_water) {
    hlog->recovery_frontier = hlog->tx_high_water;
  }

  const uint32_t oldest = FlashLog_OldestSequence(hlog);

  /* SP-19 (#252): a long TX-silent run can wrap the ring past H. Sequences
   * in [H, oldest) are OVERWRITTEN - not pending, not corrupt, gone. Lift H
   * so pending-live starts at resident data; otherwise every archive
   * opportunity burned probes on R2-03 identity skips (up to the R13
   * 256-probe bound) and GetUnsentCount overstated the real backlog. The
   * lift is RAM-side only until the next MarkRecoverySent persists it;
   * re-deriving it here each call is idempotent and cheap. */
  if (hlog->tx_high_water < oldest) {
    hlog->tx_high_water = oldest;
  }

  /* BEH-05: keep the drain edge and episode top inside
   * [tx_high_water, next_sequence]. */
  if (hlog->pending_frontier > hlog->next_sequence) {
    hlog->pending_frontier = hlog->next_sequence;
  }
  if (hlog->pending_frontier < hlog->tx_high_water) {
    hlog->pending_frontier = hlog->tx_high_water;
  }
  if (hlog->drain_top > hlog->next_sequence) {
    hlog->drain_top = hlog->next_sequence;
  }
  if (hlog->drain_top < hlog->pending_frontier) {
    hlog->drain_top = hlog->pending_frontier;
  }

  uint32_t sendable = FlashLog_GetUnsentCount(hlog);
  if (max_count > sendable) {
    max_count = sendable;
  }

  /* BEH-05 (#286): pending-live [tx_high_water, pending_frontier) DESCENDING
   * first (a fresh outage drains newest-first), then the walker range
   * (recovery_frontier - 1 .. oldest) DESCENDING. int64 loop vars: both
   * ranges can legitimately reach seq 0. */
  uint32_t retire_live = 0; /* S-G (#214): deferred leading-run retire */
  uint32_t retire_walker = 0;
  bool live_retire_pending = false;
  bool walker_retire_pending = false;

  for (int range = 0; range < 2 && (*actual_count) < max_count &&
                      probes < FLASH_LOG_MAX_PROBES_PER_CALL;
       range++) {
    int64_t seq, seq_end;
    if (range == 0) {
      if (hlog->pending_frontier <= hlog->tx_high_water)
        continue;
      seq = (int64_t)hlog->pending_frontier - 1; /* descending */
      seq_end = (int64_t)hlog->tx_high_water;
    } else {
      if (hlog->recovery_frontier <= oldest)
        continue;
      seq = (int64_t)hlog->recovery_frontier - 1; /* descending */
      seq_end = (int64_t)oldest;
    }

    for (; seq >= seq_end; seq--) {
      if ((*actual_count) >= max_count ||
          probes >= FLASH_LOG_MAX_PROBES_PER_CALL)
        break;
      /* Convert sequence to ReadRecord offset (newest-relative) */
      uint32_t offset = (hlog->next_sequence - 1) - (uint32_t)seq;
      probes++;

      status = FlashLog_ReadRecord(hlog, &records[*actual_count], offset);

      /* R2-03 (#107): identity check — a CRC-valid record carrying the
       * wrong sequence is corruption: skip it, count it. */
      if (status == FLASH_LOG_OK &&
          records[*actual_count].sequence != (uint32_t)seq) {
        status = FLASH_LOG_ERROR_CRC;
      }

      if (status != FLASH_LOG_OK) {
        /* F-006/R13 (#51): corrupt/torn — count it, never silent
         * (DDR-0003). Retire inline ONLY while nothing good has been
         * read this call (RV-01 #160 leading-run rule, mirrored for
         * the descending walker); holes past a good record retire
         * when a later MarkRecoverySent lands below them. */
        if (skipped_count != NULL) {
          (*skipped_count)++;
        }
        if (*actual_count == 0) {
          /* BEH-05: BOTH ranges now walk descending, so both track the
           * LOWEST corrupt of the leading run; the deferred retire
           * (B := seq for the pending drain, F := seq for the walker)
           * covers the contiguous run down from the range's top edge. */
          if (range == 0) {
            retire_live = (uint32_t)seq;
            live_retire_pending = true;
          } else {
            retire_walker = (uint32_t)seq;
            walker_retire_pending = true;
          }
        }
        continue;
      }

      (*actual_count)++;
    }
  }

  /* S-G (#214): one deferred retire per range, after the loop — never a
   * header sector erase per corrupt record. MarkRecoverySent honors the
   * deferred-header batching (#8) and the edge monotonicities. The leading
   * corrupt run is contiguous while *actual_count == 0; both ranges walk
   * descending (BEH-05), so both retire the LOWEST corrupt of the run:
   *   pending drain: B := retire_live
   *   walker:        F := retire_walker */
  if (live_retire_pending) {
    FlashLog_MarkRecoverySent(hlog, retire_live);
  }
  if (walker_retire_pending) {
    FlashLog_MarkRecoverySent(hlog, retire_walker);
  }

  return FLASH_LOG_OK;
}

/* R3-04 (#218, DDR-0005 BR-TX-009/010/011): the one-pass watermark advances
 * AT SEND TIME - there is no commit-on-ACK and no autonomous retry.
 * Supersedes FlashLog_CommitThrough (R2-02/#106) and the count-based
 * FlashLog_MarkRecordsTransmitted.
 *
 * Three edges (invariant: recovery_frontier <= tx_high_water <=
 * pending_frontier <= next_sequence):
 *   pending_frontier B - BEH-05 (#286): the pending-live drain edge; a send
 *                        or leading-run retire inside [H, B) lowers B := s.
 *                        When s == H the edges have met: the whole pending
 *                        range is sent/retired, so H lifts past it
 *                        (H := old B) and nothing is ever re-offered.
 *   tx_high_water H    - bottom edge of the pending drain; rises on the
 *                        edge-meet fold (and clamps F down to H).
 *   recovery_frontier F - walker has visited every seq >= F; s < F lowers
 *                        F := s (contiguous send or leading-run retire).
 * A mark that lands in an already-covered zone (above B, or in [F, H)) is
 * already accounted for -> no-op. */
FlashLog_StatusTypeDef FlashLog_MarkRecoverySent(FlashLog_HandleTypeDef *hlog, uint32_t sequence) {
  if (hlog == NULL || !hlog->initialized) {
    return FLASH_LOG_ERROR_PARAM;
  }

  bool moved = false;
  if (sequence >= hlog->tx_high_water && sequence < hlog->pending_frontier) {
    hlog->pending_frontier = sequence;
    moved = true;
    if (sequence == hlog->tx_high_water) {
      /* Episode complete: everything below the drain top is covered. Lift
       * the bottom edge to it, then fold in whatever accumulated above the
       * edge mid-episode (deferred writes drain; live-sent records re-offer
       * once - the backend dedupes by sequence). */
      hlog->tx_high_water = hlog->drain_top;
      hlog->pending_frontier = hlog->next_sequence;
      hlog->drain_top = hlog->next_sequence;
    }
  } else if (sequence == hlog->tx_high_water &&
             hlog->pending_frontier == hlog->tx_high_water) {
    /* Drained pending range: the bottom-edge lift (defensive; the normal
     * write -> live-send flow is the drain rule above). */
    hlog->tx_high_water = sequence + 1U;
    hlog->pending_frontier = hlog->tx_high_water;
    moved = true;
  } else if (sequence < hlog->recovery_frontier) {
    hlog->recovery_frontier = sequence;
    moved = true;
  }
  if (hlog->recovery_frontier > hlog->tx_high_water) {
    hlog->recovery_frontier = hlog->tx_high_water; /* F <= H invariant */
  }
  if (!moved) {
    return FLASH_LOG_OK;
  }

  /* Finding #8 (2026-08-10): deferred header sync during a bulk burst — N
   * sent packets cost ONE 4 KB header sector erase at burst end instead of
   * N (the two header sectors take all the wear; W25Q16JV is rated 100k
   * erase cycles). A mid-burst reset replays from the last synced
   * watermarks — at most one burst retransmitted, deduped by the backend. */
  if (hlog->sync_deferred) {
    hlog->header_dirty = 1;
    return FLASH_LOG_OK;
  }

  /* Update header to persist transmission tracking */
  return FlashLog_SyncHeader(hlog);
}

void FlashLog_DeferHeaderSync(FlashLog_HandleTypeDef *hlog) {
  if (hlog != NULL) {
    hlog->sync_deferred = 1;
  }
}

FlashLog_StatusTypeDef FlashLog_FlushHeaderSync(FlashLog_HandleTypeDef *hlog) {
  if (hlog == NULL || !hlog->initialized) {
    return FLASH_LOG_ERROR_PARAM;
  }
  hlog->sync_deferred = 0;
  if (!hlog->header_dirty) {
    return FLASH_LOG_OK;
  }
  /* Stay dirty on failure so the next flush retries (never wedges: the RAM
   * watermark is still correct for this boot, and a reset replays from the
   * last persisted one). */
  FlashLog_StatusTypeDef st = FlashLog_SyncHeader(hlog);
  if (st == FLASH_LOG_OK) {
    hlog->header_dirty = 0;
  }
  return st;
}

uint32_t FlashLog_GetUnsentCount(FlashLog_HandleTypeDef *hlog) {
  if (hlog == NULL || !hlog->initialized) {
    return 0;
  }

  /* v5 (R3-04/#218) + BEH-05 (#286): pending-drain + walker-eligible. */
  uint32_t count = 0;
  if (hlog->pending_frontier > hlog->tx_high_water) {
    count += hlog->pending_frontier - hlog->tx_high_water;
  }
  uint32_t oldest = FlashLog_OldestSequence(hlog);
  if (hlog->recovery_frontier > oldest) {
    count += hlog->recovery_frontier - oldest;
  }
  return count;
}

/**
 * @brief  Validate header magic, version, and CRC
 */
static bool FlashLog_ValidateHeader(const FlashLog_HandleTypeDef *hlog, const FlashLog_Header_t *header) {
  uint32_t calculated_crc;

  /* Check magic number */
  if (header->magic != FLASH_LOG_HEADER_MAGIC) {
    return false;
  }

  /* Check version. R3-04 (#218): v4 (legacy FIFO watermark) headers are
   * ACCEPTED and migrated by Init (the retained archive is re-walked once,
   * newest-first; the backend dedupes by sequence). Older than v4: clean
   * init as before. */
  if (header->version != FLASH_LOG_HEADER_VERSION &&
      header->version != FLASH_LOG_HEADER_VERSION_LEGACY_FIFO) {
    return false;
  }

  /* F-021 (#55): beyond magic+CRC, header field CONTENTS must be plausible
   * before a header is trusted — a CRC-valid but semantically impossible
   * header must not steer the log. */
  /* Bounds follow the RUNTIME device capacity (hlog->data_end), so a
   * W25Q80 bench board validates its own compressed ring correctly. */
  if (header->write_addr < FLASH_LOG_DATA_START || header->write_addr >= hlog->data_end ||
      (header->write_addr % FLASH_LOG_RECORD_SIZE) != 0) {
    return false; /* write frontier outside the data region or unaligned */
  }
  if (header->oldest_addr < FLASH_LOG_DATA_START || header->oldest_addr >= hlog->data_end ||
      (header->oldest_addr % FLASH_LOG_RECORD_SIZE) != 0) {
    return false;
  }
  /* F-02 (#66): record_count is a MONOTONIC total-written counter and
   * legitimately exceeds FLASH_LOG_MAX_RECORDS once the ring wraps
   * (flash_log.h: "may exceed MAX if wrapped"; the write path at
   * FlashLog_WriteRecord increments it unbounded and GetRecordCount()/
   * HasWrapped() already handle > MAX). The ceiling check added by F-021
   * (#55) rejected exactly those legitimate wrapped headers: ~day 113 at
   * 5-min cadence BOTH headers failed validation and the next reboot wiped
   * the record index and TX watermark. The watermark relation below is the
   * meaningful plausibility constraint on record_count. */
  if (header->last_transmitted_seq > header->record_count) {
    return false; /* watermark ahead of the frontier is impossible */
  }
  /* R3-04 (#218): v5 carries the walker frontier in reserved[0]; it must
   * satisfy the same bound and the F <= H invariant. (v4 headers predate
   * the frontier — Init migrates them, no check here.) */
  if (header->version == FLASH_LOG_HEADER_VERSION &&
      (header->reserved[0] > header->record_count ||
       header->reserved[0] > header->last_transmitted_seq)) {
    return false;
  }

  /* Validate CRC32 */
  calculated_crc = FlashLog_CRC32((const uint8_t *)header,
                                  sizeof(FlashLog_Header_t) - sizeof(uint32_t));

  return (calculated_crc == header->crc32);
}

/**
 * @brief  Write current header to flash (ping-pong update)
 */
static FlashLog_StatusTypeDef FlashLog_WriteHeader(FlashLog_HandleTypeDef *hlog) {
  FlashLog_Header_t header;
  W25Q_StatusTypeDef status;
  uint32_t write_addr;

  /* Prepare header */
  memset(&header, 0, sizeof(header));
  header.magic = FLASH_LOG_HEADER_MAGIC;
  header.version = FLASH_LOG_HEADER_VERSION;
  header.write_addr = hlog->write_addr;
  header.record_count = hlog->record_count;
  /* F-007/R12 (#50): real monotonic generation — record_count was NOT a
   * freshness counter (a watermark-only SyncHeader left two valid headers
   * with equal "freshness" and different watermarks; the tie-break could
   * resurrect an older watermark → duplicate retransmission). */
  header.sequence = hlog->header_generation;
  header.oldest_addr = hlog->oldest_addr;
  header.flags = 0;
  header.last_transmitted_seq = hlog->tx_high_water; /* v5 semantics */
  header.reserved[0] = hlog->recovery_frontier;      /* v5: walker */
  header.reserved[1] = hlog->pending_frontier;       /* BEH-05: drain edge */

  /* Calculate CRC32 (all fields except crc32 itself) */
  header.crc32 = FlashLog_CRC32((const uint8_t *)&header,
                                sizeof(FlashLog_Header_t) - sizeof(uint32_t));

  /* Ping-pong between header A and B for wear leveling.
   * #135 (2026-08-10 finding #4): target the INACTIVE slot but commit
   * active_header ONLY after a successful program. Toggling first meant two
   * consecutive failed writes (erase OK, program fails) erased BOTH slots —
   * the second call flipped back and destroyed the surviving good header,
   * orphaning the whole archive (record_count/watermark reset to 0). */
  uint8_t target = (hlog->active_header == 0) ? 1U : 0U;
  write_addr = (target == 0) ? HEADER_A_ADDR : HEADER_B_ADDR;

  /* T4 FIX (FlashStorageNotes.md): erase-before-write invariant. Each header lives in its
   * own sector, so erasing it can only destroy the STALE copy — the other
   * (current) header survives in its own sector. Without this erase, NOR
   * flash rewrites silently corrupt (bits only flip 1->0). */
  status = W25Q_EraseSector(hlog->hw25q, write_addr);
  if (status != W25Q_OK) {
    return FLASH_LOG_ERROR_FLASH;
  }

  status = W25Q_Write(hlog->hw25q, write_addr, (const uint8_t *)&header, sizeof(header));
  if (status != W25Q_OK) {
    return FLASH_LOG_ERROR_FLASH;
  }

  hlog->active_header = target; /* #135: commit ONLY after a successful program */
  hlog->header_generation++;    /* F-007/R12 (#50): monotonic per successful write */

  return FLASH_LOG_OK;
}

/**
 * @brief  Calculate record address from index
 */
static uint32_t FlashLog_GetRecordAddress(FlashLog_HandleTypeDef *hlog, uint32_t record_index) {
  uint32_t offset = (record_index % hlog->max_records) * FLASH_LOG_RECORD_SIZE;
  return FLASH_LOG_DATA_START + offset;
}

/**
 * @brief  Erase sector if we're about to overwrite old data
 */
static FlashLog_StatusTypeDef FlashLog_EraseSectorIfNeeded(FlashLog_HandleTypeDef *hlog, uint32_t addr) {
  W25Q_StatusTypeDef status;
  uint32_t sector_addr = (addr / W25Q_SECTOR_SIZE) * W25Q_SECTOR_SIZE;

  /* Only erase if this is the start of a new sector */
  if ((addr % W25Q_SECTOR_SIZE) == 0) {
    status = W25Q_EraseSector(hlog->hw25q, sector_addr);
    if (status != W25Q_OK) {
      return FLASH_LOG_ERROR_FLASH;
    }
    /* FR-10 (#90): on a wrapped ring this erase destroys up to 63
     * not-yet-overwritten records ahead of the frontier. oldest_addr is
     * maintained at the write site with the sector-boundary convention:
     * once wrapped, oldest_addr = end of the sector containing
     * write_addr, i.e. the oldest slot that can still hold valid data. */
  }

  return FLASH_LOG_OK;
}

/* Public Functions ----------------------------------------------------------*/

FlashLog_StatusTypeDef FlashLog_Init(FlashLog_HandleTypeDef *hlog, W25Q_HandleTypeDef *hw25q) {
  FlashLog_Header_t header_a, header_b;
  FlashLog_StatusTypeDef status;
  bool valid_a, valid_b;
  bool from_v4 = false; /* R3-04 (#218): legacy FIFO watermark migration */
  if (hlog == NULL || hw25q == NULL) {
    return FLASH_LOG_ERROR_PARAM;
  }

  /* Clear handle */
  memset(hlog, 0, sizeof(FlashLog_HandleTypeDef));
  hlog->hw25q = hw25q;

  /* Resolve the ring geometry from the actual device (W25Q16 flight part or
   * W25Q80 bench fallback). Header validation below uses these bounds. */
  hlog->data_end = W25Q_GetCapacity(hw25q);
  hlog->max_records = (hlog->data_end - FLASH_LOG_DATA_START) / FLASH_LOG_RECORD_SIZE;

  /* Initialize CRC32 table */
  FlashLog_InitCRC32Table();

  /* Try to read both headers */
  status = FlashLog_ReadHeader(hlog, HEADER_A_ADDR, &header_a);
  if (status != FLASH_LOG_OK) {
    return status;
  }

  status = FlashLog_ReadHeader(hlog, HEADER_B_ADDR, &header_b);
  if (status != FLASH_LOG_OK) {
    return status;
  }

  /* Validate headers */
  valid_a = FlashLog_ValidateHeader(hlog, &header_a);
  valid_b = FlashLog_ValidateHeader(hlog, &header_b);

  if (valid_a && valid_b) {
    /* Both valid — F-007/R12 (#50): sequence is now a real generation
     * counter; pick the newest with a wrap-safe compare. */
    bool a_newer = ((int32_t)(header_a.sequence - header_b.sequence) > 0);
    if (a_newer) {
      hlog->write_addr = header_a.write_addr;
      hlog->record_count = header_a.record_count;
      hlog->oldest_addr = header_a.oldest_addr;
      hlog->tx_high_water = header_a.last_transmitted_seq;
      hlog->recovery_frontier = header_a.reserved[0];
      hlog->header_generation = header_a.sequence + 1;
      hlog->active_header = 0;
      from_v4 = (header_a.version == FLASH_LOG_HEADER_VERSION_LEGACY_FIFO);
    } else {
      hlog->write_addr = header_b.write_addr;
      hlog->record_count = header_b.record_count;
      hlog->oldest_addr = header_b.oldest_addr;
      hlog->tx_high_water = header_b.last_transmitted_seq;
      hlog->recovery_frontier = header_b.reserved[0];
      hlog->header_generation = header_b.sequence + 1;
      hlog->active_header = 1;
      from_v4 = (header_b.version == FLASH_LOG_HEADER_VERSION_LEGACY_FIFO);
    }
  } else if (valid_a) {
    /* Only A valid */
    hlog->write_addr = header_a.write_addr;
    hlog->record_count = header_a.record_count;
    hlog->oldest_addr = header_a.oldest_addr;
    hlog->tx_high_water = header_a.last_transmitted_seq;
    hlog->recovery_frontier = header_a.reserved[0];
    hlog->header_generation = header_a.sequence + 1;
    hlog->active_header = 0;
    from_v4 = (header_a.version == FLASH_LOG_HEADER_VERSION_LEGACY_FIFO);
  } else if (valid_b) {
    /* Only B valid */
    hlog->write_addr = header_b.write_addr;
    hlog->record_count = header_b.record_count;
    hlog->oldest_addr = header_b.oldest_addr;
    hlog->tx_high_water = header_b.last_transmitted_seq;
    hlog->recovery_frontier = header_b.reserved[0];
    hlog->header_generation = header_b.sequence + 1;
    hlog->active_header = 1;
    from_v4 = (header_b.version == FLASH_LOG_HEADER_VERSION_LEGACY_FIFO);
  } else {
    /* No valid headers - R3-07 (#221): do NOT immediately treat the
     * archive as disposable. The data ring may still hold thousands of
     * valid CRC-protected records; reconstruct the state from them. Only
     * a demonstrably blank ring falls through to a fresh init. */
    FlashLog_Record_t probe;
    uint32_t valid = 0;
    uint32_t max_seq = 0, min_seq = 0;
    uint32_t max_slot = 0, min_slot = 0;
    for (uint32_t slot = 0; slot < hlog->max_records; slot++) {
      uint32_t addr = FLASH_LOG_DATA_START + slot * FLASH_LOG_RECORD_SIZE;
      if (W25Q_Read(hw25q, addr, (uint8_t *)&probe, sizeof(probe)) != W25Q_OK) {
        return FLASH_LOG_ERROR_FLASH;
      }
      if (!FlashLog_VerifyRecord(&probe)) {
        continue; /* blank (0xFF), torn, or corrupt slot */
      }
      if (valid == 0 || probe.sequence > max_seq) {
        max_seq = probe.sequence;
        max_slot = slot;
      }
      if (valid == 0 || probe.sequence < min_seq) {
        min_seq = probe.sequence;
        min_slot = slot;
      }
      valid++;
    }

    if (valid > 0) {
      /* Reconstruct a plausible frontier: identity continues ABOVE the
       * maximum sequence seen (never reused); the write address resumes
       * after the newest record's slot (wrap-aware). Both watermarks
       * restart at 0: the whole recovered ring becomes pending-live and
       * re-drains once (the backend dedupes by sequence) - recovering
       * possibly-unsent science beats the risk of silently dropping it. */
      hlog->record_count = max_seq + 1U;
      hlog->write_addr = FLASH_LOG_DATA_START +
                         (((max_slot + 1U) % hlog->max_records) * FLASH_LOG_RECORD_SIZE);
      hlog->oldest_addr = FLASH_LOG_DATA_START + min_slot * FLASH_LOG_RECORD_SIZE;
      hlog->tx_high_water = 0;
      hlog->recovery_frontier = 0;
      hlog->active_header = 0;
      SONDE_LOG_STR("FLASH LOG: both headers invalid - RECONSTRUCTED from data ring\r\n");

      /* Persist the reconstructed header (a reset mid-write simply
       * triggers reconstruction again - idempotent). */
      W25Q_EraseSector(hw25q, HEADER_A_ADDR);
      W25Q_EraseSector(hw25q, HEADER_B_ADDR);
      status = FlashLog_WriteHeader(hlog);
      if (status != FLASH_LOG_OK) {
        return status;
      }
    } else {
      /* Demonstrably blank - initialize fresh */
      hlog->write_addr = FLASH_LOG_DATA_START;
      hlog->record_count = 0;
      hlog->oldest_addr = FLASH_LOG_DATA_START;
      hlog->active_header = 0;

      /* Erase BOTH header sectors to start fresh (T4: separate sectors) */
      W25Q_EraseSector(hw25q, HEADER_A_ADDR);
      W25Q_EraseSector(hw25q, HEADER_B_ADDR);

      /* Write initial header */
      status = FlashLog_WriteHeader(hlog);
      if (status != FLASH_LOG_OK) {
        return status;
      }
    }
  }

  /* FW-12 (FlashStorageNotes.md): the header checkpoints every HEADER_UPDATE_INTERVAL
   * records; recover the uncheckpointed tail before trusting the frontier. */
  if (valid_a || valid_b) {
    status = FlashLog_FrontierScan(hlog);
    if (status != FLASH_LOG_OK) {
      return status;
    }
  }

  hlog->next_sequence = hlog->record_count;

  /* R3-04 (#218): v4 -> v5 migration. The legacy rising FIFO commit
   * watermark has no usable direction under one-pass semantics; restart
   * both watermarks at the top so the whole RETAINED archive becomes
   * walker-eligible exactly once (newest-first). Records sent under v4
   * are re-sent at most once; the backend dedupes by sequence. */
  if (from_v4) {
    hlog->tx_high_water = hlog->next_sequence;
    hlog->recovery_frontier = hlog->next_sequence;
  }

  /* BEH-05 (#286): the pending-live drain edge, persisted in reserved[1].
   * Headers written before this change carry 0 there (memset); a zero
   * edge means "no persisted edge" - restart the drain at the top: the
   * pending range re-drains once, newest-first, deduped by the backend
   * (same discipline as the v4 migration). Post-fix firmware never
   * persists a zero edge with pending data (every write sets B := next). */
  {
    uint32_t persisted_pending = 0;
    if (valid_a || valid_b) {
      persisted_pending = (hlog->active_header == 0) ? header_a.reserved[1]
                                                     : header_b.reserved[1];
    }
    hlog->pending_frontier = persisted_pending;
    if (hlog->pending_frontier == 0 ||
        hlog->pending_frontier > hlog->next_sequence) {
      hlog->pending_frontier = hlog->next_sequence;
    }
    if (hlog->pending_frontier < hlog->tx_high_water) {
      hlog->pending_frontier = hlog->tx_high_water;
    }
    /* The episode top is RAM-only: a reset resumes the drain from the
     * persisted edge, so the resumed episode's top is that edge. */
    hlog->drain_top = hlog->pending_frontier;
  }

  hlog->initialized = true;

  return FLASH_LOG_OK;
}

/**
 * @brief  FW-12 (FlashStorageNotes.md): frontier scan. The header is only persisted every
 *         HEADER_UPDATE_INTERVAL records, so up to that many records can be
 *         written-but-uncheckpointed when power is cut. Trusting the stale
 *         header blindly would reuse those sequence numbers and wedge
 *         MarkRecordsTransmitted. Scan forward from the header write_addr
 *         across the possible uncheckpointed window (wrap-aware); a record
 *         whose magic+CRC verifies extends the recovered frontier.
 * @note   Erase-ahead means the next-record slot reads 0xFF (magic check
 *         fails fast); a CRC failure here is a torn record at the true
 *         frontier, not an error.
 */
static FlashLog_StatusTypeDef FlashLog_FrontierScan(FlashLog_HandleTypeDef *hlog) {
  FlashLog_Record_t probe;

  /* F-008 (#49): no early return on record_count == 0 — a fresh count-0
   * header followed by 1-9 uncheckpointed records and a power cut would
   * orphan them (write_addr reused, records overwritten). The scan from
   * DATA_START is bounded (<= HEADER_UPDATE_INTERVAL probes) and safe. */

  for (uint32_t i = 0; i < HEADER_UPDATE_INTERVAL; i++) {
    if (W25Q_Read(hlog->hw25q, hlog->write_addr,
                  (uint8_t *)&probe, sizeof(probe)) != W25Q_OK) {
      return FLASH_LOG_ERROR_FLASH; /* real hardware error */
    }
    if (!FlashLog_VerifyRecord(&probe)) {
      break; /* erased or torn slot: frontier found */
    }
    /* 2026-08-11 handoff §6c: identity check. On a WRAPPED ring whose true
     * frontier sits exactly on a sector boundary, the next sector was never
     * erased and still holds valid previous-lap records — magic+CRC pass.
     * Without this the scan adopted up to 10 stale records, inflated
     * record_count, and the next write landed on an un-erased slot. The
     * record at the true frontier must carry exactly the current count. */
    if (probe.sequence != hlog->record_count) {
      break; /* valid but from a previous lap: frontier found */
    }
    hlog->write_addr += FLASH_LOG_RECORD_SIZE;
    if (hlog->write_addr >= hlog->data_end) {
      hlog->write_addr = FLASH_LOG_DATA_START;
    }
    hlog->record_count++;
    if (hlog->record_count > hlog->max_records) {
      /* FR-10 (#90): sector-boundary convention — see WriteRecord */
      hlog->oldest_addr = ((hlog->write_addr / W25Q_SECTOR_SIZE) + 1U) * W25Q_SECTOR_SIZE;
      if (hlog->oldest_addr >= hlog->data_end) {
        hlog->oldest_addr = FLASH_LOG_DATA_START;
      }
    }
  }
  return FLASH_LOG_OK;
}

FlashLog_StatusTypeDef FlashLog_DeInit(FlashLog_HandleTypeDef *hlog) {
  if (hlog == NULL) {
    return FLASH_LOG_ERROR_PARAM;
  }

  /* Sync header before deinit */
  FlashLog_SyncHeader(hlog);

  hlog->initialized = false;
  hlog->hw25q = NULL;

  return FLASH_LOG_OK;
}

FlashLog_StatusTypeDef FlashLog_WriteRecord(FlashLog_HandleTypeDef *hlog,
                                            const sensor_t *sensor_data,
                                            uint32_t timestamp,
                                            int16_t voltage_slope,
                                            uint8_t power_mode,
                                            uint8_t veto) {
  FlashLog_Record_t record;
  W25Q_StatusTypeDef status;
  FlashLog_StatusTypeDef log_status;

  if (hlog == NULL || !hlog->initialized || sensor_data == NULL) {
    return FLASH_LOG_ERROR_PARAM;
  }

  /* FR-12 (#292): a failed periodic checkpoint (below) left the durable
   * header stale with no retry until the next scheduled checkpoint -
   * consecutive failures (worn header sector, sagging rail) pushed it more
   * than HEADER_UPDATE_INTERVAL records behind, and the bounded boot
   * FrontierScan then reclaimed a write address still holding valid
   * records. Retry the sync BEFORE appending so the staleness window never
   * exceeds one record. Non-fatal on failure: the flag stays set and the
   * append proceeds. The !sync_deferred guard keeps the finding-#8 burst
   * batching intact (a deferred burst sync is owned by FlushHeaderSync). */
  if (hlog->header_dirty && !hlog->sync_deferred) {
    if (FlashLog_SyncHeader(hlog) == FLASH_LOG_OK) {
      hlog->header_dirty = 0;
    }
  }

  /* Erase sector if starting new sector */
  log_status = FlashLog_EraseSectorIfNeeded(hlog, hlog->write_addr);
  if (log_status != FLASH_LOG_OK) {
    return log_status;
  }

  /* Build record */
  memset(&record, 0, sizeof(record));

  /* Header */
  record.magic = FLASH_LOG_RECORD_MAGIC;
  record.sequence = hlog->next_sequence; /* F-010 (#52): incremented only after write success below */
  record.timestamp = timestamp;

  /* Environmental sensors */
  record.pressure = sensor_data->pressure;
  record.temperature = sensor_data->temperature;
  record.humidity = sensor_data->humidity;

  /* GNSS data */
  record.latitude = sensor_data->latitude;
  record.longitude = sensor_data->longitude;
  record.altitude_gps = sensor_data->altitudeGps; /* D5/#35: int32 */
  /* altitude_bar deleted (D5/#35) — ground computes it */
  record.satellites = sensor_data->satellites;
  record.gnss_fix_quality = sensor_data->gnss_fix_quality;
  /* F-16 (#71): clamp before casting — NaN or out-of-range float->int is
   * undefined behaviour. (!(x > 0)) also catches NaN. Mirrors the solar_mv
   * clamp below. */
  {
    float hdop_x10_f = sensor_data->gnss_hdop * 10.0f;
    if (!(hdop_x10_f > 0.0f))
      hdop_x10_f = 0.0f;
    if (hdop_x10_f > 255.0f)
      hdop_x10_f = 255.0f;
    record.gnss_hdop_x10 = (uint8_t)(hdop_x10_f + 0.5f);
  }
  record.gnss_valid = sensor_data->gnss_valid ? 1 : 0;

  /* Power + status (D5/#35: solar/slope/mode archived at write time — F-025/R19) */
  { /* F-16 (#71): clamp like solar_mv below — NaN/out-of-range cast is UB */
    float batt_mv_f = sensor_data->battery_voltage * 1000.0f;
    if (!(batt_mv_f > 0.0f))
      batt_mv_f = 0.0f;
    if (batt_mv_f > 65535.0f)
      batt_mv_f = 65535.0f;
    record.battery_mv = (uint16_t)(batt_mv_f + 0.5f);
  }
  {
    float solar_mv_f = sensor_data->solar_voltage * 1000.0f;
    if (solar_mv_f < 0.0f)
      solar_mv_f = 0.0f;
    if (solar_mv_f > 65535.0f)
      solar_mv_f = 65535.0f;
    record.solar_mv = (uint16_t)(solar_mv_f + 0.5f);
  }
  record.voltage_slope = voltage_slope;
  record.power_mode = power_mode;

  /* FW-7 (DDR-0003): archive carries the reading's own freshness */
  record.flags = (sensor_data->press_stale ? 0x01 : 0) | (sensor_data->temp_stale ? 0x02 : 0) | (sensor_data->hum_stale ? 0x04 : 0) | (sensor_data->gnss_stale ? 0x08 : 0) | (sensor_data->batt_stale ? 0x10 : 0) /* #136 */
                 | (uint8_t)((veto & 0x07U) << 5);                                                                                                                                                                /* §6a: DDR-0003 record WHY */

  /* Calculate CRC32 (all fields except crc32) */
  record.crc32 = FlashLog_CRC32((const uint8_t *)&record,
                                sizeof(FlashLog_Record_t) - sizeof(uint32_t));

  /* Write record to flash */
  status = W25Q_Write(hlog->hw25q, hlog->write_addr,
                      (const uint8_t *)&record, sizeof(record));
  if (status != W25Q_OK) {
    return FLASH_LOG_ERROR_FLASH;
  }

  /* Update write pointer (F-010: sequence consumed only now, after success) */
  hlog->next_sequence++;
  /* BEH-05 (#286): the new record joins the top of the pending drain -
   * but only when nothing sits above the drain edge. Mid-episode (a
   * partial drain sent the top records already) the write DEFERS: the
   * record is folded in when the episode completes, so an in-flight
   * drain never re-offers what it already sent this episode. */
  if (hlog->pending_frontier == hlog->next_sequence - 1U) {
    hlog->pending_frontier = hlog->next_sequence;
    hlog->drain_top = hlog->next_sequence;
  }
  hlog->write_addr += FLASH_LOG_RECORD_SIZE;
  hlog->record_count++;

  /* Handle wraparound */
  if (hlog->write_addr >= hlog->data_end) {
    hlog->write_addr = FLASH_LOG_DATA_START;
  }

  /* Update oldest address if we wrapped.
   * FR-10 (#90): erase-ahead destroys the whole sector containing
   * write_addr before its first record is written, so the slots between
   * write_addr and that sector's end are erased, not available. The old
   * convention (oldest = write_addr) counted them anyway — up to 63
   * phantom records reported as corrupt on every post-wrap bulk read.
   * Now: oldest_addr = end of the sector containing write_addr — the
   * oldest slot that can still hold valid data. */
  if (hlog->record_count > hlog->max_records) {
    hlog->oldest_addr = ((hlog->write_addr / W25Q_SECTOR_SIZE) + 1U) * W25Q_SECTOR_SIZE;
    if (hlog->oldest_addr >= hlog->data_end) {
      hlog->oldest_addr = FLASH_LOG_DATA_START;
    }
  }

  /* Periodically update header to flash */
  if ((hlog->record_count % HEADER_UPDATE_INTERVAL) == 0) {
    log_status = FlashLog_WriteHeader(hlog);
    if (log_status != FLASH_LOG_OK) {
      /* FR-12 (#292): the RECORD is durable - only the checkpoint
       * failed. Do not report the record write as failed (the caller
       * would treat a good record as lost); defer the sync instead.
       * The retry at the top of the next WriteRecord closes the
       * window. */
      hlog->header_dirty = 1;
    }
  }

  return FLASH_LOG_OK;
}

FlashLog_StatusTypeDef FlashLog_ReadRecord(FlashLog_HandleTypeDef *hlog,
                                           FlashLog_Record_t *record,
                                           uint32_t offset) {
  W25Q_StatusTypeDef status;
  uint32_t available_records;
  uint32_t read_addr;
  uint32_t record_index;

  if (hlog == NULL || !hlog->initialized || record == NULL) {
    return FLASH_LOG_ERROR_PARAM;
  }

  available_records = FlashLog_GetAvailableRecords(hlog);

  if (offset >= available_records) {
    return FLASH_LOG_ERROR_EMPTY;
  }

  /* Calculate address: most recent is one record before write_addr */
  /* For LIFO: record 0 = newest, record N = oldest */
  record_index = hlog->next_sequence - 1 - offset;
  read_addr = FlashLog_GetRecordAddress(hlog, record_index);

  /* Read record */
  status = W25Q_Read(hlog->hw25q, read_addr, (uint8_t *)record, sizeof(FlashLog_Record_t));
  if (status != W25Q_OK) {
    return FLASH_LOG_ERROR_FLASH;
  }

  /* Validate record */
  if (!FlashLog_VerifyRecord(record)) {
    return FLASH_LOG_ERROR_CRC;
  }

  return FLASH_LOG_OK;
}

FlashLog_StatusTypeDef FlashLog_ReadRecords(FlashLog_HandleTypeDef *hlog,
                                            FlashLog_Record_t *records,
                                            uint32_t max_count,
                                            uint32_t *actual_count,
                                            uint32_t start_offset) {
  FlashLog_StatusTypeDef status;
  uint32_t i;
  uint32_t available;

  if (hlog == NULL || !hlog->initialized || records == NULL || actual_count == NULL) {
    return FLASH_LOG_ERROR_PARAM;
  }

  *actual_count = 0;
  available = FlashLog_GetAvailableRecords(hlog);

  if (start_offset >= available) {
    return FLASH_LOG_ERROR_EMPTY;
  }

  /* Read up to max_count records or until we run out */
  for (i = 0; i < max_count; i++) {
    uint32_t offset = start_offset + i;

    if (offset >= available) {
      break; /* No more records */
    }

    status = FlashLog_ReadRecord(hlog, &records[i], offset);
    if (status != FLASH_LOG_OK) {
      return status; /* Stop on error */
    }

    (*actual_count)++;
  }

  return FLASH_LOG_OK;
}

uint32_t FlashLog_GetRecordCount(FlashLog_HandleTypeDef *hlog) {
  if (hlog == NULL || !hlog->initialized) {
    return 0;
  }

  return hlog->record_count;
}

uint32_t FlashLog_GetAvailableRecords(FlashLog_HandleTypeDef *hlog) {
  if (hlog == NULL || !hlog->initialized) {
    return 0;
  }

  /* If we haven't wrapped yet, all records are available */
  if (hlog->record_count <= hlog->max_records) {
    return hlog->record_count;
  }

  /* FR-10 (#90): after wrap, available = the valid span between
   * oldest_addr and write_addr along the ring. With the sector-boundary
   * oldest convention this excludes the erase-ahead slack (erased 0xFF
   * slots) that min(record_count, MAX) used to count. */
  uint32_t span;
  if (hlog->write_addr >= hlog->oldest_addr) {
    span = hlog->write_addr - hlog->oldest_addr;
  } else {
    span = (hlog->data_end - hlog->oldest_addr) +
           (hlog->write_addr - FLASH_LOG_DATA_START);
  }
  return span / FLASH_LOG_RECORD_SIZE;
}

bool FlashLog_HasWrapped(FlashLog_HandleTypeDef *hlog) {
  if (hlog == NULL || !hlog->initialized) {
    return false;
  }

  return (hlog->record_count > hlog->max_records);
}

FlashLog_StatusTypeDef FlashLog_SyncHeader(FlashLog_HandleTypeDef *hlog) {
  if (hlog == NULL || !hlog->initialized) {
    return FLASH_LOG_ERROR_PARAM;
  }

  return FlashLog_WriteHeader(hlog);
}

bool FlashLog_VerifyRecord(const FlashLog_Record_t *record) {
  uint32_t calculated_crc;

  if (record == NULL) {
    return false;
  }

  /* Check magic number */
  if (record->magic != FLASH_LOG_RECORD_MAGIC) {
    return false;
  }

  /* Verify CRC32 */
  calculated_crc = FlashLog_CRC32((const uint8_t *)record,
                                  sizeof(FlashLog_Record_t) - sizeof(uint32_t));

  return (calculated_crc == record->crc32);
}

FlashLog_StatusTypeDef FlashLog_GetStats(FlashLog_HandleTypeDef *hlog,
                                         uint32_t *total_capacity,
                                         uint32_t *used_records,
                                         uint32_t *free_records) {
  uint32_t available;

  if (hlog == NULL || !hlog->initialized) {
    return FLASH_LOG_ERROR_PARAM;
  }

  available = FlashLog_GetAvailableRecords(hlog);

  if (total_capacity != NULL) {
    *total_capacity = hlog->max_records;
  }

  if (used_records != NULL) {
    *used_records = available;
  }

  if (free_records != NULL) {
    if (hlog->record_count < hlog->max_records) {
      *free_records = hlog->max_records - hlog->record_count;
    } else {
      *free_records = 0; /* Circular buffer is full, will overwrite */
    }
  }

  return FLASH_LOG_OK;
}
