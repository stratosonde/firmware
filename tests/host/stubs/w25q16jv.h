/* Host-test stub: W25Q flash header (R49). Only what flash_log.h needs. */
#ifndef W25Q16JV_H_STUB
#define W25Q16JV_H_STUB
/* Suppress the real Core/Inc/w25q16jv.h: flash_log.h includes "w25q16jv.h",
 * and a quoted include resolves against the includer's directory first, which
 * would pull the real (SPI-dependent) header in alongside this stub. Claiming
 * its guard keeps exactly one definition in play. */
#define __W25Q16JV_H
#include <stdint.h>
#include <stdbool.h>
#define W25Q_SECTOR_SIZE  4096U
#define W25Q_FLASH_SIZE   (2U * 1024U * 1024U)
typedef enum { W25Q_OK = 0, W25Q_ERROR, W25Q_ERROR_INIT } W25Q_StatusTypeDef;
typedef struct { int _host_stub; } W25Q_HandleTypeDef;

/* FR-10 / T-7: full API surface used by flash_log.c. Implemented by
 * tests/host/fake_w25q.c (simulated NOR semantics). */
W25Q_StatusTypeDef W25Q_Read(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                             uint8_t *data, uint32_t len);
W25Q_StatusTypeDef W25Q_Write(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                              const uint8_t *data, uint32_t len);
W25Q_StatusTypeDef W25Q_EraseSector(W25Q_HandleTypeDef *hw25q, uint32_t addr);
#endif
