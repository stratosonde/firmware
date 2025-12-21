/**
  ******************************************************************************
  * @file    w25q16jv.h
  * @brief   W25Q16JV SPI Flash Driver Header
  * @details Low-level driver for Winbond W25Q16JV 16Mbit (2MB) SPI NOR Flash
  *          Used for sensor data logging on the radiosonde
  *
  * Hardware Connection (SPI2):
  *   - PA10: MOSI (SPI2_MOSI)
  *   - PB14: MISO (SPI2_MISO)
  *   - PB13: SCK  (SPI2_SCK)
  *   - PB9:  CS   (SPI2_NSS)
  *
  * Memory Organization:
  *   - Total:   2MB (2,097,152 bytes)
  *   - Pages:   256 bytes each (8,192 pages)
  *   - Sectors: 4KB each (512 sectors) - smallest erase unit
  *   - Blocks:  64KB each (32 blocks)
  *
  ******************************************************************************
  */

#ifndef __W25Q16JV_H
#define __W25Q16JV_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32wlxx_hal.h"

/* Exported defines ----------------------------------------------------------*/

/* Memory size definitions */
#define W25Q_FLASH_SIZE           (2 * 1024 * 1024)  /* 2MB total */
#define W25Q_PAGE_SIZE            256                 /* 256 bytes per page */
#define W25Q_SECTOR_SIZE          (4 * 1024)          /* 4KB per sector */
#define W25Q_BLOCK_SIZE_32K       (32 * 1024)         /* 32KB block */
#define W25Q_BLOCK_SIZE_64K       (64 * 1024)         /* 64KB block */

#define W25Q_PAGE_COUNT           (W25Q_FLASH_SIZE / W25Q_PAGE_SIZE)     /* 8192 pages */
#define W25Q_SECTOR_COUNT         (W25Q_FLASH_SIZE / W25Q_SECTOR_SIZE)   /* 512 sectors */
#define W25Q_BLOCK_COUNT_64K      (W25Q_FLASH_SIZE / W25Q_BLOCK_SIZE_64K) /* 32 blocks */

/* SPI Flash Commands (Standard SPI mode) */
#define W25Q_CMD_WRITE_ENABLE     0x06
#define W25Q_CMD_WRITE_DISABLE    0x04
#define W25Q_CMD_READ_STATUS_1    0x05
#define W25Q_CMD_READ_STATUS_2    0x35
#define W25Q_CMD_WRITE_STATUS     0x01
#define W25Q_CMD_READ_DATA        0x03
#define W25Q_CMD_FAST_READ        0x0B
#define W25Q_CMD_PAGE_PROGRAM     0x02
#define W25Q_CMD_SECTOR_ERASE     0x20  /* 4KB */
#define W25Q_CMD_BLOCK_ERASE_32K  0x52  /* 32KB */
#define W25Q_CMD_BLOCK_ERASE_64K  0xD8  /* 64KB */
#define W25Q_CMD_CHIP_ERASE       0xC7
#define W25Q_CMD_POWER_DOWN       0xB9
#define W25Q_CMD_RELEASE_POWER    0xAB
#define W25Q_CMD_READ_JEDEC_ID    0x9F
#define W25Q_CMD_READ_UNIQUE_ID   0x4B
#define W25Q_CMD_ENABLE_RESET     0x66
#define W25Q_CMD_RESET            0x99

/* Status Register Bits */
#define W25Q_STATUS_BUSY          0x01  /* Erase/Write in progress */
#define W25Q_STATUS_WEL           0x02  /* Write Enable Latch */
#define W25Q_STATUS_BP0           0x04  /* Block Protect bit 0 */
#define W25Q_STATUS_BP1           0x08  /* Block Protect bit 1 */
#define W25Q_STATUS_BP2           0x10  /* Block Protect bit 2 */
#define W25Q_STATUS_TB            0x20  /* Top/Bottom Protect */
#define W25Q_STATUS_SEC           0x40  /* Sector Protect */
#define W25Q_STATUS_SRP0          0x80  /* Status Register Protect 0 */

/* JEDEC ID for W25Q16JV */
#define W25Q16JV_JEDEC_ID         0xEF4015  /* Manufacturer EF, Device 4015 */

/* Timing specifications (in ms) */
#define W25Q_TIMEOUT_PAGE_PROG    5     /* Page program max 3ms, use 5ms */
#define W25Q_TIMEOUT_SECTOR_ERASE 500   /* Sector erase max 400ms, use 500ms */
#define W25Q_TIMEOUT_BLOCK_ERASE  2000  /* 64KB block erase max 2000ms */
#define W25Q_TIMEOUT_CHIP_ERASE   100000 /* Chip erase max 100s (!!) */
#define W25Q_TIMEOUT_GENERAL      100   /* General timeout */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief W25Q16JV driver status codes
  */
typedef enum {
    W25Q_OK = 0,              /* Operation successful */
    W25Q_ERROR,               /* General error */
    W25Q_ERROR_INIT,          /* Initialization failed */
    W25Q_ERROR_BUSY,          /* Device busy (timeout) */
    W25Q_ERROR_PARAM,         /* Invalid parameter */
    W25Q_ERROR_VERIFY,        /* Verification failed */
    W25Q_ERROR_SPI,           /* SPI communication error */
    W25Q_ERROR_NOT_FOUND      /* Device not found/wrong ID */
} W25Q_StatusTypeDef;

/**
  * @brief W25Q16JV device handle
  */
typedef struct {
    SPI_HandleTypeDef *hspi;  /* SPI handle pointer */
    GPIO_TypeDef *cs_port;    /* CS GPIO port (NULL if using hardware NSS) */
    uint16_t cs_pin;          /* CS GPIO pin */
    bool initialized;         /* Initialization flag */
    uint32_t jedec_id;        /* Cached JEDEC ID */
} W25Q_HandleTypeDef;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize W25Q16JV flash driver
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  hspi: Pointer to SPI handle
  * @param  cs_port: CS GPIO port (NULL for hardware NSS)
  * @param  cs_pin: CS GPIO pin
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_Init(W25Q_HandleTypeDef *hw25q, SPI_HandleTypeDef *hspi,
                             GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
  * @brief  De-initialize W25Q16JV flash driver
  * @param  hw25q: Pointer to W25Q handle structure
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_DeInit(W25Q_HandleTypeDef *hw25q);

/**
  * @brief  Read JEDEC ID from device
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  jedec_id: Pointer to store 24-bit JEDEC ID
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_ReadJEDECID(W25Q_HandleTypeDef *hw25q, uint32_t *jedec_id);

/**
  * @brief  Read status register 1
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  status: Pointer to store status byte
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_ReadStatus1(W25Q_HandleTypeDef *hw25q, uint8_t *status);

/**
  * @brief  Wait until device is ready (not busy)
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  timeout_ms: Timeout in milliseconds
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_WaitReady(W25Q_HandleTypeDef *hw25q, uint32_t timeout_ms);

/**
  * @brief  Enable write operations (required before program/erase)
  * @param  hw25q: Pointer to W25Q handle structure
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_WriteEnable(W25Q_HandleTypeDef *hw25q);

/**
  * @brief  Disable write operations
  * @param  hw25q: Pointer to W25Q handle structure
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_WriteDisable(W25Q_HandleTypeDef *hw25q);

/**
  * @brief  Read data from flash
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  addr: Read start address (24-bit)
  * @param  data: Pointer to data buffer
  * @param  len: Number of bytes to read
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_Read(W25Q_HandleTypeDef *hw25q, uint32_t addr, 
                             uint8_t *data, uint32_t len);

/**
  * @brief  Fast read data from flash (with dummy byte)
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  addr: Read start address (24-bit)
  * @param  data: Pointer to data buffer
  * @param  len: Number of bytes to read
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_FastRead(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                                 uint8_t *data, uint32_t len);

/**
  * @brief  Program a page (up to 256 bytes)
  * @note   Address must be page-aligned for full page write
  * @note   Data must fit within page boundary (wraps if exceeded)
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  addr: Program start address (24-bit)
  * @param  data: Pointer to data buffer
  * @param  len: Number of bytes to program (max 256)
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_PageProgram(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                                    const uint8_t *data, uint32_t len);

/**
  * @brief  Write data to flash (handles page boundaries automatically)
  * @note   Assumes destination area is already erased (0xFF)
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  addr: Write start address (24-bit)
  * @param  data: Pointer to data buffer
  * @param  len: Number of bytes to write
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_Write(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                              const uint8_t *data, uint32_t len);

/**
  * @brief  Erase a 4KB sector
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  addr: Any address within the sector to erase
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_EraseSector(W25Q_HandleTypeDef *hw25q, uint32_t addr);

/**
  * @brief  Erase a 32KB block
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  addr: Any address within the block to erase
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_EraseBlock32K(W25Q_HandleTypeDef *hw25q, uint32_t addr);

/**
  * @brief  Erase a 64KB block
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  addr: Any address within the block to erase
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_EraseBlock64K(W25Q_HandleTypeDef *hw25q, uint32_t addr);

/**
  * @brief  Erase entire chip (CAUTION: Takes up to 100 seconds!)
  * @param  hw25q: Pointer to W25Q handle structure
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_EraseChip(W25Q_HandleTypeDef *hw25q);

/**
  * @brief  Enter deep power-down mode
  * @param  hw25q: Pointer to W25Q handle structure
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_PowerDown(W25Q_HandleTypeDef *hw25q);

/**
  * @brief  Release from deep power-down mode
  * @param  hw25q: Pointer to W25Q handle structure
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_ReleasePowerDown(W25Q_HandleTypeDef *hw25q);

/**
  * @brief  Software reset the device
  * @param  hw25q: Pointer to W25Q handle structure
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_Reset(W25Q_HandleTypeDef *hw25q);

/**
  * @brief  Check if a region is erased (all 0xFF)
  * @param  hw25q: Pointer to W25Q handle structure
  * @param  addr: Start address to check
  * @param  len: Number of bytes to check
  * @param  is_erased: Pointer to store result (true if erased)
  * @retval W25Q_StatusTypeDef
  */
W25Q_StatusTypeDef W25Q_IsErased(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                                 uint32_t len, bool *is_erased);

/* Utility macros ------------------------------------------------------------*/

/** @brief Get sector number from address */
#define W25Q_ADDR_TO_SECTOR(addr)   ((addr) / W25Q_SECTOR_SIZE)

/** @brief Get sector start address */
#define W25Q_SECTOR_TO_ADDR(sector) ((sector) * W25Q_SECTOR_SIZE)

/** @brief Get page number from address */
#define W25Q_ADDR_TO_PAGE(addr)     ((addr) / W25Q_PAGE_SIZE)

/** @brief Get page start address */
#define W25Q_PAGE_TO_ADDR(page)     ((page) * W25Q_PAGE_SIZE)

/** @brief Check if address is sector-aligned */
#define W25Q_IS_SECTOR_ALIGNED(addr) (((addr) & (W25Q_SECTOR_SIZE - 1)) == 0)

/** @brief Check if address is page-aligned */
#define W25Q_IS_PAGE_ALIGNED(addr)   (((addr) & (W25Q_PAGE_SIZE - 1)) == 0)

#ifdef __cplusplus
}
#endif

#endif /* __W25Q16JV_H */
