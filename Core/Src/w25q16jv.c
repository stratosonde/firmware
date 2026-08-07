/**
  ******************************************************************************
  * @file    w25q16jv.c
  * @brief   W25Q16JV SPI Flash Driver Implementation
  * @details Low-level driver for Winbond W25Q16JV 16Mbit (2MB) SPI NOR Flash
  *
  * Implementation Notes:
  *   - Uses blocking SPI transfers for simplicity and reliability
  *   - Software CS is used (GPIO PB9) - hardware NSS pulse mode corrupts
  *     multi-byte command sequences on the W25Q16JV
  *   - All operations check BUSY status before proceeding
  *   - Timeouts prevent infinite loops on hardware failures
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "w25q16jv.h"
#include <string.h>
#include "SEGGER_RTT.h"
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */

/* Private defines -----------------------------------------------------------*/
#define W25Q_SPI_TIMEOUT     100   /* SPI HAL timeout in ms */

/* Private function prototypes -----------------------------------------------*/
static void W25Q_CS_Low(W25Q_HandleTypeDef *hw25q);
static void W25Q_CS_High(W25Q_HandleTypeDef *hw25q);
static W25Q_StatusTypeDef W25Q_SPI_Transmit(W25Q_HandleTypeDef *hw25q, uint8_t *data, uint16_t len);
static W25Q_StatusTypeDef W25Q_SPI_Receive(W25Q_HandleTypeDef *hw25q, uint8_t *data, uint16_t len);
static W25Q_StatusTypeDef W25Q_SPI_TransmitReceive(W25Q_HandleTypeDef *hw25q, uint8_t *tx, uint8_t *rx, uint16_t len);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Assert chip select (drive low)
  */
static void W25Q_CS_Low(W25Q_HandleTypeDef *hw25q)
{
    if (hw25q->cs_port != NULL) {
        HAL_GPIO_WritePin(hw25q->cs_port, hw25q->cs_pin, GPIO_PIN_RESET);
    }
    /* If cs_port is NULL, hardware NSS is used - no action needed */
}

/**
  * @brief  Deassert chip select (drive high)
  */
static void W25Q_CS_High(W25Q_HandleTypeDef *hw25q)
{
    if (hw25q->cs_port != NULL) {
        HAL_GPIO_WritePin(hw25q->cs_port, hw25q->cs_pin, GPIO_PIN_SET);
    }
    /* If cs_port is NULL, hardware NSS is used - no action needed */
}

/**
  * @brief  SPI transmit wrapper
  */
static W25Q_StatusTypeDef W25Q_SPI_Transmit(W25Q_HandleTypeDef *hw25q, uint8_t *data, uint16_t len)
{
    if (HAL_SPI_Transmit(hw25q->hspi, data, len, W25Q_SPI_TIMEOUT) != HAL_OK) {
        return W25Q_ERROR_SPI;
    }
    return W25Q_OK;
}

/**
  * @brief  SPI receive wrapper
  */
static W25Q_StatusTypeDef W25Q_SPI_Receive(W25Q_HandleTypeDef *hw25q, uint8_t *data, uint16_t len)
{
    if (HAL_SPI_Receive(hw25q->hspi, data, len, W25Q_SPI_TIMEOUT) != HAL_OK) {
        return W25Q_ERROR_SPI;
    }
    return W25Q_OK;
}

/**
  * @brief  SPI transmit and receive wrapper
  */
static W25Q_StatusTypeDef __attribute__((unused)) W25Q_SPI_TransmitReceive(W25Q_HandleTypeDef *hw25q, uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (HAL_SPI_TransmitReceive(hw25q->hspi, tx, rx, len, W25Q_SPI_TIMEOUT) != HAL_OK) {
        return W25Q_ERROR_SPI;
    }
    return W25Q_OK;
}

/* Exported functions --------------------------------------------------------*/

W25Q_StatusTypeDef W25Q_Init(W25Q_HandleTypeDef *hw25q, SPI_HandleTypeDef *hspi,
                             GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    W25Q_StatusTypeDef status;
    uint32_t jedec_id;
    
    if (hw25q == NULL || hspi == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Store configuration */
    hw25q->hspi = hspi;
    hw25q->cs_port = cs_port;
    hw25q->cs_pin = cs_pin;
    hw25q->initialized = false;
    hw25q->jedec_id = 0;
    
    /* Ensure CS is high initially */
    W25Q_CS_High(hw25q);
    
    /* Power-up delay: tVSL max 10ms per datasheet (using 20ms for safety) */
    HAL_Delay(20);
    
    /* Release from power-down in case device was in sleep mode */
    status = W25Q_ReleasePowerDown(hw25q);
    if (status != W25Q_OK) {
        SONDE_LOG("W25Q_Init: ReleasePowerDown FAILED (status=%d)\r\n", status);
        return status;
    }
    
    /* Delay after release from power-down: tRES1 = 3us typ, using 5ms for safety */
    HAL_Delay(5);
    
    /* Software reset to ensure clean state */
    status = W25Q_Reset(hw25q);
    if (status != W25Q_OK) {
        SONDE_LOG("W25Q_Init: Reset FAILED (status=%d)\r\n", status);
        return status;
    }
    HAL_Delay(1);  /* tRST = 30us max, using 1ms */
    
    /* Read and verify JEDEC ID */
    status = W25Q_ReadJEDECID(hw25q, &jedec_id);
    if (status != W25Q_OK) {
        SONDE_LOG("W25Q_Init: ReadJEDEC FAILED (status=%d)\r\n", status);
        return status;
    }
    
    /* Verify this is a W25Q16JV or compatible (check manufacturer and memory type) */
    /* W25Q16JV: EF 40 15, W25Q16DV: EF 40 15, W25Q16BV: EF 40 15 */
    /* We accept any W25Q16 variant (manufacturer 0xEF, memory type 0x40, capacity 0x15) */
    if ((jedec_id & 0xFFFF00) != 0xEF4000) {
        /* Not a Winbond W25Q series device */
        SONDE_LOG("W25Q_Init: VERIFICATION FAILED - Wrong device ID: 0x%06lX\r\n", jedec_id);
        return W25Q_ERROR_NOT_FOUND;
    }
    
    hw25q->jedec_id = jedec_id;

    /* R29 (#36): block-protect / write-path verification. With BP|SEC|TB set,
     * PageProgram is silently ignored by the device: WriteEnable succeeds,
     * BUSY never asserts, WaitReady returns OK, and without a read-back the
     * archive would report success while storing nothing. */
    {
        uint8_t sr1 = 0, sr2 = 0;
        if (W25Q_ReadStatus1(hw25q, &sr1) != W25Q_OK) {
            SONDE_LOG_STR("W25Q_Init: SR1 read FAILED\r\n");
            return W25Q_ERROR_SPI;
        }
        {
            uint8_t cmd2 = W25Q_CMD_READ_STATUS_2;
            W25Q_CS_Low(hw25q);
            if (W25Q_SPI_Transmit(hw25q, &cmd2, 1) == W25Q_OK) {
                W25Q_SPI_Receive(hw25q, &sr2, 1);
            }
            W25Q_CS_High(hw25q);
        }
        SONDE_LOG("W25Q_Init: SR1=0x%02X SR2=0x%02X\r\n", sr1, sr2);

        if ((sr1 & W25Q_SR1_PROTECT_MASK) != 0) {
            /* Attempt to clear BP/SEC/TB (bits 2-6); SRP0 (bit 7) and the
             * read-only BUSY/WEL bits are preserved/masked out of the write. */
            uint8_t clear_val = (uint8_t)(sr1 & ~W25Q_SR1_PROTECT_MASK);
            SONDE_LOG("W25Q_Init: block-protect bits set (SR1=0x%02X) - clearing to 0x%02X\r\n",
                              sr1, clear_val);
            if (W25Q_WriteEnable(hw25q) != W25Q_OK) {
                return W25Q_ERROR_PROTECTED;
            }
            uint8_t wr[2] = { W25Q_CMD_WRITE_STATUS, clear_val };
            W25Q_CS_Low(hw25q);
            W25Q_StatusTypeDef wst = W25Q_SPI_Transmit(hw25q, wr, 2);
            W25Q_CS_High(hw25q);
            if (wst != W25Q_OK || W25Q_WaitReady(hw25q, 100) != W25Q_OK) {
                return W25Q_ERROR_PROTECTED;
            }
            if (W25Q_ReadStatus1(hw25q, &sr1) != W25Q_OK ||
                (sr1 & W25Q_SR1_PROTECT_MASK) != 0) {
                SONDE_LOG("W25Q_Init: PROTECTED - BP bits stuck (SR1=0x%02X, maybe SRP0/SRL locked)\r\n", sr1);
                return W25Q_ERROR_PROTECTED;   /* distinct error: archive cannot store */
            }
            SONDE_LOG_STR("W25Q_Init: block-protect cleared OK\r\n");
        }

        /* Write-path self-test WITHOUT touching the data array (a sector
         * self-test would erase 64 archive records per boot): toggle WEL and
         * read it back. WEL only sets if WriteEnable genuinely reached the
         * device — the same command path PageProgram requires. */
        if (W25Q_WriteEnable(hw25q) != W25Q_OK) {
            return W25Q_ERROR_SPI;
        }
        uint8_t wel = 0;
        W25Q_ReadStatus1(hw25q, &wel);
        if ((wel & 0x02u) == 0) {
            SONDE_LOG_STR("W25Q_Init: SELF-TEST FAILED - WEL did not set\r\n");
            return W25Q_ERROR_VERIFY;
        }
        W25Q_WriteDisable(hw25q);
        W25Q_ReadStatus1(hw25q, &wel);
        if ((wel & 0x02u) != 0) {
            SONDE_LOG_STR("W25Q_Init: SELF-TEST FAILED - WEL did not clear\r\n");
            return W25Q_ERROR_VERIFY;
        }
        SONDE_LOG_STR("W25Q_Init: write-path self-test OK (WEL toggle verified)\r\n");
    }

    hw25q->initialized = true;

    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_DeInit(W25Q_HandleTypeDef *hw25q)
{
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Put device in power-down mode for minimum current */
    W25Q_PowerDown(hw25q);
    
    hw25q->initialized = false;
    hw25q->hspi = NULL;
    
    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_ReadJEDECID(W25Q_HandleTypeDef *hw25q, uint32_t *jedec_id)
{
    W25Q_StatusTypeDef status;
    uint8_t cmd = W25Q_CMD_READ_JEDEC_ID;
    uint8_t buf[3] = {0};
    
    if (hw25q == NULL || jedec_id == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    W25Q_CS_Low(hw25q);
    
    status = W25Q_SPI_Transmit(hw25q, &cmd, 1);
    if (status == W25Q_OK) {
        status = W25Q_SPI_Receive(hw25q, buf, 3);
    }
    
    W25Q_CS_High(hw25q);
    
    if (status == W25Q_OK) {
        /* JEDEC ID: Manufacturer (buf[0]), Memory Type (buf[1]), Capacity (buf[2]) */
        *jedec_id = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    }
    
    return status;
}

W25Q_StatusTypeDef W25Q_ReadStatus1(W25Q_HandleTypeDef *hw25q, uint8_t *status)
{
    W25Q_StatusTypeDef ret;
    uint8_t cmd = W25Q_CMD_READ_STATUS_1;
    
    if (hw25q == NULL || status == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    W25Q_CS_Low(hw25q);
    
    ret = W25Q_SPI_Transmit(hw25q, &cmd, 1);
    if (ret == W25Q_OK) {
        ret = W25Q_SPI_Receive(hw25q, status, 1);
    }
    
    W25Q_CS_High(hw25q);
    
    return ret;
}

W25Q_StatusTypeDef W25Q_WaitReady(W25Q_HandleTypeDef *hw25q, uint32_t timeout_ms)
{
    uint8_t status;
    W25Q_StatusTypeDef ret;
    uint32_t start_tick = HAL_GetTick();
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    do {
        ret = W25Q_ReadStatus1(hw25q, &status);
        if (ret != W25Q_OK) {
            return ret;
        }
        
        if ((status & W25Q_STATUS_BUSY) == 0) {
            return W25Q_OK;  /* Device ready */
        }
        
        /* R39 (#31): __WFI() between polls instead of a 1 ms NOP-spin — the
         * core sleeps until ANY interrupt (SysTick at 1 kHz bounds the stall
         * to ~1 ms, same cadence as before at a fraction of the energy). */
        __WFI();

        /* F-11 (#69): pet the watchdog inside the wait loop. Safe today only
         * because no path issues CHIP_ERASE (~100 s); with F-10's widened
         * cold-margin timeouts a long erase would otherwise IWDG-reset. */
        {
            extern IWDG_HandleTypeDef hiwdg;
            if (hiwdg.Instance != NULL) {
                HAL_IWDG_Refresh(&hiwdg);
            }
        }

    } while ((HAL_GetTick() - start_tick) < timeout_ms);
    
    return W25Q_ERROR_BUSY;  /* Timeout */
}

W25Q_StatusTypeDef W25Q_WriteEnable(W25Q_HandleTypeDef *hw25q)
{
    W25Q_StatusTypeDef status;
    uint8_t cmd = W25Q_CMD_WRITE_ENABLE;
    uint8_t sr;
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, &cmd, 1);
    W25Q_CS_High(hw25q);
    
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Verify WEL bit is set */
    status = W25Q_ReadStatus1(hw25q, &sr);
    if (status != W25Q_OK) {
        return status;
    }
    
    if ((sr & W25Q_STATUS_WEL) == 0) {
        return W25Q_ERROR;  /* WEL not set */
    }
    
    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_WriteDisable(W25Q_HandleTypeDef *hw25q)
{
    W25Q_StatusTypeDef status;
    uint8_t cmd = W25Q_CMD_WRITE_DISABLE;
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, &cmd, 1);
    W25Q_CS_High(hw25q);
    
    return status;
}

W25Q_StatusTypeDef W25Q_Read(W25Q_HandleTypeDef *hw25q, uint32_t addr, 
                             uint8_t *data, uint32_t len)
{
    /* R29 (#36): public entry points require a verified-initialized device
     * (block-protect cleared, write path self-tested in W25Q_Init). */
    if (hw25q == NULL || !hw25q->initialized) {
        return W25Q_ERROR_INIT;
    }
    W25Q_StatusTypeDef status;
    uint8_t cmd[4];
    
    if (hw25q == NULL || data == NULL || len == 0) {
        return W25Q_ERROR_PARAM;
    }
    
    if ((addr + len) > W25Q_FLASH_SIZE) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Wait for any previous operation to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_GENERAL);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Build command: CMD + 24-bit address */
    cmd[0] = W25Q_CMD_READ_DATA;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    
    W25Q_CS_Low(hw25q);
    
    status = W25Q_SPI_Transmit(hw25q, cmd, 4);
    if (status == W25Q_OK) {
        /* Read data - may need to split for very large reads */
        while (len > 0) {
            uint16_t chunk = (len > 65535) ? 65535 : (uint16_t)len;
            status = W25Q_SPI_Receive(hw25q, data, chunk);
            if (status != W25Q_OK) {
                break;
            }
            data += chunk;
            len -= chunk;
        }
    }
    
    W25Q_CS_High(hw25q);
    
    return status;
}

W25Q_StatusTypeDef W25Q_FastRead(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                                 uint8_t *data, uint32_t len)
{
    /* R29 (#36): public entry points require a verified-initialized device
     * (block-protect cleared, write path self-tested in W25Q_Init). */
    if (hw25q == NULL || !hw25q->initialized) {
        return W25Q_ERROR_INIT;
    }
    W25Q_StatusTypeDef status;
    uint8_t cmd[5];
    
    if (hw25q == NULL || data == NULL || len == 0) {
        return W25Q_ERROR_PARAM;
    }
    
    if ((addr + len) > W25Q_FLASH_SIZE) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Wait for any previous operation to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_GENERAL);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Build command: CMD + 24-bit address + dummy byte */
    cmd[0] = W25Q_CMD_FAST_READ;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    cmd[4] = 0x00;  /* Dummy byte */
    
    W25Q_CS_Low(hw25q);
    
    status = W25Q_SPI_Transmit(hw25q, cmd, 5);
    if (status == W25Q_OK) {
        while (len > 0) {
            uint16_t chunk = (len > 65535) ? 65535 : (uint16_t)len;
            status = W25Q_SPI_Receive(hw25q, data, chunk);
            if (status != W25Q_OK) {
                break;
            }
            data += chunk;
            len -= chunk;
        }
    }
    
    W25Q_CS_High(hw25q);
    
    return status;
}

W25Q_StatusTypeDef W25Q_PageProgram(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                                    const uint8_t *data, uint32_t len)
{
    /* R29 (#36): public entry points require a verified-initialized device
     * (block-protect cleared, write path self-tested in W25Q_Init). */
    if (hw25q == NULL || !hw25q->initialized) {
        return W25Q_ERROR_INIT;
    }
    W25Q_StatusTypeDef status;
    uint8_t cmd[4];
    
    if (hw25q == NULL || data == NULL || len == 0 || len > W25Q_PAGE_SIZE) {
        return W25Q_ERROR_PARAM;
    }
    
    if ((addr + len) > W25Q_FLASH_SIZE) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Check that we don't cross page boundary */
    uint32_t page_offset = addr & (W25Q_PAGE_SIZE - 1);
    if ((page_offset + len) > W25Q_PAGE_SIZE) {
        return W25Q_ERROR_PARAM;  /* Would wrap within page */
    }
    
    /* Wait for any previous operation to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_GENERAL);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Enable write */
    status = W25Q_WriteEnable(hw25q);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Build command: CMD + 24-bit address */
    cmd[0] = W25Q_CMD_PAGE_PROGRAM;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    
    W25Q_CS_Low(hw25q);
    
    status = W25Q_SPI_Transmit(hw25q, cmd, 4);
    if (status == W25Q_OK) {
        status = W25Q_SPI_Transmit(hw25q, (uint8_t *)data, (uint16_t)len);
    }
    
    W25Q_CS_High(hw25q);
    
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Wait for program to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_PAGE_PROG);
    
    return status;
}

W25Q_StatusTypeDef W25Q_Write(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                              const uint8_t *data, uint32_t len)
{
    /* R29 (#36): public entry points require a verified-initialized device
     * (block-protect cleared, write path self-tested in W25Q_Init). */
    if (hw25q == NULL || !hw25q->initialized) {
        return W25Q_ERROR_INIT;
    }
    W25Q_StatusTypeDef status;
    uint32_t bytes_to_write;
    uint32_t page_offset;
    
    if (hw25q == NULL || data == NULL || len == 0) {
        return W25Q_ERROR_PARAM;
    }
    
    if ((addr + len) > W25Q_FLASH_SIZE) {
        return W25Q_ERROR_PARAM;
    }
    
    while (len > 0) {
        /* Calculate how many bytes we can write in the current page */
        page_offset = addr & (W25Q_PAGE_SIZE - 1);
        bytes_to_write = W25Q_PAGE_SIZE - page_offset;
        
        if (bytes_to_write > len) {
            bytes_to_write = len;
        }
        
        /* Write this chunk */
        status = W25Q_PageProgram(hw25q, addr, data, bytes_to_write);
        if (status != W25Q_OK) {
            return status;
        }
        
        addr += bytes_to_write;
        data += bytes_to_write;
        len -= bytes_to_write;
    }
    
    return W25Q_OK;
}

W25Q_StatusTypeDef W25Q_EraseSector(W25Q_HandleTypeDef *hw25q, uint32_t addr)
{
    /* R29 (#36): public entry points require a verified-initialized device
     * (block-protect cleared, write path self-tested in W25Q_Init). */
    if (hw25q == NULL || !hw25q->initialized) {
        return W25Q_ERROR_INIT;
    }
    W25Q_StatusTypeDef status;
    uint8_t cmd[4];
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    if (addr >= W25Q_FLASH_SIZE) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Wait for any previous operation to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_GENERAL);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Enable write */
    status = W25Q_WriteEnable(hw25q);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Build command: CMD + 24-bit address (any address in sector) */
    cmd[0] = W25Q_CMD_SECTOR_ERASE;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, cmd, 4);
    W25Q_CS_High(hw25q);
    
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Wait for erase to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_SECTOR_ERASE);
    
    return status;
}

W25Q_StatusTypeDef W25Q_EraseBlock32K(W25Q_HandleTypeDef *hw25q, uint32_t addr)
{
    /* R29 (#36): public entry points require a verified-initialized device
     * (block-protect cleared, write path self-tested in W25Q_Init). */
    if (hw25q == NULL || !hw25q->initialized) {
        return W25Q_ERROR_INIT;
    }
    W25Q_StatusTypeDef status;
    uint8_t cmd[4];
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    if (addr >= W25Q_FLASH_SIZE) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Wait for any previous operation to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_GENERAL);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Enable write */
    status = W25Q_WriteEnable(hw25q);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Build command */
    cmd[0] = W25Q_CMD_BLOCK_ERASE_32K;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, cmd, 4);
    W25Q_CS_High(hw25q);
    
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Wait for erase to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_BLOCK_ERASE);
    
    return status;
}

W25Q_StatusTypeDef W25Q_EraseBlock64K(W25Q_HandleTypeDef *hw25q, uint32_t addr)
{
    /* R29 (#36): public entry points require a verified-initialized device
     * (block-protect cleared, write path self-tested in W25Q_Init). */
    if (hw25q == NULL || !hw25q->initialized) {
        return W25Q_ERROR_INIT;
    }
    W25Q_StatusTypeDef status;
    uint8_t cmd[4];
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    if (addr >= W25Q_FLASH_SIZE) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Wait for any previous operation to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_GENERAL);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Enable write */
    status = W25Q_WriteEnable(hw25q);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Build command */
    cmd[0] = W25Q_CMD_BLOCK_ERASE_64K;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, cmd, 4);
    W25Q_CS_High(hw25q);
    
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Wait for erase to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_BLOCK_ERASE);
    
    return status;
}

W25Q_StatusTypeDef W25Q_EraseChip(W25Q_HandleTypeDef *hw25q)
{
    /* R29 (#36): public entry points require a verified-initialized device
     * (block-protect cleared, write path self-tested in W25Q_Init). */
    if (hw25q == NULL || !hw25q->initialized) {
        return W25Q_ERROR_INIT;
    }
    W25Q_StatusTypeDef status;
    uint8_t cmd = W25Q_CMD_CHIP_ERASE;
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Wait for any previous operation to complete */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_GENERAL);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Enable write */
    status = W25Q_WriteEnable(hw25q);
    if (status != W25Q_OK) {
        return status;
    }
    
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, &cmd, 1);
    W25Q_CS_High(hw25q);
    
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Wait for erase to complete - this takes a LONG time! */
    status = W25Q_WaitReady(hw25q, W25Q_TIMEOUT_CHIP_ERASE);
    
    return status;
}

W25Q_StatusTypeDef W25Q_PowerDown(W25Q_HandleTypeDef *hw25q)
{
    W25Q_StatusTypeDef status;
    uint8_t cmd = W25Q_CMD_POWER_DOWN;
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, &cmd, 1);
    W25Q_CS_High(hw25q);
    
    /* tDP = 3us max, we'll add a small delay */
    HAL_Delay(1);
    
    return status;
}

W25Q_StatusTypeDef W25Q_ReleasePowerDown(W25Q_HandleTypeDef *hw25q)
{
    W25Q_StatusTypeDef status;
    uint8_t cmd = W25Q_CMD_RELEASE_POWER;
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, &cmd, 1);
    W25Q_CS_High(hw25q);
    
    /* tRES1 = 3us max, we'll add a small delay */
    HAL_Delay(1);
    
    return status;
}

W25Q_StatusTypeDef W25Q_Reset(W25Q_HandleTypeDef *hw25q)
{
    W25Q_StatusTypeDef status;
    uint8_t cmd;
    
    if (hw25q == NULL) {
        return W25Q_ERROR_PARAM;
    }
    
    /* Two-step reset: Enable Reset (66h) then Reset (99h) */
    cmd = W25Q_CMD_ENABLE_RESET;
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, &cmd, 1);
    W25Q_CS_High(hw25q);
    
    if (status != W25Q_OK) {
        return status;
    }
    
    cmd = W25Q_CMD_RESET;
    W25Q_CS_Low(hw25q);
    status = W25Q_SPI_Transmit(hw25q, &cmd, 1);
    W25Q_CS_High(hw25q);
    
    /* tRST = 30us max */
    HAL_Delay(1);
    
    return status;
}

W25Q_StatusTypeDef W25Q_IsErased(W25Q_HandleTypeDef *hw25q, uint32_t addr,
                                 uint32_t len, bool *is_erased)
{
    /* R29 (#36): public entry points require a verified-initialized device
     * (block-protect cleared, write path self-tested in W25Q_Init). */
    if (hw25q == NULL || !hw25q->initialized) {
        return W25Q_ERROR_INIT;
    }
    W25Q_StatusTypeDef status;
    uint8_t buf[64];  /* Read in chunks */
    uint32_t chunk;
    uint32_t i;
    
    if (hw25q == NULL || is_erased == NULL || len == 0) {
        return W25Q_ERROR_PARAM;
    }
    
    if ((addr + len) > W25Q_FLASH_SIZE) {
        return W25Q_ERROR_PARAM;
    }
    
    *is_erased = true;
    
    while (len > 0) {
        chunk = (len > sizeof(buf)) ? sizeof(buf) : len;
        
        status = W25Q_Read(hw25q, addr, buf, chunk);
        if (status != W25Q_OK) {
            return status;
        }
        
        for (i = 0; i < chunk; i++) {
            if (buf[i] != 0xFF) {
                *is_erased = false;
                return W25Q_OK;
            }
        }
        
        addr += chunk;
        len -= chunk;
    }
    
    return W25Q_OK;
}
