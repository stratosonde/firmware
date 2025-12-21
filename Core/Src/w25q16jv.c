/**
  ******************************************************************************
  * @file    w25q16jv.c
  * @brief   W25Q16JV SPI Flash Driver Implementation
  * @details Low-level driver for Winbond W25Q16JV 16Mbit (2MB) SPI NOR Flash
  *
  * Implementation Notes:
  *   - Uses blocking SPI transfers for simplicity and reliability
  *   - Hardware NSS is used (SPI2_NSS on PB9) - no software CS needed
  *   - All operations check BUSY status before proceeding
  *   - Timeouts prevent infinite loops on hardware failures
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "w25q16jv.h"
#include <string.h>

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
static W25Q_StatusTypeDef W25Q_SPI_TransmitReceive(W25Q_HandleTypeDef *hw25q, uint8_t *tx, uint8_t *rx, uint16_t len)
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
    
    /* Small delay for device to be ready after power-up */
    HAL_Delay(1);
    
    /* Release from power-down in case device was in sleep mode */
    status = W25Q_ReleasePowerDown(hw25q);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Small delay after release from power-down (tRES1 = 3us) */
    HAL_Delay(1);
    
    /* Read and verify JEDEC ID */
    status = W25Q_ReadJEDECID(hw25q, &jedec_id);
    if (status != W25Q_OK) {
        return status;
    }
    
    /* Verify this is a W25Q16JV or compatible (check manufacturer and memory type) */
    /* W25Q16JV: EF 40 15, W25Q16DV: EF 40 15, W25Q16BV: EF 40 15 */
    /* We accept any W25Q16 variant (manufacturer 0xEF, memory type 0x40, capacity 0x15) */
    if ((jedec_id & 0xFFFF00) != 0xEF4000) {
        /* Not a Winbond W25Q series device */
        return W25Q_ERROR_NOT_FOUND;
    }
    
    hw25q->jedec_id = jedec_id;
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
        
        /* Small delay to avoid hammering the SPI bus */
        HAL_Delay(1);
        
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
