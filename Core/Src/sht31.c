/**
  ******************************************************************************
  * @file    sht31.c
  * @brief   SHT31 temperature and humidity sensor driver implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sht31.h"
#include "SEGGER_RTT.h"

/* Private define ------------------------------------------------------------*/
/* SHT31 Commands */
#define SHT31_CMD_READ_SERIALNBR        0x3780  /* Read Serial Number */
#define SHT31_CMD_READ_STATUS           0xF32D  /* Read Status Register */
#define SHT31_CMD_CLEAR_STATUS          0x3041  /* Clear Status Register */
#define SHT31_CMD_SOFT_RESET            0x30A2  /* Soft Reset */

/* Measurement commands - High repeatability */
#define SHT31_CMD_MEAS_HIGHREP_STRETCH  0x2C06  /* High repeatability with clock stretching */
#define SHT31_CMD_MEAS_HIGHREP          0x2400  /* High repeatability */

/* Measurement commands - Medium repeatability */
#define SHT31_CMD_MEAS_MEDREP_STRETCH   0x2C0D  /* Medium repeatability with clock stretching */
#define SHT31_CMD_MEAS_MEDREP           0x240B  /* Medium repeatability */

/* Measurement commands - Low repeatability */
#define SHT31_CMD_MEAS_LOWREP_STRETCH   0x2C10  /* Low repeatability with clock stretching */
#define SHT31_CMD_MEAS_LOWREP           0x2416  /* Low repeatability */

/* Measurement delay times in milliseconds */
#define SHT31_MEAS_DELAY_HIGH           15      /* High repeatability measurement delay */
#define SHT31_MEAS_DELAY_MEDIUM         6       /* Medium repeatability measurement delay */
#define SHT31_MEAS_DELAY_LOW            4       /* Low repeatability measurement delay */

/* I2C timeout in milliseconds */
#define SHT31_I2C_TIMEOUT               100

/* Private function prototypes -----------------------------------------------*/
static uint8_t SHT31_CalcCRC8(uint8_t *data, uint8_t len);
static SHT31_StatusTypeDef SHT31_SendCommand(SHT31_HandleTypeDef *hsht31, uint16_t cmd);
static uint16_t SHT31_GetMeasurementCommand(SHT31_HandleTypeDef *hsht31);
static uint32_t SHT31_GetMeasurementDelay(SHT31_HandleTypeDef *hsht31);

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initialize the SHT31 sensor
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure that contains
  *                the configuration information for the SHT31 sensor
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_Init(SHT31_HandleTypeDef *hsht31)
{
  SEGGER_RTT_WriteString(0, "  SHT31_Init: Starting...\r\n");
  
  /* Check the SHT31 handle allocation */
  if (hsht31 == NULL)
  {
    SEGGER_RTT_WriteString(0, "  SHT31_Init: FAIL - handle NULL\r\n");
    return SHT31_ERROR;
  }

  /* Check the I2C handle allocation */
  if (hsht31->hi2c == NULL)
  {
    SEGGER_RTT_WriteString(0, "  SHT31_Init: FAIL - I2C handle NULL\r\n");
    return SHT31_ERROR;
  }

  SEGGER_RTT_printf(0, "  SHT31_Init: Using addr 0x%02X, I2C addr 0x%02X\r\n", 
                    hsht31->Address, (hsht31->Address << 1));

  /* Check if I2C bus is ready by doing a simple I2C bus scan for the device */
  HAL_StatusTypeDef i2c_status;
  uint8_t i2c_retry = 5; // Increase retry count
  
  /* Turn on LED to indicate initialization start */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // Using PA0 for LED
  
  SEGGER_RTT_WriteString(0, "  SHT31_Init: Checking device ready...\r\n");
  while (i2c_retry > 0)
  {
    /* Try to communicate with the device */
    i2c_status = HAL_I2C_IsDeviceReady(hsht31->hi2c, (hsht31->Address << 1), 2, 100);
    if (i2c_status == HAL_OK)
    {
      break;
    }
    SEGGER_RTT_printf(0, "  SHT31_Init: DeviceReady retry %d, status=%d\r\n", 5-i2c_retry+1, i2c_status);
    i2c_retry--;
    HAL_Delay(10);
  }
  
  if (i2c_status != HAL_OK)
  {
    /* Device not responding on I2C bus */
    /* Turn off LED to indicate failure */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    SEGGER_RTT_printf(0, "  SHT31_Init: FAIL - device not ready, HAL status=%d\r\n", i2c_status);
    return SHT31_ERROR; // Return error instead of OK
  }
  SEGGER_RTT_WriteString(0, "  SHT31_Init: Device ready OK\r\n");

  /* Perform soft reset to ensure sensor is in a known state */
  SEGGER_RTT_WriteString(0, "  SHT31_Init: Sending soft reset...\r\n");
  if (SHT31_SoftReset(hsht31) != SHT31_OK)
  {
    /* Soft reset failed */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    SEGGER_RTT_WriteString(0, "  SHT31_Init: FAIL - soft reset failed\r\n");
    return SHT31_ERROR;
  }
  SEGGER_RTT_WriteString(0, "  SHT31_Init: Soft reset OK\r\n");
  
  /* Wait for sensor to initialize after reset */
  HAL_Delay(50);
  
  /* Read status register to verify communication */
  uint16_t status;
  SEGGER_RTT_WriteString(0, "  SHT31_Init: Reading status register...\r\n");
  if (SHT31_ReadStatus(hsht31, &status) != SHT31_OK)
  {
    /* Status read failed */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    SEGGER_RTT_WriteString(0, "  SHT31_Init: FAIL - status read failed\r\n");
    return SHT31_ERROR;
  }
  SEGGER_RTT_printf(0, "  SHT31_Init: Status=0x%04X\r\n", status);
  
  /* Clear status register */
  SHT31_ClearStatus(hsht31);
  
  /* Turn off LED to indicate successful initialization */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  
  SEGGER_RTT_WriteString(0, "  SHT31_Init: SUCCESS\r\n");
  return SHT31_OK;
}

/**
  * @brief  Read temperature and humidity from SHT31 sensor with simplified approach
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @param  Temperature pointer to temperature value (in °C * 100)
  * @param  Humidity pointer to humidity value (in % * 100)
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_ReadTempAndHumidity(SHT31_HandleTypeDef *hsht31, 
                                             int32_t *Temperature, 
                                             int32_t *Humidity)
{
  /* Turn on LED to indicate read operation start */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  
  /* Using clock stretching command for more reliable communication */
  uint16_t cmd = SHT31_CMD_MEAS_HIGHREP_STRETCH;
  uint8_t data[6] = {0};
  float temp_float, hum_float;
  
  /* Send measurement command */
  if (SHT31_SendCommand(hsht31, cmd) != SHT31_OK) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    return SHT31_ERROR;
  }
  
  /* Wait for measurement to complete */
  HAL_Delay(1);
  
  /* Read measurement data */
  if (HAL_I2C_Master_Receive(hsht31->hi2c, (hsht31->Address << 1), data, 6, SHT31_I2C_TIMEOUT) != HAL_OK) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    return SHT31_ERROR;
  }
  
  /* Check CRC for temperature */
  if (SHT31_CalcCRC8(&data[0], 2) != data[2]) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    return SHT31_CRC_ERROR;
  }
  
  /* Check CRC for humidity */
  if (SHT31_CalcCRC8(&data[3], 2) != data[5]) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    return SHT31_CRC_ERROR;
  }
  
  /* Convert raw temperature data using the formula from the reference implementation */
  uint16_t rawTemp = ((uint16_t)data[0] << 8) | data[1];
  temp_float = -45.0f + 175.0f * (float)rawTemp / 65535.0f;
  *Temperature = (int32_t)(temp_float * 100.0f);
  
  /* Convert raw humidity data using the formula from the reference implementation */
  uint16_t rawHum = ((uint16_t)data[3] << 8) | data[4];
  hum_float = 100.0f * (float)rawHum / 65535.0f;
  *Humidity = (int32_t)(hum_float * 100.0f);
  
  /* Ensure humidity is within valid range (0-10000) */
  if (*Humidity > 10000) {
    *Humidity = 10000;
  }
  else if (*Humidity < 0) {
    *Humidity = 0;
  }
  
  /* Turn off LED to indicate operation complete */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  
  return SHT31_OK;
}

/**
  * @brief  Read the SHT31 serial number
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @param  SerialNumber pointer to store the serial number
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_ReadSerialNumber(SHT31_HandleTypeDef *hsht31, 
                                          uint32_t *SerialNumber)
{
  uint8_t data[6];
  
  /* Send read serial number command */
  if (SHT31_SendCommand(hsht31, SHT31_CMD_READ_SERIALNBR) != SHT31_OK)
  {
    return SHT31_ERROR;
  }
  
  /* Wait for command processing */
  HAL_Delay(1);
  
  /* Read serial number data */
  if (HAL_I2C_Master_Receive(hsht31->hi2c, (hsht31->Address << 1), data, 6, SHT31_I2C_TIMEOUT) != HAL_OK)
  {
    return SHT31_ERROR;
  }
  
  /* Check CRC for first word */
  if (SHT31_CalcCRC8(&data[0], 2) != data[2])
  {
    return SHT31_CRC_ERROR;
  }
  
  /* Check CRC for second word */
  if (SHT31_CalcCRC8(&data[3], 2) != data[5])
  {
    return SHT31_CRC_ERROR;
  }
  
  /* Combine the two words to form the serial number */
  *SerialNumber = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | 
                  ((uint32_t)data[3] << 8) | data[4];
  
  return SHT31_OK;
}

/**
  * @brief  Soft reset the SHT31 sensor
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_SoftReset(SHT31_HandleTypeDef *hsht31)
{
  return SHT31_SendCommand(hsht31, SHT31_CMD_SOFT_RESET);
}

/**
  * @brief  Read the SHT31 status register
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @param  Status pointer to store the status register value
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_ReadStatus(SHT31_HandleTypeDef *hsht31, 
                                    uint16_t *Status)
{
  uint8_t data[3];
  
  /* Send read status command */
  if (SHT31_SendCommand(hsht31, SHT31_CMD_READ_STATUS) != SHT31_OK)
  {
    return SHT31_ERROR;
  }
  
  /* Wait for command processing */
  HAL_Delay(1);
  
  /* Read status data */
  if (HAL_I2C_Master_Receive(hsht31->hi2c, (hsht31->Address << 1), data, 3, SHT31_I2C_TIMEOUT) != HAL_OK)
  {
    return SHT31_ERROR;
  }
  
  /* Check CRC */
  if (SHT31_CalcCRC8(&data[0], 2) != data[2])
  {
    return SHT31_CRC_ERROR;
  }
  
  /* Return status value */
  *Status = ((uint16_t)data[0] << 8) | data[1];
  
  return SHT31_OK;
}

/**
  * @brief  Clear the SHT31 status register
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_ClearStatus(SHT31_HandleTypeDef *hsht31)
{
  return SHT31_SendCommand(hsht31, SHT31_CMD_CLEAR_STATUS);
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Calculate CRC-8 checksum
  * @param  data pointer to data to calculate CRC for
  * @param  len length of data
  * @retval CRC-8 checksum
  */
static uint8_t SHT31_CalcCRC8(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0xFF;
  uint8_t i, j;
  
  for (i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (j = 0; j < 8; j++)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ 0x31; /* Polynomial: x^8 + x^5 + x^4 + 1 */
      }
      else
      {
        crc = crc << 1;
      }
    }
  }
  
  return crc;
}

/**
  * @brief  Send command to SHT31 sensor
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @param  cmd command to send
  * @retval SHT31 status
  */
static SHT31_StatusTypeDef SHT31_SendCommand(SHT31_HandleTypeDef *hsht31, uint16_t cmd)
{
  uint8_t data[2];
  
  /* Prepare command bytes */
  data[0] = (uint8_t)(cmd >> 8);
  data[1] = (uint8_t)(cmd & 0xFF);
  
  /* Send command */
  if (HAL_I2C_Master_Transmit(hsht31->hi2c, (hsht31->Address << 1), data, 2, SHT31_I2C_TIMEOUT) != HAL_OK)
  {
    return SHT31_ERROR;
  }
  
  return SHT31_OK;
}

/**
  * @brief  Get measurement command based on mode
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @retval Measurement command
  */
static uint16_t SHT31_GetMeasurementCommand(SHT31_HandleTypeDef *hsht31)
{
  switch (hsht31->Mode)
  {
    case SHT31_MODE_LOW_PRECISION:
      return SHT31_CMD_MEAS_LOWREP;
    
    case SHT31_MODE_MEDIUM_PRECISION:
      return SHT31_CMD_MEAS_MEDREP;
    
    case SHT31_MODE_HIGH_PRECISION:
    default:
      return SHT31_CMD_MEAS_HIGHREP;
  }
}

/**
  * @brief  Get measurement delay based on mode
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @retval Measurement delay in milliseconds
  */
static uint32_t SHT31_GetMeasurementDelay(SHT31_HandleTypeDef *hsht31)
{
  switch (hsht31->Mode)
  {
    case SHT31_MODE_LOW_PRECISION:
      return SHT31_MEAS_DELAY_LOW;
    
    case SHT31_MODE_MEDIUM_PRECISION:
      return SHT31_MEAS_DELAY_MEDIUM;
    
    case SHT31_MODE_HIGH_PRECISION:
    default:
      return SHT31_MEAS_DELAY_HIGH;
  }
}
