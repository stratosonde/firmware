/**
  ******************************************************************************
  * @file    ms5607.c
  * @brief   MS5607-02BA03 pressure sensor driver implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ms5607.h"
#include "sys_app.h" /* For APP_LOG */
#include "SEGGER_RTT.h" /* For RTT debug output */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MS5607_I2C_TIMEOUT        1000  /* I2C timeout in ms */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static MS5607_StatusTypeDef MS5607_RecoverI2C(MS5607_HandleTypeDef *hms5607);
static MS5607_StatusTypeDef MS5607_ReadProm(MS5607_HandleTypeDef *hms5607, uint8_t address, uint16_t *value);
static MS5607_StatusTypeDef MS5607_StartConversion(MS5607_HandleTypeDef *hms5607, uint8_t command);
static MS5607_StatusTypeDef MS5607_ReadADC(MS5607_HandleTypeDef *hms5607, uint32_t *value);
static uint8_t MS5607_CRC4(uint16_t n_prom[]);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize the MS5607 sensor
  * @param  hms5607 Pointer to MS5607 handle
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_Init(MS5607_HandleTypeDef *hms5607)
{
  MS5607_StatusTypeDef status;
  
  /* Check if handle is valid */
  if (hms5607 == NULL || hms5607->hi2c == NULL)
  {
    return MS5607_ERROR;
  }
  
  /* Set default OSR values if not already set */
  /* Note: We need to check if the handle was zero-initialized since MS5607_OSR_256 = 0x00 */
  if (hms5607->PressureOsr != MS5607_OSR_256 && hms5607->PressureOsr != MS5607_OSR_512 && 
      hms5607->PressureOsr != MS5607_OSR_1024 && hms5607->PressureOsr != MS5607_OSR_2048 && 
      hms5607->PressureOsr != MS5607_OSR_4096)
  {
    hms5607->PressureOsr = MS5607_OSR_4096;
  }
  
  if (hms5607->TemperatureOsr != MS5607_OSR_256 && hms5607->TemperatureOsr != MS5607_OSR_512 && 
      hms5607->TemperatureOsr != MS5607_OSR_1024 && hms5607->TemperatureOsr != MS5607_OSR_2048 && 
      hms5607->TemperatureOsr != MS5607_OSR_4096)
  {
    hms5607->TemperatureOsr = MS5607_OSR_4096;
  }
  
  /* Reset the sensor */
  SEGGER_RTT_WriteString(0, "MS5607: Sending reset...\r\n");
  status = MS5607_Reset(hms5607);
  if (status != MS5607_OK)
  {
    SEGGER_RTT_WriteString(0, "MS5607: Reset FAILED\r\n");
    return status;
  }
  SEGGER_RTT_WriteString(0, "MS5607: Reset OK\r\n");
  
  /* Wait for reset to complete - datasheet specifies minimum 2.8ms */
  HAL_Delay(20);
  
  /* Read calibration data */
  SEGGER_RTT_WriteString(0, "MS5607: Reading PROM calibration...\r\n");
  status = MS5607_ReadCalibration(hms5607);
  if (status != MS5607_OK)
  {
    SEGGER_RTT_WriteString(0, "MS5607: PROM read FAILED\r\n");
    return status;
  }
  
  /* Print raw calibration values via RTT */
  SEGGER_RTT_WriteString(0, "MS5607: Calibration data read\r\n");
  
  /* Verify calibration data is valid (not all zeros or all ones) */
  if (hms5607->CalData.c1 == 0 || hms5607->CalData.c1 == 0xFFFF ||
      hms5607->CalData.c2 == 0 || hms5607->CalData.c2 == 0xFFFF ||
      hms5607->CalData.c3 == 0 || hms5607->CalData.c3 == 0xFFFF ||
      hms5607->CalData.c4 == 0 || hms5607->CalData.c4 == 0xFFFF ||
      hms5607->CalData.c5 == 0 || hms5607->CalData.c5 == 0xFFFF ||
      hms5607->CalData.c6 == 0 || hms5607->CalData.c6 == 0xFFFF)
  {
    SEGGER_RTT_WriteString(0, "MS5607: Cal data INVALID (0 or 0xFFFF)\r\n");
    return MS5607_ERROR;
  }
  
  /* Verify CRC */
  uint16_t prom[8];
  prom[0] = hms5607->CalData.reserved;
  prom[1] = hms5607->CalData.c1;
  prom[2] = hms5607->CalData.c2;
  prom[3] = hms5607->CalData.c3;
  prom[4] = hms5607->CalData.c4;
  prom[5] = hms5607->CalData.c5;
  prom[6] = hms5607->CalData.c6;
  prom[7] = hms5607->CalData.crc;
  
  /* Debug: print raw PROM values */
  SEGGER_RTT_WriteString(0, "MS5607: PROM values read\r\n");
  
  uint8_t crc = MS5607_CRC4(prom);
  
  /* Try different CRC extraction methods */
  uint8_t stored_crc_low4 = hms5607->CalData.crc & 0x000F;           /* Lower 4 bits of PROM[7] */
  uint8_t stored_crc_high4 = (hms5607->CalData.crc >> 12) & 0x000F;  /* Upper 4 bits of PROM[7] */
  uint8_t stored_crc_prom0 = (hms5607->CalData.reserved >> 12) & 0x000F; /* Upper 4 bits of PROM[0] */
  
  SEGGER_RTT_WriteString(0, "MS5607: CRC check in progress\r\n");
  
  /* CRC is stored in lower 4 bits of PROM[7] per AN520 CRC Notes */
  uint8_t stored_crc = stored_crc_low4;
  
  if (crc != stored_crc)
  {
    SEGGER_RTT_WriteString(0, "MS5607: CRC MISMATCH!\r\n");
    return MS5607_ERROR;
  }
  else
  {
    SEGGER_RTT_WriteString(0, "MS5607: CRC OK\r\n");
  }
  
  /* Calibration complete */
  SEGGER_RTT_WriteString(0, "MS5607: Calibration complete\r\n");
  
  /* Set initialization flag */
  hms5607->IsInitialized = 1;
  
  SEGGER_RTT_WriteString(0, "MS5607: Initialization successful\r\n");
  return MS5607_OK;
}

/**
  * @brief  Reset the MS5607 sensor
  * @param  hms5607 Pointer to MS5607 handle
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_Reset(MS5607_HandleTypeDef *hms5607)
{
  uint8_t cmd = MS5607_CMD_RESET;
  HAL_StatusTypeDef hal_status;
  
  /* Send reset command */
  hal_status = HAL_I2C_Master_Transmit(hms5607->hi2c, hms5607->Address << 1, &cmd, 1, MS5607_I2C_TIMEOUT);
  
  if (hal_status != HAL_OK)
  {
    return MS5607_ERROR;
  }
  return MS5607_OK;
}

/**
  * @brief  Read calibration data from MS5607 PROM
  * @param  hms5607 Pointer to MS5607 handle
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_ReadCalibration(MS5607_HandleTypeDef *hms5607)
{
  MS5607_StatusTypeDef status;
  
  /* Read reserved word (address 0) */
  status = MS5607_ReadProm(hms5607, 0, &hms5607->CalData.reserved);
  if (status != MS5607_OK)
    return status;
  
  /* Read coefficient 1 (address 1) */
  status = MS5607_ReadProm(hms5607, 1, &hms5607->CalData.c1);
  if (status != MS5607_OK)
    return status;
  
  /* Read coefficient 2 (address 2) */
  status = MS5607_ReadProm(hms5607, 2, &hms5607->CalData.c2);
  if (status != MS5607_OK)
    return status;
  
  /* Read coefficient 3 (address 3) */
  status = MS5607_ReadProm(hms5607, 3, &hms5607->CalData.c3);
  if (status != MS5607_OK)
    return status;
  
  /* Read coefficient 4 (address 4) */
  status = MS5607_ReadProm(hms5607, 4, &hms5607->CalData.c4);
  if (status != MS5607_OK)
    return status;
  
  /* Read coefficient 5 (address 5) */
  status = MS5607_ReadProm(hms5607, 5, &hms5607->CalData.c5);
  if (status != MS5607_OK)
    return status;
  
  /* Read coefficient 6 (address 6) */
  status = MS5607_ReadProm(hms5607, 6, &hms5607->CalData.c6);
  if (status != MS5607_OK)
    return status;
  
  /* Read CRC (address 7) */
  status = MS5607_ReadProm(hms5607, 7, &hms5607->CalData.crc);
  if (status != MS5607_OK)
    return status;
  
  return MS5607_OK;
}

/**
  * @brief  Read temperature and pressure from MS5607
  * @param  hms5607 Pointer to MS5607 handle
  * @param  temperature Pointer to temperature variable (in °C)
  * @param  pressure Pointer to pressure variable (in mbar)
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_ReadPressureAndTemperature(MS5607_HandleTypeDef *hms5607, 
                                                      float *temperature, 
                                                      float *pressure)
{
  MS5607_StatusTypeDef status;
  uint32_t d1, d2;
  int32_t dt;
  int32_t temp;
  int64_t off, sens;
  int64_t p;
  
  /* DIAGNOSTIC MODE: Skip initialization check to force I2C communication attempts */
  /* This allows us to see what's happening on the I2C bus even if init failed */
  if (!hms5607->IsInitialized)
  {
    SEGGER_RTT_WriteString(0, "MS5607: Sensor not initialized\r\n");
  }
  
  /* Start D2 (temperature) conversion */
  status = MS5607_StartConversion(hms5607, MS5607_CMD_CONVERT_D2 | hms5607->TemperatureOsr);
  if (status != MS5607_OK) {
    SEGGER_RTT_WriteString(0, "MS5607: D2 conversion failed\r\n");
    return status;
  }
  
  /* Wait for conversion to complete based on OSR */
  switch (hms5607->TemperatureOsr)
  {
    case MS5607_OSR_256:
      HAL_Delay(1);
      break;
    case MS5607_OSR_512:
      HAL_Delay(2);
      break;
    case MS5607_OSR_1024:
      HAL_Delay(3);
      break;
    case MS5607_OSR_2048:
      HAL_Delay(5);
      break;
    case MS5607_OSR_4096:
      HAL_Delay(10);
      break;
    default:
      HAL_Delay(10);
      break;
  }
  
  /* Read D2 (temperature) ADC value */
  status = MS5607_ReadADC(hms5607, &d2);
  if (status != MS5607_OK) {
    SEGGER_RTT_WriteString(0, "MS5607: D2 ADC read failed\r\n");
    return status;
  }
  
  /* Start D1 (pressure) conversion */
  status = MS5607_StartConversion(hms5607, MS5607_CMD_CONVERT_D1 | hms5607->PressureOsr);
  if (status != MS5607_OK) {
    SEGGER_RTT_WriteString(0, "MS5607: D1 conversion failed\r\n");
    return status;
  }
  
  /* Wait for conversion to complete based on OSR */
  switch (hms5607->PressureOsr)
  {
    case MS5607_OSR_256:
      HAL_Delay(1);
      break;
    case MS5607_OSR_512:
      HAL_Delay(2);
      break;
    case MS5607_OSR_1024:
      HAL_Delay(3);
      break;
    case MS5607_OSR_2048:
      HAL_Delay(5);
      break;
    case MS5607_OSR_4096:
      HAL_Delay(10);
      break;
    default:
      HAL_Delay(10);
      break;
  }
  
  /* Read D1 (pressure) ADC value */
  status = MS5607_ReadADC(hms5607, &d1);
  if (status != MS5607_OK) {
    SEGGER_RTT_WriteString(0, "MS5607: D1 ADC read failed\r\n");
    return status;
  }
  
  /* Check if D1 or D2 is zero or very small, which would indicate a communication problem */
  /* But don't return error, try to calculate with the values we have */
  if (d1 == 0 || d2 == 0 || d1 < 1000 || d2 < 1000) {
    SEGGER_RTT_WriteString(0, "MS5607: Warning - ADC values low\r\n");
    /* Continue anyway and see if we get reasonable values */
  }
  
  /* Calculate temperature per datasheet (page 8, Figure 2) */
  dt = (int32_t)d2 - ((int32_t)hms5607->CalData.c5 << 8);
  temp = 2000 + (((int64_t)dt * hms5607->CalData.c6) >> 23);
  
  /* Calculate temperature compensated pressure per datasheet (page 8, Figure 2) */
  /* OFF = C2 × 2^17 + (C4 × dT) / 2^6 */
  /* SENS = C1 × 2^16 + (C3 × dT) / 2^7 */
  off = ((int64_t)hms5607->CalData.c2 << 17) + (((int64_t)hms5607->CalData.c4 * dt) >> 6);
  sens = ((int64_t)hms5607->CalData.c1 << 16) + (((int64_t)hms5607->CalData.c3 * dt) >> 7);
  
  /* Second order temperature compensation */
  if (temp < 2000)
  {
    int32_t t2 = ((int64_t)dt * dt) >> 31;
    int32_t temp_minus_2000 = temp - 2000;
    int64_t off2 = (61 * (int64_t)temp_minus_2000 * (int64_t)temp_minus_2000) >> 4;
    int64_t sens2 = 2 * (int64_t)temp_minus_2000 * (int64_t)temp_minus_2000;
    
    if (temp < -1500)
    {
      int32_t temp_plus_1500 = temp + 1500;
      int32_t temp_plus_1500_squared = temp_plus_1500 * temp_plus_1500;
      off2 = off2 + (int64_t)15 * temp_plus_1500_squared;
      sens2 = sens2 + (int64_t)8 * temp_plus_1500_squared;
    }
    
    temp -= t2;
    off -= off2;
    sens -= sens2;
  }
  
  /* Calculate pressure - using reference implementation approach */
  p = ((((int64_t)d1 * sens) >> 21) - off) >> 15;
  
  /* Convert to float values */
  *temperature = (float)temp / 100.0f;
  *pressure = (float)p / 100.0f;
  
  return MS5607_OK;
}

/**
  * @brief  Set pressure oversampling ratio
  * @param  hms5607 Pointer to MS5607 handle
  * @param  osr Oversampling ratio
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_SetPressureOSR(MS5607_HandleTypeDef *hms5607, 
                                          MS5607_OsrTypeDef osr)
{
  /* Check if OSR value is valid */
  if (osr != MS5607_OSR_256 && osr != MS5607_OSR_512 && 
      osr != MS5607_OSR_1024 && osr != MS5607_OSR_2048 && 
      osr != MS5607_OSR_4096)
  {
    return MS5607_ERROR;
  }
  
  hms5607->PressureOsr = osr;
  return MS5607_OK;
}

/**
  * @brief  Set temperature oversampling ratio
  * @param  hms5607 Pointer to MS5607 handle
  * @param  osr Oversampling ratio
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_SetTemperatureOSR(MS5607_HandleTypeDef *hms5607, 
                                             MS5607_OsrTypeDef osr)
{
  /* Check if OSR value is valid */
  if (osr != MS5607_OSR_256 && osr != MS5607_OSR_512 && 
      osr != MS5607_OSR_1024 && osr != MS5607_OSR_2048 && 
      osr != MS5607_OSR_4096)
  {
    return MS5607_ERROR;
  }
  
  hms5607->TemperatureOsr = osr;
  return MS5607_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Recover I2C bus from error state
  * @param  hms5607 Pointer to MS5607 handle
  * @retval MS5607 status
  */
static MS5607_StatusTypeDef MS5607_RecoverI2C(MS5607_HandleTypeDef *hms5607)
{
  /* Check I2C error state */
  uint32_t error = HAL_I2C_GetError(hms5607->hi2c);
  if (error != HAL_I2C_ERROR_NONE)
  {
    /* Clear the error flags */
    __HAL_I2C_CLEAR_FLAG(hms5607->hi2c, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR);
    
    /* Reset the I2C peripheral */
    HAL_I2C_DeInit(hms5607->hi2c);
    HAL_Delay(10);
    HAL_I2C_Init(hms5607->hi2c);
    HAL_Delay(10);
  }
  
  return MS5607_OK;
}

/**
  * @brief  Read from MS5607 PROM
  * @param  hms5607 Pointer to MS5607 handle
  * @param  address PROM address (0-7)
  * @param  value Pointer to store the read value
  * @retval MS5607 status
  */
static MS5607_StatusTypeDef MS5607_ReadProm(MS5607_HandleTypeDef *hms5607, uint8_t address, uint16_t *value)
{
  uint8_t cmd;
  uint8_t data[2];
  HAL_StatusTypeDef hal_status;
  
  /* Check if address is valid */
  if (address > 7)
  {
    SEGGER_RTT_WriteString(0, "MS5607_ReadProm: Invalid address\r\n");
    return MS5607_ERROR;
  }
  
  /* Prepare command */
  cmd = MS5607_CMD_PROM_READ | (address << 1);
  
  /* Send command with retry on failure */
  uint8_t retry_count = 0;
  const uint8_t MAX_RETRIES = 3;
  
  while (retry_count < MAX_RETRIES)
  {
    hal_status = HAL_I2C_Master_Transmit(hms5607->hi2c, hms5607->Address << 1, &cmd, 1, MS5607_I2C_TIMEOUT);
    if (hal_status == HAL_OK)
      break;
    
    /* Attempt I2C recovery */
    MS5607_RecoverI2C(hms5607);
    retry_count++;
    HAL_Delay(10);
  }
  
  if (hal_status != HAL_OK)
  {
    return MS5607_ERROR;
  }
  
  /* Add longer delay for PROM read - sensor needs time to prepare data */
  HAL_Delay(10);
  
  /* Read data with retry on failure */
  retry_count = 0;
  while (retry_count < MAX_RETRIES)
  {
    hal_status = HAL_I2C_Master_Receive(hms5607->hi2c, hms5607->Address << 1, data, 2, MS5607_I2C_TIMEOUT);
    if (hal_status == HAL_OK)
      break;
    
    /* Attempt I2C recovery */
    MS5607_RecoverI2C(hms5607);
    retry_count++;
    HAL_Delay(10);
  }
  
  if (hal_status != HAL_OK)
  {
    return MS5607_ERROR;
  }
  
  /* Combine bytes into 16-bit value (big endian) */
  *value = ((uint16_t)data[0] << 8) | data[1];
  
  return MS5607_OK;
}

/**
  * @brief  Start ADC conversion
  * @param  hms5607 Pointer to MS5607 handle
  * @param  command Command to send (D1 or D2 conversion with OSR)
  * @retval MS5607 status
  */
static MS5607_StatusTypeDef MS5607_StartConversion(MS5607_HandleTypeDef *hms5607, uint8_t command)
{
  HAL_StatusTypeDef hal_status;
  
  /* Send command */
  hal_status = HAL_I2C_Master_Transmit(hms5607->hi2c, hms5607->Address << 1, &command, 1, MS5607_I2C_TIMEOUT);
  if (hal_status != HAL_OK)
  {
    return MS5607_ERROR;
  }
  
  return MS5607_OK;
}

/**
  * @brief  Read ADC result
  * @param  hms5607 Pointer to MS5607 handle
  * @param  value Pointer to store the ADC value
  * @retval MS5607 status
  */
static MS5607_StatusTypeDef MS5607_ReadADC(MS5607_HandleTypeDef *hms5607, uint32_t *value)
{
  uint8_t cmd = MS5607_CMD_ADC_READ;
  uint8_t data[3];
  HAL_StatusTypeDef hal_status;
  
  /* Send command */
  hal_status = HAL_I2C_Master_Transmit(hms5607->hi2c, hms5607->Address << 1, &cmd, 1, MS5607_I2C_TIMEOUT);
  if (hal_status != HAL_OK)
  {
    return MS5607_ERROR;
  }
  
  /* Add a small delay between transmit and receive */
  HAL_Delay(5);
  
  /* Read data */
  hal_status = HAL_I2C_Master_Receive(hms5607->hi2c, hms5607->Address << 1, data, 3, MS5607_I2C_TIMEOUT);
  if (hal_status != HAL_OK)
  {
    return MS5607_ERROR;
  }
  
  /* Combine bytes into 24-bit value (big endian) */
  *value = ((uint32_t)data[0]) << 16 | ((uint32_t)data[1]) << 8 | (uint32_t)data[2];
  
  return MS5607_OK;
}

/**
  * @brief  Calculate CRC-4 for PROM values (per AN520 official algorithm)
  * @param  n_prom Array of PROM values (8 words)
  * @retval CRC-4 value
  */
static uint8_t MS5607_CRC4(uint16_t n_prom[])
{
  uint16_t n_rem = 0;
  uint16_t crc_read;
  uint8_t n_bit;
  
  /* AN520: Save original CRC from PROM[7] */
  crc_read = n_prom[7];
  
  /* AN520: CRC byte (lower 8 bits of PROM[7]) is replaced by 0 */
  /* Keep upper 8 bits, clear lower 8 bits */
  n_prom[7] = (0xFF00 & n_prom[7]);
  
  for (uint8_t cnt = 0; cnt < 16; cnt++)
  {
    /* Choose LSB or MSB */
    if (cnt % 2 == 1)
    {
      n_rem ^= (n_prom[cnt >> 1]) & 0x00FF;
    }
    else
    {
      n_rem ^= (n_prom[cnt >> 1] >> 8);
    }
    
    for (n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & 0x8000)
      {
        n_rem = (n_rem << 1) ^ 0x3000;
      }
      else
      {
        n_rem = (n_rem << 1);
      }
    }
  }
  
  /* AN520: Final 4-bit remainder is CRC code */
  n_rem = (0x000F & (n_rem >> 12));
  
  /* Restore the original PROM[7] value */
  n_prom[7] = crc_read;
  
  return (uint8_t)n_rem;
}
