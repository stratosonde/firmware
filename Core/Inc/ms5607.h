/**
  ******************************************************************************
  * @file    ms5607.h
  * @brief   Header file for MS5607-02BA03 pressure sensor driver
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MS5607_H__
#define __MS5607_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  MS5607 Status enum
  */
typedef enum {
  MS5607_OK       = 0x00,
  MS5607_ERROR    = 0x01,
  MS5607_BUSY     = 0x02,
  MS5607_TIMEOUT  = 0x03
} MS5607_StatusTypeDef;

/**
  * @brief  MS5607 Oversampling Ratio enum
  */
typedef enum {
  MS5607_OSR_256  = 0x00, /* Conversion time 0.54ms, RMS noise 0.13mbar */
  MS5607_OSR_512  = 0x02, /* Conversion time 1.06ms, RMS noise 0.084mbar */
  MS5607_OSR_1024 = 0x04, /* Conversion time 2.08ms, RMS noise 0.054mbar */
  MS5607_OSR_2048 = 0x06, /* Conversion time 4.13ms, RMS noise 0.036mbar */
  MS5607_OSR_4096 = 0x08  /* Conversion time 8.22ms, RMS noise 0.024mbar */
} MS5607_OsrTypeDef;

/**
  * @brief  MS5607 PROM calibration data structure
  */
typedef struct {
  uint16_t reserved;   /* Factory data and setup */
  uint16_t c1;         /* Pressure sensitivity | SENS_T1 */
  uint16_t c2;         /* Pressure offset | OFF_T1 */
  uint16_t c3;         /* Temperature coefficient of pressure sensitivity | TCS */
  uint16_t c4;         /* Temperature coefficient of pressure offset | TCO */
  uint16_t c5;         /* Reference temperature | T_REF */
  uint16_t c6;         /* Temperature coefficient of the temperature | TEMPSENS */
  uint16_t crc;        /* CRC-4 checksum */
} MS5607_CalTypeDef;

/**
  * @brief  MS5607 handle structure
  */
typedef struct {
  I2C_HandleTypeDef *hi2c;                /* I2C handle */
  uint8_t            Address;             /* I2C device address */
  MS5607_OsrTypeDef  PressureOsr;         /* Pressure oversampling ratio */
  MS5607_OsrTypeDef  TemperatureOsr;      /* Temperature oversampling ratio */
  MS5607_CalTypeDef  CalData;             /* Calibration data */
  uint8_t            IsInitialized;       /* Initialization status */
} MS5607_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/
/**
  * @brief  MS5607 I2C device addresses
  */
#define MS5607_I2C_ADDRESS_A       0x76  /* When CSB pin is connected to VDD (high) */
#define MS5607_I2C_ADDRESS_B       0x77  /* When CSB pin is connected to GND (low) */

/**
  * @brief  MS5607 Commands
  */
#define MS5607_CMD_RESET           0x1E  /* Reset command */
#define MS5607_CMD_CONVERT_D1      0x40  /* Convert pressure command */
#define MS5607_CMD_CONVERT_D2      0x50  /* Convert temperature command */
#define MS5607_CMD_ADC_READ        0x00  /* Read ADC command */
#define MS5607_CMD_PROM_READ       0xA0  /* Read PROM command (0xA0 to 0xAE) */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Initialize the MS5607 sensor
  * @param  hms5607 Pointer to MS5607 handle
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_Init(MS5607_HandleTypeDef *hms5607);

/**
  * @brief  Reset the MS5607 sensor
  * @param  hms5607 Pointer to MS5607 handle
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_Reset(MS5607_HandleTypeDef *hms5607);

/**
  * @brief  Read calibration data from MS5607 PROM
  * @param  hms5607 Pointer to MS5607 handle
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_ReadCalibration(MS5607_HandleTypeDef *hms5607);

/**
  * @brief  Read temperature and pressure from MS5607
  * @param  hms5607 Pointer to MS5607 handle
  * @param  temperature Pointer to temperature variable (in °C)
  * @param  pressure Pointer to pressure variable (in mbar)
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_ReadPressureAndTemperature(MS5607_HandleTypeDef *hms5607, 
                                                      float *temperature, 
                                                      float *pressure);

/**
  * @brief  Set pressure oversampling ratio
  * @param  hms5607 Pointer to MS5607 handle
  * @param  osr Oversampling ratio
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_SetPressureOSR(MS5607_HandleTypeDef *hms5607, 
                                          MS5607_OsrTypeDef osr);

/**
  * @brief  Set temperature oversampling ratio
  * @param  hms5607 Pointer to MS5607 handle
  * @param  osr Oversampling ratio
  * @retval MS5607 status
  */
MS5607_StatusTypeDef MS5607_SetTemperatureOSR(MS5607_HandleTypeDef *hms5607, 
                                             MS5607_OsrTypeDef osr);

#ifdef __cplusplus
}
#endif

#endif /* __MS5607_H__ */
