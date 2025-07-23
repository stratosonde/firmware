/**
  ******************************************************************************
  * @file    sht31.h
  * @brief   Header file for SHT31 temperature and humidity sensor driver
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHT31_H__
#define __SHT31_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_hal.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  SHT31 Status structure definition
  */
typedef enum {
  SHT31_OK      = 0x00,
  SHT31_ERROR   = 0x01,
  SHT31_TIMEOUT = 0x02,
  SHT31_CRC_ERROR = 0x03
} SHT31_StatusTypeDef;

/**
  * @brief  SHT31 Measurement Mode structure definition
  */
typedef enum {
  SHT31_MODE_HIGH_PRECISION = 0x00,   /* High precision measurement mode */
  SHT31_MODE_MEDIUM_PRECISION = 0x01, /* Medium precision measurement mode */
  SHT31_MODE_LOW_PRECISION = 0x02     /* Low precision measurement mode */
} SHT31_MeasurementModeTypeDef;

/**
  * @brief  SHT31 Handle Structure definition
  */
typedef struct {
  I2C_HandleTypeDef *hi2c;        /* I2C handle */
  uint8_t Address;                /* I2C device address */
  SHT31_MeasurementModeTypeDef Mode; /* Measurement mode */
} SHT31_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/
/* SHT31 I2C device addresses */
#define SHT31_I2C_ADDRESS_A  0x44 /* ADDR pin connected to GND */
#define SHT31_I2C_ADDRESS_B  0x45 /* ADDR pin connected to VDD */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Initialize the SHT31 sensor
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure that contains
  *                the configuration information for the SHT31 sensor
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_Init(SHT31_HandleTypeDef *hsht31);

/**
  * @brief  Read temperature and humidity from SHT31 sensor
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @param  Temperature pointer to temperature value (in °C * 100)
  * @param  Humidity pointer to humidity value (in % * 100)
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_ReadTempAndHumidity(SHT31_HandleTypeDef *hsht31, 
                                             int32_t *Temperature, 
                                             int32_t *Humidity);

/**
  * @brief  Read the SHT31 serial number
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @param  SerialNumber pointer to store the serial number
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_ReadSerialNumber(SHT31_HandleTypeDef *hsht31, 
                                          uint32_t *SerialNumber);

/**
  * @brief  Soft reset the SHT31 sensor
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_SoftReset(SHT31_HandleTypeDef *hsht31);

/**
  * @brief  Read the SHT31 status register
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @param  Status pointer to store the status register value
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_ReadStatus(SHT31_HandleTypeDef *hsht31, 
                                    uint16_t *Status);

/**
  * @brief  Clear the SHT31 status register
  * @param  hsht31 pointer to a SHT31_HandleTypeDef structure
  * @retval SHT31 status
  */
SHT31_StatusTypeDef SHT31_ClearStatus(SHT31_HandleTypeDef *hsht31);

#ifdef __cplusplus
}
#endif

#endif /* __SHT31_H__ */
