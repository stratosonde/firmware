/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sys_sensors.h
  * @author  MCD Application Team
  * @brief   Header for sensors application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_H__
#define __SENSORS_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "atgm336h.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/**
  * Sensor data parameters
  */
typedef struct
{
  float pressure;         /*!< in mbar */
  float temperature;      /*!< in degC */
  float humidity;         /*!< in % */
  int32_t latitude;       /*!< latitude converted to binary */
  int32_t longitude;      /*!< longitude converted to binary */
  int32_t altitudeGps;    /*!< in m (D5/#35: widened int16->int32; float altitude >32767 m overflowed) */
  /* altitudeBar DELETED (D5/#35): never assigned, ground computes barometric
   * altitude from pressure+temperature (documented design). */
  /**more may be added*/
  /* USER CODE BEGIN sensor_t */
  uint8_t satellites;     /*!< number of satellites */
  uint8_t gnss_fix_quality; /*!< GNSS fix quality (0=invalid, 1=GPS, 2=DGPS) */
  float gnss_hdop;        /*!< horizontal dilution of precision */
  bool gnss_valid;        /*!< GNSS data validity flag */
   float battery_voltage;  /*!< battery voltage in volts (LOADED: post-GNSS, receiver hot) */
   float battery_rest_voltage; /*!< battery voltage in volts (RESTING: pre-GNSS, receiver off).
                                    *  Set once per cycle before GNSS; EnvSensors_Read never writes it,
                                    *  so it survives the post-GNSS re-read. Feeds battery_rest_mv on the wire. */
   float regulator_voltage; /*!< regulator voltage (VDDA/3.3V rail) in volts */
   float solar_voltage;    /*!< solar panel voltage in volts */
   /* T2/DDR-0003 data honesty: a reading carries its own freshness.
    * 1 = value is last-known-good (or default), NOT a live read. */
   uint8_t temp_stale;     /*!< temperature is stale (SHT31 read failed) */
   uint8_t hum_stale;      /*!< humidity is stale (SHT31 read failed) */
   uint8_t press_stale;    /*!< FW-7: pressure is stale (MS5607 read failed) */
   uint8_t gnss_stale;     /*!< position is last-known-good, not a fresh fix */
   uint8_t batt_stale;     /*!< #136: battery voltage is last-known-good (ADC read rejected) */
   /* USER CODE END sensor_t */
 } sensor_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* USER CODE BEGIN EC */
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1) && defined (X_NUCLEO_IKS01A2)
#define HTS221_0    0U
#define LPS22HB_0   1U
#endif /* SENSOR_ENABLED & X_NUCLEO_IKS01A2 */
/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern GNSS_HandleTypeDef hgnss;
/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  initialize the environmental sensor
  */
int32_t EnvSensors_Init(void);

/**
  * @brief  Environmental sensor  read.
  * @param  sensor_data sensor data
  */
 int32_t EnvSensors_Read(sensor_t *sensor_data);

/* FR-15 (#96): merge the current hgnss fix into an existing sensor_t with NO
 * I2C traffic — use after GPS acquisition instead of a second full read. */
void EnvSensors_MergeGnss(sensor_t *sensor_data);

/* F-030 (#59): EnvSensors_Read freshness bitmask bits */
#define ENV_SENSORS_FRESH_TEMP       0x01
#define ENV_SENSORS_FRESH_HUMIDITY   0x02
#define ENV_SENSORS_FRESH_PRESSURE   0x04
#define ENV_SENSORS_FRESH_GNSS       0x08
#define ENV_SENSORS_FRESH_BATT       0x10  /* #136 */

/**
  * @brief  Mark GNSS position data as stale/fresh (T2 / DDR-0003).
  *         Called by the TX path: true on fix timeout (last-known-good in
  *         use), false on a real fix. Stale at boot until first fix.
  */
void EnvSensors_MarkGnssStale(bool stale);

/**
  * @brief  F-3 (#178): is the current GNSS position stale (last-known-good,
  *         not a fresh fix this cycle)? The region auto-switch consults this:
  *         stale may INHIBIT but never SWITCH (DDR-0015).
  */
bool EnvSensors_GnssIsStale(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H__ */
