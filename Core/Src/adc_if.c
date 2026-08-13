/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc_if.c
  * @author  MCD Application Team
  * @brief   Read status related to the chip (battery level, VREF, chip temperature)
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

/* Includes ------------------------------------------------------------------*/
#include "adc_if.h"
#include "sys_app.h"
#include "SEGGER_RTT.h"
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */  /* F-014 timeout visibility (#31) */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/**
  * @brief ADC handle
  */
extern ADC_HandleTypeDef hadc;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define TEMPSENSOR_TYP_CAL1_V          (( int32_t)  760)        /*!< Internal temperature sensor, parameter V30 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define TEMPSENSOR_TYP_AVGSLOPE        (( int32_t) 2500)        /*!< Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief This function reads the ADC channel
  * @param channel channel number to read
  * @return adc measured level value
  */
static uint32_t ADC_ReadChannels(uint32_t channel);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* R16/F-014/A-001 (#31): ADC housekeeping state.
 * s_adc_ready: init+calibrate happen ONCE per wake cycle (the LPM pre-sleep
 * path deinits the ADC and calls SYS_ADC_NoteDeinit). s_vdda_mv: VDDA cached
 * per cycle — battery and solar conversions no longer each re-read VREFINT. */
static bool s_adc_ready = false;
static uint16_t s_vdda_mv = 0;

#define ADC_POLL_TIMEOUT_MS  10U   /* F-014: bounded poll — a faulted ADC must
                                    * never wedge the work cycle (was HAL_MAX_DELAY) */

/* S-06 (#231): VDDA is cached per wake cycle, but the cache was only
 * invalidated on STOP2 entry - and a bulk burst or ASCENT back-to-back
 * cycles need not enter STOP2 (LmHandler holds stop mode off across TX/RX
 * windows). Every battery/solar conversion then referenced a VDDA sample
 * taken before the burst, i.e. before the TX load that sags the rail.
 * Invalidate at the top of each work cycle; keeps s_adc_ready, so this
 * costs one VREFINT conversion, not a re-init + recalibration. */
void SYS_ADC_InvalidateVdda(void)
{
  s_vdda_mv = 0;
}

/* Called by the LPM pre-sleep path after HAL_ADC_DeInit (stm32_lpm_if.c). */
void SYS_ADC_NoteDeinit(void)
{
  s_adc_ready = false;
  s_vdda_mv = 0;
}

/* Exported functions --------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void SYS_InitMeasurement(void)
{
  /* USER CODE BEGIN SYS_InitMeasurement_1 */

  /* USER CODE END SYS_InitMeasurement_1 */
  hadc.Instance = ADC;
  /* USER CODE BEGIN SYS_InitMeasurement_2 */

  /* USER CODE END SYS_InitMeasurement_2 */
}

void SYS_DeInitMeasurement(void)
{
  /* USER CODE BEGIN SYS_DeInitMeasurement_1 */

  /* USER CODE END SYS_DeInitMeasurement_1 */
}

int16_t SYS_GetTemperatureLevel(void)
{
  /* USER CODE BEGIN SYS_GetTemperatureLevel_1 */

  /* USER CODE END SYS_GetTemperatureLevel_1 */
  __IO int16_t temperatureDegreeC = 0;
  uint32_t measuredLevel = 0;
  uint16_t batteryLevelmV = SYS_GetBatteryLevel();

  measuredLevel = ADC_ReadChannels(ADC_CHANNEL_TEMPSENSOR);

  /* convert ADC level to temperature */
  /* check whether device has temperature sensor calibrated in production */
  if (((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) != 0)
  {
    /* Device with temperature sensor calibrated in production:
       use device optimized parameters */
    temperatureDegreeC = __LL_ADC_CALC_TEMPERATURE(batteryLevelmV,
                                                   measuredLevel,
                                                   LL_ADC_RESOLUTION_12B);
  }
  else
  {
    /* Device with temperature sensor not calibrated in production:
       use generic parameters */
    temperatureDegreeC = __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(TEMPSENSOR_TYP_AVGSLOPE,
                                                              TEMPSENSOR_TYP_CAL1_V,
                                                              TEMPSENSOR_CAL1_TEMP,
                                                              batteryLevelmV,
                                                              measuredLevel,
                                                              LL_ADC_RESOLUTION_12B);
  }

  /* from int16 to q8.7*/
  temperatureDegreeC <<= 8;

  return (int16_t) temperatureDegreeC;
  /* USER CODE BEGIN SYS_GetTemperatureLevel_2 */

  /* USER CODE END SYS_GetTemperatureLevel_2 */
}

uint16_t SYS_GetBatteryLevel(void)
{
  /* USER CODE BEGIN SYS_GetBatteryLevel_1 */

  /* USER CODE END SYS_GetBatteryLevel_1 */
  uint16_t batteryLevelmV = 0;
  uint32_t measuredLevel = 0;

  /* A-001 (#31): VDDA is cached per wake cycle — the battery and solar
   * conversions both consume it; one VREFINT read serves all three users. */
  if (s_vdda_mv != 0)
  {
    return s_vdda_mv;
  }

  measuredLevel = ADC_ReadChannels(ADC_CHANNEL_VREFINT);

  if (measuredLevel == 0)
  {
    batteryLevelmV = 0;
  }
  else
  {
    if ((uint32_t)*VREFINT_CAL_ADDR != (uint32_t)0xFFFFU)
    {
      /* Device with Reference voltage calibrated in production:
         use device optimized parameters */
      batteryLevelmV = __LL_ADC_CALC_VREFANALOG_VOLTAGE(measuredLevel,
                                                        ADC_RESOLUTION_12B);
    }
    else
    {
      /* Device with Reference voltage not calibrated in production:
         use generic parameters */
      batteryLevelmV = (VREFINT_CAL_VREF * 1510) / measuredLevel;
    }
  }

  s_vdda_mv = batteryLevelmV;   /* A-001 (#31): cache for this wake cycle */
  return batteryLevelmV;
  /* USER CODE BEGIN SYS_GetBatteryLevel_2 */

  /* USER CODE END SYS_GetBatteryLevel_2 */
}

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */
/**
  * @brief Get the battery voltage from PB4 (ADC_CHANNEL_3)
  * @note  PB4 has a 0.5 voltage divider, function applies 2x scaling
  * @return value battery voltage in mV
  */
/* #136 (2026-08-10 finding #6): R28-style plausibility gate + last-known-good
 * cache + stale flag for the battery path — the one sensor that feeds the
 * power state machine had none of it. A single failed conversion (returns 0)
 * could latch SURVIVAL via the raw < 4300 mV floor and poison the voltage-
 * slope baseline for up to 2 h. Window covers the divider ceiling (~6600 mV)
 * and the LTO floor. batt_stale rides into telemetry via sensor_t.batt_stale
 * and flash record flag b4. */
#define BATT_PLAUSIBLE_MIN_MV   2500U
#define BATT_PLAUSIBLE_MAX_MV   7000U
static uint16_t s_batt_last_good_mv = 0;
static uint8_t  s_batt_have_good = 0;
static uint8_t  s_batt_stale = 1;   /* stale until the first plausible read */

uint8_t SYS_BatteryIsStale(void)
{
  return s_batt_stale;
}

uint16_t SYS_GetBatteryVoltage(void)
{
  uint16_t batteryVoltagemV = 0;
  uint32_t measuredLevel = 0;

  /* Read ADC channel 3 (PB4) with 0.5 voltage divider */
  measuredLevel = ADC_ReadChannels(ADC_CHANNEL_3);

  if (measuredLevel == 0)
  {
    batteryVoltagemV = 0;
  }
  else
  {
    /* F18 FIX: Ratiometric conversion using measured VDDA (VREFINT).
       VDDA sags in cold/under load; assuming 3300 mV caused ~275 mV error
       at the 4300 mV survival floor. Fall back to 3300 if VDDA read fails. */
    uint16_t vdda_mv = SYS_GetBatteryLevel();
    if (vdda_mv == 0)
    {
      vdda_mv = 3300;
    }
    /* ADC is 12-bit (0-4095); apply 2x scaling for 0.5 voltage divider */
    batteryVoltagemV = (uint16_t)(((uint32_t)measuredLevel * vdda_mv / 4096) * 2);
  }

  /* #136: plausibility gate BEFORE the last-known-good cache accepts a value
   * (mirrors the R28 SHT31/MS5607 pattern). A rejected read serves the cache;
   * with no history it returns 0 — the conservative direction (the 4300 mV
   * floor picks SURVIVAL; never a fantasy full battery). */
  if (batteryVoltagemV >= BATT_PLAUSIBLE_MIN_MV && batteryVoltagemV <= BATT_PLAUSIBLE_MAX_MV)
  {
    s_batt_last_good_mv = batteryVoltagemV;
    s_batt_have_good = 1;
    s_batt_stale = 0;
  }
  else
  {
    s_batt_stale = 1;
    if (s_batt_have_good)
    {
      batteryVoltagemV = s_batt_last_good_mv;
    }
  }

  return batteryVoltagemV;
}

/**
  * @brief Get the solar panel voltage from PB3 (ADC_CHANNEL_2)
  * @note  PB3 has no voltage divider, no scaling applied
  * @return value solar panel voltage in mV
  */
uint16_t SYS_GetSolarVoltage(void)
{
  uint16_t solarVoltagemV = 0;
  uint32_t measuredLevel = 0;

  /* Read ADC channel 2 (PB3) with no voltage divider */
  measuredLevel = ADC_ReadChannels(ADC_CHANNEL_2);

  if (measuredLevel == 0)
  {
    solarVoltagemV = 0;
  }
  else
  {
    /* F18 FIX: Ratiometric conversion using measured VDDA (same bug as battery).
       No voltage divider on PB3, so no scaling needed. */
    uint16_t vdda_mv = SYS_GetBatteryLevel();
    if (vdda_mv == 0)
    {
      vdda_mv = 3300;
    }
    solarVoltagemV = (uint16_t)((uint32_t)measuredLevel * vdda_mv / 4096);
  }

  return solarVoltagemV;
}
/* USER CODE END PrFD */

static uint32_t ADC_ReadChannels(uint32_t channel)
{
  /* USER CODE BEGIN ADC_ReadChannels_1 */

  /* USER CODE END ADC_ReadChannels_1 */
  uint32_t ADCxConvertedValues = 0;
  ADC_ChannelConfTypeDef sConfig = {0};

  /* A-001 (#31): init + calibrate once per wake cycle, not per channel read.
   * The LPM pre-sleep path deinits and calls SYS_ADC_NoteDeinit. */
  if (!s_adc_ready)
  {
    MX_ADC_Init();
    /* Start Calibration */
    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
    {
      Error_Handler();
      /* #136: do NOT mark the ADC ready after a failed calibration — the old
       * code set s_adc_ready anyway and used an uncalibrated ADC silently for
       * the rest of the wake cycle. Retry init+calibrate on the next read. */
    }
    else
    {
      s_adc_ready = true;
    }
  }

  /* Configure Regular Channel */
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    /* R10 (#195): terminate the failed transaction - do NOT continue to
     * Start with an unconfigured (or stale, wrong-channel) config: that
     * could return a plausible-looking value treated as fresh. Return 0
     * (read-failure); the #136 plausibility/stale layer degrades honestly. */
    Error_Handler();
    HAL_ADC_Stop(&hadc);
    return 0;
  }

  if (HAL_ADC_Start(&hadc) != HAL_OK)
  {
    /* Start Error - R10 (#195): abort the transaction, not a pointless poll. */
    Error_Handler();
    HAL_ADC_Stop(&hadc);
    return 0;
  }
  /** Wait for end of conversion — F-014 (#31): BOUNDED poll (was
   *  HAL_MAX_DELAY: a faulted ADC wedged the work cycle forever). On timeout
   *  return 0; every caller treats 0 as read-failure and uses its stale path. */
  if (HAL_ADC_PollForConversion(&hadc, ADC_POLL_TIMEOUT_MS) != HAL_OK)
  {
    SONDE_LOG("ADC: TIMEOUT on channel %lu - read failed\r\n",
                      (unsigned long)channel);
    HAL_ADC_Stop(&hadc);
    return 0;
  }

  HAL_ADC_Stop(&hadc);   /* it calls also ADC_Disable() */

  ADCxConvertedValues = HAL_ADC_GetValue(&hadc);

  return ADCxConvertedValues;
  /* USER CODE BEGIN ADC_ReadChannels_2 */

  /* USER CODE END ADC_ReadChannels_2 */
}
