/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    sys_sensors.c
 * @author  MCD Application Team
 * @brief   Manages the sensors on the application
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
#include "sys_sensors.h"
#include "platform.h"
#include "stdint.h"
#include "sys_conf.h"
#if defined(SENSOR_ENABLED) && (SENSOR_ENABLED == 0)
#include "adc_if.h"
#endif /* SENSOR_ENABLED */

/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "adc_if.h"
#include "atgm336h.h"
#include "sonde_log.h" /* R50 (#47): compile-time log gate */
#if defined(SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
#include "ms5607.h"
#include "sht31.h"
#include "sys_app.h" /* For APP_LOG */
#if defined(X_NUCLEO_IKS01A2)
#warning "IKS drivers are today available for several families but not stm32WL"
#warning "up to the user adapt IKS low layer to map it on WL board driver"
#warning "this code would work only if user provide necessary IKS and BSP layers"
#include "iks01a2_env_sensors.h"
#elif defined(X_NUCLEO_IKS01A3)

/*
## How to add IKS01A3 to STM32CubeWL
   Note that LoRaWAN_End_Node Example is used as an example for steps below.
 1. Open the LoRaWAN_End_Node CubeMX project by double-clicking on the LoRaWAN_End_Node.ioc under "STM32Cube_FW_WL_V1.x.x\Projects\NUCLEO-WL55JC\Applications\LoRaWAN\LoRaWAN_End_Node"
 2. From the CubeMX project, click on "Software Packs"->"Manage Software Packs" to open the Embedded Software Packages Manager. Then, click on the "STMicroelectronics" tab, expand the X-CUBE-MEMS1, check the latest version of this pack (i.e. 9.0.0), and install. Then, close the Embedded Software Packages Manager.
 3. From the CubeMX project, click on "Software Packs"->"Select Components" to open the Software Packs Component Selector, expand the X-CUBE-MEMS1 pack and select the "Board Extension IKS01A3" component by checking the respective box, and click OK.
 4. From the CubeMX project, expand the "Connectivity" category and enable I2C2 on pins PA11 (I2C2_SDA) and PA12 (I2C2_SCK).
 5. From the CubeMX project, expand the "Software Packs" category and enable the "Board Extension IKS01A3" by checking the box, and choose I2C2 under the "Found Solutions" menu.
 6. From the CubeMX project, click the "Project Manager" section
    - From the "Project Settings" section, select your Toolchain/IDE of choice (if CubeIDE, uncheck the "Generator Under Root" option).
    - From the "Code Generator" section, select "Copy only the necessary library files".
 7. Click "GENERATE CODE" to generate the code project with the MEMS drivers integrated.
 8. From the code project, find and open the sys_conf.h and make the following edits
    - Set the #define SENSOR_ENABLED to 1
    - Set the #define LOW_POWER_DISABLE to 1 to prevent the device from entering low power mode. This is needed, since the I2C2 requires handling when exiting low power modes, so to prevent issues, best is to disable low power mode, however, if low power mode is desired, you'll have to re-initialize the I2C2 from PWR_ExitStopMode() in stm32_lpm_if.c, so you can just call HAL_I2C_Init() from there.
 9. From the code project, find and open lora_app.h, and uncomment the following line
    #define CAYENNE_LPP
 10. From the code project properties, add X_NUCLEO_IKS01A3 Pre-processor Defined symbol.
 11. Save all changes and build project
 12. Connect the X-NUCLEO-IKS01A3 expansion board on the NUCLEO-WL55JC1
 13. Load and run the code
*/
#warning "IKS drivers are today available for several families but not stm32WL, follow steps defined in sys_sensors.c"
#include "iks01a3_env_sensors.h"
#else  /* not X_IKS01xx */
/* Using custom SHT31 driver, no error needed */
#endif /* X_NUCLEO_IKS01xx */
#elif !defined(SENSOR_ENABLED)
#error SENSOR_ENABLED not defined
#endif /* SENSOR_ENABLED */
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define STSOP_LATTITUDE ((float)43.618622) /*!< default latitude position */
#define STSOP_LONGITUDE ((float)7.051415)  /*!< default longitude position */
#define MAX_GPS_POS ((int32_t)8388607)     /*!< 2^23 - 1 */
#define HUMIDITY_DEFAULT_VAL 50.0f         /*!< default humidity */
#define TEMPERATURE_DEFAULT_VAL 18.0f      /*!< default temperature */
#define PRESSURE_DEFAULT_VAL 1000.0f       /*!< default pressure */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* GNSS module handle */
GNSS_HandleTypeDef hgnss;

/* F9/T2 (DDR-0003): last-known-good cache + stale flags.
 * No fabricated defaults downstream: a failed read serves the cached value
 * and sets the stale bit; the bit survives the whole pipeline. */
static float s_last_temp = TEMPERATURE_DEFAULT_VAL;
static float s_last_hum = HUMIDITY_DEFAULT_VAL;
static bool s_have_th = false; /* never had a good SHT31 read */
static float s_last_press = PRESSURE_DEFAULT_VAL;
static bool s_have_press = false; /* FW-7: never had a good MS5607 read */
static bool s_gnss_stale = true;  /* stale until first real fix */

void EnvSensors_MarkGnssStale(bool stale) {
  s_gnss_stale = stale;
}

/* F-3 (#178): the region auto-switch site must consult freshness - a stale
 * (forged last-known) fix may INHIBIT but never SWITCH (DDR-0015). */
bool EnvSensors_GnssIsStale(void) {
  return s_gnss_stale;
}

/* UART handle - declared in main.c but we need to access it here for GNSS */
extern UART_HandleTypeDef huart1;

#if defined(SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
/* SHT31 sensor handle */
SHT31_HandleTypeDef hsht31;

/* MS5607 sensor handle */
MS5607_HandleTypeDef hms5607;

/* I2C handle - declared in main.c but we need to access it here */
extern I2C_HandleTypeDef hi2c2;

#if defined(X_NUCLEO_IKS01A2)
#warning "IKS drivers are today available for several families but not stm32WL"
#warning "up to the user adapt IKS low layer to map it on WL board driver"
#warning "this code would work only if user provide necessary IKS and BSP layers"
IKS01A2_ENV_SENSOR_Capabilities_t EnvCapabilities;
#elif defined(X_NUCLEO_IKS01A3)
IKS01A3_ENV_SENSOR_Capabilities_t EnvCapabilities;
#endif /* X_NUCLEO_IKS01 */
#elif !defined(SENSOR_ENABLED)
#error SENSOR_ENABLED not defined
#endif /* SENSOR_ENABLED */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
#if defined(SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
static void I2C_BusRecover(void); /* F20: 9-clock + STOP bus recovery */
static void I2C_NoteResult(bool any_success);
#endif
/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
int32_t EnvSensors_Read(sensor_t *sensor_data) {
  /* USER CODE BEGIN EnvSensors_Read */
  float HUMIDITY_Value = HUMIDITY_DEFAULT_VAL;
  float TEMPERATURE_Value = TEMPERATURE_DEFAULT_VAL;
  float PRESSURE_Value = PRESSURE_DEFAULT_VAL;
  bool th_stale = true;    /* F9: stale until a good SHT31 read (also covers SENSOR_ENABLED=0) */
  bool press_stale = true; /* FW-7: stale until a good MS5607 read */

  /* GNSS processing removed - module is powered off to prevent LoRaWAN interference */
  /* Re-enable when GNSS power management is coordinated with LoRaWAN timing */
  /* GNSS_ProcessDMABuffer(&hgnss); */

#if defined(SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
  /* Try to read real sensor values */
  int32_t sht_temp_scaled, sht_hum_scaled;
  float ms_temp, ms_press;

  /* Read SHT31 sensor */
  SONDE_LOG_STR("SEN: sht31...\r\n"); /* [DIAG] */
  if (SHT31_ReadTempAndHumidity(&hsht31, &sht_temp_scaled, &sht_hum_scaled) == SHT31_OK) {
    float t_c = sht_temp_scaled / 100.0f;
    float h_pct = sht_hum_scaled / 100.0f;
    /* R28 (#36): plausibility gate BEFORE last-known-good cache accept — a
     * corrupt read must never poison the cache that stale mode serves. */
    if (t_c >= -90.0f && t_c <= 85.0f && h_pct >= 0.0f && h_pct <= 100.0f) {
      TEMPERATURE_Value = t_c;
      HUMIDITY_Value = h_pct;
      /* F9 FIX: update last-known-good cache, clear stale */
      s_last_temp = TEMPERATURE_Value;
      s_last_hum = HUMIDITY_Value;
      s_have_th = true;
      th_stale = false;
      /* Print using integers (no float printf support needed) */
      SONDE_LOG("SHT31: T=%d.%d°C, H=%d.%d%%\r\n",
                sht_temp_scaled / 100, (sht_temp_scaled % 100) / 10,
                sht_hum_scaled / 100, (sht_hum_scaled % 100) / 10);
    } else {
      if (s_have_th) {
        TEMPERATURE_Value = s_last_temp;
        HUMIDITY_Value = s_last_hum;
      }
      th_stale = true;
      SONDE_LOG_STR("SHT31 implausible read rejected (STALE)\r\n");
    }
  } else {
    /* F9 FIX: serve last-known-good (never the +18°C fantasy default once we
     * have a real reading) and mark stale. Fail safe, not fail sunny. */
    if (s_have_th) {
      TEMPERATURE_Value = s_last_temp;
      HUMIDITY_Value = s_last_hum;
    }
    th_stale = true;
    SONDE_LOG_STR("SHT31 read failed, using last-known-good (STALE)\r\n");
  }
  SONDE_LOG_STR("SEN: sht31 done\r\n"); /* [DIAG] */

  /* Read MS5607 sensor */
  SONDE_LOG_STR("SEN: ms5607...\r\n"); /* [DIAG] */
  if (MS5607_ReadPressureAndTemperature(&hms5607, &ms_temp, &ms_press) == MS5607_OK) {
    /* R28 (#36): plausibility gate before cache accept (defense in depth —
     * the driver gates too, 1..1200 hPa) */
    if (ms_press >= 1.0f && ms_press <= 1200.0f) {
      PRESSURE_Value = ms_press;
      /* FW-7: update last-known-good cache, clear stale (mirrors F9 SHT31 pattern) */
      s_last_press = PRESSURE_Value;
      s_have_press = true;
      press_stale = false;
      /* Use MS5607 temperature as backup/verification */
      int press_int = (int)(ms_press * 10);
      int temp_int = (int)(ms_temp * 10);
      (void)press_int;
      (void)temp_int; /* FR-19: log-only in flight */
      SONDE_LOG("MS5607: P=%d.%d hPa, T=%d.%d°C\r\n",
                press_int / 10, press_int % 10,
                temp_int / 10, temp_int % 10);
    } else {
      if (s_have_press) {
        PRESSURE_Value = s_last_press;
      }
      press_stale = true;
      SONDE_LOG_STR("MS5607 implausible pressure rejected (STALE)\r\n");
    }
  } else {
    /* FW-7: serve last-known-good (never the 1000.0 hPa sea-level default once
     * we have a real reading) and mark stale. A failed read must never
     * transmit 1000.0 hPa as real float-altitude science data. */
    if (s_have_press) {
      PRESSURE_Value = s_last_press;
    }
    press_stale = true;
    SONDE_LOG_STR("MS5607 read failed, using last-known-good (STALE)\r\n");
  }

  /* F20: track consecutive total bus failures; recover the bus after 3.
   * FW-7: track a real read-success boolean — the old sentinel compare
   * (PRESSURE_Value != PRESSURE_DEFAULT_VAL) mis-counted a legitimate
   * 1000.0 hPa reading as a bus failure. */
  I2C_NoteResult((!th_stale) || (!press_stale));
#else
  SONDE_LOG_STR("Sensors disabled, using default values\r\n");
#endif

  /* F25 FIX: LED flash + HAL_Delay(50) removed from flight path.
   * LEDs are COMMISSIONING-only (DDR-0002); blocking delays burn power. */

  /* Set sensor data */
  sensor_data->humidity = HUMIDITY_Value;
  sensor_data->temperature = TEMPERATURE_Value;
  sensor_data->pressure = PRESSURE_Value;
  sensor_data->temp_stale = th_stale ? 1 : 0;
  sensor_data->hum_stale = th_stale ? 1 : 0;
  sensor_data->press_stale = press_stale ? 1 : 0;
  sensor_data->gnss_stale = 1; /* Default stale; cleared below only if GNSS data flows */

  /* Read battery voltage from ADC (PB4 with voltage divider).
   * #136: the read is plausibility-gated in adc_if.c; carry its staleness so
   * telemetry marks a last-known-good (or rejected) sample as untrustworthy,
   * matching the R28 treatment of every other sensor. */
  sensor_data->battery_voltage = SYS_GetBatteryVoltage() / 1000.0f; /* Convert mV to V */
  sensor_data->batt_stale = SYS_BatteryIsStale();

  /* Read regulator voltage (VDDA/3.3V rail) from internal reference */
  sensor_data->regulator_voltage = SYS_GetBatteryLevel() / 1000.0f; /* Convert mV to V */

  /* Read solar panel voltage from ADC (PB3, no voltage divider) */
  sensor_data->solar_voltage = SYS_GetSolarVoltage() / 1000.0f; /* Convert mV to V */

  /* F-8 (#183) / F-9 (#184): the log must reuse THIS cycle's samples (already
   * in sensor_data above) instead of running three more ADC conversions.
   * Previously the extra battery re-read also overwrote s_batt_stale,
   * so the flag exported at :277 could describe a different conversion than
   * the voltage exported at :276 - and under SONDE_FLIGHT_BUILD the three
   * conversions ran purely to feed compiled-out logs (FR-16/R2-15 class). */
  int batt_mv = (int)(sensor_data->battery_voltage * 1000.0f);
  int vdda_mv = (int)(sensor_data->regulator_voltage * 1000.0f); /* VDDA rail */
  int solar_mv = (int)(sensor_data->solar_voltage * 1000.0f);
  (void)batt_mv;
  (void)vdda_mv;
  (void)solar_mv; /* FR-19: log-only in flight */
  SONDE_LOG("Battery: %d.%02d V (%d mV) | VDDA: %d.%02d V (%d mV)\r\n",
            batt_mv / 1000, (batt_mv % 1000) / 10, batt_mv,
            vdda_mv / 1000, (vdda_mv % 1000) / 10, vdda_mv);
  SONDE_LOG("Solar: %d.%02d V (%d mV)\r\n",
            solar_mv / 1000, (solar_mv % 1000) / 10, solar_mv);

  /* Use real GNSS data if available from hgnss (populated by SendTxData's GPS collection)
   * GPS is powered on/off in SendTxData before calling EnvSensors_Read
   * hgnss.data contains the latest parsed NMEA data */
  EnvSensors_MergeGnss(sensor_data);

  /* F-030 (#59): per-sensor freshness bitmask — the top-level return no
   * longer hides total acquisition failure. 1 bit = fresh live read this
   * cycle; 0 = stale/never-acquired (the staleness flags carry the detail). */
  int32_t status = 0;
  if (!sensor_data->temp_stale)
    status |= ENV_SENSORS_FRESH_TEMP;
  if (!sensor_data->hum_stale)
    status |= ENV_SENSORS_FRESH_HUMIDITY;
  if (!sensor_data->press_stale)
    status |= ENV_SENSORS_FRESH_PRESSURE;
  if (!sensor_data->gnss_stale && sensor_data->gnss_valid)
    status |= ENV_SENSORS_FRESH_GNSS;
  if (!sensor_data->batt_stale)
    status |= ENV_SENSORS_FRESH_BATT;
  return status;
  /* USER CODE END EnvSensors_Read */
}

/* FR-15 (#96): GPS-only merge, no I2C. SendTxData used to call the full
 * EnvSensors_Read() a second time per work cycle just to fold the fresh GPS
 * fix in — a redundant MS5607 OSR_4096 conversion pair (~9 ms each) plus a
 * full SHT31 read. This is the GNSS block of EnvSensors_Read(), extracted
 * verbatim; semantics (zeros-on-no-fix, staleness honesty) unchanged. */
void EnvSensors_MergeGnss(sensor_t *sensor_data) {
  if (hgnss.is_initialized && hgnss.data.valid &&
      hgnss.data.fix_quality != GNSS_FIX_INVALID &&
      GNSS_ValidateCoordinates(hgnss.data.latitude, hgnss.data.longitude)) {

    /* Convert decimal degrees to scaled integer format for Cayenne LPP */
    sensor_data->latitude = (int32_t)((hgnss.data.latitude * MAX_GPS_POS) / 90.0f);
    sensor_data->longitude = (int32_t)((hgnss.data.longitude * MAX_GPS_POS) / 180.0f);
    sensor_data->altitudeGps = (int32_t)(hgnss.data.altitude + (hgnss.data.altitude >= 0.0f ? 0.5f : -0.5f)); /* D5/#35: int32 metres */
    sensor_data->satellites = hgnss.data.satellites;
    sensor_data->gnss_fix_quality = hgnss.data.fix_quality;
    sensor_data->gnss_hdop = hgnss.data.hdop;
    sensor_data->gnss_valid = true;
    sensor_data->gnss_stale = s_gnss_stale ? 1 : 0; /* T2: honesty from the TX path */

    SONDE_LOG("GNSS: Valid fix | Sats:%d%s\r\n", hgnss.data.satellites,
              s_gnss_stale ? " (STALE)" : "");
  } else {
    /* No valid GPS fix (R20/#57): store ZEROS, not the ST demo coordinates
     * (43.6186, 7.0514 = Saint-Ouen, France) — a decoder that skipped the
     * flags byte used to put the balloon in France. satellites means "used in
     * fix" — 0 without one (constant semantics); in-view count is logged. */
    sensor_data->latitude = 0;
    sensor_data->longitude = 0;
    sensor_data->altitudeGps = 0;
    sensor_data->satellites = 0;
    sensor_data->gnss_fix_quality = 0;
    sensor_data->gnss_hdop = 99.9f;
    sensor_data->gnss_valid = false;
    /* DR-14: set gnss_stale explicitly - the one field in this block that
     * otherwise depended on caller convention (EnvSensors_Read pre-sets 1;
     * the direct SendTxData call zero-init'd it to "fresh"). No fix is
     * definitionally stale; never inherit the caller's default (#35 class). */
    sensor_data->gnss_stale = 1;

    SONDE_LOG("GNSS: No fix | Sats visible:%d | Position zeroed\r\n",
              hgnss.data.satellites_in_view);
  }
}

int32_t EnvSensors_Init(void) {
  int32_t ret = 0;
  /* USER CODE BEGIN EnvSensors_Init */

  SONDE_LOG_STR("EnvSensors_Init: Starting I2C sensor initialization...\r\n");

#if defined(SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
  /* Initialize SHT31 sensor handle */
  hsht31.hi2c = &hi2c2;
  hsht31.Address = SHT31_I2C_ADDRESS_B; /* Use 0x45 - hardware has ADDR pin HIGH */
  hsht31.Mode = SHT31_MODE_HIGH_PRECISION;

  /* Initialize MS5607 sensor handle */
  hms5607.hi2c = &hi2c2;
  hms5607.Address = MS5607_I2C_ADDRESS_B; /* Use 0x77 - hardware has CSB pin LOW */
  hms5607.PressureOsr = MS5607_OSR_4096;
  hms5607.TemperatureOsr = MS5607_OSR_4096;

  /* Initialize SHT31 sensor */
  if (SHT31_Init(&hsht31) == SHT31_OK) {
    SONDE_LOG_STR("SHT31 sensor initialized successfully\r\n");
  } else {
    SONDE_LOG_STR("SHT31 sensor initialization failed\r\n");
    ret = -1;
  }

  /* Initialize MS5607 sensor */
  if (MS5607_Init(&hms5607) == MS5607_OK) {
    SONDE_LOG_STR("MS5607 sensor initialized successfully\r\n");
  } else {
    SONDE_LOG_STR("MS5607 sensor initialization failed\r\n");
    ret = -2;
  }
#else
  SONDE_LOG_STR("Sensors disabled in configuration\r\n");
#endif

  /* Initialize GNSS module handle */
  SONDE_LOG_STR("Initializing GNSS module...\r\n");
  hgnss.huart = &huart1;
  hgnss.pwr_port = GPIOB;
  hgnss.pwr_pin = GPIO_PIN_10;
  hgnss.en_port = GPIOB;
  hgnss.en_pin = GPIO_PIN_5;

  /* Initialize GNSS module (configures GPIO, sets up structure - does NOT power on) */
  if (GNSS_Init(&hgnss) == GNSS_OK) {
    SONDE_LOG_STR("GNSS module initialized successfully\r\n");
  } else {
    SONDE_LOG_STR("GNSS module initialization FAILED\r\n");
    ret = -3;
  }

  /* GNSS power-on DISABLED during init to prevent interference with LoRaWAN join */
  /* GPS will be powered on/off by SendTxData() callback after join succeeds */
  SONDE_LOG_STR("GNSS init complete - will be powered on during transmissions\r\n");

  SONDE_LOG_STR("EnvSensors_Init: Initialization complete\r\n");
  /* USER CODE END EnvSensors_Init */
  return ret;
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

#if defined(SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
/* F20 (DDR-0009): a wedged I2C slave (SDA held low mid-transfer) must not
 * kill the sensor bus for the rest of the flight — there is no power-gate
 * on the sensors, so the only recovery is bit-banging the slave's state
 * machine back to idle: 9 SCL clocks to flush the stuck byte, then STOP. */
#define I2C_RECOVERY_THRESHOLD 3 /* consecutive all-fail reads before recovery */

static uint8_t s_i2c_fail_count = 0;

static void I2C_BusRecover(void) {
  GPIO_InitTypeDef g = {0};
  HAL_I2C_DeInit(&hi2c2);

  /* Take PA15 (SDA) and PB15 (SCL) as open-drain GPIO outputs */
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &g); /* SDA */
  HAL_GPIO_Init(GPIOB, &g); /* SCL */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(1);

  /* 9 SCL clocks: a slave holding SDA low mid-byte releases after <=9 clocks */
  for (int i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  /* STOP condition: SDA low->high while SCL high */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(1);

  /* Re-init peripheral (MspInit restores PA15/PB15 to AF open-drain) */
  HAL_I2C_Init(&hi2c2);
  /* S-09 (#233): re-apply the filter config from MX_I2C2_Init. It currently
   * matches the peripheral reset defaults, so omitting it was harmless by
   * accident - this makes it harmless by construction. */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
  SONDE_LOG_STR("I2C2 bus recovery: 9-clock + STOP, re-init done\r\n");
}

static void I2C_NoteResult(bool any_success) {
  if (any_success) {
    s_i2c_fail_count = 0;
  } else if (++s_i2c_fail_count >= I2C_RECOVERY_THRESHOLD) {
    s_i2c_fail_count = 0;
    I2C_BusRecover();
  }
}
#endif /* SENSOR_ENABLED */

/* USER CODE END PrFD */
