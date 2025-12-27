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
#include "stdint.h"
#include "platform.h"
#include "sys_conf.h"
#include "sys_sensors.h"
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 0)
#include "adc_if.h"
#endif /* SENSOR_ENABLED */

/* USER CODE BEGIN Includes */
#include "atgm336h.h"
#include "adc_if.h"
#include "SEGGER_RTT.h"
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
#include "sht31.h"
#include "ms5607.h"
#include "sys_app.h" /* For APP_LOG */
#if defined (X_NUCLEO_IKS01A2)
#warning "IKS drivers are today available for several families but not stm32WL"
#warning "up to the user adapt IKS low layer to map it on WL board driver"
#warning "this code would work only if user provide necessary IKS and BSP layers"
#include "iks01a2_env_sensors.h"
#elif defined (X_NUCLEO_IKS01A3)

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
#endif  /* X_NUCLEO_IKS01xx */
#elif !defined (SENSOR_ENABLED)
#error SENSOR_ENABLED not defined
#endif  /* SENSOR_ENABLED */
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define STSOP_LATTITUDE           ((float) 43.618622 )  /*!< default latitude position */
#define STSOP_LONGITUDE           ((float) 7.051415  )  /*!< default longitude position */
#define MAX_GPS_POS               ((int32_t) 8388607 )  /*!< 2^23 - 1 */
#define HUMIDITY_DEFAULT_VAL      50.0f                 /*!< default humidity */
#define TEMPERATURE_DEFAULT_VAL   18.0f                 /*!< default temperature */
#define PRESSURE_DEFAULT_VAL      1000.0f               /*!< default pressure */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* GNSS module handle */
GNSS_HandleTypeDef hgnss;

/* UART handle - declared in main.c but we need to access it here for GNSS */
extern UART_HandleTypeDef huart1;

#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
/* SHT31 sensor handle */
SHT31_HandleTypeDef hsht31;

/* MS5607 sensor handle */
MS5607_HandleTypeDef hms5607;

/* I2C handle - declared in main.c but we need to access it here */
extern I2C_HandleTypeDef hi2c2;


#if defined (X_NUCLEO_IKS01A2)
#warning "IKS drivers are today available for several families but not stm32WL"
#warning "up to the user adapt IKS low layer to map it on WL board driver"
#warning "this code would work only if user provide necessary IKS and BSP layers"
IKS01A2_ENV_SENSOR_Capabilities_t EnvCapabilities;
#elif defined (X_NUCLEO_IKS01A3)
IKS01A3_ENV_SENSOR_Capabilities_t EnvCapabilities;
#endif  /* X_NUCLEO_IKS01 */
#elif !defined (SENSOR_ENABLED)
#error SENSOR_ENABLED not defined
#endif  /* SENSOR_ENABLED */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
int32_t EnvSensors_Read(sensor_t *sensor_data)
{
  /* USER CODE BEGIN EnvSensors_Read */
  float HUMIDITY_Value = HUMIDITY_DEFAULT_VAL;
  float TEMPERATURE_Value = TEMPERATURE_DEFAULT_VAL;
  float PRESSURE_Value = PRESSURE_DEFAULT_VAL;

  /* GNSS processing removed - module is powered off to prevent LoRaWAN interference */
  /* Re-enable when GNSS power management is coordinated with LoRaWAN timing */
  /* GNSS_ProcessDMABuffer(&hgnss); */

#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
  /* Try to read real sensor values */
  int32_t sht_temp_scaled, sht_hum_scaled;
  float ms_temp, ms_press;
  
  /* Read SHT31 sensor */
  if (SHT31_ReadTempAndHumidity(&hsht31, &sht_temp_scaled, &sht_hum_scaled) == SHT31_OK) {
    TEMPERATURE_Value = sht_temp_scaled / 100.0f;  /* Convert from scaled to float */
    HUMIDITY_Value = sht_hum_scaled / 100.0f;      /* Convert from scaled to float */
    /* Print using integers (no float printf support needed) */
    SEGGER_RTT_printf(0, "SHT31: T=%d.%d°C, H=%d.%d%%\r\n", 
                      sht_temp_scaled / 100, (sht_temp_scaled % 100) / 10,
                      sht_hum_scaled / 100, (sht_hum_scaled % 100) / 10);
  } else {
    SEGGER_RTT_WriteString(0, "SHT31 read failed, using defaults\r\n");
  }
  
  /* Read MS5607 sensor */
  if (MS5607_ReadPressureAndTemperature(&hms5607, &ms_temp, &ms_press) == MS5607_OK) {
    PRESSURE_Value = ms_press;
    /* Use MS5607 temperature as backup/verification */
    int press_int = (int)(ms_press * 10);
    int temp_int = (int)(ms_temp * 10);
    SEGGER_RTT_printf(0, "MS5607: P=%d.%d hPa, T=%d.%d°C\r\n", 
                      press_int / 10, press_int % 10,
                      temp_int / 10, temp_int % 10);
  } else {
    SEGGER_RTT_WriteString(0, "MS5607 read failed, using defaults\r\n");
  }
#else
  SEGGER_RTT_WriteString(0, "Sensors disabled, using default values\r\n");
#endif

  /* Quick LED flash to show we're running */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /* Set sensor data */
  sensor_data->humidity    = HUMIDITY_Value;
  sensor_data->temperature = TEMPERATURE_Value;
  sensor_data->pressure    = PRESSURE_Value;
  
  /* Read battery voltage from ADC (PB4 with voltage divider) */
  sensor_data->battery_voltage = SYS_GetBatteryVoltage() / 1000.0f;  /* Convert mV to V */
  
  /* Read regulator voltage (VDDA/3.3V rail) from internal reference */
  sensor_data->regulator_voltage = SYS_GetBatteryLevel() / 1000.0f;  /* Convert mV to V */
  
  int batt_mv = SYS_GetBatteryVoltage();
  int vdda_mv = SYS_GetBatteryLevel();  /* VDDA rail (internal 3.3V reference) */
  SEGGER_RTT_printf(0, "Battery: %d.%02d V (%d mV) | VDDA: %d.%02d V (%d mV)\r\n", 
                    batt_mv / 1000, (batt_mv % 1000) / 10, batt_mv,
                    vdda_mv / 1000, (vdda_mv % 1000) / 10, vdda_mv);
  
 
  /* Use real GNSS data if available from hgnss (populated by SendTxData's GPS collection)
   * GPS is powered on/off in SendTxData before calling EnvSensors_Read
   * hgnss.data contains the latest parsed NMEA data */
  
  if (hgnss.is_initialized && hgnss.data.valid && 
      hgnss.data.fix_quality != GNSS_FIX_INVALID &&
      GNSS_ValidateCoordinates(hgnss.data.latitude, hgnss.data.longitude))
  {

    /* Convert decimal degrees to scaled integer format for Cayenne LPP */
    sensor_data->latitude = (int32_t)((hgnss.data.latitude * MAX_GPS_POS) / 90.0f);
    sensor_data->longitude = (int32_t)((hgnss.data.longitude * MAX_GPS_POS) / 180.0f);
    sensor_data->altitudeGps = (int16_t)hgnss.data.altitude;
    sensor_data->satellites = hgnss.data.satellites;
    sensor_data->gnss_fix_quality = hgnss.data.fix_quality;
    sensor_data->gnss_hdop = hgnss.data.hdop;
    sensor_data->gnss_valid = true;
    
    SEGGER_RTT_printf(0, "GNSS: Valid fix | Sats:%d\r\n", hgnss.data.satellites);
  }
  else
  {
    /* No valid GPS fix - use default coordinates */
    sensor_data->latitude = (int32_t)((STSOP_LATTITUDE * MAX_GPS_POS) / 90);
    sensor_data->longitude = (int32_t)((STSOP_LONGITUDE * MAX_GPS_POS) / 180);
    sensor_data->altitudeGps = 0;
    sensor_data->satellites = hgnss.data.satellites_in_view;  /* Show satellites in view even without fix */
    sensor_data->gnss_fix_quality = 0;
    sensor_data->gnss_hdop = 99.9f;
    sensor_data->gnss_valid = false;
    
    SEGGER_RTT_printf(0, "GNSS: No fix | Sats visible:%d | Using default coords\r\n",
                      hgnss.data.satellites_in_view);
  }

  return 0;
  /* USER CODE END EnvSensors_Read */
}

int32_t EnvSensors_Init(void)
{
  int32_t ret = 0;
  /* USER CODE BEGIN EnvSensors_Init */
  
  SEGGER_RTT_WriteString(0, "EnvSensors_Init: Starting I2C sensor initialization...\r\n");
  
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
  /* Initialize SHT31 sensor handle */
  hsht31.hi2c = &hi2c2;
  hsht31.Address = SHT31_I2C_ADDRESS_B;  /* Use 0x45 - hardware has ADDR pin HIGH */
  hsht31.Mode = SHT31_MODE_HIGH_PRECISION;
  
  /* Initialize MS5607 sensor handle */
  hms5607.hi2c = &hi2c2;
  hms5607.Address = MS5607_I2C_ADDRESS_B;  /* Use 0x77 - hardware has CSB pin LOW */
  hms5607.PressureOsr = MS5607_OSR_4096;
  hms5607.TemperatureOsr = MS5607_OSR_4096;
  
  /* Initialize SHT31 sensor */
  if (SHT31_Init(&hsht31) == SHT31_OK) {
    SEGGER_RTT_WriteString(0, "SHT31 sensor initialized successfully\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "SHT31 sensor initialization failed\r\n");
    ret = -1;
  }
  
  /* Initialize MS5607 sensor */
  if (MS5607_Init(&hms5607) == MS5607_OK) {
    SEGGER_RTT_WriteString(0, "MS5607 sensor initialized successfully\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "MS5607 sensor initialization failed\r\n");
    ret = -2;
  }
#else
  SEGGER_RTT_WriteString(0, "Sensors disabled in configuration\r\n");
#endif
  
  /* Initialize GNSS module handle */
  SEGGER_RTT_WriteString(0, "Initializing GNSS module...\r\n");
  hgnss.huart = &huart1;
  hgnss.pwr_port = GPIOB;
  hgnss.pwr_pin = GPIO_PIN_10;
  hgnss.en_port = GPIOB;
  hgnss.en_pin = GPIO_PIN_5;
  
  /* Initialize GNSS module (configures GPIO, sets up structure - does NOT power on) */
  if (GNSS_Init(&hgnss) == GNSS_OK) {
    SEGGER_RTT_WriteString(0, "GNSS module initialized successfully\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "GNSS module initialization FAILED\r\n");
    ret = -3;
  }
  
  /* GNSS power-on DISABLED during init to prevent interference with LoRaWAN join */
  /* GPS will be powered on/off by SendTxData() callback after join succeeds */
  SEGGER_RTT_WriteString(0, "GNSS init complete - will be powered on during transmissions\r\n");
  
  SEGGER_RTT_WriteString(0, "EnvSensors_Init: Initialization complete\r\n");
  /* USER CODE END EnvSensors_Init */
  return ret;
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
