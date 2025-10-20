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
#include "string.h"
#include "platform.h"
#include "sys_conf.h"
#include "sys_sensors.h"
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 0)
#include "adc_if.h"
#endif /* SENSOR_ENABLED */

/* USER CODE BEGIN Includes */
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
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
/* SHT31 sensor handle */
SHT31_HandleTypeDef hsht31;

/* MS5607 sensor handle */
MS5607_HandleTypeDef hms5607;

/* I2C handle - declared in main.c but we need to access it here */
extern I2C_HandleTypeDef hi2c2;

/* Function to scan I2C bus for devices with enhanced debugging */
static void ScanI2CBus(I2C_HandleTypeDef *hi2c)
{
  printf("Scanning I2C bus for devices with enhanced debugging...\r\n");
  
  uint8_t devices_found = 0;
  HAL_StatusTypeDef status;
  
  /* Try all possible 7-bit addresses with more detailed debugging */
  for (uint8_t address = 1; address < 128; address++)
  {
    /* Add a delay between scans to give the I2C bus time to recover */
    HAL_Delay(5);
    
    printf("Trying address 0x%02X: ", address);
    
    /* Try multiple times for each address */
    for (uint8_t retry = 0; retry < 3; retry++)
    {
      /* The HAL_I2C_IsDeviceReady function expects the address to be shifted left by 1 */
      /* This is because the LSB is used for read/write bit */
      status = HAL_I2C_IsDeviceReady(hi2c, (address << 1), 1, 100);
      
      if (status == HAL_OK)
      {
        printf("FOUND on retry %d!\r\n", retry);
        
        /* Print additional debug info */
        printf("  - Device found at address 0x%02X (7-bit address)\r\n", address);
        printf("  - This corresponds to 0x%02X (8-bit address) for write operations\r\n", (address << 1));
        printf("  - This corresponds to 0x%02X (8-bit address) for read operations\r\n", (address << 1) | 0x01);
        
        /* Blink LED to indicate device found */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        
        devices_found++;
        break; /* Device found, break out of retry loop */
      }
      
      /* Small delay between retries */
      HAL_Delay(5);
    }
    
    if (status != HAL_OK)
    {
      printf("not found\r\n");
    }
  }
  
  if (devices_found == 0)
  {
    printf("No I2C devices found on the bus\r\n");
    printf("Possible issues:\r\n");
    printf("1. Check physical connections (SDA, SCL, VDD, GND)\r\n");
    printf("2. Verify pull-up resistors on SDA and SCL lines\r\n");
    printf("3. Ensure sensor has proper power supply\r\n");
    printf("4. Try different I2C timing settings\r\n");
    
    /* Blink LED rapidly to indicate no devices found */
    for (int i = 0; i < 5; i++)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_Delay(50);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(50);
    }
  }
  else
  {
    printf("Total I2C devices found: %d\r\n", devices_found);
  }
}

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

#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
  /* Read from SHT31 sensor */
  int32_t temp, hum;
  SHT31_StatusTypeDef sht31_status;
  MS5607_StatusTypeDef ms5607_status;
  uint8_t retry_count = 0;
  const uint8_t MAX_RETRIES = 3;
  
  /* Read temperature and humidity from SHT31 */
  sht31_status = SHT31_ReadTempAndHumidity(&hsht31, &temp, &hum);
  if (sht31_status == SHT31_OK) {
    TEMPERATURE_Value = (float)temp / 100.0f;
    HUMIDITY_Value = (float)hum / 100.0f;
  }
  
  /* MS5607: Read PROM, temperature, and pressure every cycle */
  {
    float ms5607_temp, ms5607_press;
    
    /* Ensure handle is properly configured (in case it wasn't set during init) */
    hms5607.hi2c = &hi2c2;
    hms5607.Address = 0x77;
    hms5607.PressureOsr = MS5607_OSR_4096;
    hms5607.TemperatureOsr = MS5607_OSR_4096;
    
    /* Read calibration data from PROM */
    ms5607_status = MS5607_ReadCalibration(&hms5607);
    
    if (ms5607_status == MS5607_OK) {
      /* Set initialized flag so read function works */
      hms5607.IsInitialized = 1;
      
      /* Read pressure and temperature */
      ms5607_status = MS5607_ReadPressureAndTemperature(&hms5607, &ms5607_temp, &ms5607_press);
      
      if (ms5607_status == MS5607_OK) {
        PRESSURE_Value = ms5607_press;
      } else {
        PRESSURE_Value = PRESSURE_DEFAULT_VAL;
      }
    } else {
      PRESSURE_Value = PRESSURE_DEFAULT_VAL;
    }
  }
  
#if (USE_IKS01A2_ENV_SENSOR_HTS221_0 == 1)
  IKS01A2_ENV_SENSOR_GetValue(HTS221_0, ENV_HUMIDITY, &HUMIDITY_Value);
  IKS01A2_ENV_SENSOR_GetValue(HTS221_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
#endif /* USE_IKS01A2_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A2_ENV_SENSOR_LPS22HB_0 == 1)
  IKS01A2_ENV_SENSOR_GetValue(LPS22HB_0, ENV_PRESSURE, &PRESSURE_Value);
  IKS01A2_ENV_SENSOR_GetValue(LPS22HB_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
#endif /* USE_IKS01A2_ENV_SENSOR_LPS22HB_0 */
#if (USE_IKS01A3_ENV_SENSOR_HTS221_0 == 1)
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_HUMIDITY, &HUMIDITY_Value);
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
#endif /* USE_IKS01A3_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A3_ENV_SENSOR_LPS22HH_0 == 1)
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_PRESSURE, &PRESSURE_Value);
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
#endif /* USE_IKS01A3_ENV_SENSOR_LPS22HH_0 */
#else
  TEMPERATURE_Value = (SYS_GetTemperatureLevel() >> 8);
#endif /* SENSOR_ENABLED */

  sensor_data->humidity    = HUMIDITY_Value;
  sensor_data->temperature = TEMPERATURE_Value;
  sensor_data->pressure    = PRESSURE_Value;
  sensor_data->latitude    = (int32_t)((STSOP_LATTITUDE  * MAX_GPS_POS) / 90);
  sensor_data->longitude   = (int32_t)((STSOP_LONGITUDE * MAX_GPS_POS) / 180);

  return 0;
  /* USER CODE END EnvSensors_Read */
}

int32_t EnvSensors_Init(void)
{
  int32_t ret = 0;
  /* USER CODE BEGIN EnvSensors_Init */
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
  ScanI2CBus(&hi2c2);
  
  /* Try both possible I2C addresses for the SHT31 sensor */
  /* First try address 0x44 (ADDR pin pulled low) - this is the most common configuration */
  hsht31.hi2c = &hi2c2;
  hsht31.Address = SHT31_I2C_ADDRESS_A; /* Address when ADDR is pulled low (0x44) */
  hsht31.Mode = SHT31_MODE_HIGH_PRECISION;
  
  /* Print debug info */
  printf("Initializing SHT31 with address 0x%02X\r\n", hsht31.Address);
  
  /* Initialize the SHT31 sensor with LED debugging */
  ret = SHT31_Init(&hsht31);
  
  /* If initialization fails with address 0x44, try address 0x45 */
  if (ret != SHT31_OK)
  {
    printf("SHT31 initialization failed with address 0x44, trying address 0x45...\r\n");
    
    /* Blink LED to indicate trying alternate address */
    for (int i = 0; i < 2; i++)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_Delay(200);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(200);
    }
    
    /* Try with address 0x45 (ADDR pin pulled high) */
    hsht31.Address = SHT31_I2C_ADDRESS_B; /* Address when ADDR is pulled high (0x45) */
    ret = SHT31_Init(&hsht31);
    
    if (ret == SHT31_OK)
    {
      printf("SHT31 initialization successful with address 0x45\r\n");
    }
  }
  else
  {
    printf("SHT31 initialization successful with address 0x44\r\n");
  }
  
  /* If initialization fails, blink the LED on PA0 to indicate error */
  if (ret != SHT31_OK)
  {
    /* Blink LED on PA0 to indicate SHT31 initialization error */
    for (int i = 0; i < 5; i++)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_Delay(200);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(200);
    }
    
    /* Continue anyway to avoid system crash, but log the error */
    printf("SHT31 initialization failed with error code: %d\r\n", ret);
    /* Don't call Error_Handler() to avoid system crash */
  }
  else
  {
    /* Blink LED once to indicate successful initialization */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    
    printf("SHT31 initialized successfully\r\n");
  }
  
  /* Initialize MS5607 pressure sensor */
  /* Note: Handle will be re-initialized on every read cycle for reliability */
  memset(&hms5607, 0, sizeof(hms5607));
  hms5607.hi2c = &hi2c2;
  hms5607.Address = 0x77;  /* CSB pin is LOW */
  hms5607.PressureOsr = MS5607_OSR_4096;
  hms5607.TemperatureOsr = MS5607_OSR_4096;
  
  /* Init */
#if (USE_IKS01A2_ENV_SENSOR_HTS221_0 == 1)
  ret = IKS01A2_ENV_SENSOR_Init(HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A2_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A2_ENV_SENSOR_LPS22HB_0 == 1)
  ret = IKS01A2_ENV_SENSOR_Init(LPS22HB_0, ENV_TEMPERATURE | ENV_PRESSURE);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A2_ENV_SENSOR_LPS22HB_0 */
#if (USE_IKS01A3_ENV_SENSOR_HTS221_0 == 1)
  ret = IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A3_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A3_ENV_SENSOR_LPS22HH_0 == 1)
  ret = IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_TEMPERATURE | ENV_PRESSURE);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A3_ENV_SENSOR_LPS22HH_0 */

  /* Enable */
#if (USE_IKS01A2_ENV_SENSOR_HTS221_0 == 1)
  ret = IKS01A2_ENV_SENSOR_Enable(HTS221_0, ENV_HUMIDITY);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  ret = IKS01A2_ENV_SENSOR_Enable(HTS221_0, ENV_TEMPERATURE);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A2_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A2_ENV_SENSOR_LPS22HB_0 == 1)
  ret = IKS01A2_ENV_SENSOR_Enable(LPS22HB_0, ENV_PRESSURE);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  ret = IKS01A2_ENV_SENSOR_Enable(LPS22HB_0, ENV_TEMPERATURE);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A2_ENV_SENSOR_LPS22HB_0 */
#if (USE_IKS01A3_ENV_SENSOR_HTS221_0 == 1)
  ret = IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_HUMIDITY);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  ret = IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_TEMPERATURE);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A3_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A3_ENV_SENSOR_LPS22HH_0 == 1)
  ret = IKS01A3_ENV_SENSOR_Enable(IKS01A3_LPS22HH_0, ENV_PRESSURE);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  ret = IKS01A3_ENV_SENSOR_Enable(IKS01A3_LPS22HH_0, ENV_TEMPERATURE);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A3_ENV_SENSOR_LPS22HH_0 */

  /* Get capabilities */
#if (USE_IKS01A2_ENV_SENSOR_HTS221_0 == 1)
  ret = IKS01A2_ENV_SENSOR_GetCapabilities(HTS221_0, &EnvCapabilities);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A2_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A2_ENV_SENSOR_LPS22HB_0 == 1)
  ret = IKS01A2_ENV_SENSOR_GetCapabilities(LPS22HB_0, &EnvCapabilities);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A2_ENV_SENSOR_LPS22HB_0 */
#if (USE_IKS01A3_ENV_SENSOR_HTS221_0 == 1)
  ret = IKS01A3_ENV_SENSOR_GetCapabilities(IKS01A3_HTS221_0, &EnvCapabilities);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A3_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A3_ENV_SENSOR_LPS22HH_0 == 1)
  ret = IKS01A3_ENV_SENSOR_GetCapabilities(IKS01A3_LPS22HH_0, &EnvCapabilities);
  if (ret != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
#endif /* USE_IKS01A3_ENV_SENSOR_LPS22HH_0 */

#elif !defined (SENSOR_ENABLED)
#error SENSOR_ENABLED not defined
#endif /* SENSOR_ENABLED  */
  /* USER CODE END EnvSensors_Init */
  return ret;
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
