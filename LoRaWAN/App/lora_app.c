/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
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
#include "platform.h"
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "SEGGER_RTT.h"
#include "atgm336h.h"
#include "sys_conf.h"
#include "multiregion_h3.h"
#include "multiregion_context.h"
#include "LoRaMac.h"
#include "se-identity.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern GNSS_HandleTypeDef hgnss;  /* GNSS handle from sys_sensors.c */
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * LEDs period value of the timer in ms
  */
#define LED_PERIOD_TIME 500

/**
  * Join switch period value of the timer in ms
  */
#define JOIN_TIME 2000

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief callback when LoRaWAN application has sent a frame
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRaWAN application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
  * @brief callback when LoRaWAN Beacon status is updated
  * @param params status of Last Beacon
  */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
  * @brief callback when system time has been updated
  */
static void OnSysTimeUpdate(void);

/**
  * @brief callback when LoRaWAN application Class is changed
  * @param deviceClass new class
  */
static void OnClassChange(DeviceClass_t deviceClass);

/**
  * @brief  LoRa store context in Non Volatile Memory
  */
static void StoreContext(void);

/**
  * @brief  stop current LoRa execution to switch into non default Activation mode
  */
static void StopJoin(void);

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */
static void OnStopJoinTimerEvent(void *context);

/**
  * @brief  Notifies the upper layer that the NVM context has changed
  * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
  */
static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
  * @brief  Store the NVM Data context to the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were stored
  */
static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * @brief  Restore the NVM Data context from the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were restored
  */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * Will be called each time a Radio IRQ is handled by the MAC layer
  *
  */
static void OnMacProcessNotify(void);

/**
  * @brief Change the periodicity of the uplink frames
  * @param periodicity uplink frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnTxPeriodicityChanged(uint32_t periodicity);

/**
  * @brief Change the confirmation control of the uplink frames
  * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
  * @note Compliance test protocol callbacks
  */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
  * @brief Change the periodicity of the ping slot frames
  * @param pingSlotPeriodicity ping slot frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/**
  * @brief Will be called to reset the system
  * @note Compliance test protocol callbacks
  */
static void OnSystemReset(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV_MULTIREGION */
/* Multi-region ABP support - no join flags needed */
/* USER CODE END PV_MULTIREGION */

/**
  * @brief LoRaWAN default activation type
  */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN force rejoin even if the NVM context is restored
  */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =              GetBatteryLevel,
  .GetTemperature =               GetTemperatureLevel,
  .GetUniqueId =                  GetUniqueId,
  .GetDevAddr =                   GetDevAddr,
  .OnRestoreContextRequest =      OnRestoreContextRequest,
  .OnStoreContextRequest =        OnStoreContextRequest,
  .OnMacProcess =                 OnMacProcessNotify,
  .OnNvmDataChange =              OnNvmDataChange,
  .OnJoinRequest =                OnJoinRequest,
  .OnTxData =                     OnTxData,
  .OnRxData =                     OnRxData,
  .OnBeaconStatusChange =         OnBeaconStatusChange,
  .OnSysTimeUpdate =              OnSysTimeUpdate,
  .OnClassChange =                OnClassChange,
  .OnTxPeriodicityChanged =       OnTxPeriodicityChanged,
  .OnTxFrameCtrlChanged =         OnTxFrameCtrlChanged,
  .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
  .OnSystemReset =                OnSystemReset,
};

/**
  * @brief LoRaWAN handler parameters
  * @note Made non-static to allow access from multiregion_context.c
  */
LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .TxPower =                  LORAWAN_DEFAULT_TX_POWER,
  .PingSlotPeriodicity =      LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
  .RxBCTimeout =              LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT
};

/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_TIMER;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/**
  * @brief Tx Timer period
  */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
  * @brief Join Timer period
  */
static UTIL_TIMER_Object_t StopJoinTimer;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/**
 * @brief Reinitialize the entire LoRaWAN stack for a new region
 * @note This performs DeInit/Init cycle to clear ALL state between region joins
 * @note Caller must set DevEUI and call LmHandlerConfigure() after this returns
 */
void LoRaApp_ReInitStack(LoRaMacRegion_t new_region)
{
  SEGGER_RTT_WriteString(0, "LoRaApp_ReInitStack: Starting full stack reset...\r\n");
  
  // Halt current operations
  LmHandlerHalt();
  HAL_Delay(100);
  
  // Complete teardown
  SEGGER_RTT_WriteString(0, "LoRaApp_ReInitStack: Calling LmHandlerDeInit...\r\n");
  LmHandlerDeInit();
  HAL_Delay(200);
  
  // Rebuild from scratch
  SEGGER_RTT_WriteString(0, "LoRaApp_ReInitStack: Calling LmHandlerInit...\r\n");
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);
  HAL_Delay(100);
  
  // Set region parameter but DON'T configure yet
  // Configuration must be done AFTER DevEUI is set by caller
  LmHandlerParams.ActiveRegion = new_region;
  
  SEGGER_RTT_WriteString(0, "LoRaApp_ReInitStack: Stack reset complete (region set, not configured)\r\n");
}

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_LV */

  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */

  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT, OnStopJoinTimerEvent, NULL);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU, StopJoin);

  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
#if MULTIREGION_ABP_MODE
  /* ========== MULTI-REGION ABP MODE ========== */
  /* Initialize multi-region ABP support */
  MultiRegion_Init();
  APP_LOG(TS_ON, VLEVEL_H, "Multi-region ABP initialized\r\n");
  
  /* Load US915 ABP credentials and activate */
  SEGGER_RTT_WriteString(0, "Loading US915 ABP credentials...\r\n");
  MultiRegion_SwitchToRegion(LORAMAC_REGION_US915);
  APP_LOG(TS_ON, VLEVEL_H, "US915 ABP activated\r\n");
  
  SEGGER_RTT_WriteString(0, "Multi-region ABP mode - no join needed, starting TX timer\r\n");
#else
  /* ========== TRADITIONAL ABP MODE ========== */
  SEGGER_RTT_WriteString(0, "Traditional ABP mode - US915\r\n");
  
  // Set DevEUI from se-identity.h (US915 - use the _US915 variant which has 0x prefix)
  uint8_t devEui[] = {LORAWAN_DEVICE_EUI_US915};
  LmHandlerSetDevEUI(devEui);
  SEGGER_RTT_WriteString(0, "DevEUI set from se-identity.h\r\n");
  
  // Get NVM context and load ABP keys
  MibRequestConfirm_t mib;
  mib.Type = MIB_NVM_CTXS;
  LoRaMacMibGetRequestConfirm(&mib);
  LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t*)mib.Param.Contexts;
  
  if (nvm) {
    // Load keys from se-identity.h (formatted)
    uint8_t appSKey[] = FORMAT_KEY(LORAWAN_APP_S_KEY);
    uint8_t nwkSKey[] = FORMAT_KEY(LORAWAN_NWK_S_KEY);
    
    // Set ABP session keys (LoRaWAN 1.0.x uses simpler key structure)
    // For 1.0.x: APP_S_KEY and NWK_S_KEY are the only session keys
    memcpy(nvm->SecureElement.KeyList[APP_S_KEY].KeyValue, appSKey, 16);
    memcpy(nvm->SecureElement.KeyList[NWK_S_KEY].KeyValue, nwkSKey, 16);
    
    // Set DevAddr for US915 from Chirpstack
    nvm->MacGroup2.DevAddr = 0x26091500;  // US915 DevAddr from Chirpstack_ABP_Values.txt
    
    // Reset frame counters for fresh ABP start
    nvm->Crypto.FCntList.FCntUp = 0;
    nvm->Crypto.FCntList.NFCntDown = 0;
    
    // Mark as ABP activated
    nvm->MacGroup2.NetworkActivation = ACTIVATION_TYPE_ABP;
    
    // Debug output - print all ABP credentials
    SEGGER_RTT_printf(0, "\r\n=== ABP Credentials Loaded ===\r\n");
    SEGGER_RTT_printf(0, "DevAddr: 0x%08lX\r\n", nvm->MacGroup2.DevAddr);
    
    // Print AppSKey
    SEGGER_RTT_WriteString(0, "AppSKey: ");
    for (int i = 0; i < 16; i++) {
      SEGGER_RTT_printf(0, "%02X", appSKey[i]);
    }
    SEGGER_RTT_WriteString(0, "\r\n");
    
    // Print NwkSKey
    SEGGER_RTT_WriteString(0, "NwkSKey: ");
    for (int i = 0; i < 16; i++) {
      SEGGER_RTT_printf(0, "%02X", nwkSKey[i]);
    }
    SEGGER_RTT_WriteString(0, "\r\n");
    
    SEGGER_RTT_WriteString(0, "Compare these with your Chirpstack config!\r\n");
    SEGGER_RTT_WriteString(0, "================================\r\n\r\n");
  }
  
  // Set activation via MIB
  mib.Type = MIB_NETWORK_ACTIVATION;
  mib.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
  LoRaMacMibSetRequestConfirm(&mib);
  
  // Start MAC and allow stabilization
  LoRaMacStart();
  HAL_Delay(200);
  
  // Process MAC events to complete initialization
  for (int i = 0; i < 10; i++) {
    LmHandlerProcess();
    HAL_Delay(10);
  }
  
  SEGGER_RTT_WriteString(0, "Traditional ABP credentials loaded and MAC started\r\n");
  APP_LOG(TS_ON, VLEVEL_H, "Traditional ABP mode - US915 activated\r\n");
#endif
  /* USER CODE END LoRaWAN_Init_2 */

  // ABP doesn't need join - skip it for both modes
  // LmHandlerJoin(ActivationType, ForceRejoin);

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
  /* USER CODE END OnRxData_1 */
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
  
  SEGGER_RTT_WriteString(0, "\r\n=== SendTxData START ===\r\n");

  /* Non-blocking GNSS collection - power on briefly, capture what's available
   * GPS module needs time to get a fix (cold start ~30s, warm ~5s, hot ~1s)
   * We power on for longer on first attempts to allow cold/warm fix
   * This keeps the GNSS operation bounded and doesn't block LoRaWAN timing
   */
  #define GNSS_COLLECTION_TIME_MS  120000  /* 45 seconds - allows cold start fix */
  #define GNSS_MIN_SATS_FOR_FIX    4      /* Minimum satellites needed for fix */
  
  /* Declare gps_start and ttf_ms at function scope */
  uint32_t gps_start = 0;
  uint32_t ttf_ms = 0;  /* Time to fix in milliseconds - captured when fix obtained */
  
  SEGGER_RTT_WriteString(0, "Powering GPS ON for fix acquisition (45s max)...\r\n");
  if (GNSS_PowerOn(&hgnss) == GNSS_OK)
  {
    /* CRITICAL: Invalidate old GPS data to force waiting for fresh NMEA sentences */
    /* This prevents reusing data from previous cycle (which would give false 0ms TTF) */
    hgnss.data.valid = false;
    hgnss.data.fix_quality = GNSS_FIX_INVALID;
    SEGGER_RTT_WriteString(0, "GPS data invalidated - waiting for fresh fix...\r\n");
    
    gps_start = HAL_GetTick();
    bool got_fix = false;
    uint32_t last_status_print = 0;
    
    /* Process GPS data for up to GNSS_COLLECTION_TIME_MS */
    while ((HAL_GetTick() - gps_start) < GNSS_COLLECTION_TIME_MS)
    {
      /* Process DMA buffer - parses NMEA and updates hgnss.data */
      GNSS_ProcessDMABuffer(&hgnss);
      
      /* Check if we have a good quality fix */
      if (GNSS_IsFixGoodQuality(&hgnss))
      {
        got_fix = true;
        ttf_ms = HAL_GetTick() - gps_start;  /* Capture TTF at moment of fix */
        
        /* Convert floats to integers for safe printf (no float support needed) */
        int32_t lat_int = (int32_t)(hgnss.data.latitude * 1000000);
        int32_t lon_int = (int32_t)(hgnss.data.longitude * 1000000);
        int32_t alt_int = (int32_t)(hgnss.data.altitude * 10);
        int32_t hdop_int = (int32_t)(hgnss.data.hdop * 10);
        
        char fix_msg[150];
        snprintf(fix_msg, sizeof(fix_msg), 
                 "GPS FIX! Lat=%d.%06d Lon=%d.%06d Alt=%d.%dm Sats:%d HDOP=%d.%d (took %lums)\r\n",
                 lat_int / 1000000, abs(lat_int % 1000000),
                 lon_int / 1000000, abs(lon_int % 1000000),
                 alt_int / 10, abs(alt_int % 10),
                 hgnss.data.satellites,
                 hdop_int / 10, abs(hdop_int % 10),
                 (unsigned long)ttf_ms);
        SEGGER_RTT_WriteString(0, fix_msg);
        break;  /* Exit early - we have what we need */
      }
      
      /* Print status every 5 seconds during acquisition */
      uint32_t elapsed = HAL_GetTick() - gps_start;
      if (elapsed - last_status_print >= 5000)
      {
        char status_msg[100];
        snprintf(status_msg, sizeof(status_msg), 
                 "[GPS %lus] Sats:%d/%d HDOP:%.1f Fix:%s\r\n",
                 (unsigned long)(elapsed / 1000),
                 hgnss.data.satellites, hgnss.data.satellites_in_view,
                 hgnss.data.hdop,
                 (hgnss.data.fix_quality != GNSS_FIX_INVALID) ? "Yes" : "No");
        SEGGER_RTT_WriteString(0, status_msg);
        last_status_print = elapsed;
      }
      
      /* Small delay to prevent tight loop, allow other processing */
      HAL_Delay(100);
    }
    
    if (!got_fix)
    {
      /* Check if we have at least basic fix */
      if (GNSS_IsFixValid(&hgnss))
      {
        SEGGER_RTT_WriteString(0, "GPS: Basic fix (not high quality)\r\n");
      }
      else
      {
        SEGGER_RTT_WriteString(0, "GPS: No fix - using defaults\r\n");
      }
    }
    
    /* Power off GPS before LoRaWAN TX to avoid timing conflicts */
    GNSS_PowerOff(&hgnss);
    SEGGER_RTT_WriteString(0, "GPS powered OFF\r\n");
    
    /* Perform H3lite region lookup if we have a valid fix */
    if (GNSS_IsFixValid(&hgnss) && 
        GNSS_ValidateCoordinates(hgnss.data.latitude, hgnss.data.longitude))
    {
      /* Start timing for H3 region lookup */
      uint32_t h3_start = HAL_GetTick();
      
      /* Call H3lite region lookup */
      LoRaMacRegion_t detected_region = MultiRegion_DetectFromGPS_H3(
          hgnss.data.latitude, 
          hgnss.data.longitude
      );
      
      /* Calculate elapsed time for H3 lookup */
      uint32_t h3_elapsed = HAL_GetTick() - h3_start;
      
      /* Convert floats to integers for printing (safe for all printf implementations) */
      int32_t lat_int = (int32_t)(hgnss.data.latitude * 1000000);  // 6 decimal places
      int32_t lon_int = (int32_t)(hgnss.data.longitude * 1000000); // 6 decimal places
      
      /* Map region enum to string name */
      const char* region_name = "UNKNOWN";
      switch(detected_region) {
        case LORAMAC_REGION_US915: region_name = "US915"; break;
        case LORAMAC_REGION_EU868: region_name = "EU868"; break;
        case LORAMAC_REGION_AS923: region_name = "AS923"; break;
        case LORAMAC_REGION_AU915: region_name = "AU915"; break;
        case LORAMAC_REGION_CN470: region_name = "CN470"; break;
        case LORAMAC_REGION_KR920: region_name = "KR920"; break;
        case LORAMAC_REGION_IN865: region_name = "IN865"; break;
        case LORAMAC_REGION_RU864: region_name = "RU864"; break;
        default: region_name = "UNKNOWN"; break;
      }
      
      /* Output region lookup result to RTT with timing - use snprintf to avoid buffer issues */
      char h3_msg[200];
      snprintf(h3_msg, sizeof(h3_msg), 
               "H3 Region Lookup: Lat=%d.%06d Lon=%d.%06d -> %s (took %lums)\r\n",
               lat_int / 1000000, abs(lat_int % 1000000),
               lon_int / 1000000, abs(lon_int % 1000000),
               region_name, (unsigned long)h3_elapsed);
      SEGGER_RTT_WriteString(0, h3_msg);
      
      /* Auto-switch region if enabled and necessary */
      LmHandlerErrorStatus_t switch_status = MultiRegion_AutoSwitchForLocation(
          hgnss.data.latitude, 
          hgnss.data.longitude
      );
      
      if (switch_status == LORAMAC_HANDLER_SUCCESS) {
        SEGGER_RTT_WriteString(0, "MultiRegion: Auto-switch completed successfully\r\n");
      } else if (switch_status == LORAMAC_HANDLER_BUSY_ERROR) {
        SEGGER_RTT_WriteString(0, "MultiRegion: Switch deferred (MAC busy)\r\n");
      }
    }
    else
    {
      SEGGER_RTT_WriteString(0, "H3 Region Lookup: Skipped (no valid GPS fix)\r\n");
    }
  }
  else
  {
    SEGGER_RTT_WriteString(0, "GPS: Power on failed!\r\n");
  }

  /* Add separator before sensor read to prevent RTT buffer overwrite */
  SEGGER_RTT_WriteString(0, "\r\n");
  
  // Get sensor data
  sensor_t sensor_data;
  SEGGER_RTT_WriteString(0, "Calling EnvSensors_Read...\r\n");
  EnvSensors_Read(&sensor_data);
  
  SEGGER_RTT_WriteString(0, "Sensor data read\r\n");
  
  // Initialize Cayenne LPP payload
  CayenneLppReset();
  SEGGER_RTT_WriteString(0, "CayenneLpp reset\r\n");
  
  // Add temperature data (channel 1)
  CayenneLppAddTemperature(1, sensor_data.temperature);
  
  // Add humidity data (channel 2)
  CayenneLppAddRelativeHumidity(2, sensor_data.humidity);
  
  // Add pressure data (channel 3)
  CayenneLppAddBarometricPressure(3, sensor_data.pressure);
  
  // Add GPS data (channel 4) - use zeros if GNSS fix is invalid
  float lat, lon, alt;
  if (sensor_data.gnss_valid) {
    // Convert from binary format back to decimal degrees for Cayenne
    lat = (sensor_data.latitude * 90.0f) / 8388607.0f;
    lon = (sensor_data.longitude * 180.0f) / 8388607.0f;
    alt = (float)sensor_data.altitudeGps;
    
    SEGGER_RTT_WriteString(0, "GNSS data valid\r\n");
  } else {
    // Use zeros when no valid GNSS fix
    lat = 0.0f;
    lon = 0.0f;
    alt = 0.0f;
    
    SEGGER_RTT_WriteString(0, "GNSS data invalid\r\n");
  }
  
  CayenneLppAddGps(4, lat, lon, alt);
  
  // Add number of satellites as analog input (channel 5) - value 0-255
  CayenneLppAddAnalogInput(5, (float)sensor_data.satellites);
  
  // Add battery voltage on channel 6 (in volts)
  CayenneLppAddAnalogInput(6, sensor_data.battery_voltage);
  
  // Add regulator voltage (3.3V rail) on channel 7 (in volts)
  CayenneLppAddAnalogInput(7, sensor_data.regulator_voltage);
  
  // Add GNSS HDOP on channel 8 (Horizontal Dilution of Precision)
  CayenneLppAddAnalogInput(8, sensor_data.gnss_hdop);
  
  // Add TTF (Time To Fix) on channel 9 (in seconds, 0.01s resolution)
  // Convert from milliseconds to seconds to avoid int16_t overflow in Cayenne LPP
  // CayenneLpp analog uses int16_t with 0.01 resolution, max value = 327.67
  CayenneLppAddAnalogInput(9, (float)ttf_ms / 1000.0f);

  /* Safe RTT output - use integer conversion for HDOP to avoid float printf issues */
  int hdop_int = (int)(sensor_data.gnss_hdop * 10);
  char lpp_msg[100];
  snprintf(lpp_msg, sizeof(lpp_msg), "Cayenne LPP data prepared (HDOP=%d.%d, TTF=%lums)\r\n", 
           hdop_int / 10, hdop_int % 10, (unsigned long)ttf_ms);
  SEGGER_RTT_WriteString(0, lpp_msg);
  
  // Prepare and send the LoRaWAN packet
  LmHandlerAppData_t appData;
  appData.Port = LORAWAN_USER_APP_PORT;
  appData.BufferSize = CayenneLppGetSize();
  appData.Buffer = CayenneLppGetBuffer();
  
  // DEBUG: Log payload size
  char size_msg[64];
  snprintf(size_msg, sizeof(size_msg), "Payload size: %d bytes\r\n", appData.BufferSize);
  SEGGER_RTT_WriteString(0, size_msg);
  
  // Force data rate before each send (DR2 = 125 bytes max for US915)
  LmHandlerSetTxDatarate(LORAWAN_DEFAULT_DATA_RATE);
  SEGGER_RTT_WriteString(0, "Sending LoRaWAN packet...\r\n");
  
  LmHandlerErrorStatus_t status = LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0);
  
  // DEBUG: Log LmHandlerSend status
  char status_msg[128];
  const char* status_str;
  switch(status) {
    case LORAMAC_HANDLER_SUCCESS: status_str = "SUCCESS"; break;
    case LORAMAC_HANDLER_BUSY_ERROR: status_str = "BUSY_ERROR"; break;
    case LORAMAC_HANDLER_ERROR: status_str = "ERROR"; break;
    case LORAMAC_HANDLER_NO_NETWORK_JOINED: status_str = "NO_NETWORK_JOINED"; break;
    case LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED: status_str = "DUTYCYCLE_RESTRICTED"; break;
    default: status_str = "UNKNOWN"; break;
  }
  snprintf(status_msg, sizeof(status_msg), "LmHandlerSend status: %s (%d)\r\n", status_str, status);
  SEGGER_RTT_WriteString(0, status_msg);
  
  SEGGER_RTT_WriteString(0, "=== SendTxData END ===\r\n");
  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */
  SEGGER_RTT_WriteString(0, "\r\n*** OnTxTimerEvent FIRED ***\r\n");
  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */
  SEGGER_RTT_WriteString(0, "Timer restarted for next cycle\r\n");
  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */

/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  char rtt_buf[128];
  
  SEGGER_RTT_WriteString(0, "\r\n=== OnTxData Callback ===\r\n");
  snprintf(rtt_buf, sizeof(rtt_buf), "  IsMcpsConfirm: %d\r\n", params->IsMcpsConfirm);
  SEGGER_RTT_WriteString(0, rtt_buf);
  
  snprintf(rtt_buf, sizeof(rtt_buf), "  Status: %d\r\n", params->Status);
  SEGGER_RTT_WriteString(0, rtt_buf);
  
  snprintf(rtt_buf, sizeof(rtt_buf), "  Datarate: DR%d, TxPower: %d\r\n", params->Datarate, params->TxPower);
  SEGGER_RTT_WriteString(0, rtt_buf);
  
  snprintf(rtt_buf, sizeof(rtt_buf), "  Channel: %lu, UplinkCounter: %lu\r\n", 
           (unsigned long)params->Channel, (unsigned long)params->UplinkCounter);
  SEGGER_RTT_WriteString(0, rtt_buf);
  
  if (params->IsMcpsConfirm)
  {
    snprintf(rtt_buf, sizeof(rtt_buf), "  AckReceived: %d\r\n", params->AckReceived);
    SEGGER_RTT_WriteString(0, rtt_buf);
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  // ABP mode - join callback not used (keeping for compatibility)
  SEGGER_RTT_WriteString(0, "OnJoinRequest: ABP mode - join not used\r\n");
  /* USER CODE END OnJoinRequest_1 */
}

static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params)
{
  /* USER CODE BEGIN OnBeaconStatusChange_1 */
  /* USER CODE END OnBeaconStatusChange_1 */
}

static void OnSysTimeUpdate(void)
{
  /* USER CODE BEGIN OnSysTimeUpdate_1 */

  /* USER CODE END OnSysTimeUpdate_1 */
}

static void OnClassChange(DeviceClass_t deviceClass)
{
  /* USER CODE BEGIN OnClassChange_1 */
  /* USER CODE END OnClassChange_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

static void OnTxPeriodicityChanged(uint32_t periodicity)
{
  /* USER CODE BEGIN OnTxPeriodicityChanged_1 */

  /* USER CODE END OnTxPeriodicityChanged_1 */
  TxPeriodicity = periodicity;

  if (TxPeriodicity == 0)
  {
    /* Revert to application default periodicity */
    TxPeriodicity = APP_TX_DUTYCYCLE;
  }

  /* Update timer periodicity */
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, TxPeriodicity);
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxPeriodicityChanged_2 */

  /* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
  /* USER CODE BEGIN OnTxFrameCtrlChanged_1 */

  /* USER CODE END OnTxFrameCtrlChanged_1 */
  LmHandlerParams.IsTxConfirmed = isTxConfirmed;
  /* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

  /* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */

  /* USER CODE END OnPingSlotPeriodicityChanged_1 */
  LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

  /* USER CODE END OnPingSlotPeriodicityChanged_2 */
}

static void OnSystemReset(void)
{
  /* USER CODE BEGIN OnSystemReset_1 */

  /* USER CODE END OnSystemReset_1 */
  if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt()) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
  {
    NVIC_SystemReset();
  }
  /* USER CODE BEGIN OnSystemReset_Last */

  /* USER CODE END OnSystemReset_Last */
}

static void StopJoin(void)
{
  /* USER CODE BEGIN StopJoin_1 */

  /* USER CODE END StopJoin_1 */

  UTIL_TIMER_Stop(&TxTimer);

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop())
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
    if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP)
    {
      ActivationType = ACTIVATION_TYPE_OTAA;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
    }
    else
    {
      ActivationType = ACTIVATION_TYPE_ABP;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
    }
    LmHandlerConfigure(&LmHandlerParams);
    LmHandlerJoin(ActivationType, true);
    UTIL_TIMER_Start(&TxTimer);
  }
  UTIL_TIMER_Start(&StopJoinTimer);
  /* USER CODE BEGIN StopJoin_Last */

  /* USER CODE END StopJoin_Last */
}

static void OnStopJoinTimerEvent(void *context)
{
  /* USER CODE BEGIN OnStopJoinTimerEvent_1 */

  /* USER CODE END OnStopJoinTimerEvent_1 */
  if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE)
  {
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
  }
  /* USER CODE BEGIN OnStopJoinTimerEvent_Last */

  /* USER CODE END OnStopJoinTimerEvent_Last */
}

static void StoreContext(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  /* USER CODE BEGIN StoreContext_1 */

  /* USER CODE END StoreContext_1 */
  status = LmHandlerNvmDataStore();

  if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
  }
  else if (status == LORAMAC_HANDLER_ERROR)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
  }
  /* USER CODE BEGIN StoreContext_Last */

  /* USER CODE END StoreContext_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
  /* USER CODE BEGIN OnNvmDataChange_1 */

  /* USER CODE END OnNvmDataChange_1 */
  if (state == LORAMAC_HANDLER_NVM_STORE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
  }
  /* USER CODE BEGIN OnNvmDataChange_Last */

  /* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */

  /* USER CODE END OnStoreContextRequest_1 */
  /* store nvm in flash */
  if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
  {
    FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (const void *)nvm, nvm_size);
  }
  /* USER CODE BEGIN OnStoreContextRequest_Last */

  /* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnRestoreContextRequest_1 */

  /* USER CODE END OnRestoreContextRequest_1 */
  FLASH_IF_Read(nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size);
  /* USER CODE BEGIN OnRestoreContextRequest_Last */

  /* USER CODE END OnRestoreContextRequest_Last */
}
