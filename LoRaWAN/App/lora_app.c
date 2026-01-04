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
#include "SEGGER_RTT.h"
#include "atgm336h.h"
#include "multiregion_h3.h"
#include "multiregion_context.h"
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
/* Multi-region pre-join support */
volatile uint8_t g_multiregion_join_success = 0;
volatile uint8_t g_multiregion_in_prejoin = 0;
/* USER CODE END PV_MULTIREGION */

/**
  * @brief LoRaWAN default activation type
  */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN force rejoin even if the NVM context is restored
  */
// ForceRejoin: Uncomment to force rejoin on boot (useful for testing or network changes)
// static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

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
  
  /* Initialize Multi-region context manager */
  MultiRegion_Init();
  APP_LOG(TS_ON, VLEVEL_H, "Multi-region context manager initialized\r\n");
  
  /* TEMPORARY: Force rejoin by clearing all contexts - REMOVE AFTER TESTING */
  SEGGER_RTT_WriteString(0, "\r\n*** FORCING REJOIN - Clearing all saved contexts ***\r\n");
  MultiRegion_ClearAllContexts();
  SEGGER_RTT_WriteString(0, "*** Contexts cleared - will perform OTAA join ***\r\n\r\n");
  
  /* Auto-detect provision state: Check if we have valid saved ABP context for US915 */
  if (MultiRegion_IsRegionJoined(LORAMAC_REGION_US915)) {
    
    /* Already provisioned - use saved ABP context from flash */
    SEGGER_RTT_WriteString(0, "Found valid ABP context - using saved session\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "Using saved ABP context for US915\r\n");
    
    /* Switch to US915 as starting region */
    MultiRegion_SwitchToRegion(LORAMAC_REGION_US915);
    
    /* Display session keys for Chirpstack verification */
    SEGGER_RTT_WriteString(0, "\r\n=== VERIFY THESE KEYS MATCH YOUR CHIRPSTACK CONFIG ===\r\n");
    MultiRegion_DisplaySessionKeys();
    
  } else {
    
    /* Not provisioned yet - run OTAA provision sequence */
    SEGGER_RTT_WriteString(0, "No valid contexts found - running OTAA provision\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "Starting OTAA multi-region provision\r\n");
    
    /* Pre-join all regions via OTAA (includes post-join data packets) */
    /* This will: */
    /*   1. Join each region via OTAA */
    /*   2. Save session keys/DevAddr/counters to flash */
    /*   3. Send 2 post-join data packets per region */
    /*   4. Display session keys via RTT (for Chirpstack server setup) */
    MultiRegion_PreJoinAllRegions();
    
    APP_LOG(TS_ON, VLEVEL_H, "OTAA provision complete - contexts saved to flash\r\n");
  }

  /* USER CODE END LoRaWAN_Init_2 */

  // Skip LmHandlerJoin - already handled by auto-provision logic above
  // LmHandlerJoin(ActivationType, ForceRejoin);
  SEGGER_RTT_WriteString(0, "Skipping LmHandlerJoin - using auto-provision\r\n");

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Start(&TxTimer);
    
    /* Trigger first transmission immediately (don't wait for timer) */
    SEGGER_RTT_WriteString(0, "Triggering first transmission immediately...\r\n");
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
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
  
  /* Check join status - if not joined, trigger a new join attempt */
  if (LmHandlerJoinStatus() != LORAMAC_HANDLER_SET)
  {
    SEGGER_RTT_WriteString(0, "SendTxData: Not joined yet, triggering join retry...\r\n");
    LmHandlerJoin(ActivationType, true);
    return; /* Exit - will send data after join succeeds */
  }
  
  SEGGER_RTT_WriteString(0, "\r\n=== SendTxData START ===\r\n");

  /* ========== GPS HOT-START MODE ENABLED ========== */
  /* GPS uses hot-start standby mode: PB10=HIGH (backup power ~15µA) */
  /* Between TX cycles: PB5=LOW (standby), UART1 deinitialized, MCU can sleep */
  /* During TX: PB5=HIGH (active), UART1 active, achieves fix in ~1-5 seconds */
  // #define GPS_DISABLED_FOR_TESTING  1  // COMMENTED OUT - GPS NOW ACTIVE
  
  #ifdef GPS_DISABLED_FOR_TESTING
  
  /* Use fake GPS data for testing MCU sleep mode */
  SEGGER_RTT_WriteString(0, "GPS DISABLED FOR TESTING - using fake coordinates\r\n");
  
  /* Simulate GPS fix data */
  hgnss.data.valid = true;
  hgnss.data.fix_quality = GNSS_FIX_GPS;
  hgnss.data.latitude = 39.8283;    // Geographic center of contiguous USA (Kansas)
  hgnss.data.longitude = -98.5795;
  hgnss.data.altitude = 500.0f;     // meters (approximate)
  hgnss.data.satellites = 8;
  hgnss.data.hdop = 1.2f;
  
  uint32_t ttf_ms = 0;  /* No actual fix acquired */
  
  SEGGER_RTT_WriteString(0, "Fake GPS: Center USA (Kansas) | 39.8283°N, 98.5795°W | Alt: 500m | Sats: 8\r\n");
  
  #else
  
  /* ========== NORMAL GPS COLLECTION (CURRENTLY DISABLED) ========== */
  /* Non-blocking GNSS collection - wake from standby, capture fix quickly
   * GPS module in standby provides hot-start: <1s typical, 5s worst case
   * With Vbat backup and PMTK161 standby, we get instant fixes
   * 60s timeout allows for occasional warm/cold starts if needed
   */
  #define GNSS_COLLECTION_TIME_MS  60000  /* 60 seconds - reduced for testing */
  #define GNSS_MIN_SATS_FOR_FIX    4      /* Minimum satellites needed for fix */
  
  /* Last known GPS position storage (persistent across transmission cycles) */
  static float last_valid_lat = 39.8283f;     /* Default: Central US (Kansas) */
  static float last_valid_lon = -98.5795f;
  static float last_valid_alt = 500.0f;
  static bool have_previous_fix = false;
  
  /* Declare gps_start and ttf_ms at function scope */
  uint32_t gps_start = 0;
  uint32_t ttf_ms = 0;  /* Time to fix in milliseconds - captured when fix obtained */
  
  SEGGER_RTT_WriteString(0, "Waking GPS from standby for fix acquisition (20s max)...\r\n");
  if (GNSS_WakeFromStandby(&hgnss) == GNSS_OK)
  {
    /* CRITICAL: Invalidate old GPS data to force waiting for fresh NMEA sentences */
    /* This prevents reusing data from previous cycle (which would give false 0ms TTF) */
    hgnss.data.valid = false;
    hgnss.data.fix_quality = GNSS_FIX_INVALID;
    SEGGER_RTT_WriteString(0, "GPS data invalidated - waiting for fresh fix (hot-start <5s)...\r\n");
    
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
                 "GPS FIX! Lat=%ld.%06ld Lon=%ld.%06ld Alt=%ld.%ldm Sats:%d HDOP=%ld.%ld (took %lums)\r\n",
                 (long)(lat_int / 1000000), (long)labs(lat_int % 1000000),
                 (long)(lon_int / 1000000), (long)labs(lon_int % 1000000),
                 (long)(alt_int / 10), (long)labs(alt_int % 10),
                 hgnss.data.satellites,
                 (long)(hdop_int / 10), (long)labs(hdop_int % 10),
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
      
      /* Enter SLEEP mode - CPU halts but peripherals (UART/DMA) continue operating */
      /* CPU wakes automatically on any interrupt (DMA, SysTick, etc.) */
      /* This saves ~3-8mA during GPS acquisition compared to busy-wait polling */
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
    
    if (!got_fix)
    {
      /* Check if we have at least basic fix */
      if (GNSS_IsFixValid(&hgnss))
      {
        SEGGER_RTT_WriteString(0, "GPS: Basic fix (not high quality)\r\n");
        /* Update last known position even for basic fix */
        last_valid_lat = hgnss.data.latitude;
        last_valid_lon = hgnss.data.longitude;
        last_valid_alt = hgnss.data.altitude;
        have_previous_fix = true;
      }
      else
      {
        /* GPS timeout - use last known position if available */
        if (have_previous_fix)
        {
          SEGGER_RTT_WriteString(0, "GPS: Timeout - using last known position\r\n");
          hgnss.data.latitude = last_valid_lat;
          hgnss.data.longitude = last_valid_lon;
          hgnss.data.altitude = last_valid_alt;
          hgnss.data.valid = true;  /* Mark as valid to proceed with transmission */
          hgnss.data.fix_quality = GNSS_FIX_GPS;  /* Indicate GPS fix type */
        }
        else
        {
          SEGGER_RTT_WriteString(0, "GPS: No fix and no previous position - sending zeros\r\n");
          /* hgnss.data.valid remains false, will send zeros as fallback */
        }
      }
    }
    else
    {
      /* Successful fix - update last known position */
      last_valid_lat = hgnss.data.latitude;
      last_valid_lon = hgnss.data.longitude;
      last_valid_alt = hgnss.data.altitude;
      have_previous_fix = true;
      SEGGER_RTT_WriteString(0, "GPS: Fix acquired and stored as last known position\r\n");
    }
    
    /* Put GPS back to standby to save power and allow MCU to sleep */
    GNSS_EnterStandby(&hgnss);
    SEGGER_RTT_WriteString(0, "GPS entered standby mode (~15µA), MCU can now sleep\r\n");
    
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
               "H3 Region Lookup: Lat=%ld.%06ld Lon=%ld.%06ld -> %s (took %lums)\r\n",
               (long)(lat_int / 1000000), (long)labs(lat_int % 1000000),
               (long)(lon_int / 1000000), (long)labs(lon_int % 1000000),
               region_name, (unsigned long)h3_elapsed);
      SEGGER_RTT_WriteString(0, h3_msg);
      
      /* Production: Auto-switch region based on H3lite lookup */
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
    SEGGER_RTT_WriteString(0, "GPS: Wake from standby failed!\r\n");
  }
  
  #endif  /* GPS_DISABLED_FOR_TESTING */

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
  
  // CRITICAL FIX: Capture context after successful TX (not before region switch)
  // This ensures correct DevAddr, FCnt, and session state are saved
  if (params->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
    SEGGER_RTT_WriteString(0, "  TX successful - capturing context\r\n");
    MultiRegion_SaveCurrentContext();
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  char rtt_buf[128];
  
  SEGGER_RTT_WriteString(0, "\r\n=== OnJoinRequest Callback ===\r\n");
  snprintf(rtt_buf, sizeof(rtt_buf), "  Status: %s (%d)\r\n", 
           (joinParams->Status == LORAMAC_HANDLER_SUCCESS) ? "SUCCESS" : "FAILED",
           joinParams->Status);
  SEGGER_RTT_WriteString(0, rtt_buf);
  
  snprintf(rtt_buf, sizeof(rtt_buf), "  Mode: %s\r\n", 
           (joinParams->Mode == ACTIVATION_TYPE_OTAA) ? "OTAA" : "ABP");
  SEGGER_RTT_WriteString(0, rtt_buf);
  
  snprintf(rtt_buf, sizeof(rtt_buf), "  Datarate: DR%d, TxPower: %d\r\n", 
           joinParams->Datarate, joinParams->TxPower);
  SEGGER_RTT_WriteString(0, rtt_buf);
  
  if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
  {
    SEGGER_RTT_WriteString(0, "JOIN SUCCESS!\r\n");
    
    /* Set flag for multi-region pre-join */
    g_multiregion_join_success = true;
    
    /* ONLY start timer if NOT in pre-join mode */
    if (g_multiregion_in_prejoin) {
      SEGGER_RTT_WriteString(0, "Pre-join mode: Skipping Tx timer start\r\n");
    } else {
      /* Start the Tx timer now that we're joined */
      if (EventType == TX_ON_TIMER)
      {
        SEGGER_RTT_WriteString(0, "Starting Tx timer...\r\n");
        UTIL_TIMER_Start(&TxTimer);
      }
    }
  }
  else
  {
    SEGGER_RTT_WriteString(0, "JOIN FAILED - will retry on next timer event\r\n");
    g_multiregion_join_success = false;
  }
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
