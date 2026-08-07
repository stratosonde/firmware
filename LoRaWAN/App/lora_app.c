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
#include "LoRaMac.h"  /* LoRaMacQueryTxPossible — runtime payload budget (D3, #33) */
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"  /* memset — R31 full GNSS invalidation (#57) */
#include "SEGGER_RTT.h"
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */
#include "atgm336h.h"
#include "multiregion_h3.h"
#include "multiregion_context.h"
#include "timer_if.h"
#include "payload_format.h"
#include "flash_log.h"
#include "config.h"
#include "mission_state.h"
#include "reset_cause.h"      /* F13a: deadman breadcrumb register */
#include "stm32_systime.h"    /* F12: SysTimeSet (DDR-0003) */
#include "transmit_plan.h"    /* R47 (#44): DecideTransmitPlan */
#include "RegionUS915.h"      /* R03 (#32): region Datarates*[] tables for the SF resolver */
#include "RegionEU868.h"
#include "RegionAS923.h"
#include "RegionAU915.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern GNSS_HandleTypeDef hgnss;  /* GNSS handle from sys_sensors.c */
extern FlashLog_HandleTypeDef hflashlog;  /* Flash logging handle from main.c */
extern IWDG_HandleTypeDef hiwdg;  /* Watchdog handle from main.c */
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

/**
  * @brief Packet queue entry for deferred LoRaWAN transmission
  */
typedef struct
{
  uint8_t buffer[150];           // Packet data buffer
  uint16_t size;                 // Actual packet size
  uint8_t port;                  // LoRaWAN port number
  bool valid;                    // Entry is occupied
} PacketQueueEntry_t;

/**
  * @brief Simple circular packet queue
  */
#define PACKET_QUEUE_SIZE 8      // Max packets in queue
typedef struct
{
  PacketQueueEntry_t entries[PACKET_QUEUE_SIZE];
  uint8_t head;                  // Write position
  uint8_t tail;                  // Read position
  uint8_t count;                 // Current number of packets
} PacketQueue_t;

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
/* F24 FIX: StopJoin removed (dead SOS-button path; OTAA->ABP flip was dangerous) */

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */

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
static uint16_t EncodeGNSSDetailPacket(uint8_t *buffer, uint16_t max_size);
static void PacketQueue_Init(PacketQueue_t *queue);
static bool PacketQueue_Push(PacketQueue_t *queue, const uint8_t *data, uint16_t size, uint8_t port);
static bool PacketQueue_Pop(PacketQueue_t *queue, PacketQueueEntry_t *entry);
static bool PacketQueue_IsEmpty(PacketQueue_t *queue);
static uint8_t PacketQueue_Count(PacketQueue_t *queue);
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
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/* FW-10: this flag must NEVER be true in a build flashed to a flight unit —
 * combined with the T1 ladder it produces FLIGHT + virgin bank = permanent
 * RF silence. The runtime gate in LoRaWAN_Init() (commissioning-only clear)
 * is the real protection; this compile-time tripwire makes a mis-configured
 * build loud at build time. */
#if LORAWAN_FORCE_REJOIN_AT_BOOT
#warning "LORAWAN_FORCE_REJOIN_AT_BOOT=true: commissioning builds only - never flash to a flight unit"
#endif

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

/* USER CODE BEGIN PV */
#ifdef PROVISIONING_BUILD
/* R30/D6 (#24): ABP provisioning table — build-time alternative to the OTAA
 * ceremony. Entry macros live in the gitignored se-identity.h, e.g.:
 *   #define ABP_DEV_ADDR_US915   0x78000005
 *   #define ABP_APP_SKEY_US915   0x14,0x47,...  (16 bytes, brace-less)
 *   #define ABP_NWK_SKEY_US915   0x84,0x2d,...
 * A region with no macros defined is simply absent from the table. */
#include "se-identity.h"
typedef struct {
  LoRaMacRegion_t region;
  uint32_t dev_addr;
  uint8_t app_s_key[16];
  uint8_t nwk_s_key[16];
} ABPProvisioningEntry_t;
static const ABPProvisioningEntry_t abp_provisioning_table[] = {
#ifdef ABP_DEV_ADDR_US915
  { LORAMAC_REGION_US915, ABP_DEV_ADDR_US915, {ABP_APP_SKEY_US915}, {ABP_NWK_SKEY_US915} },
#endif
#ifdef ABP_DEV_ADDR_EU868
  { LORAMAC_REGION_EU868, ABP_DEV_ADDR_EU868, {ABP_APP_SKEY_EU868}, {ABP_NWK_SKEY_EU868} },
#endif
#ifdef ABP_DEV_ADDR_AS923
  { LORAMAC_REGION_AS923, ABP_DEV_ADDR_AS923, {ABP_APP_SKEY_AS923}, {ABP_NWK_SKEY_AS923} },
#endif
#ifdef ABP_DEV_ADDR_AU915
  { LORAMAC_REGION_AU915, ABP_DEV_ADDR_AU915, {ABP_APP_SKEY_AU915}, {ABP_NWK_SKEY_AU915} },
#endif
};
#endif

/* Packet queue for deferred transmission after RX windows */
static PacketQueue_t g_packet_queue = {0};

/* Adaptive transmission strategy state */
static TxState_t g_tx_state = TX_STATE_PROBE_SF10;
static uint8_t g_bulk_packets_sent = 0;
static uint32_t g_bulk_pending_mark = 0;  /* C4: records to mark after confirmed TX */
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
  SONDE_LOG_STR("LoRaApp_ReInitStack: Starting full stack reset...\r\n");
  
  // Halt current operations
  LmHandlerHalt();
  HAL_Delay(100);
  
  // Complete teardown
  SONDE_LOG_STR("LoRaApp_ReInitStack: Calling LmHandlerDeInit...\r\n");
  LmHandlerDeInit();
  HAL_Delay(200);
  
  // Rebuild from scratch
  SONDE_LOG_STR("LoRaApp_ReInitStack: Calling LmHandlerInit...\r\n");
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);
  HAL_Delay(100);
  
  // Set region parameter but DON'T configure yet
  // Configuration must be done AFTER DevEUI is set by caller
  LmHandlerParams.ActiveRegion = new_region;
  
  SONDE_LOG_STR("LoRaApp_ReInitStack: Stack reset complete (region set, not configured)\r\n");
}

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_LV */

  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */

  /* USER CODE END LoRaWAN_Init_1 */

  /* F24 FIX: StopJoinTimer removed */

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
  /* F24 FIX: StopJoin task removed */

  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  
  /* Initialize Multi-region context manager */
  MultiRegion_Init();
  APP_LOG(TS_ON, VLEVEL_H, "Multi-region context manager initialized\r\n");

  /* T3 (DDR-0008): decide mission state now that the session bank is loaded —
   * the bank, not a lone flag, anchors the one-way door. Idempotent. */
  MissionState_Init();
  
  /* Check ForceRejoin flag - if true, clear all saved contexts.
   * FW-10: gated behind COMMISSIONING. MissionState_Init() above has already
   * anchored the door to the bank, so a flight unit booting a mis-flashed
   * build reads FLIGHT and skips the wipe (its bank stays intact). */
  if (ForceRejoin) {
    if (MissionState_IsCommissioning()) {
      SONDE_LOG_STR("\r\n*** FORCE REJOIN ENABLED - Clearing all saved contexts ***\r\n");
      MultiRegion_ClearAllContexts();
      SONDE_LOG_STR("*** Contexts cleared - will perform fresh OTAA join ***\r\n\r\n");
    } else {
      SONDE_LOG_STR("FORCE REJOIN ignored: mission is FLIGHT (FW-10)\r\n");
    }
  }
  
  /* F22 FIX (DDR-0008): GNSS reconfiguration is COMMISSIONING-ONLY.
   * PCAS03/04/05/11 are saved to the GNSS module's internal flash via PCAS00 —
   * writing that flash on every boot wears it for zero benefit. */
  if (MissionState_IsCommissioning()) {
    SONDE_LOG_STR("*** COMMISSIONING: Reconfiguring GPS module (all constellations) ***\r\n");
    GNSS_PowerOn(&hgnss);
    HAL_Delay(1000);  // Let GPS boot
    GNSS_Configure(&hgnss);  // Sends PCAS04,7 + PCAS11 airborne + PCAS00 (save)
    HAL_Delay(500);   // Let GPS save to flash
    GNSS_PowerOff(&hgnss);
    SONDE_LOG_STR("*** GPS reconfigured and saved to flash ***\r\n\r\n");
  }
  
  /* Auto-detect provision state: Check if we have valid saved ABP context for US915 */
  /* Note: ForceRejoin will have cleared contexts above, so this will go to OTAA path */
  if (MultiRegion_IsRegionJoined(LORAMAC_REGION_US915)) {
    
    /* Already provisioned - use saved ABP context from flash */
    SONDE_LOG_STR("Found valid ABP context - using saved session\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "Using saved ABP context for US915\r\n");
    
    /* Switch to US915 as starting region */
    MultiRegion_SwitchToRegion(LORAMAC_REGION_US915);
    
    /* Display session keys for Chirpstack verification (F-017: commissioning only) */
    if (MissionState_IsCommissioning()) {
      SONDE_LOG_STR("\r\n=== VERIFY THESE KEYS MATCH YOUR CHIRPSTACK CONFIG ===\r\n");
      MultiRegion_DisplaySessionKeys();
    }
    
  } else {
    
    /* Not provisioned yet - run OTAA provision sequence */
    SONDE_LOG_STR("No valid contexts found - running OTAA provision\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "Starting OTAA multi-region provision\r\n");
    
    if (MissionState_IsCommissioning()) {
#ifdef PROVISIONING_BUILD
      /* R30/D6 (#24): ABP provisioning from a build-time table. Keys come from
       * the gitignored se-identity.h (F6/DDR-0006 — never committed). Define
       * PROVISIONING_BUILD plus any of ABP_DEV_ADDR_xxx / ABP_APP_SKEY_xxx /
       * ABP_NWK_SKEY_xxx (brace-less byte lists) to provision that region
       * without the OTAA ceremony. */
      uint8_t abp_count = 0;
      for (uint32_t i = 0; i < sizeof(abp_provisioning_table)/sizeof(abp_provisioning_table[0]); i++) {
        const ABPProvisioningEntry_t *e = &abp_provisioning_table[i];
        if (MultiRegion_InitializeRegionFromNetworkServer(e->region, e->dev_addr, e->app_s_key, e->nwk_s_key)) {
          abp_count++;
        }
      }
      SONDE_LOG("PROVISIONING_BUILD: %u region(s) initialized from table\r\n", abp_count);
      if (abp_count > 0) {
        MissionState_EnterFlight();
      } else {
        SONDE_LOG_STR("PROVISIONING FAILED: no regions initialized - staying in COMMISSIONING\r\n");
      }
#else
      /* Pre-join all regions via OTAA (includes post-join data packets) */
      /* This will: */
      /*   1. Join each region via OTAA */
      /*   2. Save session keys/DevAddr/counters to flash */
      /*   3. Send 2 post-join data packets per region */
      /*   4. Display session keys via RTT (for Chirpstack server setup) */
      /* D6 (#24): act on the return — commissioning is ground-only and MUST
       * complete; on failure say so loudly with remediation, not "complete". */
      bool provision_ok = MultiRegion_PreJoinAllRegions();
      if (provision_ok) {
        APP_LOG(TS_ON, VLEVEL_H, "OTAA provision complete - contexts saved to flash\r\n");
      } else {
        APP_LOG(TS_ON, VLEVEL_H, "PROVISION INCOMPLETE: fix gateway/credentials and power cycle to retry (still COMMISSIONING)\r\n");
        SONDE_LOG_STR("*** PROVISION INCOMPLETE - unit stays in COMMISSIONING, fix and power-cycle ***\r\n");
      }
#endif
    } else {
      /* T1 ladder (DDR-0006), rung 3: FLIGHT with a virgin session bank means
       * RF silence. Keep flying the profile — GPS, flash logging, timers —
       * but never attempt a join. */
      APP_LOG(TS_ON, VLEVEL_H, "FLIGHT: no valid session bank - RF silence, logging only\r\n");
      SONDE_LOG_STR("FLIGHT MODE with no saved session: RF SILENCE (DDR-0006)\r\n");
    }
  }

  /* USER CODE END LoRaWAN_Init_2 */

  // Skip LmHandlerJoin - already handled by auto-provision logic above
  // LmHandlerJoin(ActivationType, ForceRejoin);
  SONDE_LOG_STR("Skipping LmHandlerJoin - using auto-provision\r\n");

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Start(&TxTimer);

    /* Trigger first transmission immediately (don't wait for timer) */
    SONDE_LOG_STR("Triggering first transmission immediately...\r\n");
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */
  /* Initialize packet queue for deferred transmission */
  PacketQueue_Init(&g_packet_queue);
  SONDE_LOG_STR("Packet queue initialized\r\n");
  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/**
  * @brief  F16 FIX: Resolve a datarate by SPREADING FACTOR, not hardcoded index.
  *         DR indices are region-specific: "DR_3" means SF7@125kHz in US915/AU915
  *         but SF7 means DR_5 in EU868/AS923, and "DR_0" is SF10 in US915 but
  *         SF12 in EU868. Hardcoding an index silently changes airtime (and can
  *         even select an invalid DR) when the balloon crosses into another
  *         region. Resolve SF -> DR per active region.
  * @param  sf: desired spreading factor (7..10 supported here)
  * @retval Datarate enum for the currently active region
  */
static int8_t DatarateFromSF(uint8_t sf)
{
  /* R03/F-026 (#32): table-driven — search the ACTIVE region's own
   * Datarates*[] table for the SF. The old per-region switch hardcoded the
   * US915 mapping for AU915, but in-tree DataratesAU915 = {12,11,10,9,8,7,..}
   * (SF10 = DR_2, SF7 = DR_5), so every AU915 bulk uplink was rejected by the
   * MAC. Entries of 0 (unused) and 50 (FSK) are skipped; first match wins. */
  const uint8_t *table;
  uint8_t table_len;
  switch (LmHandlerParams.ActiveRegion) {
    case LORAMAC_REGION_US915:
      table = DataratesUS915; table_len = sizeof(DataratesUS915); break;
    case LORAMAC_REGION_EU868:
      table = DataratesEU868; table_len = sizeof(DataratesEU868); break;
    case LORAMAC_REGION_AS923:
      table = DataratesAS923; table_len = sizeof(DataratesAS923); break;
    case LORAMAC_REGION_AU915:
      table = DataratesAU915; table_len = sizeof(DataratesAU915); break;
    default:
      return LORAWAN_DEFAULT_DATA_RATE;
  }
  for (uint8_t dr = 0; dr < table_len; dr++) {
    if (table[dr] == sf) {
      return (int8_t)dr;
    }
  }
  SONDE_LOG("DatarateFromSF: SF%u not in region table - default\r\n", sf);
  return LORAWAN_DEFAULT_DATA_RATE;
}

/**
  * @brief  Encode detailed GNSS telemetry packet (satellite tracking + 3D speed)
  * @param  buffer: Destination buffer  
  * @param  max_size: Maximum buffer size
  * @retval Actual packet size in bytes
  * @note   Custom binary format on Port 3 for detailed analysis
  */
static uint16_t EncodeGNSSDetailPacket(uint8_t *buffer, uint16_t max_size)
{
  uint16_t idx = 0;
  
  if (buffer == NULL || max_size < 20)
    return 0;
  
  /* Header (4 bytes) - Version 2 format separates constellation counts */
  buffer[idx++] = 0x02;  // Packet version (v2 - separate constellation counts)
  buffer[idx++] = hgnss.extended.gps_count;
  buffer[idx++] = hgnss.extended.glonass_count;
  buffer[idx++] = hgnss.extended.beidou_count;
  
  /* GPS satellites (PRN + SNR for each) */
  for (int i = 0; i < hgnss.extended.gps_count && idx < (max_size - 2); i++)
  {
    buffer[idx++] = hgnss.extended.gps_sats[i].prn;
    buffer[idx++] = hgnss.extended.gps_sats[i].snr;
  }
  
  /* GLONASS satellites */
  for (int i = 0; i < hgnss.extended.glonass_count && idx < (max_size - 2); i++)
  {
    buffer[idx++] = hgnss.extended.glonass_sats[i].prn;
    buffer[idx++] = hgnss.extended.glonass_sats[i].snr;
  }
  
  /* BeiDou satellites */
  for (int i = 0; i < hgnss.extended.beidou_count && idx < (max_size - 2); i++)
  {
    buffer[idx++] = hgnss.extended.beidou_sats[i].prn;
    buffer[idx++] = hgnss.extended.beidou_sats[i].snr;
  }
  
  /* Speed data (12 bytes) - check we have room */
  if (idx + 12 > max_size)
    return idx;  // Return what we have so far
  
  /* Ground speed (2 bytes, 0.1 km/h resolution, 0-6553.5 km/h) */
  uint16_t ground_speed = (uint16_t)(hgnss.extended.ground_speed_kmh * 10.0f);
  buffer[idx++] = (ground_speed >> 8) & 0xFF;
  buffer[idx++] = ground_speed & 0xFF;
  
  /* Vertical speed (2 bytes signed, 0.01 m/s resolution, -327.68 to +327.67 m/s) */
  int16_t vertical_speed = (int16_t)(hgnss.extended.vertical_speed_ms * 100.0f);
  buffer[idx++] = (vertical_speed >> 8) & 0xFF;
  buffer[idx++] = vertical_speed & 0xFF;
  
  /* 3D speed (2 bytes, 0.1 km/h resolution) */
  uint16_t speed_3d = (uint16_t)(hgnss.extended.speed_3d_kmh * 10.0f);
  buffer[idx++] = (speed_3d >> 8) & 0xFF;
  buffer[idx++] = speed_3d & 0xFF;
  
  /* Track/course (2 bytes, 0.1° resolution, 0-359.9°) */
  uint16_t track = (uint16_t)(hgnss.extended.track_true * 10.0f);
  buffer[idx++] = (track >> 8) & 0xFF;
  buffer[idx++] = track & 0xFF;
  
  /* HDOP (2 bytes, 0.01 resolution) */
  uint16_t hdop = (uint16_t)(hgnss.data.hdop * 100.0f);
  buffer[idx++] = (hdop >> 8) & 0xFF;
  buffer[idx++] = hdop & 0xFF;
  
  /* Fix quality (1 byte) */
  buffer[idx++] = (uint8_t)hgnss.data.fix_quality;
  
  /* Satellites used in fix (1 byte) */
  buffer[idx++] = hgnss.data.satellites;
  
  return idx;
}

/* ========== VOLTAGE-BASED PREDICTIVE POWER MANAGEMENT ========== */
/* R49 (#46): NormalizeBatteryVoltage, CalculateVoltageSlope, PredictTimeToVoltage,
 * SelectModeFromPredictions and GetModeName moved verbatim to Core/Src/power_model.c
 * (declared in Core/Inc/power_model.h) so the pure decision logic is host-testable.
 * R47 (#44): ApplyOperatingMode + the whole decide block moved to
 * Core/Src/transmit_plan.c (DecideTransmitPlan). */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
  SONDE_LOG_STR("\r\n=== OnRxData Callback ===\r\n");
  
  // Read real LinkCheck results from LmHandler RxParams
  // LmHandlerRxParams_t already contains LinkCheck, DemodMargin, NbGateways
  // populated by the MAC layer from the server's LinkCheckAns response
  bool linkcheck_received = params->LinkCheck;
  uint8_t margin = params->DemodMargin;
  uint8_t gw_count = params->NbGateways;
  
  // FW-17: only log margin/gateway count when a LinkCheckAns was actually
  // received — otherwise these fields are garbage
  if (linkcheck_received) {
    char link_msg[128];
    snprintf(link_msg, sizeof(link_msg),
             "LinkCheckAns: Margin=%ddB, Gateways=%d\r\n",
             margin, gw_count);
    SONDE_LOG_STR(link_msg);
  }
  
  /* DDR-0011 (#34): the archive opportunity opens on the confirmed probe's ACK
   * (OnTxData), not on LinkCheckAns. LinkCheckAns now rides the FIRST archive
   * packet (protocol §5.2) and gates burst continuation (§5.3): poor margin /
   * gateway count — or no LinkCheckAns at all — ends the burst after the
   * (already ACKed) first archive packet. */
  if (g_tx_state == TX_STATE_BULK_TRANSFER && g_bulk_packets_sent == 1) {
    bool link_good = (linkcheck_received &&
                      margin >= LINK_MARGIN_THRESHOLD &&
                      gw_count >= GATEWAY_COUNT_THRESHOLD);
    SONDE_LOG("First archive response: LinkCheckAns %s, margin=%ddB (>=%d), gateways=%d (>=%d) -> %s\r\n",
                      linkcheck_received ? "received" : "MISSING",
                      margin, LINK_MARGIN_THRESHOLD, gw_count, GATEWAY_COUNT_THRESHOLD,
                      link_good ? "BURST CONTINUES" : "FALLBACK");
    if (!link_good) {
      /* Protocol §5.4: end the burst, return to LONG_RANGE_HEARTBEAT.
       * The first packet's records were already committed on its ACK — no loss. */
      g_tx_state = TX_STATE_COMPLETE;
      g_bulk_packets_sent = 0;
    }
  } else if (linkcheck_received) {
    // Link check result for non-archive transmissions (informational)
    SONDE_LOG("Link quality: margin=%ddB, gateways=%d\r\n", margin, gw_count);
  }
  
  // Process any received application data
  if (appData && appData->BufferSize > 0) {
    char rx_msg[64];
    snprintf(rx_msg, sizeof(rx_msg), "Received %d bytes on port %d\r\n", 
             appData->BufferSize, appData->Port);
    SONDE_LOG_STR(rx_msg);
  }
  
  SONDE_LOG_STR("=== OnRxData Callback END ===\r\n");
  /* USER CODE END OnRxData_1 */
}

/**
  * @brief  F13a (DDR-0001): progress deadman. SendTxData is the only place a
  *         full work cycle provably begins — mark RTC seconds here. If the
  *         sequencer/timer wedges (no cycle for 3x the worst-case interval),
  *         Deadman_Check breadcrumbs and resets. COMMISSIONING is exempt:
  *         on the bench a human can sit idle for hours legitimately.
  */
/* R01/R02: DR2 collided with timer_if SysTime MSBTICKS — moved to DR5.
 * Allocation map lives in backup_regs.h. */
#include "backup_regs.h"
#define DEADMAN_BKP_REG     BKP_REG_DEADMAN
#define DEADMAN_TIMEOUT_S   (3U * 3600U)   /* 3x worst-case cycle (SURVIVAL=1h) */

static void Deadman_MarkProgress(void)
{
  extern RTC_HandleTypeDef hrtc;
  uint16_t ms_unused;
  HAL_RTCEx_BKUPWrite(&hrtc, DEADMAN_BKP_REG, TIMER_IF_GetTime(&ms_unused));
  ResetCause_ClearBootAttempts();  /* F-03 (#65): a work cycle started — boot was good */
}

void Deadman_Check(void)
{
  /* R45: Deadman deliberately keeps boot-relative MCU time (TIMER_IF), NOT
   * SysTime — a GPS sync would jump the clock by decades and false-trip the
   * watchdog. Two clocks, two jobs: UTC for science records, monotonic MCU
   * time for liveness. */
  extern RTC_HandleTypeDef hrtc;
  uint16_t ms_unused;
  uint32_t now = TIMER_IF_GetTime(&ms_unused);
  uint32_t last = HAL_RTCEx_BKUPRead(&hrtc, DEADMAN_BKP_REG);

  if (last == 0) {
    HAL_RTCEx_BKUPWrite(&hrtc, DEADMAN_BKP_REG, now);  /* first boot: seed */
    return;
  }
  if (MissionState_IsCommissioning()) {
    return;  /* bench: human idle is legitimate */
  }
  if (now < last) {
    /* F-05 (#63): time jumped backward (e.g. pre-F-04 MSB stomp or backup
     * corruption). (now - last) is unsigned and would wrap huge, false-firing
     * the deadman. Re-seed and continue — never fire on a backward delta. */
    HAL_RTCEx_BKUPWrite(&hrtc, DEADMAN_BKP_REG, now);
    return;
  }
  if ((now - last) > DEADMAN_TIMEOUT_S) {
    SONDE_LOG_STR("DEADMAN: no work cycle for 3h - breadcrumb + reset\r\n");
    HAL_RTCEx_BKUPWrite(&hrtc, RESET_CAUSE_BKP_FAULT_REG,
                        RESET_CAUSE_FAULT_MAGIC | 6U);  /* 6 = deadman */
    /* F-05 (#63): re-seed BEFORE resetting — otherwise the post-reset boot
     * sees the same stale timestamp and fires again (deadman reset loop). */
    HAL_RTCEx_BKUPWrite(&hrtc, DEADMAN_BKP_REG, now);
    NVIC_SystemReset();
  }
}

/**
  * @brief  F12 (DDR-0003): discipline system time from a good GPS fix so flash
  *         records carry absolute UTC epoch seconds instead of boot-relative
  *         time. GPS is the only trustworthy clock source on the balloon.
  *         date = DDMMYY, timestamp = HHMMSS (NMEA RMC).
  */
static uint32_t DaysFromCivil(int y, unsigned m, unsigned d)
{
  y -= (m <= 2);
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153U * (m + (m > 2 ? (unsigned)-3 : 9U)) + 2U) / 5U + d - 1U;
  unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  return (uint32_t)(era * 146097 + (int)doe - 719468);
}

static void SysTimeSyncFromGnss(void)
{
  uint32_t d = hgnss.data.date;       /* DDMMYY */
  uint32_t t = hgnss.data.timestamp;  /* HHMMSS */
  if (d == 0 || t == 0) return;

  int day = (int)(d / 10000U);
  int mon = (int)((d / 100U) % 100U);
  int yr  = 2000 + (int)(d % 100U);
  if (yr < 2024 || mon < 1 || mon > 12 || day < 1 || day > 31) return;

  uint32_t epoch = DaysFromCivil(yr, (unsigned)mon, (unsigned)day) * 86400U
                 + (t / 10000U) * 3600U + ((t / 100U) % 100U) * 60U + (t % 100U);

  SysTime_t st;
  st.Seconds = epoch;
  st.SubSeconds = 0;
  SysTimeSet(st);
  SONDE_LOG("SysTime disciplined from GPS: %lu epoch seconds\r\n",
                    (unsigned long)epoch);
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
  Deadman_MarkProgress();  /* F13a: a work cycle provably started */

  /* ========== POWER MANAGEMENT — decide half (R47, #44) ========== */
  static VoltageSlope_t voltage_slope = {0};

  // Read current sensor data for temperature
  sensor_t sensor_data = {0};  /* #35: zero-init — uninitialized members were archived as authentic */
  EnvSensors_Read(&sensor_data);
  MissionState_Update(sensor_data.pressure, !sensor_data.press_stale);  /* D8 (#59): pressure-trend float detection, each work cycle */
  float temperature_c = sensor_data.temperature;

  // Read raw voltages
  uint16_t battery_mv_raw = SYS_GetBatteryVoltage();
  uint16_t solar_mv = SYS_GetSolarVoltage();

  // Use RTC-based time that continues during STOP2 sleep
  uint16_t ms_unused;
  uint32_t now_timestamp = TIMER_IF_GetTime(&ms_unused);  // RTC seconds

  /* R47: mode selection, slope/prediction, GPS temperature lockout and the
   * RF-silence veto all live in the pure decide half (transmit_plan.c) —
   * host-testable with zero hardware. The plan records WHY, not just THAT. */
  TransmitPlan_t plan = DecideTransmitPlan(&voltage_slope, battery_mv_raw,
                                           temperature_c, sensor_data.temp_stale != 0,
                                           now_timestamp,
                                           LmHandlerJoinStatus() == LORAMAC_HANDLER_SET,
                                           MissionState_IsCommissioning());
  bool gps_enabled_by_power_mgmt = plan.gps_enabled;
  uint32_t gps_timeout_ms = plan.gps_timeout_ms;
  OperatingMode_t current_mode = plan.power_mode;
  int16_t slope_mv_per_hour = plan.voltage_slope_mv_per_hour;
  int16_t time_to_target_signed = plan.time_to_target_h;
  uint16_t battery_mv_normalized = plan.battery_mv_normalized;

  // Update timer interval if changed
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, plan.tx_interval_ms);
  UTIL_TIMER_Start(&TxTimer);
  
  // Log power management status
  /* F27 FIX: integer-only print (no float printf support linked) */
  int temp_deci_pm = (int)(temperature_c * 10.0f);
  char pm_msg[256];
  snprintf(pm_msg, sizeof(pm_msg),
           "\r\n=== POWER MGMT: Temp=%d.%dC Bat_raw=%dmV Bat_norm=%dmV Solar=%dmV Slope=%+dmV/h ",
           temp_deci_pm / 10, abs(temp_deci_pm % 10),
           battery_mv_raw, battery_mv_normalized, solar_mv, slope_mv_per_hour);
  SONDE_LOG_STR(pm_msg);
  
  if (time_to_target_signed < 0) {
    snprintf(pm_msg, sizeof(pm_msg), "Critical_in=%dh ", abs(time_to_target_signed));
  } else if (time_to_target_signed > 0) {
    snprintf(pm_msg, sizeof(pm_msg), "Full_in=%dh ", time_to_target_signed);
  } else {
    snprintf(pm_msg, sizeof(pm_msg), "Stable ");
  }
  SONDE_LOG_STR(pm_msg);
  
  snprintf(pm_msg, sizeof(pm_msg), "Mode=%s GPS=%s ===\r\n",
           GetModeName(current_mode), gps_enabled_by_power_mgmt ? "ON" : "OFF");
  SONDE_LOG_STR(pm_msg);
  
  /* F11 FIX: Flash logging moved to AFTER GPS acquisition + sensor re-read
   * (see below). Previously the record was written here with the PREVIOUS
   * cycle's position and the CURRENT timestamp — the whole track was
   * spatially stale by one interval. */

  /* ========== END POWER MANAGEMENT ========== */
  
  /* T1 ladder (DDR-0006): rejoin is a COMMISSIONING-ONLY operation.
   * In FLIGHT, an invalid session means RF silence — keep flying the profile
   * (GPS + flash logging below), skip only the transmission. */
  bool rf_silence = false;
  if (LmHandlerJoinStatus() != LORAMAC_HANDLER_SET)
  {
    if (MissionState_IsCommissioning()) {
      SONDE_LOG_STR("SendTxData: Not joined yet, triggering join retry...\r\n");
      LmHandlerJoin(ActivationType, true);
      return; /* Exit - will send data after join succeeds */
    }
    rf_silence = true;
    SONDE_LOG_STR("SendTxData: FLIGHT with no session - RF silence, logging only\r\n");
  }

  SONDE_LOG_STR("\r\n=== SendTxData START ===\r\n");

  /* ========== GPS POWER-CYCLING MODE ========== */
  /* FW-8: full power-off between cycles (PB10=LOW, PB5=LOW, 0µA) — ephemeris
   * is persisted to GPS internal flash via PCAS12; hot-start on wake uses the
   * flash-persisted ephemeris. Between TX cycles: UART1 deinitialized, MCU
   * sleeps. During TX: PB10/PB5 HIGH, UART1 active, fix in ~1-5s expected. */
  // #define GPS_DISABLED_FOR_TESTING  1  // COMMENTED OUT - GPS NOW ACTIVE
  
  /* Declare ttf_ms at function scope so it's available for telemetry */
  uint32_t ttf_ms = 0;
  
  // Check if GPS is disabled by power management or bulk transfer mode
  // C7a FIX: Skip GPS during bulk transfer - we're sending cached flash data, not live telemetry
  if (!gps_enabled_by_power_mgmt || g_tx_state == TX_STATE_BULK_TRANSFER) {
    /* GPS disabled - skip acquisition */
    if (g_tx_state == TX_STATE_BULK_TRANSFER) {
      SONDE_LOG_STR("GPS skipped - bulk transfer mode (using cached data)\r\n");
    } else {
      SONDE_LOG_STR("GPS disabled by power management - skipping acquisition\r\n");
    }
    /* R31 (#57): FULL invalidation before/without acquisition — clear all of
     * hgnss.data (sats/hdop/lat/lon included), not just valid/fix_quality.
     * The GGA parser skips empty tokens, so a partial sentence must never meet
     * last cycle's fields. Last-known-good lives in the last_valid_* statics. */
    memset(&hgnss.data, 0, sizeof(hgnss.data));
    ttf_ms = 0;  // No GPS acquisition performed
  } else {
  
  #ifdef GPS_DISABLED_FOR_TESTING
  
  /* Use fake GPS data for testing MCU sleep mode */
  SONDE_LOG_STR("GPS DISABLED FOR TESTING - using fake coordinates\r\n");
  
  /* Simulate GPS fix data */
  hgnss.data.valid = true;
  hgnss.data.fix_quality = GNSS_FIX_GPS;
  hgnss.data.latitude = 39.8283;    // Geographic center of contiguous USA (Kansas)
  hgnss.data.longitude = -98.5795;
  hgnss.data.altitude = 500.0f;     // meters (approximate)
  hgnss.data.satellites = 8;
  hgnss.data.hdop = 1.2f;
  
  ttf_ms = 0;  /* No actual fix acquired */
  
  SONDE_LOG_STR("Fake GPS: Center USA (Kansas) | 39.8283°N, 98.5795°W | Alt: 500m | Sats: 8\r\n");
  
  #else
  
  /* ========== NORMAL GPS COLLECTION ========== */
  /* Non-blocking GNSS collection - wake from standby, capture fix quickly
   * GPS module in standby provides hot-start: <1s typical, 5s worst case
   * With Vbat backup and PMTK161 standby, we get instant fixes
   * gps_timeout_ms is dynamic (30s or 60s) based on power mode
   */
  #define GNSS_MIN_SATS_FOR_FIX    4      /* Minimum satellites needed for fix */
  
  /* Last known GPS position storage (persistent across transmission cycles) */
  static float last_valid_lat = 39.8283f;     /* Default: Central US (Kansas) */
  static float last_valid_lon = -98.5795f;
  static float last_valid_alt = 500.0f;
  static bool have_previous_fix = false;
  
  /* Declare gps_start - ttf_ms already declared above */
  uint32_t gps_start = 0;
  ttf_ms = 0;  /* Will be updated when fix is obtained */
  
  SONDE_LOG("Waking GPS from standby for fix acquisition (%lus max)...\r\n", 
                    (unsigned long)(gps_timeout_ms / 1000));
  if (GNSS_WakeFromStandby(&hgnss) == GNSS_OK)
  {
    /* CRITICAL: Invalidate old GPS data to force waiting for fresh NMEA sentences */
    /* This prevents reusing data from previous cycle (which would give false 0ms TTF) */
    /* R31 (#57): FULL invalidation — the GGA parser skips empty tokens, so a
     * partial sentence must never meet last cycle's sats/hdop/lat/lon.
     * Last-known-good lives in the last_valid_* statics below. */
    memset(&hgnss.data, 0, sizeof(hgnss.data));
    SONDE_LOG_STR("GPS data invalidated - waiting for fresh fix (hot-start <5s)...\r\n");
    
    gps_start = HAL_GetTick();
    bool got_fix = false;
    uint32_t last_status_print = 0;
    
    /* Process GPS data for up to gps_timeout_ms (dynamic based on power mode) */
    while ((HAL_GetTick() - gps_start) < gps_timeout_ms)
    {
      /* Process DMA buffer - parses NMEA and updates hgnss.data */
      GNSS_ProcessDMABuffer(&hgnss);
      
      /* CRITICAL: Refresh watchdog during GPS acquisition to prevent timeout reset */
      /* GPS can take up to 60s, approaching the 32.76s watchdog timeout */
      HAL_IWDG_Refresh(&hiwdg);
      
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
        SONDE_LOG_STR(fix_msg);
        break;  /* Exit early - we have what we need */
      }
      
      /* Print status every 5 seconds during acquisition */
      uint32_t elapsed = HAL_GetTick() - gps_start;
      if (elapsed - last_status_print >= 5000)
      {
        /* F27 FIX: integer-only print (no float printf support linked) */
        int hdop_deci = (int)(hgnss.data.hdop * 10.0f);
        char status_msg[100];
        snprintf(status_msg, sizeof(status_msg),
                 "[GPS %lus] Sats:%d/%d HDOP:%d.%d Fix:%s\r\n",
                 (unsigned long)(elapsed / 1000),
                 hgnss.data.satellites, hgnss.data.satellites_in_view,
                 hdop_deci / 10, hdop_deci % 10,
                 (hgnss.data.fix_quality != GNSS_FIX_INVALID) ? "Yes" : "No");
        SONDE_LOG_STR(status_msg);
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
        SONDE_LOG_STR("GPS: Basic fix (not high quality)\r\n");
        /* Update last known position even for basic fix */
        last_valid_lat = hgnss.data.latitude;
        last_valid_lon = hgnss.data.longitude;
        last_valid_alt = hgnss.data.altitude;
        have_previous_fix = true;
        EnvSensors_MarkGnssStale(false);  /* F8/T2: fresh data, not stale */
      }
      else
      {
        /* GPS timeout - use last known position if available */
        if (have_previous_fix)
        {
          /* F8/T2 (DDR-0007): last-known-good position still flows, but the
           * GPS-stale bit is set so nothing downstream mistakes it for live. */
          SONDE_LOG_STR("GPS: Timeout - using last known position (STALE)\r\n");
          hgnss.data.latitude = last_valid_lat;
          hgnss.data.longitude = last_valid_lon;
          hgnss.data.altitude = last_valid_alt;
          hgnss.data.valid = true;  /* Mark as valid to proceed with transmission */
          hgnss.data.fix_quality = GNSS_FIX_GPS;  /* Indicate GPS fix type */
          EnvSensors_MarkGnssStale(true);
        }
        else
        {
          SONDE_LOG_STR("GPS: No fix and no previous position - sending zeros\r\n");
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
      EnvSensors_MarkGnssStale(false);  /* F8/T2: fresh fix, clear stale */
      SysTimeSyncFromGnss();            /* F12 (DDR-0003): epoch seconds */
      SONDE_LOG_STR("GPS: Fix acquired and stored as last known position\r\n");
    }
    
    /* Put GPS back to full power-off (0µA) and allow MCU to sleep */
    GNSS_EnterStandby(&hgnss);
    SONDE_LOG_STR("GPS fully powered off (0µA), MCU can now sleep\r\n");
    
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
      
      /* BUG 1.3 FIX: Only skip transmission for REGION_RESTRICTED (regulatory prohibition).
       * NOTE: REGION_RESTRICTED is now 15 (h3lite), not 255 — the 4-bit regionId
       * field in the packed table could never emit 255; ID 15 was repurposed from
       * the CD900-1A test plan slot. This macro comparison keeps working because
       * both sides use the same h3lite.h definition.
       * REGION_UNKNOWN means open ocean or uncovered H3 cells — keep current region and
       * transmit normally (standard convention over international waters). Previously this
       * also blocked UNKNOWN, causing the balloon to go silent over every ocean crossing. */
      RegionId h3_region_id = latLngToRegion(hgnss.data.latitude, hgnss.data.longitude);
      
      if (h3_region_id == REGION_RESTRICTED) {
        /* R11 (#56): don't RETURN here — that discarded the sample entirely.
         * The archive exists precisely for data that can't be transmitted:
         * use the rf_silence pattern (DDR-0006) — GPS + re-read + flash write
         * proceed below; only the TX state machine is skipped. */
        SONDE_LOG_STR("RESTRICTED REGION: RF silence — archiving locally, radio dark\r\n");
        rf_silence = true;
      }
      
      if (h3_region_id == REGION_UNKNOWN) {
        SONDE_LOG_STR("UNKNOWN REGION (ocean/uncovered): Keeping current region, transmitting normally\r\n");
        // Do NOT return — continue with current region
      }
      
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
      SONDE_LOG_STR(h3_msg);
      
      /* Production: Auto-switch region based on H3lite lookup */
      LmHandlerErrorStatus_t switch_status = MultiRegion_AutoSwitchForLocation(
          hgnss.data.latitude, 
          hgnss.data.longitude
      );
      
      if (switch_status == LORAMAC_HANDLER_SUCCESS) {
        SONDE_LOG_STR("MultiRegion: Auto-switch completed successfully\r\n");
      } else if (switch_status == LORAMAC_HANDLER_BUSY_ERROR) {
        SONDE_LOG_STR("MultiRegion: Switch deferred (MAC busy)\r\n");
      }
    }
    else
    {
      SONDE_LOG_STR("H3 Region Lookup: Skipped (no valid GPS fix)\r\n");
    }
  }
  else
  {
    SONDE_LOG_STR("GPS: Wake from standby failed!\r\n");
  }
  
  #endif  /* GPS_DISABLED_FOR_TESTING */
  }  /* End of else block for gps_enabled_by_power_mgmt */

  /* Add separator before continuing to telemetry */
  SONDE_LOG_STR("\r\n");
  
  // CRITICAL: Re-read sensor data AFTER GPS acquisition to include fresh GPS fix
  SONDE_LOG_STR("Re-reading sensor data to capture fresh GPS fix...\r\n");
  EnvSensors_Read(&sensor_data);  // This includes the GPS fix we just acquired
  SONDE_LOG_STR("Sensor data refreshed with current GPS position\r\n");

  /* ========== FLASH LOGGING: Store high-resolution data ========== */
  /* F11 FIX: Write the archive record HERE — after the GPS fix and post-fix
   * sensor re-read — so position, time, and environment in a record describe
   * the same moment. Timestamp is taken fresh at write time. */
  /* F-07 (#68): only archive genuine, freshly-sampled telemetry records.
   * OnTxData re-arms this task once per BULK packet (DDR-0011/#34); the bulk
   * path skips GPS entirely (hgnss.data memset above), so archiving those
   * passes wrote up to 20 junk records (gnss_valid=0) per bulk cycle —
   * accelerating ring wrap and burning W25Q + Tier-2 internal-flash erase
   * cycles (worst case ~33 days to internal-flash endurance). */
  if (g_tx_state != TX_STATE_BULK_TRANSFER) {
    SONDE_LOG_STR("Logging high-resolution data to flash...\r\n");
    /* R45: stamp with disciplined UTC epoch (SysTime applies the GPS-synced
     * delta stored in backup regs), NOT boot-relative RTC calendar time.
     * Before the first GPS fix this falls back to boot-relative seconds —
     * honest, monotonic, and distinguishable (small values) from epoch. */
    now_timestamp = SysTimeGet().Seconds;  // UTC epoch seconds at write time
    FlashLog_StatusTypeDef log_status = FlashLog_WriteRecord(&hflashlog, &sensor_data, now_timestamp,
                                                             slope_mv_per_hour, (uint8_t)current_mode);
    if (log_status == FLASH_LOG_OK) {
      uint32_t record_count = FlashLog_GetRecordCount(&hflashlog);
      SONDE_LOG("Flash log: Written record %lu (total records: %lu)\r\n",
                        record_count, record_count);
    } else {
      SONDE_LOG("Flash log: Write failed (status: %d)\r\n", log_status);
    }
  } else {
    SONDE_LOG_STR("Flash log: bulk retransmit cycle — archive write skipped\r\n");
  }

  // Initialize Cayenne LPP payload
  CayenneLppReset();
  SONDE_LOG_STR("CayenneLpp reset\r\n");
  
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
    
    SONDE_LOG_STR("GNSS data valid\r\n");
  } else {
    // Use zeros when no valid GNSS fix
    lat = 0.0f;
    lon = 0.0f;
    alt = 0.0f;
    
    SONDE_LOG_STR("GNSS data invalid\r\n");
  }
  
  CayenneLppAddGps(4, lat, lon, alt);
  
  // Add number of satellites as analog input (channel 5) - value 0-255
  CayenneLppAddAnalogInput(5, (float)sensor_data.satellites);
  
  // Add battery voltage on channel 6 (in volts)
  CayenneLppAddAnalogInput(6, sensor_data.battery_voltage);

  // Add regulator voltage (3.3V rail) on channel 7 (in volts)
  CayenneLppAddAnalogInput(7, sensor_data.regulator_voltage);

  // Add solar panel voltage on channel 10 (in volts)
  CayenneLppAddAnalogInput(10, sensor_data.solar_voltage);
  
  // Add GNSS HDOP on channel 8 (Horizontal Dilution of Precision)
  CayenneLppAddAnalogInput(8, sensor_data.gnss_hdop);
  
  // Add TTF (Time To Fix) on channel 9 (in seconds, 0.01s resolution)
  // Convert from milliseconds to seconds to avoid int16_t overflow in Cayenne LPP
  // CayenneLpp analog uses int16_t with 0.01 resolution, max value = 327.67
  CayenneLppAddAnalogInput(9, (float)ttf_ms / 1000.0f);

  // Add power management telemetry
  // Channel 11: Voltage slope (mV/hour, scaled by 10 for better resolution)
  CayenneLppAddAnalogInput(11, (float)slope_mv_per_hour / 10.0f);
  
  // Channel 12: Time to target (signed: +charging hours, -depletion hours, 0=stable)
  CayenneLppAddAnalogInput(12, (float)time_to_target_signed);
  
  // Channel 14: Operating mode (0=NORMAL, 1=CONSERVATIVE, 2=REDUCED, 3=RECOVERY, 4=SURVIVAL)
  CayenneLppAddAnalogInput(14, (float)current_mode);

  /* Safe RTT output - use integer conversion for HDOP to avoid float printf issues */
  int hdop_int = (int)(sensor_data.gnss_hdop * 10);
  char lpp_msg[120];
  snprintf(lpp_msg, sizeof(lpp_msg), "Cayenne LPP: HDOP=%d.%d TTF=%lums Slope=%+d Time=%d Mode=%d\r\n", 
           hdop_int / 10, hdop_int % 10, (unsigned long)ttf_ms, 
           slope_mv_per_hour, time_to_target_signed, current_mode);
  SONDE_LOG_STR(lpp_msg);
  
  /* ========== ADAPTIVE TRANSMISSION STRATEGY ========== */
  // Step 1: Always send 10-byte compact packet at SF10 with LinkCheckReq
  // This provides maximum range and evaluates link quality for bulk transfer
  
  /* C2 FIX: Reset stale TX states so every timer cycle sends actual data.
   * If the previous cycle left us in WAIT_PROBE_ACK (no downlink received) or
   * COMPLETE (already handled), reset to PROBE_SF10 for a fresh probe.
   * BULK_TRANSFER is NOT reset here - it continues until exhausted via OnTxData. */
  if (g_tx_state == TX_STATE_WAIT_PROBE_ACK || g_tx_state == TX_STATE_COMPLETE) {
    SONDE_LOG("Resetting stale TX state %d -> PROBE_SF10\r\n", g_tx_state);
    g_tx_state = TX_STATE_PROBE_SF10;
    g_bulk_packets_sent = 0;
  }
  
  SONDE_LOG("Adaptive TX: State=%d\r\n", g_tx_state);

  /* T1 (DDR-0006): RF silence skips the entire transmit state machine —
   * GPS acquisition and flash logging above have already run. */
  if (rf_silence) {
    g_tx_state = TX_STATE_PROBE_SF10;  /* keep state machine parked */
  } else
  switch (g_tx_state) {
    case TX_STATE_PROBE_SF10:
    {
      // Encode 10-byte compact telemetry packet
      CompactTelemetryPacket_t compact_packet;
      uint16_t timestamp_min = (uint16_t)(now_timestamp / 60);  // Convert to minutes
      
      if (EncodeCompactBinaryPacket(&compact_packet, &sensor_data, timestamp_min, 
                                   slope_mv_per_hour, current_mode)) {
        
      /* D1 (#33): probe at SF9 in US915/AU915 — SF10/DR0's 11-byte budget is an
       * exact fit there with zero headroom; SF9 buys 42 B for ~2.5 dB link
       * budget. Elsewhere keep SF10. Resolved per-region via DatarateFromSF. */
      uint8_t probe_sf = ((LmHandlerParams.ActiveRegion == LORAMAC_REGION_US915) ||
                          (LmHandlerParams.ActiveRegion == LORAMAC_REGION_AU915)) ? 9 : 10;
      LmHandlerSetTxDatarate(DatarateFromSF(probe_sf));
      
      // Prepare packet data BEFORE requesting LinkCheck
      LmHandlerAppData_t compactData;
      compactData.Port = LORAWAN_COMPACT_PORT;  // Port 10
      compactData.BufferSize = sizeof(CompactTelemetryPacket_t);
      compactData.Buffer = (uint8_t*)&compact_packet;
      
      /* DDR-0011 (#34): the opportunity-probe heartbeat is a CONFIRMED uplink.
       * No archive opportunity opens without its network ACK (evaluated in
       * OnTxData via params->AckReceived). LinkCheck no longer rides the probe —
       * it attaches to the first archive packet (protocol §5.2, §14). */
      LmHandlerErrorStatus_t status = LmHandlerSend(&compactData, LORAMAC_HANDLER_CONFIRMED_MSG, 0);
        
        if (status == LORAMAC_HANDLER_SUCCESS) {
          g_tx_state = TX_STATE_WAIT_PROBE_ACK;  /* Wait for the confirmed-uplink ACK (OnTxData) */
          SONDE_LOG_STR("Confirmed heartbeat sent, waiting for network ACK...\r\n");
        } else {
          SONDE_LOG("Compact packet send failed (status: %d)\r\n", status);
          g_tx_state = TX_STATE_COMPLETE;  // Complete cycle on error
        }
      } else {
        SONDE_LOG_STR("ERROR: Failed to encode compact packet!\r\n");
        g_tx_state = TX_STATE_COMPLETE;
      }
      break;
    }
    
    case TX_STATE_WAIT_PROBE_ACK:
      /* Should not reach here - stale states are reset above before the switch.
       * Fall through to PROBE_SF10 as safety fallback. */
      SONDE_LOG_STR("WARN: Unexpected WAIT_PROBE_ACK in switch, resetting\r\n");
      g_tx_state = TX_STATE_PROBE_SF10;
      /* fall through to PROBE_SF10 - will send on next cycle */
      break;
      
    case TX_STATE_BULK_TRANSFER:
    {
      SONDE_LOG("Bulk transfer mode: packet %d/%d\r\n", 
                        g_bulk_packets_sent + 1, MAX_BULK_PACKETS_PER_CYCLE);
      
      // Check if we have unsent data and haven't exceeded packet limit
      if (FlashLog_HasUnsentData(&hflashlog) && g_bulk_packets_sent < MAX_BULK_PACKETS_PER_CYCLE) {
        
        // Read up to 6 unsent records from flash (FIFO order - oldest unsent
        // first, so MarkRecordsTransmitted advances the watermark correctly)
        FlashLog_Record_t flash_records[6];
        uint32_t record_count;
        uint32_t skipped_count = 0;  /* F-006/R13 (#51): corrupt skips are explicit */

        FlashLog_StatusTypeDef flash_status = FlashLog_GetUnsentRecordsFIFO(&hflashlog,
                                                                            flash_records,
                                                                            6,
                                                                            &record_count,
                                                                            &skipped_count);
        if (skipped_count > 0) {
          /* DDR-0007: a skipped record is visible, not silent */
          SONDE_LOG("Flash: skipped %lu corrupt record(s) (watermark advanced)\r\n",
                            (unsigned long)skipped_count);
        }
        
        if (flash_status == FLASH_LOG_OK && record_count > 0) {
          SONDE_LOG("Retrieved %lu unsent records from flash\r\n", record_count);
          
          // Convert flash records to high-res format for bulk packet
          /* F10 FIX: Failed conversion => skip that record entirely.
           * Previously the loop logged a warning and still packed the record
           * (zero-filled) — fabricated data transmitted as science.
           * A gap is honest; fabricated data is not. */
          HighResTelemetryRecord_t highres_records[6];
          uint8_t packed_count = 0;
          for (uint32_t i = 0; i < record_count && i < 6; i++) {
            if (!ConvertFlashLogToHighRes(&flash_records[i], &highres_records[packed_count],
                                         slope_mv_per_hour, current_mode)) {
              SONDE_LOG("Warning: Failed to convert flash record %lu - skipped\r\n", i);
              continue;  /* Skip bad record, keep packing the rest */
            }
            packed_count++;
          }

          if (packed_count == 0) {
            /* R21 (#51): nothing convertible. Retire any corrupt skips NOW so
             * a corrupt run can't wedge bulk transfer by being re-probed
             * forever; good records are left for the next cycle. */
            if (skipped_count > 0) {
              SONDE_LOG("Retiring %lu corrupt record(s) with no TX (anti-wedge)\r\n",
                                (unsigned long)skipped_count);
              FlashLog_MarkRecordsTransmitted(&hflashlog, skipped_count);
            }
            g_tx_state = TX_STATE_COMPLETE;
            break;
          }

          // F16 FIX: Send at SF7, resolved per-region (was hardcoded DR_3)
          LmHandlerSetTxDatarate(DatarateFromSF(7));  // SF7 in ANY region

          /* D3 (#33): wire v3 variable-length bulk (packet_type 0x03). Query the
           * runtime payload budget (current DR + pending FOpts, protocol §11)
           * and pack only complete records that fit. Records that don't fit
           * stay pending for the next cycle (stable identity, DDR-0011). */
          LoRaMacTxInfo_t txInfo;
          uint16_t max_payload = 0;
          for (uint8_t try_n = packed_count; try_n > 0; try_n--) {
            if (LoRaMacQueryTxPossible((uint8_t)(BULK_V3_OVERHEAD + try_n * sizeof(HighResTelemetryRecord_t)),
                                       &txInfo) == LORAMAC_STATUS_OK) {
              max_payload = (uint16_t)(BULK_V3_OVERHEAD + try_n * sizeof(HighResTelemetryRecord_t));
              break;
            }
          }
          if (max_payload == 0) {
            SONDE_LOG_STR("Bulk: no payload budget at current DR - retry next cycle\r\n");
            g_tx_state = TX_STATE_COMPLETE;
            break;
          }

          uint8_t v3_buf[BULK_V3_OVERHEAD + BULK_V3_MAX_RECORDS * sizeof(HighResTelemetryRecord_t)];
          uint8_t v3_packed = 0;
          uint16_t v3_len = 0;

          if (EncodeBulkPacketV3(v3_buf, sizeof(v3_buf), max_payload,
                                 highres_records, packed_count,
                                 flash_records[0].sequence,  /* DDR-0011 base identity */
                                 &v3_packed, &v3_len)) {

            /* DDR-0011 (#34): archive packets are CONFIRMED uplinks. The FIRST
             * archive packet of a burst carries LinkCheckReq (protocol §5.2);
             * records commit only on network ACK (OnTxData). */
            if (g_bulk_packets_sent == 0) {
              LmHandlerErrorStatus_t lc_status = LmHandlerLinkCheckReq();
              SONDE_LOG("LinkCheckReq on first archive packet: %d\r\n", lc_status);
            }

            LmHandlerAppData_t bulkData;
            bulkData.Port = LORAWAN_BULK_PORT;  // Port 11
            bulkData.BufferSize = v3_len;
            bulkData.Buffer = v3_buf;

            SONDE_LOG("Sending %u-byte bulk v4 packet at SF7 on port %d with %u records\r\n",
                              v3_len, LORAWAN_BULK_PORT, v3_packed);

            LmHandlerErrorStatus_t bulk_status = LmHandlerSend(&bulkData, LORAMAC_HANDLER_CONFIRMED_MSG, 0);
            
            if (bulk_status == LORAMAC_HANDLER_SUCCESS) {
              g_bulk_packets_sent++;
              
              /* F-005/R21 (#51): mark exactly what was CONSUMED from the
               * entry watermark — packed (sent) + skipped (corrupt) — so the
               * watermark lands exactly past the batch. D3 (#33): v3_packed is
               * what actually went on the air; records cut by the runtime
               * payload budget stay pending. The exact-identity ack lands with
               * the confirmed-delivery rework (#34). */
              if (v3_packed != record_count) {
                SONDE_LOG("WARN: packed %u of %lu read - marking only packed+skipped\r\n",
                                  v3_packed, (unsigned long)record_count);
              }
              g_bulk_pending_mark = v3_packed + skipped_count;
              
              SONDE_LOG("Bulk packet sent successfully! (%d/%d packets sent)\r\n",
                                g_bulk_packets_sent, MAX_BULK_PACKETS_PER_CYCLE);
              
              // Continue bulk transfer if more data available and under packet limit
              if (FlashLog_HasUnsentData(&hflashlog) && g_bulk_packets_sent < MAX_BULK_PACKETS_PER_CYCLE) {
                SONDE_LOG_STR("More unsent data available, continuing bulk transfer...\r\n");
                // Stay in TX_STATE_BULK_TRANSFER for next packet
              } else {
                SONDE_LOG_STR("Bulk transfer complete (no more data or packet limit reached)\r\n");
                g_tx_state = TX_STATE_COMPLETE;
              }
              
            } else {
              SONDE_LOG("Bulk packet send failed (status: %d)\r\n", bulk_status);
              g_tx_state = TX_STATE_COMPLETE;  // Complete on error
            }
            
          } else {
            SONDE_LOG_STR("ERROR: Failed to encode bulk packet!\r\n");
            g_tx_state = TX_STATE_COMPLETE;
          }
          
        } else {
          SONDE_LOG("No unsent records available (status: %d)\r\n", flash_status);
          g_tx_state = TX_STATE_COMPLETE;
        }
        
      } else {
        SONDE_LOG_STR("Bulk transfer complete: no data or packet limit reached\r\n");
        g_tx_state = TX_STATE_COMPLETE;
      }
      break;
    }
      
    case TX_STATE_COMPLETE:
    default:
      // Reset for next cycle
      g_tx_state = TX_STATE_PROBE_SF10;
      g_bulk_packets_sent = 0;
      SONDE_LOG_STR("Transmission cycle complete, reset to PROBE_SF10\r\n");
      break;
  }

  /* ========== LEGACY: Also send CayenneLPP for debug (during development) ========== */
  #if ENABLE_DEBUG_LPP
  static uint32_t tx_count = 0;
  tx_count++;
  
  if ((tx_count % DEBUG_LPP_TX_INTERVAL) == 0) {  // Every 5th transmission
    SONDE_LOG("Debug: Sending CayenneLPP packet (every %dth TX)\r\n", DEBUG_LPP_TX_INTERVAL);
    
    // Prepare and send the CayenneLPP packet
    LmHandlerAppData_t lppData;
    lppData.Port = LORAWAN_USER_APP_PORT;  // Port 2
    lppData.BufferSize = CayenneLppGetSize();
    lppData.Buffer = CayenneLppGetBuffer();
    
    // DEBUG: Log payload size
    char size_msg[64];
    snprintf(size_msg, sizeof(size_msg), "CayenneLPP payload size: %d bytes\r\n", lppData.BufferSize);
    SONDE_LOG_STR(size_msg);
    
    // Send with default datarate  
    LmHandlerSetTxDatarate(LORAWAN_DEFAULT_DATA_RATE);
    LmHandlerErrorStatus_t lpp_status = LmHandlerSend(&lppData, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0);
    
    SONDE_LOG("CayenneLPP send status: %d\r\n", lpp_status);
  } else {
    SONDE_LOG("Debug: Skipping CayenneLPP (TX count: %lu)\r\n", tx_count);
  }
  #endif
  
  /* ========== END ADAPTIVE TRANSMISSION STRATEGY ========== */
  
  /* ========== DEBUG: Queue detailed GNSS packet (compile-time controlled) ========== */
  #if ENABLE_GNSS_DETAIL_PACKET
  static uint32_t gnss_tx_count = 0;
  gnss_tx_count++;
  
  if (sensor_data.gnss_valid && (hgnss.extended.gps_count > 0 || hgnss.extended.beidou_count > 0) &&
      ((gnss_tx_count % DEBUG_LPP_TX_INTERVAL) == 0)) {  // Same interval as LPP debug
    
    SONDE_LOG("Debug: Sending GNSS detail packet (every %dth TX)\r\n", DEBUG_LPP_TX_INTERVAL);
    
    static uint8_t gnss_detail_buffer[150];  // Buffer for detailed GNSS packet
    uint16_t gnss_packet_size = EncodeGNSSDetailPacket(gnss_detail_buffer, sizeof(gnss_detail_buffer));
    
    if (gnss_packet_size > 0)
    {
      char gnss_msg[100];
      snprintf(gnss_msg, sizeof(gnss_msg), 
               "Queuing GNSS detail packet: %d bytes (GPS:%d BeiDou:%d GLONASS:%d)\r\n",
               gnss_packet_size, hgnss.extended.gps_count, 
               hgnss.extended.beidou_count, hgnss.extended.glonass_count);
      SONDE_LOG_STR(gnss_msg);
      
      /* Push to queue - will be sent after RX windows complete */
      if (PacketQueue_Push(&g_packet_queue, gnss_detail_buffer, gnss_packet_size, LORAWAN_GNSS_DETAIL_PORT))
      {
        char queue_msg[60];
        snprintf(queue_msg, sizeof(queue_msg), "GNSS packet queued (queue size: %d)\r\n", 
                 PacketQueue_Count(&g_packet_queue));
        SONDE_LOG_STR(queue_msg);
      }
      else
      {
        SONDE_LOG_STR("WARNING: Queue full - GNSS packet dropped!\r\n");
      }
    }
    else
    {
      SONDE_LOG_STR("GNSS detail packet encoding failed (0 bytes)\r\n");
    }
  }
  else
  {
    SONDE_LOG("Debug: Skipping GNSS detail packet (TX count: %lu)\r\n", gnss_tx_count);
  }
  #else
  SONDE_LOG_STR("GNSS detail packets disabled (ENABLE_GNSS_DETAIL_PACKET = 0)\r\n");
  #endif
  
  SONDE_LOG_STR("=== SendTxData END ===\r\n");
  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */
  SONDE_LOG_STR("\r\n*** OnTxTimerEvent FIRED ***\r\n");
  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */
  SONDE_LOG_STR("Timer restarted for next cycle\r\n");
  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */

/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  SONDE_LOG("\r\nOnTxData: Status=%d DR=%d Ch=%lu FCnt=%lu\r\n",
                    params->Status, params->Datarate,
                    (unsigned long)params->Channel, (unsigned long)params->UplinkCounter);
  
  /* CRITICAL: Capture context after successful TX (not before region switch) */
  /* This ensures correct DevAddr, FCnt, and session state are saved */
  if (params->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
    MultiRegion_SaveCurrentContext();
  }

  /* DDR-0011 (#34): delivery commit requires the NETWORK acknowledgement of a
   * confirmed uplink — never bare radio-TX completion (F-004/N-04). Records
   * without an ACK stay pending and are retransmitted later; the backend
   * deduplicates by (device, base_seq + i). */
  if (g_bulk_pending_mark > 0) {
    if (params->Status == LORAMAC_EVENT_INFO_STATUS_OK && params->AckReceived) {
      FlashLog_MarkRecordsTransmitted(&hflashlog, g_bulk_pending_mark);
      SONDE_LOG("OnTxData: ACK received — committed %lu archive records\r\n",
                        (unsigned long)g_bulk_pending_mark);
    } else {
      SONDE_LOG("OnTxData: no network ACK (status %d, ack %d) — %lu records stay PENDING\r\n",
                        params->Status, params->AckReceived,
                        (unsigned long)g_bulk_pending_mark);
    }
    g_bulk_pending_mark = 0;
  }

  /* DDR-0011 (#34): the confirmed probe heartbeat opens the archive opportunity.
   * No ACK -> stay in long-range mode, no archive probe (protocol §5.1/§15). */
  if (g_tx_state == TX_STATE_WAIT_PROBE_ACK) {
    if (params->Status == LORAMAC_EVENT_INFO_STATUS_OK && params->AckReceived) {
      uint16_t battery_mv = SYS_GetBatteryVoltage();
      bool battery_good = (battery_mv >= BULK_BATTERY_MIN_MV);
      bool has_cache = FlashLog_HasUnsentData(&hflashlog);
      SONDE_LOG("Probe ACK received — battery %dmV (%s), cache %s\r\n",
                        battery_mv, battery_good ? "GOOD" : "LOW",
                        has_cache ? "HAS_DATA" : "NO_DATA");
      if (battery_good && has_cache) {
        SONDE_LOG_STR("Archive opportunity OPEN — first archive probe\r\n");
        g_tx_state = TX_STATE_BULK_TRANSFER;
        g_bulk_packets_sent = 0;
        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
      } else {
        g_tx_state = TX_STATE_COMPLETE;
      }
    } else {
      SONDE_LOG("Probe heartbeat NOT acknowledged (status %d, ack %d) — no archive opportunity\r\n",
                        params->Status, params->AckReceived);
      g_tx_state = TX_STATE_COMPLETE;
    }
  }

  /* DDR-0011 (#34): burst continues only while archive packets are ACKed
   * (protocol §5.3). First archive packet unACKed -> immediate fallback. */
  if (g_tx_state == TX_STATE_BULK_TRANSFER &&
      g_bulk_packets_sent >= 1 && !params->AckReceived) {
    SONDE_LOG_STR("Archive packet unACKed — FALLBACK to heartbeat mode\r\n");
    g_tx_state = TX_STATE_COMPLETE;
    g_bulk_packets_sent = 0;
  }
  
  /* Drain packet queue after RX windows complete */
  /* This callback fires AFTER RX2 window closes, so MAC is ready for next TX */
  if (!PacketQueue_IsEmpty(&g_packet_queue))
  {
    PacketQueueEntry_t entry;
    if (PacketQueue_Pop(&g_packet_queue, &entry))
    {
      LmHandlerAppData_t queuedData;
      queuedData.Port = entry.port;
      queuedData.BufferSize = entry.size;
      queuedData.Buffer = entry.buffer;
      
      LmHandlerErrorStatus_t queue_status = LmHandlerSend(&queuedData, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0);
      SONDE_LOG("OnTxData: Queued packet (port %d, %d bytes) send status: %d (queue remaining: %d)\r\n",
                        entry.port, entry.size, queue_status, PacketQueue_Count(&g_packet_queue));
    }
  }
  
  /* C7b FIX: Continue bulk transfer if active - re-arm send task for next packet.
   * After TX+RX windows complete, if we're still in BULK_TRANSFER with data remaining,
   * schedule another SendTxData call to send the next bulk packet immediately. */
  if (g_tx_state == TX_STATE_BULK_TRANSFER && 
      FlashLog_HasUnsentData(&hflashlog) && 
      g_bulk_packets_sent < MAX_BULK_PACKETS_PER_CYCLE) {
    SONDE_LOG_STR("OnTxData: Re-arming bulk transfer (next packet)...\r\n");
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
  } else if (g_tx_state == TX_STATE_BULK_TRANSFER) {
    SONDE_LOG_STR("OnTxData: Bulk transfer complete\r\n");
    g_tx_state = TX_STATE_COMPLETE;
    g_bulk_packets_sent = 0;
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  char rtt_buf[128];
  
  SONDE_LOG_STR("\r\n=== OnJoinRequest Callback ===\r\n");
  snprintf(rtt_buf, sizeof(rtt_buf), "  Status: %s (%d)\r\n", 
           (joinParams->Status == LORAMAC_HANDLER_SUCCESS) ? "SUCCESS" : "FAILED",
           joinParams->Status);
  SONDE_LOG_STR(rtt_buf);
  
  snprintf(rtt_buf, sizeof(rtt_buf), "  Mode: %s\r\n", 
           (joinParams->Mode == ACTIVATION_TYPE_OTAA) ? "OTAA" : "ABP");
  SONDE_LOG_STR(rtt_buf);
  
  snprintf(rtt_buf, sizeof(rtt_buf), "  Datarate: DR%d, TxPower: %d\r\n", 
           joinParams->Datarate, joinParams->TxPower);
  SONDE_LOG_STR(rtt_buf);
  
  if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
  {
    SONDE_LOG_STR("JOIN SUCCESS!\r\n");
    
    /* Set flag for multi-region pre-join */
    g_multiregion_join_success = true;
    
    /* ONLY start timer if NOT in pre-join mode */
    if (g_multiregion_in_prejoin) {
      SONDE_LOG_STR("Pre-join mode: Skipping Tx timer start\r\n");
    } else {
      /* Start the Tx timer now that we're joined */
      if (EventType == TX_ON_TIMER)
      {
        SONDE_LOG_STR("Restarting Tx timer after rejoin...\r\n");
        UTIL_TIMER_Stop(&TxTimer);   // Stop existing timer first to prevent corruption
        UTIL_TIMER_Start(&TxTimer);  // Now safe to restart
      }
    }
  }
  else
  {
    SONDE_LOG_STR("JOIN FAILED - will retry on next timer event\r\n");
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
  /* NOTE: Do NOT refresh the watchdog here - this runs in IRQ context and
   * refreshing from an interrupt would defeat the watchdog's purpose
   * (a hung main loop with live radio IRQs would never reset) */
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

/* F24 FIX: StopJoin() and OnStopJoinTimerEvent() deleted. The SOS button EXTI
 * was dead (PB3 reconfigured to analog for solar), and the OTAA<->ABP flip
 * plus unconditional rejoin would be dangerous if ever triggered in flight. */

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

/* F-016 (#54): two-slot transactional NVM context persistence, matching the
 * proven Tier-2 ping-pong pattern (T1/FW-1). Slot A = page 126, slot B =
 * page 127 (the retired legacy store, repurposed). Each slot carries
 * magic/length/generation/CRC; newest valid generation wins. A torn write
 * can only kill the slot being written — the other survives. */
#define NVM_SLOT_MAGIC    0x4E564D43UL  /* "NVMC" */
#define NVM_SLOT_A_ADDR   LORAWAN_NVM_BASE_ADDRESS
#define NVM_SLOT_B_ADDR   ((void *)((uint32_t)LORAWAN_NVM_BASE_ADDRESS + FLASH_PAGE_SIZE))

typedef struct {
  uint32_t magic;
  uint32_t generation;
  uint32_t length;
  uint32_t crc32;      /* over the payload only */
} NvmSlotHeader_t;

static uint32_t g_nvm_generation = 0;

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */

  /* USER CODE END OnStoreContextRequest_1 */
  if (nvm == NULL || nvm_size == 0 ||
      nvm_size + sizeof(NvmSlotHeader_t) > FLASH_PAGE_SIZE) {
    SONDE_LOG("NVM store REJECTED (size %lu too large or bad ptr)\r\n",
                      (unsigned long)nvm_size);
    return;  /* honest failure, no silent drop */
  }

  /* Ping-pong: write the OTHER slot with the next generation */
  uint32_t slot_addr = (g_nvm_generation % 2 == 0) ? (uint32_t)NVM_SLOT_A_ADDR
                                                   : (uint32_t)NVM_SLOT_B_ADDR;
  NvmSlotHeader_t hdr;
  hdr.magic = NVM_SLOT_MAGIC;
  hdr.generation = g_nvm_generation + 1;
  hdr.length = nvm_size;
  hdr.crc32 = FlashLog_CRC32((const uint8_t *)nvm, nvm_size);

  if (FLASH_IF_Erase((void *)slot_addr, FLASH_PAGE_SIZE) != FLASH_IF_OK) {
    SONDE_LOG_STR("NVM store: slot erase FAILED\r\n");
    return;
  }
  if (FLASH_IF_Write((void *)slot_addr, &hdr, sizeof(hdr)) != FLASH_IF_OK ||
      FLASH_IF_Write((void *)(slot_addr + sizeof(hdr)), nvm, nvm_size) != FLASH_IF_OK) {
    SONDE_LOG_STR("NVM store: slot write FAILED\r\n");
    return;
  }
  g_nvm_generation = hdr.generation;
  /* USER CODE BEGIN OnStoreContextRequest_Last */

  /* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnRestoreContextRequest_1 */

  /* USER CODE END OnRestoreContextRequest_1 */
  /* Newest valid slot wins; validate magic + length + payload CRC */
  const uint32_t slots[2] = { (uint32_t)NVM_SLOT_A_ADDR, (uint32_t)NVM_SLOT_B_ADDR };
  int best = -1;
  NvmSlotHeader_t best_hdr = {0};
  for (int i = 0; i < 2; i++) {
    NvmSlotHeader_t hdr;
    if (FLASH_IF_Read(&hdr, (void *)slots[i], sizeof(hdr)) != FLASH_IF_OK) continue;
    if (hdr.magic != NVM_SLOT_MAGIC) continue;
    if (hdr.length != nvm_size) continue;
    if (hdr.length + sizeof(hdr) > FLASH_PAGE_SIZE) continue;
    if (best >= 0 && (int32_t)(hdr.generation - best_hdr.generation) <= 0) continue;
    best = i;
    best_hdr = hdr;
  }
  if (best < 0) {
    SONDE_LOG_STR("NVM restore: no valid slot (fresh start)\r\n");
    return;  /* leave nvm untouched — MAC treats as no context */
  }
  if (FLASH_IF_Read(nvm, (void *)(slots[best] + sizeof(NvmSlotHeader_t)), nvm_size) != FLASH_IF_OK) {
    SONDE_LOG_STR("NVM restore: payload read FAILED\r\n");
    return;
  }
  if (FlashLog_CRC32((const uint8_t *)nvm, nvm_size) != best_hdr.crc32) {
    SONDE_LOG_STR("NVM restore: payload CRC FAILED\r\n");
    return;
  }
  g_nvm_generation = best_hdr.generation;
  /* USER CODE BEGIN OnRestoreContextRequest_Last */

  /* USER CODE END OnRestoreContextRequest_Last */
}

/**
  * @brief  Initialize packet queue
  * @param  queue: Pointer to queue structure
  * @retval None
  */
static void PacketQueue_Init(PacketQueue_t *queue)
{
  memset(queue, 0, sizeof(PacketQueue_t));
}

/**
  * @brief  Push packet to queue
  * @param  queue: Pointer to queue structure
  * @param  data: Packet data
  * @param  size: Packet size
  * @param  port: LoRaWAN port
  * @retval true if successful, false if queue full
  */
static bool PacketQueue_Push(PacketQueue_t *queue, const uint8_t *data, uint16_t size, uint8_t port)
{
  if (queue->count >= PACKET_QUEUE_SIZE || size > sizeof(queue->entries[0].buffer))
    return false;

  PacketQueueEntry_t *entry = &queue->entries[queue->head];
  memcpy(entry->buffer, data, size);
  entry->size = size;
  entry->port = port;
  entry->valid = true;

  queue->head = (queue->head + 1) % PACKET_QUEUE_SIZE;
  queue->count++;

  return true;
}

/**
  * @brief  Pop packet from queue
  * @param  queue: Pointer to queue structure
  * @param  entry: Destination for popped entry
  * @retval true if successful, false if queue empty
  */
static bool PacketQueue_Pop(PacketQueue_t *queue, PacketQueueEntry_t *entry)
{
  if (queue->count == 0)
    return false;

  *entry = queue->entries[queue->tail];
  queue->entries[queue->tail].valid = false;

  queue->tail = (queue->tail + 1) % PACKET_QUEUE_SIZE;
  queue->count--;

  return true;
}

/**
  * @brief  Check if queue is empty
  * @param  queue: Pointer to queue structure
  * @retval true if empty
  */
static bool PacketQueue_IsEmpty(PacketQueue_t *queue)
{
  return (queue->count == 0);
}

/**
  * @brief  Get number of packets in queue
  * @param  queue: Pointer to queue structure
  * @retval Number of packets
  */
static uint8_t PacketQueue_Count(PacketQueue_t *queue)
{
  return queue->count;
}
