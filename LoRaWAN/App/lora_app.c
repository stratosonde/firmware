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
#include "sys_caps.h"  /* F-01 (#245): SYS_CAP_RADIO degrade marking */
#include "timer_if.h"
#include "payload_format.h"
#include "flash_log.h"
#include "config.h"
#include "mission_state.h"
#include "reset_cause.h"      /* F13a: deadman breadcrumb register */
#include "stm32_systime.h"    /* F12: SysTimeSet (DDR-0013) */
#include "transmit_plan.h"    /* R47 (#44): DecideTransmitPlan */
#include "RegionUS915.h"      /* R03 (#32): region Datarates*[] tables for the SF resolver */
#include "RegionEU868.h"
#include "RegionAS923.h"
#include "RegionAU915.h"
#include "RegionIN865.h"      /* R7 (#193): all six advertised regions need a complete SF mapping */
#include "RegionKR920.h"
#include "RegionRU864.h"      /* SP-05 (#246): seventh compiled region needs its Datarates table too */
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
#if ENABLE_GNSS_DETAIL_PACKET
static uint16_t EncodeGNSSDetailPacket(uint8_t *buffer, uint16_t max_size);
static bool PacketQueue_Push(PacketQueue_t *queue, const uint8_t *data, uint16_t size, uint8_t port);
#endif
static void PacketQueue_Init(PacketQueue_t *queue);
static bool PacketQueue_Peek(PacketQueue_t *queue, PacketQueueEntry_t *entry);
static bool PacketQueue_Pop(PacketQueue_t *queue, PacketQueueEntry_t *entry);
static bool PacketQueue_IsEmpty(PacketQueue_t *queue);
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
/* LT-07 (#277): burst-scoped deadline state (HAL_GetTick ms, the same time
 * base UTIL_TIMER uses; 0 = not open). BURST_MAX_OPEN_MS bounds how long the
 * FSM may sit in a network-wait state (probe ACK or burst train) without
 * progress. It is a BOUND, not a measurement: a confirmed probe is answered
 * inside RX1/RX2 (~1-2 s) and a full packet train (<= CfgMaxBulkPkts()
 * packets incl. RX windows) completes in tens of seconds; 60 s never
 * exceeds the science interval of any mode in which a burst can open
 * (bursts are ASCENT-inhibited, R3-03; the shortest burst-eligible interval
 * is MODE_NORMAL's 300 s). */
#define BURST_MAX_OPEN_MS  60000U
static uint32_t g_burst_opened_ms = 0;
static uint32_t g_probe_sent_ms = 0;
/* R3-04 (#218): g_bulk_commit_through DELETED - DDR-0005 one-pass recovery
 * advances the watermark AT SEND TIME (FlashLog_MarkRecoverySent); there is
 * no commit-on-ACK and no autonomous retry (BR-TX-009/010/011). */

/* R3-01 (#215): ABSOLUTE science deadline, HAL_GetTick ms domain (the same
 * time base UTIL_TIMER uses). Bulk continuation re-arms the SAME SendTxData
 * task (OnTxData/OnRxData); restarting TxTimer relative to each invocation
 * pushed the next science deadline out by a full interval per bulk packet -
 * unbounded starvation with a large backlog, violating DDR-0005
 * BR-TX-001/002 (current science always wins; recovery yields when science
 * falls due). The deadline advances ONLY when a science cycle is serviced,
 * phase-preserving (due += interval); bulk work can never move it. */
static uint32_t g_science_due_ms = 0;
/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/**
 * @brief Reinitialize the entire LoRaWAN stack for a new region
 * @note This performs DeInit/Init cycle to clear ALL state between region joins
 * @note Caller must set DevEUI and call LmHandlerConfigure() after this returns
 */
LmHandlerErrorStatus_t LoRaApp_ReInitStack(LoRaMacRegion_t new_region)
{
  SONDE_LOG_STR("LoRaApp_ReInitStack: Starting full stack reset...\r\n");

  /* F-01 (#245): every lifecycle return checked, fail closed. A mid-flight
   * region switch whose teardown/rebuild failed must NOT report success -
   * the caller would mark the region active on a torn stack and transmit-
   * retry against dead state for the rest of the flight. */
  if (LmHandlerHalt() != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("LoRaApp_ReInitStack: Halt FAILED - aborting reinit\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  HAL_Delay(100);

  // Complete teardown
  SONDE_LOG_STR("LoRaApp_ReInitStack: Calling LmHandlerDeInit...\r\n");
  if (LmHandlerDeInit() != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("LoRaApp_ReInitStack: DeInit FAILED - aborting reinit\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  HAL_Delay(200);

  // Rebuild from scratch
  SONDE_LOG_STR("LoRaApp_ReInitStack: Calling LmHandlerInit...\r\n");
  if (LmHandlerInit(&LmHandlerCallbacks, APP_VERSION) != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("LoRaApp_ReInitStack: Init FAILED - stack unusable\r\n");
    return LORAMAC_HANDLER_ERROR;
  }
  HAL_Delay(100);

  // Set region parameter but DON'T configure yet
  // Configuration must be done AFTER DevEUI is set by caller
  LmHandlerParams.ActiveRegion = new_region;

  SONDE_LOG_STR("LoRaApp_ReInitStack: Stack reset complete (region set, not configured)\r\n");
  return LORAMAC_HANDLER_SUCCESS;
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

  /* Init the Lora Stack
   * F-01 (#245): a boot-time lifecycle failure leaves the radio unusable for
   * the whole mission. Degrade-and-say-so (DDR-0009): mark the capability
   * and keep flying - science acquisition + archive logging proceed; the TX
   * paths fail visibly instead of pretending the stack is up. */
  if (LmHandlerInit(&LmHandlerCallbacks, APP_VERSION) != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("LoRaWAN_Init: LmHandlerInit FAILED - marking radio unavailable\r\n");
    SysCaps_MarkFailed(SYS_CAP_RADIO);
  }

  if (LmHandlerConfigure(&LmHandlerParams) != LORAMAC_HANDLER_SUCCESS) {
    SONDE_LOG_STR("LoRaWAN_Init: LmHandlerConfigure FAILED - marking radio unavailable\r\n");
    SysCaps_MarkFailed(SYS_CAP_RADIO);
  }

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  
  /* Initialize Multi-region context manager */
  MultiRegion_Init();
  APP_LOG(TS_ON, VLEVEL_H, "Multi-region context manager initialized\r\n");

  /* T3 (DDR-0002): decide mission state now that the session bank is loaded —
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
  
  /* F22 FIX (DDR-0002): GNSS reconfiguration is COMMISSIONING-ONLY.
   * PCAS03/04/05/11 are saved to the GNSS module's internal flash via PCAS00 —
   * writing that flash on every boot wears it for zero benefit. */
  if (MissionState_IsCommissioning()) {
    SONDE_LOG_STR("*** COMMISSIONING: Reconfiguring GPS module (all constellations) ***\r\n");
    GNSS_PowerOn(&hgnss);
    HAL_Delay(1000);  // Let GPS boot
    /* F2 (#168): the return value is now meaningful — a failed configure must
     * be loud, not a warning line above a "success" banner. */
    if (GNSS_Configure(&hgnss) != GNSS_OK) {  // PCAS04,7 + PCAS11 airborne + PCAS00
      SONDE_LOG_STR("*** COMMISSIONING: GNSS CONFIGURE FAILED - do not launch ***\r\n");
    } else {
      /* R3-06 (#220): honest claim. The old unconditional "reconfigured and
       * saved to flash" banner printed even on failure AND claimed evidence
       * no one collected - what holds is transmitted + whatever the
       * receiver-side verification inside GNSS_Configure proved. */
      SONDE_LOG_STR("*** GNSS config commands transmitted (see receiver-side verification report above) ***\r\n\r\n");
    }
    HAL_Delay(500);   // Let GPS save to flash
    GNSS_PowerOff(&hgnss);
  }
  
  /* Auto-detect provision state: resume any valid saved session.
   * Note: ForceRejoin will have cleared contexts above, so this will go to OTAA path */
  /* R5 (#191): do NOT anchor on US915 - a unit commissioned (or last flown)
   * on another region has a valid session bank that a US915-only probe
   * cannot see, and it would fall through to "no session" RF silence (or a
   * pointless re-provision) despite holding good credentials. Prefer the
   * persisted active region (restored from the flash bank); fall back to a
   * scan of every known region. */
  LoRaMacRegion_t resume_region = MultiRegion_GetActiveRegion();
  bool have_session = MultiRegion_IsRegionJoined(resume_region);
  if (!have_session) {
    static const LoRaMacRegion_t scan_regions[] = {
      LORAMAC_REGION_US915, LORAMAC_REGION_EU868, LORAMAC_REGION_AS923,
      LORAMAC_REGION_AU915, LORAMAC_REGION_IN865, LORAMAC_REGION_KR920,
      LORAMAC_REGION_RU864   /* SP-05 (#246): seventh bank */
    };
    for (uint8_t i = 0; i < sizeof(scan_regions)/sizeof(scan_regions[0]); i++) {
      if (MultiRegion_IsRegionJoined(scan_regions[i])) {
        resume_region = scan_regions[i];
        have_session = true;
        break;
      }
    }
  }
  if (have_session) {

    /* Already provisioned - use saved ABP context from flash */
    SONDE_LOG("Found valid ABP context - using saved session (%s)\r\n", RegionToString(resume_region));
    APP_LOG(TS_ON, VLEVEL_H, "Using saved ABP context\r\n");

    /* Switch to the resumed region as starting region */
    MultiRegion_SwitchToRegion(resume_region);
    
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
       * the gitignored se-identity.h (F6/DDR-0018 — never committed). Define
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
        /* R6 (#192): provisioning is a transport detail - it must NOT enter
         * flight. DDR-0002: flight entry is explicit (arming button hook /
         * BR-LIFE-007 launch detection), never join/provisioning-triggered.
         * The OTAA path below already holds COMMISSIONING; the ABP path now
         * matches it. */
        SONDE_LOG_STR("PROVISIONING_BUILD: complete - COMMISSIONING, awaiting arming/launch\r\n");
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
      /* T1 ladder (DDR-0018), rung 3: FLIGHT with a virgin session bank means
       * RF silence. Keep flying the profile — GPS, flash logging, timers —
       * but never attempt a join. */
      APP_LOG(TS_ON, VLEVEL_H, "FLIGHT: no valid session bank - RF silence, logging only\r\n");
      SONDE_LOG_STR("FLIGHT MODE with no saved session: RF SILENCE (DDR-0018)\r\n");
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
    /* R7 (#193) + SP-05 (#246): the firmware advertises SEVEN regions - the
     * resolver must not drop IN865/KR920/RU864 into the default fallback
     * (LORAWAN_DEFAULT_DATA_RATE is not necessarily valid for the active
     * region; the MAC rejects the uplink). Seven advertised, seven mapped. */
    case LORAMAC_REGION_IN865:
      table = DataratesIN865; table_len = sizeof(DataratesIN865); break;
    case LORAMAC_REGION_KR920:
      table = DataratesKR920; table_len = sizeof(DataratesKR920); break;
    case LORAMAC_REGION_RU864:
      table = DataratesRU864; table_len = sizeof(DataratesRU864); break;
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

#if ENABLE_GNSS_DETAIL_PACKET  /* FR-19 (#100): unused when the debug packet is off */
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
#endif  /* ENABLE_GNSS_DETAIL_PACKET */

/* ========== VOLTAGE-BASED PREDICTIVE POWER MANAGEMENT ========== */
/* R49 (#46): NormalizeBatteryVoltage, CalculateVoltageSlope, PredictTimeToVoltage,
 * SelectModeFromPredictions and GetModeName moved verbatim to Core/Src/power_model.c
 * (declared in Core/Inc/power_model.h) so the pure decision logic is host-testable.
 * R47 (#44): ApplyOperatingMode + the whole decide block moved to
 * Core/Src/transmit_plan.c (DecideTransmitPlan). */

/* USER CODE END PrFD */

/* 2026-08-11 handoff §6b: the bulk-gate knobs are live SystemConfig_t fields;
 * the lora_app.h macros are the defaults when config is unavailable
 * (Config_Get() returns NULL before Config_Init). Single accessor point so a
 * future rename/repack touches one place. */
/* #141 / F-1 (#176): UTC epoch of the last FRESH GNSS fix (0 = none), from
 * SysTimeGet().Seconds - the GPS-disciplined clock, NOT boot-relative
 * TIMER_IF_GetTime. Set in AcquireGnssFix AFTER SysTimeSyncFromGnss, where
 * live NMEA data confirmed a fix - the last-known-position fallback does NOT
 * count. The UTC base is what lets the value persist (BKP_REG_LASTPOS_EPOCH)
 * and restore across resets: a boot-relative stamp can never pass the UTC
 * plausibility gate, which is exactly the F-1 defect this comment used to
 * document as if it were a contract. */
static uint32_t s_last_fresh_fix_s = 0;
/* STAB-01 (#148): the GPS-loss grace epoch also persists (BKP_REG_GPS_LOSS_EPOCH)
 * so a reset cannot restart the 6 h silence window. */
static uint32_t s_gps_loss_epoch_s = 0;

/* F-1 (#176): plausibility floor for a real UTC epoch (2023-11-14). Anything
 * smaller is boot-relative or corrupt. NOT lowered to admit boot-relative
 * values - the fix is to stamp in the correct time base, not to widen the
 * gate until it hides the type confusion. */
#define UTC_EPOCH_PLAUSIBLE_MIN  1700000000UL

/* F-1 (#176): explicit "has the UTC clock ever been disciplined" guard.
 * Set this boot by SysTimeSyncFromGnss; seeded from the persisted SysTime at
 * first query - SysTime lives in RTC backup regs DR0-2, so a pre-reset GPS
 * discipline survives the reset and a plausible epoch at boot means the clock
 * is still trustworthy. Never infer sync from the magnitude of an arbitrary
 * timestamp; this is the single deliberate bridge between the two clocks. */
static bool s_utc_synced = false;

/* F-8 (#183): this cycle's single battery sample, captured in SendTxData
 * from sensor_data (where value + staleness flag are read together) so the
 * OnTxData bulk-opportunity gate uses the SAME conversion as everything
 * else instead of running its own. */
static uint16_t s_cycle_batt_mv = 0;

static bool UtcTimeIsValid(void)
{
  if (!s_utc_synced && SysTimeGet().Seconds >= UTC_EPOCH_PLAUSIBLE_MIN) {
    s_utc_synced = true;  /* discipline happened before the reset */
  }
  return s_utc_synced;
}

static uint8_t  CfgLinkMargin(void)  { const SystemConfig_t *c = Config_Get(); return c ? c->link_margin_threshold   : LINK_MARGIN_THRESHOLD; }
static uint8_t  CfgGatewayCount(void){ const SystemConfig_t *c = Config_Get(); return c ? c->gateway_count_threshold : GATEWAY_COUNT_THRESHOLD; }
static uint16_t CfgBulkBattMin(void) { const SystemConfig_t *c = Config_Get(); return c ? c->bulk_battery_min_mv     : BULK_BATTERY_MIN_MV; }
static uint8_t  CfgMaxBulkPkts(void) { const SystemConfig_t *c = Config_Get(); return c ? c->max_bulk_packets        : MAX_BULK_PACKETS_PER_CYCLE; }

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

  /* R2-06 (#110) / 2026-08-10 finding #2: the middleware LATCHES
   * RxParams.LinkCheck on the first MLME_LINK_CHECK and never clears it, so
   * without this consume-on-read every later burst would gate on a STALE
   * margin/gateway count (one good LinkCheckAns near launch would permanently
   * unlock SF7 bursts over a mid-ocean single-gateway link). Clear it here:
   * each burst decision now provably requires a FRESH LinkCheckAns. */
  params->LinkCheck = false;
  
  // FW-17: only log margin/gateway count when a LinkCheckAns was actually
  // received — otherwise these fields are garbage
  if (linkcheck_received) {
    SONDE_LOG("LinkCheckAns: Margin=%ddB, Gateways=%d\r\n", margin, gw_count);
  }
  
  /* DDR-0005 (#34): the archive opportunity opens on the confirmed probe's ACK
   * (OnTxData), not on LinkCheckAns. LinkCheckAns now rides the FIRST archive
   * packet (protocol §5.2) and gates burst continuation (§5.3): poor margin /
   * gateway count — or no LinkCheckAns at all — ends the burst after the
   * (already ACKed) first archive packet. */
  if (g_tx_state == TX_STATE_BULK_TRANSFER && g_bulk_packets_sent == 1) {
    bool link_good = (linkcheck_received &&
                      margin >= CfgLinkMargin() &&
                      gw_count >= CfgGatewayCount());
    SONDE_LOG("First archive response: LinkCheckAns %s, margin=%ddB (>=%d), gateways=%d (>=%d) -> %s\r\n",
                      linkcheck_received ? "received" : "MISSING",
                      margin, CfgLinkMargin(), gw_count, CfgGatewayCount(),
                      link_good ? "BURST CONTINUES" : "FALLBACK");
    if (!link_good) {
      /* Protocol §5.4: end the burst, return to LONG_RANGE_HEARTBEAT.
       * R3-04 (#218): the first packet's records were already marked sent
       * (one-pass advance at send time) — no loss, no retry. */
      g_tx_state = TX_STATE_COMPLETE;
      g_bulk_packets_sent = 0;
    } else {
      /* FR-14 (#88): continuation of the burst after packet 1 is owned HERE,
       * by the LinkCheck evaluation — OnTxData no longer re-arms for
       * g_bulk_packets_sent == 1 (it runs before this callback; the race
       * produced a spurious full extra work cycle). */
      if (FlashLog_HasUnsentData(&hflashlog) &&
          g_bulk_packets_sent < CfgMaxBulkPkts()) {
        SONDE_LOG_STR("OnRxData: link good - re-arming bulk transfer...\r\n");
        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
      } else {
        g_tx_state = TX_STATE_COMPLETE;
        g_bulk_packets_sent = 0;
      }
    }
  } else if (linkcheck_received) {
    // Link check result for non-archive transmissions (informational)
    SONDE_LOG("Link quality: margin=%ddB, gateways=%d\r\n", margin, gw_count);
  }
  
  // Process any received application data
  if (appData && appData->BufferSize > 0) {
    SONDE_LOG("Received %d bytes on port %d\r\n", appData->BufferSize, appData->Port);
  }
  
  SONDE_LOG_STR("=== OnRxData Callback END ===\r\n");
  /* USER CODE END OnRxData_1 */
}

/**
  * @brief  F13a (DDR-0020): progress deadman. SendTxData is the only place a
  *         full work cycle provably begins — mark RTC seconds here. If the
  *         sequencer/timer wedges (no cycle for 3x the worst-case interval),
  *         Deadman_Check breadcrumbs and resets. COMMISSIONING is exempt:
  *         on the bench a human can sit idle for hours legitimately.
  */
/* R01/R02: DR2 collided with timer_if SysTime MSBTICKS — moved to DR5.
 * Allocation map lives in backup_regs.h. */
#include "backup_regs.h"
#define DEADMAN_BKP_REG     BKP_REG_DEADMAN
/* S-04 (#228): the timeout is DERIVED from the configured survival cadence
 * (ConfigGetDeadmanTimeoutS: max(3 h, 3x survival)) - a fixed 3 h constant
 * gave a 1.5x margin at the validator's 2 h ceiling. */

static void Deadman_MarkProgress(void)
{
  extern RTC_HandleTypeDef hrtc;
  uint16_t ms_unused;
  HAL_RTCEx_BKUPWrite(&hrtc, DEADMAN_BKP_REG, TIMER_IF_GetTime(&ms_unused));
  /* F-6 (#181): boot-attempt evidence is NOT cleared here. Marking the
   * deadman at cycle ENTRY is liveness (DDR-0009); clearing boot attempts is
   * PROGRESS and belongs at successful cycle COMPLETION (end of SendTxData).
   * Cleared at entry, any deterministic fault inside a cycle reset the
   * counter on every boot and the FR-23 escape could never engage. */
}

/* F-15 (#72): last-valid position parked in backup registers so a reset does
 * not restore the compile-time default (Kansas) and feed a garbage position
 * into region selection via the deliberate F-06 stale-position hold. VALID is
 * written LAST so a mid-write reset never validates a partial triple. */
#define LASTPOS_VALID_MAGIC  0x4C415454UL  /* 'LATT' */
#define TS_WRAP_MAGIC        0x57524150UL  /* 'WRAP' (STAB-12/#159) */
static void LastPos_Store(float lat, float lon, float alt, uint32_t fix_epoch_s)
{
  extern RTC_HandleTypeDef hrtc;
  union { float f; uint32_t u; } la, lo, al;
  la.f = lat; lo.f = lon; al.f = alt;
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_LASTPOS_LAT, la.u);
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_LASTPOS_LON, lo.u);
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_LASTPOS_ALT, al.u);
  /* STAB-01 (#148): acquisition time persists WITH the position -
   * a reset must never make stale geography younger (DDR-0015). */
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_LASTPOS_EPOCH, fix_epoch_s);
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_LASTPOS_VALID, LASTPOS_VALID_MAGIC);
}

static bool LastPos_Load(float *lat, float *lon, float *alt)
{
  extern RTC_HandleTypeDef hrtc;
  if (HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_LASTPOS_VALID) != LASTPOS_VALID_MAGIC) {
    return false;  /* cold boot or backup wipe — caller applies the default */
  }
  union { float f; uint32_t u; } la, lo, al;
  la.u = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_LASTPOS_LAT);
  lo.u = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_LASTPOS_LON);
  al.u = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_LASTPOS_ALT);
  *lat = la.f; *lon = lo.f; *alt = al.f;
  return true;
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
  if ((now - last) > ConfigGetDeadmanTimeoutS()) {
    SONDE_LOG("DEADMAN: no work cycle for %lus - breadcrumb + reset\r\n",
                      (unsigned long)ConfigGetDeadmanTimeoutS());
    HAL_RTCEx_BKUPWrite(&hrtc, RESET_CAUSE_BKP_FAULT_REG,
                        RESET_CAUSE_FAULT_MAGIC | 6U);  /* 6 = deadman */
    /* F-05 (#63): re-seed BEFORE resetting — otherwise the post-reset boot
     * sees the same stale timestamp and fires again (deadman reset loop). */
    HAL_RTCEx_BKUPWrite(&hrtc, DEADMAN_BKP_REG, now);
    NVIC_SystemReset();
  }
}

/**
  * @brief  F12 (DDR-0013): discipline system time from a good GPS fix so flash
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
  /* #132: t==0 is a VALID time (00:00:00 UTC), not "no time".
   * Zero-as-sentinel skipped the sync once per 86400 fixes. Date validity is
   * fully covered by the range check below (d==0 -> day==0 -> rejected). */
  if (d == 0) return;

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
  s_utc_synced = true;  /* F-1 (#176): explicit sync event for UtcTimeIsValid */
  SONDE_LOG("SysTime disciplined from GPS: %lu epoch seconds\r\n",
                    (unsigned long)epoch);
}
/* =============================================================================
 * F-R1 (#74): SendTxData decomposed into phases. Pure code motion — every
 * hardware call, delay, log, and decision is byte-for-byte the code that used
 * to be inlined; only the surrounding braces changed. Deadman_MarkProgress()
 * remains the FIRST statement of SendTxData (DDR-0009) and the mission-state
 * early-returns stay top-level.
 * =============================================================================*/

/**
 * @brief F-R1 (#74): power-cycle the GNSS, acquire a fix within gps_timeout_ms,
 *        maintain last-known-good (F-15 backup regs; F8/T2 stale marking), then
 *        return the GPS to full power-off. Feeds the IWDG during acquisition.
 * @param gps_timeout_ms acquisition bound from the power plan
 * @param ttf_ms out: time-to-fix (0 if no fix / wake failed)
 * @retval true  GPS woke and the cycle ran (fix or documented fallback)
 * @retval false wake from standby failed — caller must skip region selection
 */
static bool AcquireGnssFix(uint32_t gps_timeout_ms, uint32_t *ttf_ms)
{
  #define GNSS_MIN_SATS_FOR_FIX    4      /* Minimum satellites needed for fix */
  
  /* Last known GPS position storage (persistent across transmission cycles) */
  /* F-15 (#72): on first use after boot, restore from backup registers if a
   * real fix was parked there; only a true cold boot falls back to the
   * Kansas default. */
  static float last_valid_lat, last_valid_lon, last_valid_alt;
  static bool have_previous_fix = false;
  static bool last_position_loaded = false;
  if (!last_position_loaded) {
    last_position_loaded = true;
    if (LastPos_Load(&last_valid_lat, &last_valid_lon, &last_valid_alt)) {
      have_previous_fix = true;
      SONDE_LOG_STR("GPS: last known position restored from backup regs\r\n");
      /* STAB-01 (#148): the restored position participates with its TRUE age
       * (DDR-0015). Plausibility-gated: a boot-relative or corrupt timestamp
       * is ignored - never invent freshness. */
      {
        extern RTC_HandleTypeDef hrtc;
        uint32_t fix_epoch = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_LASTPOS_EPOCH);
        /* F-1 (#176): compare UTC against UTC. SysTime persists in RTC backup
         * regs, so a pre-reset discipline makes this restore work even before
         * the first fix of this boot. Never synced -> now_s stays small ->
         * the gate rejects, which is the honest "age unknown" answer. */
        uint32_t now_s = SysTimeGet().Seconds;
        if (UtcTimeIsValid() &&
            fix_epoch >= UTC_EPOCH_PLAUSIBLE_MIN && fix_epoch <= now_s) {
          s_last_fresh_fix_s = fix_epoch;
        }
      }
    } else {
      last_valid_lat = 39.8283f;   /* Default: Central US (Kansas) */
      last_valid_lon = -98.5795f;
      last_valid_alt = 500.0f;
    }
  }
  
  /* gps_start local; ttf_ms is the out-param */
  uint32_t gps_start = 0;
  *ttf_ms = 0;  /* Will be updated when fix is obtained */
  
  SONDE_LOG("Waking GPS from standby for fix acquisition (%lus max)...\r\n", 
                    (unsigned long)(gps_timeout_ms / 1000));
  /* R3-02 (#216): set conservative provenance BEFORE attempting the wake.
   * If GNSS_WakeFromStandby fails we return immediately - previously the
   * previous cycle's hgnss.data survived AND s_gnss_stale kept its last
   * value (false after any good fix), so EnvSensors_MergeGnss archived the
   * OLD coordinates as FRESH: wrong-but-plausible data, worse than none.
   * Mark stale + invalidate first; the acquisition paths below re-clear
   * stale only on a REAL fix. Last-known-good still lives in the
   * last_valid_* statics. */
  EnvSensors_MarkGnssStale(true);
  /* CRITICAL: Invalidate old GPS data to force waiting for fresh NMEA
   * sentences. This prevents reusing data from the previous cycle (which
   * would give false 0ms TTF).
   * R31 (#57): FULL invalidation — the GGA parser skips empty tokens, so a
   * partial sentence must never meet last cycle's sats/hdop/lat/lon. */
  memset(&hgnss.data, 0, sizeof(hgnss.data));

  if (GNSS_WakeFromStandby(&hgnss) != GNSS_OK)
  {
    /* Data already invalidated + marked stale above: the science record
     * shows GNSS-unavailable (zeros + stale), never old-but-fresh. */
    SONDE_LOG_STR("GPS: Wake from standby failed!\r\n");
    return false;
  }

    SONDE_LOG_STR("GPS data invalidated - waiting for fresh fix (hot-start <5s)...\r\n");
    
    gps_start = HAL_GetTick();
    bool got_fix = false;
    uint32_t last_status_print = 0;

    /* F-2 (#177): clock-independent iteration bound - the W25Q_WaitReady
     * max_polls pattern (F3/#169). HAL_GetTick() is RTC-derived: if the RTC
     * stalls, the tick difference never grows and this loop - which refreshes
     * the IWDG from INSIDE - would wedge forever with the watchdog fed, and
     * RTC_LivenessCheck cannot rescue it (this is a sequencer task; the main
     * loop never regains control). The tick bound stays the normal exit;
     * this cap is the backstop that does not depend on the clock under
     * suspicion. 32/ms mirrors W25Q_MAX_BUSY_POLLS_PER_MS: far above any real
     * iteration rate (each pass sleeps in WFI until an interrupt). */
    uint32_t acq_iters = 0;
    const uint32_t max_iters = gps_timeout_ms * 32u;

    /* Process GPS data for up to gps_timeout_ms (dynamic based on power mode) */
    while ((HAL_GetTick() - gps_start) < gps_timeout_ms)
    {
      if (++acq_iters > max_iters) {
        SONDE_LOG_STR("GPS: acquisition iteration bound hit (clock-independent) - aborting\r\n");
        break;
      }

      /* Process DMA buffer - parses NMEA and updates hgnss.data */
      GNSS_ProcessDMABuffer(&hgnss);
      
      /* CRITICAL: Refresh watchdog during GPS acquisition to prevent timeout reset */
      /* GPS can take up to 60s, approaching the 32.76s watchdog timeout */
      /* Finding #9: HAL_GetTick() is 1024 Hz (RTC subseconds), not 1000 Hz —
       * every ms timeout here runs ~2.4% short (the "60 s" budget is really
       * 58.6 s). Always in the conservative direction; not worth rescaling. */
      HAL_IWDG_Refresh(&hiwdg);
      
      /* Check if we have a good quality fix */
      if (GNSS_IsFixGoodQuality(&hgnss))
      {
        got_fix = true;
        *ttf_ms = HAL_GetTick() - gps_start;  /* Capture TTF at moment of fix */
        
        /* Convert floats to integers for safe printf (no float support needed) */
        int32_t lat_int = (int32_t)(hgnss.data.latitude * 1000000);
        int32_t lon_int = (int32_t)(hgnss.data.longitude * 1000000);
        int32_t alt_int = (int32_t)(hgnss.data.altitude * 10);
        int32_t hdop_int = (int32_t)(hgnss.data.hdop * 10);
        (void)lat_int; (void)lon_int; (void)alt_int; (void)hdop_int;  /* FR-19: log-only in flight */

        SONDE_LOG("GPS FIX! Lat=%ld.%06ld Lon=%ld.%06ld Alt=%ld.%ldm Sats:%d HDOP=%ld.%ld (took %lums)\r\n",
                 (long)(lat_int / 1000000), (long)labs(lat_int % 1000000),
                 (long)(lon_int / 1000000), (long)labs(lon_int % 1000000),
                 (long)(alt_int / 10), (long)labs(alt_int % 10),
                 hgnss.data.satellites,
                 (long)(hdop_int / 10), (long)labs(hdop_int % 10),
                 (unsigned long)(*ttf_ms));  /* S-D (#214): was printing the POINTER */
        break;  /* Exit early - we have what we need */
      }
      
      /* Print status every 5 seconds during acquisition */
      uint32_t elapsed = HAL_GetTick() - gps_start;
      if (elapsed - last_status_print >= 5000)
      {
        /* F27 FIX: integer-only print (no float printf support linked) */
        int hdop_deci = (int)(hgnss.data.hdop * 10.0f);
        (void)hdop_deci;  /* FR-19: log-only in flight */
        SONDE_LOG("[GPS %lus] Sats:%d/%d HDOP:%d.%d Fix:%s\r\n",
                 (unsigned long)(elapsed / 1000),
                 hgnss.data.satellites, hgnss.data.satellites_in_view,
                 hdop_deci / 10, hdop_deci % 10,
                 (hgnss.data.fix_quality != GNSS_FIX_INVALID) ? "Yes" : "No");
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
      /* R2-16 (#120): GNSS_HasPosition, not IsFixValid - a partial
       * sentence can latch valid (RMC A) with (0,0) Null-Island fields.
       * Only a fix with real lat/lon tokens may become last-known-good. */
      if (GNSS_HasPosition(&hgnss))
      {
        SONDE_LOG_STR("GPS: Basic fix (not high quality)\r\n");
        /* Update last known position even for basic fix */
        last_valid_lat = hgnss.data.latitude;
        last_valid_lon = hgnss.data.longitude;
        last_valid_alt = hgnss.data.altitude;
        /* F-1 (#176): discipline the clock FIRST, then stamp in UTC. */
        SysTimeSyncFromGnss();
        s_last_fresh_fix_s = SysTimeGet().Seconds;  /* #141: UTC epoch */
        LastPos_Store(last_valid_lat, last_valid_lon, last_valid_alt,
                      s_last_fresh_fix_s);  /* F-15 (#72) + STAB-01 (#148) epoch */
        have_previous_fix = true;
        EnvSensors_MarkGnssStale(false);  /* F8/T2: fresh data, not stale */
      }
      else
      {
        /* GPS timeout - use last known position if available */
        if (have_previous_fix)
        {
          /* F8/T2 (DDR-0003): last-known-good position still flows, but the
           * GPS-stale bit is set so nothing downstream mistakes it for live. */
          SONDE_LOG_STR("GPS: Timeout - using last known position (STALE)\r\n");
          hgnss.data.latitude = last_valid_lat;
          hgnss.data.longitude = last_valid_lon;
          hgnss.data.altitude = last_valid_alt;
          hgnss.data.valid = true;  /* Mark as valid to proceed with transmission */
          hgnss.data.fix_quality = GNSS_FIX_GPS;  /* Indicate GPS fix type */
          hgnss.data.position_present = true;  /* R2-16 (#120): restored last-known IS a real (stale) position */
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
      /* Successful fix - update last known position.
       * R2-16 (#120): IsFixGoodQuality range-checks but (0,0) passes by
       * design (R32) - require token presence before persisting. */
      if (GNSS_HasPosition(&hgnss))
      {
        last_valid_lat = hgnss.data.latitude;
        last_valid_lon = hgnss.data.longitude;
        last_valid_alt = hgnss.data.altitude;
        /* F-1 (#176): discipline the clock FIRST, then stamp in UTC. */
        SysTimeSyncFromGnss();
        s_last_fresh_fix_s = SysTimeGet().Seconds;  /* #141: UTC epoch */
        LastPos_Store(last_valid_lat, last_valid_lon, last_valid_alt,
                      s_last_fresh_fix_s);  /* F-15 (#72) + STAB-01 (#148) epoch */
        have_previous_fix = true;
      }
      EnvSensors_MarkGnssStale(false);  /* F8/T2: fresh fix, clear stale */
      /* DR-13: the duplicate SysTimeSyncFromGnss() here is deleted - the F-1
       * (#176) ordering discipline ("discipline the clock FIRST, then stamp
       * in UTC") lives inside the GNSS_HasPosition block above; the second
       * call ran even when the position gate had just rejected the fix. */
      SONDE_LOG_STR("GPS: Fix acquired and stored as last known position\r\n");
    }
    
    /* Put GPS back to full power-off (0µA) and allow MCU to sleep */
    GNSS_EnterStandby(&hgnss);
    SONDE_LOG_STR("GPS fully powered off (0µA), MCU can now sleep\r\n");
  return true;
}

/**
 * @brief F-R1 (#74): H3 region lookup + auto-switch for the current position.
 *        F-06/DDR-0015: deliberately runs on possibly-stale last-known
 *        position (see comment inline). REGION_RESTRICTED sets *rf_silence;
 *        REGION_UNKNOWN keeps the current region and transmits normally
 *        (agreed maritime convention, path-forward 1.3 / DDR-0001 open
 *        product decision — blocking UNKNOWN darkened every ocean crossing).
 *
 * F10 (#175) — THE stale-position RF policy, stated once: a stale position
 * may INHIBIT but never SWITCH. Full region selection (including
 * auto-switch) runs only on a FRESH fix; the stale-position geofence can
 * silence (REGION_RESTRICTED) but can never change region or newly authorize
 * transmission. Anything else reads as a contradiction with the inhibit-only
 * rule and must be fixed here first.
 */
/* BURST-03 (#141): single geofence-policy point so the GPS-skip path runs the
 * SAME restricted-region test as the live-fix path. Pure computation (h3lite),
 * no hardware. Inhibit only — callers on a stale position must NOT auto-switch
 * regions (switching on a days-old fix may be worse than holding). */
/* DR-02 (#237): tri-state, not boolean. An implausible last-known position is
 * UNKNOWN, not "permitted" - the old `return false` let a validation failure
 * silently read as permission on the one policy with regulatory consequences.
 * Callers map UNKNOWN to the documented "no fix ever -> transmit" policy
 * EXPLICITLY (same outcome, different reasoning - F10/#175, DDR-0015). */
typedef enum {
  GEO_PERMISSION_PERMITTED = 0,
  GEO_PERMISSION_RESTRICTED,
  GEO_PERMISSION_UNKNOWN
} GeoPermission_t;

static GeoPermission_t GeofenceRestricted(float lat, float lon)
{
  if (!GNSS_ValidateCoordinates(lat, lon)) {
    return GEO_PERMISSION_UNKNOWN;
  }
  return (latLngToRegion(lat, lon) == REGION_RESTRICTED)
         ? GEO_PERMISSION_RESTRICTED : GEO_PERMISSION_PERMITTED;
}

/* DR-06 (#241): ONE silence helper for every RF-silence path, so the archive
 * always records WHY a cycle went dark (DDR-0003 §6a), not just THAT. First
 * veto wins (the existing DecideTransmitPlan rule). Previously only the
 * no-session veto reached plan.veto; the restricted-region, GPS-loss and
 * pre-launch quiet-watch paths archived as VETO_NONE - indistinguishable
 * from a normal cycle on recovery. */
static void Silence(TransmitPlan_t *plan, bool *rf_silence, TransmitVeto_t why)
{
  *rf_silence = true;
  if (plan->veto == VETO_NONE) {
    plan->veto = why;
  }
}

static void SelectRegionAndSession(bool *rf_silence, TransmitPlan_t *plan)
{    /* Perform H3lite region lookup if we have a valid fix */
    if (GNSS_IsFixValid(&hgnss) && 
        GNSS_ValidateCoordinates(hgnss.data.latitude, hgnss.data.longitude))
    {
      /* Start timing for H3 region lookup */
      uint32_t h3_start = HAL_GetTick();
      
      /* F-R4 (#77): ONE geofence resolution per cycle — previously three
       * (DetectFromGPS_H3 for the name print, latLngToRegion for policy, and
       * another DetectFromGPS_H3 inside AutoSwitchForLocation). */
      RegionId h3_region_id = latLngToRegion(hgnss.data.latitude, hgnss.data.longitude);
      LoRaMacRegion_t detected_region = MultiRegion_DetectFromH3Region(
          h3_region_id, hgnss.data.latitude, hgnss.data.longitude);
      
      /* Calculate elapsed time for H3 lookup */
      uint32_t h3_elapsed = HAL_GetTick() - h3_start;
      (void)h3_elapsed;  /* FR-19: log-only in flight */
      
      /* BUG 1.3 FIX: Only skip transmission for REGION_RESTRICTED (regulatory prohibition).
       * NOTE: REGION_RESTRICTED is now 15 (h3lite), not 255 — the 4-bit regionId
       * field in the packed table could never emit 255; ID 15 was repurposed from
       * the CD900-1A test plan slot. This macro comparison keeps working because
       * both sides use the same h3lite.h definition.
       * REGION_UNKNOWN means open ocean or uncovered H3 cells — keep current region and
       * transmit normally (standard convention over international waters). Previously this
       * also blocked UNKNOWN, causing the balloon to go silent over every ocean crossing. */
      /* F-06 (#64), DDR-0015: region selection DELIBERATELY runs on
       * possibly-stale (last-known) position. Holding the last region across
       * a GPS gap is intentional and safer than the alternatives: a missed
       * fix does not mean the sonde teleported, snapping region on a
       * minutes-old position would be the dangerous action, and going
       * radio-silent on stale position risks the whole mission. The DDR-0003
       * GPS-stale bit governs SCIENCE DATA HONESTY, not region selection.
       * Do not "fix" this without revisiting DDR-0015. */
      if (h3_region_id == REGION_RESTRICTED) {
        /* R11 (#56): don't RETURN here — that discarded the sample entirely.
         * The archive exists precisely for data that can't be transmitted:
         * use the rf_silence pattern (DDR-0018) — GPS + re-read + flash write
         * proceed below; only the TX state machine is skipped. */
        SONDE_LOG_STR("RESTRICTED REGION: RF silence — archiving locally, radio dark\r\n");
        Silence(plan, rf_silence, VETO_RESTRICTED_REGION);  /* DR-06 (#241) */
      }
      
      if (h3_region_id == REGION_UNKNOWN) {
        /* F-006 (#208, 2026-08-11 review disposition): the review warned that
         * UNKNOWN conflates "open ocean" with "unmapped land", making
         * continuation a regulatory hazard. DISPOSITION (documented product
         * decision): the h3lite dataset maps ALL land - every terrestrial
         * cell resolves to a region or to REGION_RESTRICTED - so UNKNOWN can
         * only mean ocean/uncovered water BY CONSTRUCTION, which is exactly
         * the case this keep-current-region-and-transmit policy exists for
         * (blocking UNKNOWN darkened every ocean crossing). DEPENDENCY: if
         * unmapped land ever enters the dataset, this policy must be
         * revisited before flight. */
        SONDE_LOG_STR("UNKNOWN REGION (ocean/uncovered): Keeping current region, transmitting normally\r\n");
        // Do NOT return — continue with current region
      }
      
      /* Convert floats to integers for printing (safe for all printf implementations) */
      int32_t lat_int = (int32_t)(hgnss.data.latitude * 1000000);  // 6 decimal places
      int32_t lon_int = (int32_t)(hgnss.data.longitude * 1000000); // 6 decimal places

      /* #77: shared RegionToString — the open-coded switch is gone */
      const char* region_name = RegionToString(detected_region);
      (void)lat_int; (void)lon_int; (void)region_name;  /* FR-19: log-only in flight */

      /* FR-16 (#97): SONDE_LOG directly — an ungated snprintf still ran in flight */
      SONDE_LOG("H3 Region Lookup: Lat=%ld.%06ld Lon=%ld.%06ld -> %s (took %lums)\r\n",
               (long)(lat_int / 1000000), (long)labs(lat_int % 1000000),
               (long)(lon_int / 1000000), (long)labs(lon_int % 1000000),
               region_name, (unsigned long)h3_elapsed);
      
      /* F-3 (#178): a stale position may INHIBIT (the REGION_RESTRICTED
       * silence above) but never SWITCH. On a GPS timeout AcquireGnssFix
       * forges hgnss.data from the last-known statics and both gates above
       * pass - so without this check a days-old position could initiate a
       * full LmHandlerDeInit/Init teardown (incl. a 400 ms blocking
       * HAL_Delay). This enforces in code what the F-06/DDR-0015 comment
       * above already argues: HOLD the last region across a GPS gap; never
       * snap region on stale data. */
      if (EnvSensors_GnssIsStale()) {
        SONDE_LOG_STR("MultiRegion: position STALE - auto-switch inhibited (never switch on stale)\r\n");
      } else {
      /* Production: Auto-switch region based on H3lite lookup.
       * SP-16 (#254): SUCCESS covers switched / same-region / not-joined-stay
       * / disabled-build outcomes; the old banner claimed success for all of
       * them. Compare before/after and say what actually happened. */
      LoRaMacRegion_t before_region = MultiRegion_GetActiveRegion();
      LmHandlerErrorStatus_t switch_status = MultiRegion_AutoSwitchToRegion(detected_region);

      if (switch_status == LORAMAC_HANDLER_SUCCESS) {
        if (MultiRegion_GetActiveRegion() != before_region) {
          SONDE_LOG("MultiRegion: Auto-switch: now on %s\r\n",
                    RegionToString(MultiRegion_GetActiveRegion()));
        } else {
          /* Same region, target not joined, or feature compiled out. */
          SONDE_LOG_STR("MultiRegion: Auto-switch: no change (same region/not joined/disabled)\r\n");
        }
      } else if (switch_status == LORAMAC_HANDLER_BUSY_ERROR) {
        SONDE_LOG_STR("MultiRegion: Switch deferred (MAC busy)\r\n");
      } else {
        /* LT-02/H-04 (#272): a failed switch now attempts rollback to the
         * previous region. Only claim to be staying when the rollback
         * actually restored a working session (was unconditional, and false,
         * #245's line); otherwise the radio is sessionless and degraded. */
        if (MultiRegion_LastSwitchRollbackOk()) {
          SONDE_LOG_STR("MultiRegion: Switch FAILED - rollback restored the previous region session\r\n");
        } else {
          SONDE_LOG_STR("MultiRegion: Switch FAILED and rollback FAILED - radio degraded\r\n");
        }
      }
      }
    }
    else
    {
      SONDE_LOG_STR("H3 Region Lookup: Skipped (no valid GPS fix)\r\n");
    }
}

/**
 * @brief F-R1 (#74): archive the freshly-sampled record to the flash ring.
 *        F11 + LT-03 (#271): runs AFTER the GPS fix AND the post-acquisition
 *        environment re-sample, so position, time and environment in one
 *        record describe the same moment (env skew bounded by the archive
 *        write, not by up to 255 s of GNSS acquisition). F-07 (#68): bulk
 *        retransmit cycles are NOT archived. R45: UTC epoch stamp (SysTime).
 */
static void ArchiveSample(sensor_t *sensor_data,
                          int16_t slope_mv_per_hour, OperatingMode_t current_mode,
                          uint8_t veto)
{
  /* ========== FLASH LOGGING: Store high-resolution data ========== */
  /* F11/LT-03 (#271): the record is written after the GPS fix and the
   * post-acquisition EnvSensors_Read, so its position, time and environment
   * describe the same moment. Timestamp is taken fresh at write time. */
  /* F-07 (#68): only archive genuine, freshly-sampled telemetry records.
   * OnTxData re-arms this task once per BULK packet (DDR-0005/#34); the bulk
   * path skips GPS entirely (hgnss.data memset above), so archiving those
   * passes wrote up to 20 junk records (gnss_valid=0) per bulk cycle —
   * accelerating ring wrap and burning W25Q + Tier-2 internal-flash erase
   * cycles (worst case ~33 days to internal-flash endurance). */
  if (g_tx_state != TX_STATE_BULK_TRANSFER) {
    SONDE_LOG_STR("Logging high-resolution data to flash...\r\n");
    /* R45: stamp with disciplined UTC epoch (SysTime applies the GPS-synced
     * delta stored in backup regs), NOT boot-relative RTC calendar time.
     * Before the first GPS fix this falls back to boot-relative seconds —
     * honest, monotonic, and distinguishable (small values) from epoch.
     * LT-11 (#278): kept as a LOCAL - the last external consumer of the
     * out-pointer died with #250. */
    uint32_t archive_timestamp = SysTimeGet().Seconds;  // UTC epoch seconds at write time
    /* §6a (2026-08-11): the veto rides into record.flags b5-b7 — DDR-0003:
     * a degraded cycle records WHY, not just THAT. */
    FlashLog_StatusTypeDef log_status = FlashLog_WriteRecord(&hflashlog, sensor_data, archive_timestamp,
                                                             slope_mv_per_hour, (uint8_t)current_mode,
                                                             veto);
    if (log_status == FLASH_LOG_OK) {
      uint32_t record_count = FlashLog_GetRecordCount(&hflashlog);
      (void)record_count;  /* FR-19: log-only in flight */
      SONDE_LOG("Flash log: Written record %lu (total records: %lu)\r\n",
                        record_count, record_count);
    } else {
      SONDE_LOG("Flash log: Write failed (status: %d)\r\n", log_status);
    }
  } else {
    SONDE_LOG_STR("Flash log: bulk retransmit cycle — archive write skipped\r\n");
  }
}

#if ENABLE_DEBUG_LPP
/**
 * @brief F-R1 (#74): build the CayenneLPP debug payload. Only transmitted when
 *        ENABLE_DEBUG_LPP is set (see SendTxData tail); the build itself stays
 *        unconditional here, exactly as before the extraction.
 */
static void BuildDebugLppPayload(const sensor_t *sensor_data, uint32_t ttf_ms,
                                 int16_t slope_mv_per_hour, int16_t time_to_target_signed,
                                 OperatingMode_t current_mode)
{
  // Initialize Cayenne LPP payload
  CayenneLppReset();
  SONDE_LOG_STR("CayenneLpp reset\r\n");
  
  // Add temperature data (channel 1)
  CayenneLppAddTemperature(1, sensor_data->temperature);
  
  // Add humidity data (channel 2)
  CayenneLppAddRelativeHumidity(2, sensor_data->humidity);
  
  // Add pressure data (channel 3)
  CayenneLppAddBarometricPressure(3, sensor_data->pressure);
  
  // Add GPS data (channel 4) - use zeros if GNSS fix is invalid
  float lat, lon, alt;
  if (sensor_data->gnss_valid) {
    // Convert from binary format back to decimal degrees for Cayenne
    lat = (sensor_data->latitude * 90.0f) / 8388607.0f;
    lon = (sensor_data->longitude * 180.0f) / 8388607.0f;
    alt = (float)sensor_data->altitudeGps;
    
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
  CayenneLppAddAnalogInput(5, (float)sensor_data->satellites);
  
  // Add battery voltage on channel 6 (in volts)
  CayenneLppAddAnalogInput(6, sensor_data->battery_voltage);

  // Add regulator voltage (3.3V rail) on channel 7 (in volts)
  CayenneLppAddAnalogInput(7, sensor_data->regulator_voltage);

  // Add solar panel voltage on channel 10 (in volts)
  CayenneLppAddAnalogInput(10, sensor_data->solar_voltage);
  
  // Add GNSS HDOP on channel 8 (Horizontal Dilution of Precision)
  CayenneLppAddAnalogInput(8, sensor_data->gnss_hdop);
  
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
  int hdop_int = (int)(sensor_data->gnss_hdop * 10);
  SONDE_LOG("Cayenne LPP: HDOP=%d.%d TTF=%lums Slope=%+d Time=%d Mode=%d\r\n",
           hdop_int / 10, hdop_int % 10, (unsigned long)ttf_ms,
           slope_mv_per_hour, time_to_target_signed, current_mode);
}
#endif  /* ENABLE_DEBUG_LPP */

/**
 * @brief F-R1 (#74): the adaptive transmit state machine (probe SF10 -> wait
 *        ACK -> bulk transfer -> complete). DDR-0005 confirmed uplinks; D3
 *        (#33) v3 variable-length bulk; F-005/F-006 (#51) watermark semantics.
 *        rf_silence (DDR-0018/R11) parks the machine without transmitting.
 *        LT-11 (#278): the dual-domain now_timestamp parameter is gone - the
 *        last consumer died with #250 (Payload_TimestampMinutesNow).
 */
static void RunTxStateMachine(const sensor_t *sensor_data,
                              int16_t slope_mv_per_hour, OperatingMode_t current_mode,
                              bool rf_silence)
{
  /* ========== ADAPTIVE TRANSMISSION STRATEGY ========== */
  // Step 1: Always send 10-byte compact packet at SF10 with LinkCheckReq
  // This provides maximum range and evaluates link quality for bulk transfer
  
  /* LT-07 (#277): burst-scoped deadline. The FSM used to lean on the science
   * deadline (SendTxData's entry ScienceIsDue yield) to unstick - a missing
   * LinkCheckAns parked it in BULK_TRANSFER with nothing scheduled, and an
   * early TxTimer fire (OnTxTimerEvent's redundant re-arm on a stale
   * ReloadValue - now removed) then skipped GPS + archive for the cycle,
   * silently losing one science record. Force COMPLETE when a network-wait
   * state outlives its bound, logging which one fired; the stale-state
   * reset below then flushes the deferred header sync like any completed
   * cycle. */
  uint32_t now_ms = HAL_GetTick();
  if (g_tx_state == TX_STATE_WAIT_PROBE_ACK && g_probe_sent_ms != 0 &&
      (uint32_t)(now_ms - g_probe_sent_ms) > BURST_MAX_OPEN_MS) {
    SONDE_LOG_STR("LT-07: probe-ACK wait exceeded BURST_MAX_OPEN_MS - forcing TX_STATE_COMPLETE\r\n");
    g_tx_state = TX_STATE_COMPLETE;
    g_probe_sent_ms = 0;
  }
  if (g_tx_state == TX_STATE_BULK_TRANSFER && g_burst_opened_ms != 0 &&
      (uint32_t)(now_ms - g_burst_opened_ms) > BURST_MAX_OPEN_MS) {
    SONDE_LOG_STR("LT-07: burst open longer than BURST_MAX_OPEN_MS - forcing TX_STATE_COMPLETE\r\n");
    g_tx_state = TX_STATE_COMPLETE;
    g_burst_opened_ms = 0;
    g_bulk_packets_sent = 0;
  }

  /* C2 FIX: Reset stale TX states so every timer cycle sends actual data.
   * If the previous cycle left us in WAIT_PROBE_ACK (no downlink received) or
   * COMPLETE (already handled), reset to PROBE_SF10 for a fresh probe.
   * BULK_TRANSFER is NOT reset here - it continues until exhausted via OnTxData.
   * SP-15 (#253): this reset guarantees the switch below only ever sees
   * PROBE_SF10 or BULK_TRANSFER - the old 'case TX_STATE_WAIT_PROBE_ACK' was
   * unreachable, and its 'fall through' comment was falsified by its break. */
  if (g_tx_state == TX_STATE_WAIT_PROBE_ACK || g_tx_state == TX_STATE_COMPLETE) {
    SONDE_LOG("Resetting stale TX state %d -> PROBE_SF10\r\n", g_tx_state);
    /* Finding #8: the burst (if any) ended last cycle — flush the deferred
     * header sync exactly once here, the single chokepoint every completed
     * cycle passes through. No-op when nothing was committed. STOP2 retains
     * RAM, so the dirty flag survives sleep; a mid-burst reset replays from
     * the last persisted watermark (conservative, backend dedupes). */
    FlashLog_FlushHeaderSync(&hflashlog);
    g_tx_state = TX_STATE_PROBE_SF10;
    g_bulk_packets_sent = 0;
  }
  
  SONDE_LOG("Adaptive TX: State=%d\r\n", g_tx_state);

  /* T1 (DDR-0018): RF silence skips the entire transmit state machine —
   * GPS acquisition and flash logging above have already run. */
  if (rf_silence) {
    /* F-5 (#180): a park that interrupts a burst must not strand the
     * deferred header sync (FlashLog_DeferHeaderSync at burst open) -
     * otherwise the persisted watermark freezes for the rest of the flight
     * and every later commit lives only in RAM, silently (the RAM copy
     * looks current). Flush here too; no-op when nothing is deferred. */
    FlashLog_FlushHeaderSync(&hflashlog);
    g_tx_state = TX_STATE_PROBE_SF10;  /* keep state machine parked */
  } else
  switch (g_tx_state) {
    case TX_STATE_PROBE_SF10:
    {
      // Encode 10-byte compact telemetry packet
      /* STAB-12 (#159): restore the timestamp-wrap latch once per boot -
       * a post-wrap reset must not make time look like an earlier epoch. */
      static bool s_ts_wrap_restored = false;
      if (!s_ts_wrap_restored) {
        s_ts_wrap_restored = true;
        extern RTC_HandleTypeDef hrtc;
        if (HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_TS_WRAP) == TS_WRAP_MAGIC) {
          Payload_SetTimestampWrapped(true);
        }
      }
      CompactTelemetryPacket_t compact_packet;
      /* SP-11/SP-12 (#250): the wire field is minutes since UTC EPOCH
       * (docs/PayloadFormats.md) - never RTC-UPTIME minutes (nothing ever
       * sets the RTC calendar). LT-11 (#278): this is the ONLY timestamp
       * basis the FSM uses. */
      uint16_t timestamp_min = Payload_TimestampMinutesNow();
      
      if (EncodeCompactBinaryPacket(&compact_packet, sensor_data, timestamp_min, 
                                   slope_mv_per_hour, current_mode)) {
        /* STAB-12 (#159): persist the wrap latch on first detection. */
        static bool s_ts_wrap_persisted = false;
        if (!s_ts_wrap_persisted && Payload_IsTimestampWrapped()) {
          s_ts_wrap_persisted = true;
          extern RTC_HandleTypeDef hrtc;
          HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_TS_WRAP, TS_WRAP_MAGIC);
        }
        
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
      
      /* DDR-0005 (#34): the opportunity-probe heartbeat is a CONFIRMED uplink.
       * No archive opportunity opens without its network ACK (evaluated in
       * OnTxData via params->AckReceived). LinkCheck no longer rides the probe —
       * it attaches to the first archive packet (protocol §5.2, §14). */
      LmHandlerErrorStatus_t status = LmHandlerSend(&compactData, LORAMAC_HANDLER_CONFIRMED_MSG, 0);
        
        if (status == LORAMAC_HANDLER_SUCCESS) {
          g_probe_sent_ms = HAL_GetTick();  /* LT-07 (#277): bound the wait */
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
    
    case TX_STATE_BULK_TRANSFER:
    {
      SONDE_LOG("Bulk transfer mode: packet %d/%d\r\n", 
                        g_bulk_packets_sent + 1, MAX_BULK_PACKETS_PER_CYCLE);
      
      // Check if we have unsent data and haven't exceeded packet limit
      if (FlashLog_HasUnsentData(&hflashlog) && g_bulk_packets_sent < CfgMaxBulkPkts()) {

        /* R3-04 (#218, DDR-0005 BR-TX-008): one-pass recovery read, NEWEST
         * FIRST - pending-live records (never sent) lead, then the walker
         * works backward from the recovery frontier. Was 6: the 6th record
         * was read and discarded every packet (finding #9) — the wire format
         * packs at most 5. */
        FlashLog_Record_t flash_records[BULK_V6_MAX_RECORDS];
        uint32_t record_count;
        uint32_t skipped_count = 0;  /* F-006/R13 (#51): corrupt skips are explicit */

        FlashLog_StatusTypeDef flash_status = FlashLog_GetRecoveryRecords(&hflashlog,
                                                                          flash_records,
                                                                          BULK_V6_MAX_RECORDS,
                                                                          &record_count,
                                                                          &skipped_count);
        if (skipped_count > 0) {
          /* DDR-0003: a skipped record is visible, not silent */
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
          /* DR-18: BULK_V6_MAX_RECORDS, not a magic 6 - flash_records[] is
           * sized by the constant, so a raise otherwise made THESE arrays
           * the overflow. */
          HighResTelemetryRecord_t highres_records[BULK_V6_MAX_RECORDS];
          uint32_t highres_seqs[BULK_V6_MAX_RECORDS];   /* FR-07 (#87): per-record explicit identity */
          uint8_t packed_count = 0;
          for (uint32_t i = 0; i < record_count && i < BULK_V6_MAX_RECORDS; i++) {
            if (!ConvertFlashLogToHighRes(&flash_records[i], &highres_records[packed_count])) {
              SONDE_LOG("Warning: Failed to convert flash record %lu - skipped\r\n", i);
              continue;  /* Skip bad record, keep packing the rest */
            }
            highres_seqs[packed_count] = flash_records[i].sequence;
            packed_count++;
          }

          if (packed_count == 0) {
            /* R21 (#51) + FR-09 (#92): nothing convertible. Retire the FULL
             * consumed batch, not just the corrupt skips: a record that reads
             * CRC-clean but fails ConvertFlashLogToHighRes is deterministically
             * unconvertible — leaving it pending re-probes it forever and
             * wedges bulk transfer.
             * R3-04 (#218): retire = advance the one-pass watermark past each
             * consumed record (MarkRecoverySent is range-aware: live records
             * raise H, walker records lower F). Leading corrupt runs were
             * already retired inline by the read. */
            SONDE_LOG("Retiring %lu unconvertible records with no TX (%lu corrupt skipped)\r\n",
                              (unsigned long)record_count, (unsigned long)skipped_count);
            for (uint32_t i = 0; i < record_count; i++) {
              FlashLog_MarkRecoverySent(&hflashlog, flash_records[i].sequence);
            }
            g_tx_state = TX_STATE_COMPLETE;
            break;
          }

          // F16 FIX: Send at SF7, resolved per-region (was hardcoded DR_3)
          LmHandlerSetTxDatarate(DatarateFromSF(7));  // SF7 in ANY region

          /* D3 (#33) + FR-07 (#87): wire v6 variable-length bulk (packet_type
           * 0x05, per-record explicit sequence). Query the runtime payload
           * budget (current DR + pending FOpts, protocol §11) and pack only
           * complete records that fit. Records that don't fit stay pending
           * for the next cycle (stable identity, DDR-0005). */
          LoRaMacTxInfo_t txInfo;
          uint16_t max_payload = 0;
          for (uint8_t try_n = packed_count; try_n > 0; try_n--) {
            if (LoRaMacQueryTxPossible((uint8_t)(BULK_V6_OVERHEAD + try_n * BULK_V6_RECORD_WIRE),
                                       &txInfo) == LORAMAC_STATUS_OK) {
              max_payload = (uint16_t)(BULK_V6_OVERHEAD + try_n * BULK_V6_RECORD_WIRE);
              break;
            }
          }
          if (max_payload == 0) {
            SONDE_LOG_STR("Bulk: no payload budget at current DR - retry next cycle\r\n");
            g_tx_state = TX_STATE_COMPLETE;
            break;
          }

          uint8_t v6_buf[BULK_V6_OVERHEAD + BULK_V6_MAX_RECORDS * BULK_V6_RECORD_WIRE];
          uint8_t v5_packed = 0;
          uint16_t v5_len = 0;

          if (EncodeBulkPacketV6(v6_buf, sizeof(v6_buf), max_payload,
                                 highres_records, highres_seqs, packed_count,
                                 &v5_packed, &v5_len)) {

            /* DDR-0005 (#34): archive packets are CONFIRMED uplinks. The FIRST
             * archive packet of a burst carries LinkCheckReq (protocol §5.2);
             * records commit only on network ACK (OnTxData). */
            if (g_bulk_packets_sent == 0) {
              LmHandlerErrorStatus_t lc_status = LmHandlerLinkCheckReq();
              (void)lc_status;  /* FR-19: log-only in flight; the call has the side effect */
              SONDE_LOG("LinkCheckReq on first archive packet: %d\r\n", lc_status);
            }

            LmHandlerAppData_t bulkData;
            bulkData.Port = LORAWAN_BULK_PORT;  // Port 11
            bulkData.BufferSize = v5_len;
            bulkData.Buffer = v6_buf;

            SONDE_LOG("Sending %u-byte bulk v6 packet at SF7 on port %d with %u records\r\n",
                              v5_len, LORAWAN_BULK_PORT, v5_packed);

            /* R3-04 (#218, DDR-0005 BR-TX-011): archive recovery is
             * UNCONFIRMED one-pass - no per-record ACK is awaited and a lost
             * frame is never retried autonomously. The backend owns gap
             * repair (BR-TX-012, deferred: #125). */
            LmHandlerErrorStatus_t bulk_status = LmHandlerSend(&bulkData, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0);

            if (bulk_status == LORAMAC_HANDLER_SUCCESS) {
              g_bulk_packets_sent++;

              /* R3-04 (#218, BR-TX-009/010): the watermark advances AT SEND
               * TIME, per packed record (newest-first as read). Records read
               * but cut by the payload budget stay sendable. */
              for (uint8_t i = 0; i < v5_packed; i++) {
                FlashLog_MarkRecoverySent(&hflashlog, highres_seqs[i]);
              }
              if (v5_packed != record_count) {
                SONDE_LOG("WARN: packed %u of %lu read - rest stay sendable\r\n",
                                  v5_packed, (unsigned long)record_count);
              }

              SONDE_LOG("Bulk packet sent successfully! (%d/%d packets sent)\r\n",
                                g_bulk_packets_sent, MAX_BULK_PACKETS_PER_CYCLE);
              
              // Continue bulk transfer if more data available and under packet limit
              if (FlashLog_HasUnsentData(&hflashlog) && g_bulk_packets_sent < CfgMaxBulkPkts()) {
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
}

/**
 * @brief R3-01 (#215): has the absolute science deadline arrived?
 *        Wrap-safe (int32 subtraction), false while unscheduled.
 */
static bool ScienceIsDue(void)
{
  return g_science_due_ms != 0 &&
         (int32_t)(HAL_GetTick() - g_science_due_ms) >= 0;
}

/**
 * @brief R3-01 (#215): re-arm TxTimer against the ABSOLUTE science deadline.
 * @param interval_ms   the plan's science interval for THIS cycle
 * @param science_cycle true when this invocation serviced current science:
 *        the deadline advances phase-preservingly (due += interval), with a
 *        re-base if we fell a full period behind (never storm catch-up).
 *        False for bulk continuations: the timer is only re-pointed at the
 *        existing deadline - archive work can never move it.
 */
static void RescheduleScienceTimer(uint32_t interval_ms, bool science_cycle)
{
  uint32_t now_ms = HAL_GetTick();
  if (science_cycle) {
    if (g_science_due_ms == 0) g_science_due_ms = now_ms;  /* establish phase */
    g_science_due_ms += interval_ms;
    if ((int32_t)(g_science_due_ms - now_ms) <= 0) {
      g_science_due_ms = now_ms + interval_ms;  /* fell >= 1 period behind */
    }
  } else if (g_science_due_ms == 0) {
    g_science_due_ms = now_ms + interval_ms;
  }
  /* LT-01 (#269): the re-base guard above runs ONLY on the science path. A bulk
   * continuation legitimately arrives with g_science_due_ms already in the past
   * (SendTxData's ScienceIsDue() check is tens of ms earlier - EnvSensors_Read
   * runs the MS5607 OSR_4096 pair, SHT31 and 3 ADC channels in between), and
   * this subtraction used to be unsigned: 50 ms overdue became a 4294967246 ms
   * period, which TIMER_IF_Convert_ms2Tick ALIASES (not saturates) into
   * ~103079163 ticks = ~28 h of ReloadValue. UTIL_TIMER_Start clamps only
   * upward, so nothing caught it and only the deadman (3-6 h) recovered.
   * Clamp in the SIGNED domain: an overdue deadline means "fire now". */
  int32_t remain_ms = (int32_t)(g_science_due_ms - now_ms);
  if (remain_ms <= 0) remain_ms = 1;
  uint32_t delay_ms = (uint32_t)remain_ms;
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, delay_ms);
  UTIL_TIMER_Start(&TxTimer);
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
  Deadman_MarkProgress();  /* F13a: a work cycle provably started */

  /* S-06 (#231): every battery/solar conversion this cycle must reference a
   * VDDA sampled NOW, not before a burst/ascent run that never saw STOP2. */
  SYS_ADC_InvalidateVdda();

  /* R3-01 (#215): a SendTxData invocation is not necessarily a science cycle -
   * bulk continuation re-arms THIS task. If the science deadline has arrived,
   * the burst yields NOW so this run takes the science path (GPS + current
   * record + probe). Current science always wins (DDR-0005 BR-TX-001/002).
   * RunTxStateMachine's COMPLETE reset performs the deferred header-sync
   * flush, so nothing already committed is stranded. */
  if (g_tx_state == TX_STATE_BULK_TRANSFER && ScienceIsDue()) {
    SONDE_LOG_STR("R3-01: science deadline reached - bulk transfer yields\r\n");
    g_tx_state = TX_STATE_COMPLETE;
    g_bulk_packets_sent = 0;
  }

  /* ========== POWER MANAGEMENT — decide half (R47, #44) ========== */
  static VoltageSlope_t voltage_slope = {0};

  // Read current sensor data for temperature
  sensor_t sensor_data = {0};  /* #35: zero-init — uninitialized members were archived as authentic */
  EnvSensors_Read(&sensor_data);
  float temperature_c = sensor_data.temperature;

  // Use RTC-based time that continues during STOP2 sleep
  uint16_t ms_unused;
  uint32_t now_timestamp = TIMER_IF_GetTime(&ms_unused);  // RTC seconds

  /* D8 (#59) / finding #7: windowed-range float detection, each work cycle */
  MissionState_Update(sensor_data.pressure, !sensor_data.press_stale, now_timestamp);

  /* F-8 (#183): ONE battery ADC conversion per cycle - taken inside
   * EnvSensors_Read above, where the value and its staleness flag are
   * captured TOGETHER. Every consumer uses that same sample: the plan's
   * battery_mv_raw is exactly the reading sensor_data.batt_stale describes
   * (RV-03 can no longer be defeated by a cached/implausible re-read paired
   * with a fresh flag), and the archived record shows the voltage the power
   * decision actually used. The bulk-opportunity gate in OnTxData uses it
   * too via s_cycle_batt_mv. */
  uint16_t battery_mv_raw = (uint16_t)(sensor_data.battery_voltage * 1000.0f + 0.5f);
  s_cycle_batt_mv = battery_mv_raw;
  uint16_t solar_mv = (uint16_t)(sensor_data.solar_voltage * 1000.0f + 0.5f);
  (void)solar_mv;  /* FR-19: log-only in flight */

  /* R47: mode selection, slope/prediction, GPS temperature lockout and the
   * RF-silence veto all live in the pure decide half (transmit_plan.c) —
   * host-testable with zero hardware. The plan records WHY, not just THAT. */
  TransmitPlan_t plan = DecideTransmitPlan(&voltage_slope, battery_mv_raw,
                                           temperature_c, sensor_data.temp_stale != 0,
                                           now_timestamp,
                                           LmHandlerJoinStatus() == LORAMAC_HANDLER_SET,
                                           MissionState_IsCommissioning(),
                                           sensor_data.batt_stale);
  /* DDR-0002 mission cadence (finding #7, 2026-08-10) — the consumer that was
   * missing: ASCENT = MISSION_ASCENT_TX_INTERVAL_MS (10 s), FLOAT =
   * MISSION_FLOAT_TX_INTERVAL_MS (5 min). Only when the battery is healthy
   * (NORMAL/CONSERVATIVE); REDUCED/RECOVERY/SURVIVAL keep their longer,
   * power-protective intervals (never toward higher power).
   * MISSION-01a (#142, maintainer decision): ASCENT keeps GNSS powered and
   * tracking continuously — a fresh position is always available and cycles
   * run nearly back-to-back by design (the 10 s period is measured from cycle
   * start and GPS acquisition can outlast it, so there is little STOP2 sleep
   * during the ~2 h climb). Accepted: ascent is short; float is long. */
  if (plan.power_mode <= MODE_CONSERVATIVE) {
    /* RV-10 (#166): explicit state gating. ASCENT takes the fast 10 s cadence
     * (deliberate, DDR-0002). FLOAT only RELAXES the interval - replacing
     * CONSERVATIVE's 10 min with the mission's 5 min moved toward HIGHER
     * power, contradicting the invariant above. COMMISSIONING gets no
     * override at all (the old ternary silently applied the FLOAT interval). */
    MissionState_t ms = MissionState_Get();
    if (ms == MISSION_ASCENT) {
      plan.tx_interval_ms = MISSION_ASCENT_TX_INTERVAL_MS;
      plan.gps_enabled = true;  /* #142: GNSS always tracking in ASCENT */
      /* LT-08 (#278): forcing the ENABLE flag without a BUDGET is a no-op
       * (S-A/#211: AcquireGnssFix loops zero times on gps_timeout_ms == 0).
       * Today NORMAL/CONSERVATIVE happen to carry 60 s - a cross-file
       * coincidence, not a guarantee. Apply the same non-zero clamp the
       * GPS-loss path uses. */
      if (plan.gps_timeout_ms == 0U) {
        plan.gps_timeout_ms = GPS_LOSS_RETRY_TIMEOUT_MS;
      }
    } else if (ms == MISSION_FLOAT &&
               MISSION_FLOAT_TX_INTERVAL_MS > plan.tx_interval_ms) {
      plan.tx_interval_ms = MISSION_FLOAT_TX_INTERVAL_MS;
    }
  }

  bool gps_enabled_by_power_mgmt = plan.gps_enabled;
  uint32_t gps_timeout_ms = plan.gps_timeout_ms;
  OperatingMode_t current_mode = plan.power_mode;
  int16_t slope_mv_per_hour = plan.voltage_slope_mv_per_hour;
  int16_t time_to_target_signed = plan.time_to_target_h;
  uint16_t battery_mv_normalized = plan.battery_mv_normalized;
  (void)battery_mv_normalized;  /* FR-19: log-only in flight */

  /* R3-01 (#215): re-arm against the ABSOLUTE science deadline instead of
   * "now + interval". A bulk continuation (g_tx_state still BULK_TRANSFER at
   * this point) re-points the timer at the existing deadline WITHOUT
   * advancing it; only a serviced science cycle moves the deadline. */
  RescheduleScienceTimer(plan.tx_interval_ms,
                         g_tx_state != TX_STATE_BULK_TRANSFER);
  
  // Log power management status
  /* F27 FIX: integer-only print (no float printf support linked) */
  int temp_deci_pm = (int)(temperature_c * 10.0f);
  (void)temp_deci_pm;  /* FR-19: log-only in flight */
  /* FR-16 (#97): ungated snprintf+STR pairs still executed in flight builds */
  SONDE_LOG("\r\n=== POWER MGMT: Temp=%d.%dC Bat_raw=%dmV Bat_norm=%dmV Solar=%dmV Slope=%+dmV/h ",
           temp_deci_pm / 10, abs(temp_deci_pm % 10),
           battery_mv_raw, battery_mv_normalized, solar_mv, slope_mv_per_hour);

  if (time_to_target_signed < 0) {
    SONDE_LOG("Critical_in=%dh ", abs(time_to_target_signed));
  } else if (time_to_target_signed > 0) {
    SONDE_LOG("Full_in=%dh ", time_to_target_signed);
  } else {
    SONDE_LOG_STR("Stable ");
  }

  SONDE_LOG("Mode=%s GPS=%s ===\r\n",
           GetModeName(current_mode), gps_enabled_by_power_mgmt ? "ON" : "OFF");
  
  /* F11 FIX: Flash logging moved to AFTER GPS acquisition + sensor re-read
   * (see below). Previously the record was written here with the PREVIOUS
   * cycle's position and the CURRENT timestamp — the whole track was
   * spatially stale by one interval. */

  /* ========== END POWER MANAGEMENT ========== */
  
  /* T1 ladder (DDR-0018): rejoin is a COMMISSIONING-ONLY operation.
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
    Silence(&plan, &rf_silence, VETO_RF_SILENCE);  /* DR-06 (#241): same veto DecideTransmitPlan set; first wins */
    SONDE_LOG_STR("SendTxData: FLIGHT with no session - RF silence, logging only\r\n");
  }

  /* MISSION-01b (#142): commissioned but not launched = QUIET WATCH. No GPS,
   * no telemetry TX — the cycle still reads sensors (pressure feeds the
   * BR-LIFE-007 launch detector in MissionState_Update above) and logs to
   * flash. Entry to ASCENT is by arming (button hook) or launch detection.
   * Unjoined commissioning is untouched above (join retry path). */
  if (MissionState_IsCommissioning() && LmHandlerJoinStatus() == LORAMAC_HANDLER_SET) {
    Silence(&plan, &rf_silence, VETO_PRELAUNCH_QUIET);  /* DR-06 (#241) */
    gps_enabled_by_power_mgmt = false;
  }

  /* #141 (maintainer decision 2026-08-11): GPS-LOSS SILENCE. No fresh fix for
   * GPS_LOSS_SILENCE_S -> stop transmitting; science logging continues (the
   * archive is the point) and GPS acquisition is FORCED ON below every cycle
   * until a fresh fix clears it — "keep doing science and logging and keep
   * trying GPS, but stop transmitting". FLIGHT only: commissioning is exempt
   * (DDR-0018). The grace epoch arms at this boot's first cycle, so a
   * never-fixed unit gets the full window before going dark.
   *
   * DDR-0015 (2026-08-13 intent interview) now OWNS this budget and binds it to
   * 24 h (BR-STALE-017). The reason for the silence changed with it: it is
   * REGULATORY (the sonde can no longer prove which region it is in), not energy
   * conservation as DDR-0016 INV-PWR-009 originally framed it. Consequences:
   *   - exactly one staleness budget exists; BR-STALE-020 forbids a second,
   *     independent time-based cutoff (notably an RTC-sync-age timer - see
   *     DDR-0013 INV-TIME-009);
   *   - one quality-valid fix clears the silence immediately, on that same wake
   *     (BR-STALE-019), which is what the s_last_fresh_fix_s reference below does;
   *   - science, archive logging, RTC timekeeping and GPS retries all continue
   *     while dark (BR-STALE-018).
   * The never-fixed (Band 0) home-region fallback shares this same budget,
   * anchored at the flight transition (DDR-0015 INV-STALE-008). */
  {
    /* F-1 (#176): the whole grace policy runs on the UTC clock
     * (SysTimeGet().Seconds). Before the first sync this is the SysTime
     * boot-relative fallback - honest, monotonic, and too small to persist,
     * which is exactly right: an unsynced unit has no trustworthy absolute
     * age to carry across a reset. Post-sync the epoch is real UTC, so the
     * persist and restore gates below are reachable and a reset can no
     * longer restart the 24 h window. */
    uint32_t utc_now_s = SysTimeGet().Seconds;
    if (s_gps_loss_epoch_s == 0) {
      /* STAB-01 (#148): restore the persisted loss epoch - a reset must not
       * restart the grace window. Plausibility-gated like the fix epoch:
       * unknown age means CONSERVATIVE (count from this boot, persist it). */
      extern RTC_HandleTypeDef hrtc;
      uint32_t persisted = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_GPS_LOSS_EPOCH);
      if (UtcTimeIsValid() &&
          persisted >= UTC_EPOCH_PLAUSIBLE_MIN && persisted <= utc_now_s) {
        s_gps_loss_epoch_s = persisted;
      } else {
        s_gps_loss_epoch_s = utc_now_s;
        if (UtcTimeIsValid()) {
          HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_GPS_LOSS_EPOCH, utc_now_s);
        }
      }
    }
    uint32_t ref_s = (s_last_fresh_fix_s > s_gps_loss_epoch_s)
                     ? s_last_fresh_fix_s : s_gps_loss_epoch_s;
    /* RV-06 (#162): backward time step (LSE->LSI failover restarts the RTC
     * counter) - re-seed, never evaluate a wrapped delta (the deadman
     * pattern). Without the guard the subtraction wraps huge and the unit
     * goes instantly dark. */
    if (utc_now_s < ref_s) {
      s_gps_loss_epoch_s = utc_now_s;
      ref_s = utc_now_s;
      if (UtcTimeIsValid()) {
        extern RTC_HandleTypeDef hrtc;
        HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_GPS_LOSS_EPOCH, utc_now_s);
      }
    }
    if (!MissionState_IsCommissioning() &&
        (utc_now_s - ref_s) > GPS_LOSS_SILENCE_S) {
      Silence(&plan, &rf_silence, VETO_GPS_LOSS);  /* DR-06 (#241) */
      /* STAB-03 (#150): keep trying GPS until a fresh fix clears the
       * silence - urgency overrides the power-mode PREFERENCE, never the
       * hard electrical floor. In MODE_SURVIVAL (raw V below the 4300 mV
       * LTO floor) a forced GNSS acquisition risks the droop/brownout
       * loop the stability review traces (brownout -> STAB-01 resets
       * stale-position age -> repeat). Stay dark and retry next cycle. */
      if (current_mode != MODE_SURVIVAL) {
        gps_enabled_by_power_mgmt = true;
        /* S-A (#211, 2026-08-12): forcing the ENABLE flag without a BUDGET is
         * a no-op. Every GPS-off mode carries gps_timeout_ms == 0 from
         * ApplyOperatingMode, so AcquireGnssFix() powered the receiver up,
         * ran `while ((HAL_GetTick() - gps_start) < 0)` zero times, took the
         * stale-position branch and powered it back down. s_last_fresh_fix_s
         * could never advance, so the silence never cleared: a permanently
         * dark unit paying a full GPS power-cycle every cycle to guarantee
         * no fix. This is the RV-08 sawtooth re-entering through the modes a
         * power-starved float actually lives in. */
        if (gps_timeout_ms == 0U) {
          gps_timeout_ms = GPS_LOSS_RETRY_TIMEOUT_MS;
        }
      }
      SONDE_LOG_STR("GPS-LOSS SILENCE: no fresh fix - radio dark, still logging, GPS retry on\r\n");
    }
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
    EnvSensors_MarkGnssStale(true);  /* DR-15: GPS skipped - s_gnss_stale must not keep the last acquisition's false across REDUCED/RECOVERY/SURVIVAL runs (same class as R3-02/#216) */
    ttf_ms = 0;  // No GPS acquisition performed

    /* BURST-03 (#141): the geofence must still run when GPS is skipped —
     * otherwise every REDUCED/RECOVERY/SURVIVAL cycle (most of a multi-week
     * float) transmits blind inside restricted regions. Evaluate against the
     * backup-register last-known position (DDR-0015 already accepts region
     * decisions on stale position; this extends it to the skip path).
     * INHIBIT ONLY — never auto-switch regions on a stale fix. Bulk cycles
     * are exempt: the probe that opened the burst ran the check first. No
     * fix EVER (cold boot) -> transmit: a sonde that has never had a fix
     * cannot be known to be restricted, and going dark forever is worse. */
    if (g_tx_state != TX_STATE_BULK_TRANSFER) {
      float la, lo, al;
      /* DR-02 (#237): only an explicit RESTRICTED silences. UNKNOWN (no fix
       * ever, or an implausible stored position) maps to the documented
       * "cannot be known restricted -> transmit" policy by choice, not by
       * inheriting the outcome of a failed validation. */
      if (LastPos_Load(&la, &lo, &al) && GeofenceRestricted(la, lo) == GEO_PERMISSION_RESTRICTED) {
        Silence(&plan, &rf_silence, VETO_RESTRICTED_REGION);  /* DR-06 (#241) */
        SONDE_LOG_STR("RESTRICTED REGION (last-known pos, GPS off): RF silence\r\n");
      }
    }
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
  /* F-R1 (#74): acquisition and region selection are extracted phases. */
  if (AcquireGnssFix(gps_timeout_ms, &ttf_ms)) {
    SelectRegionAndSession(&rf_silence, &plan);
  } else {
    /* S-02 (#226): a failed GNSS wake still leaves the restricted-region
     * duty — the last-known position is exactly what the geofence consumes
     * on every other stale path (BURST-03 GPS-skip path above). INHIBIT
     * ONLY — never switch regions on a stale fix (F10/#175). No fix EVER
     * (cold boot) -> transmit: a never-fixed sonde cannot be known
     * restricted, and going dark forever is worse. */
    float la, lo, al;
    /* DR-02 (#237): explicit RESTRICTED only; UNKNOWN -> transmit (same
     * policy as the GPS-skip path above). */
    if (LastPos_Load(&la, &lo, &al) && GeofenceRestricted(la, lo) == GEO_PERMISSION_RESTRICTED) {
      Silence(&plan, &rf_silence, VETO_RESTRICTED_REGION);  /* DR-06 (#241) */
      SONDE_LOG_STR("RESTRICTED REGION (last-known pos, GNSS wake failed): RF silence\r\n");
    }
  }

  #endif  /* GPS_DISABLED_FOR_TESTING */
  }  /* End of else block for gps_enabled_by_power_mgmt */
  /* Add separator before continuing to telemetry */
  SONDE_LOG_STR("\r\n");
  
  // CRITICAL: fold in the fresh GPS fix acquired above.
  EnvSensors_MergeGnss(&sensor_data);

  /* LT-03 (#271): re-sample the environment AFTER acquisition so the archived
   * record pairs same-moment environment and position. gps_timeout_ms is
   * 60 s in NORMAL/CONSERVATIVE and config-driven up to 255 s (uint8_t
   * seconds) - ~300 m of ascent at 5 m/s between the start-of-cycle
   * EnvSensors_Read and this point. FR-15 (#96) removed a post-fix re-read
   * here as adding "no new information"; LT-03 refutes that across the
   * acquisition gap, so do not re-apply FR-15's removal reasoning.
   * The power/plan decision above deliberately keeps the pre-acquisition
   * sample: DecideTransmitPlan is NOT re-run (mode hysteresis + the
   * F-8/#183 one-battery-conversion-per-cycle discipline) - this read feeds
   * the archive record and the payload only. EnvSensors_Read re-folds the
   * same hgnss fix internally, so the merge above is not undone.
   * Science path only: bulk continuations skip GPS and must not pay the
   * MS5607 OSR_4096 + SHT31 cost (ArchiveSample already skips them). */
  if (g_tx_state != TX_STATE_BULK_TRANSFER) {
    EnvSensors_Read(&sensor_data);
  }

  ArchiveSample(&sensor_data, slope_mv_per_hour, current_mode,
                (uint8_t)plan.veto);

  #if ENABLE_DEBUG_LPP
  BuildDebugLppPayload(&sensor_data, ttf_ms, slope_mv_per_hour, time_to_target_signed, current_mode);
  #endif

  RunTxStateMachine(&sensor_data, slope_mv_per_hour, current_mode, rf_silence);
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
    SONDE_LOG("CayenneLPP payload size: %d bytes\r\n", lppData.BufferSize);
    
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
      SONDE_LOG("Queuing GNSS detail packet: %d bytes (GPS:%d BeiDou:%d GLONASS:%d)\r\n",
               gnss_packet_size, hgnss.extended.gps_count,
               hgnss.extended.beidou_count, hgnss.extended.glonass_count);
      
      /* Push to queue - will be sent after RX windows complete */
      if (PacketQueue_Push(&g_packet_queue, gnss_detail_buffer, gnss_packet_size, LORAWAN_GNSS_DETAIL_PORT))
      {
        SONDE_LOG("GNSS packet queued (queue size: %d)\r\n",
                 g_packet_queue.count);
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
  
  /* F-6 (#181): a full work cycle COMPLETED - the boot was productive, so
   * the consecutive-boot counter resets here (F-03/#65). A deterministic
   * in-cycle fatal now accumulates evidence across boots and the FR-23/#104
   * escape (degrade instead of the 6th reset) can actually engage. */
  ResetCause_ClearBootAttempts();
  SONDE_LOG_STR("=== SendTxData END ===\r\n");
  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */
  SONDE_LOG_STR("\r\n*** OnTxTimerEvent FIRED ***\r\n");
  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /* LT-07 (#277): NO bare UTIL_TIMER_Start(&TxTimer) here - it was a second
   * arming path next to RescheduleScienceTimer and fired on a stale partial
   * ReloadValue; an early fire ran a cycle with neither GPS nor archive and
   * silently cost a science record. The period is owned solely by
   * RescheduleScienceTimer, computed from the science deadline. */
  /* USER CODE BEGIN OnTxTimerEvent_2 */
  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */

/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  /* BURST-02: LmHandler invokes THIS callback from two places — McpsConfirm
   * (LmHandler.c) with the full TxParams set, and MlmeConfirm (LmHandler.c)
   * with ONLY TxParams.Status refreshed. On the MLME path AckReceived,
   * Datarate, Channel and UplinkCounter still hold whatever the previous MCPS
   * confirm left there. Any uplink carrying an MLME request lands here twice —
   * and the first archive packet ALWAYS carries LinkCheckReq (protocol §5.2) —
   * double-counting MultiRegion_SaveCurrentContext() and the 10-TX NVM store
   * interval, and feeding the burst state machine a stale ACK flag.
   * MLME outcomes are surfaced by their own callbacks (OnJoinRequest,
   * OnRxData's LinkCheck fields); only the MCPS confirm describes an
   * application uplink, which is all this function reasons about. */
  if (params->IsMcpsConfirm == 0) {
    return;
  }

  SONDE_LOG("\r\nOnTxData: Status=%d DR=%d Ch=%lu FCnt=%lu\r\n",
                    params->Status, params->Datarate,
                    (unsigned long)params->Channel, (unsigned long)params->UplinkCounter);
  
  /* CRITICAL: Capture context after successful TX (not before region switch) */
  /* This ensures correct DevAddr, FCnt, and session state are saved */
  if (params->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
    MultiRegion_SaveCurrentContext();

    /* R2-05 (#109) / 2026-08-10 finding #1: the MAC NVM store task was
     * registered but NEVER set — OnStoreContextRequest was unreachable dead
     * code and in-flight MAC state (channel-mask edits, RX1 offset/RX2 DR,
     * ADR-adjusted DR/TxPower) died on every reset. Trigger on a save
     * interval: every 10th successful TX is often enough for MAC state to
     * survive a reset, rare enough that the internal-flash page erase per
     * store stays negligible against the 10k-cycle endurance. */
    {
      static uint8_t s_tx_since_nvm_store = 0;
      if (++s_tx_since_nvm_store >= 10U) {
        s_tx_since_nvm_store = 0;
        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
      }
    }
  }

  /* R3-04 (#218): the commit-on-ACK block is DELETED. DDR-0005 one-pass
   * recovery sends archive frames UNCONFIRMED and advances the watermark at
   * send time (FlashLog_MarkRecoverySent in RunTxStateMachine). The backend
   * deduplicates by (device, base_seq + i) and owns gap repair. */

  /* DDR-0005 (#34): the confirmed probe heartbeat opens the archive opportunity.
   * No ACK -> stay in long-range mode, no archive probe (protocol §5.1/§15). */
  if (g_tx_state == TX_STATE_WAIT_PROBE_ACK) {
    g_probe_sent_ms = 0;  /* LT-07 (#277): the wait is resolved one way or the other */
    if (params->Status == LORAMAC_EVENT_INFO_STATUS_OK && params->AckReceived) {
      /* F-8 (#183): the cycle's one conversion, not a fresh one mid-burst. */
      uint16_t battery_mv = s_cycle_batt_mv;
      bool battery_good = (battery_mv >= CfgBulkBattMin());
      bool has_cache = FlashLog_HasUnsentData(&hflashlog);
      SONDE_LOG("Probe ACK received — battery %dmV (%s), cache %s\r\n",
                        battery_mv, battery_good ? "GOOD" : "LOW",
                        has_cache ? "HAS_DATA" : "NO_DATA");
      /* R3-03 (#217): mission-aware TX policy (DDR-0005 INV-TX-006,
       * BR-TX-016/017, P-TX-008): during ASCENT the current 10 s
       * full-resolution observations are far more valuable than historical
       * recovery - never open the archive opportunity. The climb is short;
       * the backlog keeps and recovery resumes automatically in FLOAT
       * (BR-TX-019/020). */
      if (battery_good && has_cache && MissionState_Get() != MISSION_ASCENT) {
        SONDE_LOG_STR("Archive opportunity OPEN — first archive probe\r\n");
        /* Finding #8: defer header persistence for the whole burst — one
         * sector erase at flush instead of one per ACKed packet. */
        FlashLog_DeferHeaderSync(&hflashlog);
        g_burst_opened_ms = HAL_GetTick();  /* LT-07 (#277): bound the burst */
        g_tx_state = TX_STATE_BULK_TRANSFER;
        g_bulk_packets_sent = 0;
        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
      } else {
        if (battery_good && has_cache) {
          SONDE_LOG_STR("R3-03: ASCENT — archive recovery inhibited (live science prioritized)\r\n");
        }
        g_tx_state = TX_STATE_COMPLETE;
      }
    } else {
      SONDE_LOG("Probe heartbeat NOT acknowledged (status %d, ack %d) — no archive opportunity\r\n",
                        params->Status, params->AckReceived);
      g_tx_state = TX_STATE_COMPLETE;
    }
  }

  /* R3-04 (#218): the unACKed-archive fallback is DELETED. Archive frames
   * are unconfirmed now (BR-TX-011) - AckReceived carries no meaning for
   * them. Burst continuation is governed by the LinkCheck verdict on the
   * first archive packet (BR-TX-006/007, OnRxData) and the packet cap. */

  /* Drain packet queue after RX windows complete */
  /* This callback fires AFTER RX2 window closes, so MAC is ready for next TX */
  if (!PacketQueue_IsEmpty(&g_packet_queue))
  {
    PacketQueueEntry_t entry;
    /* STAB-P3#7 (#243): peek-before-send, pop on acceptance - the old
     * pop-then-send lost the queued packet on a busy/errored LmHandlerSend. */
    if (PacketQueue_Peek(&g_packet_queue, &entry))
    {
      LmHandlerAppData_t queuedData;
      queuedData.Port = entry.port;
      queuedData.BufferSize = entry.size;
      queuedData.Buffer = entry.buffer;

      LmHandlerErrorStatus_t queue_status = LmHandlerSend(&queuedData, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0);
      if (queue_status == LORAMAC_HANDLER_SUCCESS)
      {
        PacketQueue_Pop(&g_packet_queue, &entry);  /* accepted: consume it */
      }
      SONDE_LOG("OnTxData: Queued packet (port %d, %d bytes) send status: %d (queue remaining: %d)\r\n",
                        entry.port, entry.size, queue_status, g_packet_queue.count);
    }
  }
  
  /* C7b FIX: Continue bulk transfer if active - re-arm send task for next packet.
   * After TX+RX windows complete, if we're still in BULK_TRANSFER with data remaining,
   * schedule another SendTxData call to send the next bulk packet immediately.
   *
   * BURST-01: packet counts 0 and 1 are NOT this callback's to terminate.
   *   sent == 0 — the probe-ACK branch ABOVE, in this same call, just opened
   *               the archive opportunity and armed the send task. The old
   *               unguarded `else if` then reset it to COMPLETE twelve lines
   *               later, so no archive packet was ever built. The armed task
   *               still ran, spending a full work cycle (GPS power-up, sensor
   *               read, archive write) to send another confirmed heartbeat —
   *               which was ACKed, re-opened the opportunity, and was killed
   *               again: an unbounded transmit loop with zero data return,
   *               ending only when the battery fell below BULK_BATTERY_MIN_MV.
   *               Regression introduced by FR-14 (#88), which narrowed the
   *               `if` to `> 1` but left the `else if` catching everything.
   *   sent == 1 — OnRxData's LinkCheck evaluation owns the verdict. It runs
   *               AFTER us in the same LoRaMacProcess pass (LoRaMac.c:
   *               LoRaMacHandleRequestEvents -> LoRaMacHandleIndicationEvents),
   *               so touching the state here would race it — the FR-14 (#88)
   *               finding, which stays fixed.
   * Only sent > 1 is a terminal condition this callback may decide. */
  if (g_tx_state == TX_STATE_BULK_TRANSFER && g_bulk_packets_sent > 1) {
    if (FlashLog_HasUnsentData(&hflashlog) &&
        g_bulk_packets_sent < CfgMaxBulkPkts()) {
      SONDE_LOG_STR("OnTxData: Re-arming bulk transfer (next packet)...\r\n");
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
    } else {
      SONDE_LOG_STR("OnTxData: Bulk transfer complete\r\n");
      g_tx_state = TX_STATE_COMPLETE;
      g_bulk_packets_sent = 0;
    }
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  SONDE_LOG_STR("\r\n=== OnJoinRequest Callback ===\r\n");
  SONDE_LOG("  Status: %s (%d)\r\n",
           (joinParams->Status == LORAMAC_HANDLER_SUCCESS) ? "SUCCESS" : "FAILED",
           joinParams->Status);
  SONDE_LOG("  Mode: %s\r\n",
           (joinParams->Mode == ACTIVATION_TYPE_OTAA) ? "OTAA" : "ABP");
  SONDE_LOG("  Datarate: DR%d, TxPower: %d\r\n",
           joinParams->Datarate, joinParams->TxPower);
  
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

/* FR-03 (#85): ST passes two DIFFERENT lengths for the same object —
 * store gets (sizeof(LoRaMacNvmData_t)+7)&~7 (measured 1496), restore gets
 * sizeof(LoRaMacNvmData_t) (1492). The header/CRC must use the LOGICAL
 * length so both sides agree; only the physical flash write needs the
 * 64-bit-padded length. These asserts pin that contract at compile time. */
_Static_assert(sizeof(NvmSlotHeader_t) % 8U == 0U, "NVM slot header must be 8-byte aligned");
_Static_assert(((sizeof(LoRaMacNvmData_t) + 7U) & ~7U) + sizeof(NvmSlotHeader_t) <= FLASH_PAGE_SIZE,
               "padded NVM payload + header must fit one flash page");

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */

  /* USER CODE END OnStoreContextRequest_1 */
  /* FR-03 (#85): logical length = the true object size (what restore checks
   * and what the CRC covers); padded length = what FLASH_IF_Write's 64-bit
   * alignment precondition requires. Using the passed nvm_size (already
   * padded by LmHandler) for the header made every restore reject every
   * slot (1496 != 1492) and read 4 bytes out of bounds past the object. */
  const uint32_t logical_len = sizeof(LoRaMacNvmData_t);
  const uint32_t padded_len  = (logical_len + 7U) & ~7U;
  static uint8_t staging[(sizeof(LoRaMacNvmData_t) + 7U) & ~7U];

  if (nvm == NULL || nvm_size == 0 ||
      padded_len + sizeof(NvmSlotHeader_t) > FLASH_PAGE_SIZE) {
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
  hdr.length = logical_len;
  hdr.crc32 = FlashLog_CRC32((const uint8_t *)nvm, logical_len);

  /* Stage the payload so the padded tail is defined bytes, not an
   * out-of-bounds read past the caller's object. */
  memcpy(staging, nvm, logical_len);
  memset(staging + logical_len, 0, padded_len - logical_len);

  if (FLASH_IF_Erase((void *)slot_addr, FLASH_PAGE_SIZE) != FLASH_IF_OK) {
    SONDE_LOG_STR("NVM store: slot erase FAILED\r\n");
    return;
  }
  if (FLASH_IF_Write((void *)slot_addr, &hdr, sizeof(hdr)) != FLASH_IF_OK ||
      FLASH_IF_Write((void *)(slot_addr + sizeof(hdr)), staging, padded_len) != FLASH_IF_OK) {
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

/* FR-11 (#94): single owner for NVM slot erasure. The commissioning paths in
 * multiregion_context.c used to erase only page 126 via a hardcoded
 * 0x0803F000UL literal, leaving a valid older-generation slot B that restore
 * would then select — the "clean state" erase survived itself. Erasing BOTH
 * slots and resetting the generation counter is the only correct semantic. */
bool LoRaApp_EraseNvmSlots(void)
{
  bool ok = true;
  if (FLASH_IF_Erase(NVM_SLOT_A_ADDR, FLASH_PAGE_SIZE) != FLASH_IF_OK) ok = false;
  if (FLASH_IF_Erase(NVM_SLOT_B_ADDR, FLASH_PAGE_SIZE) != FLASH_IF_OK) ok = false;
  g_nvm_generation = 0;
  return ok;
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

#if ENABLE_GNSS_DETAIL_PACKET  /* FR-19 (#100): sole producer is the debug packet */
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
#endif  /* ENABLE_GNSS_DETAIL_PACKET */

/**
  * @brief  Peek at the head packet WITHOUT removing it (STAB-P3#7, #243)
  * @param  queue: Pointer to queue structure
  * @param  entry: Destination for peeked entry
  * @retval true if successful, false if queue empty
  */
static bool PacketQueue_Peek(PacketQueue_t *queue, PacketQueueEntry_t *entry)
{
  if (queue->count == 0)
    return false;

  *entry = queue->entries[queue->tail];
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

/* FR-19 (#100): PacketQueue_Count deleted — its only callers were log lines;
 * use g_packet_queue.count directly. */