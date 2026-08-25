/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.h
  * @author  MCD Application Team
  * @brief   Header of application of the LRWAN Middleware
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
#ifndef __LORA_APP_H__
#define __LORA_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include "LoRaMacInterfaces.h"
#include "LmHandler.h"   /* F-01 (#245): LmHandlerErrorStatus_t used in prototypes below */
#include "power_model.h"  /* R49: OperatingMode_t lives here now (PWR-SIMPLIFY: VoltageSlope_t deleted) */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* LoraWAN application configuration (Mw is configured by lorawan_conf.h) */
#define ACTIVE_REGION                               LORAMAC_REGION_US915

/* USER CODE BEGIN EC_CAYENNE_LPP */
/*!
 * CAYENNE_LPP is myDevices Application server.
 */
#define CAYENNE_LPP
/* USER CODE END EC_CAYENNE_LPP */

/*!
 * Defines the application data transmission duty cycle. 10s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            300000

/*!
 * LoRaWAN User application port (CayenneLPP for debug/development)
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_USER_APP_PORT                       2

/*!
 * LoRaWAN GNSS Detail port (custom binary format for debug/development)
 * @note Detailed satellite tracking and 3D speed telemetry
 */
#define LORAWAN_GNSS_DETAIL_PORT                    3

/*!
 * LoRaWAN Compact Binary port (10-byte SF10 probe packet - PRODUCTION)
 * @note Optimized for maximum range transmission
 */
#define LORAWAN_COMPACT_PORT                        10

/*!
 * LoRaWAN Bulk Binary port (222-byte SF7 bulk transfer - PRODUCTION)
 * @note High-speed bulk data transfer when link conditions are good
 */
#define LORAWAN_BULK_PORT                           11

/*!
 * LoRaWAN Switch class application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_SWITCH_CLASS_PORT                   4

/*!
 * LoRaWAN default class
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A

/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON

/*!
 * LoRaWAN Default Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when LORAWAN_ADR_STATE is disabled
 */
#define LORAWAN_DEFAULT_DATA_RATE                   DR_0

/*!
 * LoRaWAN Default Tx output power
 * @note LORAWAN_DEFAULT_TX_POWER must be defined in the [XXXX_MIN_TX_POWER - XXXX_MAX_TX_POWER] range,
         else the end-device uses the XXXX_DEFAULT_TX_POWER value
 */
#define LORAWAN_DEFAULT_TX_POWER                    TX_POWER_0

/*!
 * LoRaWAN default activation type
 */
#define LORAWAN_DEFAULT_ACTIVATION_TYPE             ACTIVATION_TYPE_ABP

/*!
 * LoRaWAN force rejoin even if the NVM context is restored
 * @note useful only when context management is enabled by CONTEXT_MANAGEMENT_ENABLED
 */
#define LORAWAN_FORCE_REJOIN_AT_BOOT                false

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * Default Unicast ping slots periodicity
 *
 * \remark periodicity is equal to 2^LORAWAN_DEFAULT_PING_SLOT_PERIODICITY seconds
 *         example: 2^4 = 16 seconds. The end-device will open an Rx slot every 16 seconds.
 */
#define LORAWAN_DEFAULT_PING_SLOT_PERIODICITY       4

/*!
 * Default response timeout for class b and class c confirmed
 * downlink frames in milli seconds.
 *
 * The value shall not be smaller than RETRANSMIT_TIMEOUT plus
 * the maximum time on air.
 */
#define LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT      8000

/* USER CODE BEGIN EC */
/* Override CubeMX defaults for US915 region */
#undef ACTIVE_REGION
#define ACTIVE_REGION                               LORAMAC_REGION_US915

#undef APP_TX_DUTYCYCLE
#define APP_TX_DUTYCYCLE                            300000  /* 5 minutes (300000 ms) */

#undef LORAWAN_ADR_STATE
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_OFF

#undef LORAWAN_DEFAULT_DATA_RATE
#define LORAWAN_DEFAULT_DATA_RATE                   DR_2  /* For larger payloads on US915 */

/* Power Management - Operating Modes */
/* R49: OperatingMode_t moved to power_model.h (included above) */

/* Power Management - Temperature Constraints */
/* PWR-SIMPLIFY Gate A (2026-08-24): the LIVE source is config's
 * gps_temperature_lockout (default -60 C). This macro is documentation
 * only; no consumer remains. */
#define GPS_TEMPERATURE_LOCKOUT  -60  // °C - pack reaches the 3.3 V GPS floor under a 30 s fix near -65 C

/* Adaptive Transmission Strategy */
typedef enum {
    TX_STATE_PROBE_SF10 = 0,      // Send 10-byte at SF10
    TX_STATE_WAIT_PROBE_ACK,      // Waiting for LinkCheckAns
    TX_STATE_BULK_TRANSFER,       // Sending cached packets at SF7
    TX_STATE_COMPLETE             // Done with cycle
} TxState_t;

/* Link quality thresholds for adaptive transmission */
/* 2026-08-11 handoff §6b: these macros are the compile-time DEFAULTS. The
 * live values are SystemConfig_t fields (link_margin_threshold,
 * gateway_count_threshold, bulk_battery_min_mv, max_bulk_packets) — lora_app.c
 * reads them via the Cfg*() helpers, which fall back to these macros when
 * config is unavailable. tests/host/test_burst_fsm.c parses THESE macros, so
 * keep them present and equal to the config.c defaults. */
/* #141: GPS-loss silence - no fresh fix for this long -> radio dark (logging
 * continues, GPS retried every cycle). FLIGHT only (DDR-0018 exempts
 * commissioning). */
/* DDR-0015 BR-STALE-017 (2026-08-13 intent interview): this macro is the binding
 * of the SINGLE stale-position RF budget, and it is 24 h. Boundary convention:
 * age <= 24 h permitted, > 24 h silent (DDR-0015 P-STALE-014).
 *
 * WAS 6 h until 2026-08-13. The 6 h value came from DDR-0016 INV-PWR-009 /
 * BR-PWR-014, which framed the silence as ENERGY conservation ("a stale position
 * is not worth radio energy"). That framing was superseded: the sonde goes silent
 * because it can no longer prove which regulatory region it is in, so the budget
 * is a REGULATORY bound owned by DDR-0015, not an energy bound. DDR-0015
 * BR-STALE-020 additionally forbids any second, independent time-based RF cutoff
 * (e.g. an RTC-sync-age timer). See docs/decisions/merge-ledger-2026-08-13.md
 * section 3 for the full conflict resolution. */

#ifndef GPS_LOSS_SILENCE_S
#define GPS_LOSS_SILENCE_S  (24U * 3600U)  /* 24 h - DDR-0015 BR-STALE-017 */
#endif

/* S-A (#211, 2026-08-12 review): acquisition budget for the #141
 * GPS-loss-silence forced retry. The forced retry MUST carry its own
 * timeout or it is a power-cycle with zero acquisition iterations and the
 * silence can never clear. Deliberately shorter than the 60 s normal
 * budget: this fires in energy-constrained corners. Decision: constant,
 * not config-authoritative (30 s covers a warm start after the dark
 * period with margin). (PWR-SIMPLIFY: the ApplyOperatingMode budget map
 * is gone; the retry-budget argument stands.) */
#ifndef GPS_LOSS_RETRY_TIMEOUT_MS
#define GPS_LOSS_RETRY_TIMEOUT_MS  30000U   /* 30 s */
#endif

#define LINK_MARGIN_THRESHOLD       15   // dB - minimum demod margin for SF7 bulk
#define GATEWAY_COUNT_THRESHOLD     2    // Minimum gateway count for SF7 bulk
#define BULK_BATTERY_MIN_MV         5000 // mV - minimum battery for bulk transfer
#define MAX_BULK_PACKETS_PER_CYCLE  20   // Maximum bulk packets per transmission cycle

/* USER CODE END EC */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Init Lora Application
  */
void LoRaWAN_Init(void);

/* USER CODE BEGIN EFP */
/* Multi-region pre-join support - flag to track join success */
extern volatile uint8_t g_multiregion_join_success;
extern volatile uint8_t g_multiregion_in_prejoin;

void LoRaWAN_SetPreJoinMode(uint8_t enabled);
uint8_t LoRaWAN_GetJoinSuccess(void);
void LoRaWAN_ResetJoinSuccess(void);

/**
 * @brief Reinitialize the entire LoRaWAN stack for a new region
 * @param new_region: Target region to configure after reinit
 * @note This performs a complete stack teardown and rebuild to ensure clean state
 * @note Caller must set DevEUI and call LmHandlerConfigure() after this returns
 * @retval LORAMAC_HANDLER_SUCCESS on full teardown+rebuild, else ERROR
 *         (F-01 #245: every lifecycle step is checked; failure is fatal to
 *         the stack, so callers must fail closed)
 */
LmHandlerErrorStatus_t LoRaApp_ReInitStack(LoRaMacRegion_t new_region);

/**
 * @brief Erase BOTH LoRaWAN NVM context slots (pages 126+127) and reset the
 *        in-RAM slot generation counter (FR-11 / #94)
 * @retval true if both slot pages erased successfully
 * @note  lora_app.c owns the slot addresses; callers must not hardcode them
 */
bool LoRaApp_EraseNvmSlots(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*__LORA_APP_H__*/
