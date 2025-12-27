/**
  ******************************************************************************
  * @file    multiregion_context.h
  * @brief   Multi-region LoRaWAN context storage and switching
  ******************************************************************************
  * @attention
  *
  * This module provides context save/restore and seamless switching between
  * LoRaWAN regions (US915, EU868, AS923, etc.) without requiring re-joins.
  *
  * Stores minimal 83-byte session context per region in flash memory.
  * Total storage: 510 bytes for 6 regions (includes DevEUI per region).
  *
  ******************************************************************************
  */

#ifndef MULTIREGION_CONTEXT_H
#define MULTIREGION_CONTEXT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "LoRaMacInterfaces.h"
#include "LmHandler.h"

/* Exported defines ----------------------------------------------------------*/
/**
 * @brief Enable/disable automatic region switching
 * Set to 0 to disable region switching (for testing)
 * Set to 1 to enable automatic region switching based on GPS
 */
#ifndef MULTIREGION_AUTO_SWITCH_ENABLED
#define MULTIREGION_AUTO_SWITCH_ENABLED  0  // Disabled by default for safety
#endif

#define MAX_REGION_CONTEXTS              6  // US915, EU868, AS923, AU915, IN865, KR920

/* Magic number for flash storage validation */
#define MULTIREGION_MAGIC                0xDEADBEEF
#define MULTIREGION_VERSION              1

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Minimal region context structure
 * Stores only essential session data for region switching including unique DevEUI per region
 */
typedef struct {
    LoRaMacRegion_t region;              // 1 byte - US915, EU868, etc.
    uint8_t dev_eui[8];                  // 8 bytes - Unique DevEUI for this region
    uint8_t activation;                   // 1 byte - OTAA/ABP
    uint32_t dev_addr;                    // 4 bytes - Device address
    
    // Session keys (32 bytes total)
    uint8_t app_s_key[16];               // 16 bytes - Application session key
    uint8_t nwk_s_key[16];               // 16 bytes - Network session key
    
    // Frame counters (critical for security)
    uint32_t uplink_counter;             // 4 bytes
    uint32_t downlink_counter;           // 4 bytes
    
    // Minimal state
    uint32_t last_rx_mic;                // 4 bytes - Replay protection
    uint8_t datarate;                    // 1 byte
    int8_t tx_power;                     // 1 byte
    uint8_t adr_enabled;                 // 1 byte (using uint8_t instead of bool for packing)
    
    // RX2 window params
    uint32_t rx2_frequency;              // 4 bytes
    uint8_t rx2_datarate;                // 1 byte
    
    // Metadata
    uint32_t last_used;                  // 4 bytes - LRU timestamp
    uint16_t crc16;                      // 2 bytes - Context validation
    
} MinimalRegionContext_t;

/**
 * @brief Multi-region storage manager (8-byte aligned for flash, size determined by compiler)
 * Forces 8-byte alignment to meet flash driver requirements regardless of optimization
 */
typedef struct {
    uint32_t magic;                      // 0xDEADBEEF - Validity check
    uint8_t active_slot;                 // Currently active context index
    uint8_t num_valid;                   // How many contexts are joined
    uint16_t version;                    // For future compatibility
    
    MinimalRegionContext_t contexts[MAX_REGION_CONTEXTS];
    
    uint32_t crc32;                      // Whole structure validation
} __attribute__((aligned(8))) MultiRegionStorage_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize the multi-region manager
 * @note Call once during system startup, after LoRaWAN init
 * @retval None
 */
void MultiRegion_Init(void);

/**
 * @brief Join a specific region and store context
 * @param region: Target LoRaWAN region to join
 * @retval LmHandlerErrorStatus_t: Join status
 * @note This is a blocking call that waits for join to complete
 */
LmHandlerErrorStatus_t MultiRegion_JoinRegion(LoRaMacRegion_t region);

/**
 * @brief Switch to a different region context
 * @param region: Target LoRaWAN region
 * @retval LmHandlerErrorStatus_t: Switch status
 * @note Only switches if target region has a valid joined context
 */
LmHandlerErrorStatus_t MultiRegion_SwitchToRegion(LoRaMacRegion_t region);

/**
 * @brief Get current active region
 * @retval LoRaMacRegion_t: Currently active region
 */
LoRaMacRegion_t MultiRegion_GetActiveRegion(void);

/**
 * @brief Check if a region has a valid joined context
 * @param region: Region to check
 * @retval bool: true if region is joined and context is valid
 */
bool MultiRegion_IsRegionJoined(LoRaMacRegion_t region);

/**
 * @brief Auto-switch based on GPS location (if enabled)
 * @param lat: Latitude in decimal degrees
 * @param lon: Longitude in decimal degrees
 * @retval LmHandlerErrorStatus_t: Switch status (or success if no switch needed)
 * @note Respects MULTIREGION_AUTO_SWITCH_ENABLED flag
 * @note Uses H3Lite for precise region detection
 */
LmHandlerErrorStatus_t MultiRegion_AutoSwitchForLocation(float lat, float lon);

/**
 * @brief Save current active context to flash
 * @retval bool: true if save successful
 */
bool MultiRegion_SaveCurrentContext(void);

/**
 * @brief Save all contexts to flash
 * @retval bool: true if save successful
 */
bool MultiRegion_SaveAllContexts(void);

/**
 * @brief Restore all contexts from flash on boot
 * @retval bool: true if restore successful
 */
bool MultiRegion_RestoreContexts(void);

/**
 * @brief Get context storage statistics
 * @param total_slots: Pointer to store total slots
 * @param used_slots: Pointer to store used slots
 * @retval None
 */
void MultiRegion_GetStats(uint8_t *total_slots, uint8_t *used_slots);

/**
 * @brief Clear all stored contexts (emergency reset)
 * @retval bool: true if clear successful
 */
bool MultiRegion_ClearAllContexts(void);

/**
 * @brief Pre-join all required regions (ground operations)
 * @note Call this on the ground before flight
 * @note Joins all 6 regions: US915, EU868, AS923, AU915, IN865, KR920
 * @retval bool: true if all pre-joins successful
 */
bool MultiRegion_PreJoinAllRegions(void);

/**
 * @brief Initialize a region context from Chirpstack session keys (ABP)
 * @param region: Target LoRaWAN region
 * @param dev_addr: DevAddr from Chirpstack
 * @param app_s_key: AppSKey from Chirpstack (16 bytes)
 * @param nwk_s_key: NwkSKey from Chirpstack (16 bytes)
 * @retval bool: true if initialization successful
 * @note This creates an ABP context with Chirpstack-provided session keys
 * @note Use this instead of OTAA join when you have the keys from Chirpstack
 */
bool MultiRegion_InitializeRegionFromChirpstack(
    LoRaMacRegion_t region,
    uint32_t dev_addr,
    const uint8_t *app_s_key,
    const uint8_t *nwk_s_key
);

/**
 * @brief Display current session keys for Chirpstack ABP configuration
 * @retval None
 * @note Outputs session keys via SEGGER_RTT for copy/paste into Chirpstack
 */
void MultiRegion_DisplaySessionKeys(void);

#ifdef __cplusplus
}
#endif

#endif /* MULTIREGION_CONTEXT_H */
