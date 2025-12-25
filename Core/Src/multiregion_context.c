/**
  ******************************************************************************
  * @file    multiregion_context.c
  * @brief   Multi-region LoRaWAN context storage and switching implementation
  ******************************************************************************
  * @attention
  *
  * This module provides context save/restore and seamless switching between
  * LoRaWAN regions (US915, EU868, AS923, etc.) without requiring re-joins.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "multiregion_context.h"
#include "multiregion_h3.h"
#include "flash_if.h"
#include "sys_app.h"
#include "lora_app.h"
#include "LmHandler.h"
#include "LoRaMac.h"
#include "se-identity.h"
#include "stm32wlxx_hal.h"
#include "SEGGER_RTT.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
/* Flash storage location - last 2KB page of STM32WLE5 flash */
#define MULTIREGION_FLASH_BASE_ADDR      0x0803F800  // Last page before end of 256KB flash
#define MULTIREGION_FLASH_PAGE_SIZE      2048

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static MultiRegionStorage_t g_storage;
static bool g_initialized = false;
static uint8_t g_flash_buffer[MULTIREGION_FLASH_PAGE_SIZE] __attribute__((aligned(8)));

/* External variables --------------------------------------------------------*/
extern LmHandlerParams_t LmHandlerParams;

/* Private function prototypes -----------------------------------------------*/
static uint16_t CalculateCRC16(const uint8_t *data, uint32_t length);
static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length);
static bool ValidateContextCRC(MinimalRegionContext_t *ctx);
static void UpdateContextCRC(MinimalRegionContext_t *ctx);
static bool FlashReadStorage(void);
static bool FlashWriteStorage(void);
static int8_t FindContextSlot(LoRaMacRegion_t region);
static bool CaptureCurrentContext(MinimalRegionContext_t *ctx);
static bool RestoreContextToMAC(MinimalRegionContext_t *ctx);
static const char* RegionToString(LoRaMacRegion_t region);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize the multi-region manager
 */
void MultiRegion_Init(void)
{
    if (g_initialized) {
        return;
    }
    
    SEGGER_RTT_WriteString(0, "\r\n=== MultiRegion_Init ===\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "\r\n=== MultiRegion_Init ===\r\n");
    
    // Initialize flash interface
    FLASH_IF_Init(g_flash_buffer);
    
    // Try to restore contexts from flash
    if (FlashReadStorage()) {
        // Validate magic number and version
        if (g_storage.magic == MULTIREGION_MAGIC && g_storage.version == MULTIREGION_VERSION) {
            // Validate CRC
            uint32_t stored_crc = g_storage.crc32;
            g_storage.crc32 = 0;
            uint32_t calculated_crc = CalculateCRC32((uint8_t*)&g_storage, sizeof(g_storage) - 4);
            
            if (stored_crc == calculated_crc) {
                APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Restored %d contexts from flash\r\n", 
                        g_storage.num_valid);
                g_initialized = true;
                return;
            } else {
                APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Flash CRC mismatch, initializing fresh\r\n");
            }
        } else {
            APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Invalid magic/version, initializing fresh\r\n");
        }
    }
    
    // Initialize fresh storage
    memset(&g_storage, 0, sizeof(g_storage));
    g_storage.magic = MULTIREGION_MAGIC;
    g_storage.version = MULTIREGION_VERSION;
    g_storage.active_slot = 0xFF;  // No active region yet
    g_storage.num_valid = 0;
    
    g_initialized = true;
    SEGGER_RTT_WriteString(0, "MultiRegion: Initialized with fresh storage\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Initialized with fresh storage\r\n");
}

/**
 * @brief Get current active region
 */
LoRaMacRegion_t MultiRegion_GetActiveRegion(void)
{
    if (!g_initialized || g_storage.active_slot >= MAX_REGION_CONTEXTS) {
        return LORAMAC_REGION_US915;  // Default fallback
    }
    
    return g_storage.contexts[g_storage.active_slot].region;
}

/**
 * @brief Check if a region has a valid joined context
 */
bool MultiRegion_IsRegionJoined(LoRaMacRegion_t region)
{
    if (!g_initialized) {
        return false;
    }
    
    int8_t slot = FindContextSlot(region);
    if (slot < 0) {
        return false;
    }
    
    MinimalRegionContext_t *ctx = &g_storage.contexts[slot];
    
    // Validate: must have valid DevAddr and pass CRC check
    if (ctx->dev_addr == 0 || ctx->dev_addr == 0xFFFFFFFF) {
        return false;
    }
    
    return ValidateContextCRC(ctx);
}

/**
 * @brief Save current active context to flash
 */
bool MultiRegion_SaveCurrentContext(void)
{
    SEGGER_RTT_WriteString(0, "\r\n=== MultiRegion_SaveCurrentContext START ===\r\n");
    APP_LOG(TS_ON, VLEVEL_M, "\r\n=== MultiRegion_SaveCurrentContext START ===\r\n");
    
    if (!g_initialized) {
        SEGGER_RTT_WriteString(0, "ERROR: Not initialized, cannot save\r\n");
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Not initialized, cannot save\r\n");
        return false;
    }
    
    SEGGER_RTT_WriteString(0, "Checking network activation status...\r\n");
    
    // Find or create slot for current region
    MibRequestConfirm_t mib;
    mib.Type = MIB_NETWORK_ACTIVATION;
    LoRaMacMibGetRequestConfirm(&mib);
    
    if (mib.Param.NetworkActivation == ACTIVATION_TYPE_NONE) {
        SEGGER_RTT_WriteString(0, "ERROR: Not joined, cannot save context\r\n");
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Not joined, cannot save context\r\n");
        return false;
    }
    
    SEGGER_RTT_WriteString(0, "Network is activated, proceeding...\r\n");
    
    LoRaMacRegion_t current_region = LmHandlerParams.ActiveRegion;
    char region_msg[64];
    snprintf(region_msg, sizeof(region_msg), "Current region: %s\r\n", RegionToString(current_region));
    SEGGER_RTT_WriteString(0, region_msg);
    
    int8_t slot = FindContextSlot(current_region);
    
    if (slot < 0) {
        SEGGER_RTT_WriteString(0, "No existing slot found, searching for empty slot...\r\n");
        // Find empty slot
        for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
            if (g_storage.contexts[i].dev_addr == 0 || 
                g_storage.contexts[i].dev_addr == 0xFFFFFFFF) {
                slot = i;
                g_storage.num_valid++;
                char slot_msg[64];
                snprintf(slot_msg, sizeof(slot_msg), "Found empty slot: %d\r\n", i);
                SEGGER_RTT_WriteString(0, slot_msg);
                break;
            }
        }
    } else {
        char slot_msg[64];
        snprintf(slot_msg, sizeof(slot_msg), "Using existing slot: %d\r\n", slot);
        SEGGER_RTT_WriteString(0, slot_msg);
    }
    
    if (slot < 0) {
        SEGGER_RTT_WriteString(0, "ERROR: No available slots\r\n");
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: No available slots\r\n");
        return false;
    }
    
    // Capture current context
    SEGGER_RTT_WriteString(0, "Calling CaptureCurrentContext...\r\n");
    MinimalRegionContext_t *ctx = &g_storage.contexts[slot];
    if (!CaptureCurrentContext(ctx)) {
        SEGGER_RTT_WriteString(0, "ERROR: Failed to capture context\r\n");
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Failed to capture context\r\n");
        return false;
    }
    
    SEGGER_RTT_WriteString(0, "Context captured successfully\r\n");
    
    g_storage.active_slot = slot;
    
    // Save to flash
    SEGGER_RTT_WriteString(0, "Calling FlashWriteStorage...\r\n");
    bool result = FlashWriteStorage();
    
    if (result) {
        SEGGER_RTT_WriteString(0, "Flash write successful!\r\n");
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Saved %s context (slot %d)\r\n", 
                RegionToString(current_region), slot);
    } else {
        SEGGER_RTT_WriteString(0, "ERROR: Flash write failed!\r\n");
    }
    
    SEGGER_RTT_WriteString(0, "=== MultiRegion_SaveCurrentContext END ===\r\n\r\n");
    
    return result;
}

/**
 * @brief Save all contexts to flash
 */
bool MultiRegion_SaveAllContexts(void)
{
    if (!g_initialized) {
        return false;
    }
    
    return FlashWriteStorage();
}

/**
 * @brief Restore all contexts from flash on boot
 */
bool MultiRegion_RestoreContexts(void)
{
    if (!g_initialized) {
        MultiRegion_Init();
    }
    
    return g_storage.num_valid > 0;
}

/**
 * @brief Switch to a different region context
 */
LmHandlerErrorStatus_t MultiRegion_SwitchToRegion(LoRaMacRegion_t region)
{
    if (!g_initialized) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Not initialized\r\n");
        return LORAMAC_HANDLER_ERROR;
    }
    
    // Check if already on this region
    if (g_storage.active_slot < MAX_REGION_CONTEXTS && 
       g_storage.contexts[g_storage.active_slot].region == region) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Already on %s\r\n", RegionToString(region));
        return LORAMAC_HANDLER_SUCCESS;
    }
    
    // Find target context
    int8_t slot = FindContextSlot(region);
    if (slot < 0) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Region %s not joined\r\n", RegionToString(region));
        return LORAMAC_HANDLER_ERROR;
    }
    
    MinimalRegionContext_t *ctx = &g_storage.contexts[slot];
    if (!ValidateContextCRC(ctx)) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Context CRC validation failed\r\n");
        return LORAMAC_HANDLER_ERROR;
    }
    
    // Check if MAC is busy
    if (LoRaMacIsBusy()) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: MAC busy, cannot switch\r\n");
        return LORAMAC_HANDLER_BUSY_ERROR;
    }
    
    SEGGER_RTT_printf(0, "\r\n=== Switching to %s (slot %d) ===\r\n", 
            RegionToString(region), slot);
    APP_LOG(TS_ON, VLEVEL_H, "\r\n=== Switching to %s (slot %d) ===\r\n", 
            RegionToString(region), slot);
    
    // Save current context before switching
    if (g_storage.active_slot < MAX_REGION_CONTEXTS) {
        CaptureCurrentContext(&g_storage.contexts[g_storage.active_slot]);
    }
    
    // Do full stack reinit to properly reconfigure radio for target region
    SEGGER_RTT_WriteString(0, "Performing full stack reinit for region switch...\r\n");
    LoRaApp_ReInitStack(region);
    
    // Allow stack to stabilize and process any pending events
    HAL_Delay(100);
    LmHandlerProcess();
    
    // Set DevEUI for target region
    LmHandlerSetDevEUI(ctx->dev_eui);
    SEGGER_RTT_printf(0, "Set DevEUI: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            ctx->dev_eui[0], ctx->dev_eui[1], ctx->dev_eui[2], ctx->dev_eui[3],
            ctx->dev_eui[4], ctx->dev_eui[5], ctx->dev_eui[6], ctx->dev_eui[7]);
    
    // Configure stack for target region
    SEGGER_RTT_WriteString(0, "Configuring stack for target region...\r\n");
    LmHandlerConfigure(&LmHandlerParams);
    
    // Restore saved OTAA session as ABP to avoid re-join
    MibRequestConfirm_t mib;
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&mib);
    LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t*)mib.Param.Contexts;
    
    if (nvm) {
        // Restore session keys
        memcpy(nvm->SecureElement.KeyList[APP_S_KEY].KeyValue, ctx->app_s_key, 16);
        memcpy(nvm->SecureElement.KeyList[NWK_S_ENC_KEY].KeyValue, ctx->nwk_s_key, 16);
        
        // Restore DevAddr
        nvm->MacGroup2.DevAddr = ctx->dev_addr;
        
        // Restore frame counters
        nvm->Crypto.FCntList.FCntUp = ctx->uplink_counter;
        nvm->Crypto.FCntList.NFCntDown = ctx->downlink_counter;
        nvm->MacGroup1.LastRxMic = ctx->last_rx_mic;
        
        // Mark as ABP activated
        nvm->MacGroup2.NetworkActivation = ACTIVATION_TYPE_ABP;
        
        SEGGER_RTT_printf(0, "Restored session: DevAddr=0x%08lX, FCnt=%lu\r\n",
                ctx->dev_addr, ctx->uplink_counter);
        
        // Debug: Print session keys for verification
        SEGGER_RTT_WriteString(0, "AppSKey: ");
        for (int i = 0; i < 16; i++) {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X ", ctx->app_s_key[i]);
            SEGGER_RTT_WriteString(0, hex);
        }
        SEGGER_RTT_WriteString(0, "\r\n");
        
        SEGGER_RTT_WriteString(0, "NwkSKey: ");
        for (int i = 0; i < 16; i++) {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X ", ctx->nwk_s_key[i]);
            SEGGER_RTT_WriteString(0, hex);
        }
        SEGGER_RTT_WriteString(0, "\r\n");
    }
    
    // Set activation via MIB as well
    mib.Type = MIB_NETWORK_ACTIVATION;
    mib.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
    LoRaMacMibSetRequestConfirm(&mib);
    
    // Start MAC and allow state machine to stabilize
    LoRaMacStart();
    HAL_Delay(200);
    
    // CRITICAL: Process MAC events to complete initialization
    // Without this, MAC remains in internal busy state after first TX
    for (int i = 0; i < 10; i++) {
        LmHandlerProcess();
        HAL_Delay(10);
    }
    
    // Verify MAC is not busy before proceeding
    if (LoRaMacIsBusy()) {
        SEGGER_RTT_WriteString(0, "WARNING: MAC still busy after initialization\r\n");
        // Give it more time
        HAL_Delay(500);
        for (int i = 0; i < 20; i++) {
            LmHandlerProcess();
            HAL_Delay(10);
        }
    }
    
    // Final MAC state verification
    if (LoRaMacIsBusy()) {
        SEGGER_RTT_WriteString(0, "ERROR: MAC is busy after full initialization!\r\n");
        return LORAMAC_HANDLER_BUSY_ERROR;
    }
    SEGGER_RTT_WriteString(0, "MAC verified idle and ready for TX\r\n");
    
    g_storage.active_slot = slot;
    ctx->last_used = HAL_GetTick();
    
    SEGGER_RTT_printf(0, "Successfully switched to %s\r\n", RegionToString(region));
    APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Successfully switched to %s\r\n", 
            RegionToString(region));
    
    return LORAMAC_HANDLER_SUCCESS;
}

/**
 * @brief Auto-switch based on GPS location
 */
LmHandlerErrorStatus_t MultiRegion_AutoSwitchForLocation(float lat, float lon)
{
    // Check if auto-switching is enabled
    #if MULTIREGION_AUTO_SWITCH_ENABLED == 0
    // Disabled - just return success
    return LORAMAC_HANDLER_SUCCESS;
    #endif
    
    if (!g_initialized) {
        return LORAMAC_HANDLER_ERROR;
    }
    
    // Use H3Lite to detect region
    LoRaMacRegion_t target_region = MultiRegion_DetectFromGPS_H3(lat, lon);
    LoRaMacRegion_t current_region = MultiRegion_GetActiveRegion();
    
    if (target_region == current_region) {
        return LORAMAC_HANDLER_SUCCESS;  // No switch needed
    }
    
    APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: GPS suggests switch %s -> %s\r\n",
            RegionToString(current_region), RegionToString(target_region));
    
    // Check if target region is joined
    if (!MultiRegion_IsRegionJoined(target_region)) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Target region not joined, staying on %s\r\n",
                RegionToString(current_region));
        return LORAMAC_HANDLER_SUCCESS;  // Not an error, just stay on current
    }
    
    // Perform the switch
    return MultiRegion_SwitchToRegion(target_region);
}

/**
 * @brief Get context storage statistics
 */
void MultiRegion_GetStats(uint8_t *total_slots, uint8_t *used_slots)
{
    if (total_slots) {
        *total_slots = MAX_REGION_CONTEXTS;
    }
    if (used_slots) {
        *used_slots = g_initialized ? g_storage.num_valid : 0;
    }
}

/**
 * @brief Clear all stored contexts
 */
bool MultiRegion_ClearAllContexts(void)
{
    APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Clearing all contexts\r\n");
    
    memset(&g_storage, 0, sizeof(g_storage));
    g_storage.magic = MULTIREGION_MAGIC;
    g_storage.version = MULTIREGION_VERSION;
    g_storage.active_slot = 0xFF;
    g_storage.num_valid = 0;
    
    return FlashWriteStorage();
}

/**
 * @brief Join a specific region and store context
 */
LmHandlerErrorStatus_t MultiRegion_JoinRegion(LoRaMacRegion_t region)
{
    // Access external join success flag from lora_app.c
    extern volatile uint8_t g_multiregion_join_success;
    
    if (!g_initialized) {
        MultiRegion_Init();
    }
    
    SEGGER_RTT_printf(0, "\r\n=== Joining region %s ===\r\n", RegionToString(region));
    APP_LOG(TS_ON, VLEVEL_H, "\r\n=== Joining region %s ===\r\n", RegionToString(region));
    
    // Check if this is a region change (not first join)
    MibRequestConfirm_t check_mib;
    check_mib.Type = MIB_NETWORK_ACTIVATION;
    LoRaMacMibGetRequestConfirm(&check_mib);
    
    if (check_mib.Param.NetworkActivation != ACTIVATION_TYPE_NONE) {
        // We were already joined to another region - full reset needed
        SEGGER_RTT_WriteString(0, "Previous join detected - performing full stack reset...\r\n");
        LoRaApp_ReInitStack(region);
        // LoRaApp_ReInitStack sets ActiveRegion but doesn't configure
        // We'll set DevEUI and configure below
    }
    else {
        // First join - set region but don't configure yet
        SEGGER_RTT_WriteString(0, "First join - setting region parameter...\r\n");
        LmHandlerParams.ActiveRegion = region;
        
        // CRITICAL: Erase LoRaWAN NVM to prevent DevEUI restoration
        SEGGER_RTT_WriteString(0, "Erasing LoRaWAN NVM to ensure clean state...\r\n");
        extern FLASH_IF_StatusTypedef FLASH_IF_Erase(void *pFlashAddress, uint32_t page_size);
        FLASH_IF_Erase((void*)0x0803F000UL, 2048);  // LORAWAN_NVM_BASE_ADDRESS
        HAL_Delay(100);
    }
    
    // CRITICAL: Set region-specific DevEUI AFTER stack reinit (if any)
    // Use LmHandlerSetDevEUI() instead of MIB (MIB has pointer issues)
    uint8_t deveui[8];
    
    // Choose DevEUI based on region
    const uint8_t deveui_us915[] = {LORAWAN_DEVICE_EUI_US915};
    const uint8_t deveui_eu868[] = {LORAWAN_DEVICE_EUI_EU868};
    const uint8_t deveui_as923[] = {LORAWAN_DEVICE_EUI_AS923};
    const uint8_t deveui_au915[] = {LORAWAN_DEVICE_EUI_AU915};
    const uint8_t deveui_in865[] = {LORAWAN_DEVICE_EUI_IN865};
    const uint8_t deveui_kr920[] = {LORAWAN_DEVICE_EUI_KR920};
    
    switch (region) {
        case LORAMAC_REGION_US915:
            memcpy(deveui, deveui_us915, 8);
            break;
        case LORAMAC_REGION_EU868:
            memcpy(deveui, deveui_eu868, 8);
            break;
        case LORAMAC_REGION_AS923:
            memcpy(deveui, deveui_as923, 8);
            break;
        case LORAMAC_REGION_AU915:
            memcpy(deveui, deveui_au915, 8);
            break;
        case LORAMAC_REGION_IN865:
            memcpy(deveui, deveui_in865, 8);
            break;
        case LORAMAC_REGION_KR920:
            memcpy(deveui, deveui_kr920, 8);
            break;
        default:
            APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Unsupported region %d\r\n", region);
            return LORAMAC_HANDLER_ERROR;
    }
    
    // Use LmHandlerSetDevEUI() API
    LmHandlerSetDevEUI(deveui);
    SEGGER_RTT_printf(0, "Set DevEUI via LmHandler: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            deveui[0], deveui[1], deveui[2], deveui[3],
            deveui[4], deveui[5], deveui[6], deveui[7]);
    
    // NOW configure the stack with the correct DevEUI already set
    SEGGER_RTT_WriteString(0, "Configuring stack for region with DevEUI set...\r\n");
    LmHandlerConfigure(&LmHandlerParams);
    
    // CRITICAL: Set DevEUI again after Configure (Configure might restore from NVM)
    LmHandlerSetDevEUI(deveui);  // Set it again using correct API to ensure it sticks
    SEGGER_RTT_printf(0, "Re-set DevEUI after Configure: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            deveui[0], deveui[1], deveui[2], deveui[3],
            deveui[4], deveui[5], deveui[6], deveui[7]);
    
    // CRITICAL: Set JoinEUI (AppEUI) - must be set before join
    const uint8_t joineui[] = FORMAT32_KEY(LORAWAN_JOIN_EUI);
    LmHandlerSetAppEUI((uint8_t*)joineui);
    SEGGER_RTT_printf(0, "Set JoinEUI: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            joineui[0], joineui[1], joineui[2], joineui[3],
            joineui[4], joineui[5], joineui[6], joineui[7]);
    
    // CRITICAL: Set AppKey and NwkKey - must be set before join
    const uint8_t appkey[] = FORMAT_KEY(LORAWAN_APP_KEY);
    const uint8_t nwkkey[] = FORMAT_KEY(LORAWAN_NWK_KEY);
    LmHandlerSetKey(APP_KEY, (uint8_t*)appkey);
    LmHandlerSetKey(NWK_KEY, (uint8_t*)nwkkey);
    SEGGER_RTT_WriteString(0, "Set AppKey and NwkKey\r\n");
    
    // Reset join success flag before triggering join
    g_multiregion_join_success = false;
    
    // Trigger join (LmHandlerJoin returns void)
    LmHandlerJoin(ACTIVATION_TYPE_OTAA, true);
    
    // Wait for join to complete - infinite retry until success
    uint32_t start_time = HAL_GetTick();
    uint32_t last_join_attempt = HAL_GetTick();
    uint32_t retry_interval = 30000;  // Retry every 30 seconds
    
    SEGGER_RTT_printf(0, "Waiting for %s join (infinite retry)...\r\n", RegionToString(region));
    
    // Wait for join to complete by checking callback flag
    while (!g_multiregion_join_success) {
        // CRITICAL: Process MAC events to handle join accept
        LmHandlerProcess();
        
        // Check if we need to retry join (every 30 seconds)
        if ((HAL_GetTick() - last_join_attempt) > retry_interval) {
            SEGGER_RTT_printf(0, "Retrying %s join...\r\n", RegionToString(region));
            LmHandlerJoin(ACTIVATION_TYPE_OTAA, true);
            last_join_attempt = HAL_GetTick();
        }
        
        // Delay to prevent tight loop (250ms is sufficient for MAC processing)
        HAL_Delay(250);
    }
    
    // Join successful
    uint32_t join_time = (HAL_GetTick() - start_time) / 1000;
    SEGGER_RTT_printf(0, "%s join SUCCESS! (took %lus)\r\n", RegionToString(region), join_time);
    APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Join successful for %s (took %lus)\r\n", 
            RegionToString(region), join_time);
    
    // Save the joined context
    HAL_Delay(500);  // Give MAC time to stabilize
    MultiRegion_SaveCurrentContext();
    
    return LORAMAC_HANDLER_SUCCESS;
}

/**
 * @brief Pre-join all required regions (ground operations)
 */
bool MultiRegion_PreJoinAllRegions(void)
{
    SEGGER_RTT_WriteString(0, "\r\n========================================\r\n");
    SEGGER_RTT_WriteString(0, "=== MULTI-REGION PRE-JOIN SEQUENCE ===\r\n");
    SEGGER_RTT_WriteString(0, "========================================\r\n\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "\r\n========================================\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "=== MULTI-REGION PRE-JOIN SEQUENCE ===\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "========================================\r\n\r\n");
    
    // CRITICAL: Erase LoRaWAN NVM first to prevent old DevEUI restoration
    SEGGER_RTT_WriteString(0, "Erasing LoRaWAN NVM for clean multi-region start...\r\n");
    FLASH_IF_Erase((void*)0x0803F000UL, 2048);  // LORAWAN_NVM_BASE_ADDRESS
    HAL_Delay(100);
    
    bool all_success = true;
    
    // Join US915
    if (MultiRegion_JoinRegion(LORAMAC_REGION_US915) != LORAMAC_HANDLER_SUCCESS) {
        APP_LOG(TS_ON, VLEVEL_H, "FAILED: US915 join\r\n");
        all_success = false;
    } else {
        APP_LOG(TS_ON, VLEVEL_H, "SUCCESS: US915 joined\r\n");
    }
    HAL_Delay(5000);
    
    // Join EU868
    if (MultiRegion_JoinRegion(LORAMAC_REGION_EU868) != LORAMAC_HANDLER_SUCCESS) {
        APP_LOG(TS_ON, VLEVEL_H, "FAILED: EU868 join\r\n");
        all_success = false;
    } else {
        APP_LOG(TS_ON, VLEVEL_H, "SUCCESS: EU868 joined\r\n");
    }
    HAL_Delay(5000);
    
    // Optionally join AS923
    /*
    if (MultiRegion_JoinRegion(LORAMAC_REGION_AS923) != LORAMAC_HANDLER_SUCCESS) {
        APP_LOG(TS_ON, VLEVEL_H, "FAILED: AS923 join\r\n");
        all_success = false;
    } else {
        APP_LOG(TS_ON, VLEVEL_H, "SUCCESS: AS923 joined\r\n");
    }
    HAL_Delay(5000);
    */
    
    // Switch back to US915 as starting region
    MultiRegion_SwitchToRegion(LORAMAC_REGION_US915);
    
    APP_LOG(TS_ON, VLEVEL_H, "\r\n========================================\r\n");
    if (all_success) {
        APP_LOG(TS_ON, VLEVEL_H, "=== ALL PRE-JOINS SUCCESSFUL ===\r\n");
    } else {
        APP_LOG(TS_ON, VLEVEL_H, "=== SOME PRE-JOINS FAILED ===\r\n");
    }
    APP_LOG(TS_ON, VLEVEL_H, "========================================\r\n\r\n");
    
    return all_success;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Calculate CRC16 for context validation
 */
static uint16_t CalculateCRC16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Calculate CRC32 for storage validation
 */
static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x00000001) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return ~crc;
}

/**
 * @brief Validate context CRC
 */
static bool ValidateContextCRC(MinimalRegionContext_t *ctx)
{
    uint16_t stored_crc = ctx->crc16;
    ctx->crc16 = 0;
    uint16_t calculated_crc = CalculateCRC16((uint8_t*)ctx, sizeof(MinimalRegionContext_t) - 2);
    ctx->crc16 = stored_crc;
    
    return (stored_crc == calculated_crc);
}

/**
 * @brief Update context CRC
 */
static void UpdateContextCRC(MinimalRegionContext_t *ctx)
{
    ctx->crc16 = 0;
    ctx->crc16 = CalculateCRC16((uint8_t*)ctx, sizeof(MinimalRegionContext_t) - 2);
}

/**
 * @brief Read storage from flash
 */
static bool FlashReadStorage(void)
{
    FLASH_IF_StatusTypedef status = FLASH_IF_Read(&g_storage, 
                                                    (void*)MULTIREGION_FLASH_BASE_ADDR, 
                                                    sizeof(MultiRegionStorage_t));
    
    return (status == FLASH_IF_OK);
}

/**
 * @brief Write storage to flash
 */
static bool FlashWriteStorage(void)
{
    // Update CRC
    g_storage.crc32 = 0;
    g_storage.crc32 = CalculateCRC32((uint8_t*)&g_storage, sizeof(g_storage) - 4);
    
    // Erase flash page
    FLASH_IF_StatusTypedef status = FLASH_IF_Erase((void*)MULTIREGION_FLASH_BASE_ADDR, 
                                                     MULTIREGION_FLASH_PAGE_SIZE);
    
    if (status != FLASH_IF_OK) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Flash erase failed\r\n");
        return false;
    }
    
    // Write to flash
    status = FLASH_IF_Write((void*)MULTIREGION_FLASH_BASE_ADDR, 
                            &g_storage, 
                            sizeof(MultiRegionStorage_t));
    
    if (status != FLASH_IF_OK) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Flash write failed\r\n");
        return false;
    }
    
    return true;
}

/**
 * @brief Find context slot for a region
 */
static int8_t FindContextSlot(LoRaMacRegion_t region)
{
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        if (g_storage.contexts[i].region == region) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Capture current MAC context
 */
static bool CaptureCurrentContext(MinimalRegionContext_t *ctx)
{
    if (!ctx) {
        return false;
    }
    
    MibRequestConfirm_t mib;
    
    // Get region
    ctx->region = LmHandlerParams.ActiveRegion;
    
    // Capture current DevEUI (determine from region instead of reading from MAC)
    const uint8_t deveui_us915[] = {LORAWAN_DEVICE_EUI_US915};
    const uint8_t deveui_eu868[] = {LORAWAN_DEVICE_EUI_EU868};
    const uint8_t deveui_as923[] = {LORAWAN_DEVICE_EUI_AS923};
    const uint8_t deveui_au915[] = {LORAWAN_DEVICE_EUI_AU915};
    const uint8_t deveui_in865[] = {LORAWAN_DEVICE_EUI_IN865};
    const uint8_t deveui_kr920[] = {LORAWAN_DEVICE_EUI_KR920};
    
    switch (ctx->region) {
        case LORAMAC_REGION_US915:
            memcpy(ctx->dev_eui, deveui_us915, 8);
            break;
        case LORAMAC_REGION_EU868:
            memcpy(ctx->dev_eui, deveui_eu868, 8);
            break;
        case LORAMAC_REGION_AS923:
            memcpy(ctx->dev_eui, deveui_as923, 8);
            break;
        case LORAMAC_REGION_AU915:
            memcpy(ctx->dev_eui, deveui_au915, 8);
            break;
        case LORAMAC_REGION_IN865:
            memcpy(ctx->dev_eui, deveui_in865, 8);
            break;
        case LORAMAC_REGION_KR920:
            memcpy(ctx->dev_eui, deveui_kr920, 8);
            break;
        default:
            memcpy(ctx->dev_eui, deveui_us915, 8);  // Fallback
            break;
    }
    
    // Get activation type
    mib.Type = MIB_NETWORK_ACTIVATION;
    LoRaMacMibGetRequestConfirm(&mib);
    ctx->activation = mib.Param.NetworkActivation;
    
    // Get DevAddr
    mib.Type = MIB_DEV_ADDR;
    LoRaMacMibGetRequestConfirm(&mib);
    ctx->dev_addr = mib.Param.DevAddr;
    
    // Get NVM contexts to extract keys and counters
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&mib);
    LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t*)mib.Param.Contexts;
    
    if (nvm) {
        // Copy session keys
        memcpy(ctx->app_s_key, nvm->SecureElement.KeyList[APP_S_KEY].KeyValue, 16);
        memcpy(ctx->nwk_s_key, nvm->SecureElement.KeyList[NWK_S_ENC_KEY].KeyValue, 16);
        
        // Copy frame counters
        ctx->uplink_counter = nvm->Crypto.FCntList.FCntUp;
        ctx->downlink_counter = nvm->Crypto.FCntList.NFCntDown;
        ctx->last_rx_mic = nvm->MacGroup1.LastRxMic;
    }
    
    // Get datarate
    mib.Type = MIB_CHANNELS_DATARATE;
    LoRaMacMibGetRequestConfirm(&mib);
    ctx->datarate = mib.Param.ChannelsDatarate;
    
    // Get TX power
    mib.Type = MIB_CHANNELS_TX_POWER;
    LoRaMacMibGetRequestConfirm(&mib);
    ctx->tx_power = mib.Param.ChannelsTxPower;
    
    // Get ADR
    mib.Type = MIB_ADR;
    LoRaMacMibGetRequestConfirm(&mib);
    ctx->adr_enabled = mib.Param.AdrEnable ? 1 : 0;
    
    // Get RX2 params
    mib.Type = MIB_RX2_CHANNEL;
    LoRaMacMibGetRequestConfirm(&mib);
    ctx->rx2_frequency = mib.Param.Rx2Channel.Frequency;
    ctx->rx2_datarate = mib.Param.Rx2Channel.Datarate;
    
    // Update timestamp
    ctx->last_used = HAL_GetTick();
    
    // Calculate CRC
    UpdateContextCRC(ctx);
    
    APP_LOG(TS_ON, VLEVEL_M, "Captured context: DevAddr=0x%08lX, FCntUp=%lu, FCntDown=%lu\r\n",
            ctx->dev_addr, ctx->uplink_counter, ctx->downlink_counter);
    
    return true;
}

/**
 * @brief Restore context to MAC layer
 */
static bool RestoreContextToMAC(MinimalRegionContext_t *ctx)
{
    if (!ctx) {
        return false;
    }
    
    APP_LOG(TS_ON, VLEVEL_M, "Restoring context: DevAddr=0x%08lX, FCntUp=%lu, FCntDown=%lu\r\n",
            ctx->dev_addr, ctx->uplink_counter, ctx->downlink_counter);
    
    MibRequestConfirm_t mib;
    
    // CRITICAL: Restore DevEUI first (before region configuration)
    mib.Type = MIB_DEV_EUI;
    memcpy(mib.Param.DevEui, ctx->dev_eui, 8);
    LoRaMacMibSetRequestConfirm(&mib);
    
APP_LOG(TS_ON, VLEVEL_M, "Restored DevEUI: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            ctx->dev_eui[0], ctx->dev_eui[1], ctx->dev_eui[2], ctx->dev_eui[3],
            ctx->dev_eui[4], ctx->dev_eui[5], ctx->dev_eui[6], ctx->dev_eui[7]);
    
    // Reconfigure for target region
    LmHandlerParams.ActiveRegion = ctx->region;
    LmHandlerConfigure(&LmHandlerParams);
    
    // Get NVM context pointer
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&mib);
    LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t*)mib.Param.Contexts;
    
    if (nvm) {
        // Restore session keys (KeyList[] returns a Key_t struct, access .KeyValue)
        memcpy(nvm->SecureElement.KeyList[APP_S_KEY].KeyValue, ctx->app_s_key, 16);
        memcpy(nvm->SecureElement.KeyList[NWK_S_ENC_KEY].KeyValue, ctx->nwk_s_key, 16);
        
        // Restore DevAddr
        nvm->MacGroup2.DevAddr = ctx->dev_addr;
        
        // CRITICAL: Restore frame counters
        nvm->Crypto.FCntList.FCntUp = ctx->uplink_counter;
        nvm->Crypto.FCntList.NFCntDown = ctx->downlink_counter;
        nvm->MacGroup1.LastRxMic = ctx->last_rx_mic;
        
        // Mark as activated (treat as ABP to skip join)
        nvm->MacGroup2.NetworkActivation = ACTIVATION_TYPE_ABP;
    }
    
    // Set DevAddr via MIB
    mib.Type = MIB_DEV_ADDR;
    mib.Param.DevAddr = ctx->dev_addr;
    LoRaMacMibSetRequestConfirm(&mib);
    
    // Set network activation
    mib.Type = MIB_NETWORK_ACTIVATION;
    mib.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
    LoRaMacMibSetRequestConfirm(&mib);
    
    // Restore datarate
    mib.Type = MIB_CHANNELS_DATARATE;
    mib.Param.ChannelsDatarate = ctx->datarate;
    LoRaMacMibSetRequestConfirm(&mib);
    
    // Restore ADR
    mib.Type = MIB_ADR;
    mib.Param.AdrEnable = ctx->adr_enabled ? true : false;
    LoRaMacMibSetRequestConfirm(&mib);
    
    // Start MAC
    LoRaMacStart();
    
    return true;
}

/**
 * @brief Convert region enum to string
 */
__attribute__((unused)) static const char* RegionToString(LoRaMacRegion_t region)
{
    switch (region) {
        case LORAMAC_REGION_AS923: return "AS923";
        case LORAMAC_REGION_AU915: return "AU915";
        case LORAMAC_REGION_CN470: return "CN470";
        case LORAMAC_REGION_CN779: return "CN779";
        case LORAMAC_REGION_EU433: return "EU433";
        case LORAMAC_REGION_EU868: return "EU868";
        case LORAMAC_REGION_KR920: return "KR920";
        case LORAMAC_REGION_IN865: return "IN865";
        case LORAMAC_REGION_US915: return "US915";
        case LORAMAC_REGION_RU864: return "RU864";
        default: return "UNKNOWN";
    }
}
