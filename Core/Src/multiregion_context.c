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
#include "sys_sensors.h"
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
    // Debug entry
    char entry_msg[128];
    snprintf(entry_msg, sizeof(entry_msg), "\r\n>>> MultiRegion_SwitchToRegion() called for %s\r\n", RegionToString(region));
    SEGGER_RTT_WriteString(0, entry_msg);
    
    if (!g_initialized) {
        SEGGER_RTT_WriteString(0, "ERROR: Not initialized, returning error\r\n");
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Not initialized\r\n");
        return LORAMAC_HANDLER_ERROR;
    }
    
    // Debug current state
    snprintf(entry_msg, sizeof(entry_msg), "Current active_slot: %d, Current region: %s\r\n", 
             g_storage.active_slot,
             g_storage.active_slot < MAX_REGION_CONTEXTS ? RegionToString(g_storage.contexts[g_storage.active_slot].region) : "NONE");
    SEGGER_RTT_WriteString(0, entry_msg);
    
    // Check if already on this region
    if (g_storage.active_slot < MAX_REGION_CONTEXTS && 
       g_storage.contexts[g_storage.active_slot].region == region) {
        SEGGER_RTT_WriteString(0, "Already on target region, returning SUCCESS without switch\r\n");
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
    
    // STEP 1: Reinitialize stack (loads zeros from se-identity.h)
    SEGGER_RTT_WriteString(0, "Step 1: Performing full stack reinit...\r\n");
    LoRaApp_ReInitStack(region);
    HAL_Delay(100);
    
    // STEP 2: Configure the handler (this might load from NVM)
    SEGGER_RTT_WriteString(0, "Step 2: Configuring handler for region...\r\n");
    LmHandlerConfigure(&LmHandlerParams);
    HAL_Delay(50);
    
    // STEP 3: NOW set identity and keys AFTER configure (to override NVM restore)
    SEGGER_RTT_WriteString(0, "Step 3: Setting DevEUI and session keys (overriding NVM)...\r\n");
    LmHandlerSetDevEUI(ctx->dev_eui);
    SEGGER_RTT_printf(0, "  DevEUI set: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            ctx->dev_eui[0], ctx->dev_eui[1], ctx->dev_eui[2], ctx->dev_eui[3],
            ctx->dev_eui[4], ctx->dev_eui[5], ctx->dev_eui[6], ctx->dev_eui[7]);
    
    // Set session keys AFTER configure (critical for multi-region)
    LmHandlerSetKey(APP_S_KEY, (uint8_t*)ctx->app_s_key);
    LmHandlerSetKey(NWK_S_KEY, (uint8_t*)ctx->nwk_s_key);
    SEGGER_RTT_WriteString(0, "  Session keys set\r\n");
    
    // STEP 4: Set DevAddr via MIB before channel mask
    SEGGER_RTT_WriteString(0, "Step 4: Setting DevAddr and activation...\r\n");
    MibRequestConfirm_t mib;
    
    mib.Type = MIB_DEV_ADDR;
    mib.Param.DevAddr = ctx->dev_addr;
    LoRaMacMibSetRequestConfirm(&mib);
    SEGGER_RTT_printf(0, "  DevAddr set: 0x%08lX\r\n", ctx->dev_addr);
    
    // Set activation type
    mib.Type = MIB_NETWORK_ACTIVATION;
    mib.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
    LoRaMacMibSetRequestConfirm(&mib);
    
    // STEP 5: Restore frame counters and VERIFY keys in NVM
    SEGGER_RTT_WriteString(0, "Step 5: Restoring frame counters and verifying keys in NVM...\r\n");
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&mib);
    LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t*)mib.Param.Contexts;
    
    if (nvm) {
        // Restore frame counters
        nvm->Crypto.FCntList.FCntUp = ctx->uplink_counter;
        nvm->Crypto.FCntList.NFCntDown = ctx->downlink_counter;
        nvm->MacGroup1.LastRxMic = ctx->last_rx_mic;
        nvm->MacGroup2.NetworkActivation = ACTIVATION_TYPE_ABP;
        
        // CRITICAL: Verify keys are correctly loaded in secure element NVM
        // LmHandlerSetKey() should have done this, but we double-check
        memcpy(nvm->SecureElement.KeyList[APP_S_KEY].KeyValue, ctx->app_s_key, 16);
        memcpy(nvm->SecureElement.KeyList[NWK_S_KEY].KeyValue, ctx->nwk_s_key, 16);
        
        SEGGER_RTT_printf(0, "  Frame counters: FCntUp=%lu, FCntDown=%lu\r\n",
                          ctx->uplink_counter, ctx->downlink_counter);
        
        // ===== COMPREHENSIVE CONTEXT LOGGING =====
        SEGGER_RTT_WriteString(0, "\r\n----- RESTORED CONTEXT DETAILS -----\r\n");
        
        // Core session parameters
        SEGGER_RTT_printf(0, "Region:       %s\r\n", RegionToString(ctx->region));
        SEGGER_RTT_printf(0, "Activation:   %s\r\n", 
                ctx->activation == ACTIVATION_TYPE_OTAA ? "OTAA" : "ABP");
        SEGGER_RTT_printf(0, "DevAddr:      0x%08lX\r\n", ctx->dev_addr);
        
        // Frame counters
        SEGGER_RTT_printf(0, "FCntUp:       %lu\r\n", ctx->uplink_counter);
        SEGGER_RTT_printf(0, "FCntDown:     %lu\r\n", ctx->downlink_counter);
        SEGGER_RTT_printf(0, "LastRxMic:    0x%08lX\r\n", ctx->last_rx_mic);
        
        // Radio parameters
        SEGGER_RTT_printf(0, "Datarate:     DR%d\r\n", ctx->datarate);
        SEGGER_RTT_printf(0, "TxPower:      %d dBm\r\n", ctx->tx_power);
        SEGGER_RTT_printf(0, "ADR:          %s\r\n", ctx->adr_enabled ? "ON" : "OFF");
        
        // RX2 window
        SEGGER_RTT_printf(0, "RX2 Freq:     %lu Hz\r\n", ctx->rx2_frequency);
        SEGGER_RTT_printf(0, "RX2 DR:       DR%d\r\n", ctx->rx2_datarate);
        
        // Session keys (full 16 bytes)
        SEGGER_RTT_WriteString(0, "AppSKey:      ");
        for (int i = 0; i < 16; i++) {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X ", ctx->app_s_key[i]);
            SEGGER_RTT_WriteString(0, hex);
        }
        SEGGER_RTT_WriteString(0, "\r\n");
        
        SEGGER_RTT_WriteString(0, "NwkSKey:      ");
        for (int i = 0; i < 16; i++) {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X ", ctx->nwk_s_key[i]);
            SEGGER_RTT_WriteString(0, hex);
        }
        SEGGER_RTT_WriteString(0, "\r\n");
        
        // CRC validation
        SEGGER_RTT_printf(0, "Context CRC:  0x%04X (validated)\r\n", ctx->crc16);
        SEGGER_RTT_WriteString(0, "------------------------------------\r\n\r\n");
    }
    
    // Set activation via MIB as well
    mib.Type = MIB_NETWORK_ACTIVATION;
    mib.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
    LoRaMacMibSetRequestConfirm(&mib);
    
    // STEP 6: Configure region-specific channel masks BEFORE starting MAC
    SEGGER_RTT_WriteString(0, "Step 6: Configuring region-specific channel masks...\r\n");
    // During OTAA join, the network configures specific channels (sub-bands for US915)
    // When switching to ABP, we must restore these channel configurations
    
    MibRequestConfirm_t mib_ch;
    
    if (region == LORAMAC_REGION_US915) {
        // Helium uses sub-band 2 (channels 8-15)
        // Channel mask: 16 bits per bank, bit set = channel enabled
        uint16_t us915_mask[6] = {
            0xFF00,  // Bank 0: Channels 0-15, enable 8-15 (sub-band 2)
            0x0000,  // Bank 1: Channels 16-31, all disabled
            0x0000,  // Bank 2: Channels 32-47, all disabled
            0x0000,  // Bank 3: Channels 48-63, all disabled
            0x0001,  // Bank 4: 500kHz channels 64-71, enable channel 64 (matches sub-band 2)
            0x0000   // Bank 5: Reserved
        };
        
        mib_ch.Type = MIB_CHANNELS_MASK;
        mib_ch.Param.ChannelsMask = us915_mask;
        if (LoRaMacMibSetRequestConfirm(&mib_ch) == LORAMAC_STATUS_OK) {
            SEGGER_RTT_WriteString(0, "US915: Set sub-band 2 (channels 8-15 + channel 64)\r\n");
        } else {
            SEGGER_RTT_WriteString(0, "US915: WARNING - Failed to set channel mask\r\n");
        }
        
        // Also set default channels to match
        mib_ch.Type = MIB_CHANNELS_DEFAULT_MASK;
        mib_ch.Param.ChannelsDefaultMask = us915_mask;
        LoRaMacMibSetRequestConfirm(&mib_ch);
        
    } else if (region == LORAMAC_REGION_EU868) {
        // EU868: Channels 0-2 are default join channels (868.1, 868.3, 868.5 MHz)
        // After OTAA join, network typically enables channels 3-7 as well
        // For ABP mode, enable all standard channels (0-7) for data transmission
        // Channels 3-7: 867.1, 867.3, 867.5, 867.7, 867.9 MHz
        uint16_t eu868_mask[1] = {0x00FF};  // Binary: 0000 0000 1111 1111 (channels 0-7 enabled)
        
        mib_ch.Type = MIB_CHANNELS_MASK;
        mib_ch.Param.ChannelsMask = eu868_mask;
        if (LoRaMacMibSetRequestConfirm(&mib_ch) == LORAMAC_STATUS_OK) {
            SEGGER_RTT_WriteString(0, "EU868: Enabled all standard channels 0-7 for data transmission\r\n");
        } else {
            SEGGER_RTT_WriteString(0, "EU868: WARNING - Failed to set channel mask\r\n");
        }
        
        mib_ch.Type = MIB_CHANNELS_DEFAULT_MASK;
        mib_ch.Param.ChannelsDefaultMask = eu868_mask;
        LoRaMacMibSetRequestConfirm(&mib_ch);
    }
    
    // STEP 7: Start MAC and allow state machine to stabilize
    SEGGER_RTT_WriteString(0, "Step 7: Starting MAC and stabilizing...\r\n");
    LoRaMacStart();
    HAL_Delay(200);
    
    // STEP 8: Set DevAddr via MIB AFTER LoRaMacStart() to ensure it persists
    SEGGER_RTT_WriteString(0, "Step 8: Re-setting DevAddr after MAC start...\r\n");
    mib.Type = MIB_DEV_ADDR;
    mib.Param.DevAddr = ctx->dev_addr;
    if (LoRaMacMibSetRequestConfirm(&mib) == LORAMAC_STATUS_OK) {
        SEGGER_RTT_printf(0, "  DevAddr confirmed: 0x%08lX\r\n", ctx->dev_addr);
    } else {
        SEGGER_RTT_WriteString(0, "  ERROR: Failed to set DevAddr!\r\n");
    }
    
    // STEP 9: Process MAC events to complete initialization
    SEGGER_RTT_WriteString(0, "Step 9: Processing MAC events to stabilize...\r\n");
    for (int i = 0; i < 10; i++) {
        LmHandlerProcess();
        HAL_Delay(10);
    }
    
    // Verify MAC is not busy before proceeding
    if (LoRaMacIsBusy()) {
        SEGGER_RTT_WriteString(0, "  WARNING: MAC still busy, giving more time...\r\n");
        HAL_Delay(500);
        for (int i = 0; i < 20; i++) {
            LmHandlerProcess();
            HAL_Delay(10);
        }
    }
    
    // Final MAC state verification
    if (LoRaMacIsBusy()) {
        SEGGER_RTT_WriteString(0, "  ERROR: MAC is busy after initialization!\r\n");
        return LORAMAC_HANDLER_BUSY_ERROR;
    }
    SEGGER_RTT_WriteString(0, "  MAC verified idle and ready\r\n");
    
    // STEP 10: VERIFY secure element has correct keys and DevAddr
    SEGGER_RTT_WriteString(0, "Step 10: Verifying secure element state...\r\n");
    
    // Verify DevAddr
    MibRequestConfirm_t verify_mib;
    verify_mib.Type = MIB_DEV_ADDR;
    LoRaMacMibGetRequestConfirm(&verify_mib);
    
    if (verify_mib.Param.DevAddr != ctx->dev_addr) {
        SEGGER_RTT_printf(0, "  ERROR: DevAddr mismatch! MAC=0x%08lX, Expected=0x%08lX\r\n", 
                          verify_mib.Param.DevAddr, ctx->dev_addr);
    } else {
        SEGGER_RTT_printf(0, "  ✓ DevAddr verified: 0x%08lX\r\n", ctx->dev_addr);
    }
    
    // Verify session keys in secure element
    verify_mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&verify_mib);
    LoRaMacNvmData_t *verify_nvm = (LoRaMacNvmData_t*)verify_mib.Param.Contexts;
    
    if (verify_nvm) {
        bool keys_match = true;
        
        // Check AppSKey
        if (memcmp(verify_nvm->SecureElement.KeyList[APP_S_KEY].KeyValue, 
                   ctx->app_s_key, 16) != 0) {
            SEGGER_RTT_WriteString(0, "  ERROR: AppSKey mismatch in secure element!\r\n");
            keys_match = false;
        }
        
        // Check NwkSKey
        if (memcmp(verify_nvm->SecureElement.KeyList[NWK_S_KEY].KeyValue, 
                   ctx->nwk_s_key, 16) != 0) {
            SEGGER_RTT_WriteString(0, "  ERROR: NwkSKey mismatch in secure element!\r\n");
            keys_match = false;
        }
        
        if (keys_match) {
            SEGGER_RTT_WriteString(0, "  ✓ Session keys verified correct\r\n");
        }
    }
    
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
 * @brief Send post-join data packets to receive MAC commands
 * @param num_packets Number of packets to send (typically 2)
 * @note Uses real sensor data to trigger any channel mask updates from network
 */
static void SendPostJoinDataPackets(uint8_t num_packets)
{
    sensor_t sensor_data;
    
    SEGGER_RTT_printf(0, "\r\n--- Sending %d post-join data packets ---\r\n", num_packets);
    
    for (uint8_t i = 0; i < num_packets; i++) {
        SEGGER_RTT_printf(0, "Packet %d/%d: ", i+1, num_packets);
        
        // Read sensor data
        EnvSensors_Read(&sensor_data);
        
        // Prepare simple LoRaWAN packet with temperature data
        LmHandlerAppData_t appData;
        uint8_t payload[3];
        int16_t temp_int = (int16_t)(sensor_data.temperature * 100);
        payload[0] = 0x01;  // Temperature channel
        payload[1] = (temp_int >> 8) & 0xFF;
        payload[2] = temp_int & 0xFF;
        
        appData.Port = 2;
        appData.BufferSize = 3;
        appData.Buffer = payload;
        
        // Send unconfirmed message
        LmHandlerErrorStatus_t status = LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0);
        
        if (status == LORAMAC_HANDLER_SUCCESS) {
            SEGGER_RTT_WriteString(0, "Sent successfully\r\n");
            
            // Wait for TX complete and any downlinks
            HAL_Delay(2000);
            
            // Process any received MAC commands
            for (int j = 0; j < 10; j++) {
                LmHandlerProcess();
                HAL_Delay(100);
            }
        } else {
            SEGGER_RTT_printf(0, "Send failed (status=%d)\r\n", status);
        }
        
        // Wait between packets
        if (i < num_packets - 1) {
            HAL_Delay(3000);
        }
    }
    
    SEGGER_RTT_WriteString(0, "--- Post-join packets complete ---\r\n\r\n");
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
    
    // Send 2 post-join data packets to receive MAC commands (channel masks, etc.)
    SendPostJoinDataPackets(2);
    
    // Save context again after data transmissions (frame counters updated)
    MultiRegion_SaveCurrentContext();
    
    return LORAMAC_HANDLER_SUCCESS;
}

/**
 * @brief Pre-join all required regions (ground operations)
 */
bool MultiRegion_PreJoinAllRegions(void)
{
    // Set pre-join flag to prevent TX timer from starting during joins
    extern volatile uint8_t g_multiregion_in_prejoin;
    g_multiregion_in_prejoin = 1;
    
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
        // Display session keys for copying to Chirpstack
        MultiRegion_DisplaySessionKeys();
    }
    HAL_Delay(5000);
    
    // Join EU868
    if (MultiRegion_JoinRegion(LORAMAC_REGION_EU868) != LORAMAC_HANDLER_SUCCESS) {
        APP_LOG(TS_ON, VLEVEL_H, "FAILED: EU868 join\r\n");
        all_success = false;
    } else {
        APP_LOG(TS_ON, VLEVEL_H, "SUCCESS: EU868 joined\r\n");
        // Display session keys for copying to Chirpstack
        MultiRegion_DisplaySessionKeys();
    }
    HAL_Delay(5000);
    
    // Optionally join AS923
    /*
    if (MultiRegion_JoinRegion(LORAMAC_REGION_AS923) != LORAMAC_HANDLER_SUCCESS) {
        APP_LOG(TS_ON, VLEVEL_H, "FAILED: AS923 join\r\n");
        all_success = false;
    } else {
        APP_LOG(TS_ON, VLEVEL_H, "SUCCESS: AS923 joined\r\n");
        MultiRegion_DisplaySessionKeys();
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
    
    // Clear pre-join flag to allow TX timer to start
    g_multiregion_in_prejoin = 0;
    
    return all_success;
}

/**
 * @brief Initialize a region context from Chirpstack session keys
 */
bool MultiRegion_InitializeRegionFromChirpstack(
    LoRaMacRegion_t region,
    uint32_t dev_addr,
    const uint8_t *app_s_key,
    const uint8_t *nwk_s_key
)
{
    if (!g_initialized) {
        MultiRegion_Init();
    }
    
    if (!app_s_key || !nwk_s_key) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Invalid key pointers\r\n");
        return false;
    }
    
    SEGGER_RTT_printf(0, "\r\n=== Initializing %s from Chirpstack keys ===\r\n", 
                      RegionToString(region));
    
    // Find or allocate slot for this region
    int8_t slot = FindContextSlot(region);
    
    if (slot < 0) {
        // Find empty slot
        for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
            if (g_storage.contexts[i].dev_addr == 0 || 
                g_storage.contexts[i].dev_addr == 0xFFFFFFFF) {
                slot = i;
                g_storage.num_valid++;
                break;
            }
        }
    }
    
    if (slot < 0) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: No available slots\r\n");
        return false;
    }
    
    // Initialize context structure
    MinimalRegionContext_t *ctx = &g_storage.contexts[slot];
    memset(ctx, 0, sizeof(MinimalRegionContext_t));
    
    // Set basic parameters
    ctx->region = region;
    ctx->activation = ACTIVATION_TYPE_ABP;
    ctx->dev_addr = dev_addr;
    
    // Copy session keys
    memcpy(ctx->app_s_key, app_s_key, 16);
    memcpy(ctx->nwk_s_key, nwk_s_key, 16);
    
    // Set DevEUI based on region
    const uint8_t deveui_us915[] = {LORAWAN_DEVICE_EUI_US915};
    const uint8_t deveui_eu868[] = {LORAWAN_DEVICE_EUI_EU868};
    const uint8_t deveui_as923[] = {LORAWAN_DEVICE_EUI_AS923};
    const uint8_t deveui_au915[] = {LORAWAN_DEVICE_EUI_AU915};
    const uint8_t deveui_in865[] = {LORAWAN_DEVICE_EUI_IN865};
    const uint8_t deveui_kr920[] = {LORAWAN_DEVICE_EUI_KR920};
    
    switch (region) {
        case LORAMAC_REGION_US915:
            memcpy(ctx->dev_eui, deveui_us915, 8);
            ctx->datarate = DR_2;              // US915 default
            ctx->rx2_frequency = 923300000;
            ctx->rx2_datarate = DR_8;
            break;
        case LORAMAC_REGION_EU868:
            memcpy(ctx->dev_eui, deveui_eu868, 8);
            ctx->datarate = DR_0;              // EU868 default
            ctx->rx2_frequency = 869525000;
            ctx->rx2_datarate = DR_0;
            break;
        case LORAMAC_REGION_AS923:
            memcpy(ctx->dev_eui, deveui_as923, 8);
            ctx->datarate = DR_2;
            ctx->rx2_frequency = 923200000;
            ctx->rx2_datarate = DR_2;
            break;
        case LORAMAC_REGION_AU915:
            memcpy(ctx->dev_eui, deveui_au915, 8);
            ctx->datarate = DR_2;
            ctx->rx2_frequency = 923300000;
            ctx->rx2_datarate = DR_8;
            break;
        case LORAMAC_REGION_IN865:
            memcpy(ctx->dev_eui, deveui_in865, 8);
            ctx->datarate = DR_0;
            ctx->rx2_frequency = 866550000;
            ctx->rx2_datarate = DR_2;
            break;
        case LORAMAC_REGION_KR920:
            memcpy(ctx->dev_eui, deveui_kr920, 8);
            ctx->datarate = DR_0;
            ctx->rx2_frequency = 921900000;
            ctx->rx2_datarate = DR_0;
            break;
        default:
            APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Unsupported region\r\n");
            return false;
    }
    
    // Initialize frame counters (will be loaded from flash if they exist)
    ctx->uplink_counter = 0;
    ctx->downlink_counter = 0;
    ctx->last_rx_mic = 0;
    
    // Radio parameters
    ctx->tx_power = 0;
    ctx->adr_enabled = 0;  // ADR off for balloon
    
    // Timestamp
    ctx->last_used = HAL_GetTick();
    
    // Calculate CRC
    UpdateContextCRC(ctx);
    
    // Save to flash
    bool result = FlashWriteStorage();
    
    if (result) {
        SEGGER_RTT_printf(0, "%s: DevAddr=0x%08lX initialized\r\n", 
                          RegionToString(region), dev_addr);
        SEGGER_RTT_WriteString(0, "AppSKey: ");
        for (int i = 0; i < 16; i++) {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X ", ctx->app_s_key[i]);
            SEGGER_RTT_WriteString(0, hex);
        }
        SEGGER_RTT_WriteString(0, "\r\nNwkSKey: ");
        for (int i = 0; i < 16; i++) {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X ", ctx->nwk_s_key[i]);
            SEGGER_RTT_WriteString(0, hex);
        }
        SEGGER_RTT_WriteString(0, "\r\n");
        APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: %s context initialized from Chirpstack\r\n", 
                RegionToString(region));
    }
    
    return result;
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
    char debug_msg[256];
    snprintf(debug_msg, sizeof(debug_msg), 
             "FindContextSlot: Searching for region %s (enum=%d)\r\n", 
             RegionToString(region), region);
    SEGGER_RTT_WriteString(0, debug_msg);
    
    SEGGER_RTT_WriteString(0, "Storage contents:\r\n");
    for (uint8_t i = 0; i < MAX_REGION_CONTEXTS; i++) {
        snprintf(debug_msg, sizeof(debug_msg),
                 "  Slot %d: region=%s (enum=%d), DevAddr=0x%08lX\r\n",
                 i, RegionToString(g_storage.contexts[i].region), 
                 g_storage.contexts[i].region,
                 g_storage.contexts[i].dev_addr);
        SEGGER_RTT_WriteString(0, debug_msg);
        
        if (g_storage.contexts[i].region == region) {
            snprintf(debug_msg, sizeof(debug_msg), "  -> Found at slot %d!\r\n", i);
            SEGGER_RTT_WriteString(0, debug_msg);
            return i;
        }
    }
    
    SEGGER_RTT_WriteString(0, "  -> NOT FOUND (returning -1)\r\n");
    return -1;
}

/**
 * @brief Capture current MAC context
 * @note This function updates ONLY dynamic values (frame counters, datarate, etc.)
 * @note Static values (DevAddr, DevEUI, session keys) are NOT overwritten
 */
static bool CaptureCurrentContext(MinimalRegionContext_t *ctx)
{
    if (!ctx) {
        return false;
    }
    
    MibRequestConfirm_t mib;
    
    // CRITICAL: DevAddr, DevEUI, and session keys are region-specific constants
    // They are set during initialization and should NEVER be overwritten
    // Only capture dynamic state that changes with transmissions
    
    SEGGER_RTT_printf(0, "Capturing dynamic context for region %s (DevAddr=0x%08lX preserved)\r\n",
                      RegionToString(ctx->region), ctx->dev_addr);
    
    // Get activation type
    mib.Type = MIB_NETWORK_ACTIVATION;
    LoRaMacMibGetRequestConfirm(&mib);
    ctx->activation = mib.Param.NetworkActivation;
    
    // Get NVM contexts to extract frame counters ONLY
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&mib);
    LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t*)mib.Param.Contexts;
    
    if (nvm) {
        // ONLY copy frame counters (keys and DevAddr already set correctly)
        ctx->uplink_counter = nvm->Crypto.FCntList.FCntUp;
        ctx->downlink_counter = nvm->Crypto.FCntList.NFCntDown;
        ctx->last_rx_mic = nvm->MacGroup1.LastRxMic;
        
        SEGGER_RTT_printf(0, "  Captured FCntUp=%lu, FCntDown=%lu\r\n",
                          ctx->uplink_counter, ctx->downlink_counter);
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
    
    SEGGER_RTT_printf(0, "Context captured: DevAddr=0x%08lX (preserved), FCntUp=%lu\r\n",
                      ctx->dev_addr, ctx->uplink_counter);
    
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
    switch (region) {case LORAMAC_REGION_AS923: return "AS923";
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

/**
 * @brief Display current session keys for Chirpstack ABP configuration
 */
void MultiRegion_DisplaySessionKeys(void)
{
    if (!g_initialized || g_storage.active_slot >= MAX_REGION_CONTEXTS) {
        SEGGER_RTT_WriteString(0, "ERROR: No active region to display\r\n");
        return;
    }
    
    MinimalRegionContext_t *ctx = &g_storage.contexts[g_storage.active_slot];
    
    SEGGER_RTT_WriteString(0, "\r\n");
    SEGGER_RTT_WriteString(0, "========================================\r\n");
    SEGGER_RTT_WriteString(0, "=== SESSION KEYS FOR CHIRPSTACK ABP ===\r\n");
    SEGGER_RTT_WriteString(0, "========================================\r\n\r\n");
    
    // Region
    SEGGER_RTT_printf(0, "Region:       %s\r\n", RegionToString(ctx->region));
    
    // DevEUI
    SEGGER_RTT_WriteString(0, "DevEUI:       ");
    for (int i = 0; i < 8; i++) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02x%s", ctx->dev_eui[i], (i < 7) ? ":" : "");
        SEGGER_RTT_WriteString(0, hex);
    }
    SEGGER_RTT_WriteString(0, "\r\n");
    
    // DevAddr
    SEGGER_RTT_printf(0, "DevAddr:      0x%08lx\r\n", ctx->dev_addr);
    
    // AppSKey (formatted for Chirpstack)
    SEGGER_RTT_WriteString(0, "AppSKey:      ");
    for (int i = 0; i < 16; i++) {
        char hex[3];
        snprintf(hex, sizeof(hex), "%02x", ctx->app_s_key[i]);
        SEGGER_RTT_WriteString(0, hex);
    }
    SEGGER_RTT_WriteString(0, "\r\n");
    
    // NwkSKey (formatted for Chirpstack)
    SEGGER_RTT_WriteString(0, "NwkSKey:      ");
    for (int i = 0; i < 16; i++) {
        char hex[3];
        snprintf(hex, sizeof(hex), "%02x", ctx->nwk_s_key[i]);
        SEGGER_RTT_WriteString(0, hex);
    }
    SEGGER_RTT_WriteString(0, "\r\n");
    
    // Frame counters
    SEGGER_RTT_printf(0, "FCntUp:       %lu\r\n", ctx->uplink_counter);
    SEGGER_RTT_printf(0, "FCntDown:     %lu\r\n", ctx->downlink_counter);
    
    SEGGER_RTT_WriteString(0, "\r\n");
    SEGGER_RTT_WriteString(0, "========================================\r\n");
    SEGGER_RTT_WriteString(0, "Copy AppSKey and NwkSKey to Chirpstack\r\n");
    SEGGER_RTT_WriteString(0, "========================================\r\n\r\n");
}
