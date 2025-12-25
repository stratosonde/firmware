/**
  ******************************************************************************
  * @file    multiregion_context.c
  * @brief   Multi-region LoRaWAN ABP implementation
  ******************************************************************************
  * @attention
  *
  * Simple ABP-based multi-region support. No OTAA joins, no session key
  * extraction, no flash storage - just load ABP credentials and switch.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "multiregion_context.h"
#include "multiregion_h3.h"
#include "sys_app.h"
#include "lora_app.h"
#include "LmHandler.h"
#include "LoRaMac.h"
#include "se-identity.h"
#include "stm32wlxx_hal.h"
#include "SEGGER_RTT.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static ABP_RegionConfig_t g_abp_configs[MAX_ABP_REGIONS];
static LoRaMacRegion_t g_active_region = LORAMAC_REGION_US915;
static bool g_initialized = false;
static bool g_abp_activated = false;  // Track if ABP credentials have been loaded

/* External variables --------------------------------------------------------*/
extern LmHandlerParams_t LmHandlerParams;

/* Private function prototypes -----------------------------------------------*/
static const char* RegionToString(LoRaMacRegion_t region);
static int8_t FindRegionIndex(LoRaMacRegion_t region);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize multi-region ABP support
 */
void MultiRegion_Init(void)
{
    if (g_initialized) {
        return;
    }
    
    SEGGER_RTT_WriteString(0, "\r\n=== MultiRegion_Init (ABP) ===\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "\r\n=== MultiRegion_Init (ABP) ===\r\n");
    
    // Initialize US915 ABP config
    const uint8_t us915_dev_eui[] = {LORAWAN_DEVICE_EUI_US915};
    const uint8_t us915_app_skey[] = LORAWAN_APP_SKEY_US915;
    const uint8_t us915_nwk_skey[] = LORAWAN_NWK_SKEY_US915;
    
    g_abp_configs[0].region = LORAMAC_REGION_US915;
    memcpy(g_abp_configs[0].dev_eui, us915_dev_eui, 8);
    g_abp_configs[0].dev_addr = LORAWAN_DEV_ADDR_US915;
    memcpy(g_abp_configs[0].app_s_key, us915_app_skey, 16);
    memcpy(g_abp_configs[0].nwk_s_key, us915_nwk_skey, 16);
    
    SEGGER_RTT_printf(0, "US915 ABP: DevAddr=0x%08lX, DevEUI=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            g_abp_configs[0].dev_addr,
            us915_dev_eui[0], us915_dev_eui[1], us915_dev_eui[2], us915_dev_eui[3],
            us915_dev_eui[4], us915_dev_eui[5], us915_dev_eui[6], us915_dev_eui[7]);
    
    // Initialize EU868 ABP config
    const uint8_t eu868_dev_eui[] = {LORAWAN_DEVICE_EUI_EU868};
    const uint8_t eu868_app_skey[] = LORAWAN_APP_SKEY_EU868;
    const uint8_t eu868_nwk_skey[] = LORAWAN_NWK_SKEY_EU868;
    
    g_abp_configs[1].region = LORAMAC_REGION_EU868;
    memcpy(g_abp_configs[1].dev_eui, eu868_dev_eui, 8);
    g_abp_configs[1].dev_addr = LORAWAN_DEV_ADDR_EU868;
    memcpy(g_abp_configs[1].app_s_key, eu868_app_skey, 16);
    memcpy(g_abp_configs[1].nwk_s_key, eu868_nwk_skey, 16);
    
    SEGGER_RTT_printf(0, "EU868 ABP: DevAddr=0x%08lX, DevEUI=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            g_abp_configs[1].dev_addr,
            eu868_dev_eui[0], eu868_dev_eui[1], eu868_dev_eui[2], eu868_dev_eui[3],
            eu868_dev_eui[4], eu868_dev_eui[5], eu868_dev_eui[6], eu868_dev_eui[7]);
    
    // Start with US915 as default
    g_active_region = LORAMAC_REGION_US915;
    
    g_initialized = true;
    SEGGER_RTT_WriteString(0, "MultiRegion ABP initialized successfully\r\n");
    APP_LOG(TS_ON, VLEVEL_H, "MultiRegion ABP initialized successfully\r\n");
}

/**
 * @brief Get current active region
 */
LoRaMacRegion_t MultiRegion_GetActiveRegion(void)
{
    return g_active_region;
}

/**
 * @brief Switch to a different region
 */
LmHandlerErrorStatus_t MultiRegion_SwitchToRegion(LoRaMacRegion_t region)
{
    if (!g_initialized) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Not initialized\r\n");
        return LORAMAC_HANDLER_ERROR;
    }
    
    // Check if already on this region AND already activated
    // Allow switch if this is initial activation (g_abp_activated == false)
    if (g_active_region == region && g_abp_activated) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Already on %s\r\n", RegionToString(region));
        return LORAMAC_HANDLER_SUCCESS;
    }
    
    // Find ABP config for target region
    int8_t idx = FindRegionIndex(region);
    if (idx < 0) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Region %s not configured\r\n", RegionToString(region));
        return LORAMAC_HANDLER_ERROR;
    }
    
    ABP_RegionConfig_t *cfg = &g_abp_configs[idx];
    
    // Check if MAC is busy
    if (LoRaMacIsBusy()) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: MAC busy, cannot switch\r\n");
        return LORAMAC_HANDLER_BUSY_ERROR;
    }
    
    SEGGER_RTT_printf(0, "\r\n=== Switching to %s ABP ===\r\n", RegionToString(region));
    APP_LOG(TS_ON, VLEVEL_H, "\r\n=== Switching to %s ABP ===\r\n", RegionToString(region));
    
    // Reinit stack for new region
    SEGGER_RTT_WriteString(0, "Performing stack reinit for region switch...\r\n");
    LoRaApp_ReInitStack(region);
    HAL_Delay(100);
    LmHandlerProcess();
    
    // Set DevEUI for target region
    LmHandlerSetDevEUI(cfg->dev_eui);
    SEGGER_RTT_printf(0, "Set DevEUI: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r\n",
            cfg->dev_eui[0], cfg->dev_eui[1], cfg->dev_eui[2], cfg->dev_eui[3],
            cfg->dev_eui[4], cfg->dev_eui[5], cfg->dev_eui[6], cfg->dev_eui[7]);
    
    // Configure stack for target region
    SEGGER_RTT_WriteString(0, "Configuring stack for target region...\r\n");
    LmHandlerConfigure(&LmHandlerParams);
    
    // Load ABP credentials
    MibRequestConfirm_t mib;
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&mib);
    LoRaMacNvmData_t *nvm = (LoRaMacNvmData_t*)mib.Param.Contexts;
    
    if (nvm) {
        // Set ABP session keys
        memcpy(nvm->SecureElement.KeyList[APP_S_KEY].KeyValue, cfg->app_s_key, 16);
        memcpy(nvm->SecureElement.KeyList[NWK_S_ENC_KEY].KeyValue, cfg->nwk_s_key, 16);
        memcpy(nvm->SecureElement.KeyList[F_NWK_S_INT_KEY].KeyValue, cfg->nwk_s_key, 16);
        memcpy(nvm->SecureElement.KeyList[S_NWK_S_INT_KEY].KeyValue, cfg->nwk_s_key, 16);
        
        // Set DevAddr
        nvm->MacGroup2.DevAddr = cfg->dev_addr;
        
        // Initialize frame counters to 0 for ABP
        nvm->Crypto.FCntList.FCntUp = 0;
        nvm->Crypto.FCntList.NFCntDown = 0;
        
        // Mark as ABP activated
        nvm->MacGroup2.NetworkActivation = ACTIVATION_TYPE_ABP;
        
        SEGGER_RTT_printf(0, "Loaded ABP: DevAddr=0x%08lX\r\n", cfg->dev_addr);
    }
    
    // Set activation via MIB
    mib.Type = MIB_NETWORK_ACTIVATION;
    mib.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
    LoRaMacMibSetRequestConfirm(&mib);
    
    // Start MAC and allow state machine to stabilize
    LoRaMacStart();
    HAL_Delay(200);
    
    // Process MAC events to complete initialization
    for (int i = 0; i < 10; i++) {
        LmHandlerProcess();
        HAL_Delay(10);
    }
    
    // Verify MAC is ready
    if (LoRaMacIsBusy()) {
        SEGGER_RTT_WriteString(0, "WARNING: MAC still busy after initialization\r\n");
        HAL_Delay(500);
        for (int i = 0; i < 20; i++) {
            LmHandlerProcess();
            HAL_Delay(10);
        }
    }
    
    if (LoRaMacIsBusy()) {
        SEGGER_RTT_WriteString(0, "ERROR: MAC is busy after full initialization!\r\n");
        return LORAMAC_HANDLER_BUSY_ERROR;
    }
    
    SEGGER_RTT_WriteString(0, "MAC verified idle and ready for TX\r\n");
    
    g_active_region = region;
    g_abp_activated = true;  // Mark ABP as activated
    
    SEGGER_RTT_printf(0, "Successfully switched to %s ABP\r\n", RegionToString(region));
    APP_LOG(TS_ON, VLEVEL_H, "MultiRegion: Successfully switched to %s ABP\r\n", RegionToString(region));
    
    return LORAMAC_HANDLER_SUCCESS;
}

/**
 * @brief Auto-switch based on GPS location
 */
LmHandlerErrorStatus_t MultiRegion_AutoSwitchForLocation(float lat, float lon)
{
    #if MULTIREGION_AUTO_SWITCH_ENABLED == 0
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
    
    // Check if target region has ABP config
    if (FindRegionIndex(target_region) < 0) {
        APP_LOG(TS_ON, VLEVEL_M, "MultiRegion: Target region not configured, staying on %s\r\n",
                RegionToString(current_region));
        return LORAMAC_HANDLER_SUCCESS;  // Not an error, just stay on current
    }
    
    // Perform the switch
    return MultiRegion_SwitchToRegion(target_region);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Convert region enum to string
 */
static const char* RegionToString(LoRaMacRegion_t region)
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

/**
 * @brief Find index of ABP config for a region
 */
static int8_t FindRegionIndex(LoRaMacRegion_t region)
{
    for (uint8_t i = 0; i < MAX_ABP_REGIONS; i++) {
        if (g_abp_configs[i].region == region) {
            return i;
        }
    }
    return -1;
}
