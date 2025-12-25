/*!
 * \file      se-identity.h
 *
 * \brief     Secure Element identity and keys
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2020 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 */
/**
  ******************************************************************************
  *
  *          Portions COPYRIGHT 2020 STMicroelectronics
  *
  * @file    se-identity.h
  * @author  MCD Application Team
  * @brief   Secure Element identity and keys
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SOFT_SE_IDENTITY_H__
#define __SOFT_SE_IDENTITY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported Includes --------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/*!
 ******************************************************************************
 ********************************** WARNING ***********************************
 ******************************************************************************
  The secure-element implementation supports both 1.0.x and 1.1.x LoRaWAN
  versions of the specification.
  Thus it has been decided to use the 1.1.x keys and EUI name definitions.
  The below table shows the names equivalence between versions:
               +---------------------+-------------------------+
               |       1.0.x         |          1.1.x          |
               +=====================+=========================+
               | LORAWAN_DEVICE_EUI  | LORAWAN_DEVICE_EUI      |
               +---------------------+-------------------------+
               | LORAWAN_APP_EUI     | LORAWAN_JOIN_EUI        |
               +---------------------+-------------------------+
               | LORAWAN_GEN_APP_KEY | LORAWAN_APP_KEY         |
               +---------------------+-------------------------+
               | LORAWAN_APP_KEY     | LORAWAN_NWK_KEY         |
               +---------------------+-------------------------+
               | LORAWAN_NWK_S_KEY   | LORAWAN_F_NWK_S_INT_KEY |
               +---------------------+-------------------------+
               | LORAWAN_NWK_S_KEY   | LORAWAN_S_NWK_S_INT_KEY |
               +---------------------+-------------------------+
               | LORAWAN_NWK_S_KEY   | LORAWAN_NWK_S_ENC_KEY   |
               +---------------------+-------------------------+
               | LORAWAN_APP_S_KEY   | LORAWAN_APP_S_KEY       |
               +---------------------+-------------------------+
 ******************************************************************************
 ******************************************************************************
 ******************************************************************************
 */
/*!
 * End-device IEEE EUI (big endian)
 * When set to 00,00,00,00,00,00,00,00 DevEui is automatically set with a value provided by MCU platform
 * 
 * Multi-region DevEUI encoding: 60:81:F9:53:25:0E:CH:FF
 *   C = Counter (upper nibble, 0-F) for disambiguating same freq
 *   H = Hundreds of MHz (8=800MHz, 9=900MHz)
 *   FF = Frequency remainder (15=915, 68=868, etc.)
 */

// Default DevEUI (US915 ABP) - used by FORMAT32_KEY macro (no 0x prefix needed)
#define LORAWAN_DEVICE_EUI                                 60,81,F9,53,25,0F,09,15

// =============================================================================
// ABP Multi-Region Configuration
// =============================================================================
// NOTE: Get DevAddr, AppSKey, NwkSKey from Chirpstack after creating ABP devices
// =============================================================================

// US915 ABP Device (DevEUI: 60:81:F9:53:25:0F:09:15)
#define LORAWAN_DEVICE_EUI_US915                           0x60,0x81,0xF9,0x53,0x25,0x0F,0x09,0x15
#define LORAWAN_DEV_ADDR_US915                             0x26091500  // DevAddr with 0915 region indicator
#define LORAWAN_APP_SKEY_US915                             { 0xF0,0x0C,0x2A,0x6E,0xBC,0x60,0x3A,0x71,0xF9,0xE4,0xE4,0x72,0x09,0x15,0xAA,0xAA }  // 16 bytes, ends with 0915AAAA
#define LORAWAN_NWK_SKEY_US915                             { 0xF0,0x0C,0x2A,0x6E,0xBC,0x60,0x3A,0x71,0xF9,0xE4,0xE4,0x72,0x09,0x15,0xBB,0xBB }  // 16 bytes, ends with 0915BBBB

// EU868 ABP Device (DevEUI: 60:81:F9:53:25:0F:08:68)
#define LORAWAN_DEVICE_EUI_EU868                           0x60,0x81,0xF9,0x53,0x25,0x0F,0x08,0x68
#define LORAWAN_DEV_ADDR_EU868                             0x26086800  // DevAddr with 0868 region indicator
#define LORAWAN_APP_SKEY_EU868                             { 0xF0,0x0C,0x2A,0x6E,0xBC,0x60,0x3A,0x71,0xF9,0xE4,0xE4,0x72,0x08,0x68,0xAA,0xAA }  // 16 bytes, ends with 0868AAAA
#define LORAWAN_NWK_SKEY_EU868                             { 0xF0,0x0C,0x2A,0x6E,0xBC,0x60,0x3A,0x71,0xF9,0xE4,0xE4,0x72,0x08,0x68,0xBB,0xBB }  // 16 bytes, ends with 0868BBBB

// Legacy region DevEUIs (not used for ABP, kept for reference)
#define LORAWAN_DEVICE_EUI_AS923                           0x60,0x81,0xF9,0x53,0x25,0x0E,0x09,0x23
#define LORAWAN_DEVICE_EUI_AU915                           0x60,0x81,0xF9,0x53,0x25,0x0E,0x19,0x15
#define LORAWAN_DEVICE_EUI_IN865                           0x60,0x81,0xF9,0x53,0x25,0x0E,0x08,0x65
#define LORAWAN_DEVICE_EUI_KR920                           0x60,0x81,0xF9,0x53,0x25,0x0E,0x09,0x20

/*!
 * App/Join server IEEE EUI (big endian)
 */
#define LORAWAN_JOIN_EUI                                   60,81,F9,58,2E,57,13,5E

/*!
 * Device address on the network (big endian)
 * When set to 00,00,00,00 DevAddr is automatically set with a value provided by MCU platform
 */
#define LORAWAN_DEVICE_ADDRESS                             00,00,00,00

/*!
 * Application root key
 */
#define LORAWAN_APP_KEY                                    F0,0C,2A,6E,BC,60,3A,71,F9,E4,E4,72,9D,27,3C,D2

/*!
 * Network root key
 */
#define LORAWAN_NWK_KEY                                    F0,0C,2A,6E,BC,60,3A,71,F9,E4,E4,72,9D,27,3C,D2

/*!
 * Forwarding Network session key (LoRaWAN 1.0.x NwkSKey)
 * For ABP US915: ends with 0915BBBB
 */
#define LORAWAN_NWK_S_KEY                                  F0,0C,2A,6E,BC,60,3A,71,F9,E4,E4,72,09,15,BB,BB

/*!
 * Application session key (LoRaWAN 1.0.x AppSKey) 
 * For ABP US915: ends with 0915AAAA
 */
#define LORAWAN_APP_S_KEY                                  F0,0C,2A,6E,BC,60,3A,71,F9,E4,E4,72,09,15,AA,AA

/*!
 * Format commissioning keys
 */
#define RAW_TO_INT8A(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p) {0x##a,0x##b,0x##c,0x##d,\
                                                       0x##e,0x##f,0x##g,0x##h,\
                                                       0x##i,0x##j,0x##k,0x##l,\
                                                       0x##m,0x##n,0x##o,0x##p}

#define RAW8_TO_INT8A(a,b,c,d) 0x##a##b##c##d
#define RAW32_TO_INT8A(a,b,c,d,e,f,g,h) {0x##a,0x##b,0x##c,0x##d,\
                                         0x##e,0x##f,0x##g,0x##h}

#define FORMAT_KEY(...) RAW_TO_INT8A(__VA_ARGS__)
#define FORMAT8_KEY(...) RAW8_TO_INT8A(__VA_ARGS__)
#define FORMAT32_KEY(...) RAW32_TO_INT8A(__VA_ARGS__)

#if (defined( LORAMAC_VERSION ) && ( LORAMAC_VERSION == 0x01010100 ))
#define SESSION_KEYS_LIST                                                                                           \
        {                                                                                                           \
            /*!                                                                                                     \
             * Join session integrity key (Dynamically updated)                                                     \
             * WARNING: NOT USED FOR 1.0.x DEVICES                                                                  \
             */                                                                                                     \
            .KeyID    = J_S_INT_KEY,                                                                                \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Join session encryption key (Dynamically updated)                                                    \
             * WARNING: NOT USED FOR 1.0.x DEVICES                                                                  \
             */                                                                                                     \
            .KeyID    = J_S_ENC_KEY,                                                                                \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Forwarding Network session integrity key                                                             \
             * WARNING: NWK_S_KEY FOR 1.0.x DEVICES                                                                 \
             */                                                                                                     \
            .KeyID    = F_NWK_S_INT_KEY,                                                                            \
            .KeyValue = FORMAT_KEY(LORAWAN_NWK_S_KEY),                                                              \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Serving Network session integrity key                                                                \
             * WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY                \
             */                                                                                                     \
            .KeyID    = S_NWK_S_INT_KEY,                                                                            \
            .KeyValue = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, \
                          0x3C },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Network session encryption key                                                                       \
             * WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY                \
             */                                                                                                     \
            .KeyID    = NWK_S_ENC_KEY,                                                                              \
            .KeyValue = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, \
                          0x3C },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Application session key                                                                              \
             */                                                                                                     \
            .KeyID    = APP_S_KEY,                                                                                  \
            .KeyValue = FORMAT_KEY(LORAWAN_APP_S_KEY),                                                              \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Datablock MIC key                                                                                    \
             */                                                                                                     \
            .KeyID    = DATABLOCK_INT_KEY,                                                                          \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },
#else
#define SESSION_KEYS_LIST                                                                                           \
        {                                                                                                           \
            /*!                                                                                                     \
             * Network session key                                                                                  \
             */                                                                                                     \
            .KeyID    = NWK_S_KEY,                                                                                  \
            .KeyValue = FORMAT_KEY(LORAWAN_NWK_S_KEY),                                                              \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Application session key                                                                              \
             */                                                                                                     \
            .KeyID    = APP_S_KEY,                                                                                  \
            .KeyValue = FORMAT_KEY(LORAWAN_APP_S_KEY),                                                              \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Datablock MIC key                                                                                    \
             */                                                                                                     \
            .KeyID    = DATABLOCK_INT_KEY,                                                                          \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },
#endif /* LORAMAC_VERSION */

#if (LORAMAC_MAX_MC_CTX == 1)
#define SESSION_MC_KEYS_LIST                                                                                        \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #0 root key (Dynamically updated)                                                    \
             */                                                                                                     \
            .KeyID    = MC_KEY_0,                                                                                   \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #0 application session key (Dynamically updated)                                     \
             */                                                                                                     \
            .KeyID    = MC_APP_S_KEY_0,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #0 network session key (Dynamically updated)                                         \
             */                                                                                                     \
            .KeyID    = MC_NWK_S_KEY_0,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },
#else /* LORAMAC_MAX_MC_CTX > 1 */
#define SESSION_MC_KEYS_LIST                                                                                        \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #0 root key (Dynamically updated)                                                    \
             */                                                                                                     \
            .KeyID    = MC_KEY_0,                                                                                   \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #0 application session key (Dynamically updated)                                     \
             */                                                                                                     \
            .KeyID    = MC_APP_S_KEY_0,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #0 network session key (Dynamically updated)                                         \
             */                                                                                                     \
            .KeyID    = MC_NWK_S_KEY_0,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #1 root key (Dynamically updated)                                                    \
             */                                                                                                     \
            .KeyID    = MC_KEY_1,                                                                                   \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #1 application session key (Dynamically updated)                                     \
             */                                                                                                     \
            .KeyID    = MC_APP_S_KEY_1,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #1 network session key (Dynamically updated)                                         \
             */                                                                                                     \
            .KeyID    = MC_NWK_S_KEY_1,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #2 root key (Dynamically updated)                                                    \
             */                                                                                                     \
            .KeyID    = MC_KEY_2,                                                                                   \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #2 application session key (Dynamically updated)                                     \
             */                                                                                                     \
            .KeyID    = MC_APP_S_KEY_2,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #2 network session key (Dynamically updated)                                         \
             */                                                                                                     \
            .KeyID    = MC_NWK_S_KEY_2,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #3 root key (Dynamically updated)                                                    \
             */                                                                                                     \
            .KeyID    = MC_KEY_3,                                                                                   \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #3 application session key (Dynamically updated)                                     \
             */                                                                                                     \
            .KeyID    = MC_APP_S_KEY_3,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast group #3 network session key (Dynamically updated)                                         \
             */                                                                                                     \
            .KeyID    = MC_NWK_S_KEY_3,                                                                             \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },
#endif /* LORAMAC_MAX_MC_CTX */

#define SOFT_SE_KEY_LIST                                                                                            \
    {                                                                                                               \
        {                                                                                                           \
            /*!                                                                                                     \
             * Application root key                                                                                 \
             * WARNING: FOR 1.0.x DEVICES IT IS THE \ref LORAWAN_GEN_APP_KEY                                        \
             */                                                                                                     \
            .KeyID    = APP_KEY,                                                                                    \
            .KeyValue = FORMAT_KEY(LORAWAN_APP_KEY),                                                                \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Network root key                                                                                     \
             * WARNING: FOR 1.0.x DEVICES IT IS THE \ref LORAWAN_APP_KEY                                            \
             */                                                                                                     \
            .KeyID    = NWK_KEY,                                                                                    \
            .KeyValue = FORMAT_KEY(LORAWAN_NWK_KEY),                                                                \
        },                                                                                                          \
        SESSION_KEYS_LIST                                                                                           \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast root key (Dynamically updated)                                                             \
             */                                                                                                     \
            .KeyID    = MC_ROOT_KEY,                                                                                \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        {                                                                                                           \
            /*!                                                                                                     \
             * Multicast key encryption key (Dynamically updated)                                                   \
             */                                                                                                     \
            .KeyID    = MC_KE_KEY,                                                                                  \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
        SESSION_MC_KEYS_LIST                                                                                        \
        {                                                                                                           \
            /*!                                                                                                     \
             * All zeros key. (ClassB usage)(constant)                                                              \
             */                                                                                                     \
            .KeyID    = SLOT_RAND_ZERO_KEY,                                                                         \
            .KeyValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                          0x00 },                                                                                   \
        },                                                                                                          \
    }

#define SOFT_SE_ID_LIST                                                                                             \
    .SeNvmDevJoinKey.DevEui = FORMAT32_KEY(LORAWAN_DEVICE_EUI),                                                     \
    .SeNvmDevJoinKey.JoinEui = FORMAT32_KEY(LORAWAN_JOIN_EUI),                                                      \
    .SeNvmDevJoinKey.DevAddrOTAA = FORMAT8_KEY(LORAWAN_DEVICE_ADDRESS),                                             \
    .SeNvmDevJoinKey.DevAddrABP = FORMAT8_KEY(LORAWAN_DEVICE_ADDRESS)                                               \

/* USER CODE BEGIN EC */

/* USER CODE END EC */

#ifdef __cplusplus
}
#endif

#endif  /*  __SOFT_SE_IDENTITY_H__ */
