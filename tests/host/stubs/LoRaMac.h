/* Host-test stub: LoRaMac core surface used by Core/Src/multiregion_context.c
 * (R15 fake-LoRaMac harness). Minimal but layout-faithful for the members the
 * real unit touches: MIB set/get, NVM context groups, secure-element key list,
 * start/busy. Implementations (with failure injection) live in
 * test_multiregion.c. */
#ifndef LORAMAC_H_STUB
#define LORAMAC_H_STUB
#include <stdint.h>
#include <stdbool.h>
#include "LoRaMacInterfaces.h"

typedef enum eLoRaMacStatus { LORAMAC_STATUS_OK = 0, LORAMAC_STATUS_ERROR, LORAMAC_STATUS_BUSY } LoRaMacStatus_t;

/* Datarate indices (region tables use DR_0..DR_15 ordinals) */
#define DR_0  0
#define DR_1  1
#define DR_2  2
#define DR_3  3
#define DR_4  4
#define DR_5  5
#define DR_6  6
#define DR_7  7
#define DR_8  8
#define DR_9  9
#define DR_10 10
#define DR_11 11
#define DR_12 12
#define DR_13 13
#define DR_14 14
#define DR_15 15

typedef enum eKeyIdentifier { APP_KEY = 0, NWK_KEY, NWK_S_KEY, APP_S_KEY, NWK_S_ENC_KEY, KEY_LIST_SIZE } KeyIdentifier_t;

typedef enum eMibType {
    MIB_DEV_ADDR = 0,
    MIB_DEV_EUI,
    MIB_NETWORK_ACTIVATION,
    MIB_NVM_CTXS,
    MIB_CHANNELS_MASK,
    MIB_CHANNELS_DEFAULT_MASK,
    MIB_CHANNELS_DATARATE,
    MIB_CHANNELS_TX_POWER,
    MIB_ADR,
    MIB_RX2_CHANNEL,
} Mib_t;

typedef union uMibParam {
    uint32_t DevAddr;
    uint8_t *DevEui;
    ActivationType_t NetworkActivation;
    void *Contexts;
    uint16_t *ChannelsMask;
    uint16_t *ChannelsDefaultMask;
    int8_t ChannelsDatarate;
    int8_t ChannelsTxPower;
    bool AdrEnable;
    struct { uint32_t Frequency; uint32_t Datarate; } Rx2Channel;
} MibParam_t;

typedef struct sMibRequestConfirm { Mib_t Type; MibParam_t Param; } MibRequestConfirm_t;

typedef struct sLoRaMacNvmData {
    struct { struct { uint32_t FCntUp; uint32_t NFCntDown; uint32_t AFCntDown; } FCntList; } Crypto;
    struct { uint32_t LastRxMic; } MacGroup1;
    struct { ActivationType_t NetworkActivation; uint32_t DevAddr; } MacGroup2;
    struct { struct { uint8_t KeyValue[16]; } KeyList[KEY_LIST_SIZE]; } SecureElement;
} LoRaMacNvmData_t;

LoRaMacStatus_t LoRaMacMibSetRequestConfirm(MibRequestConfirm_t *mib);
LoRaMacStatus_t LoRaMacMibGetRequestConfirm(MibRequestConfirm_t *mib);
LoRaMacStatus_t LoRaMacStart(void);
bool LoRaMacIsBusy(void);

#endif