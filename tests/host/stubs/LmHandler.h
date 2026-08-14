/* Host-test stub: LmHandler surface used by Core/Src/multiregion_context.c
 * (R15 fake-LoRaMac harness). Implementations live in test_multiregion.c. */
#ifndef LMHANDLER_H_STUB
#define LMHANDLER_H_STUB
#include <stdint.h>
#include <stdbool.h>
#include "LoRaMac.h"

typedef enum eLmHandlerErrorStatus {
    LORAMAC_HANDLER_ERROR = -1,
    LORAMAC_HANDLER_SUCCESS = 0,
    LORAMAC_HANDLER_BUSY_ERROR,
} LmHandlerErrorStatus_t;

typedef enum eLmHandlerFlagStatus { LORAMAC_HANDLER_RESET = 0, LORAMAC_HANDLER_SET } LmHandlerFlagStatus_t;

typedef enum eLmHandlerMsgTypes { LORAMAC_HANDLER_UNCONFIRMED_MSG = 0, LORAMAC_HANDLER_CONFIRMED_MSG } LmHandlerMsgTypes_t;

typedef struct sLmHandlerAppData {
    uint8_t *Buffer;
    uint8_t BufferSize;
    uint8_t Port;
} LmHandlerAppData_t;

typedef struct sLmHandlerParams {
    LoRaMacRegion_t ActiveRegion;
    int8_t TxDatarate;
} LmHandlerParams_t;

/* F-01 (#245): match the real API - these return statuses (previously the
 * stub said void, which is exactly how the unchecked-return bug hid). */
LmHandlerErrorStatus_t LmHandlerConfigure(LmHandlerParams_t *params);
LmHandlerErrorStatus_t LmHandlerSetKey(KeyIdentifier_t keyId, uint8_t *key);
LmHandlerErrorStatus_t LmHandlerGetKey(KeyIdentifier_t keyId, uint8_t *key);
LmHandlerErrorStatus_t LmHandlerSetDevEUI(uint8_t *devEui);
void LmHandlerSetAppEUI(uint8_t *appEui);
void LmHandlerProcess(void);
void LmHandlerJoin(ActivationType_t mode, bool forceRejoin);
LmHandlerFlagStatus_t LmHandlerJoinStatus(void);
LmHandlerErrorStatus_t LmHandlerSend(LmHandlerAppData_t *appData, LmHandlerMsgTypes_t isTxConfirmed, uint32_t nextTxIn);

#endif