/* Host-test stub: internal-flash interface (R15 harness). Signatures match
 * Core/Inc/flash_if.h; the RAM-backed fake lives in test_multiregion.c. */
#ifndef FLASH_IF_H_STUB
#define FLASH_IF_H_STUB
#include <stdint.h>

typedef enum { FLASH_IF_OK = 0, FLASH_IF_ERROR, FLASH_IF_BUSY, FLASH_IF_ERROR_TIMEOUT } FLASH_IF_StatusTypedef;

FLASH_IF_StatusTypedef FLASH_IF_Init(void *pAllocRamBuffer);
FLASH_IF_StatusTypedef FLASH_IF_DeInit(void);
FLASH_IF_StatusTypedef FLASH_IF_Write(void *pDestination, const void *pSource, uint32_t uLength);
FLASH_IF_StatusTypedef FLASH_IF_Read(void *pDestination, const void *pSource, uint32_t uLength);
FLASH_IF_StatusTypedef FLASH_IF_Erase(void *pStart, uint32_t uLength);

#endif