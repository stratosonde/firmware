/* Host-test stub: W25Q flash header (R49). Only what flash_log.h needs. */
#ifndef W25Q16JV_H_STUB
#define W25Q16JV_H_STUB
#include <stdint.h>
#include <stdbool.h>
#define W25Q_SECTOR_SIZE  4096U
#define W25Q_FLASH_SIZE   (2U * 1024U * 1024U)
typedef enum { W25Q_OK = 0, W25Q_ERROR, W25Q_ERROR_INIT } W25Q_StatusTypeDef;
typedef struct { int _host_stub; } W25Q_HandleTypeDef;
#endif
