/* Host-test stub: SysTime (R49). Test provides the implementation. */
#ifndef STM32_SYSTIME_H_STUB
#define STM32_SYSTIME_H_STUB
#include <stdint.h>
typedef struct SysTime_s { uint32_t Seconds; int16_t SubSeconds; } SysTime_t;
SysTime_t SysTimeGet(void);
#endif
