/* Host-test stub: CMSIS compiler abstraction (F-002/#201 timer_if suite).
 * Only the bits the Utilities headers need. */
#ifndef CMSIS_COMPILER_H_STUB
#define CMSIS_COMPILER_H_STUB
#include <stdint.h>
#ifndef __STATIC_INLINE
#define __STATIC_INLINE static inline
#endif
#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I const volatile
#endif
#ifndef __O
#define __O volatile
#endif
#endif
