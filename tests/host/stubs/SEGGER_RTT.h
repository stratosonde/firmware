/* Host-test stub: SEGGER RTT compiles to no-ops off-target (R49). */
#ifndef SEGGER_RTT_H_STUB
#define SEGGER_RTT_H_STUB
#include <stdarg.h>
static inline void SEGGER_RTT_WriteString(unsigned ch, const char *s) { (void)ch; (void)s; }
static inline int SEGGER_RTT_printf(unsigned ch, const char *fmt, ...) { (void)ch; (void)fmt; return 0; }
#endif
