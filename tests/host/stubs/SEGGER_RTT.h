/* Host-test stub: SEGGER RTT compiles to no-ops off-target (R49). */
#ifndef SEGGER_RTT_H
#define SEGGER_RTT_H
#include <stdarg.h>
static inline void SEGGER_RTT_WriteString(unsigned ch, const char *s) { (void)ch; (void)s; }
static inline int SEGGER_RTT_printf(unsigned ch, const char *fmt, ...) { (void)ch; (void)fmt; return 0; }
static inline int SEGGER_RTT_Write(unsigned ch, const void *d, unsigned n) { (void)ch; (void)d; (void)n; return 0; }
#endif
