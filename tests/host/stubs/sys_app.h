/* Host-test stub: sys_app (R15 harness) — multiregion_context.c calls nothing
 * from it, but APP_LOG/TS_ON/VLEVEL_* arrive via this header on target. */
#ifndef SYS_APP_H_STUB
#define SYS_APP_H_STUB

#define TS_ON     1
#define VLEVEL_H  2
#define VLEVEL_M  1
#define VLEVEL_L  0
#define APP_LOG(ts, level, ...)  ((void)0)

#endif