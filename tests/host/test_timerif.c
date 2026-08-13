/* F-002 (#201) BEHAVIORAL test: the REAL timer_if.c compiled against a
 * minimal RTC HAL fake. The 2026-08-11 review finding: after a runtime RTC
 * backup-domain reset (LSE->LSI failover), calling TIMER_IF_Init was a
 * silent no-op because of the one-shot RTC_Initialized guard - the timer
 * subsystem was never reconstructed. The old RV-09 scan proved the CALL
 * existed; this suite proves the WORK happens (F-010 hierarchy).
 *
 * Model: init -> timers scheduled -> simulated backup-domain wipe ->
 * deliberate re-init -> every invalidated assumption verifiably rebuilt.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define HOST_USE_REAL_TIMER_IF   /* bypass the stubs/timer_if.h shadow */
#include "stm32wlxx_hal.h"

/* ---- fault-injecting fakes (defs before the unit is #included) ---- */
RTC_TypeDef g_host_rtc_regs;                 /* SSR = raw down-counter */
RTC_HandleTypeDef hrtc;                      /* externed by timer_if.c */

#define HOST_BKP_MAX 44
static uint32_t g_bkup[HOST_BKP_MAX];
static int g_mx_rtc_init_calls = 0;
static int g_deactivate_alarm_calls = 0;
static int g_set_alarm_calls = 0;
static int g_bypass_shadow_calls = 0;
static int g_error_handler_calls = 0;

void MX_RTC_Init(void) { g_mx_rtc_init_calls++; }
void Error_Handler(void) { g_error_handler_calls++; }
void UTIL_TIMER_IRQ_Handler(void) { }   /* IRQ-map target; not driven here */

HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *h, uint32_t Alarm)
{ (void)h; (void)Alarm; g_deactivate_alarm_calls++; return HAL_OK; }

HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *h, RTC_AlarmTypeDef *a, uint32_t fmt)
{ (void)h; (void)a; (void)fmt; g_set_alarm_calls++; return HAL_OK; }

HAL_StatusTypeDef HAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *h)
{ (void)h; g_bypass_shadow_calls++; return HAL_OK; }

void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *h, uint32_t reg, uint32_t data)
{ (void)h; if (reg < HOST_BKP_MAX) g_bkup[reg] = data; }

/* S-05 (#230): race injection - after g_bkupread_bump_after reads of the
 * MSB-ticks register, the MSB value increments by g_bkupread_bump_delta
 * (simulating the SSRU wrap firing between two reads). 0 = disabled. */
static uint32_t g_bkupread_count = 0;
static uint32_t g_bkupread_bump_after = 0;
static uint32_t g_bkupread_bump_delta = 0;

uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *h, uint32_t reg)
{
    (void)h;
    if (reg >= HOST_BKP_MAX) return 0;
    uint32_t v = g_bkup[reg];
    if (reg == RTC_BKP_DR2 /* RTC_BKP_MSBTICKS */ && g_bkupread_bump_after != 0) {
        g_bkupread_count++;
        if (g_bkupread_count > g_bkupread_bump_after) v += g_bkupread_bump_delta;
    }
    return v;
}

/* ---- the unit under test (same-TU statics: RTC_Initialized etc.) ---- */
#include "../../Core/Src/timer_if.c"

static int g_checks = 0, g_failures = 0;
static void CHECK(bool cond, const char *name)
{
    g_checks++;
    if (!cond) { g_failures++; printf("   FAIL: %s\n", name); }
}

int main(void)
{
    printf("=== F-002 (#201): TIMER_IF re-init after RTC backup-domain reset ===\n\n");

    /* 1. Cold boot: full init runs the hardware reconstruction. */
    g_host_rtc_regs.SSR = 0x00010000u;
    CHECK(TIMER_IF_Init() == UTIL_TIMER_OK, "init ok");
    CHECK(g_mx_rtc_init_calls == 1, "cold init ran MX_RTC_Init once");
    CHECK(g_bkup[BKP_REG_SYSTIME_VALID] == TIMER_IF_SYSTIME_VALID_MAGIC,
          "SysTime magic stamped");
    CHECK(g_bkup[RTC_BKP_DR2] == 0, "MSB-tick epoch zeroed at cold boot");
    int deact_after_boot = g_deactivate_alarm_calls;
    int bypass_after_boot = g_bypass_shadow_calls;
    CHECK(deact_after_boot >= 1, "Alarm A deactivated at boot");
    CHECK(bypass_after_boot == 1, "bypass-shadow set at boot");

    /* 2. Normal second call: guard makes it a no-op (intended one-shot). */
    CHECK(TIMER_IF_Init() == UTIL_TIMER_OK, "second init ok");
    CHECK(g_mx_rtc_init_calls == 1, "one-shot guard: no re-init");
    CHECK(g_bypass_shadow_calls == bypass_after_boot, "guard skips HW work");

    /* 3. Schedule a timer, then simulate the runtime failover's
     *    backup-domain reset: every backup register is wiped. */
    CHECK(TIMER_IF_StartTimer(2048) == UTIL_TIMER_OK, "timer scheduled");
    CHECK(g_set_alarm_calls == 1, "alarm programmed");
    memset(g_bkup, 0, sizeof(g_bkup));               /* backup domain reset */

    /* 3a. The OLD failover behavior: plain TIMER_IF_Init - demonstrate the
     *     F-002 defect class against the guard directly. */
    CHECK(TIMER_IF_Init() == UTIL_TIMER_OK, "stale init returns ok");
    CHECK(g_mx_rtc_init_calls == 1,
          "BUG CLASS: plain TIMER_IF_Init after RTC reset is a no-op");
    CHECK(g_bkup[BKP_REG_SYSTIME_VALID] == 0, "magic still wiped (no repair)");

    /* 3b. The fix: deliberate reconstruction. */
    CHECK(TIMER_IF_ReInitAfterRtcReset() == UTIL_TIMER_OK, "reinit ok");
    CHECK(g_mx_rtc_init_calls == 2, "reinit re-ran RTC hardware init");
    CHECK(g_bypass_shadow_calls == bypass_after_boot + 1, "bypass-shadow restored");
    CHECK(g_bkup[BKP_REG_SYSTIME_VALID] == TIMER_IF_SYSTIME_VALID_MAGIC,
          "SysTime magic restamped after reinit");
    CHECK(g_bkup[RTC_BKP_DR2] == 0, "MSB epoch legitimately restarts at 0");
    CHECK(g_error_handler_calls == 0, "no Error_Handler through the flow");

    /* 4. Post-reinit timers are consistent with the restarted counter:
     *    context recaptured from the CURRENT SSR, alarm programs cleanly. */
    g_host_rtc_regs.SSR = 0x00008000u;
    uint32_t ctx = TIMER_IF_SetTimerContext();
    CHECK(ctx == (0xFFFFFFFFu - 0x00008000u), "context from restarted RTC");
    CHECK(TIMER_IF_StartTimer(1024) == UTIL_TIMER_OK, "post-reinit alarm");
    CHECK(g_set_alarm_calls == 2, "post-reinit alarm programmed");
    uint32_t val = TIMER_IF_GetTimerValue();
    CHECK(val != 0, "tick value live after reinit");

    /* R3-08 (#112): the tick->ms conversion HAL_GetTick must apply at the
     * abstraction boundary (1024 Hz RTC tick was treated as 1000 Hz ms -
     * every ms timeout ran ~2.4% short). Exact at the review's probe
     * points; sane at the 48.5-day tick-era wrap. */
    printf("\n-- R3-08 (#112): tick->ms boundary conversion\n");
    CHECK(TIMER_IF_Convert_Tick2ms(1024u) == 1000u, "1 s exact");
    CHECK(TIMER_IF_Convert_Tick2ms(10240u) == 10000u, "10 s exact");
    CHECK(TIMER_IF_Convert_Tick2ms(30720u) == 30000u, "30 s exact");
    CHECK(TIMER_IF_Convert_Tick2ms(614400u) == 600000u, "10 min exact");
    CHECK(TIMER_IF_Convert_Tick2ms(0xFFFFFFFFu) > 4000000000u,
          "wrap: last tick of the era does not overflow-wrap small");
    CHECK(TIMER_IF_Convert_Tick2ms(1u) == 0u, "1 tick truncates to 0 ms");

    /* S-05 (#230): MSB/LSB read race in TIMER_IF_GetTime. Inject an MSB
     * increment between the loads (the SSRU wrap firing mid-read): the
     * composed time must use the NEW MSB with a re-read LSB, never the
     * stale pair (a ~48.5-day forward jump). */
    printf("\n-- S-05 (#230): TIMER_IF_GetTime MSB/LSB race\n");
    {
        g_bkup[RTC_BKP_DR2] = 5;   /* RTC_BKP_MSBTICKS */
        g_host_rtc_regs.SSR = 0xFFFFFF00u;   /* lsb = 0xFF = 255 ticks */
        g_bkupread_count = 0;
        g_bkupread_bump_after = 1;           /* 1st MSB read stale, later reads new */
        g_bkupread_bump_delta = 1;

        uint16_t ms = 0;
        uint32_t sec = TIMER_IF_GetTime(&ms);

        g_bkupread_bump_after = 0;           /* disarm */
        /* correct: ticks = (6<<32)|255 -> sec = 6<<22 = 25165824, ms = 249 */
        printf("   mid-read wrap: sec=%lu ms=%u (want 25165824, 249)\n",
               (unsigned long)sec, ms);
        CHECK(sec == 25165824u && ms == 249u, "S-05: new MSB + re-read LSB");
    }

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
