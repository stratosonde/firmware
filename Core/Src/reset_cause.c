/**
 ******************************************************************************
 * @file    reset_cause.c
 * @brief   Reset cause capture and condensation (F13b / DDR-0003)
 ******************************************************************************
 */

#include "reset_cause.h"
#include "main.h"
#include "sonde_log.h" /* SONDE_LOG for the fault-context report */

extern RTC_HandleTypeDef hrtc;

static uint8_t s_reset_cause = RESET_CAUSE_POR_BOR; /* FW-7: unknown -> benign bucket */

void ResetCause_CaptureBoot(void) {
  /* FW-2: TAMP backup registers are unreadable (read 0) while the RTCAPB
   * clock is gated. This runs before RTC init (main.c:162 vs RTC init
   * inside MX_LoRaWAN_Init), so enable access explicitly — otherwise the
   * F1 fault breadcrumb check always fails and faults degrade to SW. */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTCAPB_CLK_ENABLE();

  /* Fault breadcrumb (F1) outranks everything: the fault handler reset us */
  uint32_t breadcrumb = HAL_RTCEx_BKUPRead(&hrtc, RESET_CAUSE_BKP_FAULT_REG);
  if ((breadcrumb & RESET_CAUSE_FAULT_MASK) == RESET_CAUSE_FAULT_MAGIC) {
    s_reset_cause = RESET_CAUSE_FAULT;
    /* Clear after reading: report the fault ONCE, not on every future boot */
    HAL_RTCEx_BKUPWrite(&hrtc, RESET_CAUSE_BKP_FAULT_REG, 0);
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    s_reset_cause = RESET_CAUSE_IWDG;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
    s_reset_cause = RESET_CAUSE_SW;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
    s_reset_cause = RESET_CAUSE_SW; /* FW-7: PIN folds into SW bucket */
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
    s_reset_cause = RESET_CAUSE_POR_BOR; /* FW-7: low-power folds into POR bucket */
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
    s_reset_cause = RESET_CAUSE_POR_BOR;
  } else {
    s_reset_cause = RESET_CAUSE_POR_BOR; /* unknown -> safest benign bucket */
  }

  /* F-03 (#65): persistent consecutive-boot counter. Incremented on every
   * boot; cleared by lora_app.c after successful cycle completion or normal
   * admission sleep. It is diagnostic evidence only. */
  uint32_t attempts = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_BOOT_ATTEMPTS);
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_BOOT_ATTEMPTS, attempts + 1U);

  /* Clear all reset flags so the next boot reads clean */
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint32_t ResetCause_GetBootAttempts(void) {
  return HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_BOOT_ATTEMPTS);
}

void ResetCause_ClearBootAttempts(void) {
  HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_BOOT_ATTEMPTS, 0);
}

uint8_t ResetCause_Get(void) {
  return s_reset_cause;
}

/* F-DIAG: cached fault context. The early (pre-SystemClock_Config) print is
 * required because the LSE-CSS recovery can BDRST-wipe TAMP before the boot
 * reaches the reliably-visible log region — but lines printed that early are
 * themselves often lost by the RTT host attach race. So the first read CACHES
 * into these statics; ResetCause_RepaintFaultContext() re-prints from RAM at
 * the "Reset cause" line, which every capture shows. */
static uint8_t s_ctx_valid = 0; /* 0 none, nonzero = full context cached */
static uint32_t s_cls = 0, s_pc = 0, s_cfsr = 0;
/* F-DIAG: the F17B breadcrumb's low 16 bits name the resetter (0=NMI,
 * 1-4=CPU fault class, 6=deadman, 16+=boot fatal e.g. FAULT_CODE_FLASH_INIT).
 * CaptureBoot reports only "cause 3" and clears it — capture the code here,
 * pre-BDRST, so it survives and prints at the visible repaint point. */
static uint8_t s_breadcrumb_valid = 0;
static uint32_t s_breadcrumb_code = 0;

static void PrintFaultContextFromCache(void) {
  if (s_breadcrumb_valid != 0U) {
    SONDE_LOG("FAULT BREADCRUMB: code=%lu (0=NMI 1=HardFault 2=MemManage 3=BusFault 4=UsageFault 6=deadman 16=clk 17=payload 18=flash-init 19=iwdg 20=rtc-init 21=rtc-stall)\r\n",
              (unsigned long)s_breadcrumb_code);
  }
  if (s_ctx_valid != 0U) {
    uint32_t cfsr = s_cfsr;
    SONDE_LOG("FAULT CONTEXT: class=%lu PC=0x%08lX CFSR=0x%08lX\r\n",
              (unsigned long)s_cls, (unsigned long)s_pc, (unsigned long)cfsr);
    SONDE_LOG("  BFSR: IBUSERR=%lu PRECISERR=%lu IMPRECISERR=%lu BFARVALID=%lu\r\n",
              (unsigned long)((cfsr >> 8) & 1UL), (unsigned long)((cfsr >> 9) & 1UL),
              (unsigned long)((cfsr >> 10) & 1UL), (unsigned long)((cfsr >> 15) & 1UL));
    SONDE_LOG("  UFSR: UNDEFINSTR=%lu INVSTATE=%lu INVPC=%lu UNALIGNED=%lu DIVBYZERO=%lu\r\n",
              (unsigned long)((cfsr >> 16) & 1UL), (unsigned long)((cfsr >> 17) & 1UL),
              (unsigned long)((cfsr >> 18) & 1UL), (unsigned long)((cfsr >> 24) & 1UL),
              (unsigned long)((cfsr >> 25) & 1UL));
  }
}

void ResetCause_RepaintFaultContext(void) {
  PrintFaultContextFromCache();
}

void ResetCause_PrintFaultContext(void) {
  /* F-DIAG (boot-loop root cause): read the TAMP backup registers DIRECTLY.
   * HAL_RTCEx_BKUPRead goes through the RTC shadow registers, which are NOT
   * synced until MX_RTC_Init (deep inside MX_LoRaWAN_Init) — so at this early
   * boot point a HAL read can return stale/0 even though the fault handler's
   * direct TAMP write landed. Direct reads match the write side exactly and
   * are reliable regardless of RTC init state. */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTCAPB_CLK_ENABLE();

  /* F-DIAG: snapshot the F17B breadcrumb code FIRST — this runs before
   * SystemClock_Config, whose LSE-CSS recovery can BDRST-wipe DR4 before
   * CaptureBoot reads it (which is why those boots degrade to "cause 2"). */
  {
    uint32_t dr4 = TAMP->BKP4R; /* BKP_REG_RESET_CAUSE_FAULT */
    if ((dr4 & RESET_CAUSE_FAULT_MASK) == RESET_CAUSE_FAULT_MAGIC) {
      s_breadcrumb_valid = 1U;
      s_breadcrumb_code = dr4 & 0xFFFFUL;
    }
  }

  uint32_t dr16 = TAMP->BKP16R;

  if (TAMP->BKP19R != RESET_CAUSE_FAULT_CTX_MAGIC) { /* BKP_REG_FAULT_MAGIC */
    return;                                          /* no fault context captured (or already reported) */
  }
  s_ctx_valid = 1U;
  s_pc = dr16;           /* BKP_REG_FAULT_PC */
  s_cls = TAMP->BKP17R;  /* class (0-4) */
  s_cfsr = TAMP->BKP18R; /* CFSR */
  TAMP->BKP19R = 0;      /* report once: clear the context magic */
  PrintFaultContextFromCache();
}
