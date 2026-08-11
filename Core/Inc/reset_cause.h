/**
  ******************************************************************************
  * @file    reset_cause.h
  * @brief   Reset cause capture and condensation (F13b / DDR-0003)
  ******************************************************************************
 * Reads RCC->CSR once at boot, condenses to a 2-bit code for the uplink
 * status byte, then clears the flags (RMVF) so the next boot reads clean.
  ******************************************************************************
  */

#ifndef __RESET_CAUSE_H
#define __RESET_CAUSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "backup_regs.h"

/* FW-7: 2-bit condensation (status byte b3-b4; b5 is now press_stale).
 * Buckets: IWDG and FAULT are flight-actionable and stay distinct;
 * PIN folds into SW (both are manual/service resets, irrelevant in flight);
 * LOWPOWER folds into POR_BOR (power-domain resets). */
typedef enum {
    RESET_CAUSE_POR_BOR   = 0,  /* POR/BOR (+ low-power reset folded in) */
    RESET_CAUSE_IWDG      = 1,  /* Independent watchdog reset */
    RESET_CAUSE_SW        = 2,  /* Software reset / external pin reset */
    RESET_CAUSE_FAULT     = 3,  /* Fault handler breadcrumb present (see F1) */
} ResetCause_t;

/** @brief RTC backup register holding the fault breadcrumb (F1).
  *        R01/R02: moved DR1 -> DR4; DR0-DR2 belong to timer_if SysTime. */
#define RESET_CAUSE_BKP_FAULT_REG   BKP_REG_RESET_CAUSE_FAULT
/** @brief Breadcrumb magic: upper 16 bits of the breadcrumb register */
#define RESET_CAUSE_FAULT_MAGIC     0xF17B0000UL
#define RESET_CAUSE_FAULT_MASK      0xFFFF0000UL

/**
 * @brief Capture and condense the reset cause. Call once, early in boot,
 *        after SystemClock_Config() (backup domain access enabled).
 *        Clears RCC reset flags so the next boot reads clean.
 */
void ResetCause_CaptureBoot(void);

/**
 * @brief Get the condensed reset cause captured at boot.
 * @retval 2-bit ResetCause_t code
 */
uint8_t ResetCause_Get(void);

/**
 * @brief Consecutive boots since the last proven work cycle (F-03/#65).
 *        Incremented in ResetCause_CaptureBoot; cleared at successful cycle
 *        COMPLETION (end of SendTxData; F-6/#181 - clearing at cycle entry
 *        let an in-cycle fault erase its own evidence on every boot).
 *        Error_Handler_Fatal()
 *        uses it to break reset loops (FR-23/#104: boot-time fatals degrade
 *        instead of resetting past 5 consecutive unproductive boots).
 */
uint32_t ResetCause_GetBootAttempts(void);

/** @brief Clear the consecutive-boot counter (call on proven progress). */
void ResetCause_ClearBootAttempts(void);

#ifdef __cplusplus
}
#endif

#endif /* __RESET_CAUSE_H */
