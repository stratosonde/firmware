/**
  ******************************************************************************
  * @file    reset_cause.h
  * @brief   Reset cause capture and condensation (F13b / ADR-0007)
  ******************************************************************************
  * Reads RCC->CSR once at boot, condenses to a 3-bit code for the uplink
  * status byte, then clears the flags (RMVF) so the next boot reads clean.
  ******************************************************************************
  */

#ifndef __RESET_CAUSE_H
#define __RESET_CAUSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief Condensed reset cause codes (3 bits, status byte b3-b5) */
typedef enum {
    RESET_CAUSE_POR_BOR   = 0,  /* POR/BOR (indistinguishable on STM32WL: BORRSTF covers both) */
    RESET_CAUSE_RESERVED1 = 1,
    RESET_CAUSE_IWDG      = 2,  /* Independent watchdog reset */
    RESET_CAUSE_SW        = 3,  /* Software reset (NVIC_SystemReset) */
    RESET_CAUSE_PIN       = 4,  /* External pin reset */
    RESET_CAUSE_FAULT     = 5,  /* Fault handler breadcrumb present (see F1) */
    RESET_CAUSE_LOWPOWER  = 6,  /* Low-power reset */
    RESET_CAUSE_UNKNOWN   = 7
} ResetCause_t;

/** @brief RTC backup register holding the fault breadcrumb (F1) */
#define RESET_CAUSE_BKP_FAULT_REG   RTC_BKP_DR1
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
 * @retval 3-bit ResetCause_t code
 */
uint8_t ResetCause_Get(void);

#ifdef __cplusplus
}
#endif

#endif /* __RESET_CAUSE_H */
