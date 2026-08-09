/**
  ******************************************************************************
  * @file    reset_cause.c
  * @brief   Reset cause capture and condensation (F13b / DDR-0007)
  ******************************************************************************
  */

#include "main.h"
#include "reset_cause.h"

extern RTC_HandleTypeDef hrtc;

static uint8_t s_reset_cause = RESET_CAUSE_POR_BOR;  /* FW-7: unknown -> benign bucket */

void ResetCause_CaptureBoot(void)
{
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
        s_reset_cause = RESET_CAUSE_SW;      /* FW-7: PIN folds into SW bucket */
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
        s_reset_cause = RESET_CAUSE_POR_BOR; /* FW-7: low-power folds into POR bucket */
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
        s_reset_cause = RESET_CAUSE_POR_BOR;
    } else {
        s_reset_cause = RESET_CAUSE_POR_BOR; /* unknown -> safest benign bucket */
    }

    /* F-03 (#65): persistent consecutive-boot counter. Incremented on every
     * boot; cleared by lora_app.c Deadman_MarkProgress() once a work cycle
     * provably starts. FR-23 (#104): Error_Handler_Fatal() reads it to break
     * deterministic fatal reset loops (degrade instead of the 6th reset). */
    uint32_t attempts = HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_BOOT_ATTEMPTS);
    HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_BOOT_ATTEMPTS, attempts + 1U);

    /* Clear all reset flags so the next boot reads clean */
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint32_t ResetCause_GetBootAttempts(void)
{
    return HAL_RTCEx_BKUPRead(&hrtc, BKP_REG_BOOT_ATTEMPTS);
}

void ResetCause_ClearBootAttempts(void)
{
    HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_BOOT_ATTEMPTS, 0);
}

uint8_t ResetCause_Get(void)
{
    return s_reset_cause;
}
