/**
  ******************************************************************************
  * @file    reset_cause.c
  * @brief   Reset cause capture and condensation (F13b / ADR-0007)
  ******************************************************************************
  */

#include "main.h"
#include "reset_cause.h"

extern RTC_HandleTypeDef hrtc;

static uint8_t s_reset_cause = RESET_CAUSE_UNKNOWN;

void ResetCause_CaptureBoot(void)
{
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
        s_reset_cause = RESET_CAUSE_PIN;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
        s_reset_cause = RESET_CAUSE_LOWPOWER;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
        s_reset_cause = RESET_CAUSE_POR_BOR;
    } else {
        s_reset_cause = RESET_CAUSE_UNKNOWN;
    }

    /* Clear all reset flags so the next boot reads clean */
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint8_t ResetCause_Get(void)
{
    return s_reset_cause;
}
