/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wlxx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32wlxx_it.h"
#include "reset_cause.h"  /* F1: breadcrumb register constants */
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern SUBGHZ_HandleTypeDef hsubghz;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
/* DMA RX handle for GNSS - declared in stm32wlxx_hal_msp.c */
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
static void Fault_Reset(uint16_t code); /* F1: defined below */

/**
 * F-DIAG: capture the fault CONTEXT into RTC backup (TAMP) registers before
 * the breadcrumb-and-reset, so a repeating boot-loop fault can be localized
 * on the next boot. Writes TAMP->BKPxxR DIRECTLY (not HAL_RTCEx_BKUPWrite):
 * the latter read-modify-writes the RTC shadow registers, which are NOT
 * reliable in fault context when the fault fires before HAL_RTC_Init. The
 * TAMP battery-backed registers are single 32-bit stores -- atomic, no sync.
 * SCB->CFSR is a plain system-register read (always safe). sp is the
 * interrupted frame stack pointer (MSP or PSP per EXC_RETURN) passed by the
 * naked handler; sp[6] is the stacked (faulting) PC.
 */
__attribute__((always_inline)) static inline void
Fault_CaptureContext(const uint32_t *sp, uint16_t code) {
  uint32_t pc = (sp != 0) ? sp[6] : 0U;
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
  TAMP->BKP16R = pc;                          /* BKP_REG_FAULT_PC */
  TAMP->BKP17R = (uint32_t)code;              /* BKP_REG_FAULT_CFSR: class */
  TAMP->BKP18R = SCB->CFSR;                   /* BKP_REG_FAULT_BFAR: CFSR */
  TAMP->BKP19R = RESET_CAUSE_FAULT_CTX_MAGIC; /* BKP_REG_FAULT_MAGIC */
}

/* F-DIAG: common tail for the naked fault handlers. r0 = interrupted frame
 * SP, r1 = fault class. Captures context, then runs the F1 epilogue. Marked
 * used + noinline: reached only via the asm trampolines. */
__attribute__((used, noinline)) static void
Fault_CaptureResetCommon(uint32_t *sp, uint16_t code) {
  Fault_CaptureContext(sp, code);
  Fault_Reset(code);
}

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* F1 FIX: NMI is also a brick trap on the stock template — breadcrumb + reset */
  Fault_Reset(0);  /* 0 = NMI (HSE CSS / flash ECC) */
  /* USER CODE END NonMaskableInt_IRQn 0 */
}

/**
  * F1 FIX (DDR-0009): Fault handlers are breadcrumb-and-reset, never brick traps.
  * At 40 km a hang is permanent death; the IWDG would eventually fire anyway,
  * but only after burning hours of LoRaWAN airtime budget and leaving no record
  * of *why*. Write a breadcrumb to an RTC backup register (survives reset,
  * read by ResetCause_CaptureBoot at next boot -> RESET_CAUSE_FAULT in the
  * uplink status byte), then reset immediately.
  * Keep these handlers minimal: no HAL calls beyond the backup write, no RTT
  * (the fault may have corrupted anything).
  */

/** @brief Shared fault epilogue: breadcrumb code, then system reset. */
static void Fault_Reset(uint16_t code)
{
  extern RTC_HandleTypeDef hrtc;
  /* Enable backup domain write access (PWR clock is always on for STM32WL) */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTC_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();  /* FW-2: TAMP regs need RTCAPB, not just RTCEN */
  HAL_RTCEx_BKUPWrite(&hrtc, RESET_CAUSE_BKP_FAULT_REG,
                      RESET_CAUSE_FAULT_MAGIC | (uint32_t)code);
  NVIC_SystemReset();
  while (1) { }  /* unreachable unless reset is somehow masked */
}

/**
 * @brief This function handles Hard fault interrupt.
 *        Naked: grab the active SP, capture fault context, reset.
 */
__attribute__((naked)) void HardFault_Handler(void) {
  __asm volatile(
      "tst lr, #4          \n"
      "ite eq              \n"
      "mrseq r0, msp       \n"
      "mrsne r0, psp       \n"
      "movs r1, #1         \n" /* 1 = HardFault */
      "b Fault_CaptureResetCommon \n");
}

/**
 * @brief This function handles Memory management fault.
 *        Naked: grab the active SP, capture fault context, reset.
 */
__attribute__((naked)) void MemManage_Handler(void) {
  __asm volatile(
      "tst lr, #4          \n"
      "ite eq              \n"
      "mrseq r0, msp       \n"
      "mrsne r0, psp       \n"
      "movs r1, #2         \n" /* 2 = MemManage */
      "b Fault_CaptureResetCommon \n");
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 *        Naked: grab the active SP, capture fault context, reset.
 */
__attribute__((naked)) void BusFault_Handler(void) {
  __asm volatile(
      "tst lr, #4          \n"
      "ite eq              \n"
      "mrseq r0, msp       \n"
      "mrsne r0, psp       \n"
      "movs r1, #3         \n" /* 3 = BusFault */
      "b Fault_CaptureResetCommon \n");
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 *        Naked: grab the active SP, capture fault context, reset.
 */
__attribute__((naked)) void UsageFault_Handler(void) {
  __asm volatile(
      "tst lr, #4          \n"
      "ite eq              \n"
      "mrseq r0, msp       \n"
      "mrsne r0, psp       \n"
      "movs r1, #4         \n" /* 4 = UsageFault */
      "b Fault_CaptureResetCommon \n");
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WLxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wlxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts.
  */
void TAMP_STAMP_LSECSS_SSRU_IRQHandler(void)
{
  /* USER CODE BEGIN TAMP_STAMP_LSECSS_SSRU_IRQn 0 */

  /* USER CODE END TAMP_STAMP_LSECSS_SSRU_IRQn 0 */
  HAL_RTCEx_SSRUIRQHandler(&hrtc);
  /* USER CODE BEGIN TAMP_STAMP_LSECSS_SSRU_IRQn 1 */
  HAL_RCCEx_LSECSS_IRQHandler();  /* F-14 (#70): LSE clock security dispatch */
  /* USER CODE END TAMP_STAMP_LSECSS_SSRU_IRQn 1 */
}

/**
  * @brief This function handles DMA1 Channel 2 Interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles USART1 Interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles RTC Alarms (A and B) Interrupt.
  */
void RTC_Alarm_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_Alarm_IRQn 0 */

  /* USER CODE END RTC_Alarm_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_Alarm_IRQn 1 */

  /* USER CODE END RTC_Alarm_IRQn 1 */
}

/**
  * @brief This function handles SUBGHZ Radio Interrupt.
  */
void SUBGHZ_Radio_IRQHandler(void)
{
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 0 */

  /* USER CODE END SUBGHZ_Radio_IRQn 0 */
  HAL_SUBGHZ_IRQHandler(&hsubghz);
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 1 */

  /* USER CODE END SUBGHZ_Radio_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief This function handles DMA1 Channel 1 Interrupt (USART1_RX for GNSS).
  * @note  CubeMX removed this, so we add it manually in USER CODE section
  */
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
  * @brief This function handles RTC Wakeup Timer Interrupt.
  * @note  Used by IWDG chunked sleep to periodically wake from STOP2
  *        and refresh the watchdog. See stm32_lpm_if.c PWR_EnterStopMode().
  */
void RTC_WKUP_IRQHandler(void)
{
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}
/* USER CODE END 1 */
