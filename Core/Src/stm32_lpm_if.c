/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32_lpm_if.c
  * @author  MCD Application Team
  * @brief   Low layer function to enter/exit low power modes (stop, sleep)
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
#include "platform.h"
#include "stm32_lpm.h"
#include "stm32_lpm_if.h"
#include "usart_if.h"

/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */
#include "atgm336h.h"  // For GNSS_HandleTypeDef and power state check
#include "../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.h"  // For TCXO control
#include "w25q16jv.h"  // For external flash deep power-down
#include "stm32wlxx_ll_pwr.h"  // For LL_PWR_ClearFlag_C1STOP_C1STB
#include "mission_state.h"  // R09: LED gating
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern I2C_HandleTypeDef hi2c2;
extern W25Q_HandleTypeDef hw25q;  /* defined in main.c */
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern SUBGHZ_HandleTypeDef hsubghz;
extern ADC_HandleTypeDef hadc;
extern IWDG_HandleTypeDef hiwdg;
extern RTC_HandleTypeDef hrtc;
void SystemClock_Config(void);
void MX_DMA_Init(void);
void MX_USART1_UART_Init(void);
void MX_SUBGHZ_Init(void);
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief Power driver callbacks handler
  */
const struct UTIL_LPM_Driver_s UTIL_PowerDriver =
{
  PWR_EnterSleepMode,
  PWR_ExitSleepMode,

  PWR_EnterStopMode,
  PWR_ExitStopMode,

  PWR_EnterOffMode,
  PWR_ExitOffMode,
};

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* IWDG Chunked Sleep Configuration (DDR-0020: Watchdog and Progress Supervision)
 * IWDG timeout = (4095 × 256) / 32000 ≈ 32.76 seconds
 * Wake interval must be safely below this to refresh the watchdog.
 * RTC Wakeup Timer: RTCCLK/16 = 32768/16 = 2048 Hz */
#define IWDG_SAFE_SLEEP_SECONDS   25     /* Must be < 32.76s IWDG timeout */
#define IWDG_WAKEUP_COUNTS        (IWDG_SAFE_SLEEP_SECONDS * 2048)  /* 51200 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Static flash handle for deep power-down control */
/* NOTE: If you use flash logging, make this extern and define it in your main code */
// static W25Q_HandleTypeDef hw25q_local = {0};  // Reserved for future flash power management
// static W25Q_HandleTypeDef *hw25q_ptr = NULL;  // Reserved for future flash power management
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

void PWR_EnterOffMode(void)
{
  /* USER CODE BEGIN EnterOffMode_1 */

  /* USER CODE END EnterOffMode_1 */
}

void PWR_ExitOffMode(void)
{
  /* USER CODE BEGIN ExitOffMode_1 */

  /* USER CODE END ExitOffMode_1 */
}

/* FR-06 (#84): the R07 (#29) ISR-set wake-source latches were deleted.
 * PWR_EnterStopMode() runs with interrupts disabled (UTIL_LPM critical
 * section -> __disable_irq), so the HAL event callbacks never execute inside
 * the chunk loop and the latches were never set — every chunk took the full
 * exit path. The chunk loop now reads RTC->SR (WUTF/ALRAF) directly, which
 * is valid under PRIMASK. Do not reintroduce ISR-set latches here. */

void PWR_EnterStopMode(void)
{
  /* USER CODE BEGIN EnterStopMode_1 */
  
  /* === DIAGNOSTIC: LED OFF while sleeping === */
  /* PA0 LOW = MCU entering STOP2 (sleep indicator) */
  /* If LED stays OFF for long periods = MCU is sleeping properly */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  
  /* TCXO Control: PB0 is automatically managed by SUBGHZ peripheral */
  /* Manual GPIO control removed - causes conflict with automatic TCXO management */
  /* LoRaWAN stack manages radio sleep - we don't touch it */
  
  /* === CRITICAL: Put External Flash into Deep Power-Down === */
  /* FW-14: this was commented out (dead hw25q_ptr plumbing). hw25q is a
   * global in main.c, so wire it directly. Deep power-down (0xB9) takes the
   * flash from ~1-3mA standby to <1uA; the first transaction after wake
   * re-releases it (see PWR_ExitStopMode). */
  if (hw25q.initialized) {
    W25Q_PowerDown(&hw25q);
  }
  
  /* === I2C2 Power Optimization: DeInit and set pins to ANALOG === */
  /* Prevents ~0.6-1.0mA leakage through external 10kΩ pullups (PA15=SDA, PB15=SCL) */
  /* Reference: archive/I2C_Power_Optimization_Fix.md */
  HAL_I2C_DeInit(&hi2c2);
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // PA15 = I2C2_SDA
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // PB15 = I2C2_SCL
  
  /* === SPI2 Power Optimization: DeInit and set pins to ANALOG === */
  /* Extra safety measure - flash already in deep power-down via W25Q_PowerDown() */
  /* SPI pins: PB13=SCK, PB14=MISO, PB15=MOSI, PC8=NSS */
  //HAL_SPI_DeInit(&hspi2);
  
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;  // SCK, MISO (PB15 already set above)
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_8;  // NSS
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* === UART1 Power Optimization === */
  /* Deinitialize UART peripheral (required for STOP2) */
  /* GPS is fully powered off (PB5=LOW, PB10=LOW), no parasitic power path */
  HAL_UART_DeInit(&huart1);
  
  /* CRITICAL: Both UART pins to ANALOG - GPS fully powered off, no leakage */
  GPIO_InitTypeDef GPIO_UART = {0};
  GPIO_UART.Pin = GPIO_PIN_6 | GPIO_PIN_7;  // PB6=TX, PB7=RX
  GPIO_UART.Mode = GPIO_MODE_ANALOG;
  GPIO_UART.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_UART);
  
  /* === ADC Power Optimization: DeInit and set pin to ANALOG === */
  /* ADC uses PB4 (ADC_CHANNEL_3) for battery voltage measurement */
  /* Prevent leakage current through ADC pin during sleep */
  HAL_ADC_DeInit(&hadc);
  { extern void SYS_ADC_NoteDeinit(void); SYS_ADC_NoteDeinit(); }  /* A-001 (#31): reset per-cycle ADC/VDDA state */
  
  GPIO_InitStruct.Pin = GPIO_PIN_4;  // PB4 = ADC_IN3 (Battery voltage)
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* === STOP2 Power Optimization: Disable Peripheral Clocks === */
  /* Reduces ~30-60µA by gating unused peripheral clocks */
  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_DMAMUX1_CLK_DISABLE();
  
  /* === STOP2 Power Optimization: VREFINT note ===
   * R17 (#31): HAL_SYSCFG_DisableVREFBUF removed. VREFBUF is the EXTERNAL VREF+
   * buffer, not VREFINT — our ratiometric VDDA path uses VREFINT + factory cal
   * and never touches VREFBUF. Worse, on this module VREF+ is bonded to VDDA,
   * so enabling the 2.5 V buffer would fight the supply. VREFBUF stays at its
   * power-on state (disabled) everywhere. */
  
  /* === Additional GPIO Power Optimization === */
  /* Set all unused/inactive pins to ANALOG mode to minimize leakage */

  /* GPS pins are managed by GNSS driver - DO NOT override here! */
  /* FW-8: GNSS_EnterStandby() actually sets PB6=OUTPUT-LOW, PB7=ANALOG,
   * PB5=LOW, **PB10=LOW** — full power-off (0µA), NOT hot-start backup mode.
   * Hot-start retention relies on ephemeris saved to GPS internal flash via
   * PCAS12, not on a live backup rail. Bench gate pending: if TTF is not
   * hot-start-fast, restoring PB10-HIGH standby is a separate measured fix. */
  
  /* External flash MOSI pin to ANALOG if on PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;  // PA10
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* External flash CS - drive HIGH to deselect flash during sleep */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);  // PB9 = Flash CS
  
  /* UART2 pins to ANALOG (if configured) */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;  // PA2=UART2_TX, PA3=UART2_RX
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* I2C3 pins to ANALOG (if configured) */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;  // PC0=I2C3_SCL, PC1=I2C3_SDA
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* USER CODE END EnterStopMode_1 */
  HAL_SuspendTick();
  /* Clear Status Flag before entering STOP/STANDBY Mode */
  LL_PWR_ClearFlag_C1STOP_C1STB();

  /* USER CODE BEGIN EnterStopMode_2 */
  /* === IWDG Chunked Sleep (DDR-0020: Watchdog and Progress Supervision) ===
   * Problem: IWDG timeout is ~32.76s, but sleep periods can be 5+ minutes.
   *   Without chunked sleep, IWDG resets the MCU during STOP2.
   * Solution: Use RTC Wakeup Timer to wake every 25s, refresh IWDG,
   *   then re-enter STOP2. LoRaWAN RTC Alarm A also wakes us for real events.
   * Exit conditions: LoRaWAN alarm fires, or any non-wakeup-timer interrupt. */

  /* Enable NVIC for RTC Wakeup Timer (uses EXTI line 19 internally) */
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

  /* FW-4: if the TxTimer/Alarm-A chain dies while the RTC wakeup timer keeps
   * ticking, this loop would sleep in 25s chunks forever with a satisfied
   * IWDG. Two guards: (1) run the progress deadman here — it reads RTC time
   * and self-resets; (2) bound the chunk count past the worst-case cycle
   * (SURVIVAL = 1h -> 150 chunks of 25s ~ 62.5 min). */
  extern void Deadman_Check(void);  /* defined in lora_app.c */
  uint32_t chunks = 0;

  while (1)
  {
    /* Set RTC Wakeup Timer: RTCCLK/16 = 2048 Hz, 25s = 51200 counts */
    /* 4th param: WakeUpAutoClr = 0 (no auto-clear, we clear manually) */
    /* F-09 (#76): if the WUT fails to arm, entering STOP2 here would sleep
     * with NO scheduled wake. Fall back to a bounded busy-wait (with IWDG
     * refresh) and exit chunked sleep instead. */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, IWDG_WAKEUP_COUNTS,
                                     RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0) != HAL_OK)
    {
      for (int i = 0; i < 25; i++)
      {
        HAL_Delay(1000);
        HAL_IWDG_Refresh(&hiwdg);
      }
      break;
    }

    /* Clear wakeup flags before sleeping */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    LL_PWR_ClearFlag_C1STOP_C1STB();

    /* USER CODE END EnterStopMode_2 */
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    /* USER CODE BEGIN EnterStopMode_3 */

    /* === Woke up — immediately refresh IWDG === */
    HAL_IWDG_Refresh(&hiwdg);
    Deadman_Check();  /* FW-4: no-op in COMMISSIONING; resets if no work cycle for 3h */

    /* The WUT interrupt pended but its ISR never ran (PRIMASK set), so the
     * NVIC pending bit is still latched. Clear it or the next WFI returns
     * immediately and the classification below sees a flag-less spurious
     * wake (which takes the "something else" exit). */
    NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);

    /* FR-06 (#84): classify the wake source from RTC hardware flags, which
     * ARE readable with PRIMASK set (no ISR runs inside this critical
     * section, so the R07 callback latches were never set and every chunk
     * took the full exit path). Sample SR BEFORE
     * HAL_RTCEx_DeactivateWakeUpTimer(), which clears WUTF. ALRAF is read
     * but deliberately NOT cleared here: after the break, IRQs re-enable and
     * the pending Alarm-A IRQ runs the UTIL_TIMER chain that owns it. */
    uint32_t rtc_sr          = READ_REG(RTC->SR);
    uint32_t is_alarm_a      = (rtc_sr & RTC_SR_ALRAF) != 0U;
    uint32_t is_wakeup_timer = (rtc_sr & RTC_SR_WUTF) != 0U;

    /* F-08 (#76): deactivate the wakeup timer BEFORE any exit path. The
     * chunk-overflow break used to skip this, leaving the WUT armed. */
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    if (++chunks > 150) break;  /* FW-4: never sleep forever (belt and braces) */

    /* Alarm A (LoRaWAN timer event) wins over the IWDG chunk timer. */
    if (is_alarm_a)
    {
      /* LoRaWAN timer event — exit chunked sleep. WUTF PR already consumed. */
      break;
    }

    if (is_wakeup_timer)
    {
      /* Only our IWDG refresh timer fired — PR consumed by handler,
       * re-enter STOP2 for the next chunk. */
      continue;
    }

    /* Something else woke us (GPIO interrupt, etc.) — exit */
    break;
  }

  /* Disable wakeup timer NVIC (no longer needed until next sleep) */
  HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
  /* USER CODE END EnterStopMode_3 */
}

void PWR_ExitStopMode(void)
{
  /* USER CODE BEGIN ExitStopMode_1 */
  
  /* === DIAGNOSTIC: LED ON while awake — COMMISSIONING only (R09/DDR-0002) ===
   * In FLIGHT the LED costs power on every 25 s wake for zero benefit. */
  if (MissionState_IsCommissioning()) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  }
  
  /* === PERIPHERAL RE-INITIALIZATION AFTER STOP2 === */
  /* STM32WL loses peripheral configuration in STOP2 mode */
  /* Must restore in proper dependency order */

  /* NOTE: SystemClock_Config() REMOVED - STM32WL auto-restores clock after STOP2 */
  /* Calling it was causing 30mA power draw issue */
  /* System wakes on MSI (4MHz) which is sufficient, radio driver handles PLL if needed */

  /* F-029 (#58): every re-init is status-checked — a peripheral that fails to
   * come back must be VISIBLE, not silently used blind. Failures are counted
   * per peripheral class and printed; a flash (SPI) re-init failure is the
   * flight-critical one (the archive depends on it) and gets its own flag. */
  static uint32_t stop2_reinit_fail_count = 0;
  bool reinit_failed = false;

  /* Re-enable peripheral clocks that were disabled for STOP2 */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  /* Re-initialize DMA before peripherals that use it (UART) */
  MX_DMA_Init();

  /* Re-initialize I2C2 - sensors need this to work */
  HAL_I2C_DeInit(&hi2c2);
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    SONDE_LOG_STR("STOP2 REINIT FAIL: I2C2\r\n");
    reinit_failed = true;
  }

  /* Re-initialize SPI2 - external flash needs this */
  HAL_SPI_DeInit(&hspi2);
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    SONDE_LOG_STR("STOP2 REINIT FAIL: SPI2 (flash archive at risk)\r\n");
    reinit_failed = true;
  }

  /* FW-14: wake the flash from deep power-down. tRES1 = 3us max per
   * datasheet; W25Q_ReleasePowerDown already delays 1ms. */
  if (hw25q.initialized) {
    if (W25Q_ReleasePowerDown(&hw25q) != W25Q_OK) {
      SONDE_LOG_STR("STOP2 REINIT FAIL: W25Q wake\r\n");
      reinit_failed = true;
    }
  }

  /* R17 (#31): VREFBUF enable removed — see the sleep-entry note. The FW-15
   * comment conflated VREFBUF (external VREF+ buffer) with VREFINT (internal
   * channel, used by the ratiometric VDDA path). VREFINT needs no enable
   * here; the ADC channel-read path enables it via the channel config. */

  /* Re-initialize UART1 only if GPS is powered */
  /* Prevents parasitic power when GPS is off */
  extern GNSS_HandleTypeDef hgnss;
  if (hgnss.is_powered) {
    HAL_UART_DeInit(&huart1);
    if (HAL_UART_Init(&huart1) != HAL_OK) {
      SONDE_LOG_STR("STOP2 REINIT FAIL: UART1 (GNSS)\r\n");
      reinit_failed = true;
    }
  }

  if (reinit_failed) {
    stop2_reinit_fail_count++;
    SONDE_LOG("STOP2 reinit failures this boot: %lu\r\n",
                      (unsigned long)stop2_reinit_fail_count);
  }
  
  /* USER CODE END ExitStopMode_1 */
  /* Resume sysTick : work around for debugger problem in dual core */
  HAL_ResumeTick();
  /*Not retained periph:
    ADC interface
    DAC interface USARTx, TIMx, i2Cx, SPIx
    SRAM ctrls, DMAx, DMAMux, AES, RNG, HSEM  */

  /* Resume not retained USARTx and DMA */
  vcom_Resume();
  /* USER CODE BEGIN ExitStopMode_2 */

  /* USER CODE END ExitStopMode_2 */
}

void PWR_EnterSleepMode(void)
{
  /* USER CODE BEGIN EnterSleepMode_1 */

  /* USER CODE END EnterSleepMode_1 */
  /* Suspend sysTick */
  HAL_SuspendTick();
  /* USER CODE BEGIN EnterSleepMode_2 */

  /* USER CODE END EnterSleepMode_2 */
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  /* USER CODE BEGIN EnterSleepMode_3 */

  /* USER CODE END EnterSleepMode_3 */
}

void PWR_ExitSleepMode(void)
{
  /* USER CODE BEGIN ExitSleepMode_1 */

  /* USER CODE END ExitSleepMode_1 */
  /* Resume sysTick */
  HAL_ResumeTick();

  /* USER CODE BEGIN ExitSleepMode_2 */

  /* USER CODE END ExitSleepMode_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
