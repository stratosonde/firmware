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
#include "sys_caps.h"  // LT-05 (#275): wake re-init failures mark capabilities
#include "config.h"  // DR-04 (#240): CONFIG_MAX_TX_INTERVAL_MS for MAX_SLEEP_CHUNKS
#include "timer_if.h"  // [DIAG] TIMER_IF_GetTimerValue for the wake-source dump
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
void GNSS_UARTPins_SleepSafe(void);  /* SP-09 (#249): GNSS driver owns PB6/PB7 sleep policy */
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
 * RTC Wakeup Timer: RTCCLK/16 = 32768/16 = 2048 Hz
 * R2-09 (#113): 25 s only LOOKS like 24% headroom — chunk clock is LSE (RTC),
 * watchdog clock is LSI, and at LSI's fast tolerance the real timeout is ~31 s;
 * LSE→LSI failover stretches the chunk further, and post-wake work
 * (Deadman_Check, wakeup-timer re-arm, register-sync waits) eats more.
 * True margin at 25 s was ~4–5 s. 20 s restores real margin for free. */
#define IWDG_SAFE_SLEEP_SECONDS   20     /* Must be < ~31 s worst-case IWDG timeout */
#define IWDG_WAKEUP_COUNTS        (IWDG_SAFE_SLEEP_SECONDS * 2048)  /* 40960 */
/* F-7 (#182): the chunk ceiling is DERIVED, not a literal. It must cover the
 * largest interval Config_Validate accepts in IWDG_SAFE_SLEEP_SECONDS chunks,
 * plus one - the hardcoded 180 (181 x 20 s = 3620 s) aborted chunked sleep
 * every cycle for any validated survival interval above ~1 h and spun the
 * main loop at full power in the mode that exists to save power (#134
 * recurring). DR-04 (#240): the ceiling is CONFIG_MAX_TX_INTERVAL_MS from
 * config.h - the SAME constant Config_Validate enforces, so the assert below
 * references a real enforced limit, not a local mirror that can drift. */
#define MAX_TX_INTERVAL_MS        CONFIG_MAX_TX_INTERVAL_MS
#define MAX_SLEEP_CHUNKS          ((MAX_TX_INTERVAL_MS / 1000UL / IWDG_SAFE_SLEEP_SECONDS) + 1UL)
_Static_assert(MAX_SLEEP_CHUNKS * IWDG_SAFE_SLEEP_SECONDS >= MAX_TX_INTERVAL_MS / 1000UL,
               "chunk ceiling must cover the maximum validated TX interval");
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
  /* SPI2 pins: PB13=SCK, PB14=MISO, PA10=MOSI (AF5), PB9=CS (GPIO, driven
   * high below). PB15 is I2C2_SCL, NOT MOSI - see stm32wlxx_hal_msp.c.
   * PA10 is set to analog separately below. (S-11, 2026-08-12 review.) */
  //HAL_SPI_DeInit(&hspi2);

  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;  // SCK, MISO
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8;  // PC8: not an SPI pin (legacy comment said NSS); analog anyway
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* === UART1 Power Optimization === */
  /* Deinitialize UART peripheral (required for STOP2) */
  HAL_UART_DeInit(&huart1);

  /* SP-09 (#249): HAL_UART_MspDeInit de-configures PB6/PB7 - re-assert the
   * GNSS driver's sleep state after the DeInit. The old blanket ANALOG here
   * undid EnterStandby's anti-parasitic PB6-LOW every single sleep (while a
   * comment below told other code 'DO NOT override' GPS pins). */
  GNSS_UARTPins_SleepSafe();
  
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
   * Solution: Use RTC Wakeup Timer to wake every IWDG_SAFE_SLEEP_SECONDS
   *   (20 s, F-011/#210), refresh IWDG,
   *   then re-enter STOP2. LoRaWAN RTC Alarm A also wakes us for real events.
   * Exit conditions: LoRaWAN alarm fires, or any non-wakeup-timer interrupt. */

  /* Enable NVIC for RTC Wakeup Timer (uses EXTI line 19 internally) */
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

  /* FW-4: if the TxTimer/Alarm-A chain dies while the RTC wakeup timer keeps
   * ticking, this loop would sleep in 20s chunks forever with a satisfied
   * IWDG. Two guards: (1) run the progress deadman here — it reads RTC time
   * and self-resets; (2) bound the chunk count past the worst-case cycle
   * (MAX_SLEEP_CHUNKS, derived from the maximum validated TX interval -
   * F-7/#182; was the hardcoded 180 that only covered 3620 s).
   * #134: the bound tracks IWDG_SAFE_SLEEP_SECONDS — 150 chunks x 20s = 50 min
   * aborted every SURVIVAL sleep ~10 min early (R2-09 shrank the chunk, not
   * the count), paying a full PWR_ExitStopMode()/re-enter cycle per hour. */
  extern void Deadman_Check(void);  /* defined in lora_app.c */
  uint32_t chunks = 0;

  /* [DIAG] radio state at sleep entry: RFBUSYS was set in PWR_SR2 at every
   * spurious wake - confirm the chip mode the stack left the radio in. */
  {
    RadioPhyStatus_t rstat = SUBGRF_GetStatus();
    SONDE_LOG("LPE: enter sleep sr2=0x%04lX chipmode=%d cmdstat=%d\r\n",
              (unsigned long)(PWR->SR2 & 0xFFFFUL),
              (int)rstat.Fields.ChipMode, (int)rstat.Fields.CmdStatus);
  }
  while (1)
  {
    /* Set RTC Wakeup Timer: RTCCLK/16 = 2048 Hz,
     * IWDG_WAKEUP_COUNTS = IWDG_SAFE_SLEEP_SECONDS x 2048 (F-011/#210) */
    /* 4th param: WakeUpAutoClr = 0 (no auto-clear, we clear manually) */
    /* F-09 (#76): if the WUT fails to arm, entering STOP2 here would sleep
     * with NO scheduled wake. Fall back to a bounded busy-wait (with IWDG
     * refresh) and exit chunked sleep instead. */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, IWDG_WAKEUP_COUNTS,
                                     RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0) != HAL_OK)
    {
      /* Bounded busy-wait fallback: one chunk's worth of 1 s IWDG-refreshed
       * delays (F-011/#210: was a hardcoded 25 s vs the 20 s chunk). */
      for (int i = 0; i < IWDG_SAFE_SLEEP_SECONDS; i++)
      {
        HAL_Delay(1000);
        HAL_IWDG_Refresh(&hiwdg);
      }
      break;
    }

    /* Clear wakeup flags before sleeping */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    /* Clear any latched SysTick pend before WFI. SysTick is the live 1 ms
     * HAL timebase (HAL_InitTick is not overridden) and wraps during the
     * PRIMASK-masked sleep-entry path, latching PENDSTSET. HAL_SuspendTick()
     * clears TICKINT but NOT the pend, and WFI wakes on a pending exception
     * even with PRIMASK set -> instant wake with no RTC flags (LPC: other),
     * a ~50 ms full exit/re-enter spin where the MCU never actually sleeps. */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    /* RTC_WKUP NVIC pend must be re-cleared HERE, not right after the wake:
     * WUTF is still set at that point (it deasserts ~2 RTCCLK cycles after
     * DeactivateWakeUpTimer writes SCR), so the pend re-asserts instantly and
     * the next WFI returns in the SAME millisecond - the flag-less "LPC: other"
     * every second chunk. The WUTWF poll inside HAL_RTCEx_SetWakeUpTimer_IT
     * guarantees WUTF is quiet by this line. */
    NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
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

    SONDE_LOG_STR("LPC: up\r\n");  /* [DIAG] */

    /* F-08 (#76): deactivate the wakeup timer BEFORE any exit path. The
     * chunk-overflow break used to skip this, leaving the WUT armed. */
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    if (++chunks > MAX_SLEEP_CHUNKS) { SONDE_LOG_STR("LPC: bound\r\n"); break; }  /* FW-4: never sleep forever; F-7 (#182) [DIAG print] */

    /* Alarm A (LoRaWAN timer event) wins over the IWDG chunk timer. */
    if (is_alarm_a)
    {
      /* LoRaWAN timer event — exit chunked sleep. WUTF PR already consumed. */
      SONDE_LOG_STR("LPC: alarm\r\n");  /* [DIAG] */
      break;
    }

    if (is_wakeup_timer)
    {
      /* Only our IWDG refresh timer fired — PR consumed by handler,
       * re-enter STOP2 for the next chunk. */
      SONDE_LOG("LPC: wut t=%lu\r\n", (unsigned long)TIMER_IF_GetTimerValue());  /* [DIAG] */
      continue;
    }

    /* Something else woke us (GPIO interrupt, etc.) — exit */
    /* [DIAG] wake-source dump, fault-safe: a raw RTC->SR read here hard-
     * faulted twice (IWDG recovered, no LPC prints). Read the HAL-shadowed
     * SR via HAL_RTCEx_GetWakeUpTimer (== WUTR content) plus EXTI/NVIC
     * pending; WUTF-in-SR is mirrored by the WUTR-then-SR sequence the HAL
     * uses, so this cannot touch a volatile RTC register directly. */
    uint32_t wutr = HAL_RTCEx_GetWakeUpTimer(&hrtc);
    SONDE_LOG("LPC: other t=%lu I=%08lX S=%08lX\r\n",
              (unsigned long)TIMER_IF_GetTimerValue(),
              (unsigned long)SCB->ICSR, (unsigned long)SysTick->CTRL);
    SONDE_LOG("LPCD: N0=%08lX N1=%08lX P1=%08lX\r\n",
              (unsigned long)NVIC->ISPR[0], (unsigned long)NVIC->ISPR[1],
              (unsigned long)EXTI->PR1);
    SONDE_LOG("LPCD: P2=%08lX WUTR=%lu\r\n",
              (unsigned long)EXTI->PR2, (unsigned long)wutr);
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
   * In FLIGHT the LED costs power on every IWDG_SAFE_SLEEP_SECONDS (20 s)
   * wake for zero benefit. (F-011/#210) */
  if (MissionState_IsCommissioning()) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  }

  /* [DIAG 2026-08-18] post-STOP2 hang bisection breadcrumbs - REMOVE AFTER DEBUG */
  SONDE_LOG_STR("LPX: wake\r\n");
  
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
  /* LT-05 (#275): the F-029 counting/logging never reached the capability
   * mask - telemetry kept claiming healthy hardware over a dead peripheral
   * (#255 fixed the once-at-boot I2C2 twin). Each failing class now marks its
   * capability: I2C2 -> SYS_CAP_SENSORS, SPI2/W25Q -> SYS_CAP_FLASH,
   * UART1 -> SYS_CAP_GNSS. Debounced per class: the capability is marked
   * only after STOP2_REINIT_CAP_FAIL_THRESHOLD CONSECUTIVE failing wakes of
   * that class (a debounce choice, not a measurement - one transient must
   * not condemn a peripheral, a persistent failure must not go unreported);
   * a successful re-init resets that class's streak. */
  #define STOP2_REINIT_CAP_FAIL_THRESHOLD  3
  static uint32_t stop2_reinit_fail_count = 0;
  static uint8_t stop2_sensors_fail_streak = 0;  /* I2C2 */
  static uint8_t stop2_flash_fail_streak = 0;    /* SPI2 + W25Q wake */
  static uint8_t stop2_gnss_fail_streak = 0;     /* UART1 */
  bool reinit_failed = false;

  /* Re-enable peripheral clocks that were disabled for STOP2 */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  /* Re-initialize DMA before peripherals that use it (UART) */
  MX_DMA_Init();
  SONDE_LOG_STR("LPX: dma\r\n");  /* [DIAG] */

  /* Re-initialize I2C2 - sensors need this to work */
  HAL_I2C_DeInit(&hi2c2);
  SONDE_LOG_STR("LPX: i2c deinit\r\n");  /* [DIAG] */
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    SONDE_LOG_STR("STOP2 REINIT FAIL: I2C2\r\n");
    reinit_failed = true;
    if (++stop2_sensors_fail_streak >= STOP2_REINIT_CAP_FAIL_THRESHOLD) {
      SysCaps_MarkFailed(SYS_CAP_SENSORS);  /* LT-05 (#275) */
    }
  } else {
    stop2_sensors_fail_streak = 0;  /* LT-05: healthy re-init clears the streak */
    /* S-09 (#233): re-apply the filter config from MX_I2C2_Init (same calls
     * as the bus-recovery path in sys_sensors.c). */
    HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
  }
  SONDE_LOG_STR("LPX: i2c done\r\n");  /* [DIAG] */

  /* Re-initialize SPI2 - external flash needs this */
  bool flash_class_ok = true;
  HAL_SPI_DeInit(&hspi2);
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    SONDE_LOG_STR("STOP2 REINIT FAIL: SPI2 (flash archive at risk)\r\n");
    reinit_failed = true;
    flash_class_ok = false;
  }
  SONDE_LOG_STR("LPX: spi\r\n");  /* [DIAG] */

  /* FW-14: wake the flash from deep power-down. tRES1 = 3us max per
   * datasheet; W25Q_ReleasePowerDown already delays 1ms. */
  if (hw25q.initialized) {
    if (W25Q_ReleasePowerDown(&hw25q) != W25Q_OK) {
      SONDE_LOG_STR("STOP2 REINIT FAIL: W25Q wake\r\n");
      reinit_failed = true;
      flash_class_ok = false;
    }
  }
  SONDE_LOG_STR("LPX: w25q\r\n");  /* [DIAG] */
  /* LT-05 (#275): SPI2 and W25Q are one capability class (the archive). */
  if (flash_class_ok) {
    stop2_flash_fail_streak = 0;
  } else if (++stop2_flash_fail_streak >= STOP2_REINIT_CAP_FAIL_THRESHOLD) {
    SysCaps_MarkFailed(SYS_CAP_FLASH);
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
      if (++stop2_gnss_fail_streak >= STOP2_REINIT_CAP_FAIL_THRESHOLD) {
        SysCaps_MarkFailed(SYS_CAP_GNSS);  /* LT-05 (#275) */
      }
    } else {
      stop2_gnss_fail_streak = 0;  /* LT-05: healthy re-init clears the streak */
    }
  }

  if (reinit_failed) {
    stop2_reinit_fail_count++;
    /* LT-05 (#275): the summary line now carries the capability mask. */
    SONDE_LOG("STOP2 reinit failures this boot: %lu (caps failed-mask=0x%02X)\r\n",
                      (unsigned long)stop2_reinit_fail_count, (unsigned)SysCaps_Raw());
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
  SONDE_LOG_STR("LPX: exit done\r\n");  /* [DIAG] */
  /* USER CODE BEGIN ExitStopMode_2 */

  /* USER CODE END ExitStopMode_2 */
}

void PWR_EnterSleepMode(void)
{
  /* USER CODE BEGIN EnterSleepMode_1 */

  /* USER CODE END EnterSleepMode_1 */
  /* Suspend sysTick */
  HAL_SuspendTick();
  /* Clear any latched SysTick pend: WFI wakes on a pending exception even
   * with PRIMASK set (see the PWR_EnterStopMode chunk-loop note). */
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
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
