/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "app_lorawan.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "stm32wlxx_hal_pwr.h"
#include "SEGGER_RTT.h"
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */
#include "atgm336h.h"
#include "sys_sensors.h"
#include "sht31.h"
#include "ms5607.h"
#include "LmHandler.h"
#include "LmHandlerTypes.h"
#include "../../Utilities/lpm/tiny_lpm/stm32_lpm.h"
#include "h3lite.h"
#include "multiregion_h3.h"
#include "w25q16jv.h"
#include "flash_log.h"
#include "payload_format.h"
#include "config.h"
#include "reset_cause.h"
#include "timer_if.h"  /* RV-09 (#165): TIMER_IF_Init in the deferred failover */
#include "mission_state.h"
#include "../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.h"  // For SUBGRF TCXO control
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* F-001 fatal fault codes (breadcrumb low 16 bits). 0-4 = CPU fault handlers
 * (stm32wlxx_it.c; 0 = NMI — MAGIC|0 reads back as the bare magic, harmless
 * given the mask check), 6 = deadman (lora_app.c); 16+ = boot-time fatal
 * errors. */
#define FAULT_CODE_CLOCK_CONFIG    16U
#define FAULT_CODE_PAYLOAD_FORMAT  17U
#define FAULT_CODE_FLASH_INIT      18U  /* R29 (#36): W25Q/archive unusable */
#define FAULT_CODE_WATCHDOG_INIT   19U  /* F6 (#171): IWDG init failed - no supervision */
#define FAULT_CODE_RTC_INIT        20U  /* F6 (#171): RTC init failed - no timebase */
#define FAULT_CODE_RTC_STALLED     21U  /* F4 (#170): RTC stalled on LSI post-failover */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

SUBGHZ_HandleTypeDef hsubghz;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* Flash logging handles */
W25Q_HandleTypeDef hw25q;
FlashLog_HandleTypeDef hflashlog;

/* USER CODE BEGIN PV */
/* Note: RTT Virtual Terminal architecture:
 * Channel 0 with virtual terminals (via 0xFF escape sequences):
 *   - Virtual Terminal 0: System messages, sensors (default)
 *   - Virtual Terminal 1: NMEA/GNSS sentences (via TerminalOut)
 *   - Virtual Terminal 2: APP_LOG (LoRaWAN middleware)
 * Requires advanced RTT viewer (J-Link RTT Viewer, Ozone, etc.) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
static void MX_IWDG_Init(void);
static void LSE_FailoverToLSI(void);   /* F-14 (#70) */
static void RTC_LivenessCheck(void);   /* F-14 (#70) */
/* RV-09 (#165): the CSS interrupt only flags; the failover runs in the main
 * loop. RCC reconfiguration + RTC re-init inside an ISR races any main-loop
 * RTC access. */
static volatile bool s_lse_fail_pending = false;
/* USER CODE BEGIN PFP */
void leds_boot_seq(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* F23 FIX: TEST_UltraMinimal_STOP2() deleted. It entered STOP2 with no wake
 * source ("will BRICK") and was one uncomment away from the boot path.
 * A function that bricks the device must not exist in a flight binary. */

/* system_sleep() deleted (finding #9, 2026-08-10): dead code — never called,
 * and it deinited I2C/ADC/UART. A function with side effects like that must
 * not sit one uncomment away from the boot path (same rule as F23). */

void leds_boot_seq(void)
{
  /* R09/DDR-0002: dark in flight — the boot blink is a bench/commissioning
   * signal only; a flight unit rebooting at altitude stays dark. */
  if (!MissionState_IsCommissioning()) {
    return;
  }
  /* Use the actual LED on PA0 for boot sequence */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
  HAL_Delay(500);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Configure RTT Terminal 0 for all debug output */
  /* Everything goes to Terminal 0 for simple viewer compatibility */
  /* Increased buffer size and non-blocking mode to prevent watchdog hangs */
  static char rtt_buffer[4096];  // 4KB buffer (up from default 1KB)
  SEGGER_RTT_ConfigUpBuffer(0, "Terminal", rtt_buffer, sizeof(rtt_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  SONDE_LOG_STR("=== RTT Terminal 0 Configured (4KB, NON-BLOCKING) ===\r\n");
  SONDE_LOG_STR("All output: System, NMEA, APP_LOG\r\n");

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* FW-5 / FR-01 (#83): arm the IWDG BEFORE clock config. HAL_RCC_OscConfig()
   * waits for the LSE with an HAL_GetTick()-based timeout, but the tick
   * override (sys_app.c) returns a constant 0 until SYS_TimerInitialisedFlag
   * is set inside MX_LoRaWAN_Init() — so a dead LSE hangs that loop forever.
   * Arming the watchdog first (HAL_IWDG_Init starts the LSI itself via the
   * key register, no clock-config dependency) turns that silent brick into
   * a ~33 s reset-and-recover. The join-wait loop already refreshes via its
   * hiwdg.Instance guard. */
  MX_IWDG_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* F13b: Capture condensed reset cause (RCC->CSR + fault breadcrumb) once,
   * early, then clear flags so the next boot reads clean. Surfaced in the
   * uplink status byte (DDR-0003). */
  ResetCause_CaptureBoot();

  /* USER CODE BEGIN SysInit */

  /* NOTE: IWDG watchdog armed above, before SystemClock_Config (FW-5 / FR-01) */
  SONDE_LOG_STR("IWDG watchdog armed (32.76s timeout)\r\n");
  
  /* CRITICAL: Initialize DMA and I2C2 BEFORE LoRaWAN_Init 
   * LoRaWAN_Init -> SystemApp_Init -> EnvSensors_Init (needs I2C2)
   * vcom_Init (inside SystemApp_Init) needs DMA for UART */
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LoRaWAN_Init();  /* Note: MissionState_Init() runs inside, after MultiRegion_Init */
  MX_SPI2_Init();
  /* MX_I2C2_Init(); - Already called in SysInit above */
  /* MX_IWDG_Init(); - FW-5/FR-01: moved up, immediately after HAL_Init() */
  /* USER CODE BEGIN 2 */
  /* Explicitly initialize RTT BEFORE J-Link connects to ensure control block is findable */

  
  leds_boot_seq();
  
  SONDE_LOG_STR("Boot sequence complete, initializing H3Lite...\r\n");
  
  // Initialize h3lite for region detection
  if (!h3liteInit()) {
    SONDE_LOG_STR("ERROR: H3Lite initialization failed!\r\n");
    Error_Handler();  /* F-001: recoverable — returns */
    /* F-001 residual: no success print on this path — failure stays honest. */
  } else {
    SONDE_LOG_STR("H3Lite initialized successfully\r\n");
  }
  
  // Initialize external flash (W25Q16JV) for logging
  /* F-03 (#65): retry then fly degraded. A cold-stuck flash chip must not
   * reset-loop the sonde at altitude — a missing archive is recoverable,
   * a dead radio is total mission loss. This REVERSES the R29 (#36) "never
   * fly with a dead data store" policy by maintainer decision (see #65). */
  SONDE_LOG_STR("Initializing external flash (W25Q16JV)...\r\n");
  W25Q_StatusTypeDef w25q_status = W25Q_ERROR;
  for (int flash_attempt = 0; flash_attempt < 3; flash_attempt++) {
    w25q_status = W25Q_Init(&hw25q, &hspi2, GPIOB, GPIO_PIN_9);  // Use software CS on PB9
    if (w25q_status == W25Q_OK) break;
    SONDE_LOG("W25Q16JV init attempt %d failed (status %d)\r\n", flash_attempt + 1, w25q_status);
    HAL_Delay(100);
  }
  if (w25q_status == W25Q_OK) {
    uint32_t jedec_id;
    if (W25Q_ReadJEDECID(&hw25q, &jedec_id) == W25Q_OK) {
      SONDE_LOG("W25Q16JV initialized successfully (JEDEC ID: 0x%06lX)\r\n", jedec_id);
    } else {
      SONDE_LOG_STR("W25Q16JV initialized but JEDEC ID read failed\r\n");
    }
  } else {
    /* Degraded: no archive this flight, but the radio flies. Every FlashLog_*
     * API guards !hlog->initialized, so downstream code is safe. */
    SONDE_LOG("ERROR: W25Q16JV init failed after 3 attempts (status %d) — flying WITHOUT archive\r\n", w25q_status);
  }
  
  // Initialize flash logging system
  /* F-001 residual: don't init the log on a dead W25Q handle — a "success"
   * here would be a false success print. Failure stays honest (DDR-0003);
   * logging is recoverable-degraded per DDR-0009. */
  if (w25q_status != W25Q_OK) {
    SONDE_LOG_STR("Flash logging disabled: W25Q init failed\r\n");
  } else {
    SONDE_LOG_STR("Initializing flash logging system...\r\n");
    FlashLog_StatusTypeDef flashlog_status = FlashLog_Init(&hflashlog, &hw25q);
    if (flashlog_status == FLASH_LOG_OK) {
      uint32_t total_capacity, used_records, free_records;
      FlashLog_GetStats(&hflashlog, &total_capacity, &used_records, &free_records);
      SONDE_LOG("Flash logging initialized: %lu/%lu records used (%lu free)\r\n",
                        used_records, total_capacity, free_records);
    } else {
      SONDE_LOG("ERROR: Flash logging initialization failed (status: %d) — flying WITHOUT archive\r\n", flashlog_status);
      /* F-03 (#65): degrade, not reset. hflashlog.initialized stays false;
       * all FlashLog_* APIs guard on it, so the TX path runs normally. */
    }
  }
  
  // Validate payload format sizes at compile time
  SONDE_LOG_STR("Validating payload format sizes...\r\n");
  if (!PayloadFormat_ValidateSizes()) {
    /* F-001: a size/layout mismatch is a BUILD bug — every uplink would be
     * malformed. Breadcrumb + reset so the failure is observable, not silent. */
    SONDE_LOG_STR("ERROR: Payload format size validation failed!\r\n");
    Error_Handler_Fatal(FAULT_CODE_PAYLOAD_FORMAT);
  }
  
  // Initialize configuration system
  SONDE_LOG_STR("Initializing configuration system...\r\n");
  ConfigStatus_t config_status = Config_Init();
  if (config_status == CONFIG_OK) {
    SONDE_LOG_STR("Configuration system initialized successfully\r\n");
    
    // Print current configuration for verification
    Config_PrintCurrent();
  } else {
    SONDE_LOG("WARNING: Configuration initialization failed (status: %d)\r\n", config_status);
    SONDE_LOG_STR("Continuing with hardcoded defaults...\r\n");
  }
  
  /* #77: H3Lite bench profiler deleted (never enabled in flight; git history
   * retains it if a bench re-run is ever needed). */

  SONDE_LOG_STR("Starting LoRaWAN...\r\n");

  /* system_sleep() was deleted outright (finding #9) — it was never called
   * and deinited I2C/ADC/UART. I2C stays active for SHT31/MS5607. */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  SONDE_LOG_STR("\r\n===== STARTING LORAWAN OPERATION =====\r\n");
  SONDE_LOG_STR("Main loop: Continuous LoRaWAN servicing\r\n");
  SONDE_LOG_STR("Join will happen in background via TxTimer\r\n\r\n");
  
  while (1)
  {
    /* USER CODE END WHILE */
    MX_LoRaWAN_Process();

    /* USER CODE BEGIN 3 */
    /* F13a (DDR-0020): progress deadman — if no work cycle has started for
     * 3x the worst-case interval in FLIGHT, breadcrumb + reset. No-op in
     * COMMISSIONING. Defined in lora_app.c. */
    extern void Deadman_Check(void);
    Deadman_Check();

    /* RV-09 (#165): deferred LSE failover — main-loop context, never ISR */
    if (s_lse_fail_pending) {
      s_lse_fail_pending = false;
      LSE_FailoverToLSI();
    }

    RTC_LivenessCheck();  /* F-14 (#70): frozen time must not look alive */

    /* Refresh watchdog to prevent reset (must be called within 32.76 seconds) */
    HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */
}

/* R08: RTC clock source actually in use. SystemClock_Config() may fail over
 * LSE -> LSI when the crystal is dead; the CubeMX-generated HAL_RTC_MspInit()
 * would otherwise force LSE again via HAL_RCCEx_PeriphCLKConfig(), which
 * resets the backup domain (wiping ALL backup registers) and re-selects the
 * dead oscillator. HAL_RTC_MspInit() honors this variable instead. */
uint32_t g_rtc_clock_source = RCC_RTCCLKSOURCE_LSE;

/* F12 (#173): build-configuration marker embedded in the binary — CI (and
 * anyone with the .bin) can prove which configuration was actually compiled
 * instead of trusting the build script. */
#ifdef SONDE_FLIGHT_BUILD
volatile const char g_sonde_build_marker[] = "SONDE_BUILD:flight";
#else
volatile const char g_sonde_build_marker[] = "SONDE_BUILD:debug";
#endif

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* F3 FIX (DDR-0009): a dead/frozen LSE crystal must not be a reboot loop.
     * Fail over to LSI for the RTC clock and keep flying — timers drift by
     * ~1% instead of the mission ending. The event is observable via the
     * low-power/fault telemetry path at next uplink. */
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      /* F-001: neither LSE nor LSI can be started — no clock tree at all.
       * Continuing is meaningless; breadcrumb + reset. */
      Error_Handler_Fatal(FAULT_CODE_CLOCK_CONFIG);
    }
    /* Switch RTC clock source LSE -> LSI (R08: recorded so HAL_RTC_MspInit
     * does not force LSE again and wipe the backup domain) */
    RCC_PeriphCLKInitTypeDef rtcClk = {0};
    rtcClk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    rtcClk.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    HAL_RCCEx_PeriphCLKConfig(&rtcClk);
    g_rtc_clock_source = RCC_RTCCLKSOURCE_LSI;
    SONDE_LOG_STR("WARNING: LSE failed - RTC on LSI (~1% drift)\r\n");
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    /* F-001: CPU/bus clocks not running at the configured rates — fatal. */
    Error_Handler_Fatal(FAULT_CODE_CLOCK_CONFIG);
  }

  /* F-14 (#70): runtime LSE clock security. The boot-time fallback above only
   * covers an LSE that never starts; a crystal that DIES in flight (extreme
   * cold) freezes the RTC and every LoRaMac timer while the main loop keeps
   * petting the IWDG — alive-looking but time-frozen. Arm the CSS so hardware
   * detects the failure and vectors to HAL_RCCEx_LSECSS_Callback() below. */
  if (g_rtc_clock_source == RCC_RTCCLKSOURCE_LSE)
  {
    HAL_RCCEx_EnableLSECSS_IT();
  }
}

/* F-14 (#70): LSE->LSI failover, shared by the CSS interrupt callback and the
 * RTC liveness check. Changing the RTC clock source resets the backup domain
 * (HAL behaviour) — this DESTROYS: DR3 (mission state), DR4 (fault
 * breadcrumb), DR5 (deadman), DR7 (SysTime-valid marker) and DR8–DR11
 * (last-known position, F-15/#72). Callers must be prepared for that loss;
 * the door anchor in MissionState_Init() survives a DR3 wipe via the Tier-1
 * bank check and is the designed blast-radius limiter. The loss is
 * acceptable: the alternative is a sonde with frozen time. The IWDG
 * (LSI-clocked) and the boot-time fallback are the backstops for anything
 * this path gets wrong. */
static void LSE_FailoverToLSI(void)
{
  RCC_PeriphCLKInitTypeDef rtcClk = {0};
  rtcClk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  rtcClk.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  /* F4 (#170): consume the HAL results — a failed source switch must be
   * visible, and the (now source-agnostic) liveness check will catch a
   * non-advancing RTC and escalate to a controlled reset. */
  HAL_StatusTypeDef ck_status = HAL_RCCEx_PeriphCLKConfig(&rtcClk);
  if (ck_status == HAL_OK) {
    g_rtc_clock_source = RCC_RTCCLKSOURCE_LSI;
  } else {
    SONDE_LOG_STR("ERROR: LSI switch FAILED - RTC source unchanged\r\n");
    return;   /* stay on the (failing) LSE; liveness will retry/escalate */
  }
  MX_RTC_Init();  /* backup domain was reset by the source change — re-init */
  /* RV-09 (#165): bare MX_RTC_Init left the timer subsystem inconsistent —
   * Alarm A re-armed with AlarmMask=NONE, RtcTimerContext stale against a
   * restarted counter (alarms scheduled from it might never fire). Run the
   * full timer post-init (DeactivateAlarm, bypass shadow, timer context,
   * MSB-tick markers), same as the boot path. */
  if (TIMER_IF_Init() != UTIL_TIMER_OK) {
    SONDE_LOG_STR("ERROR: TIMER_IF_Init failed after LSI failover\r\n");
  }
  SONDE_LOG_STR("WARNING: LSE died in flight - RTC on LSI (~1% drift)\r\n");
}

/* F-14 (#70): LSE CSS interrupt callback (fires from TAMP_STAMP_LSECSS_SSRU_IRQn).
 * RV-09 (#165): ISR context — flag only, the main loop executes the failover. */
void HAL_RCCEx_LSECSS_Callback(void)
{
  s_lse_fail_pending = true;
}

/* F-14 (#70) / FR-02 (#86): second layer — verify the RTC actually advances
 * against a clock-independent reference. Covers failure modes the CSS misses
 * (RTC path broken with LSE still oscillating). Call from the main loop.
 *
 * FR-02 rebuild: the previous version read HAL_RTC_GetTime() into an
 * uninitialized RTC_TimeTypeDef — but with BinMode = RTC_BINARY_ONLY the HAL
 * writes only SubSeconds, so Hours/Minutes/Seconds were never written. The
 * constant stack garbage looked like a frozen RTC, stall_count hit 5, and
 * LSE_FailoverToLSI() wiped the backup domain ~5 s into every boot. Its rate
 * limiter was also self-referential (HAL_GetTick() is RTC-derived — a frozen
 * RTC would freeze the reference too).
 *
 * Now: read RTC->SSR directly (the actual timebase in binary-only mode — a
 * down-counter at RTCCLK/(PREDIV_A+1) = 1024 Hz) and rate-limit on uwTick,
 * the MSI-clocked HAL tick counter. HAL_GetTick() cannot be used (RTC-derived);
 * uwTick is genuinely independent in run mode and stops in STOP2, so after a
 * sleep the RTC appears to run FAST, never slow — this can only under-trigger,
 * never false-fire. A healthy SSR changes within ~1 ms; an identical value
 * across 3 full seconds of MSI time is a genuine stall. */
static void RTC_LivenessCheck(void)
{
  static uint32_t last_check_ms = 0;
  static uint32_t last_ssr = 0;
  static uint8_t  stall_count = 0;

  /* F4 (#170): the SSR stall check is source-agnostic — it must keep running
   * after a failover to LSI. Only the CSS arm is LSE-specific (that stays in
   * the boot path). A failover that produced a dead RTC must not also remove
   * the supervision that can see it. */

  uint32_t now_ms = uwTick;                    /* MSI-clocked, RTC-independent */
  if ((now_ms - last_check_ms) < 1000U) return;  /* check ~1/s */
  last_check_ms = now_ms;

  uint32_t ssr = READ_REG(RTC->SSR);           /* binary-mode down-counter */

  if (ssr != last_ssr)
  {
    last_ssr = ssr;
    stall_count = 0;
  }
  else if (++stall_count >= 3U)  /* no RTC advance across ~3 s of MSI time */
  {
    stall_count = 0;
    if (g_rtc_clock_source == RCC_RTCCLKSOURCE_LSE)
    {
      SONDE_LOG_STR("ERROR: RTC stalled with LSE selected - forcing LSI failover\r\n");
      LSE_FailoverToLSI();
    }
    else
    {
      /* F4 (#170): RTC stalled on LSI - the failover option is already
       * consumed; breadcrumb + reset so the boot path re-initializes
       * cleanly instead of running timeless. */
      SONDE_LOG_STR("ERROR: RTC stalled on LSI - controlled reset\r\n");
      Error_Handler_Fatal(FAULT_CODE_RTC_STALLED);
    }
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00B07CB4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;  /* Maximum prescaler for longest timeout */
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;  /* Maximum reload value */
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    /* F6 (#171): the watchdog is mission-critical supervision — continuing
     * without it means every other recovery mechanism assumes a backstop
     * that does not exist. Fatal, not degrade-and-continue. */
    Error_Handler_Fatal(FAULT_CODE_WATCHDOG_INIT);
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_PREDIV_A;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_ONLY;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    /* F6 (#171): the RTC is the mission timebase — continuing with a zombie
     * one makes every timer/timeout/silence calculation meaningless. Fatal. */
    Error_Handler_Fatal(FAULT_CODE_RTC_INIT);
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  if (HAL_RTCEx_SetSSRU_IT(&hrtc) != HAL_OK)
  {
    Error_Handler_Fatal(FAULT_CODE_RTC_INIT);
  }

  /** Enable the Alarm A
  */
  sAlarm.BinaryAutoClr = RTC_ALARMSUBSECONDBIN_AUTOCLR_NO;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDBINMASK_NONE;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */
  /* Override CubeMX generated DataSize - W25Q16 flash requires 8-bit */
  /* Software NSS required - W25Q16JV needs CS low for entire command sequences */
  /* Hardware NSS pulse mode corrupts multi-byte commands */
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;  /* Software NSS for W25Q16JV compatibility */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  /* NSSPMode removed - not applicable with software NSS */
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
void MX_SUBGHZ_Init(void)
{

  /* USER CODE BEGIN SUBGHZ_Init 0 */

  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */

  /* USER CODE END SUBGHZ_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* CRITICAL: Pre-initialize Flash CS (PB9) HIGH before SPI init */
  /* W25Q16JV must see CS HIGH during power-up to avoid undefined state */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);  /* CS HIGH = deselected */

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|RF_CTRL1_Pin|RF_CTRL2_Pin, GPIO_PIN_RESET);

  /* NOTE: PB10 and PB5 (GPS power/enable pins) are now managed by GNSS driver */
  /* Removed conflicting GPIO initialization to prevent pin toggling during sleep */

  /* F24 FIX: SOS button EXTI3 config removed. PB3 is reconfigured to ANALOG
   * for solar sensing below, so the EXTI was dead code; the associated
   * StopJoin OTAA->ABP flip would be dangerous if ever wired. */

  /*Configure GPIO pins : PA0 RF_CTRL1_Pin RF_CTRL2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|RF_CTRL1_Pin|RF_CTRL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Configure PB4 as analog input for battery voltage measurement (ADC_CHANNEL_3) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* Configure PB3 as analog input for solar voltage measurement (ADC_CHANNEL_2) */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Degrade-and-continue: log the error, never halt (DDR-0009).
   * At 40 km altitude, a hang is permanent death.
   * Do NOT disable interrupts — IWDG must remain active.
   * For truly unrecoverable errors, callers should use NVIC_SystemReset() directly. */
  SONDE_LOG_STR("ERROR_HANDLER: Non-fatal error, continuing...\r\n");
  /* USER CODE END Error_Handler_Debug */
}

/* F-001 (DDR-0009): fatal/recoverable split. Error_Handler() is for
 * degrade-and-continue faults; Error_Handler_Fatal() is for faults where
 * continuing produces a silently dead or lying unit (no clock tree, malformed
 * uplink format). It leaves a fault breadcrumb (surfaced as RESET_CAUSE_FAULT
 * in the next boot's status byte) and resets — a reset gives the IWDG/deadman
 * architecture a chance; a hang or a zombie does not.
 * Codes: 0-4 = CPU fault handlers (stm32wlxx_it.c; 0 = NMI), 6 = deadman
 * (lora_app.c), 16+ = boot-time fatal errors below. */
/* STAB-02 (#149): which boot-time fatals have a DESIGNED degraded
 * continuation? FLASH_INIT does (archive unusable -> flight continues with
 * science in RAM only, watchdog/deadman still supervising). CLOCK_CONFIG and
 * PAYLOAD_FORMAT do not: continuing would run the mission on a broken clock
 * tree or a malformed wire format — that is accidental continuation, not a
 * degraded mode. Keep this list explicit; adding a code requires a designed
 * degraded path at its call site. */
static bool FatalIsDegradable(uint16_t code)
{
  return code == FAULT_CODE_FLASH_INIT;
}

void Error_Handler_Fatal(uint16_t code)
{
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTCAPB_CLK_ENABLE();

  /* FR-23 (#104): the F-03 boot-attempts counter finally acts. A
   * DETERMINISTIC fatal condition (dead hardware, persistently bad config)
   * otherwise reset-loops forever — burning power, never recovering, and the
   * counter documented for exactly this sat inert. After 5 consecutive boots
   * without a proven work cycle, boot-time fatals (codes >= 16) stop
   * resetting and fall through to the caller's degraded path instead
   * (DDR-0009: forward progress, degrade and keep flying). CPU-fault and
   * deadman codes (<16) always reset — there is no meaningful degraded
   * continuation from a crashed context.
   *
   * STAB-02 (#149): the escape is ONLY for faults with a designed degraded
   * continuation. CLOCK_CONFIG (no RTC/clock tree) and PAYLOAD_FORMAT
   * (unvalidated wire format) have none — their callers simply fall into
   * mission code, so they always breadcrumb + reset, every boot. */
  if (code >= 16U && ResetCause_GetBootAttempts() >= 5U && FatalIsDegradable(code)) {
    SONDE_LOG("FATAL_ERROR %u: 5+ consecutive boots - DEGRADING, no reset\r\n", code);
    return;
  }

  SONDE_LOG("FATAL_ERROR %u: breadcrumb + system reset\r\n", code);
  HAL_RTCEx_BKUPWrite(&hrtc, RESET_CAUSE_BKP_FAULT_REG,
                      RESET_CAUSE_FAULT_MAGIC | (uint32_t)code);
  NVIC_SystemReset();
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
