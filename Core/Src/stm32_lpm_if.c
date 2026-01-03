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
#include "atgm336h.h"  // For GNSS_HandleTypeDef and power state check
#include "../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.h"  // For TCXO control
#include "w25q16jv.h"  // For external flash deep power-down
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern SUBGHZ_HandleTypeDef hsubghz;
extern ADC_HandleTypeDef hadc;
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Static flash handle for deep power-down control */
/* NOTE: If you use flash logging, make this extern and define it in your main code */
static W25Q_HandleTypeDef hw25q_local = {0};
static W25Q_HandleTypeDef *hw25q_ptr = NULL;  /* Set this to your global flash handle if available */
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
  /* W25Q16JV draws 1-3mA in standby, <1µA in deep power-down */
  /* This is the PRIMARY fix for 2-4mA sleep current! */
  //if (hw25q_ptr != NULL && hw25q_ptr->initialized) {
  //  W25Q_PowerDown(hw25q_ptr);  /* Send 0xB9 command - flash enters deep sleep */
  //}
  /* NOTE: If you haven't initialized flash yet, this won't execute */
  /* To use: Set hw25q_ptr = &your_flash_handle after W25Q_Init() */
  
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
  
  GPIO_InitStruct.Pin = GPIO_PIN_4;  // PB4 = ADC_IN3 (Battery voltage)
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* === STOP2 Power Optimization: Disable Peripheral Clocks === */
  /* Reduces ~30-60µA by gating unused peripheral clocks */
  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_DMAMUX1_CLK_DISABLE();
  
  /* === STOP2 Power Optimization: Disable VREFINT === */
  /* Internal voltage reference consumes ~10-20µA when enabled */
  HAL_SYSCFG_DisableVREFBUF();
  
  /* === Additional GPIO Power Optimization === */
  /* Set all unused/inactive pins to ANALOG mode to minimize leakage */

  /* GPS pins are managed by GNSS driver - DO NOT override here! */
  /* GNSS_EnterStandby() sets: PB6=OUTPUT-LOW, PB7=ANALOG, PB5=LOW, PB10=HIGH */
  /* PB10 must stay HIGH for hot-start mode (~15µA backup power) */
  
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

  /* USER CODE END EnterStopMode_2 */
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
  /* USER CODE BEGIN EnterStopMode_3 */

  /* USER CODE END EnterStopMode_3 */
}

void PWR_ExitStopMode(void)
{
  /* USER CODE BEGIN ExitStopMode_1 */
  
  /* === DIAGNOSTIC: LED ON while awake === */
  /* PA0 HIGH = MCU awake and running */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  
  /* === PERIPHERAL RE-INITIALIZATION AFTER STOP2 === */
  /* STM32WL loses peripheral configuration in STOP2 mode */
  /* Must restore in proper dependency order */
  
  /* NOTE: SystemClock_Config() REMOVED - STM32WL auto-restores clock after STOP2 */
  /* Calling it was causing 30mA power draw issue */
  /* System wakes on MSI (4MHz) which is sufficient, radio driver handles PLL if needed */
  
  /* Re-enable peripheral clocks that were disabled for STOP2 */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  
  /* Re-initialize DMA before peripherals that use it (UART) */
  MX_DMA_Init();
  
  /* Re-initialize I2C2 - sensors need this to work */
  HAL_I2C_DeInit(&hi2c2);
  HAL_I2C_Init(&hi2c2);
  
  /* Re-initialize SPI2 - external flash needs this */
  HAL_SPI_DeInit(&hspi2);
  HAL_SPI_Init(&hspi2);
  
  /* Re-initialize UART1 only if GPS is powered */
  /* Prevents parasitic power when GPS is off */
  extern GNSS_HandleTypeDef hgnss;
  if (hgnss.is_powered) {
    HAL_UART_DeInit(&huart1);
    HAL_UART_Init(&huart1);
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
