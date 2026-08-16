#include "arming_input.h"

#include "mission_state.h" /* MissionState_IsCommissioning / EnterFlight */
#include "sonde_log.h"     /* SONDE_LOG_STR */
#include "stm32wlxx_hal.h" /* GPIOB, GPIO_PIN_13, HAL_GPIO_* */

/* Owner disposition 2026-08-16 (PRETEST-DEC-01, #142): the arming button is
 * PB13, wired to GND (active-low; the sample uses the internal pull-up).
 * PB13 is shared with SPI2_SCK (the W25Q flash clock): the button is used
 * ONLY during commissioning, and the commissioning quiet watch performs no
 * flash logging, so the clock net is idle while the pin is reconfigured,
 * sampled, and restored. Never poll this in FLIGHT. */

#define ARMING_GPIO_PORT GPIOB
#define ARMING_GPIO_PIN GPIO_PIN_13

static uint8_t s_press_streak;

bool ArmingInput_Debounce(uint8_t *streak, bool pressed, uint8_t confirm_wakes) {
  if (streak == NULL || confirm_wakes == 0U) {
    return false;
  }
  if (!pressed) {
    *streak = 0U;
    return false;
  }
  if (*streak < 0xFFU) {
    (*streak)++;
  }
  return (*streak >= confirm_wakes);
}

void ArmingInput_Poll(void) {
  if (!MissionState_IsCommissioning()) {
    s_press_streak = 0U;
    return;
  }

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = ARMING_GPIO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP; /* button to GND: pressed reads low */
  HAL_GPIO_Init(ARMING_GPIO_PORT, &gpio);

  const bool pressed =
      (HAL_GPIO_ReadPin(ARMING_GPIO_PORT, ARMING_GPIO_PIN) == GPIO_PIN_RESET);

  /* Restore the SPI2_SCK alternate function exactly as HAL_SPI_MspInit
   * configured it (stm32wlxx_hal_msp.c). */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(ARMING_GPIO_PORT, &gpio);

  if (ArmingInput_Debounce(&s_press_streak, pressed, ARMING_CONFIRM_WAKES)) {
    s_press_streak = 0U;
    SONDE_LOG_STR("ARMING: PB13 confirmed across consecutive commissioning wakes - entering FLIGHT\r\n");
    MissionState_EnterFlight();
  }
}
