/* Arming input (PRETEST-DEC-01, #142): the PB13 commissioning button.
 * The debounce policy is pure and host-testable; the GPIO dance is a thin
 * target shim. */
#ifndef ARMING_INPUT_H
#define ARMING_INPUT_H

#include <stdbool.h>
#include <stdint.h>

/** Consecutive pressed commissioning-wake samples required to arm. */
#define ARMING_CONFIRM_WAKES 2U

/** Pure debounce: a confirmed arming needs confirm_wakes CONSECUTIVE pressed
 *  samples; any released sample resets the streak. */
bool ArmingInput_Debounce(uint8_t *streak, bool pressed, uint8_t confirm_wakes);

/** Poll the arming button. Commissioning-only: samples PB13 (shared with
 *  SPI2_SCK) while the flash is guaranteed idle, and arms the mission on a
 *  confirmed press. Safe no-op outside commissioning. */
void ArmingInput_Poll(void);

#endif /* ARMING_INPUT_H */
