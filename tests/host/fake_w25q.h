/* Host-test simulated W25Q16JV NOR flash — see fake_w25q.c */
#ifndef FAKE_W25Q_H
#define FAKE_W25Q_H

#include <stdint.h>
#include <stdbool.h>
#include "w25q16jv.h"   /* stubs/w25q16jv.h: types + geometry */

void     fake_w25q_init(void);
void     fake_w25q_free(void);
void     fake_w25q_corrupt(uint32_t addr, uint32_t len);
void     fake_w25q_fail_next_reads(int n);
uint8_t  fake_w25q_peek(uint32_t addr);
void     fake_w25q_poke(uint32_t addr, const void *data, uint32_t len);  /* raw placement write (fault injection) */
bool     fake_w25q_is_erased(uint32_t addr, uint32_t len);

extern uint32_t fake_w25q_erase_count;
extern uint32_t fake_w25q_write_count;
extern uint32_t fake_w25q_read_count;

#endif /* FAKE_W25Q_H */
