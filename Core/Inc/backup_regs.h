/**
  ******************************************************************************
  * @file    backup_regs.h
  * @brief   RTC backup register allocation map (R01/R02 fix)
  ******************************************************************************
  * Single source of truth for who owns which RTC backup register. Backup
  * registers survive any reset that keeps VBAT/backup domain alive, so two
  * owners of the same register silently corrupt each other (R01/R02).
  *
  * DR0-DR2 are RESERVED for the ST timer_if / SysTime layer
  * (Core/Src/timer_if.c: RTC_BKP_SECONDS / RTC_BKP_SUBSECONDS /
  * RTC_BKP_MSBTICKS). Application code must NEVER touch DR0-DR2: SysTimeSet()
  * rewrites them on every GPS time discipline, which previously clobbered
  * the mission state, the fault breadcrumb, and the deadman timestamp.
  *
  * Application-owned registers start at DR3:
  *   DR3  mission_state.c   mission state record (magic | MissionState_t)
  *   DR4  reset_cause.c     fault breadcrumb (RESET_CAUSE_FAULT_MAGIC | code)
  *   DR5  lora_app.c        deadman last-progress timestamp (RTC seconds)
  *
  * RULE: any new backup-register user must add its allocation here first.
  ******************************************************************************
  */

#ifndef __BACKUP_REGS_H
#define __BACKUP_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wlxx_hal.h"

/* DR0-DR2: owned by timer_if.c (ST SysTime). Do not use. */
#define BKP_REG_RESERVED_SYSTIME_SECONDS     RTC_BKP_DR0
#define BKP_REG_RESERVED_SYSTIME_SUBSECONDS  RTC_BKP_DR1
#define BKP_REG_RESERVED_SYSTIME_MSBTICKS    RTC_BKP_DR2

/* Application-owned backup registers */
#define BKP_REG_MISSION_STATE      RTC_BKP_DR3   /* mission_state.c */
#define BKP_REG_RESET_CAUSE_FAULT  RTC_BKP_DR4   /* reset_cause.c breadcrumb */
#define BKP_REG_DEADMAN            RTC_BKP_DR5   /* lora_app.c deadman mark */

#ifdef __cplusplus
}
#endif

#endif /* __BACKUP_REGS_H */
