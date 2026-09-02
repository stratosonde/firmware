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
 *
 * DURABILITY POLICY (H-01/#281, docs/decisions/0035-mission-manifest.md): the
 * RTC backup registers are a RESET CACHE, never the durable truth. Facts that
 * must survive a full power loss without a trusted VBAT rail live in internal
 * flash (Tier-1/Tier-2 credentials, the Mission Manifest lifecycle record) or
 * the external W25Q (science archive). Registers below are diagnostics or
 * fast-path caches; losing them costs observability, not correctness.
 ******************************************************************************
 */

#ifndef __BACKUP_REGS_H
#define __BACKUP_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wlxx_hal.h"

/* DR0-DR2: owned by timer_if.c (ST SysTime). Do not use. */
#define BKP_REG_RESERVED_SYSTIME_SECONDS RTC_BKP_DR0
#define BKP_REG_RESERVED_SYSTIME_SUBSECONDS RTC_BKP_DR1
#define BKP_REG_RESERVED_SYSTIME_MSBTICKS RTC_BKP_DR2

/* Application-owned backup registers */
#define BKP_REG_MISSION_STATE RTC_BKP_DR3     /* mission_state.c */
#define BKP_REG_RESET_CAUSE_FAULT RTC_BKP_DR4 /* reset_cause.c breadcrumb */
#define BKP_REG_DEADMAN RTC_BKP_DR5           /* lora_app.c deadman mark */
#define BKP_REG_BOOT_ATTEMPTS RTC_BKP_DR6     /* reset_cause.c consecutive boot counter (F-03/#65) */
#define BKP_REG_SYSTIME_VALID RTC_BKP_DR7     /* timer_if.c MSB-ticks validity marker (F-04/#63) */
#define BKP_REG_LASTPOS_VALID RTC_BKP_DR8     /* lora_app.c last-position magic (F-15/#72) */
#define BKP_REG_LASTPOS_LAT RTC_BKP_DR9       /* lora_app.c last valid latitude (float bits) */
#define BKP_REG_LASTPOS_LON RTC_BKP_DR10      /* lora_app.c last valid longitude (float bits) */
#define BKP_REG_LASTPOS_ALT RTC_BKP_DR11      /* lora_app.c last valid altitude (float bits) */
#define BKP_REG_LASTPOS_EPOCH RTC_BKP_DR12    /* lora_app.c last fresh-fix epoch seconds (STAB-01/#148) */
#define BKP_REG_TS_WRAP RTC_BKP_DR13          /* lora_app.c timestamp-wrap latch magic (STAB-12/#159) */
#define BKP_REG_GPS_LOSS_EPOCH RTC_BKP_DR14   /* lora_app.c GPS-loss grace epoch seconds (STAB-01/#148) */
#define BKP_REG_LAUNCH_REF RTC_BKP_DR15       /* mission_state.c launch reference: magic|hPa x10 (F1/#167) */
/* F-DIAG (boot-loop root cause): DR16-DR19 carry the fault CONTEXT captured by
 * the fault handlers in stm32wlxx_it.c immediately before reset (the existing
 * DR4 breadcrumb only stores the 0-4 fault class). On the next boot,
 * main.c prints class + stacked PC + CFSR + BFAR/MMFAR so a repeating boot-loop
 * fault can be localized to an exact instruction. Diagnostic only. */
#define BKP_REG_FAULT_PC RTC_BKP_DR16    /* stacked PC (faulting instruction) */
#define BKP_REG_FAULT_CFSR RTC_BKP_DR17  /* SCB->CFSR (configurable fault status) */
#define BKP_REG_FAULT_BFAR RTC_BKP_DR18  /* SCB->BFAR (bus fault address) */
#define BKP_REG_FAULT_MAGIC RTC_BKP_DR19 /* magic so boot knows DR16-18 are valid */

#ifdef __cplusplus
}
#endif

#endif /* __BACKUP_REGS_H */
