/* Host-test stub (#57): sequencer/LPM task-id definitions used by the units
 * under test. Only the IDs atgm336h.c names.
 * F-002 (#201): suites compiling the REAL timer_if.c define
 * HOST_USE_REAL_TIMER_IF and need the real utilities_def.h (via
 * utilities_conf.h) instead. */
#ifdef HOST_USE_REAL_TIMER_IF
#include "../../Core/Inc/utilities_def.h"
#else
#ifndef UTILITIES_DEF_H_STUB
#define UTILITIES_DEF_H_STUB
typedef enum { CFG_LPM_GNSS_Id = 0 } HostCfgLpm_Id_t;
#endif
#endif
