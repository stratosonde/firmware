/* sys_caps.c - F-014 (#207): slimmed system-capability tracking. See header. */
#include "sys_caps.h"

static uint8_t g_failed = 0;

void SysCaps_MarkFailed(sys_cap_t cap)
{
    g_failed |= (uint8_t)cap;
}

bool SysCaps_Available(sys_cap_t cap)
{
    return (g_failed & (uint8_t)cap) == 0;
}

uint8_t SysCaps_Raw(void)
{
    return g_failed;
}
