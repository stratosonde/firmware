/* F6/DDR-0006: real LoRaWAN keys live in the gitignored se-identity.h
 * (copy se-identity-template.h and fill in fresh keys). CI / no-keys
 * checkouts fall back to the tracked template with zeroed placeholder
 * keys — compiles everywhere, never flyable. Include THIS header instead
 * of se-identity.h directly. */
#ifndef SE_IDENTITY_SELECT_H__
#define SE_IDENTITY_SELECT_H__

#if defined(__has_include)
  #if __has_include("se-identity.h")
    #include "se-identity.h"
  #else
    #include "se-identity-template.h"
  #endif
#else
  #include "se-identity.h"
#endif

#endif /* SE_IDENTITY_SELECT_H__ */
