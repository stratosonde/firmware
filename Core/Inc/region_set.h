/*
 * region_set.h - compile-time commissioned region set.
 *
 * Each SONDE_REGION_* flag selects whether a LoRaWAN region belongs to this
 * build's commissioned set:
 *   1 - joined during the ground commissioning ceremony and usable in flight
 *   0 - compiled out: never joined, never switched to, and reads as
 *       never-joined for the whole mission (MultiRegion_IsRegionJoined
 *       returns false) even if a stale flash bank from an all-regions build
 *       still holds a valid session for it.
 *
 * The default is every region enabled - the historical behaviour. Override
 * per build with PROJECT_CPPFLAGS (e.g. -DSONDE_REGION_EU868=0 ...) or edit
 * the defaults below for a dedicated bring-up build.
 *
 * Typical flow: US915-only for first hardware bring-up, US915+EU868 for
 * dual-region testing, all regions for flight.
 *
 * Note: this set sizes the COMMISSIONED region list only. The flash bank
 * layout (MAX_REGION_CONTEXTS slots, MULTIREGION_VERSION) is unchanged, so
 * subset and all-region builds can read each other's banks - compiled-out
 * regions are simply ignored.
 */

#ifndef REGION_SET_H
#define REGION_SET_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SONDE_REGION_US915
#define SONDE_REGION_US915 1
#endif

#ifndef SONDE_REGION_EU868
#define SONDE_REGION_EU868 0
#endif

#ifndef SONDE_REGION_AS923
#define SONDE_REGION_AS923 0
#endif

#ifndef SONDE_REGION_AU915
#define SONDE_REGION_AU915 0
#endif

#ifndef SONDE_REGION_IN865
#define SONDE_REGION_IN865 0
#endif

#ifndef SONDE_REGION_KR920
#define SONDE_REGION_KR920 0
#endif

#ifndef SONDE_REGION_RU864
#define SONDE_REGION_RU864 0
#endif

/* An empty set would leave the pre-join table zero-length (illegal C) and
 * the unit unable to commission anything at all. */
#if (SONDE_REGION_US915 + SONDE_REGION_EU868 + SONDE_REGION_AS923 + \
     SONDE_REGION_AU915 + SONDE_REGION_IN865 + SONDE_REGION_KR920 + \
     SONDE_REGION_RU864) < 1
#error "region_set.h: at least one SONDE_REGION_* must be 1"
#endif

#ifdef __cplusplus
}
#endif

#endif /* REGION_SET_H */
