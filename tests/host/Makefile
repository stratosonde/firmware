# Host test harness (R49 / issue #46) — builds the real firmware pure-logic
# sources with the host compiler and runs assert-based tests.
#
# Usage:  make -C tests/host test        (CC overridable: make CC=clang)
# CI:     .github/workflows/ci.yml runs this on ubuntu-latest (gcc).

CC      ?= gcc
# F-01 (#62): -Wsign-conversion catches the mixed-sign division class of bug
CFLAGS  ?= -std=c99 -Wall -Wextra -Wsign-conversion -Wno-unused-parameter -g
INCLUDES := -Istubs -I../../Core/Inc -I../../LoRaWAN/App -I../../Utilities/misc

SRCS := test_main.c ../../Core/Src/power_model.c ../../Core/Src/transmit_plan.c ../../Core/Src/mission_logic.c

# Flight-readiness regression suite (2026-08-08 review). Separate binary: it
# compiles flash_log.c against the simulated NOR backend in fake_w25q.c, which
# the main runner neither needs nor wants linked in.
FLIGHT_SRCS := test_flightreadiness.c fake_w25q.c
FLIGHT_INCLUDES := $(INCLUDES) -I.

# 2026-08-10 review findings: source-scan regressions for the HAL-bound
# findings (#1/#2/#3/#5, gate half of #6). Self-contained, no firmware objects.
FINDINGS_SRCS := test_review_findings.c

# 2026-08-11 review: archive-burst FSM (BURST-01, BURST-02). Replays the
# OnTxData/OnRxData/SendTxData callback order; the model shape is selected by
# a scan of lora_app.c, so it cannot go green without a firmware change.
BURST_SRCS := test_burst_fsm.c

# 2026-08-11 (second pass) stability review: F-1..F-11. Source scans for the
# HAL-bound findings plus behavioural F-4 (burst defeats mode hysteresis,
# needs the real transmit_plan.c/power_model.c) and F-10/F-11 (GNSS parser,
# atgm336h.c #included by the test TU).
STAB_SRCS := test_stability_review.c ../../Core/Src/power_model.c ../../Core/Src/transmit_plan.c ../../Core/Src/sys_caps.c

# R15 (2026-08-11 ChatGPT review): the REAL multiregion_context.c compiled
# against a fake LoRaMac/LmHandler surface with fault injection
# (test_multiregion.c #includes the unit under test). Covers R1 (restore
# fail-closed) and the other session-persistence findings.
MREG_SRCS := test_multiregion.c

# F-002 (#201): the REAL timer_if.c against a minimal RTC HAL fake - proves
# the post-failover re-init does the work (not just that the call exists).
TIMERIF_SRCS := test_timerif.c
TIMERIF_INCLUDES := $(INCLUDES) -I../../Utilities/timer

# 2026-08-12 review (S-A/S-B/S-C, #211/#212/#213): S-A/S-C structural scans,
# S-B behavioral against the real power_model.c.
REVIEW0812_SRCS := test_review_20260812.c ../../Core/Src/power_model.c

# 2026-08-12 stability & predictability deep review (R3 series, #215+):
# scan-configured scheduler replay (test_burst_fsm.c precedent) for the
# HAL-bound lora_app.c findings.
DEEP_SRCS := test_deepreview_20260812.c

# STAB-REVIEW-2026-08-12 (S-04/S-07, #228/#229): the REAL config.c against a
# stub flash - deadman derivation + validator bounds, behavioural.
CONFIG_SRCS := test_config.c

# Sources #included by the test TUs (not linked separately) must still be
# prerequisites or make will not rebuild when they change.
INCLUDED_SRCS := ../../Core/Src/payload_encode.c ../../Core/Src/atgm336h.c ../../Core/Src/flash_log.c

test_runner$(EXE): $(SRCS) $(INCLUDED_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SRCS) -lm

test_flight$(EXE): $(FLIGHT_SRCS) $(INCLUDED_SRCS)
	$(CC) $(CFLAGS) $(FLIGHT_INCLUDES) -o $@ $(FLIGHT_SRCS) -lm

test_findings$(EXE): $(FINDINGS_SRCS)
	$(CC) $(CFLAGS) -o $@ $(FINDINGS_SRCS)

test_burst$(EXE): $(BURST_SRCS) ../../LoRaWAN/App/lora_app.c ../../LoRaWAN/App/lora_app.h
	$(CC) $(CFLAGS) -o $@ $(BURST_SRCS)

test_stability$(EXE): $(STAB_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/stm32_lpm_if.c ../../Core/Src/config.c ../../Core/Src/atgm336h.c ../../Core/Src/sys_sensors.c ../../Core/Src/sys_caps.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(STAB_SRCS) -lm

# -fshort-enums matches the arm-none-eabi ABI default - the target builds
# LoRaMacRegion_t as 1 byte, and multiregion_context.c pins the persisted
# struct layout with _Static_assert(offsetof(crc16)==76, sizeof==80).
test_multiregion$(EXE): $(MREG_SRCS) ../../Core/Src/multiregion_context.c ../../Core/Inc/multiregion_context.h
	$(CC) $(CFLAGS) -fshort-enums $(INCLUDES) -o $@ $(MREG_SRCS) -lm

test_timerif$(EXE): $(TIMERIF_SRCS) ../../Core/Src/timer_if.c ../../Core/Inc/timer_if.h
	$(CC) $(CFLAGS) $(TIMERIF_INCLUDES) -o $@ $(TIMERIF_SRCS) -lm

test_review0812$(EXE): $(REVIEW0812_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/main.c ../../Core/Src/usart_if.c ../../Core/Src/atgm336h.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(REVIEW0812_SRCS) -lm

test_deepreview$(EXE): $(DEEP_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/flash_log.c ../../Core/Inc/flash_log.h ../../Core/Src/atgm336h.c ../../Core/Src/sys_app.c ../../Core/Src/adc_if.c ../../Core/Src/sys_sensors.c ../../Core/Src/stm32_lpm_if.c ../../Core/Src/multiregion_context.c
	$(CC) $(CFLAGS) -o $@ $(DEEP_SRCS)

test_config$(EXE): $(CONFIG_SRCS) ../../Core/Src/config.c ../../Core/Inc/config.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(CONFIG_SRCS)

review0812: test_review0812$(EXE)
	./test_review0812$(EXE)

deep: test_deepreview$(EXE)
	./test_deepreview$(EXE)

config: test_config$(EXE)
	./test_config$(EXE)

# R3-11 (#223): DDR identity integrity against the canonical manifest.
ddrmanifest:
	python3 ../../tools/check_ddr_manifest.py

.PHONY: test flight findings burst stability multiregion baseline all clean review0812 deep config ddrmanifest
test: test_runner$(EXE)
	./test_runner$(EXE)

# Post-fix gate: every FR-xx regression must pass. Wire this into CI once the
# P0/P1 fixes land.
flight: test_flight$(EXE)
	./test_flight$(EXE)

findings: test_findings$(EXE)
	./test_findings$(EXE)

burst: test_burst$(EXE)
	./test_burst$(EXE)

stability: test_stability$(EXE)
	./test_stability$(EXE)

multiregion: test_multiregion$(EXE)
	./test_multiregion$(EXE)

timerif: test_timerif$(EXE)
	./test_timerif$(EXE)

# Pre-fix gate: passes while the documented FR-xx regressions are still open,
# but fails on any NEW breakage. Use this until the fixes land so CI stays
# meaningful instead of red-by-default.
baseline: test_runner$(EXE) test_flight$(EXE) test_findings$(EXE) test_burst$(EXE) test_stability$(EXE) test_multiregion$(EXE) test_timerif$(EXE) test_review0812$(EXE) test_deepreview$(EXE) test_config$(EXE)
	EXPECT_UNFIXED=1 ./test_runner$(EXE)
	EXPECT_UNFIXED=1 ./test_flight$(EXE)
	EXPECT_UNFIXED=1 ./test_findings$(EXE)
	EXPECT_UNFIXED=1 ./test_burst$(EXE)
	EXPECT_UNFIXED=1 ./test_stability$(EXE)
	EXPECT_UNFIXED=1 ./test_multiregion$(EXE)
	EXPECT_UNFIXED=1 ./test_timerif$(EXE)
	EXPECT_UNFIXED=1 ./test_review0812$(EXE)
	EXPECT_UNFIXED=1 ./test_deepreview$(EXE)
	EXPECT_UNFIXED=1 ./test_config$(EXE)

# F16 (2026-08-11 stability review): UBSan/ASan build of the pure-logic
# suite — catches signed overflow / UB at the numerical boundaries the
# protocol fuzz loop drives.
SAN_FLAGS := -fsanitize=undefined,address -fno-omit-frame-pointer
test_sanitize$(EXE): $(SRCS) $(INCLUDED_SRCS)
	$(CC) $(CFLAGS) $(SAN_FLAGS) $(INCLUDES) -o $@ $(SRCS) -lm

.PHONY: sanitize
sanitize: test_sanitize$(EXE)
	./test_sanitize$(EXE)

all: test flight findings burst stability multiregion timerif review0812 deep config ddrmanifest

clean:
	$(RM) test_runner$(EXE) test_flight$(EXE) test_findings$(EXE) test_burst$(EXE) test_stability$(EXE) test_multiregion$(EXE) test_timerif$(EXE) test_review0812$(EXE) test_deepreview$(EXE) test_config$(EXE) test_sanitize$(EXE)
