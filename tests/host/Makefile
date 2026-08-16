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

# 2026-08-12 independent deep review (DR series) + stability review P1 GNSS:
# DR-01/STAB-P1#1/DR-02 behavioural against the real atgm336h.c parser,
# DR-03 behavioural against the real transmit_plan.c, DR-02c source scan.
DR_SRCS := test_dr_20260812.c ../../Core/Src/power_model.c ../../Core/Src/transmit_plan.c ../../Core/Src/sys_caps.c

# 2026-08-14 review (docs/temp/stratosonde-review-20260814.md):
# GEO-01 (dataset) / GEO-04 (guards) against the real h3lite submodule sources;
# PWR-02 behavioural against the real power_model.c. Both red-gated until the
# owner's RESTRICTED dataset and the bench characterization land.
GEO_SRCS := test_geo_20260814.c
GEO_INCLUDES := $(INCLUDES) -I../../Middlewares/Third_Party/h3lite/include

PWR_SRCS := test_pwr_20260814.c ../../Core/Src/power_model.c
FIRST_FLIGHT_SRCS := test_first_flight_policy.c ../../Core/Src/first_flight_policy.c

# Refactor stage 1: module contract for the extracted PacketQueue ADT
# (Core/Src/packet_queue.c). -DENABLE_GNSS_DETAIL_PACKET=1 so the Push path
# (compiled out of flight builds) is exercised by the contract too.
PQ_SRCS := test_packet_queue.c ../../Core/Src/packet_queue.c

# Refactor stage 2: module contract for the extracted NVM slot codec
# (Core/Src/nvm_slot.c) - header build/plausibility, CRC match, generation
# race with wraparound, torn-write selection scenarios.
NVM_SRCS := test_nvm_slot.c ../../Core/Src/nvm_slot.c

# Refactor stage 3: module contract for the extracted geofence/region policy
# (Core/Src/region_policy.c). h3lite include path for RegionId/REGION_*.
REGION_SRCS := test_region_policy.c ../../Core/Src/region_policy.c
REGION_INCLUDES := $(INCLUDES) -I../../Middlewares/Third_Party/h3lite/include

# Refactor stage 4: module contract for the extracted GNSS acquisition policy
# (Core/Src/gnss_acquire.c) - exhaustive civil-date proof 1970..2100, epoch
# composition, S-A iteration-budget contract, package-completeness rule.
GNSS_ACQ_SRCS := test_gnss_acquire.c ../../Core/Src/gnss_acquire.c

# Refactor stage 5: shadow-run of the extracted pure TX FSM (tx_fsm.c)
# against the characterised, scan-locked replay model (included by the test).
TXSHADOW_SRCS := test_tx_fsm_shadow.c ../../Core/Src/tx_fsm.c

# SP-01 (#244), 2026-08-13 SP-series review: GNSS UART error recovery - any
# FE/NE killed the circular DMA because the HAL never consults the
# DMADisableonRxError bit S-C (#212) relied on. Behavioural against the real
# atgm336h.c (#included by the test TU) + structural scans of usart_if.c/main.c.
SP_SRCS := test_sp_20260813.c

# 2026-08-13 LT series (long-term stability & predictability pass). Behavioural
# LT-01 (science-timer period model, shape gated by a scan of lora_app.c) and
# LT-06 (compensation-table monotonicity against the real power_model.c);
# structural scans for the HAL-/flow-bound findings.
LT_SRCS := test_lt_20260813.c ../../Core/Src/power_model.c

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

test_dr$(EXE): $(DR_SRCS) ../../Core/Src/atgm336h.c ../../Core/Inc/atgm336h.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(DR_SRCS) -lm

test_sp0813$(EXE): $(SP_SRCS) ../../Core/Src/atgm336h.c ../../Core/Inc/atgm336h.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SP_SRCS) -lm

test_lt0813$(EXE): $(LT_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/atgm336h.c ../../Core/Src/multiregion_context.c ../../Core/Src/stm32_lpm_if.c ../../Core/Src/main.c ../../Core/Src/mission_state.c ../../Core/Inc/multiregion_context.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(LT_SRCS) -lm

test_geo$(EXE): $(GEO_SRCS) $(wildcard ../../Middlewares/Third_Party/h3lite/src/*.c)
	$(CC) $(CFLAGS) $(GEO_INCLUDES) -o $@ $(GEO_SRCS) ../../Middlewares/Third_Party/h3lite/src/*.c -lm

test_pwr$(EXE): $(PWR_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(PWR_SRCS) -lm

test_first_flight$(EXE): $(FIRST_FLIGHT_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(FIRST_FLIGHT_SRCS) -lm

first-flight: test_first_flight$(EXE)
	./test_first_flight$(EXE)

test_pq$(EXE): $(PQ_SRCS) ../../Core/Inc/packet_queue.h
	$(CC) $(CFLAGS) -DENABLE_GNSS_DETAIL_PACKET=1 $(INCLUDES) -o $@ $(PQ_SRCS) -lm

pq: test_pq$(EXE)
	./test_pq$(EXE)

test_nvm$(EXE): $(NVM_SRCS) ../../Core/Inc/nvm_slot.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(NVM_SRCS) -lm

nvm: test_nvm$(EXE)
	./test_nvm$(EXE)

test_region$(EXE): $(REGION_SRCS) ../../Core/Inc/region_policy.h
	$(CC) $(CFLAGS) $(REGION_INCLUDES) -o $@ $(REGION_SRCS) -lm

region: test_region$(EXE)
	./test_region$(EXE)

test_gnss_acq$(EXE): $(GNSS_ACQ_SRCS) ../../Core/Inc/gnss_acquire.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(GNSS_ACQ_SRCS) -lm

gnssacq: test_gnss_acq$(EXE)
	./test_gnss_acq$(EXE)

# -Wno-unused-function: the included burst suite's own tests/main are parked
test_txshadow$(EXE): $(TXSHADOW_SRCS) test_burst_fsm.c ../../Core/Inc/tx_fsm.h
	$(CC) $(CFLAGS) -Wno-unused-function $(INCLUDES) -o $@ $(TXSHADOW_SRCS) -lm

txshadow: test_txshadow$(EXE)
	./test_txshadow$(EXE)

# TX adapter wiring assertions (Phase A1/A2 of the 2026-08-15 next-step
# handoff; finding TX-ADAPTER-01): designated-field/polarity scans on the
# OnTxData confirm-input initializer. Added to `all` by the A2 fix commit
# (the suite is RED until then, by design).
TXADAPTER_SRCS := test_tx_adapter.c

test_txadapter$(EXE): $(TXADAPTER_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/tx_fsm.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(TXADAPTER_SRCS) -lm

txadapter: test_txadapter$(EXE)
	./test_txadapter$(EXE)

review0812: test_review0812$(EXE)
	./test_review0812$(EXE)

# 2026-08-14 review (PWR-02 bench-gated; GEO-01 dataset, GEO-04 guards):
# pwr/geo run red until the Nichicon characterization and the RESTRICTED
# dataset land; in `all` (the CI gate) they run as green pre-fix gates.
pwr: test_pwr$(EXE)
	./test_pwr$(EXE)

geo: test_geo$(EXE)
	./test_geo$(EXE)

pwr-gate: test_pwr$(EXE)
	EXPECT_UNFIXED=1 ./test_pwr$(EXE)

geo-gate: test_geo$(EXE)
	EXPECT_UNFIXED=1 ./test_geo$(EXE)

deep: test_deepreview$(EXE)
	./test_deepreview$(EXE)

config: test_config$(EXE)
	./test_config$(EXE)

dr: test_dr$(EXE)
	./test_dr$(EXE)

sp0813: test_sp0813$(EXE)
	./test_sp0813$(EXE)

lt0813: test_lt0813$(EXE)
	./test_lt0813$(EXE)

# The lt0813 suite is red-by-design locally while the LT campaign lands; in
# `all` (the CI gate) it must run as the green pre-fix gate (same semantics
# as baseline) or every push fails on the documented known-unfixed checks.
# Any UNEXPECTED failure (a CHECK anchor, a compile error) still fails CI.
lt0813-gate: test_lt0813$(EXE)
	EXPECT_UNFIXED=1 ./test_lt0813$(EXE)

# R3-11 (#223): DDR identity integrity against the canonical manifest.
ddrmanifest:
	python3 ../../tools/check_ddr_manifest.py

.PHONY: test flight findings burst stability multiregion baseline all clean review0812 deep config dr sp0813 lt0813 lt0813-gate ddrmanifest pwr geo pwr-gate geo-gate first-flight pq nvm region gnssacq txshadow txadapter contracts
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
baseline: test_runner$(EXE) test_flight$(EXE) test_findings$(EXE) test_burst$(EXE) test_stability$(EXE) test_multiregion$(EXE) test_timerif$(EXE) test_review0812$(EXE) test_deepreview$(EXE) test_config$(EXE) test_dr$(EXE) test_sp0813$(EXE) test_lt0813$(EXE) test_geo$(EXE) test_pwr$(EXE)
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
	EXPECT_UNFIXED=1 ./test_dr$(EXE)
	EXPECT_UNFIXED=1 ./test_sp0813$(EXE)
	EXPECT_UNFIXED=1 ./test_lt0813$(EXE)
	EXPECT_UNFIXED=1 ./test_geo$(EXE)
	EXPECT_UNFIXED=1 ./test_pwr$(EXE)

# F16 (2026-08-11 stability review): UBSan/ASan build of the pure-logic
# suite — catches signed overflow / UB at the numerical boundaries the
# protocol fuzz loop drives.
SAN_FLAGS := -fsanitize=undefined,address -fno-omit-frame-pointer
test_sanitize$(EXE): $(SRCS) $(INCLUDED_SRCS)
	$(CC) $(CFLAGS) $(SAN_FLAGS) $(INCLUDES) -o $@ $(SRCS) -lm

.PHONY: sanitize
sanitize: test_sanitize$(EXE)
	./test_sanitize$(EXE)

all: test flight findings burst stability multiregion timerif review0812 deep config dr sp0813 lt0813-gate geo-gate pwr-gate first-flight pq nvm region gnssacq txshadow txadapter ddrmanifest

# Refactor stage 7: module-contract suites only - the "what is proven about
# module X" entry point (see R2_TEST_MAP.md). `all` keeps running everything.
contracts: first-flight pq nvm region gnssacq txshadow

clean:
	$(RM) test_runner$(EXE) test_flight$(EXE) test_findings$(EXE) test_burst$(EXE) test_stability$(EXE) test_multiregion$(EXE) test_timerif$(EXE) test_review0812$(EXE) test_deepreview$(EXE) test_config$(EXE) test_dr$(EXE) test_sp0813$(EXE) test_lt0813$(EXE) test_geo$(EXE) test_pwr$(EXE) test_first_flight$(EXE) test_pq$(EXE) test_nvm$(EXE) test_region$(EXE) test_gnss_acq$(EXE) test_txshadow$(EXE) test_txadapter$(EXE) test_sanitize$(EXE)
