#Host test harness(R49 / issue #46) — builds the real firmware pure - logic
#sources with the host compiler and runs assert - based tests.
#
#Usage : make - C tests / host test(CC overridable : make CC = clang)
#CI :.github / workflows / ci.yml runs this on ubuntu - latest(gcc).

CC      ?= gcc
PYTHON  ?= python3
#F - 01(#62) : -Wsign - conversion catches the mixed - sign division class of bug
#Phase 2 : -Werror locks the zero - warning baseline(box gcc 10.3.1 + CI)
CFLAGS  ?= -std=c99 -Wall -Wextra -Wsign-conversion -Werror -Wno-unused-parameter -g
INCLUDES := -Istubs -I../../Core/Inc -I../../LoRaWAN/App -I../../Utilities/misc

#Out - of - tree outputs(pretest - hardening handoff 2026 - 08 - 15, Phase 1 / PIPE - 01):
#every executable lands under build / < configuration> / so a changed CFLAGS
#string can never silently reuse a stale binary, and `git status` stays clean
#(only tests / host / build / is ignored).Override per configuration, e.g.
#make BUILD_DIR = build / coverage CFLAGS = "... --coverage"
BUILD_DIR ?= build/gcc
ifeq ($(OS),Windows_NT)
#cmd builtins(forced through the shell by the `if ` syntax; CreateProcess
#fast - path cannot run builtins and plain `mkdir a / b` is not portable)
MKDIR = if not exist "$(subst /,\,$@)" mkdir "$(subst /,\,$@)"
RMBUILD = if exist build rmdir /s /q build
else
MKDIR = mkdir -p $@
RMBUILD = $(RM) -r build
endif

SRCS := test_main.c ../../Core/Src/power_model.c ../../Core/Src/transmit_plan.c ../../Core/Src/mission_logic.c

#Flight - readiness regression suite(2026 - 08 - 08 review).Separate binary : it
#compiles flash_log.c against the simulated NOR backend in fake_w25q.c, which
#the main runner neither needs nor wants linked in.
FLIGHT_SRCS := test_flightreadiness.c fake_w25q.c
FLIGHT_INCLUDES := $(INCLUDES) -I.

# 2026 - 08 - 10 review findings : source - scan regressions for the HAL - bound
#findings(#1 / #2 / #3 / #5, gate half of #6).Self - contained, no firmware objects.
FINDINGS_SRCS := test_review_findings.c

# 2026 - 08 - 11 review : archive - burst FSM(BURST - 01, BURST - 02).Replays the
#OnTxData / OnRxData / SendTxData callback order; the model shape is selected by
#a scan of lora_app.c, so it cannot go green without a firmware change.
BURST_SRCS := test_burst_fsm.c

# 2026 - 08 - 11(second pass) stability review : F - 1..F - 11. Source scans for the
#HAL - bound findings plus behavioural F - 4(burst defeats mode hysteresis,
#needs the real transmit_plan.c / power_model.c) and F - 10 / F - 11(GNSS parser,
#atgm336h.c #included by the test TU).
STAB_SRCS := test_stability_review.c ../../Core/Src/power_model.c ../../Core/Src/transmit_plan.c ../../Core/Src/sys_caps.c

#R15(2026 - 08 - 11 ChatGPT review) : the REAL multiregion_context.c compiled
#against a fake LoRaMac / LmHandler surface with fault injection
#(test_multiregion.c #includes the unit under test).Covers R1(restore
#fail - closed) and the other session - persistence findings.
MREG_SRCS := test_multiregion.c

#The full - region harness is hermetic : pin every region ON so it does not
#depend on the local region_set.h defaults(a bench bring - up tree may
#legitimately default to a subset, e.g.US915 - only).
MREG_ALL_FLAGS := -DSONDE_REGION_US915=1 -DSONDE_REGION_EU868=1 \
    -DSONDE_REGION_AS923=1 -DSONDE_REGION_AU915=1 -DSONDE_REGION_IN865=1 \
    -DSONDE_REGION_KR920=1 -DSONDE_REGION_RU864=1

#Region - set subset build(region_set.h) : the SAME harness compiled with a
#US915 - only commissioned set(SONDE_REGION_SUBSET_TEST selects the subset
#main).Proves a compiled - out region is unjoinable, unswitchable and reads
#as never - joined even with a stale CRC - valid banked session.
MREG_SUBSET_FLAGS := -DSONDE_REGION_SUBSET_TEST=1 -DSONDE_REGION_US915=1 \
    -DSONDE_REGION_EU868=0 -DSONDE_REGION_AS923=0 -DSONDE_REGION_AU915=0 \
    -DSONDE_REGION_IN865=0 -DSONDE_REGION_KR920=0 -DSONDE_REGION_RU864=0 \
    -Wno-unused-function

#F - 002(#201) : the REAL timer_if.c against a minimal RTC HAL fake - proves
#the post - failover re - init does the work(not just that the call exists).
TIMERIF_SRCS := test_timerif.c
TIMERIF_INCLUDES := $(INCLUDES) -I../../Utilities/timer

# 2026 - 08 - 12 review(S - A / S - B / S - C, #211 / #212 / #213) : S - A / S - C structural scans,
#S - B behavioral against the real power_model.c.
REVIEW0812_SRCS := test_review_20260812.c ../../Core/Src/power_model.c

# 2026 - 08 - 12 stability & predictability deep review(R3 series, #215 +):
#scan - configured scheduler replay(test_burst_fsm.c precedent) for the
#HAL - bound lora_app.c findings.
DEEP_SRCS := test_deepreview_20260812.c

#STAB - REVIEW - 2026 - 08 - 12(S - 04 / S - 07, #228 / #229) : the REAL config.c against a
#stub flash - deadman derivation + validator bounds, behavioural.
CONFIG_SRCS := test_config.c

# 2026 - 08 - 12 independent deep review(DR series) + stability review P1 GNSS:
#DR - 01 / STAB - P1 #1 / DR - 02 behavioural against the real atgm336h.c parser,
#DR - 03 behavioural against the real transmit_plan.c, DR - 02c source scan.
DR_SRCS := test_dr_20260812.c ../../Core/Src/power_model.c ../../Core/Src/transmit_plan.c ../../Core/Src/sys_caps.c

# 2026 - 08 - 14 review(docs / temp / stratosonde - review - 20260814.md):
#GEO - 01(dataset) / GEO - 04(guards)against the real h3lite submodule sources;
#PWR - 02 behavioural against the real power_model.c.Both red - gated until the
#owner's RESTRICTED dataset and the bench characterization land.
GEO_SRCS := test_geo_20260814.c
GEO_INCLUDES := $(INCLUDES) -I../../Middlewares/Third_Party/h3lite/include

PWR_SRCS := test_pwr_20260814.c ../../Core/Src/power_model.c
FIRST_FLIGHT_SRCS := test_first_flight_policy.c ../../Core/Src/first_flight_policy.c

#Refactor stage 1 : module contract for the extracted PacketQueue ADT
#(Core / Src / packet_queue.c).- DENABLE_GNSS_DETAIL_PACKET = 1 so the Push path
#(compiled out of flight builds) is exercised by the contract too.
PQ_SRCS := test_packet_queue.c ../../Core/Src/packet_queue.c

#Refactor stage 2 : module contract for the extracted NVM slot codec
#(Core / Src / nvm_slot.c) - header build / plausibility, CRC match, generation
#race with wraparound, torn - write selection scenarios.
NVM_SRCS := test_nvm_slot.c ../../Core/Src/nvm_slot.c

#H - 01(#281) : durable mission - lifecycle manifest.Pure rules + full I / O path
#against an in - memory internal - flash fake(flash_if.h stub).
MANIFEST_SRCS := test_mission_manifest.c ../../Core/Src/mission_manifest.c

#Refactor stage 3 : module contract for the extracted geofence / region policy
#(Core / Src / region_policy.c).h3lite include path for RegionId / REGION_ *.
REGION_SRCS := test_region_policy.c ../../Core/Src/region_policy.c
REGION_INCLUDES := $(INCLUDES) -I../../Middlewares/Third_Party/h3lite/include

#Refactor stage 4 : module contract for the extracted GNSS acquisition policy
#(Core / Src / gnss_acquire.c) - exhaustive civil - date proof 1970..2100, epoch
#composition, S - A iteration - budget contract, package - completeness rule.
GNSS_ACQ_SRCS := test_gnss_acquire.c ../../Core/Src/gnss_acquire.c

#Refactor stage 5 : shadow - run of the extracted pure TX FSM(tx_fsm.c)
#against the characterised, scan - locked replay model(included by the test).
TXSHADOW_SRCS := test_tx_fsm_shadow.c ../../Core/Src/tx_fsm.c

#SP - 01(#244), 2026 - 08 - 13 SP - series review : GNSS UART error recovery - any
#FE / NE killed the circular DMA because the HAL never consults the
#DMADisableonRxError bit S - C(#212) relied on.Behavioural against the real
#atgm336h.c(#included by the test TU) + structural scans of usart_if.c / main.c.
SP_SRCS := test_sp_20260813.c

# 2026 - 08 - 13 LT series(long - term stability & predictability pass).Behavioural
#LT - 01(science - timer period model, shape gated by a scan of lora_app.c) and
#LT - 06(compensation - table monotonicity against the real power_model.c);
#structural scans for the HAL - / flow - bound findings.
LT_SRCS := test_lt_20260813.c ../../Core/Src/power_model.c

#Sources #included by the test TUs(not linked separately) must still be
#prerequisites or make will not rebuild when they change.
INCLUDED_SRCS := ../../Core/Src/payload_encode.c ../../Core/Src/atgm336h.c ../../Core/Src/flash_log.c

$(BUILD_DIR)/test_runner$(EXE): $(SRCS) $(INCLUDED_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SRCS) -lm

$(BUILD_DIR)/test_flight$(EXE): $(FLIGHT_SRCS) $(INCLUDED_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(FLIGHT_INCLUDES) -o $@ $(FLIGHT_SRCS) -lm

$(BUILD_DIR)/test_findings$(EXE): $(FINDINGS_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(FINDINGS_SRCS)

$(BUILD_DIR)/test_burst$(EXE): $(BURST_SRCS) ../../LoRaWAN/App/lora_app.c ../../LoRaWAN/App/lora_app.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(BURST_SRCS)

$(BUILD_DIR)/test_stability$(EXE): $(STAB_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/stm32_lpm_if.c ../../Core/Src/config.c ../../Core/Src/atgm336h.c ../../Core/Src/sys_sensors.c ../../Core/Src/sys_caps.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(STAB_SRCS) -lm

#- fshort - enums matches the arm - none - eabi ABI default - the target builds
#LoRaMacRegion_t as 1 byte, and multiregion_context.c pins the persisted
#struct layout with _Static_assert(offsetof(crc16) == 76, sizeof == 80).
$(BUILD_DIR)/test_multiregion$(EXE): $(MREG_SRCS) ../../Core/Src/multiregion_context.c ../../Core/Inc/multiregion_context.h ../../Core/Inc/region_set.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -fshort-enums $(MREG_ALL_FLAGS) $(INCLUDES) -o $@ $(MREG_SRCS) -lm

$(BUILD_DIR)/test_multiregion_subset$(EXE): $(MREG_SRCS) ../../Core/Src/multiregion_context.c ../../Core/Inc/multiregion_context.h ../../Core/Inc/region_set.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -fshort-enums $(MREG_SUBSET_FLAGS) $(INCLUDES) -o $@ $(MREG_SRCS) -lm

$(BUILD_DIR)/test_timerif$(EXE): $(TIMERIF_SRCS) ../../Core/Src/timer_if.c ../../Core/Inc/timer_if.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(TIMERIF_INCLUDES) -o $@ $(TIMERIF_SRCS) -lm

$(BUILD_DIR)/test_review0812$(EXE): $(REVIEW0812_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/main.c ../../Core/Src/usart_if.c ../../Core/Src/atgm336h.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(REVIEW0812_SRCS) -lm

$(BUILD_DIR)/test_deepreview$(EXE): $(DEEP_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/flash_log.c ../../Core/Inc/flash_log.h ../../Core/Src/atgm336h.c ../../Core/Src/sys_app.c ../../Core/Src/adc_if.c ../../Core/Src/sys_sensors.c ../../Core/Src/stm32_lpm_if.c ../../Core/Src/multiregion_context.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(DEEP_SRCS)

$(BUILD_DIR)/test_config$(EXE): $(CONFIG_SRCS) ../../Core/Src/config.c ../../Core/Inc/config.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(CONFIG_SRCS)

$(BUILD_DIR)/test_dr$(EXE): $(DR_SRCS) ../../Core/Src/atgm336h.c ../../Core/Inc/atgm336h.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(DR_SRCS) -lm

$(BUILD_DIR)/test_sp0813$(EXE): $(SP_SRCS) ../../Core/Src/atgm336h.c ../../Core/Inc/atgm336h.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SP_SRCS) -lm

$(BUILD_DIR)/test_lt0813$(EXE): $(LT_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/atgm336h.c ../../Core/Src/multiregion_context.c ../../Core/Src/stm32_lpm_if.c ../../Core/Src/main.c ../../Core/Src/mission_state.c ../../Core/Inc/multiregion_context.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(LT_SRCS) -lm

$(BUILD_DIR)/test_geo$(EXE): $(GEO_SRCS) $(wildcard ../../Middlewares/Third_Party/h3lite/src/*.c) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(GEO_INCLUDES) -o $@ $(GEO_SRCS) ../../Middlewares/Third_Party/h3lite/src/*.c -lm

$(BUILD_DIR)/test_pwr$(EXE): $(PWR_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(PWR_SRCS) -lm

$(BUILD_DIR)/test_first_flight$(EXE): $(FIRST_FLIGHT_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(FIRST_FLIGHT_SRCS) -lm

first-flight: $(BUILD_DIR)/test_first_flight$(EXE)
	./$(BUILD_DIR)/test_first_flight$(EXE)

# PRETEST-DEC-01 (#142): arming input (PB13 commissioning button). Same-TU
# pattern (the test #includes the unit); Core/Inc precedes stubs so the REAL
# mission_state.h prototypes are used with test-owned link doubles.
ARMING_SRCS := test_arming_input.c
# Core/Inc precedes stubs so the REAL mission_state.h wins (test-owned link
# doubles); -DSONDE_FLIGHT_BUILD compiles the log gate out - Core/Inc has the
# real SEGGER_RTT.h, and the host link has no RTT backend.
ARMING_INCLUDES := -DSONDE_FLIGHT_BUILD -I../../Core/Inc -Istubs -I../../LoRaWAN/App -I../../Utilities/misc

$(BUILD_DIR)/test_arming$(EXE): $(ARMING_SRCS) ../../Core/Src/arming_input.c ../../Core/Inc/arming_input.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(ARMING_INCLUDES) -o $@ $(ARMING_SRCS) -lm

arming: $(BUILD_DIR)/test_arming$(EXE)
	./$(BUILD_DIR)/test_arming$(EXE)

# MAINT-01/02/03 (Phase 5): the production adapter builders and the semantic
# FSM queries, linked against the REAL lora_app_adapters.c + tx_fsm.c. Every
# polarity-bearing mapping is exercised in both directions; the Phase-5 gate
# reverses each mapping on the box and proves this suite goes red.
ADAPTERS_SRCS := test_lora_app_adapters.c ../../Core/Src/lora_app_adapters.c ../../Core/Src/tx_fsm.c

$(BUILD_DIR)/test_adapters$(EXE): $(ADAPTERS_SRCS) ../../Core/Inc/lora_app_adapters.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(ADAPTERS_SRCS) -lm

appadapters: $(BUILD_DIR)/test_adapters$(EXE)
	./$(BUILD_DIR)/test_adapters$(EXE)

$(BUILD_DIR)/test_pq$(EXE): $(PQ_SRCS) ../../Core/Inc/packet_queue.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DENABLE_GNSS_DETAIL_PACKET=1 $(INCLUDES) -o $@ $(PQ_SRCS) -lm

pq: $(BUILD_DIR)/test_pq$(EXE)
	./$(BUILD_DIR)/test_pq$(EXE)

$(BUILD_DIR)/test_nvm$(EXE): $(NVM_SRCS) ../../Core/Inc/nvm_slot.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(NVM_SRCS) -lm

nvm: $(BUILD_DIR)/test_nvm$(EXE)
	./$(BUILD_DIR)/test_nvm$(EXE)

$(BUILD_DIR)/test_manifest$(EXE): $(MANIFEST_SRCS) ../../Core/Inc/mission_manifest.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DSONDE_FLIGHT_BUILD $(INCLUDES) -o $@ $(MANIFEST_SRCS) -lm

manifest: $(BUILD_DIR)/test_manifest$(EXE)
	./$(BUILD_DIR)/test_manifest$(EXE)

$(BUILD_DIR)/test_region$(EXE): $(REGION_SRCS) ../../Core/Inc/region_policy.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(REGION_INCLUDES) -o $@ $(REGION_SRCS) -lm

region: $(BUILD_DIR)/test_region$(EXE)
	./$(BUILD_DIR)/test_region$(EXE)

$(BUILD_DIR)/test_gnss_acq$(EXE): $(GNSS_ACQ_SRCS) ../../Core/Inc/gnss_acquire.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(GNSS_ACQ_SRCS) -lm

gnssacq: $(BUILD_DIR)/test_gnss_acq$(EXE)
	./$(BUILD_DIR)/test_gnss_acq$(EXE)

# -Wno-unused-function: the included burst suite's own tests/main are parked
$(BUILD_DIR)/test_txshadow$(EXE): $(TXSHADOW_SRCS) test_burst_fsm.c ../../Core/Inc/tx_fsm.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -Wno-unused-function $(INCLUDES) -o $@ $(TXSHADOW_SRCS) -lm

txshadow: $(BUILD_DIR)/test_txshadow$(EXE)
	./$(BUILD_DIR)/test_txshadow$(EXE)

# TX adapter wiring assertions (Phase A1/A2 of the 2026-08-15 next-step
# handoff; finding TX-ADAPTER-01): designated-field/polarity scans on the
# OnTxData confirm-input initializer. Added to `all` by the A2 fix commit
# (the suite is RED until then, by design).
TXADAPTER_SRCS := test_tx_adapter.c

$(BUILD_DIR)/test_txadapter$(EXE): $(TXADAPTER_SRCS) ../../LoRaWAN/App/lora_app.c ../../Core/Src/tx_fsm.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(TXADAPTER_SRCS) -lm

txadapter: $(BUILD_DIR)/test_txadapter$(EXE)
	./$(BUILD_DIR)/test_txadapter$(EXE)

# GNSS fix-acceptance wiring gate (Phase A4/A5 of the 2026-08-15 next-step
# handoff; tracker #284/H-08): characterization of the hardcoded thresholds
# plus red wiring scans for the one configured authoritative predicate.
# Added to `all` by the A5 implementation commit (RED until then, by design).
GNSSFIX_SRCS := test_gnss_fix_acceptance.c

$(BUILD_DIR)/test_gnssfix$(EXE): $(GNSSFIX_SRCS) ../../Core/Src/atgm336h.c ../../LoRaWAN/App/lora_app.c ../../Core/Src/gnss_acquire.c ../../Core/Inc/gnss_acquire.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(GNSSFIX_SRCS) -lm

gnssfix: $(BUILD_DIR)/test_gnssfix$(EXE)
	./$(BUILD_DIR)/test_gnssfix$(EXE)

# GPS-loss same-wake recovery wiring gate (Phase A6/A7 of the 2026-08-15
# next-step handoff; tracker #285/H-09). Added to `all` by the A7
# implementation commit (RED until then, by design).
GPSLOSS_SRCS := test_gps_loss_recovery.c

$(BUILD_DIR)/test_gpsloss$(EXE): $(GPSLOSS_SRCS) ../../LoRaWAN/App/lora_app.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(GPSLOSS_SRCS) -lm

gpsloss: $(BUILD_DIR)/test_gpsloss$(EXE)
	./$(BUILD_DIR)/test_gpsloss$(EXE)

review0812: $(BUILD_DIR)/test_review0812$(EXE)
	./$(BUILD_DIR)/test_review0812$(EXE)

# 2026-08-14 review (PWR-02 bench-gated; GEO-01 dataset + GEO-04 guards both
# landed): pwr runs red until the Nichicon characterization lands and uses
# the EXPECT_UNFIXED pre-fix gate; geo is fully green since h3lite 0c31029.
pwr: $(BUILD_DIR)/test_pwr$(EXE)
	./$(BUILD_DIR)/test_pwr$(EXE)

geo: $(BUILD_DIR)/test_geo$(EXE)
	./$(BUILD_DIR)/test_geo$(EXE)

pwr-gate: $(BUILD_DIR)/test_pwr$(EXE)
	EXPECT_UNFIXED=1 ./$(BUILD_DIR)/test_pwr$(EXE)

# geo-gate: EXPECT_UNFIXED retired 2026-08-16 with the GEO-04 landing
# (h3lite 0c31029) - the suite is a plain hard gate now.
geo-gate: $(BUILD_DIR)/test_geo$(EXE)
	./$(BUILD_DIR)/test_geo$(EXE)

deep: $(BUILD_DIR)/test_deepreview$(EXE)
	./$(BUILD_DIR)/test_deepreview$(EXE)

config: $(BUILD_DIR)/test_config$(EXE)
	./$(BUILD_DIR)/test_config$(EXE)

dr: $(BUILD_DIR)/test_dr$(EXE)
	./$(BUILD_DIR)/test_dr$(EXE)

sp0813: $(BUILD_DIR)/test_sp0813$(EXE)
	./$(BUILD_DIR)/test_sp0813$(EXE)

lt0813: $(BUILD_DIR)/test_lt0813$(EXE)
	./$(BUILD_DIR)/test_lt0813$(EXE)

# The lt0813 suite is red-by-design locally while the LT campaign lands; in
# `all` (the CI gate) it must run as the green pre-fix gate (same semantics
# as baseline) or every push fails on the documented known-unfixed checks.
# Any UNEXPECTED failure (a CHECK anchor, a compile error) still fails CI.
lt0813-gate: $(BUILD_DIR)/test_lt0813$(EXE)
	EXPECT_UNFIXED=1 ./$(BUILD_DIR)/test_lt0813$(EXE)

# R3-11 (#223): DDR identity integrity against the canonical manifest.
ddrmanifest:
	$(PYTHON) ../../tools/check_ddr_manifest.py

# PIPE-05 (pretest-hardening handoff, Phase 1): ARM build source inventory -
# every production .c compiled exactly once, no archive/reference/test/example
# sources, no stale force-added Debug fragments.
arminventory:
	$(PYTHON) ../../tools/check_arm_source_inventory.py

.PHONY: test flight findings burst stability multiregion multiregion-subset baseline all clean review0812 deep config dr sp0813 lt0813 lt0813-gate ddrmanifest arminventory pwr geo pwr-gate geo-gate first-flight pq nvm region gnssacq txshadow txadapter gnssfix gpsloss arming appadapters contracts check characterization integration structural release-gate
test: $(BUILD_DIR)/test_runner$(EXE)
	./$(BUILD_DIR)/test_runner$(EXE)

# Post-fix gate: every FR-xx regression must pass. Wire this into CI once the
# P0/P1 fixes land.
flight: $(BUILD_DIR)/test_flight$(EXE)
	./$(BUILD_DIR)/test_flight$(EXE)

findings: $(BUILD_DIR)/test_findings$(EXE)
	./$(BUILD_DIR)/test_findings$(EXE)

burst: $(BUILD_DIR)/test_burst$(EXE)
	./$(BUILD_DIR)/test_burst$(EXE)

stability: $(BUILD_DIR)/test_stability$(EXE)
	./$(BUILD_DIR)/test_stability$(EXE)

multiregion: $(BUILD_DIR)/test_multiregion$(EXE)
	./$(BUILD_DIR)/test_multiregion$(EXE)

multiregion-subset: $(BUILD_DIR)/test_multiregion_subset$(EXE)
	./$(BUILD_DIR)/test_multiregion_subset$(EXE)

timerif: $(BUILD_DIR)/test_timerif$(EXE)
	./$(BUILD_DIR)/test_timerif$(EXE)

# Deprecated alias for `all` (was the pre-fix EXPECT_UNFIXED gate over
# every suite). Kept so old docs/muscle memory land on the honest targets:
# `check` is the no-masking mandatory gate, `characterization` holds the
# EXPECT_UNFIXED suites.
baseline: all

# F16 (2026-08-11 stability review): UBSan/ASan build of the pure-logic
# suite — catches signed overflow / UB at the numerical boundaries the
# protocol fuzz loop drives.
#
# Phase 3 (#263): expanded from test_main only to every suite that EXECUTES
# production code (19 instrumented exes). Suites that only slurp+scan source
# text (burst, deepreview, findings, txadapter, gpsloss) are excluded - they
# run no firmware, so there is nothing to instrument. Suites with documented
# expected failures (lt0813 x2) exit 0 via the baseline mechanism, same as
# the normal build.
SAN_FLAGS := -fsanitize=undefined,address -fno-omit-frame-pointer
SAN_BUILD_DIR := build/asan-ubsan
H3LITE_SRCS := $(wildcard ../../Middlewares/Third_Party/h3lite/src/*.c)

# $(1) exe name, $(2) SRCS var, $(3) INCLUDES var, $(4) extra compile flags,
# $(5) extra link sources, $(6) extra prereqs (sources #included by the TU)
define SAN_SUITE_RULE
$(SAN_BUILD_DIR)/$(1)$(EXE): $$($(2)) $(INCLUDED_SRCS) $(6) | $(SAN_BUILD_DIR)
	$$(CC) $$(CFLAGS) $$(SAN_FLAGS) $$($(3)) $(4) -o $$@ $$($(2)) $(5) -lm
SAN_EXES += $(SAN_BUILD_DIR)/$(1)$(EXE)
endef

$(eval $(call SAN_SUITE_RULE,test_sanitize,SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_flight,FLIGHT_SRCS,FLIGHT_INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_stability,STAB_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_multiregion,MREG_SRCS,INCLUDES,-fshort-enums $(MREG_ALL_FLAGS),,../../Core/Src/multiregion_context.c))
$(eval $(call SAN_SUITE_RULE,test_timerif,TIMERIF_SRCS,TIMERIF_INCLUDES,,,../../Core/Src/timer_if.c))
$(eval $(call SAN_SUITE_RULE,test_review0812,REVIEW0812_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_config,CONFIG_SRCS,INCLUDES,,,../../Core/Src/config.c))
$(eval $(call SAN_SUITE_RULE,test_dr,DR_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_sp0813,SP_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_lt0813,LT_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_geo,GEO_SRCS,GEO_INCLUDES,,$(H3LITE_SRCS),$(H3LITE_SRCS)))
$(eval $(call SAN_SUITE_RULE,test_pwr,PWR_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_first_flight,FIRST_FLIGHT_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_pq,PQ_SRCS,INCLUDES,-DENABLE_GNSS_DETAIL_PACKET=1,,))
$(eval $(call SAN_SUITE_RULE,test_nvm,NVM_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_region,REGION_SRCS,REGION_INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_gnss_acq,GNSS_ACQ_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_txshadow,TXSHADOW_SRCS,INCLUDES,-Wno-unused-function,,test_burst_fsm.c))
$(eval $(call SAN_SUITE_RULE,test_gnssfix,GNSSFIX_SRCS,INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_arming,ARMING_SRCS,ARMING_INCLUDES,,,))
$(eval $(call SAN_SUITE_RULE,test_adapters,ADAPTERS_SRCS,INCLUDES,,,))

# EXPECT_UNFIXED=1 mirrors the gate semantics (`all` runs the red-by-design
# suite lt0813 the same way): documented expected failures are reported but
# do not fail the run; a NEW failure (or a sanitizer abort) still does.
.PHONY: sanitize
sanitize: $(SAN_EXES)
	@for t in $(SAN_EXES); do \
	    echo "--- sanitize: $$t"; \
	    EXPECT_UNFIXED=1 ./$$t > $$t.log 2>&1 || { cat $$t.log; exit 1; }; \
	    tail -1 $$t.log; \
	done
	@echo "sanitize: $(words $(SAN_EXES)) instrumented suites clean"

# ---------------------------------------------------------------------------
# Honest entry points (pretest-hardening handoff 2026-08-15, Phase 1 / PIPE-02)
#
# check            every mandatory host suite, NO expected-failure masking
# characterization the owner-gated known failures (EXPECT_UNFIXED=1); green
#                  only while the failures are exactly the documented set
# contracts        pure-module contract suites
# integration      linked production-object/fake suites (coarse; MAINT-04
#                  refines the classification in Phase 5)
# structural       source/build/document scan suites (regression guards only)
# sanitize         ASan+UBSan build+run (own build/asan-ubsan configuration)
# release-gate     Phase-1 shape: check + sanitize, masking refused; the ARM
#                  build/artifact legs arrive with the Phase-6 CI restructure
# all              check + characterization (the CI gate, STAB-09/#156)
# ---------------------------------------------------------------------------
check: test flight findings burst stability multiregion multiregion-subset timerif review0812 deep config dr sp0813 first-flight pq nvm region gnssacq txshadow txadapter gnssfix gpsloss arming appadapters ddrmanifest arminventory

characterization: lt0813-gate geo-gate pwr-gate
	@echo "characterization: 5 documented open failures awaited - LT-06 x2 (#248), PWR-02 x3 (#297)"

integration: test flight multiregion multiregion-subset timerif config txadapter gnssfix gpsloss

structural: findings burst stability review0812 deep dr sp0813 ddrmanifest arminventory

release-gate: check sanitize
	@if [ -n "$$EXPECT_UNFIXED" ]; then echo "FATAL: release-gate refuses EXPECT_UNFIXED masking"; exit 1; fi

all: check characterization

# Refactor stage 7: module-contract suites only - the "what is proven about
# module X" entry point (see R2_TEST_MAP.md). `all` keeps running everything.
contracts: first-flight pq nvm manifest region gnssacq txshadow arming appadapters

clean:
	-$(RMBUILD)

$(BUILD_DIR) $(SAN_BUILD_DIR):
	$(MKDIR)
