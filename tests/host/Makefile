# Host test harness (R49 / issue #46) — builds the real firmware pure-logic
# sources with the host compiler and runs assert-based tests.
#
# Usage:  make -C tests/host test        (CC overridable: make CC=clang)
# CI:     .github/workflows/ci.yml runs this on ubuntu-latest (gcc).

CC      ?= gcc
# F-01 (#62): -Wsign-conversion catches the mixed-sign division class of bug
CFLAGS  ?= -std=c99 -Wall -Wextra -Wsign-conversion -Wno-unused-parameter -g
INCLUDES := -Istubs -I../../Core/Inc -I../../LoRaWAN/App -I../../Utilities/misc

SRCS := test_main.c ../../Core/Src/power_model.c ../../Core/Src/transmit_plan.c

# Flight-readiness regression suite (2026-08-08 review). Separate binary: it
# compiles flash_log.c against the simulated NOR backend in fake_w25q.c, which
# the main runner neither needs nor wants linked in.
FLIGHT_SRCS := test_flightreadiness.c fake_w25q.c
FLIGHT_INCLUDES := $(INCLUDES) -I.

# Sources #included by the test TUs (not linked separately) must still be
# prerequisites or make will not rebuild when they change.
INCLUDED_SRCS := ../../Core/Src/payload_encode.c ../../Core/Src/atgm336h.c ../../Core/Src/flash_log.c

test_runner$(EXE): $(SRCS) $(INCLUDED_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SRCS) -lm

test_flight$(EXE): $(FLIGHT_SRCS) $(INCLUDED_SRCS)
	$(CC) $(CFLAGS) $(FLIGHT_INCLUDES) -o $@ $(FLIGHT_SRCS) -lm

.PHONY: test flight baseline all clean
test: test_runner$(EXE)
	./test_runner$(EXE)

# Post-fix gate: every FR-xx regression must pass. Wire this into CI once the
# P0/P1 fixes land.
flight: test_flight$(EXE)
	./test_flight$(EXE)

# Pre-fix gate: passes while the documented FR-xx regressions are still open,
# but fails on any NEW breakage. Use this until the fixes land so CI stays
# meaningful instead of red-by-default.
baseline: test_runner$(EXE) test_flight$(EXE)
	EXPECT_UNFIXED=1 ./test_runner$(EXE)
	EXPECT_UNFIXED=1 ./test_flight$(EXE)

all: test flight

clean:
	$(RM) test_runner$(EXE) test_flight$(EXE)
