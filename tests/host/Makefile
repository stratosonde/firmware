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

test_runner$(EXE): $(SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SRCS) -lm

.PHONY: test clean
test: test_runner$(EXE)
	./test_runner$(EXE)

clean:
	$(RM) test_runner$(EXE)
