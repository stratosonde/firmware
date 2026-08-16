#!/bin/bash
# Phase 3 (pretest-hardening): cppcheck gate over first-party sources.
# Single source of truth: root `make lint` and the CI hygiene job both run
# this. Vendor trees are include paths only (never analyzed); first-party
# diagnostics fail the build. Audited suppressions live in
# tools/cppcheck-suppressions.txt.
set -u
cd "$(dirname "$0")/.."
cppcheck --enable=warning,performance,portability --std=c99 -j 4 \
  --error-exitcode=1 --quiet --inline-suppr \
  --suppressions-list=tools/cppcheck-suppressions.txt \
  --suppress=missingIncludeSystem \
  -D__GNUC__ -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx \
  -ICore/Inc -ILoRaWAN/App -ILoRaWAN/Target \
  -IDrivers/STM32WLxx_HAL_Driver/Inc -IDrivers/STM32WLxx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32WLxx/Include -IDrivers/CMSIS/Include \
  -IUtilities/trace/adv_trace -IUtilities/misc -IUtilities/sequencer \
  -IUtilities/timer -IUtilities/lpm/tiny_lpm \
  -IMiddlewares/Third_Party/LoRaWAN/LmHandler/Packages \
  -IMiddlewares/Third_Party/LoRaWAN/Crypto \
  -IMiddlewares/Third_Party/LoRaWAN/Mac/Region \
  -IMiddlewares/Third_Party/LoRaWAN/Mac \
  -IMiddlewares/Third_Party/LoRaWAN/LmHandler \
  -IMiddlewares/Third_Party/LoRaWAN/Utilities \
  -IMiddlewares/Third_Party/SubGHz_Phy \
  -IMiddlewares/Third_Party/SubGHz_Phy/stm32_radio_driver \
  -IMiddlewares/Third_Party/h3lite/include \
  Core/Src LoRaWAN/App
echo "cppcheck: first-party clean"
