#!/bin/bash
# PIPE-07 (Phase 6): package the ARM evidence bundle for one configuration.
# Usage: tools/make_arm_evidence.sh <debug|flight>
# Produces dist/arm-evidence-<cfg>/ with the bin/elf/map, SHA-256 manifest,
# size output, largest-symbol report, and stack-usage summary. The release
# gate downloads exactly this bundle; nothing is rebuilt downstream.
set -eu
CFG=$1
[ "$CFG" = "debug" ] || [ "$CFG" = "flight" ] || {
  echo "usage: tools/make_arm_evidence.sh <debug|flight>"; exit 2; }
cd "$(dirname "$0")/.."
ELF=Debug/Radio_Sonde_E5_HF_EU.elf
OUT=dist/arm-evidence-$CFG
mkdir -p "$OUT"
BASENAME=Radio_Sonde_E5_HF_EU

[ -f "$ELF" ] || { echo "FATAL: $ELF missing - build first"; exit 1; }

# binary
arm-none-eabi-objcopy -O binary "$ELF" "$OUT/${BASENAME}_$CFG.bin"
# elf + map (map is produced by the link rule)
cp "$ELF" "$OUT/${BASENAME}_$CFG.elf"
if [ -f "Debug/$BASENAME.map" ]; then
  cp "Debug/$BASENAME.map" "$OUT/${BASENAME}_$CFG.map"
fi

# size report
arm-none-eabi-size "$ELF" > "$OUT/size.txt"
# largest symbols
arm-none-eabi-nm --print-size --size-sort --radix=d "$ELF" \
  | tail -n 25 > "$OUT/largest-symbols.txt"
# stack usage summary (Cube's .su files; worst offenders first)
find Debug -name '*.su' -print0 | xargs -0 cat 2>/dev/null \
  | sort -k2 -n | tail -n 25 > "$OUT/stack-usage.txt" || true

# hashes over every artifact in the bundle (excluding the sums file itself -
# the redirection order would otherwise capture it empty)
(cd "$OUT" && sha256sum $(ls -1 | grep -v sha256sums.txt) | sort -k2) > "$OUT/sha256sums.txt"

# manifest
{
  echo "stratosonde arm evidence manifest (PIPE-07)"
  echo "configuration:     $CFG"
  echo "git_sha:           $(git rev-parse HEAD)"
  echo "h3lite_sha:        $(git submodule status Middlewares/Third_Party/h3lite | awk '{print $1}')"
  echo "compiler:          $(command -v arm-none-eabi-gcc) $(arm-none-eabi-gcc -dumpversion)"
  echo "linker_script:     $(sha256sum STM32WLE5JCIX_FLASH.ld | awk '{print $1}')"
  echo "embedded_marker:   SONDE_BUILD:$CFG (verified by the CI job)"
  echo "power_profile:     $(grep -oE 'POWER_PROFILE_UNQUALIFIED_LEGACY_ID 0x[0-9A-F]+U' Core/Inc/power_model.h | head -1) (schema $(grep -oE 'POWER_PROFILE_SCHEMA_VERSION [0-9]+U' Core/Inc/power_model.h | head -1)) - BEH-06"
  echo "region_profile:    automatic multi-region (PRETEST-DEC-02, option 2); commissioned home region US915"
  (cd "$OUT" && sha256sum ${BASENAME}_$CFG.bin ${BASENAME}_$CFG.elf)
} > "$OUT/manifest.txt"

echo "evidence bundle: $OUT"
ls -1 "$OUT"
