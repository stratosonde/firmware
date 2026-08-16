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

# hashes over every artifact in the bundle
(cd "$OUT" && sha256sum * | sort -k2) > "$OUT/sha256sums.txt"

# manifest
{
  echo "stratosonde arm evidence manifest (PIPE-07)"
  echo "configuration:     $CFG"
  echo "git_sha:           $(git rev-parse HEAD)"
  echo "h3lite_sha:        $(git submodule status Middlewares/Third_Party/h3lite | awk '{print $1}')"
  echo "compiler:          $(command -v arm-none-eabi-gcc) $(arm-none-eabi-gcc -dumpversion)"
  echo "linker_script:     $(sha256sum STM32WLE5JCIX_FLASH.ld | awk '{print $1}')"
  echo "embedded_marker:   SONDE_BUILD:$CFG (verified by the CI job)"
  (cd "$OUT" && sha256sum ${BASENAME}_$CFG.bin ${BASENAME}_$CFG.elf)
} > "$OUT/manifest.txt"

echo "evidence bundle: $OUT"
ls -1 "$OUT"
