#!/bin/bash
# Phase 3 (pretest-hardening): changed-lines clang-format check.
# REPORT MODE - prints the diff the gate would reject; never fails the build.
# Promotion to a hard gate is blocked on the 2-space/4-space style split
# (CubeIDE-generated files vs new modules), which needs an owner decision.
set -u
cd "$(dirname "$0")/.."
base="${1:-}"
[ -z "$base" ] && base="HEAD~1"
if ! command -v git-clang-format >/dev/null 2>&1; then
    echo "clang-format: git-clang-format not installed - skipped (report mode)"
    exit 0
fi
out=$(git clang-format --diff "$base" -- Core/Src Core/Inc LoRaWAN/App 2>&1)
if [ -z "$out" ] || echo "$out" | grep -q "no modified files to format"; then
    echo "clang-format: changed lines clean (vs $base)"
else
    echo "clang-format: changed lines WOULD be reformatted (report mode):"
    echo "$out"
fi
