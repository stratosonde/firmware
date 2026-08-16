#!/bin/bash
# Changed-lines clang-format HARD GATE (owner decision 2026-08-16: 2-space
# LLVM). Logic + binary resolution live in tools/check_changed_format.py.
set -u
cd "$(dirname "$0")/.."
base="${1:-}"
[ -z "$base" ] && base="HEAD~1"
PY=python3
command -v python3 >/dev/null 2>&1 || PY=python
"$PY" tools/check_changed_format.py "$base"

