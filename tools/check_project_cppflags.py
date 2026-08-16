#!/usr/bin/env python3
"""PIPE-04 (#265) checker: every ACTIVE C compile recipe in the Cube-generated
Debug build must honour the centralized PROJECT_CPPFLAGS variable, and no
tracked build fragment may carry an injected SONDE_FLIGHT_BUILD (the retired
sed/PowerShell mutation). Active fragments are derived from Debug/Makefile's
-include list - never from a filesystem walk - so a fragment CubeIDE drops
from the build stops being checked and a new one is checked automatically.

Exit 0 = clean; exit 1 = violations listed.
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
DEBUG = ROOT / "Debug"

mk = (DEBUG / "Makefile").read_text(encoding="utf-8")
frags = [
    line.split()[1]
    for line in mk.splitlines()
    if line.startswith("-include ") and line.strip().endswith("subdir.mk")
]
if not frags:
    sys.exit("FATAL: no subdir.mk fragments found in Debug/Makefile")

violations = []
c_recipes = 0
for rel in frags:
    p = DEBUG / rel
    text = p.read_text(encoding="utf-8")
    if "SONDE_FLIGHT_BUILD" in text:
        violations.append(f"{rel}: carries SONDE_FLIGHT_BUILD (source mutation)")
    for line in text.splitlines():
        # C compile recipes: the only lines carrying -DCORE_CM4.
        if "-DCORE_CM4" not in line:
            continue
        c_recipes += 1
        if "$(PROJECT_CPPFLAGS)" not in line:
            violations.append(f"{rel}: C recipe omits $(PROJECT_CPPFLAGS)")

if c_recipes == 0:
    sys.exit("FATAL: no C compile recipes found - checker is blind")

if violations:
    print("PROJECT_CPPFLAGS gate: FAIL")
    for v in violations:
        print(f"  {v}")
    sys.exit(1)

print(f"PROJECT_CPPFLAGS gate: OK "
      f"({c_recipes} C recipes across {len(frags)} active fragments, "
      f"no injected flight macros)")
