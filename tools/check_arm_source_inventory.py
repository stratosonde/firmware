#!/usr/bin/env python3
"""ARM build source-inventory checker (pretest-hardening handoff 2026-08-15,
Phase 1 / PIPE-05).

Proves, from the build files themselves (not prose):
  1. Debug/Makefile's -include'd subdir.mk set == sources.mk SUBDIRS set
     (CubeMX keeps the two in sync; drift here means a directory silently
     dropped out of or into the ARM build).
  2. Every subdir.mk present under Debug/ is active (no stale force-added
     fragments like the pre-prune Debug/archive/** and Drivers/SHT31 ones).
  3. The C sources named by the active fragments contain no archive,
     reference, test, or example code.
  4. No C source is compiled twice.
  5. Every first-party production .c (Core/Src, LoRaWAN/App, plus the
     Drivers/STM32WLxx HAL Src actually used) appears exactly once.

Run from anywhere:  python3 tools/check_arm_source_inventory.py
Exit 0 with a summary on success; exit 1 naming every violation.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEBUG = os.path.join(ROOT, "Debug")

# First-party production source directories whose .c files must ALL be in the
# ARM build exactly once. Vendor trees (Middlewares, Utilities, Drivers/CMSIS)
# contribute whatever the active fragments name and are not filesystem-checked.
PRODUCTION_DIRS = ["Core/Src", "LoRaWAN/App"]

# Production .c files legitimately absent from the ARM build (none today;
# add with a reason if one ever appears, e.g. a host-only tool).
ALLOW_MISSING = {}

FORBIDDEN_PATTERNS = [
    (r"/archive/", "archive fragment"),
    (r"/Reference/", "reference fragment"),
    (r"[Rr]eference/", "reference fragment"),
    (r"tests?/", "test code"),
    (r"example", "example code"),
    (r"template", "template code"),
]


def read(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def backslash_block(text, start_marker):
    """Return the continuation-line block following start_marker."""
    i = text.index(start_marker) + len(start_marker)
    lines = []
    for line in text[i:].splitlines():
        line = line.strip()
        if not line:
            break
        cont = line.endswith("\\")
        lines.append(line.rstrip("\\").strip())
        if not cont:
            break
    return lines


def main():
    failures = []

    mk_path = os.path.join(DEBUG, "Makefile")
    sources_path = os.path.join(DEBUG, "sources.mk")
    mk = read(mk_path)
    sources = read(sources_path)

    # 1. Active fragment dirs from Debug/Makefile -include lines.
    include_dirs = sorted(
        os.path.dirname(m)
        for m in re.findall(r"(?m)^-include ([^\s]+/subdir\.mk)$", mk)
    )
    subdirs = sorted(x for x in backslash_block(sources, "SUBDIRS :=") if x)
    if include_dirs != subdirs:
        failures.append(
            "Makefile -include set != sources.mk SUBDIRS set:\n"
            "  only in -include: %s\n  only in SUBDIRS: %s"
            % (sorted(set(include_dirs) - set(subdirs)),
               sorted(set(subdirs) - set(include_dirs))))

    # 2. Every subdir.mk under Debug/ must be active.
    active_set = set(include_dirs)
    for dirpath, _dirnames, filenames in os.walk(DEBUG):
        if "subdir.mk" in filenames:
            rel = os.path.relpath(dirpath, DEBUG).replace(os.sep, "/")
            if rel not in active_set:
                failures.append("inactive subdir.mk present: Debug/%s/subdir.mk" % rel)

    # 3+4. Collect C sources from active fragments.
    c_sources = []
    for d in include_dirs:
        frag = read(os.path.join(DEBUG, d, "subdir.mk"))
        if "C_SRCS +=" not in frag:
            continue  # assembler-only fragment (e.g. Core/Startup)
        for entry in backslash_block(frag, "C_SRCS +="):
            if entry.endswith(".c"):
                c_sources.append(entry)
    for s in c_sources:
        for pat, why in FORBIDDEN_PATTERNS:
            if re.search(pat, s):
                failures.append("forbidden %s in ARM build: %s" % (why, s))
    seen = {}
    for s in c_sources:
        seen[s] = seen.get(s, 0) + 1
    for s, n in sorted(seen.items()):
        if n > 1:
            failures.append("compiled %d times: %s" % (n, s))

    # 5. Production coverage, keyed by basename-insensitive repo-relative path.
    build_repo_paths = set()
    for s in c_sources:
        # Fragment paths are relative to Debug/ and start with ../
        build_repo_paths.add(os.path.normpath(os.path.join("Debug", s)).replace(os.sep, "/"))
    for prod in PRODUCTION_DIRS:
        abs_prod = os.path.join(ROOT, prod)
        for name in sorted(os.listdir(abs_prod)):
            if not name.endswith(".c"):
                continue
            rel = "%s/%s" % (prod, name)
            if rel in ALLOW_MISSING:
                continue
            if rel not in build_repo_paths:
                failures.append("production source missing from ARM build: %s" % rel)

    print("ARM source inventory @ %s" % ROOT)
    print("  active fragments: %d" % len(include_dirs))
    print("  C sources in build: %d (unique: %d)" % (len(c_sources), len(seen)))
    if failures:
        print("FAIL")
        for f in failures:
            print("  - %s" % f)
        return 1
    print("PASS: every production .c compiled exactly once; no "
          "archive/reference/test/example sources; no stale fragments")
    return 0


if __name__ == "__main__":
    sys.exit(main())
