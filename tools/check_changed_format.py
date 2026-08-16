#!/usr/bin/env python3
"""Changed-lines clang-format HARD GATE (owner decision 2026-08-16: 2-space
LLVM, see .clang-format).

For every first-party C source changed vs the merge base, the ADDED line
ranges must be clang-format-clean. Whole-file gating was rejected: CubeMX
regen owns several files and must never be reformat-checked.

Uses `clang-format -lines=a:b` so only changed ranges are judged. Binary
resolution order: $CLANG_FORMAT, clang-format on PATH, the pip
clang-format package. Exit 1 if any changed range would be reformatted.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATHS = ["Core/Src", "Core/Inc", "LoRaWAN/App"]
SKIP_BASENAMES = {"se-identity.h"}  # gitignored real keys; the template is gated


def find_clang_format():
    exe = os.environ.get("CLANG_FORMAT")
    if exe and os.path.isfile(exe):
        return exe
    for name in ("clang-format.exe", "clang-format"):
        for d in os.environ.get("PATH", "").split(os.pathsep):
            cand = os.path.join(d, name)
            if os.path.isfile(cand):
                return cand
    try:
        import clang_format
        cand = os.path.join(os.path.dirname(clang_format.__file__),
                            "data", "bin",
                            "clang-format.exe" if os.name == "nt" else "clang-format")
        if os.path.isfile(cand):
            return cand
    except ImportError:
        pass
    return None


def changed_ranges(base):
    out = subprocess.check_output(
        ["git", "diff", "-U0", base, "--"] + PATHS, cwd=ROOT,
        encoding="utf-8", errors="replace")
    ranges = {}
    cur = None
    for line in out.splitlines():
        m = re.match(r"^\+\+\+ b/(.+)$", line)
        if m:
            cur = m.group(1)
            continue
        m = re.match(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@", line)
        if m and cur:
            start = int(m.group(1))
            count = int(m.group(2) or "1")
            if count > 0:  # pure deletions have no lines to format
                ranges.setdefault(cur, []).append((start, start + count - 1))
    return ranges


def main():
    base = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else "HEAD~1"
    cf = find_clang_format()
    if not cf:
        print("clang-format gate: no clang-format binary found "
              "(pip install clang-format) - FAIL")
        return 1
    ranges = changed_ranges(base)
    if not ranges:
        print("clang-format gate: no first-party C changes vs %s" % base)
        return 0
    bad = 0
    for rel, spans in sorted(ranges.items()):
        if os.path.basename(rel) in SKIP_BASENAMES:
            continue
        path = os.path.join(ROOT, rel)
        args = [cf, "--dry-run", "--Werror"]
        for a, b in spans:
            args.append("--lines=%d:%d" % (a, b))
        args.append(path)
        r = subprocess.run(args, capture_output=True, text=True)
        if r.returncode != 0:
            bad += 1
            print("clang-format gate: %s would be reformatted:" % rel)
            for l in (r.stderr or "").splitlines()[:6]:
                print("   " + l)
    if bad:
        print("clang-format gate: %d file(s) fail (run clang-format -i on them)" % bad)
        return 1
    print("clang-format gate: changed lines clean vs %s (%d file(s) checked)"
          % (base, len(ranges)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
