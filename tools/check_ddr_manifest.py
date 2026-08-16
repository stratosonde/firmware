#!/usr/bin/env python3
"""R3-11 (#223): validate DDR identity integrity against the canonical manifest.

Checks (hard failures):
  1. every docs/decisions/NNNN-*.md file appears exactly once in the manifest
  2. filename number == manifest id number
  3. each file's first '# DDR-XXXX:' heading matches its id
  4. manifest ids are unique; every manifest file exists
  5. every README.md V2-table row resolves to a manifest id
  6. every DDR-XXXX token in source (Core/Src, Core/Inc, LoRaWAN/App) and in
     docs/decisions/*.md resolves to a canonical id, OR a declared alias

Ambiguity policy: retired 2026-08-15 (DOC-01). The 0014-0021 both-generations
band no longer exists (audit found every in-repo reference means the canonical
generation; see manifest.yaml). Any reference to a still-declared retired
alias (0028-0034), and any ambiguity should one ever reappear, FAILS the gate.

Exit 0 = pass with 0 failures AND 0 warnings; 1 otherwise.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DECISIONS = os.path.join(ROOT, "docs", "decisions")
MANIFEST = os.path.join(DECISIONS, "manifest.yaml")
README = os.path.join(DECISIONS, "README.md")
SRC_DIRS = [os.path.join(ROOT, d) for d in ("Core/Src", "Core/Inc", "LoRaWAN/App")]

failures = 0
warnings = 0

def fail(msg):
    global failures
    failures += 1
    print("FAIL: " + msg)

def warn(msg):
    global warnings
    warnings += 1
    print("warn: " + msg)

def parse_manifest(path):
    records = {}
    aliases = {}
    section = None
    entry_re = re.compile(r"^\s*-\s*\{(.*)\}\s*$")
    kv_re = re.compile(r"(\w+):\s*([^,}]+)")
    with open(path, encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if s.startswith("#") or not s:
                continue
            if s in ("records:", "aliases:"):
                section = s[:-1]
                continue
            m = entry_re.match(line)
            if not m:
                continue
            kv = dict((k, v.strip()) for k, v in kv_re.findall(m.group(1)))
            if section == "records":
                rid = kv.get("id", "")
                if rid in records:
                    fail("manifest: duplicate id " + rid)
                records[rid] = kv
            elif section == "aliases":
                aliases[kv.get("alias", "")] = kv.get("of", "")
    return records, aliases

def main():
    records, aliases = parse_manifest(MANIFEST)
    if not records:
        fail("manifest: no records parsed")
        return 1

    # 1-3: files <-> manifest, filename/id agreement, title heading agreement
    files = sorted(f for f in os.listdir(DECISIONS)
                   if re.match(r"^\d{4}-.+\.md$", f))
    manifest_files = {}
    for rid, kv in records.items():
        fn = kv.get("file", "")
        if fn in manifest_files:
            fail("manifest: duplicate file " + fn)
        manifest_files[fn] = rid
        if not os.path.isfile(os.path.join(DECISIONS, fn)):
            fail("manifest: %s lists missing file %s" % (rid, fn))
        m = re.match(r"^DDR-(\d{4})$", rid)
        if not m or not fn.startswith(m.group(1) + "-"):
            fail("manifest: id %s disagrees with filename %s" % (rid, fn))

    for fn in files:
        if fn not in manifest_files:
            fail("file %s missing from manifest" % fn)
            continue
        rid = manifest_files[fn]
        with open(os.path.join(DECISIONS, fn), encoding="utf-8") as f:
            head = f.read(4000)
        hm = re.search(r"^#\s*DDR-(\d{4})", head, re.M)
        if not hm:
            fail("%s: no '# DDR-XXXX' heading found" % fn)
        elif "DDR-" + hm.group(1) != rid:
            fail("%s: heading DDR-%s disagrees with manifest id %s"
                 % (fn, hm.group(1), rid))

    # 5: README V2 table rows resolve
    with open(README, encoding="utf-8") as f:
        readme = f.read()
    for row in re.findall(r"^\|\s*(\d{4})\s*\|", readme, re.M):
        if "DDR-" + row not in records:
            fail("README table row %s does not resolve to a manifest record" % row)

    # 6: DDR-XXXX references resolve (canonical or alias; overlap = ambiguous)
    token_re = re.compile(r"DDR-(\d{4})")
    refs = []
    for base in SRC_DIRS:
        for dirpath, _dirs, names in os.walk(base):
            for name in names:
                if name.endswith((".c", ".h")):
                    refs.append(os.path.join(dirpath, name))
    refs += [os.path.join(DECISIONS, f) for f in files] + [README]

    for path in refs:
        with open(path, encoding="utf-8", errors="replace") as f:
            text = f.read()
        for tok in sorted(set(token_re.findall(text))):
            rid = "DDR-" + tok
            in_records = rid in records
            in_aliases = rid in aliases
            rel = os.path.relpath(path, ROOT)
            if in_records and in_aliases:
                warn("%s: %s is AMBIGUOUS (exists in both numbering generations)"
                     % (rel, rid))
            elif in_records:
                pass
            elif in_aliases:
                warn("%s: %s is a retired alias of %s" % (rel, rid, aliases[rid]))
            else:
                fail("%s: reference %s resolves to nothing" % (rel, rid))

    print("\nDDR manifest check: %d records, %d aliases, %d failures, %d warnings"
          % (len(records), len(aliases), failures, warnings))
    return 1 if (failures or warnings) else 0

if __name__ == "__main__":
    sys.exit(main())
