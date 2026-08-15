#!/usr/bin/env python3
"""Categorize a firmware crate's *own* compiler warnings, by lint and by file.

The gate script's totals count every crate in the dependency graph, so a firmware's
own warnings are a minority of its log and move whenever a dependency changes. This
reports only diagnostics whose primary span is inside the named crate, which is the
number worth holding steady.

`--substantive` drops `unused_imports`, which is ~70% of both firmwares' warnings and
is pure tidiness -- `cargo fix` clears it. What remains is where the signal is: dropped
Results, never-read fields, bindings that may mean an unwired feature.

Usage:
    cargo build -p <crate> --target thumbv8m.main-none-eabihf --message-format=json > w.json
    scripts/warning-report.py <crate> w.json [--substantive]

Note cargo only emits diagnostics for crates it actually recompiles; `touch` the
crate's `src/main.rs` first or the report comes back empty.
"""
import json
import sys
from collections import Counter, defaultdict

args = [a for a in sys.argv[1:] if not a.startswith("--")]
substantive = "--substantive" in sys.argv
if len(args) < 2:
    sys.exit(__doc__)

crate, logs = args[0], args[1:]
SKIP = {"unused_imports"} if substantive else set()

by_lint = Counter()
by_file = Counter()
sites = defaultdict(list)

for path in logs:
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line.startswith("{"):
                continue
            msg = json.loads(line)
            if msg.get("reason") != "compiler-message":
                continue
            d = msg["message"]
            if d.get("level") != "warning":
                continue
            spans = [s for s in d.get("spans", []) if s.get("is_primary")]
            if not spans or not spans[0]["file_name"].startswith(crate + "/"):
                continue
            code = (d.get("code") or {}).get("code") or "(uncoded)"
            if code in SKIP:
                continue
            f = spans[0]["file_name"].split("/", 1)[1]
            by_lint[code] += 1
            by_file[f] += 1
            sites[code].append((f, spans[0]["line_start"], d["message"]))

total = sum(by_lint.values())
scope = "non-import warnings" if substantive else "warnings"
print(f"=== {crate}: {total} {scope} in the crate's own sources\n")

print("By lint:")
for code, n in by_lint.most_common():
    print(f"  {n:4d}  {code}")

print("\nBy file:")
for f, n in by_file.most_common():
    print(f"  {n:4d}  {f}")

print("\nSites:")
for code, _ in by_lint.most_common():
    print(f"\n[{code}]  ({len(sites[code])})")
    for f, line, m in sorted(sites[code], key=lambda t: (t[0], t[1])):
        print(f"  {f}:{line}  {m}")
