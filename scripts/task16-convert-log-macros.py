#!/usr/bin/env python3
"""Task 16, Step 2: mechanically move `defmt` log calls onto `variegated_log`'s
`log_*!` macros so they reach both the probe and the debug bus.

The substitution is deliberately dumb so the diff stays reviewable:

  * `use defmt::{info, warn, Format};`  ->  `use defmt::Format;`
                                           `use variegated_log::{log_info, log_warn};`
    (rewritten in place, so a `use` inside a function keeps its scope)
  * bare      `info!(`                  ->  `log_info!(`
  * qualified `defmt::info!(`           ->  `variegated_log::log_info!(`
    (kept fully qualified rather than relying on an import being in scope)

Nothing else changes: no message is reworded, no format argument is touched.
Lines that are entirely a `//` comment are left alone.

Usage: scripts/task16-convert-log-macros.py [--check] <file>...
"""

import re
import sys

LEVELS = ("error", "warn", "info", "debug", "trace")

USE_DEFMT = re.compile(r"^(?P<indent>\s*)use defmt::(?P<body>\{[^}]*\}|[A-Za-z_][A-Za-z0-9_]*)\s*;\s*$")
BARE_CALL = re.compile(r"(?<![:\w])(" + "|".join(LEVELS) + r")!\(")
QUAL_CALL = re.compile(r"\bdefmt::(" + "|".join(LEVELS) + r")!\(")


def is_comment(line):
    return line.lstrip().startswith("//")


def convert(text):
    out = []
    changed = 0
    for line in text.splitlines(keepends=True):
        m = USE_DEFMT.match(line)
        if m:
            body = m.group("body")
            items = [i.strip() for i in body.strip("{}").split(",") if i.strip()]
            levels = [i for i in items if i in LEVELS]
            if levels:
                rest = [i for i in items if i not in LEVELS]
                indent, nl = m.group("indent"), "\n"
                new = ""
                if rest:
                    inner = rest[0] if len(rest) == 1 else "{" + ", ".join(rest) + "}"
                    new += f"{indent}use defmt::{inner};{nl}"
                logs = [f"log_{l}" for l in levels]
                inner = logs[0] if len(logs) == 1 else "{" + ", ".join(logs) + "}"
                new += f"{indent}use variegated_log::{inner};{nl}"
                out.append(new)
                changed += 1
                continue
        if is_comment(line):
            out.append(line)
            continue
        new = QUAL_CALL.sub(lambda m: f"variegated_log::log_{m.group(1)}!(", line)
        new = BARE_CALL.sub(lambda m: f"log_{m.group(1)}!(", new)
        if new != line:
            changed += len(QUAL_CALL.findall(line)) + len(BARE_CALL.findall(line))
        out.append(new)
    return "".join(out), changed


def main(argv):
    check = "--check" in argv
    paths = [a for a in argv if not a.startswith("--")]
    total = 0
    for path in paths:
        with open(path) as f:
            text = f.read()
        new, n = convert(text)
        total += n
        if n and not check:
            with open(path, "w") as f:
                f.write(new)
        print(f"{path}: {n}")
    print(f"total: {total}")


if __name__ == "__main__":
    main(sys.argv[1:])
