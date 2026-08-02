#!/usr/bin/env python3
"""Task 16, Step 2 companion: put a single call site back on `defmt`.

Some arguments implement `defmt::Format` but not the `core::fmt` trait the `log`
half of `log_*!` needs (`Display` for `{}`, `Debug` for `{:?}`). Those sites are
left on `defmt` rather than worked around -- see the task report for the list.

Rewrites `log_X!(` back to `defmt::X!(` on the given lines. Fully qualified so no
`use` has to be reinstated.

Usage: scripts/task16-revert-site.py <file>:<line> [...]
"""

import re
import sys
from collections import defaultdict

CALL = re.compile(r"(?<![:\w])log_(error|warn|info|debug|trace)!\(")


def main(argv):
    by_file = defaultdict(set)
    for arg in argv:
        path, line = arg.rsplit(":", 1)
        by_file[path].add(int(line))

    for path, lines in sorted(by_file.items()):
        with open(path) as f:
            src = f.readlines()
        n = 0
        for ln in sorted(lines):
            old = src[ln - 1]
            new, count = CALL.subn(lambda m: f"defmt::{m.group(1)}!(", old)
            src[ln - 1] = new
            n += count
        with open(path, "w") as f:
            f.writelines(src)
        print(f"{path}: reverted {n} site(s) on {len(lines)} line(s)")


if __name__ == "__main__":
    main(sys.argv[1:])
