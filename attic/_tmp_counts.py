# -*- coding: utf-8 -*-
"""Scratch: every line count the docs claim, read back off the files themselves.

`README.md` carries the model's nine modules and the harness's nine in two
`| module | lines |` tables, and `TODO.md` and this directory's own README name a
file's count in prose - `` `name` (N lines) ``.  Every one of those is a number a
hand wrote down, and on 2026-10-22 one of them moved in three places at once,
which is the drift this reads back.

Run from the project root:

    python attic/_tmp_counts.py

One line per claim, `OK` or the two numbers that disagree, and exit 1 if any of
them does - so a commit can be gated on it.  It asserts the other direction too:
every `.py` in the root has a row in one of the two tables (`_tmp_*.py` excepted,
since those are scratch), and every `python x.py` line in the run list names a
file that is there.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ATTIC = os.path.join(ROOT, "attic")

TABLE_ROW = re.compile(r"^\| `([A-Za-z_0-9]+\.py)` \| (\d+) \|")
CLAIM = re.compile(r"`([A-Za-z_0-9]+\.py)`\*{0,2} \((\d+) lines\)")
RUN_LINE = re.compile(r"^\s*python ([A-Za-z_0-9]+\.py)")


def read_lines(name):
    """`name`'s lines, read out of the project root."""
    with open(os.path.join(ROOT, name), encoding="utf-8") as source:
        return source.read().splitlines()


def count_of(name):
    """What `name` is on disk - the root first, `attic/` after it - or None."""
    for folder in (ROOT, ATTIC):
        path = os.path.join(folder, name)
        if os.path.exists(path):
            with open(path, encoding="utf-8") as source:
                return len(source.read().splitlines())
    return None


class Report(object):
    """The claims checked, and how many of them disagreed."""

    def __init__(self):
        self.checked = 0
        self.problems = 0

    def claim(self, where, name, claimed):
        """One `(N lines)` claim, against the file it names."""
        actual = count_of(name)
        self.checked += 1
        if actual == claimed:
            print("OK        %-18s %-24s %d" % (where, name, claimed))
            return
        self.problems += 1
        if actual is None:
            print("MISSING   %-18s %-24s claimed %d" % (where, name, claimed))
        else:
            print("MISMATCH  %-18s %-24s doc says %d, file has %d"
                  % (where, name, claimed, actual))

    def unnamed(self, where, name):
        """Something the docs should name and do not."""
        self.problems += 1
        print("UNNAMED   %-18s %-24s" % (where, name))


def main():
    """Read every claim in the three documents, and print the disagreements."""
    report = Report()
    readme = read_lines("README.md")
    named = set()
    for number, line in enumerate(readme, 1):
        row = TABLE_ROW.match(line)
        if row:
            named.add(row.group(1))
            report.claim("README.md:%d" % number, row.group(1),
                         int(row.group(2)))
    for document in ("README.md", "TODO.md", "attic/README.md"):
        for number, line in enumerate(read_lines(document), 1):
            for found in CLAIM.finditer(line):
                report.claim("%s:%d" % (document, number), found.group(1),
                             int(found.group(2)))
    for name in sorted(os.listdir(ROOT)):
        if name.endswith(".py") and not name.startswith("_tmp_"):
            if name not in named:
                report.unnamed("README.md table", name)
    for number, line in enumerate(readme, 1):
        run = RUN_LINE.match(line)
        if run and count_of(run.group(1)) is None:
            report.unnamed("README.md:%d" % number, run.group(1))
    print("%d claims, %d disagreeing" % (report.checked, report.problems))
    return 1 if report.problems else 0


if __name__ == "__main__":
    sys.exit(main())
