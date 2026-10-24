# -*- coding: utf-8 -*-
"""Scratch: the syntax of a file, and the lines around any problem, into a file.

The console on this box is not to be trusted for a traceback - the shell echoes
its own stale buffer - so this writes what it found beside the file it read.

Moved out of the root on 2026-10-22: run it from the project root as
``python attic/_tmp_syntax.py _rt_autoro_hand.py``, and it still writes
``_tmp_syntax.txt`` beside the file it read.
"""

import ast
import sys

src = open(sys.argv[1], encoding="utf-8").read()
lines = src.splitlines()
try:
    ast.parse(src, filename=sys.argv[1])
    report = "SYNTAX OK: %d lines\n" % (len(lines),)
except SyntaxError as problem:
    report = "SyntaxError: line %s: %s\n" % (problem.lineno, problem.msg)
    low = max(problem.lineno - 5, 1)
    high = min(problem.lineno + 3, len(lines))
    for number in range(low, high + 1):
        report += "%6d| %s\n" % (number, lines[number - 1])
open("_tmp_syntax.txt", "w", encoding="utf-8").write(report)
