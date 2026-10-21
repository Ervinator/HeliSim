# -*- coding: utf-8 -*-
"""Scratch: the module-level names a scratch harness binds more than once.

``_rt_pilot`` carries three missions in one module namespace, and a constant
written down twice is bound once - the last one wins, and everything above it
reads the name the section below meant for itself.  That is exactly how the
long mission's speed ramp stopped being asked for: ``CLIMB_ALT_M`` is its own
eighty metres at the top of the file and the engine-off mission's five hundred
feet further down, so the long mission's ramp was solved against a height it
never reaches.  This walks the module's own syntax tree and prints every name
assigned at module level more than once, with the lines, so the shadowing is
read off the file rather than hunted for.

Usage: ``python _tmp_shadow.py _rt_pilot.py``
"""

import ast
import sys
import collections


def module_level_bindings(path):
    """``name -> [(lineno, source)]`` for every module-level assignment."""
    with open(path, encoding="utf-8-sig") as handle:
        source = handle.read()
    tree = ast.parse(source, filename=path)
    lines = source.splitlines()
    found = collections.OrderedDict()
    for node in tree.body:
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        for target in targets:
            for name in ast.walk(target):
                if isinstance(name, ast.Name):
                    found.setdefault(name.id, []).append(
                        (node.lineno, lines[node.lineno - 1].strip()))
    return found


def main():
    """Every duplicated module-level name in the files named, or none."""
    for path in sys.argv[1:]:
        found = module_level_bindings(path)
        duplicates = [(name, where) for name, where in found.items()
                      if len(where) > 1]
        print("%s: %d module-level names, %d written down more than once"
              % (path, len(found), len(duplicates)))
        for name, where in duplicates:
            print("  %s" % (name,))
            for lineno, line in where:
                print("    %5d  %s" % (lineno, line))
        if not duplicates:
            print("  none")


if __name__ == "__main__":
    main()
