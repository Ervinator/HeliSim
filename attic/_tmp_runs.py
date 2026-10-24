# -*- coding: utf-8 -*-
"""Scratch: launch a long run detached, so nothing in this terminal's own
session can take it away mid-flight, and let the caller poll the files.

Every run is a script and that script's own arguments, named the way it is run
from the project root::

    python _tmp_runs.py attic/_tmp_sweep.py out=_tmp_a.txt cells 250,2.5,6.5,0.6

Everything the run prints goes to ``log_<tag>.txt`` - the ``out=`` file's name
if the arguments carry one, the script's own otherwise - and the transcript
itself is the script's to write, so a caller polls that rather than a terminal.
"""

import os
import subprocess
import sys

DETACHED = 0x00000008          # DETACHED_PROCESS: no console to be killed with


def tag_of(args):
    """What the run's log is named for: its ``out=`` file, or its script."""
    for arg in args:
        if arg.startswith("out="):
            return os.path.splitext(os.path.basename(arg[4:]))[0]
    return os.path.splitext(os.path.basename(args[0]))[0]


def main(argv):
    """Launch ``argv`` with no console of its own, and say where it writes."""
    if not argv:
        print("usage: python attic/_tmp_runs.py SCRIPT [ARG ...]")
        return 2
    log_path = "log_%s.txt" % (tag_of(argv),)
    with open(log_path, "w", encoding="utf-8") as log:
        subprocess.Popen([sys.executable] + list(argv), stdout=log,
                         stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                         creationflags=DETACHED)
    print("launched: %s -> %s" % (" ".join(argv), log_path))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
