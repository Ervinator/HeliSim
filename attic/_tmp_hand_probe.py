# -*- coding: utf-8 -*-
"""Scratch: run one of `_rt_autoro_hand`'s own entry points, report to a file.

The console on this box is not to be trusted for a traceback, so this catches
whatever the run raises and writes it beside the transcripts.

Moved out of the root on 2026-10-22, so it is run from the project root as
``python attic/_tmp_hand_probe.py ruler|core|sweep|full``, and the three
transcripts it leaves (``_tmp_ruler.txt``, ``_tmp_core.txt``, ``_tmp_sweep.txt``
and the ``_tmp_full.txt`` the last mode leaves) still land beside the others.
"""

import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import _rt_autoro_hand as hand

which = sys.argv[1]
try:
    if which == "ruler":
        stream = open("_tmp_ruler.txt", "w", encoding="utf-8")
        hand.same_ruler(stream)
        stream.close()
        report = "ruler: OK\n"
    elif which == "core":
        hand.probe(reactions=(hand.PILOT_PERIOD_S,), hesitations=(), band=False,
                   out_path="_tmp_core.txt")
        report = "core: OK\n"
    elif which == "sweep":
        hand.probe(reactions=(hand.PILOT_PERIOD_S, 0.10, hand.DELAY_S),
                   hesitations=(), band=False, out_path="_tmp_sweep.txt")
        report = "sweep: OK\n"
    else:
        hand.probe(out_path="_tmp_full.txt")
        report = "full: OK\n"
except Exception:
    report = "FAILED\n" + traceback.format_exc()
open("_tmp_probe_out.txt", "w", encoding="utf-8").write(report)
