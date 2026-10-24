# -*- coding: utf-8 -*-
"""Scratch: write a key log to disk, read it back and check it.

Not part of the project, and not imported by anything.  `main.py --check` asserts
the log in memory - the columns, the two clocks, the rate, the names - but not
`KeyLog.open`, which is what actually writes a file, and not a flight long enough
for the sample rate to say anything.  This drives `main.py`'s own frame loop
headless through a pickup from the pad, with W pressed and then released, writes
``_keylog_check.csv`` beside itself, reads the rows back and prints them, and
deletes the file it made.  Run from the project root:

    python attic/_keylog_check.py

Needs no window and no OpenGL context: the keyboard is a dict of scancodes,
`main.fly_frame` is the whole of the frame loop, and the flight is 240 frames of
1/60 s.  What it checks is what a log is for - a line carries the aircraft and the
hand at a time on two clocks, both to a millisecond and both the same instant, the
samples come at their 20 Hz rate, and the pickup from the pad is on the trace.
"""

import io
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pygame

import main
from simulation import SIM_TIME_STEP_S, Simulation, airframe_preset

#: Where this writes: beside itself, and removed again before it returns.
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "_keylog_check.csv")

#: The frames each half of the flight is flown for: two seconds of W, two without.
HOLD_FRAMES = 120

#: A frame time a script can reach, so that the 20 Hz samples fall between frames
#: the way they do in a real window rather than on every one of them.
FRAME_SLEEP_S = 0.004


def key_event(event_type, key, scancode):
    """A pygame key event, for the two lines of a keystroke in the log."""
    return pygame.event.Event(event_type, key=key, scancode=scancode)


def fly(sim, camera, log, keyboard, frames):
    """*frames* of main.py's own frame loop with *keyboard* held."""
    for _ in range(frames):
        main.fly_frame(sim, camera, SIM_TIME_STEP_S,
                       main.frame_keys(keyboard, set()))
        log.sample(sim, main.held_names(keyboard))
        time.sleep(FRAME_SLEEP_S)


def read_back(path):
    """``(header lines, rows)`` of a log file: its comments and its lines."""
    raw = io.open(path, encoding="utf-8").read().splitlines()
    lines = [line for line in raw if line and not line.startswith("#")]
    columns = lines[0].split(",")
    return ([line for line in raw if line.startswith("#")],
            [dict(zip(columns, line.split(","))) for line in lines[1:]])


def check():
    sim = Simulation(airframe=airframe_preset(main.AIRFRAME_PRESET))
    sim.on_the_pad()
    camera = main.ChaseCamera()
    log = main.KeyLog.open(LOG_PATH, rate_hz=main.KEYLOG_RATE_HZ)
    print("key log: %s" % LOG_PATH)

    keyboard = {}
    log.sample(sim, main.held_names(keyboard))       # the run's starting state
    keyboard = {pygame.KSCAN_W: True}
    main.log_key(log, sim, "down",
                 key_event(pygame.KEYDOWN, pygame.K_w, pygame.KSCAN_W), keyboard)
    fly(sim, camera, log, keyboard, HOLD_FRAMES)
    keyboard = {}
    main.log_key(log, sim, "up",
                 key_event(pygame.KEYUP, pygame.K_w, pygame.KSCAN_W), keyboard)
    fly(sim, camera, log, keyboard, HOLD_FRAMES)
    log.close()

    comments, rows = read_back(LOG_PATH)
    kinds = {}
    for row in rows:
        kinds[row["kind"]] = kinds.get(row["kind"], 0) + 1
    print("header lines: %d | rows: %d | kinds: %s"
          % (len(comments), len(rows), kinds))

    assert list(rows[0]) == list(main.KeyLog.COLUMNS)
    assert kinds == {"sample": len(rows) - 2, "down": 1, "up": 1}, kinds
    assert len(rows) > 40, len(rows)                 # four seconds of flight

    samples = [row for row in rows if row["kind"] == "sample"]
    times = [float(row["t_s"]) for row in samples]
    walls = [float(row["wall_ms"]) for row in samples]
    assert times == sorted(times) and walls == sorted(walls)   # both monotonic
    assert all(len(row["t_s"].split(".")[-1]) == 3 for row in rows)
    assert all(len(row["wall_ms"].split(".")[-1]) == 3 for row in rows)
    assert abs(walls[0] - log.origin_wall * 1000.0 - 1000.0 * times[0]) < 2.0
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert min(gaps) > 1.0 / main.KEYLOG_RATE_HZ - 0.001, min(gaps)
    print("t %5.3f..%5.3f s | sample gap %5.3f..%5.3f s | wall clock %.3f"
          % (times[0], times[-1], min(gaps), max(gaps), walls[-1] / 1000.0))

    down = [row for row in rows if row["kind"] == "down"][0]
    up = [row for row in rows if row["kind"] == "up"][0]
    assert down["key"] == "W" and down["held"] == "W"
    assert up["key"] == "W" and up["held"] == "-"
    print("down at frame %s: %s" % (down["frame"], down))
    print("up   at frame %s: %s" % (up["frame"], up))

    alts = [float(row["alt_m"]) for row in samples]
    colls = [float(row["coll_frac"]) for row in samples]
    vds = [float(row["vd_mps"]) for row in samples]
    assert alts[0] < 5.0                     # the run begins on the pad
    assert max(alts) > alts[0] + 0.5         # and W lifts it off the skids
    assert colls[-1] > colls[0]
    assert min(vds) < 0.0                    # climbing: vd is down positive
    assert all(float(row["spd_mps"]) >= 0.0 for row in samples)
    print("alt %5.2f -> %5.2f m | coll %.3f -> %.3f | vd min %+5.2f m/s"
          % (alts[0], alts[-1], colls[0], colls[-1], min(vds)))
    print("the last sample: %s" % (samples[-1],))

    os.remove(LOG_PATH)
    print("_keylog_check.csv written, checked and removed: passed")


if __name__ == "__main__":
    check()
