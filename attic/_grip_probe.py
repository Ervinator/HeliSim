# -*- coding: utf-8 -*-
"""Scratch: where the skids stop holding the heading.

Walks the collective from the down stop up to the up stop with the right pedal
hard against its own stop, two seconds at each lever, and reports what the floor
does with the heading: the mixing law's ``collective_deg`` the gate reads, the
heading the aircraft holds after two seconds of pedal, and the yaw rate the
frame was left with.  The gate is ``simulation.GROUND_GRIP_ANGLE_DEG``: under it
the heading must not move at all - not even the walk the Euler kinematics give a
frame whose yaw rate is zero - and above it the pedal must still turn the
aircraft on its skids.  The same boundary is asserted in `simulation.py`'s own
self test; this is the walk that found it.  Run from the project root:

    python attic/_grip_probe.py

A lever named on the command line is walked instead of the default list, and
``out=FILE`` puts the transcript in a file so the console is not needed - which
is how the pair either side of the gate was read:

    python attic/_grip_probe.py out=attic/_grip.txt 0.252866 0.254866
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation import (GROUND_GRIP_ANGLE_DEG, PAD_COLLECTIVE, SIM_TIME_STEP_S,
                        Simulation, UH1_PEDAL_TRAVEL_IN)

#: The levers to walk: the down stop, the numbers the gate's own docstring names
#: as a run up, the gate itself twice over, the pad, and the top of the travel.
FRACTIONS = (0.0, 0.02, 0.05, 0.08, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40,
             0.50, 0.60, PAD_COLLECTIVE, 0.90, 1.0)

#: Two seconds of pedal, the self-test's own hold.
HELD_S = 2.0

stream = None


def say(text):
    print(text)
    if stream is not None:
        stream.write(text + "\n")
        stream.flush()


def row(fraction):
    """One lever: the held-pedal two seconds, and what the floor made of it."""
    run = Simulation()
    run.on_the_pad(collective=fraction)
    run.pilot.set_axes(pedal=1.0)               # a script's hand, at the stop
    controls = run.pilot.controls()
    started = run.telemetry()
    gripped = started.collective_deg < GROUND_GRIP_ANGLE_DEG
    steps = int(HELD_S / SIM_TIME_STEP_S)
    for _ in range(steps):
        run.step(SIM_TIME_STEP_S, controls)
    held = run.telemetry()
    walk = held.yaw_deg - started.yaw_deg
    say("  %5.3f  %8.3f  %-7s  %+10.6f  %+9.4f  %+8.3f  %-8s  %s"
        % (fraction, started.collective_deg, "yes" if gripped else "no",
           walk, held.yaw_rate_deg, run.airframe.state.rates.z,
           "on" if held.on_ground else "off",
           "crashed" if run.crashed else "-"))


def main(argv):
    """The command line: where the rows go, and nothing else."""
    global stream
    args = list(argv)
    if args and args[0].startswith("out="):
        stream = open(args[0][4:], "w", encoding="utf-8")
        args = args[1:]
    say("skids hold the heading under %.2f deg of rotor; the pedal is at its"
        " stop (%.2f in of %.2f) and held %g s"
        % (GROUND_GRIP_ANGLE_DEG, 0.5 * UH1_PEDAL_TRAVEL_IN, UH1_PEDAL_TRAVEL_IN,
           HELD_S))
    say("")
    say("  lever   coll deg  gripping      walk deg  yaw rate   body r    floor"
        "    end")
    for fraction in [float(arg) for arg in args] or FRACTIONS:
        row(fraction)
    if stream is not None:
        stream.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
