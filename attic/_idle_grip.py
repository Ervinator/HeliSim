"""Whether a closed twist grip can still fly: the two aircraft, side by side.

The report of the bug was "set the throttle to minimum and the helicopter still
goes into the air".  There are two aircraft in this project and the answer has
to be told apart between them:

* the report's own, which is what a run of the sandbox begins on - no engine in
  the model at all, the rotor held at 100 per cent, and therefore no throttle
  for the grip to be a *minimum* of;
* the one F fits, a T53 on the shaft, where the grip is the governor's speed
  selector and a closed grip is no fuel.

Both are flown here with the same stick input - the collective rolled up from
the pad - and what comes back is the altitude and the rotor speed.

Run:  python attic/_idle_grip.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main
from simulation import SIM_TIME_STEP_S, Simulation, airframe_preset


#: How long the collective is held up: six seconds is a hover's worth of climb.
SECONDS = 6.0


def climb(engine, grip, collective=1.0):
    """The altitude a parked aircraft gains with *grip* and *collective* held."""
    sim = Simulation(airframe=airframe_preset(main.AIRFRAME_PRESET))
    sim.on_the_pad()
    if engine:
        sim.fit_engine()
        sim.set_throttle(grip)
    else:
        sim.set_throttle(grip)
    start = sim.airframe.state.altitude
    keys = dict((name, 0.0) for name in ("collective", "long_stick",
                                        "lat_stick", "pedal", "throttle"))
    keys["collective"] = collective
    rotor = []
    for frame in range(int(SECONDS / SIM_TIME_STEP_S)):
        sim.step(SIM_TIME_STEP_S, sim.pilot.step(SIM_TIME_STEP_S, **keys))
        if frame % 60 == 0:
            rotor.append(100.0 * sim.airframe.state.rotor_speed
                         / (2.0 * 3.141592653589793 * 324.0 / 60.0))
    return sim.airframe.state.altitude - start, rotor


def main_():
    print(__doc__.strip().splitlines()[0])
    print()
    print("  aircraft              grip   climb, m   rotor %%, one a second")
    for engine, name in ((False, "report's own (no engine)"),
                         (True, "T53 fitted (F)")):
        for grip in (0.0, 1.0):
            climbed, rotor = climb(engine, grip)
            print("  %-22s %4.0f %%   %+7.2f   %s"
                  % (name, 100.0 * grip, climbed,
                     " ".join("%5.1f" % value for value in rotor)))
    print()
    for engine, name in ((False, "no engine"), (True, "T53 fitted")):
        closed, _ = climb(engine, 0.0)
        open_, _ = climb(engine, 1.0)
        print("  %-10s closed grip climbs %+6.2f m, open grip %+6.2f m: %s"
              % (name, closed, open_,
                 "the grip flies the aircraft" if abs(closed - open_) > 0.5
                 else "THE GRIP FLIES NOTHING"))


if __name__ == "__main__":
    main_()
