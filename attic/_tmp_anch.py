# -*- coding: utf-8 -*-
"""Scratch: what the cushion's own anchor reads, on the way down.

One flight of the autorotation mission with one cell's numbers set on it, and
the last few hundred feet of it printed as a series: the skids' height, the
rate off the instruments, the *anchor* the cushion's schedule was opened on,
the rate it asks for at that height, the pull above the descent's own lever,
the rotor and the attitude.  Run from the project root as
``python attic/_tmp_anch.py [desc_lev]``.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import _rt_pilot
from simulation import SIM_TIME_STEP_S, Simulation, airframe_preset
from _rt_pilot import AutorotationPilot, REACTION_S, skid_ft, trims_for
from _rt_autoro_probe import key_states

PRESET = "flight test"
EVERY_S = 0.5
TS_FT = 560.0


def main(argv):
    """The flight: one cell, one series, one line each half second."""
    _rt_pilot.FLARE_SKID_FT = 200.0
    _rt_pilot.CUSHION_SINK_MPS = 2.70
    _rt_pilot.CUSHION_PULL_IN = 6.50
    _rt_pilot.CUSHION_GAIN_IN = 1.00
    _rt_pilot.FLARE_PITCH_DEG = 12.0
    _rt_pilot.FLARE_PITCH_FROM_FT = 200.0
    _rt_pilot.CUSHION_KEY_RATE_IN = 1.5125
    _rt_pilot.DESCENT_LEVER_DOWN_IN = float(argv[0]) if argv else 0.0
    trims = trims_for(airframe_preset(PRESET), AutorotationPilot)
    trim_in = trims.descent()[0]
    sim = Simulation(airframe=airframe_preset(PRESET))
    sim.on_the_pad()
    sim.fit_engine()
    pilot = AutorotationPilot(sim, trims, reaction=REACTION_S)
    print("cell: flare 200 sink 2.70 pull 6.50 gain 0.20 pitch 12 desc_lev %g"
          % (_rt_pilot.DESCENT_LEVER_DOWN_IN,))
    print("       t  phase    skid ft   rate   anchor    want   pull  rotor"
          "   pitch    kt")
    mark = -1.0
    while sim.sim_time < 240.0 and not pilot.mission_done(sim):
        sim.fly(SIM_TIME_STEP_S, **key_states(pilot.keys(sim, SIM_TIME_STEP_S)))
        tele = sim.telemetry()
        skid = skid_ft(tele)
        if skid >= TS_FT:
            continue
        step = round(sim.sim_time / EVERY_S)
        if step == mark:
            continue
        mark = step
        anchor = pilot.cushion_sink
        want = (None if anchor is None else
                -max(_rt_pilot.CUSHION_SINK_MPS,
                     anchor * skid / _rt_pilot.FLARE_SKID_FT))
        print("%8.2f  %-8s %7.1f  %+6.2f  %6s  %6s  %+5.2f  %5.1f  %+6.1f"
              "  %4.1f"
              % (sim.sim_time, pilot.phase, skid, tele.height_rate,
                 "-" if anchor is None else "%6.2f" % anchor,
                 "-" if want is None else "%6.2f" % want,
                 tele.collective_in - trim_in, tele.rotor_percent,
                 tele.pitch_deg, tele.airspeed_kt))
    print("done %.2f s crashed=%s reason=%s rate %+.2f low %.1f anchor %s"
          % (sim.sim_time, sim.crashed, sim.crash_reason,
             pilot.touchdown_rate or 0.0, pilot.lowest_rotor,
             pilot.cushion_sink))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
