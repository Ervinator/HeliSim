# -*- coding: utf-8 -*-
"""Scratch: what the autorotation mission's own grip does, frame by frame.

Run from the project root as ``python attic/_tmp_diag.py [pull_in] [flare_ft]``.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation import SIM_TIME_STEP_S, Simulation, airframe_preset
import _rt_pilot
from _rt_pilot import (THROTTLE_STEP, AutorotationPilot, CUSHION_PULL_IN,
                       FLARE_SKID_FT, REACTION_S, trims_for)
from _rt_autoro_probe import key_states, skid_ft

if len(sys.argv) > 1:
    _rt_pilot.CUSHION_PULL_IN = float(sys.argv[1])
if len(sys.argv) > 2:
    _rt_pilot.FLARE_SKID_FT = float(sys.argv[2])
print("THROTTLE_STEP %.8f  pull %.2f  flare %.1f"
      % (THROTTLE_STEP, _rt_pilot.CUSHION_PULL_IN, _rt_pilot.FLARE_SKID_FT))

import inspect
print("module file: %s" % (_rt_pilot.__file__,))
print("decide is: %s" % (AutorotationPilot.decide,))
print(inspect.getsource(AutorotationPilot.phase_of))

trims = trims_for(airframe_preset("flight test"), AutorotationPilot)
sim = Simulation(airframe=airframe_preset("flight test"))
sim.on_the_pad()
sim.fit_engine()
pilot = AutorotationPilot(sim, trims, reaction=REACTION_S)
phase = pilot.phase
next_report = 0.0
while sim.sim_time < 240.0 and not pilot.mission_done(sim):
    sim.fly(SIM_TIME_STEP_S, **key_states(pilot.keys(sim, SIM_TIME_STEP_S)))
    if pilot.phase != phase:
        phase = pilot.phase
        print("%7.2f -> %-8s grip %.6f skid %6.1f rate %+7.2f coll %5.2f"
              % (sim.sim_time, phase, sim.pilot.throttle(), skid_ft(sim),
                 sim.telemetry().height_rate, sim.telemetry().collective_in))
    if sim.pilot.throttle() <= 0.0 and pilot.phase == "cut":
        print("grip is closed while still cut at %.2f s (decision %d)"
              % (sim.sim_time, pilot.decisions))
    if sim.sim_time >= next_report:
        next_report = int(sim.sim_time) + 5.0
        print("%7.2f    %-8s grip %.6f skid %6.1f rate %+7.2f coll %5.2f"
              % (sim.sim_time, pilot.phase, sim.pilot.throttle(),
                 skid_ft(sim), sim.telemetry().height_rate,
                 sim.telemetry().collective_in))
print("done %.2f s crashed=%s reason=%s touchdown_rate=%s lowest_rotor=%.1f"
      % (sim.sim_time, sim.crashed, sim.crash_reason, pilot.touchdown_rate,
         pilot.lowest_rotor))
print("phase %s throttle %.8f" % (pilot.phase, sim.pilot.throttle()))
