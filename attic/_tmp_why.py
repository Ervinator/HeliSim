# -*- coding: utf-8 -*-
"""Scratch: what did the autorotation pilot *see* at each of its own looks?

Flies the mission exactly as ``_tmp_diag`` does, but spies on
:meth:`AutorotationPilot.decide` so the grip and the phase the pilot's own
``phase_of`` reads are printed from inside the decision, not from the loop
outside it.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation import SIM_TIME_STEP_S, Simulation, airframe_preset
from _rt_pilot import AutorotationPilot, REACTION_S, trims_for
from _rt_autoro_probe import key_states, skid_ft

OUT = "_tmp_why.txt"
lines = []


def say(text):
    lines.append(text)
    print(text)


trims = trims_for(airframe_preset("flight test"), AutorotationPilot)
sim = Simulation(airframe=airframe_preset("flight test"))
sim.on_the_pad()
sim.fit_engine()
pilot = AutorotationPilot(sim, trims, reaction=REACTION_S)

seen = []
real_decide = AutorotationPilot.decide


def spy(self, s):
    seen.append((s.sim_time, s.pilot.throttle(), self.phase))
    return real_decide(self, s)


AutorotationPilot.decide = spy
while sim.sim_time < 240.0 and not pilot.mission_done(sim):
    sim.fly(SIM_TIME_STEP_S, **key_states(pilot.keys(sim, SIM_TIME_STEP_S)))
AutorotationPilot.decide = real_decide

say("decisions: %d, last decision at %.2f s, flight ended %.2f s"
    % (len(seen), seen[-1][0] if seen else -1.0, sim.sim_time))
for t, grip, phase in seen[-14:]:
    say("  %7.2f  grip %-24r phase %s" % (t, grip, phase))
say("final grip %r phase %s padded? %s"
    % (sim.pilot.throttle(), pilot.phase, pilot.took_off))
say("closed-while-cut frames: %d"
    % (sum(1 for _t, g, ph in seen if g <= 0.0 and ph == "cut"),))

# What would one more look have done?  Put the pilot's own decision through
# once more on the end-of-flight state and see whether the phase moves.
pilot.phase = "cut"
AutorotationPilot.phase_of(pilot, sim)
say("phase after a manual look at the final state: %s" % (pilot.phase,))
say("skid at the end: %.1f ft (lowest rate seen aloft %s)"
    % (skid_ft(sim), pilot.last_airborne_rate))

with open(OUT, "w") as stream:
    stream.write("\n".join(lines) + "\n")
