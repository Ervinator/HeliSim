# -*- coding: utf-8 -*-
"""Scratch: the numbers an autorotation demo needs, measured rather than guessed.

Not part of the model, and not a demo: this is the ruler
``_rt_demo_autorotate`` is cut with.  It flies the *mission* - the same one the
demo flies, :class:`_rt_pilot.AutorotationPilot`, on the same aircraft ``main.py``
opens a window on (``"flight test"``, fitted live with
:meth:`simulation.Simulation.fit_engine`), driven one frame at a time exactly as
the window's frame loop drives it - headless, and prints what comes back into
``_rt_autoro_probe.txt``:

* the model's own trims: the level rows the climb is flown on (the baseline of
  the pilot's loops) and the *autorotation* row the descent is flown on
  (:meth:`airframe.Airframe.trim_autorotation`), with the descent rate that comes
  with the second - which is the number the whole mission is judged by;
* the flight, a line a second: the pickup, the climb, the twist grip rolled
  right off, the entry into the descent, the cushion, and the skids;
* the last ten feet of skid every tenth of a second, with the collective, the
  ground effect and the rotor speed, because that is where the cushion's work is
  either done or not;
* and the arrival: the descent rate the skids met the ground at, against
  ``simulation.UH1_HARD_LANDING_MPS``.

The two numbers the mission is cut with - ``_rt_pilot.FLARE_SKID_FT``, the height
of skid the cushion starts at, and ``_rt_pilot.CUSHION_PULL_IN``, the inches of
collective the pull is worth - are patched on the mission's own module, so a
rehearsal is a run of this file with two arguments rather than an edit of the
mission.

Usage::

    python _rt_autoro_probe.py [pull_in] [flare_ft]
"""

import sys

import pygame

from airframe import FOOT
from simulation import (SIM_TIME_STEP_S, Simulation, UH1_CG_HEIGHT_ON_GROUND,
                        UH1_HARD_LANDING_MPS, airframe_preset)
import _rt_pilot
from _rt_pilot import (AXIS_KEYS, AUTOROTATION_SPEED_KT, AutorotationPilot,
                       CLIMB_SKID_FT, REACTION_S, trims_for)

#: The aircraft the demo flies, and the two numbers this file exists to choose:
#: the feet of skid the cushion starts at, and the inches of collective the pull
#: is worth.  They are the mission's own constants, and the command line sets
#: them.
PRESET = "flight test"
PULL_IN = _rt_pilot.CUSHION_PULL_IN
FLARE_SKID_FT = _rt_pilot.FLARE_SKID_FT

#: Nothing lasts forever, and a probe that never arrives should say so.
MAX_FLIGHT_S = 240.0

#: A line a second, and a line every tenth of a second inside the last ten feet
#: of skid, which is where the whole question is answered.
REPORT_S = 1.0
ARRIVAL_SKID_FT = 10.0
ARRIVAL_REPORT_S = 0.1

OUT_PATH = "_rt_autoro_probe.txt"


def say(stream, text):
    """A line for the transcript, to the file first and flushed."""
    stream.write(text + "\n")
    stream.flush()
    sys.__stdout__.write(text + "\n")
    sys.__stdout__.flush()


def key_states(keys):
    """``_rt_pilot``'s scancode keys as :meth:`Simulation.fly`'s own keywords.

    The four axes, in ``PilotInput.step``'s order and signs, from the same
    ``{scancode: 1.0}`` :class:`_rt_pilot.KeyHands` hands out - which is what
    ``main.frame_keys`` is to the windowed frame loop - and the twist grip
    besides, since :meth:`AutorotationPilot.keys` holds E or Q as well.
    """
    names = "collective long_stick lat_stick pedal".split()
    states = {}
    for index, (up_key, down_key) in enumerate(AXIS_KEYS):
        states[names[index]] = keys.get(up_key, 0.0) - keys.get(down_key, 0.0)
    # The twist grip is the fifth position and not one of the four axes:
    # ``main.pilot_keys``' own two scancodes, E rolling it on and Q off, read on
    # its own by ``PilotInput.step``.
    states["throttle"] = (keys.get(pygame.KSCAN_E, 0.0)
                          - keys.get(pygame.KSCAN_Q, 0.0))
    return states


def skid_ft(sim):
    """How high the *skids* are, ft: what a pilot flares by."""
    return (sim.telemetry().alt_agl - UH1_CG_HEIGHT_ON_GROUND) / FOOT


def probe(pull_in=PULL_IN, flare_ft=FLARE_SKID_FT, out_path=OUT_PATH):
    """One autorotation, headless: the mission's own pilot, and what it says.

    Returns the flown :class:`simulation.Simulation`, so a caller can look at
    anything the transcript did not print.
    """
    # The two numbers the mission is cut with are patched on the mission's own
    # module, because the pilot reads them off that module at every decision:
    # this is the mission being rehearsed, not a copy of it.
    _rt_pilot.CUSHION_PULL_IN = float(pull_in)
    _rt_pilot.FLARE_SKID_FT = float(flare_ft)

    stream = open(out_path, "w", encoding="utf-8")
    say(stream, "an autorotation from %.0f ft of skid at %.0f kt on the %s"
        " aircraft, the cushion at %.0f ft of skid and %.2f in of collective"
        % (CLIMB_SKID_FT, AUTOROTATION_SPEED_KT, PRESET, flare_ft, pull_in))

    # The trims: the pilot's own table, built off a scratch aircraft exactly as
    # the demo builds it - the level rows the climb's loops are written around,
    # and the autorotation row the descent is flown on.
    trims = trims_for(airframe_preset(PRESET), AutorotationPilot)
    for speed in trims.speeds:
        row = trims.at(speed)
        say(stream, "level trim at %5.1f m/s: collective %5.2f in, long %+5.2f,"
            " lat %+5.2f, pedal %+5.2f, pitch %+5.2f deg"
            % (speed, row[0], row[1], row[2], row[3], row[4]))
    auto = trims.descent()
    say(stream, "autorotation trim at %.0f kt: collective %5.2f in, long %+5.2f,"
        " lat %+5.2f, pedal %+5.2f, pitch %+5.2f deg, sink %5.2f m/s (%+6.0f"
        " fpm) - the rate the mission has to lose before the skids"
        % (AUTOROTATION_SPEED_KT, auto[0], auto[1], auto[2], auto[3], auto[4],
           auto[5], -auto[5] / FOOT * 60.0))

    # The aircraft the demo flies: on the pad, the T53 fitted live the way
    # ``main.py`` fits it, and the mission's own pilot over it.
    sim = Simulation(airframe=airframe_preset(PRESET))
    sim.on_the_pad()
    sim.fit_engine()
    say(stream, "engine fitted on the pad: %s" % (sim.engine_telemetry(),))
    pilot = AutorotationPilot(sim, trims, reaction=REACTION_S)

    # The flight: nine keys, one frame at a time, driven exactly as the window's
    # frame loop drives them.
    say(stream, "the flight:")
    phase = pilot.phase
    tenth = report = 0.0
    while sim.sim_time < MAX_FLIGHT_S and not pilot.mission_done(sim):
        sim.fly(SIM_TIME_STEP_S, **key_states(pilot.keys(sim, SIM_TIME_STEP_S)))
        telemetry = sim.telemetry()
        if pilot.phase != phase:
            phase = pilot.phase
            say(stream, "  %5.1f s %s" % (sim.sim_time, pilot.note(sim)))
        if sim.sim_time >= report + REPORT_S:
            report = int(sim.sim_time)
            say(stream, "  %5.1f s %s" % (sim.sim_time, pilot.note(sim)))
        if skid_ft(sim) < ARRIVAL_SKID_FT and sim.sim_time >= tenth:
            tenth = sim.sim_time + ARRIVAL_REPORT_S
            say(stream, "  %5.2f s %4.1f ft of skid, %+7.0f fpm, collective"
                " %5.2f in (%4.1f %%), rotor %.1f %%, ground effect %.3f"
                % (sim.sim_time, skid_ft(sim), telemetry.height_rate_fpm,
                   telemetry.collective_in,
                   telemetry.collective_fraction * 100.0,
                   telemetry.rotor_percent, telemetry.ground_effect))

    # The arrival, and the one number the whole exercise is about.
    telemetry = sim.telemetry()
    descent = -telemetry.height_rate
    say(stream, "arrival at %.1f s: %s" % (sim.sim_time, telemetry))
    say(stream, "%s: the skids met the ground at %5.2f m/s (%+6.0f fpm),"
        " against %.1f m/s hard; the rotor was down to %.0f %%; %s"
        % ("CRASHED" if sim.crashed else "landed", descent,
           telemetry.height_rate_fpm, UH1_HARD_LANDING_MPS,
           pilot.lowest_rotor, telemetry.crash_reason
           or "no reason given, the arrival was inside the floor's own"))
    say(stream, "the cushion was worth %5.2f m/s: from the trim's %.2f to"
        " %.2f, in %.0f decisions"
        % (auto[5] - descent, auto[5], descent, pilot.decisions))
    stream.close()
    return sim


def demo():
    """The command line: the pull and the height it starts at, if they are said."""
    args = sys.argv[1:]
    pull_in = float(args[0]) if args else PULL_IN
    flare_ft = float(args[1]) if len(args) > 1 else FLARE_SKID_FT
    probe(pull_in, flare_ft)
    print("wrote %s" % (OUT_PATH,))
    return 0


if __name__ == "__main__":
    sys.exit(demo())

