# -*- coding: utf-8 -*-
"""Scratch: how hard the skids arrive, over the cushion's own numbers.

Flies the autorotation mission headless exactly as ``_rt_autoro_probe`` does -
the same pilot on the same aircraft, one frame at a time - once per cell, and
reports the rate the skids met the ground at as the *model* measures it: the
descent along the ground's normal, read off the frame that was the last one
still in the air.  The telemetry cannot say it, because by the time the pilot
could read it the floor has already taken the descent out of the state.

A *cell* is four numbers, patched onto the mission's own module the way
``_rt_autoro_probe`` rehearses too: ``flare_ft,sink_mps,pull_in,gain`` - the
height of skid the cushion opens at, the sink it is asked for when the skids
are down, the inches above the trim's own lever the pull may spend, and the
inches a second it pulls for every metre a second the sink is over the line.
Everything else is the mission as it stands.

Run from the project root, with the transcript going to its own file so several
runs can be in the air at once::

    python attic/_tmp_sweep.py out=_tmp_a.txt cells 250,2.5,6.5,0.60 ...
    python attic/_tmp_sweep.py out=_tmp_one.txt one 250,2.5,6.5,0.60 0.25

``one`` is a single cell with the last :data:`TS_FT` feet of skid printed every
fifth argument's seconds, which is how the lever, the sink and the rotor are
read on the way down.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import _rt_pilot
from simulation import (SIM_TIME_STEP_S, Simulation, UH1_HARD_LANDING_MPS,
                        airframe_preset)
from _rt_pilot import AutorotationPilot, REACTION_S, skid_ft, trims_for
from _rt_autoro_probe import key_states

#: The preset the demo flies, so a cell here is a cell in the probe.
PRESET = "flight test"

#: Nothing lasts for ever: a run that never arrives should say so rather than
#: hold a sweep open.
MAX_FLIGHT_S = 240.0

#: How high the time series of ``one`` starts printing: the flare and
#: everything under it, rather than four hundred feet of settled trim above it.
TS_FT = 520.0

#: The cushion's own key rate as the mission ships it, in inches a second - the
#: lever's fastest, read off the model.  A cell's eighth number scales this.
BASE_KEY_RATE_IN = _rt_pilot.CUSHION_KEY_RATE_IN

#: Where the rows go, when a caller asked for a file: a detached run has no
#: console anybody can read.
stream = None


def say(text):
    print(text)
    if stream is not None:
        stream.write(text + "\n")
        stream.flush()


def build_trims():
    """The mission's trim table, once: it is four solves and a big wait."""
    started = time.time()
    trims = trims_for(airframe_preset(PRESET), AutorotationPilot)
    sink = trims.descent()[5]
    say("trims built in %.1f s: autorotation trim coll %.2f in, pitch %+.2f deg,"
        " sink %.2f m/s (%.0f fpm)"
        % (time.time() - started, trims.descent()[0], trims.descent()[4], sink,
           sink / 0.3048 * 60.0))
    return trims


def fly(trims, flare, sink, pull, gain, every=None, limit=MAX_FLIGHT_S,
        pitch_in=None, speed_deg=None, flare_pitch=None, rate_scale=None,
        flare_pitch_from=None, desc_lev=None):
    """One flight of the mission, with the cushion's four numbers set for it.

    The rate kept is the model's own: the descent along the ground's normal,
    sampled before the step, so the last one whose frame was still in the air
    is the rate the skids met the ground at - and the rotor on that frame
    besides, which is the other half of the question.

    *pitch_in* and *speed_deg*, when they are said, are the two cyclic gains
    the mission flies its attitude with - ``_rt_pilot.K_PITCH_IN`` and
    ``K_SPEED_DEG`` - patched for the run: the hypothesis under test is that
    the pull's own nose-up moment is what spoils the cushion, and that a pilot
    holding the attitude through it would keep the thrust pointing down.

    *flare_pitch*, when it is said, is ``_rt_pilot.FLARE_PITCH_DEG``: the nose
    up the cushion *asks* for by the skids, which is the other half of that
    hypothesis - that the attitude has to be commanded rather than held.  And
    *rate_scale* is what the cushion's own key rate is multiplied by, under one
    for a hand gentler than the model's own.
    """
    _rt_pilot.FLARE_SKID_FT = flare
    _rt_pilot.CUSHION_SINK_MPS = sink
    _rt_pilot.CUSHION_PULL_IN = pull
    _rt_pilot.CUSHION_GAIN_IN = gain
    if pitch_in is not None:
        _rt_pilot.K_PITCH_IN = pitch_in
    if speed_deg is not None:
        _rt_pilot.K_SPEED_DEG = speed_deg
    if flare_pitch is not None:
        _rt_pilot.FLARE_PITCH_DEG = flare_pitch
    if flare_pitch_from is not None:
        # The height the nose comes up from: above the cushion's own, the cyclic
        # flare is begun early and the lever's cushion is left where it was.
        _rt_pilot.FLARE_PITCH_FROM_FT = flare_pitch_from
    if desc_lev is not None:
        # The lever held *down* under the descent's own trim, in inches, for the
        # whole of the descent: the "lever down for RPM" a real autorotation is
        # flown by, and revolutions bought without the attitude being touched.
        _rt_pilot.DESCENT_LEVER_DOWN_IN = desc_lev
    _rt_pilot.CUSHION_KEY_RATE_IN = BASE_KEY_RATE_IN * (
        1.0 if rate_scale is None else rate_scale)
    started = time.time()
    sim = Simulation(airframe=airframe_preset(PRESET))
    sim.on_the_pad()
    sim.fit_engine()
    pilot = AutorotationPilot(sim, trims, reaction=REACTION_S)
    track = []
    arrival = 0.0                 # m/s, the last rate read off the ground
    rotor = 100.0                 # %, the rotor on the frame that read it
    while sim.sim_time < limit and not pilot.mission_done(sim):
        airborne = not sim.on_ground
        descent = -sim.airframe.state.ground_velocity().dot(sim.ground.normal)
        sim.fly(SIM_TIME_STEP_S, **key_states(pilot.keys(sim, SIM_TIME_STEP_S)))
        telemetry = sim.telemetry()
        if airborne:
            arrival = descent
            rotor = telemetry.rotor_percent
        track.append((sim.sim_time, pilot.phase, skid_ft(telemetry), descent,
                      telemetry.collective_in, telemetry.rotor_percent,
                      telemetry.pitch_deg, telemetry.airspeed_kt))
    say("  (flown in %.1f s)" % (time.time() - started,))
    if every:
        series(track, every)
    return sim, pilot, track, arrival, rotor


def series(track, every):
    """The last :data:`TS_FT` feet of skid, a line every *every* seconds.

    The rate column is *descent* and so positive going down, which is the
    model's own sign along the ground's normal - the same number the cell's
    arrival is read from.
    """
    say("       t    skid ft  fpm down   coll in  rotor %  pitch deg    kt"
        "   phase")
    mark = -1.0
    for row in track:
        if row[2] >= TS_FT:
            continue
        step = round(row[0] / every)
        if step == mark:
            continue
        mark = step
        say("   %6.2f  %8.1f  %+8.0f  %8.2f  %7.1f  %+9.1f  %5.1f   %s"
            % (row[0], row[2], row[3] / 0.3048 * 60.0, row[4], row[5], row[6],
               row[7], row[1]))


def phases(track):
    """The phases the flight went through, in order and once each."""
    seen = []
    for row in track:
        if not seen or seen[-1] != row[1]:
            seen.append(row[1])
    return ">".join(seen)


def cell(arg):
    """One cell: ``flare_ft,sink_mps,pull_in,gain[,pitch_in[,speed_deg[,\
flare_pitch_deg[,rate_scale]]]]``.

    The first four are the cushion's own numbers and are always said.  The
    fifth and the sixth, when they are said, are the cyclic gains above, and
    are what the sweep adds to the constants the mission ships with - they are
    read as the *value* the gain is set to, not as a change to it.  The seventh
    is the flare attitude the cushion asks for by the skids, and the eighth a
    scale on the lever's own key rate.
    """
    return [float(part) for part in arg.split(",")]


def grid(trims, cells):
    """A cell a line: the arrival, and the rotor the arrival was made on.

    The rotor columns are the other half of the question: the rotor on the last
    frame in the air, the slowest it was seen at any time, and the fastest the
    aircraft ever came *back up* under the cushion (a negative fpm there is the
    balloon that spends the revolutions - the lever holding the machine against
    a rotor with nothing driving it).  A rotor under
    ``aerodynamics.UH1_RPM_LOW``'s 294 rpm - 90.7 % - is out of the green arc.
    """
    say("  flare   sink   pull   gain   arrival m/s   fpm  landed?  reason"
        "          rotor  low %  rise fpm  phases")
    for numbers in cells:
        flare, sink, pull, gain = numbers[:4]
        extras = {}
        names = ("pitch_in", "speed_deg", "flare_pitch", "rate_scale",
                 "flare_pitch_from", "desc_lev")
        for index, name in enumerate(names):
            if len(numbers) > 4 + index:
                extras[name] = numbers[4 + index]
        note = ""
        if extras:
            note = "  [%s]" % (" ".join(
                "%s %g" % (name, value) for name, value in extras.items()),)
        try:
            sim, pilot, track, arrival, rotor = fly(
                trims, flare, sink, pull, gain, **extras)
        except Exception as err:
            # A cell the model refuses - equation 10's two roots meeting, say -
            # is a row too, rather than the end of the sweep.
            say("  %5.1f  %5.2f  %5.2f  %5.2f   %s: %s"
                % (flare, sink, pull, gain, type(err).__name__, err))
            continue
        rise = min([row[3] for row in track if row[1] == "cushion"] or [0.0])
        say("  %5.1f  %5.2f  %5.2f  %5.2f   %+8.2f  %+5.0f  %-7s  %-14s  %4.1f"
            "  %5.1f  %+8.0f  %s%s"
            % (flare, sink, pull, gain, arrival, arrival / 0.3048 * 60.0,
               "crashed" if sim.crashed else "landed",
               (sim.crash_reason or "")[:14], rotor, pilot.lowest_rotor,
               rise / 0.3048 * 60.0, phases(track), note))


def main(argv):
    """The command line: where the rows go, a mode, and its own arguments."""
    global stream
    args = list(argv)
    if args and args[0].startswith("out="):
        stream = open(args[0][4:], "w", encoding="utf-8")
        args = args[1:]
    say("hard landing floor %.2f m/s; the trims' own autorotation sink is the"
        " baseline the cushion has to shed" % (UH1_HARD_LANDING_MPS,))
    trims = build_trims()
    say("")
    if args and args[0] == "one":
        numbers = cell(args[1])
        every = float(args[2]) if len(args) > 2 else 0.25
        extras = {}
        for index, name in enumerate(("pitch_in", "speed_deg", "flare_pitch",
                                      "rate_scale", "flare_pitch_from",
                                      "desc_lev")):
            if len(numbers) > 4 + index:
                extras[name] = numbers[4 + index]
        sim, pilot, track, arrival, rotor = fly(trims, *numbers[:4], every=every,
                                                **extras)
        say("  %s at %+5.2f m/s (%+6.0f fpm down) on a rotor down to %.1f %%"
            % ("CRASHED" if sim.crashed else "landed", arrival,
               arrival / 0.3048 * 60.0, pilot.lowest_rotor))
        say("  %s" % (sim.crash_reason or "inside the floor's own",))
    else:
        # No mode word in this branch: every argument left is a cell.  (It was
        # ``args[1:]``, which quietly dropped the first cell of every sweep.)
        grid(trims, [cell(arg) for arg in args])
    if stream is not None:
        stream.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
