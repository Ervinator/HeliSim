# -*- coding: utf-8 -*-
"""Scratch: the gas producer's own time constant, measured rather than argued.

``engine.UH1_ENGINE_TIME_CONSTANT_S`` is the one constant in the model whose
comment admits it has no source - "the order of a T53's spool time", with
"sec, no source: see docstring" printed beside it - and it is on the flying path:
it is the whole of what stands between the governor's demand and the torque at
the shaft, so every spool, every droop and every wind down in this model is flown
through it.  What is behind it is that comment, the paragraph in ``engine.py``'s
docstring that repeats it, and three assertions in the module's own self test -
``advance(0.6, slow)`` against ``advance(1.2, slow)`` - which prove that the lag
is a lag and say nothing at all about how long it should be.

This is the ruler that number is cut with: the engine's own instrument, the way
``_rt_probe.py`` is the pilot's and ``_rt_tail.py`` is the tail rotor's.  It
prints the survey behind the number, in the one place the two can be read against
each other, and it asserts what it prints, so the comment can cite a measurement
instead of a memory and a drift in it is a traceback.

**The two clocks.**  The constant is a *time*, so the first thing to measure is
the time it is a fraction of: the rotor's own, ``I omega / Q`` at the hover -
6.89 sec on the report's 8700 lb aircraft and 9.70 on the flight test one, i.e.
eleven and sixteen times the engine's 0.6.  The rotor is the slow thing in this
loop and the gas producer is a correction on it, which is why the honest answer
is a band of constants rather than one number.

**Two rulers on it.**  The manual's own is the +-40 rpm droop band (2-23), and it
is a *settled* law: this file's second table shows that the constant's first duty
is not to touch it, since the same power demand settles at the same rotor speed
at every constant in the sweep, to the last bit.  The project's own ruler is the
*clock*: the seconds a spool takes, the seconds a wind down takes, and the
transient a pull adds on top of the settled droop - the three things that are the
constant's business alone.  Which of the two a number belongs to is what the
tables are cut to separate.

**What comes back, into** ``_rt_engine.txt``**:**

* the two clocks, on both of the report's aircraft: the rotor's own time constant
  at the hover and the engine's lag, and the ratio between them;
* the settled law, which the constant does not touch: the droop curve at no load,
  at the 8700 lb hover and at full power, in engine and rotor rpm and in per cent
  of the 324 rpm reference, against the manual's own band and the green arc;
* the lag on its own, with the rotor held: the fractions of the target reached at
  one, three and five time constants for every constant in the sweep, and the
  same arithmetic run *backwards* as the fuel goes away - the power still in the
  shaft at half a second, a second, two and three;
* the spool, flown: a dead shaft in a trimmed hover with the fuel coming back on,
  which is the one maneuver in the model where a slow engine has to be caught by
  a live governor - the rotor's low point, where the arc is left if it is, and
  the seconds the loop takes to put the rotor back on the trim;
* the lever pulled to the up stop, flown, at every constant: the transient the
  lag adds to the settled droop, which is the second ruler on the manual's own
  band;
* EMER, flown: the switch that takes the governor out of the loop, after which
  the grip is the fuel control and the lag is the only thing between the hand and
  the torque - and the rotor runs away, which is 9-3's own caution;
* the mission's *own* instrument, borrowed: the autorotation's failure entry
  (``_rt_autoro_entry.flight``) flown at three constants, so the number the
  published catch is cut with is read rather than assumed;
* and the count of what reads the constant - which is one line of
  ``engine.Engine.advance`` and nothing else in the model.

Nothing in the model imports this file.  The six tables above the last one need
nothing but the model: ``aerodynamics``, ``engine``, ``airframe`` and
``simulation`` and the standard library, with no pygame anywhere in them.  The
last one flies the mission's own pilot through the autorotation, so it imports
``_rt_autoro_entry`` and ``_rt_pilot`` - and pygame with them - where it is
called, and the reason it is called rather than copied is that the entry's
numbers are that file's own.

Usage::

    python _rt_engine.py [time_constant_s ...]

The arguments *narrow* the sweep rather than replace it: the model's own default
is always flown, since that is the row every claim in this file is about.
"""

import math
import sys

from aerodynamics import UH1_RPM, UH1_RPM_100, UH1_RPM_LOW, UH1_RPM_MAX
from engine import (Engine, Governor, UH1_ENGINE_DROOP_RPM,
                    UH1_ENGINE_MAIN_RATIO, UH1_ENGINE_POWER_W, UH1_ENGINE_RPM,
                    UH1_ENGINE_TIME_CONSTANT_S, UH1_TORQUE_NM_PER_PSI,
                    UH1_TRANSMISSION_LIMIT_PSI)
from simulation import SIM_TIME_STEP_S, Simulation, airframe_preset

#: The transcript, written by the run itself: the shell's own redirection is not
#: worth trusting on this box.
OUT_PATH = "_rt_engine.txt"

#: The aircraft the engine's own numbers are quoted on, and the one ``main.py``
#: opens a window on.  ``engine.py``'s self test reads the report's 8700 lb
#: aircraft for the droop the manual's band is worth and the flight test one for
#: the faults a pilot can fly, so this file flies both: the first for the shaft
#: tables, the second for EMER and for the mission's own entry.
REPORT_PRESET = "simulation"
FLIGHT_PRESET = "flight test"

#: The gas producer's own constant read off an engine, which is how a row is
#: labelled: a constant moved in ``engine.py`` moves the label with it.
NO_LAG_S = 1e-6


#: The constants the shaft tables are cut at, sec: the model's own 0.6 with a step
#: either side of it and both ends of the band a T53's spool time plausibly sits
#: in.  ``None`` is the model's *own* default - an :class:`engine.Engine` as it
#: comes, rather than the number written down again here - and it is always flown,
#: since the claims are about it; :data:`NO_LAG_S` is the instant torque the
#: constant's own docstring refuses, flown to show what refusing it costs.
TAUS = (None, 0.2, 0.4, 0.8, 1.2, 1.5, 2.0, 3.0, NO_LAG_S)

#: The constants the mission's own entry is flown at, sec - three, because that
#: flight is a whole autorotation and is the most expensive table here: the step
#: below, the shipped one, and the step above.
ENTRY_TAUS = (0.2, 1.5)

#: How long the aircraft is left on its trim before a table's maneuver begins,
#: and how long the maneuver is flown for, s.  Two seconds is longer than any lag
#: in the loop, so the row starts from the settled hover rather than from the
#: transient the reset left.
SETTLE_S = 2.0
SPOOL_FOR_S = 15.0
PULL_FOR_S = 12.0
EMER_FOR_S = 15.0

#: The manual's own green arc in the per cent the captions are in: 294 to 339 rpm
#: around 324, so 90.7 to 104.6 - the band the rotor speed may live in, and the
#: one a spool that is too slow leaves.
GREEN_ARC_PERCENT = (100.0 * UH1_RPM_LOW / UH1_RPM_100,
                     100.0 * UH1_RPM_MAX / UH1_RPM_100)

#: What "the rotor is back on its trim" means, per cent of the reference: the
#: seconds a table quotes are the seconds to come inside this of the speed the
#: table started at.  A twentieth of a per cent is a sixteenth of the droop band
#: at the hover, so it is a settled rotor rather than a passing one - and it is the
#: *spool's* criterion, where the lever never moves and the trim the rotor returns
#: to is the one it left.
RECOVERED_PERCENT = 0.05

#: The same question for the lever pull: a pull ends with the lever somewhere
#: else, so the speed the rotor settles at is the *new* demand's droop and not the
#: trim it started on.  The seconds a pull quotes are therefore read against the
#: speed the last of these seconds holds, to within :data:`RECOVERED_PERCENT` of it.
RECOVERED_MEAN_S = 1.0

#: The rotor speed the lag's own arithmetic is measured with it held at, as a
#: fraction of the reference: five per cent slow, which is the case the governor
#: is for - the error is 330 engine rpm, the gain asks for everything there is.
HOLD_FRACTION = 0.95

#: The instants the lag's own rise and a dead shaft's decay are read at, sec, and
#: what counts as the fuel being gone, as a fraction of the ceiling - a per cent,
#: which is where the model's own flight transcripts put it.  The rise is followed
#: for long enough that even the slowest constant in the sweep reaches 99 per cent
#: inside the window, so that no column of the table reads "never" for want of
#: frames rather than for want of answer.
RISE_MARKS = (0.5, 1.0, 2.0, 3.0, 5.0)
DECAY_MARKS = (0.5, 1.0, 2.0, 3.0)
RISE_WINDOW_S = 20.0
GONE_FRACTION = 0.01


def engine_for(tau):
    """The T53 a row is flown with: the model's own default when *tau* is None.

    ``None`` builds a bare :class:`engine.Engine`, so what the default rows
    measure is the constant ``engine.py`` ships rather than the number written
    down a second time in this file - the same discipline ``_rt_tail.py`` keeps
    with ``Rotor.uh1h_tail``'s defaults.
    """
    return Engine() if tau is None else Engine(governor=Governor(time_constant=tau))


def tau_of(engine):
    """The gas producer's constant an engine was built with, sec, off the engine."""
    return engine.governor.time_constant


def the_sweep(arguments=()):
    """The constants this run sweeps: a caller's own, and the model's own default.

    The arguments *narrow* the sweep and never replace it, because the claims in
    :func:`claims` are about the shipped constant: a run that left it out would
    be a table with no baseline in it.  The model's own default goes first and the
    instant torque last, whatever a caller names.
    """
    named = [float(argument) for argument in arguments] or [
        tau for tau in TAUS if tau is not None and tau != NO_LAG_S]
    listed = [None]
    for tau in named:
        if tau not in listed:
            listed.append(tau)
    if NO_LAG_S not in listed:
        listed.append(NO_LAG_S)
    return tuple(listed)


def the_entries(arguments=()):
    """The same sweep for the entry table: :data:`ENTRY_TAUS`, and the default.

    The entry is a whole autorotation per row, so it does not fly the whole
    :data:`TAUS` sweep: what the table asks is whether the published catch moves
    when the constant does, and three points answer that.
    """
    named = [float(argument) for argument in arguments] or list(ENTRY_TAUS)
    listed = [None]
    for tau in named:
        if tau not in listed:
            listed.append(tau)
    return tuple(listed)


def say(text=""):
    """A line of this file's transcript: to the file first, and to the console.

    ``main`` points ``sys.stdout`` at the transcript file, so ``print`` is the
    file; the console gets the same line through ``sys.__stdout__``, which is what
    makes a run that takes minutes worth watching rather than waiting for.
    """
    print(text)
    sys.stdout.flush()
    sys.__stdout__.write(text + "\n")
    sys.__stdout__.flush()


def line(sim):
    """One instant of a flown row, in the units its own gauges are marked in."""
    telemetry = sim.telemetry()
    return ("t %6.2f | lever %5.3f | rotor %6.2f %% (%6.2f rpm) | n2 %6.0f"
            " | torque %6.1f %% (%6.1f %% of max) | %6.1f kW | alt %7.2f m"
            " %+6.1f m/s%s"
            % (sim.sim_time, sim.pilot.collective_axis,
               telemetry.rotor_percent, telemetry.rotor_rpm,
               telemetry.engine.rpm, telemetry.engine.torque_percent,
               telemetry.engine.maximum_percent,
               telemetry.engine.power / 1000.0, telemetry.alt_agl,
               telemetry.height_rate,
               " CRASHED %s" % (telemetry.crash_message,)
               if sim.crashed else ""))


def the_two_clocks():
    """The rotor's own clock beside the engine's, on both of the report's aircraft.

    The constant is a *time*, so the first thing to measure it against is the
    other times in the loop it sits in.  The rotor's own is ``I omega / Q``: the
    rotor's inertia against the damping its own torque is worth at the hover - the
    seconds a per cent of rotor speed would take to decay if the engine stopped
    answering - and it is the number the gas producer's lag has to be *small*
    against, since the rotor is what the governor is finally holding.  It is the
    aircraft's own and not the engine's (the same 3315 kg m^2 of hub and blades
    carries both presets' masses), and it is a different mechanism from table 3's
    0.144 sec flapping constant, which is the blade's freedom about its hinge
    rather than the rotor's about its shaft.

    Returns one row per aircraft, carrying the rotor speed, its torque and the
    clock the two make, so that the tables below quote the same two hovers.
    """
    say()
    say("the two clocks: the rotor's own at the hover, and the gas producer's")
    say("  aircraft       mass   inertia     hover Q   omega   rotor's clock"
        "   engine's    ratio")
    say("                   kg     kg m^2         N m   rad/s               s"
        "          s")
    rows = []
    for preset in (REPORT_PRESET, FLIGHT_PRESET):
        sim = Simulation(airframe=airframe_preset(preset, engine=True))
        telemetry = sim.telemetry()
        inertia = sim.airframe.rotor_inertia
        omega = telemetry.rotor_speed
        torque = telemetry.torque
        clock = inertia * omega / torque
        tau = tau_of(sim.airframe.engine)
        say("  %-12s %7.0f %9.1f %11.0f %7.2f %15.2f %10.2f %8.1f"
            % (preset, sim.airframe.mass, inertia, torque, omega, clock, tau,
               clock / tau))
        say("  %s" % (line(sim),))
        rows.append(dict(preset=preset, sim=sim, telemetry=telemetry,
                         inertia=inertia, omega=omega, torque=torque,
                         clock=clock, tau=tau))
    say("  the same 3315 kg m^2 carries both, so the heavier hover's torque is"
        " the bigger")
    say("  damping and its clock the shorter one: %.2f sec against %.2f."
        % (rows[0]["clock"], rows[1]["clock"]))
    say("  the engine's %.2f sec is %.0f and %.0f times shorter than either."
        % (UH1_ENGINE_TIME_CONSTANT_S,
           rows[0]["clock"] / UH1_ENGINE_TIME_CONSTANT_S,
           rows[1]["clock"] / UH1_ENGINE_TIME_CONSTANT_S))
    return rows


def the_settled_law(hover_powers):
    """The manual's own ruler: the droop band read as a curve, and constant-free.

    The governor is a proportional controller and nothing more, so the power
    demand a hover has is what decides the speed it settles at: ``P / gain`` under
    the selected speed, clipped to the +-40 rpm band - 2-23's own definition of
    droop, read as a curve.  That is a *law* and not a behavior: it reads the power
    demanded and the ceiling of the air the aircraft is in, and nothing else, so
    no time constant in this model can move it.  Not touching it is the constant's
    first duty, and this is the table where that duty is discharged.

    The demands are the model's own: the two trimmed hovers of the aircraft that
    ship (the report's 8700 lb one at 52 per cent of the ceiling, which is the
    number ``engine.py``'s self test holds the band to, and ``main.py``'s 6158 lb
    one at 37), read at the 324 rpm reference the psi arithmetic is exact at.
    """
    governor = Governor()
    ceiling = governor.power_available()
    omega = UH1_RPM * 2.0 * math.pi / 60.0
    say()
    say("the settled law, which no time constant in this model can move: a demand")
    say("settles P / gain under the selected speed, clipped to the +-40 rpm band")
    say("    demand   of max     power     torque     torque     droop     droop"
        "     droop   settles at")
    say("         %        %        kW        N m        psi        N2"
        "     rotor         %      rotor %")
    rows = []
    for name, power in (("no load", 0.0),
                        ("8700 lb hover", hover_powers[0]),
                        ("6158 lb hover", hover_powers[1]),
                        ("full power", ceiling)):
        torque = power / omega
        psi = torque / UH1_TORQUE_NM_PER_PSI
        droop = governor.droop_engine_rpm(power)
        droop_rotor = droop / governor.ratio
        per_cent = 100.0 * droop_rotor / UH1_RPM_100
        say("  %8.1f %8.1f %9.1f %10.0f %10.1f %9.2f %9.2f %9.2f %13.2f  %s"
            % (100.0 * power / ceiling, 100.0 * power / UH1_ENGINE_POWER_W,
               power / 1000.0, torque, psi, droop, droop_rotor, per_cent,
               governor.settled_rotor_rpm(power), name))
        rows.append(dict(name=name, power=power, torque=torque, psi=psi,
                         droop=droop, droop_rotor=droop_rotor,
                         per_cent=per_cent,
                         settled=governor.settled_rotor_rpm(power)))
    band_rotor = UH1_ENGINE_DROOP_RPM / governor.ratio
    band_per_cent = 100.0 * band_rotor / UH1_RPM_100
    say("  the band: %.0f rpm of N2 (2-23), %.2f rpm of rotor, %.2f %% of the"
        " %.0f rpm reference,"
        % (UH1_ENGINE_DROOP_RPM, band_rotor, band_per_cent, UH1_RPM_100))
    say("  and the gain that makes it so is the whole %.0f psi of ceiling per"
        " band: %.0f W" % (UH1_TRANSMISSION_LIMIT_PSI, governor.gain()))
    say("  per rpm of N2.  So the band is %.2f %% of the gauge the rotor is read"
        " on, and the" % (band_per_cent,))
    say("  manual's own green arc is %.1f %% wide: the loop that holds the rotor"
        " speed is" % (GREEN_ARC_PERCENT[1] - GREEN_ARC_PERCENT[0],))
    say("  stiff, and it is *constant-free*.  What the %.1f sec is worth is not"
        " where the loop" % (UH1_ENGINE_TIME_CONSTANT_S,))
    say("  settles but how long it takes to get there, which is the rest of this"
        " file.")
    return ceiling, rows


def words(value, form="%6.2f", none="never"):
    """A number for the transcript, or the words for not having one."""
    return none if value is None else form % value



def fraction_at(trace, when):
    """The last value a ``(s, value)`` trace has at or before *when*."""
    value = trace[0][1]
    for at, sample in trace:
        if at > when:
            break
        value = sample
    return value


def seconds_at(trace, level):
    """The first instant a ``(s, value)`` trace reaches *level*, or ``None``."""
    for at, sample in trace:
        if sample >= level:
            return at
    return None


def mean_at(trace, since):
    """The mean of a ``(s, value)`` trace over the samples at or after *since*."""
    window = [value for at, value in trace if at >= since]
    return sum(window) / len(window) if window else trace[-1][1]


def the_lag(taus):
    """The lag on its own, with the rotor held: the shape, and the clock.

    The two halves of a first order lag, measured with the governor's answer taken
    as read - the rise from nothing to a target, which is a *shape* every constant
    in the sweep flies identically (63.2 per cent of the way at one time constant,
    95.0 at three, 99.3 at five) and a *clock* that is the constant itself, and
    then the same arithmetic run backwards with the fuel gone, which is what makes
    a failure a wind down rather than a switch.

    All of it is ``Engine.advance``'s own arithmetic, one frame at a time at the
    frame loop's own step (so the clock columns carry the frame's 1/60 sec, and the
    three shape columns are read off exact ``advance(tau, ...)`` calls instead), and
    the rotor is held five per cent slow - the case the governor is for, where the
    error is 330 engine rpm and the gain asks for everything there is.
    """
    slow = HOLD_FRACTION * UH1_RPM
    say()
    say("the lag on its own, the rotor held at %.0f %% of %.0f rpm: the target"
        " is the" % (100.0 * HOLD_FRACTION, UH1_RPM))
    say("ceiling, and the decay is the same engine with the fuel gone")
    say("     tau    63 % at    95 % at    99 % at   rise: fraction reached"
        " after      decay: power left after      gone")
    say("       s       6.3e-1    9.5e-1    9.9e-1     0.5    1.0    2.0    3.0"
        "    5.0     0.5    1.0    2.0    3.0       s")
    rows = []
    for tau in taus:
        engine = engine_for(tau)
        value = tau_of(engine)
        target = engine.power_target(slow)
        engine.power = 0.0
        engine.advance(value, slow)
        at_one = engine.power / target
        engine.advance(2.0 * value, slow)
        at_three = engine.power / target
        engine.power = 0.0
        trace = [(0.0, 0.0)]
        when = 0.0
        while when < RISE_WINDOW_S and engine.power < target:
            engine.advance(SIM_TIME_STEP_S, slow)
            when += SIM_TIME_STEP_S
            trace.append((when, engine.power / target))
        at_63 = seconds_at(trace, 0.632)
        at_95 = seconds_at(trace, 0.95)
        at_99 = seconds_at(trace, 0.99)
        dead = Engine(governor=Governor(time_constant=value, failed=True))
        dead.power = UH1_ENGINE_POWER_W
        decay = [(0.0, 1.0)]
        when = 0.0
        while when < SPOOL_FOR_S and dead.power > GONE_FRACTION \
                * UH1_ENGINE_POWER_W:
            dead.advance(SIM_TIME_STEP_S, UH1_RPM)
            when += SIM_TIME_STEP_S
            decay.append((when, dead.power / UH1_ENGINE_POWER_W))
        gone = seconds_at(
            [(at, 1.0 - sample) for at, sample in decay], 1.0 - GONE_FRACTION)
        say("  %7.2f %10s %10s %10s %7.2f %6.2f %6.2f %6.2f %6.2f %7.2f %6.2f"
            " %6.2f %6.2f %7s"
            % (value, words(at_63), words(at_95), words(at_99),
               fraction_at(trace, 0.5), fraction_at(trace, 1.0),
               fraction_at(trace, 2.0), fraction_at(trace, 3.0),
               fraction_at(trace, 5.0),
               fraction_at(decay, 0.5), fraction_at(decay, 1.0),
               fraction_at(decay, 2.0), fraction_at(decay, 3.0),
               words(gone)))
        rows.append(dict(tau=value, target=target, at_one=at_one,
                         at_three=at_three, trace=trace, decay=decay,
                         at_63=at_63, at_95=at_95, at_99=at_99, gone=gone))
    say("  the shapes are the lag's own and are the same at every row: %.4f of"
        " the target at" % (rows[0]["at_one"],))
    say("  one time constant and %.4f at three (the two numbers"
        " ``engine.py``'s own self test" % (rows[0]["at_three"],))
    say("  asserts), so what a constant chooses is only *when* - which is why the"
        " columns")
    say("  that move between rows are the clock columns and not the shape ones.")
    return rows



def the_spool(taus):
    """A dead shaft in a trimmed hover, the fuel coming back on: flown.

    The one maneuver in this model in which a slow engine has to be caught by a
    live governor.  The rotor is settled on the trim with the whole aircraft's
    weight hanging on it, the engine's own state is emptied - ``Engine.reset``,
    which is a shaft with nothing in it *and* a governor that is still alive and
    still asking, and so is not a failure - and the frame loop is then left alone
    for :data:`SPOOL_FOR_S` seconds.  The rotor decays until the gas producer's
    answer catches up, and what the constant decides is how far it decays, whether
    it stays inside the manual's own green arc, and how long the loop takes to put
    it back on the trim.

    That measures *both* edges of the bracket.  Too fast and the engine is a
    switch that never lets the rotor move (:data:`NO_LAG_S`'s row); too slow and
    the gas arrives after the rotor has left the arc, which is what the row at the
    top of :data:`TAUS` is flown for.  Every row starts from the same two seconds
    of settled hover, so the rows differ in one number each.
    """
    say()
    say("a dead shaft in a trimmed hover, the fuel coming back on, flown:"
        " %d rows" % (len(taus),))
    say("     tau   rotor at the trim     low point      reached at     back at"
        "    n2 low   peak power   of max    the arc")
    say("       s                    %          rpm              s           s"
        "        rpm           kW         %")
    rows = []
    for tau in taus:
        engine = engine_for(tau)
        value = tau_of(engine)
        sim = Simulation(airframe=airframe_preset(REPORT_PRESET, engine=engine))
        for _ in range(int(SETTLE_S / SIM_TIME_STEP_S)):
            sim.fly(SIM_TIME_STEP_S)
        start = sim.telemetry()
        engine.reset()
        low, low_at = start.rotor_percent, sim.sim_time
        low_n2 = start.engine.rpm
        peak, peak_at = 0.0, sim.sim_time
        back = None
        trace = [(sim.sim_time, start.rotor_percent)]
        while sim.sim_time < SETTLE_S + SPOOL_FOR_S and not sim.crashed:
            sim.fly(SIM_TIME_STEP_S)
            telemetry = sim.telemetry()
            if telemetry.rotor_percent < low:
                low, low_at = telemetry.rotor_percent, sim.sim_time
            low_n2 = min(low_n2, telemetry.engine.rpm)
            if telemetry.engine.power > peak:
                peak, peak_at = telemetry.engine.power, sim.sim_time
            if back is None and sim.sim_time > low_at + 0.5 \
                    and abs(telemetry.rotor_percent - start.rotor_percent) \
                    < RECOVERED_PERCENT:
                back = sim.sim_time
            trace.append((sim.sim_time, telemetry.rotor_percent))
        telemetry = sim.telemetry()
        say("  %7.2f %17.2f %11.2f %12.2f %13s %9.0f %12.1f %8.1f   %s%s"
            % (value, start.rotor_percent, low, low_at, words(back), low_n2,
               peak / 1000.0, 100.0 * peak / UH1_ENGINE_POWER_W,
               "left   " if low < GREEN_ARC_PERCENT[0] else "inside",
               "  crashed" if sim.crashed else ""))
        rows.append(dict(tau=value, start=start.rotor_percent, low=low,
                         low_at=low_at, low_n2=low_n2, peak=peak,
                         peak_at=peak_at, back=back, trace=trace,
                         sim=sim, telemetry=telemetry,
                         dip=start.rotor_percent - low))
    first = rows[0]
    say("  so the dip the shipped constant pays for a dead shaft, and the time it"
        " pays it in:")
    say("  %.2f %% of the reference at %.2f sec, the rotor back on the trim %.2f s"
        " after leaving" % (first["dip"], first["tau"],
                            first["back"] - first["low_at"]))
    say("  it - against a droop band %.2f %% wide and the arc's floor %.1f %%"
        " below, and a" % (100.0 * UH1_ENGINE_DROOP_RPM / UH1_ENGINE_MAIN_RATIO
                           / UH1_RPM_100, 100.0 - GREEN_ARC_PERCENT[0]))
    say("  recovery criterion of %.2f %%: the criterion is a sixteenth of the"
        " band, so the" % (RECOVERED_PERCENT,))
    say("  seconds beside it are a settled rotor's and not a passing one's.")
    return rows



def the_pull(taus):
    """The lever from the trim to the up stop: the transient, on the manual's band.

    The second ruler on the same demand.  A pull is where the *settled* law is
    read - the droop the manual quotes is a settled droop - and the lag's cost is
    what the rotor does on the way there: a dip *below* the speed the loop is going
    to settle at, which is the droop of the power the pull finally asks for plus
    however much of that power the gas producer has not answered yet.  The lever is
    moved at the axis' own rate (``simulation.PilotInput.collective_rate``, 0.55 of
    its travel a second, so a second and a half over the last inch and a quarter),
    because a script that slammed it would be measuring a step no hand can make.

    The two parts are printed apart, which is the point of the table: the settled
    part is a number off the manual's curve for the power the pull ends up asking
    for, and the transient part is the rest of the dip, which is the constant's own.
    """
    say()
    say("the lever from the trim to the up stop, flown: the one pull the manual's")
    say("own droop definition is about, and what the lag adds to it")
    say("     tau   peak torque   of max     peak power    ends on    settled"
        " there   the flown dip   back at")
    say("       s             %        %             kW         kW"
        "       % of 324       %       rpm           s")
    rows = []
    for tau in taus:
        engine = engine_for(tau)
        value = tau_of(engine)
        sim = Simulation(airframe=airframe_preset(REPORT_PRESET, engine=engine))
        for _ in range(int(SETTLE_S / SIM_TIME_STEP_S)):
            sim.fly(SIM_TIME_STEP_S)
        start = sim.telemetry()
        low, low_at = start.rotor_percent, sim.sim_time
        peak, peak_at = 0.0, sim.sim_time
        trace = [(sim.sim_time, start.rotor_percent)]
        while sim.sim_time < SETTLE_S + PULL_FOR_S and not sim.crashed:
            sim.pilot.set_axes(collective=1.0, dt=SIM_TIME_STEP_S)
            sim.step(SIM_TIME_STEP_S, sim.pilot.controls())
            telemetry = sim.telemetry()
            if telemetry.rotor_percent < low:
                low, low_at = telemetry.rotor_percent, sim.sim_time
            if telemetry.engine.power > peak:
                peak, peak_at = telemetry.engine.power, sim.sim_time
            trace.append((sim.sim_time, telemetry.rotor_percent))
        telemetry = sim.telemetry()
        end_power = telemetry.engine.power
        # What the loop settles at is the *new* demand's droop and not the trim the
        # pull started from - the lever ends somewhere else - so the recovery is
        # read against the speed the last second of the pull holds.
        settled_end = mean_at(trace, sim.sim_time - RECOVERED_MEAN_S)
        back = seconds_at([(at, value) for at, value in trace if at > low_at],
                          settled_end - RECOVERED_PERCENT)
        settled_per_cent = 100.0 * engine.governor.droop_engine_rpm(end_power) \
            / UH1_ENGINE_MAIN_RATIO / UH1_RPM_100
        say("  %7.2f %12.1f %9.1f %14.1f %10.1f %15.2f %7.2f %7.2f %9s"
            % (value, telemetry.engine.torque_percent,
               telemetry.engine.maximum_percent, peak / 1000.0,
               end_power / 1000.0, 100.0 - settled_per_cent,
               start.rotor_percent - low, (start.rotor_percent - low) / 100.0
               * UH1_RPM_100, words(back)))
        rows.append(dict(tau=value, start=start.rotor_percent, low=low,
                         low_at=low_at, peak=peak, peak_at=peak_at, back=back,
                         end_power=end_power, settled_per_cent=settled_per_cent,
                         dip=start.rotor_percent - low, sim=sim,
                         telemetry=telemetry,
                         law_dip=start.rotor_percent - (100.0 - settled_per_cent)))
    first = rows[0]
    say("  the pull ends on %.0f per cent of the transmission, the manual's %.0f"
        " psi line, and the"
        % (first["telemetry"].engine.maximum_percent,
           UH1_TRANSMISSION_LIMIT_PSI))
    say("  trim it started from is already %.2f %% under the reference - the"
        " hover's own droop - so" % (100.0 - first["start"],))
    say("  the *law* accounts for %.2f %% of the %.2f %% the rotor dips and the"
        " gas producer's lag"
        % (first["law_dip"], first["dip"]))
    say("  the other %.2f %%, against a band %.2f %% wide: a lever pull is where"
        " the manual's curve"
        % (first["dip"] - first["law_dip"], first["settled_per_cent"]))
    say("  is read, and where the constant is worth a fifth of what the curve is."
        )
    return rows



def the_emergency(ceiling, hover_power):
    """The GOV switch to EMER in a hover, flown: the lag with nothing else in the loop.

    EMER is where the constant stops being a *correction* on the rotor's own clock
    and becomes the whole of the engine's dynamics.  9-3's "manual control of the
    engine RPM" takes the speed feedback out, so ``power_target`` is the ceiling
    times the grip and the gas producer's lag is all that is left between a hand and
    the torque - which is what that page's warning about EMER is about, and it is
    the one flight in this model where a number of this size is not buried under the
    governor's gain.

    Two rows, on ``main.py``'s aircraft - the one whose GOV switch a person can
    reach.  The first leaves the grip where the AUTO hover had it, which is full
    open: it is what a pilot who throws the switch and touches nothing flies.  The
    second puts it at the fraction the hover's own power demand is of the ceiling,
    which is the position a pilot has to find by hand and is the whole job in EMER.
    """
    say()
    say("the GOV switch to EMER in a hover, the grip as the only fuel control:")
    say("   grip   rotor at the trim      high   reached at   crossed the arc's"
        " top at       power    the arc")
    say("      %                     %         %            s"
        "                         s          kW")
    rows = []
    for name, grip in (("full open", None),
                       ("the hover's own", hover_power / ceiling)):
        sim = Simulation(airframe=airframe_preset(FLIGHT_PRESET, engine=True))
        for _ in range(int(SETTLE_S / SIM_TIME_STEP_S)):
            sim.fly(SIM_TIME_STEP_S)
        start = sim.telemetry()
        if grip is not None:
            sim.set_throttle(grip)
        sim.set_governor(emer=True)
        high, high_at = start.rotor_percent, sim.sim_time
        top = None
        while sim.sim_time < SETTLE_S + EMER_FOR_S and not sim.crashed:
            sim.fly(SIM_TIME_STEP_S)
            telemetry = sim.telemetry()
            if telemetry.rotor_percent > high:
                high, high_at = telemetry.rotor_percent, sim.sim_time
            if top is None and telemetry.rotor_percent > GREEN_ARC_PERCENT[1]:
                top = sim.sim_time
        telemetry = sim.telemetry()
        say("  %5.0f %21.2f %9.2f %12.2f %25s %10.1f %10s   %s"
            % (100.0 * sim.pilot.throttle(), start.rotor_percent, high, high_at,
               words(top, "%6.2f"), telemetry.engine.power / 1000.0,
               "left" if high > GREEN_ARC_PERCENT[1] else "inside", name))
        rows.append(dict(grip=sim.pilot.throttle(), name=name,
                         start=start.rotor_percent, high=high, high_at=high_at,
                         top=top, power=telemetry.engine.power, sim=sim,
                         telemetry=telemetry))
    first = rows[0]
    say("  with the grip full open the rotor leaves the arc's own top - %.1f %% -"
        " %.2f s after" % (GREEN_ARC_PERCENT[1], first["top"] - SETTLE_S))
    say("  the switch, and is at %.0f %% of the reference %s s later with the"
        " ceiling's whole" % (first["high"], words(EMER_FOR_S, "%.0f")))
    say("  %.0f kW still going into it: that is 9-3's overspeed, in the model."
        % (first["power"] / 1000.0,))
    say("  The second row is the same switch with the grip moved by hand to"
        " %.0f %%, and the" % (100.0 * rows[1]["grip"],))
    say("  rotor %s: in EMER the grip *is* the fuel control, so the speed a pilot"
        " has is one"
        % ("holds" if rows[1]["high"] < GREEN_ARC_PERCENT[1] else "runs away",))
    say("  held by ear and by the gauge rather than one a governor holds for"
        " them.")
    return rows


def the_entry(entries, stream, clocks):
    """The mission's own failure entry at three constants: borrowed, not copied.

    ``_rt_autoro_entry.py`` is the instrument that measures what the model does
    between ``main.py``'s X and the descent that follows it, with the mission's own
    pilot at the controls, and the number it commits - a 190 ft catch in seven and
    a half seconds - is quoted in ``README.md``.  This table flies that entry again
    at three constants by handing ``flight`` an engine of its own: the published
    number is then read against the constant it is cut with, instead of being
    assumed independent of it, and a constant that moved the catch would move a
    number the project's documents already carry.

    The import is inside this function on purpose.  The entry is flown by the
    mission's own loops, which are ``_rt_pilot``'s, which are pygame - so a file
    that imported them at the top would need a display to print the rotor's own
    arithmetic, and the six tables above it want nothing but the model.
    """
    import os

    os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
    import _rt_autoro_entry
    from _rt_pilot import AutorotationPilot, REACTION_S, trims_for

    trims = trims_for(airframe_preset(FLIGHT_PRESET), AutorotationPilot)
    say()
    say("the mission's own autorotation, the failure switch, at %d constants:"
        " flown by" % (len(entries),))
    say("_rt_autoro_entry.flight itself, handed the engine rather than building"
        " one, so")
    say("the catch the README quotes is read against the constant it is cut with")
    rows = []
    for tau in entries:
        engine = engine_for(tau)
        value = tau_of(engine)
        say()
        say("the failure entry on a gas producer of %.2f sec, %.2f s between the"
            " mission's own decisions:" % (value, REACTION_S))
        flown = _rt_autoro_entry.flight(_rt_autoro_entry.KILL_BY_FAILURE,
                                        REACTION_S, trims, stream,
                                        engine=engine)
        gone = flown.torque_gone_at - flown.switch_s
        say("  the catch: %.1f ft of skid in %.1f s, the rotor down to %.1f %% of"
            " the reference,"
            % (flown.catch_ft, flown.settle_s, flown.lowest_rotor))
        say("  the skids on the ground at %.2f m/s with the rotor at %.1f %%."
            % (flown.touchdown_rate, flown.rotor_at_touchdown))
        say("  the fuel counted gone %.2f s after the switch - %.1f time"
            " constants, which is" % (gone, gone / value))
        say("  the whole of what the constant is for on this flight.  The gauge"
            " empties sooner")
        say("  than the power does, because it reads the torque at the rotor and"
            " the rotor is")
        say("  winding down under it - Q = P / omega, and omega is falling - so"
            " %.1f time" % (gone / value,))
        say("  constants of gauge is the 4.6 a fixed speed would have shown.")
        rows.append(dict(tau=value, flown=flown, gone=gone,
                         gone_over_tau=gone / value))
    say()
    say("  so the published catch, against the constant under it: %s"
        % (", ".join("%.0f ft at %.2f s" % (row["flown"].catch_ft, row["tau"])
                     for row in rows),))
    say("  - and the entry is *insensitive* to it in the way the shaft's own"
        " seconds are not:")
    say("  the fuel's departure moves from %.2f s to %.2f s across the bracket"
        " while the catch"
        % (min(row["gone"] for row in rows),
           max(row["gone"] for row in rows)))
    say("  moves %.1f ft, because a rotor this heavy reads the shaft's seconds"
        " through an"
        % (max(row["flown"].catch_ft for row in rows)
           - min(row["flown"].catch_ft for row in rows),))
    say("  inertia %.0f times the engine's own clock (see the two clocks)."
        % (clocks[0]["clock"] / UH1_ENGINE_TIME_CONSTANT_S,))
    return rows


def what_reads_it(spools):
    """The constant's boundary: one line of the model, and a flight downstream of it.

    ``_rt_tail.py``'s boundary measures that *nothing* which flies reads the tail
    rotor's two constants.  This file's question is the opposite one, so its
    boundary is the opposite answer, and there are two halves to it.

    The first half is the reading: the same spool flown twice, once on the shipped
    engine and once on the instant torque the constant's own docstring refuses,
    with the rotor's low points the fraction of a per cent apart that is the
    constant's whole contribution on that flight.  The second half is the *once*:
    ``Governor.time_constant`` takes its default from the module constant when the
    class is built, so rebinding ``engine.UH1_ENGINE_TIME_CONSTANT_S`` afterwards
    leaves every engine built later still at the old number.  The constant is
    therefore read at import and nowhere else, which is why this transcript is the
    source the constant's own "no source" tag needs.
    """
    import engine as engine_module

    shipped = spools[0]
    instant = [row for row in spools if row["tau"] == NO_LAG_S][0]
    say()
    say("the constant's boundary - what reads it, and how often:")
    say("  the flight reads it: the same dead shaft in the same hover leaves the"
        " rotor at")
    say("  %.2f %% on the shipped %.2f sec engine and %.2f %% on an instant one,"
        " which is"
        % (shipped["low"], shipped["tau"], instant["low"]))
    say("  %.2f %% of the reference - small, and not nothing."
        % (instant["low"] - shipped["low"],))
    before = engine_module.UH1_ENGINE_TIME_CONSTANT_S
    engine_module.UH1_ENGINE_TIME_CONSTANT_S = 2.5
    try:
        after = tau_of(Engine())
    finally:
        engine_module.UH1_ENGINE_TIME_CONSTANT_S = before
    say("  and it is read once: a bare Engine() carries %.2f sec, the shipped"
        " constant, and" % (tau_of(Engine()),))
    say("  rebinding the module's %.1f to 2.5 leaves the next one at %.2f, since"
        " the dataclass" % (before, after))
    say("  took its default when the class was built.  So the model reads the"
        " number at import")
    say("  and this run is what the number is read *from*.")
    return dict(shipped=shipped, instant=instant, rebinding=after, before=before)


def claims(clocks, ceiling, law_rows, lag_rows, spool_rows, pull_rows, emer_rows,
           entry_rows, reading):
    """The claims the transcript makes, asserted: a regression is a traceback.

    Everything here was measured first and written down second, and every number is
    one the printout above already carries.  What is asserted is the *shape* of the
    answer rather than the last figure of it: the two clocks and the order of
    magnitude between them, the band that no constant in the sweep moves, the lag's
    own fixed shape against its moving clock, the bracket the spool draws around the
    constant, how nearly invisible it is on a lever pull, EMER's runaway, and the
    published autorotation catch - which is the one number here that a document
    outside this file already quotes, in ``README.md``.
    """
    held = 0

    # The two clocks: the same inertia on both aircraft, the heavier hover's the
    # shorter time, and the engine's an order of magnitude shorter than either.
    assert len(clocks) == 2, clocks
    assert clocks[0]["inertia"] == clocks[1]["inertia"], clocks
    assert clocks[0]["clock"] < clocks[1]["clock"], clocks
    assert 6.5 < clocks[0]["clock"] < 7.5, clocks[0]["clock"]
    assert 9.0 < clocks[1]["clock"] < 10.5, clocks[1]["clock"]
    assert abs(clocks[0]["torque"] - 16266.0) < 40.0, clocks[0]["torque"]
    held += 6
    for row in clocks:
        assert row["clock"] / row["tau"] > 10.0, row
        held += 1

    # The manual's own ruler, which no constant touches: zero at no load, the whole
    # band at full power, and the same settled speed and droop for every constant in
    # the sweep - which is what makes the *law* one ruler and the clock the other.
    assert abs(law_rows[0]["droop"]) < 1e-12, law_rows[0]
    assert abs(law_rows[1]["droop"] - 20.87) < 0.1, law_rows[1]["droop"]
    assert abs(law_rows[2]["droop"] - 14.86) < 0.1, law_rows[2]["droop"]
    assert abs(law_rows[3]["droop"] - UH1_ENGINE_DROOP_RPM) < 1e-9, law_rows[3]
    assert abs(law_rows[1]["psi"] - 26.2) < 0.2, law_rows[1]["psi"]
    assert abs(Governor().gain() * UH1_ENGINE_DROOP_RPM - ceiling) < 1e-6
    held += 6
    hover = law_rows[1]["power"]
    for row in lag_rows:
        governor = Governor(time_constant=row["tau"])
        assert abs(governor.settled_rotor_rpm(hover)
                   - law_rows[1]["settled"]) < 1e-12, row
        assert abs(governor.droop_engine_rpm(hover)
                   - law_rows[1]["droop"]) < 1e-12, row
        held += 2

    # The lag's own arithmetic: one fixed shape at every constant (the two numbers
    # engine.py's self test asserts), a clock that is the constant, and the same
    # arithmetic run backwards as the fuel goes - which is what makes a failure a
    # wind down, and an instant engine a *switch*: the row the constant's own
    # docstring asks to be refused.
    for row in lag_rows:
        assert abs(row["at_one"] - (1.0 - 1.0 / math.e)) < 1e-9, row["tau"]
        assert abs(row["at_three"] - (1.0 - math.exp(-3.0))) < 1e-9, row["tau"]
        held += 2
        if row["tau"] > 0.1:
            assert row["at_63"] is not None \
                and abs(row["at_63"] - row["tau"]) < 0.05, row["tau"]
            assert row["at_95"] is not None \
                and abs(row["at_95"] - 3.0 * row["tau"]) < 0.15, row["tau"]
            assert abs(fraction_at(row["decay"], 3.0 * row["tau"])
                       - math.exp(-3.0)) < 0.02, row["tau"]
            held += 3
    shipped_lag = lag_rows[0]
    assert shipped_lag["tau"] == UH1_ENGINE_TIME_CONSTANT_S, shipped_lag
    assert 1.7 < shipped_lag["at_95"] < 1.9, shipped_lag["at_95"]
    assert 2.5 < shipped_lag["at_99"] < 3.0, shipped_lag["at_99"]
    assert 2.5 < shipped_lag["gone"] < 3.0, shipped_lag["gone"]
    held += 4
    instant_lag = [row for row in lag_rows if row["tau"] == NO_LAG_S][0]
    assert instant_lag["gone"] < 0.05, instant_lag["gone"]
    held += 1
    rising = sorted([row for row in lag_rows if row["tau"] != NO_LAG_S],
                    key=lambda row: row["tau"])
    for before, after in zip(rising, rising[1:]):
        assert before["tau"] < after["tau"], (before["tau"], after["tau"])
        # A bigger constant is slower at both ends of the same arithmetic: less of
        # the target reached a second in, and more of the fuel still in the shaft.
        assert fraction_at(before["trace"], 1.0) \
            > fraction_at(after["trace"], 1.0), (before["tau"], after["tau"])
        assert fraction_at(before["decay"], 1.0) \
            < fraction_at(after["decay"], 1.0), (before["tau"], after["tau"])
        held += 3

    # The spool, which is the bracket.  The dip grows with the constant, an instant
    # engine barely lets the rotor move at all, the shipped constant's dip is small
    # and inside the arc with a recovery a pilot can watch, and the *slowest*
    # constant in the sweep takes the rotor out of the arc - which is the edge.
    orders = sorted([row for row in spool_rows if row["tau"] != NO_LAG_S],
                    key=lambda row: row["tau"])
    for before, after in zip(orders, orders[1:]):
        assert after["dip"] > before["dip"], (before["tau"], after["tau"])
        held += 1
    instant_spool = [row for row in spool_rows if row["tau"] == NO_LAG_S][0]
    assert instant_spool["dip"] < orders[0]["dip"], instant_spool["tau"]
    shipped_spool = [row for row in spool_rows
                     if row["tau"] == UH1_ENGINE_TIME_CONSTANT_S][0]
    assert 1.5 < shipped_spool["dip"] < 3.5, shipped_spool["dip"]
    assert shipped_spool["low"] > GREEN_ARC_PERCENT[0], shipped_spool["low"]
    assert shipped_spool["back"] is not None, shipped_spool
    assert shipped_spool["back"] - shipped_spool["low_at"] < 4.0, shipped_spool
    assert orders[-1]["low"] < GREEN_ARC_PERCENT[0], orders[-1]["low"]
    held += 5

    # The lever pull: the demand never reaches the ceiling - a pull at the trim is a
    # climb and not a full-power demand - so the rotor never leaves the arc and the
    # lag is worth a fraction of what the manual's own curve is.  That is the
    # negative result this table exists for, and it is asserted as a negative.
    for row in pull_rows:
        assert row["peak"] <= ceiling, row["tau"]
        assert row["low"] > GREEN_ARC_PERCENT[0], row["tau"]
        held += 2
    shipped_pull = [row for row in pull_rows
                    if row["tau"] == UH1_ENGINE_TIME_CONSTANT_S][0]
    assert shipped_pull["dip"] < 0.5, shipped_pull["dip"]
    assert shipped_pull["dip"] > shipped_pull["law_dip"], shipped_pull
    held += 2

    # EMER: with the governor out of the loop and the grip where the AUTO hover had
    # it, the rotor runs away - 9-3's own caution, and the one flight here in which
    # the constant is not buried under the governor's gain.
    assert emer_rows[0]["top"] is not None, emer_rows[0]
    assert emer_rows[0]["high"] > 130.0, emer_rows[0]["high"]
    assert emer_rows[0]["high_at"] > SETTLE_S + 5.0, emer_rows[0]
    assert emer_rows[0]["grip"] > 0.99, emer_rows[0]["grip"]
    held += 4

    # The mission's own entry at three constants: it reaches the descent, it takes
    # the rotor out of the arc, and the skids arrive.  The *shipped* row is the
    # number ``_rt_autoro_hand.py`` commits as its baseline, to a foot and a tenth of
    # a second - which is what says this file flies that instrument and not a cousin
    # of it - and every row's fuel is gone within a few time constants of the switch.
    assert len(entry_rows) >= 3, entry_rows
    shipped_entry = [row for row in entry_rows
                     if row["tau"] == UH1_ENGINE_TIME_CONSTANT_S][0]
    assert abs(shipped_entry["flown"].catch_ft - 189.7) < 1.0, \
        shipped_entry["flown"].catch_ft
    assert abs(shipped_entry["flown"].settle_s - 7.5) < 0.2, \
        shipped_entry["flown"].settle_s
    assert abs(shipped_entry["flown"].held_sink - 12.20) < 0.05, \
        shipped_entry["flown"].held_sink
    held += 3
    for row in entry_rows:
        flown = row["flown"]
        assert not flown.sim.crashed, row["tau"]
        assert flown.settled_at is not None, row["tau"]
        assert flown.lowest_rotor < GREEN_ARC_PERCENT[0], row["tau"]
        assert flown.touchdown_at is not None, row["tau"]
        assert abs(row["gone_over_tau"] - 3.4) < 0.8, row["tau"]
        held += 5

    # The boundary: the flight *does* read the constant - an instant engine's spool
    # leaves the rotor 2.5 per cent higher than the shipped one's does - and it
    # reads it once, when the class is built.
    assert reading["shipped"]["dip"] > reading["instant"]["dip"] + 0.5, \
        (reading["shipped"]["tau"], reading["shipped"]["dip"],
         reading["instant"]["dip"])
    assert abs(reading["rebinding"] - reading["before"]) < 1e-12, reading["after"]
    assert reading["before"] == UH1_ENGINE_TIME_CONSTANT_S, reading["before"]
    held += 3
    return held


def main():
    """The survey: the clocks, the law, the lag, three flights, the entry, the count."""
    arguments = sys.argv[1:]
    taus = the_sweep(arguments)
    entries = the_entries(arguments)
    stream = open(OUT_PATH, "w", encoding="utf-8")
    sys.stdout = stream
    try:
        say("the gas producer's own time constant, measured rather than argued:")
        say("engine.py's UH1_ENGINE_TIME_CONSTANT_S is %.2f sec, and this run"
            " sweeps %d" % (UH1_ENGINE_TIME_CONSTANT_S, len(taus)))
        say("constants around it - %s - on both of the model's aircraft."
            % (", ".join("the model's own" if tau is None
                         else "the instant torque" if tau == NO_LAG_S
                         else "%.2f" % (tau,) for tau in taus),))
        say("The manual's own green arc is %.1f to %.1f %% of the %.0f rpm"
            " reference, the droop"
            % (GREEN_ARC_PERCENT + (UH1_RPM_100,)))
        say("band is +-%.0f rpm of N2 (2-23), and the shaft tables are flown on"
            " the \"%s\"" % (UH1_ENGINE_DROOP_RPM, REPORT_PRESET))
        say("aircraft with the \"%s\" one's EMER and autorotation after them."
            % (FLIGHT_PRESET,))
        clocks = the_two_clocks()
        ceiling, law_rows = the_settled_law(
            (clocks[0]["telemetry"].engine.power,
             clocks[1]["telemetry"].engine.power))
        lag_rows = the_lag(taus)
        spool_rows = the_spool(taus)
        pull_rows = the_pull(taus)
        emer_rows = the_emergency(ceiling, clocks[1]["telemetry"].engine.power)
        entry_rows = the_entry(entries, stream, clocks)
        reading = what_reads_it(spool_rows)
        held = claims(clocks, ceiling, law_rows, lag_rows, spool_rows, pull_rows,
                      emer_rows, entry_rows, reading)
        say()
        say("%d claims held: the survey above is the measurement the gas"
            " producer's own" % (held,))
        say("constant is cited from, and the bracket it sits inside is drawn by"
            " the spool")
        say("table's two edges and by the published catch in the last one.")
    finally:
        sys.stdout = sys.__stdout__
        stream.close()
    print("wrote %s" % (OUT_PATH,))
    return 0


if __name__ == "__main__":
    sys.exit(main())



