# -*- coding: utf-8 -*-
"""Scratch: the autorotation's other switch - the failure, not the script.

``_rt_autoro_probe.py`` measures the mission's own entry: the twist grip rolled
right off from a settled sixty knot cruise.  That is the entry a pilot is
*handed* - the grip's own key, held until the lever is at the stop, four seconds
of a quarter rate key, with the shaft's torque coming off over those seconds as
the fuel control follows the hand.

What a pilot is not handed is the switch ``main.py`` has for the same thing:
``X``, which is :meth:`simulation.Simulation.set_governor`'s ``failed=True`` and
takes the fuel away *now*, through the engine's own lag, with the grip left
wherever the hand had it.  That is the entry nothing scripts - it is the one a
person presses - and this file flies the mission's own pilot through it, and
again through the grip's own cut, so that what the switch changes is measured
rather than argued about.  Nothing else changes: the same ``"flight test"``
aircraft ``main.py`` opens a window on, fitted live on the pad the way that
window fits it, the same :class:`_rt_pilot.AutorotationPilot`, the same trim
table, and the same frame loop, one frame at a time.  The switch is thrown on
the frame the mission reaches its own cut, which is the instant the two entries
are comparable from - from there on the mission's hands are the same in both.

What comes back, into ``_rt_autoro_entry.txt``:

* the shaft, a second apart for the first six seconds after the switch: the
  torque gauge, and the moment the fuel was finally gone.  A failure is one lag
  where the grip's own cut is four seconds of a hand on a ratchet;
* the rotor, which is where the difference lands: what it was at the switch and
  at the skids, how far it dipped, and the seconds it spent outside the manual's
  green arc, ``aerodynamics.UH1_RPM_LOW`` to ``UH1_RPM_MAX`` - a free state with
  no stall and no flap limit behind it, and both entries take it out of the arc;
* the *catch*: the height lost between the switch and the sink first reaching the
  descent it goes on to hold, the seconds that took, and how deep the sink went on
  the way - the overshoot.  A failure out of a settled climb leaves the lever at
  the *level* trim of sixty knots, which is above the autorotation's own, so the
  loops have to find the descent with the collective up in a rotor with nothing
  driving it - which is the state the windmill note in ``TODO.md`` says is where
  equation 10's two roots are least forgiving;
* the arrival: the rate the skids met the ground at, against
  ``simulation.UH1_HARD_LANDING_MPS``, and whether a frame was *refused* -
  ``simulation.CRASH_OUT_OF_ENVELOPE``, the model declining to fly a state,
  which is not the floor's own hard arrival;
* and both entries again at a slower hand, because the second decision rate is
  what says how much of this is the *switch* and how much is the hand that works
  it: a slower hand takes both rotors further down the arc - the lever spends
  longer at the level trim with the shaft already dead - and on the grip's own
  cut, which is a key held for four seconds, it is worth a fifth of the rotor,
  while the failure, which is one lag and no hand at all, barely notices.  So
  *which* of the two entries is the worse one is the hand's answer.

The loops are still ``_rt_pilot.py``'s, so nothing here says that a person could
fly either entry by hand: that is the half ``TODO.md``'s ``Next`` still has open,
and this file's job is to say what the model does *between* the two switches.

The three helpers the autorotation's ruler already has - ``key_states``,
``skid_ft`` and ``say`` - are imported from ``_rt_autoro_probe.py`` rather than
written a second time, so a scancode or a transcript line moved in one moves in
both.

Usage::

    python _rt_autoro_entry.py [reaction_seconds ...]
"""

import sys

import pygame

from aerodynamics import UH1_RPM_100, UH1_RPM_LOW, UH1_RPM_MAX
from airframe import FOOT
from simulation import (CRASH_OUT_OF_ENVELOPE, SIM_TIME_STEP_S, Simulation,
                        UH1_HARD_LANDING_MPS, airframe_preset)
from _rt_autoro_probe import key_states, say, skid_ft
from _rt_pilot import (AUTOROTATION_SPEED_KT, AUTOROTATION_SPEED_MPS,
                       AutorotationPilot, CLIMB_SKID_FT, CUT_GRIP_KEY,
                       REACTION_S, trims_for)
from main import KEY_NAMES

#: The aircraft the window opens on and this file flies - the one
#: ``_rt_autoro_probe.py`` is cut with, so the two entries are the same aircraft.
PRESET = "flight test"

#: Nothing lasts forever, and a probe that never arrives should say so.
MAX_FLIGHT_S = 240.0

#: A line a second from the switch on, and a line every quarter second inside the
#: last ten feet of skid, where the cushion's work is either done or not.
REPORT_S = 1.0
ARRIVAL_SKID_FT = 10.0
ARRIVAL_REPORT_S = 0.25

#: The shaft, a second apart for the first six seconds after the switch: long
#: enough to hold one entry's lag and the other's hand side by side, and short
#: enough to read.  The fuel counts as gone at this much of the gauge.
TORQUE_REPORT_S = 1.0
TORQUE_REPORT_FOR_S = 6.0
TORQUE_GONE_PERCENT = 1.0

#: What "the descent has settled" means, and the window the rate it settled *on* is
#: read from: inside this fraction of that rate for this many seconds, and
#: averaged over this many seconds ending at the cushion the mission opened.  The
#: rate is the descent this flight goes on to hold and not the trim's own, because
#: the model overshoots the trim's by a few per cent once the rotor is windmilling
#: - see what the transcript prints - and a criterion written against a number the
#: model does not fly would never be met by either entry.
SETTLE_TOLERANCE = 0.10
SETTLE_HOLD_S = 2.0
SETTLE_MEAN_S = 3.0

#: How far the descent a flight *holds* may sit from the trim's own rate, m/s.  The
#: model's own descent at this weight and height is about 0.7 m/s steeper than the
#: trim it was solved for - the trim is a steady state at the top of the climb and
#: the descent is flown at sixty knots on the way down - and that is printed on
#: every line rather than hidden inside the band.
DESCENT_BAND_MPS = 1.0

#: The manual's own green arc in the per cent the captions are in: 294 to 339 rpm
#: around 324, so 90.7 to 104.6.
GREEN_ARC_PERCENT = (100.0 * UH1_RPM_LOW / UH1_RPM_100,
                     100.0 * UH1_RPM_MAX / UH1_RPM_100)

#: The two ways of killing the shaft, in the order they are flown: the mission's
#: own grip cut first, because everything the failure does is read against it.
KILL_BY_GRIP = "grip cut"
KILL_BY_FAILURE = "failure"

#: What each way is, in the transcript's own words.
WORDS = {
    KILL_BY_GRIP: ("the mission's own entry, and the one _rt_autoro_probe.py"
                   " measures: the twist grip rolled right off on its own key,"
                   " held until the lever is at the stop"),
    KILL_BY_FAILURE: ("the entry nothing scripts: main.py's X -"
                      " set_governor(failed=True) - on the frame the mission"
                      " reaches its cut, with the grip where the hand left it"),
}

#: The decision rates both entries are flown at, s: the mission's own
#: :data:`_rt_pilot.REACTION_S`, and one a little slower.  Two, because the cost
#: of the switch turns out to be the *hand* as much as the switch - see
#: :func:`check`, which is where the two are read against each other - and one
#: rate alone cannot tell the two apart.  ``_rt_check.py`` flies its mission at
#: two for the same reason.
REACTIONS = (REACTION_S, 0.35)

OUT_PATH = "_rt_autoro_entry.txt"


def words(value, form="%.1f", none="not measured"):
    """A number for the transcript, or the words for not having one.

    Half of what this file prints is absent in one of the two flights - a settle
    the flight never reached, a fuel that never went - and a table with a blank in
    it reads as a bug rather than as an answer.
    """
    return none if value is None else form % value


class Flight(object):
    """One killing of the shaft, flown: the sim, the pilot, and what came back.

    Every number here is read off frames the loop has already flown, so nothing in
    it can change the flight - the same discipline ``_rt_check.Run`` keeps.  What
    it holds is the switch's own instant, what the shaft was still doing after it,
    how far the rotor dipped and for how long it was under the arc, how long the
    entry took to reach the descent it goes on to hold and what that cost in
    height, and the arrival.
    """

    def __init__(self, way, sim, pilot, switch_s, sink_ref):
        #: Which way killed it: :data:`KILL_BY_GRIP` or :data:`KILL_BY_FAILURE`.
        self.way = way
        self.sim = sim
        self.pilot = pilot
        #: The flight's own clock at the switch, s.
        self.switch_s = float(switch_s)
        #: The rate the trim says a sixty knot descent holds, m/s, positive down:
        #: what the catch is measured against and what the arrival is read with.
        self.sink_ref = float(sink_ref)
        telemetry = sim.telemetry()
        #: Where the aircraft was when the fuel went: the height and the skid
        #: height, the rotor speed, and the torque the shaft was still carrying.
        self.switch_alt = telemetry.alt_agl
        self.switch_skid_ft = skid_ft(sim)
        self.switch_rotor = telemetry.rotor_percent
        self.switch_torque = telemetry.engine.torque_percent
        self.switch_throttle = sim.pilot.throttle()
        #: The rotor's own low point and when it was reached, the seconds spent
        #: under the manual's own green arc - and how many of those were in the
        #: air rather than on the skids after it, since the model lets it wind
        #: down either way: the rotor speed is a free state with no stall and no
        #: flap limit behind it, see the windmill note in ``TODO.md``.
        self.lowest_rotor = telemetry.rotor_percent
        self.lowest_rotor_at = self.switch_s
        self.under_arc_s = 0.0
        self.under_arc_air_s = 0.0
        #: The torque gauge a second apart for the first six seconds of it, and
        #: the instant the fuel was finally gone.
        self.torque_after = []
        self.torque_gone_at = None
        #: The flight, sample by sample - ``(s, m/s down, m of altitude, phase)`` -
        #: which is what the settle below is read off, and the deepest sink of the
        #: descent, which is the overshoot the entry went through to get there.
        self.samples = []
        self.peak_sink = 0.0
        #: The descent as flown and as it settled: the rate it went on holding,
        #: when the sink first reached it, and the height that cost.  All three are
        #: filled in by :meth:`finish`, since the rate is not known while it is
        #: still changing.
        self.held_sink = None
        self.settled_at = None
        self.settled_sink = None
        self.settled_alt = None
        #: The arrival: when the skids met the floor and at what rate, the rotor
        #: that arrived on them, and the frame the model refused, if it refused one.
        self.touchdown_at = None
        self.touchdown_rate = None
        self.rotor_at_touchdown = None
        self.refused_at = None
        self.crash_reason = None
        self.last_airborne = 0.0
        self.next_torque = self.switch_s

    # -- the readings ----------------------------------------------------

    def mean_sink(self, since, until):
        """The mean sink rate between two times of the flight's clock, m/s down."""
        window = [sink for when, sink, alt, phase in self.samples
                  if since <= when <= until]
        return sum(window) / len(window) if window else None

    @property
    def catch_ft(self):
        """The height between the switch and the settle, ft, or None."""
        if self.settled_alt is None:
            return None
        return (self.switch_alt - self.settled_alt) / FOOT

    @property
    def settle_s(self):
        """How long the descent took to settle, s, or None."""
        return None if self.settled_at is None else self.settled_at - self.switch_s

    def measure(self):
        """One frame's worth, from the state the frame loop has just left."""
        telemetry = self.sim.telemetry()
        now = float(self.sim.sim_time)
        sink = -telemetry.height_rate                 # m/s, down positive
        rotor = telemetry.rotor_percent
        if rotor < self.lowest_rotor:
            self.lowest_rotor = rotor
            self.lowest_rotor_at = now
        if rotor < GREEN_ARC_PERCENT[0]:
            self.under_arc_s += SIM_TIME_STEP_S
            if not self.sim.on_ground:
                self.under_arc_air_s += SIM_TIME_STEP_S
        if (self.torque_gone_at is None
                and telemetry.engine.torque_percent <= TORQUE_GONE_PERCENT):
            self.torque_gone_at = now
        while now >= self.next_torque:
            self.torque_after.append((self.next_torque,
                                      telemetry.engine.torque_percent))
            self.next_torque += TORQUE_REPORT_S
            if self.next_torque > self.switch_s + TORQUE_REPORT_FOR_S:
                self.next_torque = float("inf")
        if not self.sim.crashed:
            self.samples.append((now, sink, telemetry.alt_agl, self.pilot.phase))
            if self.pilot.phase == "descent" and sink > self.peak_sink:
                self.peak_sink = sink
        if self.crash_reason is None and self.sim.crash_reason is not None:
            self.crash_reason = self.sim.crash_reason
            if self.sim.crash_reason == CRASH_OUT_OF_ENVELOPE:
                self.refused_at = now
        if not self.sim.on_ground:
            self.last_airborne = sink
        elif self.touchdown_at is None:
            self.touchdown_at = now
            self.touchdown_rate = self.last_airborne
            self.rotor_at_touchdown = rotor

    def finish(self):
        """The analysis only the end of the flight makes possible: the descent held.

        The rate the descent *held* is not known while it is still changing: it is
        the mean sink over the last :data:`SETTLE_MEAN_S` seconds before the cushion
        the mission's own pilot opened on ``AutorotationPilot.cushion_at``.  The
        settle is then read back off the samples against *that* rate - the first
        frame in the descent phase whose sink stayed inside
        :data:`SETTLE_TOLERANCE` of it for :data:`SETTLE_HOLD_S` - so the catch is
        what this flight's own entry cost rather than what the trim's ideal entry
        would have cost.  See the constant for why it is not the trim's own rate.
        """
        cushion = self.pilot.cushion_at
        if cushion is not None:
            self.held_sink = self.mean_sink(cushion - SETTLE_MEAN_S, cushion)
        if self.held_sink is None:
            return self
        want = self.held_sink
        run_from = None
        for when, sink, alt, phase in self.samples:
            if phase != "descent" or self.settled_at is not None:
                continue
            if abs(sink - want) > SETTLE_TOLERANCE * want:
                run_from = None
                continue
            if run_from is None:
                run_from = when
            if when - run_from >= SETTLE_HOLD_S:
                self.settled_at = when
                self.settled_alt = alt
                self.settled_sink = self.mean_sink(run_from, when)
        return self


# ---------------------------------------------------------------------------
# The two switches.  Each is handed the sim and the pilot and returns the words
# for the transcript, so that a flight is *this* switch and the other one is one
# function away.
# ---------------------------------------------------------------------------


def kill_by_grip(sim, pilot):
    """The mission's own hand: nothing to press, and that is the point of it.

    ``AutorotationPilot.phase_of`` rolls the grip off the moment the climb
    settles - ``want_throttle`` to zero - and :meth:`AutorotationPilot.keys` holds
    that key at the axis' own quarter rate until the lever is at the stop, so the
    shaft's torque comes off over four seconds as the fuel control follows the
    hand.  This is the flight ``_rt_autoro_probe.py`` measures, flown again here
    only to be the number the other switch is read against.
    """
    return ("the mission's own hand is already rolling the grip off, and the"
            " shaft follows it down")


def kill_by_failure(sim, pilot):
    """``main.py``'s X: one line, and the fuel is gone through the engine's lag.

    The key's own two lines in ``main.py`` are ``failed = not
    sim.airframe.engine.governor.failed`` and ``sim.set_governor(failed=failed)``;
    the second is repeated here so that what is measured is the sandbox's own key
    and not a cousin of it.  The grip is left alone, and the mission's own pilot
    rolls it off a decision later anyway, which is what a hand does - and which
    makes no difference at all to a failed engine, because
    :attr:`airframe.Airframe.windmill` reads the *shaft* and not the grip.
    """
    sim.set_governor(failed=not sim.airframe.engine.governor.failed)
    return "the fuel away through the governor's own lag"


#: The two switches by name, for :func:`flight`'s own call.
KILLS = {KILL_BY_GRIP: kill_by_grip, KILL_BY_FAILURE: kill_by_failure}


def flight(way, reaction, trims, stream):
    """One killing of the shaft, flown by the mission's own pilot.

    Driven one frame at a time exactly as ``main.py``'s frame loop drives it, and
    the switch is thrown on the frame the mission reaches its own cut - the
    instant the two entries are comparable from, since from there on the mission's
    hands are the same in both flights.  Returns the flown :class:`Flight`.
    """
    sim = Simulation(airframe=airframe_preset(PRESET))
    sim.on_the_pad()
    sim.fit_engine()
    pilot = AutorotationPilot(sim, trims, reaction=reaction)
    say(stream, "the %s: %s" % (way, WORDS[way]))
    flown = None
    reported = tenth = 0.0
    while sim.sim_time < MAX_FLIGHT_S and not pilot.mission_done(sim):
        keys = pilot.keys(sim, SIM_TIME_STEP_S)
        if flown is None and pilot.phase == "cut":
            note = KILLS[way](sim, pilot)
            flown = Flight(way, sim, pilot, sim.sim_time, trims.descent()[5])
            say(stream, "  %5.1f s at the cut, %4.1f ft of skid and %.0f kt:"
                " the switch is thrown - %s"
                % (flown.switch_s, flown.switch_skid_ft,
                   sim.telemetry().airspeed_kt, note))
        sim.fly(SIM_TIME_STEP_S, **key_states(keys))
        if flown is None:
            continue
        flown.measure()
        now = float(sim.sim_time)
        if now >= reported + REPORT_S:
            reported = now
            say(stream, "  %5.1f s %s" % (now, pilot.note(sim)))
        if skid_ft(sim) < ARRIVAL_SKID_FT and now >= tenth:
            tenth = now + ARRIVAL_REPORT_S
            telemetry = sim.telemetry()
            say(stream, "  %5.2f s %4.1f ft of skid, %+7.0f fpm, collective"
                " %5.2f in (%4.1f %%), rotor %.1f %%, torque %4.1f %%"
                % (now, skid_ft(sim), telemetry.height_rate_fpm,
                   telemetry.collective_in,
                   telemetry.collective_fraction * 100.0,
                   telemetry.rotor_percent,
                   telemetry.engine.torque_percent))
    if flown is None:
        raise RuntimeError("the mission never reached its own cut inside %.0f s:"
                           " the settled climb is what this switch waits for"
                           % (MAX_FLIGHT_S,))
    flown.finish()
    say_the_flight(stream, flown)
    return flown


def say_the_flight(stream, flown):
    """One flight's own block: the shaft, the rotor, the catch, and the arrival."""
    say(stream, "  the shaft: the switch caught %5.1f %% of torque with the grip"
        " %4.2f open, and the fuel was gone %s"
        % (flown.switch_torque, flown.switch_throttle,
           "at +%.1f s" % (flown.torque_gone_at - flown.switch_s)
           if flown.torque_gone_at is not None else "never in this flight"))
    say(stream, "  the gauge after it, a second apart: %s"
        % (", ".join("%+.1f s %5.1f %%" % (when - flown.switch_s, percent)
                     for when, percent in flown.torque_after),))
    say(stream, "  the rotor: %5.1f %% at the switch, %s %% when the skids"
        " arrived, down to %.1f %% at +%.1f s, and below the green arc's"
        " %.1f %% for %.0f s of the flight - %.0f s of that in the air, the"
        " rest winding down on the skids"
        % (flown.switch_rotor, words(flown.rotor_at_touchdown, "%.1f"),
           flown.lowest_rotor, flown.lowest_rotor_at - flown.switch_s,
           GREEN_ARC_PERCENT[0], flown.under_arc_s, flown.under_arc_air_s))
    if flown.settled_at is None:
        say(stream, "  the catch: none - the sink never held inside %.0f %%"
            " of the %s m/s the descent went on to fly"
            % (SETTLE_TOLERANCE * 100.0, words(flown.held_sink, "%.2f")))
    else:
        say(stream, "  the catch: %4.1f ft of height and %.1f s between the"
            " switch and the sink holding the %.2f m/s the descent went on to"
            " fly, at %.2f m/s; it went %4.2f m/s deep on the way"
            % (flown.catch_ft, flown.settle_s, flown.held_sink,
               flown.settled_sink, flown.peak_sink))
    say(stream, "  the descent as flown: %s, against the trim's %5.2f m/s"
        " (%+6.0f fpm) - the model's own answer for the ideal entry"
        % ("%5.2f m/s (%+6.0f fpm)" % (flown.held_sink,
                                       -flown.held_sink / FOOT * 60.0)
           if flown.held_sink is not None else "never held",
           flown.sink_ref, -flown.sink_ref / FOOT * 60.0))
    if flown.touchdown_at is None:
        say(stream, "  no arrival: the flight ended in the air at %.1f s, %s"
            % (flown.sim.sim_time,
               flown.sim.telemetry().crash_message or "and still falling"))
    else:
        say(stream, "  arrival at %.1f s, %+.1f s after the switch: %s"
            % (flown.touchdown_at, flown.touchdown_at - flown.switch_s,
               flown.sim.telemetry()))
        say(stream, "  the skids met the ground at %5.2f m/s (%+6.0f fpm),"
            " against %.1f m/s hard; the rotor was down to %.0f %%; %s"
            % (flown.touchdown_rate, -flown.touchdown_rate / FOOT * 60.0,
               UH1_HARD_LANDING_MPS, flown.lowest_rotor,
               flown.sim.telemetry().crash_message
               or "no crash: inside the floor's own"))
    if flown.refused_at is not None:
        say(stream, "  the model refused a frame at +%.1f s: %s is what the loop"
            " does with a state it will not fly rather than pass on"
            % (flown.refused_at - flown.switch_s,
               flown.sim.telemetry().crash_message))


def diff(first, second):
    """``second`` less ``first``, or None where either was never measured."""
    if first is None or second is None:
        return None
    return second - first


def say_the_pair(stream, grip, failure, trims):
    """The two flights read against each other: what the switch changed."""
    say(stream, "the two switches, side by side, against the trim's own"
        " %.2f m/s and the green arc's %.1f %%:"
        % (trims.descent()[5], GREEN_ARC_PERCENT[0]))
    for flown in (grip, failure):
        say(stream, "  %-8s rotor %s %% at the skids (low %.1f %%, %.0f s of it"
            " under the arc in the air), catch %-7s ft in %-7s s, descent"
            " %-7s m/s, arrival %-7s m/s, %s"
            % (flown.way, words(flown.rotor_at_touchdown, "%.1f"),
               flown.lowest_rotor, flown.under_arc_air_s,
               words(flown.catch_ft, "%5.1f"), words(flown.settle_s, "%5.1f"),
               words(flown.held_sink, "%5.2f"),
               words(flown.touchdown_rate, "%5.2f"),
               "crashed" if flown.sim.crashed else "landed"))
    say(stream, "  the failure against the grip cut: %s ft of catch, %s s of"
        " settling, %s m/s of descent held, %s m/s of arrival, %s %% of rotor"
        " at the skids, and %s s more under the arc"
        % (words(diff(grip.catch_ft, failure.catch_ft), "%+5.1f"),
           words(diff(grip.settle_s, failure.settle_s), "%+5.1f"),
           words(diff(grip.held_sink, failure.held_sink), "%+5.2f"),
           words(diff(grip.touchdown_rate, failure.touchdown_rate), "%+5.2f"),
           words(diff(grip.rotor_at_touchdown, failure.rotor_at_touchdown),
                 "%+5.1f"),
           words(diff(grip.under_arc_air_s, failure.under_arc_air_s), "%+5.0f")))
    say(stream, "  the lever the failure leaves in the hand is the *level* trim"
        " of %.0f kt, %5.2f in, against the autorotation's %5.2f: %.2f in the"
        " loops have to take off with nothing driving the rotor"
        % (AUTOROTATION_SPEED_KT, trims.at(AUTOROTATION_SPEED_MPS)[0],
           trims.descent()[0],
           trims.at(AUTOROTATION_SPEED_MPS)[0] - trims.descent()[0]))


def check(stream, pairs, trims):
    """The claims the transcript makes, asserted: a regression is a traceback.

    Everything here was measured first and written down second, and each line is a
    number the printout above already carries.  What is *asserted* is what holds at
    both decision rates: the entries are held to the same numbers where the model
    owes them the same answer, and the two switches are held to the ones that
    turned out to be the switch's own.  Which of the two leaves the rotor lower is
    printed rather than asserted - at the mission's own rate it is the failure, and
    with a slower hand the grip's own cut overtakes it, because a hand that is
    slower to get the lever down holds the level trim for longer with the shaft
    already dead.  That is the finding the second rate is flown for.
    """
    auto = trims.descent()
    for reaction, grip, failure in pairs:
        for flown in (grip, failure):
            telemetry = flown.sim.telemetry()
            assert not flown.sim.crashed, (reaction, flown.way,
                                           telemetry.crash_message)
            assert flown.sim.dropped_steps == 0, (reaction, flown.way,
                                                  flown.sim.dropped_steps)
            # No frame was refused: whatever the entry produced, the state was one
            # the loop would fly.  CRASH_OUT_OF_ENVELOPE is the other answer, and
            # it is a real one here - see the windmill note in TODO.md - so it is
            # the first thing this file has anything to say about.
            assert flown.refused_at is None, (reaction, flown.way,
                                              flown.refused_at,
                                              telemetry.crash_message)
            # The rotor leaves the manual's own green arc - in *both* entries and
            # at both rates - and nothing in the model brings it back: the rotor
            # speed is a free state with no blade stall and no flap limit behind
            # it, so what is asserted is the finding rather than an arc the model
            # cannot hold.  See airframe.py's windmill note: this is the number
            # that note is about.
            assert flown.lowest_rotor < GREEN_ARC_PERCENT[0], \
                (reaction, flown.way, flown.lowest_rotor)
            assert flown.rotor_at_touchdown < GREEN_ARC_PERCENT[0], \
                (reaction, flown.way, flown.rotor_at_touchdown)
            assert flown.under_arc_air_s > 0.0, (reaction, flown.way,
                                                 flown.under_arc_air_s)
            # It did reach the descent it went on to fly, and that descent is the
            # flight manual's own trim within the band this project prints it at:
            # the model overshoots the trim's rate by a little once the rotor is
            # windmilling, which is a number in the transcript and not a hidden
            # fudge.  See DESCENT_BAND_MPS.
            assert flown.settled_at is not None, (reaction, flown.way,
                                                  flown.held_sink)
            assert flown.settled_at > flown.switch_s, (reaction, flown.way,
                                                       flown.settled_at)
            assert abs(flown.held_sink - auto[5]) <= DESCENT_BAND_MPS, \
                (reaction, flown.way, flown.held_sink, auto[5])
            # And it arrived, inside the floor's own hard landing rate - which is
            # this project's own game constant and not a UH-1 limit.
            assert flown.touchdown_at is not None, (reaction, flown.way,
                                                    telemetry.crash_message)
            assert flown.touchdown_at > flown.settled_at, (reaction, flown.way,
                                                           flown.touchdown_at)
            assert flown.touchdown_rate < UH1_HARD_LANDING_MPS, \
                (reaction, flown.way, flown.touchdown_rate)
            # The windmill root was reached in both: with the descent driving the
            # rotor the shaft's own torque gauge is at nothing, and on the
            # failure's aircraft the governor says so.
            assert telemetry.engine.torque_percent <= TORQUE_GONE_PERCENT, \
                (reaction, flown.way, telemetry.engine.torque_percent)
            if flown.way == KILL_BY_FAILURE:
                assert telemetry.engine.failed, telemetry.engine.failed
        # And the two claims that are the *switch's* own rather than the hand's: a
        # failure is one lag where the grip's cut is four seconds of a ratchet, so
        # it is the quicker entry to a descent - less height, and fewer seconds,
        # between the switch and the rate the descent goes on to fly.  Its lever
        # starts higher, though, so it is not the cheaper one in rotor: see the
        # sweep below.
        assert failure.catch_ft < grip.catch_ft, (reaction, failure.catch_ft,
                                                  grip.catch_ft)
        assert failure.settle_s < grip.settle_s, (reaction, failure.settle_s,
                                                  grip.settle_s)
    say(stream, "check passed: both entries, at %.2f s and at %.2f s between"
        " decisions, reached the manual's own descent, arrived inside the floor's"
        " own %.1f m/s, and both took the rotor out of the green arc."
        % (pairs[0][0], pairs[-1][0], UH1_HARD_LANDING_MPS))
    sys.stdout.flush()


def say_the_finding(stream, pairs):
    """The one the second decision rate exists to establish, and the numbers.

    A slower hand takes *both* rotors further down the arc, because the lever
    spends longer at the level trim with nothing driving the shaft - and on the
    grip's own cut, which is flown from a settled climb with the lever being rolled
    off by hand, it is worth a fifth of the rotor.  So which of the two entries is
    the worse one is the *hand's* answer and not the switch's, and the sweep is
    what says so.
    """
    first, last = pairs[0], pairs[-1]
    say(stream, "the two decision rates, side by side: what the hand is worth")
    for reaction, grip, failure in pairs:
        say(stream, "  %.2f s between decisions: the grip's cut put the skids down"
            " on %.0f %% (low %.0f %%), the failure on %.0f %% (low %.0f %%), and"
            " the catch was %s ft against %s"
            % (reaction, grip.rotor_at_touchdown, grip.lowest_rotor,
               failure.rotor_at_touchdown, failure.lowest_rotor,
               words(grip.catch_ft, "%.0f"), words(failure.catch_ft, "%.0f")))
    say(stream, "  so the slower hand cost the grip's own cut %.0f %% of rotor at"
        " the skids and the failure %.0f %%: the switch that is a key is the one a"
        " slower hand makes worse, and the one that is a single lag barely cares"
        % (first[1].rotor_at_touchdown - last[1].rotor_at_touchdown,
           first[2].rotor_at_touchdown - last[2].rotor_at_touchdown))


def probe(reactions=REACTIONS, out_path=OUT_PATH):
    """Both ways of killing the shaft, flown by the mission's own pilot.

    Returns a list of ``(reaction, grip, failure)`` - one per decision rate, in the
    order they were flown - so a caller can look at anything the transcript did not
    print.  Both entries are flown at every rate, since what the *hand* is worth is
    half of what this file is about: see :func:`say_the_finding`.
    """
    stream = open(out_path, "w", encoding="utf-8")
    say(stream, "the autorotation's two switches: the mission's own grip"
        " cut, and main.py's %s - the engine failure - thrown in its place"
        % (pygame.key.name(pygame.K_x).upper(),))
    say(stream, "an autorotation from %.0f ft of skid at %.0f kt on the \"%s\""
        " aircraft, fitted live on the pad, flown by the mission's own pilot"
        " - %s - one frame at a time, and at each of the decision rates a hand"
        " is flown at below"
        % (CLIMB_SKID_FT, AUTOROTATION_SPEED_KT, PRESET,
           AutorotationPilot.DESCRIPTION))
    say(stream, "the grip's own key is %s and the failure is %s: two different"
        " events in the model and the same physics at the shaft, which is what"
        " airframe.Airframe.windmill reads"
        % (KEY_NAMES[CUT_GRIP_KEY], pygame.key.name(pygame.K_x).upper()))
    say(stream, "the manual's own green arc is %.1f to %.1f %% of the nominal"
        " rotor speed: neither entry stays inside it, which is the finding"
        % GREEN_ARC_PERCENT)

    # The trims: the pilot's own table, built off a scratch aircraft exactly as
    # the demo builds it - the level rows the climb's loops are written around,
    # and the autorotation row both entries have to reach.
    trims = trims_for(airframe_preset(PRESET), AutorotationPilot)
    for speed in trims.speeds:
        row = trims.at(speed)
        say(stream, "level trim at %5.1f m/s: collective %5.2f in, long %+5.2f,"
            " lat %+5.2f, pedal %+5.2f, pitch %+5.2f deg"
            % (speed, row[0], row[1], row[2], row[3], row[4]))
    auto = trims.descent()
    say(stream, "autorotation trim at %.0f kt: collective %5.2f in, long %+5.2f,"
        " lat %+5.2f, pedal %+5.2f, pitch %+5.2f deg, sink %5.2f m/s (%+6.0f fpm)"
        " - the rate both entries have to reach"
        % (AUTOROTATION_SPEED_KT, auto[0], auto[1], auto[2], auto[3], auto[4],
           auto[5], -auto[5] / FOOT * 60.0))

    pairs = []
    for reaction in reactions:
        say(stream, "")
        say(stream, "with %.2f s between the mission's own decisions:" % (reaction,))
        grip = flight(KILL_BY_GRIP, reaction, trims, stream)
        say(stream, "")
        failure = flight(KILL_BY_FAILURE, reaction, trims, stream)
        say(stream, "")
        say_the_pair(stream, grip, failure, trims)
        pairs.append((float(reaction), grip, failure))
    say(stream, "")
    say_the_finding(stream, pairs)
    check(stream, pairs, trims)
    stream.close()
    return pairs


def demo():
    """The command line: which decision rates to fly, if any are said."""
    reactions = tuple(float(argument) for argument in sys.argv[1:]) or REACTIONS
    probe(reactions)
    print("wrote %s" % (OUT_PATH,))
    return 0


if __name__ == "__main__":
    sys.exit(demo())
