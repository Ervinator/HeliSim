# -*- coding: utf-8 -*-
"""Scratch: a person's hands on the *outer* loops, and the script's own claims.

Four phases of this programme have measured the pilot from the stick inwards.
Phase 1 - ``_rt_handling.py`` - put a person's clock, delay and lag on the hover's
two attitude loops; phase 2 - ``_rt_axes.py`` - did the same for the lever and the
pedals and then flew every one of the four at sixty knots; phase 3 put the
instrument on the autorotation's own loops and phase 4 - ``_rt_pilot_hand.py`` -
on the harness's two missions.  Every one of them closed its loops round
``_rt_pilot``'s own *outer* numbers, and said so: the tilt a hover is held with,
the height it is asked for, the speed a cruise holds and the line it is flown up
all stayed at the model's own gains, because the question was how hard a hand may
grip the loop *under* them.

This file is the other half of that sentence.  The instrument is unchanged - the
twenty decisions a second on the flight's own clock, the ``DELAY_S`` of transport
delay read off a deque of remembered states, the ``T_LAG_S`` of lag in the hand
and the model's own limb rate underneath it, and the attitude loops still the
model's own - and the four loops *above* the attitude are what is put under
scales:

* **the station-keeping tilt**, over the pad: degrees of tilt per metre off it
  and per m/s of drift still to be taken out - ``K_HOVER_POS_DEG_M`` and
  ``K_HOVER_VEL_DEG_MPS``, which are scaled *together*, since a loop closed on a
  position through a velocity is one loop and its two gains are its shape;
* **the height**: metres a second of climb rate asked for per metre of height
  error, ``RATE_GAIN_MPS``, which both of the model's own pilots hold their
  altitude with - so it is the one outer loop that is flown in both conditions;
* **the speed**: degrees of nose per m/s of speed error, ``K_SPEED_DEG``, which is
  the cruise pilot's own way of holding the speed the trim was solved for;
* **the track**: degrees of bank per metre off the line north of the pad,
  ``K_BANK_DEG``, which is its own way of holding the line.

What is put into each is the state a pilot has to take back out of it: five metres
off the pad for the station-keeping pair, a metre of height for the height loop,
three metres a second of airspeed for the speed loop, and twenty metres off the
line for the track - each read against a band of its own, a fraction of what was
put in, which is what makes "settled" mean the same thing on four loops that are
not in the same units and are not even in the same condition.

Three measurements, one per question, and the same three phase 1 asked:

* **the nominal case of each loop.**  The model's own pilot for that condition, at
  a scale of one on every outer loop, against a perturbation it did not ask for:
  the worst it reached, how long the settling took, and what the *rest* of the
  flight cost while it did it;
* **the band of each loop.**  The same perturbation with that loop's gain scaled
  up and down from the model's own until a grip fails, which is what a margin is -
  a band and not a number, because the two edges fail differently: too hard a grip
  closes the pilot's own loop on the pilot's own delay, and one too soft cannot
  hold what the loop is being asked for.  (One of the four turns out to have no
  band at all, and that is this file's finding rather than a gap in it: a loop whose
  answer is a *swing* out past the band on the far side of the line and back is not
  a loop a pair of edges can describe.  See :func:`holds` and the track loop's
  rows.)
* **the frozen case**, with nobody flying it at all - half the reason a loop has a
  low edge as well as a high one, since a pilot's margin is what their hands buy
  against what the aircraft does on its own.

And what all of it is held to is the one claim this file can make that no phase
before it could: **the height loop at a scale of one is phase 2's own heave
case**, in both conditions - a metre of height through the same ``RATE_GAIN_MPS``
and the same lever, with the two task loops a cruise pilot flies taken back out of
it, which is exactly the pilot that file flew.  That is :func:`ruler`, and it is
the check that this file's numbers and that file's are the same numbers.

A word on *held* itself, because every number below is a verdict read off it.  The
criterion is phase 1's own three things - the error came back inside the loop's
band and stayed, what is left of it is still dying out at the end of the case, and
the machine is in the sky to have done it - and what the four phases before this
one read the middle one off is the worst error in the last third of a case against
the worst in its middle third.  On these four loops that ratio is wrong in both
directions: a loop that settles and then rides the aircraft's own slow mode within
a tenth of its band has a tail over one, and a loop whose swing takes it clean out
past the band and back has a tail under one while the error is still outside.  So
this file reads the *words*: inside the band from the settling ``Trace.settled``
reports to the last frame of the case.  It agrees with the ratio on every case of
the four phases before it, whose failures leave the band rather than loiter in it,
and the ratio itself is still printed beside every row and pinned for the nominal
case.  See :func:`holds`.

The second half of the file asks the last question ``TODO.md`` has left on this
programme.  ``_rt_check.py`` flies the harness's two missions by *script* and
asserts twenty things about them - the band the hover was met in, the 360, the
nose back on the heading it began from, the rate the skids arrived at, the
kilometre.  Phase 4 flew both missions with a person's hands and reported whether
they were flown; what it did not do is read a person's flight against the script's
own claims, one claim at a time.  That is :data:`CLAIMS` below: every assertion
``_rt_check.check`` makes of itself, in that file's own words, and the answer a
hand at the instrument's own clock gives - held or missed, with the number.  The
claims are not loosened and not edited: a claim a hand misses is this file's
*finding*, and is reported as one.

Usage::

    python _rt_outer.py

Run it with ``python _rt_outer.py``, a few minutes.
"""


import math
import sys
from collections import namedtuple

from _rt_autoro_entry import words
from _rt_axes import (CONDITION_ALT_M, CONDITION_MPS, CONDITION_WORDS, CRUISE,
                      CRUISE_SETTLED_S as PHASE_TWO_CRUISE, HOVER,
                      NOMINAL_SETTLED_S as PHASE_TWO_NOMINAL, SCALES_ONE)
from _rt_axes import Hands as PhaseTwoHands
from _rt_axes import Trace as PhaseTwoTrace
from _rt_axes import pio_report
from _rt_check import check as scripted_check
from _rt_handling import (DELAY_FRAMES, DELAY_S, DRIFT_LIMIT_M, EVERY,
                          FROZEN_S, HEIGHT_LIMIT_M, HOLD_S, PILOT_HZ,
                          PILOT_PERIOD_S, PIO_PERIOD_S, SETTLE_S,
                          SETTLE_TOLERANCE_S, T_LAG_S, Transcript)
from _rt_handling import Trace as PhaseOneTrace
from _rt_pilot import (ALT_REF_M, ALT_TOLERANCE_M, BANK_LIMIT_DEG,
                       CLIMB_LIMIT_MPS, DIST_REF_M, HOVER_ALT_M,
                       HOVER_REF_EAST_M, HOVER_REF_NORTH_M, HOVER_SKID_FT,
                       HOVER_TOLERANCE_M, HOVER_TILT_LIMIT_DEG, K_BANK_DEG,
                       K_HOVER_POS_DEG_M, K_HOVER_VEL_DEG_MPS, K_PITCH_IN,
                       K_PITCH_RATE_IN, K_ROLL_IN, K_ROLL_RATE_IN, K_SPEED_DEG,
                       K_TURN_IN, K_TURN_RATE_IN, K_YAW_IN, K_YAW_RATE_IN,
                       KEY_STEP, PITCH_LIMIT_DEG, RATE_GAIN_MPS, REACTION_S,
                       TRACK_EAST_REF_M, TURN_TOLERANCE_DEG, YAW_REF_DEG,
                       HumanPilot, clip, inches_to_axes, skid_ft, trims_for,
                       wrap)
from _rt_pilot_hand import (CLOCKS, HAND_MISSIONS, HOVER_DRIFT_LIMIT_M,
                            HOVER_TOUCHDOWN_LIMIT_MPS, LIMITS, flight, say)
from airframe import FOOT, Vector3
from main import AIRFRAME_PRESET
from simulation import SIM_TIME_STEP_S, Simulation, airframe_preset

#: The transcript, written by the run itself: see the module docstring.
LOG_FILE = "_rt_outer.txt"


# ---------------------------------------------------------------------------
# The four outer loops, where each of them lives, and what is put into it.
# ---------------------------------------------------------------------------

#: The four loops above the attitude: where the machine is over the pad, how high
#: it is, how fast it is going, and which side of the line north of the pad it is
#: on.  Between them they are the whole of the outer pilot - everything the four
#: phases before this one left at the model's own numbers.
STATION = "station"
HEIGHT = "height"
SPEED = "speed"
TRACK = "track"
AXES = (STATION, HEIGHT, SPEED, TRACK)

#: Which loop is flown in which condition, which is ``_rt_pilot``'s own
#: arrangement and not this file's: over the pad a hover is held over a *point* -
#: a tilt towards it, damped by the drift still to be taken out - and its height
#: on the lever; at sixty knots there is no point to hold station over, so the
#: aircraft's own pilot holds the *speed* the trim was solved for with the pitch
#: attitude and the line north of the pad with a bank, and its height with the
#: same lever loop it had over the pad.  Five cases, and every outer loop the
#: model's own two pilots fly is one of them.
CASES = ((STATION, HOVER), (HEIGHT, HOVER), (HEIGHT, CRUISE),
         (SPEED, CRUISE), (TRACK, CRUISE))

#: What each loop *is*, in the transcript's own words: the gain a scale
#: multiplies, and the units that gain is read in.
LOOP_WORDS = {
    STATION: "the station-keeping tilt under the hover's attitude loop",
    HEIGHT: "the height loop: a climb rate asked for, and the lever under it",
    SPEED: "the speed loop: the nose is what holds the speed, out here",
    TRACK: "the track loop: the line north of the pad, held with a bank",
}

#: What is put into each loop, in that loop's own units.  A metre of height is
#: phase 2's own perturbation, kept to the last decimal so that the two files'
#: height cases are the same case; the other three are this file's, and each is
#: the size of the thing a pilot would actually have to take back out - five
#: metres off the pad is half the drift a mission allows itself, three metres a
#: second of airspeed is a gust on the nose at sixty knots, and twenty metres off
#: the line is a wind the pilot did not allow for.  A speed perturbation is put in
#: by *speeding the aircraft up* rather than by moving a stick: it flies on with
#: the trim's own controls and a different velocity, which is what the air does to
#: it and not what the pilot did.
PERTURB_OFFSET_M = 5.0
PERTURB_HEIGHT_M = 1.0
PERTURB_SPEED_MPS = 3.0
PERTURB_TRACK_M = 20.0

PERTURB = {STATION: PERTURB_OFFSET_M, HEIGHT: PERTURB_HEIGHT_M,
           SPEED: PERTURB_SPEED_MPS, TRACK: PERTURB_TRACK_M}

#: What each of those is, in words: the sentence a case's header wants.
PERTURB_WORDS = {STATION: "%0.1f m off the pad", HEIGHT: "%0.1f m of height",
                 SPEED: "%0.1f m/s of airspeed", TRACK: "%0.1f m off the line"}

#: The band each loop counts as *back inside*, in its own units, and so what
#: settling means on it: a fraction of what was put in, which is what makes four
#: numbers in three different units comparable at all.  The height's is phase 2's
#: own fifteen centimetres of the metre; the station-keeping pair is given a
#: fifth, because a metre off the pad is a hover that is being held; the speed a
#: quarter, because three quarters of a metre a second at sixty knots is a knot
#: and a half on the clock; and the track a quarter too, which is five metres - a
#: quarter of the width of the line a mission is flown up.
SETTLE_OFFSET_M = 1.0
SETTLE_HEIGHT_M = 0.15
SETTLE_SPEED_MPS = 0.75
SETTLE_TRACK_M = 5.0

SETTLE = {STATION: SETTLE_OFFSET_M, HEIGHT: SETTLE_HEIGHT_M,
          SPEED: SETTLE_SPEED_MPS, TRACK: SETTLE_TRACK_M}

#: How long each loop's case is flown, s.  The three that come back in the ten to
#: thirty seconds a case of phase 2 takes are given that file's own forty.
#:
#: The track loop is given five times that, and the reason is a measurement rather
#: than a margin for error: it is the only one of the four with no rate term in it
#: at all - a bank proportional to the distance off the line and nothing else - so
#: what closes it is the aircraft's own lateral dynamics, and what that is worth at
#: the model's own gain is a period of *tens of seconds*: about forty-five, falling
#: as the square root of the grip.  A case of forty seconds would hold less than one
#: of those periods, and the two numbers this file's whole criterion is read off -
#: whether the error came back inside the band and *stayed*, and whether its tail is
#: worse than its middle - are both questions about how the case ends, so a case
#: shorter than the mode it is measuring would answer them out of the phase of that
#: mode rather than out of the loop.  Five of its own periods is a number this file
#: can defend; one was not.
TRACK_HOLD_S = 200.0

HOLD_OF = {STATION: HOLD_S, HEIGHT: HOLD_S, SPEED: HOLD_S, TRACK: TRACK_HOLD_S}


# ---------------------------------------------------------------------------
# What each loop is multiplied by, and the pilot that flies them.
# ---------------------------------------------------------------------------

#: What each of the four loops is multiplied by, and so what a case is *flown* at:
#: one loop at a time, with the other three left at the model's own gains, which is
#: the only way a margin can be read off one loop and quoted against it.  A
#: namedtuple of this file's own rather than phase 2's ``Scales``: that one names
#: the three loops *under* these - and is imported by phases 3 and 4 - so scaling
#: it here would move their pilots as well as this one.
OuterScales = namedtuple("OuterScales", "station height speed track")

#: Every outer loop at the model's own gain: the pilot both of ``_rt_check``'s
#: scripts are flown by, and the nominal case of every loop below.
OUTER_ONE = OuterScales(1.0, 1.0, 1.0, 1.0)


def outer_for(axis, scale):
    """The four scales a case at *scale* of *axis* is flown at.

    One loop put under the scale and the other three left at one, which is phase
    1's own rule for the tilt pair and phase 2's for the lever and the pedals: a
    scale of two is the model's own pilot gripping *that* loop twice as hard, and
    not a pilot gripping everything at once.  The station-keeping pair is the one
    exception, and it is not one really: its two gains are multiplied together,
    since a position loop closed through a velocity is a single loop and the ratio
    of its two gains is the shape of it - see :meth:`Hands.selftest`.
    """
    if axis == STATION:
        return OuterScales(scale, 1.0, 1.0, 1.0)
    if axis == HEIGHT:
        return OuterScales(1.0, scale, 1.0, 1.0)
    if axis == SPEED:
        return OuterScales(1.0, 1.0, scale, 1.0)
    if axis == TRACK:
        return OuterScales(1.0, 1.0, 1.0, scale)
    raise ValueError("no such loop: %r" % (axis,))


def gain_words(axis, outer):
    """What the loop a scale is on is worth at *outer*, in the units it is read in.

    The number a margin is quoted in, and the reason a scale is not a number that
    means the same thing on any two of the four: these are four gains in four
    different units - degrees per metre, m/s per metre, degrees per m/s, degrees
    per metre off a line - and only the loop's own units say what a grip of it is.
    """
    if axis == STATION:
        return ("%0.2f deg per m off the pad and %0.2f deg per m/s of drift"
                % (K_HOVER_POS_DEG_M * outer.station,
                   K_HOVER_VEL_DEG_MPS * outer.station))
    if axis == HEIGHT:
        return "%0.3f m/s of climb rate per m of height" % (RATE_GAIN_MPS
                                                            * outer.height,)
    if axis == SPEED:
        return "%0.3f deg of nose per m/s of speed" % (K_SPEED_DEG
                                                       * outer.speed,)
    return "%0.3f deg of bank per m off the line" % (K_BANK_DEG * outer.track,)


def loop_words(axis):
    """What the loop's own gain is called, for a header's second half."""
    if axis == STATION:
        return "K_HOVER_POS_DEG_M and K_HOVER_VEL_DEG_MPS, moved together"
    if axis == HEIGHT:
        return "RATE_GAIN_MPS"
    if axis == SPEED:
        return "K_SPEED_DEG"
    return "K_BANK_DEG"


#: The scale the nominal case is flown at - the model's own pilot, exactly - and
#: the two legs a band is read off, in the order they are flown: outward from the
#: model's own gains *up* until a grip fails, and then *down* until one does.  Both
#: edges are real and they fail differently, which is why there are two, and the
#: nominal is *not* in either ladder: it is flown once, as its own case, and both
#: ladders start a step outside it - so that a loop whose own gain does not hold is
#: a case that can be *reported* rather than a leg that stops before it starts.
#: The ladders are wider than phase 2's for the same reason: whether a grip a hand
#: cannot overcook is a grip a hand can hold at all is what the bands are for.
SCALE_NOMINAL = 1.0
SCALES_HARDER = (1.5, 2.0, 3.0, 4.0, 6.0, 8.0)
SCALES_SOFTER = (0.75, 0.5, 0.35, 0.25, 0.1)


class Hands(PhaseTwoHands):
    """A person's hands on four outer loops, with each of them under a scale.

    Phase 2's instrument to the last line - the twenty hertz clock on the flight's
    own time, the ``DELAY_S`` of delay read off a deque of remembered states, the
    lag in the hand and the model's own limb rate underneath it, :meth:`fill`,
    :meth:`view`, :meth:`push`, :meth:`move` and the clock's own ``selftest`` as
    phase 1 wrote them, and the three loops *under* these - the lever's, the
    attitude pair's and the pedals' - at the model's own numbers exactly, which is
    what ``scales=SCALES_ONE`` says - with the four loops *above* them under
    :class:`OuterScales` of their own.

    What it decides is the model's own pilot for the condition, loop for loop and
    gain for gain: over the pad, ``_rt_pilot.HoverTurnPilot``'s hover - the tilt
    that holds station, the height on the lever, the attitude pair and the
    pedals - and at sixty knots, ``_rt_pilot.HumanPilot``'s four: the same lever
    loop, the speed the trim was solved for on the nose, the line north of the pad
    on a bank, and the pedals, which at that speed are a slower pair than the
    hover's and hold a reference heading rather than the aircraft's own.  *task*
    says which of the two an aloft case is flown by: ``None`` is the model's own
    cruise pilot, and ``False`` leaves the speed and the line to the trim, which is
    the pilot phase 2 flew at sixty knots and the one :func:`ruler` is flown with.

    The two baselines are read off the trim the aircraft was *handed* rather than
    written down here or taken off ``_rt_pilot``'s own trim table - the model's own
    speed and its own attitude at the trim it was solved at, which is phase 1's own
    rule for a hover and is done here for the same reason: at sixty knots the
    mission's table has run out (its own speeds stop at 24 m/s and a cruise is
    30.87) and what it holds is an extrapolation, while what the aircraft is
    actually trimmed to is not.
    """

    def __init__(self, sim, trim, outer=OUTER_ONE, condition=HOVER, task=None,
                 **kwargs):
        PhaseTwoHands.__init__(self, sim, trim, scales=SCALES_ONE,
                               condition=condition, **kwargs)
        #: What each of the four outer loops is multiplied by, and the five gains
        #: that works out to - the station-keeping pair's two, which move together,
        #: and the height's, the speed's and the track's.
        self.outer = OuterScales(float(outer.station), float(outer.height),
                                 float(outer.speed), float(outer.track))
        self.k_station_pos = K_HOVER_POS_DEG_M * self.outer.station
        self.k_station_vel = K_HOVER_VEL_DEG_MPS * self.outer.station
        self.k_height = RATE_GAIN_MPS * self.outer.height
        self.k_speed = K_SPEED_DEG * self.outer.speed
        self.k_bank = K_BANK_DEG * self.outer.track
        #: Whether the two loops only a *cruise* has are being flown at all: the
        #: speed on the nose and the line on a bank.  A hover has neither - there
        #: is no speed to hold and no line to be off - so a task asked for over the
        #: pad is a pilot no mission has and is refused rather than flown.
        if condition == HOVER:
            assert not task, task
            self.task = False
        else:
            self.task = (condition == CRUISE) if task is None else bool(task)

    def selftest(self):
        """The instrument's own check, and the five outer gains this pilot flies.

        Phase 1's and phase 2's own assertions first, unchanged, because the clock,
        the delay, the lag and the three loops under these are still exactly what
        they claim - including the one that says the *inner* scales are all one, so
        that what this file moves is the four outer loops and nothing else.  Then
        the five gains this class builds, each against the model's own constant and
        the scale it is made of, and the one claim that cannot be made gain by gain:
        that the station-keeping pair moved *together*, so that the loop a scale of
        two flies is that loop with a stiffer grip on it rather than a different
        loop.  A gain moved in ``_rt_pilot`` moves this file's pilot with it.
        """
        PhaseTwoHands.selftest(self)
        assert all(isinstance(one, float) for one in self.outer), self.outer
        for what, gain, model, scale in (
                ("the station-keeping position gain", self.k_station_pos,
                 K_HOVER_POS_DEG_M, self.outer.station),
                ("the station-keeping velocity gain", self.k_station_vel,
                 K_HOVER_VEL_DEG_MPS, self.outer.station),
                ("the height gain", self.k_height, RATE_GAIN_MPS,
                 self.outer.height),
                ("the speed gain", self.k_speed, K_SPEED_DEG, self.outer.speed),
                ("the track gain", self.k_bank, K_BANK_DEG, self.outer.track)):
            assert abs(gain - model * scale) < 1e-12, (what, gain, model, scale)
        assert abs(self.k_station_pos * K_HOVER_VEL_DEG_MPS
                   - self.k_station_vel * K_HOVER_POS_DEG_M) < 1e-15, \
            (self.k_station_pos, self.k_station_vel, K_HOVER_POS_DEG_M,
             K_HOVER_VEL_DEG_MPS)
        assert self.condition in (HOVER, CRUISE), self.condition
        assert self.station is (self.condition == HOVER), self.station
        assert not self.task or self.condition == CRUISE, \
            (self.task, self.condition)
        return self

    def decide(self, view):
        """The four sticks wanted, in inches, from what the instruments said.

        The model's own pilot's four loops, read off a *view* - the aircraft as it
        was ``DELAY_S`` ago - rather than off the aircraft, and with one loop
        under a scale of its own.  Loop by loop, and what is worth saying about
        each is only what the scale does to it:

        * the **height** on the lever, at ``RATE_GAIN_MPS`` under this pilot's
          scale of it: the outer loop's gain is what a scale moves, and the lever
          underneath it stays the model's own, which is phase 1's own rule for the
          loop above a loop and phase 2's for the lever itself;
        * the **speed** on the nose, at ``K_SPEED_DEG`` under its scale: the
          attitude asked for is the trim's own, which is the attitude the trim was
          solved *for*, so the speed the loop is asked to hold is the speed the
          trim holds and the perturbation is nothing but the difference between
          the two.  Only a cruise flies it;
        * the **line** on a bank, at ``K_BANK_DEG`` under its scale: a bank per
          metre off the line north of the pad and *no* rate term at all, which is
          the model's own track loop to the letter and the reason its case is the
          longest in this file;
        * and the **station-keeping tilt** over the pad, at its pair of gains under
          one scale: the aircraft's position and its velocity, in the aircraft's
          own axes, as ``HoverTurnPilot`` holds a hover with them.  Only a hover
          flies it.

        What holds the two attitudes is then the model's own pair for the
        condition, and so are the pedals: the hover's, or the cruise pilot's
        slower one.  Nothing here is this file's own invention except the scales.
        """

        telemetry = view.telemetry
        collective_t, long_t, lat_t, pedal_t = self.trim[:4]

        # Vertical: the height the trim sits at, through the rate the outer height
        # loop asks for - that loop's gain under this pilot's scale of it - and the
        # lever underneath it at the model's own.
        rate_want = clip(self.k_height * (self.alt_ref - telemetry.alt_agl),
                         -CLIMB_LIMIT_MPS, CLIMB_LIMIT_MPS)
        collective = (collective_t
                      + self.k_lever * (rate_want - telemetry.height_rate))

        # And the tilt the two attitudes are measured from: over the pad the
        # position and the velocity in the aircraft's own axes - the nose is what
        # turns, the pad does not - and aloft the two task loops, or the trim's own
        # attitude where a pilot has neither.
        if self.station:
            heading = math.radians(telemetry.yaw_deg)
            north_off = self.pad[0] - view.north
            east_off = self.pad[1] - view.east
            forward_off = (north_off * math.cos(heading)
                           + east_off * math.sin(heading))
            right_off = (-north_off * math.sin(heading)
                         + east_off * math.cos(heading))
            pitch_want = clip(self.pitch_ref - self.k_station_pos * forward_off
                              + self.k_station_vel * view.vx,
                              self.pitch_ref - HOVER_TILT_LIMIT_DEG,
                              self.pitch_ref + HOVER_TILT_LIMIT_DEG)
            roll_want = clip(self.roll_ref + self.k_station_pos * right_off
                             - self.k_station_vel * view.vy,
                             self.roll_ref - HOVER_TILT_LIMIT_DEG,
                             self.roll_ref + HOVER_TILT_LIMIT_DEG)
        elif self.task:
            pitch_want = clip(
                self.pitch_ref
                - self.k_speed * (self.speed_ref - telemetry.airspeed),
                self.pitch_ref - PITCH_LIMIT_DEG,
                self.pitch_ref + PITCH_LIMIT_DEG)
            east = view.east - TRACK_EAST_REF_M
            roll_want = clip(-self.k_bank * east, -BANK_LIMIT_DEG, BANK_LIMIT_DEG)
        else:
            pitch_want = self.pitch_ref
            roll_want = self.roll_ref

        # Which is held with the model's own attitude loop for the condition: the
        # hover pilot's, or the cruise pilot's, which is the same shape at a slower
        # rate gain - a hover is where this model's two attitude answers are at
        # their worst, which is why that pair has more rate in it than the cruise's.
        if self.task:
            long_stick = (long_t
                          + K_PITCH_IN * (telemetry.pitch_deg - pitch_want)
                          + K_PITCH_RATE_IN * telemetry.pitch_rate_deg)
            lat_stick = (lat_t
                         + K_ROLL_IN * (roll_want - telemetry.roll_deg)
                         - K_ROLL_RATE_IN * telemetry.roll_rate_deg)
        else:
            long_stick = (long_t
                          + self.k_pitch * (telemetry.pitch_deg - pitch_want)
                          + self.k_pitch * self.t_lead * telemetry.pitch_rate_deg)
            lat_stick = (lat_t
                         + self.k_roll * (roll_want - telemetry.roll_deg)
                         - self.k_roll * self.t_lead * telemetry.roll_rate_deg)

        # And the pedals: over the pad the heading the trim is on, held, at the
        # hover pilot's own pair; aloft the cruise pilot's - a slower pair, holding
        # the reference heading the mission is flown up.
        if self.task:
            pedal = (pedal_t
                     - K_YAW_IN * wrap(telemetry.yaw_deg - YAW_REF_DEG)
                     - K_YAW_RATE_IN * telemetry.yaw_rate_deg)
        else:
            pedal = (pedal_t
                     + self.k_pedal * wrap(self.heading_ref - telemetry.yaw_deg)
                     + self.k_pedal * self.t_pedal_lead
                     * (0.0 - telemetry.yaw_rate_deg))
        return [collective, long_stick, lat_stick, pedal]


# ---------------------------------------------------------------------------
# What each loop measured, in that loop's own units.
# ---------------------------------------------------------------------------

#: One loop as a *reading*: which of the four inches of stick the hand is holding
#: while it is measured, the unit the error is in, the error itself against the
#: quantity the trim left that loop holding, and how fast that error is changing.
#: The error is a function of the pilot *and* the aircraft rather than a field,
#: because the four quantities it is read from - a place over the pad, a height, a
#: speed and a place across a line - are not attributes of one object; and the
#: rate is a function of all three for the same reason.
Reading = namedtuple("Reading", "stick unit error rate")


def station_error(hands, telemetry, state):
    """Metres *north* of the pad.

    The signed distance along the direction the perturbation was put in, and not
    the distance from the pad the rest of this file prints beside it: a trace is
    read for when its error came back inside a band and stayed, which is a
    question about a quantity that crosses zero, not about one that only ever
    approaches it.
    """
    return state.position.x - hands.pad[0]


def station_rate(hands, telemetry, state):
    """m/s north: the error's own rate, which is the aircraft's own velocity."""
    return state.velocity.x


def height_error(hands, telemetry, state):
    """Metres above the height the trim was on: positive is high, as phase 2."""
    return telemetry.alt_agl - hands.alt_ref


def speed_error(hands, telemetry, state):
    """m/s through the airframe, against the speed the trim was solved for.

    The reading ``_rt_axes`` takes as its speed reference and the reading
    ``HumanPilot`` holds its speed with, which are the same instrument; what this
    loop adds is a *settling* on it rather than an excursion limit.
    """
    return telemetry.airspeed - hands.speed_ref


def track_error(hands, telemetry, state):
    """Metres east of the line north of the pad: positive is off to the right."""
    return state.position.y - TRACK_EAST_REF_M


def track_rate(hands, telemetry, state):
    """m/s east: the error's own rate, which is the aircraft's own velocity."""
    return state.velocity.y


#: What each loop is read off, one per axis: the loop's own quantity against what
#: the trim left it holding, and the rate that quantity is changing at.  The
#: speed's rate is ``None`` and is differenced instead - see :meth:`Trace.measure`,
#: which is the only place in this file where a number is not read off an
#: instrument, and it is a number no criterion below uses.
READINGS = {
    STATION: Reading(1, "m", station_error, station_rate),
    HEIGHT: Reading(0, "m", height_error,
                    lambda hands, telemetry, state: telemetry.height_rate),
    SPEED: Reading(1, "m/s", speed_error, None),
    TRACK: Reading(2, "m", track_error, track_rate),
}


def error_of(axis, hands, telemetry, state):
    """The loop's error, in that loop's own units: see :data:`READINGS`."""
    return READINGS[axis].error(hands, telemetry, state)


# ---------------------------------------------------------------------------
# One case, frame by frame, and the answers read out of it.
# ---------------------------------------------------------------------------


class Trace(PhaseTwoTrace):
    """One case of one outer loop, frame by frame, and the answers read out of it.

    Phase 2's own trace and its same four answers - when the error came inside the
    band and *stayed*, whether it is still growing at the end of the case, what is
    left of it once it oscillates, and what the *rest* of the flight did while the
    loop was being argued about - with the one thing a fourth set of units makes
    necessary: the loop's own band, out of :data:`SETTLE`, and the reading it is
    measured on, out of :data:`READINGS`.

    Two of the four differences from that file's traces are worth naming here.
    Every one of these errors is measured *off zero* - the pad, the trim's own
    height, the trim's own speed, the line - rather than off the trim's own
    attitude, so :attr:`reference` is nothing and the error is the whole of what
    the loop is being asked about.  And the *speed* has no rate in this model's
    instruments at all, so its trace carries a differenced one: a pilot reading a
    speed out of a machine is reading a difference too, and no criterion in this
    file is read off that number.
    """

    def __init__(self, sim, hands, axis, seconds, condition=HOVER):
        PhaseOneTrace.__init__(self, sim, hands, axis, seconds)
        reading = READINGS[axis]
        self.reading = reading
        self.index = reading.stick
        self.unit = reading.unit
        self.reference = 0.0
        self.band = SETTLE[axis]
        self.condition = condition
        #: The speed, against the one the trim was solved for - phase 2's own pair
        #: of numbers, kept here because the rest of the flight is read off them.
        self.speed_ref = hands.speed_ref
        self.speeds = []
        self.speed_low = self.speed_ref
        self.speed_high = self.speed_ref
        #: The last error, for the one reading this file cannot be handed a rate
        #: for: see the class docstring.
        self.previous = None

    def measure(self):
        """One frame's worth, from the state the frame loop has just left.

        Phase 2's own, for four loops that are in three different units and two
        different conditions: the error against what the trim left the loop on,
        the rate that error is changing at, the stick the hand has *moved* - the
        lagged command and not the one asked for - the height, the distance from
        the pad and the speed, which are what the rest of the flight cost.
        """
        telemetry = self.sim.telemetry()
        state = self.sim.airframe.state
        drift = math.hypot(state.position.x, state.position.y)
        error = self.reading.error(self.hands, telemetry, state)
        if self.reading.rate is not None:
            rate = self.reading.rate(self.hands, telemetry, state)
        elif self.previous is None:
            rate = 0.0
        else:
            rate = (error - self.previous) / SIM_TIME_STEP_S
        self.previous = error
        self.times.append(float(self.sim.sim_time))
        self.errors.append(error)
        self.rates.append(rate)
        self.sticks.append(self.hands.command[self.index])
        self.altitudes.append(telemetry.alt_agl)
        self.drifts.append(drift)
        self.speeds.append(telemetry.airspeed)
        self.lowest = min(self.lowest, telemetry.alt_agl)
        self.highest = max(self.highest, telemetry.alt_agl)
        self.drift = max(self.drift, drift)
        self.speed_low = min(self.speed_low, telemetry.airspeed)
        self.speed_high = max(self.speed_high, telemetry.airspeed)
        return self


# ---------------------------------------------------------------------------
# A case: the model's own trim, a person's hands around it, and one loop off it.
# ---------------------------------------------------------------------------


def perturbed_state(state, axis, size, airspeed=None):
    """*state*, copied, with *size* of *axis* put into it - in that loop's units.

    A state is *copied* before it is written into, and what is written into it is a
    new object: a position and an attitude are frozen triples in this model and a
    velocity is one too, so a perturbation is a whole new state rather than a field
    quietly changed under a running aircraft.

    Three of the four loops are a *place*: five metres off the pad, a metre up -
    NED is +z down, so the metre goes in as a negative step in z, exactly as phase
    2 puts its own metre in - and twenty metres across the line, which is the east
    axis.  The fourth is a *speed*, and it is put in by scaling the velocity: what
    the model reports as the airspeed is the wind's speed in the rotor's own plane,
    so the aircraft cannot be pushed into it by moving a stick at all.  The scale
    assumes the reading is linear in the velocity, which it is, and :func:`put_in`
    reads the answer back rather than trusting that.
    """
    flown = state.copy()
    position = flown.position
    if axis == STATION:
        flown.position = Vector3(position.x + size, position.y, position.z)
    elif axis == HEIGHT:
        flown.position = Vector3(position.x, position.y, position.z - size)
    elif axis == TRACK:
        flown.position = Vector3(position.x, position.y + size, position.z)
    elif axis == SPEED:
        assert airspeed, airspeed
        flown.velocity = flown.velocity * ((airspeed + size) / airspeed)
    else:
        raise ValueError("no such loop: %r" % (axis,))
    return flown


def put_in(sim, axis, size):
    """The perturbation put in, with *sim* left on the state it makes.

    The trim's own state with *size* of *axis* written into it, and for the three
    loops that are a place that is one write and done.  The *speed* is not: the
    scale the velocity needs is the ratio of two readings of an instrument, and
    since one pass is exactly right when the reading is linear in the velocity -
    which it is - the loop below is here to *say so* rather than to converge: it
    stops on the first pass that lands, and a state it could not land is raised
    rather than flown, since a perturbation that is not the size it claims makes
    every number below it a number about something else.
    """
    airspeed = sim.telemetry().airspeed
    state = perturbed_state(sim.trim_state, axis, size, airspeed)
    for _ in range(4):
        sim.reset(state=state, controls=sim.trim_controls)
        if axis != SPEED:
            return state
        read = sim.telemetry().airspeed
        if abs(read - (airspeed + size)) < 1e-12:
            return state
        state.velocity = state.velocity * ((airspeed + size) / read)
    raise AssertionError("the speed perturbation never landed: %.2e m/s out"
                         % (read - (airspeed + size),))


def case_setup(axis=STATION, condition=HOVER, outer=OUTER_ONE, size=None,
               task=None):
    """The model's own trim, a pilot built around it, and *axis* put off it.

    Phase 2's own :func:`_rt_axes.case_setup`, for four loops and two conditions:
    the aircraft is trimmed - to the hover at ``_rt_pilot``'s own twenty feet, or
    to level flight at sixty knots at the cruise height - the pilot is built around
    *that* trim, with its stick, its attitude, its height, its heading and its speed
    read off the aircraft rather than written down here, and handed the last fifth
    of a second of it, because the first decision of a case has to be made on the
    trim and not on the perturbation.  Only then is the perturbation put in.

    *task* is the pilot's, and it is the one thing about this file's cruise cases
    that is not that file's: ``None`` flies the model's own cruise pilot, whose
    speed and track loops are alight, and ``False`` flies phase 2's - the same
    aircraft, the same perturbation, and the same loops with the two task loops
    left to the trim, which is what :func:`ruler` is flown with.

    Returns ``(sim, trim, hands)`` with the perturbation already in the aircraft and
    neither of them flown yet.
    """
    size = PERTURB[axis] if size is None else float(size)
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.trim_level_flight(CONDITION_MPS[condition],
                          altitude=CONDITION_ALT_M[condition])
    # The two places the loops are measured against are the aircraft's own, and
    # they are asserted rather than assumed: the pad is where ``on_the_pad`` parks
    # an aircraft and where ``_rt_pilot`` names it, and the line north of the pad
    # is *through* the pad - so a hover's station-keeping is a loop round where the
    # machine already is and a cruise's track is a loop round where it started.
    assert abs(sim.trim_state.position.y - TRACK_EAST_REF_M) < 1e-6, \
        sim.trim_state.position.y
    if condition == HOVER:
        assert abs(sim.trim_state.position.x - HOVER_REF_NORTH_M) < 1e-6, \
            sim.trim_state.position.x
        assert abs(sim.trim_state.position.y - HOVER_REF_EAST_M) < 1e-6, \
            sim.trim_state.position.y
    trim = (sim.trim_controls.collective, sim.trim_controls.long_stick,
            sim.trim_controls.lat_stick, sim.trim_controls.pedal)
    hands = Hands(sim, trim, outer=outer, condition=condition, task=task,
                  pad=(sim.trim_state.position.x, sim.trim_state.position.y))
    hands.selftest().fill(sim)
    # What the pilot can see before the case begins has to be the trim: the
    # perturbation goes in *after* the eyes are filled, so the first fifth of a
    # second of every case is flown on what the aircraft was and not what it is.
    eyes = hands.view()
    assert eyes is not None, "the history was not filled"
    assert abs(error_of(axis, hands, eyes.telemetry,
                        sim.airframe.state)) < 1e-9, \
        error_of(axis, hands, eyes.telemetry, sim.airframe.state)
    put_in(sim, axis, size)
    # And the error the perturbation is worth is the size that was asked for -
    # which is the check that five metres off the pad is five metres of the reading
    # the station-keeping loop is written around, and a metre a second of airspeed
    # a metre a second of the reading the speed loop is.
    got = error_of(axis, hands, sim.telemetry(), sim.airframe.state)
    assert abs(got - size) < 1e-9, (axis, got, size)
    assert all(abs(one - want) < 1e-12 for one, want
               in zip(sim.pilot.axes(), inches_to_axes(*trim))), sim.pilot.axes()
    return sim, trim, hands


def case_line(sim, trace):
    """One line of a case's progress, in the loop's own units."""
    position = sim.airframe.state.position
    return ("    t %4.1f s | %-7s %+7.2f %s %+7.2f %s/s | stick %+5.2f in |"
            " %5.2f ft skid | %5.2f m out"
            % (sim.sim_time, trace.axis, trace.errors[-1], trace.unit,
               trace.rates[-1], trace.unit, trace.sticks[-1],
               skid_ft(sim.telemetry()),
               math.hypot(position.x, position.y)))


def fly_case(axis=STATION, condition=HOVER, outer=OUTER_ONE, size=None,
             task=None, seconds=None, every=EVERY):
    """Fly the trimmed case with a person's hands on it.

    :func:`case_setup` and :class:`Hands`; what this adds is the flight, and it is
    the case every number in this file is measured on.  Returns
    ``(sim, hands, trace)``.  *every* prints a line of the case's own progress that
    many frames apart, or nothing for ``None`` - which is what the sweeps fly at, a
    line a second of a dozen cases being a transcript nobody reads.
    """
    seconds = HOLD_OF[axis] if seconds is None else float(seconds)
    sim, trim, hands = case_setup(axis=axis, condition=condition, outer=outer,
                                  size=size, task=task)
    trace = Trace(sim, hands, axis, seconds, condition=condition)
    for frame in range(int(round(seconds / SIM_TIME_STEP_S))):
        hands.frame(sim, SIM_TIME_STEP_S)
        trace.measure()
        if every and frame % every == 0:
            print(case_line(sim, trace))
            sys.stdout.flush()
        if sim.crashed:
            break
    return sim, hands, trace


def frozen_case(axis=STATION, condition=HOVER, seconds=FROZEN_S, every=EVERY):
    """The same perturbation with *nobody* flying it: what the model does alone.

    The sticks are where the trim left them and no pilot moves them - a
    :class:`Hands` is still built, because the trace needs one for its references,
    and never flown, which is exactly what a pair of hands off the controls is - so
    what follows is the aircraft's own answer to the perturbation and nothing else.
    It is the reference every band in this file is read against, and half the
    reason a loop has a *low* edge as well as a high one: not one of these four is
    a place this model stays by itself, and a pilot's margin is what their hands
    buy against that.
    """
    sim, trim, hands = case_setup(axis=axis, condition=condition)
    trace = Trace(sim, hands, axis, seconds, condition=condition)
    for frame in range(int(round(seconds / SIM_TIME_STEP_S))):
        sim.fly(SIM_TIME_STEP_S)          # no keys held: the sticks do not move
        trace.measure()
        if every and frame % every == 0:
            print(case_line(sim, trace))
            sys.stdout.flush()
        if sim.crashed:
            break
    return sim, hands, trace


def case(axis=STATION, condition=HOVER, outer=OUTER_ONE, size=None, task=None,
         every=EVERY, label=None):
    """Fly one case at *outer* of *axis* and hand back its trace.

    The case is the whole of what every number in this file is read off, so the two
    things a harness owes are paid here: the frame loop dropped no step of the
    flight, and the flight's own record of the pilot's decisions - its clock and the
    age of every view it read - is printed beside the trace.  The clock itself is
    :func:`clock`'s business below, which measures it rather than quoting it; a
    case that crashed is not an error here: it is one of the answers a grip can
    give, and the trace is what says so.
    """
    print("")
    print("--- %s: %s put in, %s"
          % (label or "case", PERTURB_WORDS[axis] % (PERTURB[axis],),
             gain_words(axis, outer)))
    sim, hands, trace = fly_case(axis=axis, condition=condition, outer=outer,
                                 size=size, task=task, every=every)
    print("  %s" % (trace,))
    print("  the hands: %d decisions (%0.1f a second), %0.2f in of stick peak to"
          " peak, worst %0.1f %s/s | height %+0.2f to %+0.2f m of the trim | %s"
          % (hands.decisions, hands.decisions / max(sim.sim_time, 1e-9),
             trace.stick_range(), trace.worst_rate(), trace.unit,
             trace.lowest - hands.alt_ref, trace.highest - hands.alt_ref,
             "no step dropped" if not sim.dropped_steps
             else "%d STEPS DROPPED" % (sim.dropped_steps,)))
    if sim.crashed:
        print("  CRASHED at %.2f s: %s" % (sim.sim_time, sim.crash_reason))
    sys.stdout.flush()
    assert not sim.dropped_steps, sim.dropped_steps
    return trace


# ---------------------------------------------------------------------------
# One loop's whole band, and the clock the pilot flies it at.
# ---------------------------------------------------------------------------


def stays(trace):
    """Did the error come back inside the band and *stay* inside it to the end?

    The half of :func:`holds` that is about the error rather than about the
    machine, and the reading the frozen cases below are described by: a case whose
    error never came inside the band at all has not stayed, and neither has one that
    came inside and left again - however small the number ``settled`` returned.
    """
    settled = trace.settled()
    if settled is None or not trace.errors:
        return False
    start = int(round(settled / SIM_TIME_STEP_S))
    return all(abs(error) <= trace.band for error in trace.errors[start:])


def holds(sim, trace):
    """Did this grip hold the loop: phase 1's own words, read off the whole case.

    Phase 1's criterion is three things - the error came back inside the loop's band
    and *stayed*, what is left of it is still dying out at the end of the case, and
    the machine is in the sky to have done it - and what ``_rt_handling.py`` and
    ``_rt_axes.py`` read the middle one off is the worst error in the last third of
    a case against the worst in its middle third.  That is an *approximation* of
    "stayed", and on these four loops it is not good enough, in both directions:

    * a loop whose answer comes home and sits a tenth of its own band *inside* it,
      riding the aircraft's own slow mode, has a tail that drifts up - which is a
      loop that stayed home, called a failure by the ratio;
    * and a loop whose answer swings clean past the band and comes back towards the
      end of the case has a tail that is *falling* while the error is still outside
      it - which is a loop that did not stay, called a hold by the ratio.  The track
      loop at x1.50 is exactly that: a tail of 0.95 and an error ten metres off the
      line at the end of the case.

    So this file reads the words themselves: :func:`stays`, and the machine in the
    sky.  The tail against the middle third is still printed beside every row and
    still pinned for the nominal case, because it is what says how much of a swing
    is left; and on every case of the four phases before this one the two rules
    agree, their failures being cases that leave the band rather than loiter in it.
    """
    return not sim.crashed and stays(trace)


def why_not(trace):
    """How a case that did not hold failed, in the loop's own units.

    :func:`holds` said back: the machine left the sky, the error never held the band
    at all, or it held it and then left it again - with the instant it left and
    where it ended up, which is what the tail ratio printed beside it is a summary
    of and what that file's own ``failed_words`` could not say, its wording being a
    tail worse than a middle third whether or not the error was ever outside the
    band.
    """
    if trace.sim.crashed:
        return "the machine left the sky at %.2f s" % (trace.sim.sim_time,)
    settled = trace.settled()
    if settled is None:
        return ("it never held the %.2f %s band for %.1f s at all, reaching %+0.2f %s"
                % (trace.band, trace.unit, SETTLE_S, trace.overshoot()[0],
                   trace.unit))
    start = int(round(settled / SIM_TIME_STEP_S))
    for index in range(start, len(trace.errors)):
        if abs(trace.errors[index]) > trace.band:
            return ("it came inside the %.2f %s band at %.2f s and left it again at"
                    " %.2f s, ending %+0.2f %s out"
                    % (trace.band, trace.unit, settled, trace.times[index],
                       trace.errors[-1], trace.unit))
    return ("it held the %.2f %s band from %.2f s to the end of the case"
            % (trace.band, trace.unit, settled))


def leg(axis, condition, ladder, harder, task=None):
    """One direction of a loop's band: outward from the model's own until a grip fails.

    *held* is :func:`holds`, and a row's three numbers are what the criterion is
    read off: the settling, the tail against the middle third, and the period of
    whatever is left.  A leg stops at the first scale that fails, since a scale is
    flown to find an edge and not to fill a column - except the harder one, which
    climbs past the edge until the *pilot's own loop* is what has closed, because
    the edge of a band and the grip a loop turns into a PIO at are two different
    numbers and both are worth having.

    Returns ``(edge, pio_at, rows)``: a row's ``(scale, held, trace)``, the first
    scale that failed, and - on the harder leg only - the first scale whose
    *failure* is a pilot induced oscillation rather than the aircraft's own mode.
    """
    print("  -- gripped %s than the model's own pilot:"
          % ("harder" if harder else "softer"))
    edge = None
    pio_at = None
    rows = []
    for scale in ladder:
        sim, hands, trace = fly_case(axis=axis, condition=condition,
                                     outer=outer_for(axis, scale), task=task,
                                     every=None)
        settled = trace.settled()
        worst, _ = trace.overshoot()
        tail, period, growth = trace.tail_growth(), trace.period(), trace.growth()
        is_pio = trace.pio()[0]
        held = holds(sim, trace)
        rows.append((scale, held, trace))
        rest = trace.drift if condition == HOVER else trace.speed_error()
        print("  %5.2f %+6.1f %+7.2f %10s %10s %9s %8s %8.2f %8.2f%s"
              % (scale, 20.0 * math.log10(scale), worst,
                 "%6.2f s" % settled if settled is not None else "never",
                 "%6.2f" % tail if tail is not None else "none",
                 "%6.2f s" % period if period else "none",
                 "%6.2f" % growth if growth else "none",
                 trace.altitude_error(), rest,
                 "  <- the edge" if not held and edge is None else
                 "  <- the PIO" if is_pio and pio_at is None else ""))
        sys.stdout.flush()
        if not held:
            print("  this grip did not hold: %s" % (why_not(trace),))
            if edge is None:
                edge = scale
            if is_pio:
                pio_at = scale
                print("  and its failure is a pilot induced oscillation: the"
                      " pilot's own %.2f s of delay is worth %.0f deg of a %.2f s"
                      " period, so the loop that has closed is the pilot's own and"
                      " not the aircraft's."
                      % (DELAY_S, Trace.phase_deg(period), period))
            if not harder or pio_at is not None:
                break
    if edge is None:
        print("  every grip in this leg held.")
    elif harder and pio_at is None:
        print("  the edge is x%.2f and nothing up to x%.2f is a pilot induced"
              " oscillation: what fails at the edge is the aircraft's own mode and"
              " not the pilot's loop." % (edge, ladder[-1]))
    return edge, pio_at, rows


def band(axis, condition, task=None):
    """Fly both legs of one loop's band and hand back the two edges.

    Every loop of this model has *two* edges and not one, and they fail
    differently: too hard a grip closes the pilot's own loop on the pilot's own
    delay, and too soft a one cannot hold what the loop is being asked for - which
    for four loops that are not all closed *by* the pilot is a real question rather
    than a formality: the height is held by the lever's loop and the ground, the
    track and the speed by the aircraft's own dynamics underneath them, and a loop
    a hand can barely close at all may have no soft edge inside any ladder.

    Returns ``(margin, pio_at, floor, harder_rows, softer_rows)``, with an edge
    ``None`` if its leg never found one - which is an answer too, and the one a
    forgiving loop gives.
    """
    print("")
    print("=== %s, %s: %s put in, %s scaled ==="
          % (LOOP_WORDS[axis], CONDITION_WORDS[condition],
             PERTURB_WORDS[axis] % (PERTURB[axis],), loop_words(axis)))
    print("  the model's own: %s" % (gain_words(axis, OUTER_ONE),))
    print("  scale      dB    worst   settled    tail/mid   period   growth"
          "   height m   %s"
          % ("drift m" if condition == HOVER else "speed m/s",))
    margin, pio_at, up = leg(axis, condition, SCALES_HARDER, True, task)
    floor, _, down = leg(axis, condition, SCALES_SOFTER, False, task)
    return margin, pio_at, floor, up, down


def clock(axis=HEIGHT, condition=CRUISE, seconds=5.0):
    """Fly a short case and hold the pilot itself to the clock it claims.

    Phase 1's own check, on a loop neither it nor phase 2 ever flew *with the task
    loops alight*: the cruise pilot's speed and track loops make this the one pilot
    in the programme that is not phase 1's hover pilot slowed down, which is why
    the clock is measured again rather than inherited.  The number of decisions has
    to be the pilot's own rate over the time flown, the age of every view a
    decision was made from has to be the ``DELAY_S`` of the delay to the last bit,
    and the whole run has to have been flown without the frame loop dropping a
    step: a harness whose clock is not what it says makes every number under it
    meaningless.
    """
    print("")
    print("--- the instrument itself: the pilot's own clock over %.0f s of a %s"
          " case, with the task loops alight" % (seconds, axis))
    sim, hands, trace = fly_case(axis=axis, condition=condition, seconds=seconds,
                                 every=None)
    flown = trace.times[-1] + SIM_TIME_STEP_S
    ages = hands.ages
    print("  %0.2f s flown: %d decisions (%0.2f a second, the pilot's own is %0.0f)"
          " | every view %0.4f s old (the delay is %.2f) | %s"
          % (flown, hands.decisions, hands.decisions / flown, PILOT_HZ,
             ages[0] if ages else float("nan"), DELAY_S,
             "no step dropped" if not sim.dropped_steps
             else "%d STEPS DROPPED" % (sim.dropped_steps,)))
    assert ages, "the pilot never looked"
    assert len(ages) == hands.decisions, (len(ages), hands.decisions)
    assert abs(ages[0] - DELAY_S) < 1e-9, ages[0]
    assert max(ages) - min(ages) < 1e-9, (min(ages), max(ages))
    assert abs(hands.decisions / flown - PILOT_HZ) <= 1.0, hands.decisions / flown
    assert not sim.dropped_steps, sim.dropped_steps
    sys.stdout.flush()
    return trace


# ---------------------------------------------------------------------------
# What the run measured, and so what it is held to.
# ---------------------------------------------------------------------------

#: The slack a pinned oscillation is compared at: a hundredth of a second of
#: period and a hundredth of the growth a cycle, which is finer than anything the
#: twenty hertz of decisions can hide in and coarser than the last bits of a float.
PIO_TOLERANCE = 0.01

#: The nominal case of every loop, s, as this file first flew it: the model's own
#: pilot for the condition, at a scale of one on all four outer loops, against a
#: perturbation it did not ask for.
#:
#: Two of the five are not this file's numbers at all and are written here as phase
#: 2's: the height loop at a scale of one *is* that file's heave case, in both
#: conditions - see :func:`ruler` - and pinning them as that file's own constants
#: is what makes the claim a check rather than a sentence.  Two more are this file's
#: own, and each of them is a loop no phase before this one ever flew: the hover's
#: station-keeping pair against a *place* rather than an attitude, and the cruise
#: pilot's speed loop, whose residual rides the aircraft's own slow airspeed mode.
#: The fifth - the track - has no settling time to pin at all, because its own grip
#: does not hold it; see :data:`NOMINAL_HELD` and :data:`NOMINAL_TAIL`.
NOMINAL_SETTLED_S = {
    (STATION, HOVER): 11.57,
    (HEIGHT, HOVER): PHASE_TWO_NOMINAL["heave"],
    (HEIGHT, CRUISE): 20.22,
    (SPEED, CRUISE): 6.45,
    (TRACK, CRUISE): None,
}

#: And whether the model's own grip *holds* each loop at all, which is :func:`holds`
#: and not merely "the number ``settled`` returned": a loop whose error comes home
#: and then leaves the band again returns a settling time and has settled nothing.
#: A loop whose own gain is outside its own band is an answer and not an error - it
#: is what a script never has to find out and a hand does - so it is pinned as the
#: ``False`` it is, with the reading that says so beside it.
NOMINAL_HELD = {(STATION, HOVER): True, (HEIGHT, HOVER): True,
                (HEIGHT, CRUISE): True, (SPEED, CRUISE): True,
                (TRACK, CRUISE): False}

#: And the tail of every nominal case: the worst error in the last third of it
#: against the worst in its middle third - the reading the four phases before this
#: one took their "still dying out" from, kept beside :func:`holds` here rather
#: than inside it.
#:
#: It is a reading of every loop and is pinned for every loop: a tail under one is a
#: swing that is still being taken out, and a tail over one on a loop whose error
#: never left the band is the aircraft's own slow mode showing through a loop that
#: *has* settled - which is the speed loop, whose residual rides the aircraft's own
#: slow airspeed mode and drifts up within a tenth of its band.
NOMINAL_TAIL = {
    (STATION, HOVER): 0.05,
    (HEIGHT, HOVER): 0.16,
    (HEIGHT, CRUISE): 0.33,
    (SPEED, CRUISE): 2.20,
    (TRACK, CRUISE): 0.92,
}

#: And the five bands, as the two legs measured them: *every* scale in the ladders
#: that held, in order, and then the scale the harder leg failed first at, the scale
#: the softer leg failed first at (``None`` where a leg never found one, which is an
#: answer and not a gap), and the scale the loop turns into a pilot induced
#: oscillation at (``None`` where its failure never is one).
#:
#: The held scales are pinned as the *set* and not as the two ends of it, because a
#: loop closed through a transport delay does not have to fail monotonically: a grip
#: can fail, a stiffer one hold, and a stiffer one than that fail again, and a band
#: reported as its two ends would hide the hole in the middle of it.  See the track
#: loop's own rows.
BANDS = {
    (STATION, HOVER): ((0.10, 0.25, 0.35, 0.50, 0.75, 1.50), 2.00, None, None),
    (HEIGHT, HOVER): ((0.25, 0.35, 0.50, 0.75, 1.50, 2.00, 3.00, 4.00, 6.00,
                       8.00), None, 0.10, None),
    (HEIGHT, CRUISE): ((0.50, 0.75, 1.50, 2.00, 3.00, 4.00, 6.00, 8.00), None,
                       0.35, None),
    (SPEED, CRUISE): ((0.10, 0.25, 0.35, 0.50, 0.75, 1.50, 2.00, 3.00, 4.00),
                      6.00, None, None),
    (TRACK, CRUISE): ((), 1.50, 0.75, None),
}

#: What the failures that *are* the pilot's oscillate at: the period, s, and the
#: growth a cycle, of the PIO each loop closes.  Empty, and that is this phase's
#: answer rather than a gap in it: **not one of the four outer loops' failures is
#: the pilot's own loop.**  Every one of them is the aircraft's - a swing past the
#: band that the delay cannot see, or a mode of tens of seconds - and the one
#: oscillation that *is* at a period a person's loop lives in, the track loop's
#: 1.67 s ripple, is neither growing nor dying: 1.00 a cycle against this file's
#: own line of 1.02 for what a PIO is.
PIO_MEASURED = {}

#: And what each loop does with nobody flying it: ``(shape, limit)``, where "out" is
#: a case whose error never *stayed* inside the band and was at least *limit* off at
#: the end of it, and "back" is one that came home and stayed, inside *limit* when
#: the case ran out.  The shape is :func:`stays`, the same reading :func:`holds`
#: passes verdicts from, and it is why there are two and not three: a frozen case
#: can cross its band and leave again - the speed does - and a vocabulary built on
#: ``settled`` alone would have to call that either never or back.
#:
#: Not one of the four is a place this model finds by itself.  The pad and the line
#: are things the aircraft does not know about at all, so an error in either is an
#: error it keeps for ever; its height is a place the *cushion* finds over the pad
#: and no place at all once the ground is a hundred metres down; and its speed is
#: given back by nothing in twenty seconds of flying.  That is the reference every
#: band above is read against, and the whole of what a pilot's hands buy.
FROZEN = {
    (STATION, HOVER): ("out", 4.99),
    (HEIGHT, HOVER): ("back", 0.05),
    (HEIGHT, CRUISE): ("out", 0.99),
    (SPEED, CRUISE): ("out", 1.50),
    (TRACK, CRUISE): ("out", 19.99),
}


def trace_at(rows, scale):
    """The trace of one scale of a leg: :func:`leg`'s own rows."""
    return [trace for one, _, trace in rows if one == scale][0]


def pio_verdict(scale, trace):
    """Say what is oscillating in a case, and whether it is a pilot's own loop.

    ``_rt_axes.pio_report``'s own reading - the period, the growth a cycle, and what
    the ``DELAY_S`` of delay is worth at that period - with one case that file never
    had to print: a trace whose period is inside the band a person's own loop lives
    in and whose *growth* cannot be measured at all.  A loop can be oscillating
    where a hand's loop lives and still have too few peaks left to read a growth
    between - a case cut shorter than the mode it is measuring is exactly that, and
    is what ``TRACK_HOLD_S`` above is chosen against - and that is a reading of the
    case and not an error: what it says is that the case ran out before the
    oscillation had repeated, not that the oscillation is not growing.
    """
    is_pio, period, growth = trace.pio()
    if period is not None and growth is None:
        print("  x%0.2f %s: period %.2f s, inside the %.1f to %.1f s a person's own"
              " loop lives in, and too little of it left to measure a growth - the"
              " case ran out before the oscillation repeated"
              % (scale, trace.axis, period, PIO_PERIOD_S[0], PIO_PERIOD_S[1]))
        sys.stdout.flush()
        return False
    return pio_report(scale, trace)


# ---------------------------------------------------------------------------
# The ruler, and the verdict on each loop.
# ---------------------------------------------------------------------------


def ruler(nominal):
    """The check that this file's height loop *is* phase 2's own heave case.

    Two cases, and the hover's is not a second flight at all: over the pad this
    file's pilot and that file's are the same pilot - there is no task loop to
    switch off in a hover - so the nominal case above *is* the ruler, and what is
    checked is that its settling time is the number that file pinned.  The cruise's
    is flown here, with the two task loops left to the trim: ``task=False`` is the
    hover pilot's four loops aloft, which is exactly the pilot that file flies at
    sixty knots, so the metre of height, the band, the forty seconds and the loops
    around them are all that file's own and nothing of this file's is in it.

    A drift here would be a drift in the instrument and not a difference of
    opinion - the two files' pilots would have stopped being one pilot - and it is
    the claim every number in this file rests on.  Returns the two settling times.
    """
    print("")
    print("=== the ruler: this file's height loop against ``_rt_axes``' own heave"
          " case ===")
    print("  A metre of height, the same RATE_GAIN_MPS, the same lever, a scale of"
          " one and the same band, with the two task loops of a cruise left to the"
          " trim - which is the pilot that file flies aloft.")
    flown = {HOVER: nominal[(HEIGHT, HOVER)].settled()}
    flown[CRUISE] = case(HEIGHT, CRUISE, task=False,
                         label="the height loop, flown by the pilot phase 2 flew"
                         ).settled()
    for condition, pinned in ((HOVER, PHASE_TWO_NOMINAL["heave"]),
                              (CRUISE, PHASE_TWO_CRUISE["heave"])):
        print("  %-6s: this file settles the metre in %s; ``_rt_axes`` pinned %.2f s"
              " and allows itself %.2f s"
              % (condition, words(flown[condition], "%5.2f s"),
                 pinned, SETTLE_TOLERANCE_S))
        assert flown[condition] is not None, (condition, flown[condition])
        assert abs(flown[condition] - pinned) <= SETTLE_TOLERANCE_S, \
            (condition, flown[condition], pinned)
    sys.stdout.flush()
    return flown


def verdict(axis, condition, nominal, result, frozen):
    """What one loop's whole case is, in that loop's own units.

    The one place the four loops are read side by side, and it is written so that
    they can be: what the model's own grip did with the perturbation, the height and
    the rest of the flight it cost, the band of grips either side of it, the two
    edges, the grip the pilot's own loop closes at where there is one, and what the
    aircraft does with nobody flying it at all.  Loop by loop these are numbers in
    three different units; read down this block they are the same answers on four
    loops, which is what a *pilot's* margin is and not a loop's.
    """
    margin, pio_at, floor, harder, softer = result
    unit, settle = READINGS[axis].unit, SETTLE[axis]
    held = sorted(scale for scale, ok, _ in harder + softer if ok)
    print("")
    print("  %s, %s: %s put in, back inside %0.2f %s, and the model's own grip %s"
          % (axis, CONDITION_WORDS[condition],
             PERTURB_WORDS[axis] % (PERTURB[axis],), settle, unit,
             "settles it in %s" % words(nominal.settled(), "%5.2f s")
             if holds(nominal.sim, nominal) else
             "does NOT hold it - %s" % (why_not(nominal),)))
    print("    having cost %.2f m of height and %s, and ending %+0.2f %s out, with"
          " a tail %s of its middle third"
          % (nominal.altitude_error(), nominal.rest_words(), nominal.errors[-1],
             unit, words(nominal.tail_growth(), "%4.2f")))
    print("    the band: %s%s"
          % ("nothing in either leg held" if not held else
             "the grips that held, %s, of the model's own %s"
             % (", ".join("x%.2f" % (one,) for one in held), loop_words(axis)),
             "" if (margin is not None or floor is not None) else
             ", and nothing in either leg failed either"))
    if floor is not None:
        print("    below it the loop does not hold what it is being asked for:"
              " x%.2f (%.1f dB) - %s"
              % (floor, 20.0 * math.log10(floor),
                 why_not(trace_at(softer, floor))))
    if margin is not None:
        hard = trace_at(harder, margin)
        print("    above it: x%.2f (%+.1f dB) - %s%s"
              % (margin, 20.0 * math.log10(margin), why_not(hard),
                 "" if hard.period() is None else
                 ", at a %.2f s period the pilot's %.2f s of delay is worth %.0f deg"
                 " of that period"
                 % (hard.period(), DELAY_S, Trace.phase_deg(hard.period()))))
    if pio_at is not None:
        _, period, growth = trace_at(harder, pio_at).pio()
        print("    and the pilot's own loop closes at x%.2f (%+.1f dB): %.2f s a"
              " cycle, growing %.2f of itself a cycle, with the delay worth %.0f deg"
              " of that period - A PILOT INDUCED OSCILLATION"
              % (pio_at, 20.0 * math.log10(pio_at), period, growth,
                 Trace.phase_deg(period)))
    print("    with nobody flying it: %s" % (frozen,))
    sys.stdout.flush()
    return margin, pio_at, floor


# ---------------------------------------------------------------------------
# The second half: the script's own claims, read against a person's flight.
# ---------------------------------------------------------------------------

#: One claim ``_rt_check.check`` makes of a mission, as a *reading* of a flight:
#: the mission it belongs to, what it asks in that file's own words, and the
#: function that answers it, ``(hands, trip) -> (held, words)``.
#:
#: ``_rt_check`` asserts these of its own *scripted* flight, which is what makes
#: them the missions' claims rather than this file's; what is asked below is what a
#: *person's* flight answers, which is a different question and the one ``TODO.md``
#: left on this programme.  The words beside each verdict are the number the claim
#: was answered with, and they are why the table is read rather than merely
#: counted: a claim that is missed says how it was missed, and a claim that is held
#: says what held it.
Claim = namedtuple("Claim", "mission asks answer")


def turn_for(hands):
    """How long the short mission's turn took, s, or ``None`` if it never ran.

    A flight's own two instants - when the schedule began and when it was called
    done - read out of the pilot rather than out of the frame loop, and ``None``
    where there is no turn to measure: a claim about a turn that never happened is
    a claim a person's flight cannot answer, which is a miss and not a gap.
    """
    if hands.turn_done is None or hands.turn_begun is None:
        return None
    return hands.turn_done - hands.turn_begun


def turn_rate_of(hands):
    """What the turn averaged, deg/s, or ``None`` if there is no turn to average."""
    took = turn_for(hands)
    if took is None or took <= 0.0:
        return None
    return hands.turned_deg / took

#: The short mission's own twenty, in ``_rt_check.check``'s own order: off the pad,
#: into the band of twenty feet, held, the whole way round to the right, the nose
#: back on the heading it began from, down onto the pad and settled - and, in front
#: of them, the three things that are true of any flight of it at all: that it did
#: not crash, that the frame loop dropped no step, and that the four axes ended
#: where the pad holds them.
HOVER_CLAIMS = (
    Claim("hover-turn", "the flight did not end as a crash",
          lambda hands, trip: (
              not trip.sim.crashed,
              trip.sim.telemetry().crash_message or "in the air")),
    Claim("hover-turn", "the frame loop dropped no step of it",
          lambda hands, trip: (
              not trip.sim.dropped_steps,
              "%d steps dropped" % (trip.sim.dropped_steps,))),
    Claim("hover-turn", "it left the pad",
          lambda hands, trip: (
              hands.took_off is not None,
              words(hands.took_off, "%5.1f s", "never off the pad"))),
    Claim("hover-turn", "it met the band of twenty feet",
          lambda hands, trip: (
              hands.hover_at is not None,
              words(hands.hover_at, "%5.1f s", "never in the band"))),
    Claim("hover-turn", "the hover was held at twenty feet of skid, inside %.1f m"
          % (HOVER_TOLERANCE_M,),
          lambda hands, trip: (
              abs(trip.skid_ft - HOVER_SKID_FT) <= HOVER_TOLERANCE_M / FOOT,
              "%5.1f ft of skid" % (trip.skid_ft,))),
    Claim("hover-turn", "the top of the climb stayed inside %.1f m of the trim"
          % (HOVER_TOLERANCE_M,),
          lambda hands, trip: (
              trip.highest - HOVER_ALT_M <= HOVER_TOLERANCE_M,
              "%+0.2f m over the trim" % (trip.highest - HOVER_ALT_M,))),
    Claim("hover-turn", "the turn finished",
          lambda hands, trip: (
              hands.turn_done is not None,
              words(hands.turn_done, "%5.1f s", "never finished"))),
    Claim("hover-turn", "the whole way round to the right: 360 degrees of schedule",
          lambda hands, trip: (
              hands.turned_deg == 360.0,
              "%5.1f deg of schedule" % (hands.turned_deg,))),
    Claim("hover-turn", "the nose back on the heading it began from, inside %0.0f"
          " deg" % (TURN_TOLERANCE_DEG,),
          lambda hands, trip: (
              hands.turn_error_deg is not None
              and abs(hands.turn_error_deg) <= TURN_TOLERANCE_DEG,
              words(hands.turn_error_deg, "%+5.1f deg"))),
    Claim("hover-turn", "the turn took between 24 and 40 s",
          lambda hands, trip: (
              24.0 <= turn_for(hands) <= 40.0,
              words(turn_for(hands), "%5.1f s"))),
    Claim("hover-turn", "the turn averaged ten degrees a second or better",
          lambda hands, trip: (
              turn_rate_of(hands) is not None and turn_rate_of(hands) >= 10.0,
              words(turn_rate_of(hands), "%5.1f deg/s"))),
    Claim("hover-turn", "the nose ended within fifteen degrees of the heading it"
          " began on",
          lambda hands, trip: (
              abs(wrap(trip.sim.telemetry().yaw_deg - hands.turn_heading_deg))
              <= 15.0,
              "%+5.1f deg" % (wrap(trip.sim.telemetry().yaw_deg
                                   - hands.turn_heading_deg),))),
    Claim("hover-turn", "down on the pad, and the phase machine called it landed",
          lambda hands, trip: (
              trip.sim.on_ground and hands.phase == "landed",
              "%s, the phase is %s"
              % ("on the pad" if trip.sim.on_ground else "in the air",
                 hands.phase))),
    Claim("hover-turn", "the mission finished",
          lambda hands, trip: (
              hands.done is not None,
              words(hands.done, "%5.1f s", "never finished"))),
    Claim("hover-turn", "the skids arrived under %.2f m/s"
          % (HOVER_TOUCHDOWN_LIMIT_MPS,),
          lambda hands, trip: (
              trip.touchdown_rate is not None
              and trip.touchdown_rate < HOVER_TOUCHDOWN_LIMIT_MPS,
              words(trip.touchdown_rate, "%5.2f m/s", "never arrived"))),
    Claim("hover-turn", "settled: the height rate under a centimetre a second",
          lambda hands, trip: (
              abs(trip.sim.telemetry().height_rate) < 0.01,
              "%+0.4f m/s of height rate"
              % (trip.sim.telemetry().height_rate,))),
    Claim("hover-turn", "it stayed inside %.0f m of the pad"
          % (HOVER_DRIFT_LIMIT_M,),
          lambda hands, trip: (
              trip.worst_drift <= HOVER_DRIFT_LIMIT_M,
              "%5.1f m off the pad at the worst" % (trip.worst_drift,))),
    Claim("hover-turn", "the roll ended under a degree",
          lambda hands, trip: (
              abs(trip.sim.telemetry().roll_deg) < 1.0,
              "%+5.2f deg" % (trip.sim.telemetry().roll_deg,))),
    Claim("hover-turn", "the pitch ended under a degree",
          lambda hands, trip: (
              abs(trip.sim.telemetry().pitch_deg) < 1.0,
              "%+5.2f deg" % (trip.sim.telemetry().pitch_deg,))),
    Claim("hover-turn", "the four axes ended on the pad's own sticks, inside a step",
          lambda hands, trip: (
              all(abs(axis - want) <= step
                  for axis, want, step
                  in zip(trip.sim.pilot.axes(), hands.pad_axes, KEY_STEP)),
              "%s in" % (", ".join("%+0.2f" % (one,)
                                   for one in trip.sim.pilot.axes()),))),
)

#: And the long mission's six, which are its own file's: not crashed, no step
#: dropped, off the pad and climbed, a hundred metres at the top inside the
#: tolerance that mission gives itself, a kilometre out, and finished.
CRUISE_CLAIMS = (
    Claim("cruise", "the flight did not end as a crash",
          lambda hands, trip: (
              not trip.sim.crashed,
              trip.sim.telemetry().crash_message or "in the air")),
    Claim("cruise", "the frame loop dropped no step of it",
          lambda hands, trip: (
              not trip.sim.dropped_steps,
              "%d steps dropped" % (trip.sim.dropped_steps,))),
    Claim("cruise", "it left the pad and climbed",
          lambda hands, trip: (
              hands.took_off is not None and hands.climbed is not None,
              "off at %s, and a hundred metres at %s"
              % (words(hands.took_off, "%5.1f s", "never"),
                 words(hands.climbed, "%5.1f s", "never")))),
    Claim("cruise", "the top of the climb was a hundred metres, inside %.0f m"
          % (ALT_TOLERANCE_M,),
          lambda hands, trip: (
              abs(trip.highest - ALT_REF_M) <= ALT_TOLERANCE_M,
              "%6.1f m at the top" % (trip.highest,))),
    Claim("cruise", "the kilometre",
          lambda hands, trip: (
              trip.distance_m >= DIST_REF_M,
              "%5.0f m out at the end" % (trip.distance_m,))),
    Claim("cruise", "the mission finished",
          lambda hands, trip: (
              hands.done is not None,
              words(hands.done, "%5.1f s", "never finished"))),
)

CLAIMS = {"hover-turn": HOVER_CLAIMS, "cruise": CRUISE_CLAIMS}

#: The claims a hand misses, mission by mission, as this file first read them.
#:
#: This is the second half's whole finding and it is pinned rather than asserted
#: away: the claims are the missions' own and are not loosened here, so a claim a
#: hand cannot meet is a fact about the hand rather than a bug in the claim - and a
#: claim this file *could* meet and does not is a change in the aircraft, which is
#: exactly what a pin is for.  See :func:`check_claims` for the two things asserted
#: about them, and :func:`say_the_finding` for what they mean.
CLAIM_MISSES = {"hover-turn": ("the skids arrived under 1.00 m/s",), "cruise": ()}


def the_claims(stream):
    """The script's own claims of each mission, read against a person's flight.

    Two flights, one a mission, each through ``_rt_pilot_hand.flight`` - that
    file's own frame loop and its own hands, so that this file adds no pilot here -
    and each **at the instrument's own clock**: ``CLOCKS[0]``, the twenty decisions
    a second phases 1 to 4 fly the instrument at.  The other clock that file flies,
    the delay itself, is its own row and not this file's.

    The script's claims are put *in force* first, by flying each mission the way
    ``_rt_check.check`` flies it - called rather than copied, with the limits that
    file gives itself - so that the claims the table is read against are the ones
    the harness actually holds itself to: a mission that stopped reaching its own
    goals would stop this file rather than quietly become the table's baseline.

    Returns ``[(mission, hands, trip, [(claim, held, words)])]``.
    """
    say(stream, "")
    say(stream, "the script's own claims, read against a person's flight of the"
        " same mission:")
    say(stream, "  every assertion ``_rt_check.check`` makes of its own scripted"
        " flight, in that file's own order - and they are the missions' claims"
        " because that is where they are written, so nothing here is loosened or"
        " edited")
    say(stream, "  the answer beside each is a *person's* flight of the same"
        " mission at the instrument's own clock: %.0f decisions a second, %.1f s of"
        " transport delay, %.1f s of lag in the hand"
        % (PILOT_HZ, DELAY_S, T_LAG_S))
    # The two files' idea of "the instrument's own clock" has to be one clock.
    assert CLOCKS[0] == PILOT_PERIOD_S, (CLOCKS, PILOT_PERIOD_S)
    rows = []
    for name, mission, hand in HAND_MISSIONS:
        # The script's own flight first, which asserts the claims this table is
        # read against rather than quoting them.
        say(stream, "")
        say(stream, "  the %s, flown by its own script first: %s"
            % (name, mission.DESCRIPTION))
        scripted_check(name, REACTION_S, LIMITS[name])
        hands, trip = flight(name, reaction=CLOCKS[0], stream=stream, trace=False,
                             label="the same mission by hand, %.2f s between"
                                   " decisions" % (CLOCKS[0],))
        answered = []
        for claim in CLAIMS[name]:
            held, why = claim.answer(hands, trip)
            answered.append((claim, bool(held), why))
        rows.append((name, hands, trip, answered))
        met = sum(1 for _, held, _ in answered if held)
        say(stream, "")
        say(stream, "  the %s's own %d claims, answered by that flight: %d met"
            % (name, len(answered), met))
        for claim, held, why in answered:
            say(stream, "    %-6s %s" % ("held" if held else "MISSED", claim.asks))
            say(stream, "           %s" % (why,))
    return rows


def check_claims(rows):
    """Hold the claims table to what it says, or raise.

    Two things, and neither of them is the claims themselves - those are the
    missions' own and are asserted where they are written, by the rehearsals above.
    First, every claim is *answered*: a claim a flight cannot answer at all - a turn
    that never ran, a number that was never measured - is reported as the miss it
    is, and a table of words where a number should be would be a reading of
    nothing.  And second, the misses are the ones pinned in :data:`CLAIM_MISSES`,
    which is this half's whole finding: a claim a hand used to meet and now misses,
    or the other way round, is a change in the aircraft or in the pilot, and this
    is the only place it is caught.
    """
    for name, hands, trip, answered in rows:
        assert len(answered) == len(CLAIMS[name]), (name, len(answered))
        asks = [claim.asks for claim, _, _ in answered]
        assert len(asks) == len(set(asks)), (name, asks)
        for claim, held, why in answered:
            assert why, (name, claim.asks, why)
            assert held in (True, False), (name, claim.asks, held)
        missed = tuple(ask for ask, (_, held, _) in zip(asks, answered) if not held)
        assert missed == CLAIM_MISSES[name], (name, missed, CLAIM_MISSES[name])
        assert trip.sim.dropped_steps == 0, (name, trip.sim.dropped_steps)
    sys.stdout.flush()
    return rows


def say_the_finding(stream, rows):
    """What a person's hands cost the script's own claims, in the claims' words.

    The count is the finding and the claims are the whole of it: the claims are the
    missions' own, this file does not edit them, and what it reports is which of
    them a hand at the instrument's own clock meets - which is the question
    ``TODO.md``'s last bullet asks, and a different one from phase 4's "was the
    mission flown", since a flight can be flown and still miss the claim, and a
    claim can be missed while the mission is flown.
    """
    say(stream, "")
    say(stream, "the finding, in the missions' own claims:")
    total = 0
    for name, hands, trip, answered in rows:
        missed = [claim.asks for claim, held, _ in answered if not held]
        total += len(answered)
        met = len(answered) - len(missed)
        if not missed:
            say(stream, "  the %s: every one of its %d claims met" % (name, met))
        else:
            say(stream, "  the %s: %d of its %d claims met, and the %d a hand does"
                " not meet %s %s"
                % (name, met, len(answered), len(missed),
                   "is" if len(missed) == 1 else "are", "; ".join(missed)))
        if name == "hover-turn":
            say(stream, "    and the one claim of the mission's own that a hand does"
                " not meet is the *arrival*: %s m/s against the %.2f the mission's"
                " own rehearsal asks of itself - which is phase 4's own finding, an"
                " arrival being where a hand shows, read here against the claim that"
                " mission makes of itself rather than against a verdict."
                % (words(trip.touchdown_rate, "%5.2f"),
                   HOVER_TOUCHDOWN_LIMIT_MPS))
    say(stream, "  %d claims in all, and the ruler, the clock and the frozen cases"
        " above are what say the flights they were answered by are this"
        " instrument's own." % (total,))
    sys.stdout.flush()
    return rows


# ---------------------------------------------------------------------------
# The run: four loops, two conditions, and the script's own claims.
# ---------------------------------------------------------------------------


def main():
    """The outer loops' bands, the ruler, and the script's claims read by hand."""
    transcript = Transcript(LOG_FILE)
    saved = sys.stdout
    sys.stdout = transcript
    try:
        # -- what the instrument is ----------------------------------------
        print("the pilot: %0.0f decisions a second on the flight's own clock,"
              " %.2f s of transport delay (%d frames), a %.2f s lag in the hand,"
              " and the four loops *above* the attitude under scales of their own"
              % (PILOT_HZ, DELAY_S, DELAY_FRAMES, T_LAG_S))
        print("  the model's own pilot is this one at a scale of one on all four:"
              " %0.2f deg of tilt per m off the pad and %0.2f per m/s of drift,"
              " %0.3f m/s of climb rate per m of height, %0.3f deg of nose per m/s"
              " of speed, and %0.3f deg of bank per m off the line"
              % (K_HOVER_POS_DEG_M, K_HOVER_VEL_DEG_MPS, RATE_GAIN_MPS,
                 K_SPEED_DEG, K_BANK_DEG))
        for axis in AXES:
            print("  %s: %s" % (axis, LOOP_WORDS[axis]))
            print("    %s put in, back inside %0.2f %s, and one loop scaled: %s"
                  % (PERTURB_WORDS[axis] % (PERTURB[axis],), SETTLE[axis],
                     READINGS[axis].unit, loop_words(axis)))
        for condition in (HOVER, CRUISE):
            print("  %s: %-56s -> the loops %s"
                  % (condition, CONDITION_WORDS[condition],
                     ", ".join("%s, %s" % (axis, gain_words(axis, OUTER_ONE))
                               for axis, one in CASES if one == condition)))
        # The one baseline in this file that is not read off the aircraft: the
        # trim table the mission's own pilot is written around, beside the trim
        # the model solves at the same speed.  They are not the same row, and the
        # difference is why this file writes its cruise loops around the model's.
        trims = trims_for(airframe_preset(AIRFRAME_PRESET), HumanPilot)
        sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
        sim.trim_level_flight(CONDITION_MPS[CRUISE],
                              altitude=CONDITION_ALT_M[CRUISE])
        solved = (sim.trim_controls.collective, sim.trim_controls.long_stick,
                  sim.trim_controls.lat_stick, sim.trim_controls.pedal,
                  sim.trim_state.attitude_deg.y)
        print("")
        print("the cruise loops' baseline, which is the one number in this file"
              " that is read off a table and not off the aircraft: the mission's"
              " own trim table stops at %.0f m/s and extrapolates to the cruise,"
              " while the model's own trim at %.2f m/s is solved here."
              % (max(trims.speeds), CONDITION_MPS[CRUISE]))
        for what, row in (("the mission's own table", trims.at(CONDITION_MPS[CRUISE])),
                          ("the model's own trim", solved)):
            print("  %-24s collective %5.2f in, long %+5.2f, lat %+5.2f, pedal"
                  " %+5.2f, pitch %+5.2f deg"
                  % (what, row[0], row[1], row[2], row[3], row[4]))
        clock()

        # -- the nominal case of each loop, with hands on --------------------
        print("")
        print("=== the model's own pilot, one loop at a time: a perturbation it did"
              " not ask for ===")
        nominal = {}
        for axis, condition in CASES:
            nominal[(axis, condition)] = case(
                axis, condition,
                label="the model's own %s pilot"
                      % ("hover" if condition == HOVER else "cruise"))
        print("")
        print("=== and what is left of each of them once it oscillates ===")
        for axis, condition in CASES:
            pio_verdict(SCALE_NOMINAL, nominal[(axis, condition)])

        # -- the ruler: phase 2's own case, re-flown through this pilot ------
        ruler(nominal)

        # -- the four loops with nobody flying them ---------------------------
        print("")
        print("=== the four loops with nobody flying them: the model's own answer"
              " ===")
        frozen = {}
        for axis, condition in CASES:
            print("--- %s at %s, the sticks where the trim left them"
                  % (PERTURB_WORDS[axis] % (PERTURB[axis],),
                     CONDITION_WORDS[condition]))
            sim, hands, trace = frozen_case(axis, condition)
            frozen[(axis, condition)] = trace
            inside = trace.settled()
            print("  %s" % (trace,))
            print("  %0.1f s with no hands on it: it ended %+0.2f %s off, and %s"
                  % (sim.sim_time, trace.errors[-1], trace.unit,
                     "the %0.2f %s band was held for two seconds at %s on the way"
                     % (trace.band, trace.unit, words(inside, "%5.2f s"))
                     if inside is not None else
                     "the %0.2f %s band was never held at all"
                     % (trace.band, trace.unit)))
            assert not sim.dropped_steps, (axis, condition, sim.dropped_steps)
            sys.stdout.flush()

        # -- the four bands ---------------------------------------------------
        print("")
        print("=== the bands: one loop at a time, from the model's own gains ===")
        bands = {}
        for axis, condition in CASES:
            bands[(axis, condition)] = band(axis, condition)

        # -- and what each of them is, in the loop's own words ----------------
        print("")
        print("=== the verdict on each loop ===")
        for axis, condition in CASES:
            verdict(axis, condition, nominal[(axis, condition)],
                    bands[(axis, condition)], frozen[(axis, condition)])

        # -- and the whole of it, one line a loop -----------------------------
        print("")
        print("=== the four loops in one line each: the model's own grip, the band,"
              " and both edges ===")
        for axis, condition in CASES:
            margin, pio_at, floor, harder, softer = bands[(axis, condition)]
            held = sorted(scale for scale, ok, _ in harder + softer if ok)
            model = nominal[(axis, condition)]
            print("  %-7s %-6s the model's own grip %s"
                  % (axis, condition,
                     "holds it, in %s" % words(model.settled(), "%5.2f s")
                     if holds(model.sim, model) else
                     "does NOT hold it: %0.1f times its middle third at the end"
                     % (model.tail_growth() or float("inf"),)))
            print("        the band: %s"
                  % ("nothing in either leg held" if not held else
                     "the grips that held, %s, of the model's own %s"
                     % (", ".join("x%.2f" % (one,) for one in held),
                        loop_words(axis)),))
            print("        hard edge: %s | soft edge: %s"
                  % ("none - every grip in that leg held"
                     if margin is None else
                     "x%.2f, where %s" % (margin, why_not(trace_at(harder, margin))),
                     "none - every grip in that leg held"
                     if floor is None else
                     "x%.2f, where %s" % (floor, why_not(trace_at(softer, floor)))))
            if pio_at is not None:
                _, period, growth = trace_at(harder, pio_at).pio()
                print("        and the pilot's own loop closes at x%.2f: %.2f s a"
                      " cycle, growing %.2f a cycle, with the %.2f s of delay worth"
                      " %.0f deg of it"
                      % (pio_at, period, growth, DELAY_S, Trace.phase_deg(period)))
            sys.stdout.flush()

        # -- the second half: the script's own claims, read by hand -----------
        rows = the_claims(transcript)
        say_the_finding(transcript, rows)
        check_claims(rows)

        # -- and what all of it is held to ------------------------------------
        # The pilot's own clock and delay were measured *in flight* by
        # :func:`clock`, and the ruler was checked where it was flown: both raise
        # rather than printing, and nothing below is worth reading if either did
        # not pass.  Everything from here is this file's own claim, and a claim
        # whose numbers drift has to be made again.
        #
        # Two of the five nominal settling times are phase 2's own pinned numbers -
        # the height loop at a scale of one *is* that file's heave case, in both
        # conditions - so a drift in those two is a drift in the instrument and not
        # a difference of opinion.  The other three are this file's, and they are
        # the three the transcript above printed.
        for key in CASES:
            axis, condition = key
            trace = nominal[key]
            assert holds(trace.sim, trace) is NOMINAL_HELD[key], \
                (key, holds(trace.sim, trace), trace.settled(),
                 trace.tail_growth(), why_not(trace))
            # The tail, which is a reading of every loop and not the criterion:
            # it says how much of a swing is still to be taken out.
            assert abs(trace.tail_growth() - NOMINAL_TAIL[key]) <= PIO_TOLERANCE, \
                (key, trace.tail_growth(), NOMINAL_TAIL[key])
            if NOMINAL_HELD[key]:
                assert trace.settled() is not None, key
                assert abs(trace.settled() - NOMINAL_SETTLED_S[key]) \
                    <= SETTLE_TOLERANCE_S, \
                    (key, trace.settled(), NOMINAL_SETTLED_S[key])
            else:
                # A loop whose own grip does not hold is pinned as the *not held*
                # it is - and with no settling time, since the number ``settled``
                # returns for one is the instant the error crossed the band on its
                # way somewhere else.
                assert NOMINAL_SETTLED_S[key] is None, (key, NOMINAL_SETTLED_S[key])
                assert why_not(trace) != "", key
            assert not trace.pio()[0], (key, trace.pio())
            assert not trace.sim.crashed, (key, trace.sim.crash_reason)
            if condition == HOVER:
                # The hover being *held* while the loop is argued about: the drift
                # the short mission's own check allows itself over a whole minute of
                # flying, and the height band it holds its own hover in.  The one
                # loop whose perturbation *is* the height is left alone: the worst
                # its height got is where the case began.
                assert trace.drift <= DRIFT_LIMIT_M, (key, trace.drift)
                if axis != HEIGHT:
                    assert trace.altitude_error() <= HEIGHT_LIMIT_M, \
                        (key, trace.altitude_error())
        # Each loop's band, as the two legs measured it: the grips that held, the
        # scale each leg first failed at, and the grip the pilot's own loop closes
        # at, with all five pinned in :data:`BANDS` and the oscillations in
        # :data:`PIO_MEASURED`.  A band whose edge is ``None`` is a leg in which
        # nothing failed, which is an answer and is pinned as one.
        for key in CASES:
            margin, pio_at, floor, harder, softer = bands[key]
            want_held, want_margin, want_floor, want_pio = BANDS[key]
            held = tuple(sorted(scale for scale, ok, _ in harder + softer if ok))
            failed = sorted(scale for scale, ok, _ in harder + softer if not ok)
            assert held == want_held, (key, held, want_held)
            assert margin == want_margin, (key, margin, want_margin)
            assert floor == want_floor, (key, floor, want_floor)
            assert pio_at == want_pio, (key, pio_at, want_pio)
            # Both legs walked *outward* from the model's own gain, in order, and
            # stopped somewhere - which is what says a band was searched for and
            # not filled in, and that the nominal case is not hiding in a ladder.
            assert [scale for scale, _, _ in harder] \
                == list(SCALES_HARDER[:len(harder)]), (key, harder)
            assert [scale for scale, _, _ in softer] \
                == list(SCALES_SOFTER[:len(softer)]), (key, softer)
            if margin is not None:
                assert margin in failed, (key, margin, failed)
            if floor is not None:
                assert floor in failed, (key, floor, failed)
            # And the failure that *is* the pilot's, where there is one: the period
            # and the growth this file pinned, at a period a person's own loop lives
            # in - which is what makes it the pilot's loop and not an aircraft mode,
            # and is the check the PIO band above rests on.
            if pio_at is not None:
                _, period, growth = trace_at(harder, pio_at).pio()
                want_period, want_growth = PIO_MEASURED[key]
                assert abs(period - want_period) <= PIO_TOLERANCE, \
                    (key, period, want_period)
                assert abs(growth - want_growth) <= PIO_TOLERANCE, \
                    (key, growth, want_growth)
                assert PIO_PERIOD_S[0] <= period <= PIO_PERIOD_S[1], (key, period)
        # And what each loop does with nobody flying it: the shape and the limit
        # pinned in :data:`FROZEN`, which is the reference every band above is read
        # against - a loop that holds nothing on its own is a loop whose whole
        # margin is the pilot's.
        for key in CASES:
            trace = frozen[key]
            shape, limit = FROZEN[key]
            assert not trace.sim.dropped_steps, (key, trace.sim.dropped_steps)
            if shape == "out":
                assert not stays(trace), (key, shape, trace.settled())
                assert abs(trace.errors[-1]) >= limit, \
                    (key, trace.errors[-1], limit)
            else:
                assert stays(trace), (key, shape, trace.settled())
                assert abs(trace.errors[-1]) <= limit, \
                    (key, trace.errors[-1], limit)

        print("")
        print("check passed: the instrument measured on the one pilot in this"
              " programme that is not phase 1's hover pilot slowed down - every view"
              " %.3f s old and no frame dropped - the height loop held to phase 2's"
              " own pinned case in both conditions, the nominal case, the band, the"
              " frozen case and the PIO of each of the four loops pinned as printed"
              " above, and the script's own claims read one at a time against a"
              " person's flight of each mission, with the claims a hand does not meet"
              " pinned rather than loosened." % (DELAY_S,))
        sys.stdout.flush()
    finally:
        sys.stdout = saved
        transcript.close()
    return nominal, bands, rows


def demo():
    """The command line: there is nothing to say, so the whole programme is flown.

    The two halves are run in one go because the second is read against the first:
    what a hand's flight of a mission is worth is a question about the same
    instrument whose outer loops were measured a section above it.
    """
    main()
    print("wrote %s" % (LOG_FILE,))
    return 0


if __name__ == "__main__":
    sys.exit(demo())
