# -*- coding: utf-8 -*-
"""Scratch: a person's hands on the *other* three axes, and again at sixty knots.

Phase 1 - ``_rt_handling.py`` - asked whether a person's hands can hold this
model's hover at all, and answered it with the two *tilt* axes.  What that file
really is, though, is an *instrument*: a clock, a transport delay, a gain, a lead
and a lag, closed around ``_rt_pilot.HoverTurnPilot``'s own hover loops, with the
two attitude gains multiplied by a scale.  And it said of itself, exactly, what it
was not:

    everything below the attitude loop stays at the model's own numbers, since
    the question here is how hard a pilot may grip the *attitude* and not how
    fast the outer loops may be.

This file is the rest of that sentence.  The instrument is unchanged - the same
twenty decisions a second on the flight's own clock, the same 0.2 s of delay read
off a deque of remembered states, the same 0.1 s of lag in the hand, and the tilt
loops still gain for gain phase 1's - and two more of the pilot's loops are put
under scales of their own:

* **the lever.**  The heave loop is ``height error -> rate asked for -> stick``,
  and the gain of it - inches of collective per m/s of climb rate the loop is
  short of - is ``_rt_pilot.K_RATE_IN``.  A scale of one is the model's own
  pilot; a scale of two is that pilot gripping the lever twice as hard.  It has
  no lead to keep, because it *is* a rate loop: the only dynamic term above it is
  the ``RATE_GAIN_MPS`` the outer height loop asks its rate with, and that stays
  the model's own, for phase 1's own reason;
* **the pedals.**  A hover's heading hold is the model's turn loop with the
  turn's schedule taken out of it: inches of pedal per degree of nose, and per
  degree a second of yaw rate - which is a *lead* of ``K_TURN_RATE_IN / K_TURN_IN``
  seconds, kept at the model's own ratio while the pair is scaled, which is phase
  1's own rule for the cyclic.

What is put into each axis is the state a pilot would have to take back out of it:
two degrees of attitude for the tilt axes (phase 1's own perturbation), a metre of
height for the lever, and ten degrees of nose for the pedals - each read against a
band of its own, a *fraction* of what was put in, which is what makes "settled"
mean the same thing on all four.

Three measurements, one per question:

* **the nominal case of each axis.**  The model's own pilot, at a scale of one on
  every loop, against a perturbation it did not ask for: the worst it reached, how
  long the settling took, and what the *rest* of the hover cost while it did it -
  the height band and the pad.  The tilt rows are phase 1's own numbers re-measured
  through this file's instrument, and they are *held* to phase 1's numbers, which is
  the check that the two files are the same instrument;
* **the band of each axis.**  The same perturbation with that axis's gain scaled up
  and down from the model's own until a grip fails, which is what a *margin* is -
  and it is a band and not a number, because the two edges fail differently: a grip
  too hard closes the pilot's own loop on the pilot's own delay, and one too soft
  cannot hold what the outer loops are asking for;
* **the PIO.**  Whether a failure is the pilot's own loop: an oscillation at a
  period a person's loop lives in, growing a cycle at a time, where the ``DELAY_S``
  of delay is worth tens of degrees of phase - which no aircraft mode can be.

And then the check, which is the reason to do the other three axes at all: **the
same four axes, the same perturbations and the same scales at sixty knots**, where
the aircraft is a different machine - at zero airspeed it has no damping at all and
its rotor is working in its own downwash, and at sixty it is flying through an
airstream with a fin behind it.  A margin read in the hover is only worth having if
the hover is the *hard* condition, so the two scales each axis fails at in the
hover are re-flown at sixty knots beside its nominal case.  See the transcript for
which of them hold.

The transcript is ``_rt_axes.txt``, written by the run itself; it is ignored by
``.gitignore``'s ``_rt_*.txt`` beside the other harnesses' own.  The run asserts
what it prints, so a regression is a traceback and not a table nobody read.
"""


import math
import sys
from collections import namedtuple

from _rt_handling import (DELAY_FRAMES, DELAY_S, DRIFT_LIMIT_M, EVERY,
                          FLOOR_FAILS_AT, FROZEN_S, HEIGHT_LIMIT_M, HOLD_S,
                          K_THETA_IN, MARGIN_FAILS_AT, MIN_PEAK_GAP_S,
                          NOMINAL_DRIFT_M, NOMINAL_FLOOR_SCALE,
                          NOMINAL_HEIGHT_M, NOMINAL_MARGIN_SCALE,
                          NOMINAL_PITCH_SETTLED_S, NOMINAL_ROLL_SETTLED_S,
                          OVERSHOOT_AFTER_S, PERTURB_DEG, PILOT_HZ,
                          PIO_AT_SCALE, PIO_GROWTH, PIO_MEASURED_GROWTH,
                          PIO_MEASURED_PERIOD_S, PIO_PERIOD_S, SCALE_NOMINAL,
                          SCALES_HARDER, SCALES_SOFTER, SETTLE_DEG, SETTLE_S,
                          SETTLE_TOLERANCE_S, T_LEAD_S, T_LAG_S, Transcript)
from _rt_handling import Hands as PhaseOneHands
from _rt_handling import Trace as PhaseOneTrace
from _rt_pilot import (ALT_REF_M, CLIMB_LIMIT_MPS, HOVER_ALT_M,
                       HOVER_REF_EAST_M, HOVER_REF_NORTH_M,
                       HOVER_TILT_LIMIT_DEG, K_HOVER_PITCH_IN,
                       K_HOVER_POS_DEG_M, K_HOVER_RATE_IN, K_HOVER_ROLL_IN,
                       K_HOVER_VEL_DEG_MPS, K_RATE_IN, K_TURN_IN,
                       K_TURN_RATE_IN, RATE_GAIN_MPS, clip, inches_to_axes,
                       skid_ft, wrap)
from airframe import KNOT, Vector3
from main import AIRFRAME_PRESET
from simulation import SIM_TIME_STEP_S, Simulation, airframe_preset

#: The transcript, written by the run itself: see the module docstring.
LOG_FILE = "_rt_axes.txt"


# ---------------------------------------------------------------------------
# The four axes, what is put into each, and what counts as taking it out.
# ---------------------------------------------------------------------------

#: The two axes phase 1 flew, and the two it did not.  All four are flown here, so
#: that one file holds the whole of the pilot's margin - and the two phase 1
#: measured are the check on the two it did not, since this file's instrument
#: either reproduces that file's numbers or it is not the same instrument.
AXES_TILT = ("pitch", "roll")
AXES_NEW = ("heave", "yaw")
AXES = AXES_TILT + AXES_NEW

#: A metre of height put into the hover: three feet of the twenty it sits at,
#: which is what a gust or a moment's inattention costs a pilot holding station.
#: Small enough to be a perturbation and large enough to be a problem - see the
#: hands-off case, which is where the same metre goes with nobody flying.
PERTURB_HEAVE_M = 1.0

#: Ten degrees of nose: what a yaw gust, or a bootful of pedal taken off
#: carelessly, leaves a hover on.  It is not the tilt pair's two degrees made up
#: into a bigger number - a hover has no directional stability at all to help with
#: it, so a heading is lost and kept in tens of degrees or not at all.
PERTURB_YAW_DEG = 10.0

#: What is put into each axis, in that axis's own units: phase 1's own two degrees
#: for the tilt pair, and the two above for the axes it left alone.
PERTURB = {"pitch": PERTURB_DEG, "roll": PERTURB_DEG,
           "heave": PERTURB_HEAVE_M, "yaw": PERTURB_YAW_DEG}

#: The band each axis counts as *back inside*, in its own units, and so what
#: settling means on it: the tilt pair is phase 1's own half degree, a quarter of
#: the two that were put in, and the two new ones are the same fraction of their
#: own perturbation - fifteen centimetres of a metre of height, and a degree and a
#: half of ten degrees of nose.  A band in the axis's own units is the whole reason
#: this file is not one table of degrees.
SETTLE_HEAVE_M = 0.15
SETTLE_YAW_DEG = 1.5
SETTLE = {"pitch": SETTLE_DEG, "roll": SETTLE_DEG,
          "heave": SETTLE_HEAVE_M, "yaw": SETTLE_YAW_DEG}

#: What the nose keeps of a heading when nobody is flying it: all but a tenth
#: of a degree of the ten it was given.  The hover ends on 9.9996 of its ten,
#: and sixty knots on the same, so this is the one hands-off answer the two
#: conditions share - the heading hold's own doing and not the pedals' - and
#: what "never held the band" means on an axis whose band is a degree and a
#: half wide.
KEPT_YAW_DEG = PERTURB_YAW_DEG - 0.1


# ---------------------------------------------------------------------------
# The two loops phase 1 left at the model's own numbers.
# ---------------------------------------------------------------------------

#: The lever's gain, in inches of collective per m/s of climb rate the loop is
#: short of, and the pedals', in inches per degree of nose - read off ``_rt_pilot``
#: rather than written down again, the way phase 1 read its two attitude gains off
#: it, so that a scale of one *is* the model's own hover pilot on every loop.
K_LEVER_IN = K_RATE_IN
K_PEDAL_IN = K_TURN_IN

#: And the lead the pedals are flown with, s, which is the ratio of that module's
#: own two pedal gains - a hover's heading hold is its turn loop with the turn's
#: schedule taken out of it, so the lead is that pilot's and not this file's.
#: :meth:`Hands.selftest` asserts the ratio rather than trusting it, so a pedal
#: gain moved in ``_rt_pilot`` moves the lead here with it.  The lever has none: an
#: inches-per-(m/s) loop is already closed on a rate, and the only dynamic term
#: above it is the outer height loop's ``RATE_GAIN_MPS``, which stays the model's
#: own for phase 1's own reason - the question is how hard a pilot may grip *this*
#: loop and not how fast the loops around it may be.
T_PEDAL_LEAD_S = K_TURN_RATE_IN / K_TURN_IN


# ---------------------------------------------------------------------------
# The two conditions, and what the check on the second one is.
# ---------------------------------------------------------------------------

#: The hover is phase 1's own condition: the model's twenty foot trim, held over
#: the pad, with the station-keeping tilt loop working underneath the attitude
#: loop exactly as ``_rt_pilot``'s hover pilot flies it.  Sixty knots is the
#: report's own validation condition, flown at the long mission's own cruise height
#: - and it is the *check* on the hover's bands rather than a second instrument:
#: the same perturbation into the same loop, on an aircraft that is flying.
HOVER = "hover"
CRUISE = "cruise"
CONDITIONS = (HOVER, CRUISE)

#: The speed and height each condition is trimmed to: zero and twenty feet of
#: skids for the hover, which is ``_rt_pilot``'s own number for both, and sixty
#: knots at a hundred metres for the cruise.
CRUISE_KT = 60.0
CRUISE_MPS = CRUISE_KT * KNOT
CRUISE_ALT_M = ALT_REF_M

CONDITION_MPS = {HOVER: 0.0, CRUISE: CRUISE_MPS}
CONDITION_ALT_M = {HOVER: HOVER_ALT_M, CRUISE: CRUISE_ALT_M}
CONDITION_WORDS = {HOVER: "the twenty foot hover over the pad",
                   CRUISE: "level flight at sixty knots, %.0f m up"
                           % (CRUISE_ALT_M,)}

#: What a case's *speed* is held against at sixty knots: the trim's own airspeed,
#: read off the aircraft rather than written down, and the band that is allowed to
#: wander in while the attitude is being argued about.  It is a *check* on the
#: cruise cases and not a criterion - a pilot holding an attitude at the trim's own
#: attitude is holding the speed the trim was solved for, since the model is
#: speed-stable at sixty knots - so the number is a limit on the excursion and not
#: a settling of its own.
CRUISE_SPEED_LIMIT_MPS = 2.5


# ---------------------------------------------------------------------------
# What each axis is read off, and the two loops this file puts under a scale.
# ---------------------------------------------------------------------------

#: One axis as a *reading*: which of the four inches of stick the hand is holding
#: while it is measured, the unit the error is in, the error itself against the
#: trim's own quantity for that axis, and the rate that axis's inner loop is closed
#: on.  The error is a function of the pilot *and* the instruments rather than a
#: field, because the four quantities it is measured from - two attitudes, a height
#: and a heading - are not attributes of one object.
Reading = namedtuple("Reading", "stick unit error rate")


def pitch_error(hands, telemetry):
    """Degrees of pitch, against the attitude the trim was on."""
    return telemetry.pitch_deg - hands.pitch_ref


def roll_error(hands, telemetry):
    """Degrees of roll, against the attitude the trim was on."""
    return telemetry.roll_deg - hands.roll_ref


def heave_error(hands, telemetry):
    """Metres of height, against the height the trim was on.

    Positive is *high*: the aircraft is put a metre above its trim and the pilot's
    job is to take it back down, which mirrors the tilt axes, where what is put in
    is an attitude the pilot has to take back out.  ``alt_agl`` is the c.g. above
    the ground and ``alt_ref`` is the same quantity as the trim left it, so both
    ends of the difference are read off the same instrument.
    """
    return telemetry.alt_agl - hands.alt_ref


def yaw_error(hands, telemetry):
    """Degrees of nose, against the heading the trim was on, wrapped.

    Wrapped because a heading that has gone right about is a heading that is a
    little off and not one that is three hundred and fifty degrees off - the same
    reading ``_rt_pilot``'s own heading hold is written around.
    """
    return wrap(telemetry.yaw_deg - hands.heading_ref)


READINGS = {
    "pitch": Reading(1, "deg", pitch_error,
                     lambda telemetry: telemetry.pitch_rate_deg),
    "roll": Reading(2, "deg", roll_error,
                    lambda telemetry: telemetry.roll_rate_deg),
    "heave": Reading(0, "m", heave_error,
                     lambda telemetry: telemetry.height_rate),
    "yaw": Reading(3, "deg", yaw_error,
                   lambda telemetry: telemetry.yaw_rate_deg),
}


def error_of(axis, hands, telemetry):
    """The axis's error, in that axis's own units: see :data:`READINGS`."""
    return READINGS[axis].error(hands, telemetry)


def reference_of(hands, axis):
    """What the axis is measured *against*: the quantity the trim left it on.

    Degrees of attitude for the tilt pair, metres of height for the lever and
    degrees of nose for the pedals - the four quantities a pilot's own trim is a
    statement about, read off the aircraft rather than written down here.
    """
    if axis == "pitch":
        return hands.pitch_ref
    if axis == "roll":
        return hands.roll_ref
    if axis == "heave":
        return hands.alt_ref
    return hands.heading_ref


#: What each of the pilot's three gains is multiplied by, and so what a case is
#: *flown* at: the two attitude gains phase 1 swept, the lever's gain, and the
#: pedals' pair - which move together, so the lead between them is the model's own
#: at every scale.  One axis is scaled at a time and the other two stay at the
#: model's own numbers, which is what makes each band the band of *that* axis.
Scales = namedtuple("Scales", "theta lever pedal")

#: Every loop at the model's own gain: the pilot ``_rt_check`` flies the short
#: mission with, and - on the tilt pair's two gains - the nominal case of phase 1.
SCALES_ONE = Scales(SCALE_NOMINAL, SCALE_NOMINAL, SCALE_NOMINAL)


def scales_for(axis, scale):
    """The three scales a case at *scale* of *axis* is flown at.

    A tilt axis puts the scale on both of the cyclic's gains, which is phase 1's
    own pair-moving; the heave puts it on the lever's, the yaw on the pedals', and
    each leaves the other two loops at the model's own numbers.  So a scale of two
    is the model's hover pilot gripping one control twice as hard, and not a pilot
    gripping everything at once - which is the only way a *margin* can be read off
    one axis and quoted against it.
    """
    if axis == "heave":
        return Scales(SCALE_NOMINAL, scale, SCALE_NOMINAL)
    if axis == "yaw":
        return Scales(SCALE_NOMINAL, SCALE_NOMINAL, scale)
    return Scales(scale, SCALE_NOMINAL, SCALE_NOMINAL)


# ---------------------------------------------------------------------------
# The pilot: phase 1's instrument, with two more knobs on it.
# ---------------------------------------------------------------------------


class Hands(PhaseOneHands):
    """A person's hands on four axes, with all three of the loops scaled.

    Phase 1's own :class:`_rt_handling.Hands` to the last line - the twenty hertz
    clock on the flight's own time, the ``DELAY_S`` of delay read off a deque of
    remembered states, the 0.1 s of lag in the hand and the limb's own rate limit
    underneath it, and :meth:`fill`, :meth:`view`, :meth:`push` and :meth:`frame`
    exactly as they were - with the three loops of it under scales of their own.
    Phase 1's ``scale`` is this class's ``scales.theta``; this class adds the
    lever's and the pedals'.

    What it does *not* do is move the loops *around* those three: the
    station-keeping tilt a hover is held with is the model's own, and the height
    the lever's loop is asked for is still asked for through the model's
    ``RATE_GAIN_MPS`` and its three metre a second cap.  A pilot may grip the lever
    harder than the model does; a pilot may not fly a different heave loop.
    """

    def __init__(self, sim, trim, scales=SCALES_ONE, condition=HOVER, **kwargs):
        PhaseOneHands.__init__(self, sim, trim, scale=scales.theta, **kwargs)
        #: What each of the three loops is multiplied by, and the two gains those
        #: scales work out to.  The tilt pair's is on the base class, as its own
        #: ``k_pitch`` and ``k_roll``, because phase 1 scales the two together.
        self.scales = Scales(float(scales.theta), float(scales.lever),
                             float(scales.pedal))
        self.k_lever = K_LEVER_IN * self.scales.lever
        self.k_pedal = K_PEDAL_IN * self.scales.pedal
        self.t_pedal_lead = T_PEDAL_LEAD_S
        #: What is being flown, and so whether the tilt loop underneath the
        #: attitude loop is flying at all: a hover is held over a point on the
        #: ground and sixty knots is not - see the module's conditions.
        self.condition = condition
        self.station = (condition == HOVER)
        #: The speed the trim was solved for, m/s, read off the aircraft rather
        #: than written down: what a cruise case's speed excursion is measured
        #: against, and no more than a curiosity in a hover.
        self.speed_ref = float(sim.telemetry().airspeed)

    def selftest(self):
        """Prove the model's own hover pilot *is* this pilot at a scale of one.

        Phase 1's own check first, which is that its attitude gain and its lead are
        ``_rt_pilot``'s numbers and that its delay is the fifth of a second the
        clock claims - and then the two loops this file adds: the lever's gain is
        that module's own, and the pedals' lead is the *ratio* of its two pedal
        gains, so a pedal gain moved in ``_rt_pilot`` moves the lead here with it
        instead of leaving this file flying a loop no pilot ever had.
        """
        PhaseOneHands.selftest(self)
        assert abs(K_PEDAL_IN * T_PEDAL_LEAD_S - K_TURN_RATE_IN) < 1e-12, \
            (K_PEDAL_IN, T_PEDAL_LEAD_S, K_TURN_RATE_IN)
        assert abs(K_LEVER_IN - K_RATE_IN) < 1e-12, (K_LEVER_IN, K_RATE_IN)
        assert abs(self.k_lever - K_RATE_IN * self.scales.lever) < 1e-12, \
            (self.k_lever, K_RATE_IN, self.scales)
        assert abs(self.k_pedal - K_TURN_IN * self.scales.pedal) < 1e-12, \
            (self.k_pedal, K_TURN_IN, self.scales)
        # And the two the base class scales: they are ``_rt_pilot``'s own pair, so a
        # scale of one on them is that module's hover pilot exactly - which is what
        # phase 1 asserted and what this file's two tilt rows are held to below.
        assert abs(K_THETA_IN - K_HOVER_PITCH_IN) < 1e-12, \
            (K_THETA_IN, K_HOVER_PITCH_IN)
        assert abs(K_THETA_IN * T_LEAD_S - K_HOVER_RATE_IN) < 1e-12, \
            (K_THETA_IN, T_LEAD_S, K_HOVER_RATE_IN)
        assert abs(self.k_roll - K_HOVER_ROLL_IN * self.scales.theta) < 1e-12, \
            (self.k_roll, K_HOVER_ROLL_IN, self.scales)
        assert self.condition in CONDITIONS, self.condition
        return self

    def decide(self, view):
        """The four sticks wanted, in inches, from what the instruments said.

        ``_rt_pilot.HoverTurnPilot.decide``'s hover, loop for loop and gain for
        gain, exactly as phase 1's own ``Hands.decide`` is - and with the two loops
        phase 1 left at the model's own gains under this pilot's scales.  See that
        module for why each loop has the shape it has; what is worth saying here is
        only what is different, and it is three things:

        * the lever is ``height error -> rate asked for -> stick``, and the gain of
          it is this pilot's rather than the module's.  There is no lead to scale
          with it: an inches-per-(m/s) loop is closed on a rate already, and the
          only term above it is the outer loop's own ``RATE_GAIN_MPS``;
        * the pedals are that module's *turn* loop with the turn taken out of it -
          the heading the trim was on, held, and the rate it is turning at held
          out - which is a gain and a lead, and the lead is the ratio of that
          module's two pedal gains;
        * and over the pad, the tilt the hover is held with comes out of the
          aircraft's own position and velocity, as the module flies it.  At sixty
          knots there is no pad to hold station over, so the attitude asked for is
          the trim's own - which is the attitude the trim was solved *for*, and so
          the one that holds the speed.  What holds a cruising aircraft over the
          ground is the aircraft.
        """
        telemetry = view.telemetry
        collective_t, long_t, lat_t, pedal_t = self.trim[:4]

        # Vertical: the height the trim sits at, and the rate to hold it with.
        rate_want = clip(RATE_GAIN_MPS * (self.alt_ref - telemetry.alt_agl),
                         -CLIMB_LIMIT_MPS, CLIMB_LIMIT_MPS)
        collective = (collective_t
                      + self.k_lever * (rate_want - telemetry.height_rate))

        # The tilt that holds station over the pad, taken in the aircraft's own
        # axes - the nose is what turns, the pad does not - and answered with a
        # tilt towards the pad against the drift still to be taken out.  Phase 1's
        # own loop, with the model's own gains underneath this pilot's attitude
        # gain, which is what a hover is held with.
        if self.station:
            heading = math.radians(telemetry.yaw_deg)
            north_off = self.pad[0] - view.north
            east_off = self.pad[1] - view.east
            forward_off = (north_off * math.cos(heading)
                           + east_off * math.sin(heading))
            right_off = (-north_off * math.sin(heading)
                         + east_off * math.cos(heading))
            pitch_want = clip(self.pitch_ref - K_HOVER_POS_DEG_M * forward_off
                              + K_HOVER_VEL_DEG_MPS * view.vx,
                              self.pitch_ref - HOVER_TILT_LIMIT_DEG,
                              self.pitch_ref + HOVER_TILT_LIMIT_DEG)
            roll_want = clip(self.roll_ref + K_HOVER_POS_DEG_M * right_off
                             - K_HOVER_VEL_DEG_MPS * view.vy,
                             self.roll_ref - HOVER_TILT_LIMIT_DEG,
                             self.roll_ref + HOVER_TILT_LIMIT_DEG)
        else:
            pitch_want = self.pitch_ref
            roll_want = self.roll_ref

        # And the stick that attitude is held with: the trim's own, plus the
        # attitude error and the rate - this pilot's gain and its lead, in inches.
        long_stick = (long_t
                      + self.k_pitch * (telemetry.pitch_deg - pitch_want)
                      + self.k_pitch * self.t_lead * telemetry.pitch_rate_deg)
        lat_stick = (lat_t
                     + self.k_roll * (roll_want - telemetry.roll_deg)
                     - self.k_roll * self.t_lead * telemetry.roll_rate_deg)

        # The pedals: the heading the trim is on, and the rate it is turning at
        # held out, at this pilot's pedal gain and the module's own lead.
        pedal = (pedal_t
                 + self.k_pedal * wrap(self.heading_ref - telemetry.yaw_deg)
                 + self.k_pedal * self.t_pedal_lead
                 * (0.0 - telemetry.yaw_rate_deg))
        return [collective, long_stick, lat_stick, pedal]


# ---------------------------------------------------------------------------
# What a case measured, in the axis's own units.
# ---------------------------------------------------------------------------


class Trace(PhaseOneTrace):
    """One case of one axis, frame by frame, and the answers read out of it.

    Phase 1's own trace, and its same four answers: when the error came inside the
    band and *stayed* (``settled``), whether it is still growing at the end of the
    case (``tail_growth``), what is left of it once it oscillates - the period, the
    growth, and so whether it is a pilot induced oscillation - and what the *rest*
    of the aircraft did while this axis was being argued about.

    What it adds is the two axes phase 1 did not fly, each read off its own
    instrument and in its own unit, and the *speed* - which is what a sixty knot
    case is about: the loop above is the same loop, and what changes out there is
    that holding the trim's attitude is holding the speed.

    The unit is not decoration.  A trace of the heave is metres and m/s where the
    tilt pair's are degrees and deg/s, and a band of half a degree is not a band of
    half a metre: :attr:`band` carries the axis's own, and ``settled`` and
    ``peaks`` default to it, so that "settled" and "an oscillation" mean the same
    thing on four axes that are not in the same units.
    """

    def __init__(self, sim, hands, axis, seconds, condition=HOVER):
        PhaseOneTrace.__init__(self, sim, hands, axis, seconds)
        reading = READINGS[axis]
        #: The stick the hand is holding, the unit of the error, and what the axis
        #: is measured against.  Phase 1 set its own two from the axes it flew;
        #: there is no way to do that for four axes except by a table.
        self.reading = reading
        self.index = reading.stick
        self.unit = reading.unit
        self.reference = reference_of(hands, axis)
        self.band = SETTLE[axis]
        self.condition = condition
        #: The speed, against the one the trim was solved for.
        self.speed_ref = hands.speed_ref
        self.speeds = []
        self.speed_low = self.speed_ref
        self.speed_high = self.speed_ref

    def measure(self):
        """One frame's worth, from the state the frame loop has just left.

        Phase 1's own, for four axes: the error against the trim's own quantity,
        that axis's rate, the stick the hand has *moved* - the lagged command and
        not the one asked for - the height, how far the machine is from the pad,
        and the speed.
        """
        telemetry = self.sim.telemetry()
        state = self.sim.airframe.state
        drift = math.hypot(state.position.x, state.position.y)
        self.times.append(float(self.sim.sim_time))
        self.errors.append(error_of(self.axis, self.hands, telemetry))
        self.rates.append(self.reading.rate(telemetry))
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

    def settled(self, band=None, hold=SETTLE_S):
        """When the error came inside the band and stayed, s - this axis's band.

        Phase 1's own reading, with the band defaulted to the axis's own: half a
        degree of attitude, fifteen centimetres of height, a degree and a half of
        nose - each of them a quarter or so of what was put into that axis, which
        is what makes the four settling times comparable at all.
        """
        return PhaseOneTrace.settled(self, self.band if band is None else band,
                                     hold)

    def peaks(self, floor=None, gap=MIN_PEAK_GAP_S):
        """What is left of the error once it oscillates, in the axis's own units.

        Phase 1's own reading, with the floor defaulted to the axis's band: an
        error inside the band the axis is supposed to be in is not an oscillation,
        whatever units it is in.
        """
        return PhaseOneTrace.peaks(self, self.band if floor is None else floor, gap)

    def speed_error(self):
        """The worst the airspeed got either side of the trim's own, m/s.

        One number and not a settling: at sixty knots a pilot holding the trim's own
        attitude is holding the speed the trim was solved for, so what there is to
        report is how far the speed swung *while* the attitude came back.
        """
        if not self.speeds:
            return 0.0
        return max(abs(self.speed_high - self.speed_ref),
                   abs(self.speed_low - self.speed_ref))

    def rest_words(self):
        """What the rest of the machine did: the pad in a hover, the speed aloft."""
        if self.condition == HOVER:
            return "drift %4.2f m" % (self.drift,)
        return "speed error %4.2f m/s" % (self.speed_error(),)

    def __str__(self):
        overshoot, when = self.overshoot()
        period, growth = self.period(), self.growth()
        settled = self.settled()
        return ("%s: worst %+0.2f %s since %.2f s (at %5.2f s), settled %s |"
                " height %+0.2f m, %s | stick %0.2f in | period %s, growth %s"
                % (self.axis, overshoot, self.unit, OVERSHOOT_AFTER_S, when,
                   "%5.2f s" % settled if settled is not None else "never",
                   self.altitude_error(), self.rest_words(), self.stick_range(),
                   "%5.2f s" % period if period else "none",
                   "%4.2f" % growth if growth else "none"))


# ---------------------------------------------------------------------------
# A case: the model's own trim, a person's hands around it, and one axis off it.
# ---------------------------------------------------------------------------


def perturbed_state(state, axis, size):
    """*state*, copied, with *size* of *axis* put into it - in that axis's units.

    A state is *copied* before it is written into, and what is written into it is a
    new object: a position and an attitude are frozen triples in this model, so a
    perturbation is a whole new state rather than a field quietly changed under a
    running aircraft.  The three attitude axes get degrees and the lever gets
    metres; NED is +z down, so the metre of height the lever is given goes in as a
    *negative* step in z - the machine is put three feet above its own trim.
    """
    flown = state.copy()
    roll, pitch, yaw = flown.attitude.as_tuple()
    if axis == "pitch":
        flown.attitude = Vector3(roll, pitch + math.radians(size), yaw)
    elif axis == "roll":
        flown.attitude = Vector3(roll + math.radians(size), pitch, yaw)
    elif axis == "yaw":
        flown.attitude = Vector3(roll, pitch, yaw + math.radians(size))
    elif axis == "heave":
        position = flown.position
        flown.position = Vector3(position.x, position.y, position.z - size)
    else:
        raise ValueError("no such axis: %r" % (axis,))
    return flown


def case_setup(axis="pitch", scales=SCALES_ONE, condition=HOVER, size=None):
    """The model's own trim, a pilot built around it, and *axis* put off it.

    Phase 1's own :func:`_rt_handling.hover_setup`, for four axes and two
    conditions: the aircraft is trimmed - to the hover at :data:`HOVER_ALT_M`, or to
    level flight at sixty knots at :data:`CRUISE_ALT_M` - the pilot is built around
    *that* trim (its stick, its attitude, its height and its heading, all read off
    the aircraft rather than written down here) and handed the last fifth of a
    second of it, because the first decision of any case has to be made on the trim
    and not on the perturbation.  Only then is the perturbation put in.

    The stick is left where the trim left it, since nobody moves a control to a
    position it is already in, and both the *eyes* and the *aircraft* are checked
    against what this thinks it did: the pilot's oldest view is the trim to the last
    bit, and the error the perturbation is worth is the size that was asked for -
    which is the check that a metre of height is a metre of the *reading* the lever's
    loop is written around, and not a metre of something else.

    Returns ``(sim, trim, hands)`` with the perturbation already in the aircraft and
    neither of them flown yet.
    """
    size = PERTURB[axis] if size is None else float(size)
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.trim_level_flight(CONDITION_MPS[condition],
                          altitude=CONDITION_ALT_M[condition])
    # The point the pilot's station-keeping loop is handed is the aircraft's own
    # position - a hover is held over where the aircraft *is*, and a case that has
    # been trimmed is at the pad.  That is the same point ``_rt_pilot`` names as the
    # pad, and the assertion is what makes this file's hover the same hover phase 1
    # flew rather than a hover somewhere else.
    if condition == HOVER:
        assert abs(sim.trim_state.position.x - HOVER_REF_NORTH_M) < 1e-6, \
            sim.trim_state.position.x
        assert abs(sim.trim_state.position.y - HOVER_REF_EAST_M) < 1e-6, \
            sim.trim_state.position.y
    trim = (sim.trim_controls.collective, sim.trim_controls.long_stick,
            sim.trim_controls.lat_stick, sim.trim_controls.pedal)
    hands = Hands(sim, trim, scales=scales, condition=condition,
                  pad=(sim.trim_state.position.x, sim.trim_state.position.y))
    hands.selftest().fill(sim)
    # What the pilot can see before the case begins has to be the trim: the
    # perturbation goes in *after* the eyes are filled, so the first fifth of a
    # second of every case is flown on what the aircraft was and not what it is.
    eyes = hands.view()
    assert eyes is not None, "the history was not filled"
    assert abs(error_of(axis, hands, eyes.telemetry)) < 1e-9, \
        error_of(axis, hands, eyes.telemetry)
    sim.reset(state=perturbed_state(sim.trim_state, axis, size),
              controls=sim.trim_controls)
    assert abs(error_of(axis, hands, sim.telemetry()) - size) < 1e-9, \
        (axis, error_of(axis, hands, sim.telemetry()), size)
    assert all(abs(one - want) < 1e-12 for one, want
               in zip(sim.pilot.axes(), inches_to_axes(*trim))), sim.pilot.axes()
    return sim, trim, hands


def case_line(sim, trace):
    """One line of a case's progress, in the axis's own units."""
    position = sim.airframe.state.position
    return ("    t %4.1f s | %s %+6.2f %s %+7.2f %s/s | stick %+5.2f in |"
            " %5.2f ft skid | %5.2f m out"
            % (sim.sim_time, trace.axis, trace.errors[-1], trace.unit,
               trace.rates[-1], trace.unit, trace.sticks[-1],
               skid_ft(sim.telemetry()), math.hypot(position.x, position.y)))


def fly_case(axis="pitch", scales=SCALES_ONE, condition=HOVER, size=None,
             seconds=HOLD_S, every=EVERY):
    """Fly the trimmed case with a person's hands on it.

    :func:`case_setup` for the condition and :class:`Hands` for the pilot; what this
    adds is the flight, and it is the case every number below is measured on.
    Returns ``(sim, hands, trace)``.  *every* prints a line of the case's own
    progress that many frames apart, or nothing for ``None`` - which is what the
    sweeps fly at, a line a second of a dozen cases being a transcript nobody reads.
    """
    sim, trim, hands = case_setup(axis=axis, scales=scales, condition=condition,
                                  size=size)
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


def frozen_case(axis="pitch", condition=HOVER, seconds=FROZEN_S, every=EVERY):
    """The same perturbation with *nobody* flying it: what the model does alone.

    The sticks are where the trim left them and no pilot moves them - a
    :class:`Hands` is still built, because the trace needs one for its references,
    and never flown, which is exactly what a pair of hands off the controls is - so
    what follows is the aircraft's own answer to the perturbation and nothing else.
    It is the reference every band in this file is read against, and half the reason
    a gain has a *low* edge as well as a high one: the trim this model holds is not
    a place it stays, and a pilot's margin is what their hands buy against that.
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


# ---------------------------------------------------------------------------
# One case, one leg, and one axis's whole band.
# ---------------------------------------------------------------------------


#: What it is that a scale scales, per axis, in the words a band's header wants.
GAIN_NAMES = {"pitch": "the two cyclic gains", "roll": "the two cyclic gains",
              "heave": "the lever's gain", "yaw": "the pedals' gain and lead"}


def gain_words(axis, scales):
    """What the axis's own loop is at *scales*, in the units a stick is in.

    The number a margin is quoted in: inches of stick per degree or per m/s, with
    the lead the loop is closed at beside it where it has one - which is the whole
    difference between the four axes, and the reason a scale is not a number that
    means the same thing on any of them.
    """
    if axis == "heave":
        return "lever %0.3f in per m/s" % (K_LEVER_IN * scales.lever,)
    if axis == "yaw":
        return "pedals %0.3f in per deg at a %.2f s lead" % (
            K_PEDAL_IN * scales.pedal, T_PEDAL_LEAD_S)
    return "cyclic %0.3f in per deg at a %.2f s lead" % (
        K_THETA_IN * scales.theta, T_LEAD_S)


def case(axis="pitch", scale=SCALE_NOMINAL, condition=HOVER, every=EVERY,
         label=None):
    """Fly one case at *scale* of *axis* and hand back its trace.

    The case is the whole of what every number in this file is read off, so the two
    things a harness owes are paid here: the frame loop dropped no step of the
    flight, and the flight's own record of the pilot's decisions is printed beside
    the trace - which :func:`instrument` is the *measurement* of and this is the
    assertion.  A case that crashed is not an error here: it is one of the answers a
    grip can give, and the trace is what says so.
    """
    reading = READINGS[axis]
    print("")
    print("--- %s: %+0.2f %s of %s put in, %s"
          % (label or "case", PERTURB[axis], reading.unit, axis,
             gain_words(axis, scales_for(axis, scale))))
    sim, hands, trace = fly_case(axis=axis, scales=scales_for(axis, scale),
                                 condition=condition, every=every)
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


def failed_words(sim, trace, settled, tail):
    """Why a grip did not hold, in the trace's own terms.

    Three ways a case can fail, and they are not the same event: the machine left
    the sky, the error never held the band at all, or it *did* hold it and then grew
    back out of it - which is the one a criterion of "it settled once" would call a
    success, and the reason the edge is read off the tail and not off the settling.
    """
    if sim.crashed:
        return "the machine left the sky at %.2f s" % (sim.sim_time,)
    if settled is None:
        return ("the error never held inside %0.2f %s for %.1f s"
                % (trace.band, trace.unit, SETTLE_S))
    return ("it settled at %.2f s and then grew back out of the band, to %.1f times"
            " its middle third" % (settled, tail))


def leg(axis, ladder, harder, condition=HOVER):
    """One direction of an axis's band: from the nominal until a grip fails.

    *held* is three things and not one - phase 1's own criterion, and the whole of
    what a margin means here: the error came back inside the axis's band and
    *stayed* (``Trace.settled``), it is still dying out at the end of the case
    (``Trace.tail_growth``), and the machine is in the sky to have done it.  A grip
    that settles at eight seconds and wanders back out of the band by thirty has
    held nothing, so a criterion of "it settled once" would put the edge a scale too
    high.

    Returns ``(edge, pio_at, rows)``: a row's ``(scale, held, trace)``, the first
    scale that failed, and - on the harder leg only - the first scale whose
    *failure* is a pilot induced oscillation rather than the model's own slow mode.
    The harder leg therefore climbs past the edge until the pilot's own loop is what
    has closed, which is this file's PIO question: the edge of a band and the grip
    an axis turns into a PIO at are two different numbers and both are worth having.
    """
    print("  -- gripped %s than the model's own pilot:"
          % ("harder" if harder else "softer"))
    edge = None
    pio_at = None
    rows = []
    for scale in ladder:
        sim, hands, trace = fly_case(axis=axis, scales=scales_for(axis, scale),
                                     condition=condition, every=None)
        settled = trace.settled()
        worst, _ = trace.overshoot()
        tail, period, growth = trace.tail_growth(), trace.period(), trace.growth()
        is_pio = trace.pio()[0]
        held = (settled is not None and not sim.crashed
                and tail is not None and tail <= 1.0)
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
            print("  this grip did not hold: %s"
                  % (failed_words(sim, trace, settled, tail),))
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
              " oscillation: what fails at the edge is the aircraft's own slow mode"
              " and not the pilot's loop." % (edge, ladder[-1]))
    return edge, pio_at, rows


def band(axis, condition=HOVER):
    """Fly both legs of one axis's gain band and hand back the two edges.

    Every axis of this model has *two* edges and not one, and they fail differently:
    too hard a grip closes the pilot's own loop on the pilot's own delay, and too
    soft a one cannot hold what the outer loop is asking the stick for.  So a margin
    is reported as the pair it is - how much harder than the model's own pilot a grip
    may be before the axis stops coming back, and how much softer before it does not
    come back at all - and inside the harder leg the scale the axis turns into a
    *PIO* at is read as well, which is further up than the edge: see :func:`leg`.

    Returns ``(margin, pio_at, floor, harder_rows, softer_rows)``, with an edge
    ``None`` if its leg never found one - which is an answer too, and the one a
    forgiving axis gives.
    """
    print("")
    print("=== %s band: %+0.2f %s of %s put in, %s scaled ==="
          % (CONDITION_WORDS[condition], PERTURB[axis], READINGS[axis].unit,
             axis, GAIN_NAMES[axis]))
    print("  scale      dB    worst   settled    tail/mid   period   growth"
          "   height m   %s"
          % ("drift m" if condition == HOVER else "speed m/s",))
    margin, pio_at, up = leg(axis, SCALES_HARDER, True, condition)
    floor, _, down = leg(axis, SCALES_SOFTER, False, condition)
    return margin, pio_at, floor, up, down


def pio_report(scale, trace):
    """Say what is oscillating at *scale* of a case, and whether it is a PIO.

    The period and the growth are the trace's own; what makes the answer a *pilot
    induced* oscillation is the number printed beside them, which is what the
    pilot's own fifth of a second of delay is worth at that period.  A loop that is
    growing where the delay costs tens of degrees of phase is a loop the pilot's own
    lag has closed, and no aircraft mode has anything to do with it.
    """
    is_pio, period, growth = trace.pio()
    if period is None:
        print("  x%0.2f %s: nothing left to oscillate - the error came back inside"
              " the band and stayed there" % (scale, trace.axis))
        sys.stdout.flush()
        return False
    if is_pio:
        reason = "A PILOT INDUCED OSCILLATION"
    elif not (PIO_PERIOD_S[0] <= period <= PIO_PERIOD_S[1]):
        reason = ("not a PIO: %.2f s is outside the %.1f to %.1f s band a person's"
                  " own loop lives in"
                  % (period, PIO_PERIOD_S[0], PIO_PERIOD_S[1]))
    else:
        reason = ("not a PIO: %.2f a cycle is dying out, and the PIO's own line is"
                  " %.2f" % (growth, PIO_GROWTH))
    print("  x%0.2f %s: period %.2f s, growing %.2f a cycle, and the pilot's own"
          " %.2f s of delay is worth %.0f deg of that period - %s"
          % (scale, trace.axis, period, growth, DELAY_S,
             Trace.phase_deg(period), reason))
    sys.stdout.flush()
    return is_pio


def instrument(axis="heave", seconds=5.0):
    """Fly a short case and hold the pilot itself to the clock it claims.

    Phase 1's own check, on an axis phase 1 never flew, which is why the axis is an
    argument here and a constant there: the number of decisions has to be the
    pilot's own rate over the time flown, and the age of every view a decision was
    made from has to be the ``DELAY_S`` of the delay to the last bit - the one thing
    :meth:`Hands.selftest` cannot check and a flight can.  The whole run also has to
    have been flown without the frame loop dropping a step, since a harness whose
    clock is not what it says makes every number under it meaningless.
    """
    print("")
    print("--- the instrument itself: the pilot's own clock over %.0f s of a %s case"
          % (seconds, axis))
    sim, hands, trace = fly_case(axis=axis, every=None, seconds=seconds)
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

#: The nominal case of each axis, s, as this file first flew it: the two tilt axes
#: are phase 1's own pinned numbers and the two new ones are what its instrument
#: measures on the loops that file never touched.
#:
#: A metre of height takes the model's own lever fourteen seconds to take back out
#: and ten degrees of nose four, so neither of the two axes phase 1 left alone is a
#: *slow* axis of the aircraft - and the lever's fourteen is not the lever's at all:
#: a hover's height is asked for through the model's own ``RATE_GAIN_MPS``, twelve
#: centimetres a second per metre, and it is that outer loop's time constant the
#: settling time is really measuring.  See the hands-off case beside it, where the
#: same metre comes back in nine and a half seconds with *nobody* flying: on this
#: axis, and out of the ground cushion, a hover's height is a place the aircraft
#: finds by itself.
NOMINAL_SETTLED_S = {"pitch": NOMINAL_PITCH_SETTLED_S,
                     "roll": NOMINAL_ROLL_SETTLED_S,
                     "heave": 14.42,
                     "yaw": 4.03}

#: And the four bands, as the two legs measured them - the softest grip that held
#: the axis, the hardest that did, the scale the harder leg failed first at, the
#: scale the softer leg failed first at (``None`` where a leg never found one,
#: which is an answer and not a gap), and the scale the axis turns into a pilot
#: induced oscillation at.
#:
#: The two tilt axes do **not** have the same band, which is the first thing this
#: file says that phase 1 could not: that file swept pitch and printed one band,
#: and the roll is both *softer* below - x0.50 holds where pitch has already failed
#: - and *harder* above, where x1.50 is the pilot's own loop at a 1.05 s period
#: while pitch's x1.50 is still the aircraft's 11.5 s mode and its PIO waits for
#: x2.00.
#:
#: The lever and the pedals have no soft edge in the hover at all: every grip down
#: to x0.25 held, because what holds those two axes is the outer loop and the
#: aircraft rather than the pilot's hand - the cushion brings the metre of height
#: back on its own, and a hover's nose with the pedals nowhere near stiff enough
#: still gets caught by the gain and the lead the model's own turn loop is flown
#: with.  Their hard edges are the pilot's own loops and they are much further up:
#: x2.00 on the lever, x4.00 on the pedals.
BANDS = {"pitch": (0.75, 1.0, 1.5, 0.5, 2.0),
         "roll": (0.5, 1.0, 1.5, 0.35, 1.5),
         "heave": (0.25, 1.5, 2.0, None, 2.0),
         "yaw": (0.25, 3.0, 4.0, None, 4.0)}

#: What the two failures that *are* the pilot's oscillate at: the period, s, and
#: the growth a cycle, of the PIO each axis closes.  Pitch's pair is phase 1's own
#: and is asserted against that file's constants rather than written down again.
PIO_MEASURED = {"pitch": (PIO_MEASURED_PERIOD_S, PIO_MEASURED_GROWTH),
                "roll": (1.05, 1.15),
                "heave": (2.72, 1.05),
                "yaw": (1.25, 1.23)}

#: The check at sixty knots: the nominal settling this file measured out there, and
#: - the claim the whole exercise is for - which of the grips each axis's *hover*
#: band failed at fail out there as well.
#:
#: The tilt pair's bands transfer, and comfortably: both of the grips the hover
#: failed at hold at sixty knots, the loop settles in half the time, and the speed
#: comes back with the attitude.  The lever's and the pedals' do not transfer at
#: all: the same scale is a *stiffer* loop on an aircraft with an airstream behind
#: its rotor and a fin behind its tail, and x2.00 of lever and x4.00 of pedals
#: diverge out there as surely as they do in the hover.
#:
#: So the hover is where the *attitude* has no damping, and it is not the hardest
#: condition for the other two axes - a margin read in the hover is a margin for
#: the cyclic and nothing else, and the two axes phase 1 left at the model's own
#: gains are held there by the ground and by the outer loops rather than by the
#: pilot's hands.  That is this phase's answer, and it is the opposite of the one
#: the check was written expecting.
CRUISE_SETTLED_S = {"pitch": 2.80, "roll": 2.87, "heave": 20.78, "yaw": 22.37}
CRUISE_FAILS = {"pitch": (), "roll": (), "heave": (2.0,), "yaw": (4.0,)}

#: And what the four axes do out there with nobody flying them: ``(shape, limit)``,
#: where "never" is a case that never held its band and ended at least *limit* off
#: the trim, and "back" is one that held it and ended inside it.  It is the other
#: half of what the check above is saying and the reason the two new axes' margins
#: are not the tilt pair's: at sixty knots a metre of height put into the aircraft
#: stays put - there is no cushion up there to be a spring, so the height is *not*
#: a place the aircraft finds by itself - and roll is given back while pitch is
#: only half given back: 1.42 deg of its two still on the clock when the twenty
#: seconds run out.  The nose is not given back at all - the perturbation is a
#: heading, so nothing is slipping for the fin to weathervane against, and the
#: nose parks where it was put, which is the same shape the hover gives it.
FROZEN_CRUISE = {"pitch": ("never", 1.4), "roll": ("back", 0.5),
                 "heave": ("never", 0.9), "yaw": ("never", KEPT_YAW_DEG)}


def trace_at(rows, scale):
    """The trace of one scale of a leg: :func:`leg`'s own rows."""
    return [trace for one, _, trace in rows if one == scale][0]


def fail_words(trace):
    """How a failed case failed, read off a trace that carries its flight.

    The three ways a case fails, in the order :func:`failed_words` reads them off a
    flight - the machine left the sky, the error never held the band at all, or it
    held it and then grew back out of it - for a caller that has the *trace* and not
    the simulation.  A trace keeps the simulation it was measured on, which is what
    makes the first of the three knowable from here at all.
    """
    if trace.sim.crashed:
        return "the machine left the sky at %.2f s" % (trace.sim.sim_time,)
    if trace.settled() is None:
        return ("it never held the %.2f %s band for %.1f s, reaching %+0.2f %s"
                % (trace.band, trace.unit, SETTLE_S, trace.overshoot()[0],
                   trace.unit))
    return ("it settles at %.2f s and then grows back out of the band, to %.1f times"
            " its middle third"
            % (trace.settled(), trace.tail_growth() or float("inf")))


def cruise_check(axis, margin, floor, condition=CRUISE):
    """The check: *axis* at sixty knots, at the scales the hover's band failed at.

    Two questions, and one flight each.  The nominal case first, flown exactly as
    the hover's was - the same perturbation, the same loop, the model's own pilot -
    so that the two settling times can be read against each other, with the *speed*
    beside them: the loop above is unchanged out there, and what is different is that
    an aircraft at sixty knots has an airstream to be damped by, so holding the
    trim's own attitude is holding the speed the trim was solved for rather than
    whatever the machine feels like next.

    And then the two scales the hover's own band failed at, re-flown at the same
    grips.  That is where the claim is: a margin measured in the hover is only worth
    having if the hover is the difficult condition.

    Returns ``(nominal_trace, rows)``, a row being ``(scale, held, trace)``.
    """
    print("")
    print("--- %s, the model's own pilot and then the two grips the hover failed at:"
          " %+0.2f %s of %s" % (CONDITION_WORDS[condition], PERTURB[axis],
                                READINGS[axis].unit, axis))
    sim, hands, nominal = fly_case(axis=axis, condition=condition, every=None)
    print("  the model's own pilot (x%.2f): %s" % (SCALE_NOMINAL, nominal))
    print("    and the speed with it: %.2f m/s either side of the %0.1f m/s the"
          " trim was solved for" % (nominal.speed_error(), nominal.speed_ref))
    rows = []
    for scale in (margin, floor):
        if scale is None:
            continue
        sim, hands, trace = fly_case(axis=axis, scales=scales_for(axis, scale),
                                     condition=condition, every=None)
        settled, tail = trace.settled(), trace.tail_growth()
        held = (settled is not None and not sim.crashed
                and tail is not None and tail <= 1.0)
        rows.append((scale, held, trace))
        print("  x%0.2f, the grip the hover's band failed at: %s: %s"
              % (scale, "holds out here" if held else "DOES NOT HOLD", trace))
        if not held:
            print("    %s" % (failed_words(sim, trace, settled, tail),))
        sys.stdout.flush()
    return nominal, rows


# ---------------------------------------------------------------------------
# The run: four axes in the hover, and then the same four at sixty knots.
# ---------------------------------------------------------------------------


def main():
    """The four axes' bands in the hover, and the check on them at sixty knots."""
    transcript = Transcript(LOG_FILE)
    sys.stdout = transcript
    try:
        # -- what the instrument is ----------------------------------------
        print("the pilot: %0.0f decisions a second on the flight's own clock,"
              " %.2f s of transport delay (%d frames), a %.2f s lag in the hand,"
              " and three loops under scales of their own"
              % (PILOT_HZ, DELAY_S, DELAY_FRAMES, T_LAG_S))
        print("  the model's own hover pilot is this one at a scale of one on every"
              " loop: %.3f in of stick per deg of attitude at a %.2f s lead, %.3f in"
              " of collective per m/s of climb rate - no lead, since the loop is a"
              " rate loop - and %.3f in of pedal per deg of nose at a %.2f s lead"
              % (K_THETA_IN, T_LEAD_S, K_LEVER_IN, K_PEDAL_IN, T_PEDAL_LEAD_S))
        for axis in AXES:
            print("  %-6s %+0.2f %s put in, back inside %0.2f %s, and one grip"
                  " scaled: %s"
                  % (axis, PERTURB[axis], READINGS[axis].unit, SETTLE[axis],
                     READINGS[axis].unit, GAIN_NAMES[axis]))
        instrument()

        # -- the nominal case of each axis, with hands on -------------------
        print("")
        print("=== the model's own pilot, one axis at a time: a perturbation it did"
              " not ask for ===")
        nominal = {}
        pio = {}
        for axis in AXES:
            nominal[axis] = case(axis, label="the model's own hover pilot")
            pio[axis] = pio_report(SCALE_NOMINAL, nominal[axis])

        # -- and phase 1's two numbers, re-measured through this instrument --
        print("")
        print("=== the two axes phase 1 measured, held to that file's own numbers ===")
        for axis, wanted in (("pitch", NOMINAL_PITCH_SETTLED_S),
                             ("roll", NOMINAL_ROLL_SETTLED_S)):
            print("  %s: this file settles it in %.2f s, that file pinned %.2f s"
                  " (it allows itself %.1f s)"
                  % (axis, nominal[axis].settled(), wanted, SETTLE_TOLERANCE_S))

        # -- the four axes with nobody flying them -------------------------
        print("")
        print("=== the four axes with nobody flying them: the model's own answer ===")
        frozen = {}
        for axis in AXES:
            print("--- %+0.2f %s of %s put in, the sticks where the trim left them"
                  % (PERTURB[axis], READINGS[axis].unit, axis))
            sim, hands, trace = frozen_case(axis)
            frozen[axis] = trace
            inside = trace.settled()
            print("  %s" % (trace,))
            print("  %0.1f s with no hands on it: it ended %+0.2f %s off the trim,"
                  " and %s"
                  % (sim.sim_time, trace.errors[-1], trace.unit,
                     "the %.2f %s band was held for two seconds at %.2f s on the"
                     " way" % (trace.band, trace.unit, inside)
                     if inside is not None else
                     "the %.2f %s band was never held at all"
                     % (trace.band, trace.unit)))
            sys.stdout.flush()

        # -- the four bands -------------------------------------------------
        print("")
        print("=== the bands: one axis's own loop at a time, from the model's own"
              " gains ===")
        bands = {}
        for axis in AXES:
            bands[axis] = band(axis)

        # -- what each band is, in the axis's own words ---------------------
        print("")
        print("=== the verdict on each axis ===")
        for axis in AXES:
            margin, pio_at, floor, harder, softer = bands[axis]
            unit, settle = READINGS[axis].unit, SETTLE[axis]
            held_up = max(scale for scale, held, _ in harder if held)
            held_down = min(scale for scale, held, _ in softer if held)
            print("")
            print("  %s: %+0.2f %s put in, back inside %0.2f %s in %.2f s, having"
                  " cost %.2f m of height and %.1f m of the pad"
                  % (axis, PERTURB[axis], unit, settle, unit,
                     nominal[axis].settled(), nominal[axis].altitude_error(),
                     nominal[axis].drift))
            print("    the band: x%.2f to x%.2f of the model's own %s - %.1f dB to"
                  " %+.1f dB%s"
                  % (held_down, held_up, GAIN_NAMES[axis],
                     20.0 * math.log10(held_down), 20.0 * math.log10(held_up),
                     "" if margin is not None else
                     ", and nothing in the harder leg failed at all"))
            if floor is not None:
                print("    below it the *%s* is what fails: x%.2f (%.1f dB) - %s"
                      % (axis, floor, 20.0 * math.log10(floor),
                         fail_words(trace_at(softer, floor))))
            if margin is not None:
                hard = trace_at(harder, margin)
                print("    above it: x%.2f (%+.1f dB) - %s, at a %.2f s period the"
                      " pilot's %.2f s of delay is worth %.0f deg of"
                      % (margin, 20.0 * math.log10(margin), fail_words(hard),
                         hard.period() or 0.0, DELAY_S,
                         Trace.phase_deg(hard.period()) or 0.0))
            if pio_at is not None:
                _, period, growth = trace_at(harder, pio_at).pio()
                print("    and the pilot's own loop closes at x%.2f (%+.1f dB): %.2f"
                      " s a cycle, growing %.2f of itself a cycle, with the delay"
                      " worth %.0f deg of that period - A PILOT INDUCED OSCILLATION"
                      % (pio_at, 20.0 * math.log10(pio_at), period, growth,
                         Trace.phase_deg(period)))
            print("    with nobody flying it: %+0.2f %s left at the end of %.0f s,"
                  " and %s"
                  % (frozen[axis].errors[-1], unit, frozen[axis].times[-1],
                     "the band was held for two seconds at %.2f s on the way"
                     % (frozen[axis].settled(),)
                     if frozen[axis].settled() is not None else
                     "the band was never held at all"))
            sys.stdout.flush()

        # -- the check: the same four axes at sixty knots -------------------
        print("")
        print("=== the check: the same four axes, the same perturbations, at sixty"
              " knots ===")
        print("  The hover is where this model has no damping at all and its rotor is"
              " working in its own downwash, so a margin read there ought to be a"
              " margin everywhere - and that is a claim about *all four* axes, which"
              " is why the other three are here.  Each axis is re-flown: its nominal"
              " case, and then the two grips its own band failed at in the hover.")
        cruise = {}
        for axis in AXES:
            margin, pio_at, floor = bands[axis][0], bands[axis][1], bands[axis][2]
            cruise[axis] = cruise_check(axis, margin, floor)

        print("")
        print("=== and the same four with nobody flying them at sixty knots: what the"
              " aircraft does out there on its own ===")
        cruise_frozen = {}
        for axis in AXES:
            print("--- %+0.2f %s of %s put in at sixty knots, the sticks where the"
                  " trim left them"
                  % (PERTURB[axis], READINGS[axis].unit, axis))
            sim, hands, trace = frozen_case(axis, condition=CRUISE)
            cruise_frozen[axis] = trace
            inside = trace.settled()
            print("  %s" % (trace,))
            print("  %0.0f s with no hands on it: it ended %+0.2f %s off the trim,"
                  " and %s"
                  % (sim.sim_time, trace.errors[-1], trace.unit,
                     "the %.2f %s band was held for two seconds at %.2f s on the"
                     " way" % (trace.band, trace.unit, inside)
                     if inside is not None else
                     "the %.2f %s band was never held at all"
                     % (trace.band, trace.unit)))
            sys.stdout.flush()

        # -- and what all of it is held to ----------------------------------
        # The pilot's own clock and delay were measured in flight by
        # :func:`instrument`, which raises rather than printing: nothing below is
        # worth reading if that did not pass.  Everything after it is this file's
        # claim, and a claim whose numbers drift has to be made again.
        #
        # The nominal case of every axis first, and the two axes phase 1 *pinned*
        # are held to that file's own numbers: the same two perturbations into the
        # same two loops, with the two gains phase 1 scaled and nothing else moved,
        # so a drift here is a drift in the instrument and not a difference of
        # opinion.  The four settling times are this file's, and they are the four
        # the transcript above printed.
        for axis, wanted in (("pitch", NOMINAL_PITCH_SETTLED_S),
                             ("roll", NOMINAL_ROLL_SETTLED_S)):
            assert nominal[axis].settled() is not None, \
                "the model's own pilot cannot recover %+0.1f deg of %s" \
                % (PERTURB[axis], axis)
            assert abs(nominal[axis].settled() - wanted) <= SETTLE_TOLERANCE_S, \
                (axis, nominal[axis].settled(), wanted)
        # And the pitch *band* is that file's too, so it is held to that file's own
        # constants rather than to this one's table: the hardest grip of the model's
        # own gains that held, the softest, the scale each leg first failed at and
        # the grip the pilot's own loop closes at are the five numbers phase 1
        # pinned, and this file is flying the same case - which is what makes the
        # other three rows of :data:`BANDS` comparable with it at all.
        assert BANDS["pitch"] == (NOMINAL_FLOOR_SCALE, NOMINAL_MARGIN_SCALE,
                                  MARGIN_FAILS_AT, FLOOR_FAILS_AT,
                                  PIO_AT_SCALE), BANDS["pitch"]
        for axis in AXES:
            settled = nominal[axis].settled()
            assert settled is not None, \
                "the model's own pilot cannot recover %+0.2f %s of %s" \
                % (PERTURB[axis], READINGS[axis].unit, axis)
            assert abs(settled - NOMINAL_SETTLED_S[axis]) <= SETTLE_TOLERANCE_S, \
                (axis, settled, NOMINAL_SETTLED_S[axis])
            assert not pio[axis], (axis, pio[axis])
            assert nominal[axis].drift <= NOMINAL_DRIFT_M, \
                (axis, nominal[axis].drift)
            # The hover being *held* while the axis is argued about: the height to
            # phase 1's centimetre for the three axes that are not the height, and
            # for the heave the perturbation itself, since the worst the height got
            # is where the case began - a lever loop that took the machine further
            # off than the gust did would be no pilot at all.
            if axis == "heave":
                assert nominal[axis].altitude_error() <= PERTURB_HEAVE_M + 0.05, \
                    nominal[axis].altitude_error()
                assert nominal[axis].altitude_error() <= HEIGHT_LIMIT_M, \
                    nominal[axis].altitude_error()
            else:
                assert nominal[axis].altitude_error() <= NOMINAL_HEIGHT_M, \
                    (axis, nominal[axis].altitude_error())
        # Each axis's band, as the two legs measured it: the grips that held, the
        # scale each leg first failed at, and the grip the pilot's own loop closes
        # at - with the four pinned in :data:`BANDS` and :data:`PIO_MEASURED`.  A
        # band whose edge is ``None`` is a leg in which nothing failed, which is an
        # answer and is pinned as one.
        for axis in AXES:
            margin, pio_at, floor, harder, softer = bands[axis]
            want_down, want_up, want_margin, want_floor, want_pio = BANDS[axis]
            held = sorted(scale for scale, ok, _ in harder + softer if ok)
            failed = sorted(scale for scale, ok, _ in harder + softer if not ok)
            assert min(held) == want_down, (axis, held, want_down)
            assert max(held) == want_up, (axis, held, want_up)
            assert margin == want_margin, (axis, margin, want_margin)
            assert floor == want_floor, (axis, floor, want_floor)
            assert pio_at == want_pio, (axis, pio_at, want_pio)
            assert SCALE_NOMINAL in held, (axis, held)
            if margin is not None:
                assert margin in failed, (axis, margin, failed)
            if floor is not None:
                assert floor in failed, (axis, floor, failed)
            # And the failure that *is* the pilot's: the period and the growth this
            # file pinned, at a period the pilot's own fifth of a second of delay is
            # worth tens of degrees of - which is what makes it the pilot's loop and
            # not an aircraft mode.  Pitch is phase 1's own case, and it is held to
            # the stricter line that file drew for it.
            _, period, growth = trace_at(harder, pio_at).pio()
            want_period, want_growth = PIO_MEASURED[axis]
            assert abs(period - want_period) <= 0.05, (axis, period, want_period)
            assert abs(growth - want_growth) <= 0.05, (axis, growth, want_growth)
            assert Trace.phase_deg(period) >= 20.0, (axis, Trace.phase_deg(period))
            if axis == "pitch":
                assert Trace.phase_deg(period) >= 45.0, Trace.phase_deg(period)
        # And the check itself: sixty knots holds the nominal case of every axis,
        # its speed comes back with it, and then the *grips* - which is the answer
        # this phase actually got, and it is in :data:`CRUISE_FAILS`.  The tilt
        # pair's bands transfer and the other two do not, because a scale is a scale
        # of the model's own gain and the same gain is not the same loop on an
        # aircraft with an airstream behind its rotor and a fin behind its tail.
        for axis in AXES:
            cruise_nominal, rows = cruise[axis]
            assert cruise_nominal.settled() is not None, \
                "sixty knots cannot recover %+0.2f %s of %s" \
                % (PERTURB[axis], READINGS[axis].unit, axis)
            assert abs(cruise_nominal.settled() - CRUISE_SETTLED_S[axis]) \
                <= SETTLE_TOLERANCE_S, (axis, cruise_nominal.settled(),
                                        CRUISE_SETTLED_S[axis])
            assert cruise_nominal.speed_error() <= CRUISE_SPEED_LIMIT_MPS, \
                (axis, cruise_nominal.speed_error())
            failed = tuple(scale for scale, held, _ in rows if not held)
            assert failed == CRUISE_FAILS[axis], (axis, failed, CRUISE_FAILS[axis])
        # The hands-off reference, and the four are not the same story.  Pitch never
        # comes back and ends further out than it was put; roll holds the band for
        # two seconds on its way past and wanders out of it the other way, which is
        # what a lightly damped aircraft does and not what an unstable one does; the
        # lever's metre of height comes back *on its own*, inside the band, because
        # the ground cushion is a spring; and the nose does not move at all.  A band
        # whose reference is "the aircraft comes back anyway" is a band about a
        # pilot's hands and nothing else, which is the whole of what the two axes
        # phase 1 left alone turned out to be.
        assert frozen["pitch"].settled() is None, frozen["pitch"].settled()
        assert frozen["pitch"].drift > DRIFT_LIMIT_M, frozen["pitch"].drift
        assert abs(frozen["pitch"].errors[-1]) >= PERTURB_DEG, \
            frozen["pitch"].errors[-1]
        assert frozen["roll"].settled() is not None, "roll never held the band"
        assert abs(frozen["roll"].errors[-1]) > SETTLE_DEG, \
            frozen["roll"].errors[-1]
        assert frozen["heave"].settled() is not None, "the cushion did nothing"
        assert abs(frozen["heave"].errors[-1]) <= SETTLE_HEAVE_M, \
            frozen["heave"].errors[-1]
        assert frozen["yaw"].settled() is None, frozen["yaw"].settled()
        assert abs(frozen["yaw"].errors[-1]) >= KEPT_YAW_DEG, \
            frozen["yaw"].errors[-1]
        # And out there with nobody flying them: the same four questions, and the
        # answer is what a margin in the hover cannot see - see FROZEN_CRUISE.
        for axis in AXES:
            shape, limit = FROZEN_CRUISE[axis]
            trace = cruise_frozen[axis]
            if shape == "never":
                assert trace.settled() is None, (axis, trace.settled())
                assert abs(trace.errors[-1]) >= limit, (axis, trace.errors[-1])
            else:
                assert trace.settled() is not None, (axis, "never held")
                assert abs(trace.errors[-1]) <= limit, (axis, trace.errors[-1])
        print("")
        print("  the transcript is %s, and every number in it is asserted - a"
              " regression is a traceback and not a table nobody read."
              % (LOG_FILE,))
        sys.stdout.flush()
    finally:
        sys.stdout = sys.__stdout__
        transcript.close()
    print("wrote %s" % (LOG_FILE,))


if __name__ == "__main__":
    main()

