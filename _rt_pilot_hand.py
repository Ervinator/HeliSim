# -*- coding: utf-8 -*-
"""Scratch: the harness's other two missions flown by a person's hands.

``_rt_autoro_hand.py`` - phase 3 - put phase 2's instrument on the
*autorotation's* own loops and flew both of its switches through it.  What it
left is the rest of the harness's missions, and ``TODO.md``'s ``Next`` names it:
``_rt_check.py``'s hover-turn and cruise fly their climbs, and the height, the
speed and the heading they hold, off ``_rt_pilot``'s own trim rows and
*scripted*, with no hand on either.

This file is that half.  The instrument is unchanged - ``_rt_axes.Hands``: twenty
decisions a second on the *flight's* own clock, ``DELAY_S`` of transport delay
read off a deque of remembered states, ``T_LAG_S`` of lag in the hand, and the
model's own limb rate underneath it - and what it decides is the two missions'
own loops rather than the instrument's hover:

* :class:`HoverTurnHands` is ``_rt_pilot.HoverTurnPilot``'s own mission, phase
  for phase: the lever through a climb rate to the band of twenty feet and back
  down on a creep, the cyclic holding the machine over the pad through the turn,
  and the pedals carrying the heading the whole way round a 360 on the mission's
  own schedule;
* :class:`CruiseHands` is ``_rt_pilot.HumanPilot``'s: a hundred metres up on the
  lever, the speed wound on over the last ten metres of the climb on the pitch
  attitude the trim of the speed asked for needs, the line north of the pad on
  the lateral stick, and the nose on the pedals.

Three things are what ``TODO.md`` asked for, and each is worth a sentence:

* **the instrument's own clock.**  Two are flown: the twenty a second the two
  phases before this one fly it at, and :data:`_rt_handling.DELAY_S` - one
  decision a delay, the slowest hand that instrument will fly at all, since its
  own ``selftest`` refuses a person who decides less often than their eyes see.
  The mission's own :data:`_rt_pilot.REACTION_S`, which is the clock its script
  commits its numbers at, is longer than that, and is therefore refused to the
  last decimal rather than flown: :func:`same_ruler` asserts the refusal;
* **the phases read off the view rather than off the aircraft.**  A mission's
  phase machine is a pilot's own bookkeeping - ``HoverTurnPilot.phase_of``'s five
  phases, ``HumanPilot``'s climb and its kilometre - and every reading it turns
  on here is the aircraft as it was ``DELAY_S`` ago, so a phase is named a fifth
  of a second late.  That is what a person flying by instruments has, and it is
  the one thing the script that flies these missions has not;
* **the trims read off the model rather than written down again.**  The rows the
  loops are written around are ``_rt_pilot.trims_for``'s: the mission class names
  its own speeds and its own height and the model solves them, so a change to the
  aircraft moves the pilot's baseline with it.  And the sticks a mission begins
  and ends on are the *pad's* own - ``simulation.Simulation.trim_controls`` at
  ``simulation.PAD_COLLECTIVE``, which is the state ``on_the_pad`` parks the
  aircraft in and so the state a mission leaves it in.

What is claimed *before* anything is flown is the smaller thing every number
below rests on, for each mission: at a scale of one, with the lag switched off and
the aircraft itself handed to the decision instead of a memory of it, one
decision by this file's pilot and one by the mission's own land on the same four
axes, to the last bit of an inch - so that what a person's row differs by is the
person, and not a second pilot.

What comes back, into ``_rt_pilot_hand.txt``:

* **the missions' own rows**, first: ``_rt_check.check``'s own flight of each
  mission, called rather than copied, at the clock ``_rt_check`` commits its own
  claims at and with those claims in force - so the row everything below is read
  against is the mission the harness actually flies, and a mission that stopped
  reaching its own goals would stop this file rather than quietly become the
  baseline;
* **a person's own row** for each mission at each of the two clocks: where the
  aircraft got to, the band its height stayed inside, how far off the pad it was
  ever taken, the rate the skids arrived at, how many decisions it took, and what
  the four axes were asked for at their furthest - the last of which is this
  file's own stand-in for ``_rt_check.Run``'s key census, since a hand is not a
  scancode and cannot be counted as one;
* **the cyclic's own band**, on both missions, from the mission's own gains
  outward each way, which is the shape phases 1, 2 and 3 all end on - a band and
  not a number - and the one axis phase 2 found the hover's margin to be
  entirely;
* **and the finding**: what a hand costs these two missions, in the missions' own
  numbers - the arrival and not the flying - and each mission's cyclic band.

Usage::

    python _rt_pilot_hand.py [rate ...]
"""

import math
import sys

from _rt_axes import CRUISE, HOVER, SCALES_ONE, Scales
from _rt_axes import Hands as Instrument
from _rt_autoro_entry import words
from _rt_check import check as scripted_check
from _rt_handling import (DELAY_FRAMES, DELAY_S, PILOT_HZ, PILOT_PERIOD_S,
                          T_LAG_S, Transcript)
from _rt_pilot import (ALT_REF_M, ALT_TOLERANCE_M, BANK_LIMIT_DEG,
                       CLIMB_ALT_M, CLIMB_LIMIT_MPS, CRUISE_MPS,
                       DESCENT_RATE_MPS, DESCENT_TAPER_FLOOR, DESCENT_TAPER_M,
                       DIST_REF_M, HOVER_ALT_M, HOVER_REF_EAST_M,
                       HOVER_REF_NORTH_M, HOVER_SETTLE_S, HOVER_SKID_FT,
                       HOVER_TOLERANCE_M, HOVER_TILT_LIMIT_DEG, K_BANK_DEG,
                       K_HOVER_PITCH_IN, K_HOVER_POS_DEG_M, K_HOVER_RATE_IN,
                       K_HOVER_ROLL_IN, K_HOVER_VEL_DEG_MPS, K_PITCH_IN,
                       K_PITCH_RATE_IN, K_RATE_IN, K_ROLL_IN, K_ROLL_RATE_IN,
                       K_SPEED_DEG, K_TURN_IN, K_TURN_RATE_IN, K_YAW_IN,
                       K_YAW_RATE_IN, KEY_STEP, LANDING_SETTLE_S,
                       LEVER_CREEP_IN_S, LEVER_LIMIT_IN, PARKED_ALT_M,
                       PEDAL_PER_DEG_S, PITCH_LIMIT_DEG, RATE_GAIN_MPS,
                       REACTION_S, SPEED_RAMP_M, TRACK_EAST_REF_M,
                       TURN_LEAD_DEG, TURN_LEAD_FLOOR, TURN_RATE_DEG_S,
                       TURN_RATE_TOLERANCE_DEG_S, TURN_SETTLE_S,
                       TURN_TOLERANCE_DEG, YAW_REF_DEG, HumanPilot,
                       HoverTurnPilot, clip, inches_to_axes,
                       skid_ft as skid_ft_of, trims_for, wrap)
from airframe import FOOT
from main import AIRFRAME_PRESET
from simulation import (PAD_COLLECTIVE, SIM_TIME_STEP_S, Simulation,
                        UH1_CG_HEIGHT_ON_GROUND, airframe_preset)

#: The transcript, written by the run itself: the shell's own redirection is not
#: worth trusting on this box.  ``_rt_handling.Transcript`` is what writes it, so
#: a line is on the console as it is written and in the file afterwards.
OUT_PATH = "_rt_pilot_hand.txt"

#: The two clocks the person is flown at, s: the instrument's own twenty
#: decisions a second, which is the instrument as the two phases before this one
#: fly it, and :data:`_rt_handling.DELAY_S` - one decision a delay, the slowest
#: hand that instrument will fly at all.  See :func:`same_ruler` for the third
#: clock, the mission's own, which is asserted to be one a hand cannot be given.
CLOCKS = (PILOT_PERIOD_S, DELAY_S)

#: What each of the two clocks is, in the transcript's own words.
CLOCK_WORDS = {
    PILOT_PERIOD_S: "the instrument's own, which is the two phases before this",
    DELAY_S: "the slowest a hand may be: one decision a delay",
}

#: How often a line of a person's flight is printed, s: ``_rt_check.py``'s own
#: once a second - both of these missions are flown through the same frame loop
#: and this transcript is read beside that file's.
REPORT_S = 1.0

#: The seconds each mission is allowed, which are ``_rt_check.py``'s own two
#: limits: the short mission is a minute and its rehearser gives it three, and
#: the long one is a minute and a half of flying at four.  A person's flight is
#: not given longer than the script's, for the same reason the script is not: a
#: mission that needs longer than this has failed, and the transcript should say
#: where rather than fly on.
LIMITS = {"hover-turn": 180.0, "cruise": 250.0}

#: The cyclic's own band: the mission's two tilt pairs scaled up from its own
#: until a mission is not flown and down until one is not, which is phase 1's and
#: phase 3's own shape - a band and not a number.  It is the cyclic that is scaled
#: because phase 2's finding is that the hover's margin is the cyclic's margin and
#: nothing else, and what is measured here is whether that stays true *inside* the
#: missions, with the turn, the speed ramp and the kilometre in them.
SCALES_HARDER = (1.25, 1.5, 2.0, 3.0)
SCALES_SOFTER = (0.75, 0.5, 0.35, 0.25)

#: How far off the pad the short mission may wander and still be that mission, m:
#: ``_rt_check.check``'s own ten metres, which is the allowance a 360 flown on
#: this model's cyclic is given - the model couples lateral cyclic into yaw, so a
#: turn that does not hold station walks.
HOVER_DRIFT_LIMIT_M = 10.0

#: What the short mission's landing has to arrive under, m/s:  ``_rt_check.check``
#: asserts ``run.touchdown_rate < 1.0`` of its own scripted flight, so one metre a
#: second is the *mission's* claim and not the floor's - ``UH1_HARD_LANDING_MPS``
#: is the rate a UH-1 is not permitted to be landed at, which is a different and
#: much larger number, and a hand that arrives between the two has flown the
#: mission and broken its own landing rather than wrecking the aircraft.
HOVER_TOUCHDOWN_LIMIT_MPS = 1.0


def say(stream, text):
    """A line for the transcript: the file and the console, once each.

    The stream is :class:`_rt_handling.Transcript`, so it is the stream that
    writes to both and this is only the line's own newline - which is why this
    file has no ``say`` of its own to keep in step with anybody else's.
    """
    stream.write(text + "\n")
    stream.flush()


class Hands(Instrument):
    """A person's hands on one mission's own four loops, at an instrument's clock.

    The instrument to the last line - the twenty hertz clock on the flight's own
    time, the ``DELAY_S`` of delay read off a deque of remembered states, the lag
    in the hand and the model's own limb rate underneath it, :meth:`fill`,
    :meth:`view`, :meth:`push`, :meth:`move` and the clock's own ``selftest`` as
    phase 1 wrote them - with three things changed and nothing else:

    * :meth:`decide` is the mission's own, loop for loop and branch for branch,
      read off the *view* rather than off the aircraft.  The instrument's own
      hover loops - its ``k_pitch``, its ``k_roll``, its lever and its pedals -
      are computed and left where they are: what this file flies is the mission's
      own pairs, scaled, for the module's own reason, and the numbers in those
      pairs are that module's constants rather than copies of them;
    * the phases are the mission's own machine, and they are read off the view
      too, so a phase is named a fifth of a second late;
    * and the baselines are the *model's* own: the rows of
      ``_rt_pilot.trims_for``, built off a scratch aircraft at the speeds and the
      height the mission class itself names, and - for the phases where the
      aircraft is on the ground - the pad's own sticks, read off
      ``simulation.Simulation.trim_controls`` at ``simulation.PAD_COLLECTIVE``.
      Nothing here is written down twice.
    """

    #: The mission's name, which is the key ``_rt_check.py``'s own ``MISSIONS``
    #: is read with, and the condition the instrument is in: a mission that
    #: begins and ends on the pad is a hover and one that ends a kilometre away
    #: is not.
    NAME = None
    CONDITION = HOVER

    def __init__(self, sim, trims, scales=SCALES_ONE, **kwargs):
        # The instrument's own baseline trim is the pad's own sticks, in the
        # sticks' own inches, and not the mission's flight trim: a mission begins
        # on the pad, where ``on_the_pad`` leaves the controls, and a person's
        # hands begin where the pad has them - which is where the script's own
        # ``KeyHands`` begins too, since it starts from the same
        # ``sim.pilot.axes()``.  The flight trim each decision is written around
        # is read off ``trims`` inside :meth:`decide`, which is where the mission
        # reads it.
        pad = sim.trim_controls.collective_at(PAD_COLLECTIVE)
        self.pad_inches = [pad.collective, pad.long_stick, pad.lat_stick,
                           pad.pedal]
        Instrument.__init__(self, sim, self.pad_inches, scales=scales,
                            condition=self.CONDITION, **kwargs)
        #: The model's own trims, and the pad's own sticks in the two forms the
        #: mission holds them in: the inches above, and the four axes the frame
        #: loop answers in, which is what the mission's own ``pad_axes`` is.
        self.trims = trims
        self.pad_axes = list(pad.to_axes())
        self.done = None
        self.took_off = None
        #: The frame the decision being made is on, in the flight's own clock.
        #: Every instant this file records is stamped with it - see
        #: :meth:`HoverTurnHands.phase_of` for the two clocks a phase machine
        #: runs on here, and which of them is which.
        self.now = float(sim.sim_time)

    # -- the frame loop's view of it ---------------------------------------

    def frame(self, sim, dt):
        """One frame of a person flying: the instrument's own, and the clock.

        ``_rt_handling.Hands.frame`` to the last line - look when the clock is
        due, off the oldest state remembered, push this frame's state into the
        history *after* the decision, then the lag in the hand and the model's
        own limb rate underneath it - with the one thing this file's phase
        machines need and the instrument's hover has no use for: the frame's own
        instant, kept in the flight's clock where :meth:`phase_of` can read it.

        That instant is the whole of what makes a phase machine work here.  A
        schedule is not a reading and is stepped *between decisions*, so it needs
        a clock the pilot is living in and not the frame rate; and the instant a
        phase or an arrival is *recorded* at is the same one, which is what lets
        every number in this transcript be read against a mission's own row.
        """
        self.now = float(sim.sim_time)
        return Instrument.frame(self, sim, dt)

    # -- what the mission's own phase machine is ---------------------------

    def pairs(self):
        """The mission's own gains, each under the instrument's scale.

        Returns ``(what it is, the hand's gain, the mission's gain, the scale)``
        per loop, so that :meth:`selftest` can assert the whole of what this file
        flies against the module it was read off - and so that a gain moved in
        ``_rt_pilot`` moves this file's hand with it, since the *pair* of them
        is scaled rather than either one alone.  A pair scaled together also
        keeps the lead the loop is closed at at the mission's own ratio, which is
        phase 1's own rule for the cyclic and phase 2's for the pedals.
        """
        raise NotImplementedError

    def phase_of(self, view):
        """What the person is doing, from what the instruments show."""
        raise NotImplementedError

    def decide(self, view):
        """Where the four axes ought to be, in inches, from what was seen."""
        raise NotImplementedError

    def mission_over(self, sim):
        """Is the mission over?  The hands come off everything when it is."""
        raise NotImplementedError

    def note(self, sim):
        """The person's own words for a caption: what the mission is doing."""
        raise NotImplementedError

    def selftest(self):
        """The instrument's own check, and this file's four pairs of gains.

        Phase 1's and phase 2's own assertions first, unchanged, because the
        clock, the delay, the lag and the four scaled loops the base class builds
        are still exactly what they claim.  Then the pairs *this* file flies,
        each against the mission's own constant and the instrument's scale it is
        made of: a hand that is not its mission's fails here rather than flying a
        loop no pilot in the project has, which is the check every phase of this
        programme ends on.
        """
        Instrument.selftest(self)
        for what, hand, gain, scale in self.pairs():
            assert abs(hand - gain * scale) < 1e-12, (what, hand, gain, scale)
        return self


class HoverTurnHands(Hands):
    """The short mission's own four loops, flown by a person's hands.

    ``_rt_pilot.HoverTurnPilot``'s mission, phase for phase and gain for gain -
    the lever through a climb rate to the band of twenty feet and back down on a
    creep, the cyclic holding the machine over the pad, the pedals carrying the
    heading round the schedule - with every reading taken out of the view, the
    five phases the mission's own, and the two baselines the model's:

    * the hover's row, ``trims.at(0.0)``, which is the trim the mission's whole
      cyclic and lever are written around and the attitude ``pitch_t`` the nose
      is held at over the pad - read at the speeds and the height
      :class:`_rt_pilot.HoverTurnPilot` itself names, since the table is *its*
      table and not this file's;
    * and the pad's own sticks for the phases at either end of the mission: on
      the ground before the pickup, and again once the skids are down.
    """

    NAME = "hover-turn"
    CONDITION = HOVER

    def __init__(self, sim, trims, scales=SCALES_ONE, **kwargs):
        Hands.__init__(self, sim, trims, scales=scales, **kwargs)
        # The four pairs, each the mission's own under one of the three scales:
        # the cyclic's two on the tilt scale, the lever's two on its own - the
        # climb loop and the descent's creep are two gains of the same control -
        # and the turn's pair on the pedals'.
        self.hand_pitch_in = K_HOVER_PITCH_IN * self.scales.theta
        self.hand_pitch_rate_in = K_HOVER_RATE_IN * self.scales.theta
        self.hand_roll_in = K_HOVER_ROLL_IN * self.scales.theta
        self.hand_roll_rate_in = K_HOVER_RATE_IN * self.scales.theta
        self.hand_lever_in = K_RATE_IN * self.scales.lever
        self.hand_creep_in_s = LEVER_CREEP_IN_S * self.scales.lever
        self.hand_turn_in = K_TURN_IN * self.scales.pedal
        self.hand_turn_rate_in = K_TURN_RATE_IN * self.scales.pedal
        # The mission's own bookkeeping, all of it off the view: the phases and
        # when each began, the band of twenty feet and when it was last in it,
        # the turn's schedule and its three-part end, the descent's own lever,
        # the last rate read off the ground, the arrival, and the trundle of
        # settling on the skids.
        self.phase = "pickup"
        self.hover_at = None
        self.hover_alt = None
        self.in_band_since = None
        self.turn_begun = None
        self.turn_at = None
        self.turn_heading_deg = float(sim.telemetry().yaw_deg)
        self.turn_settled = None
        self.turn_done = None
        self.turned_deg = 0.0
        self.turn_error_deg = None
        self.descent_at = None
        self.descent_lever = None
        self.last_airborne_rate = 0.0
        self.touchdown = None
        self.touchdown_rate = None
        self.landed_since = None

    def pairs(self):
        """See :meth:`Hands.pairs`: the cyclic's two pairs, the lever's two gains
        and the turn's pair, which is what phase 2's three scales are for."""
        return (("the cyclic's pitch", self.hand_pitch_in, K_HOVER_PITCH_IN,
                 self.scales.theta),
                ("the cyclic's pitch rate", self.hand_pitch_rate_in,
                 K_HOVER_RATE_IN, self.scales.theta),
                ("the cyclic's roll", self.hand_roll_in, K_HOVER_ROLL_IN,
                 self.scales.theta),
                ("the cyclic's roll rate", self.hand_roll_rate_in,
                 K_HOVER_RATE_IN, self.scales.theta),
                ("the lever's climb rate", self.hand_lever_in, K_RATE_IN,
                 self.scales.lever),
                ("the lever's descent creep", self.hand_creep_in_s,
                 LEVER_CREEP_IN_S, self.scales.lever),
                ("the turn's heading", self.hand_turn_in, K_TURN_IN,
                 self.scales.pedal),
                ("the turn's rate", self.hand_turn_rate_in, K_TURN_RATE_IN,
                 self.scales.pedal))

    def phase_of(self, view):
        """What the person is doing, from what the instruments show.

        ``HoverTurnPilot.phase_of``'s own sequence - the pickup, the band of
        twenty feet and its settling, the turn's schedule with its three-part
        end, the descent, the skids and the lightness on them again - read off
        the *view* rather than off the aircraft, so a phase is named the fifth of
        a second after the aircraft entered it.

        Two clocks, and they are not the same one, which is phase 3's own
        distinction and worth the same sentence here.  The *evidence* is the
        view's: the aircraft a person is looking at is the aircraft as it was,
        and a dwell - the band held for its settling, the nose settled on its
        heading - is a thing a person waits out in their own life, so an interval
        is measured on ``when``.  The *schedule* the turn marches round on is not
        a reading at all: it is a number the pilot is carrying, and it is stepped
        between decisions on the flight's own clock, which is what the mission
        steps it on too.  And every instant *recorded* - the phases, the hover,
        the turn's own end, the arrival - is the frame the decision was made on,
        the flight's own clock, so that every number here can be read against the
        mission's own row.
        """
        telemetry = view.telemetry
        alt = telemetry.alt_agl
        when = view.time
        stamp = self.now
        if self.took_off is None and not telemetry.on_ground:
            self.took_off = stamp
        if self.phase == "pickup":
            self.phase = "hover"                  # the loops do the pickup
        if self.phase == "hover":
            if abs(alt - HOVER_ALT_M) > HOVER_TOLERANCE_M:
                # Not in the band: whatever it is doing, it is not hovering
                # twenty feet up, so the settling starts again from wherever the
                # next entry into the band is.
                self.in_band_since = None
                self.hover_alt = None
            elif self.in_band_since is None:
                self.in_band_since = when
                self.hover_alt = alt
                if self.hover_at is None:
                    self.hover_at = stamp
            elif when - self.in_band_since >= HOVER_SETTLE_S:
                self.phase = "turn"
                self.turn_begun = stamp
                self.turn_heading_deg = telemetry.yaw_deg
        if self.phase == "turn":
            # The schedule: the heading asked for marching round to the right at
            # the turn's own rate, eased over its last degrees, stepped once per
            # decision on the flight's own clock - see the docstring above.
            if self.turn_at is None:
                self.turn_at = stamp
            turn_rate = TURN_RATE_DEG_S * clip(
                (360.0 - self.turned_deg) / TURN_LEAD_DEG, TURN_LEAD_FLOOR, 1.0)
            self.turned_deg = min(360.0, self.turned_deg
                                  + turn_rate * (stamp - self.turn_at))
            self.turn_at = stamp
            heading_want = self.turn_heading_deg + self.turned_deg
            self.turn_error_deg = wrap(heading_want - telemetry.yaw_deg)
            if (self.turned_deg >= 360.0
                    and abs(self.turn_error_deg) <= TURN_TOLERANCE_DEG
                    and abs(telemetry.yaw_rate_deg)
                    <= TURN_RATE_TOLERANCE_DEG_S):
                if self.turn_settled is None:
                    self.turn_settled = when
                elif when - self.turn_settled >= TURN_SETTLE_S:
                    self.phase = "descent"
                    self.turn_done = stamp
                    self.descent_at = stamp
            else:
                self.turn_settled = None
        if self.phase in ("pickup", "hover", "turn", "descent"):
            # What only a frame can see, and what the arrival on the pad has to
            # be judged by: the height rate the pilot last read in the air.
            self.last_airborne_rate = telemetry.height_rate
        if self.phase == "descent" and telemetry.on_ground:
            self.phase = "landed"
            self.touchdown = stamp
            self.touchdown_rate = self.last_airborne_rate
            self.landed_since = stamp
        if self.phase == "landed" and not telemetry.on_ground:
            # Light on its skids again - the lever is still up where the descent
            # left it - so the descent is not over after all.
            self.phase = "descent"
            self.landed_since = None

    def decide(self, view):
        """Where the four axes ought to be, in inches, from what was seen.

        ``HoverTurnPilot.decide`` one loop for the whole mission, branch for
        branch and number for number, with three substitutions and no arithmetic
        of its own:

        * every reading comes out of *view* - the aircraft as it was ``DELAY_S``
          ago - and the phase machine above is stepped first, off that same view,
          which is where the mission's own ``decide`` steps its own;
        * the eight gains are this file's own pairs, which are that module's at a
          scale of one: the cyclic's four on ``scales.theta``, the lever's two on
          ``scales.lever`` and the turn's two on ``scales.pedal`` - see
          :meth:`pairs` for the eight, and :func:`same_ruler` for the assertion
          that a scale of one is the mission's own pilot;
        * and the descent's lever is integrated over *this* pilot's clock rather
          than over the mission's :data:`_rt_pilot.REACTION_S`, since it is a
          rate a second and a hand's seconds are not a script's.

        What is *not* scaled is what is not a gain: the turn's schedule is fed
        forward off the mission's own ``PEDAL_PER_DEG_S``, the height is asked for
        through its own ``RATE_GAIN_MPS`` and its three metre a second cap, and
        the tilt the machine is held over the pad with is the model's own pair of
        gains.  A pilot may grip a loop harder than the model does and may not fly
        a different one, which is phase 2's own rule for the lever.
        """
        self.phase_of(view)
        telemetry = view.telemetry
        alt = telemetry.alt_agl
        collective_t, long_t, lat_t, pedal_t, pitch_t = self.trims.at(0.0)
        if self.phase == "landed":
            # Down and staying down: the lever back to the pad's own and the
            # cyclic and the pedals back on the trim, which is the state a
            # mission begins from - ``on_the_pad``'s own sticks.
            return list(self.pad_inches)

        # Vertical: the height asked for, and the rate to get there.  Climbing is
        # the hover's own loop, clipped like a hand.  Coming down is the lever
        # *creeping* down - and up - until the VSI reads what the mission asked
        # for, which is a rate loop around the lever rather than a height loop
        # around the cushion: inside the ground effect the height a lever buys is
        # nothing like the height it buys in free air, so the loop is closed on
        # the sink and the pad is what stops the aircraft.
        if self.phase in ("pickup", "hover", "turn"):
            rate_want = clip(RATE_GAIN_MPS * (HOVER_ALT_M - alt),
                             -CLIMB_LIMIT_MPS, CLIMB_LIMIT_MPS)
            collective = (collective_t
                          + self.hand_lever_in
                          * (rate_want - telemetry.height_rate))
        else:
            rate_want = -DESCENT_RATE_MPS * clip(
                (alt - UH1_CG_HEIGHT_ON_GROUND) / DESCENT_TAPER_M,
                DESCENT_TAPER_FLOOR, 1.0)
            if self.descent_lever is None:
                self.descent_lever = collective_t     # the descent's own start
            self.descent_lever = clip(
                self.descent_lever - self.hand_creep_in_s * self.period
                * (telemetry.height_rate - rate_want),
                collective_t - LEVER_LIMIT_IN, collective_t + LEVER_LIMIT_IN)
            collective = self.descent_lever

        # The cyclic: the machine held over the pad, which is the whole of how a
        # hover is kept - at zero airspeed this model has nothing else that stops
        # a machine.  Where it has drifted to is taken in the aircraft's *own*
        # axes, the nose being what turns and the pad what does not, and answered
        # with a tilt towards the pad and against the drift still to be taken out.
        # It is flown through the turn as well, and has to be, since the model
        # couples lateral cyclic into yaw.  ``view.vx`` is forward and ``.vy``
        # starboard, exactly as the mission reads its own state.
        heading = math.radians(telemetry.yaw_deg)
        north_off = HOVER_REF_NORTH_M - view.north
        east_off = HOVER_REF_EAST_M - view.east
        forward_off = (north_off * math.cos(heading)
                       + east_off * math.sin(heading))
        right_off = (-north_off * math.sin(heading)
                     + east_off * math.cos(heading))
        pitch_want = clip(pitch_t - K_HOVER_POS_DEG_M * forward_off
                          + K_HOVER_VEL_DEG_MPS * view.vx,
                          pitch_t - HOVER_TILT_LIMIT_DEG,
                          pitch_t + HOVER_TILT_LIMIT_DEG)
        roll_want = clip(K_HOVER_POS_DEG_M * right_off
                         - K_HOVER_VEL_DEG_MPS * view.vy,
                         -HOVER_TILT_LIMIT_DEG, HOVER_TILT_LIMIT_DEG)
        long_stick = (long_t
                      + self.hand_pitch_in * (telemetry.pitch_deg - pitch_want)
                      + self.hand_pitch_rate_in * telemetry.pitch_rate_deg)
        lat_stick = (lat_t
                     + self.hand_roll_in * (roll_want - telemetry.roll_deg)
                     - self.hand_roll_rate_in * telemetry.roll_rate_deg)

        # And the pedals: the rate the turn wants, fed forward off the mission's
        # own inches per degree a second and coming off over the last few degrees
        # of the turn, then the heading the schedule is carrying the nose towards
        # and the rate still wanted to get there.  Once the schedule is home the
        # pedals are a heading hold like any other, since a 360 that does not stop
        # is a 361.
        heading_want = self.turn_heading_deg + self.turned_deg
        yaw_rate_want = 0.0
        if self.phase == "turn":
            yaw_rate_want = TURN_RATE_DEG_S * clip(
                (360.0 - self.turned_deg) / TURN_LEAD_DEG, 0.0, 1.0)
        pedal = (pedal_t
                 + PEDAL_PER_DEG_S * yaw_rate_want
                 + self.hand_turn_in * wrap(heading_want - telemetry.yaw_deg)
                 + self.hand_turn_rate_in
                 * (yaw_rate_want - telemetry.yaw_rate_deg))
        return [collective, long_stick, lat_stick, pedal]

    # -- the frame loop's view of it --------------------------------------

    def mission_over(self, sim):
        """The pad has the aircraft and the controls again, and keeps them.

        ``HoverTurnPilot.mission_done``'s own rule - the skids down, the lever
        back at the pad's own, the trundle of settling done, and the sticks
        within one frame of a key of where the pad has them, held for
        :data:`_rt_pilot.LANDING_SETTLE_S` - with the one substitution this file
        makes everywhere else: the phase and the landing are read off the view,
        so both are a fifth of a second late, while the sticks are the aircraft's
        own, since a hand knows where it put them.  A wreck is over too.
        """
        if self.done is None:
            if sim.crashed:
                self.done = sim.sim_time
            elif self.phase == "landed" and sim.on_ground:
                axes = sim.pilot.axes()
                on_the_pad = all(abs(axis - want) <= step
                                 for axis, want, step
                                 in zip(axes, self.pad_axes, KEY_STEP))
                if (on_the_pad and self.landed_since is not None
                        and sim.sim_time - self.landed_since
                        >= LANDING_SETTLE_S):
                    self.done = sim.sim_time
        return self.done is not None

    def note(self, sim):
        """Where the person is in the mission, in their own words.

        The mission's own captions, which are about the aircraft rather than
        about whose hands are on it, so that this transcript can be read beside
        that file's - with the decisions counted, which is the person's own clock
        showing rather than the mission's.
        """
        telemetry = sim.telemetry()
        skid = skid_ft_of(telemetry)
        if sim.crashed:
            return ("%s at %.1f s, %s"
                    % (self.phase, sim.sim_time, telemetry.crash_message))
        if self.done is not None:
            return ("flown in %.0f s: %.0f ft up over the pad, %.0f deg round to"
                    " the right, down again at %.0f fpm"
                    % (self.done, HOVER_SKID_FT, self.turned_deg,
                       self.touchdown_rate / FOOT * 60.0))
        if self.phase == "pickup":
            return ("picking up off the pad, %4.1f ft up, %d decisions"
                    % (skid, self.decisions))
        if self.phase == "hover":
            return ("climbing to %.0f ft, %4.1f ft up, %+5.0f fpm,"
                    " %d decisions" % (HOVER_SKID_FT, skid,
                                       telemetry.height_rate_fpm,
                                       self.decisions))
        if self.phase == "turn":
            return ("the turn, %.0f of 360 deg to the right, %+5.0f deg/s,"
                    " %4.1f ft up, %d decisions"
                    % (self.turned_deg, telemetry.yaw_rate_deg, skid,
                       self.decisions))
        if self.phase == "descent":
            return ("coming down, %4.1f ft up, %+5.0f fpm, %d decisions"
                    % (skid, telemetry.height_rate_fpm, self.decisions))
        return ("down on the pad, lever coming back to the pad's own,"
                " %d decisions" % (self.decisions,))

class CruiseHands(Hands):
    """The long mission's own four loops, flown by a person's hands.

    ``_rt_pilot.HumanPilot``'s mission - a hundred metres up and a kilometre out -
    flown the way that mission is: on a trim row *per speed*, so the baseline
    moves with the aircraft as the speed is wound on, and it is read off
    ``trims.at(speed)`` in the same line the mission reads it in.

    The one phase the mission opens a branch on is the pad, and it is worth it:
    on the skids the lever is the only control with any weight on it - the skids
    hold the aircraft level and still however much the cyclic is waved about - so
    a pickup is the collective's doing and the rest of the aircraft comes along
    with it.  The person gets the same branch, in the same units the mission's
    ``hover_fraction`` states it in: the hover's own collective, and the pad's own
    sticks for the other three.
    """

    NAME = "cruise"
    CONDITION = CRUISE

    def __init__(self, sim, trims, scales=SCALES_ONE, cruise_mps=CRUISE_MPS,
                 **kwargs):
        Hands.__init__(self, sim, trims, scales=scales, **kwargs)
        self.cruise_mps = float(cruise_mps)
        # The four pairs: the cyclic's two on the tilt scale, the lever's on its
        # own and the pedals' on the third - the same three scales phase 2 flew
        # this mission's own hover loops with.
        self.hand_pitch_in = K_PITCH_IN * self.scales.theta
        self.hand_pitch_rate_in = K_PITCH_RATE_IN * self.scales.theta
        self.hand_roll_in = K_ROLL_IN * self.scales.theta
        self.hand_roll_rate_in = K_ROLL_RATE_IN * self.scales.theta
        self.hand_lever_in = K_RATE_IN * self.scales.lever
        self.hand_pedal_in = K_YAW_IN * self.scales.pedal
        self.hand_pedal_rate_in = K_YAW_RATE_IN * self.scales.pedal
        # The mission's own bookkeeping, off the view: when the skids left the
        # pad, when the climb met its own band, and the hand-over - which is
        # :attr:`done`, the same attribute the frame loop reads.
        self.climbed = None

    def pairs(self):
        """See :meth:`Hands.pairs`: the long mission's own seven gains."""
        return (("the cyclic's pitch", self.hand_pitch_in, K_PITCH_IN,
                 self.scales.theta),
                ("the cyclic's pitch rate", self.hand_pitch_rate_in,
                 K_PITCH_RATE_IN, self.scales.theta),
                ("the cyclic's roll", self.hand_roll_in, K_ROLL_IN,
                 self.scales.theta),
                ("the cyclic's roll rate", self.hand_roll_rate_in,
                 K_ROLL_RATE_IN, self.scales.theta),
                ("the lever's climb rate", self.hand_lever_in, K_RATE_IN,
                 self.scales.lever),
                ("the pedals' heading", self.hand_pedal_in, K_YAW_IN,
                 self.scales.pedal),
                ("the pedals' rate", self.hand_pedal_rate_in, K_YAW_RATE_IN,
                 self.scales.pedal))

    def phase_of(self, view):
        """What the person is doing, from what the instruments show.

        ``HumanPilot.mission_done``'s own three instants - off the skids, into
        the band of a hundred metres, and the kilometre - read off the *view*:
        the skids and the altitude off telemetry a fifth of a second old, and the
        distance out off the position the same view remembers.  A phase is
        therefore named a fifth of a second after the aircraft earned it, which
        is the whole of what this leg of the file measures.
        """
        telemetry = view.telemetry
        if self.took_off is None and not telemetry.on_ground:
            self.took_off = self.now
        if (self.climbed is None
                and telemetry.alt_agl >= ALT_REF_M - ALT_TOLERANCE_M):
            self.climbed = self.now
        if (self.done is None and self.climbed is not None
                and math.hypot(view.north, view.east) >= DIST_REF_M):
            self.done = self.now

    def decide(self, view):
        """Where the four axes ought to be, in inches, from what was seen.

        ``HumanPilot.decide`` loop for loop, with the two substitutions this file
        makes everywhere - every reading out of *view*, which the phase machine
        above is stepped on first, and the seven gains this file's own pairs,
        which are that module's at a scale of one - and the mission's one branch,
        stated in the units the mission states it in: on the pad the lever is
        asked for the hover's own collective, which is what the mission's own
        ``hover_fraction`` is, and the other three axes are left where
        ``on_the_pad`` put them.
        """
        self.phase_of(view)
        telemetry = view.telemetry
        alt = telemetry.alt_agl
        if telemetry.on_ground and alt < PARKED_ALT_M:
            # Still on the skids: the lever is the only axis that does anything,
            # and it is asked for the trim the aircraft hovers twenty feet on -
            # the model's own answer, read off the table the mission's own
            # ``hover_fraction`` reports.  The cyclic and the pedals stay where
            # the pad left them.
            return [self.trims.at(0.0)[0]] + self.pad_inches[1:]

        speed = telemetry.ground_speed
        speed_want = self.cruise_mps * clip((alt - CLIMB_ALT_M) / SPEED_RAMP_M,
                                            0.0, 1.0)
        # The baseline is the trim of the speed the aircraft is *at*, not of the
        # speed it is being asked for: a pilot's force trim is wound on with the
        # aircraft, so the pedals do not arrive at the cruise's own while the
        # machine is still hovering at three metres a second - a bootful of
        # nose-right the pilot would then have to take out again, and a bow out of
        # the line.  The speed asked for reaches the loops only through the pitch
        # below.
        collective_t, long_t, lat_t, pedal_t = self.trims.at(speed)[:4]

        # Vertical: a rate to ask for, and the lever position that holds it.
        rate_want = clip(RATE_GAIN_MPS * (ALT_REF_M - alt),
                         -CLIMB_LIMIT_MPS, CLIMB_LIMIT_MPS)
        collective = (collective_t
                      + self.hand_lever_in
                      * (rate_want - telemetry.height_rate))

        # Fore and aft: the pitch the speed asked for needs, then the stick for
        # it.  The attitude is aimed at the trim of the speed *asked for* - a
        # stated attitude for the nose to settle on - while the stick it is
        # measured against is the trim of the speed the aircraft is at, so the
        # wind-on is led by the nose and not by a stick position the aircraft has
        # not earned yet.  At the end of the ramp the two speeds agree and the
        # pilot is sitting on the cruise trim, hands still.
        pitch_want_t = self.trims.at(speed_want)[4]
        pitch_want = clip(pitch_want_t - K_SPEED_DEG * (speed_want - speed),
                          pitch_want_t - PITCH_LIMIT_DEG,
                          pitch_want_t + PITCH_LIMIT_DEG)
        long_stick = (long_t
                      + self.hand_pitch_in * (telemetry.pitch_deg - pitch_want)
                      + self.hand_pitch_rate_in * telemetry.pitch_rate_deg)

        # Sideways: the line north of the pad, as a bank, then the stick for it.
        east = view.east - TRACK_EAST_REF_M
        bank_want = clip(-K_BANK_DEG * east, -BANK_LIMIT_DEG, BANK_LIMIT_DEG)
        lat_stick = (lat_t
                     + self.hand_roll_in * (bank_want - telemetry.roll_deg)
                     - self.hand_roll_rate_in * telemetry.roll_rate_deg)

        # And the nose, which is the pedals' whole business.
        pedal = (pedal_t
                 - self.hand_pedal_in * wrap(telemetry.yaw_deg - YAW_REF_DEG)
                 - self.hand_pedal_rate_in * telemetry.yaw_rate_deg)
        return [collective, long_stick, lat_stick, pedal]

    # -- the frame loop's view of it --------------------------------------

    def mission_over(self, sim):
        """A hundred metres up and a kilometre out: the goal, and the hand-over.

        ``HumanPilot.mission_done``'s own two instants are in :meth:`phase_of`,
        read off the view; this is the frame loop's half of the same answer, so
        what the two differ by is the fifth of a second a person's eyes are
        behind.  A wreck is over too.
        """
        if sim.crashed and self.done is None:
            self.done = sim.sim_time
        return self.done is not None

    def note(self, sim):
        """Where the person is in the mission, in their own words.

        The mission's own three captions - parked, climbing, out to the kilometre
        - with the decisions counted, which is the person's own clock showing
        rather than the mission's.
        """
        telemetry = sim.telemetry()
        if sim.crashed:
            return ("%s at %.1f s, %s"
                    % ("parked" if self.took_off is None else "flying",
                       sim.sim_time, telemetry.crash_message))
        if self.done is not None:
            state = sim.airframe.state
            return ("mission complete at %.0f s, hands off (%.0f m out)"
                    % (self.done, math.hypot(state.position.x,
                                             state.position.y)))
        if self.took_off is None:
            return "parked, %d decisions" % (self.decisions,)
        return ("%s, %d decisions"
                % ("climbing to %.0f m" % (ALT_REF_M,)
                   if self.climbed is None
                   else "out to %.1f km" % (0.001 * DIST_REF_M,),
                   self.decisions))

# ---------------------------------------------------------------------------
# The ruler: at a scale of one, with no hand in the way, one pilot.
# ---------------------------------------------------------------------------


#: The two missions, their own pilots and this file's: read as ``(name, the
#: mission's class, the hand's class)``, so that the trims a mission is flown on
#: are built off the *mission's* class and the row this file is read against is
#: the mission's own flight.
HAND_MISSIONS = (("hover-turn", HoverTurnPilot, HoverTurnHands),
                 ("cruise", HumanPilot, CruiseHands))

#: The short mission's own instants, as ``(what it is, the phase the comparison
#: is forced into, degrees of the turn the schedule has reached)``: the band of
#: twenty feet, the turn half way round, the descent begun, and the pad again.
#: The phase is forced in *both* pilots, because what is compared here is the
#: four loops and not the machine that decides which of them is being flown - see
#: :func:`same_ruler`.
SAME_RULER_HOVER = (("the band of twenty feet", "hover", 0.0),
                    ("the turn, half way round", "turn", 180.0),
                    ("the descent", "descent", 360.0),
                    ("the pad", "landed", 360.0))

#: The long mission's own two instants, as ``(what it is, the speed the aircraft
#: is trimmed at, the altitude it is trimmed at)``, m/s and m: the climb, where
#: the speed ramp is asking for none of the cruise speed yet, and the wind-on,
#: where it is asking for half of it.  Between them they put the mission's
#: ``decide`` through both of its table reads and its whole ramp - the only two
#: lines of it that a phase does not pick between.
SAME_RULER_CRUISE = (("the climb", 0.0, 40.0), ("the wind-on", 6.0, 85.0))


def compare(stream, label, hands, inches, theirs, compared):
    """One instant's two decisions, axis for axis, or a traceback.

    What is compared is the four axes the two decisions leave, in the axes a
    ratchet moves - so this is the end of the arithmetic and not the middle of
    it - and the tolerance is a *bit* rather than a hair: two pilots that share
    every line of it agree exactly, and anything else is a second pilot.
    """
    mine = inches_to_axes(*inches)
    theirs = [float(value) for value in theirs]
    for index, (one, other) in enumerate(zip(mine, theirs)):
        assert abs(one - other) < 1e-12, (label, index, one, other)
    compared.append(label)
    return compared


def hover_turn_ruler(stream, trims):
    """The short mission's own four instants, one decision each.

    The aircraft each comparison is made on is a scratch one, parked and then
    trimmed where the instant says: the hover's own trim for the three instants
    the mission flies on it, and the pad for the fourth.  The descent is compared
    on the *hover's* trim and not on a descent, which is deliberate - the row the
    descent's creep is written around is the hover's own, and what the branch
    integrates is the error against the mission's own rate, so two pilots reading
    the same error integrate it the same way whatever the aircraft is doing.
    """
    compared = []
    for label, phase, turned in SAME_RULER_HOVER:
        sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
        sim.on_the_pad()
        if phase != "landed":
            sim.trim_level_flight(0.0, altitude=HOVER_ALT_M)
        hands = HoverTurnHands(sim, trims, delay=DELAY_FRAMES,
                               period=PILOT_PERIOD_S, t_lag=0.0)
        pilot = HoverTurnPilot(sim, trims, reaction=PILOT_PERIOD_S)
        for one in (hands, pilot):
            one.phase = phase
            one.turned_deg = turned
            one.descent_lever = None
        hands.now = float(sim.sim_time)
        compare(stream, "the short mission at %s" % (label,), hands,
                hands.decide(hands.snapshot(sim)), pilot.decide(sim), compared)
    return compared


def cruise_ruler(stream, trims):
    """The long mission's own two instants, one decision each.

    The aircraft is trimmed where the instant says - level flight at a speed and
    an altitude - and the pad is not one of the instants: the mission hands back
    the aircraft's own axes there while this pilot asks for the pad's, and on the
    pad those are the same four numbers by a fact about where the aircraft is and
    not by anything about the loops.
    """
    compared = []
    for label, speed, altitude in SAME_RULER_CRUISE:
        sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
        sim.on_the_pad()
        sim.trim_level_flight(speed, altitude=altitude)
        hands = CruiseHands(sim, trims, delay=DELAY_FRAMES,
                            period=PILOT_PERIOD_S, t_lag=0.0)
        pilot = HumanPilot(sim, trims, reaction=PILOT_PERIOD_S)
        hands.now = float(sim.sim_time)
        compare(stream, "the long mission at %s" % (label,), hands,
                hands.decide(hands.snapshot(sim)), pilot.decide(sim), compared)
    return compared


def same_ruler(stream):
    """The claim every number below rests on: at a scale of one, one pilot.

    Both missions, at instants of each that the mission's own ``decide`` reads
    differently, and at each of them one decision by this file's hand and one by
    the mission's own pilot, on the same aircraft, with the person's own dynamics
    switched off: the view handed to the decision *is* the aircraft rather than a
    memory of it, the hand's lag is zero, the scale is one, and both pilots are
    given the *same* clock - the instrument's own - because the clock is not what
    is being compared and the descent's creep is a rate integrated over it.

    What the two *cannot* share is asserted here as well, because it is the
    reason the rows below are flown at the clock they are: the mission's own
    script decides every :data:`_rt_pilot.REACTION_S`, which is longer than the
    delay a decision waits on the eyes, and the instrument's own ``selftest``
    refuses a person who decides less often than their eyes see.

    Returns the instants it compared, for the transcript to name.
    """
    compared = []
    for name, mission, hand in HAND_MISSIONS:
        trims = trims_for(airframe_preset(AIRFRAME_PRESET), mission)
        if name == "hover-turn":
            compared += hover_turn_ruler(stream, trims)
        else:
            compared += cruise_ruler(stream, trims)
    say(stream, "the ruler, checked: the two missions' own pilots and this file's"
        " hands, one decision each on the same aircraft at a scale of one with no"
        " lag in the hand and the aircraft itself handed to the decision, agree on"
        " all four axes to within 1e-12 of an inch - %s - the lag and the view"
        " being the only two things this file adds to a decision"
        % ("; ".join(compared),))
    # And the clock the two pilots cannot share, asserted rather than quoted.
    refused = HoverTurnHands(
        Simulation(airframe=airframe_preset(AIRFRAME_PRESET)),
        trims_for(airframe_preset(AIRFRAME_PRESET), HoverTurnPilot),
        period=REACTION_S)
    try:
        refused.selftest()
    except AssertionError as refusal:
        assert refusal.args[0] == (REACTION_S, DELAY_S), refusal.args
    else:
        raise AssertionError("the instrument flew a clock its own selftest"
                             " refuses: %.2f s" % (REACTION_S,))
    say(stream, "the missions' own clock, checked: the %.2f s between their own"
        " decisions is longer than the %.2f s a decision waits on the eyes, and"
        " the instrument's own selftest refuses it to the last decimal - so a"
        " person is flown at the delay, which is the slowest hand the model has,"
        " and the clock the missions' own numbers are committed at is the one"
        " clock a hand cannot be given" % (REACTION_S, DELAY_S))
    return compared


# ---------------------------------------------------------------------------
# A mission flown by hand, and what the frame loop saw of it.
# ---------------------------------------------------------------------------


class Trip(object):
    """What one flight of a mission measured, from the frame loop's own seat.

    ``_rt_check.Run``'s own record, with the one thing a hand has not got taken
    out of it: there is no key to count, since a person's hand is not a scancode,
    so what is counted instead is how far from the *pad* each of the four axes was
    ever asked to go, in the sticks' own inches - which is the units the loops are
    written in, and what a key census is a coarse statement of.

    The rest is the frame loop's half of a mission's own claims: the band the
    altitude stayed inside, how far off the pad the aircraft was ever taken, and
    the rate the skids arrived at - taken while the aircraft is still in the air,
    since the frame that puts it down is the one that zeroes it.

    What a frame cannot see is left where the mission keeps it.  The phases, the
    turn's own end, the arrival and the hand-over are the pilot's bookkeeping and
    are read off the view, so they are read off ``hands`` and not copied here.
    """

    def __init__(self, sim, hands):
        self.sim = sim
        self.hands = hands
        self.frames = 0
        self.lowest = sim.airframe.state.altitude
        self.highest = self.lowest
        self.worst_drift = 0.0
        self.asked = [0.0, 0.0, 0.0, 0.0]     # in, at their furthest from the pad
        self.worst_line = 0.0                 # m, the furthest off the line north
        self.last_airborne_rate = 0.0         # m/s, down positive, in the air
        self.on_ground = sim.on_ground
        self.touchdown_rate = None            # m/s, the rate the skids arrived at

    def measure(self):
        """One frame's worth, from the state the frame loop has just left."""
        telemetry = self.sim.telemetry()
        state = self.sim.airframe.state
        self.frames += 1
        self.lowest = min(self.lowest, telemetry.alt_agl)
        self.highest = max(self.highest, telemetry.alt_agl)
        self.worst_drift = max(self.worst_drift,
                               math.hypot(state.position.x, state.position.y))
        self.worst_line = max(self.worst_line,
                              abs(state.position.y - TRACK_EAST_REF_M))
        if self.hands.want is not None:
            for index, want in enumerate(self.hands.want):
                self.asked[index] = max(
                    self.asked[index],
                    abs(want - self.hands.pad_inches[index]))
        if not self.sim.on_ground:
            self.last_airborne_rate = -telemetry.height_rate
        elif not self.on_ground and self.touchdown_rate is None:
            self.touchdown_rate = self.last_airborne_rate
        self.on_ground = self.sim.on_ground
        assert math.isfinite(state.altitude)
        assert math.isfinite(state.velocity.x)
        assert math.isfinite(state.velocity.y)
        return self

    @property
    def skid_ft(self):
        """The highest the *skids* got, ft: what twenty feet up is measured in."""
        return (self.highest - UH1_CG_HEIGHT_ON_GROUND) / FOOT

    @property
    def distance_m(self):
        """How far from the pad the aircraft was at the end, m."""
        state = self.sim.airframe.state
        return math.hypot(state.position.x, state.position.y)

    def __str__(self):
        """The run's own summary: the band, the drift, the axes, the arrival."""
        return ("%d frames, %.1f m .. %.1f m of c.g. (%.1f ft of skid at the"
                " top), %.1f m out at the worst, touchdown %s, and the four axes"
                " asked for at their furthest from the pad: collective %4.2f in,"
                " long %4.2f, lat %4.2f, pedals %4.2f"
                % (self.frames, self.lowest, self.highest, self.skid_ft,
                   self.worst_drift,
                   ("%.0f fpm" % (self.touchdown_rate / FOOT * 60.0)
                    if self.touchdown_rate is not None else "not yet"),
                   *self.asked))

def hand_for(name):
    """The mission's own class and this file's hand for it, out of one table."""
    for mission_name, mission, hand in HAND_MISSIONS:
        if mission_name == name:
            return mission, hand
    raise ValueError("no such mission: %r" % (name,))


def flight(name, reaction=PILOT_PERIOD_S, scales=SCALES_ONE, limit=None,
           trace=True, stream=None, label=None):
    """One mission, flown by a person's hands, through the same frame loop.

    ``_rt_check.fly``'s own loop with the pilot exchanged for the instrument:
    :data:`simulation.SIM_TIME_STEP_S` at a time through
    :meth:`simulation.Simulation.fly` - which is the frame loop ``main.py``'s
    window puts a camera and a renderer around, and which is all the physics ever
    was - with the reports a second apart, in the pilot's own captions.

    The clock is the *flight's* own and not the frame's: the instrument looks
    when ``period`` seconds of flown time have gone by, whatever the frame rate
    is, so no window can make a person look more often per flown second than a
    person can.

    Returns ``(hands, trip)``: the pilot, for its own clock, its own phases and
    its own decisions, and the :class:`Trip` the frame loop measured.
    """
    mission_class, hand_class = hand_for(name)
    if limit is None:
        limit = LIMITS[name]
    trims = trims_for(airframe_preset(AIRFRAME_PRESET), mission_class)
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.on_the_pad()
    hands = hand_class(sim, trims, scales=scales, delay=DELAY_FRAMES,
                       period=reaction, t_lag=T_LAG_S)
    hands.selftest().fill(sim)
    if stream is not None:
        say(stream, "")
        say(stream, "the %s by hand, %s: %s"
            % (name, label or "the instrument's own clock and hands",
               mission_class.DESCRIPTION))
    trip = Trip(sim, hands)
    reported = 0.0
    while sim.sim_time < limit and not hands.mission_over(sim):
        hands.frame(sim, SIM_TIME_STEP_S)
        trip.measure()
        now = float(sim.sim_time)
        if trace and stream is not None and now >= reported + REPORT_S:
            reported = now
            say(stream, "  %5.1f s %s" % (now, hands.note(sim)))
    if stream is not None:
        say(stream, "  %s at %.1f s of physics: %s"
            % ("CRASHED" if sim.crashed else "ended", sim.sim_time,
               hands.note(sim)))
        say(stream, "  the person: %d decisions in %.1f s of physics (%.2f a"
            " second, the clock is %.2f s), every view %s s old"
            % (hands.decisions, sim.sim_time,
               hands.decisions / max(sim.sim_time, 1e-9), hands.period,
               words(sum(hands.ages) / len(hands.ages) if hands.ages else None,
                     "%5.3f")))
        say(stream, "  %s" % (trip,))
    return hands, trip

# ---------------------------------------------------------------------------
# The rows, read against the missions' own.
# ---------------------------------------------------------------------------


def flown_by_hand(name, hands, trip):
    """Was the mission flown?  The mission's own criteria, on a person's row.

    ``_rt_check.check``'s own criteria, mission for mission, read off a person's
    flight instead of the script's: the short one off the pad, into the band of
    twenty feet, held, the whole way round to the right with the nose back on the
    heading it began from, and down onto the pad and settled; the long one a
    hundred metres up and a kilometre out.

    The *arrival* is deliberately not part of it, which is phase 3's own split
    and worth the same sentence here: what this answers is whether a hand flew
    the mission, and not how hard it put it down - and the two are not the same
    question on this aircraft, since a hand that holds everything else can still
    arrive harder than a key does.  See :func:`arrived_softly`, which is the other
    half, and which the rows print beside this rather than inside it.

    A mission a hand did not fly is not a failure of this file - it is what the
    clock or the grip it was flown at is *worth*, which is the question - so this
    returns the verdict in the mission's own words rather than asserting it, and
    :func:`check` holds the transcript to whichever answer it gives.
    """
    if name == "hover-turn":
        if hands.hover_at is None:
            return False, "the band of twenty feet was never met"
        if hands.turn_done is None:
            return False, "the turn never came home"
        if hands.done is None:
            return False, "it never got down and settled"
        if abs(trip.skid_ft - HOVER_SKID_FT) > HOVER_TOLERANCE_M / FOOT:
            return False, ("the hover was held at %.1f ft of skid"
                           % (trip.skid_ft,))
        if hands.turn_error_deg is None:
            return False, "the schedule never reported an error"
        if abs(hands.turn_error_deg) > TURN_TOLERANCE_DEG:
            return False, ("the nose ended %+.1f deg off the heading it began on"
                           % (hands.turn_error_deg,))
        if trip.worst_drift > HOVER_DRIFT_LIMIT_M:
            return False, ("it wandered %.1f m off the pad"
                           % (trip.worst_drift,))
        return True, "flown, and landed on the pad"
    if hands.climbed is None:
        return False, "the band of a hundred metres was never met"
    if hands.done is None:
        return False, "it never got the kilometre"
    return True, "flown, a hundred metres up and a kilometre out"


def arrived_softly(name, trip):
    """Did the skids meet the pad inside the mission's own arrival rate?

    ``_rt_check.check`` asserts ``run.touchdown_rate < 1.0`` of its own scripted
    flight, so a metre a second is the *mission's* claim and not the floor's -
    see :data:`HOVER_TOUCHDOWN_LIMIT_MPS` for why the two are different numbers.
    The long mission never comes down, so its answer is ``None``: not a claim
    that was missed, but a claim it does not make.
    """
    if name != "hover-turn" or trip.touchdown_rate is None:
        return None
    return trip.touchdown_rate < HOVER_TOUCHDOWN_LIMIT_MPS


def say_row(stream, label, name, hands, trip, run, verdict):
    """One person's row: its numbers beside the mission's own for that mission.

    Every number on the left is this file's and every number on the right is
    ``_rt_check.py``'s own flight of the same mission at ``REACTION_S``, so the
    two columns differ by the clock, the eyes, the hand and the lag, and by
    nothing else.
    """
    say(stream, "  %s: %s" % (label, verdict[1]))
    if name == "hover-turn":
        say(stream, "    %s ft of skid at the top against %s, %s m off the pad at"
            " the worst against %s, %s deg of turning error against %s, an"
            " arrival of %s m/s against %s, %s decisions against %s, %s s"
            " against %s s"
            % (words(trip.skid_ft, "%5.1f"), words(run.skid_ft, "%5.1f"),
               words(trip.worst_drift, "%5.1f"), words(run.worst_drift, "%5.1f"),
               words(hands.turn_error_deg, "%+5.1f"),
               words(run.pilot.turn_error_deg, "%+5.1f"),
               words(trip.touchdown_rate, "%5.2f"),
               words(run.touchdown_rate, "%5.2f"),
               hands.decisions, run.pilot.decisions,
               words(hands.done, "%5.1f"), words(run.pilot.done, "%5.1f")))
        arrived = arrived_softly(name, trip)
        say(stream, "    the arrival: %s m/s against the mission's own %.2f - %s"
            % (words(trip.touchdown_rate, "%5.2f"), HOVER_TOUCHDOWN_LIMIT_MPS,
               "inside it" if arrived else
               "outside it" if arrived is not None
               else "never measured, the flight not being flown"))
        return
    state = run.sim.airframe.state
    say(stream, "    %.1f m up at the top against %.1f m, %.1f m out at the end"
        " against %.1f m, %.1f m off the line at the worst (which the mission's"
        " own file does not claim), %s decisions against %s, %s s against"
        " %s s"
        % (trip.highest, run.highest, trip.distance_m,
           math.hypot(state.position.x, state.position.y), trip.worst_line,
           hands.decisions, run.pilot.decisions,
           words(hands.done, "%5.1f"), words(run.pilot.done, "%5.1f")))

# ---------------------------------------------------------------------------
# The transcript: the rows, the band, and the finding.
# ---------------------------------------------------------------------------


def clock_words(reaction):
    """What a clock is, in the transcript's own words.

    The two the file flies by default have their own sentence in
    :data:`CLOCK_WORDS`; any other rate is one a caller asked for - the command
    line takes them, see :func:`demo` - and is named as the number it is.  A rate
    the model will *not* fly at gets no row at all, because the instrument's own
    ``selftest`` refuses it: :data:`_rt_handling.DELAY_S` is the line and the
    instrument draws it.
    """
    return CLOCK_WORDS.get(float(reaction),
                           "a clock asked for, %.2f s between decisions"
                           % (reaction,))


def say_the_baseline(stream, baseline):
    """The missions' own rows: what everything below is read against."""
    say(stream, "")
    say(stream, "the missions' own rows, which are ``_rt_check.check``'s own"
        " flights - the harness's rehearsal, called rather than copied, so its"
        " claims are in force here: a mission that stopped reaching its own goals"
        " would stop this file rather than quietly become the baseline")
    for name, mission, hand in HAND_MISSIONS:
        say(stream, "  %-11s %s" % (name, baseline[name]))


def say_the_rows(stream, baseline, rows):
    """The person's own rows, each beside the mission's own for that mission."""
    say(stream, "")
    say(stream, "the person's own rows, each beside the mission's own for the"
        " same mission at the %.2f s between decisions `_rt_check` commits its"
        " numbers at:" % (REACTION_S,))
    for name, reaction, hands, trip, verdict in rows:
        say(stream, "")
        say(stream, "%s at %.2f s between decisions: %s"
            % (name, reaction, clock_words(reaction)))
        say_row(stream, "the person", name, hands, trip, baseline[name], verdict)
    return rows

#: The cyclic's own ladder: the scales a mission is re-flown at, in order, either
#: side of the mission's own gains.  A scale of one is on it, and is flown, for a
#: reason worth a sentence: it is the person's own row again, at the same clock,
#: so :func:`check` can hold the ladder's x1.00 to the row it duplicates instead
#: of trusting two flights to be one flight.  The ends are wide on purpose -
#: phase 1's own hard edge was x1.00 and phase 2's cruise bands were wider still -
#: and :func:`band_by_hand` stops each way at the first scale that fails rather
#: than flying the whole ladder, since past an edge there is nothing to measure.
BAND_LADDER = (0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0)

#: What the cyclic's own ladder *found*, mission for mission - this file's own
#: result, named here rather than quoted in the prose, and asserted by
#: :func:`check`: a change to the model, the controls or the trims that moves a
#: band moves this file's claim with it and fails the run, which is what makes
#: these numbers a margin rather than a paragraph.
#:
#: The short mission's band, and the two scales either side of it that failed:
#: the model's own pilot is inside it, a grip half as hard again runs the hover's
#: height band away and a grip one and a half times as hard takes the turn out of
#: the sky - see the transcript for both failures in their own words.
HOVER_BAND = (0.75, 1.25)
HOVER_BAND_EDGES = (0.5, 1.5)

#: And the long mission's: the band, and the one scale past it.  The *soft* end
#: is the ladder's own - every half-grip down to x0.50 flew the kilometre, so
#: this mission cannot be under-gripped at anything this file flies and there is
#: no soft edge to report - while the hard edge is real and is x3.00, where the
#: demand for roll goes off the lateral stick's stop and the climb never reaches
#: the band of a hundred metres at all.
CRUISE_BAND = (0.5, 2.0)
CRUISE_BAND_EDGE = 3.0


def band_by_hand(name, ladder=BAND_LADDER, trace=False, stream=None):
    """One mission re-flown down and up the cyclic's ladder, at one clock.

    Each scale multiplies the mission's *two tilt pairs* and nothing else, which
    is phase 1's own pair-moving and the reason a scale here can be read as "the
    mission's own pilot gripping the cyclic this hard".  The clock is the
    instrument's own, since what is being measured is a grip and not a hand.

    The walk goes outward from the mission's own gains each way and *stops* at
    the first scale that fails - a scale is flown to find an edge and not to fill
    a column, and the failing scale is kept, since it is the edge.  The
    criterion at each scale is the mission's own, off :func:`flown_by_hand`: the
    ladder is a *band* because the two ends fail for different reasons, and what
    each scale's verdict is worth is in its own words and not in a number here.
    """
    flown = []
    for scale in [1.0] + [one for one in sorted(ladder, reverse=True)
                          if one < 1.0]:
        hands, trip = flight(
            name, scales=Scales(scale, SCALES_ONE.lever, SCALES_ONE.pedal),
            trace=trace, stream=stream,
            label="the cyclic at x%.2f of the mission's own" % (scale,))
        verdict = flown_by_hand(name, hands, trip)
        flown.append((float(scale), hands, trip, verdict))
        if not verdict[0]:
            break
    for scale in [one for one in sorted(ladder) if one > 1.0]:
        hands, trip = flight(
            name, scales=Scales(scale, SCALES_ONE.lever, SCALES_ONE.pedal),
            trace=trace, stream=stream,
            label="the cyclic at x%.2f of the mission's own" % (scale,))
        verdict = flown_by_hand(name, hands, trip)
        flown.append((float(scale), hands, trip, verdict))
        if not verdict[0]:
            break
    return sorted(flown, key=lambda one: one[0])


def say_the_band(stream, legs):
    """The cyclic's own band on each mission: every scale, and both edges.

    A scale is inside the band when the mission was *flown* at it, which is the
    mission's own criterion, and the two edges are the first scale either way
    that was not.  That is a band and not a number, which is phase 1's shape,
    because the two edges fail differently: a grip too hard closes the pilot's
    own loop on the pilot's own delay, and one too soft cannot hold what the outer
    loops are asking the aircraft for.
    """
    for name, flown in legs:
        say(stream, "")
        say(stream, "the cyclic's own band on the %s, the mission's own two tilt"
            " pairs scaled with nothing else, at the instrument's own clock:"
            % (name,))
        for scale, hands, trip, verdict in flown:
            say(stream, "  x%.2f: %s" % (scale, verdict[1]))
        held = [scale for scale, hands, trip, verdict in flown if verdict[0]]
        scales = [scale for scale, hands, trip, verdict in flown]
        lower = [scale for scale in scales if held and scale < min(held)]
        upper = [scale for scale in scales if held and scale > max(held)]

        def edge_words(scale):
            """What the mission said at *scale*, which is why that edge is one."""
            return [verdict[1] for one, hands, trip, verdict in flown
                    if one == scale][0]

        if not held:
            say(stream, "  so every scale this ladder flew failed, and there is no"
                " band on the %s at all" % (name,))
            continue
        say(stream, "  so the band on the %s is x%.2f to x%.2f of the mission's own"
            " cyclic gains, with x1.00 inside it - the person's own row above,"
            " flown again" % (name, min(held), max(held)))
        say(stream, "  its soft edge: %s"
            % ("x%.2f, where %s" % (max(lower), edge_words(max(lower)))
               if lower else
               "none inside x%.2f - that end of the ladder held, so the mission"
               " cannot be under-gripped at anything this file flies"
               % (min(scales),)))
        say(stream, "  its hard edge: %s"
            % ("x%.2f, where %s" % (min(upper), edge_words(min(upper)))
               if upper else
               "none inside x%.2f - that end of the ladder held"
               % (max(scales),)))
    return legs

def say_the_finding(stream, rows, legs):
    """What a hand costs the two missions, in the missions' own numbers.

    Two questions, and the rows above are the whole of the answers.  The first is
    whether each mission is flown *at all* at each of the two clocks, which is
    what says whether a clock is a hand's or is not - and it is asked of the
    mission's own criteria rather than of a number this file made up.  The second
    is whether the cyclic's own band, measured *inside* the mission, still
    contains the mission's own gains, which is phase 2's finding re-asked of an
    aircraft that is climbing, turning and winding a speed on rather than holding
    a hover.
    """
    say(stream, "")
    say(stream, "the finding, in the missions' own numbers:")
    for name, mission, hand in HAND_MISSIONS:
        mine = [row for row in rows if row[0] == name]
        say(stream, "  the %s at %s: %s"
            % (name, " and ".join("%.2f s" % (row[1],) for row in mine),
               "; ".join("%.2f s: %s" % (row[1], row[4][1]) for row in mine)))
        if name != "hover-turn":
            continue
        say(stream, "    and the one claim of its own a hand does not meet is the"
            " *arrival*: %s m/s at the two clocks, against the %.2f the mission's"
            " own rehearsal asks of itself - so the flying is flown and the"
            " putting of it down is what a hand costs, which is phase 3's own"
            " finding said of a different mission"
            % (", ".join(words(row[3].touchdown_rate, "%5.2f") for row in mine),
               HOVER_TOUCHDOWN_LIMIT_MPS))
    say(stream, "  so the cyclic's own margin *inside* the short mission is a band"
        " and not a number - x%.2f to x%.2f, with the model's own pilot inside it"
        " - and the hard edge of it is wider than phase 1's, which had the model's"
        " own pilot sitting on it; its soft edge is the same one phase 1 measured"
        " for the pitch pair.  The long mission's own band is x%.2f to x%.2f,"
        " which is phase 2's finding - the tilt pair's bands transfer out there,"
        " and *wider* - said of the mission rather than of the hover it was"
        " measured on: a cruise cannot be under-gripped at anything this file"
        " flies, and x%.2f takes the climb out of the band altogether."
        % (HOVER_BAND[0], HOVER_BAND[1], CRUISE_BAND[0], CRUISE_BAND[1],
           CRUISE_BAND_EDGE))


def check(stream, rows, legs):
    """Hold the transcript to what it can be held to, or raise.

    Two kinds of claim, and they are kept apart on purpose.  The first is a
    *claim about the instrument*, which is true of every row this file flies
    whatever the aircraft did: the clock is the clock it says, every view the
    decision read was :data:`_rt_handling.DELAY_S` old, no frame was dropped, and
    a verdict of *flown* means the aircraft was not wrecked under it - a verdict
    that is not flown is this file's own finding and is left as one.

    The second is the check that two flights are one flight: the ladder's x1.00
    is the person's own row at the same clock, and every number the two share
    must agree to the last bit, which is what says the ladder below it is the
    same pilot at a different grip rather than a different pilot.
    """
    for name, reaction, hands, trip, verdict in rows:
        sim = trip.sim
        assert not sim.dropped_steps, (name, reaction, sim.dropped_steps)
        assert abs(hands.period - reaction) < 1e-12, (hands.period, reaction)
        assert hands.decisions > 0, (name, reaction)
        assert hands.ages, (name, reaction)
        age = sum(hands.ages) / len(hands.ages)
        assert abs(age - DELAY_S) < 1e-9, (name, reaction, age)
        assert trip.frames > 0, (name, reaction)
        if verdict[0]:
            assert not sim.crashed, (name, reaction,
                                     sim.telemetry().crash_message)
    for name, flown in legs:
        for scale, hands, trip, verdict in flown:
            assert not trip.sim.dropped_steps, \
                (name, scale, trip.sim.dropped_steps)
            if verdict[0]:
                assert not trip.sim.crashed, (name, scale,
                                              trip.sim.telemetry().crash_message)
        mine = [one for one in flown if one[0] == 1.0]
        row = [one for one in rows
               if one[0] == name and abs(one[1] - PILOT_PERIOD_S) < 1e-12]
        assert len(mine) == 1 and len(row) == 1, (name, mine, row)
        again, one = mine[0], row[0]
        assert again[1].done == one[2].done, (name, again[1].done, one[2].done)
        assert again[1].decisions == one[2].decisions, \
            (name, again[1].decisions, one[2].decisions)
        assert again[2].frames == one[3].frames, \
            (name, again[2].frames, one[3].frames)
        assert again[2].highest == one[3].highest, \
            (name, again[2].highest, one[3].highest)
        assert again[2].worst_drift == one[3].worst_drift, \
            (name, again[2].worst_drift, one[3].worst_drift)
        assert again[2].touchdown_rate == one[3].touchdown_rate, \
            (name, again[2].touchdown_rate, one[3].touchdown_rate)
    # And the claims this file exists to make, held as claims: a hand flies both
    # missions at both clocks, the one thing it does not do is put the short one
    # down inside the rate that mission's own rehearsal asks of itself, and the
    # two bands are the ones named above.
    assert [row[0] for row in rows].count("hover-turn"), rows
    assert [row[0] for row in rows].count("cruise"), rows
    for name, reaction, hands, trip, verdict in rows:
        assert verdict[0], (name, reaction, verdict)
        assert arrived_softly(name, trip) is not True, \
            (name, reaction, trip.touchdown_rate)
    for name, flown in legs:
        held = [scale for scale, hands, trip, verdict in flown if verdict[0]]
        failed = [scale for scale, hands, trip, verdict in flown
                  if not verdict[0]]
        scales = [scale for scale, hands, trip, verdict in flown]
        assert abs(min(scales) - BAND_LADDER[0]) < 1e-12, (name, scales)
        if name == "hover-turn":
            assert (min(held), max(held)) == HOVER_BAND, (name, held)
            assert (min(failed), max(failed)) == HOVER_BAND_EDGES, \
                (name, failed)
        else:
            assert (min(held), max(held)) == CRUISE_BAND, (name, held)
            assert failed == [CRUISE_BAND_EDGE], (name, failed)
            assert abs(max(scales) - CRUISE_BAND_EDGE) < 1e-12, (name, scales)
    return True


def probe(reactions=CLOCKS, band=True, out_path=OUT_PATH):
    """The missions' own rows, the person's rows, and the cyclic's band.

    ``_rt_autoro_hand.probe``'s own shape: the ruler first, then the missions'
    own rows by their own file's own claims, then the person at each clock, then
    the ladder - and every line of it asserted before the transcript is closed,
    so a run that prints a verdict has proved it.

    Returns ``(baseline, rows, legs)``: the missions' own two flights as
    ``_rt_check.check`` returns them; the person's rows, as ``(name, the clock,
    the pilot, the trip, the mission's own verdict)``; and the cyclic's ladder per
    mission, as ``(name, the scales flown)`` - so that a caller can look at
    anything the transcript did not print.
    """
    stream = Transcript(out_path)
    saved = sys.stdout
    try:
        # The stream is ``sys.stdout`` for the duration, because the missions'
        # own rehearsal prints for itself: what it prints lands in this
        # transcript beside everything else, which is what makes the two rows
        # comparable at all.
        sys.stdout = stream
        say(stream, "the harness's other two missions flown by a person's hands:"
            " the hover-turn and the cruise of ``_rt_check.py``, which are the two"
            " that file flies by script and the two no hand has been put on -"
            " ``_rt_autoro_hand.py`` is the autorotation's own half of this")
        say(stream, "the person is ``_rt_axes.Hands``: %.0f decisions a second on"
            " the flight's own clock, %.1f s of transport delay read off a deque"
            " of remembered states, %.1f s of lag in the hand, and the four axes"
            " moved at the model's own limb rate rather than a key's quarter of it"
            % (PILOT_HZ, DELAY_S, T_LAG_S))
        for name, mission, hand in HAND_MISSIONS:
            trims = trims_for(airframe_preset(AIRFRAME_PRESET), mission)
            say(stream, "")
            say(stream, "the %s's own trim rows, read off the model at the speeds"
                " and the height that mission's class names - which is what the"
                " loops below are written around:" % (name,))
            for speed in trims.speeds:
                row = trims.at(speed)
                say(stream, "level trim at %5.1f m/s: collective %5.2f in, long"
                    " %+5.2f, lat %+5.2f, pedal %+5.2f, pitch %+5.2f deg"
                    % (speed, row[0], row[1], row[2], row[3], row[4]))
        same_ruler(stream)

        baseline = {}
        for name, mission, hand in HAND_MISSIONS:
            baseline[name] = scripted_check(name, REACTION_S, LIMITS[name])
        say_the_baseline(stream, baseline)

        rows = []
        for name, mission, hand in HAND_MISSIONS:
            for reaction in reactions:
                hands, trip = flight(name, reaction=reaction, stream=stream,
                                     label=clock_words(reaction))
                rows.append((name, float(reaction), hands, trip,
                             flown_by_hand(name, hands, trip)))
        say_the_rows(stream, baseline, rows)

        legs = []
        if band:
            say(stream, "")
            say(stream, "the cyclic's own band, from each mission's own gains"
                " outward each way:")
            for name, mission, hand in HAND_MISSIONS:
                legs.append((name, band_by_hand(name, stream=stream)))
            say_the_band(stream, legs)
        say_the_finding(stream, rows, legs)
        check(stream, rows, legs)
        say(stream, "")
        say(stream, "check passed: the ruler checked on both missions, the"
            " missions' own rows reproduced by their own file's own claims, every"
            " person's flight flown at the clock it claims with every view %.3f s"
            " old and no frame dropped, the ladder's x1.00 the person's own row to"
            " the last bit, and the whole of this transcript asserted."
            % (DELAY_S,))
    finally:
        sys.stdout = saved
        stream.close()
    return baseline, rows, legs


def demo():
    """The command line: which clock to fly the person at, if one is said.

    A rate the instrument will not fly at - anything longer than its own delay -
    is refused by the instrument's own ``selftest`` rather than by this file.
    """
    reactions = tuple(float(argument) for argument in sys.argv[1:]) or CLOCKS
    probe(reactions)
    print("wrote %s" % (OUT_PATH,))
    return 0


if __name__ == "__main__":
    sys.exit(demo())

