# -*- coding: utf-8 -*-
"""Scratch: the autorotation flown by a person's hands rather than by the script.

``_rt_autoro_entry.py`` measures the two ways this model can kill the shaft - the
twist grip rolled right off on its own key, and ``main.py``'s ``X``, the engine
failure - with the *mission's* pilot at the controls, and it says of itself
exactly what it is not:

    The loops are still ``_rt_pilot.py``'s, so nothing here says that a person
    could fly either entry by hand: that is the half ``TODO.md``'s ``Next`` still
    has open, and this file's job is to say what the model does *between* the two
    switches.

This file is that half.  The pilot is the human flyability programme's own
instrument - ``_rt_axes.Hands``, which is ``_rt_handling.Hands`` with the lever's
and the pedals' loops under scales of their own: twenty decisions a second on the
flight's own clock, the instruments as they read ``DELAY_S`` ago out of a deque of
remembered states, ``T_LAG_S`` between the stick asked for and the stick the hand
has, and the four axes moved at the model's own limb rate rather than at a key's
quarter of it.  What it *decides*, though, is not a hover and not a cruise: it is
``_rt_pilot.AutorotationPilot.decide``'s own mission - the same phases off the
same instruments, the same trim table, the same speed schedule, the same
autorotation trim under the descent and the same handbook flare and cushion -
read off the *view* instead of off the aircraft, so that the only things between
the mission's numbers and this file's are the clock, the eyes, the hand and the
grip.

That is the design, and why it is that design is worth a sentence: the loops are
the mission's own and *not* the instrument's hover pilot's, because the question
is whether a person can fly the entry *the mission flies*.  A file that flew this
mission on the hover's loops would be measuring a different pilot, and could not
say which of the two numbers had moved.  So every gain here is ``_rt_pilot``'s own
- ``K_PITCH_IN`` with ``K_PITCH_RATE_IN``, ``K_ROLL_IN`` with ``K_ROLL_RATE_IN``,
``K_YAW_IN`` with ``K_YAW_RATE_IN``, ``K_RATE_IN`` on the lever - with each *pair*
multiplied by the instrument's scale, so that a lead stays a ratio and does not
move with the grip, which is phase 1's own rule for a scaled loop.  A scale of one
is then the mission's pilot exactly, and :func:`same_ruler` is that claim asserted
rather than argued: one decision by each pilot, on the same aircraft, axis for
axis, to the last bit the arithmetic holds.

What comes back, into ``_rt_autoro_hand.txt``:

* **the mission's own row**, first: ``_rt_autoro_entry.flight`` itself, called
  rather than copied, at the decision rate it commits its numbers at.  It is the
  *comparable* row - 231.8 ft of catch in 8.9 s on the grip's own cut and 189.7 ft
  in 7.5 s on the failure, both settling on a 12.2 m/s descent, both inside the
  floor's own 3.0 m/s - and every number below is read against it rather than
  against a cousin of it;
* **the same two entries flown by the instrument**, at the two clocks a person is
  worth asking about: the instrument's own twenty decisions a second, and one
  decision a :data:`_rt_handling.DELAY_S`, which is the *slowest* clock that
  instrument will fly at all.  It is not the mission's own 0.25 s: a person who
  decides less often than their eyes see is not a person this model has, and the
  instrument's own ``selftest`` refuses it, which :func:`same_ruler` asserts.  So
  the second row separates the *rate* from the delay and the lag, and the clock
  the baseline row is committed at is one a hand cannot be given.  That row is
  also where the finding starts: a hand given one decision a delay never gets the
  aircraft into the mission's own band at all, so the slow clock carries no entry
  and the mission's own climb is closed to a hand that slow;
* **the failure noticed late.**  A grip cut is a *decision* - a person makes it
  and knows when they made it - but a failure is a surprise, so what tells the
  pilot is an instrument, and the one this file gives them is the torque gauge at
  the same :data:`_rt_autoro_entry.TORQUE_GONE_PERCENT` that file calls the fuel
  gone by.  The cases above that are a person who takes one of the mission's own
  decisions, then two, then three, longer to believe it - because a failure
  announces nothing, and the entry a pilot *intends* and the entry that happens to
  them are not the same flight;
* **the grip's own band**: the same two entries with the cyclic's two gains scaled
  up from the mission's own until an entry fails and down until one does, which is
  phase 1's shape - a band and not a number - with both edges where the arrival
  leaves the floor's own limit or the model refuses a frame.

What this file refuses to claim is that the person and the mission's pilot are the
same flight.  They are not, and the transcript prints where they part: the grip
takes four seconds of a held key to close and the person is an eye behind the
aircraft when they start it, the sticks move at the model's limb rate rather than
at a key's quarter of it, and the loops are closed on a clock and a memory that
the mission's script has not got.  What it *does* claim, and assert, is the
smaller thing every number below rests on: at a scale of one, with no hand lag and
with the aircraft itself handed to the decision instead of a memory of it, this
pilot's loops are the mission's loops - so what the two rows differ by is the
person, and not the pilot model.

Usage::

    python _rt_autoro_hand.py [rate ...]
"""

import sys

from _rt_axes import CRUISE, SCALES_ONE, Scales
from _rt_axes import Hands as Instrument
from _rt_handling import (DELAY_FRAMES, DELAY_S, PILOT_CLOCK_SLACK_S, PILOT_HZ,
                          PILOT_PERIOD_S, T_LAG_S)
from _rt_pilot import (AUTOROTATION_CLIMB_ALT_M, AUTOROTATION_SPEED_KT,
                       AUTOROTATION_SPEED_MPS, BANK_LIMIT_DEG, CLIMB_LIMIT_MPS,
                       CLIMB_SKID_FT, CLIMB_SPEED_TOLERANCE_MPS,
                       CLIMB_TOLERANCE_M, COLLECTIVE_TRAVEL_IN, CUSHION_GAIN_IN,
                       CUSHION_KEY_RATE_IN, CUSHION_PULL_IN, CUSHION_SINK_MPS,
                       CUT_GRIP_KEY, DESCENT_LEVER_DOWN_IN, FLARE_PITCH_DEG,
                       FLARE_PITCH_FROM_FT, FLARE_SKID_FT, HOVER_SETTLE_S,
                       K_BANK_DEG, K_PITCH_IN, K_PITCH_RATE_IN, K_RATE_IN,
                       K_ROLL_IN, K_ROLL_RATE_IN, K_SPEED_DEG, K_YAW_IN,
                       K_YAW_RATE_IN, KEY_STEP, LANDING_SETTLE_S,
                       LAT_STICK_HALF_IN, LEVEL_OFF_SKID_FT, LONG_STICK_HALF_IN,
                       PARKED_ALT_M, PEDAL_HALF_IN, PITCH_LIMIT_DEG,
                       RATE_GAIN_MPS, REACTION_S, THROTTLE_STEP, TRACK_EAST_REF_M,
                       YAW_REF_DEG, AutorotationPilot, clip, inches_to_axes,
                       trims_for, wrap)
from _rt_pilot import skid_ft as skid_ft_of
from _rt_autoro_entry import (ARRIVAL_REPORT_S, ARRIVAL_SKID_FT,
                              DESCENT_BAND_MPS, Flight, KILL_BY_FAILURE,
                              KILL_BY_GRIP, KILLS, MAX_FLIGHT_S, PRESET,
                              REPORT_S, TORQUE_GONE_PERCENT, WORDS, diff,
                              say_the_flight, say_the_pair, words)
from _rt_autoro_entry import flight as scripted
from _rt_autoro_probe import say, skid_ft
from main import KEY_NAMES
from simulation import (PAD_COLLECTIVE, SIM_TIME_STEP_S, Simulation,
                        UH1_CG_HEIGHT_ON_GROUND, UH1_HARD_LANDING_MPS,
                        airframe_preset)
from airframe import FOOT

#: The transcript, written by the run itself: the shell's own redirection is not
#: worth trusting on this box.
OUT_PATH = "_rt_autoro_hand.txt"

#: The two clocks the person is flown at, s: the instrument's own twenty
#: decisions a second, which is the instrument as the two phases before this one
#: fly it, and :data:`_rt_handling.DELAY_S` - one decision a delay, the slowest
#: hand that instrument will fly at all, since its own ``selftest`` refuses a
#: person who decides less often than their eyes see.  The mission's own
#: :data:`_rt_pilot.REACTION_S`, which is the clock its script commits its row at,
#: is longer than that, and so is the one clock a hand cannot be given: see
#: :func:`same_ruler`, which asserts the refusal rather than arguing it.
CLOCKS = (PILOT_PERIOD_S, DELAY_S)

#: What each of the two clocks is, in the transcript's own words.
CLOCK_WORDS = {
    PILOT_PERIOD_S: "the instrument's own, which is the two phases before this",
    DELAY_S: "the slowest a hand may be: one decision a delay",
}

#: The extra seconds a person may take to believe a *failure*, on top of the
#: gauge's own reading and on top of what the rows above already fly: one of the
#: mission's own decisions, and two, and three.  A grip cut is not flown these - a
#: person who rolls the grip off did it on purpose and knows it - so this leg is
#: the failure's own, and the person's own row at no hesitation is above it.
HESITATIONS = (REACTION_S, 2.0 * REACTION_S, 3.0 * REACTION_S)

#: The grip's own band: the cyclic's two gains scaled up from the mission's own
#: until an entry fails, and down until one does.  Phase 1's own shape, and phase
#: 2's own finding is why it is the cyclic that is scaled here: the lever's and
#: the pedals' bands do not transfer, and what an entry is flown on is its speed.
SCALES_HARDER = (1.5, 2.0, 3.0, 4.0, 6.0)
SCALES_SOFTER = (0.75, 0.5, 0.35, 0.25)


class Watch(object):
    """When the climb is settled: the mission's own criterion, on the aircraft.

    The instant both entries are measured from is the frame the *aircraft* is
    first settled in the mission's own band - five hundred feet and sixty knots,
    held for :data:`_rt_pilot.HOVER_SETTLE_S` - because that is the frame
    ``AutorotationPilot.phase_of`` names its own cut on, and so the frame the two
    rows of ``_rt_autoro_entry.py`` are comparable from.  It is read here off the
    aircraft rather than off a pilot, since this file's pilot does not decide it:
    the person's own entry is theirs, and it is a decision or a surprise behind
    this frame.  See :meth:`Hands.cue`.
    """

    def __init__(self):
        #: The flight's own clock, s: the first frame of the settled run, and the
        #: frame the switch was thrown on - once, since a flight has one entry.
        self.settled_at = None
        self.switch_at = None
        self.took_off = False

    def switched(self, sim):
        """Has the climb just settled?  True on the one frame that it has."""
        if self.switch_at is not None:
            return False
        telemetry = sim.telemetry()
        alt = telemetry.alt_agl
        speed = telemetry.ground_speed
        if not self.took_off:
            # The mission's own guard: on the pad the lever is the only control
            # with any weight on it, so the band is not read until the skids are
            # off and the aircraft is properly up.
            if sim.on_ground and alt < PARKED_ALT_M:
                return False
            self.took_off = True
        if (abs(alt - AUTOROTATION_CLIMB_ALT_M) <= CLIMB_TOLERANCE_M
                and abs(speed - AUTOROTATION_SPEED_MPS)
                <= CLIMB_SPEED_TOLERANCE_MPS):
            if self.settled_at is None:
                self.settled_at = float(sim.sim_time)
            elif float(sim.sim_time) - self.settled_at >= HOVER_SETTLE_S:
                self.switch_at = float(sim.sim_time)
                return True
        else:
            self.settled_at = None
        return False


class Hands(Instrument):
    """A person's hands on the mission's four loops, at an instrument's clock.

    The instrument to the last line - the twenty hertz clock on the flight's own
    time, the ``DELAY_S`` of delay read off a deque of remembered states, the lag
    in the hand and the model's own limb rate under it, :meth:`fill`,
    :meth:`view`, :meth:`push`, :meth:`move` and :meth:`selftest` as phase 1 wrote
    them - with two things changed and nothing else:

    * :meth:`decide` is ``_rt_pilot.AutorotationPilot.decide``'s mission, phase
      for phase and gain for gain, read off the *view* rather than off the
      aircraft.  The instrument's own hover loops - its ``k_pitch``, its
      ``k_roll`` and its 1.5 s lead - are computed and left where they are; what
      this file flies is the mission's pair, scaled, and the reason is the
      module's;
    * and the twist grip, which is not one of the four axes and so cannot come out
      of :meth:`move`: it is a fifth key, held here as the mission holds it and
      handed to the model's own ratchet in :meth:`frame`.

    What is read off the *view* includes the pilot's own bookkeeping: the phases,
    the lowest rotor, the last rate read off the ground, the height the cushion
    was opened at, and the grip's own position on the panel.  Every instant
    recorded is the frame the decision was made on - the flight's own clock - and
    the seeing was :data:`_rt_handling.DELAY_S` of it before, which is why the
    transcript prints both ends of the person's own entry.
    """

    def __init__(self, sim, trims, way, scales=SCALES_ONE, hesitation=0.0,
                 **kwargs):
        # The instrument's own baseline is the trim the flight *ends* on, and it
        # is not what the loops are written around: this mission's baseline moves
        # with its phase, from the level trim of the speed the aircraft is at to
        # the autorotation's own.  The descent's row is passed so that a reader
        # looking at the object sees a trim rather than a placeholder, and the
        # condition is the flying one, since neither a pad nor a hover is
        # involved - this file's own decide() reads neither.
        Instrument.__init__(self, sim, trims.descent()[:4], scales=scales,
                            condition=CRUISE, **kwargs)
        #: Which way the shaft is killed: :data:`_rt_autoro_entry.KILL_BY_GRIP`
        #: or :data:`_rt_autoro_entry.KILL_BY_FAILURE`.  It is the *cue* that
        #: turns on it and not the physics - see :meth:`cue`.
        self.way = way
        #: The seconds the person takes to believe the instruments, over and
        #: above what the instruments say.
        self.hesitation = float(hesitation)
        #: The model's own trims, and the loops this file's hand flies: the
        #: mission's gains, each pair under the instrument's scale, with the lead
        #: each loop is closed at kept at the model's own ratio.
        self.trims = trims
        self.hand_pitch_in = K_PITCH_IN * self.scales.theta
        self.hand_pitch_rate_in = K_PITCH_RATE_IN * self.scales.theta
        self.pitch_lead_s = K_PITCH_RATE_IN / K_PITCH_IN
        self.hand_roll_in = K_ROLL_IN * self.scales.theta
        self.hand_roll_rate_in = K_ROLL_RATE_IN * self.scales.theta
        self.roll_lead_s = K_ROLL_RATE_IN / K_ROLL_IN
        self.hand_lever_in = K_RATE_IN * self.scales.lever
        self.hand_pedal_in = K_YAW_IN * self.scales.pedal
        self.hand_pedal_rate_in = K_YAW_RATE_IN * self.scales.pedal
        self.pedal_lead_s = K_YAW_RATE_IN / K_YAW_IN
        # The mission's own bookkeeping, all of it off the view: the phases and
        # when each began, the rotor's own low point, the last rate read off the
        # ground, and the cushion's opening sink and lever.
        self.done = None
        self.phase = "pickup"
        self.climb_alt = AUTOROTATION_CLIMB_ALT_M
        self.took_off = None
        self.climb_settled = None
        self.cut_at = None
        self.descent_at = None
        self.cushion_at = None
        self.cushion_in = None
        self.cushion_open_in = None
        self.cushion_sink = None
        self.cushion_rotor = None
        self.lowest_rotor = 100.0
        self.last_airborne_rate = 0.0
        self.touchdown = None
        self.touchdown_rate = None
        self.landed_since = None
        #: The grip's own position, which is not an axis: one is open.  The person
        #: asks for it as the mission does, and holds a key until it is there.
        self.want_throttle = 1.0
        #: The frame the decision is being made on in the flight's own clock, and
        #: the two ends of the person's own entry: what told them and when they saw
        #: it, and the frame they acted on it.
        self.now = float(sim.sim_time)
        self.cue_at = None
        self.cue_view_at = None
        self.entry_at = None
        #: Where the pad holds the four controls, which is where the mission
        #: leaves them and what a person puts them back to: the run's own trim with
        #: the lever at ``simulation.PAD_COLLECTIVE``, in the sticks' own inches.
        pad = sim.trim_controls.collective_at(PAD_COLLECTIVE)
        self.pad_inches = [pad.collective, pad.long_stick, pad.lat_stick,
                           pad.pedal]

    def selftest(self):
        """The instrument's own check, and this file's four pairs of gains.

        Phase 1's and phase 2's own assertions first, unchanged, because the
        clock, the delay, the lag and the four scaled loops the base class builds
        are still exactly what they claim.  Then the four pairs this file flies:
        each is ``_rt_pilot``'s own pair at this pilot's scale, and each pair's
        ratio is the lead its loop is closed at - so a gain moved in the mission
        moves this file's hand with it, and a lead cannot drift away from the
        pair it belongs to.
        """
        Instrument.selftest(self)
        assert abs(self.hand_pitch_in
                   - K_PITCH_IN * self.scales.theta) < 1e-12, \
            (self.hand_pitch_in, K_PITCH_IN, self.scales)
        assert abs(self.hand_pitch_rate_in
                   - self.hand_pitch_in * self.pitch_lead_s) < 1e-12, \
            (self.hand_pitch_rate_in, self.hand_pitch_in, self.pitch_lead_s)
        assert abs(self.hand_roll_rate_in
                   - self.hand_roll_in * self.roll_lead_s) < 1e-12, \
            (self.hand_roll_rate_in, self.hand_roll_in, self.roll_lead_s)
        assert abs(self.hand_lever_in - K_RATE_IN * self.scales.lever) < 1e-12, \
            (self.hand_lever_in, K_RATE_IN, self.scales)
        assert abs(self.hand_pedal_rate_in
                   - self.hand_pedal_in * self.pedal_lead_s) < 1e-12, \
            (self.hand_pedal_rate_in, self.hand_pedal_in, self.pedal_lead_s)
        return self

    # -- the person's own reading of the flight ---------------------------

    def cue(self, telemetry, now):
        """When the instruments say the entry has begun, on the view's clock.

        Two cues, because there are two entries, and the difference between them
        is the whole of this leg:

        * a person *flies* the grip's cut, so what tells them is the thing the
          mission's own ``phase_of`` waits for - the climb inside its own band of
          five hundred feet and sixty knots, held for its own settling time - and
          they roll the grip off on it.  That is a decision, and a person knows
          when they made it;
        * a person *suffers* an engine failure, and nothing announces it: what
          tells them is an instrument, and the one here is the torque gauge, at
          the same :data:`_rt_autoro_entry.TORQUE_GONE_PERCENT` the mission's own
          ruler calls the fuel gone by.  The gauge takes the governor's own lag to
          get there, and the eyes take :data:`_rt_handling.DELAY_S` on top of it,
          which is what this file measures rather than assumes.

        Returns the instant on the *view's* clock, or None while nothing has been
        said - and see :meth:`phase_of` for why it is the view's clock the wait is
        measured on and the frame's clock it is recorded in.
        """
        if self.way == KILL_BY_GRIP:
            if (abs(telemetry.alt_agl - AUTOROTATION_CLIMB_ALT_M)
                    <= CLIMB_TOLERANCE_M
                    and abs(telemetry.ground_speed - AUTOROTATION_SPEED_MPS)
                    <= CLIMB_SPEED_TOLERANCE_MPS):
                if self.climb_settled is None:
                    self.climb_settled = now
                elif now - self.climb_settled >= HOVER_SETTLE_S:
                    return self.climb_settled + HOVER_SETTLE_S
            else:
                self.climb_settled = None
            return None
        if telemetry.engine.torque_percent <= TORQUE_GONE_PERCENT:
            return now
        return None

    def phase_of(self, view):
        """What the person is doing, from what the instruments show.

        ``AutorotationPilot.phase_of``'s own sequence - the pickup, the climb's
        band, the cut, the grip closing, the descent, the cushion's height, the
        skids and the lightness on them again - read off the view rather than off
        the aircraft, with the one thing this file adds to the mission's own
        machine: the *cue*.  The mission names its cut the frame the aircraft is
        settled; a person names theirs the frame an instrument tells them, which
        is :meth:`cue` and is at least a fifth of a second later, and which a case
        may lag further with a hesitation.

        Two clocks, and they are not the same one.  The *wait* is measured on the
        view's own clock, because that is the clock the person is living in: they
        know when the gauge moved by what the gauge read, and their hesitation is
        seconds of their own life.  The *instants recorded* are the frames the
        decisions were made on, which is the flight's own clock, so that every
        number the transcript prints can be read against the mission's own row.
        The two ends of the person's entry are therefore :attr:`cue_at` and
        :attr:`entry_at`, and the distance between them is the person's own
        latency plus the case's hesitation.
        """
        telemetry = view.telemetry
        alt = telemetry.alt_agl
        speed = telemetry.ground_speed
        when = view.time
        stamp = self.now
        if self.took_off is None and not telemetry.on_ground:
            self.took_off = stamp
        if self.phase == "pickup":
            # The loops do the pickup; the name lasts until the aircraft is
            # properly off, which is the mission's own guard.
            if not (telemetry.on_ground and alt < PARKED_ALT_M):
                self.phase = "climb"
        if self.phase == "climb":
            if self.cue_view_at is None:
                self.cue_view_at = self.cue(telemetry, when)
                if self.cue_view_at is not None:
                    self.cue_at = stamp
            if (self.cue_view_at is not None
                    and when >= self.cue_view_at + self.hesitation):
                self.phase = "cut"
                self.cut_at = stamp
                self.entry_at = stamp
                self.want_throttle = 0.0          # roll the grip off
        if self.phase == "cut" and telemetry.engine.throttle <= THROTTLE_STEP:
            # Within one frame of travel of the stop is *closed*, which is the
            # same tolerance the mission's own phase machine uses; the grip's
            # position is read off the panel with everything else, so the descent
            # is named a fifth of a second late as well.
            self.phase = "descent"
            self.descent_at = stamp
        if self.phase in ("cut", "descent", "cushion"):
            self.lowest_rotor = min(self.lowest_rotor, telemetry.rotor_percent)
            self.last_airborne_rate = telemetry.height_rate
        if self.phase == "descent" and skid_ft_of(telemetry) <= FLARE_SKID_FT:
            self.phase = "cushion"
            self.cushion_at = stamp
            self.cushion_in = None
            #: The rotor the *entry* handed over, %: read where the cushion is
            #: opened, which is the last frame the entry is still the entry.  It
            #: is here because the arrival's own rotor is the cushion's business
            #: as much as the entry's - see :meth:`decide`, whose one-way pull can
            #: outrun a *hand* where it cannot outrun a key.
            self.cushion_rotor = telemetry.rotor_percent
        if self.phase in ("cut", "descent", "cushion") and telemetry.on_ground:
            self.phase = "landed"
            self.touchdown = stamp
            self.touchdown_rate = self.last_airborne_rate
            self.landed_since = stamp
        if self.phase == "landed" and not telemetry.on_ground:
            # Light on its skids again: the lever is at the pad's own and no use
            # to a rotor with nothing driving it, so the last thirty feet are
            # flown again - the mission's own rule.
            self.phase = "descent"
            self.landed_since = None

    def decide(self, view):
        """Where the four axes ought to be, in inches, from what was seen.

        ``AutorotationPilot.decide`` loop for loop and branch for branch, with
        three substitutions and no arithmetic of its own:

        * every reading comes out of *view*, which is the aircraft as it was
          ``DELAY_S`` ago, and the phase machine above is read off the same view;
        * the four gains are this file's own pairs, which are that module's at a
          scale of one - see the class docstring, and :func:`same_ruler`, which
          asserts exactly that;
        * and the cushion's lever *rate* is integrated over this pilot's clock
          rather than over the mission's :data:`_rt_pilot.REACTION_S`, since it is
          a rate a second and the hand is on it for its own decisions.

        What is returned is inches rather than the four normalised axes the
        mission's own ``decide`` returns, because that is what the instrument's
        :meth:`move` and :meth:`frame` take: a person's stick is a position, and
        the ratchet under the hand is the model's own.
        """
        telemetry = view.telemetry
        self.phase_of(view)
        if self.phase == "landed":
            # Down and staying down: the lever back to the pad's own and the
            # sticks back on it, which is where the mission leaves them too.  The
            # pull is not held - it is spent, and a lever held up on a rotor with
            # nothing driving it is a machine lifted off its skids again.
            return list(self.pad_inches)
        if self.phase == "pickup":
            # On the skids the cyclic and the pedals do nothing; the pickup is the
            # lever's doing, and the trim's hover row is where it goes.  The two
            # cyclic positions are the ones the aircraft is holding, which is
            # where a pilot's hands are when they are not flying with them.
            return [self.trims.rows[0][0], telemetry.long_stick_in,
                    telemetry.lat_stick_in, telemetry.pedal_in]

        alt = telemetry.alt_agl
        speed = telemetry.ground_speed
        if self.phase == "climb":
            collective_t, long_t, lat_t, pedal_t, __ = self.trims.at(speed)
            rate_want = clip(RATE_GAIN_MPS * (self.climb_alt - alt),
                             -CLIMB_LIMIT_MPS, CLIMB_LIMIT_MPS)
            # The outer loop is the model's own: what this file scales is the loop
            # *this pilot* closes - inches of lever per m/s of rate the climb is
            # short of - which is phase 1's own rule, and why RATE_GAIN_MPS is not
            # scaled with it.
            collective = (collective_t
                          + self.hand_lever_in * (rate_want
                                                  - telemetry.height_rate))
            pitch_t = self.trims.at(AUTOROTATION_SPEED_MPS)[4]
            pitch_want = clip(
                pitch_t - K_SPEED_DEG * (AUTOROTATION_SPEED_MPS - speed),
                pitch_t - PITCH_LIMIT_DEG, pitch_t + PITCH_LIMIT_DEG)
        else:
            # The descent, from the cut to the skids: the autorotation's own
            # lever, sticks and attitude, held at the trim - a rotor with no engine
            # has nothing to loop a height rate around, and the trim of the speed
            # the aircraft is at *is* the rate it comes down at.  What is looped
            # instead is the last of that rate: the cushion's pull, a lever rate
            # proportional to how much more sink there is than CUSHION_SINK_MPS,
            # integrated into a position and limited to what a key can move the
            # lever at.
            collective_t, long_t, lat_t, pedal_t, pitch_t, __ = \
                self.trims.descent()
            collective = collective_t
            if self.phase == "descent":
                collective = collective_t - DESCENT_LEVER_DOWN_IN
            flare_from = (FLARE_SKID_FT if FLARE_PITCH_FROM_FT is None
                          else FLARE_PITCH_FROM_FT)
            if self.phase == "cushion":
                if self.cushion_in is None:
                    # The pull starts where the lever *is* - the descent's own
                    # trim, less whatever of it the descent was held under - so
                    # that the first frame of the cushion is a pull and not a step.
                    # And the rate it thins from is the sink the descent actually
                    # arrived with and the trim's own, whichever is the faster,
                    # which is the mission's own anchoring: the first frame's error
                    # is zero and the pull only ever follows the schedule down.
                    self.cushion_in = collective_t - DESCENT_LEVER_DOWN_IN
                    self.cushion_open_in = self.cushion_in
                    self.cushion_sink = max(-telemetry.height_rate,
                                            self.trims.descent()[5],
                                            CUSHION_SINK_MPS)
                want = -max(CUSHION_SINK_MPS,
                            self.cushion_sink * skid_ft_of(telemetry)
                            / FLARE_SKID_FT)
                error = want - telemetry.height_rate
                # The pull only ever comes up, and the key rate is the only limit
                # on how fast the difference may be paid for.
                self.cushion_in = min(
                    self.cushion_in
                    + clip(CUSHION_GAIN_IN * error, 0.0, CUSHION_KEY_RATE_IN)
                    * self.period,
                    min(collective_t + CUSHION_PULL_IN, COLLECTIVE_TRAVEL_IN))
                collective = self.cushion_in
                pitch_want = clip(
                    pitch_t + FLARE_PITCH_DEG
                    * clip(
                        min((flare_from - skid_ft_of(telemetry))
                            / (flare_from - LEVEL_OFF_SKID_FT),
                            skid_ft_of(telemetry) / LEVEL_OFF_SKID_FT),
                        0.0, 1.0),
                    pitch_t, pitch_t + FLARE_PITCH_DEG)
            elif skid_ft_of(telemetry) <= flare_from:
                # The cyclic flare, above the cushion's own height: the same
                # schedule, flown where there is height to spend on it rather than
                # revolutions.
                pitch_want = clip(
                    pitch_t + FLARE_PITCH_DEG
                    * clip(
                        min((flare_from - skid_ft_of(telemetry))
                            / (flare_from - LEVEL_OFF_SKID_FT),
                            skid_ft_of(telemetry) / LEVEL_OFF_SKID_FT),
                        0.0, 1.0),
                    pitch_t, pitch_t + FLARE_PITCH_DEG)
            else:
                pitch_want = clip(
                    pitch_t - K_SPEED_DEG * (AUTOROTATION_SPEED_MPS - speed),
                    pitch_t - PITCH_LIMIT_DEG, pitch_t + PITCH_LIMIT_DEG)

        long_stick = (long_t
                      + self.hand_pitch_in * (telemetry.pitch_deg - pitch_want)
                      + self.hand_pitch_rate_in * telemetry.pitch_rate_deg)
        east = view.east - TRACK_EAST_REF_M
        bank_want = clip(-K_BANK_DEG * east, -BANK_LIMIT_DEG, BANK_LIMIT_DEG)
        lat_stick = (lat_t
                     + self.hand_roll_in * (bank_want - telemetry.roll_deg)
                     - self.hand_roll_rate_in * telemetry.roll_rate_deg)
        pedal = (pedal_t
                 - self.hand_pedal_in * wrap(telemetry.yaw_deg - YAW_REF_DEG)
                 - self.hand_pedal_rate_in * telemetry.yaw_rate_deg)
        return [collective, long_stick, lat_stick, pedal]

    # -- the frame loop's view of it --------------------------------------

    def grip_key(self, grip):
        """The twist grip's own key, held until the grip gets where it was asked.

        The same three-way answer the mission's own ``keys`` gives and at the same
        :data:`_rt_pilot.THROTTLE_STEP`: E while the grip is below what the person
        asked for, Q while it is above, and neither inside one frame of travel of
        it - which is why the *model's* ratchet turns the grip and not this file's
        arithmetic.
        """
        if self.done is not None:
            return 0.0                            # nothing held once it is down
        error = self.want_throttle - grip
        if error > THROTTLE_STEP:
            return 1.0                            # E, rolling it on
        if error < -THROTTLE_STEP:
            return -1.0                           # Q, rolling it off
        return 0.0

    def frame(self, sim, dt):
        """One frame of a person flying: look when it is due, move, fly, and grip.

        ``_rt_handling.Hands.frame``'s own order to the line - the clock on the
        flight's own time, the decision off the oldest state remembered, this
        frame's state pushed into the history *after* the decision, the command
        through the hand's lag and the model's own limb rate - with the one
        control this file has that the instrument has not.  The grip is not one of
        the four axes, so it cannot come out of :meth:`move`: it is the fifth key,
        and it goes to :meth:`simulation.Simulation.fly` as the key it is, beside
        the four axes the instrument has already set.
        """
        now = float(sim.sim_time)
        self.now = now
        self.since_look += max(now - self.last_time, 0.0)
        self.last_time = now
        if self.since_look >= self.period - PILOT_CLOCK_SLACK_S:
            view = self.view()
            if view is not None:
                self.since_look = 0.0
                self.ages.append(now - view.time)
                self.want = self.decide(view)
                self.decisions += 1
        self.push(sim)
        axes = inches_to_axes(*self.move(dt))
        sim.pilot.set_axes(axes[0], axes[1], axes[2], axes[3], dt=dt)
        return sim.fly(dt, throttle=self.grip_key(sim.pilot.throttle()))

    def mission_over(self, sim):
        """The skids on the ground and staying there, or the flight wrecked.

        ``AutorotationPilot.mission_done``'s own rule, on this pilot's own
        bookkeeping: a rotor that has stopped has stopped, so the arrival is the
        end once the aircraft is down, has stayed down a settling's worth of
        seconds, and the sticks are back within a key of the pad's own.  The
        phase and the landing are read off the view, so both are a fifth of a
        second late; the sticks are the aircraft's own, since a hand knows where
        it put them.
        """
        if self.done is None:
            if sim.crashed:
                self.done = sim.sim_time
            elif self.phase == "landed" and sim.on_ground:
                axes = sim.pilot.axes()
                on_the_pad = all(abs(axis - want) <= step
                                 for axis, want, step
                                 in zip(axes, inches_to_axes(*self.pad_inches),
                                        KEY_STEP))
                if (on_the_pad and self.landed_since is not None
                        and sim.sim_time - self.landed_since
                        >= LANDING_SETTLE_S):
                    self.done = sim.sim_time
        return self.done is not None

    def note(self, sim):
        """Where the person is in the entry, in their own words.

        The mission's own captions, which are about the aircraft rather than about
        whose hands are on it, so that the two transcripts can be read beside each
        other - with the decisions counted, which is the person's own clock
        showing rather than the mission's.
        """
        telemetry = sim.telemetry()
        skid = skid_ft(sim)
        if sim.crashed:
            return ("%s at %.1f s, %s" % (self.phase, sim.sim_time,
                                          telemetry.crash_message))
        if self.done is not None:
            return ("flown in %.0f s: %.0f ft up at %.0f kt, the rotor down to"
                    " %.0f %%, down at %+.0f fpm"
                    % (self.done, CLIMB_SKID_FT, AUTOROTATION_SPEED_KT,
                       self.lowest_rotor, self.touchdown_rate / FOOT * 60.0))
        if self.phase == "pickup":
            return ("picking up off the pad, %4.1f ft of skid, %d decisions"
                    % (skid, self.decisions))
        if self.phase == "climb":
            return ("climbing to %.0f ft at %.0f kt, %4.1f ft of skid,"
                    " %+5.0f fpm, %.0f kt, %d decisions"
                    % (CLIMB_SKID_FT, AUTOROTATION_SPEED_KT, skid,
                       telemetry.height_rate_fpm, telemetry.airspeed_kt,
                       self.decisions))
        if self.phase == "cut":
            return ("rolling the grip off, %4.2f open, rotor %.0f %% at %4.1f ft"
                    " of skid, %d decisions"
                    % (sim.pilot.throttle(), telemetry.rotor_percent, skid,
                       self.decisions))
        if self.phase == "descent":
            return ("autorotating, %4.1f ft of skid, %+5.0f fpm, %.0f kt,"
                    " rotor %.0f %%, %d decisions"
                    % (skid, telemetry.height_rate_fpm, telemetry.airspeed_kt,
                       telemetry.rotor_percent, self.decisions))
        if self.phase == "cushion":
            return ("the cushion, %4.1f ft of skid, %+5.0f fpm, collective"
                    " %5.2f in, rotor %.0f %%, %d decisions"
                    % (skid, telemetry.height_rate_fpm,
                       telemetry.collective_in, telemetry.rotor_percent,
                       self.decisions))
        return ("down on its skids, lever coming back to the pad's own,"
                " %d decisions" % (self.decisions,))


def flight(way, trims, stream, scales=SCALES_ONE, hesitation=0.0,
           reaction=PILOT_PERIOD_S, delay=DELAY_FRAMES, lag=T_LAG_S, label=None,
           trace=True):
    """One killing of the shaft, flown by a person's hands on the mission's loops.

    ``_rt_autoro_entry.flight``'s own frame loop, with the pilot exchanged for the
    instrument and the switch thrown on the same frame: one frame of
    :data:`simulation.SIM_TIME_STEP_S` at a time, the reports a second apart and
    every quarter second under ten feet of skid, and the shaft killed by the same
    :data:`_rt_autoro_entry.KILLS` entry the mission's own file kills it with.

    The switch is thrown on the frame :class:`Watch` says the *aircraft* is
    settled - the frame the mission's own ``phase_of`` names its cut on - and not
    on the frame the person notices, which is what this file is measuring.  The
    person's own entry is therefore behind it, by their own latency and any
    hesitation, and both instants are printed.

    Returns ``(hands, flown)``: the pilot, for its own clock and its own entry,
    and the :class:`_rt_autoro_entry.Flight` the switch began.
    """
    sim = Simulation(airframe=airframe_preset(PRESET))
    sim.on_the_pad()
    sim.fit_engine()
    hands = Hands(sim, trims, way=way, scales=scales, hesitation=hesitation,
                  delay=delay, period=reaction, t_lag=lag)
    hands.selftest().fill(sim)
    say(stream, "")
    say(stream, "the %s by hand, %s: %s"
        % (way, label or "the instrument's own clock and hands", WORDS[way]))
    flown = None
    watch = Watch()
    reported = tenth = 0.0
    while sim.sim_time < MAX_FLIGHT_S and not hands.mission_over(sim):
        if flown is None and watch.switched(sim):
            note = KILLS[way](sim, hands)
            flown = Flight(way, sim, hands, sim.sim_time, trims.descent()[5])
            say(stream, "  %5.1f s at the switch, %4.1f ft of skid and %.0f kt:"
                " the switch is thrown - %s"
                % (flown.switch_s, flown.switch_skid_ft,
                   sim.telemetry().airspeed_kt, note))
        hands.frame(sim, SIM_TIME_STEP_S)
        if flown is None:
            continue
        flown.measure()
        now = float(sim.sim_time)
        if trace and now >= reported + REPORT_S:
            reported = now
            say(stream, "  %5.1f s %s" % (now, hands.note(sim)))
        if trace and skid_ft(sim) < ARRIVAL_SKID_FT and now >= tenth:
            tenth = now + ARRIVAL_REPORT_S
            telemetry = sim.telemetry()
            say(stream, "  %5.2f s %4.1f ft of skid, %+7.0f fpm, collective"
                " %5.2f in, rotor %.1f %%, decisions %d"
                % (now, skid_ft(sim), telemetry.height_rate_fpm,
                   telemetry.collective_in, telemetry.rotor_percent,
                   hands.decisions))
    if flown is None:
        # Nothing below this line happened, and there is no row in it: what the
        # transcript can say is where the aircraft was when the flight ran out -
        # of time, or of sky - in the person's own words, which name a crash if
        # there was one.  The switch waits for the mission's own band, five
        # hundred feet of skid at sixty knots held four seconds, and a hand too
        # slow for the loops it is flying never gets the aircraft into it.
        say(stream, "  no switch: the flight never reached the mission's own band"
            " in %.0f s of physics - %s" % (sim.sim_time, hands.note(sim)))
        return hands, None
    flown.finish()
    say_the_flight(stream, flown)
    say_the_hand(stream, hands, flown)
    return hands, flown


def say_the_hand(stream, hands, flown):
    """The person's own numbers: the clock, the two ends of the entry, the grip.

    ``_rt_autoro_entry.say_the_flight`` has printed the shaft, the rotor, the
    catch and the arrival in the words both files print them in, which is what
    makes the two rows comparable.  What is left is this file's own: the rate the
    person decided at and how old the instruments they decided on were, the two
    ends of their entry - what told them, and the frame they acted on it - the
    grip's own four seconds of a held key, and the height the cushion opened at
    with the lever it opened on.
    """
    ages = hands.ages
    say(stream, "  the person: %d decisions in %.1f s of physics (%.2f a second,"
        " the clock is %.2f s), every view %s s old, %s s of hesitation"
        % (hands.decisions, flown.sim.sim_time,
           hands.decisions / max(flown.sim.sim_time, 1e-9), hands.period,
           words(sum(ages) / len(ages) if ages else None, "%5.3f"),
           words(hands.hesitation, "%4.2f")))
    if hands.cue_at is None or hands.entry_at is None:
        say(stream, "  the person's own entry: not flown - nothing told them"
            " before the flight ended")
    else:
        say(stream, "  the person's own entry: told at +%.2f s after the switch,"
            " acted on at +%.2f s - %.2f s of their own latency%s"
            % (hands.cue_at - flown.switch_s, hands.entry_at - flown.switch_s,
               hands.entry_at - hands.cue_at,
               "" if not hands.hesitation
               else " and %.2f s of hesitation" % (hands.hesitation,)))
    # The height the cushion was opened at is the flight's own, read off the
    # samples the switch's own record kept: the mission measures its catch the
    # same way, so both are the same instrument's answer.
    opened = opened_sink = None
    for when, sink, alt, phase in flown.samples:
        if hands.cushion_at is not None and when >= hands.cushion_at:
            opened = (alt - UH1_CG_HEIGHT_ON_GROUND) / FOOT
            opened_sink = sink
            break
    closed = (hands.descent_at is not None
              and flown.sim.telemetry().engine.throttle <= THROTTLE_STEP)
    say(stream, "  the grip: %s closed at %s - %.2f s of a held key after the"
        " person's own entry"
        % ("fully" if closed else "NOT",
           words(hands.descent_at, "%.1f s"),
           (hands.descent_at - hands.entry_at)
           if hands.descent_at is not None and hands.entry_at is not None
           else float("nan")))
    say(stream, "  the entry's own hand-over, which is the last frame the entry is"
        " the entry: the cushion opened at %s, %s of skid, %s m/s down, on the"
        " lever the descent was holding (%s in), with the rotor at %s %% - the"
        " flare and the pull are the cushion's own business from there"
        % (words(hands.cushion_at, "%.1f s"), words(opened, "%.0f ft"),
           words(opened_sink, "%5.2f"), words(hands.cushion_open_in, "%5.2f"),
           words(hands.cushion_rotor, "%5.1f")))


def entry_held(flown, auto):
    """Did this flight reach the descent the mission's own pilot reaches?

    ``_rt_autoro_entry.check``'s criterion for the *entry*, number for number: the
    sink settled, on a rate within :data:`_rt_autoro_entry.DESCENT_BAND_MPS` of
    the trim's, and no frame refused.  The *arrival* is deliberately not part of
    it - see :func:`arrived_softly`, and why the two are kept apart - so what this
    answers is whether a hand flew the entry, and not whether it landed it.
    """
    if flown is None or flown.settled_at is None or flown.held_sink is None:
        return False
    if flown.refused_at is not None:
        return False
    return abs(flown.held_sink - auto) <= DESCENT_BAND_MPS


def arrived_softly(flown):
    """Did the skids meet the floor inside its own hard landing rate?

    The other half of what ``_rt_autoro_entry.check`` asserts, kept apart from the
    first because on this aircraft the two are not the same question - and this is
    the one place the file's own hand model changes the answer.  The mission's
    cushion is a *one-way* integrator at :data:`_rt_pilot.CUSHION_KEY_RATE_IN`,
    which a key can just about chase and never catch; a hand moves the collective
    four times that fast off a key's own quarter, so it *arrives* at a lever the
    schedule has already stopped asking for, the machine climbs on ground effect
    it no longer needs, and what comes down again does so under a rotor the pull
    has spent.  That says something about the cushion and something about a hand;
    it does not say the entry was flown badly, which is why the transcript prints
    the hand-over at the cushion's opening beside the arrival.
    """
    return (flown is not None and flown.touchdown_at is not None
            and flown.touchdown_rate is not None
            and flown.touchdown_rate < UH1_HARD_LANDING_MPS)


def never_reached(grip, failure):
    """Which of a row's two entries never reached the mission's own cut.

    A hand deciding slower than the loops it has to close never gets the aircraft
    into the mission's own band - five hundred feet of skid at sixty knots, held
    :data:`_rt_pilot.HOVER_SETTLE_S` - so :class:`Watch` never throws the switch
    and there is no entry to measure: the row is empty, and saying *which* of the
    two got nowhere is all this clock has to report.  ``None`` when both flew,
    which is the ordinary case and the only one at the instrument's own clock.
    """
    return (" and ".join([name for name, flown
                          in (("the grip cut", grip), ("the failure", failure))
                          if flown is None]) or None)


def clock_words(reaction):
    """What a clock is, in the transcript's own words.

    The two the file flies by default have their own sentence in
    :data:`CLOCK_WORDS`; any other rate is one a caller asked for - the command
    line takes them, see :func:`demo` - and is named as the number it is.  A rate
    the model will *not* fly at gets no words at all, because it never gets as far
    as a row: :data:`_rt_handling.DELAY_S` is the line and the instrument's own
    ``selftest`` is what draws it.
    """
    return CLOCK_WORDS.get(float(reaction),
                           "a clock asked for, %.2f s between decisions"
                           % (reaction,))


#: The height of skid the cushion's own comparison in :func:`same_ruler` is made
#: at, ft: inside the flare's own height, so that both the cushion's lever rate
#: and the flare's pitch schedule are doing something.  A comparison made at the
#: descent's own five hundred feet is one where the pull is clipped to nothing and
#: the nose's schedule is off the top of its band, and two pilots agreeing about
#: nothing is not a check of anything.
CUSHION_TEST_SKID_FT = 40.0

#: The three instants :func:`same_ruler` compares the two pilots on: the phase
#: each decision is made in, and the feet of skid the aircraft is put at.  ``None``
#: is level flight at the mission's own climb height, which is the aircraft the
#: climb's own loops are written for; a number is a *trimmed autorotation* at that
#: many feet of skid, which is the mission's own entry height for the descent and
#: :data:`CUSHION_TEST_SKID_FT` for the cushion.
SAME_RULER = (("climb", None), ("descent", CLIMB_SKID_FT),
              ("cushion", CUSHION_TEST_SKID_FT))


def same_ruler(stream):
    """The claim every number below rests on: at a scale of one, one pilot.

    Three instants of the mission - the climb's own band, the trimmed descent and
    the cushion opened on it - and at each of them one decision by this file's
    hand and one by ``_rt_pilot.AutorotationPilot``, on the same aircraft, with
    the person's own dynamics switched off: the view handed to the decision *is*
    the aircraft rather than a memory of it, the hand's lag is zero, the scale is
    one, and both pilots are given the *same* clock - the instrument's own -
    because the clock is not what is being compared and the cushion's lever rate
    is integrated over it.

    And it asserts the one clock the two pilots cannot share: the mission's own
    script decides and integrates every :data:`_rt_pilot.REACTION_S`, which is
    longer than the delay, and the instrument's own ``selftest`` refuses a person
    who decides less often than their eyes see.

    What is compared is the four axes the two decisions leave, in the axes a
    ratchet moves, so this is the end of the arithmetic rather than the middle of
    it.  The phase each comparison is made in is forced to the same one in both
    pilots, because the phase machine is the mission's own and is not what this
    asserts; the pickup and the landing are not compared at all, since both pilots
    leave the pad's own sticks there and one of them states them in inches and the
    other in axes.

    Returns the phases it compared, for the transcript to name.
    """
    trims = trims_for(airframe_preset(PRESET), AutorotationPilot)
    compared = []
    first = None
    for phase, skid in SAME_RULER:
        sim = Simulation(airframe=airframe_preset(PRESET))
        sim.on_the_pad()
        sim.fit_engine()
        hands = Hands(sim, trims, way=KILL_BY_FAILURE, scales=SCALES_ONE,
                      delay=DELAY_FRAMES, period=PILOT_PERIOD_S, t_lag=0.0)
        mission = AutorotationPilot(sim, trims, reaction=PILOT_PERIOD_S)
        if skid is None:
            sim.trim_level_flight(AUTOROTATION_SPEED_MPS,
                                  altitude=AUTOROTATION_CLIMB_ALT_M)
        else:
            # The sim's own way into an autorotation, which is the model's own
            # engine failure and then the trim: see ``trim_autorotation``.
            sim.set_governor(failed=True)
            sim.trim_autorotation(AUTOROTATION_SPEED_MPS,
                                  altitude=skid * FOOT
                                  + UH1_CG_HEIGHT_ON_GROUND)
        for pilot in (hands, mission):
            pilot.phase = phase
            pilot.cushion_in = None
            pilot.cushion_sink = None
        hands.now = float(sim.sim_time)
        inches = hands.decide(hands.snapshot(sim))
        theirs = mission.decide(sim)
        for index, (one, other) in enumerate(zip(inches_to_axes(*inches),
                                                 theirs)):
            assert abs(one - other) < 1e-12, (phase, index, one, other)
        compared.append(phase)
        if first is None:
            first = (hands, list(inches))
    # And the other half of "no dynamics": a hand with no lag *is* the decision,
    # so what is compared above is the person's loops and not the person's loops
    # behind a filter - and with the lag the model does give a hand, one frame of
    # it is still on its way there.
    hands, inches = first
    hands.want = list(inches)
    assert hands.move(SIM_TIME_STEP_S) == list(inches), hands.command
    lagged = Hands(sim, trims, way=KILL_BY_FAILURE, scales=SCALES_ONE,
                   delay=DELAY_FRAMES, period=PILOT_PERIOD_S, t_lag=T_LAG_S)
    lagged.want = list(inches)
    moved = lagged.move(SIM_TIME_STEP_S)
    assert moved != list(inches), moved
    assert moved != list(lagged.trim), moved
    say(stream, "the ruler, checked: the mission's own pilot and this file's hand,"
        " one decision each on the same aircraft at the %s, agree on all four"
        " axes at a scale of one to within 1e-12 of an inch - with the hand's lag"
        " the only thing this file's :meth:`move` adds to the decision"
        % (" and the ".join(compared),))
    # And the one clock the two pilots above *cannot* share, asserted rather than
    # quoted: the mission's script decides and integrates every REACTION_S, which
    # is longer than the delay a decision waits on the eyes, and the instrument's
    # own selftest refuses that.  It is why the rows below are flown at the delay
    # and not at the clock the baseline row is committed at.
    refused = Hands(sim, trims, way=KILL_BY_FAILURE, scales=SCALES_ONE,
                    period=REACTION_S)
    try:
        refused.selftest()
    except AssertionError as refusal:
        assert refusal.args[0] == (REACTION_S, DELAY_S), refusal.args
    else:
        raise AssertionError("the instrument flew a clock its own selftest"
                             " refuses: %.2f s" % (REACTION_S,))
    say(stream, "the mission's own clock, checked: the %.2f s between its own"
        " decisions is longer than the %.2f s a decision waits on the eyes, and"
        " the instrument's own selftest refuses it - so the person is flown at"
        " the delay, which is the slowest hand the model has"
        % (REACTION_S, DELAY_S))
    return compared


# ---------------------------------------------------------------------------
# The rows, read against the mission's own.
# ---------------------------------------------------------------------------


def row_line(flown, other):
    """One entry of a person's row, its four numbers beside the mission's own."""
    return ("%s ft of catch against %s, a %s m/s descent against %s, an arrival"
            " of %s against %s, and %s %% of rotor at the skids against %s"
            % (words(flown.catch_ft, "%6.1f"), words(other.catch_ft, "%6.1f"),
               words(flown.held_sink, "%5.2f"), words(other.held_sink, "%5.2f"),
               words(flown.touchdown_rate, "%5.2f"),
               words(other.touchdown_rate, "%5.2f"),
               words(flown.rotor_at_touchdown, "%4.1f"),
               words(other.rotor_at_touchdown, "%4.1f")))


def say_the_rows(stream, label, rows, baseline):
    """A set of a person's rows, entry by entry, against the mission's own."""
    say(stream, "")
    say(stream, "%s, beside the mission's own row for the same entry from the"
        " same switch:" % (label,))
    for tag, grip, failure in rows:
        say(stream, "  %s" % (tag,))
        if grip is not None:
            say(stream, "    the grip's own cut: %s"
                % (row_line(grip, baseline[0]),))
        if failure is not None:
            say(stream, "    the failure:        %s"
                % (row_line(failure, baseline[1]),))


def band_words(flown, label, auto):
    """One entry at one grip: whether it held, and how it arrived.

    A flight that never reached the mission's own cut has no entry and no arrival
    to report, and is named as that rather than as a row of blanks: the band is
    where an *entry* stops holding, and a grip the aircraft cannot be climbed at
    is past the edge for the other reason - which is the `no switch` line above it.
    """
    if flown is None:
        return "%s never reached the mission's own cut" % (label,)
    return ("%s %s, arrival %s m/s%s"
            % (label,
               "held the entry" if entry_held(flown, auto)
               else "DID NOT HOLD the entry",
               words(flown.touchdown_rate, "%5.2f"),
               "" if arrived_softly(flown) else " (hard)"))


def say_the_band(stream, legs, trims):
    """The grip's own two edges: the scales that held and the first that did not."""
    auto = trims.descent()[5]
    say(stream, "")
    say(stream, "the grip's own band, the cyclic's two gains scaled from the"
        " mission's own - a scale of one is the mission's pilot, and an entry that"
        " does not reach the manual's own descent inside %.1f m/s of its %5.2f, or"
        " does not arrive under the floor's own %.1f m/s, is an entry the person"
        " could not fly at that grip:" % (DESCENT_BAND_MPS, auto,
                                          UH1_HARD_LANDING_MPS))
    for name, ladder, scales in legs:
        edge = None
        for scale, grip, failure in scales:
            say(stream, "  %-6s x%-5.2f: %s"
                % (name, scale,
                   ", ".join(band_words(flown, label, auto)
                             for label, flown in (("grip cut", grip),
                                                  ("failure", failure)))))
            if edge is None and not (entry_held(grip, auto)
                                     and entry_held(failure, auto)):
                edge = scale
        # And *what* passed the edge, which is not always the entry: a grip the
        # aircraft cannot be climbed at has no entry to have held.
        __, last_grip, last_failure = scales[-1]
        past = never_reached(last_grip, last_failure)
        if edge is None:
            verdict = "every scale this file flew held the entry on both"
        elif past is not None:
            verdict = ("past it the aircraft never reaches the mission's own cut"
                       " at all - %s" % (past,))
        else:
            verdict = ("past it one of the two entries does not reach the"
                       " mission's own descent")
        say(stream, "  so the %s edge of the grip's own band on these two entries"
            " is x%s: %s" % (name, words(edge, "%.2f"), verdict))
    return legs


def say_the_finding(stream, rows, late, band, baseline, trims):
    """What the person is worth, in the words of the numbers above.

    Three things, and each is a row of the transcript read against the mission's
    own: the *clock* - the same hands at twenty decisions a second and at a
    decision a delay, the slowest the instrument will fly, which is what
    separates the rate from the delay and the lag; the *failure noticed late*,
    which is what a surprise costs a person who is flying the climb and not the
    entry; and the *grip's* own two edges, which is how much of this aircraft's
    cyclic the person has in hand on this entry at all.
    """
    say(stream, "")
    say(stream, "the finding:")
    for reaction, grip, failure in rows:
        missing = never_reached(grip, failure)
        if missing is not None:
            say(stream, "  at %.2f s between decisions there is no row: %s never"
                " reached the mission's own cut, so this clock has nothing to"
                " read against the mission's own - see the `no switch` lines"
                % (reaction, missing))
            continue
        say(stream, "  at %.2f s between decisions the person's hand is worth %s ft"
            " of catch, %s s of settling, %s m/s of the descent the entry goes on"
            " to hold, %s m/s of arrival and %s %% of rotor at the skids on the"
            " grip's own cut, against %s ft, %s s, %s m/s, %s m/s and %s %% on the"
            " failure - both read against the mission's own row"
            % (reaction,
               words(diff(baseline[0].catch_ft, grip.catch_ft), "%+6.1f"),
               words(diff(baseline[0].settle_s, grip.settle_s), "%+5.1f"),
               words(diff(baseline[0].held_sink, grip.held_sink), "%+5.2f"),
               words(diff(baseline[0].touchdown_rate, grip.touchdown_rate),
                     "%+5.2f"),
               words(diff(baseline[0].rotor_at_touchdown, grip.rotor_at_touchdown),
                     "%+5.1f"),
               words(baseline[1].catch_ft, "%5.1f"),
               words(baseline[1].settle_s, "%5.1f"),
               words(diff(baseline[1].held_sink, failure.held_sink), "%+5.2f"),
               words(diff(baseline[1].touchdown_rate, failure.touchdown_rate),
                     "%+5.2f"),
               words(diff(baseline[1].rotor_at_touchdown,
                          failure.rotor_at_touchdown), "%+5.1f")))
    if late:
        say(stream, "  a failure nobody announced: the gauge reads the fuel gone"
            " when the governor has been dead %.1f s, the eyes take %.2f s more,"
            " and then what believing it later costs:"
            % (baseline[1].torque_gone_at - baseline[1].switch_s, DELAY_S))
        for hesitation, failure in late:
            say(stream, "    %s s late: %s ft of catch, %s m/s of arrival and %s %%"
                " of rotor at the skids"
                % (words(hesitation, "%4.2f"),
                   words(failure.catch_ft, "%6.1f"),
                   words(failure.touchdown_rate, "%5.2f"),
                   words(failure.rotor_at_touchdown, "%4.1f")))
    auto = trims.descent()[5]
    edges = []
    for name, ladder, scales in band:
        for scale, grip, failure in scales:
            if not (entry_held(grip, auto) and entry_held(failure, auto)):
                edges.append("%s x%.2f" % (name, scale))
                break
        else:
            edges.append("%s beyond every scale flown" % (name,))
    if band:
        say(stream, "  the grip's own band, out of the mission's own gains, is"
            " where an entry stops reaching that descent: %s" % (", ".join(edges),))
    return rows


def check(stream, baseline, rows, late, legs, trims):
    """The claims the transcript makes, asserted: a regression is a traceback.

    Every number here was measured first and written down second, and each is a
    number the printout above already carries.  Three kinds of claim:

    * the *baseline* is the one ``_rt_autoro_entry.py`` committed, to the decimal
      its own transcript prints it at, which is what says the row every number
      below is read against is that file's row and not a cousin of it;
    * the *person's own clock* is the instrument's, measured in flight rather than
      trusted: every view a decision was made from is ``DELAY_S`` old to the last
      bit, the decisions come at the rate asked for, no frame was dropped, and the
      person's own entry is behind the switch's - which is the whole point of
      flying it by hand;
    * and the *finding*, which is the person's own numbers read against the
      mission's: the rows at the clocks the instrument will fly - a clock whose
      hand never reached the cut carrying nothing rather than a flight - the
      failure believed later, and the two edges of the grip's own band.
    """
    auto = trims.descent()[5]
    # The baseline: `_rt_autoro_entry.py`'s own committed numbers, at the one rate
    # it commits them at.  A row that moves is a model or a mission that moved.
    for flown, catch, settle, sink, arrival, rotor in (
            (baseline[0], 231.8, 8.9, 12.30, 2.26, 87.4),
            (baseline[1], 189.7, 7.5, 12.20, 1.90, 71.9)):
        assert abs(flown.catch_ft - catch) < 0.05, (flown.way, flown.catch_ft)
        assert abs(flown.settle_s - settle) < 0.05, (flown.way, flown.settle_s)
        assert abs(flown.held_sink - sink) < 0.005, (flown.way, flown.held_sink)
        assert abs(flown.touchdown_rate - arrival) < 0.005, \
            (flown.way, flown.touchdown_rate)
        assert abs(flown.rotor_at_touchdown - rotor) < 0.05, \
            (flown.way, flown.rotor_at_touchdown)
    assert baseline[1].catch_ft < baseline[0].catch_ft, baseline
    assert baseline[1].settle_s < baseline[0].settle_s, baseline
    # And the clocks those rows were flown at: the instrument's own first, none
    # of them slower than the delay its ``selftest`` allows, and the mission's own
    # script clock - longer than that delay, and refused by that ``selftest`` in
    # `same_ruler` - not among them.
    flown_at = [reaction for reaction, __, __ in rows]
    assert flown_at and flown_at[0] == CLOCKS[0], rows
    assert all(0.0 < reaction <= DELAY_S for reaction in flown_at), rows
    assert REACTION_S > DELAY_S, (REACTION_S, DELAY_S)
    # The instrument's own clock is the file's own row and it is a row: two
    # flights, both of them entries.  A slower clock is allowed to be nothing at
    # all, and where it is, the absence is the finding - see `never_reached`.
    assert rows[0][1] is not None and rows[0][2] is not None, rows[0]
    # The person's own clock, flight by flight: the instrument's two numbers,
    # measured rather than claimed - see `_rt_handling.instrument`.
    for reaction, grip, failure in rows:
        missing = never_reached(grip, failure)
        if missing is not None:
            # Asserted as the absence it is: at a clock the instrument will fly,
            # the hand never got the aircraft into the band the switch waits for,
            # so this is the hand and not the harness.  The clock that matters -
            # the instrument's own - was asserted above to be a row.
            assert reaction > CLOCKS[0], (reaction, missing)
            continue
        for flown in (grip, failure):
            hands = flown.pilot
            ages = hands.ages
            assert ages, (reaction, flown.way)
            assert len(ages) == hands.decisions, (len(ages), hands.decisions)
            assert abs(ages[0] - DELAY_S) < 1e-9, (reaction, flown.way, ages[0])
            assert max(ages) - min(ages) < 1e-9, (min(ages), max(ages))
            assert abs(hands.decisions / flown.sim.sim_time
                       - 1.0 / reaction) <= 1.0, \
                (reaction, flown.way, hands.decisions / flown.sim.sim_time)
            assert not flown.sim.dropped_steps, (reaction, flown.way,
                                                 flown.sim.dropped_steps)
            assert abs(hands.period - reaction) < 1e-12, (hands.period, reaction)
            # The person is behind the switch, always: their entry is a decision
            # or an instrument, and the switch is neither.
            assert hands.cue_at >= flown.switch_s, (reaction, flown.way,
                                                    hands.cue_at, flown.switch_s)
            assert hands.entry_at >= hands.cue_at, (reaction, flown.way,
                                                    hands.entry_at, hands.cue_at)
            assert hands.entry_at > flown.switch_s, (reaction, flown.way,
                                                     hands.entry_at,
                                                     flown.switch_s)
            # And the entry still reached the manual's own descent, with no frame
            # refused - the mission's own criterion for the entry, off the
            # person's own row.  The *arrival* is a separate claim, and it is
            # asserted where the transcript prints it, since on the grip's own cut
            # a hand's cushion is a different flight from a key's.
            assert entry_held(flown, auto), (reaction, flown.way, flown.catch_ft,
                                             flown.held_sink, flown.refused_at)
            assert arrived_softly(flown) == (flown.touchdown_rate
                                             < UH1_HARD_LANDING_MPS), flown.way
    # The failure believed later is worse, and by the height it costs first: the
    # hands are the same hands, the aircraft is the same aircraft, and the only
    # thing that moved is how long the lever stayed at the level trim with the
    # shaft already dead.
    for (hesitation, flown) in late:
        assert hesitation > 0.0, hesitation
        assert flown.catch_ft > rows[0][2].catch_ft, (hesitation, flown.catch_ft,
                                                      rows[0][2].catch_ft)
        assert flown.rotor_at_touchdown < rows[0][2].rotor_at_touchdown, \
            (hesitation, flown.rotor_at_touchdown,
             rows[0][2].rotor_at_touchdown)
    # And the grip's band has two edges, one either side of the mission's own
    # gains: a leg that ran off the end of its ladder has no edge to report.
    for name, ladder, scales in legs:
        assert len(scales) >= 2, (name, [scale for scale, __, __ in scales])
        assert scales[0][0] == ladder[0], (name, scales[0][0], ladder[0])
        assert not (entry_held(scales[-1][1], auto)
                    and entry_held(scales[-1][2], auto)), \
            (name, [scale for scale, __, __ in scales])
    sys.stdout.flush()


def probe(reactions=CLOCKS, hesitations=HESITATIONS, band=True,
          out_path=OUT_PATH):
    """The mission's own row, the person's rows, and the two legs above them.

    Returns ``(baseline, rows, late, legs)``: the mission's own two flights, the
    person's own two rows - one per clock, with ``None`` in the place of a flight
    whose hand never reached the mission's own cut - the failure believed later,
    and the grip's own two ladders, so that a caller can look at anything the
    transcript did not print.
    """
    stream = open(out_path, "w", encoding="utf-8")
    trims = trims_for(airframe_preset(PRESET), AutorotationPilot)
    say(stream, "the autorotation flown by hand: the mission's own pilot beside a"
        " person's hands on the mission's own loops, at %.0f ft of skid and %.0f"
        " kt on the \"%s\" aircraft"
        % (CLIMB_SKID_FT, AUTOROTATION_SPEED_KT, PRESET))
    say(stream, "the person is _rt_axes.Hands: %.0f decisions a second, %.1f s of"
        " transport delay, %.1f s of lag in the hand, and the four axes moved at"
        " the model's own limb rate rather than a key's quarter of it; the twist"
        " grip is not an axis and is the fifth key, %s, held as the mission holds"
        " it"
        % (PILOT_HZ, DELAY_S, T_LAG_S, KEY_NAMES[CUT_GRIP_KEY]))
    for speed in trims.speeds:
        row = trims.at(speed)
        say(stream, "level trim at %5.1f m/s: collective %5.2f in, long %+5.2f,"
            " lat %+5.2f, pedal %+5.2f, pitch %+5.2f deg"
            % (speed, row[0], row[1], row[2], row[3], row[4]))
    descent = trims.descent()
    say(stream, "autorotation trim at %.0f kt: collective %5.2f in, long %+5.2f,"
        " lat %+5.2f, pedal %+5.2f, pitch %+5.2f deg, sink %5.2f m/s (%+6.0f fpm)"
        " - the rate both rows have to reach"
        % (AUTOROTATION_SPEED_KT, descent[0], descent[1], descent[2], descent[3],
           descent[4], descent[5], -descent[5] / FOOT * 60.0))
    same_ruler(stream)

    # The mission's own row first, because everything below is read against it:
    # `_rt_autoro_entry.flight` itself rather than a copy of it, at the rate that
    # file commits its numbers at.
    say(stream, "")
    say(stream, "the mission's own row, at the %.2f s between its own decisions"
        " that its numbers are committed at:" % (REACTION_S,))
    baseline = (scripted(KILL_BY_GRIP, REACTION_S, trims, stream),
                scripted(KILL_BY_FAILURE, REACTION_S, trims, stream))
    say_the_pair(stream, baseline[0], baseline[1], trims)

    # The person, at each of the two clocks.
    rows = []
    for reaction in reactions:
        say(stream, "")
        say(stream, "the person's own row, at %.2f s between decisions: %s"
            % (reaction, clock_words(reaction)))
        grip = flight(KILL_BY_GRIP, trims, stream, reaction=reaction)[1]
        failure = flight(KILL_BY_FAILURE, trims, stream, reaction=reaction)[1]
        missing = never_reached(grip, failure)
        if missing is None:
            say_the_pair(stream, grip, failure, trims)
        else:
            # A hand cannot be flown slower than the loops it has to close: the
            # aircraft never settles in the mission's own band, the switch is
            # never thrown, and there is no entry to read a row out of.  The
            # `no switch` lines above carry what the aircraft was doing instead,
            # and the row stays in the list with nothing in it, because that *is*
            # the answer at this clock.
            say(stream, "  no row at %.2f s between decisions: %s never reached"
                " the mission's own cut, so there is no entry to measure - the"
                " aircraft never held the climb's own band" % (reaction, missing))
        rows.append((float(reaction), grip, failure))

    # The failure believed later than the gauge says.
    late = []
    if hesitations:
        say(stream, "")
        say(stream, "the failure believed later, at the instrument's own clock:"
            " the person's own row above is the same hands at no hesitation, and"
            " these are one, two and three of the mission's decisions after the"
            " gauge has already read the fuel gone")
        for hesitation in hesitations:
            failure = flight(KILL_BY_FAILURE, trims, stream,
                             hesitation=hesitation, trace=False,
                             label="at %.2f s of hesitation" % (hesitation,))[1]
            late.append((float(hesitation), failure))

    # And the grip's own band, both ways from the mission's own gains.
    legs = []
    if band:
        say(stream, "")
        say(stream, "the grip's own band, from the nominal case outward:")
        # What a leg is read against: the autorotation trim's own sink, m/s, taken
        # off the model the way `check` and `say_the_band` take it.  The band is
        # where an entry stops *reaching* that descent, and the arrival is the
        # separate claim above - the trim's row is the six numbers, and the rate
        # is the sixth of them.
        auto = trims.descent()[5]
        for name, ladder in (("harder", SCALES_HARDER),
                             ("softer", SCALES_SOFTER)):
            scales = []
            for scale in ladder:
                one = Scales(scale, SCALES_ONE.lever, SCALES_ONE.pedal)
                label = "the cyclic at x%.2f of the mission's own" % (scale,)
                grip = flight(KILL_BY_GRIP, trims, stream, scales=one,
                              trace=False, label=label)[1]
                failure = flight(KILL_BY_FAILURE, trims, stream, scales=one,
                                 trace=False, label=label)[1]
                scales.append((float(scale), grip, failure))
                # A leg stops where an entry does not hold: the edge is what the
                # leg is for, and past it there is nothing left to measure.
                if not (entry_held(grip, auto) and entry_held(failure, auto)):
                    break
            legs.append((name, ladder, scales))
        say_the_band(stream, legs, trims)

    say_the_rows(stream, "the person's own rows", rows, baseline)
    say_the_finding(stream, rows, late, legs, baseline, trims)
    check(stream, baseline, rows, late, legs, trims)
    say(stream, "")
    say(stream, "check passed: the mission's own row reproduced to the decimal it"
        " was committed at, every person's flight flown at the clock it claims"
        " with every view %.2f s old, every entry that reached the mission's own"
        " cut holding the manual's own descent and arriving inside the floor's"
        " own %.1f m/s, and every clock that never got there said so."
        % (DELAY_S, UH1_HARD_LANDING_MPS))
    stream.close()
    return baseline, rows, late, legs


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

