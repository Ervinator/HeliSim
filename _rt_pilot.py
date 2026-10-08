# -*- coding: utf-8 -*-
"""Scratch: a keyboard pilot with human timing, so the flight can be watched.

Not part of the model.  This is the demo harness that drives ``main.py``'s own
frame loop - its scancodes, ``Simulation.fly`` and the aircraft's own ratchets -
with eight keys and a person's reaction time, so a pickup, a climb, a turn and
a landing happen in a window at sixty frames a second rather than in a
transcript afterwards.

Three ideas and nothing else:

* **the trims are the baseline.**  A pilot flying a UH-1 has a force trim and
  handbook numbers: the hover's 9.09 in of collective, the 8.03 in and +3.6 in
  of cyclic a 47 kt cruise sits on.  :class:`TrimTable` reads those off the model
  itself - the same ``airframe.Airframe.trim_level_flight``
  ``simulation.Simulation.trim_level_flight`` wraps - so the loops below are
  small-signal and only have to catch what the trim does not;
* **the pilot looks every ``REACTION_S`` seconds.**  What the instruments say,
  the four axes the pilot then wants, and the direction each key has to turn are
  a *decision*, and a decision is made at a person's rate, not sixty times a
  second;
* **a key is held until the stick gets there.**  The aircraft answers slowly, so
  the pilot presses the key, watches the axis crawl, and lets go when it reaches
  the position asked for - :meth:`KeyHands.press`, a frame at a time.

Two missions fly on it, each its own class below and each with its own loops:

* :class:`HumanPilot` is the long one - from the pad to a hundred metres up and
  a kilometre out, which is the mission the playability runs flew;
* :class:`HoverTurnPilot` is the short one - a hover twenty feet up over the pad,
  a full turn to the right about the vertical axis, and down again onto the
  skids, which is a minute of flight anybody watching can see the whole of.

Nothing but the eight keys is touched in either, and both read the aircraft at
:data:`REACTION_S` rather than at the frame rate, so what a window shows is a
person flying and not a script stepping.
"""

import math

import pygame

from airframe import FOOT
from simulation import (PAD_COLLECTIVE, SIM_TIME_STEP_S,
                        UH1_CG_HEIGHT_ON_GROUND, PilotInput)

#: A person's reaction time: how often the pilot reads the instruments and moves
#: a key, on the *flight's* own clock - ``Simulation.sim_time`` - and not the
#: frame's, so a dragged window, which the physics keeps at most
#: ``SIM_MAX_STEPS_PER_FRAME`` of, cannot make the pilot look more often per
#: flown second than this.  The playability runs were flown at 0.25 s and 0.35 s.
REACTION_S = 0.25

#: The long mission: a hundred metres up, a kilometre out, and the cruise speed.
ALT_REF_M = 100.0
DIST_REF_M = 1000.0
CRUISE_MPS = 24.0            # about 47 kt, the trim the table is read at

#: Climb on the spot to this height before any of the cruise speed is asked for,
#: and the ten metres over which the speed is wound on.
CLIMB_ALT_M = 80.0
SPEED_RAMP_M = 10.0

#: How close to a hundred metres counts as up there: the loops hold the height
#: asymptotically, so the goal is a band, not a line.
ALT_TOLERANCE_M = 2.0

#: The four axes, in :meth:`simulation.PilotInput.axes` order, and the two keys
#: each of them is on - ``main.pilot_keys``' own scancodes, +key first.
AXIS_KEYS = (
    (pygame.KSCAN_W, pygame.KSCAN_S),        # collective, 0 down .. 1 up
    (pygame.KSCAN_UP, pygame.KSCAN_DOWN),    # long stick, + is nose down
    (pygame.KSCAN_RIGHT, pygame.KSCAN_LEFT),  # lat stick, + is right
    (pygame.KSCAN_D, pygame.KSCAN_A),        # pedals, + is nose right
)

#: One frame of a key on each axis, in axes and in ``PilotInput.axes``' own
#: order: the ratchet's own key rate at one ``simulation.SIM_TIME_STEP_S``
#: (``simulation.PilotInput.key_rates``, whose three rates are the collective,
#: both cyclic axes and the pedals), which is as close as a hand on a key can
#: hold a stick.  The pilot lets go within one of these, so a key is worth one
#: frame's travel and no less - and it is read off the model rather than written
#: down here, so a step granularity changed in ``simulation`` moves this too.
KEY_STEP = tuple(PilotInput().key_rates()[index] * SIM_TIME_STEP_S
                 for index in (0, 1, 1, 2))

#: The travels the inches below are axes in: ``rotor_control``'s own table 2.
COLLECTIVE_TRAVEL_IN = 11.00
LONG_STICK_HALF_IN = 0.5 * 12.90
LAT_STICK_HALF_IN = 0.5 * 12.60
PEDAL_HALF_IN = 0.5 * 6.90

#: The speeds the trim table is read at, m/s: a hover and three on the way out.
TRIM_SPEEDS = (0.0, 8.0, 16.0, 24.0)

# ---------------------------------------------------------------------------
# The short mission: twenty feet up, once round to the right, and down again.
# ---------------------------------------------------------------------------

#: How high the *skids* hover, ft.  Twenty feet up is what a pilot means by a
#: low hover and the skids are what it is measured from; the c.g. the model
#: measures its altitude from stands ``simulation.UH1_CG_HEIGHT_ON_GROUND``
#: higher, on the skids, so the altitude the pilot flies is that much more than
#: the twenty feet - about 24.6 ft of c.g. for 20 ft of skid.
HOVER_SKID_FT = 20.0
HOVER_ALT_M = HOVER_SKID_FT * FOOT + UH1_CG_HEIGHT_ON_GROUND

#: The band the hover counts as held in, m, and how long it has to stay in it
#: before the turn begins: twenty feet is a hover to hold, not a height to pass
#: through on the way somewhere else.
HOVER_TOLERANCE_M = 0.5
HOVER_SETTLE_S = 1.0

#: The turn: the rate the heading asked for marches round at, deg/s, the angle
#: over which that rate is eased away, and the part of it the easing never goes
#: below, so that the schedule *reaches* its 360 rather than approaching it for
#: ever.  Fifteen a second is a slow pedal turn and is what the model's own hover
#: takes: ``_rt_probe`` held half an inch of pedal off the twenty foot trim turns
#: a whole circle in twenty eight seconds, so a quarter of an inch a second of
#: rate is a rate the pedals can be asked for without the machine falling out of
#: its hover.  The easing is there because the machine answers a pedal a second
#: or two after it moves: a heading schedule stopped dead is the nose carrying
#: thirty degrees of overshoot past it, which is most of a minute's wander during
#: which the mission is still turning.  A pilot leads the roll-out for exactly
#: this reason.  And what a finished turn *is*: the schedule home, the nose
#: inside the band of the heading it began from, the rate no more than a settled
#: one, and all three held for a moment, since a 360 that does not stop is a 361.
TURN_RATE_DEG_S = 15.0
TURN_LEAD_DEG = 30.0
TURN_LEAD_FLOOR = 0.25
TURN_TOLERANCE_DEG = 3.0
TURN_RATE_TOLERANCE_DEG_S = 4.0
TURN_SETTLE_S = 1.0

#: What the pedals are asked for: inches of pedal per degree a second of rate
#: the turn wants - read off ``_rt_probe``, where half an inch of it buys twelve
#: to sixteen - and inches per degree the nose is behind the heading it has been
#: asked for and per degree a second it is behind the rate, which are the
#: pilot's own corrections on top of that.  The heading gain is the small one on
#: purpose: the machine answers a pedal slowly, so a pilot who chases the
#: schedule hard is a pilot oscillating around it, and the nose trailing the
#: heading asked for by a dozen degrees is what this hover looks like flown well.
PEDAL_PER_DEG_S = 0.030
K_TURN_IN = 0.040
K_TURN_RATE_IN = 0.020

#: The short mission's own cyclic, because this model's hover is an *unstable*
#: place to leave a helicopter in: at zero airspeed it has no speed damping at
#: all, so a tilt that is not taken out accelerates the machine for as long as it
#: is held - the report's own complaint about its hover, and what ``_rt_probe``
#: shows when it steps the long stick a fifth of an inch and gets a dozen degrees
#: of nose and twenty metres of drift inside eight seconds.  So the pilot holds
#: the *velocity* out with a small tilt towards the pad, damped by the rate the
#: aircraft is already drifting at, and caps the tilt at a few degrees: a hover
#: is held with a nudge, not with a lever.
K_HOVER_POS_DEG_M = 0.20       # deg of tilt per metre off the pad
K_HOVER_VEL_DEG_MPS = 1.5      # deg of tilt per m/s of drift still to take out
HOVER_TILT_LIMIT_DEG = 3.0     # deg of tilt a pilot holding station will use

#: And what the tilt asked for is *held* with: inches of stick per degree of
#: attitude error and per degree a second of rate.  The same shape as the long
#: mission's own two loops, with more rate in them, since the hover is where the
#: report says the model's pitch and roll answers are at their worst.
K_HOVER_PITCH_IN = 0.10
K_HOVER_ROLL_IN = 0.10
K_HOVER_RATE_IN = 0.15

#: Where the pad is, in the world: the origin, which is where ``on_the_pad``
#: parks the aircraft, and so where a mission that begins there is held while
#: the nose goes round.
HOVER_REF_NORTH_M = 0.0
HOVER_REF_EAST_M = 0.0

#: The descent: the rate to come down at, m/s, the height under the skids over
#: which that rate is tapered, and the part of it the taper never goes below -
#: because a lever inside the ground cushion is worth much less than it is in
#: free air (``simulation``'s own trim on the pad is 8.25 in against the 8.99 in
#: the same aircraft hovers at twenty feet on), so a pilot coming down onto the
#: pad keeps pulling until the machine is going down at *something*, and what
#: they are steering by then is the sink rate and not the height.  A rate that
#: tapered to nothing would leave the aircraft hanging in its own cushion, which
#: is what a first rehearsal of this mission did for a hundred seconds.  Both
#: numbers are far under ``simulation.UH1_HARD_LANDING_MPS``, which is what a
#: landing is not.
DESCENT_RATE_MPS = 0.8
DESCENT_TAPER_M = 1.5
DESCENT_TAPER_FLOOR = 0.25

#: How a descent is flown: the lever *creeps* towards the position that gives the
#: sink rate asked for, inches a second for each m/s the sink rate is short of
#: it, and how far from the hover trim a pilot will let it go, either way.  An
#: integrator around the lever rather than a loop around the height, because
#: inside the cushion the height a lever buys is nothing like the height it buys
#: in free air - the same aircraft hovers at twenty feet on 8.99 in and sits on
#: the pad on 8.25 - and a proportional loop aiming at a height through a gain
#: that moves with the height hunts, which is what a first rehearsal of this
#: descent did: four hundred feet a minute either way, over the pad.  It is also
#: what a pilot does: the lever comes down while the VSI is short of the number
#: and stops when it is on it, and what stops the aircraft is the pad.
LEVER_CREEP_IN_S = 0.40
LEVER_LIMIT_IN = 1.5

#: How long the aircraft sits on its skids with the lever back at the pad's own
#: before the mission is called flown: a landing is a settling, like the hover.
LANDING_SETTLE_S = 2.0

#: The pilot's own gains, in the units of the instruments they read:
#: inches of collective per m/s of climb-rate error, inches of cyclic per degree
#: of attitude error and per degree a second of rate, degrees of pitch per m/s of
#: speed error, degrees of bank per metre off the line, and inches of pedal per
#: degree of nose.  Small because the trims underneath them are the big numbers.
K_RATE_IN = 0.80
RATE_GAIN_MPS = 0.12
CLIMB_LIMIT_MPS = 3.0
K_PITCH_IN = 0.10
K_PITCH_RATE_IN = 0.06
K_SPEED_DEG = 0.35
PITCH_LIMIT_DEG = 5.0
K_BANK_DEG = 0.030
BANK_LIMIT_DEG = 10.0
K_ROLL_IN = 0.10
K_ROLL_RATE_IN = 0.05
K_YAW_IN = 0.05
K_YAW_RATE_IN = 0.10

#: The line the mission is flown up, and the altitude below which the aircraft is
#: still on its skids and the collective is the only axis worth moving.
TRACK_EAST_REF_M = 0.0
YAW_REF_DEG = 0.0
PARKED_ALT_M = 2.0


def clip(value, low, high):
    """*value* held between *low* and *high*."""
    return max(low, min(high, value))


def wrap(deg):
    """An angle in degrees, in (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


class TrimTable:
    """The model's own trims, read once: what the handbook hands a pilot.

    A row per speed, in the units the sticks are in - inches - and the pitch
    attitude the trim flies at, which is the reference the pilot's own attitude
    loop is written against.  Between the rows the table interpolates, so a speed
    the mission passes through has a collective and a stick position to match it.

    *airframe* is a scratch aircraft, not the one being flown: solving a trim
    moves an airframe's state, which is not something a pilot may do to the
    aircraft they are in.
    """

    def __init__(self, airframe, speeds=TRIM_SPEEDS, altitude=ALT_REF_M):
        self.speeds = tuple(float(speed) for speed in speeds)
        self.rows = []
        for speed in self.speeds:
            controls, state = airframe.trim_level_flight(speed,
                                                         altitude=altitude)
            self.rows.append((controls.collective, controls.long_stick,
                              controls.lat_stick, controls.pedal,
                              state.attitude_deg.y))

    def at(self, speed):
        """``(collective, long, lat, pedal, pitch)`` the model flies *speed* on."""
        speed = float(speed)
        if speed <= self.speeds[0]:
            return self.rows[0]
        if speed >= self.speeds[-1]:
            return self.rows[-1]
        for index in range(1, len(self.speeds)):
            if speed <= self.speeds[index]:
                low, high = self.speeds[index - 1], self.speeds[index]
                fraction = (speed - low) / (high - low)
                return tuple(a + fraction * (b - a)
                             for a, b in zip(self.rows[index - 1],
                                             self.rows[index]))
        return self.rows[-1]

    def hover_fraction(self):
        """The collective's hover trim as a fraction of its travel, 0 to 1."""
        return self.rows[0][0] / COLLECTIVE_TRAVEL_IN


def inches_to_axes(collective, long_stick, lat_stick, pedal):
    """Four stick positions in inches as the normalised axes a key ratchets."""
    return [clip(collective / COLLECTIVE_TRAVEL_IN, 0.0, 1.0),
            clip(long_stick / LONG_STICK_HALF_IN, -1.0, 1.0),
            clip(lat_stick / LAT_STICK_HALF_IN, -1.0, 1.0),
            clip(pedal / PEDAL_HALF_IN, -1.0, 1.0)]


class KeyHands:
    """The eight keys, held until the stick gets where the pilot asked.

    The pilot's four wanted axis positions come in; the keys that turn each axis
    towards its position go out, in ``main.frame_keys``' own shape - scancodes
    to 1.0 - so the frame loop cannot tell this from a keyboard.  A key is let
    go inside one frame's worth of travel of the position asked for, which is
    the finest a key can be: any finer and the ratchet would chatter.
    """

    def __init__(self, want=None, step=KEY_STEP):
        self.want = list(want) if want is not None else None
        self.step = tuple(step)
        self.keys = {}

    def press(self, axes):
        """The keys for a frame, given the four axes the pilot's hands are on."""
        keys = {}
        if self.want is not None:
            for index, (up_key, down_key) in enumerate(AXIS_KEYS):
                error = self.want[index] - axes[index]
                if error > self.step[index]:
                    keys[up_key] = 1.0
                elif error < -self.step[index]:
                    keys[down_key] = 1.0
        self.keys = keys
        return keys


class KeyboardPilot:
    """A pilot on a keyboard, of whichever mission is below it.

    The frame loop's whole view of a pilot is three things, and they are here
    rather than in each mission: the hands (:class:`KeyHands`), a look at the
    instruments every :data:`REACTION_S` seconds rather than every frame, and the
    record of what the mission has done.  A mission is :meth:`decide` - where the
    four axes ought to be, from what the instruments say - :meth:`mission_done`,
    and :meth:`note`, and nothing else.

    A decision is made at a person's rate; the keys a decision leaves are held
    from frame to frame until the sticks get where the pilot asked, which is
    :class:`KeyHands` and is why this is a *pilot* and not a script: two people
    flying the same eight keys see the same aircraft answer.
    """

    #: The speeds this pilot's trim table is read at, m/s, and the altitude it is
    #: read at: a pilot's force trim is the baseline every loop below is written
    #: around, and it is the model's own answer for where this mission flies.
    #: See :func:`trims_for`, which builds the table off a scratch aircraft.
    TRIM_SPEEDS_MPS = TRIM_SPEEDS
    TRIM_ALT_M = ALT_REF_M

    def __init__(self, sim, trims, reaction=REACTION_S, log=None):
        self.reaction = float(reaction)
        self.trims = trims
        self.log = log
        self.hands = KeyHands(list(sim.pilot.axes()))
        self.want = self.hands.want
        self.since_look = self.reaction          # look at the first frame
        self.last_sim_time = float(sim.sim_time)  # the flight clock, not the frame's
        self.decisions = 0
        self.done = None

    # -- the frame loop's view of it -------------------------------------

    def keys(self, sim, dt):
        """The keys for one frame: the hands, read at the pilot's own rate.

        The clock read here is the *flight's* own - ``Simulation.sim_time`` -
        and not the frame's *dt*: a dragged window is a longer frame and the
        physics keeps at most ``SIM_MAX_STEPS_PER_FRAME`` of it, so counting
        wall time would look more often per flown second than a person can.
        *dt* is accepted because the frame loop hands it in, and is otherwise
        unused.
        """
        now = float(sim.sim_time)
        self.since_look += max(now - self.last_sim_time, 0.0)
        self.last_sim_time = now
        if self.mission_done(sim):
            self.hands.want = None               # let go of everything
            return self.hands.press(sim.pilot.axes())
        if self.since_look >= self.reaction:
            self.since_look = 0.0
            self.want = self.decide(sim)
            self.hands.want = self.want
            self.decisions += 1
            if self.log is not None:
                self.log.decision(sim, self.want)
        return self.hands.press(sim.pilot.axes())

    def decide(self, sim):
        """Where the four axes ought to be, from what the instruments say."""
        raise NotImplementedError

    def mission_done(self, sim):
        """Is the mission over?  The hands come off everything when it is."""
        raise NotImplementedError

    def note(self, sim):
        """The pilot's own words for a caption: what the mission is doing."""
        raise NotImplementedError


class HumanPilot(KeyboardPilot):
    """The long mission: from the pad to a hundred metres up and a kilometre out.

    Four loops, one per axis, each written as a position in the sticks' own
    inches around the trim the model flies that speed on:

    * the collective holds the height, through a climb rate -
      ``height error -> rate -> collective`` - with the height-rate term leading
      the two seconds of heave between the lever and the answer;
    * the long stick holds the speed: a pitch attitude to fly, and the stick
      position that attitude needs;
    * the lateral stick holds the line north of the pad, through a bank;
    * the pedals hold the nose, which is the *only* thing holding it here - see
      the model's own directional stability.

    Everything is clipped to what a person would ask for: three metres a second
    of climb, six degrees of pitch, ten of bank.
    """

    DESCRIPTION = ("a hundred metres up and a kilometre out at the 47 kt trim")

    def __init__(self, sim, trims, reaction=REACTION_S, cruise_mps=CRUISE_MPS,
                 log=None):
        KeyboardPilot.__init__(self, sim, trims, reaction=reaction, log=log)
        self.cruise_mps = float(cruise_mps)
        self.took_off = None
        self.climbed = None

    # -- the pilot's own loops -------------------------------------------

    def decide(self, sim):
        """Where the four axes ought to be, from what the instruments say."""
        telemetry = sim.telemetry()
        state = sim.airframe.state
        axes = list(sim.pilot.axes())
        alt = telemetry.alt_agl
        if sim.on_ground and alt < PARKED_ALT_M:
            # Still on the skids: the collective is the only axis that does
            # anything, and the cyclic and pedals stay where the pad left them.
            return [self.trims.hover_fraction()] + axes[1:]

        speed = telemetry.ground_speed
        speed_want = self.cruise_mps * clip(
            (alt - CLIMB_ALT_M) / SPEED_RAMP_M, 0.0, 1.0)
        # The baseline is the trim of the speed the aircraft is *at*, not of the
        # speed it is being asked for: a pilot's force trim is wound on with the
        # aircraft, so the pedals do not arrive at the cruise's -0.05 in while the
        # machine is still hovering at 3 m/s - which is a bootful of nose-right
        # the pilot would then have to take out again, and a bow out of the line.
        # The speed asked for reaches the loops only through the pitch below.
        collective_t, long_t, lat_t, pedal_t = self.trims.at(speed)[:4]

        # Vertical: a rate to ask for, and the lever position that holds it.
        rate_want = clip(RATE_GAIN_MPS * (ALT_REF_M - alt),
                         -CLIMB_LIMIT_MPS, CLIMB_LIMIT_MPS)
        collective = (collective_t
                      + K_RATE_IN * (rate_want - telemetry.height_rate))

        # Fore and aft: the pitch the speed asked for needs, then the stick for it.
        # The attitude is aimed at the trim of the speed asked for - a stated
        # altitude for the nose to settle on - while the stick it is measured
        # against is the trim of the speed the aircraft is actually at, so the
        # wind-on is led by the nose and not by a stick position the aircraft has
        # not earned yet.  At the end of the ramp the two speeds agree and the
        # pilot is sitting on the cruise trim, hands still.
        pitch_want_t = self.trims.at(speed_want)[4]
        pitch_want = clip(pitch_want_t - K_SPEED_DEG * (speed_want - speed),
                          pitch_want_t - PITCH_LIMIT_DEG,
                          pitch_want_t + PITCH_LIMIT_DEG)
        long_stick = (long_t
                      + K_PITCH_IN * (telemetry.pitch_deg - pitch_want)
                      + K_PITCH_RATE_IN * telemetry.pitch_rate_deg)

        # Sideways: the line north of the pad, as a bank, then the stick for it.
        east = state.position.y - TRACK_EAST_REF_M
        bank_want = clip(-K_BANK_DEG * east, -BANK_LIMIT_DEG, BANK_LIMIT_DEG)
        lat_stick = (lat_t
                     + K_ROLL_IN * (bank_want - telemetry.roll_deg)
                     - K_ROLL_RATE_IN * telemetry.roll_rate_deg)

        # And the nose, which is the pedals' whole business.
        pedal = (pedal_t
                 - K_YAW_IN * wrap(telemetry.yaw_deg - YAW_REF_DEG)
                 - K_YAW_RATE_IN * telemetry.yaw_rate_deg)

        return inches_to_axes(collective, long_stick, lat_stick, pedal)

    # -- the frame loop's view of it -------------------------------------

    def mission_done(self, sim):
        """A hundred metres up and a kilometre out: the goal, and the hand-over."""
        if self.done is None:
            state = sim.airframe.state
            distance = math.hypot(state.position.x, state.position.y)
            if not sim.on_ground and self.took_off is None:
                self.took_off = sim.sim_time
            if self.climbed is None and (sim.telemetry().alt_agl
                                        >= ALT_REF_M - ALT_TOLERANCE_M):
                self.climbed = sim.sim_time
            if self.climbed is not None and distance >= DIST_REF_M:
                self.done = sim.sim_time
        return self.done is not None

    def note(self, sim):
        """Where the pilot is in the mission, in their own words."""
        if sim.crashed:
            return ("%s at %.1f s, %s"
                    % ("parked" if self.took_off is None else "flying",
                       sim.sim_time, sim.telemetry().crash_message))
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


class HoverTurnPilot(KeyboardPilot):
    """The short mission: up to twenty feet, once round to the right, down again.

    The same four axes as :class:`HumanPilot`, flown off the trim a twenty foot
    hover settles on, with the mission itself a phase - ``pickup``, ``hover``,
    ``turn``, ``descent``, ``landed`` - and nothing else changing:

    * the collective holds the height, through a climb rate as before, with the
      height asked for moving from the pad to twenty feet under the skids and,
      once the turn is done, back down to the pad on a descent rate the skids
      can take;
    * the pedals fly the turn: the heading asked for marches from the heading the
      nose is on to 360 deg past it at :data:`TURN_RATE_DEG_S`, and that schedule
      is eased away over its last degrees because the machine answers a pedal a
      second or two after it moves.  The rate the schedule is asking for is fed
      forward off the model's own number, and the pilot holds the nose on it
      exactly as they hold it on any other heading.  The schedule arriving back
      where it began, with the nose on it and still, is the turn being over;
    * the cyclic holds the machine over the pad and is flown through the turn as
      well: this model's hover is an unstable place to leave a helicopter in -
      see the constants above - and a turn on a trim'd cyclic wanders, since the
      same model couples its lateral cyclic into its yaw.

    Nothing here is the model's.  A phase is a pilot's own bookkeeping, kept at
    the pilot's own look rate; every position still goes out as the eight keys'
    own key states, through the same :class:`KeyboardPilot` frame loop, and the
    world underneath is ``main.py``'s own frame loop, camera and view.
    """

    TRIM_SPEEDS_MPS = (0.0,)
    TRIM_ALT_M = HOVER_ALT_M

    DESCRIPTION = ("twenty feet up over the pad, once round to the right about"
                   " the vertical axis, and down again onto the skids")

    def __init__(self, sim, trims, reaction=REACTION_S, log=None):
        KeyboardPilot.__init__(self, sim, trims, reaction=reaction, log=log)
        self.phase = "pickup"
        self.took_off = None           # s, when the skids left the pad
        self.hover_at = None           # s, when the band of twenty feet was met
        self.hover_alt = None          # m, the altitude it was met at
        self.in_band_since = None      # s, since it was last *in* that band
        self.turn_begun = None         # s, the schedule's own start
        self.turn_at = None            # s, the last step the schedule took
        self.turn_heading_deg = sim.telemetry().yaw_deg     # what the turn goes
        #                                                     round from and to
        self.turn_settled = None       # s, since the nose was on the heading
        self.turn_done = None          # s, the turn called done
        self.turned_deg = 0.0          # deg, the ramp's own position, 0 to 360
        self.turn_error_deg = None     # deg, how far off the heading it ended
        self.descent_at = None         # s, the descent begun
        self.descent_lever = None      # in, the lever the descent is creeping
        self.last_airborne_rate = 0.0  # m/s, the last rate read off the ground
        self.touchdown = None          # s, when the skids met the pad again
        self.touchdown_rate = None     # m/s, the rate the pilot last read aloft
        self.landed_since = None       # s, since the skids were down
        # Where the pad holds the four controls, which is where a flown mission
        # leaves them: the run's own trim with the lever at
        # ``simulation.PAD_COLLECTIVE``, which is the state
        # :meth:`simulation.Simulation.on_the_pad` parks the aircraft in - and
        # so the state this mission ends in.
        self.pad_axes = list(sim.trim_controls.collective_at(
            PAD_COLLECTIVE).to_axes())

    # -- the mission's own bookkeeping -----------------------------------

    def phase_of(self, sim):
        """What the mission is doing, from where the aircraft is.

        One step per look, since a phase is a decision like any other: the
        aircraft is on its skids or it is not, it is inside the band of twenty
        feet or it is not, the nose is on the heading it was asked for or it is
        not.  Nothing here reads the frame, and nothing here is the model's.
        """
        telemetry = sim.telemetry()
        alt = telemetry.alt_agl
        now = sim.sim_time
        if self.took_off is None and not sim.on_ground:
            self.took_off = now                   # the skids are off the pad
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
                self.in_band_since = now
                self.hover_alt = alt
                if self.hover_at is None:
                    self.hover_at = now
            elif now - self.in_band_since >= HOVER_SETTLE_S:
                self.phase = "turn"
                self.turn_begun = now
                self.turn_heading_deg = telemetry.yaw_deg
        if self.phase == "turn":
            # The schedule: the heading asked for marching round to the right at
            # the turn's own rate, eased over its last degrees, stepped once per
            # look rather than read off the clock - see the constants above.
            if self.turn_at is None:
                self.turn_at = now
            turn_rate = TURN_RATE_DEG_S * clip(
                (360.0 - self.turned_deg) / TURN_LEAD_DEG, TURN_LEAD_FLOOR, 1.0)
            self.turned_deg = min(360.0, self.turned_deg
                                  + turn_rate * (now - self.turn_at))
            self.turn_at = now
            heading_want = self.turn_heading_deg + self.turned_deg
            self.turn_error_deg = wrap(heading_want - telemetry.yaw_deg)
            if (self.turned_deg >= 360.0
                    and abs(self.turn_error_deg) <= TURN_TOLERANCE_DEG
                    and abs(telemetry.yaw_rate_deg)
                    <= TURN_RATE_TOLERANCE_DEG_S):
                if self.turn_settled is None:
                    self.turn_settled = now
                elif now - self.turn_settled >= TURN_SETTLE_S:
                    self.phase = "descent"
                    self.turn_done = now
                    self.descent_at = now
            else:
                self.turn_settled = None
        if self.phase in ("pickup", "hover", "turn", "descent"):
            # What only a frame can see, and what the arrival on the pad has to
            # be judged by: the height rate the pilot last read in the air.
            self.last_airborne_rate = telemetry.height_rate
        if self.phase == "descent" and sim.on_ground:
            self.phase = "landed"
            self.touchdown = now
            self.touchdown_rate = self.last_airborne_rate
            self.landed_since = now
        if self.phase == "landed" and not sim.on_ground:
            # Light on its skids again - the lever is still up where the descent
            # left it - so the descent is not over after all.
            self.phase = "descent"
            self.landed_since = None

    # -- the pilot's own loops -------------------------------------------

    def decide(self, sim):
        """Where the four axes ought to be, from what the instruments say.

        One loop for the whole mission, since the phases change what the
        aircraft is *asked* for rather than how it is asked: the height asked
        for, whether the heading asked for is staying put or marching round, and
        where over the pad the machine is being kept.  On the skids the lever is
        the only control with any weight on it - the skids hold the aircraft level
        and still however much the cyclic and the pedals are waved about - so a
        pickup is the collective's doing and the rest of it comes along with it.
        """
        telemetry = sim.telemetry()
        state = sim.airframe.state
        alt = telemetry.alt_agl
        self.phase_of(sim)
        if self.phase == "landed":
            # Down and staying down: the lever back to the pad's own and the
            # cyclic and the pedals back on the trim, which is the state a
            # mission begins from - ``on_the_pad``'s own sticks.
            return list(self.pad_axes)

        collective_t, long_t, lat_t, pedal_t, pitch_t = self.trims.at(0.0)

        # Vertical: the height asked for, and the rate to get there.  Climbing is
        # the long mission's own loop, clipped like a hand.  Coming down is the
        # lever *creeping* down and up until the VSI reads what the mission asked
        # for, which is a rate loop around the lever rather than a height loop
        # around the cushion - see the constants above for why, and for what it
        # is that stops the aircraft.
        if self.phase in ("pickup", "hover", "turn"):
            height_want = HOVER_ALT_M
            rate_want = clip(RATE_GAIN_MPS * (height_want - alt),
                             -CLIMB_LIMIT_MPS, CLIMB_LIMIT_MPS)
            collective = (collective_t
                          + K_RATE_IN * (rate_want - telemetry.height_rate))
        else:
            rate_want = -DESCENT_RATE_MPS * clip(
                (alt - UH1_CG_HEIGHT_ON_GROUND) / DESCENT_TAPER_M,
                DESCENT_TAPER_FLOOR, 1.0)
            if self.descent_lever is None:
                self.descent_lever = collective_t     # the descent's own start
            self.descent_lever = clip(
                self.descent_lever - LEVER_CREEP_IN_S * self.reaction
                * (telemetry.height_rate - rate_want),
                collective_t - LEVER_LIMIT_IN, collective_t + LEVER_LIMIT_IN)
            collective = self.descent_lever

        # The cyclic: the machine held over the pad.  Where it has drifted to is
        # taken in the aircraft's *own* axes - the nose is going round and the pad
        # is not - and answered with a tilt towards the pad and against the drift
        # still to take out, which is the whole of how a hover is kept: at zero
        # airspeed the model has nothing else that stops a machine.  The tilt is
        # small (see the constants above) and the stick that tilt needs is the
        # trim's own plus the two corrections, exactly as the long mission flies
        # its speed.  ``velocity.x`` is forward and ``.y`` starboard.
        #
        # It is flown through the turn as well, and has to be: the model couples
        # lateral cyclic into yaw - ``_rt_probe`` steps half an inch of it and
        # gets thirty degrees a second - so a turn with the cyclic on the trim
        # wanders twenty eight metres doing it, which the first rehearsal of this
        # mission did.
        heading = math.radians(telemetry.yaw_deg)
        north_off = HOVER_REF_NORTH_M - state.position.x
        east_off = HOVER_REF_EAST_M - state.position.y
        forward_off = (north_off * math.cos(heading)
                       + east_off * math.sin(heading))
        right_off = (-north_off * math.sin(heading)
                     + east_off * math.cos(heading))
        pitch_want = clip(pitch_t - K_HOVER_POS_DEG_M * forward_off
                          + K_HOVER_VEL_DEG_MPS * state.velocity.x,
                          pitch_t - HOVER_TILT_LIMIT_DEG,
                          pitch_t + HOVER_TILT_LIMIT_DEG)
        roll_want = clip(K_HOVER_POS_DEG_M * right_off
                         - K_HOVER_VEL_DEG_MPS * state.velocity.y,
                         -HOVER_TILT_LIMIT_DEG, HOVER_TILT_LIMIT_DEG)
        long_stick = (long_t
                      + K_HOVER_PITCH_IN * (telemetry.pitch_deg - pitch_want)
                      + K_HOVER_RATE_IN * telemetry.pitch_rate_deg)
        lat_stick = (lat_t
                     + K_HOVER_ROLL_IN * (roll_want - telemetry.roll_deg)
                     - K_HOVER_RATE_IN * telemetry.roll_rate_deg)

        # The pedals: the rate the turn wants - which is what the machine is
        # turned with, and which is fed forward off ``_rt_probe``'s own number
        # rather than left for the two corrections to find, and which comes off
        # over the last few degrees of the turn - the heading the schedule is
        # carrying the nose round towards, and the rate still wanted to get
        # there.  Once the schedule is home the pedals are a heading hold like any
        # other, since a 360 that does not stop is a 361.
        heading_want = self.turn_heading_deg + self.turned_deg
        yaw_rate_want = 0.0
        if self.phase == "turn":
            # The rate the schedule is *still* asking for, which is zero once it
            # is home - the easing's floor is the schedule's own, see the
            # constants, and a rate asked for past the end of a turn is a nose
            # held off its heading for ever.
            yaw_rate_want = TURN_RATE_DEG_S * clip(
                (360.0 - self.turned_deg) / TURN_LEAD_DEG, 0.0, 1.0)
        pedal = (pedal_t
                 + PEDAL_PER_DEG_S * yaw_rate_want
                 + K_TURN_IN * wrap(heading_want - telemetry.yaw_deg)
                 + K_TURN_RATE_IN * (yaw_rate_want - telemetry.yaw_rate_deg))

        return inches_to_axes(collective, long_stick, lat_stick, pedal)

    # -- the frame loop's view of it -------------------------------------

    def mission_done(self, sim):
        """The pad has the aircraft and the controls again, and keeps them.

        The skids down, the lever brought back to the pad's own, the trundle of
        settling done - on the ground and staying there - and the sticks within
        one frame of a key of where the pad has them, which is as close as a key
        can place a stick.  Held for :data:`LANDING_SETTLE_S`, since a landing
        is a settling too.
        """
        if self.done is None and self.phase == "landed" and sim.on_ground:
            axes = sim.pilot.axes()
            on_the_pad = all(abs(axis - want) <= step
                             for axis, want, step
                             in zip(axes, self.pad_axes, KEY_STEP))
            if (on_the_pad and self.landed_since is not None
                    and sim.sim_time - self.landed_since >= LANDING_SETTLE_S):
                self.done = sim.sim_time
        return self.done is not None

    def note(self, sim):
        """Where the pilot is in the mission, in their own words."""
        telemetry = sim.telemetry()
        skid_ft = (telemetry.alt_agl - UH1_CG_HEIGHT_ON_GROUND) / FOOT
        if sim.crashed:
            return ("%s at %.1f s, %s"
                    % (self.phase, sim.sim_time, telemetry.crash_message))
        if self.done is not None:
            return ("flown in %.0f s: %.0f ft up over the pad, %.0f deg round"
                    " to the right, down again at %.0f fpm"
                    % (self.done, HOVER_SKID_FT, self.turned_deg,
                       self.touchdown_rate / FOOT * 60.0))
        if self.phase == "pickup":
            return ("picking up off the pad, %4.1f ft up, %d decisions"
                    % (skid_ft, self.decisions))
        if self.phase == "hover":
            return ("climbing to %.0f ft, %4.1f ft up, %+5.0f fpm,"
                    " %d decisions" % (HOVER_SKID_FT, skid_ft,
                                       telemetry.height_rate_fpm,
                                       self.decisions))
        if self.phase == "turn":
            return ("the turn, %.0f of 360 deg to the right, %+5.0f deg/s,"
                    " %4.1f ft up, %d decisions"
                    % (self.turned_deg, telemetry.yaw_rate_deg, skid_ft,
                       self.decisions))
        if self.phase == "descent":
            return ("coming down, %4.1f ft up, %+5.0f fpm, %d decisions"
                    % (skid_ft, telemetry.height_rate_fpm, self.decisions))
        return ("down on the pad, lever coming back to the pad's own,"
                " %d decisions" % (self.decisions,))


#: The missions a caller may ask for by name, and the one that is flown when
#: nobody says: the short one, since it is a minute of flight and the whole of
#: it happens over the pad.
MISSIONS = {"hover-turn": HoverTurnPilot, "cruise": HumanPilot}
DEFAULT_MISSION = "hover-turn"


def trims_for(airframe, pilot_class):
    """The trim table *pilot_class* flies on: its own speeds at its own height.

    *airframe* is a scratch aircraft of the caller's - solving a row moves it,
    which is why it is never the one being flown.  See :class:`TrimTable`.
    """
    return TrimTable(airframe, speeds=pilot_class.TRIM_SPEEDS_MPS,
                     altitude=pilot_class.TRIM_ALT_M)



