# -*- coding: utf-8 -*-
"""Scratch: a person's hands in the loop, which is phase 1's instrument.

Not part of the model, and imported by nothing in it.  Every harness beside this
one flies the aircraft with a *script*: ``_rt_pilot`` decides four axes off the
instruments and holds a key until the ratchet gets there, ``_rt_probe`` steps a
stick off its trim and reads what the model answered, ``_rt_check`` flies the
missions and asserts what they did.  None of them asks the question a
handling-qualities programme is actually about - whether a person's hands, made
of a clock, a pair of eyes and a forearm, can hold the machine at all - and none
of them can say how much room is left before they cannot.

That is this file.  The model's own twenty foot hover, closed around a human's
dynamics instead of a script's:

* **a discrete pilot.**  Twenty decisions a second, on the flight's own clock,
  with the four axes held between them - a person, not a sixty hertz loop;
* **a transport delay.**  What the pilot decides on is what the instruments read
  ``DELAY_S`` ago, kept as the last :data:`DELAY_FRAMES` states in a deque, so
  the loop is closed around a *remembered* aircraft and not the one it is flying;
* **a gain, a lead and a lag.**  Inches of stick per degree of attitude error,
  ``T_LEAD_S`` of lead on the rate - which is the model's own hover pilot's ratio
  of rate gain to attitude gain, so a scale of one *is* that pilot - and
  ``T_LAG_S`` of neuromuscular lag between the stick asked for and the stick the
  hand has actually moved.

What the loops underneath are is ``_rt_pilot.HoverTurnPilot``'s hover, gain for
gain, and the gains are read off that module rather than written down again: the
tilt a hover is held with (position and velocity, to a three degree cap), the
attitude held with the cyclic, the height on the lever and the heading on the
pedals.  What is *not* that pilot's is the clock, the delay and the scale the two
attitude gains are multiplied by; everything below the attitude loop stays at the
model's own numbers, since the question here is how hard a pilot may grip the
*attitude* and not how fast the outer loops may be.

Three measurements, which are the phase's whole answer:

* **can it be held at all.**  A hover trimmed, two degrees of attitude put into
  it, and the pilot's job to get back inside ``SETTLE_DEG``: the peak it reached,
  how long the settling took, the height band it cost and the drift;
* **the gain margin.**  The same perturbation with the two attitude gains scaled
  up and down from the model's own until a grip fails, which gives the *band* a
  person can hold this hover in rather than a single number - ``SCALES_HARDER``
  and ``SCALES_SOFTER`` are where the two edges are and why they fail
  differently;
* **the PIO.**  Whether the failure is the pilot's own loop: an oscillation at a
  period inside :data:`PIO_PERIOD_S`, growing by :data:`PIO_GROWTH` a cycle for
  :data:`PIO_CYCLES` cycles, and - the number that makes it the pilot's and not
  the aircraft's - the phase that ``DELAY_S`` of delay is worth at that period,
  ``360 * DELAY_S / period`` degrees.  At the model's own gains there is none,
  and the band's own edge is not one either; a grip twice the model's own is.

The transcript is ``_rt_handling.txt``, written by the run itself, because the
shell's own redirection is not worth trusting on this box; it is ignored by
``.gitignore``'s ``_rt_*.txt`` beside the other harnesses' own.  The run asserts
what it prints, so a regression is a traceback and not a table nobody read: what
this file measures is the acceptance criterion for the phases after it, and a
criterion nobody can fail is not one.

Run it with ``python _rt_handling.py``, a minute or two.
"""


import math
import sys
from collections import deque, namedtuple

from _rt_pilot import (CLIMB_LIMIT_MPS, HOVER_ALT_M, HOVER_REF_EAST_M,
                       HOVER_REF_NORTH_M, HOVER_TILT_LIMIT_DEG,
                       K_HOVER_PITCH_IN, K_HOVER_POS_DEG_M, K_HOVER_RATE_IN,
                       K_HOVER_ROLL_IN, K_HOVER_VEL_DEG_MPS, K_RATE_IN,
                       K_TURN_IN, K_TURN_RATE_IN, RATE_GAIN_MPS, clip,
                       inches_to_axes, skid_ft, wrap)
from airframe import Vector3
from main import AIRFRAME_PRESET
from simulation import (SIM_TIME_STEP_S, Simulation, airframe_preset)

#: The transcript, written by the run itself: see the module docstring.
LOG_FILE = "_rt_handling.txt"

#: The pilot's clock: decisions a second, and how long the flight's own time
#: takes to reach the eyes.  Twenty a second and a fifth of a second are the
#: optimistic end of what a pilot model is flown at on purpose - the margin this
#: measures is the margin a *good* pilot has, and a real one has less.
PILOT_HZ = 20.0
PILOT_PERIOD_S = 1.0 / PILOT_HZ
DELAY_S = 0.2

#: The delay in frames, and so the length of the history a decision reads:
#: twelve frames of the sixty hertz step, which is the 0.2 s exactly.
DELAY_FRAMES = int(round(DELAY_S / SIM_TIME_STEP_S))

#: The loop's own gain and lead, and they are the model's hover pilot's own:
#: inches of stick per degree of attitude error, the same for pitch and for roll
#: as that pilot has it, and the *ratio* of its rate gain to its attitude gain,
#: which is the lead in seconds the loop is closed with.  A scale of one is
#: therefore exactly the pilot ``_rt_check`` flies the short mission with, and
#: every case below is a multiple of it - see :meth:`Hands.selftest`.
K_THETA_IN = K_HOVER_PITCH_IN
T_LEAD_S = K_HOVER_RATE_IN / K_HOVER_PITCH_IN

#: The lag the hand itself has, s: the stick asked for is not the stick moved,
#: and a first-order lag is what a forearm is.  It is the smallest of the three
#: dynamic terms and it is here because the term a PIO actually lives in is this
#: one and the delay above it.
T_LAG_S = 0.1

#: The perturbation: two degrees, which is the attitude a pilot loses in a gust
#: and the two degrees the report's own hover figures are read around.  Both
#: tilt axes are flown, since a hover is held with two hands.
PERTURB_DEG = 2.0

#: The scale the nominal case is flown at - the model's own pilot, exactly - and
#: the two legs the band is read off, in the order they are flown: from the
#: nominal *up* until a grip fails, and then from the nominal *down* until one
#: does.  Both edges are real and they fail *differently*, which is why there are
#: two.  Too hard a grip closes the pilot's own loop on its own delay: the hover
#: oscillates, and at 1.28 s a cycle the delay is worth 56 degrees of phase, so
#: what is growing is a pilot induced oscillation and not an aircraft mode.  Too
#: soft a one cannot hold the tilt the outer loops ask the cyclic for, so the
#: machine drifts off and the attitude error goes with it - a slow divergence of
#: ten seconds, the far side of the same band.  One number called "the gain
#: margin" would report the first of those and hide the second, and a pilot
#: between the two has a band, not a margin.
SCALE_NOMINAL = 1.0
SCALES_HARDER = (SCALE_NOMINAL, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0)
SCALES_SOFTER = (SCALE_NOMINAL, 0.75, 0.5, 0.35, 0.25)


#: How long the *frozen* case is flown, s: long enough for a pair of hands that
#: are not on the controls to show where the model's own hover goes, which is
#: faster than that.
FROZEN_S = 20.0

#: How long a case is flown, s, and what settling means: an attitude error
#: inside ``SETTLE_DEG`` of the trim's own attitude and *held* there for
#: ``SETTLE_S``, which is ``_rt_pilot``'s own way of calling a band held.
HOLD_S = 40.0
SETTLE_DEG = 0.5
SETTLE_S = 2.0

#: What a case's *response* is measured from: the first half second of one is
#: the perturbation travelling through the pilot's own 0.2 s of delay into the
#: stick, and not yet the loop's answer to it, so the worst error the response
#: is read at is taken from here on.  Without it every case's worst error is the
#: two degrees that were put in and every case looks the same at the top.
OVERSHOOT_AFTER_S = 0.5

#: What the rest of a case has to stay inside for the *hover* to count as held:
#: the height band the short mission's own check uses, and the drift it allows
#: itself over the whole of a minute's flying.
HEIGHT_LIMIT_M = 2.0
DRIFT_LIMIT_M = 10.0

#: The tail of a case a gain margin is read off: whether the loop is still
#: growing at the end of the run is whether the worst error in its last third is
#: worse than the worst error in its middle third.  A run that is dying out has
#: less left to say than it did; one that is not, has more.
TAIL_FRACTION = 3.0

#: What counts as an oscillation, and as a *pilot induced* one: a period inside
#: the band a person's own loop lives in - a second or two either side - that
#: grows by ``PIO_GROWTH`` a cycle or better for ``PIO_CYCLES`` cycles running.
#: Any oscillation is not a PIO; one the pilot's own delay has closed the loop
#: on is, which is why the phase the delay is worth at the period is printed
#: beside it - 0.2 s is 72 degrees of a one second oscillation and 36 of a two.
PIO_PERIOD_S = (0.5, 5.0)
PIO_GROWTH = 1.02
PIO_CYCLES = 3

#: What counts as a *peak* of the error when the oscillations are read: an error
#: inside the settling band is not one - a loop's own band is where it is
#: supposed to be - and two peaks closer together than this are one peak
#: sampled twice, since twenty hertz of decision puts a wiggle of its own on a
#: trace that is not the aircraft.
PEAK_FLOOR_DEG = SETTLE_DEG
MIN_PEAK_GAP_S = 0.2

#: How often a line of the transcript is printed, frames: every second of flight.
EVERY = 60

#: The slack the pilot's clock is compared against, s.  A frame is a third of the
#: period to the last bit - three of them come to 0.049999999999999996 - so a
#: pilot that asked for its decision at exactly the period would look every
#: *fourth* frame and fly at fifteen decisions a second while claiming twenty.
#: A nanosecond is small enough that nothing else in a second can hide in it, and
#: :func:`instrument` measures the rate the slack buys rather than trusting it.
PILOT_CLOCK_SLACK_S = 1e-9

#: What one frame hands the pilot: the flight's clock, the instruments as they
#: read then, and the two pieces of the airframe's own state the hover loops need
#: - where it is over the pad in NED, and its velocity in *body* axes, which is
#: the pair ``_rt_pilot`` holds a hover with.  A decision reads one of these,
#: ``DELAY_FRAMES`` frames old, and not the aircraft itself.
Snapshot = namedtuple("Snapshot", "time telemetry north east vx vy")


class Transcript(object):
    """``sys.stdout`` for the run: the console, and a file that outlives it."""

    def __init__(self, path):
        self.stream = open(path, "w", encoding="utf-8")

    def write(self, text):
        self.stream.write(text)
        self.stream.flush()
        sys.__stdout__.write(text)
        sys.__stdout__.flush()

    def flush(self):
        self.stream.flush()
        sys.__stdout__.flush()

    def close(self):
        self.stream.close()


# ---------------------------------------------------------------------------
# The pilot: a clock, a delay, a gain, a lead and a lag.
# ---------------------------------------------------------------------------


class Hands(object):
    """A person's hands on four axes: the model's hover pilot, slowed down.

    :meth:`decide` is ``_rt_pilot.HoverTurnPilot.decide``'s hover, loop for loop
    and gain for gain - the tilt a hover is held with, the attitude held with the
    cyclic, the height on the lever, the heading on the pedals - read off a
    :class:`Snapshot` of the aircraft as it was ``DELAY_S`` ago rather than off
    the aircraft itself, with the two attitude gains multiplied by *scale*.

    Around that sit the three things that make the loops a person's rather than
    a script's:

    * the decision is made every ``period`` seconds of the *flight's* own clock,
      which is sixty hertz frames flown and not wall time, so a dragged window
      cannot make the pilot look more often per flown second than a person can;
    * what it looks at is the oldest state in a ``DELAY_FRAMES`` long deque of
      them - the transport delay, which closes the pilot's own loop on a fifth
      of a second of memory;
    * the stick the decision asks for is not the stick the aircraft gets: the
      command is filtered through a first-order lag of ``T_LAG_S``, and the
      hand's *own* rate - ``simulation.PilotInput.set_axes``' 1.6 of full travel
      a second, which is the model's statement of how fast a limb is and not
      this file's - limits whatever is left of it.

    The lead is not a filter: it is the rate term in the attitude loop every one
    of these pilots has, and ``K_THETA_IN * T_LEAD_S`` is ``K_HOVER_RATE_IN`` to
    the last bit, which :meth:`selftest` proves.  A scale of one is therefore the
    model's own hover pilot and a scale of two that pilot gripping twice as hard,
    which is what the margin below is measured in.
    """

    def __init__(self, sim, trim, scale=SCALE_NOMINAL, k_theta=K_THETA_IN,
                 t_lead=T_LEAD_S, t_lag=T_LAG_S, delay=DELAY_FRAMES,
                 period=PILOT_PERIOD_S,
                 pad=(HOVER_REF_NORTH_M, HOVER_REF_EAST_M)):
        telemetry = sim.telemetry()
        angles = sim.trim_state.attitude_deg
        #: What the pilot counts from: the stick the model's own trim is on, and
        #: the attitude, height, heading and place it is trimmed at.  None of
        #: these is written down here - a hover pilot's baseline is the aircraft
        #: it was handed.
        self.trim = tuple(float(value) for value in trim)
        self.pitch_ref = angles.y
        self.roll_ref = angles.x
        self.heading_ref = angles.z
        self.alt_ref = telemetry.alt_agl
        self.pad = (float(pad[0]), float(pad[1]))
        #: The loop itself: the attitude gain, scaled, and its lead and lag, s.
        self.k_pitch = float(k_theta) * float(scale)
        self.k_roll = K_HOVER_ROLL_IN * float(scale)
        self.t_lead = float(t_lead)
        self.t_lag = float(t_lag)
        #: The pilot's own clock, eyes and hands.
        self.period = float(period)
        self.history = deque(maxlen=int(delay))
        self.want = None                  # four inches: the last decision
        self.command = list(self.trim)    # where the hands actually are
        self.since_look = self.period     # look at the first frame
        self.last_time = 0.0
        self.decisions = 0
        self.ages = []                    # how old the view of each decision was

    def selftest(self):
        """Prove the model's own hover pilot *is* this pilot at a scale of one.

        The two numbers below the clock are that pilot's own: the attitude gain
        is read off it and the lead is the ratio of its two gains, so the only
        thing this file adds to the loop is the timing.  Move those gains in
        ``_rt_pilot`` and the lead here moves with them; write a rate gain there
        that is not the attitude gain's lead and this says so, which is the check
        that keeps the two pilots the same pilot.
        """
        assert abs(K_THETA_IN * T_LEAD_S - K_HOVER_RATE_IN) < 1e-12, \
            (K_THETA_IN, T_LEAD_S, K_HOVER_RATE_IN)
        assert self.history.maxlen == DELAY_FRAMES
        assert abs(self.history.maxlen * SIM_TIME_STEP_S - DELAY_S) < 1e-12, \
            self.history.maxlen
        assert self.period <= DELAY_S, (self.period, DELAY_S)
        return self

    def fill(self, sim, count=None):
        """Fill the eyes with the aircraft as it is, for the delay to read from.

        Called *before* a case's perturbation, and the entries are dated
        *backwards* from now - the oldest ``DELAY_S`` ago, the newest a frame ago
        - because that is what they are: the aircraft as it was, which is what a
        pilot looking at instruments that lag by a fifth of a second is looking
        at.  A case's first decision is then made on the trim rather than on the
        perturbation, which is the whole difference between this pilot and the
        scripts beside it, and every decision reads a view exactly ``DELAY_S``
        old - the one thing :meth:`selftest` cannot check and
        :func:`instrument` does.
        """
        count = self.history.maxlen if count is None else int(count)
        for step in range(count, 0, -1):
            self.history.append(self.snapshot(
                sim, float(sim.sim_time) - step * SIM_TIME_STEP_S))
        return self

    def view(self):
        """The instruments as the pilot has them: the oldest state kept."""
        if len(self.history) < self.history.maxlen:
            return None
        return self.history[0]

    def snapshot(self, sim, time=None):
        """What one frame hands the pilot: see :class:`Snapshot`."""
        state = sim.airframe.state
        return Snapshot(float(sim.sim_time) if time is None else float(time),
                        sim.telemetry(), state.position.x, state.position.y,
                        state.velocity.x, state.velocity.y)

    def push(self, sim):
        """The aircraft as of now, into the history the delay reads."""
        self.history.append(self.snapshot(sim))
        return self

    def decide(self, view):
        """The four sticks wanted, in inches, from what the instruments said.

        ``_rt_pilot.HoverTurnPilot.decide``'s hover, loop for loop - see that
        method for why each one has the shape it has.  Two things differ: every
        number here is read out of *view*, which is the aircraft as it was
        ``DELAY_S`` ago, and the two cyclic gains are this pilot's own rather
        than that module's constants.
        """
        telemetry = view.telemetry
        collective_t, long_t, lat_t, pedal_t = self.trim[:4]

        # Vertical: the height the trim sits at, and the rate to hold it with.
        rate_want = clip(RATE_GAIN_MPS * (self.alt_ref - telemetry.alt_agl),
                         -CLIMB_LIMIT_MPS, CLIMB_LIMIT_MPS)
        collective = collective_t + K_RATE_IN * (rate_want - telemetry.height_rate)

        # The tilt that holds station over the pad, taken in the aircraft's own
        # axes - the nose is what is turning, the pad is not - and answered with
        # a tilt towards the pad against the drift still to be taken out.
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

        # And the stick that tilt is held with: the trim's own, plus the
        # attitude error and the rate - the loop's gain and its lead, in inches,
        # and the same shape for both cyclic axes.
        long_stick = (long_t
                      + self.k_pitch * (telemetry.pitch_deg - pitch_want)
                      + self.k_pitch * self.t_lead * telemetry.pitch_rate_deg)
        lat_stick = (lat_t
                     + self.k_roll * (roll_want - telemetry.roll_deg)
                     - self.k_roll * self.t_lead * telemetry.roll_rate_deg)

        # The pedals: the heading the trim is on, held, and the rate it is
        # turning at held out - a hover does not turn.
        pedal = (pedal_t
                 + K_TURN_IN * wrap(self.heading_ref - telemetry.yaw_deg)
                 + K_TURN_RATE_IN * (0.0 - telemetry.yaw_rate_deg))
        return [collective, long_stick, lat_stick, pedal]

    def move(self, dt):
        """The stick the hand has actually moved: the command, one lag behind.

        A first-order lag rather than a rate: what a forearm does to a step is
        set off at a speed and slow down into the position, which is what
        ``dt / (T_LAG_S + dt)`` an axis per frame is.  A ``T_LAG_S`` of zero
        makes the hands exact, which is the script every other harness flies.
        """
        if self.want is None:
            return list(self.command)
        if self.t_lag > 0.0:
            step = dt / (self.t_lag + dt)
            for index in range(4):
                self.command[index] += (self.want[index]
                                        - self.command[index]) * step
        else:
            self.command = list(self.want)
        return list(self.command)

    def frame(self, sim, dt):
        """One frame of a person flying: look when it is due, move, fly.

        The order *is* the delay: a decision reads the history as it stands and
        this frame's own state is put into it afterwards, so what a decision
        looks at is ``DELAY_FRAMES`` frames old to the frame it is made on -
        a fifth of a second, exactly, which is what :meth:`selftest` and the
        case's own record of ``ages`` are here to keep true.
        """
        now = float(sim.sim_time)
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
        return sim.fly(dt)


# ---------------------------------------------------------------------------
# What a case measured: the error, the stick, the drift, and whether it settled.
# ---------------------------------------------------------------------------


class Trace(object):
    """One case, frame by frame, and the answers read back out of it.

    The axis the perturbation was put into is the axis everything here is about:
    the error against the trim's own attitude, its rate, the stick the hand had
    *moved* (the lagged command, not the one asked for), the height above the
    trim's own, and how far the machine wandered off the pad.

    The four answers - and they are the three measurements of the module
    docstring, the first of them split in two:

    * :meth:`settled` - when the error came inside the settling band and stayed
      there, or ``None``, which is the whole of what a gain margin is;
    * :meth:`tail_growth` - whether the error is still growing at the end of the
      case, one number, which is what says a scale is past the margin;
    * :meth:`peaks` and what is read off them - the period of what is left, how
      much it grows a cycle, and so whether it is a PIO;
    * :meth:`altitude_error` and the drift - whether the *hover* was held while
      the attitude was being argued about, since an attitude loop that recovers
      the nose while the machine descends out of the sky has not recovered
      anything.
    """

    def __init__(self, sim, hands, axis, seconds):
        self.sim = sim
        self.hands = hands
        self.axis = axis                    # "pitch" or "roll"
        self.seconds = float(seconds)
        self.index = 1 if axis == "pitch" else 2
        self.reference = (hands.pitch_ref if axis == "pitch"
                          else hands.roll_ref)
        self.times = []
        self.errors = []                    # deg, of this axis, against trim
        self.rates = []                     # deg/s
        self.sticks = []                    # in, the stick the hand moved
        self.altitudes = []
        self.drifts = []
        self.lowest = sim.telemetry().alt_agl
        self.highest = self.lowest
        self.drift = 0.0

    def measure(self):
        """One frame's worth, from the state the frame loop has just left."""
        telemetry = self.sim.telemetry()
        state = self.sim.airframe.state
        if self.axis == "pitch":
            angle, rate = telemetry.pitch_deg, telemetry.pitch_rate_deg
        else:
            angle, rate = telemetry.roll_deg, telemetry.roll_rate_deg
        drift = math.hypot(state.position.x, state.position.y)
        self.times.append(float(self.sim.sim_time))
        self.errors.append(angle - self.reference)
        self.rates.append(rate)
        self.sticks.append(self.hands.command[self.index])
        self.altitudes.append(telemetry.alt_agl)
        self.drifts.append(drift)
        self.lowest = min(self.lowest, telemetry.alt_agl)
        self.highest = max(self.highest, telemetry.alt_agl)
        self.drift = max(self.drift, drift)
        return self

    def settled(self, band=SETTLE_DEG, hold=SETTLE_S):
        """When the error came inside *band* and stayed, s, or ``None``.

        The band has to be *held* and not merely passed through: an error that
        crosses it on its way somewhere worse has not settled, and a case that
        never holds it - because the loop grew, or flew into the ground first -
        is the answer a gain margin is read off rather than a failure of this.
        """
        needed = int(round(hold / SIM_TIME_STEP_S))
        if needed < 1 or len(self.errors) < needed:
            return None
        for start in range(len(self.errors) - needed + 1):
            if all(abs(error) <= band
                   for error in self.errors[start:start + needed]):
                return self.times[start]
        return None

    def overshoot(self, after=OVERSHOOT_AFTER_S):
        """The worst error of the *response*, deg, and when it was, s.

        Taken from ``after`` seconds on, because what the first half second of a
        case holds is the perturbation arriving at the pilot's eyes through the
        delay and not the loop's answer to it: an error that is only ever the two
        degrees that were put in says nothing at all about a gain margin.
        """
        worst = 0.0
        when = 0.0
        for time, error in zip(self.times, self.errors):
            if time < after:
                continue
            if abs(error) > abs(worst):
                worst, when = error, time
        return worst, when

    def peaks(self, floor=PEAK_FLOOR_DEG, gap=MIN_PEAK_GAP_S):
        """What is left of the error once it is oscillating: ``[(time, deg)]``.

        The local maxima of the error's size that are *outside* the settling
        band - what a loop does inside its own band is where it is supposed to
        be and is not an oscillation - and at least *gap* apart, because twenty
        hertz of decisions puts a wiggle on a trace that the aircraft did not.
        Two of those closer together than the gap are one peak sampled twice, and
        the bigger of them is kept.
        """
        found = []
        needed = max(int(round(gap / SIM_TIME_STEP_S)), 1)
        for index in range(1, len(self.errors) - 1):
            size = abs(self.errors[index])
            if size < floor:
                continue
            if (size < abs(self.errors[index - 1])
                    or size < abs(self.errors[index + 1])):
                continue
            if found and index - found[-1][0] < needed:
                if size > found[-1][1]:
                    found[-1] = (index, size, self.times[index])
                continue
            found.append((index, size, self.times[index]))
        return [(time, size) for _, size, time in found]

    def period(self, count=4):
        """The mean period of the last *count* peaks, s, or ``None``.

        ``None`` for a case with fewer than two peaks to measure between, which
        is what a case that settled has.
        """
        found = self.peaks()
        if len(found) < 2:
            return None
        times = [time for time, _ in found[-(count + 1):]]
        gaps = [later - earlier for earlier, later in zip(times, times[1:])]
        return sum(gaps) / len(gaps) if gaps else None

    def growth(self, count=PIO_CYCLES):
        """How much the error grows a cycle, over the last *count* cycles.

        Above one is growing and below it is dying out; ``None`` when there are
        not that many cycles left to read, which is a case with no problem to
        report rather than one this cannot say anything about.
        """
        found = self.peaks()
        ratios = [later / earlier for (_, earlier), (_, later)
                  in zip(found, found[1:]) if earlier > 0.0]
        if len(ratios) < count:
            return None
        tail = ratios[-count:]
        return sum(tail) / len(tail)

    def pio(self):
        """``(is it one, period, growth)``: an oscillation the pilot closed.

        Inside :data:`PIO_PERIOD_S` - the band a person's own loop lives in -
        and growing by :data:`PIO_GROWTH` a cycle or better.  What makes it
        *pilot induced* rather than a mode of the aircraft is not asserted here
        but read: :meth:`phase_deg` says what the 0.2 s of delay is worth at that
        period, and a loop that is growing where the delay costs tens of degrees
        of phase is a loop the pilot's own lag is closing.
        """
        period, growth = self.period(), self.growth()
        if period is None or growth is None:
            return False, period, growth
        inside = PIO_PERIOD_S[0] <= period <= PIO_PERIOD_S[1]
        return (inside and growth >= PIO_GROWTH), period, growth

    @staticmethod
    def phase_deg(period):
        """What the pilot's own delay is worth at *period*, deg: ``360*0.2/T``."""
        return 360.0 * DELAY_S / period if period else None

    def tail_growth(self):
        """The worst error in the last third against the worst in the middle.

        The one number that says whether a case is still growing at the end of
        it: a loop that is dying out has less left to say than it had and one
        that is not has more, so a scale whose tail is worse than its middle is a
        scale past the margin - and one whose tail is worse *because it crashed*
        is past it by very much more.
        """
        third = int(len(self.errors) / TAIL_FRACTION)
        if third < 1:
            return None
        middle = max(abs(error) for error in self.errors[third:2 * third])
        last = max(abs(error) for error in self.errors[2 * third:])
        return last / middle if middle > 0.0 else float("inf")

    def altitude_error(self):
        """The worst the height got either side of the trim's own, m."""
        return max(abs(self.lowest - self.hands.alt_ref),
                   abs(self.highest - self.hands.alt_ref))

    def stick_range(self):
        """How much of the axis's stick the case spent, in: the peak to peak."""
        return (max(self.sticks) - min(self.sticks)) if self.sticks else 0.0

    def worst_rate(self):
        """The worst rate of the axis, deg/s: a hover's real violence."""
        return max((abs(rate) for rate in self.rates), default=0.0)

    def __str__(self):
        overshoot, when = self.overshoot()
        period, growth = self.period(), self.growth()
        settled = self.settled()
        return ("%s: worst %+0.2f deg since %.2f s (at %5.2f s), settled %s |"
                " alt %+0.2f m, drift %4.2f m, stick %0.2f in | period %s,"
                " growth %s"
                % (self.axis, overshoot, OVERSHOOT_AFTER_S, when,
                   "%5.2f s" % settled if settled is not None else "never",
                   self.altitude_error(), self.drift, self.stick_range(),
                   "%5.2f s" % period if period else "none",
                   "%4.2f" % growth if growth else "none"))


# ---------------------------------------------------------------------------
# A case: the hover trimmed, two degrees of attitude put in, and hands on it.
# ---------------------------------------------------------------------------


def hover_setup(axis="pitch", deg=PERTURB_DEG, scale=SCALE_NOMINAL):
    """The model's own hover, a pilot built around it, and *deg* of *axis* in.

    The aircraft is trimmed to the hover at :data:`HOVER_ALT_M`; the pilot is
    built around *that* trim - its stick, its attitude, its height and its
    heading, all read off the aircraft rather than written down here - and handed
    the last fifth of a second of it, which is what a pilot who has been flying
    the hover has in their eyes.  Only then is the perturbation put in: a new
    attitude, and the trim's own stick, so that what follows a case is a person's
    answer to two degrees and not a script's.

    Returns ``(sim, trim, hands)`` with the perturbation already in the aircraft,
    the hands on the trim's own stick, and neither of them flown yet.
    """
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.trim_level_flight(0.0, altitude=HOVER_ALT_M)
    trim = (sim.trim_controls.collective, sim.trim_controls.long_stick,
            sim.trim_controls.lat_stick, sim.trim_controls.pedal)
    hands = Hands(sim, trim, scale=scale)
    hands.selftest().fill(sim)
    # What the pilot can see before the case begins has to be the trim: the
    # perturbation goes in *after* the eyes are filled, so the first fifth of a
    # second of every case is flown on what the aircraft was and not what it is.
    eyes = hands.view()
    assert eyes is not None, "the history was not filled"
    assert abs(eyes.telemetry.pitch_deg - hands.pitch_ref) < 1e-9, \
        eyes.telemetry.pitch_deg
    # An attitude is a frozen triple - roll, pitch, yaw - so the perturbation is
    # a new one rather than a field written into: what goes into the reset is a
    # state, whole.
    state = sim.trim_state.copy()
    roll, pitch, yaw = state.attitude.as_tuple()
    if axis == "pitch":
        pitch += math.radians(deg)
    else:
        roll += math.radians(deg)
    state.attitude = Vector3(roll, pitch, yaw)
    sim.reset(state=state, controls=sim.trim_controls)
    # The hands begin on the trim's own stick, which is where that reset has put
    # them: nobody moves a cyclic to reach a position it is already in.
    assert all(abs(one - want) < 1e-12 for one, want
               in zip(sim.pilot.axes(), inches_to_axes(*trim))), sim.pilot.axes()
    return sim, trim, hands


def case_line(sim, trace, axis):
    """One line of a case's progress: the axis, its stick, the height, the pad."""
    position = sim.airframe.state.position
    return ("    t %4.1f s | %s %+5.2f deg %+6.1f deg/s | stick %+5.2f in |"
            " %5.2f ft skid | %5.2f m out"
            % (sim.sim_time, axis, trace.errors[-1], trace.rates[-1],
               trace.sticks[-1], skid_ft(sim.telemetry()),
               math.hypot(position.x, position.y)))


def fly_case(scale=SCALE_NOMINAL, axis="pitch", deg=PERTURB_DEG, seconds=HOLD_S,
             every=EVERY):
    """Fly the hover with a person's hands on it, from a *deg* perturbation.

    :func:`hover_setup` for the condition and :class:`Hands` for the pilot; what
    this adds is the flight, and it is the case every number below is measured
    on.  Returns ``(sim, hands, trace)``.  *every* prints a line of the case's
    own progress that many frames apart, or nothing for ``None`` - which is what
    the sweeps fly at, a line a second of a dozen cases being a transcript nobody
    reads.
    """
    sim, trim, hands = hover_setup(axis=axis, deg=deg, scale=scale)

    trace = Trace(sim, hands, axis, seconds)
    for frame in range(int(round(seconds / SIM_TIME_STEP_S))):
        hands.frame(sim, SIM_TIME_STEP_S)
        trace.measure()
        if every and frame % every == 0:
            print(case_line(sim, trace, axis))
            sys.stdout.flush()
        if sim.crashed:
            break
    return sim, hands, trace


def frozen_case(axis="pitch", deg=PERTURB_DEG, seconds=FROZEN_S, every=EVERY):
    """The same perturbation with *nobody* flying it: the model's instability.

    The sticks are where the trim left them and no pilot moves them - a
    :class:`Hands` is built, as the trace needs one, and never flown, which is
    exactly what a pair of hands off the controls is - so what follows is the
    aircraft's own answer to two degrees of attitude and nothing else.  It is the
    reference both edges of the band are read against, because the hover this
    model trims to is not a place it stays: how fast it leaves is why a pilot's
    gain has a *low* edge as well as a high one, and the pilot's own drift beside
    it is what the loops below are buying.
    """
    sim, trim, hands = hover_setup(axis=axis, deg=deg)
    trace = Trace(sim, hands, axis, seconds)
    for frame in range(int(round(seconds / SIM_TIME_STEP_S))):
        sim.fly(SIM_TIME_STEP_S)      # no keys held: the sticks do not move
        trace.measure()
        if every and frame % every == 0:
            print(case_line(sim, trace, axis))
            sys.stdout.flush()
        if sim.crashed:
            break
    return sim, hands, trace


def case(scale=SCALE_NOMINAL, axis="pitch", every=EVERY, label=None):
    """Fly one case at *scale*, print what it did, and hand back its trace."""
    print("")
    print("--- %s: %+0.2f deg of %s, attitude gain x%0.2f (%.3f in/deg, lead"
          " %.2f s, lag %.2f s)"
          % (label or "case", PERTURB_DEG, axis, scale, K_THETA_IN * scale,
             T_LEAD_S, T_LAG_S))
    sim, hands, trace = fly_case(scale=scale, axis=axis, every=every)
    print("  %s" % (trace,))
    print("  the hands: %d decisions (%0.1f a second), %0.2f in of stick peak"
          " to peak, worst %0.1f deg/s | height %+0.2f to %+0.2f m of the trim,"
          " %d dropped steps"
          % (hands.decisions, hands.decisions / max(sim.sim_time, 1e-9),
             trace.stick_range(), trace.worst_rate(),
             trace.lowest - hands.alt_ref, trace.highest - hands.alt_ref,
             sim.dropped_steps))
    if sim.crashed:
        print("  CRASHED at %.2f s: %s" % (sim.sim_time, sim.crash_reason))
    sys.stdout.flush()
    assert not sim.dropped_steps, sim.dropped_steps
    return trace


def instrument(seconds=5.0):
    """Fly a short case and hold the pilot itself to the clock it claims.

    Two numbers that everything below is multiplied by, and both are measured in
    flight rather than asserted on paper: the age of every view a decision was
    made from, which has to be the ``DELAY_S`` of the delay to the last bit, and
    the number of decisions, which has to be the pilot's own rate over the time
    flown.  The whole run also has to have been flown without the frame loop
    dropping a step - a harness whose clock is not what it says makes every
    number under it meaningless, so this is the first thing a run does.
    """
    print("")
    print("--- the instrument itself: the pilot's own clock over %.0f s of a case"
          % seconds)
    sim, hands, trace = fly_case(seconds=seconds, every=None)
    flown = trace.times[-1] + SIM_TIME_STEP_S
    ages = hands.ages
    print("  %0.2f s flown: %d decisions (%0.2f a second, the pilot's own is"
          " %0.0f) | every view %0.4f s old (the delay is %0.2f) | %s"
          % (flown, hands.decisions, hands.decisions / flown, PILOT_HZ,
             ages[0] if ages else float("nan"), DELAY_S,
             "no step dropped" if not sim.dropped_steps
             else "%d STEPS DROPPED" % sim.dropped_steps))
    assert ages, "the pilot never looked"
    assert len(ages) == hands.decisions, (len(ages), hands.decisions)
    assert abs(ages[0] - DELAY_S) < 1e-9, ages[0]
    assert max(ages) - min(ages) < 1e-9, (min(ages), max(ages))
    assert abs(hands.decisions / flown - PILOT_HZ) <= 1.0, hands.decisions / flown
    assert not sim.dropped_steps, sim.dropped_steps
    sys.stdout.flush()
    return trace


def sweep_leg(axis, scales, harder):
    """One direction of the band: from the nominal until a grip fails.

    *held* is three things and not one: the error came back inside the settling
    band and stayed (``Trace.settled``), it is still dying out at the end of the
    case (``Trace.tail_growth``), and the machine is in the sky to have done it.
    A grip that settles at eight seconds and then wanders back out of the band by
    thirty has not held anything - the 1.5 case below is exactly that - so a
    criterion of "it settled once" would call it good and put the edge a scale
    too high.

    Returns ``(edge, pio_at, rows)``: a row's ``(scale, held, trace)``, the edge
    the first scale that failed, and - on the harder leg only - the first scale
    whose *failure* is a pilot induced oscillation rather than the model's own
    slow mode.  The harder leg therefore keeps climbing past the edge until the
    pilot's own loop is what has closed, which is the phase's PIO question: the
    edge of the band and the grip the hover becomes a PIO at are two different
    numbers and both of them are worth having.
    """
    print("  -- gripped %s than the model's own pilot:"
          % ("harder" if harder else "softer"))
    edge = None
    pio_at = None
    rows = []
    for scale in scales:
        sim, hands, trace = fly_case(scale=scale, axis=axis, every=None)
        settled = trace.settled()
        worst, _ = trace.overshoot()
        tail, period, growth = trace.tail_growth(), trace.period(), trace.growth()
        is_pio = trace.pio()[0]
        held = (settled is not None and not sim.crashed
                and tail is not None and tail <= 1.0)
        rows.append((scale, held, trace))
        print("  %5.2f %+6.1f %+7.2f %10s %10s %9s %8s %8.2f %8.2f%s"
              % (scale, 20.0 * math.log10(scale), worst,
                 "%6.2f s" % settled if settled is not None else "never",
                 "%6.2f" % tail if tail is not None else "none",
                 "%6.2f s" % period if period else "none",
                 "%6.2f" % growth if growth else "none",
                 trace.altitude_error(), trace.drift,
                 "  <- the edge" if not held and edge is None else
                 "  <- the PIO" if is_pio and pio_at is None else ""))
        sys.stdout.flush()
        if not held:
            print("  this grip did not hold: %s"
                  % ("the machine left the sky at %.2f s" % sim.sim_time
                     if sim.crashed else
                     "the error never held inside %0.2f deg for %.1f s"
                     % (SETTLE_DEG, SETTLE_S) if settled is None else
                     "it settled at %.2f s and then grew back out of the band,"
                     " to %.1f times its middle third"
                     % (settled, tail)))
            if edge is None:
                edge = scale
            if is_pio:
                pio_at = scale
                print("  and its failure is a pilot induced oscillation: the"
                      " pilot's own %.2f s of delay is worth %.0f deg of a"
                      " %.2f s period, so the loop that has closed is the"
                      " pilot's own and not the aircraft's."
                      % (DELAY_S, Trace.phase_deg(period), period))
            if not harder or pio_at is not None:
                break
    if edge is None:
        print("  every grip in this leg held.")
    elif harder and pio_at is None:
        print("  the edge is x%.2f and nothing up to x%.2f is a pilot induced"
              " oscillation: what fails at the edge is the aircraft's own slow"
              " mode and not the pilot's loop."
              % (edge, scales[-1]))
    return edge, pio_at, rows


def sweep(axis="pitch"):
    """Fly both legs of the gain band and hand back the two edges.

    This hover has *two* edges and not one - ``SCALES_HARDER``/``SCALES_SOFTER``
    are where the difference between them is written down - so the band is
    reported as the pair it is: how much harder than the model's own pilot a
    grip may be before the hover stops coming back, and how much softer before
    it does not come back at all.  Inside the harder leg, the scale the hover
    turns into a pilot induced oscillation at is read as well, which is further
    up than the edge: see :func:`sweep_leg`.

    Returns ``(margin, pio_at, floor, harder_rows, softer_rows)``, with an edge
    ``None`` if its leg never found one.
    """
    print("")
    print("=== the gain band: %+0.2f deg of %s, the two attitude gains scaled ==="
          % (PERTURB_DEG, axis))
    print("  scale      dB    worst   settled    tail/mid   period   growth"
          "    alt m   drift m")
    margin, pio_at, up = sweep_leg(axis, SCALES_HARDER, True)
    floor, _, down = sweep_leg(axis, SCALES_SOFTER, False)
    return margin, pio_at, floor, up, down


def pio_report(scale, trace):
    """Say what is oscillating at *scale*, and whether it is a PIO.

    The period and the growth are the trace's own; what makes the answer a
    *pilot induced* oscillation is the number printed beside them, which is what
    the pilot's own fifth of a second of delay is worth at that period.  A loop
    that is growing where the delay costs tens of degrees of phase is a loop the
    pilot's own lag has closed, and no aircraft mode has anything to do with it.
    """
    is_pio, period, growth = trace.pio()
    if period is None:
        print("  x%0.2f %s: nothing left to oscillate - the error came back"
              " inside the band and stayed there" % (scale, trace.axis))
        sys.stdout.flush()
        return False
    if is_pio:
        reason = "A PILOT INDUCED OSCILLATION"
    elif not (PIO_PERIOD_S[0] <= period <= PIO_PERIOD_S[1]):
        reason = ("not a PIO: %.2f s is outside the %.1f to %.1f s band a"
                  " person's own loop lives in"
                  % (period, PIO_PERIOD_S[0], PIO_PERIOD_S[1]))
    else:
        reason = ("not a PIO: %.2f a cycle is dying out, and the PIO's own line"
                  " is %.2f" % (growth, PIO_GROWTH))
    print("  x%0.2f %s: period %.2f s, growing %.2f a cycle, and the pilot's own"
          " %.2f s of delay is worth %.0f deg of that period - %s"
          % (scale, trace.axis, period, growth, DELAY_S,
             Trace.phase_deg(period), reason))
    sys.stdout.flush()
    return is_pio


# ---------------------------------------------------------------------------
# What the run measured, and so what it is held to.
# ---------------------------------------------------------------------------

#: The nominal case, as this file first flew it, and the tolerance it is
#: asserted within: these are a model's numbers and not definitions.  The pitch
#: loop is the slower of the two by a long way - two degrees of pitch come back
#: inside the settling band in eight and a half seconds and two of roll in two -
#: and both hold the height to a centimetre and the machine inside two and a half
#: metres of the pad, which is the *hover* being held while the attitude is
#: argued about.  Two and a bit metres of pad for two degrees of pitch, at a
#: twenty foot hover, is what a pilot's hands cost: see the hands-off case beside
#: it, which is where the model's own hover goes with nobody flying it.
NOMINAL_PITCH_SETTLED_S = 8.48
NOMINAL_ROLL_SETTLED_S = 2.02
NOMINAL_DRIFT_M = 2.5
NOMINAL_HEIGHT_M = 0.05
SETTLE_TOLERANCE_S = 0.5

#: The band, as the two legs measured it: the hardest grip of the model's own
#: attitude gains that still held the hover, the softest one that did, the scale
#: each leg failed first at - and then the scale the hover turns into a pilot
#: induced oscillation at, which is *not* the band's edge.
#:
#: The model's own hover pilot is the *hardest* grip that holds.  A quarter
#: softer (x0.75) still holds; x0.50 does not - there the attitude error wanders
#: about a degree either side at a 7.7 s period that no longer decays at all, and
#: the machine leaves the pad while it wanders.  On the hard side x1.50 still
#: comes back inside the band, at 7.9 s, and then grows straight back out of it
#: on an 11.5 s mode - and the pilot's own fifth of a second of delay is worth
#: only six degrees of that period, so *that* edge belongs to the aircraft and
#: the outer loops rather than to the pilot's own loop.
#:
#: The pilot's own loop is what closes at x2.00: 1.28 s a cycle, growing by 37
#: per cent of itself a cycle, with the delay worth 56 degrees of the period.
#: That is what a pilot induced oscillation *is* - a loop closed by the pilot's
#: own lag, at a period the lag can see - and it is why the harder leg climbs
#: past the band's edge instead of stopping at it: the gain margin and the PIO
#: are two different numbers and the phase was asked for both.
NOMINAL_MARGIN_SCALE = 1.0
MARGIN_FAILS_AT = 1.5
PIO_AT_SCALE = 2.0
PIO_MEASURED_PERIOD_S = 1.28
PIO_MEASURED_GROWTH = 1.37
NOMINAL_FLOOR_SCALE = 0.75
FLOOR_FAILS_AT = 0.5


def trace_at(rows, scale):
    """The trace of one scale of a sweep leg: ``sweep_leg``'s own rows."""
    return [trace for one, _, trace in rows if one == scale][0]


def main():
    """The instrument, the nominal case, the sweep, the PIO and the verdict."""
    transcript = Transcript(LOG_FILE)
    sys.stdout = transcript
    try:
        print("the pilot: %0.0f decisions a second on the flight's own clock,"
              " %.2f s of transport delay (%d frames), %0.3f in of stick per deg"
              " of attitude error times scale, at a %.2f s lead and a %.2f s lag"
              % (PILOT_HZ, DELAY_S, DELAY_FRAMES, K_THETA_IN, T_LEAD_S, T_LAG_S))
        print("  the model's own hover pilot is this one at a scale of %.2f: its"
              " attitude gain is %.3f in/deg and its rate gain is %.3f in per"
              " deg/s, which is the lead above, so the nominal case *is* that"
              " pilot" % (SCALE_NOMINAL, K_HOVER_PITCH_IN, K_HOVER_RATE_IN))
        instrument()

        nominal = {}
        pio = {}
        for axis in ("pitch", "roll"):
            nominal[axis] = case(SCALE_NOMINAL, axis,
                                 label="the model's own hover pilot")
            pio[axis] = pio_report(SCALE_NOMINAL, nominal[axis])

        print("")
        print("=== the hover with nobody flying it: the model's own instability ===")
        print("--- %+0.2f deg of pitch, the sticks where the trim left them"
              % PERTURB_DEG)
        nobody_sim, nobody_hands, frozen = frozen_case()
        print("  %s" % (frozen,))
        print("  the pilot's own case beside it: %.1f m of pad and the height to a"
              " centimetre; this one: %.1f m of pad at %s"
              % (nominal["pitch"].drift, frozen.drift,
                 "%.1f s, still in the sky" % nobody_sim.sim_time
                 if not nobody_sim.crashed else
                 "a crash at %.1f s" % nobody_sim.sim_time))

        margin, pio_at, floor, harder, softer = sweep()

        print("")
        print("=== is any of it a pilot induced oscillation? ===")
        for axis in ("pitch", "roll"):
            pio_report(SCALE_NOMINAL, nominal[axis])
        if pio_at is not None:
            pio_report(pio_at, [row[2] for row in harder if row[0] == pio_at][0])

        print("")
        print("=== the verdict ===")
        up_held = [scale for scale, held, _ in harder if held]
        down_held = [scale for scale, held, _ in softer if held]
        held_up = max(up_held) if margin is not None else SCALES_HARDER[-1]
        held_down = min(down_held) if floor is not None else SCALES_SOFTER[-1]
        print("  a pilot at the model's own hover gains - twenty decisions a second,"
              " a fifth of a second late, a %.2f s lead and a %.2f s lag - holds"
              " this hover: %+0.1f deg of pitch back inside %0.2f deg in %.2f s,"
              " %+0.1f of roll in %.2f s, %.1f m of pad and the height to a"
              " centimetre, and no oscillation in either."
              % (T_LEAD_S, T_LAG_S, PERTURB_DEG, SETTLE_DEG,
                 nominal["pitch"].settled(), PERTURB_DEG,
                 nominal["roll"].settled(),
                 max(nominal["pitch"].drift, nominal["roll"].drift)))
        print("  the band is narrow, and the model's own pilot is at the top of it:"
              " x%.2f to x%.2f of the model's own attitude gains, %.1f dB to"
              " %+.1f dB.  A quarter softer than the model's own still holds; the"
              " model's own is the hardest grip that does, so this hover has no"
              " room above the gains it is flown with and little below them."
              % (held_down, held_up, 20.0 * math.log10(held_down),
                 20.0 * math.log10(held_up)))
        if floor is not None:
            print("  below the band the *hover* is what fails: x%.2f (%.1f dB) never"
                  " comes back inside %.2f deg at all - the attitude wanders about"
                  " a degree at a %.1f s period that no longer decays, and the"
                  " machine leaves the pad underneath it."
                  % (floor, 20.0 * math.log10(floor), SETTLE_DEG,
                     softer[-1][2].period() or 0.0))
        if margin is not None:
            edge = trace_at(harder, margin)
            edge_settled = edge.settled()
            print("  above it the failure is not the pilot's yet: x%.2f (%+.1f dB)"
                  " comes back inside the band%s and then grows back out of it"
                  " again, at a period of %.1f s that the pilot's %.2f s of delay"
                  " is worth only %.0f deg of - so that edge is the model's and the"
                  " outer loops', not the pilot's loop."
                  % (margin, 20.0 * math.log10(margin),
                     " at %.2f s" % edge_settled if edge_settled is not None else "",
                     edge.period() or 0.0, DELAY_S,
                     Trace.phase_deg(edge.period()) or 0.0))
        if pio_at is not None:
            _, period, growth = trace_at(harder, pio_at).pio()
            print("  and a pilot's *own* loop is what closes at x%.2f (%+.1f dB):"
                  " %.2f s a cycle, growing %.2f of itself a cycle, with the"
                  " pilot's %.2f s of delay worth %.0f deg of that period.  That"
                  " is the pilot induced oscillation this phase was asked about,"
                  " and it is a grip and a half further up than the hover's own"
                  " edge."
                  % (pio_at, 20.0 * math.log10(pio_at), period, growth, DELAY_S,
                     Trace.phase_deg(period)))

        # -- and what all of it is held to ------------------------------------
        # The pilot's own clock and delay were measured in flight by
        # :func:`instrument`, which raises rather than printing: nothing below is
        # worth reading if that did not pass.  Everything after it is the phase's
        # own claim, and a claim whose numbers drift has to be made again.
        for axis in ("pitch", "roll"):
            assert nominal[axis].settled() is not None, \
                "the model's own hover pilot cannot recover %+0.1f deg of %s" \
                % (PERTURB_DEG, axis)
            assert not pio[axis], (axis, pio[axis])
            assert nominal[axis].altitude_error() <= NOMINAL_HEIGHT_M, \
                (axis, nominal[axis].altitude_error())
            assert nominal[axis].drift <= NOMINAL_DRIFT_M, \
                (axis, nominal[axis].drift)
        assert abs(nominal["pitch"].settled() - NOMINAL_PITCH_SETTLED_S) \
            <= SETTLE_TOLERANCE_S, nominal["pitch"].settled()
        assert abs(nominal["roll"].settled() - NOMINAL_ROLL_SETTLED_S) \
            <= SETTLE_TOLERANCE_S, nominal["roll"].settled()
        # The two edges: the nominal held, and the last scale of each leg held
        # while the one past it did not.  A leg that failed at its first scale -
        # which is what a pilot with no room at all would do - is not this model.
        assert harder[0][1] is True and harder[-1][1] is False, \
            [(scale, held) for scale, held, _ in harder]
        assert softer[0][1] is True and softer[-1][1] is False, \
            [(scale, held) for scale, held, _ in softer]
        assert max(scale for scale, held, _ in harder if held) \
            == NOMINAL_MARGIN_SCALE, harder
        assert margin == MARGIN_FAILS_AT, margin
        # The soft leg runs *down* from the nominal, so the softest grip that
        # held is the smallest of the ones that did and not the largest.
        assert min(scale for scale, held, _ in softer if held) \
            == NOMINAL_FLOOR_SCALE, softer
        assert floor == FLOOR_FAILS_AT, floor
        # And that the two failures are *not* the pilot induced one: the band's
        # edge fails on the aircraft's own slow mode, at eleven and a half
        # seconds, where the pilot's fifth of a second of delay is worth six
        # degrees of phase and cannot be what closed the loop.  What the delay
        # closes is the 1.28 s oscillation a grip and a half further up, where it
        # is worth 56 degrees - which is why both numbers are pinned and not just
        # the margin.
        slow = trace_at(harder, MARGIN_FAILS_AT).period()
        assert slow is not None and slow > PIO_PERIOD_S[1], slow
        assert Trace.phase_deg(slow) < 20.0, Trace.phase_deg(slow)
        assert pio_at == PIO_AT_SCALE, pio_at
        is_pio, period, growth = trace_at(harder, PIO_AT_SCALE).pio()
        assert is_pio, (is_pio, period, growth)
        assert abs(period - PIO_MEASURED_PERIOD_S) <= 0.05, period
        assert abs(growth - PIO_MEASURED_GROWTH) <= 0.05, growth
        assert Trace.phase_deg(period) >= 45.0, Trace.phase_deg(period)
        # The soft edge fails the other way round again: not a fast oscillation,
        # and not one that is even growing - it simply stops decaying, which is
        # what an under-gripped hover does.
        soft = trace_at(softer, FLOOR_FAILS_AT).period()
        assert soft is not None and soft > PIO_PERIOD_S[1], soft
        assert trace_at(softer, FLOOR_FAILS_AT).growth() < PIO_GROWTH, \
            trace_at(softer, FLOOR_FAILS_AT).growth()
        # The reference all of it is read against: with nobody flying, the hover
        # is not a place the aircraft stays - it leaves the ten metre box the
        # short mission allows itself and doubles the error it was given.
        assert frozen.drift > DRIFT_LIMIT_M, frozen.drift
        assert abs(frozen.overshoot()[0]) >= PERTURB_DEG, frozen.overshoot()
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
