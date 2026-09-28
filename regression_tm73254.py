"""The report's own step responses - its figures 2 to 9 - reflown by this model.

TM-73254 ends with a validation chapter.  A Bell UH-1H, serial AAB-11519, was
instrumented and flown at Crows Landing on 12 November 1974 at 29,371 N
(6158 lb), which is the flight-test configuration :func:`airframe_preset` calls
``"flight test"``; eight step control inputs were made, four from a trimmed
60 kt (figures 2 to 5) and four from a trimmed hover (figures 6 to 9); and the
aircraft's body rates, Euler attitudes and normal acceleration were drawn
against the report's own model's responses to the same inputs, in two pages of
twelve seconds each.

**What this module is.**  That chapter, as a runnable regression: it trims the
report's flight test aircraft at 60 kt (:meth:`Simulation.trim_level_flight`,
the level trim that exists for this) and in a hover, flies the same eight steps
through the frame loop, and checks the report's own account of what the
responses are.  No physics is re-derived here; what is added is the experiment.

**What the report did.**  Its own words: "Step control inputs of from +-1.25 cm
to +-2.5 cm (+-1/2 in. to +-1 in.) were made in collective, pitch, roll, and yaw
controls.  The pilot was instructed to establish a steady flight condition (zero
rate of climb, pitch, roll, and yaw) and then to input and hold the appropriate
control for as long as possible before initiating a recovery.  These inputs are
simulated as true steps in the simulator time history comparisons."  The control
channel of each figure is drawn on a full travel scale - +-6.33 in. of
longitudinal stick, +-6.25 in. of lateral stick, +-3.25 in. of pedal, 0 to
10.5 in. of collective - and the traces on them read about an inch of step,
which is the top of the report's band: :data:`UH1_REPORT_CYCLIC_STEP_IN` and
:data:`UH1_REPORT_PEDAL_STEP_IN` are that inch, and the collective step
:data:`UH1_REPORT_COLLECTIVE_STEP_IN` is half of it, the smaller end of the
band, which is what the two collective channels look like on the page.

**How a figure is read.**  The aircraft traces in figures 2 to 9 are the
*flight*, and they carry the pilot's own recovery input at the end: figure 8a's
pedal comes back at 8.5 s and the yaw rate it had been holding comes back with
it.  A flight of this model with the stick *held* is therefore comparable to a
figure only up to that recovery - and the report's own conclusions are about the
beginning anyway: "The primary responses are sufficiently well modeled so that
the pilot was given proper cues in the first one to two seconds following a
servo failure."  So the checks are on the first two seconds of the response
(:data:`FIGURE_WINDOW_S`), and the rest of the twelve seconds is printed for a
reader rather than asserted.

**What it reproduces.**  The readings in :func:`step_cases` were hand-read off
the scanned pages - ``reference`` holds scans, the traces are pen width apart
and every reading is good to a grid square or two, which is why
:data:`FIGURE_RATE_TOLERANCE` is as loose as it is.  At that two second mark,
with the primary rate being the control's own:

* figure 2, forward longitudinal cyclic from 60 kt: the pitch rate goes nose
  down, and where the flight reached 7 deg/s the model reaches 4;
* figures 3 and 4, lateral cyclic and right pedal from 60 kt: the roll and yaw
  rates lead, at 9.4 against the flight's 8.5 and 10.2 against its 9, and each is
  the largest of the three rates, which is the report's "the primary responses
  to the control inputs are reasonably well modeled";
* figures 6, 7 and 8, the same three steps from a hover: the same responses -
  figure 7's roll rate, 11.8 deg/s against the flight's 7, is the loosest of the
  eight - and figure 8's yaw rate, 22 deg/s, is the report's "the primary yaw
  rate response in hover is good";
* figures 5 and 9, up collective in both conditions: the heave, which the report
  says is the first order response ``h = K_h 6c (1 - e^(-t/T))`` - a climb that
  rises monotonically to a steady value - and the yaw rate the step couples
  into, nose right in both conditions, which is its "the magnitudes and signs of
  the coupled responses appear to be properly represented";
* and one discrepancy the report reports itself, asserted here in the same
  direction: "strong pitch coupling is evident in the flight records for both
  the lateral step input and the pedal step input, a phenomenon not present in
  the model".  Figure 8a's flight pitches 15 deg/s to a pedal step; this model's
  pitch rate to the same step stays inside a degree.

**What it does not reproduce.**  Printed with numbers by :func:`print_findings`
rather than asserted, because they are findings and not regressions:

* the coupled responses at 60 kt.  The report's "the absolute magnitudes of the
  responses are small" holds for the flight, where the roll and yaw rates are a
  fifth of the 7 deg/s pitch rate in figure 2a, and does not hold here, where
  the roll rate is 1.3 times the pitch rate over the same two seconds and the
  aircraft is 9 deg into a roll four seconds in.  The mechanism is in the
  report's own equations, and :class:`airframe.BodyForces` keeps the pieces: the
  pedal-held tail rotor keeps its +2.0 kN m of rolling moment while the rotor's
  counter-roll falls from 2.0 to 1.6 kN m as the aircraft pitches, and the
  difference rolls it;
* the long term, for every held step: the coupled responses grow over the twelve
  seconds instead of settling.  The report's own model shares the tendency as
  far as a scan can be read - figure 4a's model column has its yaw rate still
  separating from the trim at the end of the page - which is of a piece with its
  own remark that the long term responses were the weak part of the validation;
* the 60 kt trim's pitch attitude: this model trims 3.4 deg nose up there, where
  figure 2b's aircraft trace starts level.  It is the one number here a figure
  can be read against directly, and it is the answer to the question item 3 of
  this project's own list was going to ask of it;
* two configuration notes: this project's flight test preset leaves the Bell
  stabilizer bar switched off (:class:`rotor_control.ControlLags`, whose
  docstring says why) and runs the control axis lag at the physical 0.072 s,
  where the report's simulation used table 3's R6 of 0.144 s in the force model
  and in the control path both.  Flying these figures with the bar on and the
  lag matched moves the numbers by a few per cent; what it does not do is remove
  the coupling above.

Run the module and it prints the eight traces - the rates, attitudes and normal
acceleration at the figures' own time base of twelve seconds, the input stepping
in at 1.5 s where the report's own traces start - and then the findings above.
Standard library only, like the modules below it.
"""

import math
from dataclasses import dataclass, replace
from typing import List, Optional

from aerodynamics import GRAVITY
from airframe import KNOT, POUND, FlightState
from rotor_control import ControlLags, PilotControls
from simulation import SIM_TIME_STEP_S, Simulation, airframe_preset

# ---------------------------------------------------------------------------
# The experiment: the report's own two conditions and the steps it made in them.
# ---------------------------------------------------------------------------

#: The report's level flight condition, m/s: "60 knots" on every one of the four
#: 60 kt pages, and the case :meth:`Airframe.trim_level_flight` exists for.
REPORT_LEVEL_SPEED = 60.0 * KNOT

#: Step amplitudes, in inches at the pilot's hand or foot.  The report's band is
#: +-1/2 in. to +-1 in. for all four controls; the cyclic and pedal steps are
#: that inch and the collective one is half of it.  The figures' control
#: channels, which are drawn on the full travel scales of 6.33 in. of
#: longitudinal stick, 6.25 in. of lateral stick, 3.25 in. of pedal and 10.5 in.
#: of collective, are what the two readings come from, and they are the input
#: amplitudes of a 1974 flight test rather than constants of the model.
UH1_REPORT_CYCLIC_STEP_IN = 1.0
UH1_REPORT_PEDAL_STEP_IN = 1.0
UH1_REPORT_COLLECTIVE_STEP_IN = 0.5

#: The traces are twelve seconds wide and the input steps in about a second and
#: a half into them, after the aircraft has been seen sitting on the trim.
TRACE_PRE_STEP_S = 1.5
TRACE_SECONDS = 12.0

#: The time base: the report's own 60 Hz, which is :data:`SIM_TIME_STEP_S` and
#: the rate everything in :mod:`airframe` is checked at.
TRACE_STEP_S = SIM_TIME_STEP_S

#: The chosen preset of the report's two: the 6158 lb instrumented flight test
#: aircraft its figures 2 to 9 were flown in.
FLIGHT_TEST_AIRCRAFT = "flight test"

#: The window a figure is compared over, s after the step.  Two seconds, which
#: is the report's own "the pilot was given proper cues in the first one to two
#: seconds following a servo failure" - and by the time its traces have run
#: longer than that the pilot in them has started recovering, which is an input
#: this module deliberately does not fly.
FIGURE_WINDOW_S = 2.0

#: How far a primary rate may be from the figure's reading of it, as a fraction
#: of the reading: a factor of two either way.  The readings are hand-read off
#: scanned pages whose traces are pen width apart, this model's long term
#: damping is not the flight's (see the module docstring), and anything tighter
#: would be asserting the pen rather than the model.
FIGURE_RATE_TOLERANCE = 1.0

#: The smallest yaw rate that counts as a response at all, deg/s, so that a
#: coupled-response check cannot be passed by nothing happening.
RATE_FLOOR_DEG = 0.5

#: The pitch rate figure 8a's *flight* reaches to a hover pedal step, deg/s.
#: The report calls it strong pitch coupling "not present in the model", so the
#: module asserts the model is well under it.
FIGURE_HOVER_PEDAL_PITCH_DEG = 15.0


@dataclass(frozen=True)
class StepCase:
    """One of the report's eight figures: its step, and what it should do.

    ``primary`` is the body rate the control is the primary response of and
    ``primary_deg`` is the figure's own reading of that response, the first
    excursion within :data:`FIGURE_WINDOW_S` of the input.  Both are ``None``
    for the two collective steps, whose response is the heave.

    ``dominant`` says whether the module asserts the control's own rate is the
    largest of the three in that window.  It is everything except the forward
    longitudinal step at 60 kt, where this model's coupled roll rate is larger
    than its pitch rate - a finding, and one of the things
    :func:`print_findings` prints.  ``yaw_coupled`` asserts the coupled yaw rate
    of a collective step, which the report says is properly represented.
    """

    name: str
    figure: str
    condition: str
    axis: str
    step_in: float
    primary: Optional[str]
    primary_deg: float
    dominant: bool
    yaw_coupled: bool = False


def step_cases():
    """The report's eight steps, in figure order, with its own readings.

    The readings are of the figures' *aircraft* traces, hand-read: the peak of
    the control's own rate within two seconds of the input, in deg/s.  They are
    ``(figure, name, condition, axis, step, primary rate, reading, dominant)``.
    """
    return [
        # Figure 2: nose down, 7 deg/s at about a second and a half in.  The
        # flight's roll and yaw rates are under 2 deg/s; this model's roll rate
        # is not, which is why the dominance of its pitch rate is not asserted.
        StepCase("forward longitudinal cyclic", "figure 2", "level",
                 "long_stick", UH1_REPORT_CYCLIC_STEP_IN, "q", -7.0, False),
        # Figure 3: roll right, 8.5 deg/s, with the yaw rate trailing it.
        StepCase("lateral cyclic", "figure 3", "level",
                 "lat_stick", UH1_REPORT_CYCLIC_STEP_IN, "p", 8.5, True),
        # Figure 4: yaw right, 9 deg/s, with the roll rate oscillating behind it.
        StepCase("right pedal", "figure 4", "level",
                 "pedal", UH1_REPORT_PEDAL_STEP_IN, "r", 9.0, True),
        # Figure 5: the heave, with the yaw rate coupling in nose right.
        StepCase("up collective", "figure 5", "level", "collective",
                 UH1_REPORT_COLLECTIVE_STEP_IN, None, 0.0, False, True),
        # Figure 6: the same forward step from a hover, and a bigger one - 8
        # deg/s - with the long term pitch rate the report says its own model
        # exaggerates.
        StepCase("forward longitudinal cyclic", "figure 6", "hover",
                 "long_stick", UH1_REPORT_CYCLIC_STEP_IN, "q", -8.0, True),
        # Figure 7: roll right, 7 deg/s.  The flight's coupled pitch rate runs
        # to 10 deg/s and its yaw rate to the stop, which is the report's
        # "strong pitch coupling ... not present in the model"; this model's own
        # roll rate is still the largest of the three.
        StepCase("lateral cyclic", "figure 7", "hover",
                 "lat_stick", UH1_REPORT_CYCLIC_STEP_IN, "p", 7.0, True),
        # Figure 8: yaw right, past 20 deg/s and held against its stop for
        # three seconds - "the primary yaw rate response in hover is good".
        StepCase("right pedal", "figure 8", "hover",
                 "pedal", UH1_REPORT_PEDAL_STEP_IN, "r", 20.0, True),
        # Figure 9: the heave again, and 7 deg/s of coupled yaw, nose right.
        StepCase("up collective", "figure 9", "hover", "collective",
                 UH1_REPORT_COLLECTIVE_STEP_IN, None, 0.0, False, True),
    ]


def _step(controls, axis, inches):
    """The trimmed controls with the step added to one of the axes."""
    return replace(controls, **{axis: getattr(controls, axis) + inches})


# ---------------------------------------------------------------------------
# Flying one of them: trim, hold, step, and watch, at the figures' time base.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Sample:
    """One instant of a trace, in the units the figures are drawn in."""

    time: float          # s, from the start of the trace
    p: float             # deg/s, roll rate, right positive
    q: float             # deg/s, pitch rate, up positive
    r: float             # deg/s, yaw rate, right positive
    roll: float          # deg
    pitch: float         # deg
    yaw: float           # deg
    az: float            # g, the normal acceleration: lift over weight
    height_rate: float   # m/s, + climbing
    airspeed_kt: float
    stick_in: float      # in, the axis the step went into
    input_on: bool


@dataclass
class StepResponse:
    """A flown step: the trace, the trim it came from and the stick it held."""

    case: StepCase
    samples: List[Sample]
    trim_controls: PilotControls
    trim_state: FlightState
    mass: float

    def held(self):
        """The samples from the step onwards, which is what a figure shows."""
        return [sample for sample in self.samples if sample.input_on]


def fly_step(case, airframe=None, sim=None):
    """Fly one of the report's steps and hand back its trace.

    The aircraft is trimmed (60 kt for the figures 2 to 5 cases, a hover for 6
    to 9), watched for :data:`TRACE_PRE_STEP_S` seconds on the trim - the part of
    a figure before the input starts - and then flown for
    :data:`TRACE_SECONDS` more with the step held on the one axis, sampled every
    :data:`TRACE_STEP_S`.  The step is a true step: :class:`PilotControls` is
    rebuilt once and held, so the trace after it is the model's response and
    nothing else.
    """
    if sim is None:
        sim = Simulation(airframe=airframe or airframe_preset(FLIGHT_TEST_AIRCRAFT),
                         trim_at_start=False)
    if case.condition == "level":
        controls, state = sim.trim_level_flight(REPORT_LEVEL_SPEED)
    else:
        controls, state = sim.trim()
    held = _step(controls, case.axis, case.step_in)
    frames = int(round((TRACE_PRE_STEP_S + TRACE_SECONDS) / TRACE_STEP_S))
    started = int(round(TRACE_PRE_STEP_S / TRACE_STEP_S))
    samples = []
    for frame in range(frames + 1):
        if frame:
            sim.step(TRACE_STEP_S, held if frame >= started else controls)
        telemetry = sim.telemetry()
        samples.append(Sample(
            time=sim.sim_time, p=telemetry.roll_rate_deg,
            q=telemetry.pitch_rate_deg, r=telemetry.yaw_rate_deg,
            roll=telemetry.roll_deg, pitch=telemetry.pitch_deg,
            yaw=telemetry.yaw_deg,
            az=telemetry.lift / (sim.airframe.mass * GRAVITY),
            height_rate=telemetry.height_rate,
            airspeed_kt=telemetry.airspeed_kt,
            stick_in=getattr(sim.controls, case.axis),
            input_on=frame >= started))
    return StepResponse(case, samples, controls, state, sim.airframe.mass)


# ---------------------------------------------------------------------------
# Laying a trace next to a figure: the report's own time base and units.
# ---------------------------------------------------------------------------

#: Every half second, which is a grid line on the figures' own time scale.
PRINT_INTERVAL_S = 0.5


def _heading(response):
    """A figure's own data box, plus this model's trim.

    The pitch attitude is printed as the report's pens leave it - absolute - so
    the trim's own attitude belongs here rather than subtracted from every row
    of the trace: a figure's aircraft trace starts on it.
    """
    case = response.case
    where = ("a trimmed 60 kt" if case.condition == "level"
             else "a trimmed hover")
    attitude = response.trim_state.attitude_deg
    return ("%s - %s, %+.2f in of %s from %s, at %.0f lb"
            % (case.figure, case.name, case.step_in,
               case.axis.replace("_", " "), where, response.mass / POUND)
            + " (trim: roll %+.2f, pitch %+.2f deg)"
            % (attitude.x, attitude.y))


def print_trace(response, heading=True):
    """Print one response at the figures' own time base and units.

    Eleven channels short of a page, but the six the figures draw: the three
    body rates, the three Euler angles, the normal acceleration and the stick.
    """
    if heading:
        print(_heading(response))
    print("      time      p      q      r   |  roll  pitch    yaw  |"
          "    Az  |   in")
    step = int(round(PRINT_INTERVAL_S / TRACE_STEP_S))
    for index in range(0, len(response.samples), step):
        sample = response.samples[index]
        marker = ("  <- the step" if sample.input_on
                  and sample.time - TRACE_PRE_STEP_S < 0.5 * TRACE_STEP_S
                  else "")
        print("  %5.2f s %+6.1f %+6.1f %+6.1f   | %+5.1f %+5.1f %+6.1f  |"
              " %5.3f  | %5.2f%s"
              % (sample.time, sample.p, sample.q, sample.r, sample.roll,
                 sample.pitch, sample.yaw, sample.az, sample.stick_in,
                 marker))


# ---------------------------------------------------------------------------
# The self test and the demo, in the style of the modules below.
# ---------------------------------------------------------------------------


def window(response, within=FIGURE_WINDOW_S):
    """The samples from the step to *within* seconds after it."""
    return [sample for sample in response.held()
            if sample.time <= TRACE_PRE_STEP_S + within]


def peaks(response, within=FIGURE_WINDOW_S):
    """The three rates' signed extremes in a window, as ``rate: (value, time)``."""
    samples = window(response, within)
    out = {}
    for rate in ("p", "q", "r"):
        sample = max(samples, key=lambda s: abs(getattr(s, rate)))
        out[rate] = (getattr(sample, rate), sample.time)
    return out


def summary_lines(response):
    """One line per response: this model's numbers beside the figure's reading."""
    case = response.case
    extremes = peaks(response)
    text = "  %-9s %-31s %-6s" % (case.figure, case.name, case.condition)
    if case.primary is not None:
        value, when = extremes[case.primary]
        text += "%s %+5.1f deg/s at %4.1f s (figure %+5.1f)" % (
            case.primary, value, when, case.primary_deg)
    else:
        text += "heave: climb %+6.2f m/s at 2 s, yaw %+5.1f deg/s" % (
            window(response)[-1].height_rate, extremes["r"][0])
    last = response.samples[-1]
    return text + ("  | p %+5.1f q %+5.1f r %+5.1f  | 12 s: roll %+6.1f"
                   " pitch %+6.1f" % (extremes["p"][0], extremes["q"][0],
                                      extremes["r"][0], last.roll, last.pitch))


def coupling_line(response):
    """A control's own rate against the largest of the two it couples into.

    The ratio of the two is what the report's "the absolute magnitudes of the
    responses are small" is about, and it is the part of figure 2 that this
    model does not reproduce: the flight's coupled rates are a fifth of its
    pitch rate, and this model's are larger than it.
    """
    case = response.case
    extremes = peaks(response)
    primary = case.primary or max(("p", "q", "r"),
                                 key=lambda rate: abs(extremes[rate][0]))
    others = [rate for rate in ("p", "q", "r") if rate != primary]
    coupled = max(others, key=lambda rate: abs(extremes[rate][0]))
    ratio = abs(extremes[coupled][0] / extremes[primary][0])
    own = ("%s %+5.1f deg/s" % (primary, extremes[primary][0])
           if case.primary is not None else
           "largest rate %s %+5.1f deg/s" % (primary, extremes[primary][0]))
    return ("  %-9s %-31s %-6s %-26s coupled %+5.1f = %4.2f of it"
            % (case.figure, case.name, case.condition, own,
               extremes[coupled][0], ratio))


def drift_line(response):
    """Where a held step has got to by the end of the figures' twelve seconds."""
    case = response.case
    last = response.samples[-1]
    return ("  %-9s %-31s %-6s roll %+6.1f pitch %+6.1f yaw %+7.1f deg,"
            " climb %+6.2f m/s, %+.1f kt"
            % (case.figure, case.name, case.condition, last.roll, last.pitch,
               last.yaw, last.height_rate, last.airspeed_kt))


def _trim_flight(condition):
    """A flight test aircraft trimmed for one of the report's two conditions."""
    sim = Simulation(airframe=airframe_preset(FLIGHT_TEST_AIRCRAFT),
                     trim_at_start=False)
    if condition == "level":
        controls, state = sim.trim_level_flight(REPORT_LEVEL_SPEED)
    else:
        controls, state = sim.trim()
    return sim, controls, state


def _self_test():
    """Checks on the two trims and the report's own conclusions about the steps.

    Raises AssertionError on failure.  The physics is :mod:`airframe`'s to vouch
    for and its own self test does it; what is checked here is what the report's
    figures say - that the two conditions its figures are flown from are trims,
    and that the eight steps answer the way its text says they do, to within
    :data:`FIGURE_RATE_TOLERANCE` of its own traces.  What the figures show that
    this model does *not* do is printed by :func:`print_findings` instead.
    """
    # The two trims.  A trim is checked and not trusted: equilibrium_residual is
    # the force and moment the model still has to answer there, and then twenty
    # seconds of flying the trim with nothing touched has to leave it where it
    # was.
    for condition in ("level", "hover"):
        sim, controls, state = _trim_flight(condition)
        force, moment = sim.airframe.equilibrium_residual(controls, state)
        assert force.length() < 1.0, force.length()
        assert moment.length() < 1.0, moment.length()
        for _ in range(int(20.0 / TRACE_STEP_S)):
            sim.step(TRACE_STEP_S, controls)
        after = sim.airframe.state
        assert abs(after.speed - state.speed) < 0.01, after.speed
        assert abs(after.altitude - state.altitude) < 0.01, after.altitude
        assert abs(math.degrees(after.attitude.y - state.attitude.y)) < 0.01
        assert abs(math.degrees(after.rates.length())) < 0.01

    # The report's own two conditions, as far as a figure can be read against
    # them: 60 kt level, and the 4.4 deg nose up a UH-1H hovers at because its
    # c.g. is aft of the hub.
    _, level_controls, level_state = _trim_flight("level")
    _, hover_controls, hover_state = _trim_flight("hover")
    assert abs(level_state.speed - REPORT_LEVEL_SPEED) < 1e-9
    assert abs(math.degrees(level_state.attitude.x)) < 0.5
    assert 3.5 < math.degrees(hover_state.attitude.y) < 5.5
    assert level_controls.collective < hover_controls.collective

    # The eight figures.  Each one: the control's own rate comes out with the
    # sign the figure has and within a factor of two of its size; where the
    # module asserts it, that rate is also the largest of the three; and for the
    # two collective steps the response is the heave instead - a climb that
    # rises without overshoot, with the yaw rate coupling in nose right.
    responses = {}
    for case in step_cases():
        response = fly_step(case, airframe=airframe_preset(FLIGHT_TEST_AIRCRAFT))
        responses[case.figure] = response
        extremes = peaks(response)
        if case.primary is not None:
            value = extremes[case.primary][0]
            assert value * case.primary_deg > 0.0, (case.figure, value)
            assert (abs(value - case.primary_deg)
                    <= FIGURE_RATE_TOLERANCE * abs(case.primary_deg)), (
                        case.figure, value, case.primary_deg)
            if case.dominant:
                others = [abs(extremes[rate][0]) for rate in ("p", "q", "r")
                          if rate != case.primary]
                assert abs(value) >= max(others), (case.figure, value, others)
        else:
            climbs = [sample.height_rate for sample in window(response)]
            assert climbs[-1] > 0.3, (case.figure, climbs[-1])
            assert all(later > earlier - 0.02
                       for earlier, later in zip(climbs, climbs[1:])), (
                           case.figure, climbs)
            assert case.yaw_coupled and extremes["r"][0] > RATE_FLOOR_DEG, (
                case.figure, extremes["r"])

    # One discrepancy the report reports itself: "strong pitch coupling is
    # evident in the flight records for both the lateral step input and the
    # pedal step input, a phenomenon not present in the model".  Checked in the
    # direction the report describes it, so that anything that *added* that
    # coupling to this model would be caught rather than welcomed.
    model_pitch = abs(peaks(responses["figure 8"])["q"][0])
    assert model_pitch < 0.5 * FIGURE_HOVER_PEDAL_PITCH_DEG, model_pitch

    # Two runs are the same run: the frame loop reads no clock.
    again = fly_step(step_cases()[0],
                     airframe=airframe_preset(FLIGHT_TEST_AIRCRAFT))
    assert [(s.time, s.p, s.q, s.r, s.roll)
            for s in responses["figure 2"].samples] == [
                (s.time, s.p, s.q, s.r, s.roll) for s in again.samples]


def print_findings(responses=None):
    """Print what the figures say that this model does not, with its numbers.

    Not asserted: these are findings rather than regressions, and a change that
    fixes one of them should show up here as a change instead of as a test that
    quietly starts passing.  *responses* is a ``{figure: StepResponse}`` such as
    :func:`_demo` has already flown; without it the eight are flown here.

    Three of the four are the model's - the coupled responses at 60 kt and the
    roll couple that drives them, the long term departure of every held step and
    the pitch attitude of the 60 kt trim - and the last is configuration: the
    stabilizer bar and the control lag the report's simulation flew, which this
    project's preset does not.
    """
    if responses is None:
        responses = {case.figure: fly_step(case) for case in step_cases()}

    print("the four 60 kt figures against the report's \"the absolute magnitudes")
    print("of the responses are small\" - the flight's coupled rates in figure 2a")
    print("are a fifth of its pitch rate, and this model's are as large as the")
    print("rate they couple into:")
    for case in step_cases()[:4]:
        print(coupling_line(responses[case.figure]))

    print()
    print("where the roll comes from, in the report's own equations and in the")
    print("pieces BodyForces keeps.  The pedal is held at its trim, so the tail")
    print("rotor's rolling moment stays where the trim put it while the rotor's")
    print("counter-roll falls away as the aircraft pitches, and the difference")
    print("is a rolling moment nothing in figure 2a was answering:")
    sim, controls, _ = _trim_flight("level")
    held = _step(controls, "long_stick", UH1_REPORT_CYCLIC_STEP_IN)
    for watch in (0.0, 2.0, 4.0, 8.0):
        while sim.sim_time < watch - 1e-9:
            sim.step(TRACE_STEP_S, held if sim.sim_time >= TRACE_PRE_STEP_S
                     else controls)
        forces = sim.forces
        telemetry = sim.telemetry()
        print("  %4.1f s  roll %+6.2f, pitch %+6.2f deg | L %+7.0f N m = rotor"
              " %+7.0f + tail %+7.0f" % (watch, telemetry.roll_deg,
                                         telemetry.pitch_deg, forces.moment.x,
                                         forces.rotor_moment.x,
                                         forces.tail_moment.x))

    print()
    print("the long term of every held step: where it has got to by twelve")
    print("seconds.  The report's own traces have the pilot's recovery in them")
    print("by then - figure 8a's pedal comes back at 8.5 s - so this is this")
    print("model's departure as much as its response:")
    for case in step_cases():
        print(drift_line(responses[case.figure]))

    print()
    print("the 60 kt trim's pitch attitude.  Figure 2b's aircraft trace starts")
    print("level where this model trims %+.2f deg nose up, and the hover the"
          % math.degrees(_trim_flight("level")[2].attitude.y))
    print("report's 4.4 deg is this model's %+.2f deg"
          % math.degrees(_trim_flight("hover")[2].attitude.y))

    print()
    print("and the configuration: the report's simulation against this project's")
    print("flight test preset, on the figure 2 reading")
    preset = airframe_preset(FLIGHT_TEST_AIRCRAFT)
    print("  stabilizer bar %s here, on in the report's model; control axis lag"
          " %.3f s" % ("on" if preset.lags.bar_enabled else "off",
                       preset.lags.cyclic_tau))
    print("  here against table 3's R6 = %.3f s, which the report's model has in"
          % preset.rotor_time_constant)
    print("  its control path as well as in its flapping equations")
    for name, overrides in (("the bar on", {"lags": ControlLags(
            cyclic_tau=preset.rotor_time_constant, bar_enabled=True)}),
                            ("the lag matched", {"lags": ControlLags(
                                cyclic_tau=preset.rotor_time_constant)})):
        response = fly_step(step_cases()[0], airframe=airframe_preset(
            FLIGHT_TEST_AIRCRAFT, **overrides))
        extremes = peaks(response)
        print("  figure 2 with %-18s q %+5.1f, p %+5.1f, r %+5.1f deg/s"
              % (name, extremes["q"][0], extremes["p"][0], extremes["r"][0]))


def _demo():
    """Print the eight traces, then what they say next to the figures.

    The traces are the point of the module: at the report's own time base and in
    its own units they can be laid next to figures 2 to 9 and read off the page,
    which is the comparison this model was built to be judged by.
    """
    print("the report's flight test aircraft, the 6158 lb instrumented UH-1H of")
    print("its figures 2 to 9, trimmed to the two conditions they are flown from")
    for condition in ("level", "hover"):
        _, controls, state = _trim_flight(condition)
        attitude = state.attitude_deg
        print("  %-5s %s -> %6.2f kt, roll %+5.2f, pitch %+5.2f deg"
              % (condition, controls, state.speed / KNOT, attitude.x,
                 attitude.y))

    print()
    print("the eight steps, flown through the frame loop: the rates and angles")
    print("and normal acceleration the figures draw, with the input in at 1.5 s")
    responses = {}
    for case in step_cases():
        response = fly_step(case,
                            airframe=airframe_preset(FLIGHT_TEST_AIRCRAFT))
        responses[case.figure] = response
        print()
        print_trace(response)

    print()
    print("what those traces say against the figures' own readings")
    for case in step_cases():
        print(summary_lines(responses[case.figure]))

    print()
    print("and what this model does not reproduce - the module docstring says why")
    print("these are printed rather than asserted")
    print()
    print_findings(responses)

    print()
    _self_test()
    print("self test passed")


if __name__ == "__main__":
    _demo()
