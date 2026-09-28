"""The frame loop: the clock around :mod:`airframe`, the pilot's hands, and the
ground under the helicopter.

The three modules below this one are pure physics.  ``Airframe.step`` flies
exactly one frame of a given length with the controls held, asks nothing about
where that frame came from, and cannot put the helicopter on the ground because
TM-73254 has no ground contact in it at all.  Everything a program around them
needs is here instead:

* **the clock.**  A window delivers frames whenever it likes, and a frame that
  took 30 ms must still fly 30 ms of aircraft.  :meth:`Simulation.step` adds the
  frame time to an accumulator and runs whole 1/60 s physics steps out of it, so
  the physics is always the same size however ragged the rendering is.  Nothing
  in this module reads a clock - the frame time is an argument - which is what
  makes a scripted run reproducible to the last bit;
* **the pilot's hands.**  :class:`PilotInput` turns held keys into the inches of
  stick travel :class:`rotor_control.PilotControls` wants, at a rate a hand
  could move them: the cyclic and the pedals spring back to centre, the
  collective is a ratchet that stays where it is left, and a full travel takes
  about a second of holding the key.  A joystick, which gives absolute axes
  instead, comes in through :meth:`PilotInput.set_axes`;
* **the ground.**  :meth:`Simulation._ground` is this project's own addition and
  the only place in it that is not TM-73254.  See below;
* **the renderer.**  :meth:`Simulation.render_position`,
  :meth:`Simulation.render_matrix` and :meth:`Simulation.render_basis` hand the
  aircraft to the OpenGL world of main.py, and :class:`ChaseCamera` follows it
  around.

**The frame loop.**  Sixty hertz, the rate the report's own simulation ran at
and the rate :mod:`airframe`'s self test was written against.  A frame worth
more than ``max_steps_per_frame`` steps (five, i.e. 83 ms) keeps that much of
the physics and counts the remainder in ``dropped_steps``: a dragged window or
an alt-tab must not be answered with a quarter of a second of flying in one
jump.  A step whose state comes back out of the envelope - non-finite, or beyond
the modest ceilings of ``UH1_ENVELOPE_CEILING_*`` - is refused outright: the last
good state is kept and the aircraft is called crashed, because at 100 per cent
rotor with no engine and no rotor speed dynamics a violent hands-off case can run
away, and nothing downstream should ever see the result.  Nothing is
interpolated - a frame that lands between two steps
draws the older one - because at 60 Hz that is a fifth of a millimetre of
altitude and the code is simpler without it.

**The configurations.**  :func:`airframe_preset` gives the report's own two: the
8700 lb aircraft its simulation flew, and the 6158 lb instrumented flight test
aircraft of its figures 2 to 9, both at the 0.144 s rotor time constant table 3
lists as flown.  That is deliberately not the 0.072 s of
:data:`aerodynamics.UH1_ROTOR_TIME_CONSTANT_S`: one blade's inertia is the
physical answer and the report's constant is the one whose responses were
measured, and the flown value is the easier of the two to fly, which is what a
person holding a key wants.

**The trims.**  A :class:`Simulation` is not interesting until it is flying
something steady, and the model has two steady conditions worth having: a hover
(:meth:`Simulation.trim`, which is :meth:`airframe.Airframe.trim_hover`) and
level flight at an airspeed (:meth:`Simulation.trim_level_flight`, which is
:meth:`airframe.Airframe.trim_level_flight`, and whose report's-own case is
``60 * KNOT``).  Both solve the model's balance of forces and moments in stick
positions and two attitudes, and both leave the answer as the reference
:meth:`Simulation.in_trim` compares against, so a HUD has a trim light at 60 kt
as well as in the hover and :meth:`Simulation.reset` comes back to whichever
trim was found last.

**The ground.**  The model has ground effect (equation 10a) and no skid contact,
so something has to decide what happens when the c.g. reaches the skid line
1.40 m above the ground (:data:`UH1_CG_HEIGHT_ON_GROUND`, the hub's 136.5 in
waterline less the 6.79 ft of hub above the c.g.).  What this module does is a
*floor*, not a contact model: the c.g. is held on the skid line, the descent is
taken out of the velocity, the skids hold the aircraft where it stands - no
sliding is modelled - taking the yaw rate out as they do, and roll and pitch are
levelled again, while the heading stays free so that pedal can pivot it.  A
descent rate at touchdown above :data:`UH1_HARD_LANDING_MPS` sets ``crashed``,
and the aircraft stays crashed until :meth:`Simulation.reset`; what a program
shows for that is its business, and the model itself does not stop flying.  A
parked helicopter stays parked, and a pickup leaves the ground smoothly from
rest rather than jumping.  The friction, the levelling, the crash threshold and
the skid line are this project's own, are named here, and all live in one
function so that a real spring-damper skid model has one place to replace.

**The renderer's world.**  main.py's world is x east, y up, z south: the OpenGL
camera's own frame, since a level camera there looks along -z with +y up and +x
right.  A NED vector therefore arrives as ``(east, -down, -north)``, which
:func:`to_renderer` does and which is one proper rotation of
:mod:`airframe`'s NED.  What the aircraft needs from it is a rotation to draw
with: :meth:`Simulation.render_matrix` is the body to renderer matrix in the
column-major sixteen numbers ``glMultMatrixf`` takes, translation included, so
drawing the helicopter is one call with no Euler order to get wrong in between.
:meth:`Simulation.render_basis` gives the same rotation as the three unit
vectors a camera or a shadow wants, and the self test checks the signs a
renderer gets wrong: a nose pointed east is renderer +x, a nose-up attitude
points the nose up, and a roll to starboard drops the starboard wing.

Standard library only, so it runs and tests headless.
"""

import math
from dataclasses import dataclass, field
from typing import Optional

from aerodynamics import (GRAVITY, UH1_COLLECTIVE_TRAVEL_IN,
                          UH1_HUB_WATERLINE_M, UH1_LAT_STICK_TRAVEL_IN,
                          UH1_LONG_STICK_TRAVEL_IN, UH1_PEDAL_TRAVEL_IN,
                          UH1_ROTOR_TIME_CONSTANT_SIM_S, Vector3)
from airframe import (FOOT, KNOT, POUND, Airframe, BodyForces, FlightState,
                      UH1_HUB_HEIGHT, UH1_SIM_MASS, UH1_TEST_MASS, apply,
                      direction_cosine_matrix, transpose)
from rotor_control import PilotControls

# ---------------------------------------------------------------------------
# The clock.
# ---------------------------------------------------------------------------

#: Physics time step, s.  Sixty hertz: the rate the report's simulation ran at
#: and the rate every number in :mod:`airframe`'s self test was checked at.
SIM_TIME_STEP_S = 1.0 / 60.0

#: How many physics steps one frame may be worth before the rest is dropped.
#: Five is 83 ms, a frame rate of 12 Hz: below that the simulation falls behind
#: the clock deliberately rather than spiralling.
SIM_MAX_STEPS_PER_FRAME = 5

#: Metres in an inch.  :mod:`airframe` has the foot and the pound; the report's
#: geometry is in inches, and the stations in this file are too.
INCH = FOOT / 12.0

#: Height of the c.g. above the ground with the aircraft on its skids, m.
#: TM-73254's geometry is measured from the ground line: the main rotor hub is
#: at waterline 136.5 in (:data:`aerodynamics.UH1_HUB_WATERLINE_M`, whose name
#: says metres and whose value is the report's inches) and the c.g. sits
#: UH1_HUB_HEIGHT below it, which leaves the 55.0 in an upright UH-1 stands on.
UH1_CG_HEIGHT_ON_GROUND = UH1_HUB_WATERLINE_M * INCH - UH1_HUB_HEIGHT

# ---------------------------------------------------------------------------
# The ground.  Neither constant is in the report; see the module docstring.
# ---------------------------------------------------------------------------

#: Descent rate at touchdown that counts as a crash, m/s (591 ft/min).  A game
#: constant, not a UH-1 limit: the report has no ground contact at all, so there
#: is no number of its own to use, and the skids are not modelled flexing or
#: sliding either.
UH1_HARD_LANDING_MPS = 3.0

#: Rate at which the skids take a yaw rate out, 1/s: pedal pivots the helicopter
#: on the ground and letting go stops it.  Six is about half a second.
GROUND_FRICTION_PER_S = 6.0

#: Rate at which a rolled or pitched aircraft is levelled again on the ground,
#: 1/s.  Level is the only attitude a floor can hold, since the skid line is one
#: height above the ground.
GROUND_LEVELLING_PER_S = 2.0

#: Clearance above the skid line that counts as flying again, m, so that a
#: helicopter sitting in its own ground effect does not chatter on and off the
#: ground every step.
GROUND_CLEARANCE_M = 0.02

# ---------------------------------------------------------------------------
# How far a state may go.  See :meth:`Simulation._advance`.
# ---------------------------------------------------------------------------

#: Altitude, m, speed, m/s and rate, rad/s, beyond which the frame that produced
#: a state is refused rather than passed on.  The model has no engine and no rotor
#: speed dynamics, so a violent hands-off case can grow without bound - a dive
#: with the collective at the down stop ends in 10^11 m of altitude and then in a
#: NaN - and a wrapper that hands a renderer that has failed whatever the
#: arithmetic said.  The three numbers are this project's own and sit far outside
#: any UH-1 flight: 30 km up, 200 m/s, 300 deg/s.
UH1_ENVELOPE_CEILING_M = 30000.0
UH1_ENVELOPE_CEILING_MPS = 200.0
UH1_ENVELOPE_CEILING_RADPS = math.radians(300.0)

# ---------------------------------------------------------------------------
# The configurations of TM-73254.
# ---------------------------------------------------------------------------

#: The two aircraft the report flies, and the rotor time constant it flew both
#: of them at: ``(mass kg, R6 s)``.  See the module docstring.
UH1_AIRFRAME_PRESETS = {
    "simulation": (UH1_SIM_MASS, UH1_ROTOR_TIME_CONSTANT_SIM_S),
    "flight test": (UH1_TEST_MASS, UH1_ROTOR_TIME_CONSTANT_SIM_S),
}


def airframe_preset(name="simulation", **overrides):
    """An :class:`Airframe` on one of the report's own two configurations.

    ``name`` is ``"simulation"`` for the 8700 lb aircraft of TM-73254's
    simulation or ``"flight test"`` for the 6158 lb instrumented aircraft of its
    figures 2 to 9.  Both come with the 0.144 s rotor time constant of table 3,
    which is what its responses were measured with - pass
    ``rotor_time_constant=UH1_ROTOR_TIME_CONSTANT_S`` for the 0.072 s the blade
    inertia gives instead.  Any other :class:`Airframe` keyword goes straight
    through, so ``airframe_preset("flight test", ground_effect=False)`` is the
    flight test aircraft flying out of the cushion.
    """
    if name not in UH1_AIRFRAME_PRESETS:
        raise ValueError("unknown airframe preset %r, expected one of %s"
                         % (name, ", ".join(sorted(UH1_AIRFRAME_PRESETS))))
    mass, rotor_time_constant = UH1_AIRFRAME_PRESETS[name]
    settings = {"mass": mass, "rotor_time_constant": rotor_time_constant}
    settings.update(overrides)
    return Airframe(**settings)


# ---------------------------------------------------------------------------
# The renderer's world: (east, -down, -north).  See the module docstring.
# ---------------------------------------------------------------------------


def to_renderer(vector):
    """A NED vector in the renderer's world: ``(east, -down, -north)``."""
    return Vector3(vector.y, -vector.z, -vector.x)


def from_renderer(vector):
    """The inverse of :func:`to_renderer`: north is -z, east is x, down is -y."""
    return Vector3(-vector.z, vector.x, -vector.y)


# ---------------------------------------------------------------------------
# What a HUD or an instrument panel asks for.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Telemetry:
    """One instant of the aircraft, in the units an instrument panel wants.

    SI through and through, with knots and feet on the four numbers a pilot
    reads in those, because TM-73254's own figures are in them.  ``sim_time``
    belongs to the simulation and not to the aircraft, and is here rather than in
    a class of its own because a HUD shows it next to the rest.

    The control angles are what the *mixing* commands, not what the rotor is
    holding: the lags between the two are :class:`rotor_control.ControlLags`,
    read off :attr:`Airframe.lags` by anything that wants to watch them, and at
    the 0.2 s of the collective and pedals they are not a gauge's business.
    """

    sim_time: float           # s, physics time actually flown
    alt_agl: float            # m, c.g. above the ground
    height_rate: float        # m/s, vertical speed, + climbing
    airspeed: float           # m/s, through the airframe
    airspeed_kt: float        # kt
    ground_speed: float       # m/s, over the ground
    roll_deg: float           # deg, right positive
    pitch_deg: float          # deg, up positive
    yaw_deg: float            # deg, from north towards east
    roll_rate_deg: float      # deg/s, p
    pitch_rate_deg: float     # deg/s, q
    yaw_rate_deg: float       # deg/s, r
    collective_in: float      # in, both sticks and the pedals too
    long_stick_in: float
    lat_stick_in: float
    pedal_in: float
    collective_fraction: float  # 0 at the down stop, 1 at the up stop
    collective_deg: float     # deg, root collective the mixing commands
    tail_collective_deg: float
    control_tilt_deg: float   # deg, commanded tip path plane tilt
    thrust: float             # N, main rotor T
    torque: float             # N m, main rotor shaft torque Q
    h_force: float            # N, main rotor H
    y_force: float            # N, main rotor Y
    tail_thrust: float        # N, starboard positive
    lift: float               # N, -Z_B, the model's upward force
    ground_effect: float      # K_G, 1 in free air
    inflow_ratio: float       # lambda, the report's
    advance_ratio: float      # mu
    on_ground: bool
    crashed: bool
    in_trim: bool

    @property
    def height_rate_fpm(self):
        """Vertical speed in feet per minute, the unit a vertical speed needle
        is marked in and the one the report's climb figures are drawn in."""
        return self.height_rate / FOOT * 60.0

    @property
    def alt_ft(self):
        """Altitude above the ground in feet."""
        return self.alt_agl / FOOT

    def __str__(self):
        return ("t %6.2f s | alt %6.1f m %+7.0f fpm | %5.1f kt | roll %+6.1f"
                " pitch %+6.1f yaw %+6.1f deg | coll %5.2f in (%4.1f %%), long"
                " %+5.2f lat %+5.2f pedal %+5.2f in | T %6.0f N, Q %+7.0f N m,"
                " tail %+6.0f N |%s%s%s"
                % (self.sim_time, self.alt_agl, self.height_rate_fpm,
                   self.airspeed_kt, self.roll_deg, self.pitch_deg,
                   self.yaw_deg, self.collective_in,
                   100.0 * self.collective_fraction, self.long_stick_in,
                   self.lat_stick_in, self.pedal_in, self.thrust, self.torque,
                   self.tail_thrust,
                   " trim" if self.in_trim else "",
                   " on the ground" if self.on_ground else "",
                   " CRASHED" if self.crashed else ""))


# ---------------------------------------------------------------------------
# The pilot's hands.
# ---------------------------------------------------------------------------


@dataclass
class PilotInput:
    """A hand on the controls: where the four axes are, and how fast they move.

    The axes are the normalised ones a keyboard or a joystick gives: -1 to +1 on
    the cyclic and the pedals, 0 to 1 on the collective.  :meth:`controls` is the
    only way out, and it hands them to
    :meth:`rotor_control.PilotControls.from_axes`, so the travels of TM-73254
    table 2 are applied by the model rather than guessed at here.

    Two ways in, because there are two kinds of device:

    * :meth:`step` is a keyboard.  It is told which keys are *held* this frame -
      0 for none, -1 or +1 for a direction - and moves the axes the way a hand
      would: the cyclic and the pedals at ``cyclic_rate`` and ``pedal_rate``, and
      back to centre at ``centring_rate`` when released, while the collective is
      a friction control that stays where it is put and creeps up or down at
      ``collective_rate`` only while its key is held;
    * :meth:`set_axes` is a joystick or a script: absolute positions, slewed at
      the same rates when a frame time is given and taken as they come when it
      is not.

    The rates are full travels per second and they are feel, not physics: a
    cyclic that reaches its stop in 0.6 s, a collective that takes 1.8 s over its
    11 in of travel because the real one is heavy to move.  Nothing in the
    report says how fast a hand works.
    """

    collective_axis: float = 0.0     # 0 at the down stop, 1 at the up stop
    long_stick_axis: float = 0.0     # + forward
    lat_stick_axis: float = 0.0      # + right
    pedal_axis: float = 0.0          # + right
    collective_rate: float = 0.55    # full travel per second
    cyclic_rate: float = 1.6
    pedal_rate: float = 2.0
    centring_rate: float = 2.4       # how fast a released stick centres

    def axes(self):
        """The four normalised axes, ``(collective, long, lat, pedal)``."""
        return (self.collective_axis, self.long_stick_axis,
                self.lat_stick_axis, self.pedal_axis)

    def controls(self):
        """The stick inches these axes are worth, stops applied."""
        return PilotControls.from_axes(*self.axes())

    def reset(self, controls=None):
        """Put the axes on a set of stick positions (all centred by default).

        This is the inverse of :meth:`controls`, and it is what a reset or a trim
        needs: the hands have to be put where the aircraft already is, or the
        first frame after the reset is a step input from centre.
        """
        if controls is None:
            self.collective_axis = 0.0
            self.long_stick_axis = 0.0
            self.lat_stick_axis = 0.0
            self.pedal_axis = 0.0
        else:
            self.collective_axis = _clip(controls.collective_fraction, 0.0, 1.0)
            self.long_stick_axis = _clip(
                controls.long_stick / (0.5 * UH1_LONG_STICK_TRAVEL_IN), -1.0, 1.0)
            self.lat_stick_axis = _clip(
                controls.lat_stick / (0.5 * UH1_LAT_STICK_TRAVEL_IN), -1.0, 1.0)
            self.pedal_axis = _clip(
                controls.pedal / (0.5 * UH1_PEDAL_TRAVEL_IN), -1.0, 1.0)
        return self

    def step(self, dt, collective=0.0, long_stick=0.0, lat_stick=0.0,
             pedal=0.0):
        """One frame of a keyboard, told which keys are *held*.

        Each argument is that control's key state: 0 for nothing held, -1 or +1
        for a direction held.  Returns the stick positions, so the frame loop of
        a caller is ``sim.step(dt, pilot.step(dt, **keys))`` - or
        :meth:`Simulation.fly`, which is that in one call.
        """
        dt = max(float(dt), 0.0)
        if collective:
            self.collective_axis = _clip(
                _slew(self.collective_axis, math.copysign(1.0, collective),
                      self.collective_rate, dt), 0.0, 1.0)
        self.long_stick_axis = self._hand(self.long_stick_axis, long_stick,
                                          self.cyclic_rate, dt)
        self.lat_stick_axis = self._hand(self.lat_stick_axis, lat_stick,
                                         self.cyclic_rate, dt)
        self.pedal_axis = self._hand(self.pedal_axis, pedal, self.pedal_rate,
                                     dt)
        return self.controls()

    def _hand(self, axis, key, rate, dt):
        """Move one spring-centred axis: toward the key, or back to centre."""
        if key:
            return _slew(axis, math.copysign(1.0, key), rate, dt)
        return _slew(axis, 0.0, self.centring_rate, dt)

    def set_axes(self, collective=None, long_stick=None, lat_stick=None,
                 pedal=None, dt=None):
        """Put axes at absolute positions, for a joystick or a script.

        An axis is left alone when it is ``None``.  With a ``dt`` the move is
        limited by the rate constants, so a joystick that jumps to its stop
        still takes the 0.6 s a hand would; without one it is instant, which is
        what a script setting up an initial condition wants.
        """
        if collective is not None:
            self.collective_axis = _clip(
                _slew(self.collective_axis, _clip(collective, 0.0, 1.0),
                      self.collective_rate, dt), 0.0, 1.0)
        if long_stick is not None:
            self.long_stick_axis = _slew(self.long_stick_axis,
                                         _clip(long_stick, -1.0, 1.0),
                                         self.cyclic_rate, dt)
        if lat_stick is not None:
            self.lat_stick_axis = _slew(self.lat_stick_axis,
                                        _clip(lat_stick, -1.0, 1.0),
                                        self.cyclic_rate, dt)
        if pedal is not None:
            self.pedal_axis = _slew(self.pedal_axis, _clip(pedal, -1.0, 1.0),
                                    self.pedal_rate, dt)
        return self

    def __str__(self):
        return ("collective %5.2f | long %+5.2f | lat %+5.2f | pedal %+5.2f"
                % self.axes())


def _clip(value, low, high):
    """Clamp *value* into ``[low, high]``."""
    return max(low, min(high, float(value)))


def _finite(state):
    """Is every number of a :class:`FlightState` finite?"""
    return all(math.isfinite(value) for value in state.values())


def _out_of_envelope(state):
    """Has a state left the volume any UH-1 flight could stay inside?

    Non-finite numbers count as out, as do an altitude, a speed or a rate beyond
    ``UH1_ENVELOPE_CEILING_*``.  This is the check that stops a run away
    :mod:`airframe` state before a renderer, a camera or a gauge is handed it;
    see :meth:`Simulation._advance` for why a state can run away at all.
    """
    if not _finite(state):
        return True
    if abs(state.altitude) > UH1_ENVELOPE_CEILING_M:
        return True
    if max(abs(state.position.x), abs(state.position.y)) > UH1_ENVELOPE_CEILING_M:
        return True
    return (state.speed > UH1_ENVELOPE_CEILING_MPS
            or state.rates.length() > UH1_ENVELOPE_CEILING_RADPS)


def _slew(axis, target, rate, dt):
    """Move *axis* toward *target*, at most ``rate`` per second for *dt*.

    A *dt* of ``None`` means no time passes for the rate, so the axis arrives:
    that is how a script sets an initial condition.  The move is linear rather
    than a first order lag because a hand that has reached the stop stays there,
    which a lag would not do.
    """
    if dt is None or rate <= 0.0:
        return target
    span = rate * max(dt, 0.0)
    if target > axis:
        return min(target, axis + span)
    return max(target, axis - span)


# ---------------------------------------------------------------------------
# The camera that follows it.
# ---------------------------------------------------------------------------


@dataclass
class ChaseCamera:
    """A camera behind and above the helicopter, following it with a lag.

    Everything is in the renderer's world of :func:`to_renderer`, so
    :meth:`eye_target_up` is exactly the triple ``gluLookAt`` takes and a caller
    has nothing left to convert.  The eye sits behind the *smoothed* nose and
    above the aircraft along the world's own y, and the up vector is always the
    world's own ``(0, 1, 0)``, which keeps the horizon level however the
    helicopter rolls; a camera that rolls with its subject is a taste, and a
    caller wanting it has only to build its own triple.

    The lag is on the aircraft's pose rather than on the camera's position, and
    it is a first order one of ``lag_s`` seconds: the camera trails where the
    helicopter *was*, which is what stops a chase camera twitching with every
    attitude change.  ``lag_s`` of zero locks it on.

    ``min_height`` keeps the eye off the ground, which matters on the pad: the
    eye hangs ``height`` above the aircraft, and a parked helicopter is only
    1.4 m up.
    """

    distance: float = 14.0        # m behind the aircraft
    height: float = 4.5           # m above it
    look_ahead: float = 6.0       # m ahead of it that the view is aimed at
    min_height: float = 0.6       # m, the eye never goes below this
    lag_s: float = 0.25
    position: Vector3 = field(default_factory=Vector3)   # the smoothed aircraft
    nose: Vector3 = field(default_factory=lambda: Vector3(0.0, 0.0, -1.0))
    ready: bool = False

    def update(self, dt, position, basis):
        """Follow an aircraft: its renderer position and its body basis.

        ``basis`` is :meth:`Simulation.render_basis`'s ``(nose, starboard,
        down)``; only the nose is used, since the eye only needs to know which
        way the helicopter is pointing over the ground.
        """
        if not self.ready:
            self.position, self.nose, self.ready = position, basis[0], True
            return self
        weight = (1.0 if self.lag_s <= 0.0
                  else 1.0 - math.exp(-max(float(dt), 0.0) / self.lag_s))
        self.position = self.position + (position - self.position) * weight
        self.nose = (self.nose + (basis[0] - self.nose) * weight).normalized()
        return self

    def reset(self):
        """Forget the aircraft, so that the next update snaps onto it."""
        self.position = Vector3()
        self.nose = Vector3(0.0, 0.0, -1.0)
        self.ready = False
        return self

    def eye_target_up(self):
        """``(eye, target, up)`` in the renderer's world, for ``gluLookAt``.

        The nose flattened onto the horizontal plane, so that a helicopter in a
        steep climb does not put the camera underneath itself.
        """
        flat = Vector3(self.nose.x, 0.0, self.nose.z).normalized()
        if flat.length() < 1e-6:
            flat = Vector3(0.0, 0.0, -1.0)
        eye = (self.position - flat * self.distance
               + Vector3(0.0, self.height, 0.0))
        if eye.y < self.min_height:
            eye = Vector3(eye.x, self.min_height, eye.z)
        return (eye, self.position + flat * self.look_ahead,
                Vector3(0.0, 1.0, 0.0))


# ---------------------------------------------------------------------------
# The frame loop.
# ---------------------------------------------------------------------------


@dataclass
class Simulation:
    """An aircraft, a clock and the ground under it: the whole frame loop.

    A fresh :class:`Simulation` is a trimmed hover at ``start_altitude`` -
    ``trim_at_start=False`` leaves the aircraft wherever it already is, for a
    script bringing its own initial condition.  A caller's frame loop is two
    lines::

        controls = sim.pilot.step(frame_dt, long_stick=1.0)   # the keys held
        sim.step(frame_dt, controls)                          # fly the frame

    or one, :meth:`fly`, and the aircraft is drawn with
    ``glMultMatrixf(sim.render_matrix())`` while :meth:`render_position` and
    :meth:`render_basis` feed a :class:`ChaseCamera`.

    What the counters mean, since a HUD shows them and a test asserts them:
    ``frames`` is how many frames a caller asked for, ``steps`` how many 1/60 s
    physics steps were actually flown, ``sim_time`` the physics time those steps
    have flown (the sum of the steps taken, so ``steps * dt`` to the rounding of
    ``steps`` additions for a frame loop), ``dropped_steps`` how many steps the
    accumulator had to throw
    away because a frame was worth more than ``max_steps_per_frame`` of them, and
    ``frame_carry`` the leftover - shorter than one step - that the next frame
    will spend, which is the number an interpolation scheme would want if one is
    ever added.

    Nothing here reads a clock, so two runs on the same frame times are
    bit-identical: the self test asserts exactly that.
    """

    airframe: Airframe = field(default_factory=airframe_preset)
    pilot: PilotInput = field(default_factory=PilotInput)
    dt: float = SIM_TIME_STEP_S
    max_steps_per_frame: int = SIM_MAX_STEPS_PER_FRAME
    wind: Optional[Vector3] = None
    start_altitude: float = 200.0
    trim_at_start: bool = True

    # Where the simulation has got to.  Read them; the methods keep them true.
    sim_time: float = 0.0
    frames: int = 0
    steps: int = 0
    dropped_steps: int = 0
    frame_carry: float = 0.0
    controls: PilotControls = field(default_factory=PilotControls)
    forces: Optional[BodyForces] = None
    on_ground: bool = False
    crashed: bool = False
    #: Where the skids are standing, the NED x and y the aircraft touched down
    #: at, while it is on the ground; ``None`` in the air.  See :meth:`_ground`.
    ground_position: Optional[Vector3] = None
    trim_controls: Optional[PilotControls] = None
    trim_state: Optional[FlightState] = None

    def __post_init__(self):
        if self.trim_at_start:
            self.trim(self.start_altitude)
        else:
            self.reset(state=self.airframe.state, controls=self.controls)

    def trim(self, altitude=200.0, iterations=40):
        """Trim to a hover and start from it, remembering it as the reference.

        Returns ``(PilotControls, FlightState)`` from
        :meth:`airframe.Airframe.trim_hover`, and leaves the aircraft flying it
        with the pilot's own axes on the trimmed positions - so a keyboard that
        then releases everything holds the hover instead of stepping the cyclic
        back to centre.  The reference is what :meth:`in_trim` compares against
        and what a later :meth:`reset` comes back to.
        """
        controls, state = self.airframe.trim_hover(altitude=altitude,
                                                   iterations=iterations)
        self.trim_controls, self.trim_state = controls, state
        self.reset(state=state, controls=controls)
        return controls, state

    def trim_level_flight(self, airspeed, altitude=None, iterations=40,
                          tolerance=0.1):
        """Trim to level flight at *airspeed* and start from it.

        Returns ``(PilotControls, FlightState)`` from
        :meth:`airframe.Airframe.trim_level_flight` - the same pair
        :meth:`trim` returns from the hover - and leaves the aircraft flying it:
        the trim's own state, at its airspeed and its attitude, with the pilot's
        axes on the trimmed positions, so a keyboard that then releases
        everything flies it level instead of stepping the sticks back to centre.
        The trim is remembered as the reference :meth:`in_trim` compares against
        and :meth:`reset` comes back to, so a HUD has a trim light at 60 kt as
        well as in a hover: ``sim.trim_level_flight(60 * KNOT)``.

        *altitude* defaults to where the aircraft is - the last trim's altitude,
        or ``start_altitude`` - and matters only inside the ground effect, since
        the trim is solved out of the cushion from 200 m up.  A speed the model
        cannot hold at all comes back with a residual around it rather than as an
        error, and :meth:`airframe.Airframe.equilibrium_residual` is what says
        how big.
        """
        if altitude is None:
            altitude = (self.trim_state.altitude if self.trim_state is not None
                        else self.start_altitude)
        controls, state = self.airframe.trim_level_flight(
            airspeed, altitude=altitude, iterations=iterations,
            tolerance=tolerance)
        self.trim_controls, self.trim_state = controls, state
        self.reset(state=state, controls=controls)
        return controls, state

    def reset(self, state=None, controls=None, on_ground=False, altitude=None,
              heading_deg=0.0):
        """Start again: a level aircraft, no rates, the clock back to zero.

        With no *state* the aircraft is placed at the ground or at *altitude*
        (the trim state's own altitude, or ``start_altitude``, when neither
        applies) pointing at *heading_deg*, with no rates; with no *controls* it
        gets the trim stick positions, or the collective's down stop for the
        parked ``on_ground=True`` start that :meth:`on_the_pad` is.  The last
        trim's roll, pitch *and velocity* come with it, since a UH-1's hover
        attitude - nose up by the 4.4 deg its c.g. is aft of the hub - is part
        of the trim rather than something to find again and a level trim is
        flown at its own airspeed, which is 0 m/s for a hover; a parked or never
        trimmed aircraft is placed level and still instead.  The airframe's lags
        are reset onto whichever positions those are, so the first frame is
        flown from the trim rather than pulling the collective up from nothing.
        """
        parked = on_ground and state is None and controls is None
        if state is None:
            roll, pitch = 0.0, 0.0
            velocity = Vector3()
            if self.trim_state is not None and not on_ground:
                roll, pitch = self.trim_state.attitude.x, self.trim_state.attitude.y
                velocity = self.trim_state.velocity
            if on_ground:
                height = UH1_CG_HEIGHT_ON_GROUND
            elif altitude is not None:
                height = altitude
            elif self.trim_state is not None:
                height = self.trim_state.altitude
            else:
                height = self.start_altitude
            state = FlightState(position=Vector3(0.0, 0.0, -height),
                                velocity=velocity,
                                attitude=Vector3(roll, pitch,
                                                 math.radians(heading_deg)),
                                rates=Vector3())
        if parked:
            controls = PilotControls()
        elif controls is None:
            controls = (self.trim_controls if self.trim_controls is not None
                        else PilotControls())
        self.airframe.reset(state, controls)
        self.controls = controls
        self.pilot.reset(controls)
        # A zero length step, so that the forces a HUD reads before the first
        # frame are this state's own rather than the last flight's.
        self.forces = self.airframe.step(0.0, controls, wind=self.wind)
        self.sim_time = 0.0
        self.frames = 0
        self.steps = 0
        self.dropped_steps = 0
        self.frame_carry = 0.0
        self.crashed = False
        self.ground_position = None
        self.on_ground = (self.airframe.state.altitude
                          <= UH1_CG_HEIGHT_ON_GROUND + GROUND_CLEARANCE_M)
        return self

    def on_the_pad(self, heading_deg=0.0):
        """Park it: skids on the ground, collective down, everything level."""
        return self.reset(on_ground=True, heading_deg=heading_deg)

    def set_wind(self, wind):
        """A steady wind, in NED m/s (``None`` for still air)."""
        self.wind = wind
        return self

    def step(self, frame_dt, controls=None):
        """Fly one frame of *frame_dt* seconds with *controls* held.

        The frame goes into the accumulator and whole ``dt`` steps are run out of
        it, at most ``max_steps_per_frame`` of them; the remainder is carried
        into the next frame, and if more steps were owed than the cap allows, the
        rest is counted in ``dropped_steps`` and never flown.  *controls*
        defaults to the last set, which is what a headless hold wants.  Returns
        the :class:`BodyForces` at the end of the last step of the frame.
        """
        controls = self.controls if controls is None else controls.clipped()
        self.controls = controls
        self.frames += 1
        self.frame_carry += max(float(frame_dt), 0.0)
        steps = 0
        while self.frame_carry >= self.dt and steps < self.max_steps_per_frame:
            self._advance(self.dt, controls)
            self.frame_carry -= self.dt
            steps += 1
        if self.frame_carry >= self.dt:
            dropped = int(self.frame_carry / self.dt)
            self.dropped_steps += dropped
            self.frame_carry -= dropped * self.dt
        return self.forces

    def fly(self, frame_dt, **keys):
        """One frame with the keyboard: sample :attr:`pilot`, then step.

        The keywords are :meth:`PilotInput.step`'s held key states, so a frame
        loop reads ``sim.fly(frame_dt, collective=1.0, long_stick=up)``.
        """
        return self.step(frame_dt, self.pilot.step(frame_dt, **keys))

    def step_fixed(self, dt=None, controls=None):
        """One physics step of exactly *dt* seconds, no accumulator.

        For a headless script that wants to fly a set number of steps: the
        regression against the report's figures, :mod:`regression_tm73254`, is
        who this exists for.  The clock, the step count and the ground all move
        as they do inside :meth:`step`; only ``frames`` and the accumulator do
        not.
        """
        controls = self.controls if controls is None else controls.clipped()
        self.controls = controls
        self._advance(self.dt if dt is None else dt, controls)
        return self.forces

    def _advance(self, dt, controls):
        """One physics step, then the ground policy for the state it left.

        A step whose state comes back out of the envelope - non-finite, or beyond
        the ceilings of ``UH1_ENVELOPE_CEILING_*`` - is refused: the last good
        state goes back into the airframe, the aircraft is called crashed, and
        the frame is spent without moving it.  That is not a substitute for
        physics, it is the honest end of the model's envelope: the rotor is at
        100 per cent with no engine and no rotor speed dynamics, so a violent
        hands-off case, a dive with the collective at the down stop above all,
        can run away inside a 60 Hz explicit Runge-Kutta step, and a renderer, a
        camera or a gauge must never be handed the result.
        """
        before = self.airframe.state
        descent = before.ground_velocity().z
        self.forces = self.airframe.step(dt, controls, wind=self.wind)
        self.steps += 1
        self.sim_time += dt
        if _out_of_envelope(self.airframe.state):
            self.airframe.state = before
            self.forces = self.airframe.step(0.0, controls, wind=self.wind)
            self.crashed = True
            return
        self._ground(dt, descent, before.position)

    def _ground(self, dt, descent, touchdown):
        """Put the aircraft back on the ground if the step took it through it.

        A floor, not a skid model: the c.g. is held on the skid line, the descent
        is taken out of the velocity, ``GROUND_FRICTION_PER_S`` takes out the
        residual ground speed and yaw rate, and roll and pitch are levelled again
        at ``GROUND_LEVELLING_PER_S`` - the heading stays free, so pedal still
        pivots it on its skids.  A descent faster than ``UH1_HARD_LANDING_MPS``
        at the moment of touchdown sets ``crashed``, which only a
        :meth:`reset` clears.

        The skids carry the aircraft's weight, which is the one piece of physics
        in here: while the rotor's lift is no more than the weight's component
        along the body z axis, they hold it - the whole ground velocity goes, so
        a helicopter sitting on the pad stays sitting and one that arrives does
        not bounce or slide, since no skid friction is modelled - with only the
        heading free, turned by pedal against ``GROUND_FRICTION_PER_S``.  The
        moment the rotor lifts more than that the aircraft is free to accelerate
        away, so a pickup is a smooth departure from rest rather than the jump an
        accumulating velocity would give.

        It replaces the state wholesale, which is fine because everything outside
        this policy reads the state through the airframe, and it is the one thing
        in this module TM-73254 does not contain.  *touchdown* is the position the
        frame started at, so that the skids are pinned where the frame found the
        aircraft rather than where the frame's own integration took it.
        """
        state = self.airframe.state
        if state.altitude > UH1_CG_HEIGHT_ON_GROUND + GROUND_CLEARANCE_M:
            self.on_ground = False
            self.ground_position = None
            return
        if not self.on_ground and descent > UH1_HARD_LANDING_MPS:
            self.crashed = True
        self.on_ground = True
        if self.ground_position is None:
            self.ground_position = Vector3(touchdown.x, touchdown.y)
        matrix = direction_cosine_matrix(state.attitude.x, state.attitude.y,
                                         state.attitude.z)
        fade = math.exp(-max(float(dt), 0.0) * GROUND_FRICTION_PER_S)
        level = math.exp(-max(float(dt), 0.0) * GROUND_LEVELLING_PER_S)
        # The ground velocity the skids see: NED, down positive.
        ground = apply(matrix, state.velocity)
        lift = 0.0 if self.forces is None else self.forces.lift
        weight = (self.airframe.mass * GRAVITY * math.cos(state.attitude.x)
                  * math.cos(state.attitude.y))
        if lift <= weight:
            # The skids hold it: no sliding is modelled, so it stays standing
            # where it touched down until the rotor lifts more than it weighs.
            ground = Vector3()
            position = self.ground_position
        else:
            # Climbing out: the skids' friction takes the ground speed out, and
            # an upward velocity is left alone.
            ground = Vector3(ground.x * fade, ground.y * fade,
                             min(ground.z, 0.0))
            position = Vector3(state.position.x, state.position.y)
        self.airframe.state = FlightState(
            position=Vector3(position.x, position.y, -UH1_CG_HEIGHT_ON_GROUND),
            velocity=apply(transpose(matrix), ground),
            attitude=Vector3(state.attitude.x * level, state.attitude.y * level,
                             state.attitude.z),
            rates=Vector3(0.0, 0.0, state.rates.z * fade))

    # --- what a renderer asks for -----------------------------------------

    def render_position(self):
        """The c.g. in the renderer's world, m: ``(east, up, -north)``.

        So ``x`` is east, ``y`` is altitude above the ground and ``z`` is south.
        """
        return to_renderer(self.airframe.state.position)

    def render_rotation(self):
        """The body to renderer rotation, as a 3 by 3 of row tuples.

        Row *i* is how much of each body axis points along the renderer's *i*th
        axis, so column *j* - which :meth:`render_basis` returns - is the body
        axis *j* in the renderer's world.  It is ``to_renderer`` applied to the
        rows of :func:`airframe.direction_cosine_matrix`: the renderer's east is
        the NED row for east, and its up and south are the negated NED rows for
        down and north.
        """
        attitude = self.airframe.state.attitude
        ned = direction_cosine_matrix(attitude.x, attitude.y, attitude.z)
        return (ned[1], tuple(-value for value in ned[2]),
                tuple(-value for value in ned[0]))

    def render_basis(self):
        """``(nose, starboard, down)`` in the renderer's world, unit vectors.

        Body x, y and z as the renderer sees them, which is what a shadow, a
        camera or an instrument wanting a direction rather than a matrix wants.
        """
        rotation = self.render_rotation()
        return (Vector3(*(rotation[row][0] for row in range(3))),
                Vector3(*(rotation[row][1] for row in range(3))),
                Vector3(*(rotation[row][2] for row in range(3))))

    def render_matrix(self):
        """The aircraft's place in the renderer's world, as 16 floats.

        Column major, translation included, in exactly the order
        ``glMultMatrixf`` reads: the rotation's first column, then its
        translation x, and so on, then ``(0, 0, 0, 1)``.  Drawing the helicopter
        is therefore one call, with no Euler order and no sign left to get wrong
        between here and the screen.
        """
        rotation = self.render_rotation()
        position = self.render_position().as_tuple()
        values = []
        for column in range(3):
            for row in range(3):
                values.append(rotation[row][column])
            values.append(0.0)
        values.extend(position)
        values.append(1.0)
        return tuple(values)

    # --- what a HUD asks for ----------------------------------------------

    def in_trim(self, tolerance_in=0.25, tolerance_mps=0.5,
                tolerance_deg=2.0):
        """Is the aircraft still flying the trim :meth:`trim` last found?

        True when every control is within *tolerance_in* inches of the trimmed
        position and the state is within *tolerance_mps* of the trim speed and
        *tolerance_deg* of its attitude.  False when there is no trim to compare
        with, which is what ``trim_at_start=False`` and a hand-flown script both
        leave behind.
        """
        if self.trim_controls is None or self.trim_state is None:
            return False
        for name in ("collective", "long_stick", "lat_stick", "pedal"):
            if abs(getattr(self.controls, name)
                   - getattr(self.trim_controls, name)) > tolerance_in:
                return False
        state, reference = self.airframe.state, self.trim_state
        if abs(state.speed - reference.speed) > tolerance_mps:
            return False
        degrees = math.degrees
        return (abs(degrees(state.attitude.x - reference.attitude.x))
                <= tolerance_deg
                and abs(degrees(state.attitude.y - reference.attitude.y))
                <= tolerance_deg)

    def telemetry(self):
        """A :class:`Telemetry` for the state the last step left behind."""
        state = self.airframe.state
        forces = self.forces
        angles = self.airframe.command_angles(self.controls)
        ground = state.ground_velocity()
        attitude = state.attitude_deg
        rates = state.rates_deg
        rotor = forces.rotor
        return Telemetry(
            sim_time=self.sim_time,
            alt_agl=state.altitude,
            height_rate=0.0 - ground.z,
            airspeed=state.speed,
            airspeed_kt=state.speed / KNOT,
            ground_speed=ground.length(),
            roll_deg=attitude.x, pitch_deg=attitude.y, yaw_deg=attitude.z,
            roll_rate_deg=rates.x, pitch_rate_deg=rates.y, yaw_rate_deg=rates.z,
            collective_in=self.controls.collective,
            long_stick_in=self.controls.long_stick,
            lat_stick_in=self.controls.lat_stick,
            pedal_in=self.controls.pedal,
            collective_fraction=self.controls.collective_fraction,
            collective_deg=angles.collective_pitch_deg,
            tail_collective_deg=angles.tail_collective_deg,
            control_tilt_deg=angles.control_tilt_deg,
            thrust=rotor.thrust, torque=rotor.torque, h_force=rotor.h_force,
            y_force=rotor.y_force, tail_thrust=forces.tail_thrust,
            lift=forces.lift, ground_effect=rotor.ground_effect,
            inflow_ratio=rotor.inflow_ratio,
            advance_ratio=rotor.advance_ratio,
            on_ground=self.on_ground, crashed=self.crashed,
            in_trim=self.in_trim())


# ---------------------------------------------------------------------------
# The self test and the demo, in the style of the three modules below.
# ---------------------------------------------------------------------------


def _self_test():
    """Checks on the clock, the ground, the pilot's hands and the renderer.

    Raises AssertionError on failure.  The demo calls this, so running this
    module is enough to validate the whole frame loop.  The physics itself is
    :mod:`airframe`'s to vouch for and its own self test is the one that does it;
    what is checked here is what this module adds around it: that the accumulator
    flies the frame times it is given and drops the steps it cannot, that a
    trimmed hover holds through the loop, that the pilot's axes move the way a
    hand would and arrive as the stick positions the model expects, that the
    ground floor holds, levels, lets a helicopter lift off from it and calls a
    hard arrival a crash, and that the renderer's conversion points a nose east,
    a nose-up attitude up and a starboard roll down.
    """
    # The two configurations, and the skid line the ground works from.
    assert airframe_preset("simulation").mass == UH1_SIM_MASS
    flight_test = airframe_preset("flight test")
    assert flight_test.mass == UH1_TEST_MASS
    assert flight_test.rotor_time_constant == UH1_ROTOR_TIME_CONSTANT_SIM_S
    assert not airframe_preset("simulation", ground_effect=False).ground_effect
    try:
        airframe_preset("nonsense")
    except ValueError:
        pass
    else:
        raise AssertionError("an unknown preset name was accepted")
    # 136.5 in of hub waterline less the 6.79 ft of hub above the c.g. leaves
    # 55.0 in, which is the height an upright UH-1's c.g. stands at.
    assert abs(UH1_CG_HEIGHT_ON_GROUND / INCH - 55.0) < 0.1, UH1_CG_HEIGHT_ON_GROUND

    # The renderer's world is a rotation, and the way back is its inverse.
    north = Vector3(1.0, 0.0, 0.0)
    east = Vector3(0.0, 1.0, 0.0)
    down = Vector3(0.0, 0.0, 1.0)
    assert to_renderer(north).as_tuple() == (0.0, 0.0, -1.0)
    assert to_renderer(east).as_tuple() == (1.0, 0.0, 0.0)
    assert to_renderer(down).as_tuple() == (0.0, -1.0, 0.0)
    for vector in (north, east, down, Vector3(1.5, -2.5, 3.5)):
        assert (from_renderer(to_renderer(vector)) - vector).length() < 1e-15

    # A fresh simulation is the trimmed hover airframe's self test proves holds,
    # with the forces and the trim light already readable before any frame.
    sim = Simulation()
    assert sim.airframe.mass == UH1_SIM_MASS
    assert abs(sim.airframe.state.altitude - 200.0) < 1e-9
    assert not sim.on_ground and not sim.crashed
    assert sim.in_trim()
    telemetry = sim.telemetry()
    weight = sim.airframe.mass * GRAVITY
    # The rotor's lift balances the weight's component along the body z axis,
    # which is what a trimmed hover is: the same balance airframe's own self test
    # asserts to within a newton, seen through the telemetry.
    tilt = (math.cos(math.radians(telemetry.roll_deg))
            * math.cos(math.radians(telemetry.pitch_deg)))
    assert abs(telemetry.lift - weight * tilt) < 1.0, telemetry.lift
    assert abs(telemetry.thrust - weight) < 0.05 * weight
    assert abs(telemetry.airspeed) < 1e-9 and abs(telemetry.height_rate) < 1e-9
    assert abs(telemetry.alt_ft - 200.0 / FOOT) < 0.01
    assert abs(telemetry.height_rate_fpm) < 1e-9
    assert abs(telemetry.collective_deg
               - sim.airframe.command_angles(sim.controls)
               .collective_pitch_deg) < 1e-12
    assert "nan" not in str(telemetry) and "trim" in str(telemetry)

    # Five seconds of frames hold it: the same assertion airframe's self test
    # makes, now through the clock.
    for _ in range(300):
        sim.step(SIM_TIME_STEP_S)
    assert sim.frames == 300 and sim.steps == 300
    assert sim.dropped_steps == 0 and sim.frame_carry < SIM_TIME_STEP_S
    assert abs(sim.sim_time - 300.0 * SIM_TIME_STEP_S) < 1e-9
    state = sim.airframe.state
    assert abs(state.altitude - 200.0) < 0.01
    assert abs(state.position.x) < 0.01 and abs(state.position.y) < 0.01
    assert state.speed < 0.01

    # Level flight through the same loop: the report's own 60 kt condition, on
    # the 6158 lb aircraft of its figures 2 to 9, trimmed and then held for five
    # seconds of frames at its own airspeed and attitude.  The trim is a
    # reference like the hover's, so the light reads the same way, and a reset
    # comes back to it at 60 kt rather than to a hover.
    flying = Simulation(airframe=airframe_preset("flight test"))
    hover_controls, hover_state = flying.airframe.trim_hover()
    flying.trim_level_flight(60.0 * KNOT)
    assert flying.in_trim()
    level = flying.telemetry()
    assert abs(level.airspeed_kt - 60.0) < 1e-9, level.airspeed_kt
    assert abs(level.height_rate) < 1e-9 and abs(level.height_rate_fpm) < 1e-6
    assert abs(level.alt_agl - 200.0) < 1e-9
    # Less collective and less nose up than the hover of the same weight: the
    # rotor is in its own wash, which is the whole of translational lift.
    assert level.collective_in < hover_controls.collective - 0.5
    assert level.pitch_deg < math.degrees(hover_state.attitude.y)
    assert abs(flying.pilot.collective_axis
               - flying.trim_controls.collective_fraction) < 1e-12
    for _ in range(300):
        flying.step(SIM_TIME_STEP_S)
    assert flying.in_trim()
    held = flying.telemetry()
    assert abs(held.airspeed_kt - 60.0) < 0.01
    assert abs(held.alt_agl - 200.0) < 0.01 and abs(held.height_rate) < 0.01
    state = flying.airframe.state
    assert abs(state.position.x - flying.sim_time * 60.0 * KNOT) < 0.01
    assert abs(state.position.y) < 1e-4          # straight up the north axis
    flying.reset()
    assert flying.in_trim() and abs(flying.airframe.state.speed
                                    - 60.0 * KNOT) < 1e-9

    # The same frame times in different slices fly the same aircraft: ten frames
    # of 16.7 ms are ten steps, bit for bit the ten steps step_fixed flies.
    sliced = Simulation()
    stepwise = Simulation()
    for _ in range(10):
        sliced.step(0.0167)
    for _ in range(10):
        stepwise.step_fixed()
    assert sliced.steps == 10 and stepwise.steps == 10
    assert sliced.airframe.state.values() == stepwise.airframe.state.values()
    assert abs(sliced.sim_time - 10.0 * SIM_TIME_STEP_S) < 1e-9

    # A frame that took a second keeps 83 ms of it and drops the rest, so that a
    # dragged window cannot teleport the aircraft: five steps, about fifty five
    # dropped, and less than one step left over.
    hitch = Simulation()
    hitch.step(1.0)
    assert hitch.frames == 1 and hitch.steps == 5
    assert 54 <= hitch.dropped_steps <= 55, hitch.dropped_steps
    assert hitch.frame_carry < SIM_TIME_STEP_S

    # With no trim found yet there is no trim light and nothing to reset to; the
    # aircraft is left exactly where it was instead.
    raw = Simulation(trim_at_start=False)
    assert not raw.in_trim() and raw.trim_state is None
    assert raw.airframe.state.values() == FlightState().values()
    assert raw.on_ground and not raw.crashed

    # The pilot's hands.  A held key moves an axis at its rate - full cyclic
    # travel in 0.625 s at 1.6 per second, the collective at 0.55 of its travel
    # per second - and the stops are the model's own, applied on the way out.
    hands = PilotInput()
    hands.step(0.25, collective=1.0, long_stick=1.0, lat_stick=-1.0, pedal=1.0)
    assert abs(hands.collective_axis - 0.1375) < 1e-12    # 0.55 per second
    assert abs(hands.long_stick_axis - 0.4) < 1e-12       # 1.6 per second
    assert abs(hands.lat_stick_axis + 0.4) < 1e-12
    assert abs(hands.pedal_axis - 0.5) < 1e-12            # 2.0 per second
    # Released, the cyclic and the pedals spring back to centre - 0.6 of travel
    # in the 0.25 s below, which is more than any of them has left - and the
    # collective is a ratchet that stays exactly where it was left.
    hands.step(0.25)
    assert hands.long_stick_axis == 0.0 and hands.lat_stick_axis == 0.0
    assert hands.pedal_axis == 0.0
    assert hands.collective_axis == 0.1375
    # Held long enough, the axes reach their stops and no further, and the inches
    # they are worth are the travels of TM-73254 table 2.
    hands.step(20.0, long_stick=1.0, collective=-1.0)
    assert hands.long_stick_axis == 1.0 and hands.collective_axis == 0.0
    assert hands.controls().long_stick == 0.5 * UH1_LONG_STICK_TRAVEL_IN
    assert hands.controls() == PilotControls.from_axes(*hands.axes())
    # A joystick arrives absolute; with a frame time it still moves at a hand's
    # rate rather than jumping, and without one it is instant.
    hands.set_axes(collective=1.0)
    assert hands.collective_axis == 1.0
    hands.set_axes(collective=0.0, dt=0.1)
    assert abs(hands.collective_axis - (1.0 - 0.055)) < 1e-12
    # And the axes can be put on stick positions, which is the inverse of that.
    hands.reset(sim.trim_controls)
    round_trip = hands.controls()
    for name in ("collective", "long_stick", "lat_stick", "pedal"):
        assert abs(getattr(round_trip, name)
                   - getattr(sim.trim_controls, name)) < 1e-12

    # On the pad: the collective's down stop, the skids on the ground.  Two
    # seconds of frames must not move it at all, even though a rotor at zero
    # collective pushes downwards - that is the floor doing its work.
    pad = Simulation()
    pad.on_the_pad()
    assert pad.on_ground and not pad.crashed
    assert pad.controls == PilotControls()
    assert pad.pilot.axes() == (0.0, 0.0, 0.0, 0.0)
    assert abs(pad.airframe.state.altitude - UH1_CG_HEIGHT_ON_GROUND) < 1e-12
    for _ in range(120):
        pad.step(SIM_TIME_STEP_S)
    assert pad.on_ground and not pad.crashed
    assert abs(pad.airframe.state.altitude - UH1_CG_HEIGHT_ON_GROUND) < 1e-9
    # The skids hold it: it does not slide or fly, and its roll and pitch stay
    # levelled, with only the heading free to pivot.
    assert pad.airframe.state.position.x == 0.0
    assert pad.airframe.state.position.y == 0.0
    assert pad.airframe.state.speed < 1e-9
    # The heading stays free, so the tail rotor's moment at the down stop is only
    # damped by the skids rather than held; the roll and pitch rates are zero
    # because the floor levels the aircraft.
    assert abs(pad.airframe.state.rates.x) < 1e-9
    assert abs(pad.airframe.state.rates.y) < 1e-9
    assert abs(pad.airframe.state.rates.z) < 0.2
    assert pad.telemetry().on_ground

    # And it lifts off from there with the keyboard alone: the collective ratchets
    # up to the trimmed fraction and stops, the skids let go once the rotor is
    # lifting more than the aircraft weighs, and the aircraft flies away from the
    # ground.  What it does after that is the heave mode's business, so what is
    # asserted here is only what the wrapper owns: that it left, that the ratchet
    # stopped where it was told to, that it never went through the ground, that
    # nothing broke and that the state stayed inside the envelope.
    hands = PilotInput()
    hands.reset(pad.trim_controls)
    hands.set_axes(collective=0.0)
    pad.pilot = hands
    trim_fraction = pad.trim_controls.collective_fraction
    lifted = False
    lowest = pad.airframe.state.altitude
    for frame in range(int(12.0 * 60)):
        pad.fly(SIM_TIME_STEP_S,
                collective=1.0 if hands.collective_axis < trim_fraction else 0.0)
        lifted = lifted or not pad.on_ground
        lowest = min(lowest, pad.airframe.state.altitude)
        assert not _out_of_envelope(pad.airframe.state)
    assert lifted
    assert abs(hands.collective_axis - trim_fraction) < 0.02
    assert lowest >= UH1_CG_HEIGHT_ON_GROUND - 1e-9
    # What the aircraft does once it is off - a lightly damped heave that brings
    # it back onto the skids a few seconds later, over the crash threshold, with
    # the ground cushion's kick behind it - is the model's own, and it is what a
    # pilot holding nothing but the collective gets.  What this module owns is
    # the ratchet and the floor: the collective stopped at the trim, nothing went
    # through the ground, and no state left the envelope.

    # A failed pickup, low enough to arrive before the model runs out of
    # envelope: the collective to the down stop from three metres above the skid
    # line, with the cyclic and pedals left in trim.  Nothing may ever be below
    # the skid line, the floor has to level it and take its motion away, and the
    # arrival is over UH1_HARD_LANDING_MPS, so it is a crash.
    failed = Simulation()
    failed.reset(
        altitude=UH1_CG_HEIGHT_ON_GROUND + 3.0,
        controls=PilotControls(long_stick=failed.trim_controls.long_stick,
                               lat_stick=failed.trim_controls.lat_stick,
                               pedal=failed.trim_controls.pedal))
    assert not failed.on_ground and not failed.crashed
    lowest = failed.airframe.state.altitude
    for _ in range(120):
        failed.step_fixed()
        lowest = min(lowest, failed.airframe.state.altitude)
        assert _finite(failed.airframe.state)
    assert lowest >= UH1_CG_HEIGHT_ON_GROUND - 1e-9
    assert failed.on_ground and failed.crashed
    assert failed.airframe.state.altitude == UH1_CG_HEIGHT_ON_GROUND
    assert failed.airframe.state.speed < 1e-6
    # The skid friction only damps the yaw rate, because the heading is left
    # free: at the down stop the tail rotor's own yaw moment is still there, and
    # a pilot with the collective down sits on the pedals.  What must be zero is
    # the roll and pitch rates, since the floor levels the aircraft.
    assert abs(failed.airframe.state.rates.x) < 1e-9
    assert abs(failed.airframe.state.rates.y) < 1e-9
    # A crashed aircraft stays crashed until it is reset, whatever is done with
    # the controls afterwards - here, holding the trim, which lifts it off its
    # skids again, because the model does not stop flying when the floor calls an
    # arrival a crash.  What a program does about that is its business.
    for _ in range(120):
        failed.step(SIM_TIME_STEP_S, failed.trim_controls)
    assert failed.crashed
    failed.reset()
    assert not failed.crashed and failed.in_trim()

    # The end of the envelope: the same pickup from thirty metres, which the
    # model cannot fly - at 100 per cent rotor with the collective at the down
    # stop the closed form's rotor pushes down by nearly two weights and there is
    # no engine or rotor speed dynamics to fall back on, so the state runs away
    # into a NaN inside one 60 Hz step.  The loop refuses that frame rather than
    # passing it on: the last good state is kept, the aircraft is called crashed
    # and every number a caller can read stays finite.
    lost = Simulation()
    lost.reset(altitude=30.0,
               controls=PilotControls(long_stick=lost.trim_controls.long_stick,
                                      lat_stick=lost.trim_controls.lat_stick,
                                      pedal=lost.trim_controls.pedal))
    kept = None
    for _ in range(int(10.0 * 60)):
        lost.step(SIM_TIME_STEP_S)
        assert _finite(lost.airframe.state)
        assert math.isfinite(lost.telemetry().thrust)
        if lost.crashed:
            if kept is None:
                kept = lost.airframe.state.values()
            else:
                assert lost.airframe.state.values() == kept
    assert kept is not None and lost.crashed and not lost.on_ground
    assert "nan" not in str(lost.telemetry())

    # The same floor entered slowly is a landing and not a crash: thirty
    # centimetres above the skids, descending at a metre and a half a second, in
    # trim.  It touches down without being called a crash - and then climbs away
    # again, because a trim collective inside the ground effect lifts more than
    # the aircraft weighs, which is what the cushion is.  (Coming down a metre a
    # second from half a metre higher never reaches the ground at all: the
    # cushion catches it first, which is the same effect.)
    soft = Simulation()
    soft.reset(state=FlightState(
        position=Vector3(0.0, 0.0, -(UH1_CG_HEIGHT_ON_GROUND + 0.3)),
        velocity=Vector3(0.0, 0.0, 1.5), attitude=Vector3(), rates=Vector3()),
        controls=soft.trim_controls)
    assert not soft.on_ground and not soft.crashed
    touched = False
    for _ in range(120):
        soft.step(SIM_TIME_STEP_S)
        touched = touched or soft.on_ground
        assert not soft.crashed
    assert touched

    # A steady wind is plumbed through to the model, and a helicopter hovering in
    # it drifts downwind.
    windy = Simulation()
    windy.set_wind(Vector3(0.0, 5.0, 0.0))       # 5 m/s towards the east
    for _ in range(600):
        windy.step(SIM_TIME_STEP_S)
    assert windy.airframe.state.ground_velocity().y > 0.5
    assert windy.telemetry().ground_speed > 0.5

    # The renderer's world: east is +x, up is +y, south is +z, so a helicopter
    # pointing east is drawn pointing east with its right hand to the south.
    pointing = Simulation()
    pointing.reset(state=FlightState(attitude=Vector3(0.0, 0.0,
                                                      math.radians(90.0))),
                   controls=pointing.trim_controls)
    nose, starboard, down_axis = pointing.render_basis()
    assert abs(nose.x - 1.0) < 1e-12 and abs(nose.z) < 1e-12
    assert abs(starboard.z - 1.0) < 1e-12
    assert abs(down_axis.y + 1.0) < 1e-12 and abs(down_axis.x) < 1e-12
    assert pointing.render_position().as_tuple() == (0.0, 0.0, 0.0)

    # The rotation is a rotation at any attitude, and the matrix that
    # glMultMatrixf is handed is those three columns, each with the position
    # after it.
    for roll, pitch, yaw in ((0.0, 0.0, 0.0), (0.3, -0.2, 1.0),
                             (-0.7, 0.4, -2.0)):
        trial = Simulation()
        trial.reset(state=FlightState(position=Vector3(1.0, 2.0, -3.0),
                                      velocity=Vector3(),
                                      attitude=Vector3(roll, pitch, yaw),
                                      rates=Vector3()))
        nose, starboard, down_axis = trial.render_basis()
        assert abs(nose.length() - 1.0) < 1e-12
        assert abs(nose.dot(starboard)) < 1e-12
        assert abs(nose.cross(starboard).dot(down_axis) - 1.0) < 1e-12
        matrix = trial.render_matrix()
        assert len(matrix) == 16 and matrix[15] == 1.0
        assert matrix[0:3] == nose.as_tuple()
        assert matrix[4:7] == starboard.as_tuple()
        assert matrix[8:11] == down_axis.as_tuple()
        assert matrix[3] == 0.0 and matrix[7] == 0.0 and matrix[11] == 0.0
        assert matrix[12:15] == (2.0, 3.0, -1.0)
        assert trial.render_position().as_tuple() == (2.0, 3.0, -1.0)

    # The two signs a renderer gets wrong: a nose-up attitude points the nose up,
    # and a roll to starboard drops the starboard wing.
    nose_up = Simulation()
    nose_up.reset(state=FlightState(attitude=Vector3(0.0, math.radians(10.0),
                                                     0.0)))
    assert nose_up.render_basis()[0].y > 0.15            # sin 10 deg = 0.1736
    rolled = Simulation()
    rolled.reset(state=FlightState(attitude=Vector3(math.radians(10.0), 0.0,
                                                    0.0)))
    assert rolled.render_basis()[1].y < -0.15

    # The chase camera hangs behind and above, aims ahead along the nose, keeps
    # the horizon level, stays off the ground, and trails when it has a lag.  The
    # aircraft is a freshly reset one, so that its heading is exactly north and
    # the geometry below is exact: the hover that was flown above has drifted a
    # microradian of yaw by now, and the camera follows that.
    level = Simulation()
    level.reset()
    camera = ChaseCamera(lag_s=0.0)
    camera.update(0.0, level.render_position(), level.render_basis())
    eye, target, up = camera.eye_target_up()
    position = level.render_position()
    assert abs(eye.x - position.x) < 1e-12
    assert abs(eye.y - (position.y + 4.5)) < 1e-12
    assert abs(eye.z - (position.z + 14.0)) < 1e-12      # 14 m south of it
    assert abs(target.z - (position.z - 6.0)) < 1e-12
    assert up.as_tuple() == (0.0, 1.0, 0.0)
    parked = Simulation()
    parked.on_the_pad()
    low = ChaseCamera(height=0.0, min_height=10.0)
    low.update(0.0, parked.render_position(), parked.render_basis())
    assert abs(low.eye_target_up()[0].y - 10.0) < 1e-12
    basis = level.render_basis()
    trailing = ChaseCamera(lag_s=0.5)
    trailing.update(0.0, Vector3(0.0, 0.0, 0.0), basis)
    assert trailing.position.as_tuple() == (0.0, 0.0, 0.0)
    trailing.update(0.5, Vector3(100.0, 0.0, 0.0), basis)
    assert abs(trailing.position.x - 100.0 * (1.0 - math.exp(-1.0))) < 1e-9
    for _ in range(200):
        trailing.update(0.5, Vector3(100.0, 0.0, 0.0), basis)
    assert abs(trailing.position.x - 100.0) < 1e-9

    # Two runs on the same frame times are the same run, to the last bit, which
    # is the whole reason the clock is an argument rather than time.time().
    def flown():
        run = Simulation()
        for frame in range(300):
            run.fly(SIM_TIME_STEP_S,
                    collective=1.0 if frame % 120 < 30 else 0.0,
                    long_stick=1.0 if frame % 120 > 90 else 0.0)
        return run.airframe.state.values()

    assert flown() == flown()

    # The physics is airframe's own and its self test is the one that vouches for
    # it; everything above is only what this module adds around it.


def _demo():
    """Print what the frame loop does: the clock, the hands, the ground, the world.

    The flying on show is :mod:`airframe`'s - a trimmed hover, and a failure to
    pick one up - and what is new here is the loop around it: how frames become
    steps, what a long frame is worth, what a keyboard does to the axes, what the
    ground floor does to a parked aircraft and to one that flew into it, and
    where the aircraft ends up in the renderer's world.
    """
    print("the report's two configurations")
    for name in ("simulation", "flight test"):
        preset = airframe_preset(name)
        print("  %-12s %6.0f lb, R6 %.3f s, Ixx %.0f kg m^2"
              % (name, preset.mass / POUND, preset.rotor_time_constant,
                 preset.inertia[0]))

    print()
    print("a fresh simulation is a trimmed hover, and holds it through the loop")
    sim = Simulation()
    print("  trim: %s" % (sim.controls,))
    print("  " + str(sim.telemetry()))
    for _ in range(int(5.0 * 60)):
        sim.step(SIM_TIME_STEP_S)
    state = sim.airframe.state
    print("  after 5 s of frames: alt %.4f m (%.2f mm off), drift %.2f mm,"
          " %.3f mm/s"
          % (state.altitude, 1000.0 * (state.altitude - 200.0),
             1000.0 * math.hypot(state.position.x, state.position.y),
             1000.0 * state.speed))
    print("  %d frames -> %d steps of %.4f s, %d dropped, clock %.4f s"
          % (sim.frames, sim.steps, sim.dt, sim.dropped_steps, sim.sim_time))

    print()
    print("60 kt level flight through the same loop, on the report's 6158 lb")
    print("instrumented aircraft: the condition its figures 2 to 9 are step")
    print("responses from, and the one a regression flies its steps out of")
    flying = Simulation(airframe=airframe_preset("flight test"))
    flying.trim_level_flight(60.0 * KNOT)
    print("  trim: %s" % (flying.controls,))
    print("  %s" % flying.airframe.state)
    print("  %s" % flying.telemetry())
    for frame in range(1, int(4.0 * 60) + 1):
        flying.step(SIM_TIME_STEP_S)
        if frame % 60:
            continue
        telemetry = flying.telemetry()
        print("  %4.1f s  %6.2f kt | %+7.0f fpm | %7.2f m north | coll %5.2f in |"
              " pitch %+5.2f deg | %s%s"
              % (frame / 60.0, telemetry.airspeed_kt, telemetry.height_rate_fpm,
                 flying.airframe.state.position.x, telemetry.collective_in,
                 telemetry.pitch_deg,
                 "trim" if telemetry.in_trim else "off the trim",
                 " CRASHED" if telemetry.crashed else ""))
    print("  %.4f s of frames put it %.4f m north, where 60 kt for that long is"
          " %.4f m," % (flying.sim_time, flying.airframe.state.position.x,
                        flying.sim_time * 60.0 * KNOT))

    print()
    print("the clock: whatever the frame rate, the physics is 1/60 s of aircraft")
    ragged = Simulation()
    for _ in range(300):
        ragged.step(0.0167)
    print("  300 frames of 16.7 ms: %.4f s of physics in %d steps, %d dropped,"
          " %.6f s carried" % (ragged.sim_time, ragged.steps,
                               ragged.dropped_steps, ragged.frame_carry))
    hitch = Simulation()
    hitch.step(0.5)
    print("  one frame of 500 ms:   %.4f s of physics in %d steps, %d dropped,"
          " %.6f s carried" % (hitch.sim_time, hitch.steps,
                               hitch.dropped_steps, hitch.frame_carry))

    print()
    print("the keyboard: rates, spring centring, and a collective that ratchets")
    hands = PilotInput()
    print("  hold forward stick 0.25 s:    %s"
          % (hands.step(0.25, long_stick=1.0),))
    print("  released for 0.25 s:          %s" % (hands.step(0.25),))
    print("  collective up for 0.5 s:      %s"
          % (hands.step(0.5, collective=1.0),))
    print("  collective released 0.5 s:    %s" % (hands.step(0.5),))
    print("  axes after all that:          %s" % (hands,))

    print()
    print("the ground: parked on the pad, then lifted off with the keyboard")
    pad = Simulation()
    pad.on_the_pad()
    print("  parked: " + str(pad.telemetry()))
    pad.pilot.reset(pad.trim_controls)
    pad.pilot.set_axes(collective=0.0)
    trim_fraction = pad.trim_controls.collective_fraction
    for frame in range(1, int(8.0 * 60) + 1):
        pad.fly(SIM_TIME_STEP_S,
                collective=1.0 if pad.pilot.collective_axis < trim_fraction
                else 0.0)
        if frame % 60:
            continue
        telemetry = pad.telemetry()
        print("  %4.1f s  coll %5.2f in | alt %6.2f m | %+7.0f fpm | %s%s"
              % (frame / 60.0, telemetry.collective_in, telemetry.alt_agl,
                 telemetry.height_rate_fpm,
                 "on the ground" if pad.on_ground else "airborne",
                 " CRASHED" if pad.crashed else ""))
    print("  the collective ratcheted to the trim and stopped there, and the skids")
    print("  let go as soon as the rotor was lifting more than the aircraft weighs.")
    print("  What follows is the model's own lightly damped heave: with the")
    print("  collective a hair above the trim, the ground cushion's kick at the")
    print("  break away is enough to put it back onto the skids a few seconds later.")

    print()
    print("a failed pickup: the collective to the down stop, three metres up")
    failed = Simulation()
    failed.reset(
        altitude=UH1_CG_HEIGHT_ON_GROUND + 3.0,
        controls=PilotControls(long_stick=failed.trim_controls.long_stick,
                               lat_stick=failed.trim_controls.lat_stick,
                               pedal=failed.trim_controls.pedal))
    for frame in range(1, int(1.0 * 60) + 1):
        failed.step(SIM_TIME_STEP_S)
        if frame % 6:
            continue
        state = failed.airframe.state
        print("  %4.2f s  alt %6.2f m | %+7.1f m/s | %s"
              % (frame / 60.0, state.altitude,
                 0.0 - state.ground_velocity().z,
                 "CRASHED" if failed.crashed else "falling"))
    print("  " + str(failed.telemetry()))
    print("  the skid line is %.3f m, so the floor held it and took the arrival"
          " out of the state; anything over %.1f m/s down is a crash here"
          % (UH1_CG_HEIGHT_ON_GROUND, UH1_HARD_LANDING_MPS))

    print()
    print("the same pickup from 30 m, which the model itself cannot fly: at 100 %")
    print("rotor with the collective at the down stop its rotor pushes down by")
    print("nearly two weights, and there is no engine or rotor speed dynamics to")
    print("fall back on.  The frame whose state leaves the envelope is refused")
    print("rather than passed on, so nothing downstream ever sees a NaN.")
    lost = Simulation()
    lost.reset(altitude=30.0,
               controls=PilotControls(long_stick=lost.trim_controls.long_stick,
                                      lat_stick=lost.trim_controls.lat_stick,
                                      pedal=lost.trim_controls.pedal))
    for _ in range(int(4.0 * 60)):
        lost.step(SIM_TIME_STEP_S)
        if lost.crashed:
            break
    print("  refused at %.2f s: alt %6.2f m, %+7.0f fpm, crashed %s, thrust"
          " %.0f N" % (lost.sim_time, lost.airframe.state.altitude,
                       lost.telemetry().height_rate_fpm, lost.crashed,
                       lost.telemetry().thrust))

    print()
    print("the renderer's world: (east, up, -north), and a camera to look at it")
    sim.reset(heading_deg=90.0)
    position = sim.render_position()
    nose, starboard, down_axis = sim.render_basis()
    print("  at (%.1f, %.1f, %.1f) m with the nose at (%.3f, %.3f, %.3f), the"
          " starboard wing at (%.3f, %.3f, %.3f)"
          % (position.x, position.y, position.z, nose.x, nose.y, nose.z,
             starboard.x, starboard.y, starboard.z))
    matrix = sim.render_matrix()
    print("  glMultMatrixf(%s)"
          % (", ".join("%.3f" % value for value in matrix),))
    camera = ChaseCamera()
    camera.update(0.0, position, sim.render_basis())
    eye, target, up = camera.eye_target_up()
    print("  gluLookAt(%.1f, %.1f, %.1f,  %.1f, %.1f, %.1f,  %.0f, %.0f, %.0f)"
          % (eye.x, eye.y, eye.z, target.x, target.y, target.z, up.x, up.y,
             up.z))
    print("  the eye is %.0f m behind and %.1f m above the smoothed aircraft,"
          " and the up vector is the world's own, so the horizon stays level"
          % (camera.distance, camera.height))

    print()
    _self_test()
    print("self test passed")


if __name__ == "__main__":
    _demo()
