"""The control path: from the pilot's hands and feet to the rotor.

:mod:`aerodynamics` answers "what does the air do to this rotor" given a
collective pitch and a flow field.  This module answers the question in front
of that: what pitch *is there* when the pilot moves the stick, and what does
the rotor then do with it.  Three stages, in the order the signal travels.

**1. Mixing.**  A pilot control position in inches becomes a control angle in
degrees through the linkage constants of TM-73254 table 3 (c1 to c6), over the
travels of table 2::

    A1c = c1 * long_stick      commanded longitudinal tilt,  12.9 in throw
    B1c = c4 * lat_stick       commanded lateral tilt,       12.6 in throw
    theta0 = c5 * collective   root collective pitch,        11.0 in throw
    thetaTR = c6 * pedal       tail rotor collective,         6.9 in throw

Two cross checks make those numbers vouch for each other.  The full
longitudinal throw at c1 sweeps 23.95 deg of cyclic pitch and table 2 gives the
longitudinal cyclic range as +12 to -11 deg; the lateral throw at c4 sweeps
20.0 deg against a lateral range of +9 to -11 deg; the pedals at c6 sweep
28.1 deg against a tail rotor collective range of +18 to -10 deg.  All three
agree to within a degree, which is the strongest evidence available for a set
of gearing constants that only exist as OCR damage and microfilm.

The table also carries the rigging: the lateral cyclic is rigged 2 deg left,
and the two cyclic ranges are not centred on zero, because that is what
rigging means.  Both are in :class:`MixingLaw` as biases.

A note on the source material, because it matters for anything built on top of
this: the text layer of TM-73254 is badly damaged - its tables come back as
runs of numbers with the rows shuffled, and its equations lose their operators
altogether - but the scanned page images under ``reference/fm_pages``, and the
crops already taken from them into ``_imgs``, are perfectly readable.  Tables 2
and 3 above were read off those images rather than off the OCR, which is why
three independent pairs of numbers could be checked against each other.  The
TM's equations (its stabilizer bar transfer function, its ground effect factor
K_G, its flapping coefficient relations) are still not recovered, and anything
that depends on them is marked as uncertain where it is used.

**2. Blade pitch.**  The angles above are the control *axis*: the direction the
swashplate commands the thrust to lean.  The blade pitch has to lead it by a
quarter turn of azimuth, because a blade flaps about a quarter of a turn after
the pitch that lifts it and the thrust leans towards the side the blades are
flapped *down* on.  With the rotor turning counter clockwise seen from above,
and with this frame's +y pointing *port* (see the note below), a commanded tilt
of

    v = (A1c, B1c)      forward tilt, starboard tilt

produces the blade pitch distribution

    theta(psi) = theta0 + twist * r / R + B1c * cos(psi) + A1c * sin(psi)

so the pitch is maximum over the nose when the command is a starboard tilt, and
maximum to port when it is a forward one: the blade that is pitched up over the
nose, a quarter of a turn before it gets to port, is the one whose lift pushes
the disc over to starboard.  That quarter turn is the whole reason a
helicopter's control rigging is not a straight pushrod, and
:meth:`RotorControlModel.respond` checks it end to end by pushing the stick
forward and watching which way the thrust points.

A note on the two y axes, because it is the one place where a sign can hide.
This module is in the rotor frame of :mod:`aerodynamics`: +x nose, +z up the
shaft, and therefore +y *port*, since forward cross port is up and the frame is
right handed.  A right handed frame with the nose on +x and up on +z cannot
have +y to starboard.  The claim checks out against the rotor itself: with the
UH-1 turning counter clockwise seen from above, the advancing blade in forward
flight is on the starboard side, which is -y here.  :mod:`airframe` works in the
report's own axes - x nose, y starboard, z down - where the same physical tilt
is a positive y component, so the two frames differ by a sign on y and on z.

**3. Flapping and induction.**  :func:`solve_flapping` solves for the cone angle
and the once per revolution tilt the blades actually take, by balancing the
aerodynamic hinge moment against the centrifugal spring (see that function for
the equation).  Its answer is what makes the rotor steer: the thrust of every
section acts perpendicular to its own flapped blade, so the summed force tilts
with the tip path plane.  The magnitude of that tilt follows the commanded tilt
one for one, which is the classic result for a teetering rotor and the reason a
UH-1H needs no hub moment to fly.  The induction is solved alongside it with
momentum theory (:func:`aerodynamics.solve_inflow`), because the inflow sets
the angle of attack that both the thrust and the flapping come from.

The lags are in :class:`ControlLags`: the collective lags by 0.20 sec, the
control axis by the rotor time constant 16 / (gamma * omega), and the Bell
stabilizer bar rides along in parallel.  At a trim point all of them settle out
and the three stages above are what is left, which is why they can be checked
separately from the dynamics.

Only the standard library is used, and everything here runs without pygame,
OpenGL or numpy, so the flight model can be tested headless.

Sign conventions, all in the rotor frame of :mod:`aerodynamics` (+x nose, +y
port, +z up the shaft), all of them the ones a pilot's hands and feet would
expect:

    + collective    more thrust
    + long_stick    forward, so the thrust leans forward and the nose goes down
    + lat_stick     right, so the thrust leans starboard and it rolls right
    + pedal         right, so the tail rotor collective comes *down* and the
                    nose goes right: the tail rotor's thrust holds the nose
                    left, so taking that thrust away is what turns it right
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

from aerodynamics import (Flapping, GRAVITY, Rotor, RotorLoads,
                          UH1_BAR_NUMERATOR_S, UH1_BAR_TIME_CONSTANT_S,
                          UH1_COLLECTIVE_PER_IN, UH1_COLLECTIVE_TIME_CONSTANT_S,
                          UH1_COLLECTIVE_TRAVEL_IN, UH1_LAT_CYCLIC_PER_IN,
                          UH1_LAT_STICK_TRAVEL_IN, UH1_LONG_CYCLIC_PER_IN,
                          UH1_LONG_STICK_TRAVEL_IN, UH1_MAX_GROSS_MASS,
                          UH1_PEDAL_TRAVEL_IN, UH1_RIGGING_PHASE_SIM_DEG,
                          UH1_ROTOR_TIME_CONSTANT_S, UH1_TAIL_ARM,
                          UH1_TAIL_PEDAL_PER_IN, UH1_WEIGHT_N,
                          collective_for_thrust, hover_loads, solve_inflow,
                          uniform_airflow, Vector3)

#: Tail rotor collective pitch per inch of *right* pedal, rad/in.  This is c6
#: of TM-73254 table 3, and it is negative: a pedal taken as positive to the
#: right takes tail rotor collective *away*.  That is table 2's sense as well,
#: where full right pedal gives -10 deg of tail rotor collective and full left
#: pedal +18 deg.  The reason is the one a pilot would give: the tail rotor
#: pushes the tail to starboard and so holds the nose to the left, and taking
#: that thrust away is what turns the nose right.
TAIL_PER_PEDAL_IN = UH1_TAIL_PEDAL_PER_IN            # rad/in, -0.071

#: Tail rotor collective pitch range at the pedal stops (TM-73254 table 2), and
#: the neutral that sits between them.  The spread, 18 - (-10) = 28 deg, is
#: also what 6.9 in of pedal travel at 0.071 rad/in gives (28.1 deg), so the two
#: tables pin each other; the neutral is their midpoint, not zero.
UH1_TAIL_COLLECTIVE_LEFT_DEG = 18.0     # deg, tail collective at full left pedal
UH1_TAIL_COLLECTIVE_RIGHT_DEG = -10.0   # deg, tail collective at full right pedal
UH1_TAIL_NEUTRAL_DEG = 0.5 * (UH1_TAIL_COLLECTIVE_LEFT_DEG
                              + UH1_TAIL_COLLECTIVE_RIGHT_DEG)   # 4 deg

#: Main rotor cyclic pitch ranges over the full stick throw (TM-73254 table 2),
#: which are not centred on zero because of the rigging.  Their midpoints are
#: the longitudinal and lateral rigging biases, and the lateral one agrees with
#: the table's own "lateral cyclic rigging 2 deg left" - carried in
#: :mod:`aerodynamics` as UH1_LATERAL_RIGGING_DEG - to within a degree.  The
#: range midpoints are what the model uses, because they are the same kind of
#: number as the stick travel they sit on; set ``lat_rigging_deg`` to -2.0 for
#: the rigging line itself.
UH1_LONG_CYCLIC_RANGE_DEG = (12.0, -11.0)   # deg, forward stop .. aft stop
UH1_LAT_CYCLIC_RANGE_DEG = (9.0, -11.0)     # deg, right stop .. left stop
UH1_LONG_RIGGING_DEG = 0.5 * sum(UH1_LONG_CYCLIC_RANGE_DEG)   # 0.5 deg
UH1_LAT_RIGGING_DEG = 0.5 * sum(UH1_LAT_CYCLIC_RANGE_DEG)     # -1 deg

#: Collective stick travel, 0 at the down stop to 11 in at the up stop.
#: TM-73254 table 2 gives the travel but not the pitch range, so the model takes
#: the down stop as zero root collective pitch, which is what puts a maximum
#: weight hover at 97 per cent of the stick travel.  Override
#: ``collective_neutral_deg`` for a different rigging.
UH1_COLLECTIVE_RANGE_DEG = (0.0, math.degrees(UH1_COLLECTIVE_PER_IN
                                             * UH1_COLLECTIVE_TRAVEL_IN))


@dataclass(frozen=True)
class PilotControls:
    """Where the pilot's hands and feet are, in inches of control travel.

    Collective runs 0 at the down stop to 11 in at the up stop.  The sticks and
    the pedals are centred at zero and are positive forward, right and right
    respectively, so their stops are at half their full travel.
    """

    collective: float = 0.0     # in, 0 down stop .. 11 up stop
    long_stick: float = 0.0     # in, + is forward
    lat_stick: float = 0.0      # in, + is right
    pedal: float = 0.0          # in, + is right pedal

    @classmethod
    def from_axes(cls, collective=0.0, long_stick=0.0, lat_stick=0.0,
                  pedal=0.0):
        """Build from normalised axes, the shape a joystick or keyboard gives.

        ``collective`` is 0 at the down stop and 1 at full up; the sticks and
        the pedals are -1 to +1 with the same sense as the fields above.  This
        is the call the input code will make, so it is kept next to the inches
        it produces rather than inside the renderer.
        """
        def clip(value, low, high):
            return max(low, min(high, float(value)))

        return cls(
            collective=UH1_COLLECTIVE_TRAVEL_IN * clip(collective, 0.0, 1.0),
            long_stick=(0.5 * UH1_LONG_STICK_TRAVEL_IN
                        * clip(long_stick, -1.0, 1.0)),
            lat_stick=(0.5 * UH1_LAT_STICK_TRAVEL_IN
                       * clip(lat_stick, -1.0, 1.0)),
            pedal=0.5 * UH1_PEDAL_TRAVEL_IN * clip(pedal, -1.0, 1.0))

    @property
    def collective_fraction(self):
        """Collective stick position as a fraction of its travel, 0 to 1."""
        return self.collective / UH1_COLLECTIVE_TRAVEL_IN

    def clipped(self):
        """A copy limited to the travels of TM-73254 table 2.

        A pilot cannot push a stick past its stop, so every path into this
        module ends up here.  The travel in the table is the full throw, so the
        sticks stop at half of it either side of centre.
        """
        return PilotControls(
            collective=max(0.0, min(UH1_COLLECTIVE_TRAVEL_IN, self.collective)),
            long_stick=max(-0.5 * UH1_LONG_STICK_TRAVEL_IN,
                           min(0.5 * UH1_LONG_STICK_TRAVEL_IN,
                               self.long_stick)),
            lat_stick=max(-0.5 * UH1_LAT_STICK_TRAVEL_IN,
                          min(0.5 * UH1_LAT_STICK_TRAVEL_IN, self.lat_stick)),
            pedal=max(-0.5 * UH1_PEDAL_TRAVEL_IN,
                      min(0.5 * UH1_PEDAL_TRAVEL_IN, self.pedal)))

    def at_stop(self):
        """Names of the controls that are sitting on a stop."""
        clipped = self.clipped()
        stops = []
        if clipped.collective != self.collective:
            stops.append("collective")
        if clipped.long_stick != self.long_stick:
            stops.append("long stick")
        if clipped.lat_stick != self.lat_stick:
            stops.append("lateral stick")
        if clipped.pedal != self.pedal:
            stops.append("pedals")
        return tuple(stops)

    def __str__(self):
        return ("collective %5.2f in (%4.1f %%) | long %+5.2f in | lat %+5.2f"
                " in | pedal %+5.2f in"
                % (self.collective, 100.0 * self.collective_fraction,
                   self.long_stick, self.lat_stick, self.pedal))


@dataclass(frozen=True)
class ControlAngles:
    """The control axis the swashplate commands, and the pitch it makes.

    ``control_long_deg`` and ``control_lat_deg`` are the commanded tilt of the
    thrust, forward and starboard, in the TM-73254 naming that calls them A1c
    and B1c.  ``cyclic_long_deg`` and ``cyclic_lat_deg`` are the blade pitch
    coefficients that lean it there, the first on cos(psi) and the second on
    sin(psi).  ``collective_pitch_deg`` is the root collective pitch the main
    rotor sees and ``tail_collective_deg`` the same for the tail rotor.
    """

    collective_pitch_deg: float
    control_long_deg: float
    control_lat_deg: float
    tail_collective_deg: float
    cyclic_long_deg: float
    cyclic_lat_deg: float
    stopped: Tuple[str, ...] = ()

    @property
    def control_tilt_deg(self):
        """Magnitude of the commanded tip path plane tilt, deg."""
        return math.hypot(self.control_long_deg, self.control_lat_deg)

    @property
    def control_tilt_azimuth_deg(self):
        """Azimuth, deg, the commanded tilt leans the thrust towards.

        Measured in the rotor frame of :mod:`aerodynamics`, from +x (the nose)
        towards +y, which is *port*, so a starboard command comes out at a
        negative azimuth - the same convention the flapping solution reports its
        own tilt in.
        """
        return math.degrees(math.atan2(-self.control_lat_deg,
                                       self.control_long_deg))

    @property
    def blade_pitch_amps_deg(self):
        """Magnitude of the cyclic blade pitch, deg."""
        return math.hypot(self.cyclic_long_deg, self.cyclic_lat_deg)

    def as_flapping_command(self):
        """The tilt the pilot asked for, as a :class:`~aerodynamics.Flapping`.

        The tip path plane of a teetering rotor settles at the commanded tilt,
        one for one, so this is where the flapping solution is expected to land
        and the self test checks it there.  The flapping is written
        ``beta = a0 - a1 cos psi - b1 sin psi`` with the thrust leaning towards
        the azimuth ``atan2(b1, a1)``, so the forward tilt is ``a1`` and the
        starboard one is ``-b1``: a starboard command leans the thrust towards
        -y here.
        """
        return Flapping(0.0, math.radians(self.control_long_deg),
                        -math.radians(self.control_lat_deg))

    def __str__(self):
        return ("collective %6.2f deg | control tilt %5.2f deg at %6.1f deg"
                " | blade cyclic %5.2f deg (cos %+5.2f, sin %+5.2f)"
                " | tail %+6.2f deg"
                % (self.collective_pitch_deg, self.control_tilt_deg,
                   self.control_tilt_azimuth_deg, self.blade_pitch_amps_deg,
                   self.cyclic_long_deg, self.cyclic_lat_deg,
                   self.tail_collective_deg))


@dataclass
class MixingLaw:
    """The linkage from pilot positions in inches to rotor pitch in degrees.

    TM-73254 table 2 gives the travels and the cyclic pitch ranges, table 3 the
    linkage constants, and page 5 the equations: the simple form

        A1c = c1 * long_stick      B1c = c4 * lat_stick
        theta0 = c5 * collective   thetaTR = c6 * pedal

    and the general form that folds in the rigging phase phi_p::

        A1cp = c1 * de * cos(phi_p) - c4 * dl * sin(phi_p)
        B1cp = c1 * de * sin(phi_p) + c4 * dl * cos(phi_p)

    which is the commanded control axis rotated by phi_p, 5 deg on the aircraft
    and zero in the NASA simulation, which is the default here.  The cross
    coupling the general form exists for is exactly what a 5 deg rigging phase
    is: a little of the longitudinal stick appearing in the lateral tilt.

    The blade pitch coefficients come out of the control axis rotated a quarter
    turn, as the module docstring explains, and the stops of table 2 are
    applied on the way in, so nothing downstream has to know they exist.
    """

    long_cyclic_per_in: float = UH1_LONG_CYCLIC_PER_IN   # rad/in, c1
    lat_cyclic_per_in: float = UH1_LAT_CYCLIC_PER_IN     # rad/in, c4
    collective_per_in: float = UH1_COLLECTIVE_PER_IN     # rad/in, c5
    tail_per_pedal_in: float = TAIL_PER_PEDAL_IN         # rad/in, c6
    rigging_phase_deg: float = UH1_RIGGING_PHASE_SIM_DEG
    long_rigging_deg: float = UH1_LONG_RIGGING_DEG
    lat_rigging_deg: float = UH1_LAT_RIGGING_DEG
    collective_neutral_deg: float = UH1_COLLECTIVE_RANGE_DEG[0]
    tail_neutral_deg: float = UH1_TAIL_NEUTRAL_DEG

    def control_axis_deg(self, controls):
        """Commanded tilt ``(forward, starboard)``, deg, from the stick inches.

        This is the TM-73254 form of the mixing, in the sense of the *effect* on
        the aircraft rather than the sense of a pushrod, which is what makes it
        worth keeping separate from the blade pitch below.
        """
        controls = controls.clipped()
        long_rad = self.long_cyclic_per_in * controls.long_stick
        lat_rad = self.lat_cyclic_per_in * controls.lat_stick
        phase = math.radians(self.rigging_phase_deg)
        long_rotated = long_rad * math.cos(phase) - lat_rad * math.sin(phase)
        lat_rotated = long_rad * math.sin(phase) + lat_rad * math.cos(phase)
        return (math.degrees(long_rotated) + self.long_rigging_deg,
                math.degrees(lat_rotated) + self.lat_rigging_deg)

    def blade_pitch_deg(self, controls):
        """Cyclic blade pitch ``(on cos psi, on sin psi)``, deg, for the sticks.

        The quarter turn between the control axis the pilot commands and the
        blade pitch distribution that leans the thrust there, in one place.

        A blade reaches its flap about a quarter of a turn of azimuth *after*
        the pitch that lifts it, and the thrust leans towards the side where the
        blades are flapped *down*, so the pitch has to be maximum a quarter of a
        turn of azimuth *after* the direction the thrust is to lean in - which
        makes the two cyclic coefficients below the two control axis components
        read the other way round, with a sign that depends on which way this
        frame's y axis points.

        To lean the thrust to starboard (azimuth -y here) the pitch is maximum
        over the nose, and to lean it forward (azimuth 0, the way the nose
        points) the pitch is maximum to port (azimuth +y, because that is
        forward the way the rotor turns, and this frame's +y is port - see the
        module docstring).  So the coefficient on cos(psi), which is the pitch
        over the nose, carries the *starboard* command, and the coefficient on
        sin(psi), which is the pitch to port, carries the *forward* one.
        """
        control_long, control_lat = self.control_axis_deg(controls)
        return (control_lat, control_long)

    def mix(self, controls):
        """The whole mixing stage: inches in, :class:`ControlAngles` out."""
        controls = controls.clipped()
        control_long, control_lat = self.control_axis_deg(controls)
        cyclic_long, cyclic_lat = self.blade_pitch_deg(controls)
        return ControlAngles(
            collective_pitch_deg=(self.collective_neutral_deg
                                  + math.degrees(self.collective_per_in
                                                 * controls.collective)),
            control_long_deg=control_long, control_lat_deg=control_lat,
            tail_collective_deg=(self.tail_neutral_deg
                                 + math.degrees(self.tail_per_pedal_in
                                                * controls.pedal)),
            cyclic_long_deg=cyclic_long, cyclic_lat_deg=cyclic_lat,
            stopped=controls.at_stop())


@dataclass
class ControlLags:
    """The first order lags between the pilot's controls and the rotor.

    Three of them, all first order, all from TM-73254:

    * the collective stick and the pedals reach their pitch through
      ``tau_C = 0.20 sec``, the lag the TM matched to the acceleration data of
      the flight tests (its "main and tail rotor collective pitch" section);
    * the control axis, the tilt the swashplate commands, follows the cyclic
      pitch with ``tau_R = 16 / (gamma * omega)``, the rotor time constant of
      table 3, 0.072 sec from the inertia of one blade.  The TM's simulation
      used the doubled 0.144 sec because pilots found the quicker rotor harder
      to fly; both values are in :mod:`aerodynamics` if you want the other one;
    * the Bell stabilizer bar feeds a lagged part of the cyclic pitch in
      parallel with the pilot: the linkage mixing ratio K_phi = 0.16 seen
      through the bar's damping time constant tau_B = 3.3 sec, whose product
      ``K_B = 0.528 sec`` is the numerator table 3 lists.

    The bar is **off by default**, and deliberately.  What table 3 pins is the
    pair of constants and the fact that K_B is a *numerator*, i.e. that a rate
    sits in it; the equation the two belong to is one of the parts of TM-73254
    its OCR destroyed.  The reading used here is the one a rate device takes:
    the bar rides the mast, answers the rate the control axis is actually
    moving at, and therefore leaves every steady state exactly where it was -
    which matters, because the linkage constants have to reproduce table 2's
    cyclic pitch ranges, and those ranges say the bar cannot be adding a fixed
    percentage of the pilot's stick at a trim point.  Turn ``bar_enabled`` on to
    see it work, and treat its magnitude as the least certain number in this
    module: ``bar_limit_deg`` bounds it at the few degrees of flapping travel a
    bar has, because the TM's own limit for it is not legible either.

    The integration is the exact first order step ``1 - exp(-dt / tau)`` rather
    than ``dt / tau``, so a large dt stays stable and a step in the controls is
    answered the way the continuous lag would answer it.
    """

    collective_tau: float = UH1_COLLECTIVE_TIME_CONSTANT_S
    cyclic_tau: float = UH1_ROTOR_TIME_CONSTANT_S
    bar_tau: float = UH1_BAR_TIME_CONSTANT_S
    bar_numerator: float = UH1_BAR_NUMERATOR_S
    bar_enabled: bool = False
    bar_limit_deg: float = 5.0

    # State: where the rotor actually is, between the command and the answer.
    collective_pitch_deg: float = 0.0
    cyclic_long_deg: float = 0.0
    cyclic_lat_deg: float = 0.0
    tail_collective_deg: float = 0.0
    bar_long_deg: float = 0.0
    bar_lat_deg: float = 0.0

    def reset(self, angles=None):
        """Jump the state straight to *angles* (or to a hover-like zero)."""
        if angles is None:
            self.collective_pitch_deg = 0.0
            self.cyclic_long_deg = 0.0
            self.cyclic_lat_deg = 0.0
            self.tail_collective_deg = 0.0
        else:
            self.collective_pitch_deg = angles.collective_pitch_deg
            self.cyclic_long_deg = angles.cyclic_long_deg
            self.cyclic_lat_deg = angles.cyclic_lat_deg
            self.tail_collective_deg = angles.tail_collective_deg
        self.bar_long_deg = 0.0
        self.bar_lat_deg = 0.0
        return self

    @staticmethod
    def _first_order(state, target, dt, tau):
        """Exact first order step of *state* toward *target*."""
        if tau <= 0.0:
            return target
        weight = 1.0 - math.exp(-max(dt, 0.0) / tau)
        return state + (target - state) * weight

    def step(self, dt, angles):
        """Advance the lags by *dt* seconds toward *angles*.

        Returns the :class:`ControlAngles` the rotor sees, which is the command
        after the lags and after whatever the stabilizer bar adds to the cyclic.
        """
        previous_long = self.cyclic_long_deg
        previous_lat = self.cyclic_lat_deg
        cyclic_long = self._first_order(previous_long, angles.cyclic_long_deg,
                                        dt, self.cyclic_tau)
        cyclic_lat = self._first_order(previous_lat, angles.cyclic_lat_deg, dt,
                                       self.cyclic_tau)
        if self.bar_enabled and dt > 0.0:
            # The bar rides the mast and answers the rate the control axis is
            # actually moving at, which is the rate after the rotor lag and not
            # the rate of the pilot's command.
            rate_long = (cyclic_long - previous_long) / dt
            rate_lat = (cyclic_lat - previous_lat) / dt
            limit = abs(self.bar_limit_deg)
            self.bar_long_deg = max(-limit, min(limit, self._first_order(
                self.bar_long_deg, self.bar_numerator * rate_long, dt,
                self.bar_tau)))
            self.bar_lat_deg = max(-limit, min(limit, self._first_order(
                self.bar_lat_deg, self.bar_numerator * rate_lat, dt,
                self.bar_tau)))

        self.cyclic_long_deg = cyclic_long
        self.cyclic_lat_deg = cyclic_lat
        self.collective_pitch_deg = self._first_order(
            self.collective_pitch_deg, angles.collective_pitch_deg, dt,
            self.collective_tau)
        self.tail_collective_deg = self._first_order(
            self.tail_collective_deg, angles.tail_collective_deg, dt,
            self.collective_tau)

        return ControlAngles(
            collective_pitch_deg=self.collective_pitch_deg,
            control_long_deg=angles.control_long_deg,
            control_lat_deg=angles.control_lat_deg,
            tail_collective_deg=self.tail_collective_deg,
            cyclic_long_deg=self.cyclic_long_deg - self.bar_long_deg,
            cyclic_lat_deg=self.cyclic_lat_deg - self.bar_lat_deg,
            stopped=angles.stopped)


def solve_linear(matrix, vector):
    """Solve a small dense linear system by Gaussian elimination.

    Called with a 3 by 3 from the Newton step of the flapping solve, and by
    :mod:`airframe` with a 4 by 4 for its hover trim, so it is written for any
    size.  Returns None if the system is singular, which the caller treats as
    "do not move this cycle" rather than dividing by a zero pivot.
    """
    size = len(vector)
    rows = [list(matrix[i]) + [vector[i]] for i in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda r: abs(rows[r][column]))
        if abs(rows[pivot][column]) < 1e-14:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        for row in range(column + 1, size):
            factor = rows[row][column] / rows[column][column]
            for k in range(column, size + 1):
                rows[row][k] -= factor * rows[column][k]
    answer = [0.0] * size
    for row in range(size - 1, -1, -1):
        total = rows[row][size] - sum(rows[row][k] * answer[k]
                                      for k in range(row + 1, size))
        answer[row] = total / rows[row][row]
    return answer


@dataclass
class FlappingSolution:
    """What the flap solve found, and how well it settled."""

    flapping: Flapping
    cycles: int
    error: float                # rad, the largest harmonic residual
    converged: bool
    thrust: float = 0.0         # N, at the solution, if it was evaluated

    @property
    def coning_deg(self):
        return self.flapping.coning_deg

    @property
    def long_deg(self):
        return self.flapping.long_deg

    @property
    def lat_deg(self):
        return self.flapping.lat_deg

    @property
    def tilt_deg(self):
        return self.flapping.tilt_deg

    @property
    def tilt_azimuth_deg(self):
        return self.flapping.tilt_azimuth_deg

    def __str__(self):
        return ("%s | %d cycles, residual %.2e rad, %s"
                % (self.flapping, self.cycles, self.error,
                   "converged" if self.converged else "not converged"))


def solve_flapping(rotor, collective_pitch_deg, airflow, cyclic_long_deg=0.0,
                   cyclic_lat_deg=0.0, air_density=None, azimuth_steps=24,
                   iterations=8, tolerance=1e-6, start=None, relax=1.0):
    """Solve the flap angles a blade settles at, by harmonic balance.

    The flap equation of a blade on a hinge at radius e is

        I_b * beta_ddot + omega ** 2 * K_c * beta = M_aero

    with ``I_b`` the flap inertia of the blade about the hinge, ``K_c`` the
    centrifugal coefficient and ``M_aero`` the aerodynamic moment about the
    hinge, which the blade element model returns as
    :attr:`~aerodynamics.RotorLoads.flap_moment`.  Writing the flapping as a
    cone angle plus a first harmonic and dividing through by
    ``I_b * omega ** 2`` turns that into::

        beta(psi)       = a0 - a1 cos(psi) - b1 sin(psi)
        beta'' + k beta = M / (I_b omega ** 2),        k = K_c / I_b

    so the *mean* of the hinge moment sets the cone angle a0, and the cos(psi)
    and sin(psi) components of it set the tilt (a1, b1).  With a teetering
    rotor, whose hinge is on the shaft and whose k is therefore exactly 1, the
    tilt drops out of the left hand side entirely and the equations that fix it
    simply say "the once per rev hinge moment is zero".  That is well posed
    only because a blade's flap *rate* changes the flow it sees - a blade
    rising into the air above it loses lift - and that damping is what pins the
    tilt a quarter turn after the cyclic pitch driving it.  A rotor with a
    hinge offset has k above 1, the tilt stays on the left hand side, and the
    aero moment survives into the hub as a moment.

    The residual is smooth in the three angles, so a Newton step on a forward
    difference Jacobian is used, seeded with the last answer (``start``) so a
    trim sweep costs two or three passes per point.  Returns a
    :class:`FlappingSolution`; ``converged`` False alongside the last angles is
    better than raising, because a caller sweeping collective wants the whole
    curve, not an exception half way along it.
    """
    if rotor.blade_count < 1:
        raise ValueError("blade_count must be at least 1")
    if rotor.flap_inertia <= 0.0:
        raise ValueError("rotor has no flap inertia")
    omega = rotor.omega
    density = rotor.air_density if air_density is None else air_density
    scale = 1.0 / (rotor.flap_inertia * omega * omega)
    ratio = rotor.flap_centrifugal_ratio()
    steps = max(int(azimuth_steps), 4)
    azimuths = [360.0 * index / float(steps) for index in range(steps)]
    cosines = [math.cos(math.radians(az)) for az in azimuths]
    sines = [math.sin(math.radians(az)) for az in azimuths]

    def harmonics(state):
        """Mean and first harmonic coefficients of the *one blade* flap moment.

        Blade 0's strips only: the total in ``RotorLoads.flap_moment`` sums
        every blade, and two blades half a turn apart have once per rev
        moments that cancel exactly, which is precisely the part that sets the
        tilt.  The mean is the same either way, but the harmonics have to come
        from a single blade.
        """
        flap = Flapping(*state)
        mean = cosine = sine = 0.0
        for index, azimuth in enumerate(azimuths):
            loads = rotor.compute(airflow, collective_pitch_deg,
                                  azimuth_deg=azimuth, air_density=density,
                                  cyclic_long_deg=cyclic_long_deg,
                                  cyclic_lat_deg=cyclic_lat_deg,
                                  flapping=flap)
            moment = sum(sample.flap_moment for sample in loads.samples
                         if sample.blade_index == 0)
            mean += moment
            cosine += moment * cosines[index]
            sine += moment * sines[index]
        return (mean / steps, 2.0 * cosine / steps, 2.0 * sine / steps)

    def residual(state):
        a0, a1, b1 = state
        mean, cosine, sine = harmonics(state)
        return (scale * mean - ratio * a0,
                scale * cosine - (1.0 - ratio) * a1,
                scale * sine - (1.0 - ratio) * b1)

    if start is None:
        state = [math.radians(rotor.precone_deg), 0.0, 0.0]
    else:
        # Accept either a Flapping or a FlappingSolution, so a caller can hand
        # back whatever the last solve returned.
        seed = getattr(start, "flapping", start)
        state = [seed.coning, seed.long, seed.lat]

    cycles, error, converged = 0, float("inf"), False
    for cycle in range(1, max(int(iterations), 1) + 1):
        cycles = cycle
        values = residual(state)
        error = max(abs(value) for value in values)
        if error <= tolerance:
            converged = True
            break
        jacobian = [[0.0] * 3 for _ in range(3)]
        for column in range(3):
            step = [0.0, 0.0, 0.0]
            step[column] = 1e-4
            shifted = [state[i] + step[i] for i in range(3)]
            delta = residual(shifted)
            for row in range(3):
                jacobian[row][column] = ((delta[row] - values[row])
                                         / step[column])
        correction = solve_linear(jacobian, [-value for value in values])
        if correction is None:
            break
        state = [state[i] + relax * max(-0.15, min(0.15, correction[i]))
                 for i in range(3)]
    else:
        error = max(abs(value) for value in residual(state))

    return FlappingSolution(Flapping(*state), cycles, error, converged)


@dataclass
class RotorResponse:
    """What the rotor does in answer to a set of pilot controls."""

    controls: Optional[PilotControls]
    angles: ControlAngles
    inflow: float                   # m/s, induced velocity through the disc
    inflow_evaluations: int
    inflow_converged: bool
    flapping: FlappingSolution
    loads: RotorLoads               # averaged over one revolution
    tail_collective_deg: float
    tail_thrust: float              # N, starboard positive
    tail_inflow: float              # m/s
    azimuth_steps: int = 36

    @property
    def thrust(self):
        """Mean thrust along the shaft, N."""
        return self.loads.thrust

    @property
    def thrust_tilt_deg(self):
        """How far the mean force leans from the shaft, deg."""
        return math.degrees(math.atan2(self.loads.in_plane_force,
                                       self.loads.force.z))

    @property
    def thrust_tilt_azimuth_deg(self):
        """Which way the mean force leans, deg azimuth from the nose."""
        return math.degrees(math.atan2(self.loads.side_force,
                                       self.loads.h_force))

    @property
    def tail_yaw_moment(self):
        """Yawing moment the tail rotor makes about +z, N m.

        The tail rotor pushes starboard, which holds the nose to the left, so a
        positive tail thrust is a negative moment about the shaft-up axis.
        """
        return -self.tail_thrust * UH1_TAIL_ARM

    @property
    def tilt_error_deg(self):
        """Flapping tilt minus the commanded tilt, deg.

        A teetering rotor should settle with its tip path plane on the control
        axis the swashplate commands, so this is the model's own answer to "did
        the mixing and the flapping agree", and it is what the self test
        watches.
        """
        return self.flapping.tilt_deg - self.angles.control_tilt_deg

    @property
    def yaw_torque_balance(self):
        """Main rotor torque minus what the tail rotor balances, N m.

        Positive means more tail thrust than the torque needs, i.e. the nose is
        being pulled left.
        """
        return self.loads.shaft_torque + self.tail_yaw_moment

    def __str__(self):
        return ("%s\n    %s\n    flapping %s\n    inflow %5.2f m/s"
                " (%d evaluations%s) | %s\n    tail %+6.2f deg, %6.0f N,"
                " yaw %+8.0f N m, out of balance %+7.0f N m"
                % (self.controls, self.angles, self.flapping, self.inflow,
                   self.inflow_evaluations,
                   "" if self.inflow_converged else ", NOT converged",
                   self.loads, self.tail_collective_deg, self.tail_thrust,
                   self.tail_yaw_moment, self.yaw_torque_balance))


@dataclass
class RotorControlModel:
    """The whole control path for one helicopter, from inches to loads.

    Holds the rotor, the tail rotor and the mixing law, and does the three
    stages in order:

    1. mix the pilot's positions into control angles;
    2. solve the induced velocity the thrust it makes needs
       (:func:`aerodynamics.solve_inflow`);
    3. solve the flapping at that inflow (:func:`solve_flapping`) and, if
       ``passes`` is above zero, put the flapping back into the inflow and
       solve the flapping again, since a tilted disc makes a little more thrust
       than the level one the first pass assumed.

    No passes is enough for a hover, where the tilt is small; one pass is the
    reasonable default for a flight condition where the tilt is large.
    ``start`` carries the previous flapping in as the starting guess, which is
    what makes a sweep along a line of collective settings cheap.
    """

    rotor: Rotor = field(default_factory=Rotor.uh1h)
    law: MixingLaw = field(default_factory=MixingLaw)
    tail_rotor: Rotor = field(default_factory=Rotor.uh1h_tail)
    azimuth_steps: int = 24          # azimuths per flapping solve
    load_azimuth_steps: int = 36     # azimuths per revolution for the mean loads
    inflow_iterations: int = 30
    inflow_tolerance: float = 1e-3
    flapping_iterations: int = 8
    flapping_tolerance: float = 1e-6

    def angles(self, controls):
        """The mixing stage on its own, without any aerodynamics."""
        return self.law.mix(controls)

    def _flap(self, angles, airflow, density, start):
        """One flapping solve with this model's settings."""
        return solve_flapping(
            self.rotor, angles.collective_pitch_deg, airflow,
            angles.cyclic_long_deg, angles.cyclic_lat_deg, air_density=density,
            azimuth_steps=self.azimuth_steps, start=start,
            iterations=self.flapping_iterations,
            tolerance=self.flapping_tolerance)

    def respond(self, controls, forward_speed=0.0, climb_speed=0.0,
                air_density=None, start=None, passes=0, tail=True,
                angles=None):
        """Mix *controls* and run the whole path (see :meth:`respond_angles`).

        ``forward_speed`` is the airspeed along the nose and ``climb_speed`` the
        rate of climb, both m/s; the air the rotor sees is that free stream plus
        the induced velocity the solve finds.  ``start`` is the previous
        :class:`~aerodynamics.Flapping`, so passing it back in keeps a sweep to
        one or two passes per point.  ``angles`` overrides the mixing, which is
        how a caller that has already run the lags hands the rotor the pitch it
        actually has rather than the one the pilot asked for.
        """
        controls = controls.clipped()
        if angles is None:
            angles = self.law.mix(controls)
        return self.respond_angles(angles, controls,
                                   forward_speed=forward_speed,
                                   climb_speed=climb_speed,
                                   air_density=air_density, start=start,
                                   passes=passes, tail=tail)

    def respond_angles(self, angles, controls=None, forward_speed=0.0,
                       climb_speed=0.0, air_density=None, start=None,
                       passes=0, tail=True):
        """Run the aerodynamics for a set of *control angles*.

        This is the half of the path that a flight loop calls every frame: mix,
        run the lags (:class:`ControlLags`), then hand the lagged angles here.
        ``controls`` is only carried into the answer for the record, so it may
        be None.
        """
        density = self.rotor.air_density if air_density is None else air_density

        def airflow_for(inflow):
            return uniform_airflow(Vector3(-forward_speed, 0.0,
                                           -(climb_speed + inflow)))

        def thrust_at(value, flapping=None):
            return self.rotor.compute(
                airflow_for(value), angles.collective_pitch_deg,
                air_density=density, cyclic_long_deg=angles.cyclic_long_deg,
                cyclic_lat_deg=angles.cyclic_lat_deg, flapping=flapping).thrust

        inflow, evaluations, converged = solve_inflow(
            thrust_at, self.rotor.radius, self.rotor.omega,
            forward_speed=forward_speed, climb_speed=climb_speed,
            air_density=density, iterations=self.inflow_iterations,
            tolerance=self.inflow_tolerance)

        solution = self._flap(angles, airflow_for(inflow), density, start)

        for _ in range(max(int(passes), 0)):
            inflow, extra, ok = solve_inflow(
                lambda value: thrust_at(value, solution.flapping),
                self.rotor.radius, self.rotor.omega,
                forward_speed=forward_speed, climb_speed=climb_speed,
                air_density=density, inflow=inflow,
                iterations=self.inflow_iterations,
                tolerance=self.inflow_tolerance)
            evaluations += extra
            converged = converged and ok
            solution = self._flap(angles, airflow_for(inflow), density,
                                  solution.flapping)

        loads = self.rotor.mean_loads(
            airflow_for(inflow), angles.collective_pitch_deg,
            azimuth_steps=self.load_azimuth_steps, air_density=density,
            cyclic_long_deg=angles.cyclic_long_deg,
            cyclic_lat_deg=angles.cyclic_lat_deg, flapping=solution.flapping)
        solution.thrust = loads.thrust

        tail_inflow, tail_thrust, tail_collective = 0.0, 0.0, 0.0
        if tail:
            tail_collective = angles.tail_collective_deg
            tail_inflow, tail_loads = hover_loads(self.tail_rotor,
                                                  tail_collective, density)
            tail_thrust = tail_loads.thrust

        return RotorResponse(
            controls=controls, angles=angles, inflow=inflow,
            inflow_evaluations=evaluations, inflow_converged=converged,
            flapping=solution, loads=loads,
            tail_collective_deg=tail_collective, tail_thrust=tail_thrust,
            tail_inflow=tail_inflow, azimuth_steps=self.load_azimuth_steps)

    def trim_hover(self, mass=UH1_MAX_GROSS_MASS, air_density=None,
                   passes=0, **kwargs):
        """The stick positions that hold a steady hover at *mass*.

        Two things hold a hover and both are in inches: collective, so the main
        rotor thrust carries the weight, and pedals, so the tail rotor carries
        the main rotor torque.  The cyclic stays centred here, so this is the
        hover of those two things and not the whole picture: the 2 deg of left
        lateral rigging that biases a UH-1H's cyclic is a bias inside the mixing
        law rather than something the pilot holds, and what is left of the tail
        rotor's own push - better than a kilonewton of it, since the rigging
        leans the thrust to port but not nearly that far - is not trimmed here.
        :meth:`airframe.Airframe.trim_hover` does the whole job, lateral stick
        and attitudes included.

        The collective comes from a bisection on thrust and the pedal position
        from the torque the rotor actually makes at that collective, so both
        ends of the model have to agree for this to return anything sensible,
        which is what makes it a good test of the pair.
        """
        density = self.rotor.air_density if air_density is None else air_density
        pitch = collective_for_thrust(self.rotor, mass * GRAVITY, density)
        collective_in = ((pitch - self.law.collective_neutral_deg)
                         / math.degrees(self.law.collective_per_in))
        response = self.respond(PilotControls(collective=collective_in),
                                air_density=density, passes=passes, **kwargs)
        tail_pitch = collective_for_thrust(self.tail_rotor,
                                           response.loads.shaft_torque
                                           / UH1_TAIL_ARM, density)
        pedal_in = ((tail_pitch - self.law.tail_neutral_deg)
                    / math.degrees(self.law.tail_per_pedal_in))
        return PilotControls(collective=collective_in, pedal=pedal_in)


def _self_test():
    """Checks on the mixing, the blade pitch quarter turn and the flap solve.

    Raises AssertionError on failure.  The demo calls this, so running this
    module is enough to validate the whole control path.
    """
    law = MixingLaw()
    half_long = 0.5 * UH1_LONG_STICK_TRAVEL_IN
    half_lat = 0.5 * UH1_LAT_STICK_TRAVEL_IN
    half_pedal = 0.5 * UH1_PEDAL_TRAVEL_IN

    # The two tables have to agree.  Table 3's linkage constants, over table 2's
    # travels, have to reproduce table 2's own cyclic pitch ranges and the tail
    # rotor collective at each pedal stop; the rigging biases are the only
    # freedom, and they are the midpoints of those ranges.
    forward, _ = law.control_axis_deg(PilotControls(long_stick=half_long))
    aft, _ = law.control_axis_deg(PilotControls(long_stick=-half_long))
    assert abs(forward - UH1_LONG_CYCLIC_RANGE_DEG[0]) < 0.6, forward
    assert abs(aft - UH1_LONG_CYCLIC_RANGE_DEG[1]) < 0.6, aft
    _, right = law.control_axis_deg(PilotControls(lat_stick=half_lat))
    _, left = law.control_axis_deg(PilotControls(lat_stick=-half_lat))
    assert abs(right - UH1_LAT_CYCLIC_RANGE_DEG[0]) < 0.6, right
    assert abs(left - UH1_LAT_CYCLIC_RANGE_DEG[1]) < 0.6, left

    tail_left = law.mix(PilotControls(pedal=-half_pedal)).tail_collective_deg
    tail_right = law.mix(PilotControls(pedal=half_pedal)).tail_collective_deg
    assert abs(tail_left - UH1_TAIL_COLLECTIVE_LEFT_DEG) < 0.2, tail_left
    assert abs(tail_right - UH1_TAIL_COLLECTIVE_RIGHT_DEG) < 0.2, tail_right
    # Right pedal takes tail rotor collective *away*, which is the table 2
    # sense and the sense that turns the nose right.
    assert tail_right < tail_left

    # Stops, and the normalised axes a joystick hands over.
    clipped = PilotControls(collective=20.0, long_stick=9.0, pedal=-9.0).clipped()
    assert clipped.collective == UH1_COLLECTIVE_TRAVEL_IN
    assert clipped.long_stick == half_long
    assert clipped.pedal == -half_pedal
    assert PilotControls(collective=20.0).at_stop() == ("collective",)
    assert PilotControls(long_stick=1.0).at_stop() == ()
    axes = PilotControls.from_axes(0.5, 1.0, -1.0, -1.0)
    assert abs(axes.collective - 0.5 * UH1_COLLECTIVE_TRAVEL_IN) < 1e-9
    assert abs(axes.long_stick - half_long) < 1e-9
    assert abs(axes.lat_stick + half_lat) < 1e-9
    assert abs(axes.pedal + half_pedal) < 1e-9

    # The quarter turn between the commanded control axis and the blade pitch:
    # a starboard command is a pitch over the nose, a forward command a pitch to
    # port, both with the same magnitude.
    angles = law.mix(PilotControls(long_stick=half_long, lat_stick=half_lat))
    assert angles.control_long_deg > 0.0 and angles.control_lat_deg > 0.0
    assert angles.cyclic_long_deg > 0.0        # cos psi: the starboard command
    assert angles.cyclic_lat_deg > 0.0         # sin psi: the forward command
    assert abs(angles.cyclic_long_deg - angles.control_lat_deg) < 1e-9
    assert abs(angles.cyclic_lat_deg - angles.control_long_deg) < 1e-9
    assert abs(angles.blade_pitch_amps_deg - angles.control_tilt_deg) < 1e-9
    # The flapping command that goes with it is the tilt itself, with the
    # starboard part on -b1 because this frame's +y is port.
    command = angles.as_flapping_command()
    assert abs(command.long_deg - angles.control_long_deg) < 1e-9
    assert abs(command.lat_deg + angles.control_lat_deg) < 1e-9

    model = RotorControlModel()
    rotor = model.rotor
    assert abs(rotor.lock_number - 6.55) < 0.05, rotor.lock_number
    assert abs(rotor.flap_inertia - 1658.0) < 20.0, rotor.flap_inertia
    assert abs(rotor.flap_centrifugal_ratio() - 1.0) < 1e-12

    trim = model.trim_hover()
    response = model.respond(trim)
    assert response.flapping.converged, response.flapping
    assert abs(response.thrust - UH1_WEIGHT_N) < 0.05 * UH1_WEIGHT_N
    assert abs(response.yaw_torque_balance) < 0.05 * response.loads.shaft_torque
    # A maximum weight hover sits near the top of the collective travel, and the
    # cone angle lands in the couple of degrees the precone of 2.75 deg
    # suggests, which is the only independent check the flap solve has.
    assert 0.90 < trim.collective_fraction < 1.0, trim
    assert 1.5 < response.flapping.coning_deg < 6.0, response.flapping
    assert response.flapping.tilt_deg < 2.0, response.flapping

    # The conventions, end to end: push each control and see which way the
    # rotor force goes.  This is the check that catches a sign error anywhere
    # between the stick and the blade.
    def stick(**kwargs):
        kwargs.setdefault("collective", trim.collective)
        return model.respond(PilotControls(**kwargs), start=response.flapping)

    baseline = stick()
    forward = stick(long_stick=3.0)
    assert forward.loads.force.x > baseline.loads.force.x + 1000.0, forward.loads
    assert forward.flapping.long_deg > baseline.flapping.long_deg + 1.0
    assert (abs(forward.tilt_error_deg)
            < 0.1 * forward.angles.control_tilt_deg + 0.4), forward
    aft = stick(long_stick=-3.0)
    assert aft.loads.force.x < baseline.loads.force.x - 1000.0, aft.loads
    right = stick(lat_stick=3.0)
    # A right stick leans the thrust to starboard, which is -y in this frame, so
    # both the side force and the flapping's y component go *down*.
    assert right.loads.force.y < baseline.loads.force.y - 1000.0, right.loads
    assert right.flapping.lat_deg < baseline.flapping.lat_deg - 1.0
    left = stick(lat_stick=-3.0)
    assert left.loads.force.y > baseline.loads.force.y + 1000.0, left.loads

    higher = stick(collective=trim.collective + 0.5)
    assert higher.thrust > baseline.thrust
    assert higher.inflow > baseline.inflow

    # Right pedal means less tail rotor collective, less tail thrust, and a nose
    # right yaw moment; the tail rotor is what holds the nose left.
    right_pedal = model.respond(PilotControls(collective=trim.collective,
                                              pedal=half_pedal))
    assert right_pedal.tail_collective_deg < response.tail_collective_deg
    assert right_pedal.tail_thrust < response.tail_thrust
    assert right_pedal.yaw_torque_balance > response.yaw_torque_balance

    # With no flapping there is no tilt at all, which is the axisymmetric rotor
    # the loads started as, while the flap hinge moment that drives the solve is
    # still there.
    plain = rotor.compute(uniform_airflow(Vector3(0.0, 0.0, -response.inflow)),
                          response.angles.collective_pitch_deg)
    assert abs(plain.force.x) < 1e-6 and abs(plain.force.y) < 1e-6
    assert abs(plain.flap_moment) > 1.0

    # The lags: a command is reached after a few time constants, the cyclic
    # arrives before the collective, and the stabilizer bar is a transient that
    # goes away once the controls stop.
    lags = ControlLags().reset(response.angles)
    command = law.mix(PilotControls(collective=trim.collective,
                                    long_stick=half_long))
    first = lags.step(1.0 / 60.0, command)
    assert first.cyclic_lat_deg < command.cyclic_lat_deg
    assert abs(first.cyclic_lat_deg - response.angles.cyclic_lat_deg) > 0.1
    assert abs(first.collective_pitch_deg
               - response.angles.collective_pitch_deg) < 1e-9
    bar_after_one = abs(lags.bar_long_deg) + abs(lags.bar_lat_deg)
    assert bar_after_one == 0.0                # the bar is off by default
    settled = first
    for _ in range(15 * 60):
        settled = lags.step(1.0 / 60.0, command)
    assert abs(settled.cyclic_lat_deg - command.cyclic_lat_deg) < 1e-3
    assert abs(settled.collective_pitch_deg - command.collective_pitch_deg) < 1e-3
    assert abs(settled.tail_collective_deg - command.tail_collective_deg) < 1e-3
    assert abs(lags.bar_long_deg) + abs(lags.bar_lat_deg) < 1e-12
    # The cyclic lag really is the rotor time constant 16 / (gamma omega).
    assert abs(lags.cyclic_tau - rotor.rotor_time_constant) < 1e-12

    # With the stabilizer bar switched on, it answers the rate the control axis
    # moves at, stays inside its flapping travel, and decays again once the
    # controls stop - so it can change a transient but never a trim point.
    bar_lags = ControlLags(bar_enabled=True).reset(response.angles)
    bar_first = bar_lags.step(1.0 / 60.0, command)
    assert bar_first.cyclic_lat_deg < command.cyclic_lat_deg
    bar_spike = abs(bar_lags.bar_long_deg) + abs(bar_lags.bar_lat_deg)
    assert 0.0 < bar_spike <= math.sqrt(2.0) * bar_lags.bar_limit_deg + 1e-9
    for _ in range(15 * 60):
        bar_settled = bar_lags.step(1.0 / 60.0, command)
    assert abs(bar_settled.cyclic_lat_deg - command.cyclic_lat_deg) < 0.05
    assert abs(bar_lags.bar_long_deg) + abs(bar_lags.bar_lat_deg) < 0.1 * bar_spike

    # The frame path a flight loop will use: mix, run the lags, then hand the
    # lagged angles straight to the rotor, which must take them as given.
    lagged = ControlLags().reset(response.angles).step(1.0 / 60.0, command)
    frame = model.respond_angles(lagged, start=response.flapping)
    assert frame.controls is None
    assert abs(frame.angles.collective_pitch_deg
               - lagged.collective_pitch_deg) < 1e-12
    assert abs(frame.angles.cyclic_lat_deg - lagged.cyclic_lat_deg) < 1e-12
    assert frame.flapping.converged and frame.thrust > 0.0

    # A collective too low to make thrust at all needs no induced flow, and the
    # solve says so rather than hunting for an inflow that is not there; the
    # hover above converged on a real one.
    assert response.inflow > 5.0, response.inflow
    assert rotor.compute(uniform_airflow(Vector3(0.0, 0.0, 0.0)), 2.0).thrust < 0.0

    def idle_thrust(value):
        return rotor.compute(uniform_airflow(Vector3(0.0, 0.0, -value)),
                             2.0).thrust

    idle_inflow, evaluations, converged = solve_inflow(idle_thrust,
                                                       rotor.radius,
                                                       rotor.omega)
    assert converged and idle_inflow == 0.0
    assert evaluations >= 1


def _demo():
    model = RotorControlModel()
    rotor, law = model.rotor, model.law
    half_long = 0.5 * UH1_LONG_STICK_TRAVEL_IN
    half_lat = 0.5 * UH1_LAT_STICK_TRAVEL_IN
    half_pedal = 0.5 * UH1_PEDAL_TRAVEL_IN

    print("UH-1H control path, TM-73254 tables 2 and 3")
    print("  " + rotor.describe())
    print("  Lock number %.2f, rotor time constant 16/(gamma omega) %.3f sec,"
          " flap inertia %.0f kg m2"
          % (rotor.lock_number, rotor.rotor_time_constant, rotor.flap_inertia))
    print("  a %.0f kg blade spread evenly has that inertia; TM-73254 table 3"
          " does not give a blade mass"
          % rotor.blade_mass_equivalent)
    print("  " + model.tail_rotor.describe())

    print()
    print("what each control sweeps, in inches and in pitch (table 3 gearings"
          " c1 %.4f, c4 %.4f, c5 %.3f, c6 %.4f rad/in):"
          % (law.long_cyclic_per_in, law.lat_cyclic_per_in,
             law.collective_per_in, law.tail_per_pedal_in))
    for name, controls in (("long stick forward", PilotControls(long_stick=half_long)),
                           ("long stick aft", PilotControls(long_stick=-half_long)),
                           ("lat stick right", PilotControls(lat_stick=half_lat)),
                           ("lat stick left", PilotControls(lat_stick=-half_lat)),
                           ("collective up", PilotControls(collective=UH1_COLLECTIVE_TRAVEL_IN)),
                           ("collective down", PilotControls(collective=0.0)),
                           ("pedals left", PilotControls(pedal=-half_pedal)),
                           ("pedals right", PilotControls(pedal=half_pedal))):
        angles = law.mix(controls)
        print("  %-18s %s" % (name, angles))
    print("  table 2 gives the longitudinal cyclic range as %s deg, the lateral"
          " as %s deg," % (UH1_LONG_CYCLIC_RANGE_DEG, UH1_LAT_CYCLIC_RANGE_DEG))
    print("  and the tail rotor collective as %+.0f deg at full left pedal and"
          " %+.0f deg at full right"
          % (UH1_TAIL_COLLECTIVE_LEFT_DEG, UH1_TAIL_COLLECTIVE_RIGHT_DEG))

    print()
    print("hover at maximum gross mass %.0f kg:" % UH1_MAX_GROSS_MASS)
    trim = model.trim_hover()
    print("  stick positions " + str(trim))
    response = model.respond(trim, passes=1)
    print(response)

    print()
    print("stick sweep from that hover, one inch at a time:")
    print("  stick               collective   blade cyclic (cos, sin)   flapping"
          " tilt        force x      force y     thrust")
    for offset in (-2.0, -1.0, 0.0, 1.0, 2.0):
        for which in ("long_stick", "lat_stick"):
            controls = PilotControls(collective=trim.collective,
                                     pedal=trim.pedal, **{which: offset})
            got = model.respond(controls, start=response.flapping)
            print("  %-8s %+5.2f in  %6.2f deg   %+6.2f %+6.2f     %5.2f deg"
                  " at %6.1f   %+8.0f    %+8.0f   %7.0f"
                  % (which.split("_")[0], offset,
                     got.angles.collective_pitch_deg, got.angles.cyclic_long_deg,
                     got.angles.cyclic_lat_deg, got.flapping.tilt_deg,
                     got.flapping.tilt_azimuth_deg, got.loads.force.x,
                     got.loads.force.y, got.thrust))

    print()
    torque = model.respond(trim).loads.shaft_torque
    print("pedals, and the yaw moment that balances the main rotor torque of"
          " %.0f N m:" % torque)
    for offset in (-half_pedal, 0.0, half_pedal):
        controls = PilotControls(collective=trim.collective, pedal=offset)
        got = model.respond(controls, tail=True)
        print("  pedal %+5.2f in: tail %+6.2f deg, %6.0f N, yaw %+8.0f N m,"
              " out of balance %+7.0f N m"
              % (offset, got.tail_collective_deg, got.tail_thrust,
                 got.tail_yaw_moment, got.yaw_torque_balance))

    print()
    print("forward flight at the hover controls, cyclic centred:")
    print("  speed  inflow   flapping tilt      thrust tilt        H force"
          "   side force   torque")
    for speed in (0.0, 20.0, 30.0, 55.0):
        got = model.respond(trim, forward_speed=speed, start=response.flapping,
                            passes=1)
        print("  %5.1f  %5.2f m/s  %5.2f deg at %6.1f deg  %5.2f deg at %6.1f"
              "  %+8.0f    %+8.0f  %7.0f"
              % (speed, got.inflow, got.flapping.tilt_deg,
                 got.flapping.tilt_azimuth_deg, got.thrust_tilt_deg,
                 got.thrust_tilt_azimuth_deg, got.loads.h_force,
                 got.loads.side_force, got.loads.shaft_torque))

    print()
    print("a one inch forward stick step, held, at 60 Hz, from the hover trim:")
    print("   time    cyclic sin   cyclic cos   control tilt   collective"
          "   bar contribution")
    lags = ControlLags().reset(response.angles)
    command = law.mix(PilotControls(collective=trim.collective, pedal=trim.pedal,
                                    long_stick=1.0))
    elapsed = 0.0
    for step in range(int(1.0 * 60) + 1):
        if step:
            lags.step(1.0 / 60.0, command)
            elapsed += 1.0 / 60.0
        if abs(elapsed - round(elapsed, 2)) < 1e-9 and (step % 6) == 0:
            print("  %5.2f s   %+6.2f deg   %+6.2f deg    %+6.2f deg    %6.2f deg"
                  "   %+7.3f deg"
                  % (elapsed, lags.cyclic_lat_deg, lags.cyclic_long_deg,
                     command.control_tilt_deg, lags.collective_pitch_deg,
                     lags.bar_long_deg))

    print()
    _self_test()
    print("self test passed")


if __name__ == "__main__":
    _demo()
