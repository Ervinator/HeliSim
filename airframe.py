"""The six degree of freedom airframe: how the aircraft's forces move it.

:mod:`aerodynamics` answers what the air does to a rotor and
:mod:`rotor_control` what the pilot's hands and feet do to that rotor.  This
module is the third piece: the helicopter as a rigid body, with the forces and
moments of every component of the aircraft summed at its centre of gravity and
integrated forward in time.

**The model is TM-73254's own.**  That report exists precisely for this job -
"A Mathematical Force and Moment Model of a UH-1H Helicopter for Flight
Dynamics Simulations" - and it prints its equations (1) to (66) together with
every constant, so it is used here as printed rather than re-derived.  What
that means for the rest of this project:

* the main rotor is the report's quasi-static closed form (equations 1 to 10):
  thrust, H force, Y force and torque as functions of collective pitch,
  inflow ratio, advance ratio, mean blade drag coefficient and flapping, in
  terms of the constants R1 to R9.  :mod:`aerodynamics`' blade element rotor is
  a higher fidelity answer to the same question; the self test checks the two
  against each other at the hover, and where they agree is documented in
  :func:`main_rotor_forces`;
* the tail rotor is equation 37, an isolated-rotor approximation whose damping
  constant T4 the report admits was "adjusted empirically" to match the flight
  test yaw responses;
* fuselage, vertical fin and horizontal stabilizer are the report's separate
  aerodynamic derivations (equations 42 to 60), with the UH-1B stabilizer
  incidence schedule of its table 1 because the UH-1H one was not available;
* ground effect is its equation 10a, K_G = 1 - exp(-(Z/D)/G1), and component
  interference is deliberately absent because the report has none.

Everything is quasi-steady, the rotor speed is constant, and there is no
engine, no rotor governor and no ground contact: this is the airframe, not the
whole simulation.

**Axes.**  Two frames, both the ones the report uses, because every equation
below is a line out of it.

Body axes, origin at the c.g.:

    +x  forward (the way the nose points)
    +y  starboard (right)
    +z  down, parallel to the main rotor shaft

with the moments L about +x (roll right positive), M about +y (pitch up
positive) and N about +z (yaw right positive).  Earth axes are NED - +x north,
+y east, +z down - and the attitude is the usual Euler triple (roll, pitch,
yaw) carried in :class:`FlightState`, pitch-up and yaw-right positive.

A sign trap worth stating once, because it is a trap: the rotor frame of
:mod:`aerodynamics` is +x nose, +y *port*, +z up (right handed, since forward
cross port is up).  Its +y is therefore the opposite of this module's +y, and
its +z is the opposite of this module's +z.  Nothing in the control path needs
a flip for that - the mixing carries physical directions (forward, starboard),
which are the same in both - but a vector read out of a rotor load and dropped
into a body force would need y and z negated, and that is the one thing to get
right when :mod:`aerodynamics`' blade element rotor is ever wired in here in
place of equations 1 to 3.

The renderer's world (see main.py: north is -z, up is +y, east is +x) is one
conversion away: ``renderer = (east, -down, -north)``.

**The collective pitch.**  Equation 1 takes the pitch a blade would have if it
had no twist, which for a linearly twisted blade is its pitch at three quarters
of the radius.  This project's rotor is twisted -10 deg from root to tip
(:data:`aerodynamics.UH1_TWIST_DEG`), so the report's collective pitch is
7.5 deg below the root pitch that :mod:`rotor_control` produces, and
:func:`report_collective_rad` makes that conversion in one place.  The
self test asserts that the two formulations then agree: at the trim hover,
equation 1 and the blade element rotor's own thrust land within 1 per cent of
each other, on a rotor model neither of which was fitted to the other.

**What is deliberately missing:** unsteady aerodynamics, dynamic stall, rotor
wake geometry, the vortex ring state, component interference, a flapping
solution of its own (the flapping is equations 6 and 7, quasi-static, with the
report's fixed a0 = 0.048 rad cone angle rather than the coning
:func:`rotor_control.solve_flapping` would give), lead-lag, and - in the
aircraft this module flies by default - any engine or rotor speed dynamics:
the rotor speed is held at 100 per cent unless an :class:`engine.Engine` is
handed to :class:`Airframe`, which is where the governor, the droop band and
the wind down after a failure live.  The inertia matrix carries the UH-1H's
2007 kg m^2 product of inertia, which is what couples roll and yaw.

Standard library only, so it runs and tests headless.
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

from aerodynamics import (GRAVITY, RHO_SEA_LEVEL, Vector3,
                          glauert_induced_velocity, UH1_RADIUS, UH1_RPM,
                          UH1_RPM_LOW,
                          UH1_TAIL_RADIUS, UH1_TWIST_DEG,
                          UH1_ROTOR_TIME_CONSTANT_SIM_S)
from rotor_control import (ControlLags, MixingLaw, PilotControls,
                           UH1_COLLECTIVE_TRAVEL_IN,
                           UH1_ROTOR_TIME_CONSTANT_S, solve_linear)
from engine import Engine, UH1_ENGINE_DROOP_RPM, UH1_ENGINE_RPM

# ---------------------------------------------------------------------------
# Unit conversions, so that the report's English column can be quoted next to
# the SI value the model uses.  All exact or NIST defined.
# ---------------------------------------------------------------------------

#: Metres in a foot.
FOOT = 0.3048
#: Kilograms in a pound (mass).
POUND = 0.45359237
#: Newtons in a pound (force).
POUND_FORCE = 4.4482216152605
#: Kilograms in a slug: 32.17405 lb, which is the g_c the English column of
#: table 3 carries implicitly.  Forgetting it is a factor of 32 error, which is
#: why the self test converts every tabulated constant rather than trusting the
#: arithmetic in the docstrings.
SLUG = 32.17405 * POUND
#: kg m^2 in a slug ft^2.
SLUG_FT2 = SLUG * FOOT * FOOT
#: N/(m/s)^2 in a lb/(ft/s)^2, i.e. SLUG / FOOT.  Table 3's fuselage and
#: empennage constants are all in this unit (it is a mass per length), and
#: converting the English column reproduces the metric column exactly - which is
#: how the metric column was checked, see the self test.
LB_PER_FT_S2 = SLUG / FOOT
#: J/(m/s)^2 in a ft lb/(ft/s)^2, which is a mass: one slug, 14.594 kg.  The
#: moment constants M1 and N1 are in this unit.
FT_LB_PER_FT_S2 = SLUG
#: N/m in a lb/ft, for the tail rotor's damping constant T4.  The number is the
#: same 14.594 as the line above, which is the sort of coincidence that makes
#: this file worth testing rather than reading.
LBF_PER_FT = POUND_FORCE / FOOT
#: Metres per second in a knot, one nautical mile per hour exactly.  The
#: report's level flight condition reads "60 knots", so this is the unit
#: :meth:`Airframe.trim_level_flight` is asked for its airspeed in.
KNOT = 1852.0 / 3600.0

# ---------------------------------------------------------------------------
# Mass, inertia and geometry: TM-73254 table 3 (weights and inertias, from
# Bell Helicopter Company data) and table 2 (the geometry).
# ---------------------------------------------------------------------------

#: Weight of the aircraft the report's simulation flew, 8700 lb.  Table 3
#: lists both that and the 6158 lb of the instrumented flight test aircraft;
#: the inertias and every characteristic length belong to the 8700 lb
#: simulation, so that is the default here.  For the flight test comparisons
#: of the report's figures 2 to 9, fly UH1_TEST_MASS instead.
UH1_SIM_MASS = 8700.0 * POUND             # kg, 3946.3 kg
UH1_TEST_MASS = 6158.0 * POUND            # kg, 2793.2 kg

#: Inertia matrix in body axes, table 3, English column: 2800, 12733, 10800
#: and 1480 slug ft^2 (the report's own metric column rounds these to 3795,
#: 17259, 14639 and 2006 kg m^2).  The product of inertia is what couples roll
#: and yaw, and 2007 kg m^2 is a big one - 53 per cent of Ixx - which is why a
#: UH-1H rolls when it yaws.
UH1_IXX = 2800.0 * SLUG_FT2               # kg m^2, 3796
UH1_IYY = 12733.0 * SLUG_FT2              # kg m^2, 17265
UH1_IZZ = 10800.0 * SLUG_FT2              # kg m^2, 14643
UH1_IXZ = 1480.0 * SLUG_FT2               # kg m^2, 2007

#: Characteristic lengths from the c.g., table 3, in feet, all "Ref. 9" or a
#: reasonable estimate of the waterline the report did not know.  They check
#: against table 2's stations and waterlines: hub waterline 136.5 in less the
#: 55 in c.g. waterline is 81.5 in = 6.79 ft, and the tail rotor shaft station
#: 479.4 in less 28 ft is station 143.4 in, inside table 2's 130 to 144 in
#: c.g. range.
UH1_HUB_HEIGHT = 6.79 * FOOT              # m, hub waterline above the c.g.
UH1_TAIL_ARM = 28.0 * FOOT                # m, tail rotor hub aft of the c.g.
UH1_TAIL_HEIGHT = 6.88 * FOOT             # m, tail rotor hub above the c.g.
UH1_STAB_ARM = 19.67 * FOOT               # m, stabilizer a.c. aft of the c.g.
UH1_FIN_ARM = 25.0 * FOOT                 # m, vertical fin a.c. aft of the c.g.

#: How far the c.g. sits *forward* of the rotor hub, table 3: the report
#: simulated -0.520 ft (the other column, for the instrumented aircraft, is
#: -0.541 ft).  Negative, so the c.g. is 6.2 in aft of the hub, which is inside
#: table 2's c.g. range and is why a trimmed hover needs a few degrees of
#: forward thrust tilt: see :meth:`Airframe.trim_hover`.
UH1_CG_AHEAD_OF_HUB = -0.520 * FOOT       # m, -0.158 m

#: Ground effect range, equation 10a: K_G = 1 - exp(-(Z/D)/G1).
UH1_GROUND_EFFECT_RANGE = 0.28            # -, G1, table 3, from Ref. 6

# ---------------------------------------------------------------------------
# Main rotor force constants, TM-73254 table 3 (metric column, which is the
# English column converted).  They are not free parameters: every one of them
# is a closed form in the rotor's geometry and the report's own a and sigma, so
# the self test rebuilds them from aerodynamics.UH-1's rotor and asserts they
# agree.  A tie like that is what makes a set of numbers out of a scanned table
# trustworthy.
# ---------------------------------------------------------------------------

UH1_ROTOR_R1 = 17.1e5     # N,        sigma*a/2 * rho*A*(omega R)^2
UH1_ROTOR_R2 = 7.37e-3    # s,        1 / (4 omega),  so omega = 33.9 rad/s
UH1_ROTOR_R3 = 8.70e-2    # s,        1 / (2 a),  the lift curve slope a = 5.75
UH1_ROTOR_R4 = 2.95e-2    # s,        1 / omega
UH1_ROTOR_R5 = 4.36e-2    # s,        1 / (4 a)
UH1_ROTOR_R7 = 4.00e-3    # s/m,      1 / (omega R),  so omega R = 250 m/s
UH1_ROTOR_R8 = 0.38e-7    # 1/N,      1 / (2 rho A (omega R)^2)
UH1_ROTOR_R9 = 12.35e6    # J,        R1 * R, the constant in the torque
UH1_ROTOR_DELTA0 = 0.009  # -,        mean blade drag coefficient, equation 9
UH1_ROTOR_DELTA2 = 0.092e-11  # 1/N^2, the thrust squared part of the same
#: Cone angle the report fixes as the blade pitch reference, equation 5: the
#: hub precone angle of table 2, 2.75 deg.  Its English column prints 0.48 rad,
#: a typo for this value; the metric column and table 2 both say 0.048.
UH1_ROTOR_A0 = 0.048      # rad,      2.75 deg

#: The rotor time constant R6 of table 3 is the flapping lag 16 / (gamma omega)
#: and is carried by :mod:`rotor_control` with its footnote: 0.144 sec is what
#: the simulation flew, 0.072 sec is what one blade's inertia gives, and the
#: quicker rotor was rejected because pilots found it harder to fly.  This
#: module uses R6 in the flapping coefficients (equations 6 and 7), so the
#: choice of value is the same choice :class:`rotor_control.ControlLags` makes
#: and defaults the same way: the physical 0.072 sec, with the simulated
#: 0.144 sec a field away for reproducing the report's step responses.

# ---------------------------------------------------------------------------
# Tail rotor, fuselage, fin and stabilizer constants, TM-73254 table 3.
# All in SI, converted from the English column with LB_PER_FT_S2 or
# FT_LB_PER_FT_S2; table 3's own metric column is quoted where it helps, and
# its one clear slip is noted at UH1_FUSELAGE_SIDE.
# ---------------------------------------------------------------------------

UH1_TAIL_T1 = 22.17e-4    # s/m,      1 / (2 (omega R)_TR), tip speed 225.5 m/s
UH1_TAIL_T2 = 35.7        # N^0.5,    the isolated rotor approximation, eq 37
UH1_TAIL_T3 = 31.1e3      # N/rad,    with T2 squared, the collective term
UH1_TAIL_T4 = 217.4       # N/m,      the damping the report adjusted to fit
UH1_TAIL_T5 = 0.0         # N/m,      zero in the simulation
#: C7 of table 3: the tail rotor collective that goes with zero pedal, 0.119
#: rad = 6.8 deg.  :mod:`rotor_control`'s mixing uses the midpoint of table 2's
#: pedal stop range (4 deg) instead, because that is a rigging constant of the
#: same kind as the cyclic rigging; the report's C7 is the value its own trim
#: sat at, and the difference is pedal rigging.
UH1_TAIL_ZERO_PEDAL = 0.119   # rad,  6.8 deg

UH1_FUSELAGE_D1 = 1.05    # N/(m/s)^2, 0.022 lb/(ft/s)^2, equation 42
UH1_FUSELAGE_D2 = 9.62    # N/(m/s)^2, 0.201, equation 43
UH1_FUSELAGE_D3 = 30.92   # N/(m/s)^2, 0.646, equation 44
UH1_FUSELAGE_LIFT = 3.13  # N/(m/s)^2, 0.0654, L1, equations 42 and 44
#: Y1 of equation 43, "assumed same as L1".  Table 3's metric column prints
#: .29 for it, which is not 0.0654 lb/(ft/s)^2 but a slip: the English column
#: says 6.54e-2 and the origin line says it equals L1, so L1 is used.
UH1_FUSELAGE_SIDE = 3.13  # N/(m/s)^2, 0.0654
UH1_FUSELAGE_MOMENT = 13.95     # J/(m/s)^2, 0.956 ft lb/(ft/s)^2, M1, eq 45
UH1_FUSELAGE_YAW_MOMENT = 13.95  # J/(m/s)^2, N1, "assumed same as L1", eq 46

UH1_FIN_F1 = 0.77         # N/(m/s)^2, 0.016, the fin's drag, equations 50-52
UH1_FIN_K1 = 1.40         # N/(m/s)^2, 0.0292, its lift, equation 50
UH1_FIN_K2 = 0.51         # N/(m/s)^2, 0.0106, its stalled branches, 51 and 52
UH1_STAB_H1 = 1.48        # N/(m/s)^2, 0.031, the stabilizer's lift, equation 57
UH1_STAB_H2 = 0.54        # N/(m/s)^2, 0.01128, H1 * tan 20 deg, its stall
UH1_STAB_H4 = 0.40        # N/(m/s)^2, 0.0083, its drag at 90 deg

#: Horizontal stabilizer incidence against longitudinal stick position,
#: TM-73254 table 1, in the units the report tabulates it: stick inches and
#: radians.  The report used a UH-1B schedule because the UH-1H one was not
#: available, and notes that the stabilizer is linked to the longitudinal
#: cyclic control, so this is a function of the stick and not of the cyclic
#: pitch the stick eventually makes.  The table's middle column is headed
#: "delta_e, in." - the OCR of the report loses that heading and prints the
#: inches as if they were degrees, so read the page image, not the text.
UH1_STABILIZER_SCHEDULE = (
    (-6.45, 0.0224), (-6.00, 0.0174), (-5.00, 0.0), (-4.00, -0.0192),
    (-3.00, -0.0384), (-2.00, -0.0541), (-1.00, -0.0690), (0.0, -0.0820),
    (1.00, -0.0850), (2.00, -0.0803), (3.00, -0.0628), (4.00, -0.0300),
    (5.00, 0.0035), (6.00, 0.0593), (6.45, 0.0942))

#: The schedule spans 6.45 in either way, which is the half throw of table 2's
#: 12.9 in longitudinal stick: the UH-1B and the UH-1H stick travels are the
#: same, so the UH-1B schedule drops straight onto the UH-1H control.
UH1_STABILIZER_STICK_TRAVEL_IN = 6.45


def _interpolate(points, x):
    """Linear interpolation in a table of ``(x, y)`` pairs, clamped at the ends."""
    if x <= points[0][0]:
        return points[0][1]
    if x >= points[-1][0]:
        return points[-1][1]
    for index in range(1, len(points)):
        x1, y1 = points[index]
        if x <= x1:
            x0, y0 = points[index - 1]
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return points[-1][1]


def stabilizer_incidence_rad(long_stick_in):
    """Horizontal stabilizer incidence, rad, for a longitudinal stick position.

    Equation 4 of the report's stabilizer section links the stabilizer to the
    longitudinal cyclic control, so this is the table 1 schedule evaluated at
    the stick, not at the cyclic pitch the stick produces.  Held to the ends of
    the table rather than extrapolated, which is what a mechanical linkage does
    when the stick hits its stop.
    """
    return _interpolate(UH1_STABILIZER_SCHEDULE, long_stick_in)


def ground_effect_factor(rotor_height, radius=UH1_RADIUS):
    """Equation 10a: K_G = 1 - exp(-(Z/D)/G1), the reduction in inflow.

    ``rotor_height`` is the hub's height above the ground, m, and D = 2R.  A
    rotor on the ground (Z = 0) has K_G = 0, i.e. no induced flow at all and
    the whole ground cushion; one diameter up it is 0.97, and only past a few
    diameters is it indistinguishable from 1.
    """
    diameter = 2.0 * radius
    if diameter <= 0.0:
        return 1.0
    return 1.0 - math.exp(-(max(rotor_height, 0.0) / diameter)
                          / UH1_GROUND_EFFECT_RANGE)


@dataclass(frozen=True)
class ControlPitch:
    """The rotor pitches the force model needs, in the report's own terms.

    ``collective_rad`` is the report's collective pitch, which is a blade with
    no twist: for a linearly twisted blade it is the pitch at three quarters of
    the radius, so :meth:`from_angles` subtracts the same three quarters of the
    twist from the root pitch :mod:`rotor_control` produces.
    ``control_axis_long_rad`` and ``control_axis_lat_rad`` are B1S and A1S, the
    instantaneous orientation of the control axis in the body axes of this
    module: forward and starboard positive, which is the same physical tilt
    :class:`rotor_control.ControlAngles` carries as ``control_long_deg`` and
    ``control_lat_deg`` (see the module docstring on the two y axes).
    ``long_stick_in`` is only for the stabilizer schedule.
    """

    collective_rad: float
    control_axis_long_rad: float
    control_axis_lat_rad: float
    tail_collective_rad: float
    long_stick_in: float = 0.0

    @classmethod
    def from_angles(cls, angles, long_stick_in=0.0, twist_deg=UH1_TWIST_DEG):
        """Build from the :mod:`rotor_control` angles the rotor actually sees."""
        return cls(
            collective_rad=report_collective_rad(angles.collective_pitch_deg,
                                                 twist_deg),
            control_axis_long_rad=math.radians(angles.control_long_deg),
            control_axis_lat_rad=math.radians(angles.control_lat_deg),
            tail_collective_rad=math.radians(angles.tail_collective_deg),
            long_stick_in=long_stick_in)


def report_collective_rad(root_collective_deg, twist_deg=UH1_TWIST_DEG):
    """The report's collective pitch, rad, for a root collective pitch in deg.

    TM-73254's thrust equation is the no-twist relation
    ``C_T = sigma a / 2 (theta_0 / 3 - lambda / 2)``, while a twisted blade is
    ``C_T = sigma a / 2 (theta_root / 3 + theta_twist / 4 - lambda / 2)``.  The
    two agree when ``theta_0 = theta_root + 3/4 theta_twist``, i.e. when the
    report's collective pitch is the pitch at three quarters of the radius.
    With this project's -10 deg washout that is 7.5 deg below the root pitch,
    and the self test checks the identity on the real hover rather than on the
    algebra: equation 1 and the blade element rotor then agree to 0.2 per cent.
    """
    return (math.radians(root_collective_deg)
            + 0.75 * math.radians(twist_deg))


#: Shaft speed and tip speed of the reference rotor, from table 2's 324 rpm and
#: 48 ft diameter.  Table 3's R2 = 1 / (4 omega) and R7 = 1 / (omega R) are
#: what pin these: 7.37e-3 sec is 33.9 rad/s and 4.00e-3 sec/m is 250 m/s, and
#: the self test asserts both, so the report's constants and this project's
#: rotor geometry vouch for each other.
UH1_OMEGA = UH1_RPM * 2.0 * math.pi / 60.0    # rad/s, 33.93
UH1_TIP_SPEED = UH1_OMEGA * UH1_RADIUS        # m/s, 249.4

#: Polar inertia of the two bladed rotor about the shaft, kg m^2.
#:
#: This is the one number in a rotor speed equation that neither report gives.
#: It does not have to be mined from anywhere, though: the UH-1's teetering
#: hinge sits *on* the shaft, so a blade's flap inertia about the hinge is also
#: its polar inertia about the shaft, and :mod:`rotor_control` already derives
#: and asserts one blade's 1658 kg m^2 from two 92 kg blades spread evenly along
#: the span.  The pair is therefore 3315 kg m^2, and the self test checks this
#: against that rotor rather than against a table.
UH1_ROTOR_INERTIA = 3315.0    # kg m^2, 2 X 1658 about the shaft


def _solve_inflow_ratio(thrust_at, mu, wind_along_axis, ground_effect,
                        r7=UH1_ROTOR_R7, r8=UH1_ROTOR_R8,
                        tolerance=1e-10, iterations=80):
    """Solve the report's inflow ratio, equation 10, for a rotor disc.

    Equation 10 is implicit in lambda::

        lambda = -R7 * w_C - R8 * T * K_G / sqrt(mu ** 2 + lambda ** 2)

    and so is equation 1, because the thrust it gives depends on lambda.  The
    pair is solved together by writing x = -lambda, the inflow the rotor
    actually pulls through the disc divided by omega R, which makes both
    equations monotone in x and gives::

        h(x) = x - R7 * w_C - R8 * K_G * T(x) / sqrt(mu ** 2 + x ** 2)

    with a single root bracketed between the zero inflow at one end (where the
    momentum term blows up) and a deep inflow at the other.  ``thrust_at(x)``
    returns the thrust equation 1 gives when the inflow ratio is -x, in N.  A
    bisection is used, so it cannot oscillate the way the report's fixed point
    iteration would: that iteration's own derivative at a UH-1H hover is about
    -40, which diverges.  ``r7`` and ``r8`` are table 3's two inflow constants,
    both closed forms in the rotor speed, so a rotor that is not at 100 per cent
    solves its own equation rather than the reference one.  Returns
    ``(inflow_ratio, evaluations, converged)`` and
    reports the answer as unconverged rather than guessing when no bracket can
    be found, which is what a windmill brake state looks like from here.
    """
    low, high = 1e-9, 0.5
    evaluations = 0

    def residual(x):
        return (x - r7 * wind_along_axis
                - r8 * ground_effect * thrust_at(x)
                / math.sqrt(mu * mu + x * x))

    for _ in range(60):
        evaluations += 1
        if residual(high) > 0.0:
            break
        high *= 2.0
    else:
        return -low, evaluations, False

    for _ in range(max(int(iterations), 8)):
        evaluations += 2
        middle = 0.5 * (low + high)
        if residual(middle) < 0.0:
            low = middle
        else:
            high = middle
        if high - low < tolerance:
            break
    return -0.5 * (low + high), evaluations, True


@dataclass
class RotorHubForces:
    """What TM-73254 equations 1 to 10 say the main rotor is doing.

    ``thrust``, ``h_force`` and ``y_force`` are T, H and Y: the thrust along the
    control axis and the two in-plane forces orthogonal to it, in the control
    axis - wind system of the report's figure 1.  ``torque`` is Q, the shaft
    torque the engine supplies.  ``inflow_ratio`` is lambda, negative in normal
    flight because the flow goes down through the disc, and
    ``induced_velocity`` is the speed of that flow, m/s, positive downwards.
    ``flapping_a1`` and ``flapping_b1`` are equations 6 and 7, the once per
    revolution flapping with respect to the control axis, not
    :func:`rotor_control.solve_flapping`'s cone angle and tilt: this rotor cones
    to the report's fixed a0 of 2.75 deg and only tilts with speed.
    """

    thrust: float                  # T, N
    h_force: float                 # H, N
    y_force: float                 # Y, N
    torque: float                  # Q, N m
    inflow_ratio: float            # lambda
    advance_ratio: float           # mu
    drag_coefficient: float        # delta, equation 9
    flapping_a1: float             # rad
    flapping_b1: float             # rad
    induced_velocity: float        # m/s, positive down through the disc
    wind_u: float                  # u_C, m/s, the longitudinal wind at the hub
    wind_v: float                  # v_C, m/s, the lateral wind at the hub
    wind_along_axis: float         # w_C, m/s, positive up the control axis
    control_axis_pitch_rate: float  # q_C, rad/s
    control_axis_roll_rate: float   # p_C, rad/s
    ground_effect: float           # K_G
    evaluations: int = 0
    converged: bool = True

    @property
    def in_plane_speed(self):
        """Speed of the wind in the disc plane, m/s, equations 14 and 15."""
        return math.hypot(self.wind_u, self.wind_v)

    @property
    def direction_cosines(self):
        """``(u_C, v_C) / |V|``, the unit vector of the in-plane wind.

        Zero in a hover, which is the limit equations 35 and 36 need and the
        state the report's constants were tuned in.
        """
        speed = self.in_plane_speed
        if speed <= 1e-9:
            return 0.0, 0.0
        return self.wind_u / speed, self.wind_v / speed


@dataclass(frozen=True)
class RotorConstants:
    """TM-73254 table 3's rotor constants at one rotor speed.

    The table is written for one rotor speed and every one of its nine constants
    is a closed form in omega, so the whole set moves with the rotor: ``R1`` and
    ``R9`` go as ``omega ** 2``, ``R2``, ``R4``, ``R7`` and the tail rotor's
    ``T1`` as ``1 / omega``, ``R8`` as ``1 / omega ** 2``, and ``R3`` and ``R5``
    (which carry only the lift curve slope) do not move at all.  ``tip_speed`` is
    ``omega R`` and is what the report calls ``omega R`` throughout.

    The tabulated numbers *are* the 100 per cent values, so
    :func:`rotor_constants` scales them and at :data:`UH1_OMEGA` returns exactly
    the transcribed table, to the last bit.  That is deliberate: the closed forms
    in the rotor's geometry are checked once, where the table is transcribed (see
    :func:`_self_test`), and what is checked here is the scaling, which is the
    part that a rotor speed dynamics gets wrong quietly.
    """

    r1: float            # N,     sigma a / 2 rho A (omega R)^2
    r2: float            # s,     1 / (4 omega)
    r3: float            # s,     1 / (2 a)
    r4: float            # s,     1 / omega
    r5: float            # s,     1 / (4 a)
    r6: float            # s,     16 / (gamma omega), the flapping lag
    r7: float            # s/m,   1 / (omega R)
    r8: float            # 1/N,   1 / (2 rho A (omega R)^2)
    r9: float            # J,     R1 R, the constant in the torque, equation 4
    tip_speed: float     # m/s,   omega R
    tail_t1: float       # s/m,   1 / (2 (omega R)_TR)
    omega: float         # rad/s, the rotor speed these belong to
    rpm: float           # rpm,   the same, in the units a gauge is marked in


def rotor_constants(omega=UH1_OMEGA):
    """Table 3's rotor constants at a rotor speed *omega*, rad/s.

    At the default - the 324 rpm reference of :data:`UH1_OMEGA` - this is the
    table itself, unchanged; anywhere else it is the table scaled by the closed
    forms in :class:`RotorConstants`.  ``R6`` is carried here for the table's
    sake from the physical 0.072 sec reading of it, which is ``16 / (gamma
    omega)`` and so moves with the rotor like the rest; the force model takes the
    flapping lag from :attr:`Airframe.rotor_time_constant` instead, where the
    choice between the table's two readings is made.
    """
    if omega <= 0.0:
        raise ValueError("the rotor has to be turning")
    speed = omega / UH1_OMEGA
    return RotorConstants(
        r1=UH1_ROTOR_R1 * speed * speed,
        r2=UH1_ROTOR_R2 / speed,
        r3=UH1_ROTOR_R3,
        r4=UH1_ROTOR_R4 / speed,
        r5=UH1_ROTOR_R5,
        r6=UH1_ROTOR_TIME_CONSTANT_S / speed,
        r7=UH1_ROTOR_R7 / speed,
        r8=UH1_ROTOR_R8 / (speed * speed),
        r9=UH1_ROTOR_R9 * speed * speed,
        tip_speed=UH1_TIP_SPEED * speed,
        tail_t1=UH1_TAIL_T1 / speed,
        omega=omega,
        rpm=omega * 60.0 / (2.0 * math.pi))


def main_rotor_forces(controls, body_velocity, body_rates,
                      air_density=RHO_SEA_LEVEL, rotor_height=1000.0,
                      rotor_time_constant=UH1_ROTOR_TIME_CONSTANT_S,
                      omega=UH1_OMEGA, constants=None):
    """TM-73254 equations 1 to 10: the main rotor's force at the hub.

    ``controls`` is a :class:`ControlPitch`, ``body_velocity`` the aircraft's
    velocity in body axes, m/s, and ``body_rates`` its (p, q, r), rad/s.
    ``rotor_height`` is the hub's height above the ground, m, for the ground
    effect factor of equation 10a; the default is high enough to be out of it.
    ``air_density`` reaches only the momentum theory behind the inflow, since
    the report's own constants are frozen at sea level.

    ``omega`` is the rotor speed the table 3 constants are taken at, rad/s, and
    defaults to the 324 rpm reference; ``constants`` is the whole
    :class:`RotorConstants` set put in place of it, for a caller that has built
    them once - a trim, or an integrator evaluating the same frame four times.
    Note that *air_density* does not scale them: the report's constants carry
    sea level density inside them, exactly as they carry omega, and moving them
    with the rotor speed is the one of those two the model can honestly do.

    ``rotor_time_constant`` is R6 of table 3 where it appears in the flapping
    coefficients, equations 6 and 7.  Flip it to
    :data:`aerodynamics.UH1_ROTOR_TIME_CONSTANT_SIM_S` to reproduce the report's
    own step responses.

    The quasi-static rotor this gives is the report's, not this project's:
    :mod:`aerodynamics` solves a two bladed blade element rotor with a real
    flapping solution, and this solves one closed form in nine constants.  They
    agree to a fraction of a per cent on the hover (see the self test) and part
    company as the disc tilts, because the closed form is a first harmonic
    approximation.
    """
    if constants is None:
        constants = rotor_constants(omega)
    collective = controls.collective_rad
    b1s = controls.control_axis_long_rad
    a1s = controls.control_axis_lat_rad
    u_b, v_b, w_b = body_velocity.x, body_velocity.y, body_velocity.z
    p_b, q_b = body_rates.x, body_rates.y
    hub = UH1_HUB_HEIGHT

    # Wind components at the rotor hub, equations 11 to 13.  The rotor's own
    # downwash is not in here - it is the inflow the solve below finds - and the
    # hub's velocity from the body rates is the rate damping the report wrote
    # into these three lines.
    u_c = -u_b - w_b * b1s + q_b * hub                       # (11)
    v_c = -v_b - w_b * a1s - p_b * hub                       # (12)
    w_c = (-w_b + b1s * (u_b - q_b * hub)                    # (13)
           + a1s * (v_b + p_b * hub))

    in_plane = math.hypot(u_c, v_c)
    mu = in_plane * constants.r7                             # (8)
    # Direction cosines of the in-plane wind.  A hover has none, and the
    # report's equations 35 and 36 then give p_C = q_C = 0 rather than dividing
    # by zero, which is their limit and the state the constants were tuned in.
    if in_plane > 1e-9:
        cos_c, sin_c = u_c / in_plane, v_c / in_plane
    else:
        cos_c, sin_c = 0.0, 0.0
    p_c = -p_b * cos_c - q_b * sin_c                         # (35)
    q_c = -q_b * cos_c + p_b * sin_c                         # (36)

    shape = collective * (1.0 / 3.0 + 0.5 * mu * mu) + constants.r2 * mu * p_c

    def thrust_at(inflow):
        """Equation 1 with lambda = *inflow*."""
        return constants.r1 * (shape + 0.5 * inflow)

    ground_effect = ground_effect_factor(rotor_height)
    inflow, evaluations, converged = _solve_inflow_ratio(
        lambda x: thrust_at(-x), mu, w_c, ground_effect,
        r7=constants.r7, r8=constants.r8)
    induced = -inflow * constants.tip_speed - w_c

    # Flapping coefficients, equations 5 to 7, about the report's fixed cone
    # angle.  R6 is the rotor time constant: 0.072 sec from one blade's inertia,
    # or the 0.144 sec the simulation flew.
    a0 = UH1_ROTOR_A0
    r4, r6 = constants.r4, rotor_time_constant
    a1 = (mu * (8.0 * collective / 3.0 + 2.0 * inflow) + r4 * p_c
          - r6 * q_c) / (1.0 - 0.5 * mu * mu)                # (6)
    b1 = (4.0 * mu * a0 / 3.0 - r4 * q_c - r6 * p_c) / (1.0 + 0.5 * mu * mu)

    thrust = thrust_at(inflow)                               # (1)
    delta = UH1_ROTOR_DELTA0 + UH1_ROTOR_DELTA2 * thrust * thrust   # (9)
    r1, r3, r5, r9 = (constants.r1, constants.r3, constants.r5,
                      constants.r9)

    h_force = r1 * (                                         # (2)
        r3 * delta * mu
        + a1 * (collective / 3.0 + 0.75 * inflow * (1.0 - 0.375 * mu * mu)
                + 0.25 * mu * a1)
        - (0.5 * mu * inflow * collective) * (1.0 - 2.0 * mu / (3.0 * math.pi))
        - 0.25 * inflow * inflow * mu
        - 0.375 * inflow * mu * mu * (1.0 + 3.33 * mu)
        - a0 * (b1 / 6.0 - 0.25 * a0 * mu)
        - r4 * q_c * (a0 / 6.0 + mu * b1 / 16.0)
        - r4 * p_c * (collective / 6.0 + inflow / 2.0)
        + mu * a1 / 16.0)

    y_force = r1 * (                                         # (3)
        a0 * (a1 * (1.0 / 6.0 - mu * mu)
              - 1.5 * mu * (inflow + 0.5 * collective))
        + b1 * (collective / 3.0 * (1.0 + 1.5 * mu * mu)
                + 0.75 * inflow + 0.25 * mu * a1)
        + r4 * q_c * (collective / 6.0 + inflow / 2.0 + 0.4375 * mu * a1)
        - r4 * p_c * (a0 / 6.0 - 0.3125 * mu * b1))

    torque = r9 * (                                          # (4)
        r5 * delta * (1.0 + mu * mu)
        - inflow * collective / 3.0
        - 0.5 * inflow * inflow
        - (a1 * a1 + b1 * b1) / 8.0
        - 0.5 * mu * mu * (0.5 * a0 * a0 + 0.375 * a1 * a1 + b1 * b1 / 8.0)
        - 0.5 * mu * inflow * a1
        + mu * a0 * b1 / 3.0)

    return RotorHubForces(
        thrust=thrust, h_force=h_force, y_force=y_force, torque=torque,
        inflow_ratio=inflow, advance_ratio=mu, drag_coefficient=delta,
        flapping_a1=a1, flapping_b1=b1, induced_velocity=induced,
        wind_u=u_c, wind_v=v_c, wind_along_axis=w_c,
        control_axis_pitch_rate=q_c, control_axis_roll_rate=p_c,
        ground_effect=ground_effect, evaluations=evaluations,
        converged=converged)


def tail_rotor_thrust(collective_rad, u_b, v_t, tail_t1=UH1_TAIL_T1):
    """TM-73254 equation 37: the tail rotor's thrust, N, starboard positive.

    ``collective_rad`` is theta_TR from the pedals, ``u_b`` the forward speed,
    m/s, and ``v_t`` the side wash at the tail rotor, m/s, from equation 38.
    The report calls this an "economical approximation to the tail rotor thrust
    of an isolated tail rotor" and warns that it has no vortex ring state and no
    fin interference; its T4 damping constant was then adjusted empirically,
    because the derivation behind it "grossly underpredicted" the damping seen
    in flight.

    The shape of it: at a standstill the thrust rises as
    ``[-T2 + sqrt(T2 ** 2 + T3 |theta|)] ** 2``, the isolated rotor result with
    the collective in place of a blade element integral, and the two further
    factors are the inflow it makes (through T1, the reciprocal of twice the
    tail rotor's tip speed) and the side wash's damping (through T4).

    ``tail_t1`` is T1 of table 3, the reciprocal of twice the tail rotor's tip
    speed, and is a parameter because the tail rotor's speed is the main rotor's
    through a fixed drive ratio: at 90 per cent of the reference it is 1 / 0.9
    times as large, and a rotor that is slowing down cannot go on pushing the
    same thrust per degree of pedal.
    """
    magnitude = abs(collective_rad)
    # theta_2 of equation 37: the collective the inflow factor is divided by,
    # held at a floor so that a centred pedal cannot divide by zero.
    theta2 = magnitude if magnitude > 0.0873 else 0.0873
    sign = 1.0 if collective_rad >= 0.0 else -1.0
    root = math.sqrt(UH1_TAIL_T2 * UH1_TAIL_T2 + UH1_TAIL_T3 * magnitude)
    standstill = (root - UH1_TAIL_T2) ** 2
    inflow = 1.0 + tail_t1 * u_b / theta2
    damping = UH1_TAIL_T4 + UH1_TAIL_T5 * abs(u_b)
    return sign * standstill * max(inflow, 0.0) - damping * v_t


def fuselage_forces(body_velocity):
    """TM-73254 equations 42 to 46: the fuselage's force and moment, body axes.

    The report derived these from wind tunnel data of a full scale UH-1
    fuselage (its reference 4), resolved into body axes with trigonometric
    approximations to the lift and drag curves, so the terms below are that
    resolution rather than a derivation::

        X_F = u_B (-D1 |u_B| + L1 w_B ** 2 / |u_B|)

    and its two siblings.  The division by ``|u_B|`` is the one place the
    printed form is not safe, and it is written here multiplied out as
    ``-D1 u_B |u_B| + L1 w_B ** 2 sign(u_B)``, which is what it means and stays
    defined in a hover.

    Returns ``(force, moment)``, N and N m.
    """
    u_b, v_b, w_b = body_velocity.x, body_velocity.y, body_velocity.z
    sign_u = 0.0
    if u_b > 0.0:
        sign_u = 1.0
    elif u_b < 0.0:
        sign_u = -1.0
    x_f = -UH1_FUSELAGE_D1 * u_b * abs(u_b) + UH1_FUSELAGE_LIFT * w_b * w_b * sign_u
    y_f = v_b * (-UH1_FUSELAGE_D2 * abs(v_b) - UH1_FUSELAGE_SIDE * abs(u_b))
    z_f = w_b * (-UH1_FUSELAGE_D3 * abs(w_b) - UH1_FUSELAGE_LIFT * abs(u_b))
    # M_F and N_F: a pitching moment from the fuselage's angle of attack and a
    # yawing moment from its sideslip.
    moment = Vector3(0.0,
                     UH1_FUSELAGE_MOMENT * w_b * abs(u_b),
                     -UH1_FUSELAGE_YAW_MOMENT * v_b * u_b)
    return Vector3(x_f, y_f, z_f), moment


def fin_forces(body_velocity, body_rates):
    """TM-73254 equations 47 to 53: the vertical fin's force and moment.

    ``u_F = u_B`` and ``v_F = -v_B + l_VF r_B``: the fin sits 25 ft aft of the
    c.g., so a yaw rate drives it sideways, exactly as the side wash drives the
    tail rotor in equation 38.

    The three branches are the report's stall model in wind axes, and they are
    why the fin does not simply reverse when the flow does: attached flow is
    ``k1 v_F u_F``, and past 20 deg the fin goes to a fully separated form in
    ``u_F ** 2``, with F1's drag term on top of either.  In the attached branch
    a positive yaw rate makes a force that opposes it, which is the fin's whole
    purpose on this helicopter: it is where the UH-1H's yaw damping starts
    before the tail rotor's own is added.

    Returns ``(force, moment, angle_deg)`` with the fin's own angle of attack.
    """
    u_f = body_velocity.x
    v_f = -body_velocity.y + UH1_FIN_ARM * body_rates.z
    speed = math.hypot(u_f, v_f)
    if speed <= 1e-9:
        return Vector3(), Vector3(), 0.0
    alpha = math.degrees(math.asin(max(-1.0, min(1.0, v_f / speed))))
    drag = UH1_FIN_F1 * v_f * abs(v_f)
    if -20.0 <= alpha <= 20.0 or 160.0 <= alpha <= 200.0:
        y_vf = UH1_FIN_K1 * v_f * u_f + drag                      # (50)
    elif 20.0 < alpha < 160.0:
        y_vf = UH1_FIN_K2 * u_f * u_f + drag                      # (51)
    else:
        y_vf = -UH1_FIN_K2 * u_f * u_f + drag                     # (52)
    return (Vector3(0.0, y_vf, 0.0),
            Vector3(0.0, 0.0, -y_vf * UH1_FIN_ARM),               # (53)
            alpha)


def stabilizer_forces(body_velocity, body_rates, inflow_ratio,
                      wind_along_axis, incidence_rad):
    """TM-73254 equations 54 to 60: the horizontal stabilizer's force, moment.

    The stabilizer flies in the rotor's downwash as well as in the free stream,
    which is the ``lambda / R7`` term of equation 54, and its incidence comes
    from table 1 through :func:`stabilizer_incidence_rad`, on the UH-1B schedule
    the report used in place of a UH-1H one.

    The three branches are the same stall model as the fin's, in the vertical
    plane, and ``M_H = Z_H l_HS`` turns the resulting force into a pitch moment
    on a 19.67 ft arm, which is what makes the stabilizer a pitch damper: a
    nose-up pitch rate q brings the tail down, which adds to w_H, which
    *reduces* the download this negative-incidence surface makes, and so makes
    the nose-down moment that opposes the rate.

    Two things about equation 54 are as printed and worth knowing.  First, it
    adds both ``w_B`` and ``w_C``, and ``w_C`` (equation 13) already contains
    ``-w_B``, so the vertical velocity is counted twice; second, lambda is
    negative in normal flight in this model, so the ``lambda / R7`` term is an
    *upward* flow at the stabilizer where the rotor's downwash is downward.  The
    text layer of the report is damaged here, so neither was "corrected": they
    are kept, and the demo prints what the term is worth at the trim hover -
    some tens of newtons, a couple of hundred newton metres of bias moment,
    which the trim absorbs in the cyclic.  On a surface whose lift term is
    ``H1 w_H u_H`` that bias is small next to the incidence term
    ``u_B delta_s``, which is the one the schedule drives.

    Returns ``(force, moment, angle_deg)`` with the stabilizer's own angle of
    attack, deg, which is what picks the branch.
    """
    u_b = body_velocity.x
    w_h = (body_velocity.z + inflow_ratio / UH1_ROTOR_R7
           + u_b * incidence_rad + UH1_STAB_ARM * body_rates.y
           + wind_along_axis)                                     # (54)
    u_h = u_b                                                      # (55)
    speed = math.hypot(u_h, w_h)
    if speed <= 1e-9:
        return Vector3(), Vector3(), 0.0
    alpha = math.degrees(math.asin(max(-1.0, min(1.0, w_h / speed))))
    drag = -UH1_STAB_H4 * w_h * abs(w_h)
    if -20.0 <= alpha <= 20.0 or 160.0 <= alpha <= 200.0:
        z_h = -UH1_STAB_H1 * w_h * u_h + drag                      # (57)
    elif 20.0 < alpha < 160.0:
        z_h = -UH1_STAB_H2 * u_h * u_h + drag                      # (58)
    else:
        z_h = UH1_STAB_H2 * u_h * u_h + drag                       # (59)
    return (Vector3(0.0, 0.0, z_h),
            Vector3(0.0, z_h * UH1_STAB_ARM, 0.0),                 # (60)
            alpha)


@dataclass
class BodyForces:
    """Everything TM-73254 equations 11 to 66 add up to, in body axes.

    ``force`` and ``moment`` are the totals applied at the c.g.: ``force`` in N
    with +x forward, +y starboard, +z down, and ``moment`` in N m with L
    rolling right, M pitching up and N yawing right.  Gravity is not in here:
    it is a body force the equations of motion add, since the report's model is
    aerodynamics only.  The pieces are kept so that a flight can be looked at
    component by component, which is how the report's own figures were compared
    with flight test data.
    """

    force: Vector3                 # N, body axes
    moment: Vector3                # N m, body axes
    controls: ControlPitch
    rotor: RotorHubForces
    rotor_force: Vector3           # X_R, Y_R, Z_R
    rotor_moment: Vector3          # L_R, M_R, N_R, the shaft torque included
    tail_thrust: float             # Y_TR, N, starboard positive
    tail_moment: Vector3
    fuselage_force: Vector3
    fuselage_moment: Vector3
    fin_force: Vector3
    fin_moment: Vector3
    fin_angle_deg: float
    stabilizer_force: Vector3
    stabilizer_moment: Vector3
    stabilizer_incidence_rad: float
    stabilizer_angle_deg: float
    side_wash: float               # v_T, m/s, the tail rotor's own side wash

    @property
    def lift(self):
        """N, the model's upward force: -Z_B, since +z is down."""
        return -self.force.z

    @property
    def yaw_balance(self):
        """N m, the yaw moment the pedals still have to answer."""
        return self.moment.z

    @property
    def airspeed(self):
        """The in-plane wind speed at the rotor, m/s, i.e. mu * omega R."""
        return self.rotor.in_plane_speed

    def __str__(self):
        return ("force (%+7.0f, %+7.0f, %+7.0f) N | moment (%+7.0f, %+7.0f,"
                " %+7.0f) N m | thrust %6.0f N, H %+6.0f, Y %+6.0f, Q %+7.0f"
                " | tail %+6.0f N | lambda %+6.3f, mu %5.3f"
                % (self.force.x, self.force.y, self.force.z,
                   self.moment.x, self.moment.y, self.moment.z,
                   self.rotor.thrust, self.rotor.h_force, self.rotor.y_force,
                   self.rotor.torque, self.tail_thrust,
                   self.rotor.inflow_ratio, self.rotor.advance_ratio))


def body_forces(controls, body_velocity, body_rates, air_density=RHO_SEA_LEVEL,
                rotor_height=1000.0,
                rotor_time_constant=UH1_ROTOR_TIME_CONSTANT_S,
                omega=UH1_OMEGA, constants=None):
    """The whole aircraft's force and moment, TM-73254 equations 11 to 66.

    ``controls`` is a :class:`ControlPitch`, ``body_velocity`` the aircraft's
    velocity in body axes, m/s, ``body_rates`` its (p, q, r), rad/s, and
    ``rotor_height`` the hub's height above the ground for the ground effect.
    ``omega`` is the rotor speed both rotors are turning at, the main rotor's
    directly and the tail rotor's through the drive ratio, and ``constants`` the
    table 3 set built once for it.

    The order is the report's.  The main rotor's T, H and Y are resolved into
    the control axis - wind system (equations 14 to 16), then into body axes
    through the control axis tilt (17 to 19); the tail rotor, fuselage, fin and
    stabilizer are added from their own functions; and the moments come from
    the hub's and tail rotor's positions (20 to 22 and 39 to 41) plus the
    fuselage's and stabilizer's own (45 and 60).

    The hub sits at ``(-x_c.g., 0, -hub height)`` in these axes - the c.g. is
    aft of and below it - and the tail rotor at ``(-l_TR, 0, -h_TR)``, aft of
    and above it.  Both are used as positions with the moments taken as
    ``r cross F`` rather than as the printed forms, and the self test asserts
    the two agree, which is what makes the signs of those six lines checkable.
    """
    if constants is None:
        constants = rotor_constants(omega)
    rotor = main_rotor_forces(controls, body_velocity, body_rates,
                              air_density=air_density,
                              rotor_height=rotor_height,
                              rotor_time_constant=rotor_time_constant,
                              constants=constants)
    u_b, v_b = body_velocity.x, body_velocity.y
    p_b, r_b = body_rates.x, body_rates.z
    b1s = controls.control_axis_long_rad
    a1s = controls.control_axis_lat_rad

    # Rotor forces in the control axis - wind system, equations 14 to 16.
    cos_c, sin_c = rotor.direction_cosines
    x_c = rotor.h_force * cos_c + rotor.y_force * sin_c         # (14)
    y_c = rotor.h_force * sin_c - rotor.y_force * cos_c         # (15)
    z_c = -rotor.thrust                                         # (16)
    rotor_force = Vector3(x_c - z_c * b1s,                      # (17)
                          y_c - z_c * a1s,                      # (18)
                          z_c + x_c * b1s + y_c * a1s)          # (19)
    # The hub's position from the c.g. and its moment.  The report writes
    # L_R = Y_R l_H, M_R = -X_R l_H + Z_R x_c.g. and N_R = Q - Y_R x_c.g.,
    # which is this cross product with the shaft torque added to it.
    hub_position = Vector3(-UH1_CG_AHEAD_OF_HUB, 0.0, -UH1_HUB_HEIGHT)
    rotor_moment = hub_position.cross(rotor_force) + Vector3(0.0, 0.0,
                                                             rotor.torque)

    # Tail rotor: the side wash at it (equation 38), then the thrust and the
    # moment it makes about the c.g. (39 to 41).
    side_wash = v_b - r_b * UH1_TAIL_ARM + p_b * UH1_TAIL_HEIGHT    # (38)
    tail_thrust = tail_rotor_thrust(controls.tail_collective_rad, u_b,
                                    side_wash, tail_t1=constants.tail_t1)
    tail_force = Vector3(0.0, tail_thrust, 0.0)                     # (39)
    tail_position = Vector3(-UH1_TAIL_ARM, 0.0, -UH1_TAIL_HEIGHT)
    tail_moment = tail_position.cross(tail_force)                   # (40, 41)

    fuselage_force, fuselage_moment = fuselage_forces(body_velocity)
    fin_force, fin_moment, fin_angle = fin_forces(body_velocity, body_rates)
    incidence = stabilizer_incidence_rad(controls.long_stick_in)
    stab_force, stab_moment, stab_angle = stabilizer_forces(
        body_velocity, body_rates, rotor.inflow_ratio,
        rotor.wind_along_axis, incidence)

    force = rotor_force + tail_force + fuselage_force + fin_force + stab_force
    moment = rotor_moment + tail_moment + fuselage_moment + fin_moment \
        + stab_moment
    return BodyForces(
        force=force, moment=moment, controls=controls, rotor=rotor,
        rotor_force=rotor_force, rotor_moment=rotor_moment,
        tail_thrust=tail_thrust, tail_moment=tail_moment,
        fuselage_force=fuselage_force, fuselage_moment=fuselage_moment,
        fin_force=fin_force, fin_moment=fin_moment, fin_angle_deg=fin_angle,
        stabilizer_force=stab_force, stabilizer_moment=stab_moment,
        stabilizer_incidence_rad=incidence, stabilizer_angle_deg=stab_angle,
        side_wash=side_wash)


# ---------------------------------------------------------------------------
# The rigid body: state, kinematics and the equations of motion.
# ---------------------------------------------------------------------------


def direction_cosine_matrix(roll, pitch, yaw):
    """The body to NED matrix, the usual 3-2-1 Euler sequence.

    A rotation through the yaw angle about -z, then pitch about +y, then roll
    about +x, which for these axes (NED earth, x nose, y starboard, z down)
    gives the standard result::

        [ cos t cos p,  sin r sin t cos p - cos r sin p,  cos r sin t cos p + sin r sin p ]
        [ cos t sin p,  sin r sin t sin p + cos r cos p,  cos r sin t sin p - sin r cos p ]
        [ -sin t,       sin r cos t,                      cos r cos t                    ]

    with roll = r, pitch = t, yaw = p for this one expression.  Multiply it by a
    body vector to get NED, or by its transpose to get body from NED.
    """
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_t, sin_t = math.cos(pitch), math.sin(pitch)
    cos_p, sin_p = math.cos(yaw), math.sin(yaw)
    return (
        (cos_t * cos_p, sin_r * sin_t * cos_p - cos_r * sin_p,
         cos_r * sin_t * cos_p + sin_r * sin_p),
        (cos_t * sin_p, sin_r * sin_t * sin_p + cos_r * cos_p,
         cos_r * sin_t * sin_p - sin_r * cos_p),
        (-sin_t, sin_r * cos_t, cos_r * cos_t))


def transpose(matrix):
    """Transpose of a 3 by 3, so that a body vector can go to NED and back."""
    return tuple(tuple(matrix[row][column] for row in range(3))
                 for column in range(3))


def apply(matrix, vector):
    """Multiply a 3 by 3 by a vector: ``matrix . vector``."""
    return Vector3(matrix[0][0] * vector.x + matrix[0][1] * vector.y
                   + matrix[0][2] * vector.z,
                   matrix[1][0] * vector.x + matrix[1][1] * vector.y
                   + matrix[1][2] * vector.z,
                   matrix[2][0] * vector.x + matrix[2][1] * vector.y
                   + matrix[2][2] * vector.z)


def euler_rates(attitude, rates):
    """The attitude derivative ``(roll_dot, pitch_dot, yaw_dot)``, rad/s.

    The standard kinematic relation between the body rates and the Euler
    angles, with the one singularity these axes have at 90 deg of pitch, which
    a helicopter does not reach in normal flight and which is clamped here
    rather than allowed to divide by zero.
    """
    roll, pitch = attitude.x, attitude.y
    p, q, r = rates.x, rates.y, rates.z
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_t = math.cos(pitch)
    if abs(cos_t) < 1e-6:
        cos_t = 1e-6 if cos_t >= 0.0 else -1e-6
    return Vector3(p + (q * sin_r + r * cos_r) * math.tan(pitch),
                   q * cos_r - r * sin_r,
                   (q * sin_r + r * cos_r) / cos_t)


@dataclass
class FlightState:
    """Where the helicopter is and how it is moving.

    ``position`` is NED metres: north, east and down, so altitude is
    ``-position.z``.  ``velocity`` is the aircraft's velocity in *body* axes,
    m/s, with +u forward, +v starboard and +w down, so a climb has a negative
    w.  ``attitude`` is the Euler triple (roll, pitch, yaw) in radians, roll
    right and pitch up positive, yaw measured from north towards east; and
    ``rates`` is the body rate triple (p, q, r), rad/s, with the same senses:
    p rolling right, q pitching up, r yawing right.

    Both frames appear in one object because that is how the equations of
    motion are written: forces are summed in body axes, where the airframe is
    fixed, and the position is integrated in NED, where the ground is.

    ``rotor_speed`` is the main rotor's angular velocity, rad/s, and is the
    thirteenth state variable: a rotor is a flywheel, and its speed is as much
    a part of where the aircraft is as its pitch rate is.  It is the last field
    so that every state that does not care about it - which is every state until
    a torque balance is flown - gets the 324 rpm reference and the fixed rotor of
    the report's model with it.
    """

    position: Vector3 = field(default_factory=Vector3)
    velocity: Vector3 = field(default_factory=Vector3)
    attitude: Vector3 = field(default_factory=Vector3)
    rates: Vector3 = field(default_factory=Vector3)
    rotor_speed: float = UH1_OMEGA

    def copy(self):
        """A copy, so that a trim or an integrator can shuffle states about."""
        return FlightState(self.position, self.velocity, self.attitude,
                           self.rates, self.rotor_speed)

    def values(self):
        """The thirteen state variables as a tuple, for a Runge-Kutta step."""
        return (self.position.x, self.position.y, self.position.z,
                self.velocity.x, self.velocity.y, self.velocity.z,
                self.attitude.x, self.attitude.y, self.attitude.z,
                self.rates.x, self.rates.y, self.rates.z,
                self.rotor_speed)

    @classmethod
    def from_values(cls, values):
        """Rebuild a state from :meth:`values`."""
        return cls(Vector3(*values[0:3]), Vector3(*values[3:6]),
                   Vector3(*values[6:9]), Vector3(*values[9:12]),
                   values[12])

    @property
    def rotor_rpm(self):
        """The main rotor's speed in rpm, the unit its gauge is marked in."""
        return self.rotor_speed * 60.0 / (2.0 * math.pi)

    @property
    def altitude(self):
        """Height above the ground, m, from the NED down coordinate."""
        return -self.position.z

    @property
    def rotor_height(self):
        """Height of the main rotor hub above the ground, m.

        The hub stands UH1_HUB_HEIGHT above the c.g. along the body z axis, so
        a rolled or pitched aircraft has its hub a little lower than that; the
        cosine of both angles projects it onto the vertical.
        """
        return (self.altitude + UH1_HUB_HEIGHT
                * math.cos(self.attitude.x) * math.cos(self.attitude.y))

    @property
    def speed(self):
        """Airspeed through the airframe, m/s, i.e. |velocity|."""
        return self.velocity.length()

    @property
    def attitude_deg(self):
        """The Euler triple in degrees, for printing."""
        return Vector3(math.degrees(self.attitude.x),
                       math.degrees(self.attitude.y),
                       math.degrees(self.attitude.z))

    @property
    def rates_deg(self):
        """The body rates in degrees per second, for printing."""
        return Vector3(math.degrees(self.rates.x),
                       math.degrees(self.rates.y),
                       math.degrees(self.rates.z))

    def ground_velocity(self):
        """Velocity over the ground in NED, m/s, from the body velocity."""
        return apply(direction_cosine_matrix(self.attitude.x, self.attitude.y,
                                             self.attitude.z), self.velocity)

    def wind_axes(self):
        """``(angle of attack, sideslip)``, deg, of the airframe.

        Both are zero when there is no air motion; alpha is positive with the
        flow coming from below the nose, which is the sense that makes a
        descending forward flight a positive angle of attack.
        """
        u, v, w = self.velocity.x, self.velocity.y, self.velocity.z
        speed = math.sqrt(u * u + v * v + w * w)
        if speed <= 1e-9:
            return 0.0, 0.0
        return (math.degrees(math.atan2(w, u)),
                math.degrees(math.asin(max(-1.0, min(1.0, v / speed)))))

    def __str__(self):
        return ("N %+8.1f m, E %+8.1f m, alt %6.1f m | body u %+6.1f, v %+6.1f,"
                " w %+6.1f m/s | roll %+6.1f, pitch %+6.1f, yaw %+6.1f deg |"
                " p %+6.1f, q %+6.1f, r %+6.1f deg/s | rotor %3.0f %% (%4.0f"
                " rpm)"
                % (self.position.x, self.position.y, self.altitude,
                   self.velocity.x, self.velocity.y, self.velocity.z,
                   *self.attitude_deg.as_tuple(), *self.rates_deg.as_tuple(),
                   100.0 * self.rotor_speed / UH1_OMEGA, self.rotor_rpm))


@dataclass
class Airframe:
    """The whole aircraft: control path, aerodynamics and equations of motion.

    The frame loop is three calls, in the order the signal travels:

    1. :meth:`command_angles` mixes the stick positions into control angles
       (:class:`rotor_control.MixingLaw`): where the pilot asked the swashplate
       to be;
    2. :meth:`advance_controls` runs the first order lags
       (:class:`rotor_control.ControlLags`) and reads the *control axis* out of
       the lagged blade cyclic - the quarter turn of
       :mod:`rotor_control` read backwards, which is why the two modules meet
       without either knowing the other's convention - then converts to the
       pitch the report wants (:class:`ControlPitch`);
    3. :meth:`derivatives` asks :func:`body_forces` what that does to the
       aircraft, adds gravity, and returns the state derivative.

    :meth:`step` does all three and integrates one frame with a fourth order
    Runge-Kutta step, holding the control angles fixed across it, which is what
    a real time simulator does at 60 Hz.  Each force evaluation is a closed
    form, so a frame costs four of them.

    The defaults are the report's own: 8700 lb, its inertia matrix, and its
    constants.  ``rotor_time_constant`` is the one place a choice has to be
    made, and it is the same choice :class:`rotor_control.ControlLags` makes:
    the physical 0.072 sec from one blade's inertia by default, with the 0.144
    sec of the report's simulation available for reproducing its step
    responses.  ``ground_effect`` can be turned off to fly out of the cushion.

    ``rotor_inertia`` and ``engine_torque`` are what make the rotor speed a
    state rather than a constant, and by default they do not: ``engine_torque``
    is ``None``, which means the engine holds whatever the rotor is doing, so
    :attr:`FlightState.rotor_speed` never changes and this is exactly the
    report's fixed rotor.  Give it a number in N m and a shaft balance appears
    in :meth:`derivatives`: the rotor accelerates at ``(Q_engine - Q_drag) /
    I``, with ``Q_drag`` the torque equation 4 of the report gives it.  What
    turns the rotor is then the collective in the pilot's hand, which is the
    physics of an engine failure, of a governor that lags, and of nothing at
    all once the torque is set to zero and the rotor is left to windmill.

    An ``engine`` is the same shaft balance with something real on the other
    end of it: :class:`engine.Engine` is asked for its torque once per frame,
    from the rotor speed at the start of the frame, and that number is written
    into ``engine_torque`` for :meth:`rotor_acceleration` to use - so the two
    ways of driving the shaft meet in one field and a fixed torque and a T53
    are told apart by nothing but where the number came from.  With an engine
    the throttle is a pilot control (:attr:`throttle`, and
    :class:`simulation.PilotInput`), and the rotor speed becomes a free state
    the governor holds rather than a constant the model holds: the aircraft
    settles a little under its selected speed, by however much of the 40 rpm
    droop band its power demand is worth.
    """

    mass: float = UH1_SIM_MASS
    inertia: Tuple[float, float, float, float] = (UH1_IXX, UH1_IYY, UH1_IZZ,
                                                  UH1_IXZ)
    law: MixingLaw = field(default_factory=MixingLaw)
    lags: ControlLags = field(default_factory=ControlLags)
    state: FlightState = field(default_factory=FlightState)
    air_density: float = RHO_SEA_LEVEL
    twist_deg: float = UH1_TWIST_DEG
    rotor_time_constant: float = UH1_ROTOR_TIME_CONSTANT_S
    ground_effect: bool = True
    rotor_inertia: float = UH1_ROTOR_INERTIA
    #: Torque at the main rotor shaft, N m, for this frame: ``None`` means
    #: there is no shaft balance to fly and the report's fixed rotor is what
    #: this airframe is.  A *number* is a torque an engine is delivering, and
    #: an :attr:`engine` overwrites it every frame with the one it is.
    engine_torque: Optional[float] = None
    #: The engine: ``None`` by default, which is the report's fixed rotor to
    #: the last bit.  Give it an :class:`engine.Engine` and the shaft balance
    #: is driven by a T53 with a governor, the throttle comes in through the
    #: pilot's hand, and an engine failure is a torque that goes away.
    engine: Optional[Engine] = None

    @property
    def throttle(self):
        """The twist grip, 0 to 1, on whichever governor is there.

        Without an engine there is no governor to roll, so this is always 1:
        the aircraft of TM-73254 has no engine in the model at all and its
        rotor is held at whatever speed the state has it at.  With one, this
        is :attr:`engine.Governor.throttle` seen from outside, which is what
        :mod:`simulation`'s :class:`simulation.PilotInput` sets.
        """
        return self.engine.governor.throttle if self.engine is not None else 1.0

    @throttle.setter
    def throttle(self, value):
        if self.engine is not None:
            self.engine.governor.throttle = float(value)

    def command_angles(self, controls):
        """The mixing stage on its own: stick positions in, control angles out."""
        return self.law.mix(controls)

    def advance_controls(self, dt, controls):
        """Mix, run the lags, and hand back the pitch the rotor actually has.

        Returns ``(ControlPitch, ControlAngles)``: the report's own control
        variables, and the lagged angles they came from for the record.  The
        control axis is read back out of the lagged blade cyclic by taking the
        two coefficients the other way round, which is the quarter turn of
        :mod:`rotor_control` run backwards: its ``cyclic_long_deg`` is the pitch
        over the nose, and so the *starboard* tilt, and its ``cyclic_lat_deg`` is
        the pitch to port, and so the forward one.  Lagging commutes with that
        fixed recolouring of the two numbers, so the lagged blade cyclic is
        exactly the lagged control axis, swapped.

        The report's A1S and B1S are the same physical directions as
        :mod:`rotor_control`'s lateral and longitudinal tilts - starboard and
        forward - even though this module's +y is starboard and that module's is
        port, so no sign is needed here, only the swap.
        """
        seen = self.lags.step(dt, self.command_angles(controls))
        return (self.control_pitch_of(seen, controls.long_stick), seen)

    def control_pitch_of(self, seen, long_stick_in=0.0):
        """The report's :class:`ControlPitch` for lagged mixing angles *seen*.

        One place, because there are two callers: :meth:`advance_controls` with
        the angles the rotor is at after the lags, and :meth:`reset` with the
        angles the trim put the lags on.  See :meth:`advance_controls` for why
        the two cyclic numbers are swapped on the way through.  *long_stick_in*
        is the pilot's stick position, which is not part of the mixing angles
        and is carried into the force model on its own.
        """
        return ControlPitch(
            collective_rad=report_collective_rad(seen.collective_pitch_deg,
                                                 self.twist_deg),
            control_axis_long_rad=math.radians(seen.cyclic_lat_deg),
            control_axis_lat_rad=math.radians(seen.cyclic_long_deg),
            tail_collective_rad=math.radians(seen.tail_collective_deg),
            long_stick_in=long_stick_in)

    def forces(self, control_pitch, state=None, wind=None, air_density=None):
        """The report's forces and moments for one instant, in body axes.

        *air_density* defaults to the airframe's own; a trim at some other
        density is what asks for another.  The table 3 constants are taken at
        the state's own rotor speed, so a rotor that is turning slower makes
        less thrust for the collective in the pilot's hand, which is the whole
        of what the rotor speed state does to the airframe.
        """
        state = self.state if state is None else state
        return body_forces(
            control_pitch, self.air_relative_velocity(state, wind),
            state.rates,
            air_density=(self.air_density if air_density is None
                         else air_density),
            rotor_height=(state.rotor_height if self.ground_effect
                          else 10.0 * 2.0 * UH1_RADIUS),
            rotor_time_constant=self.rotor_time_constant,
            omega=state.rotor_speed)

    def air_relative_velocity(self, state, wind=None):
        """The aircraft's velocity relative to the air, in body axes, m/s.

        ``wind`` is a steady air velocity in NED, m/s, and is zero by default:
        with no wind this is simply the body velocity, since the air is assumed
        still.  A wind is turned into the body axes with the transpose of the
        direction cosine matrix, which is the same rotation the other way round.
        """
        if wind is None:
            return state.velocity
        matrix = transpose(direction_cosine_matrix(state.attitude.x,
                                                   state.attitude.y,
                                                   state.attitude.z))
        return state.velocity - apply(matrix, wind)

    def angular_acceleration(self, moment, rates):
        """``(p_dot, q_dot, r_dot)``, rad/s^2, from the moment and the rates.

        Euler's equations for a body with the UH-1H's inertia matrix - Ixx,
        Iyy, Izz and one product Ixz, in the body axis order of this module::

            L = Ixx p' - Ixz r' - Ixz p q + (Izz - Iyy) q r
            M = Iyy q' + Ixz (p ** 2 - r ** 2) + (Ixx - Izz) r p
            N = Izz r' - Ixz p' + (Iyy - Ixx) p q + Ixz q r

        Only p' and r' are coupled, through the product of inertia, so the pitch
        acceleration falls straight out and the roll and yaw pair solve a 2 by 2
        - which is where the UH-1H's habit of rolling when it yaws comes from.
        """
        ixx, iyy, izz, ixz = self.inertia
        p, q, r = rates.x, rates.y, rates.z
        roll_rate = (moment.x + ixz * p * q - (izz - iyy) * q * r)
        yaw_rate = (moment.z - (iyy - ixx) * p * q - ixz * q * r)
        solution = solve_linear([[ixx, -ixz], [-ixz, izz]],
                                [roll_rate, yaw_rate])
        if solution is None:
            solution = [0.0, 0.0]
        pitch_rate = ((moment.y - ixz * (p * p - r * r)
                       - (ixx - izz) * r * p) / iyy)
        return Vector3(solution[0], pitch_rate, solution[1])

    def derivatives(self, state, control_pitch, wind=None):
        """The state derivative, and the forces it came from.

        The equations of motion in these axes, with gravity resolved into the
        body frame: a nose-up attitude puts a component of gravity aft, which is
        the whole reason a helicopter has to lean to fly anywhere::

            u' = r v - q w - g sin(theta) + X_B / m
            v' = p w - r u + g sin(phi) cos(theta) + Y_B / m
            w' = q u - p v + g cos(phi) cos(theta) + Z_B / m

        Returns ``(FlightState, BodyForces)``: the derivative as a state of the
        same shape, and the forces behind it, which is what a caller watching a
        transient wants to see.  The thirteenth derivative is the rotor's own,
        from :meth:`rotor_acceleration`.
        """
        forces = self.forces(control_pitch, state, wind=wind)
        mass = self.mass
        u, v, w = state.velocity.x, state.velocity.y, state.velocity.z
        p, q, r = state.rates.x, state.rates.y, state.rates.z
        roll, theta = state.attitude.x, state.attitude.y
        cos_r, sin_r = math.cos(roll), math.sin(roll)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        total = forces.force
        acceleration = Vector3(
            total.x / mass + r * v - q * w - GRAVITY * sin_t,
            total.y / mass + p * w - r * u + GRAVITY * sin_r * cos_t,
            total.z / mass + q * u - p * v + GRAVITY * cos_r * cos_t)
        return (FlightState(
            position=self.ground_velocity_of(state),
            velocity=acceleration,
            attitude=euler_rates(state.attitude, state.rates),
            rates=self.angular_acceleration(forces.moment, state.rates),
            rotor_speed=self.rotor_acceleration(forces, state)),
            forces)

    def rotor_acceleration(self, forces, state):
        """``omega_dot``, rad/s^2: the shaft balance, ``(Q_engine - Q) / I``.

        ``Q`` is the torque the report's equation 4 says the rotor is absorbing
        - positive in normal flight, since the engine has to drive it - so with
        an engine torque smaller than it the rotor slows down, which is what an
        autorotation is, and with a larger one it speeds up, which is what
        raising the collective with a fixed throttle setting does.

        The default ``engine_torque`` of ``None`` means there is no such balance
        to fly: the engine is taken to hold the rotor at whatever speed the
        state has it at, so this is zero and the model is the report's fixed
        rotor to the last bit.  That is deliberate - every figure in TM-73254
        was computed with the rotor speed held constant, and the simulator this
        came from has to be able to reproduce them.

        A number here is a torque on the shaft, and with an
        :class:`engine.Engine` aboard this method's field is written by
        :meth:`step` once a frame, so the difference between a fixed torque and
        a T53 is nothing but where the number came from.
        """
        if self.engine_torque is None:
            return 0.0
        return (self.engine_torque - forces.rotor.torque) / self.rotor_inertia

    def ground_velocity_of(self, state):
        """The NED velocity of a state, which its position integrates."""
        return apply(direction_cosine_matrix(state.attitude.x, state.attitude.y,
                                             state.attitude.z), state.velocity)

    def step(self, dt, controls, wind=None, throttle=None):
        """Fly one frame of *dt* seconds with *controls* held: the frame loop.

        Mixes the controls, runs the lags by *dt*, converts to the report's
        control pitch, and integrates the equations of motion over the frame
        with a fourth order Runge-Kutta step, holding the control pitch fixed
        across it - the pilot's hands do not move inside one frame.  The state
        is advanced in place and the forces at the end of the frame are
        returned, since a renderer or a HUD wants them and they cost one more
        evaluation than the step itself.

        *throttle*, 0 to 1, is the twist grip: it is written onto the engine's
        governor when there is an engine, and ignored when there is not (which
        is also what leaving it ``None`` does).  With an engine the frame
        begins by asking it for a torque at the rotor speed the frame started
        at, and that torque is held across the four Runge-Kutta stages exactly
        as the control pitch is: a gas producer does not spool up inside one
        sixteenth of a second, and a frame loop that let it would be
        integrating an engine it never modelled.
        """
        if throttle is not None:
            self.throttle = throttle
        control_pitch, _ = self.advance_controls(dt, controls)
        if self.engine is not None:
            self.engine_torque = self.engine.advance(
                dt, self.state.rotor_rpm, self.air_density)
        if dt <= 0.0:
            return self.forces(control_pitch, wind=wind)
        state = self.state
        initial = state.values()
        count = len(initial)

        def slope(values):
            derivative, _ = self.derivatives(FlightState.from_values(values),
                                             control_pitch, wind=wind)
            return derivative.values()

        k1 = slope(initial)
        k2 = slope(tuple(initial[i] + 0.5 * dt * k1[i] for i in range(count)))
        k3 = slope(tuple(initial[i] + 0.5 * dt * k2[i] for i in range(count)))
        k4 = slope(tuple(initial[i] + dt * k3[i] for i in range(count)))
        self.state = FlightState.from_values(
            tuple(initial[i] + dt * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i]
                                     + k4[i]) / 6.0 for i in range(count)))
        return self.forces(control_pitch, wind=wind)

    def reset(self, state=None, controls=None):
        """Put the aircraft on a state, and the lags on the matching angles.

        Resetting the lags to the trim angles matters: a fresh
        :class:`rotor_control.ControlLags` starts at zero pitch, so a flight
        that began without this would spend its first half second pulling the
        collective up from nothing.

        The engine, when there is one, is put on the torque the rotor is
        *actually absorbing* at the state being reset to - the shaft balance
        this method can evaluate and no other - so a run starts in equilibrium
        and the first frames are a flight rather than an engine spooling up
        behind it.  From there the governor walks it to wherever its own droop
        band puts it, which is the 1 rpm or so of sag a hovering UH-1 has.
        """
        if state is not None:
            self.state = state
        if controls is not None:
            self.lags.reset(self.command_angles(controls))
        if self.engine is not None:
            if controls is None:
                # No stick positions to evaluate a shaft balance at, so the
                # engine goes to whatever its governor is asking for at that
                # rotor speed: the best a reset without controls can do.
                self.engine_torque = self.engine.settle(self.state.rotor_rpm,
                                                        self.air_density)
            else:
                angles = self.lags.step(0.0, self.command_angles(controls))
                self.engine_torque = self.engine.settle_on(
                    self.forces(self.control_pitch_of(angles,
                                                      controls.long_stick),
                                state=self.state).rotor.torque,
                    self.state.rotor_rpm)
        return self

    def trim_hover(self, weight=None, air_density=None, altitude=200.0,
                   iterations=40, tolerance=0.1, rotor_speed=UH1_OMEGA):
        """The stick positions and attitude that hold a still hover.

        Six unknowns - collective, pedals, both cyclic sticks and the *pitch and
        roll attitudes* - against six residuals: the three body forces have to
        be balanced by the three components of gravity, and the three moments
        have to vanish.  That is a real hover and not just "thrust equals
        weight", and both of the attitudes are there for a reason:

        * the pitch one is the c.g. 6.2 in aft of the rotor hub, which makes the
          thrust lean forward by ``atan(x_c.g. / l_H)``, 4.4 deg of ``B1S``, to
          keep the pitching moment zero; the aircraft then hangs nose up by the
          same angle and the forward force is balanced by gravity.  A UH-1H in a
          hover is flown exactly like that;
        * the roll one is the tail rotor pushing 1.9 kN of thrust to starboard:
          the lateral cyclic can zero the roll moment or the side force but not
          both, so what is left, about 25 N, is balanced by a few hundredths of
          a degree of roll.  Without it a "trimmed" hover slides sideways at a
          centimetre a second.

        The solve is a damped Newton on a numerical Jacobian, in inches for the
        three sticks and radians for the two attitudes, and the residual is
        allowed *tolerance* newtons or newton metres because this is a model
        whose constants were fitted to flight test data, not an identity.

        Returns ``(PilotControls, FlightState)``: what to hold, and the state it
        holds.  Feed both to :meth:`reset` and the next ``step`` starts from a
        trimmed hover rather than dropping onto it.

        *rotor_speed* is the rotor speed the trim is asked for, rad/s, and
        defaults to the 324 rpm reference.  A hover on a slowing rotor is a real
        condition and not a curiosity - it is what a pilot sees on the way down
        in an autorotation - so the trim comes back with the state carrying it
        and with the collective raised to make up for the thrust the rotor is no
        longer making per degree.  The shaft balance is *not* closed here: with
        an ``engine_torque`` set, a trim at some rotor speed is an equilibrium of
        the airframe and not of the engine, and the two are closed together only
        by a rotor speed trim, which is not this.
        """
        air_density = self.air_density if air_density is None else air_density
        if weight is None:
            weight = self.mass * GRAVITY
        law = self.law
        constants = rotor_constants(rotor_speed)
        per_in = math.degrees(law.collective_per_in)
        # A first guess from equation 1 with no inflow at all: T = R1 theta_0/3,
        # turned back into a root collective and then into inches.
        report_rad = 3.0 * weight / constants.r1
        guess_deg = (math.degrees(report_rad) - 0.75 * self.twist_deg)
        collective_in = (guess_deg - law.collective_neutral_deg) / per_in
        # unknowns: collective in, pedal in, long stick in, lat stick in,
        # pitch rad, roll rad
        guess = [collective_in, 0.0, 0.0, 0.0,
                 -math.atan2(-UH1_CG_AHEAD_OF_HUB, UH1_HUB_HEIGHT), 0.0]
        position = Vector3(0.0, 0.0, -altitude)

        def residual(values):
            controls = PilotControls(collective=values[0], pedal=values[1],
                                     long_stick=values[2], lat_stick=values[3])
            attitude = Vector3(values[5], values[4], 0.0)
            trial = FlightState(position=position, velocity=Vector3(),
                                attitude=attitude, rates=Vector3(),
                                rotor_speed=rotor_speed)
            pitch = ControlPitch.from_angles(law.mix(controls), values[2],
                                             self.twist_deg)
            forces = body_forces(pitch, Vector3(), Vector3(),
                                 air_density=air_density,
                                 rotor_height=trial.rotor_height,
                                 rotor_time_constant=self.rotor_time_constant,
                                 constants=constants)
            theta, phi = values[4], values[5]
            return [forces.force.x - weight * math.sin(theta),
                    forces.force.y + weight * math.sin(phi) * math.cos(theta),
                    forces.force.z + weight * math.cos(phi) * math.cos(theta),
                    forces.moment.x, forces.moment.y, forces.moment.z]

        for _ in range(max(int(iterations), 4)):
            values = residual(guess)
            if max(abs(value) for value in values) < tolerance:
                break
            jacobian = [[0.0] * 6 for _ in range(6)]
            for column in range(6):
                step = 1e-5 if column >= 4 else 0.01
                shifted = list(guess)
                shifted[column] += step
                delta = residual(shifted)
                for row in range(6):
                    jacobian[row][column] = (delta[row] - values[row]) / step
            correction = solve_linear(jacobian, [-value for value in values])
            if correction is None:
                break
            guess = [guess[index] + max(-3.0, min(3.0, correction[index]))
                     for index in range(6)]

        controls = PilotControls(collective=guess[0], pedal=guess[1],
                                 long_stick=guess[2], lat_stick=guess[3])
        state = FlightState(position=position, velocity=Vector3(),
                            attitude=Vector3(guess[5], guess[4], 0.0),
                            rates=Vector3(), rotor_speed=rotor_speed)
        return controls, state

    def trim_level_flight(self, airspeed, weight=None, air_density=None,
                          altitude=200.0, iterations=40, tolerance=0.1,
                          rotor_speed=UH1_OMEGA):
        """The stick positions and attitude that hold level flight at *airspeed*.

        Level flight is a condition on the *path*, not on the attitude: the
        flight path is horizontal and nothing is accelerating along it, so in
        earth axes the velocity is ``(airspeed, 0, 0)`` - level flight north,
        which is a heading anyone can rotate to - and the three components of
        the net force have to vanish.  Both attitudes are unknowns again, and
        this time the pitch is not geometry: a hover hangs nose up by the angle
        that puts its hub over the c.g., while at speed the thrust leans forward
        to carry the drag and the attitude that goes with it is part of what is
        solved for, along with the flapping and the fuselage's own moments.

        Six unknowns - collective, pedals, both cyclic sticks, pitch and roll -
        against six residuals: the net force in earth axes, gravity included,
        which is zero for a steady flight path, and the three body moments,
        which are zero for a steady attitude.  The roll is in there for the
        hover's reason (:meth:`trim_hover`): the tail rotor pushes to starboard
        and the lateral cyclic can zero the roll moment or the side force but
        not both.

        The velocity the air sees is the flight path turned into body axes, so
        the sideslip is zero by construction and the angle of attack falls out
        of the pitch - the wind axis the report's own figures are flown along.
        The solve is the same damped Newton on a numerical Jacobian as
        :meth:`trim_hover`, whose answer at zero airspeed this reproduces (the
        self test asserts the two agree), and *tolerance* is in newtons and
        newton metres for the same reason: this is a model whose constants were
        fitted to flight test data, not an identity.

        Returns ``(PilotControls, FlightState)``, and the report's own level
        flight condition of its figures 2 to 9 is ``60 * KNOT`` m/s.  A speed
        the model cannot hold does not raise: the solve returns whatever its last
        step left, and :meth:`equilibrium_residual` says how far that is from a
        trim.

        *rotor_speed* is the rotor speed the condition is asked at, rad/s, the
        same knob :meth:`trim_hover` has and for the same reason.
        """
        air_density = self.air_density if air_density is None else air_density
        if weight is None:
            weight = self.mass * GRAVITY
        law = self.law
        constants = rotor_constants(rotor_speed)
        per_in = math.degrees(law.collective_per_in)
        # A first guess from equation 1 with no inflow at all: the collective
        # that carries the weight in a hover.  At 60 kt that is within a per
        # cent of the thrust actually needed there, because the drag is a couple
        # of hundredths of the weight and the rotor carries it by leaning.
        report_rad = 3.0 * weight / constants.r1
        guess_deg = (math.degrees(report_rad) - 0.75 * self.twist_deg)
        collective_in = (guess_deg - law.collective_neutral_deg) / per_in
        # unknowns: collective in, pedal in, long stick in, lat stick in,
        # pitch rad, roll rad - and the attitude starts level, which is a few
        # degrees from the hover's 4.4 deg and from the trim of any speed, so
        # that the first Newton step is short in both of them.
        guess = [collective_in, 0.0, 0.0, 0.0, 0.0, 0.0]
        position = Vector3(0.0, 0.0, -altitude)

        def trial_state(values):
            """The state a guess describes: level, north, at the airspeed asked."""
            matrix = direction_cosine_matrix(values[5], values[4], 0.0)
            return FlightState(
                position=position,
                velocity=apply(transpose(matrix), Vector3(airspeed, 0.0, 0.0)),
                attitude=Vector3(values[5], values[4], 0.0), rates=Vector3(),
                rotor_speed=rotor_speed)

        def residual(values):
            controls = PilotControls(collective=values[0], pedal=values[1],
                                     long_stick=values[2], lat_stick=values[3])
            force, moment = self.equilibrium_residual(
                controls, trial_state(values), weight=weight,
                air_density=air_density)
            return [force.x, force.y, force.z, moment.x, moment.y, moment.z]

        for _ in range(max(int(iterations), 4)):
            values = residual(guess)
            if max(abs(value) for value in values) < tolerance:
                break
            jacobian = [[0.0] * 6 for _ in range(6)]
            for column in range(6):
                step = 1e-5 if column >= 4 else 0.01
                shifted = list(guess)
                shifted[column] += step
                delta = residual(shifted)
                for row in range(6):
                    jacobian[row][column] = (delta[row] - values[row]) / step
            correction = solve_linear(jacobian, [-value for value in values])
            if correction is None:
                break
            guess = [guess[index] + max(-3.0, min(3.0, correction[index]))
                     for index in range(6)]

        controls = PilotControls(collective=guess[0], pedal=guess[1],
                                 long_stick=guess[2], lat_stick=guess[3])
        return controls, trial_state(guess)

    def equilibrium_residual(self, controls, state, weight=None,
                             air_density=None, wind=None):
        """``(force, moment)``: how far a state is from flying itself.

        *force* is the net force in **earth axes**, newtons, gravity included,
        and *moment* the body moments, newton metres.  Both are zero at any
        steady condition - a hover, a climb, level flight - because a state with
        no acceleration along its path and no rate of change of attitude is
        exactly one whose net force, turned into the frame the path is drawn in,
        vanishes.  That is what :meth:`trim_hover` and
        :meth:`trim_level_flight` minimise, six unknowns at a time in both, and
        it is also what a caller checks one with: a trim asked for a condition
        the model cannot hold comes back with a residual around it rather than as
        an error, and this is what says how big.

        *weight* defaults to the aircraft's own and *air_density* to its air, so
        that a trim at another density is checked with the same numbers it was
        solved for.  The state's rates are not part of the balance: a trim is a
        state with none.  The forces come from the *commanded* angles rather
        than from the lags, so this does not depend on whether the lags have
        been reset yet - the same choice the two trims make.
        """
        if weight is None:
            weight = self.mass * GRAVITY
        control_pitch = ControlPitch.from_angles(self.command_angles(controls),
                                                 controls.long_stick,
                                                 self.twist_deg)
        forces = self.forces(control_pitch, state, wind=wind,
                             air_density=air_density)
        roll, theta = state.attitude.x, state.attitude.y
        # Gravity in body axes, written the way :meth:`derivatives` writes it,
        # so that the net force can be turned into earth axes, where the
        # condition "steady flight path" is three zeros.
        net = forces.force + Vector3(-weight * math.sin(theta),
                                     weight * math.sin(roll) * math.cos(theta),
                                     weight * math.cos(roll) * math.cos(theta))
        matrix = direction_cosine_matrix(roll, theta, state.attitude.z)
        return apply(matrix, net), forces.moment


def _self_test():
    """Checks on the constants, the force model and the equations of motion.

    Raises AssertionError on failure.  The demo calls this, so running this
    module is enough to validate the airframe.  Three kinds of check are in
    here: that the transcribed constants are self consistent (each unit
    conversion and each closed form in the rotor's geometry), that the force
    model's own lines agree with each other (the moment arms against the
    printed equations, the inflow against the implicit equation), and that the
    aircraft does what an aircraft does (a trimmed hover stays put, and each
    control moves it the way the stick points).  A fourth is the rotor speed as
    a state: that table 3's constants move with it the way their closed forms
    say, that the drag torque slows a rotor and settles it, and that an engine
    torque equal to that drag leaves the whole thing exactly where the report
    left it.
    """
    from aerodynamics import Rotor
    from rotor_control import RotorControlModel

    rotor = Rotor.uh1h()
    tail = Rotor.uh1h_tail()
    radius = rotor.radius
    area = rotor.disk_area
    slope = rotor.section.lift_slope
    solidity = rotor.solidity
    tip = rotor.tip_speed
    omega = rotor.omega

    # The seven rotor constants of table 3 are closed forms in the rotor's own
    # geometry, so rebuilding them from aerodynamics' UH-1 rotor is a check on
    # the table, on the transcription and on this project's parameters at once.
    assert abs(UH1_ROTOR_R1 - 0.5 * solidity * slope * RHO_SEA_LEVEL * area
               * tip * tip) < 0.02 * UH1_ROTOR_R1
    for tabulated, rebuilt in ((UH1_ROTOR_R2, 1.0 / (4.0 * omega)),
                               (UH1_ROTOR_R3, 1.0 / (2.0 * slope)),
                               (UH1_ROTOR_R4, 1.0 / omega),
                               (UH1_ROTOR_R5, 1.0 / (4.0 * slope)),
                               (UH1_ROTOR_R7, 1.0 / tip),
                               (UH1_ROTOR_R8, 1.0 / (2.0 * RHO_SEA_LEVEL * area
                                                     * tip * tip)),
                               (UH1_ROTOR_R9, UH1_ROTOR_R1 * radius)):
        assert abs(tabulated - rebuilt) < 0.02 * tabulated, (tabulated, rebuilt)
    delta2 = 0.3 * (6.0 / (RHO_SEA_LEVEL * area * tip * tip * solidity
                           * slope)) ** 2
    assert abs(UH1_ROTOR_DELTA2 - delta2) < 0.05 * delta2, delta2
    # The tail rotor's T1 is 1 / 2 (omega R) at the *table 2* tip speed of
    # 740 ft/s, which belongs to the report's 301 rpm reference.  This project
    # flies the 324 rpm reference, where the same rotor turns 7 per cent faster,
    # so the two agree to within that and no closer.
    assert abs(UH1_TAIL_T1 - 1.0 / (2.0 * 740.0 * FOOT)) < 0.001 * UH1_TAIL_T1
    assert abs(tail.tip_speed - 740.0 * FOOT) < 0.1 * 740.0 * FOOT
    # The ratio of the two rotors is what ties the table's two tip speeds
    # together (see aerodynamics).
    assert abs(tail.radius - UH1_TAIL_RADIUS) < 1e-9

    # The English and metric columns of table 3 have to be the same numbers in
    # two sets of units, which is how the metric column was read off the page
    # image and checked.  Every conversion factor here is spelled out: the one
    # that is easy to get wrong is the slug, which is 32.174 lb and not 1 lb.
    for english, metric, factor in ((0.022, UH1_FUSELAGE_D1, LB_PER_FT_S2),
                                    (0.201, UH1_FUSELAGE_D2, LB_PER_FT_S2),
                                    (0.646, UH1_FUSELAGE_D3, LB_PER_FT_S2),
                                    (0.0654, UH1_FUSELAGE_LIFT, LB_PER_FT_S2),
                                    (0.0654, UH1_FUSELAGE_SIDE, LB_PER_FT_S2),
                                    (0.031, UH1_STAB_H1, LB_PER_FT_S2),
                                    (0.01128, UH1_STAB_H2, LB_PER_FT_S2),
                                    (0.0083, UH1_STAB_H4, LB_PER_FT_S2),
                                    (0.0292, UH1_FIN_K1, LB_PER_FT_S2),
                                    (0.0106, UH1_FIN_K2, LB_PER_FT_S2),
                                    (0.016, UH1_FIN_F1, LB_PER_FT_S2),
                                    (0.956, UH1_FUSELAGE_MOMENT,
                                     FT_LB_PER_FT_S2),
                                    (0.956, UH1_FUSELAGE_YAW_MOMENT,
                                     FT_LB_PER_FT_S2),
                                    (6.76e-4, UH1_TAIL_T1, 1.0 / FOOT),
                                    (17.0, UH1_TAIL_T2, math.sqrt(POUND_FORCE)),
                                    (7.0e3, UH1_TAIL_T3, POUND_FORCE),
                                    (14.9, UH1_TAIL_T4, LBF_PER_FT),
                                    (1.715e-7, UH1_ROTOR_R8,
                                     1.0 / POUND_FORCE),
                                    (9.107e6, UH1_ROTOR_R9,
                                     POUND_FORCE * FOOT)):
        assert abs(english * factor - metric) < 0.03 * metric, \
            (english, metric, english * factor)

    # Geometry, against table 2's stations and waterlines.  Table 3 gives these
    # to two decimals of a foot, so the agreement is to a millimetre and not to
    # the last bit.
    assert abs(UH1_HUB_HEIGHT - 81.5 * 0.0254) < 2e-3        # 136.5 - 55 in
    assert abs(UH1_TAIL_ARM - 28.0 * FOOT) < 1e-9
    assert abs(UH1_TAIL_HEIGHT - 82.5 * 0.0254) < 2e-3       # 137.5 - 55 in
    assert abs(UH1_SIM_MASS - 3946.25) < 0.5
    assert abs(UH1_IXX / SLUG_FT2 - 2800.0) < 1e-9

    # The ground effect factor: none at the ground plane, and out of it by a few
    # diameters.
    assert abs(ground_effect_factor(0.0)) < 1e-12
    assert abs(ground_effect_factor(2.0 * radius)
               - (1.0 - math.exp(-1.0 / UH1_GROUND_EFFECT_RANGE))) < 1e-12
    assert ground_effect_factor(2.0 * radius) < ground_effect_factor(4.0 * radius)
    assert ground_effect_factor(20.0 * radius) > 0.999

    # The stabilizer schedule, at its own points and held flat past the ends.
    for stick, expected in UH1_STABILIZER_SCHEDULE:
        assert abs(stabilizer_incidence_rad(stick) - expected) < 1e-12
    assert abs(stabilizer_incidence_rad(-20.0)
               - UH1_STABILIZER_SCHEDULE[0][1]) < 1e-12
    assert abs(stabilizer_incidence_rad(20.0)
               - UH1_STABILIZER_SCHEDULE[-1][1]) < 1e-12
    # The schedule's most negative point is at a centred stick: the stabilizer
    # is rigged nose down in the middle of the travel.
    assert abs(stabilizer_incidence_rad(0.0) + 0.0820) < 1e-12
    assert stabilizer_incidence_rad(0.0) < stabilizer_incidence_rad(6.0)

    # The report's collective pitch is the root pitch less three quarters of the
    # twist, and it is the pitch the thrust equation's own identity needs.
    assert abs(report_collective_rad(10.0) - math.radians(10.0 - 7.5)) < 1e-12
    assert abs(report_collective_rad(10.0, 0.0)
               - math.radians(10.0)) < 1e-12

    # The force model's own lines: the positions used for the moments have to
    # reproduce the printed equations 20 to 22 and 40 to 41, which is what pins
    # the signs of the hub and tail rotor offsets.
    hover = ControlPitch(collective_rad=report_collective_rad(15.23),
                         control_axis_long_rad=0.0,
                         control_axis_lat_rad=0.0,
                         tail_collective_rad=math.radians(7.74))
    state = FlightState(position=Vector3(0.0, 0.0, -200.0))
    airframe = Airframe()
    forces = airframe.forces(hover, state)
    rotor_force = forces.rotor_force
    hub = Vector3(-UH1_CG_AHEAD_OF_HUB, 0.0, -UH1_HUB_HEIGHT)
    assert abs(hub.cross(rotor_force).x - rotor_force.y * UH1_HUB_HEIGHT) < 1e-9
    assert abs(hub.cross(rotor_force).y
               - (-rotor_force.x * UH1_HUB_HEIGHT
                  + rotor_force.z * UH1_CG_AHEAD_OF_HUB)) < 1e-9
    assert abs(hub.cross(rotor_force).z
               + rotor_force.y * UH1_CG_AHEAD_OF_HUB) < 1e-9
    tail_position = Vector3(-UH1_TAIL_ARM, 0.0, -UH1_TAIL_HEIGHT)
    tail_force = Vector3(0.0, forces.tail_thrust, 0.0)
    assert abs(tail_position.cross(tail_force).x
               - forces.tail_thrust * UH1_TAIL_HEIGHT) < 1e-9
    assert abs(tail_position.cross(tail_force).z
               + forces.tail_thrust * UH1_TAIL_ARM) < 1e-9

    # The inflow: the report's equation 10 has to be satisfied by the solve, and
    # the answer has to match the Glauert solver in aerodynamics, which is an
    # independent implementation of the same physics.  They agree to about a per
    # cent and not exactly, because the closed form carries R8 as tabulated to
    # two significant figures while Glauert's coefficient comes from the rotor's
    # geometry: a 1.7 per cent difference in R8 is 0.9 per cent in the velocity.
    rotor_loads = forces.rotor
    residual = (rotor_loads.inflow_ratio + UH1_ROTOR_R7
                * rotor_loads.wind_along_axis
                + UH1_ROTOR_R8 * rotor_loads.ground_effect * rotor_loads.thrust
                / math.sqrt(rotor_loads.advance_ratio ** 2
                            + rotor_loads.inflow_ratio ** 2))
    assert abs(residual) < 1e-9, residual
    glauert = glauert_induced_velocity(
        rotor_loads.thrust, radius, rotor.omega,
        forward_speed=rotor_loads.in_plane_speed,
        climb_speed=rotor_loads.wind_along_axis,
        ground_effect=rotor_loads.ground_effect)
    assert abs(glauert - max(rotor_loads.induced_velocity, 0.0)) \
        < 0.02 * glauert, (glauert, rotor_loads.induced_velocity)
    assert rotor_loads.converged

    # Equation 1 against this project's blade element rotor, at the hover the
    # blade element model trims to.  The two share no constants: one is a closed
    # form in R1 to R9, the other integrates two blades.  They agree to well
    # under a per cent when the collective is read as the report's - which is
    # the whole point of report_collective_rad, and the only reason the numbers
    # in this module can be trusted.
    control_model = RotorControlModel()
    trim = control_model.trim_hover(mass=UH1_SIM_MASS)
    response = control_model.respond(trim)
    blade_element = ControlPitch.from_angles(response.angles)
    assert abs(blade_element.collective_rad
               - report_collective_rad(response.angles.collective_pitch_deg,
                                       rotor.twist_deg)) < 1e-12
    closed_form = main_rotor_forces(blade_element, Vector3(), Vector3())
    assert abs(closed_form.thrust - response.thrust) < 0.01 * response.thrust, \
        (closed_form.thrust, response.thrust)
    # The inflow the two find, for the same collective: the closed form's own
    # momentum theory against the blade element solve.
    assert abs(closed_form.induced_velocity
               - response.inflow) < 0.02 * response.inflow, \
        (closed_form.induced_velocity, response.inflow)

    # The rigid body's kinematics and its gravity terms.
    matrix = direction_cosine_matrix(0.2, -0.3, 1.1)
    identity = [[sum(matrix[row][k] * matrix[column][k] for k in range(3))
                 for column in range(3)] for row in range(3)]
    for row in range(3):
        for column in range(3):
            assert abs(identity[row][column] - (1.0 if row == column else 0.0)) \
                < 1e-12
    level = FlightState(position=Vector3(), velocity=Vector3(),
                        attitude=Vector3(0.0, 0.3, 0.0), rates=Vector3())
    derivative, _ = airframe.derivatives(
        level, ControlPitch(0.0, 0.0, 0.0, 0.0))
    # With no aerodynamic force at all, the equations of motion have to hand
    # back gravity alone: an aft and downward pull for a nose-up attitude.
    assert abs(derivative.velocity.x + GRAVITY * math.sin(0.3)) < 2.0
    assert abs(derivative.velocity.z - GRAVITY * math.cos(0.3)) < 2.0

    # The roll-yaw coupling the product of inertia makes: a pure nose-right yaw
    # moment rolls the aircraft right as well as yawing it, which is the UH-1H's
    # signature and the reason its inertia matrix carries Ixz at all.  With
    # moments (0, 0, N) the roll and yaw accelerations solve to the two closed
    # forms below.
    coupled = airframe.angular_acceleration(Vector3(0.0, 0.0, 1000.0),
                                            Vector3())
    ixx, iyy, izz, ixz = airframe.inertia
    determinant = ixx * izz - ixz * ixz
    assert abs(coupled.z - 1000.0 * ixx / determinant) < 1e-9
    assert abs(coupled.x - 1000.0 * ixz / determinant) < 1e-9
    assert coupled.x > 0.0
    # And the other coupling, the one every spinning body has: a yaw rate on top
    # of a pitch rate rolls it, this time through the (Izz - Iyy) q r term and
    # again with the product of inertia correcting the answer.
    spinning = airframe.angular_acceleration(Vector3(), Vector3(0.0, 0.1, 1.0))
    roll_source = (iyy - izz) * 0.1
    yaw_source = -ixz * 0.1
    assert abs(spinning.x - (izz * roll_source + ixz * yaw_source)
               / determinant) < 1e-12
    assert abs(spinning.y - ixz / iyy) < 1e-12

    # The trim: the five residuals have to close, and the answer has to be the
    # one the geometry implies.  The pitch is taken straight from the mixing
    # rather than from the lags, so that this does not depend on whether the
    # lags have been reset yet.
    controls, trim_state = airframe.trim_hover()
    weight = airframe.mass * GRAVITY
    pitch = ControlPitch.from_angles(airframe.command_angles(controls),
                                     controls.long_stick)
    trimmed = airframe.forces(pitch, trim_state)
    theta = trim_state.attitude.y
    phi = trim_state.attitude.x
    assert abs(trimmed.force.x - weight * math.sin(theta)) < 1.0
    assert abs(trimmed.force.y + weight * math.sin(phi)
               * math.cos(theta)) < 1.0
    assert abs(trimmed.force.z + weight * math.cos(phi)
               * math.cos(theta)) < 1.0
    assert abs(trimmed.moment.x) < 1.0
    assert abs(trimmed.moment.y) < 1.0
    assert abs(trimmed.moment.z) < 1.0
    assert abs(trimmed.rotor.thrust - weight) < 0.05 * weight
    assert controls.collective > 8.0 and controls.collective < 11.0
    assert controls.long_stick > 0.5          # forward cyclic, c.g. is aft
    assert -3.0 < controls.pedal < 0.0        # left pedal: torque is nose right
    # The attitude is the geometry: the hub has to swing to sit over the c.g.
    expected = math.degrees(math.atan2(-UH1_CG_AHEAD_OF_HUB, UH1_HUB_HEIGHT))
    assert abs(math.degrees(theta) - expected) < 1.0, math.degrees(theta)
    assert abs(pitch.control_axis_long_rad - math.radians(expected)) < 0.05

    # And it has to hold: two seconds of frames with nothing moving at all, now
    # that all six residuals are closed.
    airframe.reset(trim_state, controls)
    for _ in range(120):
        airframe.step(1.0 / 60.0, controls)
    assert abs(airframe.state.altitude - trim_state.altitude) < 0.002
    assert abs(airframe.state.position.x) < 0.002
    assert abs(airframe.state.position.y) < 0.002
    assert airframe.state.velocity.length() < 0.002
    assert abs(airframe.state.attitude.y - theta) < 1e-3

    # Level flight: the same six residuals, but in earth axes and on a state
    # that is moving, which is a different solve - the air sees the flight path
    # at an angle of attack, the fuselage, fin and stabilizer have their say, and
    # the attitude is an answer rather than the hover's geometry.  The report's
    # own level flight condition, figures 2 to 9, is 60 kt.
    level_controls, level_state = airframe.trim_level_flight(60.0 * KNOT)
    level_pitch = ControlPitch.from_angles(
        airframe.command_angles(level_controls), level_controls.long_stick)
    levelled = airframe.forces(level_pitch, level_state)
    theta, phi = level_state.attitude.y, level_state.attitude.x
    assert abs(level_state.speed - 60.0 * KNOT) < 1e-9
    assert abs(levelled.force.x - weight * math.sin(theta)) < 1.0
    assert abs(levelled.force.y + weight * math.sin(phi) * math.cos(theta)) < 1.0
    assert abs(levelled.force.z + weight * math.cos(phi) * math.cos(theta)) < 1.0
    assert abs(levelled.moment.x) < 1.0
    assert abs(levelled.moment.y) < 1.0
    assert abs(levelled.moment.z) < 1.0
    # The rotor is flying through its own wash rather than hovering in still air,
    # so 60 kt needs less collective than the hover of the same weight carries.
    assert level_controls.collective < controls.collective - 0.5
    assert abs(level_controls.collective - 9.06) < 0.05, level_controls.collective
    # And it has to hold, which for a moving aircraft means holding a *path*: two
    # seconds of frames, 61.7 m of north, and the velocity neither changing size
    # nor tilting out of the horizontal plane.
    airframe.reset(level_state, level_controls)
    north = level_state.position.x
    for _ in range(120):
        airframe.step(1.0 / 60.0, level_controls)
    assert abs(airframe.state.speed - level_state.speed) < 0.002
    assert abs(airframe.state.ground_velocity().z) < 0.002
    assert abs(airframe.state.altitude - level_state.altitude) < 0.002
    assert abs(airframe.state.position.x - north - 2.0 * level_state.speed) < 0.01
    assert abs(airframe.state.attitude.y - theta) < 1e-3

    # At zero airspeed this is the hover, and it is the same root: the two
    # solves have their residuals written in different frames, five of the six
    # unknowns come from the same model and they agree to a thousandth of an inch.
    still_controls, still_state = airframe.trim_level_flight(0.0)
    assert abs(still_controls.collective - controls.collective) < 1e-3
    assert abs(still_controls.long_stick - controls.long_stick) < 1e-3
    assert abs(still_controls.lat_stick - controls.lat_stick) < 1e-3
    assert abs(still_controls.pedal - controls.pedal) < 1e-3
    assert abs(still_state.attitude.y - trim_state.attitude.y) < 1e-6
    assert abs(still_state.attitude.x - trim_state.attitude.x) < 1e-6

    # A condition the model cannot hold does not raise: the solve comes back with
    # whatever its last step left and the residual says how far from a trim that
    # is.  Between 80 and 100 kt the 6158 lb aircraft runs out - at 100 kt the
    # collective has reached its up stop and the balance cannot close - and that
    # is where this model's envelope ends in forward flight.
    instrumented = Airframe(mass=UH1_TEST_MASS,
                            rotor_time_constant=UH1_ROTOR_TIME_CONSTANT_SIM_S)
    reached, reached_state = instrumented.trim_level_flight(80.0 * KNOT)
    force, _ = instrumented.equilibrium_residual(reached, reached_state)
    assert force.length() < 1.0, force.length()
    fast_controls, fast_state = instrumented.trim_level_flight(100.0 * KNOT)
    force, _ = instrumented.equilibrium_residual(fast_controls, fast_state)
    assert force.length() > 1000.0, force.length()

    # Each control moves the aircraft the way the pilot's hand or foot points.
    def fly(seconds, **offsets):
        held = dict(collective=controls.collective, pedal=controls.pedal,
                    long_stick=controls.long_stick,
                    lat_stick=controls.lat_stick)
        held.update(offsets)
        airframe.reset(trim_state, controls)
        for _ in range(int(seconds * 60.0)):
            airframe.step(1.0 / 60.0, PilotControls(**held))
        return airframe.state

    forward = fly(1.0, long_stick=controls.long_stick + 1.0)
    assert forward.rates.y < 0.0                # nose down
    assert forward.attitude.y < theta
    assert forward.velocity.x > 0.0             # accelerating forward
    right = fly(1.0, lat_stick=controls.lat_stick + 1.0)
    assert right.rates.x > 0.0                  # rolling right
    assert right.attitude.x > 0.0
    yawed = fly(1.0, pedal=controls.pedal + 1.0)
    assert yawed.rates.z > 0.0                  # nose right
    assert yawed.attitude.z > 0.0
    climbed = fly(2.0, collective=controls.collective + 1.0)
    assert climbed.altitude > trim_state.altitude
    assert climbed.velocity.z < 0.0             # z is down, so climbing is -w
    # Right pedal takes the tail rotor's nose-left moment away, so the nose goes
    # right: the yaw moment with the pedal in has to be less negative.
    more_pedal = ControlPitch.from_angles(
        airframe.command_angles(PilotControls(collective=controls.collective,
                                              pedal=controls.pedal + 1.0)))
    assert airframe.forces(more_pedal, trim_state).moment.z > trimmed.moment.z

    # The rotor time constant of table 3, both readings, and the two ways of
    # flying the report's step responses.
    sim_time = Airframe(rotor_time_constant=UH1_ROTOR_TIME_CONSTANT_SIM_S)
    assert sim_time.rotor_time_constant == UH1_ROTOR_TIME_CONSTANT_SIM_S
    assert sim_time.rotor_time_constant > airframe.rotor_time_constant

    # ---------------------------------------------------------------------
    # The rotor speed as a state, and the table 3 constants as functions of it.
    # ---------------------------------------------------------------------

    # The polar inertia neither report gives, derived rather than mined: the
    # teetering hinge sits on the shaft, so a blade's flap inertia about the
    # hinge is also its polar inertia about the shaft, and rotor_control's own
    # rotor already carries one blade's, asserted there against a blade mass.
    assert abs(UH1_ROTOR_INERTIA
               - rotor.blade_count * rotor.flap_inertia) \
        < 0.005 * UH1_ROTOR_INERTIA, rotor.flap_inertia
    # The same inertia is behind the flapping time constant: R6 is 16 / (gamma
    # omega), so table 3's 0.072 sec and the rotor's 6.55 lock number are the
    # same statement about the same blades.
    assert abs(UH1_ROTOR_TIME_CONSTANT_S
               - 16.0 / (rotor.lock_number * UH1_OMEGA)) < 1e-4
    assert airframe.engine_torque is None              # the report's fixed rotor
    assert airframe.rotor_inertia == UH1_ROTOR_INERTIA

    # At the reference speed the table's constants are the table itself, to the
    # last bit: the scaling is the identity there, or the whole idea is wrong.
    reference = rotor_constants()
    assert reference.omega == UH1_OMEGA
    assert reference.rpm == UH1_RPM
    assert reference.tip_speed == UH1_TIP_SPEED
    for scaled, tabulated in ((reference.r1, UH1_ROTOR_R1),
                              (reference.r2, UH1_ROTOR_R2),
                              (reference.r3, UH1_ROTOR_R3),
                              (reference.r4, UH1_ROTOR_R4),
                              (reference.r5, UH1_ROTOR_R5),
                              (reference.r6, UH1_ROTOR_TIME_CONSTANT_S),
                              (reference.r7, UH1_ROTOR_R7),
                              (reference.r8, UH1_ROTOR_R8),
                              (reference.r9, UH1_ROTOR_R9),
                              (reference.tail_t1, UH1_TAIL_T1)):
        assert scaled == tabulated, (scaled, tabulated)

    # And away from it each one moves the way its own closed form says, rebuilt
    # at that speed rather than scaled from the table - which is the same check
    # the constants above get at 100 per cent, asked at 90.
    slow = rotor_constants(0.9 * UH1_OMEGA)
    for scaled, rebuilt in ((slow.r1, 0.5 * solidity * slope * RHO_SEA_LEVEL
                             * area * slow.tip_speed ** 2),
                            (slow.r2, 1.0 / (4.0 * slow.omega)),
                            (slow.r3, 1.0 / (2.0 * slope)),
                            (slow.r4, 1.0 / slow.omega),
                            (slow.r5, 1.0 / (4.0 * slope)),
                            (slow.r6, 16.0 / (rotor.lock_number * slow.omega)),
                            (slow.r7, 1.0 / slow.tip_speed),
                            (slow.r8, 1.0 / (2.0 * RHO_SEA_LEVEL * area
                                             * slow.tip_speed ** 2)),
                            (slow.r9, slow.r1 * radius)):
        assert abs(scaled - rebuilt) < 0.03 * scaled, (scaled, rebuilt)
    assert abs(slow.r1 - 0.81 * UH1_ROTOR_R1) < 1e-12 * UH1_ROTOR_R1
    assert abs(slow.r3 - UH1_ROTOR_R3) < 1e-15
    assert abs(slow.r5 - UH1_ROTOR_R5) < 1e-15
    assert abs(slow.r4 - UH1_ROTOR_R4 / 0.9) < 1e-12 * UH1_ROTOR_R4
    assert abs(slow.r7 - UH1_ROTOR_R7 / 0.9) < 1e-12 * UH1_ROTOR_R7
    assert abs(slow.r8 - UH1_ROTOR_R8 / 0.81) < 1e-12 * UH1_ROTOR_R8
    assert abs(slow.r9 - 0.81 * UH1_ROTOR_R9) < 1e-12 * UH1_ROTOR_R9
    assert abs(slow.tip_speed - 0.9 * UH1_TIP_SPEED) < 1e-12 * UH1_TIP_SPEED
    assert abs(slow.tail_t1 - UH1_TAIL_T1 / 0.9) < 1e-12 * UH1_TAIL_T1
    assert abs(slow.rpm - 0.9 * UH1_RPM) < 1e-9
    try:
        rotor_constants(0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("a stopped rotor was accepted")

    # Threading the constants into the force model changes nothing when they are
    # the reference ones: the report's own rotor is reproduced to the last bit,
    # which is what every figure in TM-73254 is drawn against.
    argued = main_rotor_forces(hover, Vector3(), Vector3(),
                               rotor_height=state.rotor_height)
    threaded = main_rotor_forces(hover, Vector3(), Vector3(),
                                 rotor_height=state.rotor_height,
                                 constants=rotor_constants())
    assert argued.thrust == threaded.thrust
    assert argued.torque == threaded.torque
    assert argued.inflow_ratio == threaded.inflow_ratio
    # The tail rotor's T1 is 1 / 2 (omega R) there, through the drive ratio the
    # report's two tip speeds share.  Nothing at all in a hover, where equation
    # 37 carries no tip speed, and more thrust per degree of pedal once the
    # aircraft is moving, because the same wash over a slower rotor is a larger
    # advance ratio and, with it, a larger thrust coefficient.
    pedal = math.radians(7.74)
    assert tail_rotor_thrust(pedal, 0.0, 0.0, tail_t1=slow.tail_t1) \
        == tail_rotor_thrust(pedal, 0.0, 0.0)
    assert tail_rotor_thrust(pedal, 40.0, 0.0, tail_t1=slow.tail_t1) \
        > tail_rotor_thrust(pedal, 40.0, 0.0)
    assert tail_rotor_thrust(pedal, 40.0, 5.0) == tail_rotor_thrust(
        pedal, 40.0, 5.0, tail_t1=reference.tail_t1)

    # Equation 1 at a fixed collective, on rotors that are turning slower: the
    # thrust goes as the square of the speed and the inflow ratio does not move
    # at all.  The second one is neither a coincidence nor a bug - in a hover
    # equation 10 has R7 w_C = 0 and R8 T, and T's omega squared from R1 cancels
    # R8's reciprocal of it from (omega R) ** 2 - so lambda is a function of the
    # collective alone.  The torque is another matter: it falls a little faster
    # than omega squared, because the profile drag's delta grows with the thrust
    # that is still going.
    upright = FlightState(position=Vector3(0.0, 0.0, -trim_state.altitude))
    base = airframe.forces(pitch, upright)
    for fraction in (0.95, 0.9, 0.8):
        turning = rotor_constants(fraction * UH1_OMEGA)
        slower = airframe.forces(pitch, FlightState(
            position=upright.position, rotor_speed=turning.omega))
        assert abs(slower.rotor.thrust
                   - fraction * fraction * base.rotor.thrust) \
            < 1e-9 * base.rotor.thrust, fraction
        assert abs(slower.rotor.inflow_ratio - base.rotor.inflow_ratio) < 1e-9
        assert slower.rotor.torque < fraction * fraction * base.rotor.torque

    # A hover trimmed on a rotor that is not at the reference speed: the same
    # weight, and the collective has to come up because R1 has come down as the
    # square of the speed.  The 8700 lb aircraft cannot do it at all below about
    # 97 per cent - its stick already sits at 93 per cent of the travel - so the
    # 6158 lb one is the one to ask, which is the difference between an aircraft
    # with collective in hand and one without.
    light = Airframe(mass=UH1_TEST_MASS)
    light_controls, light_state = light.trim_hover()
    light_weight = light.mass * GRAVITY
    previous = light_controls.collective
    for fraction in (0.95, 0.9):
        turning = rotor_constants(fraction * UH1_OMEGA)
        slow_controls, slow_state = light.trim_hover(rotor_speed=turning.omega)
        assert slow_state.rotor_speed == turning.omega
        slow_pitch = ControlPitch.from_angles(
            light.command_angles(slow_controls), slow_controls.long_stick)
        slow_trimmed = light.forces(slow_pitch, slow_state)
        slow_roll = slow_state.attitude.x
        slow_theta = slow_state.attitude.y
        assert abs(slow_trimmed.force.x
                   - light_weight * math.sin(slow_theta)) < 1.0
        assert abs(slow_trimmed.force.y + light_weight * math.sin(slow_roll)
                   * math.cos(slow_theta)) < 1.0
        assert abs(slow_trimmed.force.z + light_weight * math.cos(slow_roll)
                   * math.cos(slow_theta)) < 1.0
        assert abs(slow_trimmed.moment.x) < 1.0
        assert abs(slow_trimmed.moment.y) < 1.0
        assert abs(slow_trimmed.moment.z) < 1.0
        assert abs(slow_trimmed.rotor.thrust - light_weight) \
            < 0.01 * light_weight
        assert slow_controls.collective > previous
        previous = slow_controls.collective
        assert slow_controls.collective < UH1_COLLECTIVE_TRAVEL_IN
    assert abs(previous - light_controls.collective - 0.66) < 0.02, previous

    # The shaft balance, which is what the thirteenth state variable integrates.
    # With the engine's torque set to what the trimmed rotor is absorbing, the
    # balance is exactly zero and the aircraft is the report's fixed rotor
    # again: three seconds of frames leave the rotor at 100 per cent and the
    # altitude unmoved, which is the check that this has changed nothing.
    drag = trimmed.rotor.torque
    held = Airframe(engine_torque=drag)
    assert abs(held.rotor_acceleration(trimmed, trim_state)) < 1e-12
    held.reset(trim_state, controls)
    for _ in range(180):
        held.step(1.0 / 60.0, controls)
    assert abs(held.state.rotor_speed - UH1_OMEGA) < 1e-3, held.state.rotor_speed
    assert abs(held.state.altitude - trim_state.altitude) < 0.002

    # Take the engine's torque away and it winds down at the rate Q / I: a
    # seventh of the speed in the first second, and then it settles, because the
    # collective is still at the hover setting and what it settles at is a rotor
    # turning the air over rather than one carrying an aircraft.  Twice the drag
    # instead and it winds up, by the same arithmetic.
    cut = Airframe(engine_torque=0.0)
    assert abs(cut.rotor_acceleration(trimmed, trim_state)
               + drag / cut.rotor_inertia) < 1e-12
    cut.reset(trim_state, controls)
    previous = UH1_OMEGA
    for _ in range(6):
        for _ in range(60):
            cut.step(1.0 / 60.0, controls)
        assert cut.state.rotor_speed < previous
        previous = cut.state.rotor_speed
    assert 0.5 * UH1_OMEGA < cut.state.rotor_speed < 0.65 * UH1_OMEGA
    assert cut.state.rotor_rpm < UH1_RPM_LOW      # out of the green arc with it
    surge = Airframe(engine_torque=2.0 * drag)
    assert abs(surge.rotor_acceleration(trimmed, trim_state)
               - drag / surge.rotor_inertia) < 1e-12
    # And the frame integrates it: ten frames of a slow rotor are about three
    # quarters of a rad/s, the balance's own rate over that sixth of a second.
    short = Airframe(engine_torque=0.0)
    short.reset(trim_state, controls)
    for _ in range(10):
        short.step(1.0 / 60.0, controls)
    assert UH1_OMEGA - 1.5 < short.state.rotor_speed < UH1_OMEGA - 0.5

    # The state variable itself: thirteen of them, and the new one round trips
    # through the integration's own tuple and through a copy.
    assert len(FlightState().values()) == 13
    assert FlightState().rotor_speed == UH1_OMEGA
    assert abs(FlightState().rotor_rpm - UH1_RPM) < 1e-9
    assert FlightState.from_values(trim_state.values()) == trim_state
    assert trim_state.copy() == trim_state
    crawling = FlightState(rotor_speed=0.8 * UH1_OMEGA)
    assert crawling.copy().rotor_speed == 0.8 * UH1_OMEGA
    assert FlightState.from_values(crawling.values()).rotor_speed \
        == 0.8 * UH1_OMEGA
    assert crawling.rotor_rpm < UH1_RPM_LOW
    # A fixed rotor has no rotor speed derivative at all, which is the whole of
    # what the default engine torque means.
    spinning_down, _ = airframe.derivatives(trim_state, pitch)
    assert spinning_down.rotor_speed == 0.0

    # The engine: the same shaft balance with a T53 on the other end of it.
    # There is no throttle to roll on an aircraft with no engine, and there is
    # no rotor speed dynamics either - this is the flag that keeps every figure
    # in the report where it was.
    assert airframe.throttle == 1.0
    airframe.throttle = 0.25
    assert airframe.throttle == 1.0

    # With one, a reset puts it on the torque the rotor is absorbing - 16268
    # N m for this trim - so the shaft balance starts where the trim left it,
    # and then the governor walks the aircraft to its own droop point: a shade
    # under 1 rpm of rotor for a hover, 21 of the 40 rpm of N2 the manual rigs
    # the compensator to (2-23).  Six seconds of frames are enough to sit down
    # on it, and the closed form and the integrated loop have to agree about
    # where that is, or the droop curve is decorative.
    governed = Airframe(engine=Engine())
    governed.reset(trim_state, controls)
    assert abs(governed.engine_torque - drag) < 1.0, governed.engine_torque
    for _ in range(360):
        governed.step(1.0 / 60.0, controls)
    settled = governed.state.rotor_rpm
    assert UH1_RPM_LOW < settled < UH1_RPM, settled
    assert abs(UH1_RPM - settled) < 2.0, settled
    assert abs(governed.engine.governor.n2_rpm(settled)
               - UH1_ENGINE_RPM) < UH1_ENGINE_DROOP_RPM
    assert abs(settled - governed.engine.governor.settled_rotor_rpm(
        drag * UH1_OMEGA)) < 1.0, settled
    # The altitude has moved a little - the trim was made at 100 per cent and
    # the governor is holding 99.7 - but this is a flight, not a fall: the
    # aircraft is sinking at centimetres a second, not metres.
    assert governed.state.ground_velocity().z < 1.0

    # An engine failure is the torque going away, and the lag is what makes it
    # a wind down rather than a switch: half a second in, the engine is still
    # delivering better than a third of its power - and the rotor has lost more
    # than a warning band's worth of speed, because a hovering rotor with no
    # engine is a rotor with 16 kN m of drag and nothing driving it.  Six
    # seconds in, the rotor is out of the green arc, and the collective the
    # trim left in the pilot's hand is what has it there.
    failed = Airframe(engine=Engine())
    failed.reset(trim_state, controls)
    failed.engine.governor.failed = True
    for _ in range(30):
        failed.step(1.0 / 60.0, controls)
    assert failed.engine.power > 0.3 * drag * UH1_OMEGA, failed.engine.power
    assert UH1_RPM - 20.0 < failed.state.rotor_rpm < UH1_RPM
    for _ in range(330):
        failed.step(1.0 / 60.0, controls)
    assert failed.engine.power < 0.02 * drag * UH1_OMEGA, failed.engine.power
    assert failed.state.rotor_rpm < UH1_RPM_LOW, failed.state.rotor_rpm


def _demo():
    """Print what the airframe does: constants, a hover, and four steps.

    Raises AssertionError if the self test fails.
    """
    from aerodynamics import Rotor

    rotor = Rotor.uh1h()
    tail = Rotor.uh1h_tail()
    area = rotor.disk_area
    tip = rotor.tip_speed

    print("TM-73254 force and moment model, in SI")
    print("  weight %d lb (%0.0f kg) simulated, %0.0f kg instrumented"
          % (8700, UH1_SIM_MASS, UH1_TEST_MASS))
    print("  inertia Ixx %.0f, Iyy %.0f, Izz %.0f, Ixz %.0f kg m^2"
          % (UH1_IXX, UH1_IYY, UH1_IZZ, UH1_IXZ))
    print("  hub %.2f m above the c.g., tail rotor %.2f m aft and %.2f m above,"
          " stabilizer %.2f m aft"
          % (UH1_HUB_HEIGHT, UH1_TAIL_ARM, UH1_TAIL_HEIGHT, UH1_STAB_ARM))
    print("  c.g. %.2f m %s of the hub"
          % (abs(UH1_CG_AHEAD_OF_HUB),
             "aft" if UH1_CG_AHEAD_OF_HUB < 0.0 else "forward"))
    print()
    print("  table 3's rotor constants against the rotor they came from:")
    print("    R1 %8.2e N     sigma a / 2 rho A (omega R)^2   %8.2e"
          % (UH1_ROTOR_R1, 0.5 * rotor.solidity * rotor.section.lift_slope
             * RHO_SEA_LEVEL * area * tip * tip))
    print("    R2 %8.3e s     1 / 4 omega                     %8.3e"
          % (UH1_ROTOR_R2, 1.0 / (4.0 * rotor.omega)))
    print("    R4 %8.3e s     1 / omega                       %8.3e"
          % (UH1_ROTOR_R4, 1.0 / rotor.omega))
    print("    R7 %8.3e s/m   1 / omega R                     %8.3e"
          % (UH1_ROTOR_R7, 1.0 / tip))
    print("    R8 %8.3e 1/N   1 / 2 rho A (omega R)^2         %8.3e"
          % (UH1_ROTOR_R8, 1.0 / (2.0 * RHO_SEA_LEVEL * area * tip * tip)))
    print("    R9 %8.3e J     R1 R                            %8.3e"
          % (UH1_ROTOR_R9, UH1_ROTOR_R1 * rotor.radius))
    print("    T1 %8.3e s/m   1 / 2 (omega R) at the table's 740 ft/s %8.3e"
          % (UH1_TAIL_T1, 1.0 / (2.0 * 740.0 * FOOT)))
    print("       (this project's tail rotor runs at %.1f m/s, the 324 rpm"
          " reference, so 1 / 2 omega R there is %.3e)"
          % (tail.tip_speed, 1.0 / (2.0 * tail.tip_speed)))
    print("  and the whole set is parametric in the rotor speed, each one of them"
          " being a")
    slow = rotor_constants(0.9 * UH1_OMEGA)
    print("  closed form in omega.  At 100 %%: R1 %8.2e N, R7 %8.3e s/m, R6"
          " %.4f s," % (UH1_ROTOR_R1, UH1_ROTOR_R7,
                        UH1_ROTOR_TIME_CONSTANT_S))
    print("    T1 %8.3e s/m, tip %.1f m/s.  At 90 %%: R1 %8.2e N, R7 %8.3e s/m,"
          " R6 %.4f s," % (UH1_TAIL_T1, UH1_TIP_SPEED, slow.r1, slow.r7,
                           slow.r6))
    print("    T1 %8.3e s/m, tip %.1f m/s - R1 down 19 %% as the square of the"
          " speed, the" % (slow.tail_t1, slow.tip_speed))
    print("    reciprocals up a ninth, the tail rotor's through its drive ratio,"
          " and R3")
    print("    and R5 not moving at all.  The flywheel this moves is %.0f kg m^2,"
          % UH1_ROTOR_INERTIA)
    print("    the pair of %.0f kg m^2 blades rotor_control derives - their"
          " teetering" % rotor.flap_inertia)
    print("    hinge being on the shaft - and the hover below turns on its"
          " balance.")

    airframe = Airframe()
    controls, trim_state = airframe.trim_hover()
    airframe.reset(trim_state, controls)
    pitch, seen = airframe.advance_controls(0.0, controls)
    forces = airframe.forces(pitch, trim_state)
    print()
    print("hover trim at %0.0f kg, out of ground effect:"
          % airframe.mass)
    print("  sticks  " + str(controls))
    print("  %s" % trim_state)
    print("  %s" % seen)
    print("  " + str(forces))
    print("  attitude roll %+5.2f, pitch %+5.2f deg (nose up so the hub sits"
          " over the c.g., atan(%.2f / %.2f) = %+5.2f deg)"
          % (math.degrees(trim_state.attitude.x),
             math.degrees(trim_state.attitude.y), -UH1_CG_AHEAD_OF_HUB,
             UH1_HUB_HEIGHT,
             math.degrees(math.atan2(-UH1_CG_AHEAD_OF_HUB, UH1_HUB_HEIGHT))))
    print("  rotor   T %7.0f N, H %+8.0f, Y %+7.0f, Q %+8.0f N m"
          % (forces.rotor.thrust, forces.rotor.h_force,
             forces.rotor.y_force, forces.rotor.torque))
    print("          %.1f rpm (%.2f rad/s), so the shaft balance on %.0f kg m^2"
          " is (Q_eng" % (trim_state.rotor_rpm, trim_state.rotor_speed,
                          airframe.rotor_inertia))
    print("          - %.0f) / I = %.3f rad/s^2 with nothing turning it, and the"
          " %.1f s of" % (forces.rotor.torque,
                          -forces.rotor.torque / airframe.rotor_inertia,
                          airframe.rotor_inertia * trim_state.rotor_speed
                          / forces.rotor.torque))
    print("          I omega / Q it would take to stop if the torque stayed put,"
          " against the")
    print("          eighth of its speed the first second takes with the torque"
          " falling as")
    print("          it turns.")
    print("  tail    %7.0f N at %.2f m aft and %.2f m above the c.g. ->"
          " %+8.0f N m of yaw"
          % (forces.tail_thrust, UH1_TAIL_ARM, UH1_TAIL_HEIGHT,
             forces.tail_moment.z))
    print("  fuselage (%+7.2f, %+7.2f, %+7.2f) N, fin (%+6.2f, %+7.2f,"
          " %+6.2f) N"
          % (forces.fuselage_force.x, forces.fuselage_force.y,
             forces.fuselage_force.z, forces.fin_force.x, forces.fin_force.y,
             forces.fin_force.z))
    print("  stabilizer %.2f deg of incidence, %+5.2f N of force and %+6.0f N m"
          " of pitch moment (a bias at a hover, where u_H is zero: this is the"
          " lambda / R7 term of equation 54, and the trim's cyclic answers it)"
          % (math.degrees(forces.stabilizer_incidence_rad),
             forces.stabilizer_force.z, forces.stabilizer_moment.y))
    print("  inflow  %5.2f m/s (lambda %+6.4f), mu %5.3f, delta %6.4f"
          % (forces.rotor.induced_velocity, forces.rotor.inflow_ratio,
             forces.rotor.advance_ratio, forces.rotor.drag_coefficient))

    from rotor_control import RotorControlModel
    element_model = RotorControlModel()
    element_trim = element_model.trim_hover(mass=airframe.mass)
    element = element_model.respond(element_trim)
    element_pitch = ControlPitch.from_angles(element.angles)
    element_closed = main_rotor_forces(element_pitch, Vector3(), Vector3())
    print()
    print("the same hover through this project's blade element rotor, for"
          " comparison:")
    print("  sticks  " + str(element_trim))
    print("  collective root %6.2f deg = report %6.2f deg | thrust %7.0f N"
          " against equation 1's %7.0f N (%.2f %%)"
          % (element.angles.collective_pitch_deg,
             math.degrees(element_pitch.collective_rad), element.thrust,
             element_closed.thrust,
             100.0 * (element_closed.thrust - element.thrust) / element.thrust))
    print("  inflow  %5.2f m/s against %5.2f m/s from equation 10 | torque"
          " %+8.0f N m against %+8.0f N m"
          % (element.inflow, element_closed.induced_velocity,
             element.loads.shaft_torque, element_closed.torque))
    print("  tail rotor %7.0f N at %+5.2f deg of collective, against %7.0f N"
          " from equation 37" 
          % (element.tail_thrust, element.tail_collective_deg,
             tail_rotor_thrust(math.radians(element.tail_collective_deg), 0.0,
                               0.0)))

    print()
    print("holding that trim for two seconds at 60 Hz:")
    airframe.reset(trim_state, controls)
    for _ in range(120):
        airframe.step(1.0 / 60.0, controls)
    print("  %s" % airframe.state)
    print("  drift from the trim: %.4f m north, %.4f m east, %+.4f m of"
          " altitude, %.4f m/s of speed, %+.4f deg of pitch"
          % (airframe.state.position.x - trim_state.position.x,
             airframe.state.position.y - trim_state.position.y,
             airframe.state.altitude - trim_state.altitude,
             airframe.state.velocity.length(),
             math.degrees(airframe.state.attitude.y - trim_state.attitude.y)))

    print()
    print("60 kt level flight, the condition of the report's figures 2 to 9, on")
    print("the 6158 lb instrumented aircraft it flew them on:")
    level = Airframe(mass=UH1_TEST_MASS,
                     rotor_time_constant=UH1_ROTOR_TIME_CONSTANT_SIM_S)
    level_hover, _ = level.trim_hover()
    level_controls, level_state = level.trim_level_flight(60.0 * KNOT)
    # As for the hover above: the lags have to be put on the trim before a
    # zero length control step can read the pitch the rotor is holding.
    level.reset(level_state, level_controls)
    level_pitch, level_seen = level.advance_controls(0.0, level_controls)
    level_forces = level.forces(level_pitch, level_state)
    residual, moment = level.equilibrium_residual(level_controls, level_state)
    alpha, sideslip = level_state.wind_axes()
    print("  sticks  " + str(level_controls))
    print("  %s" % level_state)
    print("  %s" % level_seen)
    print("  " + str(level_forces))
    print("  attitude right %+5.2f, nose up %+5.2f deg, at %+5.2f deg of angle of"
          " attack and %+5.2f deg of sideslip"
          % (math.degrees(level_state.attitude.x),
             math.degrees(level_state.attitude.y), alpha, sideslip))
    print("  collective %.2f in against the %.2f in this aircraft's hover needs, so"
          " 60 kt rides"
          % (level_controls.collective, level_hover.collective))
    print("  %.2f in lower: mu %.3f, lambda %+.4f, %+8.0f N of H force, so the"
          " rotor's work"
          % (level_hover.collective - level_controls.collective,
             level_forces.rotor.advance_ratio, level_forces.rotor.inflow_ratio,
             level_forces.rotor.h_force))
    print("  goes into the airframe's drag instead of into the air below it.")

    print()
    print("holding that for two seconds at 60 Hz, which for a moving aircraft")
    print("means holding a *path*, since the trim's velocity is 60 kt north:")
    level.reset(level_state, level_controls)
    for _ in range(120):
        level.step(1.0 / 60.0, level_controls)
    print("  %s" % level.state)
    print("  flown %.3f m north in the two seconds at %.4f m/s, and the drift from"
          % (level.state.position.x - level_state.position.x,
             level.state.ground_velocity().length()))
    print("  the trim is %+.6f m/s of speed, %+.6f m/s vertically, %+.6f m of"
          " altitude,"
          % (level.state.speed - level_state.speed,
             level.state.ground_velocity().z,
             level.state.altitude - level_state.altitude))
    print("  %+.6f deg of pitch and %.4f N m of residual moment"
          % (math.degrees(level.state.attitude.y - level_state.attitude.y),
             moment.length()))

    print()
    print("and the other edge of the envelope, at 100 kt: this rotor cannot hold")
    print("level flight there at this weight, so the collective runs into its up")
    print("stop and what comes back is a residual rather than a trim.")
    fast_controls, fast_state = level.trim_level_flight(100.0 * KNOT)
    fast_pitch = ControlPitch.from_angles(level.command_angles(fast_controls),
                                          fast_controls.long_stick)
    fast_forces = level.forces(fast_pitch, fast_state)
    residual, _ = level.equilibrium_residual(fast_controls, fast_state)
    print("  collective %.2f in at mu %.3f, and %.0f N of net earth force left over"
          % (fast_controls.collective, fast_forces.rotor.advance_ratio,
             residual.length()))
    print("  against the %.0f N the aircraft weighs - which is what"
          " equilibrium_residual is" % (level.mass * GRAVITY))
    print("  for: a trim is checked, not trusted.")

    def step_response(name, **offsets):
        """Fly a step from the trim and print the response, as the report did."""
        held = dict(collective=controls.collective, pedal=controls.pedal,
                    long_stick=controls.long_stick,
                    lat_stick=controls.lat_stick)
        held.update(offsets)
        airframe.reset(trim_state, controls)
        print()
        print("%s, held from the hover trim:" % name)
        print("    time      p      q      r      roll   pitch    yaw"
              "      u      v      w     alt")
        for frame in range(int(2.0 * 60) + 1):
            if frame:
                airframe.step(1.0 / 60.0, PilotControls(**held))
            if frame % 12:
                continue
            state = airframe.state
            attitude = state.attitude_deg
            print("  %5.2f s %+6.1f %+6.1f %+6.1f  %+6.1f %+6.1f %+6.1f"
                  "  %+5.1f  %+5.1f  %+5.1f  %6.2f"
                  % (frame / 60.0, math.degrees(state.rates.x),
                     math.degrees(state.rates.y), math.degrees(state.rates.z),
                     attitude.x, attitude.y, attitude.z, state.velocity.x,
                     state.velocity.y, state.velocity.z, state.altitude))
        return airframe.state

    one_inch = 1.0
    step_response("one inch of forward stick",
                  long_stick=controls.long_stick + one_inch)
    step_response("one inch of right lateral stick",
                  lat_stick=controls.lat_stick + one_inch)
    step_response("one inch of right pedal", pedal=controls.pedal + one_inch)
    step_response("one inch of collective",
                  collective=controls.collective + one_inch)

    print()
    print("the rotor time constants of table 3, on a one inch forward stick"
          " step:")
    print("   time    R6 = 0.072 s (blade inertia)   R6 = 0.144 s (as flown)")
    slow = Airframe(rotor_time_constant=UH1_ROTOR_TIME_CONSTANT_SIM_S)
    slow_controls, slow_state = slow.trim_hover()
    airframe.reset(trim_state, controls)
    slow.reset(slow_state, slow_controls)
    for frame in range(int(1.5 * 60) + 1):
        if frame:
            airframe.step(1.0 / 60.0, PilotControls(
                collective=controls.collective, pedal=controls.pedal,
                long_stick=controls.long_stick + 1.0,
                lat_stick=controls.lat_stick))
            slow.step(1.0 / 60.0, PilotControls(
                collective=slow_controls.collective,
                pedal=slow_controls.pedal,
                long_stick=slow_controls.long_stick + 1.0,
                lat_stick=slow_controls.lat_stick))
        if frame % 12:
            continue
        print("  %5.2f s     q %+6.2f deg/s, tilt %+5.2f deg     q %+6.2f"
              " deg/s, tilt %+5.2f deg"
              % (frame / 60.0, math.degrees(airframe.state.rates.y),
                 math.degrees(airframe.state.attitude.y),
                 math.degrees(slow.state.rates.y),
                 math.degrees(slow.state.attitude.y)))

    print()
    _self_test()
    print("self test passed")


if __name__ == "__main__":
    _demo()
