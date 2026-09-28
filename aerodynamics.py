"""Quasi-steady blade element aerodynamics for a single helicopter rotor.

Models one rotor (default: the two bladed main rotor of a Vietnam era UH-1
Huey) as a line of sample points from the blade root out to the tip.  For
every sample point the caller supplies the speed and direction of the air at
the *leading edge* of that section, and the module

1. adds the velocity of the section itself (the rotor turning),
2. derives the local angle of attack from pitch, twist and inflow,
3. turns that into a section lift and drag force,
4. resolves it into thrust along the shaft and in plane drag, and
5. aggregates every point of every blade into one force and one moment
   acting at the base (hub) of the rotor.

The rest of the aircraft is out of scope: this module only answers "what does
the air do to this rotor, and what does this rotor do to the airframe".

Rotor frame, right handed, origin at the hub, shaft along +z::

    +x  forward (the way the nose points)
    +y  left (port);  x cross y = z, so the frame is right handed
    +z  up the shaft, roughly the thrust direction

A blade at azimuth psi lies along::

    e_r = ( cos psi,  sin psi, 0)   radial, outwards along the blade
    e_t = (-sin psi,  cos psi, 0)   tangential, the way the blade moves
    e_z = (0, 0, 1)                 along the shaft

(e_r, e_t, e_z) is right handed.  The UH-1 rotor turns counter clockwise seen
from above, which is a positive (right hand rule) rotation about +z, so its
angular velocity is +omega * e_z.  A consequence that makes a useful check:
flying along +x, the *advancing* blade is the one on the -y (starboard) side,
because that is where the rotation adds to the flight speed.  That is the sign
that pins this frame down, and it is why +y is port: with the nose and the
shaft fixed, a right handed frame cannot have +y both starboard and up.
:mod:`airframe` uses the report's own axes - x nose, y starboard, z down - so
its y and z are this module's negated.

In hover the rotor does not move through the airframe, so the only air motion
is the induced downwash through the disc, v_i = sqrt(T / (2 * rho * A)) from
momentum theory (:func:`momentum_theory_inflow`).  The rotor never applies
that implicitly - the caller passes it in as part of the airflow, which is
what the demo at the end of this file does.

UH-1 parameters come from NASA TM-73254 (Talbot and Corliss, 1977), table 2:
48 ft (14.63 m) diameter so R = 7.35 m, 21 in (0.53 m) chord, 2.75 deg of hub
precone, two blades, NACA 0012 section, 324 rpm, maximum gross weight 9500 lb
(4309 kg).  The table's two tip speeds, 760 ft/s (231.6 m/s) for the main
rotor and 740 ft/s (225.5 m/s) for the tail rotor, both belong to a main rotor
speed of about 301 rpm, a shade below 324 rpm, which is kept as the frozen
reference speed here.  The main to tail rotor drive ratio is 5.56:1, so the
tail rotor turns at 1801 rpm (tip speed 243.3 m/s) when the main rotor is at
the 324 rpm reference; that ratio is what ties the table's two tip speeds
together (:func:`tail_rotor_rpm`).  Vertipedia (Vertical Flight Society) gives
the UH-1C/M Model 540 rotor as 44 ft diameter with a 0.686 m (27 in) chord and
solidity 0.0651, and confirms the rotation direction; it is UH1_MODEL_540.

The tail rotor is the same table's 1.29 m radius, 0.2133 m chord and two blades
at 5.56 times the main rotor speed, with zero twist and 1.5 deg of built in
coning; see UH1_TAIL_TWIST_DEG and UH1_TAIL_PRECONE_DEG for why those two are
the values the model ships with, and Rotor.uh1h_tail for the rotor itself.  The
8.79 m arm from the main rotor hub to the tail rotor hub turns the main rotor
torque into the tail rotor thrust that has to balance it.

Reverse flow is handled explicitly.  When the tangential velocity U_T goes
negative, which happens near the retreating blade root at forward speed, the
inflow angle is measured from the magnitude of U_T and the angle of attack is
measured from the reversed chord.  The section therefore stays on a smooth,
symmetric polar and the in plane force keeps opposing the blade motion instead
of flipping sign as it used to.

Limitations, all deliberate: quasi steady (no dynamic stall, no wake); no
compressibility although the tip Mach number is 0.73; no tip loss, and no root
cut out unless root_cutout is set; an analytical section model (linear lift
curve, Viterna style post stall, flat plate drag) rather than wind tunnel
tables; and no lead lag.

Cyclic pitch and flapping are in, since a rotor that cannot tilt its own thrust
cannot fly: ``compute`` takes the once per revolution pitch and a
:class:`Flapping`, resolves every section's force against its own flapped
blade, and reports the flap hinge moment that the flapping solve balances
(:mod:`rotor_control`, which also owns the pilot-stick-to-blade-pitch mixing
and the induced velocity solve this module's caller needs).

Standard library only, so this module imports and tests without pygame, OpenGL
or numpy.
"""

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Union

#: Sea level ISA air density, kg/m^3.
RHO_SEA_LEVEL = 1.225

#: Standard gravity, m/s^2.
GRAVITY = 9.80665


@dataclass(frozen=True)
class Vector3:
    """An immutable 3 component vector, used for positions and velocities."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __neg__(self):
        return Vector3(-self.x, -self.y, -self.z)

    def __mul__(self, scalar):
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar):
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def dot(self, other):
        """Scalar (dot) product."""
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other):
        """Vector (cross) product."""
        return Vector3(self.y * other.z - self.z * other.y,
                       self.z * other.x - self.x * other.z,
                       self.x * other.y - self.y * other.x)

    def length(self):
        """Euclidean norm."""
        return math.sqrt(self.dot(self))

    def normalized(self):
        """Unit vector in the same direction (the zero vector stays zero)."""
        norm = self.length()
        if norm == 0.0:
            return Vector3()
        return self / norm

    def as_tuple(self):
        """Plain ``(x, y, z)`` tuple, for use outside this module."""
        return (self.x, self.y, self.z)

    @classmethod
    def from_iterable(cls, values):
        """Build a vector from any 3 element iterable."""
        x, y, z = values
        return cls(float(x), float(y), float(z))


# ---------------------------------------------------------------------------
# Rotor parameters.  Two documented UH-1 rotor systems are provided.
# ---------------------------------------------------------------------------

#: Bell UH-1H (Model 205) main rotor - the classic Vietnam era Huey.  Geometry
#: from NASA TM-73254 table 2: 24.13 ft (7.35 m) radius, 1.75 ft (0.53 m)
#: chord, 2.75 deg hub precone, two blades, NACA 0012, 324 rpm.
UH1_RADIUS = 7.35            # m, 24.13 ft
UH1_CHORD = 0.53             # m, 1.75 ft
UH1_BLADE_COUNT = 2          # -
UH1_RPM = 324.0              # rpm, 100 % rotor speed
UH1_TWIST_DEG = -10.0        # deg, linear washout from the root to the tip
UH1_PRECONE_DEG = 2.75       # deg, built in coning (TM-73254 table 2)
UH1_HUB_STATION_M = 133.5    # in, fuselage station of the main rotor hub
UH1_HUB_WATERLINE_M = 136.5  # in, waterline of the main rotor hub
UH1_MAX_GROSS_MASS = 4309.0  # kg, 9500 lb
UH1_EMPTY_MASS = 2365.0      # kg, 5215 lb
UH1_WEIGHT_N = UH1_MAX_GROSS_MASS * GRAVITY   # 42257 N at maximum weight

#: Rotor speed points on the UH-1H operational band, rpm.  TM 55-1520-210-10
#: gives 324 rpm as 100 %; the normal (green arc) band spans 294 to 339 rpm.
UH1_RPM_LOW = 294.0          # rpm, bottom of the normal (green arc) range
UH1_RPM_100 = 324.0          # rpm, 100 % rotor speed
UH1_RPM_MAX = 339.0          # rpm, maximum / autorotation overspeed limit

#: UH-1H tail rotor (TM-73254 table 2): 4.25 ft (1.29 m) radius, 0.70 ft
#: (21.33 cm) chord, solidity 0.105, tip speed 740 ft/s (225.5 m/s).  The main
#: to tail rotor drive ratio is 5.56:1 (the 42 degree and 90 degree gearboxes
#: step the speed up), i.e. the tail rotor turns 5.56 revolutions per main rotor
#: revolution, so 1801.4 rpm and 243.3 m/s of tip speed at the 324 rpm
#: reference.  The table's 740 ft/s is reproduced by that ratio at the same
#: 301 rpm reference as its 760 ft/s main rotor tip speed,
#: 301.1 * 5.56 * 2 pi * 1.29 / 60 = 226.0 m/s, so the two figures agree.
UH1_TAIL_MAIN_RATIO = 5.56   # -, tail rotor revolutions per main rotor rev
UH1_TAIL_RADIUS = 1.29       # m, 4.25 ft
UH1_TAIL_CHORD = 0.2133      # m, 21.33 cm
UH1_TAIL_BLADE_COUNT = 2     # -
UH1_TAIL_SOLIDITY = 0.105    # -, N * c / (pi * R)
UH1_TAIL_ARM = 8.79          # m, tail rotor hub behind the main rotor hub
UH1_TAIL_RPM = UH1_RPM * UH1_TAIL_MAIN_RATIO   # rpm, 1801.44 at the reference

#: Tail rotor blade twist and built in coning.  Neither number is in TM-73254
#: table 2 nor in TM 55-1520-210-10, so both come from the blade element survey
#: the self test runs, and both are set to what that survey supports:
#:
#: twist 0.  A tail rotor has to produce thrust in both directions (right pedal
#: and left pedal, powered and autorotating), and a symmetric NACA 0012 blade
#: with no twist does that with one blade design.  Washout buys little either:
#: at the 2180 N that balances the main rotor torque in a 4309 kg hover, zero
#: twist needs 7.85 deg of collective at a figure of merit of 0.664, -4 deg
#: needs 10.85 at 0.672 and -10 deg needs 15.35 at 0.680.  The untwisted blade
#: does run its inboard fifth at a negative angle of attack (down to -52 deg at
#: the innermost station), so that part of the blade makes a little negative
#: thrust, about 4 % of the total, which is real and small.
#:
#: precone 1.5 to 2 deg.  The flight manual has the tail rotor blades running on
#: a flapping axis, so they cone to wherever the aerodynamic moment about the
#: flap hinge balances the centrifugal field.  The model puts that at 0.88 deg
#: for a 4 kg blade and 0.35 deg for a 10 kg blade, rising to 1.12 and 0.45 deg
#: at 130 % of the hover thrust the torque balance sets.  A built in coning of
#: 1.5 to 2 deg is therefore a mounting choice that holds the blades off the
#: cone plane in hover and, more usefully, keeps them off the droop stop and the
#: tail boom when the thrust goes away (left pedal, autorotation).  It raises
#: the tips by R * sin(precone), 34 mm at 1.5 deg and 45 mm at 2 deg, and that
#: clearance is the one number here that only a drawing could confirm.
UH1_TAIL_TWIST_DEG = 0.0     # deg, untwisted blade
UH1_TAIL_PRECONE_DEG = 1.5   # deg, built in coning, the low end of 1.5 to 2.0

#: Centre of gravity travel and mast tilt (TM-73254 table 2).  Stations and
#: waterlines are in inches on the UH-1H fuselage reference system.
UH1_CG_STATION_FWD = 130.0   # in, most forward CG station
UH1_CG_STATION_AFT = 144.0   # in, most aft CG station
UH1_CG_WATERLINE = 55.0      # in, CG waterline
UH1_MAST_TILT_DEG = 5.0      # deg, mast tilted forward

#: Full throw control travels (TM-73254 table 2).  The same table gives the
#: pitch each throws, which is what makes these numbers usable: the
#: longitudinal stick sweeps +12 to -11 deg of cyclic pitch, the lateral stick
#: +9 to -11 deg, and the pedals +18 to -10 deg of tail rotor collective pitch.
#: :mod:`rotor_control` carries the ranges and the rigging biases read off them;
#: each one agrees with the gearings below to within a degree, which is how the
#: two tables were cross checked against each other.
UH1_COLLECTIVE_TRAVEL_IN = 11.0   # in, collective stick, full throw
UH1_LONG_STICK_TRAVEL_IN = 12.9   # in, longitudinal cyclic stick
UH1_LAT_STICK_TRAVEL_IN = 12.6    # in, lateral cyclic stick
UH1_PEDAL_TRAVEL_IN = 6.9         # in, pedals, full throw

#: Pilot to swashplate control gearing (TM-73254 table 3, every value from its
#: ref. 9).  These are the linkage constants that turn a pilot control position
#: in inches into rotor blade pitch in radians, so they sit on top of the stick
#: travels above: the 12.9 in longitudinal stick throw at c1 sweeps 23.9 deg of
#: cyclic pitch and the 12.6 in lateral throw at c4 sweeps 20.0 deg, against
#: cyclic pitch ranges of +12 to -11 deg and +9 to -11 deg in table 2, and the
#: 6.9 in of pedal at c6 sweeps 28.1 deg against a table 2 tail rotor
#: collective of +18 to -10 deg.  The table labels c1 and c4 as the longitudinal
#: and lateral cyclic stick to swashplate constants, c5 as collective pitch to
#: collective stick and c6 (with c7, which the table does not carry) as tail
#: rotor collective pitch to pedal motion.  Its metric column gives 0.0127,
#: 0.0109, 0.0098 and -0.028 rad/cm for the same four.  :mod:`rotor_control`
#: does that mixing, including the rigging biases that table 2 carries with the
#: ranges.
UH1_LONG_CYCLIC_PER_IN = 0.0324   # rad/in, c1, long. cyclic per in of stick
UH1_LAT_CYCLIC_PER_IN = 0.0277    # rad/in, c4, lat. cyclic per in of stick
UH1_COLLECTIVE_PER_IN = 0.025     # rad/in, c5, collective per in of stick
UH1_TAIL_PEDAL_PER_IN = -0.071    # rad/in, c6, tail collective per in of pedal

#: Rotor flap dynamics, from TM-73254 table 3.  R6 is the rotor time
#: constant ``16 / (gamma * omega)``, given there as 0.144 sec together with a
#: footnote that the value from the inertia of one blade should really be
#: 0.072 sec; the slower 0.144 sec was kept in the simulation because pilots
#: judged the quicker rotor harder to fly.  Both are carried here: the
#: physical one fixes the Lock number the flap model uses,
#: ``gamma = 16 / (tau * omega) = 6.55`` at 324 rpm, and the simulated one is
#: available for anyone who wants to reproduce the NASA handling qualities.
#: A Lock number of 6.55 on this blade means a flap inertia about the hub of
#: ``rho * a * c * R ** 4 / gamma``, about 1.7e3 kg m^2, i.e. the inertia of a
#: 92 kg blade spread evenly along the span.  That is the right order for a
#: UH-1H metal blade and it is the one number table 3 does not carry.
UH1_ROTOR_TIME_CONSTANT_S = 0.072       # sec, 16 / (gamma * omega), blade inertia
UH1_ROTOR_TIME_CONSTANT_SIM_S = 0.144   # sec, table 3 R6 as flown in the simulation

#: Collective and pedal move through the same first order lag; table 3 gives
#: 0.20 sec for it, matched to the acceleration data of the flight tests.
UH1_COLLECTIVE_TIME_CONSTANT_S = 0.20   # sec, table 3, tau_C

#: Bell stabilizer bar (TM-73254 table 3).  The bar feeds a fraction of the
#: cyclic pitch into the rotor in parallel with the pilot: a linkage mixing
#: ratio K_phi = 0.16 through its own damping time constant tau_B = 3.3 sec,
#: whose product K_B = K_phi * tau_B is the 0.528 sec numerator the table
#: lists.
UH1_BAR_MIXING_RATIO = 0.16             # dimensionless, K_phi
UH1_BAR_TIME_CONSTANT_S = 3.3           # sec, tau_B, mechanical damping of the bar
UH1_BAR_NUMERATOR_S = 0.528             # sec, K_B = K_phi * tau_B

#: Control rigging (TM-73254 table 2 and the control equations on its page 5).
#: The lateral cyclic is rigged 2 deg left, which is what keeps a UH-1H level
#: in hover with a centred stick.  The longitudinal and lateral pilot inputs
#: are mixed through the same rigging phase phi_p, 5 deg on the aircraft,
#: which the NASA simulation set to zero and which is therefore the default
#: here too; ``UH1_RIGGING_PHASE_DEG`` is the aircraft's real value for anyone
#: who wants the cross-coupling.
UH1_LATERAL_RIGGING_DEG = -2.0          # deg, 2 deg left, a tilt to port
UH1_RIGGING_PHASE_DEG = 5.0             # deg, phi_p, the aircraft as rigged
UH1_RIGGING_PHASE_SIM_DEG = 0.0         # deg, phi_p, what the simulation used

#: Bell UH-1C/M Model 540 main rotor: 44 ft (13.41 m) diameter, 27 in
#: (0.686 m) chord, solidity 0.0651 (Vertipedia, Vertical Flight Society).
UH1_MODEL_540 = dict(radius=6.705, chord=0.686, blade_count=2, rpm=324.0,
                     twist_deg=-10.0)


def momentum_theory_inflow(thrust, radius, air_density=RHO_SEA_LEVEL):
    """Mean induced velocity through the disc, m/s, from momentum theory.

    ``v_i = sqrt(T / (2 * rho * A))`` with ``A = pi * R ** 2``.  This is the
    axial (hover or climb) result; use it to build the airflow that the rotor
    is then asked about.
    """
    area = math.pi * radius * radius
    if thrust <= 0.0 or area <= 0.0 or air_density <= 0.0:
        return 0.0
    return math.sqrt(thrust / (2.0 * air_density * area))


def tail_rotor_rpm(main_rotor_rpm=UH1_RPM, ratio=UH1_TAIL_MAIN_RATIO):
    """Tail rotor speed, rpm, at a given main rotor speed.

    The drive steps the speed up through the 42 degree and 90 degree
    gearboxes, so the tail rotor turns *ratio* (5.56) revolutions for every
    main rotor revolution: 1801.4 rpm when the main rotor is at the 324 rpm
    reference.
    """
    return float(main_rotor_rpm) * float(ratio)


def tail_rotor_tip_speed(main_rotor_rpm=UH1_RPM, ratio=UH1_TAIL_MAIN_RATIO,
                         radius=UH1_TAIL_RADIUS):
    """Tail rotor tip speed, m/s, at a given main rotor speed."""
    omega = tail_rotor_rpm(main_rotor_rpm, ratio) * 2.0 * math.pi / 60.0
    return omega * float(radius)


def _as_vector(value):
    """Coerce a Vector3 or a 3 element iterable into a Vector3."""
    if isinstance(value, Vector3):
        return value
    return Vector3.from_iterable(value)


def uniform_airflow(velocity):
    """An airflow function returning the same air velocity at every point.

    The velocity is given in the rotor frame: hover is ``(0, 0, -v_i)``, the
    air being drawn down through the disc, and forward flight at speed V along
    +x is ``(-V, 0, 0)``.
    """
    velocity = _as_vector(velocity)

    def airflow(position):
        return velocity

    return airflow


def as_airflow(airflow):
    """Accept a Vector3, a 3 element iterable or a callable; return a callable.

    A callable is handed the leading edge position of each sample point, in
    the rotor frame, and must return the air velocity there as a Vector3 or a
    3 element iterable.  That is the hook for wind fields, downwash from other
    rotors, ground effect, and so on.
    """
    if callable(airflow):
        return lambda position: _as_vector(airflow(position))
    return uniform_airflow(airflow)


@dataclass(frozen=True)
class Flapping:
    """The blade flapping of one rotor, as a cone angle plus a once per rev tilt.

    A blade flaps out of the hub plane as it goes round.  Over one revolution
    that motion is the mean cone angle ``coning`` plus a first harmonic, the
    tilt of the tip path plane::

        beta(psi) = coning - long * cos(psi) - lat * sin(psi)
        beta'(psi) = long * sin(psi) - lat * cos(psi)      d/d psi
        beta_dot = omega * beta'                          rad/s, the flap rate

    ``psi`` is measured from the nose in the rotor frame of this module, so
    the signs read straight off the geometry:

    ``coning``  the blade sits this far above the hub plane on average; the
                hub precone is the angle the blades are built with and the
                aerodynamic cone angle is what the flap hinge moment balances.
    ``long``    positive means the blade is *low* at the nose and high at the
                tail.  A disc low at the nose has its normal, and with it the
                thrust vector, leaning forward, which pitches the nose down.
    ``lat``     positive means the blade is low on the starboard side, so the
                tip path plane and the thrust vector lean to starboard and the
                aircraft rolls right.

    The flap rate matters because a blade moving up sees more air coming down
    through the disc, which is the damping that sets where the tip path plane
    settles with respect to the cyclic pitch, and it is also the reason a
    teetering rotor can solve for its tilt at all (the tilt appears in the
    moment only through ``beta_dot``, never through ``beta`` itself).
    """

    coning: float = 0.0     # rad, a0, mean cone angle above the hub plane
    long: float = 0.0       # rad, a1, the cos(psi) harmonic
    lat: float = 0.0        # rad, b1, the sin(psi) harmonic

    def angle_at(self, azimuth_deg):
        """Flap angle, rad, of a blade at *azimuth_deg*."""
        psi = math.radians(azimuth_deg)
        return (self.coning - self.long * math.cos(psi)
                - self.lat * math.sin(psi))

    def derivative_at(self, azimuth_deg):
        """``d beta / d psi`` at *azimuth_deg*, dimensionless."""
        psi = math.radians(azimuth_deg)
        return self.long * math.sin(psi) - self.lat * math.cos(psi)

    def rate_at(self, azimuth_deg, omega):
        """Flap rate, rad/s, of a blade at *azimuth_deg* turning at *omega*."""
        return omega * self.derivative_at(azimuth_deg)

    @property
    def tilt_rad(self):
        """Magnitude of the once per rev tilt of the tip path plane, rad."""
        return math.hypot(self.long, self.lat)

    @property
    def tilt_azimuth_deg(self):
        """Azimuth, deg, of the *low* side of the tip path plane.

        The thrust vector, perpendicular to the disc, leans the other way, so
        this is the direction the rotor pushes opposite to.
        """
        return math.degrees(math.atan2(self.lat, self.long))

    @property
    def coning_deg(self):
        return math.degrees(self.coning)

    @property
    def long_deg(self):
        return math.degrees(self.long)

    @property
    def lat_deg(self):
        return math.degrees(self.lat)

    @property
    def tilt_deg(self):
        return math.degrees(self.tilt_rad)

    @classmethod
    def with_deg(cls, coning_deg=0.0, long_deg=0.0, lat_deg=0.0):
        """Build a Flapping from angles in degrees."""
        return cls(math.radians(coning_deg), math.radians(long_deg),
                   math.radians(lat_deg))

    def __str__(self):
        return ("coning %5.2f deg | tilt %5.2f deg at %6.1f deg azimuth"
                % (self.coning_deg, self.tilt_deg, self.tilt_azimuth_deg))


@dataclass
class Section:
    """Analytical NACA 0012 section model, shared by every sample point.

    A linear lift curve up to the stall angle with a Viterna style post stall
    extension that stays smooth all the way to 90 degrees of attack and mirrors
    past it, and a drag polar ``Cd0 + k * Cl ** 2`` that blends into a flat
    plate level once stalled.  The pitching moment about the quarter chord is
    zero while attached, because the section is symmetric, and grows toward a
    flat plate value once stalled.  This is a readable stand-in for wind tunnel
    tables, and the same method names can be backed by a table later, not a
    replacement for them.
    """

    lift_slope: float = 5.73         # per radian; NACA 0012, about 0.1 / deg
    zero_lift_deg: float = 0.0       # symmetric section: zero lift at 0 deg
    stall_angle_deg: float = 14.0    # deg, end of the linear lift curve
    profile_drag: float = 0.009      # Cd at zero lift
    drag_due_to_lift: float = 0.017  # k in Cd = Cd0 + k * Cl ** 2
    flat_plate_drag: float = 1.11    # Cd of a flat plate at 90 degrees
    cm_ac: float = 0.0               # attached pitching moment, symmetric
    cp_offset: float = 0.25          # flat plate centre of pressure offset

    @property
    def cl_max(self):
        """Lift coefficient where the linear curve reaches the stall."""
        return self.lift_slope * math.radians(self.stall_angle_deg)

    def _viterna_constants(self):
        """Coefficients of the Viterna style post stall extension.

        Chosen so the post stall curves join the attached values at the stall
        angle exactly, following Viterna and Corrigan (1982) with the flat
        plate drag level of this section.
        """
        stall = math.radians(self.stall_angle_deg)
        cl_stall = self.lift_slope * stall
        cd_stall = self.profile_drag + self.drag_due_to_lift * cl_stall ** 2
        b1 = self.flat_plate_drag
        a1 = 0.5 * b1
        a2 = ((cl_stall - a1 * math.sin(2.0 * stall)) * math.sin(stall)
              / math.cos(stall) ** 2)
        b2 = (cd_stall - b1 * math.sin(stall) ** 2) / math.cos(stall)
        return a1, a2, b1, b2

    def _positive_coefficients(self, alpha):
        """``(Cl, Cd)`` for an angle of attack in ``[0, pi]`` radians.

        Angles from 0 to 90 degrees use the attached curve and the Viterna
        extension.  Past 90 degrees the section is in reversed flow, so the
        curve mirrors about 90 degrees and the lift sign flips.
        """
        stall = math.radians(self.stall_angle_deg)
        if alpha <= stall:
            cl = self.lift_slope * alpha
            return cl, self.profile_drag + self.drag_due_to_lift * cl * cl
        if alpha <= 0.5 * math.pi:
            a1, a2, b1, b2 = self._viterna_constants()
            cl = (a1 * math.sin(2.0 * alpha)
                  + a2 * math.cos(alpha) ** 2 / math.sin(alpha))
            cd = b1 * math.sin(alpha) ** 2 + b2 * math.cos(alpha)
            return cl, cd
        cl, cd = self._positive_coefficients(math.pi - alpha)
        return -cl, cd

    def lift_coefficient(self, alpha_deg):
        """Cl for an angle of attack in degrees (signed, symmetric)."""
        alpha = math.radians(alpha_deg)
        sign = 1.0 if alpha >= 0.0 else -1.0
        cl, _ = self._positive_coefficients(min(abs(alpha), math.pi))
        return sign * cl

    def drag_coefficient(self, alpha_deg, cl=None):
        """Cd for an angle of attack in degrees.

        The drag comes from the same folded curve as the lift, so *cl* is
        accepted for backwards compatibility and then ignored.
        """
        alpha = math.radians(alpha_deg)
        _, cd = self._positive_coefficients(min(abs(alpha), math.pi))
        return cd

    def moment_coefficient(self, alpha_deg):
        """Cm about the quarter chord for an angle of attack in degrees.

        The section is symmetric, so the attached value is zero and the curve
        is odd in the angle of attack; once stalled the moment grows toward the
        flat plate value, a normal force acting behind the quarter chord.
        """
        alpha = math.radians(alpha_deg)
        stall = math.radians(self.stall_angle_deg)
        sign = 1.0 if alpha >= 0.0 else -1.0
        angle = min(abs(alpha), math.pi)
        if angle <= stall:
            return sign * self.cm_ac
        blend = min((angle - stall) / (0.5 * math.pi - stall), 1.0)
        cm_flat_plate = -self.cp_offset * 2.0 * math.sin(angle)
        return sign * (self.cm_ac + (cm_flat_plate - self.cm_ac) * blend)

    def coefficients(self, alpha_deg):
        """``(Cl, Cd)`` for an angle of attack in degrees."""
        cl = self.lift_coefficient(alpha_deg)
        return cl, self.drag_coefficient(alpha_deg, cl)


@dataclass
class SectionSample:
    """Everything the blade element model worked out at one sample point."""

    blade_index: int            # which blade, 0 based
    azimuth_deg: float          # azimuth of that blade
    radius: float               # station radius along the blade, m
    span: float                 # width of the strip this station stands for
    position: Vector3           # leading edge point, in the rotor frame
    air_velocity: Vector3       # air speed and direction given by the caller
    blade_velocity: Vector3     # omega x r, the section moving through the air
    relative_velocity: Vector3  # air velocity minus blade velocity
    speed: float                # |relative_velocity|, m/s
    dynamic_pressure: float     # 0.5 * rho * speed ** 2, Pa
    pitch_deg: float            # local blade pitch, collective plus twist
    inflow_deg: float           # phi
    angle_of_attack_deg: float  # alpha
    cl: float
    cd: float
    cm: float                   # pitching moment about the quarter chord
    lift_per_span: float        # q * chord * Cl, N/m
    drag_per_span: float        # q * chord * Cd, N/m
    thrust_per_span: float      # along +e_z, N/m
    in_plane_per_span: float    # along -e_t, opposing the rotation, N/m
    force: Vector3              # force on this strip, N
    moment: Vector3             # moment of that force about the hub, N m
    hinge_moment: Vector3       # moment of that force about the flap hinge
    cone_deg: float = 0.0       # flap angle of this blade at this azimuth
    flap_moment: float = 0.0    # N m, flap hinge moment, positive lifting the blade


@dataclass
class RotorLoads:
    """The combined effect of one rotor, aggregated at the base (hub)."""

    thrust: float               # N, along +z; the useful force
    force: Vector3              # N, total force on the hub, rotor frame
    moment: Vector3             # N m, total moment about the hub, rotor frame
    shaft_torque: float         # N m, torque the engine must supply
    power: float                # W, shaft torque times omega
    h_force: float              # N, in plane force along +x (rotor drag)
    side_force: float           # N, in plane force along +y
    radius: float               # m, rotor radius of this evaluation
    omega: float                # rad/s used
    air_density: float          # kg/m^3 used
    collective_pitch_deg: float
    azimuth_deg: float
    sample_count: int           # stations per blade
    blade_count: int
    hinge_moment: Vector3 = field(default_factory=Vector3)
    samples: List[SectionSample] = field(default_factory=list)
    cyclic_long_deg: float = 0.0
    cyclic_lat_deg: float = 0.0
    flapping: Optional[Flapping] = None
    flap_moment: float = 0.0    # N m, flap hinge moment summed over every blade

    @property
    def disk_area(self):
        """Disc area, m^2."""
        return math.pi * self.radius * self.radius

    @property
    def in_plane_force(self):
        """Magnitude of the in plane (horizontal) force on the hub, N."""
        return math.hypot(self.force.x, self.force.y)

    @property
    def ideal_hover_power(self):
        """Ideal induced power for this thrust, W (momentum theory)."""
        if self.thrust <= 0.0:
            return 0.0
        return self.thrust * momentum_theory_inflow(self.thrust, self.radius,
                                                    self.air_density)

    @property
    def figure_of_merit(self):
        """Ideal induced power divided by the actual shaft power.

        A real rotor lands between about 0.6 and 0.8 in hover, so this is a
        good single number with which to sanity check a set of parameters.
        """
        if self.power <= 0.0:
            return 0.0
        return self.ideal_hover_power / self.power

    def __str__(self):
        return ("thrust %8.0f N | torque %8.0f Nm | power %7.1f kW | "
                "H %6.0f N | FoM %5.3f"
                % (self.thrust, self.shaft_torque, self.power / 1000.0,
                   self.h_force, self.figure_of_merit))


@dataclass
class Rotor:
    """One rotor: geometry, section and the blade element evaluation."""

    radius: float = UH1_RADIUS
    chord: float = UH1_CHORD
    blade_count: int = UH1_BLADE_COUNT
    rpm: float = UH1_RPM
    twist_deg: float = UH1_TWIST_DEG
    precone_deg: float = UH1_PRECONE_DEG   # deg, built in coning of the hub
    hinge_offset: float = 0.0   # m, flap/teeter hinge outboard of the shaft
    root_cutout: float = 0.0    # m, no sample points inside this radius
    sample_count: int = 20      # stations per blade, 20 by default
    air_density: float = RHO_SEA_LEVEL
    section: Section = field(default_factory=Section)

    @classmethod
    def uh1h(cls, **overrides):
        """The UH-1H (Model 205) main rotor, with optional field overrides."""
        fields = dict(radius=UH1_RADIUS, chord=UH1_CHORD,
                      blade_count=UH1_BLADE_COUNT, rpm=UH1_RPM,
                      twist_deg=UH1_TWIST_DEG, precone_deg=UH1_PRECONE_DEG)
        fields.update(overrides)
        return cls(**fields)

    @classmethod
    def uh1_model_540(cls, **overrides):
        """The UH-1C/M Model 540 main rotor (44 ft disc, 27 in chord)."""
        fields = dict(UH1_MODEL_540)
        fields.update(overrides)
        return cls(**fields)

    @classmethod
    def uh1h_tail(cls, **overrides):
        """The UH-1H tail rotor: 1.29 m radius, 0.2133 m chord, two blades.

        It spins 5.56 times faster than the main rotor and carries no twist,
        which keeps the symmetric NACA 0012 section equally good for thrust
        either way, and it flies behind UH1_TAIL_PRECONE_DEG of built in
        coning.  Note that ``compute`` still resolves the blades in the disc
        plane, so for now the precone only sets the flapping reference that the
        teeter step will use rather than changing the loads; override
        ``twist_deg`` or ``precone_deg`` to fly the alternatives.
        """
        fields = dict(radius=UH1_TAIL_RADIUS, chord=UH1_TAIL_CHORD,
                      blade_count=UH1_TAIL_BLADE_COUNT, rpm=UH1_TAIL_RPM,
                      twist_deg=UH1_TAIL_TWIST_DEG,
                      precone_deg=UH1_TAIL_PRECONE_DEG)
        fields.update(overrides)
        return cls(**fields)

    @property
    def omega(self):
        """Angular velocity of the shaft, rad/s."""
        return self.rpm * 2.0 * math.pi / 60.0

    @property
    def tip_speed(self):
        """Speed of the blade tip, m/s."""
        return self.omega * self.radius

    @property
    def disk_area(self):
        """Disc area, m^2."""
        return math.pi * self.radius * self.radius

    @property
    def solidity(self):
        """Rotor solidity: total blade area over disc area, N * c / (pi * R)."""
        return self.blade_count * self.chord / (math.pi * self.radius)

    @property
    def strip_width(self):
        """Span each sample point stands for, m."""
        return (self.radius - self.root_cutout) / float(self.sample_count)

    def stations(self):
        """Radii of the sample points, m.

        ``sample_count`` equally spaced points, each in the middle of its own
        strip, i.e. the midpoint rule.  Midpoints rather than strip edges so
        that no sample sits exactly on the root, where the dynamic pressure
        vanishes but the inflow angle blows up, and so that summing the strips
        integrates a linear load distribution exactly.
        """
        width = self.strip_width
        return [self.root_cutout + (index + 0.5) * width
                for index in range(self.sample_count)]

    def pitch_deg(self, radius, collective_pitch_deg, cyclic_long_deg=0.0,
                  cyclic_lat_deg=0.0, azimuth_deg=0.0):
        """Local blade pitch at *radius* and *azimuth_deg*.

        Three things add up: the collective pitch at the root, the linear
        twist, and the cyclic pitch, which is the once per revolution part of
        the pitch the swashplate commands::

            theta(r, psi) = collective + twist * (r / R)
                            + long * cos(psi) + lat * sin(psi)

        ``cyclic_long_deg`` is therefore the pitch when the blade is over the
        nose and ``cyclic_lat_deg`` the pitch when it is over the starboard
        side, both in the rotor frame of this module.  With the rotor turning
        counter clockwise seen from above, a blade flaps about a quarter of a
        turn *after* the pitch that lifts it, so these two coefficients are a
        quarter turn away from the control axis the pilot commands, and that
        quarter turn is the whole reason a helicopter needs a control rigging
        (:mod:`rotor_control` does that mixing).

        The collective pitch is the pitch at the blade root, so with the
        UH-1's -10 deg of washout the tip flies 10 deg flatter than the root.
        """
        psi = math.radians(azimuth_deg)
        return (collective_pitch_deg + self.twist_deg * (radius / self.radius)
                + cyclic_long_deg * math.cos(psi)
                + cyclic_lat_deg * math.sin(psi))

    @property
    def flap_inertia(self):
        """Flap inertia of one blade about the flap hinge, kg m^2.

        ``I_b = rho * a * c * R ** 4 / gamma``, the definition of the Lock
        number turned around, with the Lock number the rotor time constant of
        TM-73254 table 3 gives: ``gamma = 16 / (tau * omega)``.  The flap model
        needs this to turn the hinge moment into an angle, and it is also what
        says how heavy the blade is: evenly spread over the span, this inertia
        belongs to a blade of ``3 * I_b / R ** 2`` kg.
        """
        slope = self.section.lift_slope
        return (self.air_density * slope * self.chord * self.radius ** 4
                / self.lock_number)

    @property
    def lock_number(self):
        """Lock number of one blade, ``gamma = 16 / (tau * omega)``.

        TM-73254 table 3 gives the rotor time constant 16 / (gamma * omega) as
        0.072 sec from the inertia of one blade (its 0.144 sec entry is the
        doubled value the simulation used), which at 324 rpm is this Lock
        number.  Around 6 is typical of an articulated or teetering rotor, and
        it is what the flapping solution and the control axis lag both hang
        off, so it is computed rather than tabulated.
        """
        tau = UH1_ROTOR_TIME_CONSTANT_S
        if self.rpm <= 0.0:
            return 0.0
        return 16.0 / (tau * self.omega)

    @property
    def rotor_time_constant(self):
        """Time constant of the rotor's response to cyclic, sec, 16 / (gamma w).

        The first order surrogate the NASA simulation used for the rotor and
        control axis lag, from the Lock number of this rotor rather than from
        the tabulated 0.144 sec, so that it stays consistent when the geometry
        or the rotor speed are overridden.
        """
        if self.lock_number <= 0.0:
            return 0.0
        return 16.0 / (self.lock_number * self.omega)

    def flap_centrifugal_ratio(self):
        """``K_c / I_b`` for a uniform blade, the centrifugal stiffening ratio.

        The flap equation of a hinged blade is
        ``I_b * beta_ddot + omega**2 * K_c * beta = M`` where
        ``I_b = int (r - e)**2 dm`` and ``K_c = int r (r - e) dm``.  For a
        teetering rotor, whose hinge is on the shaft, ``e`` is zero and this
        ratio is exactly 1, which is the classic ``beta'' + beta = M / I`` form
        and the reason a teetering rotor settles at a tilt that makes the once
        per rev flap moment vanish: only the mean of the moment is left to set
        the cone angle.  A hinge further out makes the ratio larger and lets
        the aero moment leak into the tilt, which is how a rotor with a hinge
        offset transmits a hub moment.
        """
        e = self.hinge_offset
        radius = self.radius
        inertia = (radius - e) ** 3 / 3.0
        centrifugal = ((radius ** 3 - e ** 3) / 3.0
                       - e * (radius ** 2 - e ** 2) / 2.0)
        return centrifugal / inertia if inertia > 0.0 else 1.0

    @property
    def blade_mass_equivalent(self):
        """Mass of a uniform blade with this flap inertia, kg, 3 * I_b / R ** 2."""
        return 3.0 * self.flap_inertia / self.radius ** 2

    def azimuths(self, azimuth_deg=0.0):
        """Azimuth of every blade in degrees, evenly spaced around the disc."""
        return [azimuth_deg + 360.0 * index / float(self.blade_count)
                for index in range(self.blade_count)]

    def describe(self):
        """A short human readable summary of the geometry, for printouts."""
        return ("%.2f m radius (%.1f ft disc), %d blades, chord %.3f m (%.1f in)"
                ", precone %.2f deg, %.3f rad/s, tip speed %.1f m/s"
                ", solidity %.4f"
                % (self.radius, 2.0 * self.radius * 3.28084, self.blade_count,
                   self.chord, self.chord * 39.3701, self.precone_deg,
                   self.omega, self.tip_speed, self.solidity))

    def compute(self,
                airflow: Union[Vector3, Sequence[float],
                               Callable[[Vector3], Vector3]],
                collective_pitch_deg: float,
                azimuth_deg: float = 0.0,
                angular_velocity: Optional[float] = None,
                air_density: Optional[float] = None,
                cyclic_long_deg: float = 0.0,
                cyclic_lat_deg: float = 0.0,
                flapping: Optional[Flapping] = None) -> RotorLoads:
        """Run the blade element model and aggregate the result at the hub.

        ``airflow`` is the speed and direction of the air at the sample points:
        either one Vector3 / 3 element iterable used everywhere, or a callable
        ``airflow(leading_edge_position) -> Vector3`` for a varying field.
        ``collective_pitch_deg`` is the blade pitch at the root, the twist
        being added on top of it.  ``azimuth_deg`` positions blade 0, and the
        other blades follow evenly around the disc.  ``angular_velocity``
        (rad/s) and ``air_density`` (kg/m^3) default to this rotor's own.

        ``cyclic_long_deg`` and ``cyclic_lat_deg`` are the once per revolution
        pitch: the coefficients of cos(psi) and of sin(psi), i.e. the pitch over
        the nose and the pitch over the starboard side.  With both zero, and no
        ``flapping``, this is the axisymmetric rotor the module started with.

        ``flapping`` adds the blade motion out of the hub plane.  A blade at
        azimuth psi is lifted by ``beta``, so it spans along
        ``e_rb = cos(beta) e_r + sin(beta) e_z`` and its normal, the direction
        its section lift acts along, is ``e_n = cos(beta) e_z - sin(beta) e_r``.
        The section also moves with the flap, adding ``(r - e) * beta_dot`` to
        its velocity along that normal.  Three consequences, all of them the
        ones a flight model needs:

        * every section's thrust follows ``e_n``, so the summed force tilts with
          the tip path plane instead of staying on the shaft, which is how a
          teetering rotor steers at all;
        * the flap rate enters U_P, so a blade moving up sees more downflow and
          loses lift, which is the damping that makes solving for the flapping
          angles well posed;
        * :attr:`SectionSample.flap_moment` and :attr:`RotorLoads.flap_moment`
          come out of the same pass, so the flapping loop can be closed without
          evaluating anything twice.

        For every station r of every blade::

            beta    = cone angle + first harmonic at this azimuth
            V_blade = omega * r * e_t + (r - e) * beta_dot * e_n
            V       = airflow(leading edge) - V_blade   flow it sees
            U_T     = -(V . e_t)  in plane, set by omega * r
            U_P     = -(V . e_n)  through the disc, set by climb / inflow
            q       = 0.5 * rho * |V| ** 2              dynamic pressure
            phi     = atan2(U_P, U_T)                   inflow angle
            theta   = collective + twist * (r / R) + cyclic
            alpha   = theta - phi                       angle of attack

            lift_per_span   = q * chord * Cl(alpha)     N/m
            drag_per_span   = q * chord * Cd(alpha)     N/m
            thrust_per_span = lift * cos(phi) - drag * sin(phi)   along +e_n
            in_plane_span   = lift * sin(phi) + drag * cos(phi)   along -e_t

            F = (thrust_per_span * e_n - in_plane_span * e_t) * dr
            M = p cross F
            flap moment = -(moment about the flap hinge) . e_t

        Returns a :class:`RotorLoads` holding the summed force and moment at
        the hub plus every per point :class:`SectionSample`, so the caller can
        inspect the spanwise distribution as well as the total.
        """
        omega = self.omega if angular_velocity is None else float(angular_velocity)
        rho = self.air_density if air_density is None else float(air_density)
        if self.sample_count < 1:
            raise ValueError("sample_count must be at least 1")
        if self.blade_count < 1:
            raise ValueError("blade_count must be at least 1")

        air_at = as_airflow(airflow)
        strip = self.strip_width
        up = Vector3(0.0, 0.0, 1.0)
        quarter_chord = 0.25 * self.chord

        samples = []
        total_force = Vector3()
        total_moment = Vector3()
        total_hinge_moment = Vector3()
        total_flap_moment = 0.0
        hinge_offset = self.hinge_offset

        for blade_index, psi_deg in enumerate(self.azimuths(azimuth_deg)):
            psi = math.radians(psi_deg)
            e_r = Vector3(math.cos(psi), math.sin(psi), 0.0)
            e_t = Vector3(-math.sin(psi), math.cos(psi), 0.0)

            # Flapping, if any, turns the blade out of the hub plane about its
            # flap hinge.  With no flapping the blade frame is the hub frame and
            # every line below is the one the axisymmetric code used.
            if flapping is None:
                beta = 0.0
                beta_dot = 0.0
            else:
                beta = flapping.angle_at(psi_deg)
                beta_dot = flapping.rate_at(psi_deg, omega)
            cos_beta = math.cos(beta)
            sin_beta = math.sin(beta)
            e_rb = e_r * cos_beta + up * sin_beta      # out along the blade
            e_n = up * cos_beta - e_r * sin_beta       # the way it lifts

            for radius in self.stations():
                theta_deg = self.pitch_deg(radius, collective_pitch_deg,
                                           cyclic_long_deg, cyclic_lat_deg,
                                           psi_deg)
                theta = math.radians(theta_deg)

                # The sample point sits on the leading edge: the feathering
                # axis is at the quarter chord, so the leading edge is a
                # quarter chord ahead of it, lifted by the pitch angle.
                position = (e_rb * radius
                            + (e_t * math.cos(theta) + e_n * math.sin(theta))
                            * quarter_chord)

                air_velocity = air_at(position)
                # The section moves with the rotation and with the flap, which
                # turns about a hinge hinge_offset out along the blade.
                blade_velocity = (e_t * (omega * radius)
                                  + e_n * ((radius - hinge_offset) * beta_dot))
                relative = air_velocity - blade_velocity

                # Both are positive when they raise the angle of attack: U_T
                # comes from omega * r, U_P from descent or induced inflow.
                u_t = -relative.dot(e_t)
                u_p = -relative.dot(e_n)
                speed = relative.length()
                dynamic_pressure = 0.5 * rho * speed * speed

                # Reverse flow: near the retreating blade root the air moves
                # with the blade, so the trailing edge leads.  The tangential
                # speed is kept positive for the inflow angle, and the angle of
                # attack is measured from the reversed chord.  That keeps the
                # section on a smooth, symmetric polar and keeps the in plane
                # force, the drag, opposing the blade motion.
                reverse = u_t < 0.0
                u_t = abs(u_t)
                phi = math.atan2(u_p, u_t)
                if reverse:
                    alpha_deg = theta_deg + math.degrees(phi)
                else:
                    alpha_deg = theta_deg - math.degrees(phi)

                cl, cd = self.section.coefficients(alpha_deg)
                cm = self.section.moment_coefficient(alpha_deg)
                lift_per_span = dynamic_pressure * self.chord * cl
                drag_per_span = dynamic_pressure * self.chord * cd

                cos_phi = math.cos(phi)
                sin_phi = math.sin(phi)
                thrust_per_span = lift_per_span * cos_phi - drag_per_span * sin_phi
                in_plane_per_span = (lift_per_span * sin_phi
                                     + drag_per_span * cos_phi)

                # Thrust acts along the blade normal, the in plane drag opposes
                # the motion.
                force = (e_n * thrust_per_span
                         - e_t * in_plane_per_span) * strip
                moment = position.cross(force)
                # The pitching moment about the quarter chord acts along the
                # blade span; positive Cm is taken nose up.
                moment += e_rb * (dynamic_pressure * self.chord ** 2 * cm * strip)
                # The same force about the flap hinge, hinge_offset outboard of
                # the shaft, gives the flap moment arm the rotor dynamics use.
                hinge_moment = (position - e_rb * hinge_offset).cross(force)
                # The flap angle grows about -e_t, so this is the moment that
                # lifts the blade, the one the centrifugal spring balances.
                flap_moment = -hinge_moment.dot(e_t)

                total_force += force
                total_moment += moment
                total_hinge_moment += hinge_moment
                total_flap_moment += flap_moment
                samples.append(SectionSample(
                    blade_index=blade_index, azimuth_deg=psi_deg,
                    radius=radius, span=strip, position=position,
                    air_velocity=air_velocity, blade_velocity=blade_velocity,
                    relative_velocity=relative, speed=speed,
                    dynamic_pressure=dynamic_pressure, pitch_deg=theta_deg,
                    inflow_deg=math.degrees(phi),
                    angle_of_attack_deg=alpha_deg, cl=cl, cd=cd, cm=cm,
                    lift_per_span=lift_per_span, drag_per_span=drag_per_span,
                    thrust_per_span=thrust_per_span,
                    in_plane_per_span=in_plane_per_span,
                    force=force, moment=moment, hinge_moment=hinge_moment,
                    cone_deg=math.degrees(beta), flap_moment=flap_moment))

        # The aerodynamic torque about the shaft is the z part of the hub
        # moment and it opposes the rotation, so the engine has to supply its
        # negative.
        shaft_torque = -total_moment.z
        return RotorLoads(
            thrust=total_force.z, force=total_force, moment=total_moment,
            hinge_moment=total_hinge_moment,
            flap_moment=total_flap_moment,
            shaft_torque=shaft_torque, power=shaft_torque * omega,
            h_force=total_force.x, side_force=total_force.y,
            radius=self.radius, omega=omega, air_density=rho,
            collective_pitch_deg=collective_pitch_deg,
            azimuth_deg=azimuth_deg, sample_count=self.sample_count,
            blade_count=self.blade_count, samples=samples,
            cyclic_long_deg=cyclic_long_deg, cyclic_lat_deg=cyclic_lat_deg,
            flapping=flapping)

    def mean_loads(self, airflow, collective_pitch_deg, azimuth_steps=36,
                   **kwargs):
        """Average the hub loads over a whole revolution of the rotor.

        One azimuth is only one instant.  With two blades in forward flight
        the loads really do pulse twice per revolution, so averaging the force
        and moment vectors over evenly spaced azimuths gives the steady part
        that the airframe feels.  The returned samples are those at azimuth 0,
        as a representative snapshot.
        """
        if azimuth_steps < 1:
            raise ValueError("azimuth_steps must be at least 1")
        force = Vector3()
        moment = Vector3()
        hinge_moment = Vector3()
        flap_moment = 0.0
        for step in range(azimuth_steps):
            loads = self.compute(airflow, collective_pitch_deg,
                                 azimuth_deg=360.0 * step / float(azimuth_steps),
                                 **kwargs)
            force += loads.force
            moment += loads.moment
            hinge_moment += loads.hinge_moment
            flap_moment += loads.flap_moment

        mean = self.compute(airflow, collective_pitch_deg, **kwargs)
        mean.force = force / azimuth_steps
        mean.moment = moment / azimuth_steps
        mean.hinge_moment = hinge_moment / azimuth_steps
        mean.flap_moment = flap_moment / azimuth_steps
        mean.thrust = mean.force.z
        mean.h_force = mean.force.x
        mean.side_force = mean.force.y
        mean.shaft_torque = -mean.moment.z
        mean.power = mean.shaft_torque * mean.omega
        return mean


def glauert_induced_velocity(thrust, radius, omega, forward_speed=0.0,
                             climb_speed=0.0, air_density=RHO_SEA_LEVEL,
                             ground_effect=1.0):
    """Induced velocity, m/s, momentum theory gives a rotor in any flight.

    The inflow ratio ``lam_i = v_i / (omega * R)`` solves the implicit Glauert
    relation::

        lam_i = K_G * C_T / (2 * sqrt(mu ** 2 + (mu_z + lam_i) ** 2))
        C_T   = T / (rho * A * (omega * R) ** 2)
        mu    = V / (omega * R)          in the disc plane
        mu_z  = w / (omega * R)          through it, positive climbing

    which TM-73254 carries as its equation 10, noting that it "is an implicit
    function and required an iterative solution in the computer program".  The
    ground effect factor K_G, which the TM multiplies in to represent the
    reduction in inflow near the ground, is a parameter here because the table
    of K_G values is one of the parts its OCR lost; 1.0 is out of ground
    effect, and a value below 1.0 is what ground effect does.

    The solution is a bisection, because the right hand side falls as the
    inflow rises, so the residual is monotone and a bisection cannot oscillate
    the way a fixed point iteration can when the rotor is slow to make thrust.
    """
    omega = float(omega)
    rho = float(air_density)
    area = math.pi * radius * radius
    if thrust <= 0.0 or area <= 0.0 or rho <= 0.0 or omega <= 0.0:
        return 0.0
    tip_speed = omega * radius
    thrust_coefficient = thrust / (rho * area * tip_speed * tip_speed)
    mu = abs(float(forward_speed)) / tip_speed
    mu_z = float(climb_speed) / tip_speed
    gain = max(float(ground_effect), 0.0)

    if mu < 1e-9 and abs(mu_z) < 1e-9:
        # Pure hover or axial climb: lam ** 2 = K_G * C_T / 2 exactly.
        return tip_speed * math.sqrt(0.5 * gain * thrust_coefficient)

    def target(lam):
        axial = mu_z + lam
        return (gain * thrust_coefficient
                / (2.0 * math.sqrt(mu * mu + axial * axial)))

    low, high = 0.0, max(4.0 * target(0.0), 1e-6)
    for _ in range(80):
        middle = 0.5 * (low + high)
        if middle - target(middle) < 0.0:
            low = middle
        else:
            high = middle
    return tip_speed * 0.5 * (low + high)


def solve_inflow(thrust_at, radius, omega, forward_speed=0.0, climb_speed=0.0,
                 air_density=RHO_SEA_LEVEL, ground_effect=1.0, inflow=0.0,
                 iterations=30, tolerance=1e-3):
    """Induced velocity that agrees with the thrust it produces, m/s.

    ``thrust_at(inflow)`` returns the thrust, N, the rotor makes when the air
    is drawn down through the disc at ``inflow`` m/s.  Momentum theory gives
    the inflow a thrust needs and the blade element model gives the thrust an
    inflow makes, so the two are solved against each other.

    The residual is monotone over the range a rotor actually flies in, so a
    bracketed false position iteration (Illinois) is used and it lands in
    about five evaluations.  If no bracket can be found, which is what a
    collective too low to support a steady induced flow looks like, the
    routine falls back to a damped fixed point and reports ``converged``
    False rather than handing back a number nobody should use: with too little
    pitch there is no hover to converge to, and the honest answer is the zero
    inflow limit.

    Returns ``(inflow, evaluations, converged)``.
    """
    def momentum(value):
        return glauert_induced_velocity(max(thrust_at(value), 0.0), radius,
                                        omega, forward_speed, climb_speed,
                                        air_density, ground_effect)

    def residual(value):
        return value - momentum(value)

    low = max(float(inflow), 0.0)
    f_low = residual(low)
    evaluations = 1
    if abs(f_low) <= tolerance:
        return low, evaluations, True

    # The momentum theory answer for the thrust at the starting point is the
    # natural other end of the bracket, and it is on the far side of the root
    # whenever a root exists at all.
    high = momentum(low)
    if high <= low:
        high = low + 1.0
    f_high = residual(high)
    evaluations += 2
    if abs(f_high) <= tolerance:
        return high, evaluations, True

    if f_low * f_high <= 0.0:
        for _ in range(iterations):
            middle = high - f_high * (high - low) / (f_high - f_low)
            if not low < middle < high:
                middle = 0.5 * (low + high)
            f_middle = residual(middle)
            evaluations += 1
            if abs(f_middle) <= tolerance or high - low <= 1e-9:
                return middle, evaluations, True
            if f_middle * f_low > 0.0:
                low, f_low = middle, f_middle
                f_high *= 0.5          # Illinois: keep the far end awake
            else:
                high, f_high = middle, f_middle
                f_low *= 0.5
        return 0.5 * (low + high), evaluations, False

    # No bracket: there is no steady induced flow to settle at, so ease toward
    # the fixed point and say so.
    value = low
    for _ in range(iterations):
        target = momentum(value)
        evaluations += 1
        if abs(target - value) <= tolerance:
            return target, evaluations, True
        value = 0.5 * (value + target)
    return value, evaluations, False


def hover_loads(rotor, collective_pitch_deg, air_density=RHO_SEA_LEVEL,
                iterations=30, tolerance=1e-4, **kwargs):
    """Hover at a given collective: thrust and induced flow settle together.

    Momentum theory gives the inflow for a thrust, but the thrust depends on
    the inflow, so the two are solved against each other
    (:func:`solve_inflow`).  Returns the converged induced velocity (m/s) and
    the rotor loads.  A collective too low to hold a steady induced flow
    returns the best iteration and the zero inflow loads rather than failing,
    which keeps :func:`collective_for_thrust` bisecting through the low end;
    call :func:`solve_inflow` directly when the convergence flag matters.
    """
    def thrust_at(value):
        return rotor.compute(uniform_airflow(Vector3(0.0, 0.0, -value)),
                             collective_pitch_deg, air_density=air_density,
                             **kwargs).thrust

    inflow, _, _ = solve_inflow(thrust_at, rotor.radius, rotor.omega,
                                air_density=air_density, iterations=iterations,
                                tolerance=tolerance)
    loads = rotor.compute(uniform_airflow(Vector3(0.0, 0.0, -inflow)),
                          collective_pitch_deg, air_density=air_density,
                          **kwargs)
    return inflow, loads


def collective_for_thrust(rotor, target_thrust, air_density=RHO_SEA_LEVEL,
                          iterations=25, **kwargs):
    """Bisect the root collective pitch that hovers at *target_thrust*.

    The induced velocity of one step is carried into the next, which is what
    keeps the whole bisection to a couple of blade element passes per step
    instead of an iteration up from rest every time.
    """
    def loads_at(collective, inflow):
        return inflow, rotor.compute(uniform_airflow(Vector3(0.0, 0.0, -inflow)),
                                     collective, air_density=air_density,
                                     **kwargs)

    def thrust_at(collective, depth=8):
        def thrust(value):
            return rotor.compute(uniform_airflow(Vector3(0.0, 0.0, -value)),
                                 collective, air_density=air_density,
                                 **kwargs).thrust
        inflow, _, _ = solve_inflow(thrust, rotor.radius, rotor.omega,
                                    air_density=air_density, inflow=depth)
        return inflow

    low, high = 0.0, 45.0
    middle, inflow = 0.5 * (low + high), 0.0
    for _ in range(iterations):
        middle = 0.5 * (low + high)
        inflow, loads = loads_at(middle, thrust_at(middle, inflow))
        if loads.thrust < target_thrust:
            low = middle
        else:
            high = middle
    return 0.5 * (low + high)


def _demo():
    rotor = Rotor.uh1h()
    print("UH-1H main rotor")
    print("  " + rotor.describe())
    print("  maximum gross mass %.0f kg, weight %.0f N"
          % (UH1_MAX_GROSS_MASS, UH1_WEIGHT_N))
    print("  tail rotor %.0f rpm at %.2f tail revs per main rev, tip speed"
          " %.1f m/s"
          % (UH1_TAIL_RPM, UH1_TAIL_MAIN_RATIO, tail_rotor_tip_speed()))

    collective = collective_for_thrust(rotor, UH1_WEIGHT_N)
    inflow, loads = hover_loads(rotor, collective)
    print()
    print("hover at maximum weight: root collective pitch %.2f deg" % collective)
    print("  inflow %.2f m/s, inflow angle at the tip %.2f deg"
          % (inflow, loads.samples[-1].inflow_deg))
    print("  " + str(loads))

    print()
    print("blade 0, sample points every %.3f m of span:" % rotor.strip_width)
    print("   r/R   r [m]  pitch  inflow   alpha      Cl      Cd      Cm"
          "   lift/span  thrust/span")
    for sample in loads.samples:
        if sample.blade_index:
            continue
        print("  %5.2f  %6.3f  %6.2f  %6.2f  %6.2f  %6.3f  %6.4f  %7.3f"
              "  %9.1f  %11.1f"
              % (sample.radius / rotor.radius, sample.radius, sample.pitch_deg,
                 sample.inflow_deg, sample.angle_of_attack_deg, sample.cl,
                 sample.cd, sample.cm, sample.lift_per_span,
                 sample.thrust_per_span))

    print()
    print("steady loads, averaged over a revolution:")
    for speed in (0.0, 30.0, 55.0):
        airflow = uniform_airflow(Vector3(-speed, 0.0, -inflow))
        print("  %5.1f m/s forward:  %s"
              % (speed, rotor.mean_loads(airflow, collective)))

    print()
    print("reverse flow check, retreating blade at %.0f m/s (U_T < 0 near the"
          " root):" % 90.0)
    airflow = uniform_airflow(Vector3(-90.0, 0.0, -inflow))
    reverse_loads = rotor.compute(airflow, collective, azimuth_deg=90.0)
    print("   r/R   r [m]  pitch  inflow   alpha      Cl      Cd      Cm"
          "   in-plane")
    for sample in reverse_loads.samples:
        if sample.blade_index:
            continue
        print("  %5.2f  %6.3f  %6.2f  %6.2f  %6.2f  %6.3f  %6.4f  %7.3f"
              "  %9.1f"
              % (sample.radius / rotor.radius, sample.radius, sample.pitch_deg,
                 sample.inflow_deg, sample.angle_of_attack_deg, sample.cl,
                 sample.cd, sample.cm, sample.in_plane_per_span))

    print()
    print("thrust against the number of sample points per blade:")
    reference = hover_loads(Rotor.uh1h(sample_count=80), collective)[1].thrust
    for count in (5, 10, 20, 40, 80):
        _, coarse = hover_loads(Rotor.uh1h(sample_count=count), collective)
        print("  %3d points: %9.0f N  (%6.2f %% of the 80 point answer)"
              % (count, coarse.thrust, 100.0 * coarse.thrust / reference))

    print()
    print("the other documented rotor, for comparison:")
    print("  UH-1C/M Model 540: " + Rotor.uh1_model_540().describe())
    model_540 = Rotor.uh1_model_540()
    collective_540 = collective_for_thrust(model_540, UH1_WEIGHT_N)
    _, loads_540 = hover_loads(model_540, collective_540)
    print("  hover at maximum weight with %.2f deg collective: %s"
          % (collective_540, loads_540))

    print()
    print("tail rotor, an untwisted blade at %.2f times the main rotor speed:"
          % UH1_TAIL_MAIN_RATIO)
    tail = Rotor.uh1h_tail()
    print("  " + tail.describe())
    tail_thrust = loads.shaft_torque / UH1_TAIL_ARM
    tail_inflow, tail_loads = hover_loads(tail, 8.0)
    print("  %.0f N of thrust balances the main rotor torque, and 8 deg of"
          " collective gives %.0f N at %.1f m/s of inflow"
          % (tail_thrust, tail_loads.thrust, tail_inflow))
    print("  " + str(tail_loads))

    _self_test()
    print()
    print("self test passed")


def _self_test():
    """Cheap checks on the section model, the reverse flow fix and geometry.

    Raises AssertionError on failure.  The demo calls this so that simply
    running this module is enough to validate the analytics.
    """
    section = Section()
    for alpha in range(-180, 181, 5):
        cl = section.lift_coefficient(alpha)
        cd = section.drag_coefficient(alpha)
        cm = section.moment_coefficient(alpha)
        assert abs(cl + section.lift_coefficient(-alpha)) < 1e-9, alpha
        assert abs(cd - section.drag_coefficient(-alpha)) < 1e-9, alpha
        assert abs(cm + section.moment_coefficient(-alpha)) < 1e-9, alpha
        assert cd >= section.profile_drag - 1e-12, alpha

    # The post stall curves must join the attached curves at the stall angle.
    stall = section.stall_angle_deg
    for eps in (1e-6, 1e-3, 0.1):
        assert abs(section.lift_coefficient(stall + eps)
                   - section.lift_coefficient(stall - eps)) < 0.05
        assert abs(section.drag_coefficient(stall + eps)
                   - section.drag_coefficient(stall - eps)) < 0.05

    # At 90 degrees the lift vanishes and the drag is at the flat plate level.
    assert abs(section.lift_coefficient(90.0)) < 1e-9
    assert abs(section.drag_coefficient(90.0) - section.flat_plate_drag) < 1e-9

    rotor = Rotor.uh1h()
    assert abs(rotor.solidity - 0.046) < 0.002
    collective = collective_for_thrust(rotor, UH1_WEIGHT_N)
    inflow, loads = hover_loads(rotor, collective)
    assert loads.thrust > 0.0

    # Tail rotor drive: 5.56 tail revolutions per main rotor revolution, i.e.
    # 1801.4 rpm at the 324 rpm reference.  That ratio is what reconciles the
    # two tip speeds of TM-73254 table 2: the 760 ft/s main rotor figure puts
    # the main rotor at 301 rpm, and 5.56 times that on a 1.29 m tail rotor
    # gives the 740 ft/s (225.5 m/s) the table quotes for the tail rotor.
    assert abs(UH1_TAIL_RPM - 1801.44) < 0.01
    assert abs(tail_rotor_rpm() - 1801.44) < 0.01
    assert abs(tail_rotor_rpm(294.0) - 1634.64) < 0.01
    table_main_rpm = 60.0 * 231.6 / (2.0 * math.pi * UH1_RADIUS)
    assert abs(table_main_rpm - 300.9) < 0.5
    assert abs(tail_rotor_tip_speed(table_main_rpm) - 225.5) < 2.0
    assert abs(tail_rotor_tip_speed() - 243.3) < 0.1

    # Control gearing.  The table 3 linkage constants are what turn the table 2
    # stick travels into cyclic pitch: 23.9 deg of longitudinal pitch over the
    # 12.9 in stick throw and 20.0 deg of lateral pitch over 12.6 in.  The
    # legible part of the table 2 cyclic range, +12 deg to -11 deg, is the same
    # order, so gearing and travels are consistent with each other.
    long_pitch_deg = math.degrees(UH1_LONG_CYCLIC_PER_IN
                                  * UH1_LONG_STICK_TRAVEL_IN)
    lat_pitch_deg = math.degrees(UH1_LAT_CYCLIC_PER_IN
                                 * UH1_LAT_STICK_TRAVEL_IN)
    assert abs(long_pitch_deg - 23.9) < 0.1, long_pitch_deg
    assert abs(lat_pitch_deg - 20.0) < 0.1, lat_pitch_deg
    assert UH1_COLLECTIVE_PER_IN > 0.0
    # Right pedal raises the tail rotor collective, and table 3 carries that
    # sense as a negative gearing per inch of pedal.
    assert UH1_TAIL_PEDAL_PER_IN < 0.0

    # Tail rotor blade element check.  Neither the twist nor the built in
    # coning is in the reference tables, so the values the model ships with are
    # pinned here along with the loads that justify them: 8 deg of collective on
    # the untwisted blade covers the thrust that balancing the main rotor torque
    # demands, and the coning the blade wants about a root hinge stays far below
    # the built in coning of the hub.
    tail_thrust = loads.shaft_torque / UH1_TAIL_ARM
    tail = Rotor.uh1h_tail(sample_count=10)
    assert abs(tail.solidity - UH1_TAIL_SOLIDITY) < 0.002
    assert tail.twist_deg == 0.0
    assert 1.5 <= tail.precone_deg <= 2.0
    tail_inflow, tail_loads = hover_loads(tail, 8.0)
    assert tail_inflow > 5.0
    assert abs(tail_loads.thrust - tail_thrust) < 0.25 * tail_thrust, \
        tail_loads.thrust
    assert tail_loads.figure_of_merit > 0.6
    blade = [s for s in tail_loads.samples if s.blade_index == 0]
    flap_moment = sum((s.radius - tail.hinge_offset) * s.thrust_per_span
                      * s.span for s in blade)
    root_inertia = sum((s.radius - tail.hinge_offset) * s.radius * s.span
                       / tail.radius for s in blade)
    for mass in (4.0, 10.0):
        coning = math.degrees(flap_moment
                              / (tail.omega ** 2 * root_inertia * mass))
        assert 0.2 < coning < 1.3, (mass, coning)

    # Forward flight: the inflow angle must stay inside +/- 90 degrees, and
    # every reversed flow station must have its in plane load opposing motion.
    reversed_seen = 0
    for speed in (30.0, 55.0, 90.0):
        airflow = uniform_airflow(Vector3(-speed, 0.0, -inflow))
        for step in range(180):
            flight = rotor.compute(airflow, collective,
                                   azimuth_deg=2.0 * step)
            for sample in flight.samples:
                # The old code reported inflow angles near 168 degrees here.
                assert -90.0001 <= sample.inflow_deg <= 90.0001
                psi = math.radians(sample.azimuth_deg)
                e_t = Vector3(-math.sin(psi), math.cos(psi), 0.0)
                u_t = -sample.relative_velocity.dot(e_t)
                if u_t < 0.0 and sample.dynamic_pressure > 1.0:
                    reversed_seen += 1
                    assert sample.cd > 0.0
                    # Away from the near axial band the reversed section is
                    # still attached, and its in plane load must oppose the
                    # blade motion rather than drive it.
                    if abs(sample.angle_of_attack_deg) < 90.0:
                        assert sample.in_plane_per_span > 0.0
    assert reversed_seen > 0, "no reverse flow stations were generated"

    # The hinge moment equals the hub moment with no offset, and differs once
    # the hinge moves outboard.
    airflow = uniform_airflow(Vector3(0.0, 0.0, -inflow))
    plain = rotor.compute(airflow, collective)
    assert (plain.hinge_moment - plain.moment).length() < 1e-9
    hinged = Rotor.uh1h(hinge_offset=0.3)
    offset = hinged.compute(airflow, collective)
    assert (offset.hinge_moment - offset.moment).length() > 1e-6

    # The flapping harmonics read the way the geometry says: a positive cos
    # harmonic is a blade low at the nose, a positive sin harmonic is a blade
    # low to starboard, and the rate is omega times the slope.
    flap = Flapping(0.05, 0.1, -0.03)
    assert abs(flap.angle_at(0.0) - 0.05 + 0.1) < 1e-12
    assert abs(flap.angle_at(180.0) - 0.05 - 0.1) < 1e-12
    assert abs(flap.angle_at(90.0) - (0.05 + 0.03)) < 1e-12
    assert abs(flap.derivative_at(0.0) - 0.03) < 1e-12
    assert abs(flap.rate_at(90.0, rotor.omega) - rotor.omega * 0.1) < 1e-12
    assert abs(flap.tilt_rad - math.hypot(0.1, 0.03)) < 1e-12
    assert abs(flap.tilt_azimuth_deg - math.degrees(math.atan2(-0.03, 0.1))) < 1e-9
    assert abs(Flapping.with_deg(2.0, 3.0, 4.0).coning - math.radians(2.0)) < 1e-12

    # A flapping that is low at the nose leans the thrust forward, one low to
    # starboard leans it to starboard, and the tilt is a real force: this is the
    # whole reason the flapping is in the model.  The mean is the honest place
    # to look, because the once per revolution in plane forces cancel over a
    # revolution.
    level = rotor.mean_loads(airflow, collective,
                             flapping=Flapping(0.05, 0.0, 0.0))
    forward_flap = rotor.mean_loads(airflow, collective,
                                    flapping=Flapping(0.05, 0.05, 0.0))
    assert forward_flap.force.x > 1000.0, forward_flap.force.x
    assert abs(forward_flap.thrust - level.thrust) < 0.02 * level.thrust
    sided = rotor.mean_loads(airflow, collective,
                             flapping=Flapping(0.05, 0.0, 0.05))
    assert sided.force.y > 1000.0, sided.force.y
    # At a quarter turn the two blades are flapping hardest in opposite
    # directions, so their hinge moments differ: blade 0 and blade 1 are half a
    # turn apart, which is why a harmonic analysis of the flapping has to be
    # done on the blades of one blade and not on the total.
    tilted = rotor.compute(airflow, collective, azimuth_deg=90.0,
                           flapping=Flapping(0.05, 0.05, 0.0))

    # A coned rotor with a once per revolution tilt carries a cross axis force
    # as well: coning tips each blade's thrust inboard, so a thrust that varies
    # around the azimuth (which the flap rate makes it do) no longer cancels.
    # It is real, it is why a rotor with cyclic has some pitch and roll coupling,
    # and it disappears the moment the coning does - which is the sharp way to
    # test that the mechanism in the code is this one and not something else.
    assert abs(forward_flap.force.y) < 0.75 * forward_flap.force.x, \
        forward_flap.force
    flat_cone = rotor.mean_loads(airflow, collective,
                                 flapping=Flapping(0.0, 0.05, 0.0))
    assert abs(flat_cone.force.y) < 1e-6, flat_cone.force
    assert abs(flat_cone.force.x) > 1000.0, flat_cone.force

    # Blade 0 and blade 1 are half a turn apart, so a tilt makes their flap
    # hinge moments differ while their mean stays put: the once per rev part
    # cancels between them, which is why a harmonic analysis of the flapping
    # has to be done on one blade.
    blade_0 = sum(s.flap_moment for s in tilted.samples if s.blade_index == 0)
    blade_1 = sum(s.flap_moment for s in tilted.samples if s.blade_index == 1)
    if abs(blade_1) > 1.0:
        assert abs(blade_0 - blade_1) > 0.01 * abs(blade_0)

    # Cyclic pitch, and where it puts the blade: the pitch over the nose, the
    # pitch over the starboard side, the collective and the twist underneath
    # both, and a rotor at one azimuth with the two blades a few degrees either
    # side of each other.
    assert abs(rotor.pitch_deg(0.0, collective, 4.0, -2.0, 0.0)
               - (collective + 4.0)) < 1e-9
    assert abs(rotor.pitch_deg(0.0, collective, 4.0, -2.0, 90.0)
               - (collective - 2.0)) < 1e-9
    assert abs(rotor.pitch_deg(0.0, collective, 4.0, -2.0, 180.0)
               - (collective - 4.0)) < 1e-9
    plain_90 = rotor.compute(airflow, collective, azimuth_deg=90.0)
    cyclic_90 = rotor.compute(airflow, collective, azimuth_deg=90.0,
                              cyclic_lat_deg=3.0)
    starboard = [s.pitch_deg for s in cyclic_90.samples if s.blade_index == 0]
    port = [s.pitch_deg for s in cyclic_90.samples if s.blade_index == 1]
    assert abs(starboard[0] - port[0] - 6.0) < 1e-9
    assert starboard[0] > [s.pitch_deg for s in plain_90.samples][0]
    assert (cyclic_90.force - plain_90.force).length() > 1.0

    # Inflow: the Glauert relation has to hold when the answer is put back into
    # it, the hover case has to reduce to momentum theory, and a forward speed
    # has to drop the inflow.
    axial = momentum_theory_inflow(UH1_WEIGHT_N, rotor.radius)
    assert abs(glauert_induced_velocity(UH1_WEIGHT_N, rotor.radius, rotor.omega)
               - axial) < 1e-9
    cruise = glauert_induced_velocity(UH1_WEIGHT_N, rotor.radius, rotor.omega,
                                      forward_speed=55.0)
    assert 0.0 < cruise < axial
    tip_speed = rotor.tip_speed
    thrust_coefficient = UH1_WEIGHT_N / (RHO_SEA_LEVEL * rotor.disk_area
                                         * tip_speed ** 2)
    mu = 55.0 / tip_speed
    lam = cruise / tip_speed
    assert abs(lam - thrust_coefficient
               / (2.0 * math.sqrt(mu * mu + lam * lam))) < 1e-6

    def hover_thrust(value):
        return rotor.compute(uniform_airflow(Vector3(0.0, 0.0, -value)),
                             collective).thrust

    solved, evaluations, converged = solve_inflow(hover_thrust, rotor.radius,
                                                  rotor.omega)
    assert converged and evaluations >= 1
    assert abs(solved - inflow) < 0.05, (solved, inflow)
    # A rotor that makes no thrust needs no induced flow, and says so.
    assert solve_inflow(lambda value: -1.0, rotor.radius,
                        rotor.omega)[0] == 0.0


if __name__ == "__main__":
    _demo()
