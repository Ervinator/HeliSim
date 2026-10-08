"""The engine on the other end of the shaft: a T53 and its N2 governor.

:mod:`airframe` closes the rotor's speed with ``omega_dot = (Q_engine - Q_drag)
/ I`` and leaves ``Q_engine`` to whoever is flying the aircraft.  This module
is that whoever: the Lycoming T53-L-13 of TM 55-1520-210-10, a *free turbine*
whose power turbine (N2) turns 20.37 revolutions for every revolution of the
main rotor, driving the transmission the tail rotor hangs off as well.

The airframe only ever asks one question - what torque is at the main rotor
shaft this frame - and the answer comes back in the same newton metres the
report's equation 4 gives its ``Q`` in, so the two meet without either knowing
the other's units.

**The numbers, and where each comes from.**  Nothing below is invented except
where it says so; the page references are TM 55-1520-210-10 as text.

*ratio.*  The manual's own two figures are 324 rotor / 6600 engine rpm and 314
rotor / 6400 engine rpm (7.1-13 and 7.1-15): the two ratios are 20.370 and
20.382, a fifth of a per cent apart, which is a cross check on the gearing
rather than a choice between them.  The first pair is the one used here because
the report's rotor reference is 324 rpm.

*The speed the governor holds.*  N2 is held at 6600 rpm at the 324 rpm rotor
reference, and a droop compensator - "a direct mechanical linkage between the
collective stick and the speed selector lever on the N2 governor ... will hold
N2 rpm to +-40 rpm when properly rigged" (2-23) - is what keeps it there as the
collective is raised.  The same paragraph defines droop as "the speed change in
engine rpm (N2) as power is increased from a no-load condition" and says why
the governor is built that way: without it the governor would hunt, and "if N2
power were allowed droop other than momentarily the reduction in rotor speed
could become critical".  That definition is the model's too: the gain is set so
that full power costs the whole 40 rpm of the band, no load costs none, and
anything in between is proportional.

*Torque available.*  1125 ft-lb is "the indicated torque pressure at 1125 ft-lbs
actual output shaft torque" - the data plate calibration (7.1-14) - and the
transmission's structural limit is "50 psi calibrated torque" (7.1-13).  Both
are needed to say how much torque the shaft can take, and the manual's own
caution is that the engine "can exceed the transmission structural limit under
certain conditions", which the density scaling here reproduces: at sea level
the available torque is the transmission limit and nothing more, and in cold
dense air it goes past it.

*The two ways the available torque is reduced.*  The GOV AUTO/EMER switch: "the
maximum engine torque available for any ambient condition will be reduced by 6
to 8 PSI when the GOV AUTO/EMER switch is placed in the EMER position" (9-3),
and in EMER "automatic acceleration, deceleration, and overspeed control are
not provided" so "manual control of the engine RPM" is the pilot's (9-3, EMER
GOV OPNS) - i.e. the twist grip sets the fuel and nothing holds N2.  And bleed
air: "decrease torque available 1.4 psi for heater on and 2.1 for device on;
decrease torque available 3.5 psi if both bleed air heater and device are
operating" (7.1-15).  7 psi of the 6 to 8, and the manual's 1.4, 2.1 and 3.5,
are the numbers here; each is a reduction in the available torque, expressed
the only way the manual expresses it, in psi of calibrated torque.

*What is this project's own choice*, and named as such:

* **the slope between psi and newton metres.**  The manual never prints it - it
  lives inside the torque available charts, figure 7.1-2, which are scans of
  graphs of calibrated torque against pressure altitude and temperature.  What
  this module takes instead is the one reading of them the text supports: the
  50 psi transmission structural limit at sea level standard, which is where
  7.1-13 puts the engine's own capability too.  So 100 % of the gauge is 50
  psi, and :data:`UH1_TORQUE_NM_PER_PSI` is 621.4 N m of main rotor shaft
  torque for each one of them - which is what makes 7 psi of EMER worth 14 per
  cent of the torque available, and a trimmed hover 26 psi;
* **the gas producer's lag.**  A turbine does not reach a new power level in
  the frame a lever moves: the fuel control schedules it and the compressor has
  to wind up first.  Neither report carries a number, so
  :data:`UH1_ENGINE_TIME_CONSTANT_S` is 0.6 sec, the order of a T53's own spool
  time, and it is this project's second choice about the engine - the first
  being the rotor inertia in :mod:`airframe`, and the reason is the same one: a
  first order lag on a number nobody published is honest, and an instant torque
  is not;
* **density and altitude.**  Torque available falls with pressure altitude and
  with air temperature (7.1-12), and both of those reach the model as one
  number, the air density the airframe is already flying at, so the available
  torque is scaled by ``rho / rho_0``.  That is the shape of the charts and not
  their numbers: the charts themselves, or a digitised figure 7.1-2, would
  replace :meth:`Governor.power_available` and nothing else.

Standard library only, like the rest of the model, so this module imports and
tests without pygame, OpenGL or numpy.
"""

import math
from dataclasses import dataclass, field

from aerodynamics import (RHO_SEA_LEVEL, UH1_RPM, UH1_RPM_LOW, UH1_RPM_MAX,
                          UH1_RPM_100)

# ---------------------------------------------------------------------------
# The engine's own numbers, TM 55-1520-210-10.
# ---------------------------------------------------------------------------

#: Newton metres in a pound-foot.  The manual's own conversion table at the
#: back of the book rounds it to 1.356, which is the figure a reader checking
#: this against the page will find.
FT_LB_TO_NM = 1.3558179483314004

#: Watts in a mechanical horsepower, for the one check that says the torque
#: below is a 1400 shaft horsepower engine and not a made up number.
HORSEPOWER = 745.6998715822702

#: Main rotor revolutions per engine revolution, 324 rotor / 6600 engine rpm
#: (TM 55-1520-210-10, 7.1-13 and 7.1-15).  The manual's other pair, 314 rotor
#: / 6400 engine rpm, gives 20.382: the two are 0.06 per cent apart, so the
#: gearing is the same drive read twice.
UH1_ENGINE_MAIN_RATIO = 6600.0 / 324.0        # -, 20.3704

#: Engine (N2) rpm.  6600 is the speed the governor holds at the 324 rpm rotor
#: reference; 6400 is the manual's other chart, at 314 rpm of rotor.
UH1_ENGINE_RPM = 6600.0                       # rpm, 100 % N2
UH1_ENGINE_CRUISE_RPM = 6400.0                # rpm, the second chart's speed

#: The governor's droop band, engine rpm.  "The compensator will hold N2 rpm to
#: +-40 rpm when properly rigged" (2-23), and droop is defined there as the
#: speed change "as power is increased from a no-load condition".
UH1_ENGINE_DROOP_RPM = 40.0                   # rpm, the band, and the gain

#: The data plate calibration: "the indicated torque pressure at 1125 ft-lbs
#: actual output shaft torque" (7.1-14).  This is the one absolute torque the
#: manual gives, and it is at the engine's output shaft.
UH1_ENGINE_TORQUE_FT_LB = 1125.0              # ft lb, at the output shaft
UH1_ENGINE_SHAFT_TORQUE_NM = UH1_ENGINE_TORQUE_FT_LB * FT_LB_TO_NM
UH1_ENGINE_OMEGA = UH1_ENGINE_RPM * 2.0 * math.pi / 60.0      # rad/s, 691.15

#: What that calibration torque is worth at 100 % N2: 1525 N m of shaft torque
#: and 1.054 MW of power, which is 1414 shp - the T53-L-13's own 1400 shp
#: rating to within a per cent, which is what makes this a real engine rather
#: than a torque limit picked to suit the aircraft.
UH1_ENGINE_POWER_W = UH1_ENGINE_SHAFT_TORQUE_NM * UH1_ENGINE_OMEGA  # 1.054 MW

#: The same torque at the *main rotor* shaft, which is the shaft
#: :mod:`airframe` balances: 1125 ft-lb of engine shaft torque is 31.1 kN m of
#: rotor shaft torque, 20.37 times as much for 20.37 times less speed.
UH1_ENGINE_MAIN_TORQUE_NM = UH1_ENGINE_SHAFT_TORQUE_NM * UH1_ENGINE_MAIN_RATIO

#: The transmission's structural limit: "the power output capability of the
#: T53-L-13 engine can exceed the transmission structural limit (50 psi
#: calibrated torque) under certain conditions" (7.1-13).  With no chart to read
#: a slope off, this project takes it as the gauge's full scale, i.e. as the
#: torque available at sea level standard - see the module docstring.
UH1_TRANSMISSION_LIMIT_PSI = 50.0             # psi, calibrated torque
UH1_TORQUE_NM_PER_PSI = UH1_ENGINE_MAIN_TORQUE_NM / UH1_TRANSMISSION_LIMIT_PSI

#: What the GOV AUTO/EMER switch in EMER costs: "the maximum engine torque
#: available for any ambient condition will be reduced by 6 to 8 PSI when the
#: GOV AUTO/EMER switch is placed in the EMER position" (9-3).  The middle of
#: the 6 to 8 is what is taken here.
UH1_GOVERNOR_EMER_PSI = 7.0                   # psi, 6 to 8 in the manual

#: Bleed air, and its two consumers (7.1-15): 1.4 psi for the heater, 2.1 for
#: the device, 3.5 for both - which is the two together to the precision the
#: manual prints them at.
UH1_BLEED_HEATER_PSI = 1.4                    # psi, bleed air heater on
UH1_BLEED_DEVICE_PSI = 2.1                    # psi, a bleed air device on
UH1_BLEED_BOTH_PSI = 3.5                      # psi, both at once

#: The gas producer's time constant, sec: this project's own, like the rotor
#: inertia over in :mod:`airframe` and for the same reason - a first order lag
#: on a number neither report carries.  It is the order of a T53's spool time,
#: and it is what makes an engine failure a *wind down* rather than a switch,
#: and a governor a lagged answer rather than an instant one.
UH1_ENGINE_TIME_CONSTANT_S = 0.6              # sec, no source: see docstring


def _clip(value, low, high):
    """Clamp *value* into ``[low, high]``."""
    return max(low, min(high, float(value)))


@dataclass
class Governor:
    """The N2 governor, and the pilot's hand on the twist grip that feeds it.

    Four things decide how much power the engine is asked for, and they are the
    four the aircraft has:

    * what speed the speed selector is set to, ``target_rpm`` (the manual's
      rotor speeds are 324 and 314 rpm, and the pilot's GOV CONT INCR/DECR
      switches trim it from there, 2-22);
    * how far the twist grip is rolled on, ``throttle``, 1 being full open.  At
      full open the governor holds the speed it selected; rolled toward closed
      it selects a lower speed instead, which is the manual's own description
      of the grip (2-22), so the same field does both;
    * whether the GOV switch is in AUTO or EMER, ``emer``: in AUTO the governor
      is in the loop, in EMER it is out of it and the grip is the fuel control;
    * what the engine has failed to, ``failed``, and what is being bled off it,
      ``bleed_psi``.

    The governor is a proportional controller and nothing more, because
    proportional is what the manual describes: the droop compensator's whole
    job is to make a *speed error* worth power (2-23), and its +-40 rpm band is
    the size of the error at full power.  So :meth:`gain` is the power
    available divided by the band, :meth:`power_target` is that gain times the
    speed error, and :meth:`droop_engine_rpm` is the error the loop settles at
    for a given power demand - which is the manual's own definition of droop,
    read as a curve rather than as one number.
    """

    #: The speed the governor is asked to hold, in *rotor* rpm, since that is
    #: the gauge the pilot reads it off: 324 rpm of rotor is 6600 of engine.
    target_rpm: float = UH1_RPM
    #: Rotor revolutions per engine revolution.
    ratio: float = UH1_ENGINE_MAIN_RATIO
    #: The droop band, engine rpm: the speed error that buys full power.
    droop_rpm: float = UH1_ENGINE_DROOP_RPM
    #: The gas producer's lag, sec.
    time_constant: float = UH1_ENGINE_TIME_CONSTANT_S
    #: The twist grip: 1 is full open (governor on the selected speed), 0 is
    #: closed (no fuel at all), and anything between selects a lower speed.
    throttle: float = 1.0
    #: The GOV AUTO/EMER switch: True is EMER, the governor out of the loop.
    emer: bool = False
    #: The engine is out: no fuel, and it stays out until a reset says so.  A
    #: failed engine and a closed throttle are the same physics and are kept
    #: apart because they are different events to show a pilot.
    failed: bool = False
    #: Bleed air: 0 in normal flight, or the manual's 1.4 psi with the heater
    #: on, 2.1 with a device on and 3.5 with both (7.1-15).
    bleed_psi: float = 0.0

    def n2_rpm(self, rotor_rpm):
        """Engine (N2) rpm at a main rotor speed, rpm."""
        return float(rotor_rpm) * self.ratio

    def target_n2_rpm(self):
        """The engine rpm the governor is asked to hold."""
        return self.target_rpm * self.ratio

    def psi_lost(self):
        """Torque available lost, psi: bleed air plus EMER if it is selected.

        The two are added, which is the arithmetic the manual's own numbers
        invite - 7 psi of EMER and 1.4 of heater are both quoted as reductions
        in the same calibrated torque - and no page adds them for us, so a
        reader is told here that this is the sum of the parts.
        """
        return (max(float(self.bleed_psi), 0.0)
                + (UH1_GOVERNOR_EMER_PSI if self.emer else 0.0))

    def power_available(self, air_density=RHO_SEA_LEVEL):
        """The most power there is at the shaft, W, in this air.

        Sea level standard is :data:`UH1_ENGINE_POWER_W`, which is 50 psi of
        calibrated torque worth of it, and it scales with the air density the
        airframe is flying in (:meth:`airframe.Airframe.air_density`), which is
        the shape of the manual's torque available charts in one number.  The
        EMER and bleed penalties come off afterwards, in psi, so that they
        reduce whatever the density left.
        """
        fraction = (float(air_density) / RHO_SEA_LEVEL
                    * (1.0 - self.psi_lost() / UH1_TRANSMISSION_LIMIT_PSI))
        return UH1_ENGINE_POWER_W * max(fraction, 0.0)

    def gain(self, air_density=RHO_SEA_LEVEL):
        """The governor's gain, W of shaft power per engine rpm of N2 error.

        Full power at the bottom of the droop band and none at the top of it,
        which is what "will hold N2 rpm to +-40 rpm" (2-23) means read as a
        loop: the band *is* the gain, expressed in the unit the pilot's gauge
        is marked in.
        """
        return self.power_available(air_density) / self.droop_rpm


    def selected_n2_rpm(self):
        """The engine rpm the twist grip is asking for, at the speed selector.

        Full open is the governor's own setting - "rotating the throttle toward
        closed will cause the rpm to be manually selected instead of
        automatically" (2-20) - so the grip position is read as a fraction of
        the selected speed rather than as a fuel flow of its own.  In EMER the
        grip is a fuel flow and this is not what the loop uses; see
        :meth:`power_target`.
        """
        return self.target_n2_rpm() * _clip(self.throttle, 0.0, 1.0)

    def power_target(self, rotor_rpm, air_density=RHO_SEA_LEVEL):
        """The power the engine is being asked for, W, at a rotor speed.

        Four regimes, in the order they are checked:

        * a failed engine, or a grip rolled right off, is asked for nothing -
          the fuel is gone and the power that is left has to decay away;
        * EMER, the GOV switch away from AUTO, is asked for what the grip
          says: the fuel control is the pilot's hand and there is no speed
          feedback at all, which is exactly the "manual control of the engine
          RPM" of 9-3, and the reason that page warns about compressor stall
          and overspeed;
        * AUTO is the governor: the power that makes up the error between the
          selected speed and the speed the rotor is actually doing, the gain
          being :meth:`gain`.  At or above the selected speed the error is
          negative and the answer is no power - an idling free turbine is not a
          brake, and the rotor winds down on its own drag from there;
        * and never more than :meth:`power_available`, which is what makes the
          droop band a band rather than a cliff.
        """
        if self.failed or self.throttle <= 0.0:
            return 0.0
        ceiling = self.power_available(air_density)
        if self.emer:
            return ceiling * _clip(self.throttle, 0.0, 1.0)
        error = self.selected_n2_rpm() - self.n2_rpm(rotor_rpm)
        return _clip(self.gain(air_density) * error, 0.0, ceiling)

    def droop_engine_rpm(self, power, air_density=RHO_SEA_LEVEL):
        """The N2 speed error a power demand settles at, engine rpm.

        ``P / gain``, clipped to the band, which is the manual's droop curve
        read the way 2-23 defines it: zero at no load, the whole 40 rpm at
        full power, and proportional in between because the governor is
        proportional.  Note that it is the *error*, so a settled speed is
        :meth:`settled_n2_rpm`, i.e. this much below the selected speed.
        """
        ceiling = self.power_available(air_density)
        if ceiling <= 0.0:
            return 0.0
        return _clip(power, 0.0, ceiling) / ceiling * self.droop_rpm

    def settled_n2_rpm(self, power, air_density=RHO_SEA_LEVEL):
        """Where the loop settles for a power demand: the selected speed less droop."""
        return self.selected_n2_rpm() - self.droop_engine_rpm(power, air_density)

    def settled_rotor_rpm(self, power, air_density=RHO_SEA_LEVEL):
        """The same thing in the unit the pilot's rotor gauge is marked in."""
        return self.settled_n2_rpm(power, air_density) / self.ratio

    def __str__(self):
        return ("governor %s %+4.0f rpm, throttle %3.0f %%"
                % ("EMER" if self.emer else "AUTO", self.target_rpm - UH1_RPM,
                   100.0 * _clip(self.throttle, 0.0, 1.0)))




@dataclass
class Engine:
    """A T53 on a shaft: the governor's demand, lagged, as a torque.

    This is the whole of what :mod:`airframe` needs from an engine, and it is
    three numbers and a lag:

    * :meth:`power_target` - what the governor (or the pilot's hand, in EMER)
      is asking the engine for, from the rotor speed it can see;
    * :attr:`power` - what the engine is actually delivering, because a turbine
      takes :data:`UH1_ENGINE_TIME_CONSTANT_S` to get anywhere: the gas
      producer winds up first, and only then does the power turbine get the gas
      to turn;
    * :meth:`torque` - that power as a torque at the *main rotor* shaft, which
      is where the airframe's balance wants it.  A free turbine's output torque
      is ``P / omega`` and this one is no exception, so a rotor that is slow
      gets more torque out of the same gas - which is how the engine helps
      arrest a wind down, and why a wind up has to be caught by the governor.

    The state is :attr:`power` alone.  It is deliberately neither a member of
    :class:`airframe.FlightState` nor a hidden integration: the lag is advanced
    once per physics step from the rotor speed at the start of it - the frame
    loop's own arrangement, and the one the report's simulation used for the
    rotor's flapping - and :meth:`settle` is the matching way to start a run in
    equilibrium rather than from a stopped engine.
    """

    #: The governor, and the pilot's hand on it.
    governor: Governor = field(default_factory=Governor)
    #: Shaft power being delivered now, W, at the engine's output shaft.  The
    #: airframe never touches this; it asks for :meth:`torque`.
    power: float = 0.0

    def n2_rpm(self, rotor_rpm):
        """Engine (N2) rpm at a main rotor speed, rpm."""
        return self.governor.n2_rpm(rotor_rpm)

    def rotor_rpm_of(self, n2_rpm_value):
        """The main rotor speed a given N2 speed means, rpm."""
        return float(n2_rpm_value) / self.governor.ratio

    def omega_of(self, rotor_rpm):
        """The rotor's angular speed, rad/s, from its rpm - zero if it is stopped."""
        return abs(float(rotor_rpm)) * 2.0 * math.pi / 60.0

    def torque(self, rotor_rpm):
        """Torque at the main rotor shaft, N m, from the power now delivered.

        ``P / omega`` at the rotor: the free turbine's own relation, and the
        one place the engine and the rotor are coupled in the direction that
        matters for a wind down - a rotor at 90 per cent of its speed takes 11
        per cent more torque for the same gas.
        """
        omega = self.omega_of(rotor_rpm)
        if omega <= 0.0:
            return 0.0
        return self.power / omega

    def max_torque(self, rotor_rpm, air_density=RHO_SEA_LEVEL):
        """The most torque there is at that rotor speed, N m, in that air."""
        omega = self.omega_of(rotor_rpm)
        if omega <= 0.0:
            return 0.0
        return self.governor.power_available(air_density) / omega

    def torque_percent(self, rotor_rpm):
        """Where the torque gauge would be pointing, % of the 1125 ft-lb point.

        The data plate calibration is what the gauge is calibrated against
        (7.1-14) and the transmission limit is what it is limited to (7.1-13);
        with this project's 50 psi full scale the two are the same reading, so
        100 % here is 50 psi of calibrated torque.
        """
        return 100.0 * self.torque(rotor_rpm) / UH1_ENGINE_MAIN_TORQUE_NM

    def power_target(self, rotor_rpm, air_density=RHO_SEA_LEVEL):
        """What the governor is asking the engine for at this rotor speed, W."""
        return self.governor.power_target(rotor_rpm, air_density)

    def settled_torque(self, rotor_rpm, air_density=RHO_SEA_LEVEL):
        """The torque the shaft settles at on that rotor speed, N m.

        ``P / omega`` of what the governor is asking for at the speed the rotor
        is doing: the engine's own half of the shaft balance, and a *function* of
        the rotor speed rather than the last frame's lagged power.  That is what
        lets a trim solve *for* the rotor speed - the gas producer's lag has no
        meaning in a steady condition, so the balance a trim closes is the one
        the loop is walking towards and not the transient it is inside.

        It is exactly the loop's own equilibrium: :meth:`Governor.droop_engine_rpm`
        says where a power demand settles and this is that point read as a torque
        at the rotor, so the rotor speed a trim solves for with it is the one six
        seconds of frames would have walked to.  The self test asserts the two
        agree, so the closed form and the integrated loop vouch for each other.
        A failed engine, or a grip rolled right off, settles on nothing at all.
        """
        omega = self.omega_of(rotor_rpm)
        if omega <= 0.0:
            return 0.0
        return self.power_target(rotor_rpm, air_density) / omega

    def advance(self, dt, rotor_rpm, air_density=RHO_SEA_LEVEL):
        """One frame of the gas producer's lag, and the torque it leaves.

        The power moves a first order step of *dt* seconds toward what the
        governor is asking for, and the torque that comes out is the new power
        over the rotor's own speed - so a frame is ``engine.advance(dt,
        state.rotor_rpm)`` and the answer goes straight into the shaft balance.
        A *dt* of zero advances nothing, which is what a caller re-evaluating
        forces inside a frame wants.
        """
        target = self.power_target(rotor_rpm, air_density)
        if dt > 0.0:
            weight = 1.0 - math.exp(-float(dt) / self.governor.time_constant)
            self.power += (target - self.power) * weight
        return self.torque(rotor_rpm)

    def settle(self, rotor_rpm, air_density=RHO_SEA_LEVEL):
        """Start the engine on the power its governor is asking it for, W.

        What a reset calls when it has no stick positions to work from, so
        that a run begins with an engine at the power its own governor wants
        for that rotor speed rather than at a stopped engine: from zero it
        takes about three seconds of frames to catch up with a hovering rotor,
        which would put a sag into every script that started that way and is
        not a transient anybody asked for.
        """
        self.power = self.power_target(rotor_rpm, air_density)
        return self.power

    def settle_on(self, torque, rotor_rpm):
        """Start the engine on the power that delivers *torque*, N m, W.

        ``P = Q omega``, the inverse of :meth:`torque`: what
        :meth:`airframe.Airframe.reset` calls with the torque the rotor is
        *absorbing* at the state a run begins on, which is the shaft balance it
        can evaluate.  The balance then starts at zero and the governor walks
        the aircraft to its own droop point from there, rather than the frames
        being spent spooling an engine up.
        """
        self.power = max(float(torque), 0.0) * self.omega_of(rotor_rpm)
        return self.torque(rotor_rpm)

    def reset(self):
        """Stop the engine: no power in it and nothing left in the lag."""
        self.power = 0.0
        return self

    def __str__(self):
        return ("engine %6.1f kW, %.1f %% of maximum power, %s"
                % (self.power / 1000.0,
                   100.0 * self.power / UH1_ENGINE_POWER_W, self.governor))


# ---------------------------------------------------------------------------
# The self test and the demo, in the style of the modules around this one.
# ---------------------------------------------------------------------------


def _self_test():
    """Checks on the engine's constants, its governor and its two lags.

    Raises AssertionError on failure.  Everything the manual prints is checked
    against the page it is printed on - the 324/6600 pair twice over, the 1125
    ft-lb calibration as both a torque and a power, the 50 psi transmission
    limit, the +-40 rpm droop band and its definition, and the psi the bleed
    air and EMER penalties are worth - and everything this project chose is
    checked against the two things a choice has to do here: be a *shape* that
    answers to the physics, and sit inside the envelope the airframe flies.
    The engine on a real rotor, and what the airframe does with it, is
    :mod:`airframe`'s own self test; this one only has to be right about the
    engine.
    """
    # The gearing, which the manual prints twice and both times the same to a
    # fifth of a per cent: that agreement is the check.
    assert abs(UH1_ENGINE_MAIN_RATIO - 20.3704) < 5e-5, UH1_ENGINE_MAIN_RATIO
    other = UH1_ENGINE_CRUISE_RPM / 314.0
    assert abs(other - UH1_ENGINE_MAIN_RATIO) / UH1_ENGINE_MAIN_RATIO < 0.001
    assert abs(UH1_ENGINE_OMEGA - 691.15) < 0.01, UH1_ENGINE_OMEGA
    assert abs(Governor().n2_rpm(UH1_RPM) - UH1_ENGINE_RPM) < 1e-9
    assert abs(Governor().n2_rpm(UH1_RPM_100) - UH1_ENGINE_RPM) < 1e-9
    assert abs(Governor().target_n2_rpm() - UH1_ENGINE_RPM) < 1e-9

    # The data plate calibration, read as both of the things it is: 1525 N m of
    # output shaft torque, and 1414 shp of power at 6600 rpm.  The second is
    # the check that matters - the T53-L-13 is a 1400 shp engine (2-1), and
    # this lands within a per cent of its rating, which is as close as a
    # calibration torque quoted to four figures can.
    assert abs(UH1_ENGINE_SHAFT_TORQUE_NM - 1525.30) < 0.01
    assert abs(UH1_ENGINE_POWER_W / HORSEPOWER - 1414.0) < 1.0
    assert abs(UH1_ENGINE_POWER_W / HORSEPOWER - 1400.0) / 1400.0 < 0.02
    # At the main rotor shaft, 20.37 times the torque for 20.37 times less
    # speed: 31.1 kN m, and 50 psi of calibrated torque at sea level.
    assert abs(UH1_ENGINE_MAIN_TORQUE_NM - 31070.8) < 0.5
    assert abs(UH1_TORQUE_NM_PER_PSI - 621.4) < 0.05
    assert abs(UH1_TORQUE_NM_PER_PSI * UH1_TRANSMISSION_LIMIT_PSI
               - UH1_ENGINE_MAIN_TORQUE_NM) < 1e-9

    # Droop, which is the manual's own definition (2-23) read as a curve: no
    # load is no speed error, full power is the whole +-40 rpm band, and the
    # governor is proportional so half power is half the band.
    governor = Governor()
    ceiling = governor.power_available()
    assert abs(ceiling - UH1_ENGINE_POWER_W) < 1e-6
    assert governor.droop_engine_rpm(0.0) == 0.0
    assert abs(governor.droop_engine_rpm(ceiling) - UH1_ENGINE_DROOP_RPM) < 1e-9
    assert abs(governor.droop_engine_rpm(0.5 * ceiling) - 20.0) < 1e-9
    assert governor.droop_engine_rpm(2.0 * ceiling) == UH1_ENGINE_DROOP_RPM
    assert abs(governor.settled_n2_rpm(0.5 * ceiling)
               - (UH1_ENGINE_RPM - 20.0)) < 1e-9
    # The gain that makes that so: full power at the bottom of the band.
    assert abs(governor.gain() * UH1_ENGINE_DROOP_RPM - ceiling) < 1e-9

    # What that curve is worth on this aircraft.  16268 N m is the trimmed
    # hover of the 8700 lb airframe at the 324 rpm reference - the number
    # airframe's self test measures off the report's own equation 4 - and it
    # asks the engine for 552 kW, 52 per cent of what there is.  The governor
    # settles 21 rpm of N2 down for it, a shade over 1 rpm of rotor, which is
    # inside its own band and well inside the rotor's green arc: the aircraft
    # has an engine with margin, not one on the stop.
    hover = 16268.0                                   # N m, at the rotor shaft
    rotor_omega = UH1_RPM * 2.0 * math.pi / 60.0      # rad/s, the rotor at 100 %
    assert abs(hover / UH1_ENGINE_MAIN_TORQUE_NM - 0.52) < 0.01
    assert abs(hover / UH1_TORQUE_NM_PER_PSI - 26.2) < 0.1
    hover_power = hover * rotor_omega
    assert abs(hover_power / 1000.0 - 552.0) < 1.0
    settled = governor.settled_rotor_rpm(hover_power)
    assert 0.5 < UH1_RPM - settled < 2.0, UH1_RPM - settled
    assert UH1_RPM_LOW < settled < UH1_RPM_MAX
    assert abs(governor.n2_rpm(settled)
               - (UH1_ENGINE_RPM - governor.droop_engine_rpm(hover_power))) < 1e-9

    # The gas producer's lag: 63 per cent of the way in one time constant, 95
    # in three, and nothing at all in a frame of no time - and what it chases
    # is the governor's own target, which never exceeds what the air can give.
    # The rotor is held 5 per cent slow here, which is the case the governor is
    # for: it is under its selected speed, the error is 330 engine rpm and the
    # gain clips the answer at everything the engine has.
    engine = Engine()
    slow = 0.95 * UH1_RPM
    target = engine.power_target(slow)
    assert abs(target - ceiling) < 1e-6
    assert engine.power_target(UH1_RPM) == 0.0  # at the speed asked for: none
    engine.power = 0.0
    engine.advance(0.0, slow)
    assert engine.power == 0.0
    engine.advance(UH1_ENGINE_TIME_CONSTANT_S, slow)
    assert abs(engine.power / target - (1.0 - 1.0 / math.e)) < 1e-9
    engine.advance(2.0 * UH1_ENGINE_TIME_CONSTANT_S, slow)
    assert abs(engine.power / target - (1.0 - math.exp(-3.0))) < 1e-9
    for _ in range(120):
        engine.advance(1.0 / 60.0, slow)
    assert abs(engine.power / target - 1.0) < 0.01

    # A free turbine's output torque is P / omega, so the same gas is worth
    # more torque at a lower rotor speed: 11 per cent more at 90 per cent of
    # the speed, which is what helps catch a wind down.  The *maximum* torque
    # moves with it too, so at a slow rotor speed the transmission limit is
    # nearer than the gauge suggests.
    engine.power = UH1_ENGINE_POWER_W
    assert abs(engine.torque(UH1_RPM) - UH1_ENGINE_POWER_W / rotor_omega) < 1e-9
    assert abs(engine.torque(0.9 * UH1_RPM) / engine.torque(UH1_RPM)
               - 1.0 / 0.9) < 1e-9
    assert engine.max_torque(0.9 * UH1_RPM) > engine.max_torque(UH1_RPM)
    assert engine.torque(0.0) == 0.0 and engine.max_torque(0.0) == 0.0
    assert abs(engine.max_torque(UH1_RPM) - UH1_ENGINE_MAIN_TORQUE_NM) < 1e-6
    # The gauge: full power at the reference speed is 100 per cent of the data
    # plate torque, and the trimmed hover is 52.
    assert abs(engine.torque_percent(UH1_RPM) - 100.0) < 1e-6
    # Settling on the hover's own speed asks for that hover's own power, which
    # is the whole point of settle(): no sag on the first frames of a run.
    engine.reset().settle(settled)
    for _ in range(60):
        engine.advance(1.0 / 60.0, settled)
    assert abs(engine.torque_percent(settled)
               - hover / UH1_ENGINE_MAIN_TORQUE_NM * 100.0) < 0.5
    # The torque itself is a little *more* than the hover's 16268 N m, because
    # at the settled speed the free turbine turns 0.3 per cent slower and P /
    # omega reads that much higher: it is the same power, and the aircraft is
    # flying the droop rather than the trim.  A per cent is the agreement.
    assert abs(engine.torque(settled) - hover) < 0.01 * hover, engine.torque(settled)

    # Fuel off, and the engine out: the same physics, and the lag is what makes
    # it a wind down rather than a switch.  A grip rolled right off is *not* a
    # failure, which is the difference the two flags keep.
    stopped = Engine()
    stopped.power = UH1_ENGINE_POWER_W           # spooled up, at sea level
    stopped.governor.throttle = 0.0
    assert stopped.power_target(UH1_RPM) == 0.0
    stopped.advance(UH1_ENGINE_TIME_CONSTANT_S, UH1_RPM)
    assert abs(stopped.power - UH1_ENGINE_POWER_W / math.e) < 1e-6
    assert stopped.torque(UH1_RPM) > 0.0        # still turning: the lag, not a switch
    stopped.reset()
    assert stopped.power == 0.0
    failed = Engine(governor=Governor(failed=True))
    assert failed.power_target(UH1_RPM) == 0.0
    assert failed.power_target(0.5 * UH1_RPM) == 0.0

    # The grip between open and closed: a free turbine governor selects a
    # *speed* (2-20), so half a grip is half the selected N2, 3300 engine rpm
    # or 162 of rotor, and a rotor at 324 is above the speed it is being asked
    # for - so it gets no fuel and winds down to it.  That is what rolling the
    # throttle off in flight does, and why it is not an engine failure.
    half_open = Governor(throttle=0.5)
    assert abs(half_open.selected_n2_rpm() - 0.5 * UH1_ENGINE_RPM) < 1e-9
    assert abs(half_open.settled_rotor_rpm(0.0) - 0.5 * UH1_RPM) < 1e-9
    assert half_open.power_target(UH1_RPM) == 0.0
    assert half_open.power_target(0.4 * UH1_RPM) > 0.0

    # AUTO is a governor and EMER is not, which is the whole of what the switch
    # does: in AUTO the power depends on the speed error, in EMER on nothing but
    # the grip - "manual control of the engine RPM" (9-3) - and 7 psi of the
    # available torque is gone either way, of the manual's 6 to 8.
    emer = Governor(emer=True)
    assert abs(emer.psi_lost() - UH1_GOVERNOR_EMER_PSI) < 1e-9
    assert abs(emer.power_available() - ceiling * 43.0 / 50.0) < 1e-6
    assert emer.power_target(0.8 * UH1_RPM) == emer.power_target(UH1_RPM)
    assert abs(emer.power_target(UH1_RPM) - ceiling * 43.0 / 50.0) < 1e-6
    assert governor.power_target(0.8 * UH1_RPM) > governor.power_target(UH1_RPM)
    assert governor.power_target(UH1_RPM) == 0.0        # at the speed asked for
    assert abs(governor.power_target(0.8 * UH1_RPM) - ceiling) < 1e-6   # clipped

    # Bleed air and its two consumers (7.1-15), and the sum with EMER.
    assert abs(Governor(bleed_psi=UH1_BLEED_HEATER_PSI).psi_lost() - 1.4) < 1e-9
    assert abs(Governor(bleed_psi=UH1_BLEED_DEVICE_PSI).psi_lost() - 2.1) < 1e-9
    assert abs(Governor(bleed_psi=UH1_BLEED_BOTH_PSI).psi_lost() - 3.5) < 1e-9
    assert abs(Governor(bleed_psi=3.5, emer=True).psi_lost() - 10.5) < 1e-9

    # Density, which is how altitude and temperature reach the engine: 80 per
    # cent of the air is 80 per cent of the torque available, and the penalties
    # come off what is left rather than off the sea level figure.
    thin = Governor()
    assert abs(thin.power_available(0.8 * RHO_SEA_LEVEL)
               - 0.8 * UH1_ENGINE_POWER_W) < 1e-6
    cold = Governor(emer=True)
    assert abs(cold.power_available(1.05 * RHO_SEA_LEVEL)
               - 1.05 * ceiling * 43.0 / 50.0) < 1e-6
    # At sea level standard the available torque *is* the transmission limit,
    # which is what makes 50 psi the gauge's full scale; in denser air it goes
    # past it, which is the manual's own warning that it can.
    assert abs(Engine().max_torque(UH1_RPM) - UH1_ENGINE_MAIN_TORQUE_NM) < 1e-6
    assert Engine().max_torque(UH1_RPM, 1.1 * RHO_SEA_LEVEL) \
        > UH1_ENGINE_MAIN_TORQUE_NM

    # The settled torque, which is what a rotor speed trim closes the shaft on:
    # nothing at or above the speed the governor is asking for, since an idling
    # free turbine is not a brake; more the further the rotor is below it and the
    # more power that asks for; and the ceiling over omega once the whole droop
    # band is used up, so it is :meth:`max_torque` there.
    settled = Engine()
    assert settled.settled_torque(UH1_RPM) == 0.0
    assert settled.settled_torque(UH1_RPM + 1.0) == 0.0
    assert settled.settled_torque(0.0) == 0.0
    slow = settled.settled_torque(0.99 * UH1_RPM)
    assert slow > 0.0
    assert settled.settled_torque(0.95 * UH1_RPM) > slow
    assert abs(settled.settled_torque(0.9 * UH1_RPM)
               - settled.max_torque(0.9 * UH1_RPM)) < 1e-6      # clipped
    assert abs(settled.settled_torque(0.99 * UH1_RPM)
               - settled.power_target(0.99 * UH1_RPM)
               / settled.omega_of(0.99 * UH1_RPM)) < 1e-12
    # Asking is free: it reads the rotor speed and the air and not the engine's
    # own state, which is the whole point of it being usable inside a trim.
    assert settled.power == 0.0
    # And it is where the loop arrives: an engine settled there delivers it.
    assert settled.settle(0.99 * UH1_RPM) == settled.power
    assert abs(settled.torque(0.99 * UH1_RPM) - slow) < 1e-9
    # A failed engine, and one whose grip is rolled off, hold nothing up.
    assert Engine(governor=Governor(failed=True)).settled_torque(
        0.9 * UH1_RPM) == 0.0
    assert Engine(governor=Governor(throttle=0.0)).settled_torque(
        0.9 * UH1_RPM) == 0.0



def _demo():
    """Print what the engine does: its constants, its droop, and its four modes.

    The flying is :mod:`airframe`'s job; what is on show here is what the
    engine offers the shaft - how much torque there is and in what air, where
    the governor settles for each power demand, and what the twist grip, the
    GOV switch and a fuel shutoff each do to that answer.
    """
    print("T53-L-13, as TM 55-1520-210-10 gives it")
    print("  324 rotor / 6600 engine rpm, ratio %.4f, and the manual's other"
          " pair 314 / 6400 is %.4f" % (UH1_ENGINE_MAIN_RATIO,
                                        UH1_ENGINE_CRUISE_RPM / 314.0))
    print("  data plate 1125 ft lb at the output shaft = %.1f N m shaft,"
          " %.1f kN m rotor" % (UH1_ENGINE_SHAFT_TORQUE_NM,
                                UH1_ENGINE_MAIN_TORQUE_NM / 1000.0))
    print("  which at %.0f rad/s is %.1f kW, %.1f shp (the engine is rated"
          " 1400 shp)" % (UH1_ENGINE_OMEGA, UH1_ENGINE_POWER_W / 1000.0,
                          UH1_ENGINE_POWER_W / HORSEPOWER))
    print("  transmission limit %.0f psi calibrated torque = %.1f N m per psi,"
          " so this project reads the gauge at %.1f N m"
          % (UH1_TRANSMISSION_LIMIT_PSI, UH1_TORQUE_NM_PER_PSI,
             UH1_ENGINE_MAIN_TORQUE_NM))
    print("  droop compensator holds N2 to +-%.0f rpm; GOV EMER costs %.0f psi"
          " (the manual's 6 to 8); gas producer lag %.2f s, this project's own"
          % (UH1_ENGINE_DROOP_RPM, UH1_GOVERNOR_EMER_PSI,
             UH1_ENGINE_TIME_CONSTANT_S))

    print()
    print("droop, which is the manual's definition read as a curve")
    print("  power    shaft kW   N2 error   settled N2   settled rotor   gauge")
    governor = Governor()
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        power = fraction * UH1_ENGINE_POWER_W
        error = governor.droop_engine_rpm(power)
        print("  %4.0f %%   %8.1f   %6.1f     %7.0f      %7.2f rpm    %5.1f %%"
              % (100.0 * fraction, power / 1000.0, error,
                 governor.settled_n2_rpm(power),
                 governor.settled_rotor_rpm(power),
                 fraction * 100.0))

    print()
    print("what the engine is asked for, at 324 rpm of rotor unless stated")
    engine = Engine()
    for name, governor in (("AUTO, at the selected speed", Governor()),
                           ("AUTO, rotor 5 % slow", Governor()),
                           ("AUTO, rotor 10 % fast", Governor()),
                           ("EMER, full grip", Governor(emer=True)),
                           ("EMER, half a grip", Governor(emer=True, throttle=0.5)),
                           ("AUTO, half a grip", Governor(throttle=0.5)),
                           ("throttle closed", Governor(throttle=0.0)),
                           ("engine failed", Governor(failed=True)),
                           ("heater and device on", Governor(bleed_psi=3.5))):
        engine.governor = governor
        rotor = UH1_RPM
        if "slow" in name:
            rotor = 0.95 * UH1_RPM
        elif "fast" in name:
            rotor = 1.10 * UH1_RPM
        power = engine.power_target(rotor)
        torque = power / engine.omega_of(rotor)
        print("  %-26s rotor %5.1f rpm, N2 %5.0f, asked for %6.1f kW (%4.1f %%),"
              " %5.1f %% of maximum torque"
              % (name, rotor, governor.n2_rpm(rotor), power / 1000.0,
                 100.0 * power / UH1_ENGINE_POWER_W,
                 100.0 * torque / UH1_ENGINE_MAIN_TORQUE_NM))

    print()
    print("the 8700 lb aircraft's trimmed hover, on this engine")
    hover = 16268.0
    rotor_omega = UH1_RPM * 2.0 * math.pi / 60.0
    hover_power = hover * rotor_omega
    settled = Governor().settled_rotor_rpm(hover_power)
    print("  rotor torque %.0f N m (%.1f kN m), %.1f %% of the %.1f kN m the"
          " transmission will take" % (hover, hover / 1000.0,
                                       100.0 * hover / UH1_ENGINE_MAIN_TORQUE_NM,
                                       UH1_ENGINE_MAIN_TORQUE_NM / 1000.0))
    print("  %.1f kW (%.1f shp) of shaft power, %.0f %% of what there is, and"
          " %.1f psi on the gauge" % (hover_power / 1000.0,
                                      hover_power / HORSEPOWER,
                                      100.0 * hover_power / UH1_ENGINE_POWER_W,
                                      hover / UH1_TORQUE_NM_PER_PSI))
    print("  the governor settles at %.2f rpm of rotor, %.2f %% below the"
          " reference and above the %.0f rpm green arc"
          % (settled, 100.0 * (UH1_RPM - settled) / UH1_RPM, UH1_RPM_LOW))

    print()
    _self_test()
    print("self test passed")


if __name__ == "__main__":
    _demo()

