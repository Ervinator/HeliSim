# -*- coding: utf-8 -*-
"""Scratch: the tail rotor's two unwritten constants, measured rather than argued.

``aerodynamics.UH1_TAIL_TWIST_DEG`` and ``aerodynamics.UH1_TAIL_PRECONE_DEG`` are
the one pair of numbers in the model that neither TM-73254 nor
TM 55-1520-210-10 carries.  What stands behind them is the comment beside them
and three assertions in ``aerodynamics.py``'s self test, and that comment quotes
figures - 2180 N, 7.85 deg, 0.664, -52 deg, "about 4 per cent", 0.88 and 0.35
deg, "130 per cent", 34 and 45 mm, and "130 %" of thrust at a pedal stop - that
nothing in the project ever computed.  This is the ruler those figures are cut
with: the tail rotor's own instrument, the way ``_rt_probe.py`` is the pilot's.
It prints the survey behind each number, in the one place the two can be read
against each other, and it asserts what it prints, so the comment can cite a
measurement instead of a memory and a drift in it is a traceback.

**Two rulers, one question.**  The tail rotor that *flies* is equation 37
(``airframe.tail_rotor_thrust``) - the report's own "economical approximation to
the tail rotor thrust of an isolated tail rotor", parametric in the tail
collective.  The tail rotor the *constants* describe is the blade element one
(``Rotor.uh1h_tail``), which is what the self test runs.  Ask the two of them for
the same newtons and they answer with different degrees of collective, so the
first table asks both for the thrust a hover's torque balance sets, and prints
the answer in **inches of pedal**, which is the unit the pilot, the trim and the
mixing law are all in.

**What comes back, into** ``_rt_tail.txt``**:**

* the hover the tail rotor is sized on - the main rotor blade element hover at
  maximum gross weight - and the thrust its torque balance asks of the tail;
* the two rulers on that one thrust: the collective each needs, the pedal that
  is, and the gap between them, in degrees and in inches;
* the twist sweep, 6 deg of washin down to 12 deg of washout: collective needed,
  pedal from both rulers, the gap, figure of merit, the worst angle of attack,
  the share of the thrust the inboard (negative angle) part of the blade makes,
  the authority in newtons per degree, and the thrust at both pedal stops;
* the precone sweep, 0 to 3 deg: what the flight's own flapping wants at two
  blade masses and at 130 per cent of the hover thrust, the tip rise, and the
  measured fact that the precone moves no load at all in this model;
* the same two sweeps at 10, 20 and 40 stations, since a blade element table is
  only worth its sampling;
* and the claim the model's own prose rests on - that the two constants are
  reached by nothing that flies - by rebinding them and re-flying the trims.

Nothing in the model imports this file, and it imports nothing of the display:
``aerodynamics``, ``rotor_control`` and ``airframe`` and the standard library,
with no pygame anywhere in it, which is why it can be run on a box that has none.
"""

import math
import sys

import aerodynamics
from aerodynamics import (RHO_SEA_LEVEL, Rotor, UH1_MAX_GROSS_MASS,
                          UH1_PEDAL_TRAVEL_IN, UH1_TAIL_ARM,
                          UH1_TAIL_PRECONE_DEG, UH1_TAIL_TWIST_DEG, UH1_WEIGHT_N,
                          collective_for_thrust, hover_loads)
from airframe import Airframe, tail_rotor_thrust
from rotor_control import (MixingLaw, RotorControlModel,
                           UH1_TAIL_COLLECTIVE_LEFT_DEG,
                           UH1_TAIL_COLLECTIVE_RIGHT_DEG, UH1_TAIL_NEUTRAL_DEG)

#: The transcript, written by the run itself: the shell's own redirection is not
#: worth trusting on this box.
OUT_PATH = "_rt_tail.txt"

#: Stations per blade for the sweep's own numbers, and the three spacings the
#: sweep is repeated at to show that a table is not an artefact of the sampling.
STATIONS = 20
CONVERGENCE_STATIONS = (10, 20, 40)

#: The twists the first table is cut at, degrees: the two the constants' comment
#: quotes, a step either side of them, and out to the ends of the band a tail
#: blade plausibly carries.  Washout is negative, as the main rotor's 12.75 deg
#: of it is.
TWIST_SWEEP = (6.0, 4.0, 2.0, 0.0, -2.0, -4.0, -6.0, -8.0, -10.0, -12.0)

#: The precone angles the second table is cut at, degrees: the band the manual
#: and the constants' comment both name, and a step either side of it.  The
#: model ships the low end of the band.
PRECONE_SWEEP = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0)

#: The blade masses the built in coning is asked about, kg: the light end of a
#: metal tail blade and a heavy one, the pair the model's own self test uses.
BLADE_MASSES = (4.0, 10.0)

#: The overload the coning is also asked at, as a multiple of the thrust the
#: torque balance sets: what a pedal to the stop asks of the same blade.
OVERLOAD = 1.3

#: The half step the authority in newtons per degree is measured over, deg.
AUTHORITY_STEP_DEG = 0.5

#: The washout the two rulers are quoted as agreeing across, degrees: from here
#: to zero twist the blade element and equation 37 stay within half an inch of
#: pedal of each other, which is the band the constant is chosen inside.
AGREEMENT_BAND_DEG = -6.0

#: How long the boundary check flies the airframe before it compares: sixty
#: frames of the frame loop's own step.  A second is longer than any lag in the
#: model, so a hover whose constants had reached the aircraft would have moved in
#: it, which is what makes "identical" a measurement rather than a coincidence.
FLIGHT_FRAMES = 60
STEP_S = 1.0 / 60.0



def tail_rotor(twist_deg=UH1_TAIL_TWIST_DEG, precone_deg=UH1_TAIL_PRECONE_DEG,
               sample_count=STATIONS):
    """The tail rotor the survey flies, with the two constants under test."""
    return Rotor.uh1h_tail(twist_deg=twist_deg, precone_deg=precone_deg,
                           sample_count=sample_count)


def hover(rotor, weight):
    """A blade element hover carrying *weight*: collective, inflow and loads."""
    collective = collective_for_thrust(rotor, weight)
    inflow, loads = hover_loads(rotor, collective)
    return collective, inflow, loads


def balance_thrust(loads):
    """The tail rotor thrust a main rotor torque balance asks for, N."""
    return loads.shaft_torque / UH1_TAIL_ARM


def pedal_for_collective(law, collective_deg):
    """The pedal position a tail rotor collective is worth, inches of travel.

    Left pedal is negative, which is the sense a hover needs: the tail rotor
    holds the nose to the left, so a hovering UH-1H carries a little left pedal.
    """
    return ((collective_deg - law.tail_neutral_deg)
            / math.degrees(law.tail_per_pedal_in))


def collective_for_pedal(law, pedal_inches):
    """The inverse: the tail collective a pedal position is worth, degrees."""
    return (law.tail_neutral_deg
            + math.degrees(law.tail_per_pedal_in) * pedal_inches)


def collective_by_equation_37(thrust):
    """Bisect equation 37 for the tail collective that makes *thrust*, degrees.

    The tail rotor that *flies* is this function and not a blade element, so this
    is the answer a flight's pedals are cut with.  At a standstill the report's
    thrust rises as ``[-T2 + sqrt(T2 ** 2 + T3 |theta|)] ** 2``, which is
    monotone in the collective, so the pedals' own range brackets it.
    """
    low, high = 0.0, math.radians(UH1_TAIL_COLLECTIVE_LEFT_DEG)
    for _ in range(60):
        middle = 0.5 * (low + high)
        if tail_rotor_thrust(middle, 0.0, 0.0) < thrust:
            low = middle
        else:
            high = middle
    return math.degrees(0.5 * (low + high))


def coning_deg(rotor, loads, mass):
    """The coning the moment about the flap hinge wants, degrees.

    The model's own self test arithmetic, in one place so that the survey and the
    test it is the ruler for are measuring the same thing: the flap moment summed
    over one blade against the centrifugal field of a blade of *mass*.
    """
    blade = [sample for sample in loads.samples if sample.blade_index == 0]
    flap_moment = sum((sample.radius - rotor.hinge_offset) * sample.thrust_per_span
                      * sample.span for sample in blade)
    root_inertia = sum((sample.radius - rotor.hinge_offset) * sample.radius
                       * sample.span / rotor.radius for sample in blade)
    return math.degrees(flap_moment
                        / (rotor.omega ** 2 * root_inertia * mass))


def tip_rise_mm(rotor, precone_deg):
    """How far a precone lifts a blade tip, mm: R * sin(precone)."""
    return 1000.0 * rotor.radius * math.sin(math.radians(precone_deg))


def blade_of(loads):
    """One blade's samples out of a set of loads, in span order."""
    return [sample for sample in loads.samples if sample.blade_index == 0]


def inboard_share(rotor, loads):
    """What share of the thrust the blade's negative angle stations make.

    The untwisted blade runs its innermost stations at a negative angle of attack
    in a hover, so that part of the blade pulls the other way.  Returns
    ``(share, worst_deg, crossing_radius)``: the signed fraction of the total
    thrust those stations make, the most negative angle of attack on the blade,
    and the radius the angle of attack changes sign at, in metres - or the tip
    radius when it never does.
    """
    blade = blade_of(loads)
    negative = sum(sample.thrust_per_span * sample.span for sample in blade
                   if sample.angle_of_attack_deg < 0.0)
    worst = min(sample.angle_of_attack_deg for sample in blade)
    crossing = rotor.radius
    for sample in blade:
        if sample.angle_of_attack_deg >= 0.0:
            crossing = sample.radius
            break
    return negative / loads.thrust, worst, crossing


def the_hover():
    """The hover the tail rotor is sized on, and the thrust its balance asks.

    The main rotor's own blade element hover at maximum gross weight - the
    42257 N of TM-73254's 9500 lb - because that is the flight both the constant's
    comment and the model's own self test quote the tail rotor's numbers at.
    """
    main = Rotor.uh1h()
    collective, inflow, loads = hover(main, UH1_WEIGHT_N)
    thrust = balance_thrust(loads)
    print("the hover the tail rotor is sized on - main rotor, blade element:")
    print("  %s" % (main.describe(),))
    print("  root collective %6.2f deg, inflow %5.2f m/s, thrust %8.0f N"
          % (collective, inflow, loads.thrust))
    print("  shaft torque %8.0f N m, power %7.1f kW, figure of merit %5.3f"
          % (loads.shaft_torque, loads.power / 1000.0, loads.figure_of_merit))
    print("  the torque balance asks the tail rotor for %6.0f N on the %.2f m"
          " arm" % (thrust, UH1_TAIL_ARM))
    return thrust


def the_two_rulers(thrust):
    """One thrust, two rules, and the pedals each of them answers with."""
    law = MixingLaw()
    by_element = collective_for_thrust(tail_rotor(), thrust)
    by_report = collective_by_equation_37(thrust)
    element_pedal = pedal_for_collective(law, by_element)
    report_pedal = pedal_for_collective(law, by_report)
    # The mixing law read both ways, which is what puts the two collectives above
    # into inches: a collective to a pedal and back again is the same collective.
    for probe in (0.0, 7.85, 18.0):
        assert abs(collective_for_pedal(law, pedal_for_collective(law, probe))
                   - probe) < 1e-12, probe
    gap_deg = by_element - by_report
    gap_in = element_pedal - report_pedal
    travel = UH1_TAIL_COLLECTIVE_LEFT_DEG - UH1_TAIL_COLLECTIVE_RIGHT_DEG
    print()
    print("the two rulers on that one thrust, and the pedal each asks for it:")
    print("  blade element (Rotor.uh1h_tail, twist %+.2f deg, %d stations):"
          " %6.2f deg -> %+5.2f in of pedal"
          % (UH1_TAIL_TWIST_DEG, STATIONS, by_element, element_pedal))
    print("  equation 37   (airframe.tail_rotor_thrust, what flies):"
          "         %6.2f deg -> %+5.2f in of pedal"
          % (by_report, report_pedal))
    print("  the gap: %+5.2f deg of collective, %+5.2f in of pedal, %.0f per"
          " cent of the %.2f deg the pedals sweep"
          % (gap_deg, gap_in, abs(gap_deg) / travel * 100.0, travel))
    print("  (the pedals' own neutral is %.1f deg of tail collective, so the"
          " element's %+5.2f in is %.2f deg of left)"
          % (law.tail_neutral_deg, element_pedal,
             by_element - law.tail_neutral_deg))

    # The same two rulers again, this time as the aircraft's *own* trims at the
    # same mass: the airframe's six unknown solve goes through equation 37, and
    # RotorControlModel's two unknown one through the blade element.  Neither is
    # the sweep above - each carries its own hover - which is the point of
    # printing them: four answers to "where do the pedals sit in a hover".
    airframe = Airframe(mass=UH1_MAX_GROSS_MASS)
    controls, state = airframe.trim_hover()
    angles = airframe.law.mix(controls)
    forces = airframe.forces(airframe.control_pitch_of(angles, controls.long_stick),
                             state=state)
    element = RotorControlModel()
    element_controls = element.trim_hover(mass=UH1_MAX_GROSS_MASS)
    print()
    print("the flight's own two trims at that mass, each with its own hover:")
    print("  Airframe.trim_hover       : collective %5.2f in, pedal %+5.2f in ->"
          " tail collective %6.2f deg"
          % (controls.collective, controls.pedal, angles.tail_collective_deg))
    print("    its main rotor torque is %8.0f N m, so its own balance asks %6.0f N"
          " of tail thrust"
          % (forces.rotor.torque, forces.rotor.torque / UH1_TAIL_ARM))
    print("    on the %.2f m arm, and equation 37 held at the pedal it returned"
          " makes %6.0f N" % (UH1_TAIL_ARM, forces.tail_thrust))
    print("  RotorControlModel.trim_hover: collective %5.2f in, pedal %+5.2f in"
          " -> tail collective %6.2f deg"
          % (element_controls.collective, element_controls.pedal,
             element.law.mix(element_controls).tail_collective_deg))
    return element_pedal, report_pedal, gap_deg, gap_in


def twist_sweep(thrust, sample_count=STATIONS):
    """What a degree of tail blade twist is worth, at the balance's own thrust.

    For every twist: the collective that holds that thrust, the pedal the blade
    element answers with, the gap to the pedal equation 37 answers with, the
    figure of merit at that collective, the worst angle of attack on the blade,
    the share of the total thrust the inboard negative angle stations make, what
    one more degree of collective is worth in newtons, and the thrust at the
    right pedal stop - which is the condition the twist argument is really about.
    A tail rotor has to push both ways - the nose has to go right as well as
    left, powered and autorotating - and -10 deg of tail collective is the stop a
    pilot reaches for to do it.
    """
    law = MixingLaw()
    report_pedal = pedal_for_collective(law, collective_by_equation_37(thrust))
    print()
    print("the twist sweep at the %6.0f N balance, %d stations a blade:"
          % (thrust, sample_count))
    print("   twist  coll.    pedal    gap    gap     FoM   worst   inboard"
          "   N per   stop at")
    print("     deg    deg       in     deg     in           alpha    thrust"
          "     deg    -10 deg")
    rows = []
    for twist in TWIST_SWEEP:
        tail = tail_rotor(twist_deg=twist, sample_count=sample_count)
        collective = collective_for_thrust(tail, thrust)
        inflow, loads = hover_loads(tail, collective)
        pedal = pedal_for_collective(law, collective)
        gap_deg = collective - collective_by_equation_37(thrust)
        share, worst, crossing = inboard_share(tail, loads)
        lower = hover_loads(tail, collective - AUTHORITY_STEP_DEG)[1]
        upper = hover_loads(tail, collective + AUTHORITY_STEP_DEG)[1]
        per_deg = (upper.thrust - lower.thrust) / (2.0 * AUTHORITY_STEP_DEG)
        right = hover_loads(tail, UH1_TAIL_COLLECTIVE_RIGHT_DEG)[1]
        left = hover_loads(tail, UH1_TAIL_COLLECTIVE_LEFT_DEG)[1]
        print("  %+6.2f  %6.2f  %+7.2f  %+6.2f  %+6.2f  %5.3f  %+6.1f"
              "  %+7.1f%%  %6.0f  %8.0f"
              % (twist, collective, pedal, gap_deg, pedal - report_pedal,
                 loads.figure_of_merit, worst, share * 100.0, per_deg,
                 right.thrust))
        rows.append(dict(twist=twist, tail=tail, collective=collective,
                         inflow=inflow, loads=loads, pedal=pedal,
                         gap_deg=gap_deg, gap_in=pedal - report_pedal,
                         report_pedal=report_pedal, share=share, worst=worst,
                         crossing=crossing, per_deg=per_deg, right=right,
                         left=left))
    return rows


def twist_notes(rows):
    """Both pedal stops at every twist, and where the two rulers agree.

    The stop table is the second half of the twist argument and the half the
    figure of merit does not show: washing a tail blade out buys a little power
    and takes it out of the *thrust*, and it does so asymmetrically, because the
    right pedal stop is negative collective and the left one is positive.  The
    gap line under it is the other number this file exists for - the twist at
    which the blade element and equation 37 ask for the same pedals, which is
    where the two rulers the constants sit between stop disagreeing.
    """
    print()
    print("the two stops and the two rulers at every twist:")
    print("   twist  coll.   worst   inboard   crosses   stop at   stop at"
          "     FoM")
    print("     deg    deg   alpha    share     at r/R    -10 deg   +18 deg")
    for row in rows:
        print("  %+6.2f  %5.2f  %+6.1f  %+7.1f%%    %5.2f  %+8.0f  %+8.0f  %5.3f"
              % (row["twist"], row["collective"], row["worst"],
                 row["share"] * 100.0, row["crossing"] / row["tail"].radius,
                 row["right"].thrust, row["left"].thrust,
                 row["loads"].figure_of_merit))
    crossing = twist_of_no_gap(rows)
    settled = [row for row in rows if row["twist"] == 0.0][0]
    band = max(abs(row["gap_in"]) for row in rows
               if AGREEMENT_BAND_DEG <= row["twist"] <= 0.0)
    print()
    print("  the two rulers ask for the same pedals at %+.2f deg of twist; at 0"
          " the blade element" % (crossing,))
    print("  asks %.2f deg less collective than equation 37, which is %+.2f in"
          " of pedal, and the two" % (-settled["gap_deg"], settled["gap_in"]))
    print("  are never more than %.2f in apart between %+.0f and 0 deg."
          % (band, AGREEMENT_BAND_DEG,))


def twist_of_no_gap(rows):
    """The twist the blade element and equation 37 agree on, by interpolation.

    Between the two sweep points whose gap changes sign: the survey is only as
    fine as its step, and this is a reading of where that step crosses rather
    than a fit.
    """
    for index in range(1, len(rows)):
        before, after = rows[index - 1], rows[index]
        if before["gap_deg"] == 0.0:
            return before["twist"]
        if (before["gap_deg"] < 0.0) != (after["gap_deg"] < 0.0):
            share = -before["gap_deg"] / (after["gap_deg"] - before["gap_deg"])
            return before["twist"] + share * (after["twist"] - before["twist"])
    return None


def precone_sweep(thrust, sample_count=STATIONS):
    """What the precone buys and what it costs, at the same balance.

    The precone moves no load at all in this model - ``Rotor.compute`` resolves
    the blades in the disc plane, so the angle only sets the flapping reference
    the teeter step will use - and the last column says so by comparing every
    station's thrust against the zero precone case to the last bit.  What is left
    is the two things the constant is really chosen by: the coning the flight's
    own flapping wants, for two blade masses and at an overload pedal worth 130
    per cent of the balance's thrust, and the tip rise that buys, which is the
    clearance only a drawing can confirm.
    """
    reference = tail_rotor(precone_deg=0.0, sample_count=sample_count)
    _, plain = hover_loads(reference, collective_for_thrust(reference, thrust))
    print()
    print("the precone sweep, %d stations a blade, coning for a %.1f kg and a"
          " %.1f kg blade:" % (sample_count, BLADE_MASSES[0], BLADE_MASSES[1]))
    print("  precone   tip rise   coning    coning    at %.0f %%:   coning"
          "    coning   loads" % (OVERLOAD * 100.0,))
    print("      deg         mm   4.0 kg   10.0 kg          4.0 kg   10.0 kg"
          "   move?")
    rows = []
    for precone in PRECONE_SWEEP:
        tail = tail_rotor(precone_deg=precone, sample_count=sample_count)
        collective = collective_for_thrust(tail, thrust)
        _, loads = hover_loads(tail, collective)
        overload = hover_loads(tail,
                               collective_for_thrust(tail, OVERLOAD * thrust))[1]
        same = (loads.thrust == plain.thrust
                and loads.shaft_torque == plain.shaft_torque
                and [sample.thrust_per_span for sample in loads.samples]
                == [sample.thrust_per_span for sample in plain.samples])
        print("  %7.2f  %10.1f  %8.2f  %8.2f  %14.2f  %8.2f  %s"
              % (precone, tip_rise_mm(tail, precone),
                 coning_deg(tail, loads, BLADE_MASSES[0]),
                 coning_deg(tail, loads, BLADE_MASSES[1]),
                 coning_deg(tail, overload, BLADE_MASSES[0]),
                 coning_deg(tail, overload, BLADE_MASSES[1]),
                 "no" if same else "YES"))
        rows.append(dict(tail=tail, precone=precone, loads=loads,
                         overload=overload, collective=collective, same=same,
                         tip_rise=tip_rise_mm(tail, precone),
                         coning_light=coning_deg(tail, loads, BLADE_MASSES[0]),
                         coning_heavy=coning_deg(tail, loads, BLADE_MASSES[1]),
                         overload_light=coning_deg(tail, overload, BLADE_MASSES[0]),
                         overload_heavy=coning_deg(tail, overload, BLADE_MASSES[1])))
    return rows


def convergence(thrust):
    """The sweep at three spacings, since a blade element table is a sampling.

    The two twists that matter - the one the model ships and the washout the
    comment's own argument is about - asked at 10, 20 and 40 stations a blade.
    A constant quoted off a table is only worth the spacing it was read at.
    """
    law = MixingLaw()
    report = pedal_for_collective(law, collective_by_equation_37(thrust))
    print()
    print("the same two twists at three spacings a blade, which is what settles"
          " the comment's own")
    print("figures: it quotes -52 deg and four per cent, and those are 10 station"
          " readings.")
    print("   twist  stations   coll.    pedal  gap_deg   gap_in     FoM   worst"
          "   inboard   crosses")
    print("     deg                     in      deg       in           alpha"
          "    thrust    at r/R")
    rows = []
    for twist in (0.0, -10.0):
        for stations in CONVERGENCE_STATIONS:
            tail = tail_rotor(twist_deg=twist, sample_count=stations)
            collective = collective_for_thrust(tail, thrust)
            inflow, loads = hover_loads(tail, collective)
            pedal = pedal_for_collective(law, collective)
            gap = collective - collective_by_equation_37(thrust)
            share, worst, crossing = inboard_share(tail, loads)
            print("  %+6.2f  %8d  %6.2f  %+6.2f  %+7.2f  %+7.2f  %5.3f  %+6.1f"
                  "  %+7.1f%%  %5.2f"
                  % (twist, stations, collective, pedal, gap, pedal - report,
                     loads.figure_of_merit, worst, share * 100.0,
                     crossing / tail.radius))
            rows.append(dict(twist=twist, stations=stations,
                             collective=collective, pedal=pedal, gap=gap,
                             inflow=inflow, fom=loads.figure_of_merit,
                             worst=worst, share=share, crossing=crossing))
    return rows


def flight_numbers():
    """One second of the aircraft that flies, as a tuple to compare bit for bit.

    Everything a constant on the tail rotor's *description* could reach if
    anything that flies read it: the trim's four sticks and the tail collective
    they mix to, a second of flight from that trim, and the forces at the end of
    it - tail thrust and main rotor torque among them, the two numbers a tail
    rotor is felt through.
    """
    airframe = Airframe(mass=UH1_MAX_GROSS_MASS)
    controls, state = airframe.trim_hover()
    airframe.reset(state, controls)
    for _ in range(FLIGHT_FRAMES):
        airframe.step(STEP_S, controls)
    final = airframe.state
    angles = airframe.law.mix(controls)
    pitch = airframe.control_pitch_of(angles, controls.long_stick)
    forces = airframe.forces(pitch, state=final)
    numbers = [controls.collective, controls.pedal, controls.long_stick,
               controls.lat_stick, angles.tail_collective_deg,
               forces.tail_thrust, forces.rotor.torque]
    for vector in (final.position, final.velocity, final.attitude, final.rates):
        numbers.extend([vector.x, vector.y, vector.z])
    numbers.append(final.rotor_speed)
    return tuple(numbers)


def boundary():
    """The claim the two constants rest on: nothing that flies can reach them.

    Both are rebound on the module that owns them - which is what
    ``Rotor.uh1h_tail`` reads when it is called - and the aircraft is re-trimmed
    and re-flown on the new numbers.  Everything that flies has to come back
    identical to the last bit, while a freshly built tail rotor has to come back
    carrying the new values, so the experiment is shown to bite where it should as
    well as not to bite where it should not.  That is the one way to *measure*
    the claim the constants' comment makes in prose - that the flight model's tail
    rotor is TM-73254's equation 37 and not this blade element one.
    """
    before = flight_numbers()
    aerodynamics.UH1_TAIL_TWIST_DEG = -10.0
    aerodynamics.UH1_TAIL_PRECONE_DEG = 3.0
    try:
        after = flight_numbers()
        fresh = Rotor.uh1h_tail()
        assert fresh.twist_deg == -10.0 and fresh.precone_deg == 3.0
        assert before == after, "rebinding the constants moved the flight"
    finally:
        aerodynamics.UH1_TAIL_TWIST_DEG = UH1_TAIL_TWIST_DEG
        aerodynamics.UH1_TAIL_PRECONE_DEG = UH1_TAIL_PRECONE_DEG
    restored = Rotor.uh1h_tail()
    assert restored.twist_deg == UH1_TAIL_TWIST_DEG
    assert restored.precone_deg == UH1_TAIL_PRECONE_DEG
    print()
    print("the two constants rebound to %+.1f deg of twist and %.1f deg of"
          " coning, and the flight re-flown:" % (fresh.twist_deg,
                                                 fresh.precone_deg))
    print("  a tail rotor built now carries them: %+.2f deg of twist, %.2f deg of"
          " coning" % (fresh.twist_deg, fresh.precone_deg))
    print("  and the aircraft is unmoved: %d numbers - the trim's four sticks,"
          " the tail collective it mixes to," % (len(before),))
    print("  the tail thrust, the main rotor torque, the position, the velocity,"
          " the attitude, the rates and the rotor speed, over %d frames -"
          % (FLIGHT_FRAMES,))
    print("  identical to the last bit.  Nothing that flies reads them; the"
          " flight's tail rotor is equation 37.")
    return before, after


def claims(thrust, rulers, rows, precodes, spacings):
    """Every number the tables print, asserted into a band of its own.

    The comments in the model cite this file, so the survey has to be a *check*
    and not a printout: a band a tenth of a degree or a couple of per cent wide,
    the width the comment and the model's own self test are written in, so that a
    change in the survey is a traceback while the last bit of a converged
    bisection is not the subject.
    """
    element_pedal, report_pedal, gap_deg, gap_in = rulers
    # The hover, and the thrust its torque balance asks of the tail rotor.
    assert 1800.0 < thrust < 2600.0, thrust
    # Two rulers, one thrust.  Both pedals are inside the pedals' own travel, and
    # the blade element is the more generous of the two rules: it needs *less*
    # collective than equation 37 for the same newtons, so its pedal is the closer
    # to neutral.  That gap is the width the constant is chosen inside.
    assert abs(element_pedal) < UH1_PEDAL_TRAVEL_IN, element_pedal
    assert abs(report_pedal) < UH1_PEDAL_TRAVEL_IN, report_pedal
    assert -3.0 < gap_deg < 0.0, gap_deg
    assert 0.0 < gap_in < 1.0, gap_in

    by_twist = dict((row["twist"], row) for row in rows)
    order = sorted(by_twist, reverse=True)
    # Washout is paid for in collective, monotonically: a degree of blade twist
    # is about a degree of tail collective here, so the list reads from no twist
    # at all at the top down to the most washed out blade at the bottom.
    for index in range(1, len(order)):
        assert (by_twist[order[index]]["collective"]
                > by_twist[order[index - 1]]["collective"]), order[index]
    # The three collectives and three figures of merit the constants' comment
    # quotes, at the 20 stations this table is cut at.
    for twist, collective, fom in ((0.0, 7.85, 0.663), (-4.0, 10.85, 0.672),
                                   (-10.0, 15.35, 0.680)):
        assert abs(by_twist[twist]["collective"] - collective) < 0.02, twist
        assert abs(by_twist[twist]["loads"].figure_of_merit - fom) < 0.002, twist
    # The inboard blade at zero twist: the worst angle on it, and the share of
    # the thrust it makes, which is small and negative rather than a loss.
    assert abs(by_twist[0.0]["worst"] + 57.1) < 0.5, by_twist[0.0]["worst"]
    assert abs(by_twist[0.0]["share"] + 0.032) < 0.005, by_twist[0.0]["share"]
    for row in rows:
        assert row["per_deg"] > 0.0, row["twist"]
        assert 0.6 < row["loads"].figure_of_merit < 0.75, row["twist"]
        # Pushing both ways: the right pedal stop pulls the other way at every
        # twist, and even at 12 deg of washout the left stop makes a fifth again
        # the hover's own thrust.
        assert row["right"].thrust < 0.0, (row["twist"], row["right"].thrust)
        assert row["left"].thrust > 1.2 * row["loads"].thrust, row["twist"]
        # The inboard stations pull the wrong way on every twist in the sweep,
        # wash in included, and the angle of attack changes sign further out the
        # more the blade is washed out.
        assert row["worst"] < 0.0, (row["twist"], row["worst"])
        assert -0.07 < row["share"] < 0.0, (row["twist"], row["share"])

    # What the twist costs where it is felt.  The untwisted blade is *symmetric*
    # in authority - its two stops pull within three per cent of the same
    # newtons, which is the property the constants' comment argues for - and wash
    # out breaks it the expensive way round: at 10 deg of it the left stop has
    # lost two thirds of its thrust and the right one has gained a fifth.
    assert abs(by_twist[0.0]["left"].thrust + by_twist[0.0]["right"].thrust) \
        < 0.05 * thrust, (by_twist[0.0]["left"].thrust,
                          by_twist[0.0]["right"].thrust)
    assert abs(by_twist[0.0]["left"].thrust / thrust - 3.08) < 0.05
    assert abs(by_twist[-10.0]["left"].thrust / thrust - 1.50) < 0.05
    assert abs(by_twist[-10.0]["right"].thrust / thrust + 3.68) < 0.05
    for twist in (2.0, 4.0, 6.0):
        assert by_twist[twist]["left"].thrust > by_twist[0.0]["left"].thrust, twist
    for twist in (-2.0, -4.0, -6.0, -8.0, -10.0, -12.0):
        assert by_twist[twist]["left"].thrust < by_twist[0.0]["left"].thrust, twist

    # The precone: no load anywhere moves, the coning the flight wants is a small
    # fraction of the built in coning at every angle in the sweep, and the tip
    # rise is the geometry it says it is.
    for row in precodes:
        assert row["same"], row["precone"]
        assert row["coning_light"] > row["coning_heavy"] > 0.0, row["precone"]
        assert row["overload_light"] > row["coning_light"], row["precone"]
        assert row["overload_heavy"] > row["coning_heavy"], row["precone"]
        assert row["coning_light"] < UH1_TAIL_PRECONE_DEG, row["precone"]
        assert abs(row["tip_rise"] - 1000.0 * row["tail"].radius
                   * math.sin(math.radians(row["precone"]))) < 1e-9
    # The four conings and two tip rises the constants' comment quotes.
    shipped = by_precone(precodes, UH1_TAIL_PRECONE_DEG)
    assert abs(shipped["coning_light"] - 0.88) < 0.02, shipped["coning_light"]
    assert abs(shipped["coning_heavy"] - 0.35) < 0.02, shipped["coning_heavy"]
    assert abs(shipped["overload_light"] - 1.12) < 0.02
    assert abs(shipped["overload_heavy"] - 0.45) < 0.02
    assert abs(shipped["tip_rise"] - 33.8) < 0.1, shipped["tip_rise"]
    assert abs(by_precone(precodes, 2.0)["tip_rise"] - 45.0) < 0.1
    assert UH1_TAIL_PRECONE_DEG in PRECONE_SWEEP
    assert UH1_TAIL_TWIST_DEG in TWIST_SWEEP

    # Two rulers, and the band of twist inside which they agree: the gap crosses
    # zero a little past two degrees of washout, and stays inside half an inch of
    # pedal all the way from there to no twist at all.
    assert -4.0 < twist_of_no_gap(rows) < -2.0, twist_of_no_gap(rows)
    band = [row for row in rows
            if AGREEMENT_BAND_DEG <= row["twist"] <= 0.0]
    assert len(band) == 4, len(band)
    assert all(abs(row["gap_in"]) < 0.6 for row in band)

    # The sampling.  The collective and the figures of merit are settled well
    # inside their own last figure at 20 stations, and the *share* the inboard
    # blade makes is stable to a tenth of a per cent at every spacing - while the
    # worst angle of attack on it is not: the innermost station is nearer the root
    # the finer the blade is cut, so the angle runs away to -39, -57 and -69 deg
    # at 10, 20 and 40 stations and only means anything with a spacing beside it.
    settled = {}
    for entry in spacings:
        settled.setdefault(entry["twist"], {})[entry["stations"]] = entry
    for twist in settled:
        fine = settled[twist][CONVERGENCE_STATIONS[-1]]["collective"]
        for stations in CONVERGENCE_STATIONS[:-1]:
            assert abs(settled[twist][stations]["collective"] - fine) < 0.1, \
                (twist, stations)
    for twist in settled:
        shares = [settled[twist][stations]["share"]
                  for stations in CONVERGENCE_STATIONS]
        assert max(shares) - min(shares) < 0.005, twist
        angles = [settled[twist][stations]["worst"]
                  for stations in CONVERGENCE_STATIONS]
        assert max(angles) - min(angles) > 10.0, twist
    assert abs(settled[0.0][10]["worst"] + 39.1) < 0.5, settled[0.0][10]["worst"]
    assert abs(settled[0.0][20]["worst"] + 57.1) < 0.5, settled[0.0][20]["worst"]
    assert abs(settled[0.0][40]["worst"] + 69.0) < 0.5, settled[0.0][40]["worst"]
    assert abs(settled[0.0][10]["share"] + 0.033) < 0.005, settled[0.0][10]["share"]
    assert settled[0.0][10]["crossing"] > settled[0.0][20]["crossing"]
    return 3 + 3 * len(rows) + 4 * len(precodes) + 2 * len(spacings) + 18


def by_precone(precodes, precone_deg):
    """The sweep's row for one precone angle."""
    return [row for row in precodes if row["precone"] == precone_deg][0]


def main():
    """The survey: three tables, the boundary claim, and the transcript."""
    stream = open(OUT_PATH, "w", encoding="utf-8")
    sys.stdout = stream
    try:
        thrust = the_hover()
        rulers = the_two_rulers(thrust)
        rows = twist_sweep(thrust)
        twist_notes(rows)
        precodes = precone_sweep(thrust)
        spacings = convergence(thrust)
        boundary()
        held = claims(thrust, rulers, rows, precodes, spacings)
        print()
        print("%d claims held: the survey above is the measurement the tail"
          " rotor's two constants are cited from." % (held,))
    finally:
        sys.stdout = sys.__stdout__
        stream.close()
    print("wrote %s" % (OUT_PATH,))
    return 0


if __name__ == "__main__":
    sys.exit(main())



