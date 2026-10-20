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

Three missions fly on it, each its own class below and each with its own loops:

* :class:`HumanPilot` is the long one - from the pad to a hundred metres up and
  a kilometre out, which is the mission the playability runs flew;
* :class:`HoverTurnPilot` is the short one - a hover twenty feet up over the pad,
  a full turn to the right about the vertical axis, and down again onto the
  skids, which is a minute of flight anybody watching can see the whole of;
* :class:`AutorotationPilot` is the one with the engine in it - five hundred feet
  up at sixty knots, the twist grip rolled right off, and down again on the
  rotor, which is the mission ``_rt_demo_autorotate`` opens a window on and
  ``_rt_autoro_probe`` measures.

Nothing but the nine keys is touched in any of them - the eight the four axes are
on, and the twist grip's own two, E and Q, which is a fifth position and not one
of the axes - and all three read the aircraft at :data:`REACTION_S` rather than
at the frame rate, so what a window shows is a person flying and not a script
stepping.
"""

import math

import pygame

from airframe import FOOT, KNOT
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

#: One frame of the twist grip's own key, in grip travel: the fourth of
#: :meth:`simulation.PilotInput.key_rates`, which is the rate the fifth position
#: turns at rather than the four axes above it - E rolling the grip on, Q off, the
#: two scancodes ``main.pilot_keys`` holds.  The grip is not an axis: it is read
#: on its own by :meth:`simulation.PilotInput.throttle`, and a pilot who cuts it
#: holds a key for it exactly as they hold one for a stick.
THROTTLE_STEP = PilotInput().key_rates()[3] * SIM_TIME_STEP_S

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


# ---------------------------------------------------------------------------
# The engine-off mission: five hundred feet up at sixty knots, the twist grip
# rolled right off, and down again on the rotor.
# ---------------------------------------------------------------------------

#: The condition the mission is flown at: sixty knots, which is the report's own
#: autorotation speed - ``airframe.Airframe.trim_autorotation``'s docstring is
#: where the 9.1 m/s at 60 kt comes from - and five hundred feet of skid, which is
#: high enough for the descent to settle on its trim before the cushion rather
#: than being flown into the ground out of the entry.
AUTOROTATION_SPEED_KT = 60.0
AUTOROTATION_SPEED_MPS = AUTOROTATION_SPEED_KT * KNOT
CLIMB_SKID_FT = 500.0
#: And the height that climb ends at, in metres of c.g. - named for *this*
#: mission, because ``CLIMB_ALT_M`` is the long mission's own height at the top
#: of the file and is what ``HumanPilot``'s speed ramp is solved from.  A second
#: binding of that name down here is the cruise winding its speed on against a
#: height it never reaches, which is a hundred metre hover instead of a kilometre.
AUTOROTATION_CLIMB_ALT_M = CLIMB_SKID_FT * FOOT + UH1_CG_HEIGHT_ON_GROUND

#: The descent holds the lever and nothing else.  Banked rotor speed is the trade
#: a pilot has and this model does not: the autorotation trim is the *equilibrium*
#: lever, so taking the lever below it is buying sink, and the revolutions a
#: surplus of driving torque would bank for it are not there to buy yet - the
#: disc has to be descending hard enough to drive that much harder first, and the
#: heave that pays for it takes seconds the flap of a descent does not have.  A
#: loop that asked the descent for head speed instead bought the sink and lost the
#: rotor both: two inches of lever below the trim, asked for a hundred and six per
#: cent, arrived on a rotor at fifty-eight.

#: How close to the climb's height and speed counts as up there and on the
#: number: the loops hold both asymptotically, so the two goals are bands and the
#: cut is made from inside them.  The speed band is the wider of the two, since
#: the cut is a moment and the trim is flown either side of it.
CLIMB_TOLERANCE_M = 3.0
CLIMB_SPEED_TOLERANCE_MPS = 2.0

#: The cut, held for as long as the grip takes to reach the stop: the twist grip
#: is the fifth position and not one of the four axes, so it has a key of its own -
#: ``main.pilot_keys``' Q rolling it off, which is what :meth:`AutorotationPilot.keys`
#: presses.  The shaft is dead from the moment it is closed, so the lever is at
#: the autorotation trim and the speed is held with the cyclic for the whole of
#: the cut: the aircraft is on its way down as soon as the engine is off.
CUT_GRIP_KEY = pygame.KSCAN_Q

#: The lever's own rate on a held key, in inches a second: the fastest a hand
#: under the cushion can move the collective, since the cushion is flown on the
#: same key the whole mission is - the collective's own rate times the step
#: granularity, read off the model, which is 1.51 in/s.  Nothing that asks the
#: lever for more than this gets it: a demand that jumps further than a key can
#: travel is a demand the hand chases for ever, and the lever answers with a
#: limit cycle rather than a position - which is what the first cut of the
#: cushion did, between full pull and the trim, three seconds to the cycle.  It
#: is also what sets how long a flare has to be: four inches of lever is three
#: seconds of hand whatever the loop asks for, and at eleven metres a second
#: that is thirty metres of height and no less.
CUSHION_KEY_RATE_IN = PilotInput().key_rates()[0] * COLLECTIVE_TRAVEL_IN

#: The flare, and the cushion under it.  From :data:`FLARE_SKID_FT` of skid the
#: sink the descent is asked for thins from the rate the descent was already at
#: - the trim's own sink, which is what the table's descent row solved to - to
#: :data:`CUSHION_SINK_MPS` on the skids, and the lever is asked to pay for the
#: difference: :data:`CUSHION_GAIN_IN` inches a second of lever for every metre
#: a second the sink is over the line, integrated into a position, limited to
#: what a key can move the lever at and to :data:`CUSHION_PULL_IN` inches above
#: the descent's own trim.  A schedule and not a height to pull at, which is
#: what the first cut of the cushion was: the lever's own key rate means a pull
#: big enough to arrest a descent takes seconds, so asking for it all at once
#: asks for it too early - the sink stops fifty feet up, the lever then holds
#: the aircraft against a rotor with no engine driving it, and the last of the
#: revolutions go on hovering rather than on landing, which is how eleven inches
#: from a hundred and forty feet ended in a fall from forty feet.  Asking for it
#: as the descent gives time to be asked is the whole of the fix, so the pull
#: opens with nothing to do and is still coming in when the skids meet.  The
#: rate is the number that matters, and a rate under
#: ``simulation.UH1_HARD_LANDING_MPS`` is the whole of what the landing owes.
#:
#: The flare is an *attitude* as well as a lever, and the schedule is the same
#: shape: :data:`FLARE_PITCH_DEG` degrees of nose up on top of the descent's own
#: attitude, coming up from none of it at the flare's height.  It has to be
#: commanded, because a lever pulled on a sixty knot rotor is a nose up moment
#: the pilot never asked for: the first cut of this cushion held the descent's
#: own attitude and let the pull have its head, and the model pitched to
#: twenty-seven degrees at eleven metres a second - the thrust tilted aft, the
#: blades stalled on a collective the speed was still too high to carry, and the
#: sink *rose* from 2 200 to 3 000 fpm for the first half of a two hundred foot
#: cushion before a foot of it was arrested.  Flaring the attitude is also what
#: keeps the rotor turning: an aft tilted disc at sixty knots has more air
#: driving it, which is the model's own windmill brake state and the whole of
#: why a real flare recovers the revolutions it spent getting there - twenty
#: degrees of it at two hundred feet lands on a rotor two and a half per cent
#: faster than holding the trim's attitude does.
#:
#: What it cannot do is *land* on it: a thrust tilted twenty degrees aft is a
#: fifth of the cushion's lift pointing backwards, and a flare held all the way
#: in arrives on whatever the tilt left - four metres a second in the same two
#: hundred feet that lands at two with the attitude back on the trim.  The
#: obvious answer, to peak the flare above the ground and take the nose back down
#: to the trim for the last of the height, is worse than the problem: the model
#: answers a nose up at sixty knots with the whole of the speed converted in
#: seconds, so an attitude that comes off again lets the machine balloon - eight
#: hundred feet a minute *up* at the peak of one - and then fall from it on a
#: rotor with no height left under it, which is twelve metres a second at the
#: ground and a rotor still at ninety-five.  The flare is monotone instead,
#: :data:`FLARE_PITCH_DEG` at the skids and none of it at the height, and what
#: the attitude does not arrest the lever does.
#:
#: The attitude's own height is not the cushion's, and need not be:
#: :data:`FLARE_PITCH_FROM_FT`, when it is said, is the height the *nose* comes
#: up from, and a flare begun above the cushion is revolutions bought out of the
#: air driving the disc rather than out of the lever - which is the one place in
#: this descent where rotor speed can be had without sink paid for it, since the
#: ramp is monotone either way and the lever's cushion is untouched by it.  It is
#: also what a real autorotation does: the cyclic flare first, at height, and the
#: lever after it, which is why the one schedule is two.  ``None`` is the
#: cushion's own height, which is the two schedules as they were first cut.

#: The cushion is two seconds of heave on a lever a key moves at 1.51 in/s, and
#: both of those numbers are spent out of the *height*: twelve feet of skid at
#: the trim's eleven metres a second is one second of flight, so the lever has
#: not crossed before the skids have - the cushion as it was first cut is worth
#: nothing at all, 11.23 m/s at the ground, which is the trim's own sink and no
#: less.  Six and a half inches of lever is four seconds of hand, and a hundred
#: and fifty feet of skid is the height four seconds can be had from: from
#: there the same schedule arrives at 2.26 m/s on a rotor still at eighty-seven.
#: The pull and the height it opens at are one number: the height without the
#: pull is *worse* than twelve feet of skid, 12.73 m/s, because a schedule with
#: the height to finish over-flares the machine instead of arresting it, and the
#: pull without the height is the same 11.23 m/s the mission already flew.
#: The nose is asked for the gentler of the manual's two: eight degrees against
#: the twelve, which is worth a fifth of a metre a second of arrival on the same
#: rotor, the flare being monotone either way.
FLARE_SKID_FT = 150.0
FLARE_PITCH_DEG = 8.0
#: The height the nose comes up *from*: ``None`` for the cushion's own height,
#: and a number for a cyclic flare begun above the lever's cushion.  See the
#: comment above this block.
FLARE_PITCH_FROM_FT = 55.0
#: The height the nose comes back *down* at: the manual's own level off, ten
#: to fifteen feet, which is where the flare peaks and the attitude starts
#: back towards the trim.  Above it the flare is monotone, as the long
#: schedule was; at it the disc starts back level and the lever pays for the
#: last of the height under a disc that is not tilted - which is the one
#: place a nose up taken back down is not the balloon it is at height,
#: because the height it would climb into is the twelve feet it has left.
LEVEL_OFF_SKID_FT = 12.0
#: See :data:`FLARE_SKID_FT`, which is the same number: what the pull is worth
#: and the height it opens at are chosen together, and the schedule's original
#: two inches arrive on the ground at the trim's own sink.
CUSHION_PULL_IN = 6.5
CUSHION_SINK_MPS = 2.0
CUSHION_GAIN_IN = 1.0

#: The descent is flown on the trim's own lever, and this is the one way it is
#: not: :data:`DESCENT_LEVER_DOWN_IN` inches *under* that lever, held for the
#: whole of the descent and given back by the cushion.  A rotor with no engine
#: spins up when the lever comes down - the blades meet the air at a shallower
#: angle, so the windmill drives them harder and the machine comes down faster -
#: which is the model's own physics and the whole of the "lever down for RPM"
#: every real autorotation is flown by.  It is the one place in this descent
#: where revolutions are bought with the *attitude* untouched, and so without the
#: speed being converted to height: a flare begun at height carries the machine
#: up on its own sixty knots and drops it again - thirteen hundred feet a minute
#: of rise at three hundred, in this model, arriving at nine metres a second on a
#: rotor it had already charged - and what the lever costs instead is the sink it
#: adds, which is the one thing the cushion is there to pay.  The cushion's pull
#: starts where the lever the descent actually held is, so that the pull is a
#: pull and not a step.
DESCENT_LEVER_DOWN_IN = 0.0


def clip(value, low, high):
    """*value* held between *low* and *high*."""
    return max(low, min(high, value))


def wrap(deg):
    """An angle in degrees, in (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def skid_ft(telemetry):
    """How high the *skids* are, ft: what a pilot flares by.

    The model measures its altitude from the c.g., which stands
    ``simulation.UH1_CG_HEIGHT_ON_GROUND`` above the skids, so the height a
    pilot calls theirs is that much less than ``telemetry.alt_agl``.  It is the
    height a cushion is begun at and the height a hover is held at, and it is
    the same subtraction :class:`HoverTurnPilot` makes of its hover.
    """
    return (telemetry.alt_agl - UH1_CG_HEIGHT_ON_GROUND) / FOOT


class TrimTable:
    """The model's own trims, read once: what the handbook hands a pilot.

    A row per speed, in the units the sticks are in - inches - and the pitch
    attitude the trim flies at, which is the reference the pilot's own attitude
    loop is written against.  Between the rows the table interpolates, so a speed
    the mission passes through has a collective and a stick position to match it.

    *airframe* is a scratch aircraft, not the one being flown: solving a trim
    moves an airframe's state, which is not something a pilot may do to the
    aircraft they are in.

    A pilot with a *descent* in their mission - an autorotation, which is flown
    on a dead shaft - gets one more row: see *autorotation_speed*, which is
    zero for a pilot that never has the engine off and is where
    :class:`AutorotationPilot`'s own mission reads its lever position from.
    """

    def __init__(self, airframe, speeds=TRIM_SPEEDS, altitude=ALT_REF_M,
                 autorotation_speed=0.0):
        self.speeds = tuple(float(speed) for speed in speeds)
        self.rows = []
        for speed in self.speeds:
            controls, state = airframe.trim_level_flight(speed,
                                                         altitude=altitude)
            self.rows.append((controls.collective, controls.long_stick,
                              controls.lat_stick, controls.pedal,
                              state.attitude_deg.y))

        #: The descent's own row - ``(collective, long, lat, pedal, pitch,
        #: sink)`` - or None for a pilot with no engine-off phase in their
        #: mission.  It is a *different* solve and not an interpolation of the
        #: rows above: an engine failure leaves the rotor free, and what the
        #: model then settles at in a sixty knot descent is
        #: :meth:`airframe.Airframe.trim_autorotation`'s answer with the torque
        #: taken off the shaft, which no level flight row can be.  The sink rate
        #: comes back with it, positive down, because it is what the whole
        #: mission is judged by and what ``_rt_demo_autorotate`` prints.
        self.autorotation = None
        if autorotation_speed:
            airframe.engine_torque = 0.0
            controls, state = airframe.trim_autorotation(
                float(autorotation_speed), altitude=altitude)
            self.autorotation = (controls.collective, controls.long_stick,
                                 controls.lat_stick, controls.pedal,
                                 state.attitude_deg.y,
                                 state.ground_velocity().z)

    def descent(self):
        """``(collective, long, lat, pedal, pitch, sink)`` of the autorotation.

        Raises :exc:`ValueError` for a pilot whose table was built without an
        *autorotation_speed*, so a mission that reads it before it has one says
        so where it happens rather than multiplying a None.
        """
        if self.autorotation is None:
            raise ValueError("this trim table has no autorotation row: it was"
                             " built with autorotation_speed=0.0")
        return self.autorotation

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
    #: And the third: the speed the table solves a *descent* row at, m/s, which
    #: is zero for the two missions that never have the engine off and is what
    #: :class:`AutorotationPilot` sets.
    TRIM_SPEEDS_MPS = TRIM_SPEEDS
    TRIM_ALT_M = ALT_REF_M
    TRIM_AUTOROTATION_MPS = 0.0

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


class AutorotationPilot(KeyboardPilot):
    """The mission with the engine in it: five hundred feet, sixty knots, down.

    The long mission's own climb, on the same four loops, at the mission's own
    speed:

    * the collective holds the height through a climb rate, the same shape
      :class:`HumanPilot` flies a hundred metres on;
    * the long stick holds sixty knots, on the pitch the model trims that speed
      at, so the climb is a cruise flown steeply;
    * the lateral stick holds the line north of the pad through a bank, and the
      pedals hold the nose, which are those missions' two loops unchanged.

    Then what the other two never do:

    * the twist grip is rolled right off - the fifth position, which is not one
      of the four axes and turns on a key of its own, E on and Q off - and the
      shaft is dead from there on;
    * the descent is flown on the model's *autorotation* trim, which is not the
      level flight trim of sixty knots and could not be: the rotor is free, and
      what that trim solves for instead is the *descent rate* the machine
      settles at with the torque off the shaft.  See :meth:`TrimTable.descent`;
    * and the collective comes up into ground effect at :data:`FLARE_SKID_FT` of
      skid, spending what the rotor has left - at most :data:`CUSHION_PULL_IN`
      inches - on :data:`CUSHION_SINK_MPS` rather than on a lever position,
      which is what turns a nine metre a second descent into an arrival rather
      than a wreck, and which stops pulling as soon as the sink has been paid
      for, rather than ballooning the aircraft back up on a rotor already
      running down.

    Nine keys and nothing else: the eight the four axes are on, and the grip's
    own two.  The grip is the one control here that is read every frame rather
    than every decision: :meth:`keys` holds E or Q until the grip reaches the
    position the mission asked for, exactly as :class:`KeyHands` holds an axis
    key, and the mission asks for full open until the cut and closed after it.
    """

    DESCRIPTION = ("five hundred feet up at 60 kt, the twist grip rolled right"
                   " off, and down again on the rotor")

    #: The trim table's own speeds and height: the two ends of the climb, so the
    #: baseline under the climb's loops is the trim of the speed the aircraft is
    #: *at*, and the height the whole mission is flown at.  And the third: the
    #: speed the table solves its *descent* row at, which is what
    #: :meth:`airframe.Airframe.trim_autorotation` is asked for.
    TRIM_SPEEDS_MPS = (0.0, 0.5 * AUTOROTATION_SPEED_MPS,
                       AUTOROTATION_SPEED_MPS)
    TRIM_ALT_M = AUTOROTATION_CLIMB_ALT_M
    TRIM_AUTOROTATION_MPS = AUTOROTATION_SPEED_MPS

    def __init__(self, sim, trims, reaction=REACTION_S, log=None):
        KeyboardPilot.__init__(self, sim, trims, reaction=reaction, log=log)
        self.phase = "pickup"
        self.climb_alt = AUTOROTATION_CLIMB_ALT_M
        self.took_off = None          # s, the skids off the pad
        self.climb_settled = None     # s, height and speed both held
        self.cut_at = None            # s, the twist grip rolled off
        self.descent_at = None        # s, the grip closed, the shaft dead
        self.cushion_at = None        # s, the lever pulled in ground effect
        self.cushion_in = None        # in, the cushion's own lever position
        self.cushion_sink = None      # m/s, the sink the cushion opened on
        self.lowest_rotor = 100.0     # %, the slowest the rotor was seen
        self.last_airborne_rate = 0.0  # m/s, the last rate read off the ground
        self.touchdown = None         # s, when the skids met the ground
        self.touchdown_rate = None    # m/s, the rate the pilot last read aloft
        self.landed_since = None      # s, since the skids were down
        self.want_throttle = 1.0      # the grip's own position, not an axis
        # Where the pad holds the four controls, which is where the mission
        # leaves them: the run's own trim with the lever at
        # ``simulation.PAD_COLLECTIVE``, which is what
        # :meth:`simulation.Simulation.on_the_pad` parks the aircraft in - and
        # what a machine whose engine has stopped has to be left in, since the
        # pad's own lever is *below* the hover trim and so holds it down.
        self.pad_axes = list(sim.trim_controls.collective_at(
            PAD_COLLECTIVE).to_axes())

    def keys(self, sim, dt):
        """The frame's keys: the four axes' hands, and the twist grip's two.

        :class:`KeyHands` has nothing to say about the grip - it is not a stick
        position - so the fifth control is held here, on the same terms and at
        the same rate: E while the grip is below what the mission asked for, Q
        while it is above, and neither inside :data:`THROTTLE_STEP` of it, which
        is one frame of a key.
        """
        keys = KeyboardPilot.keys(self, sim, dt)
        if self.done is not None:
            return keys
        error = self.want_throttle - sim.pilot.throttle()
        if error > THROTTLE_STEP:
            keys[pygame.KSCAN_E] = 1.0
        elif error < -THROTTLE_STEP:
            keys[CUT_GRIP_KEY] = 1.0
        return keys

    # -- the mission's own bookkeeping -----------------------------------

    def phase_of(self, sim):
        """What the mission is doing, from where the aircraft is.

        One step per look, like the other missions': off the skids, into the
        band of five hundred feet and sixty knots, the grip on its way to the
        stop, the grip within one frame of travel of it, the skids down to the
        cushion's height, and the skids on the ground.  Nothing here reads the
        frame.
        """
        telemetry = sim.telemetry()
        alt = telemetry.alt_agl
        speed = telemetry.ground_speed
        now = sim.sim_time
        if self.took_off is None and not sim.on_ground:
            self.took_off = now                      # the skids are off the pad
        if self.phase == "pickup":
            # The loops do the pickup; the name lasts until the aircraft is
            # properly off - the long mission's own guard, since on the pad the
            # lever is the only control with any weight on it.
            if not (sim.on_ground and alt < PARKED_ALT_M):
                self.phase = "climb"
        if self.phase == "climb":
            if (abs(alt - self.climb_alt) <= CLIMB_TOLERANCE_M
                    and abs(speed - AUTOROTATION_SPEED_MPS)
                    <= CLIMB_SPEED_TOLERANCE_MPS):
                if self.climb_settled is None:
                    self.climb_settled = now
                elif now - self.climb_settled >= HOVER_SETTLE_S:
                    self.phase = "cut"
                    self.cut_at = now
                    self.want_throttle = 0.0         # roll the grip off
            else:
                self.climb_settled = None
        if self.phase == "cut" and sim.pilot.throttle() <= THROTTLE_STEP:
            # Within one frame of travel of the stop is *closed*: the ratchet
            # turns the grip toward the stop a step at a time and the step is
            # clipped, not snapped, so a grip held down parks a rounding residue
            # above the stop - and an exact test against zero would leave the
            # mission in the cut for ever.  :data:`THROTTLE_STEP` is the same
            # tolerance the hands use to say they have arrived.
            self.phase = "descent"
            self.descent_at = now
        if self.phase in ("cut", "descent", "cushion"):
            self.lowest_rotor = min(self.lowest_rotor,
                                    telemetry.rotor_percent)
            self.last_airborne_rate = telemetry.height_rate
        if self.phase == "descent" and skid_ft(telemetry) <= FLARE_SKID_FT:
            # Into the cushion: the pull starts from the trim it is holding, so
            # the loop below opens on the lever the descent was flown at rather
            # than on the last flight's answer to the same question.
            self.phase = "cushion"
            self.cushion_at = now
            self.cushion_in = None
        if self.phase in ("cut", "descent", "cushion") and sim.on_ground:
            self.phase = "landed"
            self.touchdown = now
            self.touchdown_rate = self.last_airborne_rate
            self.landed_since = now
        if self.phase == "landed" and not sim.on_ground:
            # Light on its skids again: the lever is at the pad's own, which is
            # above the autorotation trim and no use to a rotor with nothing
            # driving it - so the arrival is not over after all, and the last
            # thirty feet are flown again.
            self.phase = "descent"
            self.landed_since = None

    # -- the pilot's own loops -------------------------------------------

    def decide(self, sim):
        """Where the four axes ought to be, from what the instruments say.

        Two things change with the phase and nothing else does: the *baseline*
        the loops are written around - the level flight trim of the speed the
        aircraft is at while it climbs, the autorotation trim once the shaft is
        dead - and whether the lever is at that trim or the pull above it.  The
        three cyclic loops are the long mission's own, unchanged.
        """
        telemetry = sim.telemetry()
        state = sim.airframe.state
        alt = telemetry.alt_agl
        speed = telemetry.ground_speed
        self.phase_of(sim)
        if self.phase == "landed":
            # Down and staying down: the lever back to the pad's own and the
            # cyclic and pedals back on the trim, which is where a mission
            # starts - the pad's own sticks, as the short mission also lands on.
            # The pull is not held: it is spent, and a lever held up on a rotor
            # with nothing driving it is a machine lifted off its skids again.
            return list(self.pad_axes)
        if self.phase == "pickup":
            # On the skids the cyclic and the pedals do nothing; the pickup is
            # the lever's doing, and the trim's hover fraction is where it goes.
            return [self.trims.hover_fraction()] + list(sim.pilot.axes()[1:])

        if self.phase == "climb":
            collective_t, long_t, lat_t, pedal_t, __ = self.trims.at(speed)
            rate_want = clip(RATE_GAIN_MPS * (self.climb_alt - alt),
                             -CLIMB_LIMIT_MPS, CLIMB_LIMIT_MPS)
            collective = (collective_t
                          + K_RATE_IN * (rate_want - telemetry.height_rate))
            pitch_t = self.trims.at(AUTOROTATION_SPEED_MPS)[4]
            pitch_want = clip(
                pitch_t - K_SPEED_DEG * (AUTOROTATION_SPEED_MPS - speed),
                pitch_t - PITCH_LIMIT_DEG, pitch_t + PITCH_LIMIT_DEG)
        else:
            # The descent, from the cut to the skids: the autorotation's own
            # lever, sticks and attitude, held at the trim - a rotor with no
            # engine has nothing to loop a height rate around, and the trim of
            # the speed the aircraft is at *is* the rate it comes down at.  What
            # is looped instead is the last of that rate: the cushion's pull is
            # a lever *rate* proportional to how much more sink there is than
            # :data:`CUSHION_SINK_MPS`, integrated into a position and limited
            # to what a key can move the lever at, so the pull stops where the
            # rate asked for is held and comes off again if the ground effect
            # holds more than that.
            collective_t, long_t, lat_t, pedal_t, pitch_t, __ = \
                self.trims.descent()
            collective = collective_t
            # The cyclic flare's own height, which is not the lever's: the
            # cushion's unless the mission said otherwise, and see the constants
            # above for what a flare begun above the cushion buys.  It is read
            # here, above the lever's own branch, because the *attitude* half of
            # the flare is not the cushion's to command - the descent is flown
            # through it - and the two branches below are therefore the flare and
            # the descent's own sixty knot attitude: inside the flare's height the
            # nose is on the schedule, above it the speed is held.
            if self.phase == "descent":
                # The lever down for the descent's own length, under the trim's
                # own lever: see :data:`DESCENT_LEVER_DOWN_IN` for what it buys
                # and what it costs.  It is not held through the cushion - the
                # pull starts under it - nor in the cut, where the shaft is still
                # turning.
                collective = collective_t - DESCENT_LEVER_DOWN_IN
            flare_from = (FLARE_SKID_FT if FLARE_PITCH_FROM_FT is None
                          else FLARE_PITCH_FROM_FT)
            if self.phase == "cushion":
                # The rate the descent is asked for as a *height rate*, which
                # is what the instruments are in: the sink the descent came in
                # with, thinning to -CUSHION_SINK_MPS on the skids, and the
                # lever's own key rate as the only limit on how fast the
                # the difference may be paid for - see the constants above for
                # why the schedule is a schedule and not a height.
                if self.cushion_in is None:
                    # The pull starts where the lever *is* - the descent's own
                    # trim, less whatever of it the descent was held under - so
                    # that the first frame of the cushion is a pull and not a
                    # step.  See :data:`DESCENT_LEVER_DOWN_IN`.
                    self.cushion_in = collective_t - DESCENT_LEVER_DOWN_IN
                    # And the rate the schedule thins from: the sink the
                    # descent actually arrived with, and the trim's own, whichever
                    # is the faster: see below.  A descent flown
                    # on its trim arrives at the trim's own sink, and the
                    # schedule is the one it always was; a descent held under
                    # the trim arrives faster than the trim's number, and a
                    # schedule anchored on the trim's would open with the
                    # aircraft already below it - a large error on the very
                    # first frame, the pull at its key rate and still climbing
                    # when the sink was beaten, which is the balloon that took
                    # two thousand feet a minute back off the rotor it had just
                    # been paid.  Anchored here, the first frame's error is
                    # zero and the pull only ever follows the schedule down.
                    # The trim's is kept when the aircraft arrives *slower* than
                    # it, because the other way round is a slow version of the
                    # same failure: an error that never goes away creeps into the
                    # cap over the whole of a two hundred foot cushion, and the
                    # flight that anchors on the slower number balloons seven
                    # hundred feet a minute where the trim's own schedule lands
                    # it at two point eight.
                    # See :data:`DESCENT_LEVER_DOWN_IN`.
                    self.cushion_sink = max(-telemetry.height_rate,
                                            self.trims.descent()[5],
                                            CUSHION_SINK_MPS)
                want = -max(CUSHION_SINK_MPS,
                            self.cushion_sink * skid_ft(telemetry)
                            / FLARE_SKID_FT)
                error = want - telemetry.height_rate
                # The pull only ever comes *up*: a flare is a lever raised, and
                # the first cut - which let the loop take the lever back down as
                # soon as the sink was beaten - oscillated, because the lever's
                # answer lags the sink by the two seconds of heave and the loop
                # was still spending when the sink had already gone.  One way,
                # the pull eases itself off as the schedule is met and never
                # starts pulling again.
                self.cushion_in = min(
                    self.cushion_in
                    + clip(CUSHION_GAIN_IN * error, 0.0, CUSHION_KEY_RATE_IN)
                    * self.reaction,
                    min(collective_t + CUSHION_PULL_IN, COLLECTIVE_TRAVEL_IN))
                collective = self.cushion_in
                # The attitude half of the flare: the manual's own beat, nose up
                # from the flare's height to FLARE_PITCH_DEG at the level off and
                # back to the trim under that, so the speed is bled at fifty feet
                # and the disc is level again for the last twelve - see the
                # constants above.  The cyclic below flies it against the pull's
                # own nose up moment.
                pitch_want = clip(
                    pitch_t + FLARE_PITCH_DEG
                    * clip(
                        min((flare_from - skid_ft(telemetry))
                            / (flare_from - LEVEL_OFF_SKID_FT),
                            skid_ft(telemetry) / LEVEL_OFF_SKID_FT),
                        0.0, 1.0),
                    pitch_t, pitch_t + FLARE_PITCH_DEG)
            elif skid_ft(telemetry) <= flare_from:
                # The cyclic flare, above the cushion's own height: the same
                # schedule the cushion flies, flown where there is height to
                # spend on it rather than revolutions - an aft tilted disc at
                # sixty knots has more air driving it, which is the model's own
                # windmill brake state, and the chimney the flare's revolutions
                # come up.  It rises to the level off's own height and holds
                # nothing: see the constants above for where the nose comes back
                # down, and why that is safe there and nowhere else.
                pitch_want = clip(
                    pitch_t + FLARE_PITCH_DEG
                    * clip(
                        min((flare_from - skid_ft(telemetry))
                            / (flare_from - LEVEL_OFF_SKID_FT),
                            skid_ft(telemetry) / LEVEL_OFF_SKID_FT),
                        0.0, 1.0),
                    pitch_t, pitch_t + FLARE_PITCH_DEG)
            else:
                pitch_want = clip(
                    pitch_t - K_SPEED_DEG * (AUTOROTATION_SPEED_MPS - speed),
                    pitch_t - PITCH_LIMIT_DEG, pitch_t + PITCH_LIMIT_DEG)

        long_stick = (long_t
                      + K_PITCH_IN * (telemetry.pitch_deg - pitch_want)
                      + K_PITCH_RATE_IN * telemetry.pitch_rate_deg)
        east = state.position.y - TRACK_EAST_REF_M
        bank_want = clip(-K_BANK_DEG * east, -BANK_LIMIT_DEG, BANK_LIMIT_DEG)
        lat_stick = (lat_t
                     + K_ROLL_IN * (bank_want - telemetry.roll_deg)
                     - K_ROLL_RATE_IN * telemetry.roll_rate_deg)
        pedal = (pedal_t
                 - K_YAW_IN * wrap(telemetry.yaw_deg - YAW_REF_DEG)
                 - K_YAW_RATE_IN * telemetry.yaw_rate_deg)
        return inches_to_axes(collective, long_stick, lat_stick, pedal)

    # -- the frame loop's view of it -------------------------------------

    def mission_done(self, sim):
        """The skids on the ground and staying there, or the flight wrecked.

        A rotor that has stopped has stopped: there is no long settling the other
        missions wait for, so the arrival is the end - once the aircraft is down,
        has stayed down a settling's worth of seconds, and the sticks are back
        within a key of the pad's own, which is where the mission left them.  A
        crash is the end too: the flight is over, and a pilot still on the keys
        of a wreck is a pilot walking away from it.
        """
        if self.done is None:
            if sim.crashed:
                self.done = sim.sim_time
            elif self.phase == "landed" and sim.on_ground:
                axes = sim.pilot.axes()
                on_the_pad = all(abs(axis - want) <= step
                                 for axis, want, step
                                 in zip(axes, self.pad_axes, KEY_STEP))
                if (on_the_pad and self.landed_since is not None
                        and sim.sim_time - self.landed_since
                        >= LANDING_SETTLE_S):
                    self.done = sim.sim_time
        return self.done is not None

    def note(self, sim):
        """Where the pilot is in the mission, in their own words."""
        telemetry = sim.telemetry()
        skid = skid_ft(telemetry)
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





#: The missions a caller may ask for by name, and the one that is flown when
#: nobody says: the short one, since it is a minute of flight and the whole of
#: it happens over the pad.
MISSIONS = {"hover-turn": HoverTurnPilot, "cruise": HumanPilot,
            "autorotate": AutorotationPilot}
DEFAULT_MISSION = "hover-turn"


def trims_for(airframe, pilot_class):
    """The trim table *pilot_class* flies on: its own speeds at its own height.

    *airframe* is a scratch aircraft of the caller's - solving a row moves it,
    which is why it is never the one being flown.  See :class:`TrimTable`.
    """
    return TrimTable(airframe, speeds=pilot_class.TRIM_SPEEDS_MPS,
                     altitude=pilot_class.TRIM_ALT_M,
                     autorotation_speed=pilot_class.TRIM_AUTOROTATION_MPS)



