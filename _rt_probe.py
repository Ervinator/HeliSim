# -*- coding: utf-8 -*-
"""Scratch: the numbers the pilot's gains have to be built from.

Travels, the pad's stick, the hover trim, and the model's own trims for the
speeds the mission wants - read, not guessed, so the loops in ``_rt_pilot`` can
be scaled to the aircraft instead of to trial and error.
"""

import math

from airframe import FOOT
from main import AIRFRAME_PRESET
from rotor_control import (PilotControls, UH1_COLLECTIVE_TRAVEL_IN,
                           UH1_LAT_STICK_TRAVEL_IN, UH1_LONG_STICK_TRAVEL_IN,
                           UH1_PEDAL_TRAVEL_IN)
from simulation import (SIM_TIME_STEP_S, Simulation, UH1_CG_HEIGHT_ON_GROUND,
                        airframe_preset)
from _rt_pilot import HOVER_ALT_M


def hover_step(label, seconds=8.0, every=60, **offsets):
    """Fly the twenty foot hover with one control stepped off its own trim.

    The trim is the model's own hover at that height, solved by
    :meth:`simulation.Simulation.trim_level_flight`; the step is *inches* added
    to the stick position that flies it, put there by
    :meth:`simulation.PilotInput.reset` - an axis, a ratchet, nothing held - so
    what is printed is the model's answer and not a pilot's.  This is the number
    the short mission's loops have to be built from: how many degrees a second
    an inch of pedal buys, how much pitch an inch of long stick buys and what
    speed that pitch settles at.
    """
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.trim_level_flight(0.0, altitude=HOVER_ALT_M)
    trim = sim.controls
    stepped = PilotControls(
        collective=trim.collective + offsets.get("collective", 0.0),
        long_stick=trim.long_stick + offsets.get("long_stick", 0.0),
        lat_stick=trim.lat_stick + offsets.get("lat_stick", 0.0),
        pedal=trim.pedal + offsets.get("pedal", 0.0))
    sim.pilot.reset(stepped)
    print()
    print("%s off the trim: coll %5.2f in  long %+5.2f  lat %+5.2f  ped %+5.2f"
          % (label, stepped.collective, stepped.long_stick, stepped.lat_stick,
             stepped.pedal))
    for frame in range(int(seconds / SIM_TIME_STEP_S)):
        sim.fly(SIM_TIME_STEP_S)
        if frame % every == 0 or frame == int(seconds / SIM_TIME_STEP_S) - 1:
            telemetry = sim.telemetry()
            state = sim.airframe.state
            print("  t %4.1f s | alt %5.2f ft up %+6.0f fpm | roll %+5.1f pitch"
                  " %+5.1f yaw %+6.1f %+6.1f deg/s | gs %4.1f m/s | %6.2f m out"
                  " | u %+5.2f v %+5.2f m/s"
                  % (sim.sim_time,
                     (telemetry.alt_agl - UH1_CG_HEIGHT_ON_GROUND) / FOOT,
                     telemetry.height_rate_fpm, telemetry.roll_deg,
                     telemetry.pitch_deg, telemetry.yaw_deg,
                     telemetry.yaw_rate_deg, telemetry.ground_speed,
                     math.hypot(state.position.x, state.position.y),
                     state.velocity.x, state.velocity.y))
    return sim



def main():
    print("travels, in: collective %.2f  long %.2f  lat %.2f  pedals %.2f"
          % (UH1_COLLECTIVE_TRAVEL_IN, UH1_LONG_STICK_TRAVEL_IN,
             UH1_LAT_STICK_TRAVEL_IN, UH1_PEDAL_TRAVEL_IN))
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.on_the_pad()
    print("hover trim: %s" % (sim.trim_controls,))
    print("  as axes: collective %.4f long %+0.4f lat %+0.4f ped %+0.4f"
          % sim.trim_controls.to_axes())
    print("  on the pad: %s" % (sim.controls,))
    print("  pad axes: %r" % (sim.pilot.axes(),))
    print("  key rates, axes/s: %r  (per frame %r)"
          % (sim.pilot.key_rates(),
             tuple(rate / 60.0 for rate in sim.pilot.key_rates())))
    for speed_kt, speed in ((30, 15.4), (47, 24.0), (60, 30.9)):
        probe = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
        controls, state = probe.trim_level_flight(speed, altitude=100.0)
        angles = state.attitude_deg
        print("level trim %3d kt: coll %5.2f in (%3.0f%%)  long %+5.2f in"
              "  lat %+5.2f in  ped %+5.2f in  pitch %+5.1f  roll %+5.1f"
              "  collective %.1f deg"
              % (speed_kt, controls.collective,
                 100.0 * controls.collective_fraction, controls.long_stick,
                 controls.lat_stick, controls.pedal, angles.y, angles.x,
                 probe.airframe.command_angles(controls).collective_pitch_deg))

    # The short mission's own numbers: one control stepped off the twenty foot
    # hover's trim at a time, with nothing held and no keys, so these are the
    # model's own gains and are what the pilot's loops are scaled to.
    print()
    print("the twenty foot hover, one control stepped off its trim at a time:")
    for label, offset in (("pedal +0.5 in", dict(pedal=0.5)),
                          ("pedal +1.0 in", dict(pedal=1.0)),
                          ("pedal +2.0 in", dict(pedal=2.0)),
                          ("pedal -1.0 in", dict(pedal=-1.0)),
                          ("long +0.2 in", dict(long_stick=0.2)),
                          ("long +0.5 in", dict(long_stick=0.5)),
                          ("long +1.0 in", dict(long_stick=1.0)),
                          ("lat +0.5 in", dict(lat_stick=0.5)),
                          ("collective +0.4 in", dict(collective=0.4))):
        hover_step(label, **offset)

    # And what the mission actually does with it: the trim held for a minute,
    # and the half inch of pedal a slow turn wants held for half of one - which
    # is the drift the machine does under the turn on its own.
    print()
    print("what the short mission's own two holds do:")
    hover_step("nothing at all, for a minute", seconds=60.0, every=600)
    hover_step("pedal +0.4 in, for half a minute", seconds=30.0, every=300,
               pedal=0.4)


if __name__ == "__main__":
    main()
