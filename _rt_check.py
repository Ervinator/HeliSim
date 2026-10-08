# -*- coding: utf-8 -*-
"""Scratch: fly the missions headless and fast, and print what happened.

The realtime demo's own rehearsal: the pilots through ``main.fly_frame`` - the
same keys, the same ratchet, the same clock, the same camera and the same view
as the window - with no renderer and no waiting, so the gains can be read off a
transcript before anybody watches them, and so that a mission's own claims can
be *asserted* rather than eyeballed.

Both missions are flown.  The short one - twenty feet up over the pad, once
round to the right about the vertical axis, and down again onto the skids - at
two of a person's reaction times, since it is a minute of flight and its shape
should be the same at both; and the long one as the regression that the frame
loop the two pilots share still flies what it flew.

The transcript is ``_rt_log.txt``, written by the run itself: the shell's own
redirection is not worth trusting on this box.
"""

import math
import sys

from _rt_pilot import (ALT_REF_M, ALT_TOLERANCE_M, AXIS_KEYS,
                       COLLECTIVE_TRAVEL_IN, DEFAULT_MISSION, DIST_REF_M,
                       HOVER_ALT_M, HOVER_SKID_FT, HOVER_TOLERANCE_M, KEY_STEP,
                       LAT_STICK_HALF_IN, LONG_STICK_HALF_IN, MISSIONS,
                       PEDAL_HALF_IN, REACTION_S, TURN_TOLERANCE_DEG,
                       trims_for, wrap)
from airframe import FOOT
from main import AIRFRAME_PRESET, KEY_NAMES, VIEW_COCKPIT, fly_frame
from simulation import (ChaseCamera, SIM_TIME_STEP_S, Simulation,
                        UH1_CG_HEIGHT_ON_GROUND, airframe_preset)

#: The transcript, written by the run itself: see the module docstring.
LOG_FILE = "_rt_log.txt"

#: How often a line is printed, frames: every second of the short mission, and
#: every five of the long one, which is a minute and a half of flying.
SHORT_EVERY = 60
LONG_EVERY = 300


class Run(object):
    """What one flight of a mission measured, beside the pilot and the sim.

    The pilot's own record is in the pilot - the phases, and when each began -
    and this is the frame loop's half of it: the altitude band the run stayed
    in, how far from the pad it ever drifted, the rate its skids arrived at, and
    how much of the run each of the four axes had a key held on it.
    """

    def __init__(self, sim, pilot):
        self.sim = sim
        self.pilot = pilot
        self.lowest = sim.airframe.state.altitude
        self.highest = self.lowest
        self.worst_drift = 0.0
        self.held = [0, 0, 0, 0]
        self.frames = 0
        self.on_ground = sim.on_ground
        self.last_airborne_rate = 0.0     # m/s, down positive, in the air
        self.touchdown_rate = None        # m/s, the rate the skids arrived at

    def measure(self, keys):
        """One frame's worth, from the state the frame loop has just left."""
        telemetry = self.sim.telemetry()
        state = self.sim.airframe.state
        self.frames += 1
        self.lowest = min(self.lowest, telemetry.alt_agl)
        self.highest = max(self.highest, telemetry.alt_agl)
        self.worst_drift = max(self.worst_drift,
                               math.hypot(state.position.x, state.position.y))
        for index, (up, down) in enumerate(AXIS_KEYS):
            if up in keys or down in keys:
                self.held[index] += 1
        if not self.sim.on_ground:
            # The skids have not arrived yet, so this is the last rate a frame
            # saw in the air - which is the rate they arrive at, since the frame
            # that puts them down is the one that zeroes it.
            self.last_airborne_rate = -telemetry.height_rate
        elif not self.on_ground and self.touchdown_rate is None:
            self.touchdown_rate = self.last_airborne_rate
        self.on_ground = self.sim.on_ground
        assert math.isfinite(state.altitude)
        assert math.isfinite(state.velocity.x)
        assert math.isfinite(state.velocity.y)
        return self

    @property
    def skid_ft(self):
        """The highest the *skids* got, ft: what twenty feet up is measured in."""
        return (self.highest - UH1_CG_HEIGHT_ON_GROUND) / FOOT

    def __str__(self):
        """The run's own summary: the band, the drift, the keys, the arrival."""
        return ("%d frames, %.1f m .. %.1f m of c.g. (%.1f ft of skid at the"
                " top), %.1f m out at the worst, touchdown %s, keys held:"
                " collective %4.1f%%  long %4.1f%%  lat %4.1f%%  pedals %4.1f%%"
                % (self.frames, self.lowest, self.highest, self.skid_ft,
                   self.worst_drift,
                   ("%.0f fpm" % (self.touchdown_rate / FOOT * 60.0)
                    if self.touchdown_rate is not None else "not yet"),
                   *[100.0 * count / max(self.frames, 1)
                     for count in self.held]))


def line(sim):
    """One line of a run: the instruments a watcher in the window would read."""
    telemetry = sim.telemetry()
    state = sim.airframe.state
    return ("t %5.1f s | alt %6.2f m %+6.0f fpm %5.2f ft up | %5.1f kt gs %4.1f"
            " m/s | roll %+5.1f pitch %+5.1f yaw %+6.1f %+6.0f deg/s | coll"
            " %5.2f in long %+5.2f lat %+5.2f ped %+5.2f | %6.2f m out%s%s"
            % (sim.sim_time, telemetry.alt_agl, telemetry.height_rate_fpm,
               (telemetry.alt_agl - UH1_CG_HEIGHT_ON_GROUND) / FOOT,
               telemetry.airspeed_kt, telemetry.ground_speed,
               telemetry.roll_deg, telemetry.pitch_deg, telemetry.yaw_deg,
               telemetry.yaw_rate_deg, telemetry.collective_in,
               telemetry.long_stick_in, telemetry.lat_stick_in,
               telemetry.pedal_in,
               math.hypot(state.position.x, state.position.y),
               " on the ground" if telemetry.on_ground else "",
               " %s" % telemetry.crash_message if telemetry.crashed else ""))


def wanted_inches(pilot):
    """What the pilot's hands are asking for, in the sticks' own inches.

    The four axes the last decision left, in ``PilotControls``' units, so that a
    transcript reads what the pilot asked for beside what the aircraft answered -
    which is the one thing a rehearsal of a control loop needs and the
    instruments cannot show: a pilot's own hand, in the units the loops are
    written in.
    """
    axes = pilot.want or (0.0, 0.0, 0.0, 0.0)
    return tuple(axis * travel for axis, travel
                 in zip(axes, (COLLECTIVE_TRAVEL_IN, LONG_STICK_HALF_IN,
                               LAT_STICK_HALF_IN, PEDAL_HALF_IN)))


def fly(name=DEFAULT_MISSION, reaction=REACTION_S, limit=180.0, every=None):
    """Fly *name* to the end of its mission or to *limit* seconds, and print it.

    The aircraft, the pad, the camera and the view are ``main.py``'s own and the
    keys are the pilot's, so this is the window's frame loop with the waiting
    taken out - and the phase line printed beside each frame is the pilot's own
    words, which are the ones a caption shows.
    """
    pilot_class = MISSIONS[name]
    trims = trims_for(airframe_preset(AIRFRAME_PRESET), pilot_class)
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.on_the_pad()
    camera = ChaseCamera()
    pilot = pilot_class(sim, trims, reaction=reaction)
    run = Run(sim, pilot)
    if every is None:
        every = SHORT_EVERY if name == DEFAULT_MISSION else LONG_EVERY
    trim = trims.at(0.0)

    print("mission %s at reaction %.2f s: %s"
          % (name, reaction, pilot_class.DESCRIPTION))
    print("  the pilot's own trim: collective %5.2f in (%4.1f %%), long %+5.2f,"
          " lat %+5.2f, pedal %+5.2f in | pitch %+5.2f deg"
          % (trim[0], 100.0 * trims.hover_fraction(), trim[1], trim[2], trim[3],
             trim[4]))
    print("  parked on the pad:    %s" % (line(sim),))
    sys.stdout.flush()

    for frame in range(int(limit / SIM_TIME_STEP_S)):
        keys = pilot.keys(sim, SIM_TIME_STEP_S)
        fly_frame(sim, camera, SIM_TIME_STEP_S, keys, VIEW_COCKPIT)
        run.measure(keys)
        if frame % every == 0:
            keys_held = ",".join(sorted(KEY_NAMES[k] for k in keys)) or "none"
            print("  %s | pilot: %s" % (line(sim), pilot.note(sim)))
            print("      the pilot's hands ask for: collective %5.2f in, long"
                  " %+5.2f, lat %+5.2f, pedal %+5.2f | keys %s"
                  % (wanted_inches(pilot) + (keys_held,)))
            sys.stdout.flush()
        if sim.crashed or pilot.done is not None:
            break

    print("  %s at %.1f s of physics: %s"
          % ("CRASHED" if sim.crashed else "ended", sim.sim_time,
             pilot.note(sim)))
    print("  %s" % (run,))
    sys.stdout.flush()
    return run


def check(name=DEFAULT_MISSION, reaction=REACTION_S, limit=180.0):
    """Fly one mission and hold it to its own claims.  Returns the :class:`Run`.

    Raises AssertionError on failure, so a run of this file that prints a verdict
    has *proved* it: the numbers are the run's own, and the claims are the ones
    the mission is for - it got there, it held it, it went the whole way round,
    and it came down onto the pad rather than merely arriving.
    """
    run = fly(name, reaction, limit)
    pilot, sim = run.pilot, run.sim
    telemetry = sim.telemetry()
    assert not sim.crashed, "the flight ended as %s" % (telemetry.crash_message,)
    assert not sim.dropped_steps, sim.dropped_steps
    if name == DEFAULT_MISSION:
        # Off the pad, up to twenty feet of skid, and held there.
        assert pilot.took_off is not None, "it never left the pad"
        assert pilot.hover_at is not None, "it never met the band of twenty feet"
        assert abs(run.skid_ft - HOVER_SKID_FT) <= HOVER_TOLERANCE_M / FOOT, \
            run.skid_ft
        assert run.highest - HOVER_ALT_M <= HOVER_TOLERANCE_M, run.highest
        # Round the whole way, to the right, and stopped on the heading it began
        # from: a 360 that does not stop is a 361, and one that stopped early is
        # a 340.  The ramp is 24 s of it at 15 deg/s, and the stopping a few
        # more; what the nose is within by the end of the run is the model's own
        # yaw drift, which is why fifteen degrees is what is asked of that.
        assert pilot.turn_done is not None, "the turn never finished"
        assert pilot.turned_deg == 360.0, pilot.turned_deg
        assert abs(pilot.turn_error_deg) <= TURN_TOLERANCE_DEG, \
            pilot.turn_error_deg
        turned_for = pilot.turn_done - pilot.turn_begun
        assert 24.0 <= turned_for <= 40.0, turned_for
        assert pilot.turned_deg / turned_for >= 10.0, \
            "the turn averaged less than ten degrees a second"
        assert abs(wrap(telemetry.yaw_deg - pilot.turn_heading_deg)) <= 15.0, \
            telemetry.yaw_deg
        # And down again: the skids, the pad's own lever, the trim's own sticks.
        assert sim.on_ground and pilot.phase == "landed", pilot.phase
        assert pilot.done is not None, "the mission never finished"
        assert run.touchdown_rate is not None and run.touchdown_rate < 1.0, \
            run.touchdown_rate
        assert abs(telemetry.height_rate) < 0.01, telemetry.height_rate
        assert run.worst_drift <= 10.0, run.worst_drift
        assert abs(telemetry.roll_deg) < 1.0, telemetry.roll_deg
        assert abs(telemetry.pitch_deg) < 1.0, telemetry.pitch_deg
        assert all(abs(axis - want) <= step
                   for axis, want, step
                   in zip(sim.pilot.axes(), pilot.pad_axes, KEY_STEP)), \
            sim.pilot.axes()
    else:
        # The long mission's own: up, out, and finished over the kilometre.
        state = sim.airframe.state
        distance = math.hypot(state.position.x, state.position.y)
        assert pilot.took_off is not None and pilot.climbed is not None
        assert abs(run.highest - ALT_REF_M) <= ALT_TOLERANCE_M, run.highest
        assert distance >= DIST_REF_M, distance
        assert pilot.done is not None, "the mission never finished"
    print("check passed: %s at reaction %.2f s, %s"
          % (name, reaction, pilot.note(sim)))
    sys.stdout.flush()
    return run


def main():
    """Both missions, and the transcript left behind by the run itself."""
    stream = open(LOG_FILE, "w", encoding="utf-8")
    sys.stdout = stream
    try:
        for reaction in (0.25, 0.35):
            check(DEFAULT_MISSION, reaction=reaction)
        check("cruise", reaction=REACTION_S, limit=250.0)
    finally:
        sys.stdout = sys.__stdout__
        stream.close()
    print("wrote %s" % (LOG_FILE,))


if __name__ == "__main__":
    main()


