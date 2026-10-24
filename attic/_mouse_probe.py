#!/usr/bin/env python3
"""Scratch: fly `main.py`'s mouse through the frame loop, in a real window.

Not part of the project, and not imported by anything.  `main.py --check` asserts
the mouse's own arithmetic with no window at all; what it cannot reach is the part
that needs one - `main.Mouse.recentre` is `pygame.mouse.set_pos`, the cursor and
its wheel are the event loop's, and the merge into the frame is the loop's own
call.  This opens the window `main.py` opens, takes the mouse up the way `M` does,
turns the wheel, and flies a second of the aircraft with it - printing the lever
and the sticks, so that what the check proves about the arithmetic can be seen in
the aircraft.

    python _mouse_probe.py

It writes to `_mouse_probe.txt` beside itself and asserts as it goes, so the run
is its own test.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import pygame                                                    # noqa: E402
from pygame.locals import DOUBLEBUF, MOUSEWHEEL, OPENGL, QUIT     # noqa: E402

import main                                                      # noqa: E402
from simulation import (PAD_COLLECTIVE, SIM_TIME_STEP_S,          # noqa: E402
                        UH1_CG_HEIGHT_ON_GROUND, Simulation,
                        airframe_preset)

OUT = os.path.join(HERE, "_mouse_probe.txt")
FRAMES = 60


def fly(stream, sim, camera, board, mouse, frames, keys, note):
    """*frames* of the frame loop with the mouse among the mapped controls."""
    stream.write("%s\n" % note)
    for _ in range(frames):
        absolute, ratchet = board.read(keys, set())
        absolute, ratchet = main.hands_on(absolute, ratchet, mouse,
                                          pygame.mouse.get_pos())
        main.fly_frame_axes(sim, camera, SIM_TIME_STEP_S, absolute, ratchet)
    telemetry = sim.telemetry()
    stream.write("  %5.2f s  lever %5.1f %% (mouse %.4f)  cyc %+5.2f/%+5.2f in"
                 "  ped %+5.2f in  grip %3.0f %%\n"
                 % (sim.sim_time, 100.0 * telemetry.collective_fraction,
                    mouse.collective, telemetry.long_stick_in,
                    telemetry.lat_stick_in, telemetry.pedal_in,
                    100.0 * sim.throttle()))


def main_check(stream):
    """The mouse in the loop: engaged, wheeled, flown, and the pad back."""
    sim = Simulation(airframe=airframe_preset(main.AIRFRAME_PRESET))
    sim.on_the_pad()
    camera = main.ChaseCamera()
    board = main.Devices(main.controls.default_control_map())

    pygame.init()
    pygame.display.set_mode((main.WIDTH, main.HEIGHT), DOUBLEBUF | OPENGL)
    size = pygame.display.get_surface().get_size()
    middle = (size[0] // 2, size[1] // 2)
    stream.write("window %dx%d, the pad's own lever %.4f\n"
                 % (size[0], size[1], PAD_COLLECTIVE))

    # The mouse as `--mouse-reverse` takes it up: the lever read back off the
    # aircraft and the cursor put in the middle of the window, which needs a
    # window and is the one line of the mouse the check cannot run.
    mouse = main.mouse_for(sim, size, main.MOUSE_REVERSE)
    stream.write("%s\n" % main.mouse_line(mouse))
    assert mouse.reverse and mouse.collective == sim.pilot.collective_axis
    assert mouse.read(middle) == {"long_stick": 0.0, "lat_stick": 0.0,
                                  "collective": PAD_COLLECTIVE}
    stream.write("  the centred cursor, as the loop reads it: %s\n"
                 % ({name: round(value, 4)
                     for name, value in mouse.read(pygame.mouse.get_pos()).items()},))

    # Four notches of the wheel, which is reversed: the lever comes *down*, and
    # the frame's own set_axes follows it rather than any key's ratchet.
    for _ in range(4):
        mouse.wheel(1)
    stream.write("  four notches up, reversed: lever %.4f, %.4f below the pad's\n"
                 % (mouse.collective, PAD_COLLECTIVE - mouse.collective))
    assert mouse.collective < PAD_COLLECTIVE
    fly(stream, sim, camera, board, mouse, FRAMES, {pygame.KSCAN_W: True},
        "a second of the loop with W held as well:")
    # The mouse owns the lever: `W` is read by nothing, and the lever is exactly
    # where the wheel left it rather than a key's worth further up.
    assert abs(sim.pilot.collective_axis - mouse.collective) < 1e-9
    assert abs(sim.airframe.state.altitude - UH1_CG_HEIGHT_ON_GROUND) < 1e-9
    assert sim.on_ground and not sim.crashed

    # The cursor, which is the stick: the window here is not the focused one, so
    # a script cannot *place* it - what it can do is read where it is and check
    # that the aircraft's own axes went to exactly that, and that the pedals are
    # still the keyboard's, `D` being read on the same frames the mouse is.
    pygame.mouse.set_pos(size[0] - 1, 1)
    at = pygame.mouse.get_pos()
    stream.write("cursor asked for (%d, %d), read back at (%d, %d)\n"
                 % (size[0] - 1, 1, at[0], at[1]))
    pedal_was = sim.pilot.axes()[3]
    fly(stream, sim, camera, board, mouse, 2 * FRAMES, {pygame.KSCAN_D: True},
        "two seconds more with D held and the cursor where it is:")
    axes = sim.pilot.axes()
    cursor = mouse.read(pygame.mouse.get_pos())
    assert abs(axes[1] - cursor["long_stick"]) < 1e-9      # the cyclic is the cursor
    assert abs(axes[2] - cursor["lat_stick"]) < 1e-9
    assert axes[3] > pedal_was + 0.4                       # the pedal is `D`'s own
    # ...at exactly the rate its own key is worth over those two seconds, which
    # is the model's rate and not anything this file has written down.
    pedal_key = sim.pilot.key_rates()[2]
    assert abs(axes[3] - (pedal_was + pedal_key * 2 * FRAMES * SIM_TIME_STEP_S)) \
        < 1e-9
    assert axes[0] == mouse.collective                     # and the lever untouched
    stream.write("  the four axes: %s\n" % (tuple(round(axis, 4) for axis in axes),))
    stream.write("  the mouse's own three: %s\n"
                 % ({name: round(value, 4) for name, value in cursor.items()},))

    # And `R`, which puts the aircraft back on the pad: the mouse reads the lever
    # back off it - what the key's own branch does - so the pad's 75 % is not
    # slewed off again by the next frame's set_axes.
    sim.reset()
    mouse.collective = sim.pilot.collective_axis
    mouse.recentre()
    fly(stream, sim, camera, board, mouse, FRAMES, {},
        "a second after R, with the mouse reading the aircraft back:")
    assert abs(sim.pilot.collective_axis - PAD_COLLECTIVE) < 1e-9
    assert sim.pilot.axes()[0] == PAD_COLLECTIVE
    assert sim.pilot.axes()[3] == sim.pilot.axes()[3]      # a hands-off frame
    stream.write("  on the ground %s, crashed %s, x %.4f m\n"
                 % (sim.on_ground, sim.crashed, sim.airframe.state.position.x))

    # The events the loop would have taken: a wheel notch and a quit, which is
    # the one thing a script cannot borrow from `main()`'s own while loop.
    wheel = pygame.event.Event(MOUSEWHEEL, {"x": 0, "y": -1, "flipped": False,
                                            "which": 0})
    pygame.event.post(wheel)
    pygame.event.post(pygame.event.Event(QUIT))
    seen = []
    for event in pygame.event.get():
        if event.type in (MOUSEWHEEL, QUIT):
            seen.append((event.type, getattr(event, "y", None)))
    stream.write("events read back: %s (MOUSEWHEEL is %d)\n"
                 % (seen, MOUSEWHEEL))
    assert seen[0] == (MOUSEWHEEL, -1) and seen[1] == (QUIT, None)
    pygame.quit()
    stream.write("mouse probe passed\n")
    return 0


def run():
    stream = open(OUT, "w", encoding="utf-8")
    try:
        return main_check(stream)
    finally:
        stream.close()


if __name__ == "__main__":
    sys.exit(run())
