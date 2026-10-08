# -*- coding: utf-8 -*-
"""Scratch: watch the keyboard pilot fly the mission, in main.py's own window.

Not part of the model and not part of ``main.py``: nothing in either imports
this file.  It is the same window ``python main.py`` opens - the same
``pygame.display.set_mode``, the same ``main.init_gl``, the same
``main.fly_frame``, ``main.draw_scene``, caption and control position panels -
with :class:`_rt_pilot.HumanPilot` or :class:`_rt_pilot.HoverTurnPilot` standing
at the keyboard instead of a person, so the pickup, a hover, a turn and a
landing can be *watched* at sixty frames a second rather than read off a
transcript afterwards.  Which of the two it is is the command line's first
argument; the short mission - twenty feet up over the pad, once round to the
right about the vertical axis, and down again onto the skids - is what is flown
when nobody says.

What the pilot sends is what the frame loop would have read from the events: a
scancode to 1.0 for each key it holds this frame, in ``main.frame_keys``' own
shape.  Everything downstream - ``main.fly_frame``, ``Simulation.fly``, the four
ratchets of ``simulation.PilotInput`` and the panels - cannot tell the
difference, which is the point: the panels in the window's corner move under the
pilot's hands exactly as they move under a watcher's.

Keys, on top of the pilot's own eight:

    C       camera: behind the aircraft, or fixed on the pad
    R       reset to the pad and hand the pilot the mission again
    P       park on the pad, the pilot still flying
    Esc     quit

Usage::

    python _rt_demo.py [mission] [scenery.xml] [reaction_seconds] [frames]

*mission* is a name out of ``_rt_pilot.MISSIONS`` - the short one by default, or
``cruise`` - and anything else in that place is taken as a scenery file, so the
command line this had before still means what it did.  The last argument is a
count of physics steps to stop after, for a look rather than a flight: ``900``
is fifteen seconds of it, which is how the driver is checked end to end -
display, GL, render, caption, frame loop - in one command.
"""

import sys

import pygame
from pygame.locals import DOUBLEBUF, KEYDOWN, OPENGL, QUIT

import main
from main import (AIRFRAME_PRESET, DEFAULT_SCENERY_FILE, HEIGHT, VIEW_COCKPIT,
                  VIEWS, WIDTH, draw_scene, fly_frame, load_scenery, resize,
                  window_title)
from simulation import ChaseCamera, Simulation, airframe_preset
from _rt_pilot import DEFAULT_MISSION, MISSIONS, REACTION_S, trims_for


#: The transcript of the windowed run, written by the run itself, flushed line by
#: line: the shell's own redirection is not worth trusting on this box, and a
#: window is a process a watcher can close before a buffered pipe ever drains.
LOG_FILE = "_rt_demo_log.txt"


class Transcript(object):
    """``sys.stdout`` for the run: the console, and a file that outlives it."""

    def __init__(self, path):
        self.stream = open(path, "w", encoding="utf-8")

    def write(self, text):
        self.stream.write(text)
        self.stream.flush()
        sys.__stdout__.write(text)
        sys.__stdout__.flush()

    def flush(self):
        self.stream.flush()
        sys.__stdout__.flush()

    def close(self):
        self.stream.close()


def pilot_note(pilot, sim):
    """The pilot's own line for the caption: what the mission is doing.

    The four control positions are already in the caption, in the model's own
    units; this is the other half a watcher wants, which is where the *pilot* is
    - parked, climbing, turning, coming down, or down and finished - and it is in
    the mission's own words, since each pilot says it for itself: see
    :meth:`_rt_pilot.KeyboardPilot.note`.
    """
    return " | pilot: %s" % (pilot.note(sim),)


def fly(scenery, mission=DEFAULT_MISSION, reaction=REACTION_S, frames=0):
    """One mission in a window: ``main.py``'s frame loop, a pilot on the keys.

    *mission* is a name out of ``_rt_pilot.MISSIONS``.  *frames* is a frame count
    to stop after, in place of a watcher pressing Esc: it is what lets the whole
    windowed driver - the display, the GL, the render, the caption, the frame
    loop - be run and looked at in one command, rather than only reasoned about.
    Zero means the window stays up.
    """
    main.SCENERY = scenery

    # The aircraft, and the pilot's trim table off a scratch aircraft of its own:
    # solving a trim moves an airframe's state, which is not something a pilot may
    # do to the one they are sitting in.
    pilot_class = MISSIONS[mission]
    trims = trims_for(airframe_preset(AIRFRAME_PRESET), pilot_class)
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.on_the_pad()
    pilot = pilot_class(sim, trims, reaction=reaction)
    print("aircraft: the report's %s configuration, %.3f s rotor"
          % (AIRFRAME_PRESET, sim.airframe.rotor_time_constant))
    trim = pilot.trims.at(0.0)
    print("  the pilot's own trim: collective %5.2f in (%4.1f %%), long %+5.2f,"
          " lat %+5.2f, pedal %+5.2f in | pitch %+5.2f deg"
          % (trim[0], 100.0 * pilot.trims.hover_fraction(), trim[1], trim[2],
             trim[3], trim[4]))
    print("  parked on the pad:    %s" % (sim.telemetry(),))
    print("mission %s at reaction %.2f s: %s"
          % (mission, reaction, pilot_class.DESCRIPTION))
    print("keys: C camera - the cockpit first, as ``main.py`` opens - then the"
          " chase and the pad; R resets and hands the pilot the mission again;"
          " P parks; Esc quits")

    camera = ChaseCamera()
    view = VIEW_COCKPIT        # main.py's own default, so a watcher sees what
    #                            the pilot sees, and C cycles on from here

    pygame.init()
    pygame.display.set_mode((WIDTH, HEIGHT), DOUBLEBUF | OPENGL)
    resize(WIDTH, HEIGHT)
    main.init_gl()
    pygame.display.set_caption(window_title(sim, view) + pilot_note(pilot, sim))

    clock = pygame.time.Clock()
    running = True
    announced = False
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_c:
                    view = VIEWS[(VIEWS.index(view) + 1) % len(VIEWS)]
                    camera.reset()
                elif event.key in (pygame.K_r, pygame.K_p):
                    # The sandbox's own two keys, and one more thing: R hands the
                    # pilot the mission again from the pad, because a new flight is
                    # a new flight and the loops carry no memory of the last one
                    # beyond the aircraft they read each frame.
                    if event.key == pygame.K_r:
                        sim.reset()
                        pilot = pilot_class(sim, trims, reaction=reaction)
                        announced = False
                        print("  reset, pilot back on the mission: %s"
                              % (sim.telemetry(),))
                    else:
                        sim.on_the_pad()
                        print("  parked on the pad: %s" % (sim.telemetry(),))
                    camera.reset()

        # The frame's own time, whatever it turned out to be - the physics inside
        # every step is still 1/60 s, so a dragged window is a longer frame and
        # never a faster aircraft - and the pilot reads the aircraft on their own
        # clock rather than on the frame's.
        frame_dt = clock.tick(60) / 1000.0
        fly_frame(sim, camera, frame_dt, pilot.keys(sim, frame_dt), view)
        draw_scene(sim, camera, view)
        pygame.display.flip()

        if pilot.done is not None and not announced:
            announced = True
            print("  mission complete at %.1f s: %s"
                  % (pilot.done, sim.telemetry()))

        if frames and sim.frames >= frames:
            print("  stopping after %d physics steps, asked for" % (sim.frames,))
            running = False

        if sim.frames % 15 == 0:
            pygame.display.set_caption(window_title(sim, view)
                                       + pilot_note(pilot, sim))
        if sim.frames % 60 == 0:
            print("  " + str(sim.telemetry()) + pilot_note(pilot, sim))

    pygame.quit()
    print("flown %.1f s of physics, %d decisions, %s"
          % (sim.sim_time, pilot.decisions, pilot_note(pilot, sim)))
    return 0


def demo():
    """The command line: a mission, a scenery file, a reaction time, the window.

    The mission is a name out of ``_rt_pilot.MISSIONS``, and anything that is not
    one is taken as the scenery file, so the command line this had before - a
    scenery file first - still means what it did.  The run's own transcript is
    ``_rt_demo_log.txt``, so a window that a watcher closes - or a command line
    that a later command closes - still leaves the flight's own account of itself
    behind.
    """
    args = sys.argv[1:]
    mission = DEFAULT_MISSION
    if args and args[0] in MISSIONS:
        mission = args.pop(0)
    path = DEFAULT_SCENERY_FILE
    if args and not args[0].replace(".", "", 1).isdigit():
        path = args.pop(0)
    reaction = float(args[0]) if args else REACTION_S
    frames = int(args[1]) if len(args) > 1 else 0
    scenery = load_scenery(path)
    if scenery is None:
        return 2
    print("scenery: %d object(s) loaded from %s" % (len(scenery), path))
    transcript = Transcript(LOG_FILE)
    sys.stdout = transcript
    try:
        return fly(scenery, mission, reaction, frames)
    finally:
        sys.stdout = sys.__stdout__
        transcript.close()
        print("wrote %s" % (LOG_FILE,))


if __name__ == "__main__":
    sys.exit(demo())
