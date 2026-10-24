# -*- coding: utf-8 -*-
"""Scratch: Q and E through the window's own path, with no window.

The frame loop in ``main()`` does not read the keyboard itself: it hands
``Devices.read`` the scancodes the events left down and the ones that were
*tapped* inside the frame, and flies the frame with ``fly_frame_axes``.  This is
that loop headless - the same map, the same two dictionaries, the same two calls
- so that what the twist grip does when a key is held can be watched without a
window to hold it in.  Run from the project root:

    python attic/_qe_wire.py

It asserts both halves of the wiring: that Q and E move the grip the panel's pie
is drawn from, and that the grip and ``pilot_keys``' own fifth position are the
same number - which is what makes the window and a scripted run fly alike.

It also flies the loop's own reading of the grip across frames, which is what the
one-time ``grip_notice`` line is decided by: the grip before the frame's input
and after it, and a flag for whether the run has already spoken.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pygame

import main
from simulation import ChaseCamera, Simulation, airframe_preset

#: One frame of the loop, s: the window's own clock, so a key's travel here is a
#: key's travel there.
FRAME_DT = main.SIM_TIME_STEP_S

#: A key held for a second: ``PilotInput.throttle_rate`` is one full travel a
#: second and a key is worth a quarter of it, so a second of E or Q is a quarter
#: of the grip's travel.
HELD_FRAMES = int(round(1.0 / FRAME_DT))


def new_flight():
    """An aircraft on the pad, and the map this project ships, as a run begins."""
    return Simulation(airframe=airframe_preset(main.AIRFRAME_PRESET))


def fly(sim, camera, board, keyboard, tapped, frames):
    """*frames* frames of the window's loop: the keys in, the frame flown."""
    for _ in range(frames):
        absolute, ratchet = board.read(keyboard, tapped)
        main.fly_frame_axes(sim, camera, FRAME_DT, absolute, ratchet)
    return sim


def check_the_window_path():
    """Q and E, held and tapped, through the map the window flies with."""
    board = main.Devices(main.controls.default_control_map())
    sim = new_flight()
    camera = ChaseCamera()
    sim.on_the_pad()

    # The grip begins full open - the governor's own position, where a run has
    # always begun - so the pie's hand begins on its upper stop.  E has nothing
    # to do there, which is why E looked like a dead key in the window: it was
    # one, and opening the grip is not what E is for.
    low, split, high = main.throttle_pie_spans()
    assert sim.pilot.throttle() == 1.0
    assert main.throttle_hand_deg(sim.throttle()) == high
    fly(sim, camera, board, {}, set(), HELD_FRAMES)
    assert sim.pilot.throttle() == 1.0            # nothing held moves nothing
    fly(sim, camera, board, {pygame.KSCAN_E: True}, set(), HELD_FRAMES)
    assert sim.pilot.throttle() == 1.0            # open already: on its stop
    print("  the grip begins full open, at %.1f deg" % (high,))

    # Q rolls it off, a key's quarter of its travel a second, and the hand the
    # panel's pie is drawn from follows the grip down its arc.
    fly(sim, camera, board, {pygame.KSCAN_Q: True}, set(), HELD_FRAMES)
    assert abs(sim.pilot.throttle() - 0.75) < 1e-9
    quarter = main.throttle_hand_deg(sim.throttle())
    assert low < quarter < high
    print("  a second of Q: grip %.2f, hand %.1f deg"
          % (sim.throttle(), quarter))

    # And E rolls it back open, to the stop it came from.
    fly(sim, camera, board, {pygame.KSCAN_E: True}, set(), HELD_FRAMES * 4)
    assert sim.pilot.throttle() == 1.0
    assert main.throttle_hand_deg(sim.throttle()) == high
    # The grip is a ratchet, so Q rolls it off to the closed stop and no further.
    fly(sim, camera, board, {pygame.KSCAN_Q: True}, set(), HELD_FRAMES * 8)
    assert sim.pilot.throttle() == 0.0
    assert main.throttle_hand_deg(sim.throttle()) == low
    # The two keys cancel, which is what a hand on both of them does.
    fly(sim, camera, board, {pygame.KSCAN_Q: True, pygame.KSCAN_E: True},
        set(), HELD_FRAMES)
    assert sim.pilot.throttle() == 0.0
    print("  Q to the closed stop, E to the open one, both at once: still")

    # A *tap* is a key that went down and up inside one frame, which the loop
    # counts for that frame through the tapped set: without that the grip would
    # lose a keypress exactly as the collective used to.
    before = sim.pilot.throttle()
    absolute, ratchet = board.read({}, {pygame.KSCAN_E})
    assert ratchet["throttle"] == 1.0
    main.fly_frame_axes(sim, camera, FRAME_DT, absolute, ratchet)
    tap = sim.pilot.throttle() - before
    assert abs(tap - 1.0 / (4.0 * HELD_FRAMES)) < 1e-12
    print("  a tap of E: one frame of a key, %.4f of the grip" % (tap,))

    # The grip and pilot_keys' own fifth position are one number: the window and
    # fly_frame, flown side by side on two aircraft that began identically, hold
    # the same grip at the end of the same keys.
    by_keys, by_map = new_flight(), new_flight()
    keys_camera, map_camera = ChaseCamera(), ChaseCamera()
    held = {pygame.KSCAN_E: True, pygame.KSCAN_Q: True, pygame.KSCAN_W: True}
    for _ in range(HELD_FRAMES):
        main.fly_frame(by_keys, keys_camera, FRAME_DT,
                       main.frame_keys(held, set()))
        absolute, ratchet = board.read(held, set())
        main.fly_frame_axes(by_map, map_camera, FRAME_DT, absolute, ratchet)
    assert by_keys.pilot.throttle() == by_map.pilot.throttle() == 1.0
    assert by_keys.pilot.axes() == by_map.pilot.axes()
    # ...and Q alone rolls both of them off by the same key's worth.
    gripped = main.pilot_keys({pygame.KSCAN_Q: True})["throttle"]
    assert board.read({}, {pygame.KSCAN_Q})[1]["throttle"] == gripped
    print("  the window's path and pilot_keys' own: one grip")


def check_the_engine():
    """The fitted engine: the grip the window rolls is the governor's own."""
    board = main.Devices(main.controls.default_control_map())
    sim = new_flight()
    camera = ChaseCamera()
    sim.on_the_pad()
    sim.fit_engine()
    assert sim.airframe.engine is not None
    fly(sim, camera, board, {pygame.KSCAN_Q: True}, set(), HELD_FRAMES * 2)
    assert abs(sim.throttle() - 0.5) < 1e-9
    # Q rolled the grip off through the window's own loop, and the governor the
    # engine is flying on is at that grip: this is what a keyboard has always
    # done through pilot_keys, and it is what the map does now too.
    assert sim.airframe.throttle == sim.throttle()
    print("  with an engine fitted: governor at %.2f, from Q" % (sim.throttle(),))


def check_the_notice():
    """The grip's own warning: once a run, and only when there is no engine.

    This is the window's loop over again, line for line: the grip read off the
    aircraft before the frame's own input and read again after it, and the one
    line the run owes printed the first time the answer is not None.  What it
    checks is that the default run *does* owe the line the moment the grip is
    touched, that a held key owes it once rather than once a frame, that a grip
    no hand has touched owes nothing at all, and that a run which fitted the T53
    at startup - ``--engine`` - owes nothing whatever it does with the grip.
    """
    board = main.Devices(main.controls.default_control_map())
    sim = new_flight()
    camera = ChaseCamera()
    sim.on_the_pad()
    said = []
    grip_noticed = False

    def frame(keyboard, tapped=set()):
        """One frame of the loop, warning and all."""
        nonlocal grip_noticed
        absolute, ratchet = board.read(keyboard, tapped)
        grip_was = sim.throttle()
        main.fly_frame_axes(sim, camera, FRAME_DT, absolute, ratchet)
        if not grip_noticed:
            notice = main.grip_notice(sim, grip_was, sim.throttle())
            grip_noticed = notice is not None
            if notice is not None:
                said.append(notice)

    # Hands off, the grip at its full open stop: a grip that never moved has not
    # been rolled off, so there is nothing to warn about and this run is silent.
    for _ in range(HELD_FRAMES):
        frame({})
    assert sim.throttle() == 1.0 and said == []
    print("  hands off: %d lines, the grip still at %.2f"
          % (len(said), sim.throttle()))

    # Q now, held for a second and left held for four more: one line, the first
    # frame it moves on, and not one a frame for as long as the key is down.
    for _ in range(HELD_FRAMES):
        frame({pygame.KSCAN_Q: True})
    assert len(said) == 1, said
    assert "F" in said[0] and "--engine" in said[0], said[0]
    for _ in range(HELD_FRAMES * 4):
        frame({pygame.KSCAN_Q: True})
    assert len(said) == 1 and sim.throttle() == 0.0
    print("  Q, held five seconds: %d line, and the grip at %.2f"
          % (len(said), sim.throttle()))
    print("  the line itself:")
    print(said[0])

    # And the aircraft the flag fits, which is the whole of what --engine buys: a
    # fresh run with the T53 on before the first frame says nothing at all, since
    # the grip now has a governor on the other end of it to be the minimum of.
    fitted = new_flight()
    fitted_camera = ChaseCamera()
    fitted.on_the_pad()
    fitted.fit_engine()
    quiet = []
    for _ in range(HELD_FRAMES):
        absolute, ratchet = board.read({pygame.KSCAN_Q: True}, set())
        grip_was = fitted.throttle()
        main.fly_frame_axes(fitted, fitted_camera, FRAME_DT, absolute, ratchet)
        notice = main.grip_notice(fitted, grip_was, fitted.throttle())
        if notice is not None:
            quiet.append(notice)
    assert abs(fitted.throttle() - 0.75) < 1e-9 and quiet == []
    print("  --engine, Q for a second: %d lines, governor at %.2f"
          % (len(quiet), fitted.throttle()))


def run():
    print("attic/_qe_wire.py - Q and E through the window's own path")
    check_the_window_path()
    check_the_engine()
    check_the_notice()
    print("qe wiring check passed")


if __name__ == "__main__":
    run()