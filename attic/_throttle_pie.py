# -*- coding: utf-8 -*-
"""Scratch: read the collective panel's throttle pie back out of a real context.

Not part of the project, and not imported by anything.  ``main.py --check``
asserts the panel's arithmetic headless; this asserts the *drawing* of it, which
needs a context, by drawing the panel into a hidden window and reading the pixels
back with ``glReadPixels``.  Run from the project root:

    python attic/_throttle_pie.py

The pie is drawn exactly as ``main.draw_collective_panel`` draws it - the same
pixel projection, the same blend state, the same functions - so what is read back
is what the window shows, and the assertions are the picture: the yellow hundred
degrees below the panel's horizon, the green hundred and five above it, the 155
degrees left open facing the panel's own left edge, the red hand on the grip's
own angle with its tail through the middle of the pie and out of the far side,
and the pie and the lever's field sharing the panel without touching.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import DOUBLEBUF, HIDDEN, OPENGL

import main
from simulation import ChaseCamera, Simulation, airframe_preset

#: The readback: how close a colour has to be to a panel's own to count as that
#: panel's colour, and how far out along a radius the probes go, as a fraction of
#: THROTTLE_PIE_RADIUS - well inside the arc, so a probe is a point of a colour
#: rather than the edge between two of them.  The hand is thick where the colours
#: are thin, and it is a line through the middle of the pie rather than a needle:
#: the tail is the far half of it.  A probe this far off that whole line is off
#: the hand - 0.7 * 0.15 * sin(30 deg) is 0.05 of the panel, half again the
#: hand's own 0.03 - and the probe below asserts it rather than trusting to it.
COLOUR_TOLERANCE = 0.06
PROBE = 0.70
HAND_CLEAR_DEG = 30.0

#: What the panel's own black comes to over the harness's clear colour, which is
#: what a probe outside the pie and the field should read: nothing above this.
PANEL_DARK = 0.10


def colour_at(point, half=0):
    """The mean colour of a small block of pixels, three floats 0 to 1."""
    size = 2 * half + 1
    x, y = point
    data = glReadPixels(int(round(x)) - half, int(round(y)) - half, size, size,
                        GL_RGB, GL_UNSIGNED_BYTE)
    values = [byte / 255.0 for byte in data]
    return tuple(sum(values[channel::3]) / float(size * size)
                 for channel in range(3))


def is_colour(actual, want, tolerance=COLOUR_TOLERANCE):
    """Is *actual* the colour *want*, to within the readback's own tolerance?"""
    return all(abs(a - w) <= tolerance for a, w in zip(actual, want))


def at(box, point):
    """A point of the panel in panel fractions as window pixels."""
    return main.panel_point(box, point[0], point[1])


def pie_point(deg, fraction=PROBE):
    """A point of the pie's own ray at *deg*, panel fractions."""
    angle = math.radians(deg)
    return (main.THROTTLE_PIE_CENTRE_X
            + fraction * main.THROTTLE_PIE_RADIUS * math.cos(angle),
            main.THROTTLE_PIE_CENTRE_Y
            + fraction * main.THROTTLE_PIE_RADIUS * math.sin(angle))


def hand_point(throttle, fraction):
    """A point on the hand's own line, *fraction* of the radius from the middle.

    Negative is behind the middle, which is where the hand's tail is.
    """
    return pie_point(main.throttle_hand_deg(throttle), fraction)


def field_point(deg, fraction):
    """A point on one of the field's own rays, panel fractions from its pivot."""
    angle = math.radians(deg)
    return (main.QUADRANT_PIVOT_X
            + fraction * main.QUADRANT_RADIUS * math.cos(angle),
            main.QUADRANT_PIVOT_Y
            + fraction * main.QUADRANT_RADIUS * math.sin(angle))


def is_open(point, box):
    """Is nothing of the pie or the field drawn at *point* - the panel's black?"""
    return max(colour_at(at(box, point))) < PANEL_DARK


def dump_panel(box, label, cells=26):
    """Print the panel as characters, for a human: one cell to a few pixels.

    A picture in the terminal rather than in a file, because the thing being
    checked is an angle: ``Y`` is the pie's yellow, ``G`` its green, ``R`` the
    hand, ``.`` anything else - the panel's black, or the lever's own field.
    """
    rows = []
    for row in range(cells):
        line = []
        for column in range(cells):
            point = ((column + 0.5) / cells, 1.0 - (row + 0.5) / cells)
            actual = colour_at(at(box, point))
            if is_colour(actual, main.PANEL_RED):
                line.append("R")
            elif is_colour(actual, main.PANEL_YELLOW):
                line.append("Y")
            elif is_colour(actual, main.PANEL_GREEN):
                line.append("G")
            else:
                line.append(".")
        rows.append("".join(line))
    print("  %s:" % (label,))
    for line in rows:
        print("    %s" % (line,))


def is_bare(point, box):
    """Has *point* none of the pie's own colours on it, whatever is behind it?

    The panel is translucent, so over the world its own black is the world
    showing through it and not a colour at all: in the frame below the question
    is only whether a colour of the pie's is there.
    """
    actual = colour_at(at(box, point))
    return not any(is_colour(actual, colour)
                   for colour in (main.PANEL_YELLOW, main.PANEL_GREEN,
                                  main.PANEL_RED))


def is_clear(deg, throttle):
    """Is *deg* far enough off the hand's own line, with the grip at *throttle*?

    The line rather than the pointer: the hand runs through the middle of the pie
    and out of the far side of it, so its tail covers the angle half a turn round
    from the hand's own, and a probe just there reads red like the head does.
    """
    difference = abs(deg - main.throttle_hand_deg(throttle)) % 180.0
    return min(difference, 180.0 - difference) >= HAND_CLEAR_DEG


def clear_between(low_deg, high_deg, throttle):
    """The angle in ``(low_deg, high_deg)`` furthest from the hand's own line.

    ``(deg, gap)``: a scratch harness cannot know what the grip will be at when
    the keys have had their way with it, so the probes of the frame below are
    aimed at the two colours rather than picked by hand - and the gap says
    whether the aim was good enough to read a colour beside the hand.
    """
    best, best_gap = low_deg, -1.0
    for step in range(int(round(high_deg - low_deg)) + 1):
        deg = low_deg + step
        difference = abs(deg - main.throttle_hand_deg(throttle)) % 180.0
        gap = min(difference, 180.0 - difference)
        if gap > best_gap:
            best, best_gap = deg, gap
    return best, best_gap


def probe(box, deg, throttle, want, what):
    """Assert what the pie shows at *deg*, with the grip at *throttle*.

    ``want`` is the colour to find there, or ``None`` for the panel's own black,
    which is what the 155 degrees the grip has no colour for are.
    """
    assert is_clear(deg, throttle), \
        "the probe at %.1f deg, with the grip at %.2f, is under the hand" % (
            deg, throttle)
    actual = colour_at(at(box, pie_point(deg)))
    if want is None:
        assert max(actual) < PANEL_DARK, "%s, at %.1f deg" % (what, deg)
    else:
        assert is_colour(actual, want), "%s, at %.1f deg" % (what, deg)


def draw_panel(box, collective_axis, throttle):
    """One frame of the panel alone, in ``draw_control_panel``'s own GL state."""
    glClearColor(0.02, 0.02, 0.02, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0.0, main.WIDTH, 0.0, main.HEIGHT)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)
    glDisable(GL_FOG)
    glDisable(GL_CULL_FACE)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    main.draw_panel_background(box)
    main.draw_collective_panel(box, collective_axis, throttle)
    glDisable(GL_BLEND)
    glEnable(GL_CULL_FACE)
    glEnable(GL_FOG)
    glEnable(GL_DEPTH_TEST)
    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glFinish()


def check_the_pie(box):
    """The pie's own two colours, its open wedge and its hand, as pixels."""
    low, split, high = main.throttle_pie_spans()
    yellow, green, red = (main.PANEL_YELLOW, main.PANEL_GREEN, main.PANEL_RED)

    # Closed: the hand lies on the bottom edge of the pie, the yellow is the whole
    # hundred degrees below the panel's horizon and the green the hundred and five
    # above it, and the 155 degrees between the two ends - the wedge facing the
    # panel's own left edge - is the panel's black.  The probes are kept clear of
    # the hand's own line, which pointed down the pie's left is across the middle
    # of the wedge rather than beside it.
    draw_panel(box, 0.0, 0.0)
    assert is_colour(colour_at(at(box, hand_point(0.0, PROBE))), red), \
        "the hand, closed, on the bottom of the pie"
    assert is_colour(colour_at(at(box, hand_point(0.0, -0.3))), red), \
        "and its tail through the middle of the pie"
    for deg in (low + 75.0, low + 55.0, low + 35.0):
        probe(box, deg, 0.0, yellow, "the yellow, below the horizon")
    for deg in (10.0, 25.0, 40.0):
        probe(box, deg, 0.0, green, "the green, above it")
    for deg in (130.0, 160.0, 190.0, 215.0):
        probe(box, deg, 0.0, None, "the open wedge")
    # The horizon is the boundary and not a gap: five degrees either side of it
    # are the two colours, and nothing else is drawn between them.
    probe(box, split - 5.0, 0.0, yellow, "the yellow at the horizon's own foot")
    probe(box, split + 5.0, 0.0, green, "the green just over it")

    # Full open: the hand is on the top edge of the green, and it is a hand and
    # not an arm - a probe just past the arc is the panel and not the red.
    draw_panel(box, 0.0, 1.0)
    assert is_colour(colour_at(at(box, hand_point(1.0, PROBE))), red), \
        "the hand, full open, on the top of the pie"
    assert is_colour(colour_at(at(box, hand_point(1.0, -0.3))), red)
    probe(box, -40.0, 1.0, yellow, "the yellow, with the grip full open")
    probe(box, 40.0, 1.0, green, "the green, with the grip full open")
    for throttle in (0.0, 1.0):
        assert is_open(hand_point(throttle, 1.15), box), \
            "no hand past the arc, with the grip at %.1f" % (throttle,)
    print("  the pie alone, at two grips: %.1f px of radius, %.0f deg of travel"
          % (main.THROTTLE_PIE_RADIUS * main.PANEL_PX, high - low))

    # The two colours' own boundary: the grip wound to the top of the yellow half
    # puts the hand on the panel's horizon, with the yellow under it and the green
    # over it - the one angle the pie's two colours meet at, and the one the two
    # halves of the grip's travel meet at with it.
    boundary = main.THROTTLE_PIE_YELLOW_DEG / main.THROTTLE_PIE_SPAN_DEG
    draw_panel(box, 0.0, boundary)
    assert abs(main.throttle_hand_deg(boundary) - split) < 1e-12
    assert is_colour(colour_at(at(box, hand_point(boundary, PROBE))), red)
    assert is_colour(colour_at(at(box, hand_point(boundary, -0.3))), red)
    probe(box, -50.0, boundary, yellow, "the yellow, at the two colours' boundary")
    probe(box, 50.0, boundary, green, "the green, at the two colours' boundary")
    print("  the two colours' boundary, at the grip's own %.3f, on the horizon"
          % (boundary,))

    # And the two of them on the one panel: the lever in its field and the pie
    # above it, with the panel's own black between them - the pie is in the one
    # corner of the panel the lever's field does not reach.
    draw_panel(box, 1.0, 1.0)
    mid_field = 0.5 * (main.QUADRANT_LOW_DEG + main.QUADRANT_HIGH_DEG)
    assert is_colour(colour_at(at(box, field_point(mid_field, 0.5))), green), \
        "the field, under the lever"
    assert is_colour(colour_at(at(box, field_point(
        main.QUADRANT_HIGH_DEG, 0.6))), red), "the lever, on the up stop"
    assert is_colour(colour_at(at(box, hand_point(1.0, PROBE))), red), \
        "the grip, full open, at the same time"
    assert is_open((0.18, 0.45), box), "the panel's black between the two"
    # The field's arc and the pie's are two circles of different middles, and
    # neither is drawn over the other: the pie is whole with the lever on its stop.
    probe(box, 45.0, 1.0, green, "the pie's green, with the lever up")
    probe(box, -45.0, 1.0, yellow, "the pie's yellow, with the lever up")
    print("  the lever and the grip on one panel, and the black between them")


def check_the_frame(sim):
    """The panel in the frame it is drawn in, over the world, from a real sim."""
    camera = ChaseCamera()
    sim.on_the_pad()
    box = main.panel_box(main.PANEL_COLLECTIVE)
    # The keyboard's own grip: E winds it open at the grip's own rate - its whole
    # travel a second, where one *press* is worth a quarter of it - and what the
    # panel shows follows the hand rather than the key.
    for _ in range(int(round(1.0 / main.SIM_TIME_STEP_S))):
        main.fly_frame(sim, camera, main.SIM_TIME_STEP_S, {pygame.KSCAN_E: True})
    throttle = sim.throttle()
    assert throttle > 0.0, "E opened the grip"
    main.draw_scene(sim, camera, main.VIEW_CHASE)
    glFinish()
    assert is_colour(colour_at(at(box, hand_point(throttle, PROBE))),
                     main.PANEL_RED), "the hand, over the world"
    low, split, high = main.throttle_pie_spans()
    yellow_deg, yellow_gap = clear_between(low + 3.0, split - 3.0, throttle)
    green_deg, green_gap = clear_between(split + 3.0, high - 3.0, throttle)
    open_deg, open_gap = clear_between(high + 3.0, low + 357.0, throttle)
    assert min(yellow_gap, green_gap, open_gap) >= HAND_CLEAR_DEG, \
        "the frame's probes are off the hand's own line"
    probe(box, yellow_deg, throttle, main.PANEL_YELLOW, "the pie's yellow")
    probe(box, green_deg, throttle, main.PANEL_GREEN, "the pie's green")
    assert is_bare(pie_point(open_deg), box), \
        "with none of the pie's colours in its open wedge, over the world"
    print("  over the world, after a second of E: the grip at %.3f, the hand at"
          " %.1f deg" % (throttle, main.throttle_hand_deg(throttle)))


def save_png(path, x0, y0, width, height):
    """Write a window rectangle out as a PNG, the right way up, for a human.

    The framebuffer's own origin is the bottom left and a PNG's is the top left,
    so the rows come back flipped - which is why this is worth doing rather than
    handed to anybody as an image: the file is the window's own view, not a
    mirror of it.
    """
    data = glReadPixels(int(x0), int(y0), int(width), int(height), GL_RGB,
                        GL_UNSIGNED_BYTE)
    image = pygame.image.frombuffer(data, (int(width), int(height)), "RGB")
    pygame.image.save(pygame.transform.flip(image, False, True), path)
    print("  wrote %s" % (path,))


def main_check():
    """Draw the panel, read it back, and assert the picture it is."""
    pygame.init()
    pygame.display.set_mode((main.WIDTH, main.HEIGHT), DOUBLEBUF | OPENGL | HIDDEN)
    main.resize(main.WIDTH, main.HEIGHT)
    main.init_gl()
    box = main.panel_box(main.PANEL_COLLECTIVE)
    check_the_pie(box)
    sim = Simulation(airframe=airframe_preset(main.AIRFRAME_PRESET))
    check_the_frame(sim)
    # The panel at five grips, side by side, and the whole window with the panels
    # on it: the files to hold next to attic/collective_and_throttle.png.
    camera = ChaseCamera()
    here = os.path.dirname(os.path.abspath(__file__))
    x0, y0, x1, y1 = box
    for index, throttle in enumerate((0.0, 0.25, 0.5, 0.75, 1.0)):
        draw_panel(box, 0.4, throttle)
        save_png(os.path.join(here, "_throttle_pie_%d.png" % index),
                 x0 - 6.0, y0 - 6.0, x1 - x0 + 12.0, y1 - y0 + 12.0)
        dump_panel(box, "the panel with the grip at %.2f" % (throttle,))
    sim.on_the_pad()
    main.draw_scene(sim, camera, main.VIEW_CHASE)
    glFinish()
    save_png(os.path.join(here, "_throttle_window.png"), 0.0, 0.0,
             main.WIDTH, main.HEIGHT)
    print("throttle pie check passed")
    pygame.quit()


if __name__ == "__main__":
    main_check()
