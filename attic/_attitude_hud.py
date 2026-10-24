# -*- coding: utf-8 -*-
"""Scratch: read the attitude indicator back out of a real OpenGL context.

Not part of the project, and not imported by anything.  `main.py --check` asserts
the panel's arithmetic headless; this asserts the *drawing* of it, which needs a
context, by drawing the panel into a hidden window and reading the pixels back
with ``glReadPixels``.  Run from the project root:

    python attic/_attitude_hud.py

The panel is drawn exactly as ``main.draw_control_panel`` draws it - the same
pixel projection, the same blend state, the same functions - so what is read back
is what the window shows, and the assertions are the picture: the blue sky over
the brown ground with the green horizon between them, the green ladder where the
pitch puts it, the ball turning to port under a roll to starboard, the white
reference fixed in the middle of the dial at every attitude, and that reference
white over the green wherever the two cross.
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

#: The readback: a small block of the framebuffer, and how close a colour has to
#: be to a panel's own to count as that panel's colour.
BLOCK_HALF = 2
COLOUR_TOLERANCE = 0.06


def colour_at(point, half=BLOCK_HALF):
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


def at(box, fraction_x, fraction_y):
    """A point of the panel in window pixels, from its own fractions."""
    return main.panel_point(box, fraction_x, fraction_y)


def pixel_offset(box, point, offset_x, offset_y):
    """A point of the panel *offset_x*, *offset_y* window pixels from *point*."""
    x, y = at(box, *point)
    return (x + offset_x, y + offset_y)


def draw_panel(box, roll_deg, pitch_deg):
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
    main.draw_attitude_panel(box, roll_deg, pitch_deg)
    glDisable(GL_BLEND)
    glEnable(GL_CULL_FACE)
    glEnable(GL_FOG)
    glEnable(GL_DEPTH_TEST)
    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glFinish()


def check_the_picture(box):
    """Every shape of the instrument, as the pixels the window shows.

    The panel's own fractions are the units throughout: the middle of the dial is
    ``(ATTITUDE_CENTRE_X, ATTITUDE_CENTRE_Y)`` and the pitch scale is
    ``ATTITUDE_PITCH_DEG_PER_RADIUS`` degrees to the ball's own radius, so a
    window pixel is ``1 / PANEL_PX`` of the panel and the assertions below can be
    written in the same numbers the drawing is.
    """
    px = main.PANEL_PX
    centre = (main.ATTITUDE_CENTRE_X, main.ATTITUDE_CENTRE_Y)
    scale = main.ATTITUDE_BALL_RADIUS * px / main.ATTITUDE_PITCH_DEG_PER_RADIUS
    sky, ground = main.ATTITUDE_SKY_COLOUR, main.ATTITUDE_GROUND_COLOUR
    green, white = main.ATTITUDE_GREEN, main.ATTITUDE_WHITE
    case = main.ATTITUDE_CASE_COLOUR

    # Wings level and level: the sky above the middle of the dial, the ground
    # below it, the case outside the ball, and one green line along the middle
    # between them - one pixel of it, with a column of sky a pixel above and a
    # column of ground a pixel below.
    draw_panel(box, 0.0, 0.0)
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, 12.0)), sky)
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, -12.0)), ground)
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, 60.0)), case)
    for offset in (-55.0, 55.0):
        assert is_colour(colour_at(pixel_offset(box, centre, offset, 0.0), half=0),
                         green), "the horizon runs the width of the ball"
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, 1.0), half=0), sky), \
        "the horizon is one pixel thick and the sky is over it"
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, -1.0), half=0),
                     ground), "and the ground under it"
    # Where the reference crosses the horizon the reference is what is there: the
    # bars are drawn after the ball's own lines and over them.
    for offset in (-30.0, 30.0):
        assert is_colour(colour_at(pixel_offset(box, centre, offset, 0.0), half=0),
                         white), "the reference over the horizon it crosses"
    # The ladder: a rung a whole 10 deg up and one 10 deg down, and a 20 deg rung
    # beyond them, each of them drawn short of the ball's own width.
    for step in (10.0, -10.0, 20.0):
        assert is_colour(colour_at(pixel_offset(box, centre, 0.0, step * scale),
                                   half=0), green), "the rung at %+.0f deg" % step
    assert is_colour(colour_at(pixel_offset(box, centre,
                                            main.ATTITUDE_LADDER_HALF * 0.5 * px,
                                            10.0 * scale)), sky), \
        "a rung is a short line, not the ball's own width"

    # A nose up takes the horizon down the dial, and the aircraft's reference is
    # left white in the middle, over the sky it is now in the middle of.
    draw_panel(box, 0.0, 10.0)
    horizon = -10.0 * scale
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, horizon), half=0),
                     green), "the horizon at 10 deg of pitch"
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, horizon + 5.0)), sky)
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, horizon - 5.0)), ground)
    for offset in (-30.0, 30.0):
        assert is_colour(colour_at(pixel_offset(box, centre, offset, 0.0), half=0),
                         white), "the reference's bars, on the dial's middle line"
    # Every segment of the reference is on the dial where its own arithmetic says.
    # Each is sampled at its own middle rather than at an end: one pixel of a
    # diagonal line does not have to reach into the pixel that its corner is in.
    for (x0, y0), (x1, y1) in main.attitude_reference_strokes():
        midpoint = (0.5 * (x0 + x1), 0.5 * (y0 + y1))
        assert is_colour(colour_at(at(box, *midpoint), half=0), white), \
            "the reference at %s" % (midpoint,)
    valley = -main.ATTITUDE_W_DEPTH * px
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, valley)), sky), \
        "the middle of the W is a peak, not a valley"

    # A roll to starboard turns the sky to port and lifts the starboard end of the
    # horizon, so at the same height either side of the middle the port pixel is
    # sky and the starboard one is ground - and the ball's own up, which the
    # ladder follows, leans to port by the roll.
    draw_panel(box, 20.0, 0.0)
    for column, height, want in ((-30.0, -6.0, sky), (30.0, 6.0, ground),
                                 (-30.0, 6.0, sky), (30.0, -6.0, ground)):
        assert is_colour(colour_at(pixel_offset(box, centre, column, height),
                                   half=0), want), \
            "the horizon is tilted: %+.0f px across at %+.0f px up" % (column, height)
    assert is_colour(colour_at(pixel_offset(
        box, centre, -math.sin(math.radians(20.0)) * 10.0 * scale,
        math.cos(math.radians(20.0)) * 10.0 * scale), half=0), green), \
        "the 10 deg rung is drawn off the dial's own vertical"
    for offset in (-30.0, 30.0):
        assert is_colour(colour_at(pixel_offset(box, centre, offset, 0.0), half=0),
                         white), "and the reference has not moved a pixel"

    # A pitch past the scale's own end: the horizon has left the dial, so the ball
    # is one colour - all sky nose up, all ground nose down - with no line across
    # it at all, and the reference is still exactly where it was.
    draw_panel(box, 0.0, main.ATTITUDE_PITCH_DEG_PER_RADIUS + 10.0)
    for offset_x, offset_y in ((-45.0, 30.0), (45.0, -30.0), (0.0, 30.0),
                               (-12.0, 0.0)):
        assert is_colour(colour_at(pixel_offset(box, centre, offset_x, offset_y)),
                         sky), "all sky at a nose up"
    draw_panel(box, 0.0, -main.ATTITUDE_PITCH_DEG_PER_RADIUS - 10.0)
    for offset_x, offset_y in ((-45.0, 30.0), (45.0, -30.0), (0.0, -30.0),
                               (-12.0, 0.0)):
        assert is_colour(colour_at(pixel_offset(box, centre, offset_x, offset_y)),
                         ground), "all ground at a nose down"
    for offset in (-30.0, 30.0):
        assert is_colour(colour_at(pixel_offset(box, centre, offset, 0.0), half=0),
                         white), "the reference again"

    # And the horizon is never drawn over the case: at a pitch that leaves it in
    # the dial its ends are the ball's own chord, so a pixel or two inside the end
    # is the line and a pixel or two outside it is the ring of case - which is
    # what says the line was cut to the ball rather than run out over the dial.
    draw_panel(box, 0.0, 20.0)
    half_chord = main.ATTITUDE_BALL_RADIUS * math.sin(
        main.attitude_horizon_gamma(0.0, 20.0)) * px
    horizon = -20.0 * scale
    assert is_colour(colour_at(pixel_offset(box, centre, half_chord - 2.0, horizon),
                               half=0), green), "the horizon ends inside the ball"
    assert is_colour(colour_at(pixel_offset(box, centre, half_chord + 2.5, horizon),
                               half=0), case), "and the case is outside it"
    ring = 0.5 * (main.ATTITUDE_BALL_RADIUS + main.ATTITUDE_RADIUS) * px
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, ring)), case), \
        "with the ring of case around the ball"
    print("  the panel alone, at five attitudes: %.1f px per 10 deg of pitch"
          % (10.0 * scale))


def check_the_frame(sim):
    """The panel in the frame it is drawn in, over the world, from a real sim."""
    camera = ChaseCamera()
    sim.on_the_pad()
    main.draw_scene(sim, camera, main.VIEW_CHASE)
    glFinish()
    box = main.panel_box(main.PANEL_ATTITUDE)
    centre = (main.ATTITUDE_CENTRE_X, main.ATTITUDE_CENTRE_Y)
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, 10.0)),
                     main.ATTITUDE_SKY_COLOUR), "the panel is drawn over the world"
    assert is_colour(colour_at(pixel_offset(box, centre, 0.0, -10.0)),
                     main.ATTITUDE_GROUND_COLOUR)
    assert is_colour(colour_at(pixel_offset(box, centre, -30.0, 0.0), half=0),
                     main.ATTITUDE_WHITE), "the reference, on the pad"
    for index in range(main.PANEL_COUNT):
        assert main.panel_box(index)[2] <= main.WIDTH
    print("  the seven panels are over the world, in the chase view")


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
    box = main.panel_box(main.PANEL_ATTITUDE)
    check_the_picture(box)
    sim = Simulation(airframe=airframe_preset(main.AIRFRAME_PRESET))
    check_the_frame(sim)
    camera = ChaseCamera()
    for _ in range(120):
        main.fly_frame(sim, camera, main.SIM_TIME_STEP_S,
                       {pygame.KSCAN_RIGHT: True, pygame.KSCAN_D: True})
    pad = sim.telemetry()
    draw_panel(box, pad.roll_deg, pad.pitch_deg)
    print("  the pad's own attitude, drawn: roll %+.3f, pitch %+.3f deg"
          % (pad.roll_deg, pad.pitch_deg))
    # A strip of the panel at four attitudes, side by side, and the whole window
    # with the panels on it: the files to hold next to attic/attitude.png.
    strip = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "_attitude_hud.png")
    x0, y0, x1, y1 = box
    glClearColor(0.02, 0.02, 0.02, 1.0)
    glClear(GL_COLOR_BUFFER_BIT)
    for index, (roll, pitch) in enumerate(((0.0, 0.0), (20.0, 0.0),
                                           (0.0, 10.0), (0.0, 30.0))):
        draw_panel(box, roll, pitch)
        save_png(strip.replace(".png", "_%d.png" % index),
                 x0 - 6.0, y0 - 6.0, x1 - x0 + 12.0, y1 - y0 + 12.0)
    main.draw_scene(sim, camera, main.VIEW_CHASE)
    glFinish()
    save_png(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "_attitude_window.png"), 0.0, 0.0,
             main.WIDTH, main.HEIGHT)
    print("attitude hud check passed")
    pygame.quit()


if __name__ == "__main__":
    main_check()
