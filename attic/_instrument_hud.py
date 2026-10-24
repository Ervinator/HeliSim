# -*- coding: utf-8 -*-
"""Scratch: read the three instrument faces back out of a real OpenGL context.

Not part of the project, and not imported by anything.  `main.py --check` asserts
the faces' arithmetic headless; this asserts the *drawing* of them, which needs a
context, the way `attic/_attitude_hud.py` does for the attitude indicator.  The
altimeter, the vertical speed indicator and the airspeed indicator are drawn into
a hidden window exactly as ``main.draw_control_panel`` draws them, and their
numerals are measured from the font's own metrics - twice-size numbers have to
stay inside the ring of ticks, clear of one another and clear of the faces' own
words - and a PNG of each face is written to look at.  Run from the project root:

    python attic/_instrument_hud.py

The three ``_instrument_*.png`` files beside this one are what it wrote, to hold
next to attic/altimeter.png, attic/vertspeed.png and attic/airspeed.png.
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

#: A margin round a face in the saved picture, so its whole case is in it.
MARGIN = 8.0

#: The clear space the bigger numerals and the faces' own words have to keep, px.
CLEARANCE_PX = 0.5


def ink(text, size_px):
    """The ink of *text* at *size_px*: its rect and the surface it sits in.

    ``(left, top, width, height, box_w, box_h)``: the HUD centres the surface the
    font renders on the text's own position, so the ink is offset inside it.
    """
    surface = main.hud_font(size_px).render(text, True, (255, 255, 255))
    rect = surface.get_bounding_rect()
    return (rect.left, rect.top, rect.width, rect.height,
            surface.get_width(), surface.get_height())


def numeral_box(text, radius, deg):
    """A numeral as drawn: ``(x, y, width, height)`` right and up from the middle.

    *radius* is in pixels and *deg* is clockwise from straight up, the way
    :func:`main.dial_point` measures both.
    """
    left, top, width, height, box_w, box_h = ink(text, main.INSTRUMENT_NUMERAL_PX)
    angle = math.radians(deg)
    return (radius * math.sin(angle) - 0.5 * box_w + left + 0.5 * width,
            radius * math.cos(angle) + 0.5 * box_h - top - 0.5 * height,
            width, height)


def word_box(text, size_px, fraction_x, fraction_y):
    """A face's own word as drawn, in the panel fractions it is placed at."""
    left, top, width, height, box_w, box_h = ink(text, size_px)
    return ((fraction_x - 0.5) * main.PANEL_PX - 0.5 * box_w + left
            + 0.5 * width,
            (fraction_y - 0.5) * main.PANEL_PX + 0.5 * box_h - top
            - 0.5 * height,
            width, height)


def reach(boxed):
    """The furthest corner of *boxed* from the middle of the face, px."""
    x, y, width, height = boxed
    return max(math.hypot(x + sx * 0.5 * width, y + sy * 0.5 * height)
               for sx in (-1.0, 1.0) for sy in (-1.0, 1.0))


def separation(first, second):
    """The clear space between two boxes, px; negative where they overlap."""
    x1, y1, w1, h1 = first
    x2, y2, w2, h2 = second
    return max(abs(x1 - x2) - 0.5 * (w1 + w2), abs(y1 - y2) - 0.5 * (h1 + h2))


def faces():
    """The three faces: their numerals, the degrees between them, and their words.

    The words are the same ones :func:`main.draw_altimeter_panel`,
    :func:`main.draw_vsi_panel` and :func:`main.draw_airspeed_panel` draw, at the
    sizes and the panel fractions they draw them at.
    """
    return (("altimeter",
             [str(n) for n in range(main.ALTIMETER_NUMERALS)],
             360.0 / main.ALTIMETER_NUMERALS, 0.0,
             (("ALT", main.INSTRUMENT_LABEL_PX, 0.69, 0.50),)),
            ("vertspeed",
             [str(n) for n in range(main.VSI_NUMERAL_STEPS + 1)],
             main.VSI_DEG_PER_1000_FPM, main.VSI_ZERO_DEG,
             (("VERTICAL SPEED", main.INSTRUMENT_SMALL_LABEL_PX, 0.5, 0.42),
              ("DOWN", main.INSTRUMENT_LABEL_PX, 0.22, 0.62))),
            ("airspeed",
             [str(20 * n) for n in range(1, main.AIRSPEED_NUMERALS + 1)],
             360.0 / main.AIRSPEED_NUMERALS, 0.0,
             (("KNOTS", main.INSTRUMENT_LABEL_PX, 0.5, 0.40),)))


def check_the_numbers():
    """The numerals of the three faces, as the pixels they are drawn at.

    Every numeral has to stay inside the ring of ticks - it is read against that
    ring - and clear of its neighbours and of the face's own words by
    :data:`CLEARANCE_PX`, whatever :data:`main.INSTRUMENT_NUMERAL_PX` and
    :data:`main.INSTRUMENT_NUMERAL_RADIUS` are set to.
    """
    radius = main.INSTRUMENT_NUMERAL_RADIUS * main.PANEL_PX
    ring = main.INSTRUMENT_TICK_LONG_INNER * main.PANEL_PX
    for name, texts, step_deg, first_deg, words in faces():
        boxes = [numeral_box(text, radius, first_deg + index * step_deg)
                 for index, text in enumerate(texts)]
        widest = max(reach(boxed) for boxed in boxes)
        nearest = min(separation(boxes[index], boxes[(index + 1) % len(boxes)])
                      for index in range(len(boxes)))
        print("  %-10s %2d numerals, widest reaches %5.1f px of the ring's %5.1f,"
              " nearest pair %+5.1f px"
              % (name, len(texts), widest, ring, nearest))
        assert widest <= ring - CLEARANCE_PX, name
        assert nearest >= CLEARANCE_PX, name
        for text, size_px, fraction_x, fraction_y in words:
            word = word_box(text, size_px, fraction_x, fraction_y)
            clear = min(separation(word, boxed) for boxed in boxes)
            print("  %-10s the word \"%s\" keeps %+5.1f px of the numerals"
                  % (name, text, clear))
            assert clear >= CLEARANCE_PX, (name, text)


def draw_face(box, painter, value):
    """One face alone, in ``draw_control_panel``'s own GL state."""
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
    painter(box, value)
    glDisable(GL_BLEND)
    glEnable(GL_CULL_FACE)
    glEnable(GL_FOG)
    glEnable(GL_DEPTH_TEST)
    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glFinish()


def save_png(path, x0, y0, width, height):
    """Write a window rectangle out as a PNG, the right way up, for a human."""
    data = glReadPixels(int(x0), int(y0), int(width), int(height), GL_RGB,
                        GL_UNSIGNED_BYTE)
    image = pygame.image.frombuffer(data, (int(width), int(height)), "RGB")
    pygame.image.save(pygame.transform.flip(image, False, True), path)
    print("  wrote %s" % (path,))


def main_check():
    """Measure the numerals, then draw each face and save the picture of it."""
    pygame.init()
    pygame.display.set_mode((main.WIDTH, main.HEIGHT), DOUBLEBUF | OPENGL | HIDDEN)
    main.resize(main.WIDTH, main.HEIGHT)
    main.init_gl()
    check_the_numbers()
    here = os.path.dirname(os.path.abspath(__file__))
    drawn = (("altimeter", main.PANEL_ALTIMETER, main.draw_altimeter_panel, 1234.0),
             ("vertspeed", main.PANEL_VSI, main.draw_vsi_panel, -2400.0),
             ("airspeed", main.PANEL_AIRSPEED, main.draw_airspeed_panel, 120.0))
    for name, index, painter, value in drawn:
        box = main.panel_box(index)
        x0, y0, x1, y1 = box
        glClearColor(0.02, 0.02, 0.02, 1.0)
        glClear(GL_COLOR_BUFFER_BIT)
        draw_face(box, painter, value)
        save_png(os.path.join(here, "_instrument_%s.png" % (name,)),
                 x0 - MARGIN, y0 - MARGIN, x1 - x0 + 2.0 * MARGIN,
                 y1 - y0 + 2.0 * MARGIN)
    print("instrument hud check passed")
    pygame.quit()


if __name__ == "__main__":
    main_check()
