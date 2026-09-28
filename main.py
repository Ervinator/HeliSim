#!/usr/bin/env python3
"""HeliSim - the UH-1H of NASA TM-73254, flown in a small OpenGL world.

The world is made of two parts:

* an *implicit* flat grass ground plane with a grid.  It is always present and
  is deliberately not part of the scenery description, and
* the objects listed in an XML scenery file (see scenery.py and
  sample_scenery.xml).  Every object has a coordinate and an orientation.

The aircraft is :class:`simulation.Simulation`: the frame loop of simulation.py
around the report's own 6158 lb instrumented UH-1H, the aircraft its figures 2 to
9 were measured on.  The keyboard goes into the model through
:class:`simulation.PilotInput`, the helicopter is drawn with one ``glMultMatrixf``
of :meth:`simulation.Simulation.render_matrix` - no Euler order and no sign left
to get wrong between the model and the screen - and the camera is
:class:`simulation.ChaseCamera`, whose ``eye_target_up`` triple goes straight into
``gluLookAt``.

**The world is kilometres across, because the aircraft is.**  A UH-1 covers a
kilometre in twenty seconds at 60 kt and the model's hover trims sit at 200 m, so
the ground is a plane 16 km on a side, its grid has two scales - 10 m lines
within 400 m of the aircraft, 250 m lines out to 8 km - and fog fades the plane's
own edge into the sky where a horizon would be.  Nothing clamps the aircraft or
the camera: the envelopes of simulation.py are the limits.

Usage::

    python main.py [scenery.xml]
    python main.py --check          # the wiring, headless, no window

Keys:

    W / S           collective up / down; it is a ratchet and stays where it is
    Up / Down       cyclic forward (nose down) / aft (nose up)
    Left / Right    cyclic left / right
    A / D           pedals: nose left / right, the anti torque control
    R               recover: back to the trimmed hover at 200 m
    P               park: skids on the pad, collective down
    C               camera: behind the aircraft, or fixed on the pad
    Esc             quit

The collective is the one control that does not spring back, and it is the one
that flies the machine: the aircraft starts parked on the pad, so hold W until
the rotor lifts it.  A crashed aircraft is still flown where it is - the model
does not stop - until R puts it back in the air.

The window's caption is this sandbox's instrument panel, and it carries all
four control positions: the collective in per cent, the longitudinal and
lateral cyclic and the pedals in inches of travel.  They are there because the
aircraft cannot always show a key on its own - on the pad the skids hold it
level and still, so the cyclic and the pedals move it by nothing at all until
it is off the ground.

pygame and PyOpenGL are this file's only third party imports: everything under it
is standard library only, and numpy is not imported at all.
"""

import math
import os
import sys

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import scenery
from aerodynamics import (UH1_BLADE_COUNT, UH1_CHORD, UH1_PRECONE_DEG,
                          UH1_RADIUS, UH1_RPM, UH1_TAIL_BLADE_COUNT,
                          UH1_TAIL_CHORD, UH1_TAIL_MAIN_RATIO, UH1_TAIL_RADIUS,
                          Vector3)
from airframe import (FOOT, POUND, UH1_HUB_HEIGHT, UH1_TAIL_ARM,
                      UH1_TAIL_HEIGHT)
from simulation import (ChaseCamera, SIM_TIME_STEP_S, Simulation,
                        UH1_CG_HEIGHT_ON_GROUND, airframe_preset)

WIDTH, HEIGHT = 960, 640

# Scenery file used when no path is given on the command line.
DEFAULT_SCENERY_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "sample_scenery.xml")

# The report's own configuration to fly: the 6158 lb instrumented aircraft of
# TM-73254's figures 2 to 9.  Its 0.144 s rotor time constant is also the easier
# of the report's two to hold a key against; see simulation.airframe_preset.
AIRFRAME_PRESET = "flight test"

# ---------------------------------------------------------------------------
# The world's scale.  See the module docstring: the ground is a plane kilometres
# on a side and its grid is drawn at two scales around the aircraft.
# ---------------------------------------------------------------------------

#: Half the width of the implicit ground plane, m.  The model's trims sit at
#: 200 m and a 60 kt trace covers a kilometre in twenty seconds, so the ground
#: has to be a flying world rather than a field to walk around: 16 km across,
#: with fog to hide where it ends.
GROUND_HALF_M = 8000.0

#: The near grid: its spacing and how far from the aircraft it is drawn, m.  Both
#: grids are aligned to their own spacing in world coordinates, so their lines
#: stay where they are in the world as the aircraft moves; only the window the
#: lines are drawn in follows it.
GRID_NEAR_STEP_M = 10.0
GRID_NEAR_HALF_M = 400.0
GRID_FAR_STEP_M = 250.0
GRID_FAR_HALF_M = 8000.0

#: Where the ground's edge and the far grid fade into the sky, m.
FOG_START_M = 1200.0
FOG_END_M = 9000.0

#: The sky, which is also the fog's colour: one colour, so that the ground plane
#: ends in a horizon rather than in a rim of a different shade.
SKY_COLOUR = (0.58, 0.80, 0.96, 1.0)

FOV_DEG = 62.0
#: Near and far planes, m.  The far plane has to reach the ground plane's far
#: corner through fog that is opaque well before it, and the near plane only has
#: to be in front of a camera that sits 14 m behind the aircraft.
NEAR_PLANE_M = 1.0
FAR_PLANE_M = 20000.0

# ---------------------------------------------------------------------------
# The aircraft: body axes, +x nose, +y starboard, +z down, the c.g. at the
# origin.  simulation.render_matrix() puts this frame in the renderer's world, so
# every shape below is drawn in the model's own body axes and nothing here
# converts a coordinate before drawing it.
#
# The sizes that matter are the model's own - the main rotor's radius, chord,
# blade count and precone, the tail rotor's arm, height, radius and chord, the hub
# height above the c.g., the waterline the skids stand at - and what is left is a
# sketch of a UH-1's outline out of boxes.
# ---------------------------------------------------------------------------

#: Metres in an inch: the report's geometry is in inches, and table 2's stations
#: are where the fuselage's own numbers below come from.
INCH = FOOT / 12.0

#: The hub sits 9.9 in ahead of the c.g.: hub station 133.5 in against the
#: 143.4 in the report's c.g. is at (see the comment on airframe.UH1_HUB_HEIGHT).
HUB_FORWARD_M = 9.9 * INCH

#: The skids are drawn on the model's own skid line, so the aircraft's drawn
#: undercarriage sits on the drawn ground: an upright UH-1's c.g. stands
#: simulation.UH1_CG_HEIGHT_ON_GROUND up, which is the 55.0 in of hub waterline
#: less the hub's own 6.79 ft.
SKID_HEIGHT_M = UH1_CG_HEIGHT_ON_GROUND
SKID_HALF_WIDTH_M = 1.15
SKID_FRONT_M = 2.60
SKID_BACK_M = -2.60
SKID_STRUT_X_M = 1.50
SKID_STRUT_Z_M = 0.72           # the cabin floor the struts reach up to

#: A UH-1's tail rotor is on the port side of the fin, which is -y.
TAIL_PORT_M = 0.45

BODY_COLOUR = (0.36, 0.38, 0.26)
GLASS_COLOUR = (0.20, 0.28, 0.34)
SKID_COLOUR = (0.60, 0.62, 0.64)
BLADE_COLOUR = (0.15, 0.15, 0.16)

#: The cabin, the windscreen, the nose, the tail boom, the fin and the horizontal
#: stabiliser, as (colour, x0, x1, y0, y1, z0, z1) in body axes.
AIRFRAME_BOXES = (
    (BODY_COLOUR, (-2.10, 2.30, -1.25, 1.25, -1.15, 0.75)),
    (GLASS_COLOUR, (2.30, 2.60, -1.05, 1.05, -0.95, 0.10)),
    (BODY_COLOUR, (2.60, 3.10, -0.90, 0.90, -0.75, 0.55)),
    (BODY_COLOUR, (-7.70, -2.10, -0.42, 0.42, -0.25, 0.45)),
    (BODY_COLOUR, (-UH1_TAIL_ARM - 0.45, -UH1_TAIL_ARM + 0.45,
                   -0.12, 0.12, -UH1_TAIL_HEIGHT - 0.35, -1.15)),
    (BODY_COLOUR, (-UH1_TAIL_ARM + 0.60, -UH1_TAIL_ARM + 1.60,
                   -1.50, 1.50, -0.50, -0.25)),
)

#: The shadow under the aircraft, which is what gives the chase view a sense of
#: height: a disc of fixed radius on the ground, faded by altitude.  It sits
#: above the near grid's lines so that the two do not fight for depth.
SHADOW_RADIUS_M = 5.5
SHADOW_ALPHA = 0.35
SHADOW_FADE_M = 45.0
SHADOW_SEGMENTS = 40
SHADOW_HEIGHT_M = 0.06

#: Where the fixed pad camera watches from, and the world's own up vector.  In the
#: renderer's world x is east, y is up and z is south.
PAD_EYE = Vector3(26.0, 11.0, 26.0)
WORLD_UP = Vector3(0.0, 1.0, 0.0)

# The scenery description, filled in by main() from the XML file.
SCENERY = scenery.Scenery()


# ---------------------------------------------------------------------------
# The world: the implicit ground, the scenery, and the aircraft in it.
# ---------------------------------------------------------------------------


def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(FOV_DEG, width / float(height), NEAR_PLANE_M, FAR_PLANE_M)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init_gl():
    glClearColor(*SKY_COLOUR)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_CULL_FACE)
    glCullFace(GL_BACK)
    glShadeModel(GL_SMOOTH)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    # Fog in the sky's own colour.  This is what turns the edge of a 16 km ground
    # plane into a horizon instead of a rim, and what keeps the far grid from
    # drawing an absurdity in the distance.
    glEnable(GL_FOG)
    glFogf(GL_FOG_MODE, GL_LINEAR)
    glFogf(GL_FOG_START, FOG_START_M)
    glFogf(GL_FOG_END, FOG_END_M)
    glFogfv(GL_FOG_COLOR, SKY_COLOUR)


def draw_ground():
    """Draw the implicit ground plane and its grid.

    The ground and its grid always exist and are deliberately not part of the
    scenery description.  The quad is wound counter clockwise as seen from
    above, so its upper side survives GL_CULL_FACE, and it is as wide as the
    world: GROUND_HALF_M either side of the origin.
    """
    # Large flat grass surface.
    glColor3f(0.34, 0.60, 0.20)
    glBegin(GL_QUADS)
    glVertex3f(-GROUND_HALF_M, 0.0, GROUND_HALF_M)
    glVertex3f(GROUND_HALF_M, 0.0, GROUND_HALF_M)
    glVertex3f(GROUND_HALF_M, 0.0, -GROUND_HALF_M)
    glVertex3f(-GROUND_HALF_M, 0.0, -GROUND_HALF_M)
    glEnd()


def draw_ground_grid(centre_x, centre_z, step, half, height, colour):
    """Draw one scale of the grid around the aircraft.

    The window is *half* metres either side of the aircraft - clipped to the
    ground plane, so that no line is drawn over the void - but every line is
    placed at a multiple of *step* in world coordinates, so the pattern belongs
    to the world rather than to the aircraft: only the window's edge moves as the
    aircraft does.  The height keeps this scale off the plane and off the other
    scale, since there is no polygon offset here.
    """
    low_x = max(centre_x - half, -GROUND_HALF_M)
    high_x = min(centre_x + half, GROUND_HALF_M)
    low_z = max(centre_z - half, -GROUND_HALF_M)
    high_z = min(centre_z + half, GROUND_HALF_M)

    glColor3f(*colour)
    glBegin(GL_LINES)

    # Lines running north and south, then lines running east and west.
    x = math.ceil(low_x / step) * step
    while x <= high_x:
        glVertex3f(x, height, low_z)
        glVertex3f(x, height, high_z)
        x += step
    z = math.ceil(low_z / step) * step
    while z <= high_z:
        glVertex3f(low_x, height, z)
        glVertex3f(high_x, height, z)
        z += step

    glEnd()


# --- the "tree" object, in local space: base centre at the origin, +Y up ----
TREE_TRUNK_HALF = 0.20
TREE_TRUNK_HEIGHT = 1.20


def draw_tree_geometry():
    """Draw a tree around the local origin (same shape as the old code)."""
    half = TREE_TRUNK_HALF
    top = TREE_TRUNK_HEIGHT

    # Trunk: filled vector box, every side wound so that it faces outwards.
    glColor3f(0.44, 0.25, 0.10)
    glBegin(GL_QUADS)
    # -Z
    glVertex3f(-half, top, -half)
    glVertex3f(half, top, -half)
    glVertex3f(half, 0.0, -half)
    glVertex3f(-half, 0.0, -half)
    # +X
    glVertex3f(half, top, -half)
    glVertex3f(half, top, half)
    glVertex3f(half, 0.0, half)
    glVertex3f(half, 0.0, -half)
    # +Z
    glVertex3f(-half, 0.0, half)
    glVertex3f(half, 0.0, half)
    glVertex3f(half, top, half)
    glVertex3f(-half, top, half)
    # -X
    glVertex3f(-half, 0.0, -half)
    glVertex3f(-half, 0.0, half)
    glVertex3f(-half, top, half)
    glVertex3f(-half, top, -half)
    glEnd()

    # Crown: the three triangular canopy sheets of the original tree.  They are
    # flat, so back face culling would hide them from one side - draw them two
    # sided instead.
    glColor3f(0.12, 0.56, 0.17)
    glDisable(GL_CULL_FACE)
    glBegin(GL_TRIANGLES)
    glVertex3f(0.0, 1.80, 0.0)
    glVertex3f(-1.2, 1.0, 0.0)
    glVertex3f(1.2, 1.0, 0.0)

    glVertex3f(0.0, 1.85, 0.0)
    glVertex3f(0.0, 1.0, -1.2)
    glVertex3f(0.0, 1.0, 1.2)

    glVertex3f(0.0, 1.60, 0.0)
    glVertex3f(-1.0, 1.0, -0.5)
    glVertex3f(1.0, 1.0, 0.5)
    glEnd()
    glEnable(GL_CULL_FACE)


# --- the "hill" object: a pyramid on a convex quad base ---------------------
def draw_hill_geometry(params):
    """Draw a hill around the local origin (base centre at y = 0)."""
    front = params["base_front"]
    back = params["base_back"]
    depth = params["base_depth"]
    height = params["height"]

    # Base corners, all at y = 0.  Two different widths give a trapezium, which
    # is always a convex quad.
    p0 = (-0.5 * front, 0.0, -0.5 * depth)
    p1 = (0.5 * front, 0.0, -0.5 * depth)
    p2 = (0.5 * back, 0.0, 0.5 * depth)
    p3 = (-0.5 * back, 0.0, 0.5 * depth)
    apex = (params["apex_x"], height, params["apex_z"])

    # p3 -> p2 -> p1 -> p0 walks the base counter clockwise as seen from above.
    # Used with consecutive neighbours it winds every side face outwards, and
    # on its own it is the upward facing base quad.  The per face shading
    # compensates for the fact that there is no OpenGL lighting.
    base_loop = [p3, p2, p1, p0]
    shades = (0.85, 1.08, 0.92, 1.15)

    glBegin(GL_TRIANGLES)
    for index in range(len(base_loop)):
        start = base_loop[index]
        end = base_loop[(index + 1) % len(base_loop)]
        shade = shades[index]
        glColor3f(0.36 * shade, 0.42 * shade, 0.22 * shade)
        glVertex3f(start[0], start[1], start[2])
        glVertex3f(end[0], end[1], end[2])
        glVertex3f(apex[0], apex[1], apex[2])
    glEnd()

    # The base quad itself, wound so that it faces upwards.
    glColor3f(0.30, 0.35, 0.19)
    glBegin(GL_QUADS)
    for vertex in base_loop:
        glVertex3f(vertex[0], vertex[1], vertex[2])
    glEnd()


def draw_object(obj):
    """Draw one scenery object, honouring its coordinate and orientation."""
    glPushMatrix()
    glTranslatef(obj.position.x, obj.position.y, obj.position.z)
    # Orientation is applied as R = Ry * Rx * Rz (yaw, then pitch, then roll).
    glRotatef(obj.rotation.ry, 0.0, 1.0, 0.0)
    glRotatef(obj.rotation.rx, 1.0, 0.0, 0.0)
    glRotatef(obj.rotation.rz, 0.0, 0.0, 1.0)

    if obj.kind == scenery.TREE:
        scale = obj.param("scale")
        glScalef(scale, scale, scale)
        draw_tree_geometry()
    elif obj.kind == scenery.HILL:
        draw_hill_geometry(obj.effective_params())
    else:
        raise ValueError("no renderer for scenery object %r" % (obj.kind,))

    glPopMatrix()


def draw_scenery():
    """Draw every object of the scenery description loaded from XML."""
    for obj in SCENERY:
        draw_object(obj)


# ---------------------------------------------------------------------------
# The helicopter, drawn in the body axes its place in the world comes with.
# ---------------------------------------------------------------------------


def draw_box(colour, x0, x1, y0, y1, z0, z1):
    """Draw an axis aligned box, with a flat shade on each of its faces.

    Nothing here uses OpenGL lighting - the trees and the hills got by with a
    shade per face too - so each face carries a fixed brightness of its own,
    which is enough to read one box as a solid rather than as a silhouette.
    """
    x0, x1 = min(x0, x1), max(x0, x1)
    y0, y1 = min(y0, y1), max(y0, y1)
    z0, z1 = min(z0, z1), max(z0, z1)
    red, green, blue = colour

    faces = (
        (((x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)), 0.82),
        (((x1, y0, z0), (x1, y0, z1), (x1, y1, z1), (x1, y1, z0)), 1.02),
        (((x0, y0, z0), (x0, y0, z1), (x1, y0, z1), (x1, y0, z0)), 0.90),
        (((x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)), 0.90),
        (((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)), 0.72),
        (((x0, y0, z1), (x0, y1, z1), (x1, y1, z1), (x1, y0, z1)), 1.10),
    )
    for corners, shade in faces:
        glColor3f(red * shade, green * shade, blue * shade)
        glBegin(GL_QUADS)
        for x, y, z in corners:
            glVertex3f(x, y, z)
        glEnd()


def draw_rotor_disc(radius, segments=48, alpha=0.20):
    """A translucent disc standing in for a rotor's swept area.

    Drawn in the current frame's own x-y plane, about its z, so a caller whose
    rotor turns about something else has only to turn the frame first.
    """
    glColor4f(0.12, 0.14, 0.12, alpha)
    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(0.0, 0.0, 0.0)
    for index in range(segments + 1):
        angle = 2.0 * math.pi * index / segments
        glVertex3f(radius * math.cos(angle), radius * math.sin(angle), 0.0)
    glEnd()


def rotor_azimuth_deg(sim, ratio=1.0):
    """A rotor's azimuth, degrees, from the simulation's own clock.

    The model holds 100 per cent rotor speed whatever the aircraft is doing -
    it has no engine and no rotor speed dynamics in it - so the blades turn at
    UH1_RPM.  *ratio* is how many revolutions a rotor makes in one of the main
    rotor's, which is 1 for the main rotor and table 2's 5.56 for the tail.
    A rotor seen through a 60 Hz renderer strobes; so does one on film.
    """
    return math.fmod(sim.sim_time * UH1_RPM * 6.0 * ratio, 360.0)


def draw_main_rotor(azimuth_deg):
    """The two bladed main rotor, turning about the body z axis."""
    glPushMatrix()
    glTranslatef(HUB_FORWARD_M, 0.0, -UH1_HUB_HEIGHT)
    draw_rotor_disc(UH1_RADIUS)
    for index in range(UH1_BLADE_COUNT):
        glPushMatrix()
        glRotatef(azimuth_deg + 360.0 * index / UH1_BLADE_COUNT,
                  0.0, 0.0, 1.0)
        # Table 2's 2.75 deg of built in coning.  A +x blade tips up, and up is
        # -z in this frame, so the cone is a positive turn about y.
        glRotatef(UH1_PRECONE_DEG, 0.0, 1.0, 0.0)
        draw_box(BLADE_COLOUR, 0.0, UH1_RADIUS,
                 -0.5 * UH1_CHORD, 0.5 * UH1_CHORD, -0.02, 0.02)
        glPopMatrix()
    glPopMatrix()


def draw_tail_rotor(azimuth_deg):
    """The two bladed tail rotor, out to port and turning about body y."""
    glPushMatrix()
    glTranslatef(-UH1_TAIL_ARM, -TAIL_PORT_M, -UH1_TAIL_HEIGHT)
    # A frame whose own z is the tail rotor's axis, which is the body y axis:
    # the disc and the blades are then drawn in that frame's x-y plane.
    glRotatef(90.0, 1.0, 0.0, 0.0)
    draw_rotor_disc(UH1_TAIL_RADIUS, segments=24, alpha=0.22)
    for index in range(UH1_TAIL_BLADE_COUNT):
        glPushMatrix()
        glRotatef(azimuth_deg + 360.0 * index / UH1_TAIL_BLADE_COUNT,
                  0.0, 0.0, 1.0)
        draw_box(BLADE_COLOUR, 0.0, UH1_TAIL_RADIUS,
                 -0.5 * UH1_TAIL_CHORD, 0.5 * UH1_TAIL_CHORD, -0.01, 0.01)
        glPopMatrix()
    glPopMatrix()


def draw_helicopter(sim):
    """Draw the aircraft where the model puts it, at the attitude it has.

    One ``glMultMatrixf`` of :meth:`Simulation.render_matrix` puts this frame
    into the body axes every shape here is drawn in, so nothing below converts a
    coordinate and there is no Euler order to get wrong on the way to the screen.
    Back face culling is off for the boxes: they are closed and opaque, and which
    way their faces wind is not worth getting wrong for a sketch of an outline.
    """
    glPushMatrix()
    glMultMatrixf((GLfloat * 16)(*sim.render_matrix()))

    glDisable(GL_CULL_FACE)
    for colour, corners in AIRFRAME_BOXES:
        draw_box(colour, *corners)
    for side in (-1.0, 1.0):
        # A skid: the rail below, on the model's own skid line, and its struts.
        rail = side * SKID_HALF_WIDTH_M
        draw_box(SKID_COLOUR, SKID_BACK_M, SKID_FRONT_M,
                 rail - 0.09, rail + 0.09,
                 SKID_HEIGHT_M - 0.06, SKID_HEIGHT_M + 0.06)
        for x in (-SKID_STRUT_X_M, SKID_STRUT_X_M):
            draw_box(SKID_COLOUR, x - 0.10, x + 0.10,
                     rail - 0.07, rail + 0.07,
                     SKID_STRUT_Z_M, SKID_HEIGHT_M)

    # The rotors last, translucent and without depth writes, so that a disc does
    # not hide the aircraft behind it, and the blades at the clock's azimuth.
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glDepthMask(GL_FALSE)
    draw_main_rotor(rotor_azimuth_deg(sim))
    draw_tail_rotor(rotor_azimuth_deg(sim, UH1_TAIL_MAIN_RATIO))
    glDepthMask(GL_TRUE)
    glDisable(GL_BLEND)

    glEnable(GL_CULL_FACE)
    glPopMatrix()


def draw_shadow(sim):
    """A soft disc on the ground under the aircraft, faded by its altitude.

    This is the chase view's only cue for how far off the ground the aircraft
    is, so it is worth the blending: a dark patch under the skids on the pad, and
    nearly gone by a hundred metres up.
    """
    position = sim.render_position()
    altitude = max(sim.airframe.state.altitude, 0.0)
    alpha = SHADOW_ALPHA * math.exp(-altitude / SHADOW_FADE_M)
    if alpha < 0.01:
        return

    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glColor4f(0.05, 0.12, 0.05, alpha)
    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(position.x, SHADOW_HEIGHT_M, position.z)
    for index in range(SHADOW_SEGMENTS + 1):
        angle = 2.0 * math.pi * index / SHADOW_SEGMENTS
        glVertex3f(position.x + SHADOW_RADIUS_M * math.cos(angle),
                   SHADOW_HEIGHT_M,
                   position.z + SHADOW_RADIUS_M * math.sin(angle))
    glEnd()
    glDisable(GL_BLEND)


def draw_scene(sim, camera, view):
    """One frame of the world, from the camera the view asks for."""
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    eye, target, up = camera_eye_target_up(sim, camera, view)
    gluLookAt(eye.x, eye.y, eye.z, target.x, target.y, target.z,
              up.x, up.y, up.z)

    # The ground first: the plane, then its two grids, both of them centred on
    # the aircraft so that they follow it without a line ever moving in the world.
    aircraft = sim.render_position()
    draw_ground()
    draw_ground_grid(aircraft.x, aircraft.z, GRID_FAR_STEP_M, GRID_FAR_HALF_M,
                     0.02, (0.24, 0.46, 0.16))
    draw_ground_grid(aircraft.x, aircraft.z, GRID_NEAR_STEP_M, GRID_NEAR_HALF_M,
                     0.04, (0.16, 0.36, 0.12))

    draw_scenery()
    draw_shadow(sim)
    draw_helicopter(sim)


def load_scenery(path):
    """Read a scenery description, reporting problems on stderr."""
    try:
        return scenery.Scenery.load(path)
    except OSError as error:
        print("error: cannot read scenery %s: %s" % (path, error),
              file=sys.stderr)
    except scenery.SceneryError as error:
        print("error: invalid scenery %s: %s" % (path, error), file=sys.stderr)
    return None


# ---------------------------------------------------------------------------
# The frame loop: the keys in, the clock, the camera and the window.
# ---------------------------------------------------------------------------

#: The two views this sandbox has.  Both look at the aircraft - from behind it, or
#: from the pad - because the keys a free camera would need are the aircraft's own.
VIEW_CHASE = "chase"
VIEW_PAD = "pad"
VIEWS = (VIEW_CHASE, VIEW_PAD)


def pilot_keys(keys):
    """The keys held this frame, as :meth:`PilotInput.step`'s own keywords.

    *keys* is anything indexable by a key constant: ``pygame.key.get_pressed()``
    in the frame loop, and a plain dict in the check at the bottom of this file.
    pygame 2 indexes that sequence by *scancode*, so the keys below are read as
    the KSCAN_* constants - the physical key positions - which also means that a
    keyboard whose layout moves the letters does not move the controls.
    """
    return {
        "collective": _held(keys, pygame.KSCAN_W)
                      - _held(keys, pygame.KSCAN_S),
        "long_stick": _held(keys, pygame.KSCAN_UP)
                      - _held(keys, pygame.KSCAN_DOWN),
        "lat_stick": _held(keys, pygame.KSCAN_RIGHT)
                     - _held(keys, pygame.KSCAN_LEFT),
        "pedal": _held(keys, pygame.KSCAN_D) - _held(keys, pygame.KSCAN_A),
    }


def _held(keys, key):
    """Is one key down?  A dict answers as readily as get_pressed()'s sequence."""
    try:
        return 1.0 if keys[key] else 0.0
    except (IndexError, KeyError, TypeError):
        return 0.0


def camera_eye_target_up(sim, camera, view):
    """``(eye, target, up)`` for ``gluLookAt``, from the view asked for.

    The chase view is :class:`simulation.ChaseCamera`'s own triple, which is in
    the renderer's world already.  The pad view is this file's: a fixed eye south
    east of the pad looking at the aircraft, which is what one wants when the
    aircraft has flown out of the chase camera's reach.
    """
    if view == VIEW_PAD:
        return PAD_EYE, sim.render_position(), WORLD_UP
    return camera.eye_target_up()


def fly_frame(sim, camera, frame_dt, keys, view=VIEW_CHASE):
    """One frame of the sandbox without a renderer: keys, clock, camera.

    The keyboard goes to the model through :meth:`Simulation.fly`, which samples
    :class:`simulation.PilotInput` and spends the frame on whole physics steps;
    the camera then follows the aircraft whichever view is in use, so that
    changing views never means flying the camera across the world.  This is the
    whole of the frame loop that needs no OpenGL context, which is what makes the
    ``--check`` at the bottom of this file possible.
    """
    sim.fly(frame_dt, **pilot_keys(keys))
    camera.update(frame_dt, sim.render_position(), sim.render_basis())
    return sim


def window_title(sim, view):
    """The window's caption: what a HUD would show, and the view in use.

    The four control positions are on it in the model's own units - the
    collective in per cent of its travel, the two cyclic sticks and the
    pedals in inches - because this caption is the sandbox's only instrument.
    The collective alone would leave a key that the aircraft cannot show
    looking like a key that does nothing: on the pad the skids hold the
    aircraft level and still, so the cyclic and the pedals move it by nothing
    at all until it is off the ground.
    """
    telemetry = sim.telemetry()
    return ("HeliSim - UH-1H | alt %6.1f m %+6.0f fpm | %5.1f kt | coll %3.0f %%"
            " | cyc %+6.2f/%+6.2f in | ped %+5.2f in%s%s%s | %s view"
            % (telemetry.alt_agl, telemetry.height_rate_fpm,
               telemetry.airspeed_kt, 100.0 * telemetry.collective_fraction,
               telemetry.long_stick_in, telemetry.lat_stick_in,
               telemetry.pedal_in,
               " | trim" if telemetry.in_trim else "",
               " | on the ground" if telemetry.on_ground else "",
               " | CRASHED" if telemetry.crashed else "", view))


def main():
    global SCENERY

    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SCENERY_FILE
    loaded = load_scenery(path)
    if loaded is None:
        sys.exit(2)
    SCENERY = loaded

    print("scenery: %d object(s) loaded from %s" % (len(SCENERY), path))
    for obj in SCENERY:
        print("  %-4s at (%6.1f, %5.1f, %6.1f)  orientation (%5.1f, %5.1f, %5.1f)"
              % (obj.kind, obj.position.x, obj.position.y, obj.position.z,
                 obj.rotation.rx, obj.rotation.ry, obj.rotation.rz))

    # The aircraft, before there is a window: the trim is solved here, which
    # takes a moment, and the flight it describes is worth having in writing.
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    print("aircraft: the report's %s configuration, %.0f lb, rotor R6 %.3f s"
          % (AIRFRAME_PRESET, sim.airframe.mass / POUND,
             sim.airframe.rotor_time_constant))
    print("  the run's own trim: %s" % (sim.trim_controls,))
    sim.on_the_pad()
    print("  parked on the pad:  %s" % (sim.telemetry(),))
    print("  R recovers to that trim, a %.0f m hover, and the world is %.0f km"
          " across" % (sim.start_altitude, 2.0 * GROUND_HALF_M / 1000.0))
    print("keys: W/S collective, arrows cyclic, A/D pedals, R recover, P park,"
          " C camera, Esc quit")

    camera = ChaseCamera()
    view = VIEW_CHASE

    pygame.init()
    pygame.display.set_mode((WIDTH, HEIGHT), DOUBLEBUF | OPENGL)
    resize(WIDTH, HEIGHT)
    init_gl()
    pygame.display.set_caption(window_title(sim, view))

    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in (pygame.K_r, pygame.K_p):
                    # R is Simulation.reset, which comes back to the trim the run
                    # began with, and P is on_the_pad.  Both put the pilot's axes
                    # where the aircraft already is, and both need the camera to
                    # snap onto it rather than fly across the map.
                    recovering = event.key == pygame.K_r
                    if recovering:
                        sim.reset()
                    else:
                        sim.on_the_pad()
                    camera.reset()
                    print("  %s: %s"
                          % ("recovered to the trim" if recovering
                             else "parked on the pad", sim.telemetry()))
                elif event.key == pygame.K_c:
                    view = VIEWS[(VIEWS.index(view) + 1) % len(VIEWS)]
                    camera.reset()
                    print("  view: %s" % (view,))

        # The frame's own time, whatever it turned out to be: the physics inside
        # Simulation.step is always 1/60 s steps, so a dragged window is a longer
        # frame rather than a faster aircraft.
        frame_dt = clock.tick(60) / 1000.0
        fly_frame(sim, camera, frame_dt, pygame.key.get_pressed(), view)
        draw_scene(sim, camera, view)
        pygame.display.flip()

        # The window's caption is the HUD, and it is cheap enough to set a few
        # times a second rather than on every frame.
        if sim.frames % 15 == 0:
            pygame.display.set_caption(window_title(sim, view))
        if sim.frames % 60 == 0:
            print("  " + str(sim.telemetry()))

    pygame.quit()
    sys.exit()


# ---------------------------------------------------------------------------
# The check, in the style of the modules underneath: assertions, no window.
# ---------------------------------------------------------------------------


def _self_check():
    """The wiring of this file, headless: keys, clock, camera, reset keys.

    pygame is imported for its key constants and nothing else - no window, no
    OpenGL context, no rendering - so this runs wherever the dependencies are
    installed.  Raises AssertionError on failure, which is what
    ``python main.py --check`` runs.
    """
    # The keyboard: W and S are the collective, the up and down arrows cyclic fore
    # and aft, left and right cyclic sideways, A and D the pedals - and opposite
    # keys cancel, which is what a pilot's other hand does.
    assert pilot_keys({}) == {"collective": 0.0, "long_stick": 0.0,
                              "lat_stick": 0.0, "pedal": 0.0}
    assert pilot_keys({pygame.KSCAN_W: True})["collective"] == 1.0
    assert pilot_keys({pygame.KSCAN_S: True})["collective"] == -1.0
    assert pilot_keys({pygame.KSCAN_W: True,
                       pygame.KSCAN_S: True})["collective"] == 0.0
    assert pilot_keys({pygame.KSCAN_UP: True})["long_stick"] == 1.0    # nose down
    assert pilot_keys({pygame.KSCAN_DOWN: True})["long_stick"] == -1.0  # nose up
    assert pilot_keys({pygame.KSCAN_RIGHT: True})["lat_stick"] == 1.0   # roll right
    assert pilot_keys({pygame.KSCAN_LEFT: True})["lat_stick"] == -1.0
    assert pilot_keys({pygame.KSCAN_D: True})["pedal"] == 1.0     # nose right
    assert pilot_keys({pygame.KSCAN_A: True})["pedal"] == -1.0

    # A fresh sandbox starts the way main() leaves it: parked on the pad, the
    # collective at its down stop, level, with the trim the run began with
    # remembered as what a recover comes back to.
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.on_the_pad()
    assert sim.on_ground and not sim.crashed
    assert abs(sim.airframe.state.altitude - UH1_CG_HEIGHT_ON_GROUND) < 1e-9
    assert sim.pilot.collective_axis == 0.0
    assert sim.controls.collective_fraction == 0.0
    assert sim.trim_controls is not None and sim.trim_state is not None

    camera = ChaseCamera()
    # The collective key lifts it off the skids: simulation.py's own demo pickup,
    # now through this file's frame loop, key mapping and camera.  It is a lightly
    # damped heave and may settle back onto the skids, so what is checked is that
    # it leaves the ground and gets clear of the skid line on the way.
    airborne = False
    highest = 0.0
    trim_fraction = sim.trim_controls.collective_fraction
    for _ in range(int(6.0 / SIM_TIME_STEP_S)):
        keys = ({pygame.KSCAN_W: True}
                if sim.pilot.collective_axis < trim_fraction else {})
        fly_frame(sim, camera, SIM_TIME_STEP_S, keys)
        airborne = airborne or not sim.on_ground
        highest = max(highest, sim.airframe.state.altitude)
        assert math.isfinite(sim.airframe.state.altitude)
    assert airborne and highest > UH1_CG_HEIGHT_ON_GROUND + 0.5, highest
    assert not sim.crashed and math.isfinite(sim.telemetry().thrust)

    # R: recover is Simulation.reset, which comes back to that trim with the
    # pilot's own hands on it - and they are not centred, since a hover trim is
    # 9.09 in of collective and a couple of inches of cyclic against the tail
    # rotor.  What the frame loop does with them is then a real input: released,
    # the cyclic springs back to centre the way a helicopter's does, the
    # collective ratchets where the trim left it, and the aircraft leaves the
    # hover on its own.  That the *trim itself* holds is simulation.py's own
    # self test's business, not this file's.
    sim.reset()
    camera.reset()
    assert not sim.on_ground and not sim.crashed and sim.in_trim()
    assert abs(sim.airframe.state.altitude - 200.0) < 1e-9
    assert abs(sim.pilot.collective_axis - trim_fraction) < 1e-12
    assert sim.pilot.long_stick_axis != 0.0 and sim.pilot.pedal_axis != 0.0
    for _ in range(120):
        fly_frame(sim, camera, SIM_TIME_STEP_S, {})
    state = sim.airframe.state
    assert sim.frames == 120 and sim.steps == 120
    assert abs(sim.sim_time - 2.0) < 1e-9      # one clock, and it is the model's
    assert not sim.crashed and abs(state.altitude - 200.0) < 1.0
    assert math.hypot(state.position.x, state.position.y) > 0.5   # it moved
    assert abs(sim.pilot.long_stick_axis) < 1e-9    # the cyclic centred itself
    assert abs(sim.pilot.lat_stick_axis) < 1e-9
    assert abs(sim.pilot.pedal_axis) < 1e-9
    assert abs(sim.pilot.collective_axis - trim_fraction) < 1e-12  # it ratcheted

    # The camera: the chase view sits behind and above the smoothed aircraft, in
    # the renderer's world, and it carries the world's own up vector so that the
    # horizon stays level however the helicopter rolls.
    position = sim.render_position()
    eye, target, up = camera_eye_target_up(sim, camera, VIEW_CHASE)
    assert up.as_tuple() == WORLD_UP.as_tuple()
    assert eye.y >= camera.min_height
    assert (camera.position - eye).dot(camera.nose) > 0.0    # the eye is behind
    assert abs((eye - camera.position).length()
               - math.hypot(camera.distance, camera.height)) < 1e-9
    assert abs((target - camera.position).length() - camera.look_ahead) < 1e-9
    # The pad view is this file's own: a fixed eye, looking at the aircraft.
    pad_eye, pad_target, pad_up = camera_eye_target_up(sim, camera, VIEW_PAD)
    assert pad_eye.as_tuple() == PAD_EYE.as_tuple()
    assert pad_target.as_tuple() == position.as_tuple()
    assert pad_up.as_tuple() == WORLD_UP.as_tuple()

    # The one glMultMatrixf of the whole renderer: sixteen floats, column major,
    # the body axes of render_basis in its columns and the aircraft's position in
    # its translation - and no numpy array anywhere in the handing over of it.
    matrix = sim.render_matrix()
    assert len(matrix) == 16 and matrix[3] == 0.0 and matrix[15] == 1.0
    for column, axis in enumerate(sim.render_basis()):
        for row in range(3):
            assert abs(matrix[column * 4 + row] - axis.as_tuple()[row]) < 1e-12
    assert abs(matrix[12] - position.x) < 1e-12
    assert abs(matrix[13] - position.y) < 1e-12
    assert abs(matrix[14] - position.z) < 1e-12
    assert len((GLfloat * 16)(*matrix)) == 16

    # The rotors turn at 100 per cent whatever the aircraft does - the model has
    # no engine and no rotor speed dynamics - at an azimuth taken from its own
    # clock: two seconds at 324 rpm is 10.8 revolutions, which leaves 288 deg,
    # and the tail rotor is that same clock at table 2's 5.56 times the speed.
    assert abs(rotor_azimuth_deg(sim) - 288.0) < 1e-6
    assert 0.0 <= rotor_azimuth_deg(sim) < 360.0
    assert abs(rotor_azimuth_deg(sim, UH1_TAIL_MAIN_RATIO) - 17.28) < 1e-6

    # The world's own scale, since it is a flying world and not a field to walk
    # around: the far grid is coarser than the near one and stays on the plane,
    # the fog is opaque before the far plane and starts beyond the aircraft, and
    # the plane is wide enough for the model's own speeds to be worth flying.
    assert 0.0 < GRID_NEAR_HALF_M < GRID_FAR_HALF_M <= GROUND_HALF_M
    assert GRID_NEAR_STEP_M < GRID_FAR_STEP_M
    assert NEAR_PLANE_M < FOG_START_M < FOG_END_M < FAR_PLANE_M
    assert GROUND_HALF_M >= 4000.0
    assert SHADOW_HEIGHT_M > 0.04          # above the near grid's own lines

    # The caption a HUD shows, and the scenery beside this file: neither needs a
    # window either, only the file itself.
    assert "UH-1H" in window_title(sim, VIEW_CHASE)
    assert "CRASHED" not in window_title(sim, VIEW_CHASE)
    assert VIEW_CHASE in window_title(sim, VIEW_CHASE)
    assert len(load_scenery(DEFAULT_SCENERY_FILE)) > 0

    # The caption carries the four control positions, and they are read off the
    # same telemetry the aircraft is flown from.  The case worth asserting is
    # the pad, where the skids hold the aircraft exactly level and still and a
    # held key moves nothing but the caption: W ratcheting the collective up,
    # the right cyclic and D arriving as inches of right stick and right pedal.
    sim.on_the_pad()
    parked_x = sim.airframe.state.position.x
    parked_y = sim.airframe.state.position.y
    for _ in range(30):
        fly_frame(sim, camera, SIM_TIME_STEP_S,
                  {pygame.KSCAN_W: True, pygame.KSCAN_RIGHT: True,
                   pygame.KSCAN_D: True})
    assert sim.on_ground and not sim.crashed
    assert sim.airframe.state.position.x == parked_x     # the skids hold it
    assert sim.airframe.state.position.y == parked_y
    assert sim.airframe.state.speed < 1e-9
    assert abs(sim.airframe.state.rates.x) < 1e-9        # and hold it level
    pad = sim.telemetry()
    assert 0.25 < pad.collective_fraction < 0.30         # W ratcheted, 0.55 a s
    assert pad.long_stick_in == 0.0
    assert pad.lat_stick_in > 1.0            # Right is a real input, in inches
    assert pad.pedal_in > 1.0                # and so is D
    assert ("| coll %3.0f %% | cyc %+6.2f/%+6.2f in | ped %+5.2f in"
            % (100.0 * pad.collective_fraction, pad.long_stick_in,
               pad.lat_stick_in, pad.pedal_in)) in window_title(sim, VIEW_CHASE)


if __name__ == "__main__":
    if "--check" in sys.argv[1:]:
        _self_check()
        print("main.py check passed")
    else:
        main()
