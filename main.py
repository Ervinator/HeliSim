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

    python main.py [scenery.xml] [--log FILE | --no-log]
    python main.py --check          # the wiring, headless, no window

Keys:

    W / S           collective up / down
    Up / Down       cyclic forward (nose down) / aft (nose up)
    Left / Right    cyclic left / right
    A / D           pedals: nose left / right, the anti torque control
    R               reset: back to the state the run started in, on the pad
    P               park: skids on the pad, the lever at 75 %
    C               camera: behind the aircraft, or fixed on the pad
    Esc             quit

All four axes are ratchets: a key slews its control while the key is held and a
key that is released is no control input at all, so nothing springs back.  The
cyclic and the pedals stay where the hand leaves them, which is what a UH-1's
friction and force trim do and what a centring spring would not, and the
collective stays where it is put.  The collective is the one that flies the
machine: the aircraft starts parked on the pad with the lever three quarters of
the way up (`simulation.PAD_COLLECTIVE`), which is where a pickup begins rather
than seven seconds of winding, so hold W for a moment and the rotor lifts it.  A
flight that has ended is still flown where it is - the
model does not stop - until `R` clears it, and the caption says which end it came
to: `CRASHED` for the skids arriving on the ground over 3 m/s, `OUT OF ENVELOPE`
for a state the model refused to fly, which is an overload in mid-air rather than
a landing, and the altitude beside that one is where it was refused.  R resets it,
which puts it back on the pad this run began
on rather than on any trim: a trim is the reference the caption's light reads,
not a state a reset is obliged to jump to, and handing a pilot the hover's
9.09 in of collective is a control position they never made.  W may be tapped as
well as held: the frame loop takes its keys from the events, so a press and
release that both happen inside one frame still turn the ratchet.  The keys are
deliberately fine: `PilotInput`'s three step granularities - a quarter by
default, `simulation.COLLECTIVE_STEP_GRANULARITY` and its two neighbours - scale
each control's rate down for the keyboard alone, so a key is worth a quarter of
the travel a full rate frame is and a control takes four times the presses, and
four times as long on the key, to cross the same ground.  That is what makes a
stick placeable rather than something only thrown from one stop to the other; a
joystick or a script is not scaled, since a position that is told where to be
has no keypress to step.

The window's caption is this sandbox's instrument panel, and it carries all
four control positions: the collective in per cent, the longitudinal and
lateral cyclic and the pedals in inches of travel.  They are there because the
aircraft cannot always show a key on its own - on the pad the skids hold it
level and still, so the cyclic and the pedals move it by nothing at all until
it is off the ground.  The caption also says when the window does not have the
keyboard, which is the one failure that looks exactly like a dead key, and which
end a flight came to when one has ended: the model's own two messages, `CRASHED`
and `OUT OF ENVELOPE`, because the second is an overload in the air and one word
for both would put a landing under an altitude nothing landed from.

**Every keystroke is logged, with the flight it was made in.**  A run writes
`keylog.txt` beside this file (`--log FILE` puts it elsewhere, `--no-log` flies
without one): one line for each key as it is taken out of pygame's queue, and
`KEYLOG_RATE_HZ` lines a second of the flight between them, so a session's keys
and the aircraft they flew are in one file on one clock.  Every line carries the
aircraft's own velocity - the airspeed through the airframe and the ground
velocity north, east and down - its heading, pitch, roll and altitude, and all
four of the pilot's control positions in the inches of travel the caption shows.
Each is stamped twice: `t_s`, monotonic seconds since the log was opened, and
`wall_ms`, the same instant as `time.time` milliseconds, so a key can be read
both against the flight around it and against anything else that happened at
that time of day.  Both are written to a millisecond.

Those same four positions are drawn in the window as well, in the three panels
of ``attic/controls_simple.png``: a red line for each pedal, moving oppositely,
a red disc for the cyclic, and a red collective lever in a green field.  The
panels are read from :meth:`simulation.PilotInput.axes` - the hands, which is
what a pilot's own view of the controls is - so a key moves them whether or not
the aircraft can move, and the panels say it without a word.  Beside them is a
fourth, an attitude indicator in the shape of ``attic/attitude.png``: a small
moving horizon instrument, with the aircraft's own white reference fixed over the
middle of it and the ball - blue sky over brown ground, a green horizon and a
green pitch ladder - turning and sliding behind that.  It is the one panel the
hands cannot draw, and so the one read from the aircraft itself:
:meth:`simulation.Simulation.telemetry`'s own roll and pitch, since a control
position is not an attitude.

pygame and PyOpenGL are this file's only third party imports: everything under it
is standard library only, and numpy is not imported at all.
"""

import io
import math
import os
import sys
import time

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import scenery
from aerodynamics import (UH1_BLADE_COUNT, UH1_CHORD, UH1_COLLECTIVE_TRAVEL_IN,
                          UH1_PRECONE_DEG, UH1_RADIUS, UH1_RPM,
                          UH1_TAIL_BLADE_COUNT, UH1_TAIL_CHORD,
                          UH1_TAIL_MAIN_RATIO, UH1_TAIL_RADIUS, Vector3)
from airframe import (FOOT, POUND, UH1_HUB_HEIGHT, UH1_TAIL_ARM,
                      UH1_TAIL_HEIGHT)
from simulation import (ChaseCamera, PAD_COLLECTIVE, SIM_TIME_STEP_S, Simulation,
                        UH1_CG_HEIGHT_ON_GROUND, airframe_preset)

WIDTH, HEIGHT = 960, 640

# Scenery file used when no path is given on the command line.
DEFAULT_SCENERY_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "sample_scenery.xml")

#: Where the key log goes unless the command line says otherwise: beside this
#: file rather than in the directory a run happens to be started from, so that
#: the log of the last flight is always in the same place.  One run overwrites
#: the last one's; ``--log FILE`` writes it elsewhere and ``--no-log`` flies
#: without one.
DEFAULT_KEYLOG_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "keylog.txt")

#: The key log's sample rate, Hz: how often the flight itself is written down
#: between the keystrokes.  Sixty hertz would be one line per frame and a log
#: three times the size for a flight the physics cannot resolve any finer than
#: its own 1/60 s step; twenty is enough to place every key's effect on the
#: trace and cheap enough to leave switched on.
KEYLOG_RATE_HZ = 20.0

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
    """One frame of the world, from the camera the view asks for.

    The last thing drawn is this file's own HUD, the control position panel of
    :func:`draw_control_panel`, which is in window pixels rather than in the
    world and so is not the camera's business at all.
    """
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
    draw_control_panel(sim)


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
# The control position panel: the three panels of attic/controls_simple.png and
# the attitude indicator of attic/attitude.png, drawn over the world in the
# bottom left corner of the window.
#
# The caption says where the controls are in the model's own units; these say it
# again without a word, and in the shapes the mockup gives them: a red line for
# each pedal, moving oppositely, a red disc for the cyclic, and a red collective
# lever in a green field.  Each panel is drawn from PilotInput's own axes - the
# hands, the same four control *positions* the caption prints - so a key the
# aircraft cannot show (a pedal against a pair of skids on the pad) still moves
# something a pilot can see, and nothing here reads a trimmed control or the
# model's inches.
#
# The fourth panel is not a control at all.  It is an attitude indicator, and the
# one thing on this HUD that no hand holds: it is drawn from the model's own roll
# and pitch, in the shape of the classic moving horizon instrument - the case with
# the aircraft's own reference fixed over the middle of it and, behind that, the
# ball, whose sky and ground are split by the horizon the attitude puts there and
# laddered by the pitch lines that say how far from level it is.  The sky over the
# ground is blue over brown, the ball's own lines are green, and the aircraft's
# reference is the one white thing on the dial: it is painted last of all, over
# those lines, so neither the bars nor the W is ever lost in one of them.
#
# The last three are not controls either, but the instruments a real panel
# carries: an altimeter, a vertical speed indicator and an airspeed indicator,
# each a round face on the same grey case as the attitude indicator's own dial,
# with white ticks and a white needle and, on the whole of this HUD, one red
# mark - the airspeed indicator's radial at its caution speed.  They are read
# from the aircraft rather than from a hand - the height above the ground, the
# rate of climb and the airspeed, all three of them straight off
# :meth:`simulation.Simulation.telemetry` - and their numbers and words are the
# only text on the HUD, so they are the one thing here that is drawn rather
# than shaped.
#
# The panels are pixels rather than metres: they are drawn with the projection
# switched to the window's own coordinates, after the world, so no camera move
# and no distance in the world touches them.
# ---------------------------------------------------------------------------

#: The panels: square, in the mockup's own order and size, with a gap between them
#: and a margin around them, all px.  They are no longer one row but an L: the four
#: control and attitude panels stacked up the window's left edge, from the pedals
#: at the top to the attitude indicator at the bottom, and the three instruments -
#: the altimeter, the vertical speed indicator and the airspeed indicator - along
#: the bottom of the window beside them.  A panel is 132 px of a 960 px window,
#: which is as small as a UH-1's travels can be read at and no bigger than a
#: sandbox HUD deserves.
PANEL_PX = 132.0
PANEL_GAP_PX = 14.0
PANEL_MARGIN_PX = 14.0
PANEL_COUNT = 7

#: How the seven fall: this many up the left column, the pedals first at the top of
#: it and the attitude indicator last at the bottom, and the rest - the instruments
#: - along the bottom row to the right of the column.
PANEL_COLUMN_COUNT = 4

#: The mockup itself: three black squares with a red line, a red disc and a green
#: field on them.  It is named here because the run prints it, and because it is
#: where a shape or an angle is changed - the numbers below are read off it.
#: ``attic`` is this project's ignored scratch, so the file is a reference and
#: never an input: nothing here opens it, and the shapes are described in the
#: comments above rather than left to a picture a reader may not have.
PANEL_MOCKUP_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "attic", "controls_simple.png")

#: The attitude indicator's own reference, named and treated the same way: a small
#: instrument whose case carries the aircraft's reference over the middle of it,
#: with the sky, the ground, the horizon and the pitch lines of a moving horizon
#: ball behind that.  It shares the mockup's row, so it is the fourth panel.
PANEL_ATTITUDE_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "attic", "attitude.png")

#: The three instruments' own references, named and treated the same way again:
#: the altimeter, the vertical speed indicator and the airspeed indicator of
#: attic/altimeter.png, attic/vertspeed.png and attic/airspeed.png.  They are what
#: a real panel carries rather than a control, so they are round-faced and read a
#: number; they finish the L the controls begin.
PANEL_ALTIMETER_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "attic", "altimeter.png")
PANEL_VSI_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "attic", "vertspeed.png")
PANEL_AIRSPEED_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "attic", "airspeed.png")

#: The panels' own indices, so that each drawing call names the control it is.
PANEL_PEDALS = 0
PANEL_CYCLIC = 1
PANEL_COLLECTIVE = 2
PANEL_ATTITUDE = 3
PANEL_ALTIMETER = 4
PANEL_VSI = 5
PANEL_AIRSPEED = 6

#: The mockup's own colours: black panels, one red for all four axes - it is one
#: hand's worth of positions - and the green of the collective's field.
PANEL_BACK_COLOUR = (0.03, 0.03, 0.04, 0.72)
PANEL_RED = (0.90, 0.08, 0.10)
PANEL_GREEN = (0.16, 0.92, 0.18)

#: The attitude indicator's own colours: a grey case, the sky blue over the ground
#: brown, and one green for the ball's own lines - the horizon and the rungs of
#: the ladder above and below it.  The aircraft's own reference is the one white
#: thing on the dial, and it is painted over those lines rather than under them,
#: so the bars and the W stay white wherever a green line crosses them.  The case
#: and the ball are opaque, as the collective's green field is: the world shows
#: through the black panel behind the instrument and not through the instrument
#: itself.
ATTITUDE_CASE_COLOUR = (0.22, 0.23, 0.25)
ATTITUDE_SKY_COLOUR = (0.16, 0.42, 0.80)
ATTITUDE_GROUND_COLOUR = (0.45, 0.29, 0.13)
ATTITUDE_GREEN = (0.20, 0.90, 0.24)
ATTITUDE_WHITE = (0.93, 0.95, 0.96)

#: The pedal panel: the mockup's two lines, as fractions of the panel.  Each line
#: has an x of its own - the left pedal drawn on the left and the right one on
#: the right - and the two of them share the height they swing about.
PEDAL_LEFT_X = 0.22
PEDAL_RIGHT_X = 0.70
PEDAL_LINE_HALF_X = 0.20
PEDAL_LINE_HALF_Y = 0.028

#: How far a full pedal moves a line from that shared middle, panel fractions:
#: the two lines cross at the middle - both pedals are where they belong - and
#: are this far apart at a stop, one up and one down.  It is less than half the
#: panel so that neither line leaves it.
PEDAL_LINE_TRAVEL = 0.62

#: The cyclic panel: the mockup's red disc, a plan view of the stick.  A full
#: stick puts the disc's middle this far from the middle of the panel, and this
#: is its radius, both as panel fractions: the stops are inside the panel, so a
#: stick on a stop has a whole disc rather than one the panel's edge cuts.
CYCLIC_DOT_TRAVEL = 0.62
CYCLIC_DOT_RADIUS = 0.075
CYCLIC_DOT_SEGMENTS = 24

#: The collective panel: a lever on a quadrant, pivoted in the panel's own
#: bottom left corner, the field under it the sector between the down stop and
#: the up stop.  These are the mockup's own angles and radius - degrees above the
#: panel's horizontal, and panel fractions from the pivot, which is a little
#: inside the corner so that the lever's own thickness stays on the panel.
QUADRANT_LOW_DEG = 12.0
QUADRANT_HIGH_DEG = 55.0
QUADRANT_PIVOT_X = 0.035
QUADRANT_PIVOT_Y = 0.035
QUADRANT_RADIUS = 0.965
QUADRANT_LEVER_HALF = 0.032
QUADRANT_SEGMENTS = 32

#: The attitude panel: the moving horizon instrument of attic/attitude.png, in
#: panel fractions and the classic layout.  The dial is the case's own circle and
#: the ball is what is left inside the plain rim around it; the ball's sky, its
#: ground and its horizon are drawn inside that, and the aircraft's own reference
#: is fixed over the middle of it and never moves.  Nothing else of the picture's
#: case is drawn - no rim ticks and no index - because the reference and the
#: horizon are the whole of what a panel this size has to say.
ATTITUDE_CENTRE_X = 0.5
ATTITUDE_CENTRE_Y = 0.5
ATTITUDE_RADIUS = 0.47
ATTITUDE_RIM = 0.035
ATTITUDE_BALL_RADIUS = ATTITUDE_RADIUS - ATTITUDE_RIM
ATTITUDE_SEGMENTS = 48

#: The ball's own pitch scale: this many degrees of pitch put the horizon on the
#: ball's rim and on its way off the dial.  It is a scale and not a limit - the
#: instrument is a drawing, and a pitch past it is a ball that is all sky or all
#: ground - and 25 deg is a scale a UH-1's own trim and its gentle climbs sit
#: comfortably inside while a big one still reads.
ATTITUDE_PITCH_DEG_PER_RADIUS = 25.0

#: The ladder: a pitch line every 10 deg, two of them each side of the horizon -
#: the lines the picture has room for - with the first rung this fraction of the
#: ball's radius long, half of it, and each rung further out this much shorter.
#: The rungs are shorter than the horizon, which runs the whole visible width of
#: the ball, and shorter again as they go out, which is what the picture does.
ATTITUDE_LADDER_DEG = 10.0
ATTITUDE_LADDER_RUNGS = 2
ATTITUDE_LADDER_HALF = 0.70
ATTITUDE_LADDER_SHRINK = 0.22

#: How thick a line of the dial is drawn, half of it: the horizon, the rungs of
#: the ladder and the aircraft's own reference are one pixel wide, as the thin
#: white lines of the reference picture are, and a pixel is ``1 / PANEL_PX`` of
#: the panel.  It is written as a pixel and divided by the panel here rather than
#: written as a fraction, so that it stays one pixel whatever ``PANEL_PX``
#: becomes; the other three panels are several pixels thick everywhere because
#: none of their shapes is a line.
ATTITUDE_LINE_PX = 1.0
ATTITUDE_LINE_HALF = 0.5 * ATTITUDE_LINE_PX / PANEL_PX

#: The aircraft's own reference: the mockup's shape is two short bars either side
#: of a small W, their inner ends and the W's own middle peak on the dial's own
#: horizontal through its centre, and the W's two valleys this far below it.  The
#: bars begin inside the W's span and end well inside the ball, so the whole
#: reference is drawn on the dial at any attitude and never leaves it.  Its six
#: segments are the same one pixel as the ball's own lines and are drawn after
#: them, so the reference is over the green wherever the two cross.
ATTITUDE_BAR_INNER = 0.13
ATTITUDE_BAR_OUTER = 0.34
ATTITUDE_W_DEPTH = 0.055

#: The instruments: the altimeter, the vertical speed indicator and the airspeed
#: indicator, each a round face at the middle of its own panel and on the same grey
#: case the attitude indicator's own dial is - the three of them and the attitude
#: indicator are one instrument seen four ways.  Everything drawn on a face is the
#: one white of the reference, but for the airspeed indicator's red radial.
INSTRUMENT_CENTRE_X = 0.5
INSTRUMENT_CENTRE_Y = 0.5
INSTRUMENT_RADIUS = 0.47

#: The ring of ticks and the numbers inside it, panel fractions from the middle of
#: the face: a long tick reaches in towards the numbers and a short one starts
#: further out, so the two are told apart by their length as the reference faces'
#: own ticks are.  A tick is a line and so is a needle, so both are written as
#: pixels and divided by the panel, the way the attitude indicator's own line is.
INSTRUMENT_TICK_OUTER = 0.425
INSTRUMENT_TICK_LONG_INNER = 0.350
INSTRUMENT_TICK_SHORT_INNER = 0.385
INSTRUMENT_TICK_HALF = 0.5 * 1.4 / PANEL_PX
INSTRUMENT_NEEDLE_HALF = 0.5 * 2.2 / PANEL_PX
INSTRUMENT_THIN_NEEDLE_HALF = 0.5 * 1.2 / PANEL_PX
INSTRUMENT_NUMERAL_RADIUS = 0.300
INSTRUMENT_NUMERAL_PX = 10
INSTRUMENT_LABEL_PX = 9
INSTRUMENT_SMALL_LABEL_PX = 8

#: The altimeter's own face: a ring of ticks one to each hundred feet with every
#: tenth of them - one to each thousand - the long one, the numbers nought to nine
#: inside that, and two needles.  The long thin one is the hundreds and turns once
#: in a thousand feet, the short thick one the thousands and turns once in ten
#: thousand, and both of them point straight up at zero.
ALTIMETER_TICKS = 100
ALTIMETER_NUMERALS = 10
ALTIMETER_HUNDREDS_OUTER = 0.425
ALTIMETER_THOUSANDS_OUTER = 0.205

#: The vertical speed indicator: nought at nine o'clock and a scale of three
#: thousand feet a minute either way, fifty degrees of the dial to every thousand -
#: the reference face's own spread, which puts the top of the scale at about
#: two o'clock and the bottom at about four and leaves the middle of the dial
#: free for its words.  Every 250 feet a minute of it is a tick and every thousand a
#: number.
VSI_FULL_SCALE_FPM = 3000.0
VSI_ZERO_DEG = 270.0
VSI_DEG_PER_1000_FPM = 50.0
VSI_TICK_FPM = 250.0
VSI_TICK_STEPS = 24
VSI_NUMERAL_FPM = 1000.0
VSI_NUMERAL_STEPS = 3
VSI_NEEDLE_OUTER = 0.335

#: The airspeed indicator: nought at twelve o'clock and a whole turn to 160 kt, a
#: number every twenty, and the caution speed the one red mark on the whole HUD, a
#: radial at the same nine o'clock the rest of the HUD's numbers are read from.
AIRSPEED_FULL_SCALE_KT = 160.0
AIRSPEED_CAUTION_KT = 120.0
AIRSPEED_TICKS = 16
AIRSPEED_NUMERALS = 8
AIRSPEED_NEEDLE_OUTER = 0.335


def _clamp_axis(axis, low=-1.0, high=1.0):
    """One control axis inside its own stops.

    The panel is drawn from four numbers a keyboard ratcheted, and no shape here
    should be able to leave its panel; the model clips its own axes the same way,
    so this is the belt to that pair of braces rather than the only clamp.
    """
    return low if axis < low else (high if axis > high else axis)


def pedal_line_heights(pedal_axis):
    """The two pedal lines' middles in the panel, ``(left, right)``, 0 to 1 up.

    The pedals are one axis with two ends, which is exactly what the mockup's two
    lines are: the right pedal goes forward as the left one comes aft, so the axis
    is the *difference* between them and the two lines move the same amount in
    opposite directions.  +1 is the right pedal forward - the key D, nose right -
    which raises the right hand line and lowers the left one by just as much, and
    0 leaves the two of them level at the middle, which is both pedals where they
    belong.
    """
    offset = 0.5 * PEDAL_LINE_TRAVEL * _clamp_axis(pedal_axis)
    return (0.5 - offset, 0.5 + offset)


def cyclic_dot_centre(lat_axis, long_axis):
    """The cyclic panel's disc as ``(x, y)`` panel fractions.

    A plan view of the stick, which is the view a pilot's own hand has of it:
    right is the lateral cyclic's +1 - a roll to starboard is the stick to the
    right - and the top of the panel is the longitudinal cyclic's, so a stick
    pushed forward, which puts the nose down, puts the disc up.
    """
    return (0.5 + 0.5 * CYCLIC_DOT_TRAVEL * _clamp_axis(lat_axis),
            0.5 + 0.5 * CYCLIC_DOT_TRAVEL * _clamp_axis(long_axis))


def collective_lever_deg(collective_axis):
    """The collective panel's lever, degrees above the panel's own horizontal.

    The green field is the lever's travel and its own lower edge is the down stop,
    so the lever lies along that edge with the collective down and along the
    field's upper edge at full up.  Between the two it is inside the field, because
    the field is the sector the lever sweeps rather than a quadrant it happens to
    sit in - and a pad start, at ``simulation.PAD_COLLECTIVE``, puts it well up
    inside the field rather than on that edge.
    """
    return QUADRANT_LOW_DEG + (QUADRANT_HIGH_DEG - QUADRANT_LOW_DEG) * _clamp_axis(
        collective_axis, 0.0, 1.0)


def attitude_ball_axes(roll_deg):
    """The ball's own right and up in panel fractions, ``(right, up)``.

    The ball turns under a fixed case, and it turns the way the world appears to
    the pilot when the aircraft rolls: the ball's up leaves the case's up by the
    roll itself, so a roll to starboard - the starboard wing down - turns the sky
    to port and lifts the starboard end of the horizon.  Both are unit vectors,
    which is what lets a chord be cut out of the ball with them.
    """
    angle = math.radians(roll_deg)
    return ((math.cos(angle), math.sin(angle)),
            (-math.sin(angle), math.cos(angle)))


def attitude_pitch_offset(pitch_deg):
    """The pitch as a distance on the ball, panel fractions, nose up positive.

    The ball is slid down the case by its pitch - a nose up puts the horizon
    below the middle of the dial, which is what a pilot seeing more sky through
    the windshield is looking at - so this is how far the ball's own zero pitch
    line is from where the case calls level.  Any line of the ball's own scale is
    the same distance for the angle it names, and at
    ``ATTITUDE_PITCH_DEG_PER_RADIUS`` degrees that distance is the ball's own
    radius, and so its rim.
    """
    return ATTITUDE_BALL_RADIUS * pitch_deg / ATTITUDE_PITCH_DEG_PER_RADIUS


def attitude_line_height(line_deg, pitch_deg, roll_deg):
    """How high on the ball one of its lines sits, panel fractions, up positive.

    *line_deg* is the angle the line names on the ball's own scale - 0 is the
    horizon itself, and a rung a whole 10 deg above it is 10 deg of the ball
    above it - and the ball's own slide is the pitch.  That slide is down the
    *case*, which is the ball's up only with the wings level, so a bank cuts the
    pitch's own share of it by the cosine of the roll: rolled onto its side, a
    helicopter's pitch moves the horizon across the dial rather than down it.
    """
    return (attitude_pitch_offset(line_deg)
            - attitude_pitch_offset(pitch_deg) * math.cos(math.radians(roll_deg)))


def attitude_line_ends(roll_deg, pitch_deg, line_deg, half_length):
    """One of the ball's lines, as its two ends in panel fractions.

    *half_length* is how long the line wants to be, half of it.  The line is
    straight and the ball is round, so what is drawn is the shorter of that and
    the chord the ball allows: the horizon, which wants the whole ball, gets the
    chord, and a ladder rung stays the short line it is meant to be.  ``None``
    comes back when the ball allows nothing at all, which is a line the pitch has
    carried off the dial.

    The middle of the line is the foot of the perpendicular dropped from the
    middle of the dial, which is where the ball's own scales put it, so the ends
    are ``middle - half * right`` and ``middle + half * right`` in that order.
    """
    right, up = attitude_ball_axes(roll_deg)
    height = attitude_line_height(line_deg, pitch_deg, roll_deg)
    allowed = (ATTITUDE_BALL_RADIUS * ATTITUDE_BALL_RADIUS - height * height)
    if allowed <= 0.0:
        return None
    half = min(half_length, math.sqrt(allowed))
    middle_x = ATTITUDE_CENTRE_X + height * up[0]
    middle_y = ATTITUDE_CENTRE_Y + height * up[1]
    return ((middle_x - half * right[0], middle_y - half * right[1]),
            (middle_x + half * right[0], middle_y + half * right[1]))


def attitude_ball_point(roll_deg, beta):
    """A point on the ball's own edge, *beta* radians from its up to its right.

    The sky and the ground are filled along these: the horizon's chord is where
    the two meet, the arc either side of it from ``-gamma`` to ``+gamma`` is the
    sky, and the rest of the circle - the long way round past the ball's own
    bottom - is the ground.
    """
    right, up = attitude_ball_axes(roll_deg)
    return (ATTITUDE_CENTRE_X + ATTITUDE_BALL_RADIUS
            * (math.cos(beta) * up[0] + math.sin(beta) * right[0]),
            ATTITUDE_CENTRE_Y + ATTITUDE_BALL_RADIUS
            * (math.cos(beta) * up[1] + math.sin(beta) * right[1]))


def attitude_horizon_gamma(roll_deg, pitch_deg):
    """The half angle the horizon's chord spans, radians, or ``None`` if it is off.

    The angle from the ball's own up to either end of the chord, so ``acos`` of
    the horizon's height in ball radii - and ``None`` when that height is a whole
    radius or more, which is a pitch past ``ATTITUDE_PITCH_DEG_PER_RADIUS`` and a
    ball that has lost its horizon and is all sky or all ground.
    """
    height = attitude_line_height(0.0, pitch_deg, roll_deg)
    if abs(height) >= ATTITUDE_BALL_RADIUS:
        return None
    return math.acos(height / ATTITUDE_BALL_RADIUS)


def attitude_ladder_half(line_deg):
    """How long a ladder rung wants to be, half of it, panel fractions.

    The first rung is ``ATTITUDE_LADDER_HALF`` of the ball's radius and every
    rung further out is ``ATTITUDE_LADDER_SHRINK`` shorter, which is the
    picture's own ladder: the further a line is from the horizon, the less of the
    ball it spans.
    """
    rungs = abs(line_deg) / ATTITUDE_LADDER_DEG
    return ATTITUDE_BALL_RADIUS * max(
        0.0, ATTITUDE_LADDER_HALF - ATTITUDE_LADDER_SHRINK * (rungs - 1.0))


def attitude_stroke_corners(x0, y0, x1, y1, half):
    """The four corners of the thin quad a straight line of the dial is drawn as.

    *half* is the line's own half thickness, laid across it rather than along it,
    so a stroke is as thick at one end as at the other.  ``None`` comes back for
    a stroke of no length, which has no direction to be across - and the corners
    are in the order the drawing walks them, which is what makes one loop enough
    for every line of the panel.
    """
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length <= 0.0:
        return None
    across_x = -dy / length * half
    across_y = dx / length * half
    return ((x0 + across_x, y0 + across_y), (x1 + across_x, y1 + across_y),
            (x1 - across_x, y1 - across_y), (x0 - across_x, y0 - across_y))


def attitude_reference_strokes():
    """The fixed aircraft reference, as its straight segments, panel fractions.

    The mockup's shape is two short bars either side of a small W, and it is the
    one part of the instrument that never moves: the ball turns and slides behind
    it.  Four segments are the W - down into a valley, up to the middle, down
    into the other valley, up - and two are the bars, level with the W's own
    peaks and ending well inside the ball, which is why they are drawn short.
    """
    middle_x = ATTITUDE_CENTRE_X
    peak_y = ATTITUDE_CENTRE_Y
    valley_x = 0.5 * ATTITUDE_BAR_INNER
    valley_y = peak_y - ATTITUDE_W_DEPTH
    return (
        ((middle_x - ATTITUDE_BAR_OUTER, peak_y),
         (middle_x - ATTITUDE_BAR_INNER, peak_y)),
        ((middle_x + ATTITUDE_BAR_INNER, peak_y),
         (middle_x + ATTITUDE_BAR_OUTER, peak_y)),
        ((middle_x - ATTITUDE_BAR_INNER, peak_y),
         (middle_x - valley_x, valley_y)),
        ((middle_x - valley_x, valley_y), (middle_x, peak_y)),
        ((middle_x, peak_y), (middle_x + valley_x, valley_y)),
        ((middle_x + valley_x, valley_y),
         (middle_x + ATTITUDE_BAR_INNER, peak_y)),
    )


def dial_point(centre_x, centre_y, radius, deg):
    """A point on a dial, *deg* clockwise from straight up, panel fractions.

    Every round face on this HUD is marked the same way: nought is the top of the
    dial and the angle grows clockwise, so a whole turn is a whole scale and a
    face's own numbers can be placed by the scale they read rather than from a
    table of angles.
    """
    angle = math.radians(deg)
    return (centre_x + radius * math.sin(angle),
            centre_y + radius * math.cos(angle))


def altimeter_needle_deg(alt_ft):
    """The two altimeter needles' angles, deg clockwise from straight up.

    ``(hundreds, thousands)``: the long thin needle reads the hundreds of feet
    and turns once in a thousand, the short thick one reads the thousands and
    turns once in ten thousand.  Both are straight up at zero, so the pair of
    them reads as one height rather than as two numbers.
    """
    hundreds = 360.0 * (alt_ft % 1000.0) / 1000.0
    thousands = 360.0 * (alt_ft % 10000.0) / 10000.0
    return (hundreds, thousands)


def vsi_needle_deg(height_rate_fpm):
    """The vertical speed needle's angle, deg clockwise from straight up.

    Nought is at nine o'clock and a climb turns the needle clockwise from there,
    fifty degrees of the dial to every thousand feet a minute, so the top of the
    scale is about two o'clock and the bottom about four.  Past either end of
    the scale the needle stops rather than running on round the dial.
    """
    rate = _clamp_axis(height_rate_fpm, -VSI_FULL_SCALE_FPM,
                       VSI_FULL_SCALE_FPM)
    return (VSI_ZERO_DEG + VSI_DEG_PER_1000_FPM * rate / 1000.0) % 360.0


def airspeed_needle_deg(airspeed_kt):
    """The airspeed needle's angle, deg clockwise from straight up.

    Nought is at twelve o'clock and the scale is a whole turn to 160 kt, so the
    needle is straight up both at a standstill and at the top of the scale, and
    the caution speed is a quarter turn round from it, at nine o'clock.
    """
    fraction = _clamp_axis(airspeed_kt / AIRSPEED_FULL_SCALE_KT, 0.0, 1.0)
    return 360.0 * fraction


def panel_box(index):
    """Panel *index*'s own rectangle in window pixels, ``(x0, y0, x1, y1)``.

    The panels are a HUD in the window's bottom left corner, square whatever the
    window's aspect is, and laid out as an L: the first :data:`PANEL_COLUMN_COUNT`
    of them up the left edge - the pedals at the top of the column and the
    attitude indicator at the bottom of it - and the rest along the bottom edge
    beside the column, the instruments to the right of it.  They are read rather
    than looked at, and the world behind them is what the window is for.
    """
    if index < PANEL_COLUMN_COUNT:
        # Up the column: the lowest index at the top of it, so the attitude
        # indicator - the last of the four - sits on the margin with the bottom
        # row and shares its own bottom edge with it.
        x0 = PANEL_MARGIN_PX
        y0 = PANEL_MARGIN_PX + (PANEL_COLUMN_COUNT - 1 - index) * (
            PANEL_PX + PANEL_GAP_PX)
    else:
        # Along the bottom row, one gap to the right of the column's own edge.
        x0 = (PANEL_MARGIN_PX + PANEL_PX + PANEL_GAP_PX
              + (index - PANEL_COLUMN_COUNT) * (PANEL_PX + PANEL_GAP_PX))
        y0 = PANEL_MARGIN_PX
    return (x0, y0, x0 + PANEL_PX, y0 + PANEL_PX)


def panel_point(box, x, y):
    """A point in a panel's own fractions - 0 to 1, up - as window pixels."""
    x0, y0, x1, y1 = box
    return (x0 + (x1 - x0) * x, y0 + (y1 - y0) * y)


def attitude_point(box, x, y):
    """A point of the dial in panel fractions as a window pixel, on a pixel middle.

    A window pixel is a square between two of the projection's own coordinates, so
    its middle is half a pixel in from any of its edges; :func:`panel_point` hands
    back the corner a fraction lands on and this shifts it to that middle.  It is
    what makes a line of one pixel a row or a column of pixels rather than a
    straddle of two - the dial's own lines are one pixel and the four panels' other
    shapes are several, so it is only the dial that has to care.
    """
    px, py = panel_point(box, x, y)
    return (px + 0.5, py + 0.5)


def draw_panel_background(box):
    """The mockup's black panel: one quad, a little translucent, and no border."""
    x0, y0, x1, y1 = box
    glColor4f(*PANEL_BACK_COLOUR)
    glBegin(GL_QUADS)
    glVertex2f(x0, y0)
    glVertex2f(x1, y0)
    glVertex2f(x1, y1)
    glVertex2f(x0, y1)
    glEnd()


def draw_panel_rect(box, centre_x, centre_y, half_x, half_y):
    """A filled rectangle in panel fractions: the mockup's two pedals are these.

    Nothing here is a thin line: a red line in the mockup is a long thin rectangle,
    which is what a pedal looks like from the side and what keeps it visible at a
    panel 132 px wide.
    """
    corners = ((centre_x - half_x, centre_y - half_y),
               (centre_x + half_x, centre_y - half_y),
               (centre_x + half_x, centre_y + half_y),
               (centre_x - half_x, centre_y + half_y))
    glBegin(GL_QUADS)
    for x, y in corners:
        glVertex2f(*panel_point(box, x, y))
    glEnd()


def draw_pedal_panel(box, pedal_axis):
    """The mockup's first panel: one red line for each pedal, moving oppositely."""
    left_y, right_y = pedal_line_heights(pedal_axis)
    glColor3f(*PANEL_RED)
    draw_panel_rect(box, PEDAL_LEFT_X, left_y, PEDAL_LINE_HALF_X,
                    PEDAL_LINE_HALF_Y)
    draw_panel_rect(box, PEDAL_RIGHT_X, right_y, PEDAL_LINE_HALF_X,
                    PEDAL_LINE_HALF_Y)


def draw_cyclic_panel(box, lat_axis, long_axis):
    """The mockup's second panel: the cyclic as a red disc where the stick is."""
    centre_x, centre_y = cyclic_dot_centre(lat_axis, long_axis)
    glColor3f(*PANEL_RED)
    glBegin(GL_TRIANGLE_FAN)
    glVertex2f(*panel_point(box, centre_x, centre_y))
    for index in range(CYCLIC_DOT_SEGMENTS + 1):
        angle = 2.0 * math.pi * index / CYCLIC_DOT_SEGMENTS
        glVertex2f(*panel_point(box,
                                centre_x + CYCLIC_DOT_RADIUS * math.cos(angle),
                                centre_y + CYCLIC_DOT_RADIUS * math.sin(angle)))
    glEnd()


def draw_collective_panel(box, collective_axis):
    """The mockup's third panel: the green field, and the red lever on it.

    The field is the lever's travel, drawn from the pivot out one triangle per
    segment of the arc between the two stops, so the green a pilot sees is exactly
    the sector the lever can be in and its own edges are the stops.  The lever is
    drawn last, on top, as a thick red line from the pivot to the arc at the angle
    the collective is at: the mockup's lever, and the whole of what this panel has
    to say.
    """
    glColor3f(*PANEL_GREEN)
    glBegin(GL_TRIANGLE_FAN)
    glVertex2f(*panel_point(box, QUADRANT_PIVOT_X, QUADRANT_PIVOT_Y))
    for index in range(QUADRANT_SEGMENTS + 1):
        angle = math.radians(QUADRANT_LOW_DEG
                             + (QUADRANT_HIGH_DEG - QUADRANT_LOW_DEG)
                             * index / QUADRANT_SEGMENTS)
        glVertex2f(*panel_point(
            box,
            QUADRANT_PIVOT_X + QUADRANT_RADIUS * math.cos(angle),
            QUADRANT_PIVOT_Y + QUADRANT_RADIUS * math.sin(angle)))
    glEnd()

    angle = math.radians(collective_lever_deg(collective_axis))
    # Across the lever rather than along it, so its thickness is the same at the
    # pivot and at the tip; the tip is on the field's own arc.
    across_x = -math.sin(angle) * QUADRANT_LEVER_HALF
    across_y = math.cos(angle) * QUADRANT_LEVER_HALF
    tip_x = QUADRANT_PIVOT_X + QUADRANT_RADIUS * math.cos(angle)
    tip_y = QUADRANT_PIVOT_Y + QUADRANT_RADIUS * math.sin(angle)
    glColor3f(*PANEL_RED)
    glBegin(GL_QUADS)
    for x, y in ((QUADRANT_PIVOT_X + across_x, QUADRANT_PIVOT_Y + across_y),
                 (tip_x + across_x, tip_y + across_y),
                 (tip_x - across_x, tip_y - across_y),
                 (QUADRANT_PIVOT_X - across_x, QUADRANT_PIVOT_Y - across_y)):
        glVertex2f(*panel_point(box, x, y))
    glEnd()


def draw_attitude_disc(box, centre_x, centre_y, radius, colour):
    """One filled circle of the dial, in panel fractions: a triangle fan.

    Its points are :func:`attitude_point`'s, on pixel middles like every other
    point of the dial, so a line cut to this edge at one pixel shares its edge
    rather than half a pixel over it.
    """
    glColor3f(*colour)
    glBegin(GL_TRIANGLE_FAN)
    glVertex2f(*attitude_point(box, centre_x, centre_y))
    for index in range(ATTITUDE_SEGMENTS + 1):
        angle = 2.0 * math.pi * index / ATTITUDE_SEGMENTS
        glVertex2f(*attitude_point(box, centre_x + radius * math.cos(angle),
                                   centre_y + radius * math.sin(angle)))
    glEnd()


def draw_attitude_stroke(box, x0, y0, x1, y1, half, colour):
    """One straight line of the dial, drawn as the quad that a line here is.

    A line of no length, which is a ladder rung at the edge of the dial, draws
    nothing at all rather than a degenerate quad.
    """
    corners = attitude_stroke_corners(x0, y0, x1, y1, half)
    if corners is None:
        return
    glColor3f(*colour)
    glBegin(GL_QUADS)
    for x, y in corners:
        glVertex2f(*attitude_point(box, x, y))
    glEnd()


def draw_attitude_line(box, roll_deg, pitch_deg, line_deg, half_length):
    """One green line of the ball: the horizon, or a rung of its own ladder.

    Green is the ball's own line and nothing else's on this dial: the aircraft's
    reference over it is white and the case around it is grey.
    """
    ends = attitude_line_ends(roll_deg, pitch_deg, line_deg, half_length)
    if ends is None:
        return
    draw_attitude_stroke(box, ends[0][0], ends[0][1], ends[1][0], ends[1][1],
                         ATTITUDE_LINE_HALF, ATTITUDE_GREEN)


def draw_attitude_ball(box, roll_deg, pitch_deg):
    """The ball's sky and ground, split by the horizon the attitude puts there.

    The ball is a circle and the horizon is a straight chord across it, so each
    of the two regions is a triangle fan from the middle of that chord: the sky
    is the fan round the arc that passes over the ball's own up, the ground the
    fan round the rest of the circle, and which side of the chord each is drawn
    on follows from the sign of the horizon's own height rather than from which
    of them looks right on the day.  ``None`` from
    :func:`attitude_horizon_gamma` is the pitch that has carried the horizon off
    the dial, and then the ball is one colour and one fan.
    """
    height = attitude_line_height(0.0, pitch_deg, roll_deg)
    gamma = attitude_horizon_gamma(roll_deg, pitch_deg)
    if gamma is None:
        draw_attitude_disc(box, ATTITUDE_CENTRE_X, ATTITUDE_CENTRE_Y,
                           ATTITUDE_BALL_RADIUS,
                           ATTITUDE_SKY_COLOUR if height < 0.0
                           else ATTITUDE_GROUND_COLOUR)
        return
    right, up = attitude_ball_axes(roll_deg)
    apex_x = ATTITUDE_CENTRE_X + height * up[0]
    apex_y = ATTITUDE_CENTRE_Y + height * up[1]
    # The sky over the ground: the first arc runs from the port end of the
    # horizon, over the ball's own top, to its starboard end, and the second the
    # long way round underneath, so the fans share the chord and no gap is left
    # between them for the case to show through.
    for colour, start, span in ((ATTITUDE_SKY_COLOUR, -gamma, 2.0 * gamma),
                                (ATTITUDE_GROUND_COLOUR, gamma,
                                 2.0 * (math.pi - gamma))):
        steps = max(1, int(math.ceil(ATTITUDE_SEGMENTS * span / (2.0 * math.pi))))
        glColor3f(*colour)
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(*attitude_point(box, apex_x, apex_y))
        for index in range(steps + 1):
            beta = start + span * index / steps
            glVertex2f(*attitude_point(box, *attitude_ball_point(roll_deg, beta)))
        glEnd()


def draw_attitude_panel(box, roll_deg, pitch_deg):
    """The mockup's fourth panel: the attitude indicator of attic/attitude.png.

    It is the model's own attitude rather than a hand's - the roll and pitch of
    :meth:`simulation.Simulation.telemetry` - drawn as the instrument they are:
    the case, the ball's sky and ground with the horizon and pitch ladder across
    them, and the aircraft's own reference fixed over the middle of all of it.

    The ball turns the way the world appears to a pilot who rolls and slides the
    way it appears to one who pitches, so the panel is a window on the attitude
    rather than a diagram of it - and the reference never moves at all, which is
    the whole of what makes the moving horizon readable.

    Nothing here reads a key, and nothing is drawn beyond the ball: the fill stops
    at the ball's own edge and every line of it is cut to the chord that edge
    allows, by :func:`attitude_line_ends`, rather than run out over the case.

    The order of the drawing is the order of the picture: the case, then the ball's
    two colours and the green horizon and ladder over them, then the white
    reference over the whole of it, so the bars and the W are never lost where one
    of the ball's own lines crosses them.
    """
    draw_attitude_disc(box, ATTITUDE_CENTRE_X, ATTITUDE_CENTRE_Y,
                       ATTITUDE_RADIUS, ATTITUDE_CASE_COLOUR)
    draw_attitude_ball(box, roll_deg, pitch_deg)
    # The horizon first, the whole visible width of the ball, and then the ladder
    # either side of it: each rung the shorter line the scale makes it, so the
    # further a line is from the horizon, the less of the ball it spans.
    draw_attitude_line(box, roll_deg, pitch_deg, 0.0, ATTITUDE_BALL_RADIUS)
    for rung in range(1, ATTITUDE_LADDER_RUNGS + 1):
        for sign in (1.0, -1.0):
            line_deg = sign * rung * ATTITUDE_LADDER_DEG
            draw_attitude_line(box, roll_deg, pitch_deg, line_deg,
                               attitude_ladder_half(line_deg))
    # And last the reference, over everything the ball says - over the green lines
    # it crosses as well - the two bars and the W between them, the one shape on
    # the panel that no attitude moves.
    for (x0, y0), (x1, y1) in attitude_reference_strokes():
        draw_attitude_stroke(box, x0, y0, x1, y1, ATTITUDE_LINE_HALF,
                             ATTITUDE_WHITE)


#: The instruments' own numbers and words are the only text on this HUD, so they
#: are the only thing here pygame draws: a string becomes a surface, the surface
#: becomes one texture, and the texture is a flat quad in the panel's own pixels.
#: A font is made once and kept and so is a texture - a face says the same handful
#: of words every frame - and the whole path is lazy, so none of it runs until
#: there is a window to draw it in.
HUD_FONTS = {}
HUD_TEXT_TEXTURES = {}


def hud_font(size_px):
    """pygame's own font at *size_px*, made once and kept."""
    if not pygame.font.get_init():
        pygame.font.init()
    font = HUD_FONTS.get(size_px)
    if font is None:
        font = pygame.font.Font(None, size_px)
        HUD_FONTS[size_px] = font
    return font


def hud_text_texture(text, size_px):
    """``(texture, width, height)`` for *text* at *size_px*, made once."""
    cached = HUD_TEXT_TEXTURES.get((text, size_px))
    if cached is None:
        surface = hud_font(size_px).render(text, True, (255, 255, 255))
        texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, surface.get_width(),
                     surface.get_height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     pygame.image.tostring(surface, "RGBA", True))
        cached = (texture, surface.get_width(), surface.get_height())
        HUD_TEXT_TEXTURES[(text, size_px)] = cached
    return cached


def draw_hud_text(box, x, y, text, size_px, colour):
    """*text* centred on panel fraction ``(x, y)``, in *size_px* pixels.

    The one thing this HUD draws that is not a shape: pygame renders the string
    to a surface, the surface is uploaded as a texture the first time that string
    is asked for, and the quad here is that texture's own size in the panel's
    pixels.  The surface's white is tinted by the colour set here, so one texture
    serves a white number and the red mark alike, and the texture is turned off
    again so that the next tick or needle drawn is a shape once more.
    """
    texture, width, height = hud_text_texture(text, size_px)
    centre_x, centre_y = panel_point(box, x, y)
    half_x, half_y = 0.5 * width, 0.5 * height
    glEnable(GL_TEXTURE_2D)
    glBindTexture(GL_TEXTURE_2D, texture)
    glColor4f(colour[0], colour[1], colour[2], 1.0)
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 0.0)
    glVertex2f(centre_x - half_x, centre_y - half_y)
    glTexCoord2f(1.0, 0.0)
    glVertex2f(centre_x + half_x, centre_y - half_y)
    glTexCoord2f(1.0, 1.0)
    glVertex2f(centre_x + half_x, centre_y + half_y)
    glTexCoord2f(0.0, 1.0)
    glVertex2f(centre_x - half_x, centre_y + half_y)
    glEnd()
    glDisable(GL_TEXTURE_2D)


def draw_dial_case(box):
    """One instrument's case: the attitude indicator's own grey disc."""
    draw_attitude_disc(box, INSTRUMENT_CENTRE_X, INSTRUMENT_CENTRE_Y,
                       INSTRUMENT_RADIUS, ATTITUDE_CASE_COLOUR)


def draw_dial_radial(box, deg, inner, outer, half, colour):
    """One radial line of a face, *deg* round it: a tick of its ring or a needle.

    It is a straight line of the same kind the attitude indicator's own are drawn
    with, so a tick and a needle share one piece of arithmetic and are as thick at
    one end as at the other.
    """
    x0, y0 = dial_point(INSTRUMENT_CENTRE_X, INSTRUMENT_CENTRE_Y, inner, deg)
    x1, y1 = dial_point(INSTRUMENT_CENTRE_X, INSTRUMENT_CENTRE_Y, outer, deg)
    draw_attitude_stroke(box, x0, y0, x1, y1, half, colour)


def draw_dial_text(box, deg, radius, text, size_px, colour):
    """A number or a word placed round a face, *radius* out from its middle."""
    x, y = dial_point(INSTRUMENT_CENTRE_X, INSTRUMENT_CENTRE_Y, radius, deg)
    draw_hud_text(box, x, y, text, size_px, colour)


def draw_altimeter_panel(box, alt_ft):
    """The altimeter of attic/altimeter.png: the height above the ground, in feet.

    The ring of ticks is one to each hundred feet, every tenth of them - one to
    each thousand - is the long one, the numbers nought to nine stand inside it,
    and the two needles are the long thin one on the hundreds and the short thick
    one on the thousands.  The short one is drawn first, so that the long one
    reads over it where the two of them cross, which is most of the dial.
    """
    draw_dial_case(box)
    for step in range(ALTIMETER_TICKS):
        deg = 360.0 * step / ALTIMETER_TICKS
        long_tick = step % (ALTIMETER_TICKS // ALTIMETER_NUMERALS) == 0
        draw_dial_radial(box, deg,
                         INSTRUMENT_TICK_LONG_INNER if long_tick
                         else INSTRUMENT_TICK_SHORT_INNER,
                         INSTRUMENT_TICK_OUTER, INSTRUMENT_TICK_HALF,
                         ATTITUDE_WHITE)
    for numeral in range(ALTIMETER_NUMERALS):
        draw_dial_text(box, 360.0 * numeral / ALTIMETER_NUMERALS,
                       INSTRUMENT_NUMERAL_RADIUS, str(numeral),
                       INSTRUMENT_NUMERAL_PX, ATTITUDE_WHITE)
    hundreds, thousands = altimeter_needle_deg(alt_ft)
    draw_dial_radial(box, thousands, 0.0, ALTIMETER_THOUSANDS_OUTER,
                     INSTRUMENT_NEEDLE_HALF, ATTITUDE_WHITE)
    draw_dial_radial(box, hundreds, 0.0, ALTIMETER_HUNDREDS_OUTER,
                     INSTRUMENT_THIN_NEEDLE_HALF, ATTITUDE_WHITE)
    draw_hud_text(box, 0.69, 0.50, "ALT", INSTRUMENT_LABEL_PX, ATTITUDE_WHITE)


def draw_vsi_panel(box, height_rate_fpm):
    """The vertical speed indicator of attic/vertspeed.png: the rate of climb.

    Nought is at nine o'clock and the needle swings clockwise for a climb and
    counter-clockwise for a descent, three thousand feet a minute either way - the
    reference face's own spread - and that face's words are on it: the
    instrument's name across the middle of the dial under the needle's own
    centre, and DOWN out at the side a descent takes the needle to.
    """
    draw_dial_case(box)
    # Ticks the whole length of the scale, a long one at each thousand feet - the
    # numbers' own positions - and the rest of them short.
    for step in range(VSI_TICK_STEPS + 1):
        rate = (step - 0.5 * VSI_TICK_STEPS) * VSI_TICK_FPM
        draw_dial_radial(box, vsi_needle_deg(rate),
                         INSTRUMENT_TICK_LONG_INNER
                         if rate % VSI_NUMERAL_FPM == 0.0
                         else INSTRUMENT_TICK_SHORT_INNER,
                         INSTRUMENT_TICK_OUTER, INSTRUMENT_TICK_HALF,
                         ATTITUDE_WHITE)
    for step in range(-VSI_NUMERAL_STEPS, VSI_NUMERAL_STEPS + 1):
        draw_dial_text(box, vsi_needle_deg(step * VSI_NUMERAL_FPM),
                       INSTRUMENT_NUMERAL_RADIUS, str(abs(step)),
                       INSTRUMENT_NUMERAL_PX, ATTITUDE_WHITE)
    draw_dial_radial(box, vsi_needle_deg(height_rate_fpm), 0.0,
                     VSI_NEEDLE_OUTER, INSTRUMENT_NEEDLE_HALF, ATTITUDE_WHITE)
    draw_hud_text(box, 0.5, 0.42, "VERTICAL SPEED", INSTRUMENT_SMALL_LABEL_PX,
                  ATTITUDE_WHITE)
    draw_hud_text(box, 0.22, 0.62, "DOWN", INSTRUMENT_LABEL_PX, ATTITUDE_WHITE)


def draw_airspeed_panel(box, airspeed_kt):
    """The airspeed indicator of attic/airspeed.png: the airspeed, in knots.

    Nought is at twelve o'clock and the scale is a whole turn to 160 kt, with a
    number every twenty both sides of it.  The caution speed is a red radial out at
    the rim - the one red mark on this HUD - and the needle is drawn after it, so
    that the needle reads over it wherever the two of them meet.
    """
    draw_dial_case(box)
    for step in range(AIRSPEED_TICKS):
        deg = 360.0 * step / AIRSPEED_TICKS
        long_tick = deg % (360.0 / AIRSPEED_NUMERALS) == 0.0
        draw_dial_radial(box, deg,
                         INSTRUMENT_TICK_LONG_INNER if long_tick
                         else INSTRUMENT_TICK_SHORT_INNER,
                         INSTRUMENT_TICK_OUTER, INSTRUMENT_TICK_HALF,
                         ATTITUDE_WHITE)
    for numeral in range(1, AIRSPEED_NUMERALS + 1):
        mark_deg = 360.0 * numeral / AIRSPEED_NUMERALS
        draw_dial_text(box, mark_deg, INSTRUMENT_NUMERAL_RADIUS,
                       str(20 * numeral), INSTRUMENT_NUMERAL_PX, ATTITUDE_WHITE)
    # The one red mark on the whole HUD: the caution speed, printed on the dial
    # out at the rim the numbers are read against rather than sweeping over them
    # like a needle, so that its own 120 stays legible under it.
    draw_dial_radial(box, airspeed_needle_deg(AIRSPEED_CAUTION_KT),
                     INSTRUMENT_TICK_LONG_INNER, INSTRUMENT_TICK_OUTER,
                     INSTRUMENT_NEEDLE_HALF, PANEL_RED)
    draw_dial_radial(box, airspeed_needle_deg(airspeed_kt), 0.0,
                     AIRSPEED_NEEDLE_OUTER, INSTRUMENT_THIN_NEEDLE_HALF,
                     ATTITUDE_WHITE)
    draw_hud_text(box, 0.5, 0.40, "KNOTS", INSTRUMENT_LABEL_PX, ATTITUDE_WHITE)


def draw_control_panel(sim):
    """The panels of the mockup, the attitude indicator and the instruments, over
    the world.

    The projection is switched to a pixel one and back, with the depth test, the
    fog and the culling off for the duration: a panel is a HUD, so nothing in the
    world is in front of it, no distance in the world fades it, and its shapes are
    flat quads whose winding is nobody's business.  Blend stays on, so the panels
    are a little translucent and the world shows through them - they are for
    reading a key's effect at a glance, not for hiding the aircraft.

    The four control numbers drawn are :meth:`simulation.PilotInput.axes` and
    nothing else: the hands, which is what the caption's own control positions are
    read from too, so the window says the same thing twice in two languages.  The
    attitude indicator and the three instruments are the panels that cannot be
    drawn from a hand at all, so they are drawn from
    :meth:`simulation.Simulation.telemetry` - the aircraft's roll and pitch, its
    height above the ground, its rate of climb and its airspeed, which is what
    the instruments are for.
    """
    collective, long_stick, lat_stick, pedal = sim.pilot.axes()
    attitude = sim.telemetry()

    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0.0, WIDTH, 0.0, HEIGHT)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    glDisable(GL_DEPTH_TEST)
    glDisable(GL_FOG)
    glDisable(GL_CULL_FACE)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    boxes = [panel_box(index) for index in range(PANEL_COUNT)]
    for box in boxes:
        draw_panel_background(box)
    draw_pedal_panel(boxes[PANEL_PEDALS], pedal)
    draw_cyclic_panel(boxes[PANEL_CYCLIC], lat_stick, long_stick)
    draw_collective_panel(boxes[PANEL_COLLECTIVE], collective)
    draw_attitude_panel(boxes[PANEL_ATTITUDE], attitude.roll_deg,
                        attitude.pitch_deg)
    draw_altimeter_panel(boxes[PANEL_ALTIMETER], attitude.alt_ft)
    draw_vsi_panel(boxes[PANEL_VSI], attitude.height_rate_fpm)
    draw_airspeed_panel(boxes[PANEL_AIRSPEED], attitude.airspeed_kt)

    glDisable(GL_BLEND)
    glEnable(GL_CULL_FACE)
    glEnable(GL_FOG)
    glEnable(GL_DEPTH_TEST)

    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)


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

    *keys* is anything indexable by a key constant: the frame loop's own record
    of the keys the events left down, and a plain dict in the check at the
    bottom of this file.  pygame 2 indexes ``pygame.key.get_pressed()`` by
    *scancode*, so the keys below are read as the KSCAN_* constants - the
    physical key positions - which also means that a keyboard whose layout
    moves the letters does not move the controls.
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


#: The eight keys the aircraft is flown from, by scancode: the keys
#: :func:`pilot_keys` reads, and so the keys a frame's own event handling has to
#: keep track of.
FLIGHT_SCANCODES = (pygame.KSCAN_W, pygame.KSCAN_S,
                    pygame.KSCAN_UP, pygame.KSCAN_DOWN,
                    pygame.KSCAN_LEFT, pygame.KSCAN_RIGHT,
                    pygame.KSCAN_A, pygame.KSCAN_D)

#: What the key log calls each of the eight flight keys: the physical key's own
#: name, since that is the position the four axes are read from, so that a
#: keyboard whose layout moves the letters does not move the names in the log.
KEY_NAMES = {pygame.KSCAN_W: "W", pygame.KSCAN_S: "S",
             pygame.KSCAN_UP: "UP", pygame.KSCAN_DOWN: "DOWN",
             pygame.KSCAN_LEFT: "LEFT", pygame.KSCAN_RIGHT: "RIGHT",
             pygame.KSCAN_A: "A", pygame.KSCAN_D: "D"}


def frame_keys(pressed, tapped):
    """One frame's keys: what is held, plus what was tapped inside the frame.

    *pressed* is any mapping of scancode to truth - the frame loop's own record
    of the keys the events left down, and a plain dict in the check - and
    *tapped* is the scancodes that went down at some point in the frame.  A tap
    shorter than a frame is pressed and released between two samples of the
    keyboard, and the collective is a ratchet: a tap thrown away is travel
    thrown away, so a tapped flight key counts as held for the frame it
    happened in.

    The result holds the flight keys only, in the 1.0 and 0.0 that
    :func:`pilot_keys` reads; the command keys are the event loop's own.
    """
    keys = dict((scancode, _held(pressed, scancode))
                for scancode in FLIGHT_SCANCODES)
    for scancode in tapped:
        if scancode in FLIGHT_SCANCODES:
            keys[scancode] = 1.0
    return keys


def key_name(event):
    """A key event's own name, for the key log's ``key`` column.

    The eight flight keys are named by position, through :data:`KEY_NAMES` and
    the ``KSCAN_*`` constants :func:`pilot_keys` reads, so their lines do not
    move when a keyboard's layout does; every other key - the command keys
    among them - is named the way pygame names it, which is what the event loop
    reads them by anyway.
    """
    if event.scancode in KEY_NAMES:
        return KEY_NAMES[event.scancode]
    return pygame.key.name(event.key) or str(event.key)


def held_names(keys):
    """The flight keys down in *keys*, for the key log's ``held`` column.

    In :data:`FLIGHT_SCANCODES` order and by the position names of
    :data:`KEY_NAMES`, so the column reads like the two axes it means - a
    collective key and a cyclic key held together are ``W+RIGHT`` - with ``-``
    for a hand that is on nothing at all.  Command keys are the event loop's
    own and are never in it.
    """
    names = [KEY_NAMES[scancode] for scancode in FLIGHT_SCANCODES
             if _held(keys, scancode)]
    return "+".join(names) or "-"


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


def window_title(sim, view, focused=True):
    """The window's caption: what a HUD would show, the view, the keyboard.

    The four control positions are on it in the model's own units - the
    collective in per cent of its travel, the two cyclic sticks and the
    pedals in inches - because this caption is the sandbox's only instrument.
    The collective alone would leave a key that the aircraft cannot show
    looking like a key that does nothing: on the pad the skids hold the
    aircraft level and still, so the cyclic and the pedals move it by nothing
    at all until it is off the ground.

    The end of a flight is on it in the model's own words, which are the two of
    :data:`simulation.CRASH_MESSAGES`: ``CRASHED`` for the skids arriving on the
    ground over ``UH1_HARD_LANDING_MPS``, and ``OUT OF ENVELOPE`` for a state
    the model refused to fly - which is an overload, happens in the air, and is
    what the altitude beside it is the altitude *of*.  One word for both would
    report that refusal as a landing the aircraft never made.

    *focused* is :func:`pygame.key.get_focused`, and while it is false the
    window is not being sent keys at all: all eight then move nothing, which
    is the one case no control position can tell from a dead key.  Saying so
    is cheaper than answering it, and it names the cure - a click.  It goes
    next to the aircraft's name, at the front of the caption, because a title
    bar is as long as the window and the end of the line can be cut off.
    """
    telemetry = sim.telemetry()
    return ("HeliSim - UH-1H%s | alt %6.1f m %+6.0f fpm | %5.1f kt"
            " | coll %3.0f %% | cyc %+6.2f/%+6.2f in | ped %+5.2f in%s%s%s"
            " | %s view"
            % ("" if focused else " | no keyboard: click the window",
               telemetry.alt_agl, telemetry.height_rate_fpm,
               telemetry.airspeed_kt, 100.0 * telemetry.collective_fraction,
               telemetry.long_stick_in, telemetry.lat_stick_in,
               telemetry.pedal_in,
               " | trim" if telemetry.in_trim else "",
               " | on the ground" if telemetry.on_ground else "",
               (" | " + telemetry.crash_message) if telemetry.crashed else "",
               view))


# ---------------------------------------------------------------------------
# The key log: every keystroke and the flight it was made in, beside the run.
# ---------------------------------------------------------------------------


def flight_columns(sim):
    """The instant a key log line is about: velocity, attitude, controls.

    The aircraft's own numbers, in the units the model works in: velocity as
    :meth:`simulation.Telemetry`'s airspeed through the airframe and the ground
    velocity in NED - north, east and down, the velocity that carries the
    aircraft across the map - then heading, pitch, roll and altitude in degrees
    and metres, and the pilot's four controls in the inches of travel the
    caption shows them in, with the collective as a fraction of its own travel
    beside its own inches.

    Nothing here is a hand's own reading and nothing is the log's invention:
    these are the aircraft's telemetry, so a line stands on its own whether the
    key that wrote it was a flight key or a command key - a ``park`` line is
    read the same way as one written in mid-flight.
    """
    telemetry = sim.telemetry()
    ground = sim.airframe.state.ground_velocity()
    return (telemetry.airspeed, ground.x, ground.y, ground.z,
            telemetry.yaw_deg, telemetry.pitch_deg, telemetry.roll_deg,
            telemetry.alt_agl, telemetry.collective_in, telemetry.long_stick_in,
            telemetry.lat_stick_in, telemetry.pedal_in,
            telemetry.collective_fraction)


class KeyLog:
    """A run's log: every keystroke and a sample of the flight, one line each.

    Why a file at all: the caption and the panels show one instant and forget
    it, so a flight that went wrong is gone by the time anybody reads it.  A
    line of this log is that instant written down - what the aircraft was doing
    and where the pilot's hands were - and the log together is the two halves of
    a session: what was pressed, and what the aircraft did about it.

    The clocks, because *when* is most of what makes a log a log:

    * ``t_s`` is :func:`time.perf_counter` seconds since the log was opened,
      written to a millisecond.  It is monotonic, so a flight's own timeline
      cannot jump backwards when the wall clock is corrected mid-session, and it
      is read beside the model's own ``frame`` and ``sim_s``.
    * ``wall_ms`` is that same instant as :func:`time.time` milliseconds, also
      to a millisecond, so a line can be read against anything else that
      happened at that time of day.

    Both are stamped where the line is written.  For a keystroke that is where
    the event is taken out of pygame's queue - the loop drains the queue before
    it flies the frame - so a key's line carries the state the key was pressed
    in and the ``sample`` lines after it are what the frame then did with it.
    The stamp is therefore up to a frame later than the press itself; the
    physics is stepped in whole 1/60 s steps (``simulation.SIM_TIME_STEP_S``),
    so nothing finer than that is a time the aircraft could have shown anyway.

    The kinds of line are:

    ``down`` / ``up``      a key the aircraft is flown from, or any other
    ``reset`` / ``park``   the R and P keys, logged once their state is in
    ``view`` / ``quit``    the C and Esc keys
    ``focus`` / ``blur``   the window gaining and losing the keyboard
    ``sample``             the flight itself, ``rate_hz`` times a second

    and the ``key`` column is the key's own name while ``held`` is the flight
    keys down at that instant, so a key the aircraft cannot show - one pressed
    on the pad, or a cyclic key with the skids down - is in the log as itself.
    """

    #: The columns, in the order they are written: the file's own header row.
    COLUMNS = ("t_s", "wall_ms", "frame", "sim_s", "kind", "key", "held",
               "spd_mps", "vn_mps", "ve_mps", "vd_mps", "hdg_deg", "pitch_deg",
               "roll_deg", "alt_m", "coll_in", "long_in", "lat_in", "pedal_in",
               "coll_frac")

    def __init__(self, stream, rate_hz=KEYLOG_RATE_HZ):
        self.stream = stream
        self.rate_hz = float(rate_hz)
        self.period = 1.0 / self.rate_hz
        self.origin = time.perf_counter()
        self.origin_wall = time.time()
        self.last_sample = None
        self.entries = 0
        self._write_header()

    @classmethod
    def open(cls, path, rate_hz=KEYLOG_RATE_HZ):
        """A key log writing to *path*, buffered one line at a time.

        Line buffered, so the one line a crash can cost is the line it
        interrupted: everything before it is already on disk.
        """
        return cls(io.open(path, "w", encoding="utf-8", buffering=1), rate_hz)

    def close(self):
        """Finish the file: the last lines, and the handle itself."""
        self.stream.flush()
        self.stream.close()

    def _write_header(self):
        self.stream.write(
            "# HeliSim key log, written by main.py: one line per keystroke and"
            " one per sample of the flight.\n"
            "# t_s is time.perf_counter seconds since this log was opened,"
            " monotonic; wall_ms is the same instant as time.time milliseconds."
            "  Both to a millisecond.\n"
            "# kind is down/up for a key, reset/park/view/quit for the four"
            " command keys, focus/blur for the keyboard, sample for the flight;"
            " key is the key itself or - on a sample.\n"
            "# held is the flight keys down at that instant.  Velocity is m/s"
            " (airspeed, then north/east/down over the ground), attitude is"
            " degrees, altitude is m,\n"
            "# and the four controls are inches of travel, with the collective"
            " as a fraction of its own.\n"
            + ",".join(self.COLUMNS) + "\n")

    def _stamp(self):
        """``(t_s, wall_ms)``: now, on both of the log's own clocks."""
        elapsed = time.perf_counter() - self.origin
        return elapsed, (self.origin_wall + elapsed) * 1000.0

    def _write(self, kind, key, held, sim, t_s, wall_ms):
        fields = [("%.3f" % t_s), ("%.3f" % wall_ms), str(sim.frames),
                  ("%.4f" % sim.sim_time), kind, key, held]
        fields.extend("%.3f" % value for value in flight_columns(sim))
        self.stream.write(",".join(fields) + "\n")
        self.entries += 1

    def key(self, sim, kind, key, held):
        """One keystroke - or one change of the keyboard - in the file.

        *kind* is what the loop did with it (``down``, ``reset``, ``blur``...)
        and the state written down is the one standing at that instant: the
        event loop logs a command key after its own effect, so a ``park`` line
        is the aircraft already on the pad.
        """
        t_s, wall_ms = self._stamp()
        self._write(kind, key, held, sim, t_s, wall_ms)
        return t_s

    def sample(self, sim, held):
        """The flight itself, at most ``rate_hz`` of these a second.

        The rate is its own clock rather than every Nth frame, so a dragged
        frame is a longer gap between two lines and not a missing one.  The
        first call always writes, which is what puts the run's own starting
        state at the top of the file.  True when a line was written.
        """
        t_s, wall_ms = self._stamp()
        if self.last_sample is not None and t_s - self.last_sample < self.period:
            return False
        self.last_sample = t_s
        self._write("sample", "-", held, sim, t_s, wall_ms)
        return True


def log_key(log, sim, kind, event, keyboard):
    """One keystroke - or one change of the keyboard - into the key log.

    *event* is the pygame event, or None for a line that is not a key at all
    (the window gaining and losing the keyboard, which the loop logs itself),
    and *keyboard* is the frame loop's own record of what is held, so the
    ``held`` column is the hand as it stood when the line was written.  A run
    that is flying with ``--no-log`` has no log, and this then does nothing.
    """
    if log is None:
        return
    log.key(sim, kind, "-" if event is None else key_name(event),
            held_names(keyboard))


def command_line(args):
    """``(scenery path, key log path)`` from the command line.

    The scenery file is the one bare argument, ``sample_scenery.xml`` when there
    is none, exactly as it always was; the key log has two options of its own:
    ``--log FILE`` writes it to *FILE* and ``--no-log`` flies without one, which
    leaves the second half of the pair None.  ``--log`` with no file after it
    names nothing, so the default stands, and a file named beats ``--no-log``,
    since it is the option that says where the log goes.

    ``--check`` never reaches here: it is read at the bottom of the file, before
    ``main()`` is called at all.
    """
    args = list(args)
    log_path = DEFAULT_KEYLOG_FILE
    if "--no-log" in args:
        args.remove("--no-log")
        log_path = None
    if "--log" in args:
        index = args.index("--log")
        if index + 1 < len(args):
            log_path = args[index + 1]
            del args[index:index + 2]
        else:
            del args[index]
    return (args[0] if args else DEFAULT_SCENERY_FILE), log_path


def main():
    global SCENERY

    path, log_path = command_line(sys.argv[1:])
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
    print("  R resets to the state this run began in - the pad, the lever at %d %% -"
          % (100.0 * PAD_COLLECTIVE,))
    print("  and the world is %.0f km across"
          % (2.0 * GROUND_HALF_M / 1000.0,))
    print("keys: W/S collective, arrows cyclic, A/D pedals, R reset, P park,"
          " C camera, Esc quit")
    print("  and the panels up the window's left edge are the pilot's controls:")
    print("  two red lines for the pedals, a red disc for the cyclic, and a red")
    print("  lever for the collective in a green field - the mockup %s"
          % (PANEL_MOCKUP_FILE,))
    print("  under them an attitude indicator, the aircraft's own roll and pitch")
    print("  on a moving horizon, blue over brown with green lines, and its")
    print("  reference fixed over its middle - %s" % (PANEL_ATTITUDE_FILE,))
    print("  and along the bottom of the window, on the same grey case, the three")
    print("  instruments: the altimeter, the vertical speed indicator and the")
    print("  airspeed indicator - the height, the rate of climb and the airspeed,")
    print("  with one red radial at 120 kt - %s, %s and %s"
          % (PANEL_ALTIMETER_FILE, PANEL_VSI_FILE, PANEL_AIRSPEED_FILE))

    # The run's key log, opened before there is a window: its clock starts here,
    # and its first line is the state this run is beginning from, so a session
    # that dies in the renderer still says where it started.
    log = KeyLog.open(log_path) if log_path else None
    if log is not None:
        log.sample(sim, held_names({}))       # a hand on nothing, at t = 0
        print("key log: %s" % (log_path,))
        print("  one line per keystroke and %g a second of the flight, each with"
              " the aircraft's" % (KEYLOG_RATE_HZ,))
        print("  velocity, heading, pitch, roll, altitude and the four control"
              " positions; --no-log flies without one")

    camera = ChaseCamera()
    view = VIEW_CHASE

    pygame.init()
    pygame.display.set_mode((WIDTH, HEIGHT), DOUBLEBUF | OPENGL)
    resize(WIDTH, HEIGHT)
    init_gl()
    pygame.display.set_caption(window_title(sim, view))

    clock = pygame.time.Clock()
    running = True
    # The keys of every frame, straight from the events rather than sampled from
    # pygame.key.get_pressed(): which are down, and which went down inside the
    # frame and were let go again before it ended.  A press shorter than a frame
    # is the one a ratchet must not lose, and the event queue never misses one.
    keyboard = {}
    keyboard_was = pygame.key.get_focused()
    if not keyboard_was:
        print("  no keyboard: click the window, or the keys go nowhere")
    while running:
        tapped = set()
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    log_key(log, sim, "quit", event, keyboard)
                    running = False
                elif event.key in (pygame.K_r, pygame.K_p):
                    # R is Simulation.reset and P is on_the_pad.  Neither jumps
                    # to a control position the pilot has not made: R comes back
                    # to the state and the stick positions this run began with -
                    # here the pad, since main() starts parked - so pressing it
                    # on a freshly started aircraft changes nothing.  Both put
                    # the pilot's axes where the aircraft already is, and both
                    # need the camera to snap onto it rather than fly it across
                    # the map.
                    resetting = event.key == pygame.K_r
                    if resetting:
                        sim.reset()
                    else:
                        sim.on_the_pad()
                    camera.reset()
                    # Logged after the reset, so the line is the state the key
                    # produced and not the one it was pressed in.
                    log_key(log, sim, "reset" if resetting else "park", event,
                            keyboard)
                    print("  %s: %s"
                          % ("reset to the run's start" if resetting
                             else "parked on the pad", sim.telemetry()))
                elif event.key == pygame.K_c:
                    view = VIEWS[(VIEWS.index(view) + 1) % len(VIEWS)]
                    camera.reset()
                    log_key(log, sim, "view", event, keyboard)
                    print("  view: %s" % (view,))
                else:
                    keyboard[event.scancode] = True
                    tapped.add(event.scancode)
                    # The keys the aircraft is flown from, and any other key the
                    # four axes do not read: the log has both, since a key that
                    # flies nothing is still a key a hand pressed.
                    log_key(log, sim, "down", event, keyboard)
            elif event.type == KEYUP:
                keyboard[event.scancode] = False
                log_key(log, sim, "up", event, keyboard)

        # The frame's own time, whatever it turned out to be: the physics inside
        # Simulation.step is always 1/60 s steps, so a dragged window is a longer
        # frame rather than a faster aircraft.
        frame_dt = clock.tick(60) / 1000.0
        focused = pygame.key.get_focused()
        if focused != keyboard_was:
            if not focused:
                # A key held when the window loses the keyboard never gets its
                # KEYUP, so let go of all of them rather than fly with them.
                keyboard.clear()
            log_key(log, sim, "focus" if focused else "blur", None, keyboard)
            print("  keyboard: %s" % ("the window has it" if focused
                                      else "click the window, it has not"))
            keyboard_was = focused
        fly_frame(sim, camera, frame_dt, frame_keys(keyboard, tapped), view)
        draw_scene(sim, camera, view)
        pygame.display.flip()

        # One line of the flight between the keystrokes: the state that frame
        # left behind, on the log's own sample rate rather than on every frame.
        if log is not None:
            log.sample(sim, held_names(keyboard))

        # The caption is the HUD's other half: the control position panel is drawn
        # into the frame itself, and the same four positions are here in the
        # model's own units.  It is cheap enough to set a few times a second
        # rather than on every frame.
        if sim.frames % 15 == 0:
            pygame.display.set_caption(window_title(sim, view, focused))
        if sim.frames % 60 == 0:
            print("  " + str(sim.telemetry()))

    if log is not None:
        log.close()
        print("  key log: %d line(s) in %s" % (log.entries, log_path))
    pygame.quit()
    sys.exit()


# ---------------------------------------------------------------------------
# The check, in the style of the modules underneath: assertions, no window.
# ---------------------------------------------------------------------------


def _self_check():
    """The wiring of this file, headless: keys, clock, camera, reset keys, log.

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

    # A frame's keys: the eight keys of the four axes, the ones the events leave
    # down, and a tap that was over before the frame's state could be sampled.
    # Both are the same shape to pilot_keys - scancodes to 1.0 and 0.0 - and a
    # tap has to count for its frame, or the collective ratchet loses it.
    assert len(FLIGHT_SCANCODES) == 8
    for scancode in FLIGHT_SCANCODES:
        assert pilot_keys(frame_keys({}, {scancode})) != pilot_keys({})
    assert pilot_keys(frame_keys({}, {pygame.KSCAN_W}))["collective"] == 1.0
    assert pilot_keys(frame_keys({pygame.KSCAN_S: True},
                                 {pygame.KSCAN_W}))["collective"] == 0.0
    assert pilot_keys(frame_keys({}, set())) == pilot_keys({})
    # R is a command key, not a flight key: tapping one flies nothing.
    assert pilot_keys(frame_keys({}, {pygame.KSCAN_R})) == pilot_keys({})
    assert pilot_keys(frame_keys({pygame.KSCAN_RIGHT: True},
                                 {pygame.KSCAN_W}))["lat_stick"] == 1.0

    # A fresh sandbox starts the way main() leaves it: parked on the pad, the
    # lever at the 75 % a park leaves it at (simulation.PAD_COLLECTIVE, 0.84 in
    # under this run's 9.09 in hover trim), level, with the
    # run's own trim remembered as the reference the caption's light reads and
    # that park remembered as the state this run began in - which is what R comes
    # back to.
    sim = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    sim.on_the_pad()
    assert sim.on_ground and not sim.crashed
    assert abs(sim.airframe.state.altitude - UH1_CG_HEIGHT_ON_GROUND) < 1e-9
    assert sim.pilot.collective_axis == PAD_COLLECTIVE
    assert sim.controls.collective_fraction == PAD_COLLECTIVE
    assert sim.trim_controls is not None and sim.trim_state is not None
    assert sim.start_state is not None and sim.start_controls is not None
    pad_start = sim.airframe.state.values()

    camera = ChaseCamera()
    # The collective key lifts it off the skids: simulation.py's own demo pickup,
    # now through this file's frame loop, key mapping and camera.  The lever is
    # already most of the way to the trim - 75 % against the 82.6 % this aircraft
    # hovers at, so half a second of key rather than the seven the whole travel
    # takes - and what follows it is the model's lightly damped heave, so what is
    # checked is that it leaves the ground and gets clear of the skid line on the
    # way.
    airborne = False
    highest = 0.0
    trim_fraction = sim.trim_controls.collective_fraction
    for _ in range(int(12.0 / SIM_TIME_STEP_S)):
        keys = ({pygame.KSCAN_W: True}
                if sim.pilot.collective_axis < trim_fraction else {})
        fly_frame(sim, camera, SIM_TIME_STEP_S, keys)
        airborne = airborne or not sim.on_ground
        highest = max(highest, sim.airframe.state.altitude)
        assert math.isfinite(sim.airframe.state.altitude)
    assert airborne and highest > UH1_CG_HEIGHT_ON_GROUND + 0.5, highest
    assert not sim.crashed and math.isfinite(sim.telemetry().thrust)

    # R: reset, which comes back to where the *run* began and not to where the
    # trim says the aircraft belongs.  This run began on the pad, so that is
    # where R puts it - at any moment, and at the first one too, which is what a
    # person does to a flight model first.  Nothing of the pickup above is left
    # in it, and the collective nobody has touched is not handed to the pilot's
    # hands either: the hover trim's 9.09 in of collective are where the *trim*
    # is, not where the reset is, and the caption shows the pad's own 75 % with
    # neither the trim light nor the crashed flag and the aircraft on the ground
    # where it started.  The cyclic and the pedals come back to the stick the pad
    # left them on, which is the trim's own position - the one they were already
    # in, so a reset is no step for them either - and the caption reads them out.
    # Every axis is a ratchet, so the hands stay exactly there afterwards.
    sim.reset()
    camera.reset()
    assert sim.airframe.state.values() == pad_start, sim.telemetry()
    assert sim.pilot.axes() == sim.controls.to_axes()
    assert sim.controls.collective_fraction == PAD_COLLECTIVE
    # The hands are on the pad's own stick, so those are the inches they read:
    # the same three the caption below has to be showing.
    assert sim.controls.long_stick == sim.trim_controls.long_stick
    assert sim.controls.lat_stick == sim.trim_controls.lat_stick
    assert sim.controls.pedal == sim.trim_controls.pedal
    assert sim.pilot.collective_axis == PAD_COLLECTIVE
    assert sim.on_ground and not sim.crashed and not sim.in_trim()
    assert sim.sim_time == 0.0 and sim.frames == 0 and sim.steps == 0
    caption = window_title(sim, VIEW_CHASE)
    assert "| trim" not in caption and "| on the ground" in caption
    assert "| coll  75 % |" in caption          # 100 * PAD_COLLECTIVE, whole per cent
    assert ("| cyc %+6.2f/%+6.2f in | ped %+5.2f in"
            % (sim.controls.long_stick, sim.controls.lat_stick,
               sim.controls.pedal)) in caption
    # And pressing it again, which is the case a new pilot is in, changes nothing
    # either: the reset is idempotent, so it is the one key that cannot surprise.
    sim.reset()
    assert sim.airframe.state.values() == pad_start
    # The hands go back to the same stick the pad left them on - the trim's own
    # cyclic and pedals, which is where on_the_pad put them - so nothing springs
    # back and the reset is idempotent for them as well.  The lever comes back to
    # the 75 % it was parked at; the three sticks stay where they were, which is
    # why this is not four axes at the rigging's zero.
    settled = sim.pilot.axes()
    assert settled == sim.controls.to_axes()
    assert settled[0] == PAD_COLLECTIVE
    # Hands off from there it stays where the run started: the skids hold it and
    # no axis moves, so a pilot who presses R and lets go gets their pad back
    # rather than a helicopter leaving a hover they never asked for.
    for _ in range(120):
        fly_frame(sim, camera, SIM_TIME_STEP_S, {})
    state = sim.airframe.state
    assert sim.frames == 120 and sim.steps == 120
    assert abs(sim.sim_time - 2.0) < 1e-9      # one clock, and it is the model's
    assert sim.on_ground and not sim.crashed
    assert abs(state.altitude - UH1_CG_HEIGHT_ON_GROUND) < 1e-9
    assert state.position.x == 0.0 and state.position.y == 0.0  # it did not move
    assert sim.pilot.axes() == settled                 # nothing pulled it up off

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
    # A window without the keyboard is not a window with four dead keys: the
    # caption says which it is, and it is the only place that can.
    assert "no keyboard" not in window_title(sim, VIEW_CHASE)
    assert "no keyboard" in window_title(sim, VIEW_CHASE, focused=False)
    assert "no keyboard" in window_title(sim, VIEW_PAD, focused=False)
    # And it is at the front, where a title bar cannot cut it off.
    assert window_title(sim, VIEW_CHASE, focused=False).startswith(
        "HeliSim - UH-1H | no keyboard")
    assert len(load_scenery(DEFAULT_SCENERY_FILE)) > 0

    # The caption names which end a flight came to, and the two are deliberately
    # not the same word.  A hard arrival on the skids is a crash: the aircraft is
    # on the ground, and the caption says both.  A state the model refuses to fly
    # is out of its envelope: nothing touched anything, and an `OUT OF ENVELOPE`
    # beside an altitude is the altitude it was refused at - where it still is,
    # since a refused frame puts the last good state back.  No number of frames
    # moves it again, and only R clears either.
    hit = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    hit.reset(altitude=UH1_CG_HEIGHT_ON_GROUND + 3.0,
              controls=hit.trim_controls.collective_down())
    for _ in range(120):
        hit.step(SIM_TIME_STEP_S)
    assert hit.crashed and hit.on_ground
    caption = window_title(hit, VIEW_CHASE)
    assert " | on the ground | CRASHED |" in caption, caption
    refused = Simulation(airframe=airframe_preset(AIRFRAME_PRESET))
    refused.reset(altitude=30.0,
                  controls=refused.trim_controls.collective_down())
    for _ in range(600):
        refused.step(SIM_TIME_STEP_S)
        if refused.crashed:
            break
    assert refused.crashed and not refused.on_ground
    assert refused.airframe.state.altitude > 1.0     # nowhere near the ground
    frozen = refused.airframe.state.values()
    for _ in range(120):
        refused.step(SIM_TIME_STEP_S)
    assert refused.airframe.state.values() == frozen
    caption = window_title(refused, VIEW_CHASE)
    assert " | OUT OF ENVELOPE |" in caption, caption
    assert "CRASHED" not in caption and "on the ground" not in caption
    assert not refused.in_trim()

    # The control position panel: each of the mockup's three panels is drawn from
    # the pilot's own axes through one small function of its own, so the arithmetic
    # of the whole HUD can be checked without a window and without an OpenGL
    # context.  What is checked is the shape of it: the pedal lines move oppositely
    # about a level middle and stay on the panel, the cyclic disc is the panel's
    # middle at the stick's own middle and stays on the panel at a stop, and the
    # collective lever is on the field's lower edge at the down stop and on its
    # upper edge at full up - which is what makes the green field the travel
    # rather than a quadrant the lever happens to sit in.  A pad start is the case
    # in between, three quarters of the way up it.
    assert pedal_line_heights(0.0) == (0.5, 0.5)         # both pedals where they belong
    for pedal in (0.0, 0.25, -0.8, 1.0, -1.0):
        left_y, right_y = pedal_line_heights(pedal)
        assert abs(left_y + right_y - 1.0) < 1e-12       # one middle, two ways
        assert abs(abs(right_y - left_y) - PEDAL_LINE_TRAVEL * abs(pedal)) < 1e-12
        assert 0.0 < left_y < 1.0 and 0.0 < right_y < 1.0
        # Each line is on the panel whole, own thickness included.
        assert PEDAL_LINE_HALF_Y <= left_y <= 1.0 - PEDAL_LINE_HALF_Y
        assert PEDAL_LINE_HALF_Y <= right_y <= 1.0 - PEDAL_LINE_HALF_Y
    assert pedal_line_heights(1.0) == (0.5 - 0.5 * PEDAL_LINE_TRAVEL,
                                      0.5 + 0.5 * PEDAL_LINE_TRAVEL)
    # A pedal past its stop is on it rather than off the panel, and the panel has
    # the room for both lines whole: their middles and their own ends.
    assert pedal_line_heights(4.0) == pedal_line_heights(1.0)
    assert pedal_line_heights(-4.0) == pedal_line_heights(-1.0)
    assert PEDAL_LEFT_X - PEDAL_LINE_HALF_X >= 0.0
    assert PEDAL_RIGHT_X + PEDAL_LINE_HALF_X <= 1.0
    assert PEDAL_LEFT_X + PEDAL_LINE_HALF_X < PEDAL_RIGHT_X - PEDAL_LINE_HALF_X

    assert cyclic_dot_centre(0.0, 0.0) == (0.5, 0.5)
    assert cyclic_dot_centre(1.0, 0.0)[0] > 0.5          # right is right
    assert cyclic_dot_centre(-1.0, 0.0)[0] < 0.5
    assert cyclic_dot_centre(0.0, 1.0)[1] > 0.5          # and forward is up
    assert cyclic_dot_centre(0.0, -1.0)[1] < 0.5
    assert cyclic_dot_centre(3.0, -3.0) == cyclic_dot_centre(1.0, -1.0)
    for lat, long in ((0.0, 0.0), (1.0, 1.0), (-1.0, 1.0), (0.4, -0.9)):
        dot_x, dot_y = cyclic_dot_centre(lat, long)
        # With the stick on a stop the disc is still a whole disc: the panel's own
        # edge does not cut it.
        assert CYCLIC_DOT_RADIUS <= dot_x <= 1.0 - CYCLIC_DOT_RADIUS
        assert CYCLIC_DOT_RADIUS <= dot_y <= 1.0 - CYCLIC_DOT_RADIUS
        assert abs(dot_x - 0.5) <= 0.5 * CYCLIC_DOT_TRAVEL + 1e-12
        assert abs(dot_y - 0.5) <= 0.5 * CYCLIC_DOT_TRAVEL + 1e-12

    assert collective_lever_deg(0.0) == QUADRANT_LOW_DEG   # the field's own floor
    assert collective_lever_deg(1.0) == QUADRANT_HIGH_DEG  # and its own roof
    assert collective_lever_deg(0.5) == 0.5 * (QUADRANT_LOW_DEG + QUADRANT_HIGH_DEG)
    assert collective_lever_deg(-1.0) == QUADRANT_LOW_DEG  # a stop is a stop
    for fraction in (0.0, 0.35, 1.0):
        assert (QUADRANT_LOW_DEG <= collective_lever_deg(fraction)
                <= QUADRANT_HIGH_DEG)
    # The field and the lever are inside the panel, and so is the lever's own
    # thickness: the pivot is inset by more than half of it.
    assert QUADRANT_PIVOT_X + QUADRANT_RADIUS <= 1.0
    assert QUADRANT_PIVOT_Y + QUADRANT_RADIUS <= 1.0
    assert QUADRANT_PIVOT_X > QUADRANT_LEVER_HALF
    assert QUADRANT_PIVOT_Y > QUADRANT_LEVER_HALF
    assert QUADRANT_LOW_DEG < QUADRANT_HIGH_DEG < 90.0

    # The attitude panel: the dial is on the panel, the ball inside the dial and
    # the aircraft's own reference inside the ball, so no part of the instrument
    # can be drawn off its own square.
    assert 0.0 < ATTITUDE_CENTRE_X - ATTITUDE_RADIUS
    assert 0.0 < ATTITUDE_CENTRE_Y - ATTITUDE_RADIUS
    assert ATTITUDE_CENTRE_X + ATTITUDE_RADIUS <= 1.0
    assert ATTITUDE_CENTRE_Y + ATTITUDE_RADIUS <= 1.0
    assert 0.0 < ATTITUDE_BALL_RADIUS < ATTITUDE_RADIUS
    assert ATTITUDE_SEGMENTS >= 16 and ATTITUDE_LADDER_RUNGS >= 1
    assert ATTITUDE_PITCH_DEG_PER_RADIUS > 0.0 and ATTITUDE_LADDER_DEG > 0.0
    # A line of the dial is one pixel, and half of one is half a pixel of the
    # panel: the picture is a drawing of lines rather than of shapes, and a pixel
    # is what keeps them lines whatever PANEL_PX is.
    assert ATTITUDE_LINE_PX == 1.0
    assert abs(2.0 * ATTITUDE_LINE_HALF - ATTITUDE_LINE_PX / PANEL_PX) < 1e-12

    # The dial's own colours: the sky over the ground blue over brown, the ball's
    # own lines green and the aircraft's own reference the white over them - five
    # colours, and no two of them anywhere near one another, which is what lets a
    # readback of the drawing tell the shape it is looking at by its colour.
    assert ATTITUDE_SKY_COLOUR[2] > ATTITUDE_SKY_COLOUR[1] > ATTITUDE_SKY_COLOUR[0]
    assert (ATTITUDE_GROUND_COLOUR[0] > ATTITUDE_GROUND_COLOUR[1]
            > ATTITUDE_GROUND_COLOUR[2])
    assert ATTITUDE_GREEN[1] > ATTITUDE_GREEN[0]
    assert ATTITUDE_GREEN[1] > ATTITUDE_GREEN[2]
    assert ATTITUDE_WHITE[0] > 0.9 and ATTITUDE_WHITE[1] > 0.9
    colours = (ATTITUDE_CASE_COLOUR, ATTITUDE_SKY_COLOUR,
               ATTITUDE_GROUND_COLOUR, ATTITUDE_GREEN, ATTITUDE_WHITE)
    for index, first in enumerate(colours):
        for second in colours[index + 1:]:
            assert max(abs(a - b) for a, b in zip(first, second)) > 0.2

    # Wings level and level: the ball's own axes are the case's, the horizon is
    # the whole width of the ball through the middle of the dial, and the ball's
    # own scales put no line anywhere else.
    assert attitude_ball_axes(0.0) == ((1.0, 0.0), (0.0, 1.0))
    assert attitude_pitch_offset(0.0) == 0.0
    assert attitude_line_height(0.0, 0.0, 0.0) == 0.0
    for roll in (-30.0, 0.0, 45.0):
        for axis in attitude_ball_axes(roll):
            assert abs(math.hypot(*axis) - 1.0) < 1e-12
    horizon = attitude_line_ends(0.0, 0.0, 0.0, ATTITUDE_BALL_RADIUS)
    assert horizon is not None
    assert abs(horizon[0][1] - ATTITUDE_CENTRE_Y) < 1e-12
    assert abs(horizon[1][1] - ATTITUDE_CENTRE_Y) < 1e-12
    assert abs(horizon[0][0] - (ATTITUDE_CENTRE_X - ATTITUDE_BALL_RADIUS)) < 1e-12
    assert abs(horizon[1][0] - (ATTITUDE_CENTRE_X + ATTITUDE_BALL_RADIUS)) < 1e-12

    # A nose up puts the horizon below the middle of the dial by the pitch's own
    # share of the ball's radius, a nose down above it, and either way the two
    # ends of the horizon are on the ball's own edge: a shorter chord rather than
    # a line drawn over the case.
    for pitch in (-18.0, -4.5, 4.5, 18.0):
        ends = attitude_line_ends(0.0, pitch, 0.0, ATTITUDE_BALL_RADIUS)
        assert ends is not None
        assert abs(ends[0][1] - ends[1][1]) < 1e-12
        assert abs(ends[0][1] - (ATTITUDE_CENTRE_Y
                                 - attitude_pitch_offset(pitch))) < 1e-12
        assert attitude_line_height(0.0, pitch, 0.0) * pitch < 0.0
        for x, y in ends:
            assert abs(math.hypot(x - ATTITUDE_CENTRE_X, y - ATTITUDE_CENTRE_Y)
                       - ATTITUDE_BALL_RADIUS) < 1e-12

    # A roll turns the ball's own up off the case's by the roll itself and to
    # port, which is the way the world appears to turn to a pilot who rolls to
    # starboard: the sky ends up up and to port and the starboard end of the
    # horizon is the high one.
    for roll in (-25.0, 12.0):
        right, up = attitude_ball_axes(roll)
        assert abs(math.atan2(up[0], up[1]) + math.radians(roll)) < 1e-12
        ends = attitude_line_ends(roll, 0.0, 0.0, ATTITUDE_BALL_RADIUS)
        assert ends is not None
        starboard = max(ends, key=lambda end: end[0])
        port = min(ends, key=lambda end: end[0])
        assert (starboard[1] - ATTITUDE_CENTRE_Y) * roll > 0.0
        assert (port[1] - ATTITUDE_CENTRE_Y) * roll < 0.0
        for x, y in ends:
            assert abs(math.hypot(x - ATTITUDE_CENTRE_X, y - ATTITUDE_CENTRE_Y)
                       - ATTITUDE_BALL_RADIUS) < 1e-12

    # The slide is down the *case* and only the ball's own up when the wings are
    # level, so a bank cuts the pitch's share of it by the cosine of the roll:
    # rolled onto its side, a pitch takes the horizon across the dial and not
    # down it at all.
    assert attitude_line_height(0.0, 10.0, 60.0) == (
        -attitude_pitch_offset(10.0) * math.cos(math.radians(60.0)))
    assert abs(attitude_line_height(0.0, 10.0, 90.0)) < 1e-15

    # A pitch past the scale's own end carries the horizon off the dial: the ball
    # is all sky at a nose up and all ground at a nose down, with no chord to fill
    # to and no line to draw.
    assert attitude_line_height(0.0, ATTITUDE_PITCH_DEG_PER_RADIUS, 0.0) == (
        -ATTITUDE_BALL_RADIUS)
    assert attitude_line_ends(0.0, ATTITUDE_PITCH_DEG_PER_RADIUS + 5.0, 0.0,
                              ATTITUDE_BALL_RADIUS) is None
    assert attitude_horizon_gamma(0.0, -ATTITUDE_PITCH_DEG_PER_RADIUS - 0.5) is None
    assert attitude_horizon_gamma(0.0,
                                  ATTITUDE_PITCH_DEG_PER_RADIUS - 0.5) is not None

    # The pitch ladder: a rung at 10 and 20 deg each way, each of them the whole
    # step it names above or below the horizon on the ball, each shorter than the
    # horizon and shorter again further out, and all of them short enough to be
    # drawn rather than cut by the ball's own edge.
    halves = [attitude_ladder_half(rung * ATTITUDE_LADDER_DEG)
              for rung in range(1, ATTITUDE_LADDER_RUNGS + 3)]
    assert halves == sorted(halves, reverse=True)
    assert 0.0 < halves[ATTITUDE_LADDER_RUNGS - 1] < halves[0] < ATTITUDE_BALL_RADIUS
    assert attitude_ladder_half(-ATTITUDE_LADDER_DEG) == attitude_ladder_half(
        ATTITUDE_LADDER_DEG)                       # the ladder is the same both ways
    for rung in range(1, ATTITUDE_LADDER_RUNGS + 1):
        for sign in (1.0, -1.0):
            line_deg = sign * rung * ATTITUDE_LADDER_DEG
            height = attitude_line_height(line_deg, 0.0, 0.0)
            assert abs(height - attitude_pitch_offset(line_deg)) < 1e-12
            ends = attitude_line_ends(0.0, 0.0, line_deg,
                                      attitude_ladder_half(line_deg))
            assert ends is not None
            assert abs(abs(ends[1][0] - ends[0][0])
                       - 2.0 * attitude_ladder_half(line_deg)) < 1e-12
            assert abs(0.5 * (ends[0][1] + ends[1][1])
                       - (ATTITUDE_CENTRE_Y + height)) < 1e-12
            for x, y in ends:
                assert math.hypot(x - ATTITUDE_CENTRE_X,
                                  y - ATTITUDE_CENTRE_Y) < ATTITUDE_BALL_RADIUS

    # The horizon is the one line that wants the whole ball, since its own half
    # length is the ball's radius: what is drawn is always the chord the ball
    # allows, and the arc the sky is filled along is that same chord - the two
    # ends of the line, on the ball's own edge and at +/- gamma from its up.  The
    # fans' own apex is the middle of it, where the perpendicular from the middle
    # of the dial lands.
    ends = attitude_line_ends(20.0, 6.0, 0.0, ATTITUDE_BALL_RADIUS)
    assert ends is not None
    gamma = attitude_horizon_gamma(20.0, 6.0)
    height = attitude_line_height(0.0, 6.0, 20.0)
    assert abs(0.5 * math.hypot(ends[1][0] - ends[0][0], ends[1][1] - ends[0][1])
               - ATTITUDE_BALL_RADIUS * math.sin(gamma)) < 1e-12
    for beta, end in ((-gamma, ends[0]), (gamma, ends[1])):
        x, y = attitude_ball_point(20.0, beta)
        assert abs(x - end[0]) < 1e-12 and abs(y - end[1]) < 1e-12
        assert abs(math.hypot(x - ATTITUDE_CENTRE_X, y - ATTITUDE_CENTRE_Y)
                   - ATTITUDE_BALL_RADIUS) < 1e-12
    up = attitude_ball_axes(20.0)[1]
    assert abs(0.5 * (ends[0][0] + ends[1][0])
               - (ATTITUDE_CENTRE_X + height * up[0])) < 1e-12
    assert abs(0.5 * (ends[0][1] + ends[1][1])
               - (ATTITUDE_CENTRE_Y + height * up[1])) < 1e-12
    # Any point of the ball's own edge is on it, which is what the fans are
    # filled along rather than an estimate of one.
    for beta in (0.0, 1.234, -2.5, math.pi):
        x, y = attitude_ball_point(20.0, beta)
        assert abs(math.hypot(x - ATTITUDE_CENTRE_X, y - ATTITUDE_CENTRE_Y)
                   - ATTITUDE_BALL_RADIUS) < 1e-12

    # The aircraft's own reference: two bars and the W between them, in the middle
    # of the dial and inside the ball.  It is built from constants and takes no
    # roll and no pitch at all, because it is the one shape on the panel that no
    # attitude moves - and it hangs below its own middle line, which is the
    # picture's shape and not a W drawn upside down.
    strokes = attitude_reference_strokes()
    assert len(strokes) == 6
    for (x0, y0), (x1, y1) in strokes[:2]:
        assert y0 == y1 == ATTITUDE_CENTRE_Y            # the two bars
    for (x0, y0), (x1, y1) in strokes[2:]:
        # Each stroke of the W runs between the bars' own line and its own
        # valley, which is below that line: the W hangs, as the picture's does.
        assert sorted((y0, y1)) == [ATTITUDE_CENTRE_Y - ATTITUDE_W_DEPTH,
                                    ATTITUDE_CENTRE_Y]
    assert ATTITUDE_BAR_INNER < ATTITUDE_BAR_OUTER < ATTITUDE_BALL_RADIUS
    assert 0.0 < ATTITUDE_W_DEPTH < ATTITUDE_BAR_OUTER
    for stroke in strokes:
        (x0, y0), (x1, y1) = stroke
        corners = attitude_stroke_corners(x0, y0, x1, y1, ATTITUDE_LINE_HALF)
        assert corners is not None and len(corners) == 4
        for x, y in corners:
            assert 0.0 <= x <= 1.0 and 0.0 <= y <= 1.0
            assert math.hypot(x - ATTITUDE_CENTRE_X,
                              y - ATTITUDE_CENTRE_Y) < ATTITUDE_BALL_RADIUS
        # Every stroke has its mirror the other side of the middle of the dial,
        # and a stroke across it is as thick on one side as on the other.
        mirror = ((2.0 * ATTITUDE_CENTRE_X - x1, y1),
                  (2.0 * ATTITUDE_CENTRE_X - x0, y0))
        assert any(math.isclose(mirror[0][0], sx0) and math.isclose(mirror[1][0], sx1)
                   for (sx0, _), (sx1, _) in strokes)
        # The quad is the stroke: its two long sides are the line's own length,
        # and its two ends are the same thickness across it at either end.
        sides = [math.hypot(corners[index][0] - corners[index - 1][0],
                            corners[index][1] - corners[index - 1][1])
                 for index in range(4)]
        assert abs(sides[1] - math.hypot(x1 - x0, y1 - y0)) < 1e-12
        assert abs(sides[3] - sides[1]) < 1e-12
        assert abs(sides[0] - 2.0 * ATTITUDE_LINE_HALF) < 1e-12
        assert abs(sides[2] - sides[0]) < 1e-12
    assert attitude_stroke_corners(0.0, 0.0, 0.0, 0.0, ATTITUDE_LINE_HALF) is None

    # The seven panels themselves: square, on the window, and laid out as an L -
    # the first PANEL_COLUMN_COUNT up the left edge and the rest along the bottom
    # edge beside them - and a panel's own fractions are pixels, so a shape can be
    # placed by the fractions above and drawn by the two functions below.
    boxes = [panel_box(index) for index in range(PANEL_COUNT)]
    for x0, y0, x1, y1 in boxes:
        assert (x1 - x0, y1 - y0) == (PANEL_PX, PANEL_PX)
        assert x1 <= WIDTH and y1 <= HEIGHT
    column = boxes[:PANEL_COLUMN_COUNT]
    row = boxes[PANEL_COLUMN_COUNT:]
    # The column shares the window's left margin as its own left edge and the row
    # the bottom one as its own bottom edge; the column runs down from the pedals
    # at the top to the attitude indicator at its foot, and one PANEL_GAP_PX
    # separates every pair of them, both across the L and down it.
    for box in column:
        assert box[0] == PANEL_MARGIN_PX
    for box in row:
        assert box[1] == PANEL_MARGIN_PX
    for upper, lower in zip(column[:-1], column[1:]):
        assert upper[1] - lower[3] == PANEL_GAP_PX
    for left, right in zip(row[:-1], row[1:]):
        assert right[0] - left[2] == PANEL_GAP_PX
    assert column[0][3] <= HEIGHT                     # the top of the column
    assert row[0][0] - column[0][2] == PANEL_GAP_PX   # one gap across the L
    assert row[-1][2] <= WIDTH                        # and the row on the window
    cyclic_box = boxes[PANEL_CYCLIC]
    assert panel_point(cyclic_box, 0.5, 0.5) == (cyclic_box[0] + 0.5 * PANEL_PX,
                                                 cyclic_box[1] + 0.5 * PANEL_PX)
    assert panel_point(boxes[0], 0.0, 0.0) == (boxes[0][0], boxes[0][1])
    assert panel_point(boxes[0], 1.0, 1.0) == (boxes[0][2], boxes[0][3])
    # The dial's own points are those shifted by half a pixel, to the middle of a
    # pixel rather than the corner of one, which is what makes its one pixel lines
    # a row or a column of pixels instead of a straddle of two.
    assert attitude_point(boxes[0], 0.0, 0.0) == (boxes[0][0] + 0.5,
                                                   boxes[0][1] + 0.5)
    assert attitude_point(boxes[0], 1.0, 1.0) == (boxes[0][2] + 0.5,
                                                   boxes[0][3] + 0.5)
    assert "attic" in PANEL_MOCKUP_FILE and PANEL_MOCKUP_FILE.endswith(
        "controls_simple.png")
    # The attitude indicator is the foot of the column - the last of the four -
    # and the three instruments finish the row beside it, each drawn from the file
    # its own face is pictured in; five files in all describe this HUD.
    assert PANEL_ATTITUDE == PANEL_COLUMN_COUNT - 1
    assert boxes[PANEL_ATTITUDE] == column[-1]
    assert PANEL_ALTIMETER == PANEL_COLUMN_COUNT
    assert PANEL_AIRSPEED == PANEL_COUNT - 1
    assert boxes[PANEL_AIRSPEED] == row[-1]
    assert "attic" in PANEL_ATTITUDE_FILE and PANEL_ATTITUDE_FILE.endswith(
        "attitude.png")
    for name, path in (("altimeter", PANEL_ALTIMETER_FILE),
                       ("vertspeed", PANEL_VSI_FILE),
                       ("airspeed", PANEL_AIRSPEED_FILE)):
        assert "attic" in path and path.endswith(name + ".png")

    # The three instruments are read from the aircraft rather than held, and each
    # face's needle is a whole turn or less of its own dial: the arithmetic is
    # asserted here, since the drawing of it needs a window.  A dial's own point
    # is nought degrees straight up from its middle and ninety to its right.
    assert abs(dial_point(0.5, 0.5, 0.25, 0.0)[0] - 0.5) < 1e-12
    assert abs(dial_point(0.5, 0.5, 0.25, 0.0)[1] - 0.75) < 1e-12
    assert abs(dial_point(0.5, 0.5, 0.25, 90.0)[0] - 0.75) < 1e-12
    assert abs(dial_point(0.5, 0.5, 0.25, 180.0)[1] - 0.25) < 1e-12
    # The altimeter's two needles are up together at zero, the long thin one the
    # hundreds - a turn a thousand feet - and the short thick one the thousands: at
    # 1,234 ft the thin needle is 0.234 of a turn round and the thick one 0.1234,
    # and at a whole ten thousand feet both of them are back at the top.
    assert altimeter_needle_deg(0.0) == (0.0, 0.0)
    thin, thick = altimeter_needle_deg(1234.0)
    assert abs(thin - 360.0 * 0.234) < 1e-9
    assert abs(thick - 360.0 * 0.1234) < 1e-9
    assert altimeter_needle_deg(10000.0) == (0.0, 0.0)
    assert abs(altimeter_needle_deg(2500.0)[0] - 180.0) < 1e-9   # half a turn
    # The vertical speed indicator: nought at nine o'clock with a climb turning
    # the needle clockwise and fifty degrees of the dial to every thousand feet a
    # minute, so the scale's top is about two o'clock and its bottom about
    # four; past either end of it the needle stops rather than running on round the
    # dial.
    assert vsi_needle_deg(0.0) == VSI_ZERO_DEG == 270.0
    assert vsi_needle_deg(VSI_FULL_SCALE_FPM) == 60.0
    assert vsi_needle_deg(-VSI_FULL_SCALE_FPM) == 120.0
    assert vsi_needle_deg(1000.0) == 320.0
    assert vsi_needle_deg(-1000.0) == 220.0
    assert vsi_needle_deg(9000.0) == 60.0 and vsi_needle_deg(-9000.0) == 120.0
    # The airspeed indicator: nought at twelve o'clock and a whole turn to 160 kt,
    # so the needle is up at a standstill and at the top of the scale alike, and
    # the caution speed - the one red mark on the HUD - is a quarter turn round, at
    # the same nine o'clock the vertical speed indicator's own nought is at.
    assert airspeed_needle_deg(0.0) == 0.0
    assert airspeed_needle_deg(AIRSPEED_FULL_SCALE_KT) == 360.0
    assert airspeed_needle_deg(AIRSPEED_CAUTION_KT) == VSI_ZERO_DEG
    assert airspeed_needle_deg(80.0) == 180.0
    assert airspeed_needle_deg(400.0) == 360.0        # past the scale it stops
    assert airspeed_needle_deg(-10.0) == 0.0

    # The caption carries the four control positions, and they are read off the
    # same telemetry the aircraft is flown from.  The case worth asserting is
    # the pad, where the skids hold the aircraft exactly level and still and a
    # held key moves nothing but the caption: the right cyclic and D arriving as
    # inches of right stick and right pedal - while the long stick, with no key on
    # it, sits at the position the run started with rather than moving anywhere at
    # all, and the collective, which nobody is touching, sits at the 75 % the park
    # left it at, up in the green field.  Two seconds of the two keys, since a key
    # turns a ratchet at a quarter of its rate: half a second used to give this
    # same pedal hard against its stop, and now needs four times as long.  W is
    # deliberately not among them - from 75 % it reaches the hover trim in half a
    # second and the skids let go, which is the pickup above and not this.
    sim.on_the_pad()
    parked = sim.controls
    parked_axes = sim.pilot.axes()
    parked_x = sim.airframe.state.position.x
    parked_y = sim.airframe.state.position.y
    for _ in range(120):
        fly_frame(sim, camera, SIM_TIME_STEP_S,
                  {pygame.KSCAN_RIGHT: True, pygame.KSCAN_D: True})
    assert sim.on_ground and not sim.crashed
    assert sim.airframe.state.position.x == parked_x     # the skids hold it
    assert sim.airframe.state.position.y == parked_y
    assert sim.airframe.state.speed < 1e-9
    assert abs(sim.airframe.state.rates.x) < 1e-9        # and hold it level
    pad = sim.telemetry()
    assert pad.collective_fraction == PAD_COLLECTIVE     # nothing is on that key
    assert pad.collective_in == parked.collective
    assert pad.long_stick_in == parked.long_stick        # nothing is on it either
    assert pad.lat_stick_in > parked.lat_stick + 1.0     # Right is a real input
    assert pad.pedal_in > parked.pedal + 1.0             # and so is D
    # The attitude indicator is drawn from the aircraft rather than from a hand,
    # and the skids hold the aircraft level whatever the two keys are doing:
    # ``simulation.GROUND_LEVELLING_PER_S`` brings the roll and the pitch back at
    # its own rate rather than snapping them, so after two seconds of a held
    # lateral stick and pedal the ball is level to a fraction of a degree - its
    # horizon across the middle of the dial and its ladder level either side of
    # it, which is all that panel has to say while the skids are down.
    assert abs(pad.roll_deg) < 1.0 and abs(pad.pitch_deg) < 1.0
    assert abs(attitude_line_height(0.0, pad.pitch_deg, pad.roll_deg)) < 0.02
    assert abs(attitude_horizon_gamma(pad.roll_deg, pad.pitch_deg)
               - 0.5 * math.pi) < 0.02
    assert ("| coll %3.0f %% | cyc %+6.2f/%+6.2f in | ped %+5.2f in"
            % (100.0 * pad.collective_fraction, pad.long_stick_in,
               pad.lat_stick_in, pad.pedal_in)) in window_title(sim, VIEW_CHASE)
    # And the panel over the world is drawn from those same two keys, in the same
    # frame: it reads the pilot's own axes rather than the trimmed inches, so the
    # pedal lines have separated - the right one up and the left one down, which
    # is what D means - the cyclic disc has moved right of the middle of its panel
    # by the lateral stick's own axis, and the collective lever is up in the
    # green field at the height the park left it at.  Nothing is held on the long
    # stick, so the disc's own height is exactly the height that park left it at
    # too.
    axes = sim.pilot.axes()
    collective_axis, long_axis, lat_axis, pedal_axis = axes
    assert collective_axis == PAD_COLLECTIVE and lat_axis > 0.0
    assert pedal_axis > 0.0
    assert collective_axis == pad.collective_fraction
    assert collective_axis == parked_axes[0]
    assert long_axis == parked_axes[1]
    left_y, right_y = pedal_line_heights(pedal_axis)
    assert right_y > 0.5 > left_y
    dot_x, dot_y = cyclic_dot_centre(lat_axis, long_axis)
    assert dot_x > 0.5
    assert dot_x == cyclic_dot_centre(lat_axis, 0.0)[0]
    assert dot_y == cyclic_dot_centre(0.0, long_axis)[1]
    assert collective_lever_deg(collective_axis) > QUADRANT_LOW_DEG



    # A tap is a frame of travel rather than nothing: the frame loop's own
    # frame_keys reads a key that went down and up inside one frame, and the
    # ratchet turns by that frame's own *key* rate - the collective's 0.55 a
    # second times its quarter granularity, so 0.1375 a second, which is what
    # makes four taps the travel one used to be.  Without it a tapped W would
    # leave the collective exactly where it was, which is what a dead key looks
    # like - and it is the whole difference between tapping W and holding it.
    sim.on_the_pad()
    assert sim.pilot.collective_axis == PAD_COLLECTIVE
    collective_key = sim.pilot.key_rates()[0]
    assert collective_key == 0.1375                        # 0.55 * 0.25
    fly_frame(sim, camera, SIM_TIME_STEP_S, frame_keys({}, {pygame.KSCAN_W}))
    tapped = PAD_COLLECTIVE + collective_key * SIM_TIME_STEP_S
    assert abs(sim.pilot.collective_axis - tapped) < 1e-9
    assert sim.telemetry().collective_fraction > PAD_COLLECTIVE
    # The frame after the tap, with the key gone again, leaves it there: it is
    # travel from a press, not a switch that was left on.
    fly_frame(sim, camera, SIM_TIME_STEP_S, frame_keys({}, set()))
    assert abs(sim.pilot.collective_axis - tapped) < 1e-9
    # Eighty more taps - four times the twenty that were a fifth of the
    # collective at the full rate, which is the whole of the granularity - is
    # that fifth again, and the caption says so in whole per cent - the same
    # telemetry the check has been reading.
    for _ in range(80):
        fly_frame(sim, camera, SIM_TIME_STEP_S,
                  frame_keys({}, {pygame.KSCAN_W}))
    ratcheted = sim.telemetry()
    assert abs(ratcheted.collective_fraction
               - (PAD_COLLECTIVE + 81 * collective_key * SIM_TIME_STEP_S)) < 1e-12
    assert ratcheted.collective_fraction > PAD_COLLECTIVE + 0.15
    assert ratcheted.collective_in > PAD_COLLECTIVE * UH1_COLLECTIVE_TRAVEL_IN + 1.5
    assert ("| coll %3.0f %% | cyc %+6.2f/%+6.2f in | ped %+5.2f in"
            % (100.0 * ratcheted.collective_fraction, ratcheted.long_stick_in,
               ratcheted.lat_stick_in, ratcheted.pedal_in)
            ) in window_title(sim, VIEW_CHASE)


    # The key log: the command line that names it, the names it gives a key, the
    # columns of a line, the two clocks on it and the rate that keeps the flight
    # sampled rather than written every frame.
    assert command_line([]) == (DEFAULT_SCENERY_FILE, DEFAULT_KEYLOG_FILE)
    assert command_line(["scene.xml"]) == ("scene.xml", DEFAULT_KEYLOG_FILE)
    assert command_line(["scene.xml", "--log", "run.csv"]) == ("scene.xml",
                                                              "run.csv")
    assert command_line(["--log", "run.csv"]) == (DEFAULT_SCENERY_FILE,
                                                  "run.csv")
    assert command_line(["--no-log"])[1] is None
    assert command_line(["scene.xml", "--no-log"]) == ("scene.xml", None)
    assert command_line(["--log"])[1] == DEFAULT_KEYLOG_FILE  # no file named
    assert command_line(["--no-log", "--log", "run.csv"])[1] == "run.csv"

    # A flight key is named by the position the controls read it from, and every
    # other key - the four command keys among them - by pygame's own name.
    assert key_name(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_w,
                                       scancode=pygame.KSCAN_W)) == "W"
    assert key_name(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_LEFT,
                                       scancode=pygame.KSCAN_LEFT)) == "LEFT"
    assert key_name(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE,
                                       scancode=pygame.KSCAN_ESCAPE)) == "escape"
    assert held_names({}) == "-"
    assert held_names({pygame.KSCAN_W: True}) == "W"
    assert held_names({pygame.KSCAN_RIGHT: True,
                       pygame.KSCAN_W: True}) == "W+RIGHT"
    assert held_names({pygame.KSCAN_R: True, pygame.KSCAN_W: False}) == "-"

    def last_line(log):
        """The last line of a log, as a dict of its own columns."""
        return dict(zip(KeyLog.COLUMNS,
                        log.stream.getvalue().splitlines()[-1].split(",")))

    assert len(KeyLog.COLUMNS) == len(set(KeyLog.COLUMNS))
    log = KeyLog(io.StringIO(), rate_hz=20.0)
    assert log.period == 0.05 == 1.0 / KEYLOG_RATE_HZ
    header = log.stream.getvalue().splitlines()
    assert header[0].startswith("#") and header[-1].split(",") == list(
        KeyLog.COLUMNS)
    sim.on_the_pad()
    assert log.sample(sim, held_names({})) is True    # the first is always written
    assert log.sample(sim, held_names({})) is False   # 20 Hz, not one per frame
    assert log.entries == 1
    log_key(log, sim, "down", pygame.event.Event(pygame.KEYDOWN,
                                                 key=pygame.K_w,
                                                 scancode=pygame.KSCAN_W),
            {pygame.KSCAN_W: True})
    line = last_line(log)
    assert line["kind"] == "down" and line["key"] == "W" and line["held"] == "W"
    assert line["frame"] == str(sim.frames)
    # The state columns are the aircraft's own telemetry, so a line stands on
    # its own: velocity, heading, pitch and roll, the altitude and the four
    # controls are all there.
    telemetry = sim.telemetry()
    assert abs(float(line["sim_s"]) - sim.sim_time) < 5e-5
    assert abs(float(line["spd_mps"]) - telemetry.airspeed) < 5e-4
    assert abs(float(line["hdg_deg"]) - telemetry.yaw_deg) < 5e-4
    assert abs(float(line["pitch_deg"]) - telemetry.pitch_deg) < 5e-4
    assert abs(float(line["roll_deg"]) - telemetry.roll_deg) < 5e-4
    assert abs(float(line["alt_m"]) - UH1_CG_HEIGHT_ON_GROUND) < 5e-4
    assert abs(float(line["coll_in"]) - telemetry.collective_in) < 5e-4
    assert abs(float(line["coll_frac"]) - PAD_COLLECTIVE) < 5e-4
    # A millisecond on both clocks, and the two of them the same instant: the
    # wall clock in milliseconds is that many thousand times the seconds.
    assert len(line["t_s"].split(".")[-1]) == 3
    assert len(line["wall_ms"].split(".")[-1]) == 3
    assert abs((float(line["wall_ms"]) - log.origin_wall * 1000.0)
               - 1000.0 * float(line["t_s"])) < 2.0
    # And a line that is not a key at all: the window losing the keyboard.
    log_key(log, sim, "blur", None, {})
    line = last_line(log)
    assert line["kind"] == "blur" and line["key"] == "-" and line["held"] == "-"


if __name__ == "__main__":
    if "--check" in sys.argv[1:]:
        _self_check()
        print("main.py check passed")
    else:
        main()
