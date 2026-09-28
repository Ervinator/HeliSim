#!/usr/bin/env python3
"""HeliSim - a small OpenGL helicopter style flight sandbox.

The world is made of two parts:

* an *implicit* flat grass ground plane with a grid.  It is always present and
  is deliberately not part of the scenery description, and
* the objects listed in an XML scenery file (see scenery.py and
  sample_scenery.xml).  Every object has a coordinate and an orientation.

Usage::

    python main.py [scenery.xml]
"""

import math
import os
import sys

import numpy as np

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import scenery

WIDTH, HEIGHT = 960, 640

# Scenery file used when no path is given on the command line.
DEFAULT_SCENERY_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "sample_scenery.xml")

# ---------------------------------------------------------------------------
# Camera state and navigation controls.
# ---------------------------------------------------------------------------
camera_x = 0.0
camera_y = 2.8
camera_z = 0.0
forward = (0.0, 0.0, -1.0)
up = (0.0, 1.0, 0.0)
right = (1.0, 0.0, 0.0)

# The scenery description, filled in by main() from the XML file.
SCENERY = scenery.Scenery()


def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(62.0, width / float(height), 0.1, 80.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init_gl():
    glClearColor(0.58, 0.80, 0.96, 1.0)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_CULL_FACE)
    glCullFace(GL_BACK)
    glShadeModel(GL_SMOOTH)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def draw_ground():
    """Draw the implicit ground plane and its grid.

    The ground and its grid always exist and are deliberately not part of the
    scenery description.  The quad is wound counter clockwise as seen from
    above, so its upper side survives GL_CULL_FACE.
    """
    # Large flat grass surface.
    glColor3f(0.34, 0.60, 0.20)
    glBegin(GL_QUADS)
    glVertex3f(-50.0, 0.0, 50.0)
    glVertex3f(50.0, 0.0, 50.0)
    glVertex3f(50.0, 0.0, -50.0)
    glVertex3f(-50.0, 0.0, -50.0)
    glEnd()

    # A subtle grid of vector lines to suggest depth.
    glColor3f(0.16, 0.36, 0.12)
    for x in range(-45, 51, 5):
        glBegin(GL_LINES)
        glVertex3f(float(x), 0.04, -50.0)
        glVertex3f(float(x), 0.04, 50.0)
        glEnd()
    for z in range(-45, 51, 5):
        glBegin(GL_LINES)
        glVertex3f(-50.0, 0.04, float(z))
        glVertex3f(50.0, 0.04, float(z))
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


def normalize(v):
    x, y, z = v
    length = math.sqrt(x * x + y * y + z * z)
    if length == 0:
        return (0.0, 0.0, 0.0)
    return (x / length, y / length, z / length)


def cross(a, b):
    x1, y1, z1 = a
    x2, y2, z2 = b
    return (y1 * z2 - z1 * y2,
            z1 * x2 - x1 * z2,
            x1 * y2 - y1 * x2)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def rotate_vector_around_vector(vector, axis, angle):
    axis = normalize(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    k = dot(axis, vector)
    cross_axis_vector = cross(axis, vector)
    return (
        vector[0] * c + cross_axis_vector[0] * s + axis[0] * k * (1.0 - c),
        vector[1] * c + cross_axis_vector[1] * s + axis[1] * k * (1.0 - c),
        vector[2] * c + cross_axis_vector[2] * s + axis[2] * k * (1.0 - c)
    )


def draw_scene():
    global camera_x, camera_y, camera_z, forward, up

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    target_x = camera_x + forward[0]
    target_y = camera_y + forward[1]
    target_z = camera_z + forward[2]

    gluLookAt(camera_x, camera_y, camera_z,
              target_x, target_y, target_z,
              up[0], up[1], up[2])

    draw_ground()      # implicit, always there
    draw_scenery()     # everything the XML scenery description lists


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


def main():
    global camera_x, camera_y, camera_z, forward, up, right
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

    pygame.init()
    pygame.display.set_mode((WIDTH, HEIGHT), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("HeliSim - OpenGL Terrain")
    resize(WIDTH, HEIGHT)
    init_gl()

    move_speed = 0.45
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False

        keys = pygame.key.get_pressed()

        # Camera-state mapping:
        # W/S: drive forward/back along the same R/P/Y basis vector used by the view.
        # A/D: yaw left/right.
        # Left/Right arrows: roll left/right.
        # Up/Down arrows: pitch down/up.

        if keys[K_w]:
            camera_x += forward[0] * move_speed
            camera_y += forward[1] * move_speed
            camera_z += forward[2] * move_speed

        if keys[K_s]:
            camera_x -= forward[0] * move_speed
            camera_y -= forward[1] * move_speed
            camera_z -= forward[2] * move_speed

        if keys[K_a]:
            # A/D: Yaw left/right
            forward = normalize(rotate_vector_around_vector(forward, up, 0.03))
            right = normalize(rotate_vector_around_vector(right, up, 0.03))
        if keys[K_d]:
            forward = normalize(rotate_vector_around_vector(forward, up, -0.03))
            right = normalize(rotate_vector_around_vector(right, up, -0.03))

        # Left/Right arrows: Roll left/right.
        if keys[K_LEFT]:
            up = normalize(rotate_vector_around_vector(up, forward, -0.03))
            right = normalize(rotate_vector_around_vector(right, forward, -0.03))
        if keys[K_RIGHT]:
            up = normalize(rotate_vector_around_vector(up, forward, 0.03))
            right = normalize(rotate_vector_around_vector(right, forward, 0.03))

        # Up/Down arrows: Pitch down/up.
        if keys[K_UP]:
            forward = normalize(rotate_vector_around_vector(forward, right, -0.03))
            up = normalize(rotate_vector_around_vector(up, right, -0.03))
        if keys[K_DOWN]:
            forward = normalize(rotate_vector_around_vector(forward, right, 0.03))
            up = normalize(rotate_vector_around_vector(up, right, 0.03))

        # Clamp world bounds while preserving camera altitude.
        camera_x = max(-30.0, min(30.0, camera_x))
        camera_y = max(1.0, min(8.0, camera_y))
        camera_z = max(-30.0, min(30.0, camera_z))

        draw_scene()
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
