"""The ground as a landscape: a table of heights, read as flat patches.

Design
------
The model asks the ground one question at a time - how far is this point above
the surface - and :class:`airframe.GroundPlane` answers it for one flat patch of
it: a level, a normal, and the four questions a caller asks of them.  What a
plane cannot be is a *landscape*.  A ridge, a valley, a step, or any surface
whose level varies under the aircraft is several planes and the seam between
them, and this module is that seam written down.

A :class:`Terrain` is a *heightfield*: a rectangular table of sample heights
over the horizontal plane, :attr:`heights` indexed north and then east -
``heights[i][j]`` is the surface at ``x = origin.x + i * spacing[0]`` and
``y = origin.y + j * spacing[1]``, a height measured up from the NED origin the
way :meth:`airframe.GroundPlane.level` measures one - and the surface between
those samples is **piecewise flat**: every cell of the table is cut into two
triangles along its own diagonal, and each triangle is one
:class:`airframe.GroundPlane` through its three samples.  That is the whole
design.  A landscape is the single flat patch this project already flies, many
times over, so a point of the surface has one patch under it and that patch
answers every question the model asks of the ground - equation 10a's Z, the
floor's skid line, the levelling and ``Telemetry.alt_agl`` - with exactly the
arithmetic a single plane has always used, and there is no second way a height
is ever computed.

**Where the seam is.**  The two patches of one cell meet along the cell's
diagonal, from its south-west sample to its north-east one, and the patches of
two neighbouring cells meet along the grid line between them.  Across both, the
surface is continuous but not smooth: the patches share their edge samples, so
no height jumps, while the lean does - which is what a landscape is, and what a
pilot parked on a ridge's crest feels.  Two properties follow, and neither of
them is promised by this docstring: they are the patch's own arithmetic.  A
sample of the table lies *on* the surface, because it is a corner of every patch
that touches it; and an aircraft levelled on a slope stands level *on that
patch*, because the levelling is
:meth:`airframe.GroundPlane.level_attitude`, whose answer is the patch's own
normal and the heading it is asked at.

**Off the map.**  A table is finite and a flight is not, so a position outside
it is answered by the nearest *cell's* own patch, its plane carried on: the
ground past the table is the table's own edge continued, which is one rule with
nothing special at the map's edge and no crease in it, since the boundary cell's
patch contains the boundary itself.  A ridge walked off the side of the table
therefore goes on rising, which is what more ground is; a caller who wants it to
stop says so with a wider table, or with
:meth:`airframe.GroundPlane.level`, which is what a table with no landscape in
it was worth in the first place.

**What this is not.**  Every patch is flat and the table is a *function* of
position: no hole, no overhang, no vertical face and no cave, since a heightfield
has one surface over every place.  A *deck* that pitches, or any ground that
moves under the aircraft, is not a table at all and wants to be a single
:class:`airframe.GroundPlane` handed to ``Airframe.ground`` between frames - the
same interface both kinds answer, and the reason
:meth:`airframe.GroundPlane.patch` exists: the ground is asked for the patch
under the aircraft and the patch answers everything else, so a terrain, which has
no one lean and no one normal to hand back, has no ``level_attitude`` and no
``normal`` of its own.

``main.py`` draws it, and draws it as this table: its world is a landscape of
the module's own ridge, its grass is the table's own cells, its grids are drawn
on the surface, and the shadow and the two cameras that stand on the ground read
the ground's own height rather than the world's zero - see :func:`main.draw_ground`
and its neighbours for what the renderer asks the table and why it reads the
table rather than a patch at every place it draws.  Its scenery's hills are still
objects drawn rather than flown, which ``TODO.md`` carries.  What a run can do
now is *fly* and *show* a landscape, and ``python terrain.py`` flies one: it
builds a ridge, prints what its table and its patches say about it, sits the
report's own aircraft in the cushion at the crest and hovers it there.

Only the standard library is used, so this module has no pygame and no OpenGL
dependency and can be tested headless like the model around it.
"""
import math
from dataclasses import dataclass
from typing import Sequence

from aerodynamics import Vector3
from airframe import GroundPlane
from simulation import (SIM_TIME_STEP_S, UH1_CG_HEIGHT_ON_GROUND, Simulation,
                        airframe_preset)


#: Fewest samples a terrain may have along an axis.  A single sample is a level
#: surface and no landscape, and a table with one row or one column has no *cell*
#: for a patch to be built from - so two is the smallest table there is.
TERRAIN_MIN_SAMPLES = 2


@dataclass(frozen=True)
class Terrain:
    """A landscape: a table of sample heights and the flat patch under a point.

    *heights* is the table, any sequence of equal length rows of numbers: the
    first index runs north and the second east, so ``heights[i][j]`` is the
    surface at ``x = origin.x + i * spacing[0]``, ``y = origin.y + j *
    spacing[1]``, and the number is the height there in metres *up* from the NED
    origin - the sign :meth:`airframe.GroundPlane.level` uses, so a height of
    120 m is a surface at z = -120.

    *spacing* is the distance between neighbouring samples along each axis, m,
    north then east.  *origin* is where sample (0, 0) stands, of which only x
    and y are read: a heightfield is described in the horizontal plane, and an
    origin with a z of its own describes no surface, so it is refused rather
    than ignored.

    What the object is made of is :meth:`patch`: the flat triangle of the
    surface under a position, which is a :class:`airframe.GroundPlane` and
    answers every question the model asks.  :meth:`height`,
    :meth:`surface_point` and :meth:`above` are the three the ground itself is
    asked, each of them answered by asking the patch.
    """

    heights: Sequence[Sequence[float]] = ()
    spacing: Sequence[float] = (1.0, 1.0)
    origin: Vector3 = Vector3()

    def __post_init__(self):
        rows = tuple(self.heights)
        if not rows or not rows[0]:
            raise ValueError("a terrain is a table of heights, not an empty one")
        widest = len(rows[0])
        if len(rows) < TERRAIN_MIN_SAMPLES or widest < TERRAIN_MIN_SAMPLES:
            raise ValueError(
                "a terrain needs at least %d samples along each axis, not %d by"
                " %d: a single sample is a level surface and no landscape"
                % (TERRAIN_MIN_SAMPLES, len(rows), widest))
        for row in rows:
            if len(row) != widest:
                raise ValueError(
                    "a terrain's table is ragged: it is %d samples wide here and"
                    " %d there, and a grid has one width"
                    % (len(row), widest))
        try:
            values = tuple(tuple(float(height) for height in row)
                           for row in rows)
        except (TypeError, ValueError):
            raise ValueError("a terrain's heights are numbers in metres, not %r"
                             % (self.heights,))
        for row in values:
            for height in row:
                if not math.isfinite(height):
                    raise ValueError(
                        "a height that is not a number describes no surface: %r"
                        % (height,))
        spacing = tuple(float(step) for step in self.spacing)
        if len(spacing) != 2:
            raise ValueError("a terrain has one spacing per axis, not %r"
                             % (self.spacing,))
        for step in spacing:
            if not math.isfinite(step) or step <= 0.0:
                raise ValueError(
                    "a terrain's spacing is a positive distance in metres along"
                    " each axis, not %r" % (self.spacing,))
        if self.origin.z != 0.0:
            raise ValueError(
                "a terrain is a heightfield over the horizontal plane: the"
                " origin's own z (%r) is not a height and describes no surface"
                % (self.origin.z,))
        object.__setattr__(self, "heights", values)
        object.__setattr__(self, "spacing", spacing)

    @property
    def nx(self):
        """How many samples the table has along its north axis."""
        return len(self.heights)

    @property
    def ny(self):
        """How many samples the table has along its east axis."""
        return len(self.heights[0])

    @property
    def x_range(self):
        """``(south, north)``, the two ends of the table's north axis, m."""
        return (self.origin.x, self.origin.x + (self.nx - 1) * self.spacing[0])

    @property
    def y_range(self):
        """``(west, east)``, the two ends of the table's east axis, m."""
        return (self.origin.y, self.origin.y + (self.ny - 1) * self.spacing[1])

    def sample(self, i, j):
        """The height the table holds at sample *(i, j)*, m.

        An index off the table is refused rather than wrapped: a negative one
        would otherwise quietly mean the far side of the map, which in a table
        of *places* is a mistake rather than a shorthand.
        """
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            raise IndexError("sample (%d, %d) is off a table that is %d by %d"
                             % (i, j, self.nx, self.ny))
        return self.heights[i][j]

    def sample_position(self, i, j):
        """Where sample *(i, j)* stands in NED, m: the surface's own point."""
        return Vector3(self.origin.x + i * self.spacing[0],
                       self.origin.y + j * self.spacing[1],
                       -self.sample(i, j))

    def patch(self, position):
        """The flat patch under *position*, a :class:`airframe.GroundPlane`.

        The seam this module is built on.  The position's own north and east
        coordinates name a cell of the table - the nearest one where it is off
        the map - and the cell is cut along its diagonal, from its south-west
        sample to its north-east one, into two triangles; the one the position's
        own place *inside* the cell falls in is the patch, built as a plane
        through that triangle's three sample positions.

        The two triangles are the cell's own arithmetic and nothing else.
        ``fu >= fv`` is the south-west half - the south-west sample, the
        north-east one and the north-west one, in that order - and the other
        triangle is its mirror.  Those orders are what the normals come out of,
        and they are the order that stands *out* of the ground rather than into
        it: a heightfield's patch never contains a vertical line, so its
        normal's z is always negative, which in NED is up.
        """
        i, j, fu, fv = self._cell_of(position)
        south_west = self.sample_position(i, j)
        north_east = self.sample_position(i + 1, j + 1)
        if fu >= fv:
            north_west = self.sample_position(i + 1, j)
            return GroundPlane(
                south_west,
                (north_east - south_west).cross(north_west - south_west))
        south_east = self.sample_position(i, j + 1)
        return GroundPlane(
            south_west,
            (south_east - south_west).cross(north_east - south_west))

    def _cell_of(self, position):
        """``(i, j, fu, fv)``: the cell under *position* and its place in it.

        The *cell* is clamped to the table - the nearest one where the position
        is off the map - while *fu* and *fv* are the position's own place inside
        that cell, unclamped, which is what makes the ground past the table the
        edge patch's own plane carried on rather than a second rule.

        The place is measured in samples: ``fu`` is how many cell widths north
        of the cell's south-west sample the position stands, and ``fv`` the same
        east.  Inside the cell the two are in [0, 1) and the diagonal is the
        line ``fu == fv``.
        """
        u = (position.x - self.origin.x) / self.spacing[0]
        v = (position.y - self.origin.y) / self.spacing[1]
        i = min(max(int(math.floor(u)), 0), self.nx - 2)
        j = min(max(int(math.floor(v)), 0), self.ny - 2)
        return i, j, u - i, v - j

    def height(self, position):
        """How far *position* stands above the surface, m, along the normal.

        The patch under the position answers it, which is what makes a terrain
        and a single plane the same ground to everything that asks: the ground
        effect's Z, a skid's clearance and ``Telemetry.alt_agl`` all arrive here.
        """
        return self.patch(position).height(position)

    def surface_point(self, position):
        """The point of the surface nearest *position*, NED m."""
        return self.patch(position).surface_point(position)

    def above(self, position, height):
        """The point *height* m over the surface, over *position*, NED m.

        *position* chooses the patch - the cell under it - and the answer is
        that patch's own point *height* up its own normal, which is where an
        aircraft's c.g. stands when its skids are on this ground.
        """
        return self.patch(position).above(position, height)

    @classmethod
    def from_function(cls, surface, x_range, y_range, step):
        """A terrain sampled from a formula: a ridge as a function of position.

        *surface* is called as ``surface(x, y)`` with a sample's north and east
        coordinates in metres and answers the height there, in metres; any
        formula over the plane is one, and a table held in a file is the
        constructor's own business.  The grid is the two ranges - ``(south,
        north)`` and ``(west, east)`` - sampled at about *step* metres: the
        sample count is the range over the step rounded to the whole number of
        intervals nearest it, and the *spacing* the table gets is then the range
        over that many intervals, so the table covers the ranges exactly and no
        sample stands off either end of them.

        Two samples along an axis is the least a terrain can be, so a range
        narrower than a step is sampled at its own two ends rather than refused.
        """
        x0, x1 = (float(end) for end in x_range)
        y0, y1 = (float(end) for end in y_range)
        if not x1 > x0 or not y1 > y0:
            raise ValueError(
                "a terrain's ranges are (low, high) pairs of metres, not %r and"
                " %r" % (tuple(x_range), tuple(y_range)))
        step = float(step)
        if not math.isfinite(step) or step <= 0.0:
            raise ValueError("a terrain's step is a positive distance in metres,"
                             " not %r" % (step,))
        nx = max(TERRAIN_MIN_SAMPLES, int(round((x1 - x0) / step)) + 1)
        ny = max(TERRAIN_MIN_SAMPLES, int(round((y1 - y0) / step)) + 1)
        dx = (x1 - x0) / (nx - 1)
        dy = (y1 - y0) / (ny - 1)
        heights = tuple(
            tuple(surface(x0 + i * dx, y0 + j * dy) for j in range(ny))
            for i in range(nx))
        return cls(heights, (dx, dy), Vector3(x0, y0))


# ---------------------------------------------------------------------------
# The landscape this module's own demo and self test fly over.
# ---------------------------------------------------------------------------

#: The ridge ``python terrain.py`` builds: a tent 120 m high running east and
#: west, its crest 600 m from its feet either side of it, sampled every 200 m
#: over a 2.4 km square.  Those samples land on the crease and on both feet, so
#: the surface the table describes is the tent itself, face for face, and the
#: patch either side of the crest is exactly one of its two faces.
RIDGE_HEIGHT_M = 120.0
RIDGE_HALF_WIDTH_M = 600.0
RIDGE_STEP_M = 200.0
RIDGE_HALF_SPAN_M = 1200.0


def ridge(x, y):
    """The demo's ridge: the ground's height, m, at ``(x, y)``, NED m.

    A tent.  Full height along the crest - the line ``x = 0``, which runs north
    to south - falling linearly to nothing at 600 m either side of it, and level
    along every line of constant x, so it is a ridge and not a hill.

    It is written as (height * reach) / half width rather than as a height times
    a fraction, so that the demo's own samples land on it *exactly*: the table
    then describes the tent itself rather than a surface a rounding away from
    it, which is what lets the self test compare the two with ``==``.
    """
    reach = RIDGE_HALF_WIDTH_M - abs(x)
    return RIDGE_HEIGHT_M * max(0.0, reach) / RIDGE_HALF_WIDTH_M


def _place(terrain, i, j, fu, fv):
    """A position standing inside cell *(i, j)* of *terrain* at ``(fu, fv)``.

    ``fu`` and ``fv`` are the cell's own north and east coordinates in cell
    widths, so the two triangles of a cell are reached by asking for a position
    either side of ``fu == fv``.  The z of the position describes nothing: only
    north and east choose a patch.
    """
    return Vector3(terrain.origin.x + (i + fu) * terrain.spacing[0],
                   terrain.origin.y + (j + fv) * terrain.spacing[1],
                   0.0)


def _demo():
    """Build a ridge, print what its table says, then park the model on it.

    The whole module in one run: the landscape and one of its patches, the seam
    at the crest, and then the report's own aircraft in the cushion there and
    lifting off it.  It ends by running the self test, so ``python terrain.py``
    asserts everything it has just printed.
    """
    terrain = Terrain.from_function(
        ridge, (-RIDGE_HALF_SPAN_M, RIDGE_HALF_SPAN_M),
        (-RIDGE_HALF_SPAN_M, RIDGE_HALF_SPAN_M), RIDGE_STEP_M)
    print("A terrain is a table of heights, read as flat patches:")
    print("  %d samples north and %d east, %g m apart: %g m of ground each way"
          % (terrain.nx, terrain.ny, terrain.spacing[0], RIDGE_HALF_SPAN_M))
    print("  a ridge %g m high with %g m from crest to foot, so the surface"
          " through the samples" % (RIDGE_HEIGHT_M, RIDGE_HALF_WIDTH_M))
    print("  is the tent itself: the crest sample is %g m up, the next one north"
          " of it %g m," % (terrain.sample(6, 3), terrain.sample(7, 3)))
    print("  and both of its feet - %d samples out - are %g m: the table and the"
          " tent meet exactly"
          % (6, terrain.sample(3, 3)))
    face = terrain.patch(_place(terrain, 6, 6, 0.5, 0.5))
    print("  the patch under a point on the crest's north face leans %+.2f deg,"
          " its normal (%+.4f, %+.4f, %+.4f)"
          % (math.degrees(face.level_attitude(0.0).y), face.normal.x,
             face.normal.y, face.normal.z))
    south = terrain.patch(_place(terrain, 5, 6, 0.5, 0.5))
    print("  and the cell south of the crest leans %+.2f deg: the crest is the"
          " seam, and it is the lean that steps"
          % math.degrees(south.level_attitude(0.0).y))
    print("  a park on the ridge's own face is level on it: at 90 deg along the"
          " crest the same face is %+.2f deg of roll"
          % math.degrees(face.level_attitude(math.pi / 2.0).x))
    print()
    over = Simulation(airframe=airframe_preset(ground=terrain))
    telemetry = over.telemetry()
    print("  the report's own aircraft, trimmed 200 m over the crest's patch:"
          " alt_agl %.2f m, pitch %+.2f deg, cushion %.4f"
          % (telemetry.alt_agl, telemetry.pitch_deg, telemetry.ground_effect))
    over.on_the_pad()
    for _ in range(240):
        over.step(SIM_TIME_STEP_S)
    telemetry = over.telemetry()
    print("  four seconds of a park on the crest: alt_agl %.2f m, pitch %+.2f"
          " deg, cushion %.4f - the face itself, felt through the skids"
          % (telemetry.alt_agl, telemetry.pitch_deg, telemetry.ground_effect))
    for _ in range(600):
        over.fly(SIM_TIME_STEP_S, collective=1.0)
    telemetry = over.telemetry()
    print("  ten seconds of the lever up: alt_agl %.1f m, %.1f m/s over the"
          " ground, %s"
          % (telemetry.alt_agl, telemetry.ground_speed,
             "still on the skids" if telemetry.on_ground else "off the crest"))

    print()
    _self_test()
    print("self test passed")


def _self_test():
    """The table, its patches, the seam, the map's edge, and a flight over it."""
    terrain = Terrain.from_function(
        ridge, (-RIDGE_HALF_SPAN_M, RIDGE_HALF_SPAN_M),
        (-RIDGE_HALF_SPAN_M, RIDGE_HALF_SPAN_M), RIDGE_STEP_M)

    # The table, which is the landscape: 13 samples each way 200 m apart, and
    # the samples land on the tent's crease and on both of its feet, so what the
    # linear interpolation between them describes is the tent itself.
    assert terrain.nx == 13 and terrain.ny == 13
    assert terrain.spacing == (200.0, 200.0)
    assert terrain.x_range == (-1200.0, 1200.0)
    assert terrain.y_range == (-1200.0, 1200.0)
    assert terrain.sample(6, 3) == RIDGE_HEIGHT_M
    assert terrain.sample(5, 3) == terrain.sample(7, 3) \
        == 2.0 * RIDGE_HEIGHT_M / 3.0
    assert terrain.sample(4, 3) == terrain.sample(8, 3) \
        == RIDGE_HEIGHT_M / 3.0
    assert terrain.sample(3, 3) == 0.0 and terrain.sample(12, 3) == 0.0
    assert terrain.sample_position(6, 0).as_tuple() == (0.0, -1200.0, -120.0)

    # Every sample lies *on* the surface, to the nanometre: it is a corner of
    # every patch that touches it, so the surface passes through its own table.
    for i in range(terrain.nx):
        for j in range(terrain.ny):
            assert abs(terrain.height(terrain.sample_position(i, j))) < 1e-9

    # Every patch stands out of the ground and is a unit normal, over the whole
    # map and on both sides of every diagonal: the vertex order is what makes
    # that true rather than a sign fixed up after the fact.
    for i in range(terrain.nx - 1):
        for j in range(terrain.ny - 1):
            for fu, fv in ((0.25, 0.75), (0.75, 0.25), (0.5, 0.5)):
                normal = terrain.patch(_place(terrain, i, j, fu, fv)).normal
                assert normal.z < 0.0
                assert abs(normal.length() - 1.0) < 1e-15

    # The crest is the seam, and what steps across it is the lean: the two cells
    # either side of the crease are the tent's two faces, mirrored in north and
    # leaning by the same angle, while the surface itself does not jump - each
    # face's own patch holds the crest's samples.
    north_face = terrain.patch(_place(terrain, 6, 6, 0.5, 0.5))
    south_face = terrain.patch(_place(terrain, 5, 6, 0.5, 0.5))
    slope = RIDGE_HEIGHT_M / RIDGE_HALF_WIDTH_M
    assert abs(north_face.normal.x + south_face.normal.x) < 1e-12
    assert abs(north_face.normal.z - south_face.normal.z) < 1e-12
    assert abs(abs(north_face.normal.x) - math.sin(math.atan(slope))) < 1e-12
    for face in (north_face, south_face):
        for j in range(terrain.ny):
            assert abs(face.height(terrain.sample_position(6, j))) < 1e-9
    # The patch under a point of a face *is* that face: the point of the tent
    # 100 m north of the crest is 100 m up, and the patch there says so.
    assert abs(north_face.height(Vector3(100.0, 0.0, -100.0))) < 1e-9

    # A cell whose own four samples lie in one plane is one plane, diagonal or
    # not - the tent is linear across every cell of this table - while a cell
    # that is not planar has its diagonal as a seam like any other: the two
    # patches meet along it, to the nanometre, and lean differently.
    across = [terrain.patch(_place(terrain, 3, 3, 0.25, 0.75)),
              terrain.patch(_place(terrain, 3, 3, 0.75, 0.25))]
    assert (across[0].normal - across[1].normal).length() < 1e-15
    saddle = Terrain(((0.0, 100.0), (100.0, 0.0)), spacing=(200.0, 200.0))
    halves = [saddle.patch(_place(saddle, 0, 0, 0.3, 0.7)),
              saddle.patch(_place(saddle, 0, 0, 0.7, 0.3))]
    assert (halves[0].normal - halves[1].normal).length() > 0.1
    for half in halves:
        assert abs(half.height(Vector3(100.0, 100.0, 0.0))) < 1e-9

    # What a table with no landscape in it is: the plane the model flew before
    # there was a terrain, and the same arithmetic to the last bit - the patch's
    # normal is exactly (0, 0, -1) when every sample is level, so the height,
    # the surface point, the point above it and the levelling are the plane's
    # own answers rather than merely close to them.
    flat = Terrain.from_function(lambda x, y: 200.0, (-1200.0, 1200.0),
                                 (-1200.0, 1200.0), 200.0)
    level = GroundPlane.level(200.0)
    for position in (Vector3(0.0, 0.0, -200.0),
                     Vector3(37.5, -812.25, -13.75),
                     Vector3(-0.5, 0.5, -1201.0),
                     Vector3(4000.0, -9000.0, -300.0)):
        assert flat.height(position) == level.height(position)
        assert flat.surface_point(position) == level.surface_point(position)
        assert flat.above(position, UH1_CG_HEIGHT_ON_GROUND) \
            == level.above(position, UH1_CG_HEIGHT_ON_GROUND)
        assert flat.patch(position).level_attitude(37.0) \
            == level.level_attitude(37.0)

    # A ramp is a table too, and every patch in it is that one plane: a linear
    # surface is what linear interpolation reproduces exactly, so a slope is
    # flyable over a terrain by making the terrain the slope.
    lean = math.radians(15.0)
    ramp = Terrain.from_function(lambda x, y: -math.tan(lean) * x,
                                 (-600.0, 600.0), (-600.0, 600.0), 150.0)
    ramp_plane = GroundPlane(
        normal=Vector3(math.sin(lean), 0.0, -math.cos(lean)))
    for position in (Vector3(0.0, 0.0, -200.0),
                     Vector3(-450.0, 275.0, -37.5),
                     Vector3(3000.0, 0.0, -20.0)):
        assert abs(ramp.height(position) - ramp_plane.height(position)) < 1e-9
        assert abs(ramp.patch(position).normal.x - ramp_plane.normal.x) < 1e-12
        assert abs(ramp.patch(position).normal.z - ramp_plane.normal.z) < 1e-12
    down_the_ramp = ramp.patch(Vector3()).level_attitude(0.0)
    assert down_the_ramp.x == 0.0
    assert abs(math.degrees(down_the_ramp.y) + 15.0) < 1e-9

    # Off the map the nearest cell's own patch carries on, which is one rule and
    # no crease: a point past the table's edge is answered by the patch of the
    # cell nearest to it, and that patch's own plane holds the edge - so the ramp
    # goes on rising past its own edge, and the ridge's own foot is the level
    # ground the table runs out on.
    for outside, nearest in (
            (Vector3(601.0, 0.0, 0.0), Vector3(599.0, 0.0, 0.0)),
            (Vector3(50000.0, -90000.0, 0.0), Vector3(599.0, -599.0, 0.0))):
        assert ramp.patch(outside) == ramp.patch(nearest)
        assert abs(ramp.height(outside) - ramp_plane.height(outside)) < 1e-6
    for outside, nearest in (
            (Vector3(1201.0, 0.0, 0.0), Vector3(1199.0, 0.0, 0.0)),
            (Vector3(50000.0, -90000.0, 0.0), Vector3(1199.0, -1199.0, 0.0))):
        assert terrain.patch(outside) == terrain.patch(nearest)
    assert abs(terrain.surface_point(Vector3(5000.0, 0.0, -30.0)).z) < 1e-9

    # The levelling over a landscape is the heading's business as much as the
    # ground's: down the fall line a ridge's face is pitch and no roll, and along
    # the crease the same angle of roll and no pitch - to the rounding of cos 90,
    # which is 6e-17 rather than zero.
    down_the_face = north_face.level_attitude(0.0)
    along_the_face = north_face.level_attitude(math.pi / 2.0)
    assert abs(down_the_face.x) < 1e-15
    assert abs(along_the_face.y) < 1e-15
    assert abs(abs(down_the_face.y) - abs(along_the_face.x)) < 1e-12
    assert abs(math.degrees(down_the_face.y)
               + math.degrees(math.atan(slope))) < 1e-9

    # What this module refuses, and in its own words: a table with no cell in it,
    # a ragged one, a height that is not a number, a spacing that is not a
    # distance, an origin that names a height rather than a place; and, from the
    # other end, a range that is not a span and a step that is not a length.
    table = ((0.0, 1.0), (2.0, 3.0))
    for bad, refused in (
            (dict(heights=()), "a table with no table in it"),
            (dict(heights=((0.0,), (1.0,))), "a table one column wide"),
            (dict(heights=((0.0, 1.0),)), "a table one row deep"),
            (dict(heights=((0.0, 1.0), (2.0,))), "a ragged table"),
            (dict(heights=((0.0, 1.0), (2.0, float("nan")))),
             "a height that is not a number"),
            (dict(heights=((0.0, 1.0), (2.0, "high"))),
             "a height that is not a number at all"),
            (dict(heights=table, spacing=(0.0, 1.0)), "a zero spacing"),
            (dict(heights=table, spacing=(10.0, -1.0)), "a negative spacing"),
            (dict(heights=table, spacing=(10.0,)), "one spacing for two axes"),
            (dict(heights=table, origin=Vector3(0.0, 0.0, -5.0)),
             "an origin with a z of its own")):
        try:
            Terrain(**bad)
        except ValueError:
            pass
        else:
            raise AssertionError("a terrain took %s" % (refused,))
    for bad, refused in ((((0.0, 0.0), (0.0, 1.0), 100.0), "a range with no span"),
                         (((0.0, 1.0), (1.0, 0.0), 100.0), "a reversed range"),
                         (((0.0, 1.0), (0.0, 1.0), 0.0), "a zero step"),
                         (((0.0, 1.0), (0.0, 1.0), -3.0), "a negative step")):
        try:
            Terrain.from_function(ridge, *bad)
        except ValueError:
            pass
        else:
            raise AssertionError("a terrain sampled %s" % (refused,))
    try:
        terrain.sample(-1, 0)
    except IndexError:
        pass
    else:
        raise AssertionError("a sample off the table was wrapped round")
    # A range narrower than a step is *not* refused: it is sampled at its own two
    # ends, which is the least table there is, and it still covers the range.
    narrow = Terrain.from_function(ridge, (-10.0, 10.0), (-5.0, 5.0), 500.0)
    assert narrow.nx == TERRAIN_MIN_SAMPLES == narrow.ny
    assert narrow.spacing == (20.0, 10.0)
    assert narrow.x_range == (-10.0, 10.0) and narrow.y_range == (-5.0, 5.0)

    # The whole of it: the report's own aircraft, on a landscape, flown by the
    # frame loop.  Trimmed 200 m over the crest's own patch it holds that hover
    # as it holds one over level ground, because the ground is one patch at a
    # time and this one is the crest's.
    over = Simulation(airframe=airframe_preset(ground=terrain))
    assert abs(over.telemetry().alt_agl - 200.0) < 1e-9
    assert over.in_trim() and not over.on_ground
    for _ in range(120):
        over.step(SIM_TIME_STEP_S)
    assert over.in_trim() and not over.crashed and not over.on_ground
    assert abs(over.telemetry().alt_agl - 200.0) < 1.0

    # Parked on the crest: the skids stand on the *surface*, the c.g. is the
    # skid line up the patch's own normal, and the floor levels the aircraft to
    # the face's own level - a nose down attitude that is level on this face and
    # level to nothing on the flat ground beside it.  The heading is free as
    # ever, so what is asserted is the patch's own levelling at whatever heading
    # the tail rotor has walked it round to.
    over.on_the_pad()
    assert over.on_ground and not over.crashed
    assert abs(terrain.height(over.airframe.state.position)
               - UH1_CG_HEIGHT_ON_GROUND) < 1e-12
    standing = over.airframe.state.position
    for _ in range(240):
        over.step(SIM_TIME_STEP_S)
    assert over.on_ground and not over.crashed
    assert (over.airframe.state.position - standing).length() < 1e-9
    assert abs(terrain.height(over.airframe.state.position)
               - UH1_CG_HEIGHT_ON_GROUND) < 1e-9
    standing_lean = terrain.patch(over.airframe.state.position).level_attitude(
        over.airframe.state.attitude.z)
    assert abs(math.degrees(standing_lean.y)) > 10.0
    assert abs(math.degrees(over.airframe.state.attitude.y
                            - standing_lean.y)) < 0.2
    assert abs(math.degrees(over.airframe.state.attitude.x
                            - standing_lean.x)) < 0.2
    # The cushion it sits in is the crest's own, and it is the same cushion a
    # park on level ground gets: the floor holds it the same height over the
    # *surface*, so a ridge's crest cushions exactly as the flat does.
    level_pad = Simulation()
    level_pad.on_the_pad()
    for _ in range(240):
        level_pad.step(SIM_TIME_STEP_S)
    for settled in (over, level_pad):
        assert settled.on_ground
        settled.forces = settled.airframe.step(0.0, settled.controls)
    assert abs(over.airframe.hub_height()
               - level_pad.airframe.hub_height()) < 1e-6
    assert abs(over.telemetry().ground_effect
               - level_pad.telemetry().ground_effect) < 1e-7

    # And the lever still lifts it off the crest, which is what a landscape is
    # for: ten seconds of it and the aircraft is flying over its own ridge.
    for _ in range(600):
        over.fly(SIM_TIME_STEP_S, collective=1.0)
    assert not over.on_ground and not over.crashed
    assert terrain.height(over.airframe.state.position) > 10.0


if __name__ == "__main__":
    _demo()




