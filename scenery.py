"""XML based description of the HeliSim virtual scenery.

Design
------
The scenery is an ordered list of *objects*.  Every object has

* a **kind** (which is also its XML tag), one of SUPPORTED_KINDS: "tree" or
  "hill",
* a **coordinate** x/y/z in metres (mandatory), and
* an **orientation** rx/ry/rz in degrees (optional, default 0),
* plus optional kind specific parameters (a uniform "scale" for a tree, the
  base and height dimensions for a hill, ...).

The flat ground plane, the grid drawn on it and the sky are *implicit*: they
are always present and therefore deliberately **not** part of the description.

XML layout (the element tag is the object kind)::

    <?xml version="1.0" encoding="UTF-8"?>
    <scenery version="1.0">
        <tree x="-8" y="0" z="-6" rx="0" ry="15" rz="0" scale="1" />
        <hill x="0" y="0" z="-38" rx="0" ry="0" rz="0"
              base_front="20" base_back="26" base_depth="20" height="9" />
    </scenery>

The orientation is applied to an object's local geometry as R = Ry * Rx * Rz,
that is yaw first, then pitch, then roll.

Serialising and reading back::

    scape = Scenery.load("sample_scenery.xml")
    scape.save("copy.xml")                  # write the description out again
    scape = Scenery.from_xml_string(scape.to_xml_string())

XML comments are not kept by a load/save round trip (they are for the human
reader of the file only).  Only the standard library is used, so this module
has no pygame/OpenGL dependency and can be tested on its own: ``python
scenery.py`` reads the project's own ``sample_scenery.xml``, lists it, writes
it out again and reads that back, shows what a shorter description leaves to
the defaults, prints what this module says about the descriptions it refuses,
and then runs the self test, which asserts all of it.
"""

import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List

# --------------------------------------------------------------------------
# Object kinds and their parameters.
# --------------------------------------------------------------------------

TREE = "tree"
HILL = "hill"

#: Object kinds the renderer knows how to draw.
SUPPORTED_KINDS = (TREE, HILL)

#: Attributes holding the coordinate (mandatory) and the orientation.
POSITION_ATTRIBUTES = ("x", "y", "z")
ORIENTATION_ATTRIBUTES = ("rx", "ry", "rz")

#: Kind specific defaults.  Every attribute of an object element that is not a
#: position or orientation attribute must be listed here; anything else is
#: rejected while loading, so typos such as hieght="3" get reported.
PARAM_DEFAULTS = {
    TREE: {
        "scale": 1.0,
    },
    HILL: {
        "base_front": 6.0,
        "base_back": 6.0,
        "base_depth": 6.0,
        "height": 2.5,
        "apex_x": 0.0,
        "apex_z": 0.0,
    },
}

ROOT_TAG = "scenery"
DEFAULT_VERSION = "1.0"

#: The scenery file this project ships and main.py loads when it is given no
#: path.  The demo and the self test round trip it: it is the format's only
#: real example, so the round trip is made over the file itself rather than
#: over a fixture, which would only check this module against itself.
SAMPLE_SCENERY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "sample_scenery.xml")


class SceneryError(ValueError):
    """Raised when a scenery description is malformed."""


def _number(value):
    """Compact, round trippable textual form of a number."""
    return "%.10g" % float(value)


def _attr_float(element, name, default=None):
    """Read the float attribute *name* of *element*, honouring *default*."""
    raw = element.get(name)
    if raw is None:
        if default is None:
            raise SceneryError("<%s> is missing the mandatory attribute %r"
                               % (element.tag, name))
        return float(default)
    try:
        return float(raw)
    except ValueError:
        raise SceneryError("<%s>: attribute %s=%r is not a number"
                           % (element.tag, name, raw))


@dataclass
class Position:
    """World coordinate of an object, in metres."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def as_tuple(self):
        return (self.x, self.y, self.z)


@dataclass
class Orientation:
    """Rotation in degrees, applied as R = Ry * Rx * Rz."""

    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    def as_tuple(self):
        return (self.rx, self.ry, self.rz)


@dataclass
class SceneryObject:
    """One placed object: kind, coordinate, orientation and parameters."""

    kind: str
    position: Position = field(default_factory=Position)
    rotation: Orientation = field(default_factory=Orientation)
    params: Dict[str, float] = field(default_factory=dict)

    def __post_init__(self):
        if self.kind not in SUPPORTED_KINDS:
            raise SceneryError("unsupported object type %r (supported: %s)"
                               % (self.kind, ", ".join(SUPPORTED_KINDS)))
        unknown = sorted(set(self.params) - set(PARAM_DEFAULTS[self.kind]))
        if unknown:
            raise SceneryError(
                "object <%s>: unknown attribute(s) %s (allowed: %s)"
                % (self.kind, ", ".join(unknown),
                   ", ".join(sorted(PARAM_DEFAULTS[self.kind]))))

    def effective_params(self):
        """Kind defaults, overridden by what the description specifies."""
        merged = dict(PARAM_DEFAULTS[self.kind])
        merged.update(self.params)
        return merged

    def param(self, name):
        """Value of one kind specific parameter (default when unspecified)."""
        if name not in PARAM_DEFAULTS[self.kind]:
            raise KeyError("object <%s> has no parameter %r"
                           % (self.kind, name))
        return self.params.get(name, PARAM_DEFAULTS[self.kind][name])

    def to_element(self):
        """Serialise this object to an ElementTree element."""
        element = ET.Element(self.kind)
        for name, value in zip(POSITION_ATTRIBUTES, self.position.as_tuple()):
            element.set(name, _number(value))
        rotation = self.rotation.as_tuple()
        for name, value in zip(ORIENTATION_ATTRIBUTES, rotation):
            element.set(name, _number(value))
        for name in sorted(self.params):
            element.set(name, _number(self.params[name]))
        return element

    @classmethod
    def from_element(cls, element):
        """Build an object from an ElementTree element."""
        if len(element):
            raise SceneryError("object <%s> must be empty, found child <%s>"
                               % (element.tag, element[0].tag))
        reserved = set(POSITION_ATTRIBUTES) | set(ORIENTATION_ATTRIBUTES)
        params = {name: _attr_float(element, name)
                  for name in element.attrib if name not in reserved}
        return cls(
            kind=element.tag,
            position=Position(*[_attr_float(element, name)
                                for name in POSITION_ATTRIBUTES]),
            rotation=Orientation(*[_attr_float(element, name, 0.0)
                                   for name in ORIENTATION_ATTRIBUTES]),
            params=params,
        )


@dataclass
class Scenery:
    """Ordered collection of scenery objects plus XML (de)serialisation."""

    objects: List[SceneryObject] = field(default_factory=list)
    version: str = DEFAULT_VERSION

    def __iter__(self):
        return iter(self.objects)

    def __len__(self):
        return len(self.objects)

    def __repr__(self):
        return "Scenery(%d object(s), version=%r)" % (len(self.objects),
                                                       self.version)

    def add(self, obj):
        """Append *obj* and return it."""
        self.objects.append(obj)
        return obj

    def to_element(self):
        """Serialise the whole scenery to an ElementTree element."""
        root = ET.Element(ROOT_TAG)
        root.set("version", self.version)
        for obj in self.objects:
            root.append(obj.to_element())
        return root

    def to_xml_string(self):
        """Serialise the scenery to an indented XML string."""
        return ET.tostring(_indent(self.to_element()), encoding="unicode")

    def to_xml_bytes(self):
        """Serialise the scenery to UTF-8 XML including the declaration."""
        declaration = '<?xml version="1.0" encoding="UTF-8"?>\n'
        return (declaration + self.to_xml_string() + "\n").encode("utf-8")

    def save(self, path):
        """Write the scenery description to *path* as an XML file."""
        with open(path, "wb") as handle:
            handle.write(self.to_xml_bytes())

    @classmethod
    def from_element(cls, root):
        """Build a scenery from the root ElementTree element."""
        if root.tag != ROOT_TAG:
            raise SceneryError("expected root element <%s>, found <%s>"
                               % (ROOT_TAG, root.tag))
        scape = cls(version=root.get("version", DEFAULT_VERSION))
        for element in root:
            scape.objects.append(SceneryObject.from_element(element))
        return scape

    @classmethod
    def from_xml_string(cls, text):
        """Build a scenery from an XML string."""
        try:
            root = ET.fromstring(text)
        except ET.ParseError as error:
            raise SceneryError("not well formed XML: %s" % (error,))
        return cls.from_element(root)

    @classmethod
    def load(cls, path):
        """Read a scenery description from the XML file *path*.

        Raises OSError when the file cannot be read and SceneryError when its
        contents are malformed.  The caller is expected to mention *path*
        itself, so these messages deliberately do not repeat it.
        """
        try:
            tree = ET.parse(path)
        except ET.ParseError as error:
            raise SceneryError("not well formed XML: %s" % (error,))
        return cls.from_element(tree.getroot())


def _indent(element, level=0, width="    "):
    """Indent *element* in place (ET.indent only exists from Python 3.9)."""
    pad = "\n" + width * level
    if len(element):
        if not (element.text or "").strip():
            element.text = pad + width
        for child in element:
            _indent(child, level + 1, width)
        if not (element[-1].tail or "").strip():
            element[-1].tail = pad
    elif level and not (element.tail or "").strip():
        element.tail = pad
    return element

# ---------------------------------------------------------------------------
# The self test and the demo, in the style of the modules around this one.
# ---------------------------------------------------------------------------


def _self_test():
    """Checks on the format: the sample round trips, the bad ones are refused.

    Raises AssertionError on failure.  The demo calls this, so running this
    module is enough to validate it.  A description can go wrong in two ways -
    it can fail to survive the trip out to XML and back, or it can be malformed
    and be accepted anyway - and both directions are checked here, the first on
    the project's own ``sample_scenery.xml`` and the second on descriptions
    written to be refused.
    """
    # The sample file is the format's documentation as well as its example, so
    # every check below is made against it rather than against a fixture built
    # here: a fixture would only check this test against itself.
    scape = Scenery.load(SAMPLE_SCENERY_FILE)
    assert scape.version == DEFAULT_VERSION
    assert len(scape) == 21, len(scape)
    kinds = [obj.kind for obj in scape]
    assert kinds.count(TREE) == 17 and kinds.count(HILL) == 4, kinds

    # The attributes the file spells out are the only ones in params: a default
    # belongs to the kind and not to the object, so what is read is what an
    # object writes back out, rather than a file filling up with defaults.
    first = scape.objects[0]
    assert first.kind == TREE
    assert first.position.as_tuple() == (-8.0, 0.0, -6.0)
    assert first.rotation.as_tuple() == (0.0, 20.0, 0.0)
    assert first.params == {"scale": 1.15}
    assert first.param("scale") == 1.15
    assert first.effective_params()["scale"] == 1.15

    # A hill that names only its dimensions still has the two apex offsets, and
    # a tree that names nothing at all still stands at scale 1 with a zero
    # orientation: the defaults apply without having to be written down.
    hill = Scenery.from_xml_string(
        '<scenery version="1.0"><hill x="0" y="0" z="-38" base_front="20"'
        ' base_back="26" base_depth="20" height="9" /></scenery>').objects[0]
    assert hill.params == {"base_front": 20.0, "base_back": 26.0,
                           "base_depth": 20.0, "height": 9.0}
    assert hill.param("apex_x") == 0.0 and hill.param("apex_z") == 0.0
    plain = Scenery.from_xml_string(
        '<scenery><tree x="1" y="2" z="3"/></scenery>')
    assert plain.version == DEFAULT_VERSION      # the root may leave it out
    bare = plain.objects[0]
    assert bare.rotation.as_tuple() == (0.0, 0.0, 0.0)      # and so may a tree
    assert bare.params == {} and bare.param("scale") == 1.0
    # effective_params hands back a fresh dictionary, so filling defaults in
    # cannot be used to edit PARAM_DEFAULTS by accident.
    filled = bare.effective_params()
    assert filled == PARAM_DEFAULTS[TREE]
    assert filled is not PARAM_DEFAULTS[TREE]
    filled["scale"] = 99.0
    assert PARAM_DEFAULTS[TREE]["scale"] == 1.0

    # In memory: the text out, the same scenery back, and the same text again.
    # The last of those is what a file a person also edits needs - a serializer
    # that drifted a little on every trip would make a load and a save a diff.
    text = scape.to_xml_string()
    again = Scenery.from_xml_string(text)
    assert again.version == scape.version and len(again) == len(scape)
    for before, after in zip(scape, again):
        assert before.kind == after.kind
        assert before.position.as_tuple() == after.position.as_tuple()
        assert before.rotation.as_tuple() == after.rotation.as_tuple()
        assert before.params == after.params
    assert again.to_xml_string() == text

    # And through a file, which is how main.py reads the scenery: the same
    # objects in the same order, behind a declaration the reader can use and
    # with the comments gone, since they belong to the file's reader rather
    # than to the description.
    with tempfile.TemporaryDirectory() as directory:
        copy = os.path.join(directory, "copy.xml")
        scape.save(copy)
        with open(copy, "rb") as handle:
            written = handle.read()
        with open(SAMPLE_SCENERY_FILE, "rb") as handle:
            sample = handle.read()
        assert written.startswith(b'<?xml version="1.0" encoding="UTF-8"?>')
        assert b"<!--" in sample and b"<!--" not in written
        assert Scenery.load(copy).to_xml_string() == text

        # A file whose XML is broken raises the same SceneryError the string
        # form does, and a file that is not there is an OSError instead: that
        # split is the one main.py catches on, OSError for the path and
        # SceneryError for what is in the file.
        broken = os.path.join(directory, "broken.xml")
        with open(broken, "w", encoding="utf-8") as handle:
            handle.write("<scenery><tree x='0' y='0' z='0'>")
        try:
            Scenery.load(broken)
        except SceneryError as error:
            assert "not well formed XML" in str(error), str(error)
        else:
            raise AssertionError("a malformed file was accepted")
        try:
            Scenery.load(os.path.join(directory, "not_there.xml"))
        except OSError as error:
            assert isinstance(error, FileNotFoundError), error
        else:
            raise AssertionError("a file that is not there was read")

    # The element of an object is its kind and its attributes are the
    # coordinate, the orientation and its parameters, in that order; all six of
    # the first two are written even when they are zero, which is what lets the
    # reader take every attribute it finds literally.
    element = first.to_element()
    assert element.tag == TREE
    assert list(element.attrib) == ["x", "y", "z", "rx", "ry", "rz", "scale"]
    assert element.get("scale") == _number(1.15)
    # _number is the compact form the format promises is round trippable - what
    # is written is what the next load reads - and compact matters: %.10g's
    # trailing zeros would otherwise grow the file on every save.
    assert _number(1.0) == "1" and _number(1.15) == "1.15"
    assert _number(1e-07) == "1e-07" and _number(-0.5) == "-0.5"
    for value in (1.15, 0.0, -8.0, 20.0, 1.05, 26.0, 1e-07, -0.5):
        assert float(_number(value)) == value, value
    # The root carries the version, and a version that is not the default is
    # what proves it travels: the sample's own is the default, so it could be
    # dropped from the file without anything above looking any different.
    odd = Scenery(version="2.5")
    odd.add(SceneryObject(TREE, Position(0.0, 0.0, 0.0)))
    assert odd.to_element().get("version") == "2.5"
    assert Scenery.from_xml_string(odd.to_xml_string()).version == "2.5"

    # The container itself: an empty scenery is legal, iteration is the order
    # the objects are held in - the order the renderer draws them - and add()
    # appends and returns the object it was given.
    empty = Scenery()
    assert len(empty) == 0 and list(empty) == []
    assert repr(empty) == "Scenery(0 object(s), version='1.0')"
    added = empty.add(SceneryObject(TREE, Position(1.0, 2.0, 3.0)))
    assert added is empty.objects[0] and len(empty) == 1
    assert added.param("scale") == 1.0 and added.params == {}
    assert len(Scenery.from_xml_string(empty.to_xml_string())) == 1
    assert Scenery.from_xml_string("<scenery/>").objects == []

    def refuses(what, text, fragment):
        """Assert that the description *text* is refused, and says why."""
        try:
            Scenery.from_xml_string(text)
        except SceneryError as error:
            assert fragment in str(error), (what, str(error))
        else:
            raise AssertionError("%s was accepted" % what)

    # Every way a description can be malformed, each with the words it is
    # reported in: a description that is quietly misread is worse than one that
    # is refused, so the message is part of what is checked.
    refuses("a root element that is not scenery", "<scape/>",
            "expected root element <scenery>, found <scape>")
    refuses("XML that is not well formed", "<scenery><tree/>",
            "not well formed XML")
    refuses("an unknown object kind",
            '<scenery><rocket x="0" y="0" z="0" /></scenery>',
            "unsupported object type 'rocket'")
    refuses("an object with no coordinate",
            '<scenery><tree y="0" z="0"/></scenery>',
            "missing the mandatory attribute 'x'")
    refuses("a coordinate that is not a number",
            '<scenery><tree x="near" y="0" z="0" /></scenery>',
            "attribute x='near' is not a number")
    refuses("a misspelled parameter",
            '<scenery><tree x="0" y="0" z="0" hieght="3" /></scenery>',
            "unknown attribute(s) hieght")
    refuses("something inside an object",
            '<scenery><tree x="0" y="0" z="0"><trunk /></tree></scenery>',
            "must be empty, found child <trunk>")

    # The object's own guard, for a caller building one in Python and not in
    # XML, and the parameter lookup's, for a name the kind has not got.
    try:
        SceneryObject("rocket")
    except SceneryError as error:
        assert "unsupported object type 'rocket'" in str(error), str(error)
    else:
        raise AssertionError("an object of an unknown kind was built")
    try:
        SceneryObject(TREE, params={"hieght": 3.0})
    except SceneryError as error:
        assert "unknown attribute(s) hieght" in str(error), str(error)
    else:
        raise AssertionError("an object with a misspelled parameter was built")
    try:
        SceneryObject(TREE).param("height")
    except KeyError:
        pass
    else:
        raise AssertionError("a tree was asked for a hill's parameter")


def _demo():
    """Print what the scenery module does: a description, in and out of XML.

    The description on show is the project's own ``sample_scenery.xml`` - read,
    listed, written out again and read back, because a format is only as good
    as that trip.  What follows is what a shorter description leaves to the
    defaults, what this module says about the descriptions it refuses, and then
    the self test, which asserts everything the demo has just shown.
    """
    with open(SAMPLE_SCENERY_FILE, "rb") as handle:
        sample = handle.read()
    scape = Scenery.load(SAMPLE_SCENERY_FILE)
    print("the scenery of %s: %d bytes of XML, version %s, %d objects"
          % (os.path.basename(SAMPLE_SCENERY_FILE), len(sample),
             scape.version, len(scape)))
    print("in the order the file gives them - the order the renderer draws")
    print("them in.  The ground plane and its grid are implicit, not here.")
    print()
    print("    %-5s %7s %7s %7s  %6s %6s %6s  %s"
          % ("kind", "x", "y", "z", "rx", "ry", "rz", "what the file says"))
    for obj in scape:
        numbers = [_number(value) for value in (obj.position.as_tuple()
                                                + obj.rotation.as_tuple())]
        params = " ".join("%s=%s" % (name, _number(obj.params[name]))
                          for name in sorted(obj.params))
        print("    %-5s %7s %7s %7s  %6s %6s %6s  %s"
              % tuple([obj.kind] + numbers + [params]))

    # Round tripping through a temporary file rather than through memory only,
    # since a file is what main.py is handed and what a reader edits.
    text = scape.to_xml_string()
    print()
    print("written out again and read back, which is the whole of the format:")
    with tempfile.TemporaryDirectory() as directory:
        copy = os.path.join(directory, "copy.xml")
        scape.save(copy)
        with open(copy, "rb") as handle:
            written = handle.read()
        reloaded = Scenery.load(copy)
        print("  %d bytes written, in UTF-8 and behind the declaration,"
              % len(written))
        print("  and %d objects read back from them: the same %d that went in,"
              % (len(reloaded), len(scape)))
        print("  in the same order.")
        print("  writing those out again gives %s, so a round trip through a"
              % ("the same text" if reloaded.to_xml_string() == text
                 else "DIFFERENT TEXT"))
        print("  file is not a diff.")
        print("  the %d comments the sample carries for a reader come back as"
              % sample.count(b"<!--"))
        print("  %d, since a comment is not part of the description."
              % written.count(b"<!--"))

    print()
    print("a description may leave the defaults out:")
    print("the kind fills them in, so a short file stays short:")
    bare = Scenery.from_xml_string(
        '<scenery><tree x="4" y="0" z="4"/></scenery>').objects[0]
    print("  a tree that names only its coordinate still has the kind's")
    print("  defaults: orientation %s at %s, and scale %.1f."
          % (bare.rotation.as_tuple(), bare.position.as_tuple(),
             bare.param("scale")))
    hill = Scenery.from_xml_string(
        '<scenery><hill x="0" y="0" z="-38" base_front="20" base_back="26"'
        ' base_depth="20" height="9"/></scenery>').objects[0]
    print("  a hill that names its four dimensions gets the other two: %s"
          % ", ".join("%s=%s" % (name, _number(value)) for name, value in
                      sorted(hill.effective_params().items())))

    print()
    print("and the descriptions it refuses, and what it says about each.")
    print("The words matter: a description read quietly and wrongly is the")
    print("one worth having a refusal for.")

    def refuse(what, text):
        try:
            Scenery.from_xml_string(text)
        except SceneryError as error:
            print("  %s:" % what)
            print("    %s" % error)
        else:
            raise AssertionError("%s was accepted" % what)

    refuse("a root element that is not scenery", "<scape/>")
    refuse("XML that is not well formed", "<scenery><tree/>")
    refuse("an unknown object kind",
           '<scenery><rocket x="0" y="0" z="0" /></scenery>')
    refuse("an object with no coordinate",
           '<scenery><tree y="0" z="0"/></scenery>')
    refuse("a coordinate that is not a number",
           '<scenery><tree x="near" y="0" z="0" /></scenery>')
    refuse("a misspelled parameter",
           '<scenery><tree x="0" y="0" z="0" hieght="3" /></scenery>')
    refuse("something inside an object",
           '<scenery><tree x="0" y="0" z="0"><trunk /></tree></scenery>')

    print()
    print("a file that is not there is an OSError rather than a SceneryError,")
    print("which is the split main.py catches on to report a bad path:")
    try:
        Scenery.load(os.path.join(tempfile.gettempdir(), "not_a_scenery.xml"))
    except OSError as error:
        print("  %s" % error)

    print()
    _self_test()
    print("self test passed")


if __name__ == "__main__":
    _demo()

