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
has no pygame/OpenGL dependency and can be tested on its own.
"""

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
            raise KeyError("object <%s> has no parameter %r" % (self.kind, name))
        return self.params.get(name, PARAM_DEFAULTS[self.kind][name])

    def to_element(self):
        """Serialise this object to an ElementTree element."""
        element = ET.Element(self.kind)
        for name, value in zip(POSITION_ATTRIBUTES, self.position.as_tuple()):
            element.set(name, _number(value))
        for name, value in zip(ORIENTATION_ATTRIBUTES, self.rotation.as_tuple()):
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
