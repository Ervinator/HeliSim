"""Which physical control flies which: the pilot's devices, described in XML.

Design
------
The sim reads four axes - the collective, the longitudinal and lateral cyclic
and the pedals - and a fifth control a real UH-1 has but this model does not yet
fly, the throttle.  *This* module does not read them: it says **where each one
comes from**, on a device by device and control by control basis, so that a run
can be flown from a keyboard, from a joystick, or from a partly built set of
hardware with some controls on each.  The switching is per control, which is the
point: a panel that arrives one axis at a time is one line at a time here.

XML layout (a ``<control>`` names its ``<device>`` by the ``name`` it was given)::

    <?xml version="1.0" encoding="UTF-8"?>
    <controls version="1.0">
        <device name="keyboard" kind="keyboard" />
        <device name="huey" kind="joystick" match="Arduino" />

        <control name="collective"  source="huey" axis="2" invert="true"
                 deadzone="0.03" minimum="-1" maximum="1" />
        <control name="cyclic-long" source="huey" axis="1" deadzone="0.03" />
        <control name="cyclic-lat"  source="huey" axis="0" deadzone="0.03" />
        <control name="pedals"      source="keyboard" increase="d" decrease="a" />

        <control name="throttle" source="huey" axis="3" />
    </controls>

Two kinds of source, because there are two kinds of device:

* a **keyboard** is a ratchet - it has no position, only keys - so a control on
  it names two key *names* and the two directions they turn it, exactly as
  ``main.py``'s own eight keys do today;
* a **joystick** may be either: ``mode="absolute"`` (the default when an ``axis``
  or a ``hat`` is given) reads a position straight through, which is what a
  cyclic is, and ``mode="ratchet"`` (the default when buttons are given) turns a
  control by a pair of buttons, for a build whose collective is switches rather
  than a lever.

The five control names are :data:`CONTROL_NAMES`: ``cyclic-long`` (cyclic up and
down), ``cyclic-lat`` (cyclic left and right), ``pedals`` (the anti torque
pedals), ``collective`` and ``throttle``.  The first four drive
:class:`simulation.PilotInput`'s four axes - the mapping from a control name to
that keyword is :data:`AXIS_FOR_CONTROL` - and ``throttle`` is mapped here so
that the hardware can be wired up for it now and named in one place: the model
holds 100 per cent rotor speed and has no engine in it (see ``main.py``), so a
throttle position is carried and logged but flies nothing until the model grows
rotor speed dynamics.  A control a file does not name keeps
:func:`default_control`'s own keyboard mapping, so a file may override one
control and leave the rest alone.

Only the standard library is used, so this module has no pygame/OpenGL
dependency and can be tested on its own: ``python controls.py`` reads the
project's own ``default_controls.xml``, lists it, writes it out again and reads
that back, shows what a shorter description leaves to the defaults, prints what
this module says about the descriptions it refuses, and then runs the self test,
which asserts all of it.
"""

import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# --------------------------------------------------------------------------
# The five controls, the two kinds of device and the two modes.
# --------------------------------------------------------------------------

ROOT_TAG = "controls"
DEVICE_TAG = "device"
CONTROL_TAG = "control"
DEFAULT_VERSION = "1.0"

#: The control map this project ships, and the one ``main.py`` reads when it is
#: given no other: the keyboard mapping the sandbox has always been flown from,
#: written down and movable a control at a time.
DEFAULT_CONTROLS_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "default_controls.xml")

#: The control names, in the order a caption or a list reads them: the cyclic
#: up and down, the cyclic left and right, the pedals, the collective lever and
#: the throttle twist grip.  See the module docstring for what each one is.
CYCLIC_LONG = "cyclic-long"
CYCLIC_LAT = "cyclic-lat"
PEDALS = "pedals"
COLLECTIVE = "collective"
THROTTLE = "throttle"
CONTROL_NAMES = (CYCLIC_LONG, CYCLIC_LAT, PEDALS, COLLECTIVE, THROTTLE)

#: The controls that have a stop either side of a centre, and so read -1 to +1,
#: against the two that run 0 at the bottom to 1 at the top.  This is what
#: :meth:`Control.bipolar` reads, and it is why one joystick axis can be a
#: cyclic or a lever without the device map saying which.
BIPOLAR = (CYCLIC_LONG, CYCLIC_LAT, PEDALS)
UNIPOLAR = (COLLECTIVE, THROTTLE)

#: The :class:`simulation.PilotInput` keyword each control drives.  ``throttle``
#: is deliberately absent: it has no axis to drive yet (module docstring).
AXIS_FOR_CONTROL = {
    CYCLIC_LONG: "long_stick",
    CYCLIC_LAT: "lat_stick",
    PEDALS: "pedal",
    COLLECTIVE: "collective",
}

KEYBOARD = "keyboard"
JOYSTICK = "joystick"
DEVICE_KINDS = (KEYBOARD, JOYSTICK)

ABSOLUTE = "absolute"
RATCHET = "ratchet"
MODES = (ABSOLUTE, RATCHET)

HAT_AXES = ("x", "y")

#: Every attribute a ``<device>`` and a ``<control>`` may carry.  Anything else
#: is rejected while loading, so a typo such as ``axis="2" deadzon="0.03"`` is
#: reported rather than silently ignored.
DEVICE_ATTRIBUTES = ("name", "kind", "index", "match")
CONTROL_ATTRIBUTES = ("name", "source", "mode", "axis", "hat", "hat-axis",
                      "invert", "deadzone", "minimum", "maximum",
                      "increase", "decrease",
                      "button-increase", "button-decrease")


class ControlsError(ValueError):
    """Raised when a control map is malformed."""


def _check_attributes(element, allowed):
    """Reject any attribute of *element* that is not in *allowed*."""
    for name in element.keys():
        if name not in allowed:
            raise ControlsError("<%s>: unknown attribute %r, expected one of %s"
                                % (element.tag, name, ", ".join(allowed)))


def _attr_string(element, name, default=None):
    """Read the string attribute *name* of *element*, honouring *default*."""
    raw = element.get(name)
    if raw is None:
        if default is None:
            raise ControlsError("<%s> is missing the mandatory attribute %r"
                                % (element.tag, name))
        return default
    return raw


def _attr_float(element, name, default=None):
    """Read the float attribute *name* of *element*, honouring *default*."""
    raw = element.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        raise ControlsError("<%s>: attribute %s=%r is not a number"
                            % (element.tag, name, raw))


def _attr_int(element, name, default=None):
    """Read the integer attribute *name* of *element*, honouring *default*."""
    raw = element.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError:
        raise ControlsError("<%s>: attribute %s=%r is not a whole number"
                            % (element.tag, name, raw))


def _attr_bool(element, name, default=False):
    """Read the boolean attribute *name* of *element*, honouring *default*.

    ``true``, ``yes`` and ``1`` are true and ``false``, ``no`` and ``0`` are
    false, in any case, so the file can be written the human way.
    """
    raw = element.get(name)
    if raw is None:
        return default
    value = raw.strip().lower()
    if value in ("true", "yes", "1"):
        return True
    if value in ("false", "no", "0"):
        return False
    raise ControlsError("<%s>: attribute %s=%r is not a yes or a no"
                        % (element.tag, name, raw))


def _number(value):
    """Compact, round trippable textual form of a number."""
    return "%.10g" % float(value)


# --------------------------------------------------------------------------
# A device, and a control's mapping onto one.
# --------------------------------------------------------------------------


@dataclass
class Device:
    """One thing a control can be mapped onto: a keyboard, or a joystick.

    A keyboard is one device whatever else is plugged in - there is only ever
    one of them - and a control on it names two keys.  A joystick is found
    either by ``index``, its position in pygame's own enumeration of the
    joysticks, or by ``match``, a piece of its name.  The name is the one to use
    for hardware that comes and goes: an Arduino Leonardo sent as a generic HID
    game controller reports a name beginning ``"Arduino"``, so
    ``match="Arduino"`` finds it wherever it sits in the list, while an index
    would move the day another controller is plugged in.  Both may be given, and
    then the name is what has to match and the index is only where to look.
    """

    name: str
    kind: str = KEYBOARD
    index: Optional[int] = None
    match: Optional[str] = None

    def check(self):
        """Raise :class:`ControlsError` unless this device adds up."""
        if self.kind not in DEVICE_KINDS:
            raise ControlsError("device %r: unknown kind %r, expected one of %s"
                                % (self.name, self.kind, ", ".join(DEVICE_KINDS)))
        if self.kind == KEYBOARD:
            if self.index is not None or self.match is not None:
                raise ControlsError("device %r: a keyboard is the one device "
                                    "there is, so it takes no index or match"
                                    % (self.name,))
        elif self.index is None and self.match is None:
            raise ControlsError("device %r: a joystick needs an index or a match "
                                "to be found by" % (self.name,))
        if self.index is not None and self.index < 0:
            raise ControlsError("device %r: index %d is not a position"
                                % (self.name, self.index))
        return self

    def describe(self):
        """How this device is found, in words, for a listing or a caption."""
        if self.kind == KEYBOARD:
            return "keyboard"
        if self.match is not None:
            return "joystick %r" % (self.match,)
        return "joystick #%d" % (self.index,)

    def to_element(self):
        """This device as an ElementTree element."""
        element = ET.Element(DEVICE_TAG)
        element.set("name", self.name)
        element.set("kind", self.kind)
        if self.index is not None:
            element.set("index", str(self.index))
        if self.match is not None:
            element.set("match", self.match)
        return element

    @classmethod
    def from_element(cls, element):
        """The device an ElementTree element describes."""
        _check_attributes(element, DEVICE_ATTRIBUTES)
        return cls(name=_attr_string(element, "name"),
                   kind=_attr_string(element, "kind", KEYBOARD),
                   index=_attr_int(element, "index"),
                   match=element.get("match")).check()


@dataclass
class Control:
    """One control, on one device, in one mode.

    An **absolute** control is read as a position: a joystick ``axis`` (-1 to
    +1, or 0 to 1 for the unipolar two), or a ``hat`` direction.  ``invert``
    flips the sense, ``deadzone`` is how far off its rest the device has to read
    before it is believed - a real stick jitters about its centre, and a home
    made lever may rest above its bottom stop - and ``minimum`` and ``maximum``
    calibrate a raw axis that does not reach the ends of its range, which is
    what a lever built from a potentiometer usually needs: the reading is
    stretched from that pair onto the control's own full travel.

    A **ratchet** control is turned rather than placed: a keyboard's two key
    ``increase`` and ``decrease`` names, or a joystick's two button numbers,
    which is the shape :meth:`simulation.PilotInput.step` already takes.
    """

    name: str
    source: str
    mode: str = RATCHET
    axis: Optional[int] = None
    hat: Optional[int] = None
    hat_axis: str = "x"
    invert: bool = False
    deadzone: float = 0.0
    minimum: Optional[float] = None
    maximum: Optional[float] = None
    increase: Optional[str] = None
    decrease: Optional[str] = None
    button_increase: Optional[int] = None
    button_decrease: Optional[int] = None

    @property
    def bipolar(self):
        """Does this control read -1 to +1 rather than 0 to 1?"""
        return self.name in BIPOLAR

    @property
    def absolute(self):
        """Is this control a position rather than a ratchet?"""
        return self.mode == ABSOLUTE

    def check(self, devices):
        """Raise :class:`ControlsError` unless this control adds up.

        *devices* is the mapping of name to :class:`Device` this control has to
        sit on, so that a control naming a device the file does not have is
        caught here rather than at the first frame.
        """
        if self.name not in CONTROL_NAMES:
            raise ControlsError("control %r: unknown name, expected one of %s"
                                % (self.name, ", ".join(CONTROL_NAMES)))
        if self.source not in devices:
            raise ControlsError("control %r: no device named %r in this file"
                                % (self.name, self.source))
        if self.mode not in MODES:
            raise ControlsError("control %r: unknown mode %r, expected one of %s"
                                % (self.name, self.mode, ", ".join(MODES)))
        if self.hat_axis not in HAT_AXES:
            raise ControlsError("control %r: hat-axis %r is not one of %s"
                                % (self.name, self.hat_axis, ", ".join(HAT_AXES)))
        if self.deadzone < 0.0:
            raise ControlsError("control %r: deadzone %g is not a distance"
                                % (self.name, self.deadzone))
        if self.absolute:
            if devices[self.source].kind == KEYBOARD:
                raise ControlsError("control %r: a keyboard is a ratchet - there "
                                    "is no position to read - so it cannot be "
                                    "absolute" % (self.name,))
            self._check_absolute()
        else:
            self._check_ratchet(devices[self.source])
        return self

    def _check_absolute(self):
        """The absolute half of :meth:`check`."""
        if (self.axis is None) == (self.hat is None):
            raise ControlsError("control %r: an absolute control needs exactly "
                                "one of axis and hat" % (self.name,))
        if (self.increase is not None or self.decrease is not None
                or self.button_increase is not None
                or self.button_decrease is not None):
            raise ControlsError("control %r: an absolute control is a position, "
                                "so it takes no keys or buttons" % (self.name,))
        if self.hat is not None and (self.minimum is not None
                                     or self.maximum is not None):
            raise ControlsError("control %r: a hat reads -1, 0 or 1 already, so "
                                "it has nothing to calibrate" % (self.name,))
        if (self.minimum is None) != (self.maximum is None):
            raise ControlsError("control %r: minimum and maximum go together"
                                % (self.name,))
        if self.minimum is not None and self.maximum <= self.minimum:
            raise ControlsError("control %r: maximum %g is not above minimum %g"
                                % (self.name, self.maximum, self.minimum))

    def _check_ratchet(self, device):
        """The ratchet half of :meth:`check`, which knows its device's kind."""
        if self.axis is not None or self.hat is not None:
            raise ControlsError("control %r: a ratchet turns, so it takes an "
                                "axis or a hat only in absolute mode"
                                % (self.name,))
        if self.invert or self.deadzone or self.minimum is not None:
            raise ControlsError("control %r: invert, deadzone and calibration "
                                "belong to a position, not a ratchet"
                                % (self.name,))
        if device.kind == KEYBOARD:
            if self.increase is None or self.decrease is None:
                raise ControlsError("control %r: a keyboard control needs both "
                                    "increase and decrease key names"
                                    % (self.name,))
            if (self.button_increase is not None
                    or self.button_decrease is not None):
                raise ControlsError("control %r: %r is a keyboard, so it takes "
                                    "key names and not buttons"
                                    % (self.name, device.name))
        else:
            if self.button_increase is None or self.button_decrease is None:
                raise ControlsError("control %r: a joystick ratchet needs both "
                                    "button-increase and button-decrease"
                                    % (self.name,))
            if self.increase is not None or self.decrease is not None:
                raise ControlsError("control %r: %r is a joystick, so it takes "
                                    "buttons and not key names"
                                    % (self.name, device.name))

    def to_element(self):
        """This control as an ElementTree element, in its own mode's shape."""
        element = ET.Element(CONTROL_TAG)
        element.set("name", self.name)
        element.set("source", self.source)
        element.set("mode", self.mode)
        if self.absolute:
            if self.axis is not None:
                element.set("axis", str(self.axis))
            if self.hat is not None:
                element.set("hat", str(self.hat))
                element.set("hat-axis", self.hat_axis)
            if self.minimum is not None:
                element.set("minimum", _number(self.minimum))
                element.set("maximum", _number(self.maximum))
            if self.invert:
                element.set("invert", "true")
            if self.deadzone:
                element.set("deadzone", _number(self.deadzone))
        elif self.increase is not None:
            element.set("increase", self.increase)
            element.set("decrease", self.decrease)
        else:
            element.set("button-increase", str(self.button_increase))
            element.set("button-decrease", str(self.button_decrease))
        return element

    @classmethod
    def from_element(cls, element):
        """The control an ElementTree element describes, before :meth:`check`.

        The mode is worked out from the attributes when the element does not
        name one, which is what lets the two common cases - a stick's axis and a
        keyboard's two keys - say only what matters.
        """
        _check_attributes(element, CONTROL_ATTRIBUTES)
        axis = _attr_int(element, "axis")
        hat = _attr_int(element, "hat")
        mode = element.get("mode")
        if mode is None:
            mode = ABSOLUTE if (axis is not None or hat is not None) else RATCHET
        return cls(
            name=_attr_string(element, "name"),
            source=_attr_string(element, "source"),
            mode=mode,
            axis=axis,
            hat=hat,
            hat_axis=_attr_string(element, "hat-axis", "x"),
            invert=_attr_bool(element, "invert"),
            deadzone=_attr_float(element, "deadzone", 0.0),
            minimum=_attr_float(element, "minimum"),
            maximum=_attr_float(element, "maximum"),
            increase=element.get("increase"),
            decrease=element.get("decrease"),
            button_increase=_attr_int(element, "button-increase"),
            button_decrease=_attr_int(element, "button-decrease"))


# --------------------------------------------------------------------------
# A device's reading as the aircraft's own axis: the arithmetic, and nothing
# that needs a device to run it, so a test can drive it with numbers.
# --------------------------------------------------------------------------


def axis_value(control, raw):
    """One absolute axis reading as the control's own normalised position.

    *raw* is what a joystick reports, -1 to +1.  ``minimum`` and ``maximum``,
    when the control names them, are the raw readings that *are* its two stops,
    so a pot that only swings over part of its range still uses all of the
    control's travel; the reading is then clipped to its stops, its deadzone is
    taken out - a band about the centre of a bipolar control or above the bottom
    stop of a unipolar one - and ``invert`` turns the sense over.

    The last step is what the control's own name decides: a cyclic or the pedals
    come out -1 to +1, and the collective and the throttle 0 at the bottom to 1
    at the top, which is the shape :meth:`simulation.PilotInput.set_axes` takes.
    """
    low, high = control.minimum, control.maximum
    value = float(raw)
    if low is not None and high is not None and high > low:
        value = (value - low) / (high - low) * 2.0 - 1.0
    value = max(-1.0, min(1.0, value))
    if control.invert:
        value = -value
    if control.bipolar:
        if -control.deadzone < value < control.deadzone:
            value = 0.0
        return value
    value = (value + 1.0) * 0.5
    return 0.0 if value < control.deadzone else value


def hat_value(control, reading):
    """A hat direction as the control's own position.

    A hat reads -1, 0 or +1 with nothing in between, so there is no deadzone to
    take out and nothing to calibrate: all this does is pick the axis the
    control named and pass ``invert`` on, through the same -1 to +1 the axis
    path uses so that both are described by one :func:`axis_value` rule.
    """
    value = float(-1 if reading < 0 else (1 if reading > 0 else 0))
    if control.invert:
        value = -value
    return value


def command(increase, decrease):
    """A ratchet's own -1, 0 or +1 from the two directions it can be turned.

    The directions cancel, which is what a pilot's other hand does on the
    collective and what ``main.py``'s own key pair does today, and nothing held
    is 0 - so a ratchet with neither direction held leaves its axis exactly
    where it is.
    """
    return (1.0 if increase else 0.0) - (1.0 if decrease else 0.0)


def key_command(control, held):
    """The ratchet command of a keyboard control, from a ``held(name)`` test.

    *held* is called with each of the control's two key names and answers
    whether it is down, which is the shape a dict of scancodes is read in.
    """
    return command(held(control.increase), held(control.decrease))


def button_command(control, held):
    """The ratchet command of a joystick control, from a ``held(index)`` test.

    *held* is called with each of the control's two button numbers.
    """
    return command(held(control.button_increase), held(control.button_decrease))


# --------------------------------------------------------------------------
# The whole file.
# --------------------------------------------------------------------------

#: The name of the keyboard device, which is always there whether or not a file
#: spells it out: a control left out of a file keeps the keyboard mapping of
#: :func:`default_control`, and that control needs a device to sit on.
KEYBOARD_DEVICE = "keyboard"


@dataclass
class ControlMap:
    """Every device a file names, and where each control sits among them.

    A map is not required to name every control: one it leaves out is flown by
    :func:`default_control`, the keyboard mapping this project has always had,
    so a file that arrives one panel at a time only has to say what has changed.
    That is also why the keyboard device is added whether a file mentions it or
    not - a fallback control has to sit somewhere.

    :meth:`check` is the only validator and the only way in from XML, so a map
    that exists is a map that adds up: every device named once, every control on
    a device the file has, and every mode's own attributes present and no
    others.
    """

    version: str = DEFAULT_VERSION
    devices: Dict[str, Device] = field(default_factory=dict)
    controls: Dict[str, Control] = field(default_factory=dict)

    def add_device(self, device):
        """Append *device*, refusing a name already taken, and return it."""
        if device.name in self.devices:
            raise ControlsError("two devices named %r" % (device.name,))
        self.devices[device.name] = device
        return device

    def add_control(self, control):
        """Append *control*, refusing a name already taken, and return it."""
        if control.name in self.controls:
            raise ControlsError("two controls named %r" % (control.name,))
        self.controls[control.name] = control
        return control

    def check(self):
        """Validate the whole map, keyboard device and all.  Returns self."""
        self.devices.setdefault(KEYBOARD_DEVICE, Device(name=KEYBOARD_DEVICE))
        for device in self.devices.values():
            device.check()
        for control in self.controls.values():
            control.check(self.devices)
        return self

    def device(self, name):
        """The device called *name*, or a :class:`ControlsError`."""
        if name not in self.devices:
            raise ControlsError("no device named %r" % (name,))
        return self.devices[name]

    def control(self, name):
        """The control called *name*: the file's own, or the keyboard default.

        A control the file does not name is not an error and not a gap: it is
        the project's own keyboard mapping for that control, so a map is always
        complete.  ``None`` means neither - the throttle, before the model flies
        it (module docstring).
        """
        if name in self.controls:
            return self.controls[name]
        return default_control(name)

    def device_for(self, control):
        """The :class:`Device` *control* sits on."""
        return self.device(control.source)

    def mapped_names(self):
        """The control names the file itself maps, in :data:`CONTROL_NAMES` order."""
        return [name for name in CONTROL_NAMES if name in self.controls]

    def key_names(self):
        """Every keyboard key name this map reads, sorted and without repeats.

        What a caller resolves to scancodes once, rather than per frame.
        """
        names = set()
        for name in CONTROL_NAMES:
            control = self.control(name)
            if control is None:
                continue
            device = self.device(control.source)
            if device.kind == KEYBOARD:
                names.add(control.increase)
                names.add(control.decrease)
        return sorted(names)

    def describe(self):
        """One line per control: its device, its mode and what it is read from."""
        lines = ["devices:"]
        for name in sorted(self.devices):
            lines.append("  %-10s %s" % (name, self.devices[name].describe()))
        lines.append("controls:")
        for name in CONTROL_NAMES:
            control = self.control(name)
            if control is None:
                lines.append("  %-11s not mapped" % (name,))
            elif control.absolute:
                if control.axis is not None:
                    reading = "axis %d" % (control.axis,)
                else:
                    reading = "hat %d %s" % (control.hat, control.hat_axis)
                lines.append("  %-11s %-10s %-8s %s"
                             % (name, control.source, control.mode, reading))
            elif control.increase is not None:
                lines.append("  %-11s %-10s %-8s keys %s/%s"
                             % (name, control.source, control.mode,
                                control.increase, control.decrease))
            else:
                lines.append("  %-11s %-10s %-8s buttons %d/%d"
                             % (name, control.source, control.mode,
                                control.button_increase, control.button_decrease))
        return "\n".join(lines)

    def to_element(self):
        """Serialise the whole map to an ElementTree element.

        Devices come out sorted by name and controls in :data:`CONTROL_NAMES`
        order, so the file a run writes is the file it read whatever order they
        were read in.
        """
        root = ET.Element(ROOT_TAG)
        root.set("version", self.version)
        for name in sorted(self.devices):
            root.append(self.devices[name].to_element())
        for name in self.mapped_names():
            root.append(self.controls[name].to_element())
        return root

    def to_xml_string(self):
        """Serialise the map to an indented XML string."""
        return ET.tostring(_indent(self.to_element()), encoding="unicode")

    def to_xml_bytes(self):
        """Serialise the map to UTF-8 XML including the declaration."""
        declaration = '<?xml version="1.0" encoding="UTF-8"?>\n'
        return (declaration + self.to_xml_string() + "\n").encode("utf-8")

    def save(self, path):
        """Write the control map to *path* as an XML file."""
        with open(path, "wb") as handle:
            handle.write(self.to_xml_bytes())

    @classmethod
    def from_element(cls, root):
        """Build a map from the root ElementTree element."""
        if root.tag != ROOT_TAG:
            raise ControlsError("expected root element <%s>, found <%s>"
                                % (ROOT_TAG, root.tag))
        control_map = cls(version=root.get("version", DEFAULT_VERSION))
        for element in root:
            if element.tag == DEVICE_TAG:
                control_map.add_device(Device.from_element(element))
            elif element.tag == CONTROL_TAG:
                control_map.add_control(Control.from_element(element))
            else:
                raise ControlsError("expected <%s> or <%s>, found <%s>"
                                    % (DEVICE_TAG, CONTROL_TAG, element.tag))
        return control_map.check()

    @classmethod
    def from_xml_string(cls, text):
        """Build a map from an XML string."""
        try:
            root = ET.fromstring(text)
        except ET.ParseError as error:
            raise ControlsError("not well formed XML: %s" % (error,))
        return cls.from_element(root)

    @classmethod
    def load(cls, path):
        """Read a control map from the XML file *path*.

        Raises OSError when the file cannot be read and ControlsError when its
        contents are malformed.  The caller is expected to mention *path*
        itself, so these messages deliberately do not repeat it.
        """
        try:
            tree = ET.parse(path)
        except ET.ParseError as error:
            raise ControlsError("not well formed XML: %s" % (error,))
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


# --------------------------------------------------------------------------
# The project's own mapping: the keyboard it has always been flown from.
# --------------------------------------------------------------------------

#: The pair of keys each control is flown from unless a file says otherwise,
#: written as ``(increase, decrease)`` and named the way pygame names the
#: physical key positions - so a keyboard whose layout moves the letters does
#: not move the controls.  The throttle is absent: the model does not fly one
#: yet, and inventing keys for it would name a control that does nothing.
DEFAULT_KEYS = {
    COLLECTIVE: ("w", "s"),
    CYCLIC_LONG: ("up", "down"),
    CYCLIC_LAT: ("right", "left"),
    PEDALS: ("d", "a"),
}


def default_control(name):
    """The keyboard mapping of *name*, or ``None`` if the project has none.

    This is what a file that does not mention a control leaves in its place, and
    it is the mapping ``main.py`` read before there was a file at all: W and S
    for the collective, the up and down arrows for the longitudinal cyclic, left
    and right for the lateral one, and A and D for the pedals.  The throttle has
    no default because the model has no throttle, so ``None`` says that rather
    than inventing a pair of keys that fly nothing.
    """
    if name not in DEFAULT_KEYS:
        return None
    increase, decrease = DEFAULT_KEYS[name]
    return Control(name=name, source=KEYBOARD_DEVICE, mode=RATCHET,
                   increase=increase, decrease=decrease)


def default_control_map():
    """The project's own map: the four controls of the keyboard, and nothing else.

    What ``main.py`` flies with when it is told to read no file - the same four
    axes, the same eight keys and the same sense the sandbox has always had.  The
    throttle is not in it, because no key flies one.
    """
    control_map = ControlMap()
    for name in CONTROL_NAMES:
        control = default_control(name)
        if control is not None:
            control_map.add_control(control)
    return control_map.check()


# ---------------------------------------------------------------------------
# The self test and the demo, in the style of the modules around this one.
# ---------------------------------------------------------------------------

#: Maps written to be refused, each with the reason it is wrong.  The demo
#: prints what this module says about them and the self test asserts that every
#: one is turned away, so an attribute this module stops looking at fails here
#: rather than quietly accepting a file that names it.
REFUSALS = (
    ("<scenery version='1.0'/>",
     "not a control map"),
    ("<controls version='1.0'/>"
     "<control name='collective' source='keyboard' increase='w' decrease='s'/>",
     "not well formed"),
    ("<controls version='1.0'>"
     "<device name='k' kind='gamepad'/></controls>",
     "unknown device kind"),
    ("<controls version='1.0'>"
     "<device name='k' kind='keyboard' index='0'/></controls>",
     "a keyboard with an index"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick'/></controls>",
     "a joystick with no index or match"),
    ("<controls version='1.0'>"
     "<device name='keyboard' kind='keyboard'/>"
     "<device name='keyboard' kind='keyboard'/></controls>",
     "the same device name twice"),
    ("<controls version='1.0'>"
     "<control name='collective' source='keyboard' increase='w' decrease='s'/>"
     "<control name='collective' source='keyboard' increase='w' decrease='s'/>"
     "</controls>",
     "the same control name twice"),
    ("<controls version='1.0'>"
     "<control name='flaps' source='keyboard' increase='w' decrease='s'/>"
     "</controls>",
     "an unknown control name"),
    ("<controls version='1.0'>"
     "<control name='collective' source='huey' increase='w' decrease='s'/>"
     "</controls>",
     "a device the file has not got"),
    ("<controls version='1.0'>"
     "<control name='collective' source='keyboard' increase='w'/></controls>",
     "one key of a pair"),
    ("<controls version='1.0'>"
     "<control name='collective' source='keyboard' increase='w' decrease='s' "
     "deadzone='0.1'/></controls>",
     "a deadzone on a ratchet"),
    ("<controls version='1.0'>"
     "<control name='collective' source='keyboard' axis='2'/></controls>",
     "a keyboard read as a position"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' mode='absolute'/></controls>",
     "an absolute control with neither axis nor hat"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' axis='1' hat='0'/></controls>",
     "an axis and a hat together"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' mode='ratchet' axis='1' "
     "button-increase='0' button-decrease='1'/></controls>",
     "a ratchet with an axis"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' mode='ratchet' button-increase='0'/>"
     "</controls>",
     "one button of a pair"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' axis='1' minimum='0'/></controls>",
     "a minimum with no maximum"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' axis='1' minimum='1' "
     "maximum='0'/></controls>",
     "a maximum below its minimum"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' axis='1' deadzon='0.1'/></controls>",
     "an unknown attribute"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' axis='1' invert='perhaps'/>"
     "</controls>",
     "a yes or a no that is neither"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' hat='0' minimum='0' maximum='1'/>"
     "</controls>",
     "a hat with something to calibrate"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' axis='1' hat-axis='z'/></controls>",
     "a hat axis that is neither x nor y"),
    ("<controls version='1.0'>"
     "<device name='j' kind='joystick' index='0'/>"
     "<control name='collective' source='j' mode='toggle' axis='1'/></controls>",
     "an unknown mode"),
)


def _refuse(text, why):
    """Report what this module says about a map written to be refused."""
    try:
        ControlMap.from_xml_string(text)
    except ControlsError as error:
        print("  refused (%-26s): %s" % (why, error))
        return
    raise AssertionError("accepted a map that should be refused: %s" % (why,))


def _self_test():
    """Checks on the format: the sample round trips, the bad ones are refused.

    Raises AssertionError on failure.  The demo calls this, so running this
    module is enough to validate it.  Two directions are checked, as in
    ``scenery.py``: that a map survives the trip out to XML and back, and that a
    map written to be malformed is refused rather than accepted anyway.
    """
    # The default map and the file this project ships are the same map, so a run
    # that reads no file and a run that reads the one it ships fly identically -
    # which is the whole of what this feature must not change.
    control_map = default_control_map()
    assert control_map.mapped_names() == [CYCLIC_LONG, CYCLIC_LAT, PEDALS,
                                          COLLECTIVE]
    assert control_map.key_names() == ["a", "d", "down", "left", "right", "s",
                                       "up", "w"]
    assert control_map.control(THROTTLE) is None
    assert default_control(THROTTLE) is None
    assert control_map.control(PEDALS).bipolar
    assert not control_map.control(COLLECTIVE).bipolar
    assert control_map.control(COLLECTIVE).increase == "w"
    assert control_map.control(COLLECTIVE).decrease == "s"
    assert control_map.control(CYCLIC_LONG).increase == "up"
    assert control_map.control(CYCLIC_LAT).increase == "right"
    assert control_map.control(PEDALS).increase == "d"

    # The file is the format's documentation as well as its example, so every
    # check below is made against it rather than against a fixture built here.
    sample = ControlMap.load(DEFAULT_CONTROLS_FILE)
    assert sample.version == DEFAULT_VERSION
    assert sample.to_xml_string() == control_map.to_xml_string()
    text = sample.to_xml_string()
    assert ControlMap.from_xml_string(text).to_xml_string() == text

    # And out to a file and back, which is the path a caller actually uses.
    handle = tempfile.NamedTemporaryFile(suffix=".xml", delete=False)
    handle.close()
    try:
        sample.save(handle.name)
        assert ControlMap.load(handle.name).to_xml_bytes() == sample.to_xml_bytes()
    finally:
        os.remove(handle.name)

    # A map that names one control leaves the other four where they were: the
    # keyboard default, which is what makes the mapping switchable one axis at a
    # time.  The control it does name is the joystick's, and the keys of that one
    # are the only keys it stops reading.
    partial = ControlMap.from_xml_string(
        '<controls version="1.0">'
        '<device name="huey" kind="joystick" match="Arduino"/>'
        '<control name="cyclic-lat" source="huey" axis="0" deadzone="0.05"/>'
        '</controls>')
    assert partial.mapped_names() == [CYCLIC_LAT]
    assert partial.control(CYCLIC_LAT).absolute
    assert partial.control(CYCLIC_LAT).axis == 0
    assert partial.control(CYCLIC_LAT).deadzone == 0.05
    assert partial.control(CYCLIC_LONG).increase == "up"        # the fallback
    assert partial.key_names() == ["a", "d", "down", "s", "up", "w"]

    # A joystick axis as the aircraft's own axis.  A bipolar control reads -1 to
    # +1, is clipped to its stops, loses its deadzone about the centre and turns
    # over when it is inverted; a calibrated one is stretched between the two
    # readings that are its stops first.
    stick = Control(name=CYCLIC_LONG, source="huey")
    assert stick.bipolar
    assert axis_value(stick, 0.0) == 0.0
    assert axis_value(stick, 1.0) == 1.0
    assert axis_value(stick, -1.0) == -1.0
    assert axis_value(stick, 4.0) == 1.0                # clipped to the stop
    inverted = Control(name=CYCLIC_LONG, source="huey", invert=True)
    assert axis_value(inverted, 1.0) == -1.0
    dead = Control(name=CYCLIC_LONG, source="huey", deadzone=0.1)
    assert axis_value(dead, 0.05) == 0.0
    assert axis_value(dead, -0.05) == 0.0
    assert abs(axis_value(dead, 0.5) - 0.5) < 1e-12
    pot = Control(name=CYCLIC_LONG, source="huey", minimum=0.2, maximum=0.8)
    assert abs(axis_value(pot, 0.2) + 1.0) < 1e-12
    assert abs(axis_value(pot, 0.5)) < 1e-12
    assert abs(axis_value(pot, 0.8) - 1.0) < 1e-12
    assert abs(axis_value(pot, 0.0) + 1.0) < 1e-12      # below the low stop

    # The unipolar two - the collective and the throttle - read 0 at the bottom
    # stop to 1 at the top, which is the shape PilotInput.set_axes takes, and
    # their deadzone is a band above the bottom stop rather than about a centre.
    lever = Control(name=COLLECTIVE, source="huey")
    assert not lever.bipolar
    assert axis_value(lever, -1.0) == 0.0
    assert axis_value(lever, 1.0) == 1.0
    assert abs(axis_value(lever, 0.0) - 0.5) < 1e-12
    resting = Control(name=COLLECTIVE, source="huey", deadzone=0.2)
    assert axis_value(resting, -1.0) == 0.0
    assert axis_value(resting, -0.8) == 0.0             # 0.1 is inside the band
    assert abs(axis_value(resting, 0.0) - 0.5) < 1e-12

    # A hat is three positions and nothing in between, and reaches the same -1
    # to +1 rule the axis path uses.
    trim = Control(name=CYCLIC_LONG, source="huey", mode=ABSOLUTE, hat=0)
    assert hat_value(trim, -1) == -1.0
    assert hat_value(trim, 0) == 0.0
    assert hat_value(trim, 1) == 1.0
    flipped = Control(name=CYCLIC_LONG, source="huey", mode=ABSOLUTE, hat=0,
                      invert=True)
    assert hat_value(flipped, 1) == -1.0

    # A ratchet's own command, from keys and from buttons: the directions
    # cancel, nothing held is nothing, and that zero is what leaves an axis
    # exactly where the last frame put it.
    assert command(True, False) == 1.0
    assert command(False, True) == -1.0
    assert command(True, True) == 0.0
    assert command(False, False) == 0.0
    keys = control_map.control(COLLECTIVE)
    assert key_command(keys, lambda name: name == "w") == 1.0
    assert key_command(keys, lambda name: name == "s") == -1.0
    assert key_command(keys, lambda name: name in ("w", "s")) == 0.0
    assert key_command(keys, lambda name: False) == 0.0
    buttons = Control(name=COLLECTIVE, source="huey", mode=RATCHET,
                      button_increase=0, button_decrease=1)
    assert button_command(buttons, lambda index: index == 0) == 1.0
    assert button_command(buttons, lambda index: index == 1) == -1.0
    assert button_command(buttons, lambda index: True) == 0.0

    # Every map written to be refused is refused.
    for bad_text, why in REFUSALS:
        try:
            ControlMap.from_xml_string(bad_text)
        except ControlsError:
            continue
        raise AssertionError("accepted a map that should be refused: %s"
                             % (why,))


def _demo():
    """Print what this module does: a map, in and out of XML, and its refusals.

    The map on show is the project's own ``default_controls.xml`` - read,
    listed, written out again and read back - and then a map that names one
    control and leaves the rest to the keyboard, which is how the hardware this
    project is heading for will arrive: one axis at a time.
    """
    print("controls.py - the pilot's devices, and which control flies which")
    print()
    print("the project's own map, %s:"
          % (os.path.basename(DEFAULT_CONTROLS_FILE),))
    control_map = ControlMap.load(DEFAULT_CONTROLS_FILE)
    print(control_map.describe())
    print()
    print("  the keys it reads: %s" % (", ".join(control_map.key_names()),))
    print("  the throttle: %s"
          % ("mapped" if control_map.control(THROTTLE)
             else "not mapped - the model flies no throttle yet",))
    print()
    print("written out again:")
    for line in control_map.to_xml_bytes().decode("utf-8").splitlines():
        print("  " + line)
    print()
    print("a map that names one control and leaves the rest to the keyboard:")
    partial = ControlMap.from_xml_string(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<controls version="1.0">\n'
        '  <device name="huey" kind="joystick" match="Arduino" />\n'
        '  <control name="cyclic-lat" source="huey" axis="0" deadzone="0.05" />\n'
        '</controls>\n')
    print(partial.describe())
    print()
    print("a stick's raw reading as the aircraft's own axis - with a 0.05")
    print("deadzone, calibrated to a pot that only swings 0.2 to 0.8, and the")
    print("collective, which reads 0 at the bottom stop to 1 at the top:")
    dead = Control(name=CYCLIC_LONG, source="huey", deadzone=0.05)
    pot = Control(name=CYCLIC_LONG, source="huey", minimum=0.2, maximum=0.8)
    lever = Control(name=COLLECTIVE, source="huey")
    for raw in (-1.0, -0.05, 0.0, 0.5, 1.0):
        print("  raw %+5.2f -> %+5.2f deadzoned, %+5.2f calibrated, %5.2f lever"
              % (raw, axis_value(dead, raw), axis_value(pot, raw),
                 axis_value(lever, raw)))
    print()
    print("maps this module refuses, and what it says about each:")
    for bad_text, why in REFUSALS:
        _refuse(bad_text, why)
    print()
    _self_test()
    print("controls.py self test passed")


if __name__ == "__main__":
    _demo()
