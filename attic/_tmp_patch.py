# -*- coding: utf-8 -*-
"""Scratch: the manual's own beat, patched onto the mission's module.

The mission as it stands flies *one* two hundred foot manoeuvre: the lever is
pulled from two hundred feet of skid and the nose stays up all the way in.  The
manual flies two, at two heights: the cyclic flare at fifty to sixty feet, the
level off at ten to fifteen, and the collective spent under *that*.

This patches ``_rt_pilot`` to the manual's beat so ``_rt_autoro_probe`` and
``_tmp_sweep`` rehearse it headless, without an edit of the mission: the flare
begins at :data:`FLARE_PITCH_FROM_FT`, peaks at :data:`LEVEL_OFF_SKID_FT` and
comes back to the trim under it, and the cushion opens where the manual puts it
rather than two hundred feet up.  ``_rt_pilot.py`` is pure CRLF, so the
replacements are made on the text as it is and written back the same way.

Run from the project root, because ``_rt_pilot.py`` is opened by that name and
the working directory is what the patch edits::

    python attic/_tmp_patch.py            # the mission to the manual's beat
    python attic/_tmp_patch.py --revert   # back to the two hundred foot schedule
"""

import io
import sys

PATH = "_rt_pilot.py"

#: The height the nose comes back down at, in feet of skid: the manual's own
#: level off.  It is also the cushion's own top as this patch cuts it - the
#: attitude starts back to the trim exactly where the manual starts the
#: collective, which is the whole point of the beat.
LEVEL_OFF_BLOCK = "\r\n".join([
    "#: The height the nose comes back *down* at: the manual's own level off, ten",
    "#: to fifteen feet, which is where the flare peaks and the attitude starts",
    "#: back towards the trim.  Above it the flare is monotone, as the long",
    "#: schedule was; at it the disc starts back level and the lever pays for the",
    "#: last of the height under a disc that is not tilted - which is the one",
    "#: place a nose up taken back down is not the balloon it is at height,",
    "#: because the height it would climb into is the twelve feet it has left.",
    "LEVEL_OFF_SKID_FT = 12.0",
])

#: The schedule itself: a rise to the level off and a come down under it, which
#: is :data:`LEVEL_OFF_SKID_FT`'s own fraction, min'd with the flare's rise.
#: Both branches of the attitude half use the one line.
SCHEDULE_OLD = "                    * (1.0 - skid_ft(telemetry) / flare_from),"
SCHEDULE_NEW = "\r\n".join([
    "                    * clip(",
    "                        min((flare_from - skid_ft(telemetry))",
    "                            / (flare_from - LEVEL_OFF_SKID_FT),",
    "                            skid_ft(telemetry) / LEVEL_OFF_SKID_FT),",
    "                        0.0, 1.0),",
])

CUSHION_COMMENT_OLD = "\r\n".join([
    "                # The attitude half of the flare: the same schedule as the sink,",
    "                # nose up by FLARE_PITCH_DEG on the skids and by none of it at",
    "                # the flare's height, so the speed is bleeding while the lever is",
    "                # still on the descent's own - see the constants above, and for",
    "                # why it is monotone rather than a band that peaks and comes off",
    "                # again.  The cyclic below flies it against the pull's own nose up",
    "                # moment.",
])
CUSHION_COMMENT_NEW = "\r\n".join([
    "                # The attitude half of the flare: the manual's own beat, nose up",
    "                # from the flare's height to FLARE_PITCH_DEG at the level off and",
    "                # back to the trim under that, so the speed is bled at fifty feet",
    "                # and the disc is level again for the last twelve - see the",
    "                # constants above.  The cyclic below flies it against the pull's",
    "                # own nose up moment.",
])

FLARE_COMMENT_OLD = "\r\n".join([
    "                # come up.  Monotone, and it does not relax: see the constants",
    "                # above for what a nose up taken back down does to an aircraft",
    "                # with two hundred feet under it.",
])
FLARE_COMMENT_NEW = "\r\n".join([
    "                # come up.  It rises to the level off's own height and holds",
    "                # nothing: see the constants above for where the nose comes back",
    "                # down, and why that is safe there and nowhere else.",
])


def renamed(text, old, new, count=1):
    """*text* with *old* replaced by *new*, or a loud failure to say why not."""
    found = text.count(old)
    if found != count:
        raise SystemExit("%d of %r, wanted %d: the mission has moved since"
                         % (found, old[:48], count))
    return text.replace(old, new)


def to_the_manual(text):
    """The mission's module, re-cut to the manual's two heights."""
    text = renamed(text, "FLARE_SKID_FT = 200.0", "FLARE_SKID_FT = 12.0")
    text = renamed(text, "FLARE_PITCH_FROM_FT = None",
                   "FLARE_PITCH_FROM_FT = 55.0\r\n" + LEVEL_OFF_BLOCK)
    text = renamed(text, "CUSHION_PULL_IN = 6.5", "CUSHION_PULL_IN = 2.0")
    text = renamed(text, SCHEDULE_OLD, SCHEDULE_NEW, count=2)
    text = renamed(text, CUSHION_COMMENT_OLD, CUSHION_COMMENT_NEW)
    text = renamed(text, FLARE_COMMENT_OLD, FLARE_COMMENT_NEW)
    return text


def as_it_stands(text):
    """And back: the two hundred foot schedule the mission shipped with."""
    text = renamed(text, "FLARE_SKID_FT = 12.0", "FLARE_SKID_FT = 200.0")
    text = renamed(text, "FLARE_PITCH_FROM_FT = 55.0\r\n" + LEVEL_OFF_BLOCK,
                   "FLARE_PITCH_FROM_FT = None")
    text = renamed(text, "CUSHION_PULL_IN = 2.0", "CUSHION_PULL_IN = 6.5")
    text = renamed(text, SCHEDULE_NEW, SCHEDULE_OLD, count=2)
    text = renamed(text, CUSHION_COMMENT_NEW, CUSHION_COMMENT_OLD)
    text = renamed(text, FLARE_COMMENT_NEW, FLARE_COMMENT_OLD)
    return text


def main():
    """The command line: re-cut the mission, or put it back."""
    back = "--revert" in sys.argv[1:]
    with io.open(PATH, "r", newline="") as handle:
        text = handle.read()
    edited = as_it_stands(text) if back else to_the_manual(text)
    with io.open(PATH, "w", newline="") as handle:
        handle.write(edited)
    print("%s: %s"
          % (PATH, "the mission as it shipped" if back
             else "the manual's two heights"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
