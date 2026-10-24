# Attic

The scratch that used to fill the project root, moved here on 2026-09-28.  The
root held 114 files, of which nine were the project; getting that down to the
nine meant putting this somewhere rather than deleting it, because the numbers
in the physics can be traced back through it.

Nothing here is imported by anything, and `attic/` is in `.gitignore` with the
rest of its bulk - the text dumps, the page crops, the pre-scenery main.py - so
those cost the repository nothing, and any of them that has stopped being
interesting can be deleted.  The `.py` tools in here and this file are the
exception: since 2026-10-23 they are tracked, because `_tmp_counts.py` reads
every `(N lines)` claim in `../README.md`, `../TODO.md` and this file back off
the files themselves, and a check nobody can run is not one - the end of
`../.gitignore` says which files those are and why the bulk is not among them.

## What is in it

**`_*.txt`, 131 files** - the text dumps and greps from mining the reference
PDFs: per-page extracts of TM-73254 (`_pg21.txt` onwards), the line numbered
dumps of TM-73258 and TN 54/58 (`_n54.txt`, `_n58.txt`, `_n58b.txt`), the
table-region greps (`_rot_tables.txt`, `_table2_region.txt`, `_n54_hits.txt`,
`_regA.txt` to `_regD.txt`), console logs from the mining scripts
(`_png_out.txt`, `_crop_log.txt`, `_tiff_out.txt`), old directory listings, the
node dumps that came before the scripts (`_raw24.txt` to `_raw28.txt`) and, from
the very first session, a listing of the reference documents with their sizes
(`_ref.txt`, `_ls_root.txt`).  Seventy-nine of the 131 were here when the sweep
was done and the rest are named below.

Also in here, because they were the same kind of throwaway: `_pip.txt` and
`_pipall.txt` (frozen pip lists), `_gitlog.txt` and `_gitstatus.txt` (each
holding the same "fatal: not a git repository" - the project was not under
version control until 2026-09-28), `_allfiles.txt`, `_todo.txt` and `_ls*.txt`.

`_todo.txt` is worth a warning: despite its name it was never a todo list.  It
is a grep for every line containing "right" or "collective" across `main.py`
and `aerodynamics.py`.  The real list is `../TODO.md`.

**Nineteen more `_*.txt`, moved here on 2026-10-23** - the excerpts of the model
read off a region at a time while the engine level's two commits were written:
`_m1.txt` to `_m4.txt` out of `main.py` (the fourth of them a numbered grep of
it rather than an excerpt, in the encoding a redirection leaves on this box),
`_m_a1.txt` and `_m_a2.txt` out of `airframe.py`'s autorotation trim and its
throttle, `_m_d1.txt` out of `_rt_demo.py`'s `pilot_note`, `_m_p1.txt` to
`_m_p4.txt` out of `_rt_pilot.py`'s trims and its own words, and `_m_s1.txt` to
`_m_s8.txt` out of `simulation.py`'s engine telemetry and its trims.  Every one
of them is a region of a file that is still in the project, so nothing is lost
if they go; they were moved rather than deleted because the sweep of 2026-09-28
set that precedent.

**`_imgs/`, 43 files** - the page image crops and overlays the mining scripts
produced from the scanned pages: PNG exports of the table pages and their
overviews (`page23_0.png` onwards, `ov24.png` onwards), the TIFF wraps of the
CCITT scans (`p23.tif` onwards), and the magnified bands that the linkage and
stabilizer bar constants were read off (`t2_ctl_en.png`, `t2_ctl_met.png`,
`p25_kb.png`, `t2_bottom.png`).  A human can still read those bands more
reliably than any OCR of the same page.

**`main.py.orig`** - the pre-scenery version of `main.py`: the grass plane, the
grid and five hardcoded trees, before the XML scenery loader existed.  Kept
only as the "before" picture; the tree and hill geometry it drew moved into
`main.py` with the scenery work.

**`controls_simple.png`, `attitude.png`, `altimeter.png`, `vertspeed.png` and
`airspeed.png`** - the five sketches the HUD of `main.py` is drawn from: a black
square with a red line, a red disc and a green field on it, a small attitude
indicator, and the three round faced instruments beside them.  All are references
and never inputs, so nothing opens them, and each shape is described in
`main.py`'s own comments as well as drawn.  `_attitude_hud.py` beside them is the
scratch that reads the attitude indicator back out of a real OpenGL context with
`glReadPixels`: it asserts the picture - the blue sky over the brown ground with
the green horizon and ladder across them, the one pixel those lines are drawn at,
the white reference fixed in the middle and over the green wherever the two cross
- and writes the five PNGs next to it, four attitudes of the panel and the whole
window with all seven of the HUD's panels on it.  Run it from the project root as
`python attic/_attitude_hud.py`.

**`_cockpit_window.py`** - the scratch that renders `main.py`'s three views into
PNGs for a human to look at, out of a hidden OpenGL context: the cockpit view on
the pad, the same frame with the two rotor functions stubbed out so that what is
overhead of the cabin can be seen to be the rotor and nothing else, the cockpit
view hovering, and the chase view of the same aircraft.  It writes
`_cockpit_pad.png`, `_cockpit_norotor.png`, `_cockpit_air.png` and
`_cockpit_chase.png` beside itself.  Run it from the project root as
`python attic/_cockpit_window.py`.

**`_keylog_check.py`** - the scratch that proves `main.py`'s key log writes a file
and not only the columns the `--check` holds in memory: it flies 240 frames of
1/60 s through `main.fly_frame` headless - a pickup from the pad with W held and
then released - writes `_keylog_check.csv` beside itself, reads the rows back,
checks the two clocks, the 20 Hz sample rate and the trace, prints them, and
deletes the file it made.  Run it from the project root as
`python attic/_keylog_check.py`.

**`_grip_probe.py`** - the scratch that walked the floor's heading gate lever by
lever, with the right pedal hard against its own stop for two seconds at each: it
prints the mixing law's own `collective_deg`, whether that reading is under
`simulation.GROUND_GRIP_ANGLE_DEG`, the heading the aircraft walked in those two
seconds, the yaw rate and the body rate the frame was left with, and whether the
skids were still down at the end.  Levers named on the command line are walked
instead of its own default list, and `out=FILE` puts the transcript in a file - so
the console is not needed - which is how the pair either side of the gate was
read.  Run it from the project root as `python attic/_grip_probe.py`; the same
boundary is asserted in `simulation.py`'s own self test.

**`_mouse_probe.py`** - the scratch that flies the mouse of `main.py` through the
frame loop in a real window, which is the part `main.py --check` cannot reach:
`main.Mouse.recentre` is `pygame.mouse.set_pos`, the cursor and its wheel are the event
loop's, and the merge into the frame is the loop's own call.  It takes the mouse
up the way `M` does (`main.mouse_for`, with the wheel reversed), turns four
notches of it, flies a second with `W` held to show the lever stays exactly where
the wheel left it, two seconds with `D` held to show the pedals are still the
keyboard's at their own key's rate while the cyclic is the cursor's own reading,
and presses `R` to show the lever comes back to the pad's 75 per cent.  It writes
`_mouse_probe.txt` beside itself.  The cursor cannot be *placed* from a script on
a window that is not focused, so it reads where the cursor is and checks the
aircraft went to that, which is the same assertion either way.  Run it from the
project root as `python attic/_mouse_probe.py`.

**The six `_tmp_` tools, moved here on 2026-10-21** - the scratch the
autorotation's cushion and the human flyability programme were worked out with.
They were the last loose files in the root: tracked by nothing, ignored by
nothing, and one `git add -A` from being committed, while `TODO.md`'s own
verifications were citing the seventh of them by name.  That seventh,
`_tmp_shadow.py`, is a *check* rather than a flight, so it stayed in the root and
is tracked - the other six are here, all of them still runnable from the project
root.

Four of them import the tracked harness as a library, so each carries the same
`sys.path` line as the scripts above: **`_tmp_sweep.py`** (271 lines) flies the
autorotation mission headless once per cell - `flare_ft,sink_mps,pull_in,gain` -
and reports the rate the skids met the ground at as the *model* measures it, read
off the last frame still in the air, which is where `_rt_pilot.py`'s
`CUSHION_PULL_IN`, `FLARE_SKID_FT` and the pull's own gain were read;
**`_tmp_anch.py`** (76 lines) prints the last few hundred feet of one such flight
as a series, the cushion's own *anchor* among the columns, and takes the descent's
lever on the command line; **`_tmp_diag.py`** (55 lines) prints the grip and the
phase frame by frame; **`_tmp_why.py`** (65 lines) prints the same through a spy
on `AutorotationPilot.decide`, so what is printed is what the pilot *saw* at each
of its looks rather than what the loop outside it made of that.

The other two import nothing.  **`_tmp_patch.py`** (133 lines) rewrites
`_rt_pilot.py`'s own text, CRLF and all, from the mission's one two hundred foot
manoeuvre to the manual's beat - the cyclic flare at fifty to sixty feet, the
level off at twelve - and `--revert` puts the two hundred foot schedule back;
`_rt_pilot.py` is opened by that name, so it is run from the project root as
`python attic/_tmp_patch.py`.  **`_tmp_runs.py`** (45 lines) launches a script
detached from this terminal's session, so a long sweep cannot be killed by the
shell it was started from, and writes `log_<tag>.txt` for the caller to poll.
What they write - `_tmp_*.txt` and those `log_*.txt` files - is in `.gitignore`
beside them.

**Three more `_tmp_` tools, moved here on 2026-10-22** - the scratch the ninth
harness module, `_rt_autoro_hand.py`, was flown and checked with.  All three
run from the project root like the six above.

**`_tmp_hand_probe.py`** (41 lines) runs one of that module's entry points -
`ruler`, `core`, `sweep` or `full` - inside a `try`, because the console on
this box is not to be trusted for a traceback, and writes its verdict to a
single slot, `_tmp_probe_out.txt`, beside the transcript that mode leaves
(`_tmp_ruler.txt`, `_tmp_core.txt`, `_tmp_sweep.txt` and `_tmp_full.txt`, all
in `.gitignore` with the rest).  It carries the same `sys.path` line as the
four above, and it is what flew the clock sweep - `0.05`, `0.10` and `0.20 s`
between decisions - at the same time as the full run: `probe()` writes the
path it is handed rather than `_rt_autoro_hand.txt`, so two such runs do not
collide.

**`_tmp_syntax.py`** (26 lines) imports nothing and parses one file with `ast`,
writing `_tmp_syntax.txt` with the lines around any problem if there is one -
which is how `_rt_autoro_hand.py` was read while its count was moving.

**`_tmp_counts.py`** (105 lines) is the check the two above were owed, and the
reason those counts are worth anything: `README.md` carries the model's nine
modules and the harness's nine in two `| module | lines |` tables, and
`TODO.md` and this file name a file's count in prose - `` `name` (N lines) ``
- and every one of those numbers was written by hand, so one file's count can
move in three documents at once.  It reads every one of them back off the
files themselves (the root first, `attic/` after it), prints a line each, and
exits 1 on any disagreement; it also names any `.py` in the root with no row in
a table and any `python x.py` in the run list with no file behind it.  Its
claims about this paragraph and about the two tools above are among those it
checks.

The same session deleted 31 console dumps and greps from the root:
`_tmp_v1.txt` to `_tmp_v12.txt`, the `*_console.txt` captures of a mode's own
output, the `_tmp_*.log` redirections, `_tmp_err.txt`, and the four
`log_*.txt` the detached runs had left.  What a run writes *for itself* stayed
- `_rt_autoro_hand.txt` (451 lines), `_rt_autoro_entry.txt` (226 lines),
`_rt_autoro_probe.txt`, `_rt_log.txt` and the probe's `_tmp_core.txt`,
`_tmp_full.txt`, `_tmp_ruler.txt`, `_tmp_sweep.txt` and `_tmp_probe_out.txt` -
because re-flying regenerates them, which is the whole difference between a
transcript and a copy of a terminal.

## What is not in it

Three things were moved out of the root rather than in here, because they turned
out to be source material and not scratch:

* `_tm73258.txt`, `_DTIC_AD0734343.txt` and `_DTIC_AD0739559.txt` are OCR texts
  of real reports, which nothing else in the project had a copy of.  They are
  now `reference/NASA-TM-73258_OCR-text.txt`,
  `reference/USAAMRDL-TR-71-34_AD0734343_OCR-text.txt` and
  `reference/AD0739559_OCR-text.txt`;
* the 19 mining scripts are in `../tools/`, with `../tools/README.md` to say
  what each one is for.

Two things were deleted, and can be regenerated:

* `_tm73254.pdf`, which was byte for byte the same file as
  `reference/NASA-TM-73254_UH-1H-force-and-moment-model.pdf` (MD5
  `f71dfd1e178bf7730d1476538cfc0ba5` on both);
* `out.txt` and `error_log.txt`, both zero bytes, and both empty since the first
  day of the project.
