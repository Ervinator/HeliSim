# TODO

## Next

The model's own modules are verified where they live: every one has a demo that
asserts as it prints - `scenery.py` and `controls.py` among them - and `main.py`
has `--check` for the wiring that flying cannot test.  What follows is what
`Done` leaves open, in the order it is worth taking up.

* **The engine is in; what is left of the level is around it.**  `engine.py`
  landed on 2026-10-08 with the airframe and simulation wiring that lets it drive
  the shaft - see `Done` below - so a torque now comes from somewhere: a T53 with
  a twist grip, an N2 governor with the manual's own +-40 rpm droop band, a
  0.6 sec spool, 1125 ft-lb of data plate torque, 50 psi of transmission, 7 psi
  for EMER and the manual's bleed air numbers.  Two things were open around the
  engine here; the first has closed and the second is what is left:

  * *main.py's own aircraft.*  It flies the report's fixed rotor, as it should -
    every figure in TM-73254 was computed with that - and what was left there was
    around the engine: the caption showed the engine's panel but not the engine's
    own torque against the transmission limit.  It reads it now, in the manual's
    own psi with the 50 psi limit beside them (2026-10-22, see `Done`).  (The
    device map named no throttle until 2026-10-16, so a mapped joystick or a home
    made collective could not roll the grip the keyboard could; it names the
    keyboard's own `Q` and `E` now, and one naming `throttle` on a device takes
    that pair over - see `Done`.)
  * *the torque available chart.*  Figure 7.1-2 is the real answer to "how much
    torque is there at this altitude and temperature", and it is read now:
    `UH1_TORQUE_CHART_10000FT_PSI` is the chart's 10,000 foot curve at every
    temperature the sheet prints, `torque_available_psi` puts the rest of the
    family on it at the chart's own spacing, and `Governor.power_available`
    answers from the chart rather than from an air density when it is handed a
    pressure altitude and a free air temperature (2026-10-22, see `Done`).
    What is left is finer than that: the 10,000 foot curve was read at each
    printed temperature and the other altitudes stand on the spacing fitted to
    the two values appendix C tabulates for 1,000 feet, so reading each curve's
    own crossings at each printed gridline would put every altitude on its own
    reading rather than on the family's spacing; and the airframe still hands
    the governor an air density, so nothing in a flight reads the chart yet.

* **A hand has been put on the autorotation; the harness's other two missions
  have not had one.**  `_rt_autoro_hand.py` - see `Done` - flies the
  autorotation's own entry with a person's clock on it, so what is left of that
  question is the rest of the harness's missions: `_rt_check.py`'s hover-turn and
  cruise fly their climbs, and the height, speed and heading they hold, off
  `_rt_pilot`'s own trim rows and *scripted*, and no hand has been put on either.
  What it needs is what the autorotation's took: the instrument's own clock, the
  phases of the mission read off the *view* rather than off the aircraft, and the
  trims read off the model rather than written down again.  The trim a hand is
  handed is the other half of it - the loops are `_rt_pilot.py`'s own and the lever
  position is the handbook's, with no blade stall and no flap limit behind them -
  so see the windmill bullet under `Known limitations` for what the model has
  behind a rotor taken off that trim, and why a vertical descent with the
  collective up is the frame it refuses outright.

* **The ground is a plane, and the next level is a terrain.**  Everything that
  touches the ground - equation 10a's cushion, the floor's skid line, the point
  the skids are pinned by, the levelling, `Telemetry.alt_agl` - is measured
  against one `airframe.GroundPlane`: a level and a normal, which a run may
  replace between frames.  What that is not is a *landscape*: a ridge, a valley,
  a step, a deck that pitches, or any surface whose level varies under the
  aircraft is several planes and the seam between them, and the work left is to
  choose where that seam lives - a terrain object that answers `height` and
  `above` per frame by this same interface, with `GroundPlane` the single flat
  patch it is built out of, or a plane-per-frame the way a deck would be.  Two
  other things belong with it:

  * *the renderer is still flat.*  `main.py`'s implicit grass plane and its grid
    are horizontal at y = 0 and its scenery's hills are drawn, not flown, so a
    leaning or a moving ground is the physics' own until a terrain is drawn and
    the camera, the shadow and the grid read it;
  * *the air over a terrain is a wind field.*  `aerodynamics.py` has the hook
    (`airflow`, a callable of position) and `simulation.Simulation.wind` is one
    steady vector, so the upwash off a ridge and the sink over a valley are the
    same level's work as the ridge.

* **Human flyability has two phases and its next one is named here.**  The
  harness has a section in `README.md` now, and the programme's own file says
  what the phase after *it* is for: `_rt_handling.py`'s docstring ends on "what
  this file measures is the acceptance criterion for the phases after it, and a
  criterion nobody can fail is not one".  What is left is the loops *above* the
  attitude.  Both phases hold the height, the speed and the heading at the
  model's own numbers on purpose - the question they ask is how hard a pilot may
  grip the *attitude*, not how fast the outer loops may be - so nobody has yet
  asked whether a person's clock can hold what `_rt_check.py`'s missions fly
  *scripted*, and phase 2's own finding is why it matters: the hover's margin is
  the cyclic's margin and nothing else, which says nothing at all about the lever
  and the pedals once a person is closing them.  `_rt_axes.py` is the scaffolding
  to build it on - the four axes, the scale sweep, the settle criterion and the
  PIO detector - so the phase is closing that instrument around the outer loops
  and pinning their bands the way phase 1 pinned the attitude's.

## Known limitations, from the modules themselves

These are choices, not oversights: each is stated where it lives.

* **Rotor speed is held, not driven - on the report's own aircraft.**  The rotor
  speed is a state variable, and an engine drives it: `engine.py` is a T53 with a
  twist grip, an N2 governor and its droop band, and `Airframe(engine=Engine())`
  - or `airframe_preset(engine=True)` - closes the shaft against it, so on that
  aircraft an engine failure, a governor that lags and a rotor that winds down are
  all in the model.  What is *not* in it is the default: the 8700 lb and 6158 lb
  aircraft of TM-73254 have no engine, because every figure in the report was
  computed with the rotor speed held at 100 per cent and the regression against
  those figures has to reproduce them to the last bit.  `controls.py` maps a
  `throttle`, and `main.py` reads it, carries it and shows it - as the pie on the
  collective's panel and beside the telemetry - but neither of those two aircraft
  has an engine for the grip to be the hand on - which a run no longer has to
  guess at: a grip moved with nothing behind it prints one line saying so and
  naming both cures, `F` and `--engine`, and `--engine` on the command line fits
  the T53 before the first frame instead of at the tenth (see `Done`).  `Next`
  records what is left around the engine.
* **The windmill root is read off the shaft, and it is unstable away from the
  trim.**  An engine failure is what offers the rotor the second root of
  equation 10: `Airframe.windmill` answers "has this shaft nothing driving it?"
  - a governor that has failed, a grip rolled right off, or a fixed
  `engine_torque` of zero - so there is no switch to throw, and the report's own
  aircraft, with no shaft at all, still never sees the root and every figure in
  TM-73254 stays where it was.  What that root makes possible is a *trimmed*
  autorotation - `trim_autorotation`, and `Simulation.trim_autorotation` around
  it - and what is *not* in the model is anything that holds a rotor *near* that
  condition: there is no blade stall and no flap limit, so a rotor taken well off
  the trim finds the root doing more than the trim asked of it and overspeeds
  without bound.  A *vertical* descent with the collective up is the worst of
  them - equation 10's two roots meet at zero airspeed - which is the frame
  `simulation.py` refuses as out of its envelope rather than passing on.  What the
  model does with a rotor taken off the trim is measured now, as far as the entry
  goes: `_rt_autoro_entry.py` takes it under the manual's own 90.7 per cent on
  both switches, out of a settled 60 kt climb at 500 ft, and nothing brings it
  back - which is the number this note is about.
* **The ground is a floor, not a contact model.**  `simulation.py` holds the
  c.g. on the skid line, takes the descent out of the velocity, and levels roll
  and pitch; the skids do not flex, slide or spring.
* **The ground is one flat plane, not a terrain.**  `airframe.GroundPlane` has a
  level and a normal and no shape, and it is what equation 10a's Z, the floor's
  skid line, the levelling and `Telemetry.alt_agl` are all measured against.  A
  level or a leaning ground, a ramp, a deck and a plane that moves under a
  hovering aircraft are all in it; a ridge, a valley, a step or any surface whose
  level varies under the aircraft is not - that is a terrain of several patches,
  and `Next` records where the seam for one belongs.  `main.py`'s world is still
  a horizontal plane at y = 0, so a leaning ground is the model's until a terrain
  is drawn.
* **The controls have no feel.**  `PilotInput` is rates rather than forces: full
  travels per second, so a cyclic crosses its stops in 0.6 s and the heavy
  collective takes 1.8 s over its 11 in of travel, and every axis ratchets.
  There is no breakout force, no friction level, no hydraulic boost and no force
  trim release to model, and nothing in either report says how fast a hand works.
  The keyboard is a quarter of those rates - `COLLECTIVE_STEP_GRANULARITY` and
  its two neighbours - so a key is a step and not a throw, and the same travel
  takes four times the presses; a key still has no force behind it.
* **Two tail rotor numbers are not in either report.**  The tail blade twist and
  its built in coning come from this project's own blade element survey - what
  `aerodynamics.py`'s self test runs - rather than from TM-73254 or
  TM 55-1520-210-10.
* **The model is evaluated in SI** while the reports tabulate English units.
  The conversions are written next to each constant, so both columns can be
  read, but a difference in the fourth figure can come from either.
* **`main.py`'s key log is stamped where the loop handles the key**, not where it
  was pressed: the event is taken out of pygame's queue before the frame is flown,
  so a line is up to a frame late - inside the 1/60 s the physics steps in
  anyway, which is the limit that matters, since nothing finer is a state the
  aircraft could have shown.  The two clocks are on every line, so a log can be
  read against the flight and against the time of day, but neither is a measure of
  how long the window took to see the key.

* 2026-10-22 - **the caption's torque gauge is read against the transmission, in
  the manual's own psi.**  The sandbox entry further down names this where it
  lives - "the caption's torque gauge is not read against the 50 psi
  transmission limit" - and `Next` was carrying it as the first of the engine
  level's two open halves.  It is closed without a new gauge and without a new
  telemetry field, by printing one number in the manual's own units: the caption
  reads

      | rotor 100.0 % | N2 6600 rpm | Q  37 % (19/50 psi) | AUTO

  on the T53 `F` fits live, which is the run the line above was printed by.

  * *the two numbers it is made of.*  TM 55-1520-210-10 keeps the limit and the
    calibration in consecutive paragraphs: "50 psi calibrated torque" is the
    transmission's structural limit (7.1-13) and the 1125 ft-lb data plate is
    "the indicated torque pressure at 1125 ft-lbs actual output shaft torque"
    (7.1-14), i.e. where the gauge reads 100 per cent.  With this project's own
    50 psi full scale (`engine.UH1_TRANSMISSION_LIMIT_PSI`) the two are the same
    reading, which is why a percentage could stand for it - and is also why the
    percentage was the wrong half of the pair: a pilot reads a psi against a
    psi, and 100 per cent of this gauge is a full scale the manual never prints;
  * *what it took.*  `EngineTelemetry` already carried the torque in newton
    metres, so the psi on the caption are `telemetry.engine.torque` at that
    slope - `engine.UH1_TORQUE_NM_PER_PSI`, 621.4 N m of rotor shaft torque per
    psi - with `engine.UH1_TRANSMISSION_LIMIT_PSI` beside them as the limit the
    reading is against.  Two imports in `main.py` (4559 lines), one format
    string, and nothing under the physics touched: which limit a reading is read
    against was always the caption's business, and the model already had both
    numbers.  A gauge shown against what the transmission will take is also the
    manual's own caution made visible - the engine "can exceed the transmission
    structural limit ... under certain conditions" (7.1-13) - because in dense
    air `Governor.power_available` scales the available torque past that limit,
    and the same line then reads over 50 psi.

  Verified: `main.py --check` printed `main.py check passed` with the new
  assertions written in the style the check already keeps - the engine panel as
  the exact string built from the telemetry it prints, `| Q %3.0f %% (%2.0f/%2.0f
  psi) | AUTO` on the freshly fitted T53; the psi and the percentage agreeing to
  1e-9, since they are one torque in two units; and the pad's own idling rotor
  reading above zero and under the limit rather than reading nothing.  The gate
  was re-run in full for it: the seven module self tests each printed their own
  `self test passed`, `regression_tm73254.py` printed its own, `_rt_check.py`
  flew both missions and wrote `_rt_log.txt`, and every one exited 0.  The
  table's own row for `main.py` is read back at what it now is,
  `README.md`'s caption paragraph names the limit the gauge is read against, and
  `attic/_tmp_counts.py` is at 39 claims - this entry's own line count among
  them - with none disagreeing.


## Done

* 2026-10-22 - **figure 7.1-2 is digitised: the chart's own 10,000 foot curve is
  in `engine.py`, the family's spacing is fitted to appendix C, and the three
  numbers the manual prints for the chart are asserted against both.**  The page
  had to be recovered before it could be read - the chapter 7 sheets are the ones
  the PDF's page tree lost, so the chart came back out of the raw objects (the
  bookmark tree gives page object 5763, its `/Contents` is six scan strips, each
  strip's rows arrive bottom-up, and the strips are decoded light on dark, which
  the first measurements had backwards and which made the grid's dots look like
  ink and its paper like a rule).  Read, the sheet says what the manual's text
  says it says: curves of calibrated torque against free air temperature for
  pressure altitudes from sea level to 14,000 feet, a printed maximum limit line
  at 50 psi, and a torque axis in *calibrated* psi - the manual's own worked
  example on the figure reads 36.8 psi calibrated and 39.8 indicated at 10,000
  feet and +15 C, the two differing by the engine's calibration factor of 66 psi
  at 1125 ft-lb (7.1-14), and this project's psi are the calibrated ones because
  50 of them is what the transmission will take.  The axes were calibrated off
  the chart's own printed numbers (a least squares line through its nine torque
  labels, worst residual 0.19 psi, and one through nine FAT labels, worst
  0.52 C), the 10,000 foot curve was then read at every temperature the sheet
  prints, and the spacing per 2,000 feet was fitted to appendix C's tabulated
  48.0 psi at 1,000 feet and +20 C and 42.7 at +30 C set against that same
  curve.  `Governor.power_available` answers from the chart when it is given a
  pressure altitude and a free air temperature, `Engine.max_torque` passes them
  through, and the chart's own limit is the cap the density scaling used to
  stand in for.  The readings are only as good as the scan: the curve is right
  to about half a psi, which is the width of the printed stroke.

* 2026-10-22 - **the hand's battery was re-flown to the last number it claims,
  the root was pruned back to transcripts, and the counts the three documents
  write down by hand are now read back off the files instead.**  The commit
  above left its questions in `Next`'s own terms and a session's worth of
  console dumps in the root; both are closed here, by re-flying every run the
  commit was verified with and by giving its numbers a check.

  * *the flights.*  `_rt_autoro_hand.py` was flown by `attic/_tmp_runs.py`
    detached from this terminal - 14:25:20 to 14:27:39, with the clock sweep
    flying beside it - and `_rt_autoro_hand.txt` (451 lines) ends on the same
    `check passed` it was committed on: the mission's own row to the decimal,
    every person's flight flown at the clock it claims, every view 0.20 s old.
    The entry was re-flown too (`_rt_autoro_entry.txt`, 226 lines), the
    aircraft `_rt_autoro_probe.py` measures, the rehearsal through
    `_rt_check.py` - the short mission at both of a person's reaction
    times, the long one as the frame loop's regression - and the wiring
    through `main.py --check`;
  * *the two tools that were owed.*  `_tmp_hand_probe.py` (41 lines) runs one
    entry point of the new module inside a `try` and writes its verdict to
    `_tmp_probe_out.txt` - `ruler: OK` and `sweep: OK` are what the ruler
    reading and the 0.05/0.10/0.20 s clock sweep produced - and
    `_tmp_syntax.py` (26 lines) parses one file with `ast` and prints the
    lines around any problem.  Both were in the root and both are in `attic/`
    now, carrying the `sys.path` line the six there already have;
  * *the prune.*  Thirty-one console dumps and greps went: `_tmp_v1..v12.txt`,
    the `*_console.txt` captures of a mode's own output, the `_tmp_*.log`
    redirections, `_tmp_err.txt` and four `log_*.txt` a detached run left.
    What a run writes for itself stayed - the transcripts above,
    `_rt_log.txt` and the probe's `_tmp_probe_out.txt` - and `_tmp_shadow.py`
    stayed because the entry above verifies with it.  Nothing was added to
    `.gitignore`: every one of them was already covered;
  * *the check underneath.*  `_tmp_counts.py` (105 lines), in `attic/`, reads
    every count the documents claim - the model's nine and the harness's nine
    in the two `| module | lines |` tables, and every `` `name` (N lines) ``
    in `README.md`, `TODO.md` and `attic/README.md` (which records this move)
    back off the files themselves, one line each, the root first and `attic/`
    after it, and exits 1 on any disagreement.  It asserts the other direction
    as well: no `.py` in the root without a table row, and no `python x.py` in
    `README.md`'s run list without a file.  Its own count is one of the claims.

  Verified: every number above is a run's own output rather than a reading of
  one - `_rt_autoro_hand.txt` ends `check passed`, `_rt_log.txt` ends
  `check passed: cruise at reaction 0.25 s, mission complete at 93 s, hands off
  (1001 m out)`, `main.py --check` printed `main.py check passed`, and the
  duplicate-name check found 8 module-level names in `_rt_autoro_hand.py`
  and 76 in `_rt_pilot.py`, none written down twice.  The check this entry is
  about was itself tried the other way: `README.md`'s row for
  `_rt_autoro_hand.py` was set to 1400 and an unnamed `_zztmp_testmod.py` was
  dropped in beside it, and the tool named both and exited 1 - both were put
  back byte for byte.  Nothing in the model was touched: the change is prose in
  this file and a comment in `.gitignore`.

* 2026-10-22 - **the autorotation flown by hand: the *entry* holds, the *arrival*
  is what a hand changes, and the model will not fly a hand slower than its own
  delay.**  `Next` had left the hand open - "a person at the keys ... is what has
  not been measured" - and the harness gained a third autorotation instrument:
  `_rt_autoro_hand.py` (1417 lines), which puts phase 2's instrument on the
  *mission's* own loops and flies both of its switches through it.  Every gain is
  `_rt_pilot.py`'s own pair under the instrument's scale and every reading comes
  off the *view*, so the loops are the mission's and the only things between its
  numbers and this file's are the clock, the eyes, the hand and the grip.  What
  the file asserts before it flies anything is that claim: at a scale of one, with
  no hand lag and with the aircraft itself handed to the decision instead of a
  memory of it, one hand decision lands on `_rt_pilot.AutorotationPilot`'s own to
  1e-12 of an inch.

  * *the clocks.*  Two, and only one of them the mission's: the instrument's own
    twenty decisions a second, and the *slowest* clock it will fly, one decision
    per delay.  The mission's own scripted 0.25 s is longer than that and the
    instrument's own `selftest` refuses it (`period <= DELAY_S`), so the file
    asserts the refusal rather than pretending to fly it - which is a finding in
    itself: the row everything is read against is a *script's* clock, and the
    slowest hand the model has decides exactly as often as its eyes lag;
  * *the finding.*  On the `"flight test"` aircraft at 500 ft of skid and 60 kt,
    the *entry* holds and the *arrival* does not: the instrument's own clock gives
    195.9 ft of catch in 7.1 s against the mission's 231.8 in 8.9 on the grip's
    own cut, on the same 11.90 m/s descent, and arrives at 3.13 m/s where the
    mission arrives at 2.26 - outside the floor's own 3.0 - while the failure's
    entry is 195.1 against 189.7 ft and arrives at 1.59 m/s, inside it.  The
    reason is in the file and it is not the entry: a hand moves the collective
    about four times faster than `CUSHION_KEY_RATE_IN`, so it outruns the
    mission's one-way cushion schedule, arrives at a lever the schedule has
    stopped asking for, and comes down again on what the pull has spent;
  * *the slow clock.*  At a decision per delay the loops give out before the
    mission has begun: the aircraft never settles in the mission's own band, the
    switch is never thrown, and there is no entry to measure - out of envelope 78 s
    into the climb on the grip's own cut, and a crash in the cushion at 43 s on the
    failure.  That row is empty in the transcript, and `check` asserts the absence
    as the finding it is: the mission's own climb is closed to a hand that slow.
    Ten decisions a second flies it - 199.5 ft of catch and a 3.03 m/s arrival - so
    the line is between the two clocks;
  * *the band.*  The grip's own two edges, out of the mission's own gains: the
    harder edge is x3.00 of the cyclic, past which the *climb* departs and neither
    entry is flown at all (x1.50 and x2.00 both hold), and the softer edge is
    x0.35, where the failure's own descent falls out of the mission's own 1.0 m/s
    band while the grip's cut still reaches it;
  * *verified.*  `python _rt_autoro_hand.py` asserts its whole transcript
    (`_rt_autoro_hand.txt`), which carries the mission's own row to the decimal
    `_rt_autoro_entry.py` commits it at, every person's flight at the clock it
    claims with every view 0.200 s old and no frame dropped, the two ends of the
    person's own entry, the refusal of the mission's own clock, and the two edges
    of the grip's own band.  `python _tmp_shadow.py _rt_autoro_hand.py` reads 8
    module-level names with none written down twice; `python _rt_autoro_entry.py`
    and `python _rt_autoro_probe.py` still print what they printed; `python
    _rt_check.py` still passes both missions at both reaction times; and the model
    itself is untouched - the change is one harness module, its transcript, and
    `README.md`'s harness table, phase list and layout tree.

* 2026-10-21 - **the autorotation's other switch is measured: a failure out of a
  settled climb *is* a flight, and what it costs is the hand.**  `Next` had
  recorded the open end as the *entry*, and it was right about the model and
  wrong about the sandbox: every key for it already existed - `X` is
  `set_governor(failed=True)`, `Q` rolls the twist grip right off, and `F` or
  `--engine` fits the engine the report's own aircraft has not got - and what had
  never been done was measure what the model does *between* a switch and the
  trimmed descent.  So the harness gained a second autorotation instrument:
  `_rt_autoro_entry.py` (676 lines), which flies the mission's own pilot through
  both switches, at two of a person's decision rates, and asserts every number it
  prints, the way `_rt_axes.py` and `_rt_handling.py` do.

  * *what the model does.*  On the `"flight test"` aircraft at 500 ft of skid and
    60 kt, both switches end in the same descent - 12.2 to 12.3 m/s against the
    trim's own 11.59 - and neither is refused: no `OUT OF ENVELOPE` frame, no
    crash, and the skids arrive at 1.7 to 2.5 m/s against the 3.0 m/s the floor
    counts as hard.  The failure is the *quicker* entry - one governor lag against
    the grip's four seconds of quarter rate key - with a 190 ft catch in 7.5 s
    against the scripted cut's 232 ft in 8.9 s;
  * *what it costs.*  The rotor, which the model does not hold: both entries take
    it under the manual's own 90.7 per cent and put the skids down on a rotor
    already below it.  How far below is the *hand's* doing as much as the
    switch's, and that is why the file flies two decision rates - at 0.25 s the
    failure leaves 72 per cent at the skids against the grip cut's 87, and at
    0.35 s the grip's own cut is the worse of the two (60 against 70), because a
    slower hand holds the level trim for longer with nothing driving the shaft;
  * *what is left.*  The loops are `_rt_pilot.py`'s own and the lever position is
    the handbook's trim, so a person at the keys - no trim row to read, and no
    blade stall and no flap limit behind them - is still unmeasured.  `Next` says
    so, and points at the windmill note for what the model has not got.
  * *verified.*  `python _rt_autoro_entry.py` asserts its whole transcript
    (`_rt_autoro_entry.txt`, 226 lines at both rates); `python
    _rt_autoro_probe.py` still prints the same trim the new file reads (11.59 m/s
    down, 4.44 in of collective); `python _tmp_shadow.py _rt_autoro_entry.py`
    reads 19 module-level names with none written down twice; `python
    _rt_check.py` still passes both missions at both reaction times; and
    `README.md`'s harness table, its layout tree and `.gitignore`'s own comment
    count eight `_rt_` modules now rather than seven.

* 2026-10-21 - **the last scratch in the root is accounted for: one tool kept,
  six retired to `attic/`.**  Seven `_tmp_*.py` files were sitting in the root
  tracked by nothing and ignored by nothing - `.gitignore` covered `_m*.txt`,
  `_rt_*.txt` and `sim_demo.txt`, and `attic/` already existed for exactly this
  kind of work - so one `git add -A` would have committed them, while the two
  entries below were already verifying with one of them by name.  That one is a
  check and not a flight, so it stayed and is tracked; the other six went to
  `attic/` beside the scratch that used to fill the root, and `.gitignore` now
  says which `_tmp_` file is the project and which is not, the way `_rt_*.txt`'s
  own comment already does for the harness's transcripts.

  * *the keeper.*  `_tmp_shadow.py` (60 lines): every name a module binds at
    module level more than once, read off its syntax tree rather than hunted for.
    It is how `_rt_pilot.py`'s own `CLIMB_ALT_M` was found written down twice -
    eighty metres at the top of the file and the engine-off mission's five hundred
    feet further down, so the long mission's speed ramp had been solved against a
    height it never reaches, which is what `AUTOROTATION_CLIMB_ALT_M` is the
    second binding of.  It reads any file, so it is run for a new module the way
    `_rt_check.py` is run for a mission: `python _tmp_shadow.py _rt_axes.py` reads
    35 module-level names with none written down twice, `_rt_handling.py` 40 with
    none, and both were run again to write this entry - and both are cited by the
    two entries below, which is the reason it could not stay untracked;
  * *the six.*  `_tmp_sweep.py` (271 lines) - the autorotation's cushion flown
    headless once per cell, `flare_ft,sink_mps,pull_in,gain`, reporting the rate
    the skids met the ground at as the *model* measures it, which is where
    `CUSHION_PULL_IN`, `FLARE_SKID_FT` and the pull's own gain were read;
    `_tmp_patch.py` (133) - the manual's own beat patched onto `_rt_pilot.py`'s
    text, the level off at twelve feet of skid and all, `--revert` to put the two
    hundred foot schedule back; `_tmp_runs.py` (45) - a run started detached, so
    this box's terminal cannot take a long sweep away mid-flight; and the three
    that print one flight's numbers rather than measuring anything -
    `_tmp_anch.py` (76) for the cushion's own anchor, `_tmp_diag.py` (55) for the
    grip frame by frame, `_tmp_why.py` (65) for what the pilot *saw* at each of
    its looks, spied on `AutorotationPilot.decide` itself.  Four of the six import
    the *tracked* harness as a library and nothing imports them, so each of those
    four keeps the `sys.path` line `attic/`'s older scripts carry, and all six
    still run from the project root as `python attic/_tmp_sweep.py` and friends -
    `_tmp_patch.py` opens `_rt_pilot.py` by that name, CRLF and all, which is why
    it is the one that cares where it is run from.  Nothing was deleted: the
    autorotation entries below were worked out with these, and their numbers can
    still be traced back through them, which is what `attic/` is for;
  * *and a rationale that was ahead of its tree.*  The entry below that tracks
    `_rt_autoro_probe.py` (`a033557`) justifies tracking it partly because "the
    four `_tmp_` tools import its `key_states` and `skid_ft`, so a clone without it
    has dangling references" - which was true of a working copy and not of the
    repository, since none of those tools was tracked.  They are in `attic/` now
    and still importing it, so the reasoning holds on both counts.

  Verified: `python _tmp_shadow.py _rt_axes.py` and `python _tmp_shadow.py
  _rt_handling.py`, green at 35 and 40 module-level names with none bound twice;
  `python main.py --check` printing `main.py check passed` and
  `python regression_tm73254.py` printing `self test passed`, the model itself
  untouched by any of it; and `git status --short` showing `_tmp_shadow.py` as the
  only file added, with nothing loose left in the root.

* 2026-10-21 - **the harness is written down: seven `_rt_` modules, four of them
  thirteen days older than the entry that names them.**  Human flyability has two
  entries of its own and nothing else, and everything the two of them stand on was
  unnamed: `_rt_pilot.py` (1480 lines) and its three siblings were added in a
  single commit on 2026-10-08 and appeared in this file only as references inside
  somebody else's entry, `_rt_demo.py` was not named here at all, `README.md`'s
  module table lists the model's nine and said nothing about the seven beside
  them, and the one place that enumerated the harness was `.gitignore`'s own
  comment.  What is new is an account: a *The harness* section in `README.md` -
  the seven, what each is, how to run each, what each writes, and where the two
  flyability phases and the autorotation probe sit in it - plus this entry, which
  is the provenance the README cannot carry.

  * *the pilot.*  `_rt_pilot.py`: the keyboard pilot with a person's timing that
    drives `main.py`'s own frame loop.  The trims are the baseline -
    :class:`TrimTable` reads them off the model, so the loops are small signal and
    only have to catch what the trim does not - the pilot reads the instruments
    every `REACTION_S` seconds on the *flight's* clock rather than the frame's,
    and a key is held until the axis crawls to the position the pilot asked for.
    Three missions, three classes: `HumanPilot` (the pad to a hundred metres up
    and a kilometre out, which is what the playability runs flew),
    `HoverTurnPilot` (the minute of hover, turn and landing),
    `AutorotationPilot` (five hundred feet at sixty knots, the twist grip rolled
    right off, and down again on the rotor - whose climb height is
    `AUTOROTATION_CLIMB_ALT_M`, the second binding phase 0.1 made when
    `CLIMB_ALT_M` turned out to be bound twice).  Nothing but the nine keys is
    touched in any of them;
  * *the ruler.*  `_rt_probe.py` (118 lines): the numbers the pilot's gains have
    to be built from - travels, the pad's stick, the hover trim and the model's
    own trims for the speeds a mission wants - read and not guessed, so a loop is
    scaled to the aircraft rather than to trial and error.  `hover_step` is the
    instrument: one control stepped *inches* off the hover's own trim through
    :meth:`simulation.PilotInput.reset` - an axis, a ratchet, nothing held - and
    what is printed is the model's answer, not a pilot's: how many degrees a
    second an inch of pedal buys, how much pitch an inch of long stick buys and
    what speed that pitch settles at;
  * *the rehearsal.*  `_rt_check.py` (270 lines): the realtime demo's own
    rehearsal - the pilots through `main.fly_frame`, the same keys, ratchet, clock,
    camera and view as the window - with no renderer and no waiting, so the loops
    are read off a transcript before anybody watches them and a mission's claims
    are *asserted* rather than eyeballed.  The short mission is flown at two of a
    person's reaction times, since it is a minute of flight and its shape should
    be the same at both, and the long one as the regression that the frame loop the
    two pilots share still flies what it flew;
  * *the window.*  `_rt_demo.py` (260 lines): the same window `python main.py`
    opens - the same `set_mode`, `init_gl`, `fly_frame`, `draw_scene` and panels -
    with `HumanPilot` or `HoverTurnPilot` standing at the keyboard instead of a
    person, so the pickup, a hover, a turn and a landing can be *watched* at sixty
    frames a second; `C`, `R`, `P` and `Esc` on top of the pilot's own eight keys,
    a mission name or a scenery file as the first argument, and a count of physics
    steps so the whole driver can be checked end to end in one command.  One thing
    in it is asked of a key a *watcher* would press rather than by the pilot: the
    engine, because an unattended pilot cannot press `F` and a rotor *held* at a
    hundred per cent cannot autorotate at all - which is the aircraft
    `_rt_autoro_probe.py` measures;
  * *and the three the programme added* - `_rt_handling.py` (2026-10-20),
    `_rt_axes.py` (2026-10-21) and `_rt_autoro_probe.py` (2026-10-21) - each
    already written up where it was measured, so the harness is seven modules
    accounted for one by one, four of which flew for thirteen days before
    anything here named them.

  Verified: the four oldest arrived in one commit, `1accaca` of 2026-10-08, read
  off `git log --diff-filter=A`; the count of seven agrees between `README.md`,
  `.gitignore` and this entry; pygame is imported by `_rt_pilot`, `_rt_demo` and
  `_rt_autoro_probe` and by the other four through `_rt_pilot`, which is what the
  README now says; and nothing in the model was touched - the change is prose in
  `README.md` and this file, with `python main.py --check` printing
  `main.py check passed` and `python regression_tm73254.py` printing
  `self test passed` where they were.

* 2026-10-21 - **the hover's margin is the cyclic's margin and nothing else: the
  tilt pair's band is *wider* at sixty knots, and the lever's and the pedals' do
  not transfer at all.**  Phase 1 asked whether a person's hands can hold this
  model's hover at all, and answered it with the two *tilt* axes - and what that
  file really is, it says of itself, is an instrument, with everything below the
  attitude loop left at the model's own numbers.  `_rt_axes.py` is the rest of
  that sentence, through the same instrument (twenty decisions a second on the
  flight's own clock, 0.2 s of delay out of a deque of remembered states, 0.1 s
  of lag in the hand, and the tilt loops still gain for gain phase 1's) and with
  two more of the pilot's loops under scales of their own: the lever, whose gain
  is `_rt_pilot.K_RATE_IN` inches of collective per m/s of climb rate the loop is
  short of, and the pedals, a hover's heading hold being the model's turn loop
  with the turn's schedule taken out of it, so what is scaled is inches of pedal
  per degree of nose and per degree a second of yaw rate, kept at the model's own
  lead ratio.  A metre of height and ten degrees of nose are then put in beside
  phase 1's two degrees of attitude, each read against a band that is a *fraction*
  of what was put in, and all of it is re-flown at sixty knots.  The answer is
  that the hover is where the *attitude* has no damping and is **not** the hardest
  condition for the other two axes: a margin read in the hover is a margin for the
  cyclic and nothing else.

  * *the nominal case of each axis, and the lever's fourteen seconds are not the
    lever's.*  A metre of height takes the model's own lever 14.42 s to take back
    out and ten degrees of nose 4.03 s, so neither of the two axes phase 1 left
    alone is a slow axis of the aircraft.  The fourteen is not the lever's
    either: a hover's height is asked for through the model's own
    `RATE_GAIN_MPS`, twelve centimetres a second per metre, and it is that outer
    loop's time constant the settling is really measuring - which the hands-off
    case beside it proves by bringing the same metre back in 9.5 s with nobody
    flying;
  * *the two tilt axes do not have the same band, and phase 1's one band was
    pitch's.*  Roll is *softer* below - x0.50 holds where pitch has already
    failed - and *harder* above, where x1.50 is the pilot's own loop at a 1.05 s
    period while pitch's x1.50 is still the aircraft's 11.5 s mode and its PIO
    waits for x2.00.  So the bands are x0.75 to x1.00 of the model's own attitude
    gains for pitch and x0.50 to x1.00 for roll, and the pilot the model is flown
    with is at the hard edge of both;
  * *the lever and the pedals have no soft edge in the hover at all.*  Every grip
    down to x0.25 held on both, because what holds those two axes is the outer
    loop and the aircraft rather than the pilot's hand: the cushion brings the
    metre of height back on its own, and a hover's nose with the pedals nowhere
    near stiff enough is still caught by the gain and the lead the model's own
    turn loop is flown with.  Their hard edges are their own loops and they are
    much further up, and both of them are PIOs - x2.00 on the lever, 2.72 s a
    cycle and growing five per cent of itself, and x4.00 on the pedals, 1.25 s
    and twenty-three;
  * *and at sixty knots the tilt pair's bands transfer while the other two do
    not.*  Both grips the hover's band failed at hold out there - pitch's x1.50
    in 2.53 s and its x0.50 in 3.30 s, roll's x1.50 in 3.17 s and its x0.35 in
    4.37 s, which is a *softer* grip than the one the hover failed at - the speed
    comes back with the attitude, and the nominal pitch loop settles in 2.80 s
    where the hover took 8.48 s.  On the lever and the pedals the same scale is a
    stiffer loop, not a looser one: x2.00 of lever runs to -9.08 m and x4.00 of
    pedals to -51.40 deg with 5.16 m of height gone, neither settling at all and
    both growing a cycle at a time.  Nor are those two merely unchanged out there
    - at the model's own grip, with nothing but the airstream different, the
    nominal settle goes 14.42 s to 20.78 s on the lever and 4.03 s to 22.37 s on
    the pedals;
  * *with nobody flying them, in both conditions, which is where it shows.*  A
    hover's height is a place the aircraft finds by itself - the metre comes back
    to +0.03 m, inside the band - and sixty knots has no cushion under it, so the
    same metre simply stays where it was put, at +1.00 m.  Roll *is* given back
    out there, to +0.05 deg, where the hover wanders out the other way to
    -3.38 deg.  Pitch comes back partway out there and no further: +1.42 deg of
    its two still on the clock when the twenty seconds run out, against +2.12 in
    the hover.  And the nose parks exactly where it was put in **both**
    conditions, 9.9996 of its ten either way - the perturbation is a heading, so
    nothing is slipping for the fin to weathervane against, which is why a
    hover's yaw margin has nothing out there to be read against.

  Verified: `python _rt_axes.py`, which writes `_rt_axes.txt` and asserts every
  number in it - the nominal case of all four axes, both legs of all four bands,
  the PIO of the two that close one, the whole check at sixty knots and the
  hands-off reference in both conditions, with the tilt rows held to phase 1's
  own pinned numbers as the check that the two files are one instrument;
  `python _tmp_shadow.py _rt_axes.py` reading 35 module level names with none
  written down twice; `python _rt_handling.py` re-run and green; and the model
  itself untouched: `python main.py --check` printing `main.py check passed`,
  `python regression_tm73254.py` printing `self test passed`, and
  `python _rt_check.py` flying its missions to `check passed` in `_rt_log.txt`.


* 2026-10-20 - **a person's hands are in the loop now: the hover has a gain
  *band*, and the model's own hover pilot is at the top of it.**  Every harness
  beside this one flies the aircraft with a *script* - `_rt_pilot` decides four
  axes off the instruments, `_rt_probe` steps a stick off its trim, `_rt_check`
  asserts what a mission did - and none of them asks the handling qualities
  question, which is whether hands slower than a frame, and later than the
  instruments, can hold the machine at all.  `_rt_handling.py` is that question:
  the model's own twenty foot hover, closed around a human's dynamics.

  * *a clock, a delay, a gain, a lead and a lag.*  Twenty decisions a second on
    the flight's own clock; what a decision reads is the aircraft as it was 0.2 s
    ago - the last twelve frames, kept in a deque - so the loop is closed on a
    *remembered* aircraft and the first fifth of a second of every case is flown
    on the trim rather than on the perturbation; the stick is 0.100 in of long
    stick per degree of attitude error, with 1.5 s of lead on the rate and 0.1 s
    of lag in the hand.  The gain and the lead are not invented here: they are
    `_rt_pilot`'s own `K_HOVER_PITCH_IN` and the ratio
    `K_HOVER_RATE_IN / K_HOVER_PITCH_IN`, read off that module and asserted back
    against it, so a scale of one *is* the pilot `_rt_check` flies the short
    mission with - and everything the file measures is that pilot's margin rather
    than a stranger's.  The loops underneath the clock are that pilot's own too,
    loop for loop: the tilt a hover is held with, the height on the lever, the
    heading on the pedals, and only the two attitude gains are the sweep's;
  * *two degrees, both tilt axes, and the band.*  A hover trimmed, +2 deg put
    into the attitude, and the pilot's job to get back inside 0.5 deg and *stay*
    there for 2 s.  The model's own pilot does it in 8.48 s of pitch and 2.02 s
    of roll, holding the height to a centimetre and the machine inside 2.3 m of
    the pad.  A quarter softer (x0.75) still holds; x0.50 does not, and the way
    it fails is that the attitude wanders a degree either side at a 7.7 s period
    that no longer decays at all.  On the hard side x1.50 still comes back inside
    the band at 7.9 s - and then grows straight back out of it at 11.5 s.  So the
    band is **x0.75 to x1.00 of the model's own attitude gains, -2.5 dB to
    0.0 dB**: the pilot the model is flown with is the *hardest grip that holds
    it*, with no room above the gains it is flown with at all;
  * *and the PIO is not the band's edge.*  At x2.00 the failure changes kind:
    1.28 s a cycle, growing 37 per cent of itself a cycle, with the pilot's own
    0.2 s of delay worth 56 degrees of that period - a pilot induced oscillation,
    closed by the pilot's lag, which is a thing no aircraft mode can be.  Both
    edges of the band fail on the model's own slow modes, where the same delay is
    worth only six and nine degrees of phase; so this hover's *margin* belongs to
    the aircraft and the outer loops, and the *PIO* belongs to the pilot, which is
    the answer the phase was for;
  * *with nobody flying it, for reference.*  The same perturbation with the sticks
    where the trim left them: 20.9 m off the pad inside 20 s, the error doubled to
    -4.2 deg, and never back inside the band - which is what the pilot's own 2.3 m
    and 8.48 s are bought against.

  Verified: `python _rt_handling.py`, which writes `_rt_handling.txt` and asserts
  every number in it - the pilot's own clock measured in flight (100 decisions in
  5.02 s, every view exactly 0.2000 s old, no step dropped), the nominal cases,
  the two legs of the band, the PIO and the hands-off reference;
  `python _tmp_shadow.py _rt_handling.py` reading 40 module-level names with none
  written down twice; and the model itself untouched: `python main.py --check`
  printing `main.py check passed`, `python regression_tm73254.py` printing
  `self test passed`, and `python _rt_check.py` flying both missions to
  `check passed`.

* 2026-10-20 - **the mouse flies the aircraft: the cursor is the cyclic and its
  wheel is the collective.**  A UH-1's two controls that need a hand's own feel
  were the two this sandbox could only fly from the keyboard or from hardware, so
  a run with no stick on the desk had no stick: `M` now gives it the mouse, and
  `--mouse` asks for it before the first frame - the one control in the project
  that arrives with no device behind it at all.

  * *the cursor is the cyclic, and it is read as a position.*  The window is the
    stick's own scale, so its middle is the cyclic's own centre and its edges are
    the stops; the cursor's direction from the middle is the arrow key's of that
    direction, which is the rule a pilot can remember - above the middle is cyclic
    forward and the nose down, right of it is roll right - and a 4 per cent band
    about the middle (`MOUSE_CYCLIC_DEADZONE`) is the cyclic centred, because a
    hand never leaves a cursor exactly there.  `mouse_cyclic` is the arithmetic,
    with no pygame mouse to run it on, the way `controls.axis_value` is a
    joystick's own reading with no joystick;
  * *the wheel is the collective, a notch at a time.*  A notch is a fiftieth of
    the lever's travel (`MOUSE_WHEEL_COLLECTIVE_STEP`, 0.22 in of the UH-1H's 11),
    the lever follows at the rate a hand would move it because the frame's own
    `set_axes` slews it, and where the wheel leaves it is remembered across frames
    - a notch arrives as an *event* and is read in a *frame*, so it is the one
    thing about the mouse that has to outlive a frame.  Both stops clip it.
    `--mouse-reverse` is the wheel's one option, and it is the option the request
    named: a notch of the wheel down for more collective instead of up for more.
    Which way the run reads it is a line it prints when the mouse is taken up,
    because a lever that goes down when the wheel goes up is a broken lever to a
    pilot who has not been told;
  * *one control is one hand, and the lever is read back off the aircraft.*  The
    mouse flies exactly the cyclic's two axes and the collective (`MOUSE_AXES`),
    and `hands_on` takes the map's own copies of those out of the frame's ratchet
    dictionary as it puts the mouse's positions in, so `W` and the arrows go quiet
    while the pedals stay on `A` and `D` and the grip on `Q` and `E`.  Taking the
    mouse up - with `M`, or at startup - reads the lever off
    `PilotInput.collective_axis` and puts the cursor in the middle of the window
    (`Mouse.recentre`), so it is no step to the hands; `R` and `P` do the same
    reading, or the wheel's last notch would slew the lever straight off the pad a
    park had just put it on;
  * *the mouse is not a device of the map.*  `default_controls.xml` names
    keyboard and joystick devices and the mouse is neither: it is `main.py`'s own,
    with its own two switches, which is why `command_line` now answers five values
    - the mouse's being one of `MOUSE_OFF`/`MOUSE_ON`/`MOUSE_REVERSE` - and why
    the key log has a `mouse`/`devices` kind for the `M` key beside the four
    halves it already had.  Both switches are stripped like `--engine`, and
    `--mouse-reverse` implies `--mouse`, since a reversed wheel that nothing reads
    is not a thing a run can ask for.

  Verified: `python main.py --check`, with its own new block - the four directions
  about the middle of the window and the clipped corners, the 19 px inside the
  deadzone and the 20 px that are not, the wheel's notches, stops and reverse, the
  `Mouse` class's own frame-to-frame memory, and `hands_on` against the mapped
  board: the mouse's three axes in the positions, the map's copies gone from the
  ratchets, the pedals and grip still read on the same frame, and a run with no
  mouse handed its dictionaries back untouched.  `attic/_mouse_probe.py` - scratch,
  not part of the project - then flew it in a real window: `--mouse-reverse` taken
  up on the pad reads the lever back at the pad's own 0.7500 and centres the
  cursor; four notches *lower* it to 0.6700 where `set_axes` follows; a second of
  the loop with `W` held as well leaves the lever at exactly 0.6700 with the skids
  down and the aircraft uncrashed (the key read by nothing); two seconds with `D`
  held moves the pedal by exactly its own key's rate - `key_rates()[2]` over those
  two seconds, 0.5 of its travel - while both cyclic axes end on the cursor's own
  reading; and `R` puts the lever and the aircraft back on the pad.  Its two
  `MOUSEWHEEL` fields were read back out of pygame's own queue as well, since a
  script cannot borrow `main()`'s while loop.  `python regression_tm73254.py`
  unchanged, `python simulation.py` printing `self test passed`, and the module
  line count of `README.md` and `main.py`'s docstring brought up to the file.

* 2026-10-19 - **the skids have the heading: below 4 deg of rotor, full pedal on
  the ground now yaws nothing at all.**  The report was that a parked UH-1 with
  the lever low turns its nose on the pedals anyway, and the aircraft was right:
  the yaw *rate* was being taken out of every frame by `GROUND_FRICTION_PER_S` -
  which is what a helicopter on its skids does - but a heading is not a rate.  The
  Euler kinematics turn a body rate into a heading at a rate of their own, so a
  frame whose yaw rate had just been zeroed still came out of the floor a fraction
  of a degree round, and two seconds of right pedal at the stop walked the nose 22
  deg the pilot never asked for.  A pedal has nothing to react against while the
  disk is not lifting the aircraft: the weight is on the skids, and the skids are
  what the yaw moment goes into.  So the floor says so:

  * *`simulation.GROUND_GRIP_ANGLE_DEG` is 4.0 deg of rotor, and under it the
    heading comes back exactly as the frame found it.*  `Simulation._ground()` is
    handed the state the frame began at and answers `before.attitude.z`, so what
    the floor owns is the heading itself and not only the rate - which is the
    whole of the fix, because damping the rate alone leaves the walk, the walk
    being in the attitude the frame is handed back;
  * *the gate reads the pilot's own number.*  It compares
    `Telemetry.collective_deg` - the mixing law's angle, the one the caption and a
    HUD show - rather than a lever fraction, so what the floor reads and what a
    hand sees cannot disagree, and the lever that lands on it is 0.253866 of the
    travel rather than anything round;
  * *and above the gate nothing changed.*  The 6158 lb aircraft's pad lever is
    11.8 deg of rotor, so a run up on the pedals still pivots it on its skids
    exactly as it did, which is what a pickup uses.  The gate is a gate and not a
    brake on the floor: what it owns is a stationary aircraft's heading, and only
    while the disk is too slow to be lifting it.

  Verified: `python simulation.py` printing `self test passed`, with a new
  boundary block in its self test beside the block that already holds the pad's
  own two levers - it bisects the lever travel for the one that lands on the gate
  (on `Telemetry.collective_deg` itself, the mixing law's angle being monotone in
  the lever) and flies a frame's worth either side of it with the pedal at the
  stop for two seconds: 3.984 deg of rotor holding the heading to the last bit
  (walk under 1e-9 deg, yaw rate and body rate exactly 0.0) and 4.016 deg turning
  it 23 deg, both on the ground at the end and neither crashed; `python main.py
  --check` and `python regression_tm73254.py`, both unchanged, the eight figures
  still reading as they did before the gate; and `attic/_grip_probe.py` - scratch,
  not part of the project - walking the whole travel with the pedal held at the
  stop for two seconds at a lever: 0.000 to 0.250 of the travel (0.000 to 3.939
  deg of rotor) with no walk and no rate at all, 0.300 to 0.900 (4.727 to 14.181
  deg) turning 20 to 27 deg in those same two seconds with the skids down
  throughout, and 1.000 (15.756 deg) the one lever where the rotor lifts it off
  the ground, which is the floor handing the heading back.

* 2026-10-16 - **a run says why the grip flies nothing: `--engine`, and one line
  the first time `Q` or `E` is touched.**  The report was "set the throttle to
  minimum and the helicopter still goes into the air", and the answer is that on
  the aircraft a run begins on there is nothing for the grip to be a minimum *of*:
  the report's own 6158 lb aircraft carries no engine in the model, so its rotor
  is held at 100 per cent and `Q` rolls a lever with nothing on the other end of
  it.  Nothing was wrong with the grip - `attic/_idle_grip.py` flies both
  aircraft with the same stick and prints them side by side, +36.96 m either way
  with no engine and +1.07 m against +36.49 m with the T53 fitted - but a run
  gave no sign of it, so the most obviously broken looking control in the sandbox
  was the one that was not broken at all.

  * *`--engine` fits the T53 before the first frame.*  `command_line` takes the
    switch, strips every one of it, and answers a fourth value -
    `(scenery, log_path, controls_path, engine)` - and `main()` re-fits after
    `on_the_pad()` and before the window, so a run that means to fly the grip
    from the start says so once instead of pressing `F` on its tenth frame.
    There is no `--no-engine`: engine-off is what a run that says nothing gets,
    so the switch has one side and the default is the report's own aircraft,
    untouched;
  * *and a grip moved with no engine says so, once.*  `grip_notice(sim, was,
    now)` answers the line - None unless the grip moved and no engine is fitted
    to read it - and the frame loop reads `sim.throttle()` before the frame's own
    input and again after it, prints the first line it is given and never asks
    again.  One line and not one a frame, however long a key is held, and the line
    names both cures: `F`, and `--engine`.  It says nothing at all for a grip no
    hand touched, and nothing on the aircraft with a governor behind the grip, so
    the fitted engine - by `F` or by `--engine` - is never warned about.

  The keys line the window prints before it opens names all five controls now
  (`Q`/`E` grip, `F` engine, `G` governor, `X` failure, beside the four axes).
  Verified: `main.py --check` passing, with the new assertions written in the same
  style - the four values `command_line` answers and the switch itself, including
  that saying `--engine` twice leaves no flag behind to be mistaken for a scenery
  path, and `grip_notice`'s own three readings (a grip that did not move, a
  movement with no engine, and the same movement with the T53 fitted, which owes
  nothing); `attic/_qe_wire.py` - scratch, not part of the project - flying the
  loop's own reading across frames: silent with the hands off, one line when `Q`
  is held for a second and still that same one line after four seconds more at
  the stop, and none at all for a run that began with `--engine`.

* 2026-10-16 - **the twist grip is a mapped control, and the panel shows it: Q
  and E roll the grip the window is flying.**  Two things were wrong with the
  fifth control and each hid the other:

  * *the map did not name it.*  `default_controls.xml` named the four axes and
    left the throttle out - its note said as much, and which name to use when a
    device arrived - so `controls.default_control_map()` had no throttle in it,
    `AXIS_FOR_CONTROL` had no keyword for the name, and `Devices.read` dropped
    the control on the floor: a `Q` or an `E` held in the window reached
    `pilot_keys`, which nothing in the frame loop calls.  The map is the
    keyboard's five controls now (`DEFAULT_KEYS`, `("e", "q")` for the grip), the
    keyword is `throttle` - which `PilotInput.step` and `set_axes` both take as
    their own argument - and `read` answers `(absolute, ratchet)`, the two shapes
    `PilotInput` already took, with the grip in whichever of them its device
    makes it: ratcheted, on the keyboard, and absolute on a lever a device
    reports the position of.  The frame loop needed no case for the one control
    that is not a stick, and the third value the throttle used to be carried out
    in - for nothing, since `fly_frame_axes` never wrote it - is gone;
  * *the panel had nothing to show it with.*  The collective's panel carries a
    205 deg pie now: a hundred degrees of yellow below its own horizon and a
    hundred and five of green above it, split at that horizon, with a red hand on
    the grip's own angle whose tail runs through the middle and out of the far
    side.  It is drawn from `sim.throttle()` beside the lever's own field, so the
    one control the aircraft cannot show is on the panel a pilot would look at.

  The grip begins where it has always begun, full open, so `E` on a fresh run
  moves nothing - it is already on its open stop - and `Q` is the key that shows
  the pie working.
  Verified: `main.py --check` passing, with the shipped map and `pilot_keys` read
  as one another key by key for all ten and by flight - two aircraft flown
  through `fly_frame` and through `fly_frame_axes` with `W` and `Q` held hold the
  same grip at the end of them, 0.5 of its travel after two seconds of a key -
  and with the grip arriving as a *position* from a stand-in lever that reports
  one; `python controls.py` passing, the shipped file round-tripping byte for
  byte with its five controls and its ten keys listed and rewritable; and
  `attic/_qe_wire.py` - scratch, not part of the project - flying the window's
  own loop headless (the map, the two dictionaries, `fly_frame_axes`) with the
  grip watched out of `sim.throttle()`: full open on `105.0 deg`, a second of `Q`
  giving grip 0.75 and a hand at `53.8 deg`, `Q` on to the closed stop and `E`
  back to the open one, a tap worth one frame of a key (`0.0042` of the travel),
  both keys at once moving nothing, and with a T53 fitted `Q` leaving the
  governor at exactly the grip the window rolled.

* 2026-10-14 - **the windmill root is read off the shaft, so the engine failure
  the sandbox already had is flyable as an autorotation.**  `Airframe.windmill`
  was a caller's switch and nothing in the project ever threw it - `simulation.py`
  and `main.py` mentioned the word only in prose - so the one thing the failure
  was *for* was out of reach: a rotor with nothing on its shaft taking its torque
  from the air going up through the disc.  It is now a reading of the shaft:
  true when a governor has failed, when the twist grip is rolled right off (which
  `Governor.power_target` reads as the same physics), or when a fixed
  `engine_torque` is zero; false on the report's own aircraft, which has no shaft
  for anything to be driving.  The failure is the whole of the switch and a reset
  or a grip rolled back open is the whole of the way back, and
  `trim_autorotation`'s refusal is that same physical condition read the other way
  round - so an airframe whose engine has just quit trims an autorotation with
  nothing else done to it, which is the path the sandbox's X key takes.

  `Simulation.trim_autorotation` is the third of the frame loop's trims, built
  like the other two: it solves the entry at the airspeed asked for (the report's
  60 kt by default), at the altitude the aircraft is over, at the rotor speed the
  rotor is *doing*, and leaves the run flying it - the reference `in_trim` reads,
  and where a bare `R` comes back to.  So the path is the sandbox's own switches,
  `F` for a T53 and `X` for the failure and then the trim, and what comes back is
  the flight manual's entry: collective 6.26 in, 9.11 m/s down, 1792 fpm, and a
  rotor that holds 324.0 rpm for the ten seconds it was flown - because a rotor in
  the windmill brake state carrying the aircraft is a flight and not a runaway.

  One behaviour moved with it, and it is an improvement rather than a cost.  A
  dead shaft whose working state runs out now *finds* equation 10's second root
  where the old reading returned an unconverged solve for a caller to notice by
  itself: at the hover, with the collective left up, that is the windmill branch at
  203 rpm, and the rotor is then driven back up - the instability the branch's
  docstring describes, and the frame `Simulation` refuses.  The airframe self test
  that asserted a settled 0.50 to 0.65 of 100 per cent there was in fact riding on
  that unconverged solve, which nothing asserted; it now pins the handover - the
  wind down to 0.65, the second root found, and the rotor turned round - instead.

  Verified: `python airframe.py`, `python simulation.py` and `python engine.py`
  printing `self test passed` with the new assertions, and `python main.py
  --check` printing `main.py check passed`; `regression_tm73254.py` printing
  `self test passed` with all eight figures and every printed magnitude unchanged,
  the report's own aircraft having no shaft and so still never seeing the second
  root; `simulation.py`'s demo showing the failure trimmed at 60 kt - rotor
  100.0 %, -1792 fpm, 62.6 kt, flying for ten seconds - where the same failure
  used to reach the floor; and `airframe.py`'s demo printing the same 9.11 m/s,
  1792 fpm trim it always did, now with nothing told to it but the dead shaft.

* 2026-10-14 - **the ground is a plane: a level and a normal, with equation 10a
  measured over it.**  Ground effect has been in the model from the start - the
  report's equation 10a, `K_G = 1 - exp(-(Z/D)/G1)` - but Z was the hub's height
  above a horizontal plain at the NED origin, and the ground was a *coordinate*
  rather than a thing: `simulation.py`'s floor placed the c.g. at
  `z = -UH1_CG_HEIGHT_ON_GROUND`, levelled roll and pitch to the world's own
  zero, and measured a hard arrival down the world's own down.  Nothing in the
  report says a surface has to be flat-footed - the cushion and the skids both
  want the distance to whatever the aircraft is standing over - so the ground is
  now `airframe.GroundPlane`: one flat patch of it, a point and a unit normal,
  with the four questions a caller asks it (`height`, `surface_point`, `above`
  and `level_attitude`) and the `level` shortcut for the horizontal case.  A
  normal that is not a unit vector is normalised rather than refused and a zero
  one is refused, since it describes no surface at all.

  `Airframe.ground` is a field, so a run replaces it between frames;
  `Airframe.hub_height` is equation 10a's Z - the hub's perpendicular distance
  from that plane, the hub and not the c.g., which is why
  `FlightState.hub_position` now exists beside the flat-ground `rotor_height` the
  self test still pins it to - and all three trims place the aircraft they solve
  over `ground.above(Vector3(), altitude)`, so a trim over a ramp is solved over
  the ramp.  In `simulation.py` the same plane answers everything:
  `Simulation.ground`, the floor's skid line (`above(..., UH1_CG_HEIGHT_ON_GROUND)`),
  the point of the surface the skids are pinned by, the descent rate a hard
  arrival is measured at (along the normal, not down the world), the attitude the
  levelling goes to (`level_attitude`, which lays the aircraft's own down axis
  along the normal, so a parked aircraft on a slope is level *on the slope*), and
  `Telemetry.alt_agl` with `Telemetry.height_rate`.

  The default is the horizontal plain at the origin and every number that was
  there before is exactly where it was: `height` is `-position.z` to the last bit
  for such a plane, the floor's `above()` is `-UH1_CG_HEIGHT_ON_GROUND` exactly,
  `level_attitude` hands back positive zeros rather than whatever signs `atan2`
  would, and `FlightState.rotor_height` stays the flat case it always was.
  Verified: `regression_tm73254.py` byte-identical to the previous commit's own
  output and `simulation.py`'s demo likewise (103 lines), by running both trees'
  copies and diffing; `_rt_check.py`'s three scripted missions passing with its
  own transcript byte-identical; `aerodynamics.py`, `rotor_control.py`,
  `engine.py`, `airframe.py`, `controls.py` and `scenery.py` all printing
  `self test passed`; and `main.py --check`.  New in the two self tests: the
  plane's geometry, normalisation and refusals; a ramp's perpendicular heights
  against `cos 30` and `level_attitude` against the direction cosine matrix's own
  third column, at three headings; a parked aircraft levelling to a 15 deg slope
  at whatever heading the skids let the tail rotor drift it to; the cushion a
  parked aircraft sits in equal on level and leaning ground to 1e-7, because the
  floor holds it the same height over the surface; and the same NED position a
  smaller clearance over a leaning ground than over a level one.

* 2026-10-13 - **the autorotation trim, on the descent rate as the unknown, and
  the windmill root it needs.**  `trim_autorotation` is the third of the trim
  family and the one the engine's failure path left open: with nothing on the
  shaft the rotor's own balance is `Q = 0`, and the one thing that closes it is
  how fast the aircraft is coming down.  Its seven unknowns are the four sticks,
  the pitch and roll attitudes and the *descent rate* in m/s; its seven residuals
  are the six of `equilibrium_residual` and the shaft's own seventh, which is the
  place `trim_hover` puts its solved-for rotor speed.  The rotor speed cannot
  also be an unknown - the two together against the seven equations would be one
  short - so it is a named `rotor_speed`, 324 rpm by default, and the descent
  rate is what comes back.

  Asked for the report's own 60 kt condition the answer is the flight manual's
  own: collective 6.26 in - nearly four inches below the 10.24 in hover - pitch
  4.8 deg nose up, and 9.11 m/s down, 1792 fpm, against the 1800 to 2000 a
  UH-1H autorotates at there.  Both halves are closed to 1e-7 N and N m, the
  rotor is on the second root of equation 10 (a positive inflow ratio, air up
  through the disc) with its torque at nothing, and flown from that state for
  five seconds of frames the aircraft falls the 45.5 m the trim was drawn for
  with the rotor still at 324.00 rpm - a trim that flies, not merely a solution.

  The branch had to be reached, and that is where the previous level's gate was
  too narrow.  It offered the windmill root only to a descending rotor whose
  collective was down (`thrust_at(0) <= 0`) - the collective-down entry the
  simulation flies a dead engine into - and an *autorotating* rotor carries the
  aircraft's weight while it turns, so its collective is up and that test never
  offered it the state it flies in.  Widening the gate to every descending rotor
  the equation has a root for turns out to be wrong the other way: a hovering
  rotor whose collective is left up and whose speed is run down then finds the
  root and overspeeds without bound, because the model has no blade stall to stop
  it.  So it is not the default.  It is `Airframe.windmill`, off by default and
  threaded through `body_forces`, `main_rotor_forces` and `_solve_inflow_ratio`,
  and `trim_autorotation` raises unless the aircraft it is asked of has it: the
  report's own aircraft keeps every figure in TM-73254 to the last bit, and only
  an airframe being flown - or trimmed - as an autorotation offers the rotor that
  root.

  Verified: `python airframe.py` printing `self test passed` with the trim, its
  residuals and the five-second flight asserted; and `python simulation.py`,
  `python regression_tm73254.py` (314 lines, the report's figures 2 to 9
  unchanged to the byte, because the default aircraft never sees the new root),
  `python engine.py`, `python rotor_control.py`, `python aerodynamics.py`,
  `python scenery.py`, `python controls.py` and `main.py --check` all passing.

* 2026-10-08 - **the sandbox flies the engine: fitted live with F, and flown from
  the keyboard.**  `main.py`'s aircraft was the report's own - no engine, the
  rotor held at 100 per cent - which is what TM-73254 needs, but it left the
  twist grip, the GOV AUTO/EMER switch and an engine failure as things only a
  script could set, and the caption with no gauges on it at all.  The choice made
  here was the keyboard's, not the command line's: a run still *begins* on the
  report's aircraft, and F fits the T53 to the shaft it is flying, in the air.

  `Simulation.fit_engine(fitted=True, engine=None)` is how it does it, and it is
  a re-fit and not a reset: **no bit of the aircraft's state moves** - not the
  position, the velocity, the attitudes, the rates or the rotor speed - and the
  run's own start is left where it is, so a later R still comes back to the pad.
  What moves is the *trim reference* `in_trim` compares against, because a driven
  shaft has a hover of its own: the reference is solved again at the condition it
  already describes - a hover, or the same airspeed when the run's reference is a
  level trim, which it knows by its own speed - and comes back at 323.27 rpm on
  the 6158 lb aircraft where the report's is 324 (322.98 on the 8700 lb one the
  default simulation flies).  The engine is settled on the torque the rotor is
  *absorbing* at the state the aircraft is in, which is what `Airframe.reset`
  does for a run that begins with one, so the balance starts closed and the
  governor takes the rotor from where it is rather than spooling up behind a
  sagging one.  `fitted=False` is its exact inverse, rotor speed included:
  taking an engine *away* is a switch, and a rotor that winds down because the
  fuel went away is what the failure key and a closed grip are.

  `main.py`: **F** fits that engine or takes it off, **G** is the GOV AUTO/EMER
  switch (TM 55-1520-210-10, 9-3) and **X** is an engine failure and then the fuel
  coming back, each logged by the existing key log with a kind of its own
  (`fit`/`unfit`, `governor`, `failure`/`recovered`) and each saying so rather
  than pretending when there is no engine to do it to.  The twist grip, which had
  been carried and shown and reachable by no key at all, is pilot_keys' fifth
  position now: **Q** rolls it off and **E** rolls it on, so the flight keys are
  ten where they were eight (`FLIGHT_SCANCODES`, `KEY_NAMES`, and the count
  `--check` asserts).  The caption gains the engine's panel - the rotor speed
  where it is no longer exactly 100 per cent, N2, the torque gauge and the switch,
  with `FAILED` for an engine that has quit - *only* when there is one to read,
  so the aircraft of TM-73254's figures is told apart from the T53 by the gauges
  and by nothing else.

  What was left was named where it lives rather than left to be found: the
  caption's torque gauge was not read against the 50 psi transmission limit, and
  it is now - `main.py` prints the reading and the limit in the manual's own psi
  (top of this list, 2026-10-22).  (The device map named no throttle until
  2026-10-16 and the note in `default_controls.xml` said which control name to
  use when one arrived; the map names it on `Q` and `E` now - see `Done`.)  See
  `Next`, which is the chart alone now.

  Verified: `main.py --check` with the new assertions written in the same style -
  the grip as pilot_keys' fifth control and not one of the four axes, Q and E
  turning that ratchet by exactly a key's worth and E putting it back on its own
  stop, F, G and X as command keys that fly nothing on their own, the fitted
  engine moving no bit of the pad's state, the caption gaining
  ``| rotor 100.0 % | N2 ... | Q ... | AUTO`` and reading EMER and FAILED as the
  switches are thrown, and F again putting the reference back on the 324 rpm it
  started from to the last bit; the seven module self tests, with
  `simulation.py`'s new block asserting that fitting moves no bit of the state
  *or* of the run's start, that a hovering run fitted live settles onto the
  governor's own 323.27 with the report's stick positions still in the pilot's
  hand - half a metre low five seconds later, which is the 0.025 in of collective
  nobody has moved yet - that the two switches are exact inverses, and that a run
  with no trim reference still fits one; and the gate, re-run in full:
  `regression_tm73254.py`'s 314 lines and `_rt_check.py`'s two missions both bit
  for bit as they were, 0 different, which is the check that fitting an engine
  changed nothing at all for the aircraft that has none.

* 2026-10-08 - **a trim that solves for the rotor speed, which is the hover the
  engine is really in.**  The two trims took an omega to solve *at* and left the
  shaft balance open, so a "trimmed" hover on the 8700 lb aircraft with a T53 on
  it began at 324 rpm with the governor holding nothing in particular and the
  rotor walked down to 322.98 over the first seconds: a hover that sinks towards
  its own trim rather than one that is in it.

  `Engine.settled_torque(rotor_rpm, air_density)` is the engine's half of that
  balance - ``P / omega`` of what the governor's droop curve is asking for at the
  speed the rotor is doing, a *function* of that speed rather than the last
  frame's lagged power, because the gas producer's lag has no meaning in a steady
  condition.  The self test asserts it is where the loop arrives (an engine
  settled on it delivers exactly it) and that it is `max_torque` once the whole
  droop band is used up.  `Airframe.shaft_torque(rotor_speed)` asks the same
  question of the airframe: the engine's settled torque, or a fixed
  `engine_torque`, and a `ValueError` on the report's own aircraft - whose engine
  holds whatever the rotor is doing, so there is no shaft balance to close and
  asking for one is an error rather than an invented number.
  `Airframe.shaft_driven` is that predicate, and `Airframe.shaft_residual(controls,
  state)` is ``Q_shaft - Q_rotor``: the seventh residual, parallel to
  `equilibrium_residual`'s six and the same quantity `rotor_acceleration` divides
  by the inertia.

  `trim_hover` and `trim_level_flight` take `rotor_speed=None` as the sentinel
  for "the rotor speed is an unknown too", and the seventh residual is the shaft
  balance - so the trim comes back at the equilibrium of the engine *and* the
  airframe, which no trim at a named rotor speed is, and `torque_tolerance` is
  that residual's own allowance in newton metres beside `tolerance`'s newtons.
  Table 3 is rebuilt at every trial, which is the price of the seventh unknown and
  the reason the other six are still solved with one set of constants.  The solve
  needs two things the other six did not: the Jacobian's rotor speed column takes
  a 1e-3 rad/s step and a 2 rad/s clamp - a rotor speed is a large number whose
  residual is a torque in the hundreds of kilonewton metres, and half the droop
  band is as far as one step should jump on a state that is a band rather than an
  attitude - and it is differenced *downwards*.  That last one is not a detail:
  a governor's droop curve has a kink at the speed it is asking for, no torque at
  or above it and an increasing slope below, a trim starts *exactly* on that kink,
  and a forward difference there reads the flat side.  Measured before it was
  fixed: the first correction came back asking for -72 rad/s, the clamp cut it to
  -2, and the loop ping-ponged between two clamped corners for all forty
  iterations with the shaft 8104 N m out.  The low side is the right side for
  every rotor, because an idling free turbine is not a brake and there is nothing
  to balance a rotor running fast.

  What it flies: the governed hover comes back at 322.9752 rpm with the shaft
  closed to 0.006 N m of its 16267.8 - 0.006 rpm off the 322.9816 the frame loop
  integrates to, and inside the band the manual rigs, 40 rpm of N2 over the 20.37
  gearing being 1.96 rpm of rotor.  Level flight at the report's 60 kt comes back
  at 323.34: less power, so less droop, and still a whole rpm under the
  nameplate.  A *fixed* `engine_torque` of that same 324 rpm trim's drag sustains
  a hover at 323.9957 rpm - 0.004 under the reference and not at it, because the
  drag is a curve and not a constant - and no torque above it has a hover near the
  reference at all, the rotor not being able to absorb one at that thrust without
  the wind up `surge` already shows.  `Simulation.trim` and `trim_level_flight`
  pass the sentinel when the shaft is driven and the nameplate otherwise, so a
  powered run now begins in its own hover with no settling transient in it: five
  seconds of frames off the trim move neither the altitude nor the rotor speed,
  where the same aircraft trimmed at 324 rpm is still walking down.

  Verified: all seven module self tests and `main.py --check` passing, with the
  new assertions written in the same style - the six residuals and the seventh all
  closed at the solved hover, the solved rotor speed against the loop's own
  settled 322.98, a fixed torque against the drag it was handed, both kinds of
  shaft flown for five seconds from the trim they solved, the report's own
  aircraft raising for a shaft torque and for a rotor speed trim, and the airframe
  that is *told* a torque balancing it to the last bit.  Four broken copies were
  flown against those assertions: the settled torque read as the lagged power
  instead (caught - the shaft cannot close at all before a frame has been flown),
  the seventh residual's sign reversed (caught by its documented sign, though a
  consistently negated row is the same equation and the same root), a convergence
  test that ignores the shaft (not caught, and recorded here as a finding: at
  these conditions the shaft row comes inside its allowance with the six, so
  `torque_tolerance` is a stated contract rather than a knob that bites), and a
  simulation that trims its driven airframe at the nameplate after all (caught:
  324 rather than 322.98).  And the gate itself, re-run in full on the report's
  own aircraft, which has no engine and must not have moved: `regression_tm73254.py`
  printing all 314 lines of its two trims and its eight figures 2 to 9 bit for bit
  as they were before this, and `_rt_check.py`'s two pilots flying their missions
  through `main.fly_frame` to a transcript bit for bit identical to the one before
  it - 304 lines, 0 different.

* 2026-10-08 - **the rotor's azimuth is an integral of the rotor speed, so the
  blades a renderer draws turn with the rotor the aircraft actually has.**  The
  last thing of the engine's level: `main.py`'s `rotor_azimuth_deg` was the
  constant clock `sim_time * UH1_RPM * 6`, which is right only while the rotor
  speed is held at exactly 100 per cent - so with a T53 aboard, and a governor
  that settles at 322.97 rpm (322.9964 of it after the first two seconds), the
  visible blades strobed 12.18 deg off the rotor they were meant to be drawn on,
  and further off for every second after that.

  The clock is the rotor's own now.  `Simulation.rotor_azimuth_deg` is a new
  field, in degrees, advanced in `_advance` by
  `degrees(0.5 * (omega started + omega ended) * dt)` - the trapezoid a rotor
  speeding up or slowing down wants, and exactly omega * dt when it is not - and
  advanced *after* the envelope check, so a refused step, which flew no aircraft,
  turns no blades.  `reset` clears it with the rest of the counter block.  It is
  a clock and not a state: nothing in the model reads it, because the rotor's own
  loads come out of a harmonic balance over a revolution and not out of where the
  blades happen to be - which is why the self test can zero it every frame and
  fly a flight bit-identical to the one that kept it.  And it is kept
  *unwrapped*: ten turns of a blade and a bit over read as 3888 deg and not as
  288, because the tail rotor's 5.56 : 1 is not an integer.  `main.rotor_azimuth_deg`
  is `fmod(sim.rotor_azimuth_deg * ratio, 360.0)` - the one place the clock
  becomes an angle - and that wrap is deliberately there rather than in the
  model: the main rotor's own remainder is 288 where the clock is 3888, and 5.56
  times 288 is 161.28 deg, which is not where the tail rotor is, while 5.56 times
  3888 leaves the 17.28 it is at.  The two are 144 deg apart for no better reason
  than that 5.56 is not an integer, which is the whole of why the clock has to
  count revolutions rather than degrees.

  `main.py`'s `--check` keeps its 288 and 17.28 assertions untouched - a default
  aircraft has no engine and no engine torque, so its shaft is still exactly
  `UH1_RPM` and two seconds is still 10.8 revolutions - and gains the properties
  behind them: that the clock is 3888 deg and the shaft is at `UH1_RPM` to the
  last bit, that the tail's angle is the remainder of `ratio` times that 3888,
  and that a wrap applied first would have given the other number.

  Verified: `python simulation.py`, `python airframe.py`, `python engine.py`,
  `python rotor_control.py`, `python aerodynamics.py`, `python controls.py`,
  `python scenery.py` and `main.py --check` all passing, with the new assertions
  written in the same style - that two seconds of the fixed rotor is 3888 deg of
  clock and 288 of angle and one frame is 32.4 of it, that half a rotor speed is
  half the azimuth (144 deg) and that a rotor dragging down under no torque at
  all is its own trapezoid rather than its last sample, that a reset clears the
  azimuth and a refused step leaves it while the clock goes on, that a governed
  rotor is 12.18 deg behind the nameplate after the same two seconds, and that
  zeroing the azimuth every frame changes no bit of the flight.  Three broken
  copies were flown against those assertions and all three were caught: the clock
  driven by the start of the frame's rotor speed alone (by the coasting
  trapezoid), the wrap forgotten (by the range check) and the wrap applied before
  the tail ratio (by the 17.28).  And the gate itself, re-run in full for the
  first time since the engine landed: `regression_tm73254.py`'s `_self_test`
  passing and its demo printing the two trims and all eight of figures 2 to 9
  exactly as its own transcript records them, and `_rt_check.py`'s two pilots
  flying their missions through `main.fly_frame` to a transcript bit for bit
  identical to the one the engine's entry was verified against - 304 lines, 0
  different.

* 2026-10-08 - **the engine: a T53 with its governor, so the torque comes from
  somewhere.**  The level before this one made the rotor speed a state and left
  `Q_engine` to a caller; `engine.py` is that caller.  It is a new module between
  `aerodynamics.py` and `airframe.py`, standard library only, and every number in
  it is TM 55-1520-210-10's own: 324 rotor / 6600 engine rpm (and the manual's
  other pair, 314 / 6400, which gives 20.382 against 20.370 - the gearing read
  twice), the 1125 ft-lb data plate calibration, which is 1525 N m and 1.054 MW
  at 6600 rpm, i.e. 1414 shp against the T53-L-13's 1400, the 50 psi
  transmission structural limit, the droop compensator's +-40 rpm band and the
  manual's own *definition* of droop, the 6 to 8 psi the GOV AUTO/EMER switch
  costs (7 taken), the 1.4, 2.1 and 3.5 psi of bleed air, and the twist grip's
  three positions - full open selects a speed, rolled off selects a lower one,
  fully closed shuts the fuel off.

  Two things are this project's own and are named as such: the *slope* between
  psi and newton metres (the charts, figure 7.1-2, are where the manual keeps it;
  here the 50 psi limit is taken as the gauge's full scale, 621.4 N m of rotor
  shaft torque per psi) and the gas producer's 0.6 sec lag.  With those,
  `Governor.power_available` scales with air density and gives back the manual's
  own warning that the engine can exceed the transmission limit in dense air;
  `Governor.gain` is the power available over the droop band, so full power costs
  the whole 40 rpm and no load costs none; `Engine.advance` is one first order
  step of the lag per frame, and `Engine.torque` is the free turbine's P / omega,
  which is what makes a slow rotor take more torque from the same gas and helps
  it catch a wind down.

  Wired in: `Airframe(engine=Engine())` asks the engine for a torque once a frame
  from the rotor speed at the start of it and writes it into the same
  `engine_torque` field a fixed torque goes into, so a T53 and a number are told
  apart by nothing but where the number came from; `Airframe.throttle` and
  `Airframe.reset` (which settles the engine on the torque the rotor is actually
  absorbing, so a run starts flying rather than spooling up); `PilotInput`'s
  fifth ratchet, the twist grip, with `step`, `set_axes`, `throttle()`,
  `THROTTLE_STEP_GRANULARITY` and `key_rates`; `Simulation.throttle`,
  `set_throttle` and `set_governor(emer=, failed=, target_rpm=)`;
  `airframe_preset(..., engine=True)`; and `Telemetry.engine`, an
  `EngineTelemetry` nested record - N2, throttle, power, torque, the torque gauge
  as a percentage of the data plate, and what fraction of what the air can give
  is being asked for - with honest zeros and `has_engine=False` on the report's
  engine-less aircraft.  `Telemetry.__str__` is deliberately *not* changed, so
  every transcript in the repository still reads exactly as it did.

  What it flies: on the 8700 lb aircraft the governor settles at 322.97 rpm of
  rotor, 0.32 per cent under the reference and inside its own band, asking 552 kW
  (52 per cent of what there is, 26 psi on the gauge) - asserted against the same
  closed form the loop integrates, so the droop curve cannot drift away from the
  physics.  An engine failure is a wind down through the lag, ten time constants
  to nothing; EMER is the grip as the fuel control at 43/50 of what there is,
  with nothing chasing the rotor; and a closed grip is no fuel at all without
  being a failure.  Verified: `python engine.py`, `python airframe.py`,
  `python simulation.py` and `main.py --check` all passing with the new physics
  asserted rather than printed; and the model's *default* aircraft - no engine,
  the report's fixed rotor - bit for bit identical to a tree built from the
  commit before this one, flying the same 1200 frame scripted profile and the
  same 600 frame flight test profile in wind, with the trajectory and the
  control lag hashes equal to the byte and `_rt_check.py`'s two missions
  unchanged as well.

* 2026-10-08 - **the airframe's rotor speed is a state, and TM-73254 table 3's
  constants are parametric in it.**  The model computed the torque that opposes
  the shaft and then threw the balance away, so autorotation, rotor droop and the
  needle split were not in it.  Two thirds of that is now fixed, at the levels
  that need no engine to fly.

  `airframe.rotor_constants(omega)` returns the whole of table 3 at any rotor
  speed as a frozen `RotorConstants`, by scaling the transcribed numbers by the
  closed forms they are: R1 and R9 as omega squared, R2, R4, R6, R7 and the tail
  rotor's T1 as 1 / omega, R8 as 1 / omega squared, R3 and R5 not at all, and
  `tip_speed` as omega R.  At the 324 rpm reference it *is* the table, to the last
  bit, so nothing that was flown before this moved; the self test asserts that
  identity and the homogeneity separately, and the closed forms at 100 per cent are
  still checked against `aerodynamics`' rotor geometry where they always were.
  `main_rotor_forces`, `tail_rotor_thrust`, `body_forces` and `_solve_inflow_ratio`
  take it as an `omega` that defaults to the reference or as a `constants` set
  built once, so a frame still costs four closed forms and a trim still costs one
  per residual.  The tail rotor follows the main through table 2's fixed 5.56 : 1
  drive ratio, which is exactly what its T1 as 1 / omega is.

  `FlightState.rotor_speed` is the thirteenth state variable, in rad/s, defaulting
  to the reference: `values()`, `from_values()`, `copy()` and the Runge-Kutta step
  carry it (the step's four `range(12)`s are now `len(initial)`), and
  `FlightState.rotor_rpm` is the gauge's own unit.  A state that never mentions it
  is the fixed rotor of the report, and every assertion that existed before this
  still passes untouched - which is what made the change safe to land in one go.

  `Airframe.rotor_inertia` and `Airframe.engine_torque` close the shaft:
  `derivatives` gains `d omega / dt = (Q_engine - Q) / I`, with Q the report's
  equation 4, and `engine_torque = None` - the default - means the engine holds
  whatever the rotor is doing, so the derivative is exactly zero and the model is
  TM-73254's fixed rotor to the last bit.  The polar inertia neither report has is
  derived rather than mined: the teetering hinge sits on the shaft, so a blade's
  flap inertia about it is its polar inertia about the shaft, and
  `rotor_control`'s own 1658 kg m^2 a blade - asserted there against a 92 kg blade
  - makes the pair 3315, which the self test checks against that rotor and against
  the 6.55 lock number behind table 3's 0.072 sec R6.  `trim_hover` and
  `trim_level_flight` take a `rotor_speed` to solve at, which is what lets the
  self test fly a hover trimmed on a 90 per cent rotor - that one needs the 6158 lb
  aircraft, the 8700 lb one's stick already sitting at 93 per cent of its travel.
  `simulation.Telemetry` carries `rotor_speed`, `rotor_rpm` and `rotor_percent`,
  and prints the percentage on every telemetry line, beside the collective it is
  the counterpart of.

  Three things are deliberately left for the level after this: the engine and the
  governor that would make the torque come from somewhere (`Next` above), the
  rotor speed in `main.py`'s rotor azimuth clock and its `--check` assertion, and a
  trim that solves for the rotor speed instead of taking one.  Verified:
  `python airframe.py` passing, with the new assertions written in the same style
  and the new physics asserted rather than printed - that a fixed collective's
  thrust goes as omega squared while its inflow ratio does not move at all, that a
  torque cut at the trimmed drag winds the rotor down and settles it while twice
  that drag winds it up, that a fixed rotor's derivative is zero and the trim's own
  balance is exactly closed, and that the 13th variable round trips; `python
  simulation.py` and `main.py --check` passing unchanged; and
  `regression_tm73254.py` bit for bit identical to a worktree at the commit before
  this one, all 313 lines of figures 2 to 9.

* 2026-10-08 - **which physical control flies which is a file, so the aircraft
  can be flown from a keyboard, a joystick, or a mixture of the two.**  The
  sandbox read eight keys and nothing else, and the next thing this project
  needs is home made hardware - an Arduino Leonardo sent as a generic USB HID
  game controller - arriving one axis at a time.  `controls.py` is the map: an
  XML file in the style of `scenery.py` (stdlib `ElementTree`, unknown
  attributes refused, a round trip and a self test of its own), naming the
  devices and then each of the five controls - `cyclic-long`, `cyclic-lat`,
  `pedals`, `collective`, `throttle` - on one of them.  A control on the
  keyboard is a ratchet and names two keys; one on a joystick is either
  absolute (an `axis`, or a `hat`), read straight through at the hand's own
  rate, or a ratchet turned by two buttons - and both take `invert`, `deadzone`
  and a raw `minimum`/`maximum` for a stick that reads backwards, jitters about
  its centre, or does not reach its own stops.  A device is found by `match`, a
  piece of its name, or by `index`, and one that is not plugged in reads as no
  input at all rather than as a fault.  `default_controls.xml` ships the mapping
  the sandbox already had, so a run that says nothing about devices flies
  exactly as before, and a control a file leaves out keeps that default: the
  switching is per control, which is the whole point.
  `main.py` gained the reader: `SCANCODES` and `key_scancode` resolve a key
  *name* through pygame's own `KSCAN_*` constants, so the map keeps the
  layout-independent scancode the eight keys were always read by (pygame 2.6.1
  has no `get_scancode_from_key`, and `pygame.key.name` answers keycodes rather
  than scancodes); `Devices` holds the map, resolves its keys once, and reads a
  frame as `(absolute, ratchet, throttle)` - the two shapes `PilotInput` already
  took, since `set_axes` and `step` each touch only the axes they are given and
  a control is one or the other (that third value went on 2026-10-16, when the
  grip became a mapped control like the other four); `fly_frame_axes` puts the
  positions and turns the ratchets; and `fly_frame` is untouched, so the scripted
  pilot and `--check` keep the entry point they had.  `--controls FILE` and
  `--no-controls` join `--log`/`--no-log` on the command line, joysticks are
  re-found on `JOYDEVICEADDED`/`JOYDEVICEREMOVED` while the window is open, and
  the `throttle` is carried and printed beside the telemetry while flying
  nothing: the model still holds 100 per cent rotor speed.  (The map this project
  shipped named no throttle until 2026-10-16; see that entry.)
  Verified: `python controls.py` passing - the shipped file round-tripping byte
  for byte, a map naming one control leaving the rest to the keyboard, and 23
  descriptions written to be refused all refused; `main.py --check` passing,
  including that the shipped map read one key at a time returns exactly
  `pilot_keys(frame_keys(...))`, that 120 frames through `fly_frame` and through
  `fly_frame_axes` leave bit-identical aircraft, and a stand-in joystick driving
  absolute axes - inverted, deadzoned, clipped and calibrated - a button ratchet
  and a hat with no hardware present; `py_compile` clean on all eight modules;
  and the scripted pilot's imports still binding (`fly_frame`'s five arguments,
  `KEY_NAMES`, `FLIGHT_SCANCODES`).
* 2026-10-01 - **every keystroke is logged, with the flight it was made in.**  The
  caption and the panels show one instant and forget it, so a flight that went
  wrong was gone by the time anybody read it.  `main.py` now writes
  `keylog.txt` beside itself (`--log FILE` puts it elsewhere, `--no-log` flies
  without one) - one line per key as the event loop takes it out of pygame's
  queue, and `KEYLOG_RATE_HZ` (20) lines a second of the flight between them, so
  that what was pressed and what the aircraft did about it are in one file.  Every
  line carries the aircraft's own telemetry and the hand: airspeed and the ground
  velocity north, east and down, heading, pitch, roll and altitude, and all four
  control positions in inches of travel with the collective as a fraction of its
  own.  Each is stamped twice and to a millisecond - `t_s`, monotonic
  `time.perf_counter` seconds since the log was opened, and `wall_ms`, the same
  instant as `time.time` - so a key can be read against the flight around it or
  against the time of day.  The kinds of line are `down`/`up` (any key, including
  one the four axes do not read), `reset`/`park`/`view`/`quit` for the command
  keys, logged once their own state is in, `focus`/`blur` for the keyboard coming
  and going, and `sample` for the flight.  `flight_columns`, `KeyLog`, `log_key`,
  `key_name`, `held_names` and `command_line` are the whole of it, and the file is
  line buffered so a crash costs only the line it interrupted.  Verified:
  `main.py --check` passing, with the header row, the columns, both clocks to a
  millisecond, the two clocks checked to be one instant, and the 20 Hz rate
  holding a second sample back - and a 240 frame headless flight logged through
  `KeyLog.open` and read back: 67 samples at a 51 ms minimum gap, the `down` line
  reading `W` in `held` and the `up` line `-`, and the pickup from the pad on the
  trace (1.40 m to 19.59 m, the lever 0.75 to 1.00 of its travel).
  `attic/_keylog_check.py` is the scratch for it, beside `_attitude_hud.py`: it
  writes a real log to disk, reads the rows back, checks the clocks, the rate and
  the trace, prints them, and deletes the file it made.

* 2026-10-01 - **the attitude indicator's lines are one pixel, and its ball has the
  picture's own colours.**  The fourth panel was drawn in one white - the same
  white for the horizon, the ladder and the aircraft's reference - over a light
  grey sky and a nearly black ground, with every line three or four pixels thick.
  That is the one arrangement a moving horizon cannot afford: at or near level the
  reference's bars lie along the horizon, so a thick white line and a thick white
  reference cross in the very middle of the dial and neither says which one it is -
  and a ladder whose rungs are 10 deg apart is four pixels thick in a 132 px panel
  whose whole scale is 25 deg, which is a panel to look *at* rather than read.  The
  sky is blue and the ground brown now, so the ball says which way up it is by
  colour alone; the ball's own lines - the horizon and the rungs of its ladder
  above and below it - are green; and the aircraft's own reference is the one white
  thing on the dial, drawn after the ball's own lines rather than before them, so
  the bars and the W are what is there wherever a green line runs under them.
  Every line is one pixel, and it is written as a pixel - `ATTITUDE_LINE_PX = 1.0`
  over `PANEL_PX` - rather than as a fraction of the panel, so a line of the dial
  stays one pixel whatever `PANEL_PX` becomes and the reference picture's own thin
  lines are what the dial is drawn at.  The new `attitude_point` is what that needs
  at the raster: it is `panel_point` with a dial's own half pixel added, because a
  one pixel wide quad centred on the *corner* of a pixel straddles two rows rather
  than covering the one row the projection puts it on, and the numbers here are
  such that the middle of the dial is exactly one - the other three panels' shapes
  are several pixels thick everywhere and go on using `panel_point` unchanged.
  Verified: `main.py --check` passing, with the line one pixel wide and half of one
  half a pixel of the panel, the dial's points half a pixel inside the corners its
  fractions land on, the sky blue above the ground brown and the ball's lines green
  by channel, and the dial's five colours - case, sky, ground, green and white - no
  two of them within 0.2 of each other in any channel; and the drawn panel read
  back out of a real OpenGL context with `glReadPixels`: the green horizon with sky
  a pixel above it and ground a pixel below, the reference white at the middle of
  each of its six segments and white over the horizon where the two cross, every
  rung green at its own step of the scale,
  the horizon still green and still tilted under a 20 deg roll, and the reference
  still white and still in the middle at every attitude.  The five PNGs beside
  `attic/_attitude_hud.py` are the same panel again, at level, at 20 deg of roll
  and at 10 and 30 deg of pitch, with the whole window behind the last of them.

* 2026-09-30 - **the window has an attitude indicator.**  The HUD could say where
  the four controls are and nothing at all about which way the aircraft was
  pointing, which on the pad is the whole story of a pickup and in the air is the
  one thing a keyboard cannot see.  There is a fourth panel now, beside the
  collective's and in the shape of `attic/attitude.png`: the case, the sky over
  the dark ground, the horizon with its pitch ladder, and - over the middle of
  all of it, fixed and never moving - the aircraft's own reference, the two bars
  and the W between them.  The ball turns the way the world appears to a pilot
  who rolls (a roll to starboard lifts the starboard end of the horizon and turns
  the sky to port) and slides the way it appears to one who pitches (a nose up
  puts the horizon below the reference), at 25 deg of pitch to the ball's own
  radius, past which the horizon is off the dial and the ball is one colour.
  It is the one panel on this HUD that is not drawn from a hand:
  `draw_control_panel` reads `Simulation.telemetry`'s own roll and pitch for it,
  which is why it still says something on the pad, where the skids hold the
  aircraft level and a key moves the other three panels but the aircraft itself
  not at all.  Every shape of it is a small function of pure arithmetic -
  `attitude_ball_axes`, `attitude_line_height`, `attitude_line_ends`,
  `attitude_ball_point`, `attitude_horizon_gamma`, `attitude_ladder_half`,
  `attitude_stroke_corners` and `attitude_reference_strokes` - with the drawing
  wrapped around it as the three older panels have, and nothing is drawn over the
  case: the ball is filled to its own edge and every line of it is cut to the
  chord that edge allows, so the sky, the ground, the horizon and the ladder all
  stop at the ball.  Of the picture's case only the plain rim is drawn - no tick
  marks and no index, since neither is readable at 132 px and the reference and
  the horizon are what the panel is for.  The window's row of panels is four
  wide now, 584 px of its 960, and the run prints where the second of the two
  references is.  Verified: `main.py --check` passing, with the ball's own axes
  level, unit and a quarter turn from the case's; the horizon the width of the
  ball through the middle of the dial with both of its ends on the ball's own
  edge at every pitch; the chord's ends the same two points the fill's arc is
  drawn to; the starboard end of the horizon the high one under a roll to
  starboard, and the ball's up off the case's by exactly the roll; a bank cutting
  the pitch's share of the slide by the cosine of that roll; the pitch that
  carries the horizon off the dial; a ladder whose rungs are the whole step they
  name, the same both ways and shorter as they go out; the reference symmetric,
  inside the ball and hanging below its own middle line; four square panels in a
  row and on the window; and the pad's own attitude read off the aircraft rather
  than a key - and then the drawn panel itself, read back out of a real OpenGL
  context with `glReadPixels`: the sky over the ground with one white line
  between them, the ladder at 10 and 20 deg each way, the horizon a whole step
  lower at 10 deg of pitch with the reference still white on the dial's middle
  line, the horizon tilted and the ladder turned with it under a 20 deg roll, all
  sky and all ground at the two ends of the scale, the horizon's own end inside
  the ball and the case outside it, and the panel over the world in the chase view
  with the frame's own scene drawn behind it.

* 2026-09-30 - **a run begins with the collective lever three quarters of the way
  up, not on the floor.**  A pad start put the lever on its down stop, which is
  where a parked UH-1's lever is, but it meant seven seconds of holding `W` at the
  keyboard's quarter rate before the rotor lifted anything: the first thing every
  flight did was wind.  `simulation.PAD_COLLECTIVE` - 0.75 of the travel, 8.25 in
  of the 11 - is where `Simulation.on_the_pad`, and so `main.py`'s start and its
  `P` and `R`, leaves it now, which is half a second of key under the 9.09 in of
  this run's hover trim.  It is a *position somebody made* rather than
  the trim, so a reset still comes back to the pad and not to the trim, and the
  whole travel is still a park's to name: `on_the_pad(collective=0.0)` is the
  lever on its down stop, which is `rotor_control.PilotControls.collective_down`,
  and the new `collective_at(fraction)` - the general form `collective_down` is
  now written in - is what names any position.  The skids still hold the aircraft
  at 0.75: on the 6158 lb flight test aircraft the rotor lifts 23.6 kN there
  against 27.4 kN of weight, 86 per cent of it, so it sits light on its skids and
  stays there, and the pickup the key then makes is 1.5 s to airborne against
  6.9 s from the down stop.  Verified: `simulation.py`'s self test reading the
  pad's lever, its 8.25 in and the pilot's own axis at 0.75, a second park named
  at 0.0 that `R` brings back to 0.0, two seconds of hands-off frames that move
  no skid, and the keyboard pickup reaching the trimmed fraction and lifting off;
  `main.py --check` reading the caption at `coll  75 %` after `R`, the pickup
  from the pad, and the tap block's travel counted from the pad's own lever; and
  the six module self tests and the headless rehearsal of the demo's mission -
  the kilometre out still flown on the keyboard - passing.

* 2026-09-29 - **a flight that ends in the air is not called a crash.**  The
  caption had one word for two different events, and it was the wrong word for
  the second one: a step whose state leaves the model's envelope is refused in
  `_advance` and the aircraft is latched crashed, so the caption read `CRASHED`
  at whatever altitude the frame was refused - 25.7 m up on the 30 m pickup the
  demo flies, with nothing near the ground and no ground contact made.  `crashed`
  stays the flag, because it is what a program resets on, but the *reason* is now
  kept beside it: `Simulation.crash_reason` is `CRASH_GROUND_CONTACT` (the
  floor's arrival, a descent over `UH1_HARD_LANDING_MPS` at touchdown) or
  `CRASH_OUT_OF_ENVELOPE` (the refusal, which is an overload the model will not
  fly), `Telemetry` carries it through, and `Telemetry.crash_message` is the text
  for it - `CRASH_MESSAGES`, `CRASHED` and `OUT OF ENVELOPE`.  That is what
  `Telemetry.__str__`, `main.py`'s caption and the demo all print, so a refusal in
  the air is named as one and never as a landing it did not make; a reason that is
  neither, an aircraft a script has called crashed by hand, is shown as the
  floor's, since before there were two messages there was one.  `reset` clears the
  reason with the flag, and nothing else about either end moved: the latch, the
  frozen state a refusal leaves and the two places that set `crashed` are exactly
  what they were.  Verified: `simulation.py`'s self test asserting both reasons
  and both messages, that the envelope one is neither `CRASHED` nor `on the
  ground` on a telemetry line, and that R leaves no reason at all; `main.py
  --check` reading both captions off a real failed pickup and a real refusal,
  with the refused state asserted identical through another two seconds of
  frames; the demo printing both reasons and the flying aircraft's empty one; and
  `main.py --check` with the six module self tests passing.

* 2026-09-29 - **a key is a quarter of a hand's rate, so a control can be
  placed.**  One keypress moved too far to be usable: the collective turned at
  0.55 of its travel a second, so `W` reached the hover trim in 1.7 s and a pilot
  could not put the lever where they wanted on purpose.  `PilotInput` now has
  three step granularities - `COLLECTIVE_STEP_GRANULARITY`,
  `CYCLIC_STEP_GRANULARITY` and `PEDAL_STEP_GRANULARITY`, 0.25 each, one per
  control, and fields on the class as well as constants, so a script can make its
  own keys as fine or as coarse as it likes - and the new `key_rates()` is each
  control's rate times its granularity.  `step` is the only reader: a held key
  turns its ratchet at 0.1375 a second on the collective, 0.4 on both cyclic axes
  and 0.5 on the pedals, so the same travel takes four times the presses, and four
  times as long on the key - 7.3 s over the 11 in of collective where 1.8 s did -
  while `set_axes`, which is a joystick or a script arriving at an absolute
  position and has no keypress to scale, keeps the rates unmodified.  Nothing else
  moved: the ratchets, the travels, the trims and the four axes are exactly what
  they were, and a key that is not held still turns nothing at all.  What it
  changes for a pilot is the pad: the collective now takes 6.8 s of `W` to reach
  the trim's fraction instead of 1.7 s, so the pickup is a slower and far more
  controllable one.  Verified: the self test asserting the three granularities,
  the three key rates and the axes a quarter second of held keys makes -
  `0.1375 * 0.25` and its neighbours, a quarter of the numbers it asserted
  yesterday - `simulation.py`'s demo printing them (`the key rates: (0.1375, 0.4,
  0.5)`, and a released stick still at 0.1 of travel), and `main.py --check`
  passing with the windows the slower keys need: two seconds of `W`, `Right` and
  `D` on the pad where half a second used to do, twelve seconds of `W` to reach
  the trim where six did, and eighty taps of `W` where twenty were a fifth of the
  collective.
* 2026-09-29 - **the controls are drawn, not only printed.**  The window had one
  instrument - its caption - and now has a second that is read without words, in
  the three panels of `attic/controls_simple.png`: two red lines for the pedals,
  the right one rising as the left one falls, the two crossing at a level middle;
  a red disc for the cyclic, placed in a plan view of its own panel, forward at
  the top and right to the right; and a red lever on a green quadrant for the
  collective, the green field being the lever's own travel between its stops, so
  the lever lies along the field's lower edge at the down stop - where the pad
  leaves it, and what the mockup is showing - and along its upper edge at full up.
  The panels are pixels rather than metres: `draw_control_panel` pushes the pixel
  projection and the modelview, turns the depth test, the fog and the culling off
  while it draws and hands all of it back afterwards, and blends the black panels
  over the world, so no camera move and no distance in the world touches them.
  What they are drawn from is `PilotInput.axes` - the hands - and not the trimmed
  inches, which is why a key on the pad moves the panels while the aircraft itself
  cannot move at all, and it is the same four positions the caption prints.  Each
  panel is one small function of pure arithmetic - `pedal_line_heights`,
  `cyclic_dot_centre`, `collective_lever_deg`, `panel_box` and `panel_point` -
  with the drawing wrapped around it, so `--check` can assert the whole HUD
  without a window, and `main()` prints where the mockup is.  Verified:
  `main.py --check` passing - opposite pedal lines that stay on the panel and
  cross at its middle, a cyclic disc that is the panel's middle at the stick's own
  middle and a whole disc at a stop, a collective lever on the field's own edge at
  either stop, three square panels in a row apart and on the window, and the same
  four axes read off the aircraft after 30 frames of `W`, `Right` and `D` on the
  pad - and the drawn frame read back out of a real OpenGL context with
  `glReadPixels`: red at each pedal line, at the cyclic disc and along the lever,
  green across the field and grass above the field's roof, the lever moved and the
  floor green again with the collective up, the disc red at a stop with no panel
  edge cutting it, and the projection and the GL state out of `draw_scene` exactly
  the ones it was handed.
* 2026-09-28 - **the cyclic and the pedals are ratchets too, so nothing
  springs.**  The input layer had grown two models: the collective ratcheted,
  and the cyclic and the pedals sprang back to a `neutral` - the inches
  `PilotInput.rest_on` put them on, with `PilotControls.springs` and `offset`
  as the two halves of that arithmetic.  It was invented to make a reset hand
  back the trim's own stick and it was wrong about the aircraft: a UH-1's cyclic
  and pedals are held by friction and by force trim rather than by a centring
  spring, so a stick pushed forward stays forward until it is pushed back - the
  same thing `README.md` and the caption had been telling a pilot all along, and
  the reason the trim light exists.  All four axes are now one kind of thing: a
  control *position* the model's own travels turn into inches.  That is why the
  axis has to stay absolute rather than an offset from the trim: a ratchet
  driven to "trim plus full throw" would jam on the stop and the trim would
  silently shrink whatever throw was left, and since `from_axes` applies the
  stops on the way out, an axis on a stop is a stick on a stop and `at_stop()`
  reads empty for a ratcheted stick - nothing has to clip it.  `PilotControls`
  lost `springs()` and `offset()` and gained `to_axes()`, the exact inverse of
  `from_axes` and all a reset needs, and `collective_down()`, the pad's own
  controls: the trim's cyclic and pedals with the collective at the down stop.
  `PilotInput` lost `neutral`, `centring_rate`, `rest_on` and `_hand` and gained
  one `_ratchet`, which a released key does nothing to, so `step` is four of
  those and `reset` is `to_axes` on a stick position.  A trim now only says
  where the axes *start* - which is how a run begins at the trimmed hover - so
  an aircraft whose cyclic was moved and released departs from the trim and
  stays departed, which is exactly the departure `in_trim` is there to report,
  and `on_the_pad` and every `reset` put the hands on
  `controls.to_axes()` rather than on the rigging's zero.  Verified:
  `python rotor_control.py` and `python simulation.py` printing `self test
  passed`, `main.py --check` passing, and the old spring assertions rewritten as
  the positions themselves - a quarter second of held forward stick then a
  quarter second of nothing leaving the axis at 0.40 and the stick at 2.58 in,
  the pad pickup holding its trim's own 2.18 in of forward cyclic while the
  collective alone moves, and a reset asserting the hands on the pad's own stick
  with only the collective at a stop.


* 2026-09-28 - **`R` comes back to where the run began, not to the trim.**
  Pressing `R` on a freshly started program was a flight: `Simulation.reset`
  rebuilt the aircraft from `trim_state`/`trim_controls`, so a bare reset handed
  it the 200 m hover and 83 per cent of collective - the hover trim's 9.09 in, a
  control position nobody had made - and the frame loop's cyclic, whose springs
  were then still centred on the rigging's zero, flew it away from there.
  Reproduced before the change: a fresh `main()`
  reset to alt 200 m and coll 83 per cent, and ten hands-off seconds later it
  read roll -142.3 deg and `CRASHED`.  `Simulation` now keeps
  `start_state`/`start_controls`, the state and the stick positions *this* run
  began with - recorded by `on_the_pad`, by `trim` and `trim_level_flight`, and
  by any explicit `reset(state=...)` - and a bare `reset()` rebuilds exactly
  those, from a copy of the state so the run's own start survives the steps that
  follow.  `begins` is `state is not None or on_ground`, so a reset that only
  renames an altitude or a heading is a reposition and leaves the start alone,
  and `heading_deg` now defaults to `None`, so a bare reset comes back to the
  start's own heading rather than to north.  It is idempotent by construction: a
  reset of an aircraft that has not flown since the last one changes nothing,
  and one reset undoes another.  The trim is untouched - it is the reference
  `in_trim` reads, not a place - which is what the docstrings of `reset`,
  `on_the_pad`, `trim` and `trim_level_flight` and `README.md` now say.
  The reset forced the input layer to agree with it: a reset on the pad can only
  leave the pilot the pad's own controls if a released cyclic is the *trimmed*
  cyclic rather than a step back to the rigging's zero, so `PilotInput` grew a
  `neutral` - the inches the cyclic and the pedals spring back to, put there by
  the new `PilotInput.rest_on`, with `rotor_control.PilotControls.springs` and
  `offset` as the two halves of what that does.  The collective is deliberately
  not part of a neutral: a ratchet stays where a hand leaves it, and a neutral is
  only what a spring can hold.  `trim`, `trim_level_flight`, `on_the_pad` and
  every `reset` therefore rest the springs on the trim the run flies, which is
  the position the trim light reads, so an aircraft flown with nothing held flies
  its trim: a fresh `Simulation()` held hands-off for ten seconds stays in trim
  with 0.005 m of drift, a 60 kt level trim holds its 60 kt and its altitude the
  same way, a second of forward cyclic and right pedal released comes back to the
  trim's own 2.18/1.14/1.26 in, and a pad pickup on the collective alone lifts
  off with the sticks still on the trim.
  Verified: `python simulation.py` printing `self test passed` with a new
  keyed/reset block - six seconds of `W` to leave the pad, then `R` back to the
  pad's exact state, the four axes at zero with the collective at its down stop,
  `sim_time`, `frames` and `steps` at zero, no trim light, a second `R`
  identical, and an explicit `reset(state=...)` start coming back on its own
  heading and 20 m/s with the run's hover trim still at 200 m, the neutral
  asserted to be `trim_controls.springs()` and 600 hands-off frames asserting
  the trim light still on; `main.py --check` passing, with the pad pickup
  followed by `R` asserting the pad's own state, the caption at `coll   0 %`
  with `on the ground` and no `trim`, the sticks at the trim's own inches and
  120 hands-off frames that do not move it; `py_compile` clean; and both repro
  scripts from the report - `R` from the start now leaves the collective at
  0 per cent with the aircraft parked, and no state leaks between runs.
  The spring model lasted the day: the entry above is what the four axes are
  now, and what `PilotInput`, `rest_on` and `springs()` became.


* 2026-09-28 - **a tapped key is a frame of travel, and a window with no
  keyboard says so.**  The frame loop sampled `pygame.key.get_pressed()` once
  per frame, so a `W` pressed and released inside one frame was travel thrown
  out of a ratchet: the second way a live key looks dead.  The loop now keeps
  its own record from the `KEYDOWN` and `KEYUP` events - `frame_keys`, beside
  `pilot_keys`, over the eight `FLIGHT_SCANCODES` the four axes are flown
  from - counts a key that went down inside the frame as held for it, and
  clears the record when the window loses the keyboard, because a key held
  then never gets its `KEYUP`.  The caption adds `no keyboard: click the
  window` whenever `pygame.key.get_focused()` is false, which is the one
  failure no control position can show - the four axes sit wherever they were
  last left -
  and the console says the same once on each focus change.  Verified: the check
  asserts that a scanned frame's keys are the eight of the four axes, that one
  frame of a tapped `W` is exactly `0.55 * 1/60` of collective travel and stays
  there when the key is gone, that twenty one taps is over 1.5 in of collective
  on the caption, and that the focus flag appears only when the window has no
  keyboard; `main.py --check` and all six module self tests pass.

* 2026-09-28 - **the caption carries all four control positions.**  A key the
  aircraft cannot show looked like a key that did nothing: the caption had the
  collective on it and nothing else, and on the pad the skids hold the aircraft
  level and still, so the cyclic and the pedals moved nothing but a `coll %`
  that was already up.  `window_title` now prints them in the model's own
  units - `cyc %+6.2f/%+6.2f in` for the longitudinal and lateral stick and
  `ped %+5.2f in` for the pedals.  They are the same numbers the check reads -
  `Telemetry.long_stick_in`, `lat_stick_in` and `pedal_in` - so no new
  plumbing.  The docstring and `README.md` say the same, since the caption is
  the sandbox's only instrument panel.  Verified: `main.py --check` passing,
  with the caption asserted on the pad after 30 frames of `W`, `Right` and `D`
  - 27 per cent of collective, the longitudinal stick on the trim the springs
  rest on, and the right stick and right pedal over an inch past it, with the
  position, the speed and the rates still exactly zero - and five deliberately
  broken copies of the caption, a frozen pedal, a duplicated cyclic field, the
  collective in inches under a per cent sign, the pilot's axes in place of the
  stick inches and a check that stopped holding the cyclic.  Every one of them
  was caught.
* 2026-09-28 - **`scenery.py` has a test.**  It is the one module whose test
  needs no window and no physics, so `python scenery.py` is the whole of it: it
  lists `sample_scenery.xml`, writes it out again, reads that back, and prints
  what the loader refuses and in whose words.  The self test under the demo
  round trips the project's own 21 objects in memory and through a temporary
  file - the same coordinates, orientations and parameters in the same order,
  and the same text again from the reloaded copy, since a serializer that
  drifted a little would make a load and a save a diff - checks the kind
  defaults a short description leaves out, the element and attribute shape
  `to_element` writes, and every way a description can be wrong: a root element
  that is not `<scenery>`, malformed XML, an unknown object kind, a missing
  coordinate, a coordinate that is not a number, a misspelled parameter such as
  `hieght`, a child element inside an object, and a file that is not there
  coming back as an `OSError` rather than as a scenery error.  Verified:
  `python scenery.py` printing `self test passed`, `py_compile` clean,
  `main.py --check` still passing, and eleven deliberately broken copies of the
  module - a dropped check, a dropped attribute or a rounded number each - every
  one of them caught by the test.
* 2026-09-28 - **the aircraft is in `main.py`.**  It is `simulation.Simulation`
  on the report's 6158 lb instrumented configuration, started parked on the pad
  with the rotor turning and the collective down.  The keyboard goes in through
  `PilotInput` (`W`/`S` collective, the arrows the cyclic, `A`/`D` the pedals),
  `R` recovers to the trimmed 200 m hover, `P` parks it on the pad and `C` swaps
  the chase camera for a fixed one on the pad.  It is drawn in its own body axes
  - one `glMultMatrixf` of `render_matrix` - with the main and tail rotor discs,
  the blades at table 2's coning and the clock's azimuth, and a shadow that fades
  with altitude.
* 2026-09-28 - **the world now fits the aircraft.**  A 16 km ground plane with a
  two scale grid - 10 m lines within 400 m of the aircraft, 250 m lines out to
  8 km, both aligned to world coordinates - fog in the sky's own colour where a
  horizon belongs, and a 1 m to 20 km depth range in place of the old 80 m far
  plane.  The camera clamp is gone entirely: +/-30 m and 1 to 8 m of altitude was
  a walk around the field the scenery describes, while the model's trims sit at
  200 m and a 60 kt trace covers a kilometre in twenty seconds.  The free camera
  and its vector helpers went with it, since the keys they used now fly the
  aircraft.
* 2026-09-28 - **`numpy` is gone.**  Nothing in the renderer wanted it: the
  model's own column-major tuple is handed to `glMultMatrixf` through a
  `GLfloat` array, so the unused import and its pin in `requirements.txt` are
  both gone, and the requirements are pygame and PyOpenGL.
* 2026-09-28 - **`python main.py --check`**: the wiring of `main.py` checked
  headless, with no window and no OpenGL context - the key mapping, the pickup
  from the pad, the recover and park keys, the camera triples, the sixteen floats
  of the render matrix, the rotor azimuths and the world's own constants.
  Verified: `py_compile` clean, `--check` passing, and 300 frames rendered in
  both views on an OpenGL 4.6 context with `glGetError` returning 0.
* 2026-09-28 - the root swept: 114 files down to the nine that are the project,
  with the extraction scratch in `attic/`, the mining scripts in `tools/`, three
  OCR texts promoted into `reference/`, and a duplicate of the TM-73254 PDF (MD5
  identical to the copy in `reference/`) removed.  `attic/README.md` says what
  was moved and why.
* 2026-09-28 - `README.md`, `reference/README.md`, `tools/README.md`, this file,
  `.gitignore`, and the first commit of the project.
* 2026-09-28 - `requirements.txt` pinned to exactly what `main.py` imports, and
  verified against the interpreter.
