# TODO

## Next

Nothing outstanding.  Every module has a demo that asserts as it prints - the
six below and `scenery.py` - and `main.py` has `--check` for the wiring that
flying cannot test.

## Known limitations, from the modules themselves

These are choices, not oversights: each is stated where it lives.

* **Rotor speed is fixed.**  There is no engine and no rotor speed dynamics, so
  the rotor turns at 100 per cent whatever the aircraft does.
* **The ground is a floor, not a contact model.**  `simulation.py` holds the
  c.g. on the skid line, takes the descent out of the velocity, and levels roll
  and pitch; the skids do not flex, slide or spring.
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

## Done

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
