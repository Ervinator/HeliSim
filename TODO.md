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
* **Two tail rotor numbers are not in either report.**  The tail blade twist and
  its built in coning come from this project's own blade element survey - what
  `aerodynamics.py`'s self test runs - rather than from TM-73254 or
  TM 55-1520-210-10.
* **The model is evaluated in SI** while the reports tabulate English units.
  The conversions are written next to each constant, so both columns can be
  read, but a difference in the fourth figure can come from either.

## Done

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
  failure no control position can show - all four sit at zero - and the
  console says the same once on each focus change.  Verified: the check
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
  - 27 per cent of collective, the longitudinal stick at zero, and the right
  stick and right pedal over an inch, with the position, the speed and the
  rates still exactly zero - and five deliberately broken copies of the
  caption, a frozen pedal, a duplicated cyclic field, the collective in inches
  under a per cent sign, the pilot's axes in place of the stick inches and a
  check that stopped holding the cyclic.  Every one of them was caught.
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
