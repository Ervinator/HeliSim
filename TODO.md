# TODO

## Next

**1. Give `scenery.py` a test.**  It is the one module with no self test and no
demo, and nothing imports it but `main.py`.  `Scenery` parses *and* writes, so a
load / save / load round trip of `sample_scenery.xml` would cover the format,
including the `SceneryError` cases.

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
