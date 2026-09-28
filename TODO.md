# TODO

## Next

**1. Fly the aircraft in the sandbox.**  `main.py` draws the world and moves a
free camera through it; the flight model in `simulation.py` is finished and is
not wired to it.  Three pieces:

* the keyboard through `simulation.PilotInput` and `Simulation.fly`, over the
  output of `Simulation.step`, instead of the free camera keys in `main.py`;
* a chase view from `Simulation.render_matrix` and `ChaseCamera.eye_target_up`,
  in place of the camera basis `main.py` keeps for itself;
* a world scale that fits the aircraft.  `main.py` clamps the camera to +/-30 m
  and 1 to 8 m of altitude, which is a walk around the field the scenery
  describes, while the trims sit at 200 m and the regression's twelve second
  traces cover kilometres.

**2. Decide the `numpy` import in `main.py`.**  `main.py` imports numpy and
never uses it; `requirements.txt` pins numpy because of that import.  Either the
renderer work wants it, or the import and the pin should both go.

**3. Give `scenery.py` a test.**  It is the one module with no self test and no
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

* 2026-09-28 - the root swept: 114 files down to the nine that are the project,
  with the extraction scratch in `attic/`, the mining scripts in `tools/`, three
  OCR texts promoted into `reference/`, and a duplicate of the TM-73254 PDF (MD5
  identical to the copy in `reference/`) removed.  `attic/README.md` says what
  was moved and why.
* 2026-09-28 - `README.md`, `reference/README.md`, `tools/README.md`, this file,
  `.gitignore`, and the first commit of the project.
* 2026-09-28 - `requirements.txt` pinned to exactly what `main.py` imports, and
  verified against the interpreter.
