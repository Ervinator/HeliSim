# HeliSim

A Bell UH-1H helicopter flight model, and a small OpenGL world to fly it in.

The model is the point.  It is built from NASA TM-73254, *A Mathematical Force
and Moment Model of a UH-1H Helicopter*: the report's own equations, its own
tables of constants, and its own validation chapter turned into a runnable
regression.  Every number the physics uses is quoted back to the page it came
from, and the whole chain is standard library Python, so it runs and tests
headless, with no window and no third party package.

## The modules

Layered, each one importing only what sits below it:

| module | lines | what it is |
| --- | --- | --- |
| `aerodynamics.py` | 1617 | quasi-steady blade element theory for one rotor: section data, flapping, hover and forward flight loads |
| `rotor_control.py` | 1233 | the control path: inches of stick travel to swashplate angles, the mixing law and its rigging, the control lags |
| `airframe.py` | 2094 | TM-73254's force and moment model - main rotor, tail rotor, fuselage, fin, stabilizer - and the six degree of freedom equations of motion |
| `simulation.py` | 1579 | the frame loop: a fixed 60 Hz clock around `airframe`, `PilotInput`, the ground under the skids, `ChaseCamera`, the trims, and what a renderer or a HUD asks for |
| `regression_tm73254.py` | 661 | the report's figures 2 to 9 reflown: eight step inputs from 60 kt and from the hover, laid next to its published responses |
| `scenery.py` | 280 | the XML scenery format and its loader - no pygame, no OpenGL |
| `main.py` | 904 | the OpenGL sandbox: the implicit grass plane and its two scale grid, the scenery, and the aircraft of `simulation.py` flown from the keyboard, drawn in its own body axes and followed by a camera |

Only `main.py` imports anything outside the standard library.

## Running

    pip install -r requirements.txt
    python main.py [scenery.xml]
    python main.py --check          # the wiring, headless, no window

`sample_scenery.xml` is used when no path is given.  The scenery file lists
trees and hills with a coordinate and an orientation; `scenery.py`'s docstring
and the file's own header describe the format.  The ground plane and its grid
are implicit and are deliberately not part of the description.

The aircraft is `simulation.py`'s own: the report's 6158 lb instrumented UH-1H,
which starts parked on the pad, and the keys fly it:

| key | what it does |
| --- | --- |
| `W` / `S` | collective up / down; a ratchet, so it stays where it is left |
| `Up` / `Down` | cyclic forward (nose down) / aft (nose up) |
| `Left` / `Right` | cyclic left / right |
| `A` / `D` | pedals: nose left / right, the anti torque control |
| `R` | recover: back to the trimmed hover at 200 m |
| `P` | park: skids on the pad, collective down |
| `C` | camera: behind the aircraft, or fixed on the pad |
| `Esc` | quit |

Hold `W` until the rotor lifts it off the skids.  The keyboard goes in through
`simulation.PilotInput`, so the cyclic and the pedals spring back to centre when
they are released, the collective does not, and the aircraft is drawn with one
`glMultMatrixf` of `Simulation.render_matrix` with a `ChaseCamera` for a view.
The window's caption is the HUD: altitude, vertical speed, airspeed, collective,
and the trim, on the ground and crashed flags.

The world is kilometres across because the aircraft is: a 16 km ground plane,
10 m grid lines within 400 m of the aircraft and 250 m lines out to 8 km, and fog
that fades the plane's edge into the sky where a horizon belongs.  Nothing
clamps the aircraft or the camera any more; the envelopes in `simulation.py` are
the limits.

## Self tests

Nothing here needs a window, so every test runs headless:

    python regression_tm73254.py

The physics modules each run a demo when executed directly - `aerodynamics.py`,
`rotor_control.py`, `airframe.py`, `simulation.py` and `regression_tm73254.py` -
and the demos assert as they go, so a module that prints a full demo has passed
its own numbers.  `regression_tm73254.py` is the one worth running after any
tuning: it is the report's validation chapter, and it prints the eight traces in
the report's own units and time base so they can be read against figures 2 to 9.

`main.py`'s wiring has a check of its own, headless and without a window:

    python main.py --check

It asserts the key mapping, the pickup from the pad, the recover and park keys,
the camera triples, the sixteen floats the renderer is handed, the rotor
azimuths and the world's own scale.  `scenery.py` is the module still without a
test; see `TODO.md`.

## Layout

    .
    |-- aerodynamics.py            the physics chain, from the rotor up
    |-- rotor_control.py
    |-- airframe.py
    |-- simulation.py
    |-- regression_tm73254.py
    |-- scenery.py
    |-- main.py
    |-- sample_scenery.xml         the default scenery
    |-- requirements.txt           pygame and PyOpenGL, for main.py only
    |-- README.md
    |-- TODO.md                    what is left
    |-- reference/                 the reports the model was built from
    |-- tools/                     the scripts that mined them out of the PDFs
    `-- attic/                     the scratch that used to fill the root

## Requirements

Python 3.8 was what this was developed against, and `requirements.txt` pins
exactly what `main.py` imports: pygame and PyOpenGL.  Nothing else in the
project imports anything outside the standard library, so the model runs on a
bare interpreter, and the renderer wants no numpy - it hands the model's own
column-major tuple to `glMultMatrixf` through a `GLfloat` array.  The pins are
exact rather than floors because these are the versions this was developed and
run against, Python 3.8.2 on Windows; loosen them to `>=` if the interpreter
moves on.

## The reference documents

`reference/` holds the source documents, their OCR text, and the 83 scanned
pages of the flight manual that were read while checking them.
`reference/README.md` says which is which.  TM-73254 is the one the model is
built from, and TM 55-1520-210-10 is where the rotor speed band and the tail
rotor's limits come from; the other four were read and are not quoted.

## Status

The physics chain is complete, the report's own validation responses are
reproduced, and `main.py` flies the aircraft in the sandbox.  What is left is
`scenery.py`'s own test.  See `TODO.md`.
