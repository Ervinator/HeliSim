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
| `main.py` | 377 | the OpenGL sandbox: the implicit grass plane and its grid, the scenery, and a free camera |

Only `main.py` imports anything outside the standard library.

## Running

    pip install -r requirements.txt
    python main.py [scenery.xml]

`sample_scenery.xml` is used when no path is given.  The scenery file lists
trees and hills with a coordinate and an orientation; `scenery.py`'s docstring
and the file's own header describe the format.  The ground plane and its grid
are implicit and are deliberately not part of the description.

The camera in `main.py` is a free one, and its keys are:

| key | what it does |
| --- | --- |
| `W` / `S` | drive forward / back along the view direction |
| `A` / `D` | yaw left / right |
| `Left` / `Right` | roll left / right |
| `Up` / `Down` | pitch down / up |
| `Esc` | quit |

**`main.py` does not fly the helicopter yet.**  It draws the world and moves a
free camera through it; the aircraft in `simulation.py`, which is complete and
trimmed and flies, is not wired to the keyboard or to a chase view.  That is the
next piece of work, and `TODO.md` has it.

## Self tests

Nothing here needs a window, so every test runs headless:

    python regression_tm73254.py

The physics modules each run a demo when executed directly - `aerodynamics.py`,
`rotor_control.py`, `airframe.py`, `simulation.py` and `regression_tm73254.py` -
and the demos assert as they go, so a module that prints a full demo has passed
its own numbers.  `regression_tm73254.py` is the one worth running after any
tuning: it is the report's validation chapter, and it prints the eight traces in
the report's own units and time base so they can be read against figures 2 to 9.

`scenery.py` and `main.py` have no self test; see `TODO.md`.

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
exactly what `main.py` imports: numpy, pygame and PyOpenGL.  Nothing else in the
project imports anything outside the standard library, so the model runs on a
bare interpreter.  The pins are exact rather than floors because 1.24 was the
first numpy to drop 3.8; loosen them to `>=` if the interpreter moves on.

## The reference documents

`reference/` holds the source documents, their OCR text, and the 83 scanned
pages of the flight manual that were read while checking them.
`reference/README.md` says which is which.  TM-73254 is the one the model is
built from, and TM 55-1520-210-10 is where the rotor speed band and the tail
rotor's limits come from; the other four were read and are not quoted.

## Status

The physics chain is complete and the report's own validation responses are
reproduced.  What is missing is the aircraft in the sandbox: `main.py` is still
a free camera over the scenery.  See `TODO.md`.
