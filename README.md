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
| `rotor_control.py` | 1280 | the control path: inches of stick travel to swashplate angles, the mixing law and its rigging, the control lags |
| `airframe.py` | 2094 | TM-73254's force and moment model - main rotor, tail rotor, fuselage, fin, stabilizer - and the six degree of freedom equations of motion |
| `simulation.py` | 2055 | the frame loop: a fixed 60 Hz clock around `airframe`, `PilotInput`, the ground under the skids, `ChaseCamera`, the trims, and what a renderer or a HUD asks for |
| `regression_tm73254.py` | 661 | the report's figures 2 to 9 reflown: eight step inputs from 60 kt and from the hover, laid next to its published responses |
| `scenery.py` | 603 | the XML scenery format and its loader - no pygame, no OpenGL, and the one module whose test needs no window and no physics |
| `main.py` | 2596 | the OpenGL sandbox: the implicit grass plane and its two scale grid, the scenery, the aircraft of `simulation.py` flown from the keyboard, drawn in its own body axes and followed by a camera, the control position panel of `attic/controls_simple.png` with the attitude indicator of `attic/attitude.png` beside it, over the world, and the run's own key log - every keystroke with the flight it was made in - beside it |

Only `main.py` imports anything outside the standard library.

## Running

    pip install -r requirements.txt
    python main.py [scenery.xml] [--log FILE | --no-log]
    python main.py --check          # the wiring, headless, no window

`sample_scenery.xml` is used when no path is given.  The scenery file lists
trees and hills with a coordinate and an orientation; `scenery.py`'s docstring
and the file's own header describe the format.  The ground plane and its grid
are implicit and are deliberately not part of the description.

The aircraft is `simulation.py`'s own: the report's 6158 lb instrumented UH-1H,
which starts parked on the pad with the collective lever three quarters of the
way up its travel, and the keys fly it:

| key | what it does |
| --- | --- |
| `W` / `S` | collective up / down |
| `Up` / `Down` | cyclic forward (nose down) / aft (nose up) |
| `Left` / `Right` | cyclic left / right |
| `A` / `D` | pedals: nose left / right, the anti torque control |
| `R` | reset: back to the state and the stick positions the run began in |
| `P` | park: skids on the pad, the collective lever at 75 % |
| `C` | camera: behind the aircraft, or fixed on the pad |
| `Esc` | quit |

Every one of those axes is a ratchet: a key slews its control while the key is
held and a released key moves nothing at all, so every control stays where the
hand left it, exactly as a UH-1's friction and force trim leave it.  The keys are
deliberately fine: `simulation.PilotInput`'s three step granularities -
`COLLECTIVE_STEP_GRANULARITY`, `CYCLIC_STEP_GRANULARITY` and
`PEDAL_STEP_GRANULARITY`, a quarter each - are what a *key* is worth, one per
control, so a key turns its ratchet at a quarter of the rate the control's own
feel gives it and the same travel takes four times the presses, and four times as
long on the key.  That is what makes a stick placeable rather than something only
thrown from one stop to the other.  A joystick or a script is not scaled:
`set_axes` is absolute and has no keypress to step.


`R` is where the *run* began, not where the trim says the aircraft belongs: this
program begins parked on the pad, so `R` on a freshly started aircraft changes
nothing at all - the lever stays at the 75 per cent the park left it at and the
skids stay on the ground - and `R` after a flight comes back to the same pad.  All
four axes are ratchets, so a reset is never a control input: it puts the hands on
the stick positions the run began with - on the pad those are
`collective_at(PAD_COLLECTIVE)`, the trim's own 2.18 in of forward cyclic and so
on with the collective lever three quarters of the way up its travel, a position
somebody made but not the one the trim sits on - and a ratchet stays there.  (The
whole travel is still a park's to name: `on_the_pad(collective=0.0)` is the down
stop, `collective_down()`.)  The 9.09 in of collective of the hover trim is a
reference the caption's trim light reads, not a control position the pilot made:
handing it to somebody who pressed
`R` would be handing them a ratchet nobody moved.  `P` is
the key that asks for the pad wherever the aircraft is, and the trim's own
numbers stay where they belong, on the light.  `simulation.py`'s own
`Simulation.reset` leaves the trim alone and remembers whichever state a run
began with, including a trim or a state named by a script.

The pad's lever position is worth a paragraph of its own, because it is the one
number in the start that is this project's choice rather than the report's.  A
UH-1's collective is heavy and its travel is 11 in, so a run that began with the
lever on the floor took seven seconds of holding `W` before the rotor lifted
anything; `simulation.PAD_COLLECTIVE` starts it at 0.75 of the travel instead,
which leaves about half a second of key to the 6158 lb aircraft's hover trim - an
inch and a third on the 8700 lb one, whose trim is higher - and is still a
position under the weight: on the 6158 lb aircraft the rotor lifts 23.6 kN at
that lever against 27.4 kN of weight, 86 per cent of it, so the skids sit light
with the aircraft parked and stay on them until the key takes the lever the last
0.84 in.  It is a lever *position* and not a trim, so `R` still
comes back to it rather than to the trim's 9.09 in.

Hold `W` until the rotor lifts it off the skids, and tapping it works as well:
the frame loop takes its keys from the events rather than sampling the keyboard
once a frame, so a press and release inside one frame still turns the ratchet.
The keyboard goes in through `simulation.PilotInput`, where all four axes are
the same kind of thing: a key slews its axis while the key is held, and a key
that is released moves nothing at all, so a stick stays exactly where the hand
left it - which is what a UH-1's friction and force trim do, and what a centring
spring would not - and the aircraft is drawn with one `glMultMatrixf` of
`Simulation.render_matrix` with a `ChaseCamera` for a view.  The window's
caption is the HUD in the model's own units: altitude, vertical speed, airspeed,
the collective in per cent and the longitudinal and lateral cyclic and the pedal
positions in inches, the trim, on the ground, how a flight ended if it has, and -
at the front of the line, where a title bar cannot cut it off - a `no keyboard`
flag while the window is not being sent keys at all.  The end of a flight is the
model's own two messages, `CRASHED` and `OUT OF ENVELOPE`: the first is the skids
arriving on the ground over `UH1_HARD_LANDING_MPS`, and the second is a state the
model refused to fly - an overload in mid-air, at whatever altitude the frame was
refused, which is why it is not called a crash.  The control positions are there
because the aircraft cannot always show a key by itself: on the pad the skids
hold it level and still, so the cyclic and the pedals move nothing until it is
off the ground.  A window with no keyboard is the one failure that looks exactly
the same from the controls alone - the four axes sit where the reset put them and
never move - which is why the caption names it and the cure.

The window draws those same four positions as well, as three square panels in
its bottom left corner, in the shapes of a mockup sketch kept in
`attic/controls_simple.png` - the attic is this project's ignored scratch, so
the panels are described in words here as well as drawn: the two pedals as red
lines that move oppositely, the cyclic as a red disc whose place in its panel is
the stick's own position - forward at the top, right to the right - and the
collective as a red lever on a green quadrant, the green field being the lever's
own travel between its stops and its lower edge the down stop.  They are drawn
from `PilotInput.axes` - the hands, the same four positions the caption is read
from - so a key moves them whether or not the aircraft can move, and they are
read without words: they are a HUD over the world rather than part of it.

Beside them is a fourth panel, an attitude indicator in the shape of a second
sketch, `attic/attitude.png`: a round dial whose case carries a white version of
the aircraft's own reference - two short bars and a W between them - fixed over
the middle of it, and behind that the ball, blue sky over brown ground with a
green horizon across them and a green pitch ladder above and below it.  Every
line of the dial is one pixel wide, and the reference is the one white thing on
it and the last thing painted on it, so the bars and the W stay white wherever
one of the ball's own green lines crosses them.  It is the one panel on this HUD
that no hand can draw, so it is read from the aircraft itself,
`Simulation.telemetry`'s roll and pitch: the ball turns the way the world appears
to a pilot who rolls - a roll to starboard lifts the starboard end of the horizon
and turns the sky to port - and slides the way it appears to one who pitches, a
nose up putting the horizon below the reference.  25 deg of pitch, which is the
whole of the ball's own scale, takes the horizon off the dial, and then the ball
is one colour and out of the way.  The reference never moves, which is what makes
a moving horizon readable, and nothing of the picture's case beyond it is drawn:
no rim ticks and no index, since neither is readable at 132 px.

The world is kilometres across because the aircraft is: a 16 km ground plane,
10 m grid lines within 400 m of the aircraft and 250 m lines out to 8 km, and fog
that fades the plane's edge into the sky where a horizon belongs.  Nothing
clamps the aircraft or the camera any more; the envelopes in `simulation.py` are
the limits.

## The key log

A run writes `keylog.txt` beside `main.py` - `--log FILE` puts it elsewhere,
`--no-log` flies with no log at all, and a run overwrites the last one's.  It is
one line per keystroke as the event loop takes it out of pygame's queue, and
twenty lines a second of the flight between them (`KEYLOG_RATE_HZ`), so what was
pressed and what the aircraft did about it are in the same file.  The first lines
of the file name the columns; every line after them is one row of them, comma
separated:

| column | what it is |
| --- | --- |
| `t_s` | `time.perf_counter` seconds since the log was opened, monotonic, to a millisecond |
| `wall_ms` | that same instant as `time.time` milliseconds, to a millisecond |
| `frame`, `sim_s` | the model's own frame count and physics time, a `SIM_TIME_STEP_S` apiece |
| `kind` | `down`/`up`, `reset`/`park`/`view`/`quit`, `focus`/`blur`, or `sample` |
| `key`, `held` | the key's own name, and the flight keys down at that instant |
| `spd_mps`, `vn_mps`, `ve_mps`, `vd_mps` | velocity: airspeed through the airframe, then over the ground north, east and down |
| `hdg_deg`, `pitch_deg`, `roll_deg` | heading from north towards east, pitch, roll |
| `alt_m` | the c.g.'s height above the ground |
| `coll_in`, `long_in`, `lat_in`, `pedal_in` | the pilot's four control positions, in inches of travel |
| `coll_frac` | the collective as a fraction of its own travel |

Every line is the aircraft's own telemetry and the hand as it stood, so it stands
on its own whether the key that wrote it was a flight key or one of the four
command keys.  A keystroke is stamped where the loop takes it out of the queue -
the queue is drained before the frame is flown - so a key's line is at most a
frame later than the press itself, which is inside the 1/60 s step the physics
could have shown it in anyway, and the `sample` lines after it are what that
frame did with it.  The two clocks are there so a line can be read against the
flight around it, through the monotonic `t_s`, and against anything else that
happened at that time of day, through `wall_ms`.  `keylog.txt` is in
`.gitignore`: the log of the last flight, not part of the project.

## Self tests

Nothing here needs a window, so every test runs headless:

    python regression_tm73254.py

The modules each run a demo when executed directly - `aerodynamics.py`,
`rotor_control.py`, `airframe.py`, `simulation.py`, `regression_tm73254.py` and
`scenery.py` - and the demos assert as they go, so a module that prints a full
demo has passed its own numbers.  `regression_tm73254.py` is the one worth
running after any tuning: it is the report's validation chapter, and it prints
the eight traces in the report's own units and time base so they can be read
against figures 2 to 9.

`scenery.py` needs neither a window nor the physics:

    python scenery.py

It lists `sample_scenery.xml`, writes it out again and reads that back - the
round trip is the whole of the format - and prints what the loader refuses and
in whose words.  Its self test asserts the same, including that a description
which is malformed is refused rather than quietly misread.

`main.py`'s wiring has a check of its own, headless and without a window:

    python main.py --check

It asserts the key mapping, the pickup from the pad, the reset and park keys, the
camera triples, the sixteen floats the renderer is handed, the rotor azimuths,
the world's own scale, the arithmetic of the control position panel - the
pedal lines, the cyclic disc, the collective lever and the attitude indicator's
own ball, its horizon's chord and the ladder above and below it, the one pixel its
lines are drawn at and the five colours of its dial - and the key log: the
command line that names it, the names it gives a key and a hand, the columns of a
line, the two clocks on it to a millisecond and the rate the flight is sampled
at.  None of it needs an OpenGL context.

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
reproduced, and `main.py` flies the aircraft in the sandbox.  Every module has a
demo that asserts as it prints, so running any one of them is its own test;
`TODO.md` lists what the model deliberately leaves out.
