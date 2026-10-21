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
| `engine.py` | 805 | the T53-L-13 on the other end of the shaft: the twist grip, the N2 governor and its droop band, the torque available and what EMER and bleed air cost, and the gas producer's lag |
| `airframe.py` | 3633 | TM-73254's force and moment model - main rotor, tail rotor, fuselage, fin, stabilizer - and the six degree of freedom equations of motion, with the rotor's own speed as a state when an engine is aboard, the second root of equation 10 read off a shaft with nothing driving it, and the ground under it as one flat plane with a level and a normal |
| `simulation.py` | 3083 | the frame loop: a fixed 60 Hz clock around `airframe`, `PilotInput`, the ground under the skids, `ChaseCamera`, the three trims, and what a renderer or a HUD asks for |
| `regression_tm73254.py` | 661 | the report's figures 2 to 9 reflown: eight step inputs from 60 kt and from the hover, laid next to its published responses |
| `scenery.py` | 603 | the XML scenery format and its loader - no pygame, no OpenGL, and the one module whose test needs no window and no physics |
| `controls.py` | 1105 | the device map: which physical control flies which - keyboard or joystick, per control, with deadzone, invert and calibration - read from the XML of `default_controls.xml`, and likewise stdlib only |
| `main.py` | 4527 | the OpenGL sandbox: the implicit grass plane and its two scale grid, the scenery, the aircraft of `simulation.py` flown from its device map - keyboard, joystick, the mouse, or a mixture - drawn in its own body axes and followed by a camera, the control position panel of `attic/controls_simple.png` with the attitude indicator of `attic/attitude.png` beside it, over the world, and the run's own key log - every keystroke with the flight it was made in - beside it, with one `--engine` switch that fits the T53 before the first frame, the cursor on the cyclic and the wheel on the collective (`M`, `--mouse` and its own `--mouse-reverse`), and one line for the grip of a run that has none |

Only `main.py` imports anything outside the standard library - and that is a
claim about the nine modules above rather than about the project.  The seven
`_rt_*.py` files in the root beside them are *the harness*: the pilots that fly
the model and the instruments that measure them, and they all want pygame, three
of them by name (`_rt_pilot` for the nine scancodes its keys are, `_rt_demo` for
the window it opens, `_rt_autoro_probe` for the grip's own two) and the other
four through `_rt_pilot`.  No harness but `_rt_demo` opens a window and none of
them needs an OpenGL context.  See *The harness* below.

## Running

    pip install -r requirements.txt
    python main.py [scenery.xml] [--engine] [--log FILE | --no-log]
                   [--controls FILE | --no-controls]
                   [--mouse | --mouse-reverse]
    python main.py --check          # the wiring, headless, no window

`sample_scenery.xml` is used when no path is given.  The scenery file lists
trees and hills with a coordinate and an orientation; `scenery.py`'s docstring
and the file's own header describe the format.  The ground plane and its grid
are implicit and are deliberately not part of the description.

The aircraft is `simulation.py`'s own: the report's 6158 lb instrumented UH-1H,
which starts parked on the pad with the collective lever three quarters of the
way up its travel, and the keys fly it.  The one thing about that aircraft a run
may decide before it starts is its engine, and `--engine` is how it says so: the
T53 is fitted before the first frame, so the grip has a governor behind it from
the beginning rather than from the `F` that would fit it live.  Flying without
one is what a run that says nothing gets, and it is what the report's own figures
were computed with.

| key | what it does |
| --- | --- |
| `W` / `S` | collective up / down |
| `Q` / `E` | twist grip rolled off / open, the fifth control, which says so the first time it moves on an aircraft with no engine behind it |
| `Up` / `Down` | cyclic forward (nose down) / aft (nose up) |
| `Left` / `Right` | cyclic left / right |
| `A` / `D` | pedals: nose left / right, the anti torque control |
| `R` | reset: back to the state and the stick positions the run began in |
| `P` | park: skids on the pad, the collective lever at 75 % |
| `F` | fit the engine this run did not begin with - or take it off again (`--engine` fits it at startup) |
| `G` | the GOV AUTO/EMER switch, which needs an engine to govern |
| `X` | an engine failure, and `X` again to clear it |
| `C` | camera: behind the aircraft, or fixed on the pad |
| `M` | the mouse: the cursor on the cyclic and the wheel on the collective (`--mouse` starts that way, `--mouse-reverse` turns the wheel over) |
| `Esc` | quit |

The aircraft of the report's figures has no engine in it, because that is what
every figure in TM-73254 was computed with - so a run begins on the report's own
aircraft and `F` fits a T53 to the shaft it is flying, live.  It is a re-fit and
not a reset (`simulation.Simulation.fit_engine`): nothing of the aircraft moves,
and what moves is the trim reference the caption's light reads, to the 322.98 rpm
hover the governor and the rotor settle on together.  From there the twist grip
is a control that does something, `G` is the GOV AUTO/EMER switch of
TM 55-1520-210-10 (9-3), and `X` is the fuel going away - the wind down through
the engine's own lag, which a hovering UH-1 has a couple of seconds of.  The
caption gains the engine's own panel (rotor speed, N2, the torque gauge and the
switch) the moment there is one to read, and gives it up again when `F` takes it
off: the two aircraft are told apart by the gauges and by nothing else.  `F` is
the whole of getting one live mid flight, and `--engine` on the command line fits
it before the first frame instead - for a run that means to fly the grip from the
start rather than from the tenth frame.

The failure is not the end of a flight, either.  A rotor whose shaft has nothing
driving it is offered the second root of the report's equation 10 - the windmill
brake state, air up through the disc - so it draws its torque from the air rather
than from the engine, and what the flight manual does about that is a descent of
its own: `simulation.Simulation.trim_autorotation`, the third of the frame loop's
trims, which solves the entry at 60 kt - collective 6.26 in, 9.1 m/s down,
1792 fpm, against the 1800 to 2000 a UH-1H holds there - and leaves the run flying
it with the rotor at 100 per cent.  `X` is what opens the door; the trim is what
a script asks for, so a run that fails its engine by hand is on its own between
the failure and the entry, and the model has no blade stall to be forgiving about
it: a *vertical* descent with the collective still up is where equation 10's two
roots meet, and that frame is one the loop refuses outright.

Every one of those axes is a ratchet: a key slews its control while the key is
held and a released key moves nothing at all, so every control stays where the
hand left it, exactly as a UH-1's friction and force trim leave it.  The keys are
deliberately fine: `simulation.PilotInput`'s four step granularities -
`COLLECTIVE_STEP_GRANULARITY`, `CYCLIC_STEP_GRANULARITY`,
`PEDAL_STEP_GRANULARITY` and `THROTTLE_STEP_GRANULARITY`, a quarter each - are
what a *key* is worth, one per control, so a key turns its ratchet at a quarter
of the rate the control's own feel gives it and the same travel takes four times
the presses, and four times as long on the key.  That is what makes a stick
placeable rather than something only thrown from one stop to the other.  A
joystick or a script is not scaled: `set_axes` is absolute and has no keypress to
step.

## The pilot's devices

Which physical control flies which is a file, not this program: `controls.py`
reads `default_controls.xml`, and each of the five controls - the cyclic up and
down, the cyclic left and right, the anti torque pedals, the collective and the
throttle - names its own device on its own line.  A control on the keyboard is a
ratchet and names two keys; a control on a joystick is either an *absolute* axis
(or a hat), which is read straight through at the hand's own rate, or a ratchet
turned by a pair of buttons.  `invert`, `deadzone` and a raw `minimum` and
`maximum` are there for a stick that reads backwards, jitters about its centre,
or does not reach the ends of its own travel.

The map this project ships is the keyboard mapping above, so a run that says
nothing about devices flies exactly as it always has - and the point of it is
that the switching is *per control*: an Arduino Leonardo sent as a generic USB
HID game controller (a name beginning `"Arduino"`, which is why a device is
found by `match` rather than by an index that moves when something else is
plugged in) can take the cyclic and the pedals while `W` keeps the collective.
A device that is not plugged in reads as no input at all rather than as a fault,
and one plugged in while the window is open is picked up as it appears.
`--controls FILE` reads another map and `--no-controls` flies on the built-in
mapping with no file at all; `python controls.py` prints the map, the
descriptions it refuses and its own self test.

The throttle is mapped like the rest and is the one control that is not a stick:
it is a twist grip, read as `simulation.PilotInput.throttle()` rather than as one
of the four axes, and it goes to the engine's governor (`engine.py`).  The map
this project ships has it on the keyboard's own `Q` and `E` - the pair
`main.py`'s ten keys have always rolled it with - so the grip the window reads,
and the pie that shows it on the collective's panel, follows those two keys; a
joystick or a home made collective takes the pair over by naming `throttle` on
its own device, the same one-control-at-a-time way as the four axes.  On the
aircraft main.py flies it is carried and shown and does nothing else, because
that aircraft has no engine in the model - the report's fixed rotor - so the first
`Q` or `E` of a run prints one line saying so, and naming both cures: `F`, and
`--engine`, which fits the T53 before the first frame.  It is said once and not
once a frame, a grip no hand has touched is not told about at all, and the
aircraft with a governor behind the grip is never told about it -
`airframe_preset(engine=True)` is where a throttle moves needles.


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
hold it level and still, and while the rotor is under the 4 deg of its own that
`simulation.GROUND_GRIP_ANGLE_DEG` names they hold the heading with it - the
pedal has nothing to react against while the disk is not lifting the aircraft -
so the cyclic and the pedals move nothing at all at the stops.  Above that angle
the skids hand the heading back and the pedals turn the aircraft on them, which
is what a pickup from the pad uses.  A window with no keyboard is the one failure
that looks exactly the same from the controls alone - the four axes sit where the
reset put them and never move - which is why the caption names it and the cure.

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

## The mouse

The one stick this sandbox can fly with nothing plugged in is the mouse: `M`
gives it the cyclic and the collective, and `M` again gives them back to the
devices.  `--mouse` asks for it at startup, and `--mouse-reverse` is the same
mouse with the wheel's sense turned over.

The cursor is the cyclic, read as a *position* rather than as a rate: the middle
of the window is the cyclic's own centre - with a 4 per cent band about it,
`MOUSE_CYCLIC_DEADZONE`, since a hand never leaves a cursor exactly there - and
the cursor's direction from the middle is the arrow key's of the same direction,
so a cursor above the middle is cyclic forward and the nose down, and one to the
right is roll right.  The window is the stick's own scale, so its edges are the
stops.  Because a cursor that stays off the middle holds the deflection it is in,
`R`, `P` and taking the mouse up all put the cursor back in the middle of the
window: that is `Mouse.recentre`, and it is the one part of the mouse that needs
a window.

The wheel is the collective, a notch at a time: `MOUSE_WHEEL_COLLECTIVE_STEP` is
a fiftieth of the lever's travel, so a notch is 0.22 in of the UH-1H's 11 in, and
the lever follows at the rate a hand would move it (`set_axes` slews it) rather
than jumping to the notch.  Where the wheel leaves the lever is remembered across
frames, because a notch arrives as an *event* and is read in a *frame*, and it is
clipped to the lever's own stops rather than running past them.
`--mouse-reverse` is the one option the wheel has: a notch of the wheel down for
more collective instead of up for more, which is a hand's own habit rather than
anything the aircraft has an opinion about, and the run says which way it is
reading it when the mouse is taken up.

Two things follow from one control being one hand.  The mouse flies exactly the
cyclic's two axes and the collective (`MOUSE_AXES`), so the aircraft is handed
over and not shared: with the mouse on, `W` is not the collective and the arrows
are not the cyclic, while the pedals stay on `A` and `D` and the grip on `Q` and
`E` - which is what lets a run fly a full pattern with the mouse as the stick.
And the lever is read back *off the aircraft* when the mouse is taken up, so
taking it up mid-flight is no step to the hands: the mouse begins wherever the
keys left the lever rather than at a position of its own.

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
| `kind` | `down`/`up`, `reset`/`park`/`view`/`quit`, `fit`/`unfit`/`governor`/`failure`/`recovered`/`mouse`/`devices`, `focus`/`blur`, or `sample` |
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
lines are drawn at and the five colours of its dial - the four values of the
command line and the mouse that is the fifth: the cursor's own reading of the
cyclic, the deadzone about the middle, the wheel's notches and its reverse, and
what the mouse does to a mapped control when it is flying it.  Then the grip's own
line - the one a grip with no engine behind it owes, and the three cases that owe
nothing - and the key log: the
command line that names it, the names it gives a key and a hand, the columns of a
line, the two clocks on it to a millisecond and the rate the flight is sampled
at.  None of it needs an OpenGL context.

## The harness

The nine modules above are the model.  The seven `_rt_*.py` files in the root
beside them are the harness: the pilots that *fly* the model, and the instruments
that measure whether it can be flown at all.  Nothing in the model imports one,
all seven are tracked, and the `_rt_` prefix is the project's own word for what
they are - scratch.  They arrived in two groups: the four the missions are flown
with, in one commit on 8 October 2026, and the three the human flyability
programme added on 20 and 21 October.

| module | lines | what it is |
| --- | --- | --- |
| `_rt_pilot.py` | 1480 | the keyboard pilot with a person's timing, and the three missions it flies: the cruise, the minute of hover, turn and landing, and the autorotation |
| `_rt_probe.py` | 118 | the ruler the pilot's gains are cut with: travels and the model's own trims, then one control stepped off the hover's trim, and what the model answered |
| `_rt_check.py` | 270 | the demo's rehearsal: the missions flown headless and fast through the same frame loop, with every claim a mission makes asserted |
| `_rt_demo.py` | 260 | the window `python main.py` opens, with a pilot at the keys instead of a person, so a mission can be watched at sixty frames a second |
| `_rt_handling.py` | 1172 | human flyability, phase 1: a clock, a transport delay, a gain, a lead and a lag closed around the model's own hover |
| `_rt_axes.py` | 1433 | phase 2: the lever and the pedals under scales of their own, and all four axes again at sixty knots |
| `_rt_autoro_probe.py` | 192 | the autorotation measured rather than guessed: its entry, its descent, and the rate its skids arrive at |

`_rt_pilot.py` is the one the others stand on, and it is three ideas.  A UH-1 has
a force trim and the handbook has numbers, so the loops are small-signal about the
trims `TrimTable` reads off the model, and they only have to catch what the trim
does not.  The pilot reads the instruments every `REACTION_S` seconds on the
*flight's* clock and not the frame's, so a dragged window does not change what was
flown.  And a key is *held* until the axis crawls to the position the pilot asked
for, which is what a hand does with a stick this slow.  Three missions fly on it,
each a class of its own - `HumanPilot` from the pad to a hundred metres up and a
kilometre out, `HoverTurnPilot` for the minute of hover, turn and landing, and
`AutorotationPilot` for five hundred feet at sixty knots with the twist grip
rolled right off - and nothing but the nine keys is touched in any of them.

Each is run the way every other module here is:

    python _rt_check.py            # both missions, headless: _rt_log.txt
    python _rt_demo.py [mission] [scenery.xml] [reaction_seconds] [frames]
    python _rt_probe.py            # the numbers a gain is built from
    python _rt_handling.py         # human flyability, phase 1: _rt_handling.txt
    python _rt_axes.py             # phase 2: _rt_axes.txt
    python _rt_autoro_probe.py     # the autorotation: _rt_autoro_probe.txt

Every transcript is written by the run itself rather than by the shell's
redirection, and `_rt_*.txt` in `.gitignore` covers all five of them, so what is
committed is the code that made them.  `_rt_handling.py`, `_rt_axes.py` and
`_rt_check.py` assert every number they print, so a regression is a traceback and
not a table nobody read.  `_rt_demo.py` is the only one that needs a display, and
one thing in it is asked of a key a *watcher* would press rather than by the
pilot: the engine, because an unattended pilot cannot press `F` and a rotor
*held* at a hundred per cent cannot autorotate at all - which is the aircraft
`_rt_autoro_probe.py` measures.  All seven want pygame installed, for the
scancodes a pilot's keys are.

The last three are the human flyability programme: the part of the harness that is
not about flying the model but about measuring the hands that would.  Two phases
have been flown, and both are written up in `TODO.md`, findings and numbers
together:

* **phase 1**, `_rt_handling.py` (20 October).  The model's own hover closed
  around a person's dynamics - twenty decisions a second, 0.2 s of transport delay
  out of a deque of remembered states, 0.1 s of lag in the hand and 1.5 s of lead
  on the rate, with the gains read off `_rt_pilot` rather than written down again,
  so a scale of one *is* the pilot the missions are flown with.  What comes back
  is a gain *band* and not a number: x0.75 to x1.00 of the model's attitude gains
  for pitch and x0.50 to x1.00 for roll, with the model's own pilot at the hard
  edge of both - and the failure above the edge is the pilot's own loop, an
  oscillation whose period makes 0.2 s of delay worth tens of degrees of phase;
* **phase 2**, `_rt_axes.py` (21 October).  The same instrument, with the lever
  and the pedals under scales of their own, and then all four axes re-flown at
  sixty knots.  Neither axis phase 1 left alone has a soft edge in the hover at
  all, and out there it is the tilt pair whose bands transfer - and *wider* -
  while the lever's and the pedals' do not.  So the hover's margin is the cyclic's
  margin and nothing else, and that is what a change to the model, the controls or
  the trims is checked against: `_rt_handling.py` and `_rt_axes.py` first, then
  `_rt_check.py` for the missions themselves.

Both phases hold the loops *above* the attitude at the model's own numbers on
purpose: what they measure is the margin a hand has on the cyclic.  Whether a
person's clock can hold the height, the speed and the heading that `_rt_check.py`'s
missions fly *scripted* is the programme's next phase, and it is named in
`TODO.md`'s `Next` rather than here, because it has not been flown yet.

One file beside all of these is not the harness: `_tmp_shadow.py`, which walks a
module's syntax tree and prints every module-level name bound twice.  It is the
check that caught `_rt_pilot.py`'s own `CLIMB_ALT_M` written down twice - eighty
metres at the top of the file and five hundred feet further down, so the long
mission's speed ramp had been solved against a height it never reaches, which is
why the engine-off mission's climb height is `AUTOROTATION_CLIMB_ALT_M`.  It is
the one `_tmp_` tool kept in the root and it is tracked, because the entries in
`TODO.md` that verify `_rt_axes.py` and `_rt_handling.py` verify them with it; the
six others - the sweeps, the probes and the one-flight diagnostics the programme
was worked out with - went to `attic/` on 21 October.

## Layout

    .
    |-- aerodynamics.py            the physics chain, from the rotor up
    |-- rotor_control.py
    |-- engine.py
    |-- airframe.py
    |-- simulation.py
    |-- regression_tm73254.py
    |-- scenery.py
    |-- controls.py
    |-- main.py
    |-- _rt_*.py                   the seven that fly the model: see The harness
    |-- _tmp_shadow.py             the one scratch tool kept: names bound twice
    |-- sample_scenery.xml         the default scenery
    |-- requirements.txt           pygame and PyOpenGL: a window and scancodes
    |-- README.md
    |-- TODO.md                    what is left
    |-- reference/                 the reports the model was built from
    |-- tools/                     the scripts that mined them out of the PDFs
    `-- attic/                     the scratch that used to fill the root

## Requirements

Python 3.8 was what this was developed against, and `requirements.txt` pins
exactly what `main.py` imports: pygame and PyOpenGL.  Nothing else in the model
imports anything outside the standard library, so the model runs on a bare
interpreter, and the renderer wants no numpy - it hands the model's own
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
demo that asserts as it prints, so running any one of them is its own test.  The
harness beside them flies the missions and measures whether the machine can be
held by hands - two phases of that so far - and `TODO.md` lists what the model
deliberately leaves out.
