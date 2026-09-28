# Reference documents

What the model was built from, and what was read while checking it.  The code
cites two of these by name - NASA TM-73254 and TM 55-1520-210-10 - and the rest
is kept because a number in the physics should always be traceable back to a
page, even when the trace goes through something that was read and then not
quoted.

## NASA TM-73254 - the report this project implements

    NASA-TM-73254_UH-1H-force-and-moment-model.pdf   2.4 MB
    NASA-TM-73254_OCR-text.txt                       43 kB

Talbot and Corliss, 1977: *A Mathematical Force and Moment Model of a UH-1H
Helicopter*.  Everything in `airframe.py` is from here, and a great deal of
`aerodynamics.py` and `rotor_control.py` is too:

* **table 1** - the control positions the report's own simulation was trimmed
  at, which `airframe.py` carries as its trim reference;
* **table 2** - the geometry: rotor radius, chord and precone, the full throw
  control travels, the cyclic pitch ranges, the control rigging, the c.g.
  travel and the mast tilt;
* **table 3** - weights and inertias, the main rotor force constants, the tail
  rotor, fuselage, fin and stabilizer constants, the linkage gearing c1 to c6,
  the rotor time constant, the stabilizer bar, the ground effect range;
* **equations 1 to 66** - the force and moment model itself, rotor through
  stabilizer to the six degree of freedom equations of motion;
* **the validation chapter** - figures 2 to 9, eight step control inputs flown
  on an instrumented UH-1H at Crows Landing, which `regression_tm73254.py`
  reflies.

**Read the text layer with suspicion.**  The OCR is badly damaged: the tables
come back as runs of numbers with their rows shuffled and the equations lose
their operators altogether (`rotor_control.py` says so at length, and explains
why).  The constants in this project were read off the scanned page images
instead, which is why several pairs of them could be checked against each
other.  Where a number could not be recovered at all, the code says so where it
uses it rather than guessing.

## TM 55-1520-210-10 - the operator's flight manual

    TM55-1520-210-10_full.pdf                        14.7 MB
    TM55-1520-210-10_fulltext.txt                    684 kB
    UH-1H-flight-manual-part1_scanned.pdf            9.9 MB
    fm_pages/page_0001.jpg ... page_0083.jpg         9.3 MB, 83 pages

The UH-1H flight manual.  The model takes the rotor speed points of the
operational band from it, and the tail rotor's limits.  The 83 page images under
`fm_pages/` were extracted from the scan because the same OCR problems apply to
its tables as to TM-73254's.

## The rest - read, not quoted

    NASA-TN-D-4632_high-advance-ratio-rotors.pdf     5.4 MB
    TN-D-4632.txt                                    99 kB
    NASA-TM-73258_OCR-text.txt                       55 kB
    USAAMRDL-TR-71-34_AD0734343_OCR-text.txt        178 kB
    AD0739559_OCR-text.txt                          309 kB

No module cites these today.  They are the wider reading that came with the
rotor modelling and the failure modes:

* **TN D-4632** - high advance ratio rotors, which is where a retreating blade
  stall and a reverse flow region actually get interesting;
* **NASA TM-73258** (Corliss and Talbot, August 1977) - *A Failure Effects
  Simulation of a Low Authority Flight Control Augmentation System on a UH-1H
  Helicopter*, read while the failure cases were being thought about;
* **USAAMRDL TR 71-34** (AD0734343) and **AD0739559** - two more rotor and
  control system reports, fetched from DTIC.  AD0739559's title page did not
  survive its OCR, hence the bare AD number in the filename.

The last three arrived as plain text at the top of the project root (as
`_tm73258.txt`, `_DTIC_AD0734343.txt` and `_DTIC_AD0739559.txt`) and were moved
here and renamed on 2026-09-28.  If they are ever deleted, they can be fetched
again: they are public NASA and DTIC documents.
