# 2026-09-15 16:10 — nine turns in a row choosing the free side. CLOSED

Nine stretches and nine turns, with the cap at 9 and a manual stop with Ctrl+C.
Script: `../back_and_forth.py`.

## Figures

It chose **R · R · R · R · R · L · L · R · R**, always towards the side that was
free.

**The heading closed at −450.42 against the −450 those nine turns add up to:
0.42 degrees of error.** The **mean error per turn was −0.18**, the best of the
whole session.

Front stops: 31, 18, 32, 33, 24, 25 and 53 mm with the target at 41, plus one
exception, stretch 5 at **173 mm**, in an area where the front sensor started
seeing 600 and probably took something for a wall that was not one.

## 🔴 The only ugly event: a 21-second turn

**TURN 6, to the left, gave +98.96 degrees and took 21.4 s** when the others
take 4, with the ESP reporting 9.05 degrees unclosed. It is the firmware
retrying the whole turn because it did not converge the first time, the same
pattern that appears when `tolgiro` is tightened below 4.4. It happened right
when going out into the open area, with 117 mm on the right and 405 on the left.

**The compensation of the next stretch absorbed the 9.69 degrees it carried**,
that is why the overall heading ended at 0.42. It is the proof that the
absolute heading scheme withstands a whole bad turn without breaking the run.

## What is still broken in open field

Stretches 6, 8 and 9 spent between **14% and 35%** of the time without seeing
any wall. With nothing to center against only the gyroscope is left, and there
the heading goes off: stretch 5 drifted +11.45 degrees and stretch 7, −9.95.

## Configuration

Identical to `best_2026-09-15_1605/` except for the stretch cap, which goes
from 6 to 9. Its README has the details of turning towards the free side, of
why the half turn does not fit in the corridor, of the softened lateral
correction and of the speed at 112.
