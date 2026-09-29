# Current version of the square circuit — 2026-09-15, 14:03

The complete log of this run (the script snapshot is kept in the original
project archive). **It was the version the work continued from**: it gathers
everything learned during the afternoon.

## What it does better than any other

**The re-centering after the turn became precise**: 52, 48, 42, 48 and 61
against a target of 52, i.e. 0 to 10 mm of error. The previous run oscillated
between −19 and +11. What fixed it was **shortening the last pulse** (minimum
of 35 ms instead of 60), not tuning the inertia constant.

**The heading compensation no longer runs out of budget.** With the cap at 8
pulses, the stretches that carried more than 3 degrees used it up, leaving
2.04 and 1.78 uncorrected. With 14, the stretches that carried 6.11 and 7.59
degrees corrected them entirely, in 10 and 7 pulses, leaving 0.25 and 0.70.

**Soft start**: the speed ramps up during the first second and a half of the
stretch instead of asking for the cruise speed at once, which leaned the robot
to the right and then made it compensate by sticking to the left.

**And the correction while moving is only by soft taps.** The big shoves
arrived on time but were abrupt: the robot did not go straight and on top of
that each translation twisted its heading (20 mm of translation = 1.3 degrees,
measured). The big fix is done by the stopped re-centering; while moving it is
only touched up.

## 🔴 What is NOT solved: the turns degrade within the run itself

The four turns of the **first lap** gave a mean error of **+0.05 degrees**,
practically perfect. Those of the **second lap**, **+2.30**, with the last
three overshooting almost three degrees each.

That is why the heading ended at +724.69 where it should read 720: **the
compensation corrects well what it carries, but the next turn adds three
degrees again and it is always one stretch behind**.

🔑 **It is not the control: it is the hardware degrading along the run.** The
take-off PWMs ended once again at 165-169 against the 170 ceiling, as in all
the runs of the afternoon. As the batch goes on the wheels need more push to
break free, and the turn is the first to feel it because it asks for little
speed per wheel. **That is what has to be looked at before tuning more numbers.**

## Configuration

Front stop at **63** with progressive braking from 330, stopping by
anticipation (0.35 s) and a 1.5 s start ramp. Lateral: setpoint **52.5**
against the right wall, lateral inertia **12**, band 5, left escape limit at
32. Centering by taps with a dynamic target (average of the two walls when it
sees them). Absolute heading with the loop closed on `yaw_ref`, band 0.8 and
up to 14 pulses.

**Nothing of the turn is touched**: `tolgiro` 4.4 and `gradfreno` 45, the ones
validated on Sep 14.

## Reference of the afternoon runs, by final heading

- 12:44 → +0.51 degrees (the best heading, but without any of the above)
- 13:53 → −0.61
- 14:00 → +1.16
- **14:03 (this one)** → +4.69, with a flawless first lap and a degraded second one
- 13:55 → +5.80
