# The best lap of the square — 2026-09-15, 12:23

The complete log of this run (the snapshot of the script that produced it is
kept in the original project archive, not in this repository). It is the point
to go back to if something breaks later.

## What it did

Four stretches and four turns, with no stalls and no failed turns.

- **The four turns worked**: +90.5 · +90.8 · +92.7 · +91.6 degrees, all in
  about 4 s. Mean error +1.39.
- **The diagonal practically disappeared**: −2.50 · −1.36 · −1.55 degrees
  against the walls, when two runs earlier it was at +10 to +14.
- **Heading during the stretches**: mean +0.43 degrees, worst +3.21.
- **Re-centering after each turn**: −12 · +2 · +7 mm from the 8 cm target.
- Accumulated over the lap: +7.27 degrees.

## The configuration that achieves it

Front stop at 90 mm with **progressive braking** from 330 and **stopping by
anticipation** (it brakes based on where the robot WILL BE 0.35 s from now,
not where it is). Turn with the parameters validated on Sep 14: `tolgiro` 4.4
and `gradfreno` 45. Lateral target 8 cm from the right wall, with the 17 mm of
lateral inertia compensated.

🔑 **And the centering by TAPS, which is what removed the abruptness.** The
force stays fixed at 40, which is the minimum these wheels can execute, and
what is modulated is how often it is applied: with a small error a tap every
now and then, with a large error on every cycle. In this run it corrected only
in 12, 35, 25 and 52 % of the cycles depending on the stretch. Lowering the
force instead of the frequency does not work: below 40 the wheels do not break
free and the robot does not move.

## What is still wrong

🔴 **The spread of the overshoot when stopping.** With a 90 mm threshold the
stretches ended between 15 and 73 mm from the wall: the second one stopped at
15, almost touching. The mean value is already tamed; what is missing is the
spread.

🔴 **The turn bias accumulates.** The four overshoot towards the same side and
nothing compensates them, so the lap closes shifted to the left. The first two
came out at +0.5 and +0.8 and the last two at +2.7 and +1.6.

⚠️ **The front sensor is still uncalibrated below 150 mm**, so when it says
15 mm we do not know how far it really is.
