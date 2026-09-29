# Two laps of the square, with the heading that stops accumulating — 2026-09-15, 12:44

**The best result of the session (for heading).** Eight stretches and eight
turns in a row, without a failure. The complete log of the run (the script
snapshot is kept in the original project archive).

## The result

🔑 **The heading ended at +720.51 degrees where it should read 720**: half a
degree of error after two full laps.

**And the error stopped accumulating, which is what was sought.** What was left
over after each turn: +0.68 · −1.02 · −0.89 · −0.32 · −0.28 · +0.76 · +0.83. It
oscillates within one degree and stays there lap after lap, instead of growing.
For comparison: with independent turns, four turns already added up to 7 or 8
degrees and kept rising.

- **Turns**: +92.3 · +89.3 · +88.6 · +90.2 · +90.9 · +90.0 · +91.2 · +89.6.
  Mean error **+0.28** (it was +1.47 without compensation). Turns below 90
  appear: that is the compensation giving back what was left over from the
  previous one.
- **Stops**: between 54 and 84 mm with a threshold of 95.
- **Re-centering** at 7 cm from the right wall: 2 to 19 mm from the target.
- **Heading drift per stretch**: mean −0.28, worst −2.34.

## How it is achieved

**The heading is tracked in absolute terms**: the target is n × 90 from zero,
and whatever is left over from one turn is discounted in the next stretch.

🔑 **It is not corrected by turning while stopped** — that was discarded on Sep
2, because the smallest motion of the car is larger than the error to correct
— but **while rolling**, taking advantage of the firmware moving its own
reference with the turn rate it is sent: `yawRef += pedidoW * 180/PI * dt`.

🔴 **And the pulse is not computed blindly: the loop is closed on `yaw_ref`**,
which travels in the telemetry. The firmware **jumps to W_ARRANQUE (36
degrees/s) as soon as any `w` is requested**, because otherwise the
short-gearbox wheel does not break free, so asking it for 8.8 degrees/s each
pulse moved four times more than planned and the compensation oscillated with
growing amplitude (−1.87, +2.75, −4.44). Giving short pulses and looking at
where the reference is, that disappears.

## The rest of the configuration

Front stop at **95 mm**, with progressive braking from 330 and **stopping by
anticipation** (it brakes based on where the robot will be 0.35 s from now).
Turn with the parameters validated on Sep 14: `tolgiro` 4.4 and `gradfreno` 45
— tightening the tolerance to 3 causes retries and makes it worse, it is
measured. Lateral target 7 cm from the right wall, with the 17 mm of lateral
inertia compensated. **Centering by taps**: fixed force at 40, the minimum these
wheels execute, modulating how often it is applied instead of how hard.

## What remains open

⚠️ **The front sensor is still uncalibrated below 150 mm**, so the stopping
distances it reports are approximate.

⚠️ The corridor has a gap in the left wall, and that is why the centering
discards side readings above 140 mm.
