# 2026-09-15 15:47 — the circuit in the RIGHT direction, closed

Two full laps, eight stretches and eight turns. **It is the best heading run of
the whole session, in either direction.** Script: `../square_right.py`.

## Figures

**The square closed at −719.79 degrees against the −720 it should: 0.21 degrees
of error over two laps.** The best one in the left direction was 0.51.

The eight front stops, in what the sensor publishes: **28, 29, 24, 56, 44, 34,
24 and 12 mm**, with the target at 41. None touches the wall.

The eight turns: −91.8 · −88.9 · −92.7 · −91.6 · −90.7 · −93.3 · −92.2 · −91.4.
Mean error −1.58 degrees. The bias is still there, but **the compensation
between stretches absorbs it**: the accumulated error of the whole square is
+0.24.

## What distinguishes it from the left-direction file

Only three constants: `GIRO` goes from `"L"` to `"R"`, `PASO_GIRO` from +90 to
−90, and the log name. **The 46 control constants are identical** to
`best_2026-09-15_1433/`: braking thresholds, lateral setpoints, turn tolerance
and braking, gains and heading compensation.

And one code change, in the re-centering after the turn:

🔴 **THE RE-CENTERING CANNOT ASSUME WHICH SIDE THE WALL IS ON.** Written against
the right sensor, when running the circuit in this direction it did not find
its wall and **it was not executed even once**. Measured on the 15:39 run: in
the seven stretches the right sensor read between 551 and 566 — i.e. nothing —
and the left one between 39 and 75. The robot started each stretch where the
turn left it, and since it arrived at the next one off center, **the turn
scraped the wall**: it sweeps 263 mm and the corridor is 250, so it only fits if
it is well placed. That is what looked like oscillation in the turns, and the
turns had nothing wrong: the six of that run gave between −90.3 and −92.2 in
less than three seconds, without a single bounce.

🔑 **The thresholds do not change because of that.** `CONSIGNA_E` 52.5 and
`CONSIGNA_O` 43.0 add up to the measured gap of 95.5: they describe **the same
physical spot seen from each side**. The re-centering sticks to the wall it
sees, with that sensor's setpoint, and if it sees both it picks the closest,
which is the one it could hit.

## What remains open

- **The turn bias**, −1.58 on average. It is not removed by tightening
  `tolgiro` (tried: with 3 the firmware retries the whole turn and bounces).
  The compensation covers it.
- **The take-off PWMs against the 170 ceiling** all afternoon, and they are
  saved in flash, so they survive the reset: resting the motors does not
  lower them.
- **The AGC of the four AS5600 encoders is at 10, 4, 26 and 38** when the healthy
  range is 80-220. The Pi's own report translates it as a magnet off center or
  at the wrong distance from the chip. It is a degraded position reading on the
  four wheels, and a candidate to explain the turn bias and the take-offs.
  **Look at that before tightening the control further.**
