# Calibration of the four VL53L0X ToF sensors — 2026-09-15

Three distances per sensor (150, 250 and 350 mm) measured with a perpendicular
board from the sensor module, robot on the floor and motors disabled. The raw
data is in `points.csv`; it is taken and fitted with `measure.py`.

## The four lines

Each sensor responds `read = c + m · real` in millimeters, and it is corrected
with `real = (read − c) / m`:

| sensor | c | m | maximum residual |
|---|---|---|---|
| N front | 46.7 | 0.890 | 3.1 mm |
| E right | 0.0 | 1.040 | 0.6 mm |
| S back | 15.6 | 1.032 | 0.5 mm |
| O (west) left | 24.9 | 1.043 | 0.7 mm |

Raw error before correcting: +5.7 to +40.3 mm depending on sensor and
distance. After correcting, below 1 mm on the three side/rear sensors and
3.5 mm on the front one.

## The pattern

**Three of the four share the slope**: 1.040, 1.032 and 1.043, i.e. a common
scale error of 3.8 % ± 0.5 for those sensors, which comes from the batch and
not from the mounting. What distinguishes them is the offset: 0.0, 15.6 and
24.9 mm.

🔴 **The front one is outside the group**: slope 0.890 (it compresses instead of
stretching) and an offset of 46.7 mm, the largest of the four. A large and
constant positive offset is the symptom of a signal returning too early — a
cover, tape or the mount inside the 25° cone. **Check the mounting of the front
sensor before taking the correction as final.**

The offset does not follow the chassis geometry: the two side sensors are
mounted the same way and they are the extremes of the group (0.0 the right
one, 24.9 the left one).

## Cross-validation

The 210 mm point of Sep 14 for the front sensor, taken in another session and
not part of this fit, falls 2.3 mm from the line.

🔴 **The 600 mm point of Sep 14 falls 10.1 mm below the line**, ten times the
spread. Near its cap the sensor compresses more than the line describes. **The
correction is only valid between 150 and 350 mm**; outside that range it is not
measured.

🔴 **The 200 mm point of Sep 14 for the right sensor (218.8) is discarded**: it
is 10.8 mm off a line whose residuals are 0.3 mm.

## Before applying the correction

🔴 **The wall thresholds already compensate part of this bias by hand.**
`UMBRAL_N` 300, `UMBRAL_E` 277, `UMBRAL_S` 300 and `UMBRAL_O` 352 are set
against UNCORRECTED readings. In real distance they are equivalent to 284.6,
266.3, 275.6 and 313.6 mm. If the distances are corrected and the thresholds
are left as they are, the wall detection changes meaning in every direction.

**To apply the correction without altering a single wall decision**, set the
thresholds to their real equivalent: N 284.6 · E 266.3 · S 275.6 · O 313.6.

🔴 **The 600 mm cap means "I see nothing" and is not corrected.** Passed through
the lines it would give 621.7, 576.9, 566.3 and 551.4, and it would stop being
a recognizable value. The correction has to skip the MAX_DIST value.

🔴 **The maze network was trained with the uncorrected distances**
(`NEURO_NORM_DIST` = 1200 normalizes `dists[]` as they arrive). Correcting them
changes its inputs, so the trained policy is not comparable before and after.

## What is left

- Physically check the mounting of the front sensor.
- Decide whether the correction goes in the firmware's `leerToF()` or on the
  Pi. The firmware is where `esPared()` and the neurocontroller use it; on the
  Pi it would only affect what is logged. (It was finally applied in the
  firmware — BLOCK 04b.)
- The behavior above 350 mm, not measured.

# Back and forth in a corridor — 2026-09-15

Corridor of 154 x 25 cm, robot with 6 cm on each side and open space ahead.
Five round-trip passes: it advances until the sensor in the direction of
motion sees 60 mm, stops, and reverses. It centers with the two side sensors
by translating sideways, without turning (the robot sweeps 263 mm when turning
and the corridor is 250). The script is `corridor.py`; the full log,
`corridor_2026-09-15_1119.jsonl`.

**The ten passes were completed and the ten stopped by sensor**, none by
timeout or by a cut-off wheel.

## What went well

**Exact speed**: 0.121 to 0.130 m/s against the 0.12 requested, with 39 wheel
rpm against the theoretical 40. No noticeable slip despite the continuous
lateral correction.

**Stable heading without cumulative drift**: the drifts per stretch range from
-2.13 to +1.82 degrees, but after the ten passes the heading went from
-178.61 to -179.68, i.e. **1.07 degrees over the whole run**. The error of
each stretch cancels with that of the next.

## 🔴 The centering oscillates, it does not converge

The difference between side sensors goes through a cycle from +16 to -24 mm
and back, with a mean per pass of 9 to 17 mm. It is not a steady-state error:
it is a limit cycle.

**The cause is the sensor delay**: each ToF refreshes every 160 ms and on top of
that the firmware publishes the mean of its last 5 readings, i.e. about 800 ms
of effective delay. A pure proportional controller against 800 ms of delay
always oscillates. Lowering `KP_LAT` reduces the amplitude but lengthens the
period; what is needed is to account for the delay (predict) or to close the
centering against the heading instead of against the position.

## 🔴 The stop overshoots by about 30 mm, with a large spread

With a 60 mm threshold, the last reading before braking is 61 to 67, but the
robot ends between 16 and 45 mm, with a mean of 29. **The overshoot is about
33 mm and its spread 29 mm**, almost as much as its value.

It comes from adding the sensor refresh (160 ms), the filter delay and the
braking inertia. To stop at a repeatable distance one has to anticipate, not
lower the threshold.

## 🔴 A ToF that regains sight lies for almost a second

On outbound pass 4 the left sensor lost the wall for several seconds — it read
a steady 600 while the right one read 49 — and **when it recovered it, it gave
a false ramp of 525, 421, 313, 203 and 109 mm in one second**, which is the
5-sample average filter dragging the previous 600. None of those values
corresponds to a real distance.

The centering loop reacted to that ramp and sent `vy` to the cap. **Any loop
that uses these readings has to discard the first samples after a stretch at
600**, not only the 600 itself.

**The reason is confirmed: the corridor has a gap in the left wall.** It is not
a sensor failure. Any test in this corridor has to account for the left side
sensor going blind in that stretch.


## The half-second pauses

They can be seen by eye and they are in the log: **26 samples out of 417 while
moving with the wheels below 12 rpm, 6.2 % of the run**, always in pauses of two
consecutive samples, half a second.

They are not all the same:

- **Ten are at the start**, one per stretch, in the first two samples. It is
  the inertia: the wheels have not yet broken free. Expected, and it fits with
  the take-off PWMs being halfway through re-learning.
- **Three are in the middle**, and all three happen while correcting: in
  `ida3` (outbound 3) with the robot 28 mm off center sending `vy`=+16, in
  `vuelta5` (return 5) 33 mm off sending `vy`=−19, and in `ida4` right inside
  the gap in the left wall.

🔑 That the middle pauses coincide with noticeable lateral correction points to
the front right wheel, the one with the shortest gearbox: the mecanum
distribution of a diagonal loads it more, and that wheel has little torque to
keep going. It is the same limit that set the working range at 70-140 rpm.
