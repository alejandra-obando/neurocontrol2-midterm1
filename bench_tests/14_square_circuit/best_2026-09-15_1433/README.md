# 2026-09-15 14:33 — current version: the braking stops smashing into the wall

Two full laps of the square, eight stretches and eight turns, without any
interruption. Script: `../square.py` (left direction).

## The front stop no longer has a dangerous tail

The eight stops, in what the front sensor publishes, against a target of 41:
**12, 39, 36, 46, 32, 51, 84 and 67**. Mean 45.9, i.e. almost on the target.
Only one falls below 20 mm and none touches the wall.

🔑 **The braking anticipation is computed with the speed the script itself has
just requested, not by measuring it with the sensor.** It is the number the
loop decides on that same iteration: exact and noise-free. The front ToF can
only *increase* the margin if it sees the robot approaching faster than
requested, never reduce it.

🔴 **Why it cannot be measured with the sensor.** The ToF refreshes every 160 ms
and the loop reads every 220, unsynchronized: two consecutive readings often
fall within the same refresh and give **zero**. The robot really approaches at
120 mm/s and the estimate between two readings swept from 0 to 295, so the
anticipation margin jumped between 0 and 103 mm **within the same stretch**.
And below 100 mm the sensor stops refreshing normally: it gives large jumps
followed by flat plateaus, exactly where the decision matters. Measured on the
14:25 run: fifteen of its 159 samples gave zero, and the two stretches that
ended up touching the wall are exactly those that fired with the estimate at
zero (margin 0 mm) or nearly (9 mm).

The constant that translates requested speed into approach is **1.5 published
mm per second and per unit of vx**, measured in the range where the stop fires
(vx from 45 to 85, where it came out between 1.45 and 1.58).

## 🔴 What this brings along: it leaves farther from the right wall

Stopping earlier leaves the robot farther from what will be the right wall
after turning. It comes out of the turn at **60, 73, 77, 80, 71, 84, 113 and
106 mm** from the right, mean 83, against a mean of 67 in the previous run. The
re-centering recovers them — it ends at 59, 54, 55, 36, 53, 36, 51 and 47
against the setpoint of 52 — but it takes up to 3.1 s and twice it overshoots
down to 36.

**It is the geometry of the corner, not a centering failure.** If the stopping
threshold is lowered again, this corrects itself.

## 🔴 What is NOT solved: the heading, and it is thermal

The eight turns: **+93.95 · +89.66 · +92.50 · +91.64 · +91.96 · +92.60 ·
+89.61 · +93.44**. Mean error **+1.92 degrees**, worst +3.95. The heading ends
at **+726.13 against the +720** it should: 6.13 degrees over two laps.

**It is the worst of the afternoon in heading, and the cause is measured.**
With the same turn code, without touching a single constant, the mean error per
run was:

- **14:25, cold motors after a rest**: +0.77 (first lap +0.36, second +1.19)
- **14:03, hot motors**: +1.17 (first lap +0.05, second +2.29)
- **14:33, this one, launched eight minutes after the previous one**: +1.92
  (first lap +1.94, second +1.90)

🔑 **When cold the error is halved and the degradation between laps almost
disappears.** This run started with the motors already hot from the 14:25 one,
and that is why its first lap is as bad as the second: there is no degradation
*within* the run because it already came degraded.

The take-off PWMs are still against the 170 ceiling — it started at
169/169/165/165 and ended at 167/165/161/165 — and **they are saved in the
ESP32 memory, so they survive the reset**: letting the motors rest does not
cool them down. That is the next thread.

## Nothing of the turn is touched

`tolgiro` 4.4 and `gradfreno` 45, the ones validated on the floor on Sep 14.
The lateral setpoints are not touched either: right 52.5 and free gap 95.5.

## Comparison of the afternoon final headings

- **12:44** → +720.51 (0.51 degrees over two laps, the best heading of the session)
- **14:03** → +724.69
- **14:25** → +721.77, cold
- **14:33 (this one)** → +726.13, but the only one without any stop against the wall
