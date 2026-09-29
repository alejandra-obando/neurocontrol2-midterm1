# Square circuit — running a corridor maze turning at the corners

The robot advances along the corridor, stops when it sees the wall ahead,
turns −90 (to the left), sticks to the right wall and continues. Everything
from the Pi with `square.py`, without touching the firmware.

## 🔴 THE TWO CLOSED VERSIONS

- **LEFT direction (turn `L`)** → `square.py`, run of `best_2026-09-15_1433/`
- **RIGHT direction (turn `R`)** → `square_right.py`, run of
  `best_right_2026-09-15_1547/`, **the best heading of the whole session: 0.21
  degrees of error over two laps.**

Between the two only three constants change (`GIRO`, `PASO_GIRO` and the log
name) plus the re-centering after the turn, which **sticks to the wall it sees
instead of assuming it is the right one**. Written against a fixed side, in the
other direction it was not executed even once and the robot arrived off center
at every turn.

Each `best_*` folder keeps the README and the log (`.jsonl`) of the run that was
promoted at that moment; the intermediate copies of the script are not part of
this repository — only the two final scripts are.

## `best_2026-09-15_1433/` — the left direction

Everything from the afternoon — fine re-centering after the turn, soft start,
heading compensation with enough budget, correction while moving only by taps,
dynamic lateral target and a safety limit on the left — plus **a front braking
that no longer smashes the robot into the wall**: the eight stops fall between
12 and 84 mm with the target at 41, none touches.

🔑 **The braking anticipation is computed with the speed the script has just
requested, not by measuring it with the sensor.** The front ToF refreshes every
160 ms and the loop reads every 220: two consecutive readings often fall in the
same refresh and give zero, and then the robot brakes without any margin. Its
`README.md` has it measured.

🔴 **What is still pending is not about control: it is thermal.** With the same
turn code, the mean error was **+0.77 degrees with the motors cold** and
**+1.92 when launching eight minutes after the previous run**. When cold, the
degradation between the first and second lap almost disappears. The take-off
PWMs always end up against the 170 ceiling and **are saved in the ESP32 memory,
so they survive the reset**: letting the motors rest does not cool them down.
**That is the next thing.**

## `best_2026-09-15_1244/` — the one with the best heading (left direction)

Two full laps, eight stretches and eight turns without a failure, and **the
heading ended at +720.51 degrees where it should read 720**. Its `README.md` has
the figures and the full mechanism. **It is the point to go back to.**

`square.py` in this folder is the working copy: it is tested here and, when a
batch confirms it goes better, it is promoted to a new folder with its README.

## The other copies, in order

- **`best_2026-09-15_1433/`** — THE CURRENT ONE. The only one without any stop
  against the wall. Final heading +726.13, the worst of the afternoon, with the
  motors already hot.
- **`best_2026-09-15_1403/`** — fine re-centering after the turn and soft start.
  Final heading +724.69, with a flawless first lap (+0.05) and a degraded
  second one (+2.30).
- **`best_2026-09-15_1244/`** — THE BEST HEADING. Two laps, half a degree of
  error at the end, and the error stops accumulating from one lap to the next.
- **`best_2026-09-15_1233/`** — the first one with absolute heading. It closed
  with 5.8 degrees of deviation, but its compensation oscillated with growing
  amplitude because it computed the pulse blindly.
- **`best_2026-09-15_1223/`** — the first one with centering by taps, which is
  what removed the abruptness. No heading compensation yet: it closed with
  7.27 degrees.
- **`DISCARDED_started_crooked_1227.jsonl`** — useless, the robot started tilted.

## What was hard to discover, and should not be tried again

🔴 **Tightening the turn tolerance does NOT remove the bias, it only adds
bounces.** With `tolgiro` at 3 the firmware retries the whole turn: one took
13.1 s and its heading bounced four times (93.4 → 80.5 → 99.3 → 87.5 → 97.0),
ending 6 degrees past. It stays at 4.4, which is what was validated on Sep 14.

🔴 **The firmware jumps to W_ARRANQUE (36 degrees/s) as soon as any `w` is
requested**, because otherwise the short-gearbox wheel does not break free.
That is why the duration of a heading correction pulse cannot be computed
blindly: the loop has to be closed on `yaw_ref`, which travels in the telemetry.

🔴 **A small `vy` does not move the robot.** Below 40 out of 255 the wheels do
not break free, so the loop asked to correct and the robot kept going straight,
stuck to the wall. The force stays at the executable minimum and **what is
modulated is how often it is applied**, not how hard. The same holds for the
stopped re-centering and for the heading pulses.

🔴 **When a lateral pulse is cut the robot keeps sliding ~17 mm**, always
towards the same side. It is anticipated, not chased.

🔴 **Send pushes (`#t=`), never pulses (`#m=`).** The pulse carries a 600 ms dead
man's switch and a loop from the Pi takes ~230 ms per iteration with peaks that
exceed it: with pulses the robot went in jerks at 0.023 m/s against 0.12
requested.

🔴 **Do not retry commands that are not idempotent.** Retrying a reading is
harmless; retrying a turn leaves another one queued on the ESP and it is
executed later, in the middle of a straight stretch.

🔴 **A side sensor reading above 140 mm is not the corridor wall**, it is a side
opening. Believing it, the loop computed 526 mm of offset and sent the
translation to the cap, dragging 15.6 degrees of heading in two seconds.
