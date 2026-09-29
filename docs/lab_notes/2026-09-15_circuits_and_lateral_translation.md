# Car — closed circuits and calibration of the sideways translation

2026-09-15, afternoon. Continuation of `2026-09-15_tof_calibration.md`.

## What is closed

**Three route experiments, all three working**, and the sideways translation
measured and explained. Everything from the Pi with scripts; in the firmware
only one new, additive command was added.

### 1. Square circuit in both directions

- **Left direction** (turn `L`): `bench_tests/14_square_circuit/best_2026-09-15_1433/`
- **Right direction** (turn `R`): `bench_tests/14_square_circuit/best_right_2026-09-15_1547/`
  — **two laps with 0.21° of heading error**, the best of the session.

Between the two files only three constants change: `GIRO`, `PASO_GIRO` and the
log name. **The 46 control constants are identical.**

🔴 **And one code change that took a whole run to discover: the re-centering
after the turn cannot assume which side the wall is on.** Written against the
right sensor, when running the circuit in the other direction it did not find
its wall and **was not executed even once**: in the seven stretches the right
sensor read between 551 and 566 — i.e. nothing — and the left one between 39
and 75. The robot started each stretch where the turn left it and arrived off
center at the next one, and there **the turn scraped the wall**, because it
sweeps 263 mm and the corridor is 250. From outside it looked like oscillation
in the turns, and the turns had nothing wrong: the six of that run gave between
−90.3 and −92.2 in less than three seconds, without a single bounce.

🔑 The setpoints do not change because of that: `CONSIGNA_E` 52.5 and
`CONSIGNA_O` 43.0 add up to the measured gap of 95.5, i.e. **they describe the
same spot seen from each side**.

### 2. Route choosing the free side

`bench_tests/15_back_and_forth_180/best_2026-09-15_1610/`. Nine stretches and
nine turns, with the cap at 9 and a manual stop with Ctrl+C.

**Final heading −450.42 against −450: 0.42° of error. Mean error per turn
−0.18**, the best of the whole session. It chose R·R·R·R·R·L·L·R·R.

🔑 **At each corner it turns towards the side that is free**, decided with
`LIMITE_LATERAL` (140 mm), which was already the threshold the script used to
tell a wall from a side opening. **The half turn is reserved for the dead
end**, because the 180 does not fit in the corridor.

🔴 **One turn went wrong and the run held**: the sixth gave +98.96° and took
21.4 s — the firmware retrying — and **the compensation of the next stretch
absorbed the 9.69° it carried**. It is the proof that the absolute heading
scheme tolerates a whole bad turn.

🔴 **What is still broken: in open field there is nothing to center against.**
Stretches 6, 8 and 9 spent between 14% and 35% of the time without seeing any
wall, and there the heading drifted up to 11.45°.

### 3. Sideways translation

`bench_tests/16_lateral_translation/` with its README. The result in one line:
**the translation has its own minimum speed, and below vy 120 the robot does
not translate, it twists.**

At vy 40 the four wheels differ by 41% and the robot turns 8.5° in a second
and a half; from vy 120 to 160, the difference drops to 11-14% and it turns
less than half a degree. The cause is the front right wheel and its different
gearbox, already measured in test 12 of Sep 14.

## What was added to the firmware

🔑 **A new command, `'A'` (CMD_HASTA_PARED): it advances and brakes only when it
sees the wall, without the cell limit.** `CMD_AVANZAR` and the maze demo stay
untouched.

**Why it exists:** the script closed from the Pi a loop the ESP can close on its
own. Between the ToF filter and the network, the Pi decided the braking with
data up to 1.3 s old and the robot ended up touching the wall — stops of 1 to
4 mm with a target of 41. The ESP reads the front sensor **directly, without
the filter, every 40 ms** and brakes at `DIST_STOP_N` = 71.1 mm. Tested alone:
it advanced 47 cm and stopped at 52 published mm, with the network at 0.8 s per
loop and without that mattering.

**It is in the firmware and in the interface, but the circuit scripts do not
use it yet**: when the stretch was going to be built on it, the run was stopped
to go back to the validated file. It is the natural next step. (It was later
used by the maze navigator to move closer to the wall at crossings.)

## Interface

**The manual control now translates sideways**, with the two new buttons and
the A/D keys, and **it goes up to vy 120 by itself** when translation is
requested even if the slider is lower. It also has the advance-to-wall button.

## 🔴 HOW TO RESUME

### 1. Apply the translation findings to the circuit

`VY_MIN_EFECT` (40) and `VY_PEGAR` (65) are **the two worst points of the
table**. That explains what was measured in the circuit without understanding
it: each lateral tap twisted the heading 0.46° in median, with a maximum of
13.73°. Raise them to 120-140 and **shorten the pulses proportionally** so it
covers the same millimeters.

### 2. Move the advance of the circuits to the `'A'` command

The front stop stops depending on the network. `tramo()` has to be rewritten to
send `'A'`, follow the `estado` (1 = advancing, 0 = stopped) and put the
lateral pulses between advances. 🔴 **Follow the `estado`, not the `ev`:** the
event is published only once and with this network it is easy to miss.

### 3. What is not about control

- **The take-off PWMs have been against the 170 ceiling all afternoon** and they
  are saved in flash, so they survive the reset: resting the motors does not
  lower them. They are reset with the `B` command.
- **The heading is limited by temperature**: with the same code, the mean error
  per turn was +0.77° cold and +1.92° launching eight minutes later.

## What should NOT be tried again

🔴 **The AGC does not judge the quality of an encoder.** It says the distance of
the magnet to the chip, not whether it is centered; test 11 already said so and
this session fell for it again. The good indicator is the per-sector error of
test 11.

🔴 **Anticipating the braking by measuring the speed with the ToF does not
work.** The sensor refreshes every 160 ms and the loop reads every 220,
unsynchronized: two consecutive readings often fall in the same refresh and
give zero. It is computed with the **requested** speed, which is exact, and the
sensor can only increase the margin.

🔴 **Inflating the braking margin when the network is slow does not work
either.** A one-second peak asked for 300 mm of margin and the robot stopped
halfway to the wall.

🔴 **When killing the Pi server, do it by the PID of the Python process.**
`pgrep -f` catches the bash wrapper and ssh itself. In this session that led to
concluding that 10 out of 11 telemetry lines were lost when none was.

## State at the end

Robot stopped, motors enabled, server up. The Pi's `config.py` restored byte by
byte against the validated base of Sep 14. Firmware with the `'A'` command
added, everything else intact. **The Pi–ESP link verified healthy**: the ESP
sends 16-18 telemetry lines/s with the correct 56 fields and the Pi receives
all of them.
