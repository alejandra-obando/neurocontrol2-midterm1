# 2026-09-15 16:05 — runs the corridor turning towards the free side

Six stretches and six turns in a row. **The turns came out better than in any
other run of the session: mean error −0.44 degrees**, worst +3.04.
Script: `../back_and_forth.py`.

## What it does

It advances along the corridor, stops when it sees the wall, and **at each
corner it chooses where to turn: towards the side that is free**. The half turn
is reserved for the dead end, when both sides are closed.

In this run it chose **R · R · R · L · L · R**, without a single half turn
because it never found a dead end.

🔑 **It is decided with `LIMITE_LATERAL` (140 mm), which was already the
threshold this script used to tell a corridor wall from a side opening.** A
side reading above that is not a wall: it is room to keep going. No new
threshold had to be invented.

🔴 **WHY IT CANNOT BE A FIXED HALF TURN.** The turn sweeps 263 mm and the
corridor measures 250, so the 180 does not fit: with a fixed half turn the robot
turned even when it had an open corridor to the side and scraped the wall.
Measured on the 16:00 run, at the end of the six stretches five had one side
clearly free — between 247 and 583 mm — against the other with a wall, between
50 and 110. Only one was a dead end.

## The two other things it brought

**Cruise speed 112**, 10% over the 102 with which the square circuit was
closed. The braking anticipation scales with the speed by itself, so the stop
did not move: the six ended at **36, 44, 12, 16, 51 and 23 mm** with the target
at 41, and none touched the wall.

**Softer lateral correction**: `BANDA_LAT` from 5 to 10 mm and `ERR_PLENO` from
18 to 30.

🔴 **What is lowered is the NUMBER of taps, not their force** — below 40 the
wheels do not break free and the robot does not move, that is measured. The
reason is measured too: **each lateral tap twists the heading 0.46 degrees in
median, but the worst 10% exceeds 5.85 and the maximum was 13.73.** With the
band at 5 and the typical error at 9.1 mm almost every cycle fired a correction
— 58 taps in 151 cycles — and the robot kept leaning and recovering along the
whole stretch; that is where the 5-degree heading drifts *within* a stretch
came from. With 10 and 30 the taps dropped from 50-55% to 5-25%.

Safety does not change: `MIN_LATERAL` still moves it away from a wall coming too
close, and that goes before everything else.

## 🔴 What breaks in open field

In stretches 4, 5 and 6 **the robot went blind on both sides** — the log marks
"ciego" (blind) 31%, 7% and 33% of the time, with the side sensors between 300
and 600 mm — and there the heading drifted **+9.19, −11.76 and +5.52 degrees**.
In stretch 6 the alignment reached 25.6 degrees of diagonal.

**With no wall in sight there is nothing to center against and only the
gyroscope is left.** While there is a corridor it goes well; as soon as it goes
out into open field it leans.

## Lineage

It comes from `14_square_circuit/best_right_2026-09-15_1547/`, which closed two
laps of the square with 0.21 degrees of error. From that file it keeps
untouched the braking thresholds, the lateral setpoints, the turn tolerance and
braking, the gains and the heading compensation.
