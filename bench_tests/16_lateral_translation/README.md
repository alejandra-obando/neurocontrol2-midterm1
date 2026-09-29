# Test 16 — sideways translation: at what speed the four wheels go the same

## What it tests

The same idea as test 12, applied to the Y axis. It raises the translation
speed in steps, alternating sides so it does not drift away, and at each step
measures at what rpm each wheel goes, how much it wobbles and how much the
robot twists.

Unlike tests 11 and 12, this one is run **with the robot on the floor and with
its weight**, because the mecanum rollers do not roll the same sideways as
forward.

    python3 lateral_sweep.py   # the step sweep
    python3 lateral.py         # a fixed translation, to look at one speed

## Result — September 15, 2026

### The faster it translates, the more even the four go

| vy | difference between fastest and slowest | twist |
|---|---|---|
| 40 | **41.4%** | **−8.48°** |
| 60 | 18.7% | −4.37° |
| 80 | 35.6% | +2.13° |
| 100 | 21.5% | +0.36° |
| 120 | 14.5% | −0.45° |
| 140 | 13.4% | +0.22° |
| 160 | 11.4% | +0.47° |
| 180 | 6.4% | −2.02° |

🔑 **The heading follows exactly the evenness between wheels.** At vy 40 the
robot twists eight and a half degrees in a second and a half; from vy 100 to
160 it twists less than half a degree. There is no need to correct the
heading: one has to translate at a speed where the wheels can go the same.

**The good zone is vy 120-160.** At 180 the wheels go even more evenly but the
heading gets worse and the noise shoots up, so it is not worth it.

### Why: the front right has a different gearbox

It is the slowest in all eight steps, without exception, and the difference
narrows as the speed goes up:

| vy | DI | **DD** | TI | TD |
|---|---|---|---|---|
| 40 | 13.7 | **8.0** | 9.2 | 10.9 |
| 80 | 22.5 | **16.1** | 24.4 | 25.0 |
| 120 | 40.0 | **34.3** | 38.8 | 40.2 |
| 160 | 52.0 | **46.3** | 50.1 | 52.3 |

(DI/DD/TI/TD = front-left / front-right / rear-left / rear-right.)

**It is not a fixed bias: at low speed it is outside its repeatable range.**
Test 12 of Sep 14 already measured it: that wheel reaches 365 rpm where the
other three stay at 160-175, its line is `PWM = 53.4 + 0.533 × rpm` (slope less
than half that of the others) and **its reading only becomes repeatable again
above PWM 100, about 90 rpm**. Asking for a slow translation is not going
slowly: it is going crooked.

### What is NOT the problem

**The kinematic distribution is right.** The targets the firmware asks of the
four add up to exactly zero in both directions, which is the condition for a
clean translation. What fails is that the wheels do not follow that target:
they fall short by 1.5 to 7.7 rpm, and their errors do not cancel out.

**The X pattern is right.** Correct signs in both directions and magnitudes at
91-95% between the weakest and the strongest at working speed.

**The four start at the same time.** Measured with the Pi log raised to 20 ms:
a lag of 0 ms between the first and the last to break free, in both starts,
with each one going out with its full take-off PWM — 169, 167, 170 and 170
against take-offs of 165, 163, 167 and 169. The X-axis start mechanism acts the
same on the Y axis: the ramp limits `pedidoVx` and `pedidoVy` with the same
`ACEL_MAX` and the take-off floor is applied per wheel regardless of the axis.

**The Pi–ESP link is healthy.** Reading the cable raw: 96 of 97 lines with the
exact 56 fields, at 16-18 telemetry lines per second with a median interval of
52 ms, and the cable at 44% of its capacity. The Pi receives all of them (16.7
rows/s with the tuned log).

## 🔑 How this is applied

**Any lateral correction below vy 120 twists the robot more than it centers
it.** The square circuit corrected at vy 40 while moving (`VY_MIN_EFECT`) and at
vy 65 while stopped (`VY_PEGAR`): the two worst points of the table. That
explains what was measured there without understanding it — **each lateral tap
twisted the heading 0.46° in median, with the worst 10% above 5.85° and a
maximum of 13.73°**.

When raising those two values to the 120-140 zone, the pulses have to be
**shortened proportionally**, so the robot covers the same millimeters.

## What was tried and must be remembered

🔴 **The AGC is not useful to judge the quality of an encoder.** It says how far
the magnet is from the chip, not whether it is centered. Test 11 already said
so: the rear right has its AGC at the top of the range and it is the one that
measures best. During this session the front right problem was attributed to
its AGC of 2, and it was the wrong explanation — the right one is its gearbox
and its non-repeatable range.

🔴 **When killing the Pi server it has to be done by the PID of the Python
process.** `pgrep -f` also catches the bash wrapper and the ssh command itself,
so a `kill` that seems to work can leave the server alive. In this session that
led to measuring "10 out of 11 telemetry lines are lost" when none was lost:
the CSV was still being written by the old server with its half-second period.

🔑 **`REGISTRO_PERIODO_S` in `raspberry_pi/config.py` governs the resolution of
the log.** It is at 0.5 s for normal use; lowering it to 0.02 captures
everything that arrives (about 60 real ms) and it is what allows measuring
starts and lags. Put it back afterwards.
