# Test 11 — how much noise the shaft tolerance adds

## What it tests

The four wheels at fixed PWM, forward, dumping the raw angle of the four
AS5600 at 100 Hz. While it runs, each shaft is moved by hand within its play:
at fixed PWM the real speed is constant, so everything that changes in the
reading comes from the sensor and the mechanics, not from the control.

Wheels in the air. Without one byte over serial every 2 seconds, the motors stop.

## Result — September 14, 2026

### The error is periodic with the revolution, not random

Splitting the error by the point of its revolution the wheel is at, a clear
dependence on the angle appears, repeatable revolution after revolution. The
front left measures almost twice the speed at one point of the revolution and
almost half at the opposite one: 52 rpm of amplitude over a mean of 55. It is
the signature of a magnet off-center with respect to the rotation axis, not of
a magnet at the wrong distance.

With the shafts still: front left 31% deviation, front right 21%, rear left
14%, rear right 9%.

Moving the shafts by hand: 40%, 26%, 15% and 9%. **The two wheels whose
mounting was already good barely change**; the defect of the other two is
there whether they are touched or not.

### It cancels itself when averaging a full revolution

An error that is a function of the angular position disappears when
integrating a full revolution. Measured, by window size:

| Window | front left | front right | rear left | rear right |
|---|---|---|---|---|
| 25 ms | 36% | 25% | 13% | 7.5% |
| 250 ms | 22% | 21% | 8% | 3.9% |
| 1000 ms | 6.5% | 17% | 2.8% | 2.1% |

Three of the four drop to 2-7%. **The front right stays at 17%: its error is
not periodic, so averaging does not fix it.**

🔑 **What this means for the design:** the distance traveled and the
accumulated angle — the odometry, and what would feed an estimator — do not
suffer from the off-center magnet error. The one that suffers is the speed
loop, which looks at the instantaneous speed and chases an error that does not
exist.

### The AGC is not the indicator

The AGC says how far the magnet is from the chip, not whether it is centered.
The rear right has its AGC stuck at 128, the top of the range, and it is the
one that measures best of the four. Do not use the AGC to judge the quality of
the measurement: that is what the per-sector error computed by this test is for.

### Centering the magnet by hand did not help

An attempt to center the front left left its per-sector error at 51.5 against
51.1 before. The scale one has to hit is tenths of a millimeter.

## What was tried with this data and did NOT work

Lowering the firmware rpm filter from 0.12 to 0.06. Offline the data said it
was better — it left the worst wheel at 7.3% instead of 12.1% — but on the
robot it does not hold: in steady state the rear right gets worse from 4.4% to
7.5%, and at the start the peaks rise from +27% to +40%. Measuring the signal
in open loop tells how much noise it has, but not what the delay costs inside
the loop. See the ALFA block in `firmware/src/main.cpp`.
