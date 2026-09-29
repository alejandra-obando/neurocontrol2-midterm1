# Test 12 — PWM to rpm curve of each wheel

## What it tests

It makes the four wheels turn at fixed PWM, without the control loop, and
steps up and down in 2-second steps from 0 to 255 and back. At each step it
measures the speed and the noise of each encoder. With the PID in between, the
PWM one sees is the one the loop decided, not the one the wheel needs; here the
PWM is imposed and one looks at what the wheel does.

Wheels in the air, motors connected, power on. The dump runs at 921600 baud:
at 115200 each line blocks for about 4 ms and samples are lost in the high
steps, which is where the wheel turns the most between samples.

## Result — 3 full sweeps, September 14, 2026

### What each wheel needs

| Wheel | Starts from standstill | Keeps turning down to | Top at PWM 255 |
|---|---|---|---|
| front left | PWM 100 (100, 100, 100) | PWM 40 | 171 rpm |
| front right | **PWM 143 (165, 165, 100)** | PWM 75 | 365 rpm |
| rear left | PWM 93 (90, 95, 95) | PWM 35 | 160 rpm |
| rear right | PWM 85 (85, 85, 85) | PWM 35 | 175 rpm |

In parentheses, what each of the three runs gave. Three wheels always break
free with the same PWM; **the front right needed 165 in two runs and 100 in the
other**, so not even its start is repeatable.

Starting costs a lot more than keeping going: the rear left needs 90 to break
free and keeps turning with 35. That is why a start without an initial pulse
leaves some wheels stopped while others are already turning.

### The front right start leaves no common minimum

When breaking free, each wheel jumps to a very different speed: the front left
to 57 rpm, the rear left to 43, the rear right to 45, and **the front right to
114**. In the worst measured case (PWM 165) that wheel comes out at 213 rpm,
**above the 160 that is the ceiling of the rear left**.

🔴 That is: in a bad run, the PWM needed for the front right to start takes that
wheel out of the range the other three can follow. There is no guaranteed
common minimum speed while that wheel is like this, and it is not a problem
that is fixed by choosing the setpoint well.

The software way out is the start pulse: give that wheel a high PWM (165) for
two or three tenths of a second to break free, and as soon as it breaks free,
bring it down to its line. Keeping going costs it much less than starting.
With that pulse, the common minimum is set by where its reading becomes
repeatable again, which is PWM 100, about 90 rpm.

### The line of each wheel

Fitted on the down sweep, discarding the steps that do not repeat between runs:

| Wheel | PWM as a function of rpm | Valid between | Maximum error |
|---|---|---|---|
| front left | `PWM = 22.6 + 1.369 × rpm` | 15 to 158 rpm | 4 PWM |
| front right | `PWM = 53.4 + 0.533 × rpm` | 70 to 325 rpm | 13 PWM |
| rear left | `PWM = 20.4 + 1.468 × rpm` | 18 to 148 rpm | 3 PWM |
| rear right | `PWM = 20.9 + 1.362 × rpm` | 17 to 162 rpm | 14 PWM |

🔴 **The line does NOT go through the origin, and the firmware assumed it did.**
`pwmDelModelo` computed `160 × rpm / rpmA160`, a line through zero. The
measurement says there is a jump of 20 to 53 PWM points before the wheel starts
moving. That jump is exactly what the PID has to invent at every start, and it
is the reason it is aggressive: it starts from a PWM it knows is too short.
With the full line the starting point is right and the loop only corrects what
is left. (The firmware now uses these lines — see BLOCK 07 of
`firmware/src/main.cpp`.)

### The encoder noise is proportional to the speed

It is not a number of rpm: it is a percentage, and it stays the same at every
speed, which is the signature of an off-center magnet.

| Wheel | Noise |
|---|---|
| front left | 36% of the speed |
| rear left | 14% |
| rear right | 7% |
| front right | 5% |

**The front right has the CLEANEST encoder of the four.** What fails on that
wheel is mechanical, not measurement.

### The two limits of the car

**Maximum speed with the four turning: about 150 rpm.** It is set by the rear
left, which at PWM 255 reaches 160 and by its line already asks for 255 for
160. With a control margin, 140 rpm is the healthy ceiling.

**Minimum speed with the four turning: about 90 rpm, and it is set by the front
right.** The other three go down to 15-18 rpm without problems. The front right
does not give a repeatable figure below PWM 100: at PWM 75 it measures 7.8 rpm
with a spread of 11 between runs, at 80 it measures 14.8 with a spread of 21.
Sometimes it turns and sometimes it does not.

🔴 **So today the useful range of the car is 90 to 150 rpm**, and a single wheel
narrows it. If that friction is removed from the front right, the range
becomes 15 to 150.

## How to run it

Flash the project and capture the dump at 921600 while sending one byte per
second; without that byte the motors stop by themselves after 2 seconds. A
full sweep takes about 2 minutes.

## The driving strategy that comes out of this

Decided on Sep 14 with this data at hand.

**Start all of them at the speed at which the front right breaks free — about
114 rpm — and brake progressively afterwards.** It takes advantage of the fact
that on that wheel breaking free and keeping going cost very different things:
breaking free asks for up to PWM 165, and once turning it keeps going with much
less.

How far one can brake, measured in the down sweep, which is exactly that
experiment:

| PWM | front right rpm | spread between runs |
|---|---|---|
| 95 | 70 | 19% — reliable |
| 90 | 56 | 25% — at the limit |
| 85 | 41 | 36% — it turns in some runs and not in others |
| 80 | 15 | 141% — no |

**Driving range: 70 to 140 rpm**, always starting from above. Stretching down
to 56 rpm is possible, accepting that a wheel will occasionally stall.

## 🔴 What is missing: measuring this under load

Everything above is measured **with the wheels in the air**, which is the
easiest case there is: no weight on top, no friction against the floor and no
inertia of the car. With the robot on the floor the four numbers move, and not
uniformly — the start PWM goes up, the speed each PWM gives goes down, and the
slope of the lines changes because the resisting torque is different. The
wheel that will cope worst is the front right, which already starts with plenty
of friction.

**None of these numbers should be taken as valid for the loaded robot until the
measurement is repeated on the floor.** (The loaded lines were later measured
on Sep 14 — see BLOCK 07 of the firmware.)

The sweep as it is does not work on the floor because the car drives away.
Another setup is needed: short pushes at fixed PWM along a straight stretch of
a couple of meters, measuring the speed of the four wheels, in steps. This
firmware can be reused by shortening the steps and limiting the range to the
PWMs we already know are of interest, 75 to 180.
