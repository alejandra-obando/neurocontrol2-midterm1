# Car: drive and turn calibration — September 14, 2026

Long calibration session of the control on the real robot. It ends with
**forward and backward driving working well** and **the 90° turn unsolved**
(the turn was solved later, see the Sep 15 notes).

## What was left working

**Forward and backward driving.** Continuous five-second pushes, progressive
start, sustained speed and heading deviation of one to four degrees (the
session started at fifteen to thirty). It corrects well if the car is pushed
sideways.

**Firmware upload over WiFi**, without cables, in less than a minute.

**Recordable telemetry** of everything the robot does, at 50 ms, with
`raspberry_pi/record_batch.py`. It is the tool that solved almost everything in
this session: each batch is stored (see `data/calibration_batches/`) and
analyzed afterwards.

**Separation by modes.** The turn settings and the driving settings no longer
share any number, and in the interface they appear grouped and labeled.

## What was fixed, in order of importance

**The DD and TI encoders counted backwards.** The protection cut them off 0.8 s
into the first push and the next three were done with two wheels, without
anything saying so except one CSV column. Fixed in the table.

**The speed model of the four wheels was overestimated** by 22 % to 41 %. The
table said DD gave 189.4 rpm at PWM 160 and it gives 111.4. Measured again with
the car rolling, 500+ samples per wheel.

**The minimum PWM inflated by itself up to the cap** (170 on all four) and left
the control without margin to slow down any wheel — that is why it did not
correct the heading however much the gains were raised. Now the searcher does
not act while little is requested, and a blocked wheel no longer "learns" a
false take-off.

**The motion depended on twenty consecutive messages over the network.** The
page sent one every 250 ms and the ESP braked after 600 without receiving one;
a single delayed message cut the push, and on re-entering the ramp restarted.
Now the ESP sustains the push on its own with a single message.

**The control loop ran at 100 ms.** Lowered to 25, with the rpm filter rescaled
accordingly and the counters that depended on the period moved to
milliseconds.

**The rpm were computed with the loop period, not with the real time** the
encoder degrees took to accumulate. With the faster loop that put 100 % jumps
in the measurement.

**The four ToF blocked the bus for 160 ms in a row.** They are read one per
loop: the loop went from 157 ms to 52 measured, without losing refresh rate.

**Stall detection in manual mode**: if speed is requested and no wheel turns,
it brakes and warns, instead of pushing against a wall for five seconds.

**Per-wheel threshold**: none can go far beyond the PWM that corresponds to
what is asked of it. It is what removed the wheels that shot up to compensate.

## The turn: a strong lead, but WITHOUT a current measured basis

> 🔴 **Read this first.** At one point this note said the cause was "found and
> measured". **It is not.** Sebas warned at the end that **since Sep 1 motors and
> encoders have been moved around**, so the measurement of that day does NOT
> describe today's robot and does not serve as a reference. The deduction relied
> on it: it remains a hypothesis.

### The lead

The firmware table says DI = M2 and TD = M3. Test 03 of Sep 1 had paired M1 =
front right, M2 = rear right, M3 = front left, M4 = rear left — i.e. DI and TD
would be using each other's motor. **But that pairing is from before the
hardware was re-mounted.**

What does hold independently of that measurement is the **mechanism**, and it
explains today's sequence:

- A swap between two rows is **invisible when driving straight** — the four
  wheels get the same setpoint — and **evident when turning**, where left and
  right wheels get opposite ones.
- BLOCK 02 of the firmware documents that DI and TD were swapped **and** with
  inverted polarity: **two errors that cancel out in the turn row** and not in
  driving. That explains why the turn worked in the morning while driving was
  wrong.
- When driving was fixed, the pins were swapped but `invertirMotor = false` was
  left: one of the two errors was removed and the other one was left loose. The
  compensation broke.

Three failures of this session have that same signature — invisible straight,
evident when turning — including the encoder signs of DD and TI. It is not a
coincidence: it is a property of the chassis.

### Why everything has to be measured from scratch before touching anything

It is not known which motor drives each wheel **today**, nor which multiplexer
channel corresponds to each encoder **today**. Without that, any change in the
table is guessing, and this session already showed what guessing costs: the
turn sign of DD and TI was changed as a test, the four signs matched on paper,
and the robot turned worse.

**Pairing again from scratch is step 1, before anything else.** Tests 03 and 04
do exactly that and are written: they turn on one motor at a time and look at
which wheel turns and which encoder counts. (They were run on Sep 14 and their
results are documented in BLOCK 02 of `firmware/src/main.cpp`.)

## How to resume

1. Charge the battery.
2. Start: `ssh user@<pi-ip> -t 'cd ~/raspberry_pi && python3 main.py --port /dev/ttyUSB0 --runs 3 --forget'`, interface at `http://<pi-ip>:8080`.
3. First confirm that forward and backward are still good, **before** touching
   the turn.
4. One change, one batch, one decision. This session went wrong from trying
   several things at once on unconfirmed hypotheses.
