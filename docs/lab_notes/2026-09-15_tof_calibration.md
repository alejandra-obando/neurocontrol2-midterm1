# Car — calibration of the four ToF sensors and a check batch

2026-09-15

## State of the system

Pi code and ESP32 firmware identical to the Sep 14 copy
(`2026-09-14_final_marcha_y_giro`), verified by hash. Parameters in force on
the ESP equal to `PARAMS_CALIBRADOS`: kp 0.40 · ki 0.10 · cruise 0.22 · vgiro
60 · vgiromin 25 · tolgiro 4.4 · odom 1.09 · vmanual 0.30.

Pi service: `ssh user@<pi-ip>`, `cd ~/raspberry_pi`,
`setsid nohup python3 main.py --port /dev/ttyUSB0 --web-port 8080 &`.
API at `<pi-ip>:8080`.

## Driving and turning batch

Robot on the floor, 5 s push at 0.22 m/s in each direction (`manual_pwm` with
vx ±187, which is 0.22 of the 0.30 m/s control cap) and 90-degree turns with
the `L` and `R` primitives.

- Forward: drift +0.49°. Backward: +0.24°. There and back close at +0.73° from
  the starting heading. The four wheels between 71 and 76 rpm against 77
  requested, none cut off. Equivalent to what was measured on Sep 14.
- Left turn +91.83° in 3.0 s; right turn −92.00° in 4.2 s. Errors of 1.8 and
  2.0°, within the 4.4 tolerance the firmware accepts, but worse than the −0.6
  to +1.4 range of Sep 14, and the right one took one second more than the 3.2
  of then.

## 🔴 Pending: the take-off PWMs

At power-up, the ESP had the four take-offs at 167, 167, 169 and 169, against
the `DESPEGUE_MAX` = 170 ceiling. The firmware learns and saves them by itself,
and four values stuck at the cap mean that in some run the wheels did not break
free. They were put back to the table with `B` (95, 83, 93, 39) and, after the
batch, they rose to 129, 111, 123 and 71.

**They are halfway through re-learning**, and the rear right — 71 against the
151 it closed with on Sep 14 — is the farthest away. It is the explanation for
the slower turn and the two degrees of error. **It is closed by running another
batch or two of driving and turns and checking that the four rise towards 113,
121, 127 and 151.**

## 🔴 The Pi is in permanent undervoltage

`vcgencmd get_throttled` returns `0x50005`: under voltage right now and
throttling, with twelve warnings in the journal since the battery was
connected. The ESP's USB reset once during boot. It is the probable cause of
the take-offs ending up against the ceiling.

## ToF calibration: closed between 150 and 350 mm

Full details, method and warnings in `bench_tests/13_tof_calibration/README.md`;
raw data in `points.csv`.

The four lines, `read = c + m · real` in millimeters:

- N front: 46.7 + 0.890 · real
- E right: 0.0 + 1.040 · real
- S back: 15.6 + 1.032 · real
- O (west) left: 24.9 + 1.043 · real

Three sensors share the slope (3.8 % of long scale, from the batch) and are
distinguished only by the offset. The front one is outside the group in both
and **its mounting has to be checked** before taking the correction as good.

**Before applying it**: the wall thresholds already compensate part of the
bias by hand and they have to be translated (N 284.6 · E 266.3 · S 275.6 · O
313.6) or the wall detection changes; the 600 cap means "I see nothing" and is
not corrected; and the maze network was trained with the uncorrected
distances.


## Correction applied to the firmware

New base copy: `2026-09-15_tof_corregidos`, which is the Sep 14 one with the
ToF correcting and **no driving or turning constant touched**. Uploaded over
WiFi (`pio run -e wifi -t upload`) and running.

`corregirToF()` in BLOCK 04b, applied in the three places where the firmware
reads a ToF, including the dedicated front reading inside the advance, which
does not go through the rotating sampling. All thresholds translated to their
real equivalent so that no wall decision changes; `UMBRAL_FUERA` is the
exception and carries the average of the four. `MAX_DIST` is not corrected.

## Back and forth in a corridor: five passes

Corridor of 154 x 25 cm, 6 cm on each side, open ahead. It advances until it
sees 60 mm, stops, reverses. Centering with the two side sensors by lateral
translation, without turning. Script in `bench_tests/13_tof_calibration/corridor.py`.

**The ten passes complete, all stopping by sensor.** Speed of 0.121 to 0.130
m/s against 0.12 requested, with 39 wheel rpm against 40 theoretical. Heading:
the drifts per stretch range from -2.13 to +1.82 degrees but **the total after
ten passes is 1.07 degrees**, i.e. it does not accumulate.

### What has to be fixed

🔴 **The centering oscillates in a limit cycle**, from +16 to -24 mm, mean of 9
to 17 per pass. It is the delay: each ToF refreshes every 160 ms and the
firmware publishes the mean of its last 5, about 800 ms. A pure proportional
controller against that delay always oscillates. The delay has to be predicted
or the loop closed against the heading.

🔴 **The stop overshoots by about 33 mm and its spread is 29**, almost as much as
its value: it detects at 61-67 mm and ends between 16 and 45. One has to
anticipate, not lower the threshold.

🔴 **A ToF that regains sight lies for almost a second**: after a stretch at 600
it publishes a false ramp (525, 421, 313, 203, 109) which is the average filter
dragging the 600. A 5-sample quarantine is already in `corridor.py`; **the
firmware does not have it**.

### The half-second pauses

26 samples out of 417 while moving with the wheels below 12 rpm, 6.2 %. Ten
are the start of each stretch (inertia, with the take-offs halfway through
re-learning). The other three happen while correcting sideways, and they point
to the front right wheel, the one with the short gearbox.

## Confirmed

**The corridor has a gap in the left wall**, which is what blinded the left
sensor on outbound pass 4. It is not a sensor failure.

## Next

Another experiment in the corridor, to be defined. Before that it is worth
measuring the short range of the front sensor (60 and 100 mm), which is the
one that decides the stops and the only one without points there.

## Square circuit: two laps with the heading under control

Everything in `bench_tests/14_square_circuit/`, and **the current version is
`best_2026-09-15_1433/`**; the one with the best heading is still
`best_2026-09-15_1244/`. The robot advances along the corridor, stops when it
sees the wall, turns −90 to the left, sticks to the right and continues. It
runs from the Pi with `square.py`; the firmware is not touched. **The
experiment in the left direction is closed.**

🔑 **Two full laps, eight stretches and eight turns without a failure, and the
heading ended at +720.51 degrees where it should read 720.** What matters is
not that half degree but that **the error stops accumulating**: what was left
over after each turn was +0.68, −1.02, −0.89, −0.32, −0.28, +0.76 and +0.83,
oscillating within one degree lap after lap. With independent turns, four
turns already added up to 7 or 8 degrees and kept growing.

Mean turn error **+0.28 degrees** (it was +1.47 uncompensated). Stops between
54 and 84 mm with a threshold of 95. Re-centering between 2 and 19 mm from the
7 cm target.

### How it is achieved

**The heading is tracked in absolute terms** (target n × 90 from zero) and
whatever is left over from a turn is discounted in the next stretch,
**correcting it while rolling and not by turning while stopped** — turning
while stopped was discarded on Sep 2 because the smallest motion of the car is
larger than the error. It takes advantage of the firmware moving its own
reference with the turn rate it is sent: `yawRef += pedidoW * 180/PI * dt`.

🔴 **And the pulse is closed on `yaw_ref`, not computed.** The firmware jumps to
W_ARRANQUE (36 degrees/s) as soon as any `w` is requested, so asking it for 8.8
each pulse moved four times more than planned and the compensation oscillated
with growing amplitude.

Also: front stop with **progressive braking and anticipation** (it brakes
based on where the robot will be 0.35 s from now), and **centering by taps** —
fixed force at the executable minimum, modulating how often it is applied.

### What was tried and does NOT work

- **Tightening `tolgiro` to 3**: the firmware retries the whole turn. One took
  13.1 s bouncing four times and ended 6 degrees past. It stays at 4.4.
- **`gradfreno` at 60**: it improved nothing. It stays at 45.
- **Stopping lateral correction in the last 300 mm**: that is more than half a
  stretch, and the robot came out diagonally at 12-14 degrees. It is the last 150.
- **Correcting with a small proportional `vy`**: below 40 out of 255 the wheels
  do not break free and the robot does not move.

### The front braking: why it cannot be measured with the sensor

🔑 **The braking anticipation is computed with the speed the script itself has
just requested, not by measuring it with the ToF.** It is the number the loop
decides on that same iteration: exact and noise-free. The sensor can only
*increase* the margin if it sees the robot approaching faster than requested,
never reduce it.

🔴 **Measuring it with the sensor, the margin jumped between 0 and 103 mm within
the same stretch.** The ToF refreshes every 160 ms and the loop reads every
220, unsynchronized: two consecutive readings often fall in the same refresh
and give **zero**. The robot really approaches at 120 mm/s and the estimate
between two readings swept from 0 to 295. And below 100 mm the sensor stops
refreshing normally: it gives large jumps followed by flat plateaus, exactly
where the decision matters. Of 159 samples of one run, fifteen gave zero, and
**the two stretches that ended up touching the wall are exactly those that
fired with the estimate at zero**.

The constant is **1.5 published mm per second and per unit of vx**, measured in
the range where the stop fires (vx from 45 to 85, where it came out between
1.45 and 1.58).

With that the eight stops fall between 12 and 84 mm with the target at 41,
mean 45.9, **and none touches the wall**. Side effect: stopping earlier leaves
the robot farther from what will be the right wall after turning (mean 83 mm
against 67), and the re-centering takes up to 3.1 s. It is the geometry of the
corner; it is corrected by lowering the threshold.

### 🔑 The heading is limited by the motor temperature, and it is measured

With the **same turn code**, without touching a constant, the mean error per
turn along the afternoon:

- **Cold motors after a rest (14:25)**: **+0.77** — first lap +0.36, second
  +1.19.
- **Hot motors (14:03)**: +1.17 — first lap +0.05, second +2.29.
- **Eight minutes after the previous one (14:33)**: **+1.92** — first lap
  +1.94, second +1.90.

🔑 **When cold the error is halved and the degradation between laps almost
disappears.** The 14:33 run did not degrade *within* itself because it already
started degraded.

🔴 **And the take-off PWMs are saved in the ESP32 memory, so they survive the
reset: letting the motors rest does NOT cool them down.** They have been all
afternoon against the 170 ceiling (169/169/165/165 at the start,
167/165/161/165 at the end). To compare two runs from the same starting point
they have to be reset by hand with `B`.

### State at the end

Robot stopped. The Pi service is still up. The live parameters were left at
the validated ones (`tolgiro` 4.4, `gradfreno` 45) because the script sets them
on every start. **The Pi is no longer in current undervoltage**: `vcgencmd
get_throttled` gives `0x50000`, i.e. it has suffered it but not right now.

## 🔴 HOW TO RESUME — the two pending items, in order

### 1. Calibrate the front sensor below 150 mm

It is the unknown that was carried along the whole session: **the front sensor
decides all the stops and its correction is only measured between 150 and 350
mm**. When it reports 15 mm we do not know how far it really is, and its 46.7
offset makes any raw reading close to that value become almost zero when
corrected.

**It takes ten minutes.** Robot still and motors off, a flat perpendicular
board in front of the front sensor, measuring from the module:

    cd bench_tests/13_tof_calibration
    python3 measure.py N 60      # and then 100, and 130
    python3 measure.py --ajuste

Wait ~1 s after moving the board: the firmware publishes the mean of the last 5
readings and before that it drags the previous one. With those three points the
short-range line comes out and the stopping distance can really be set.

🔑 The side sensors already have their 60 mm point: the right one publishes
47.7 and the left one 43.9 with the wall at 60 real mm.

### 2. 🔴 The take-off PWMs, against the ceiling

They are no longer "halfway through re-learning": **they are saturated**. All
afternoon between 161 and 169 against a cap of 170, on all four wheels. That
means the firmware asks for the maximum it is allowed to break the inertia and
even so the turns degrade with the heat.

**And they are saved in flash: they survive the ESP32 reset.** Resting the
motors does not lower them. To start two runs from the same point they have to
be reset with the `B` command (factory take-offs, `{95, 83, 93, 39}`),
accepting that the first maneuvers come out worse while it re-learns them.

**It is the thread that really limits the heading.** The open questions: why
the four need more and more push, whether the 170 ceiling is the right one,
and whether the battery voltage drop under load explains the thermal part.

### And before blaming the control for anything odd

🔴 **Check the Pi power supply**: `vcgencmd get_throttled`. It was at `0x50005`
(**current** undervoltage) for a good part of the session; at the end it gives
`0x50000`, which only records having suffered it.
