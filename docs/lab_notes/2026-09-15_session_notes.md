# Neurocontrol 2 car — notes to resume from scratch

Written on September 15, 2026 at the end of the session (with a section added
on Sep 16). Meant to be read on another machine or in another session, with
nothing else at hand. Paths are given in the layout of this repository.

---

## 1. The three machines and their roles

| What | Address | Role |
|---|---|---|
| **ESP32** | `<esp-ip>` (fixed, set in BLOCK 27 of the firmware) | **Executes.** Motors, encoders, IMU, the four ToF. Speed and heading loops. |
| **Raspberry Pi** | `<pi-ip>` | **Decides.** Talks to the ESP over the **USB cable** and serves the web interface on port 8080. |
| **Laptop** | — | Development. Uploads firmware to the ESP and code to the Pi, both over WiFi. |

🔴 **The network does NOT carry commands or telemetry.** The Pi↔ESP link is the
USB cable at 115200 baud. WiFi is only used to upload code and to serve the
interface.

---

## 2. Starting everything

**1. Power** the car (battery) and the Pi.

**2. Start the Pi server:**

    ssh user@<pi-ip>
    cd ~/raspberry_pi
    setsid nohup python3 main.py --port /dev/ttyUSB0 --web-port 8080 > /tmp/server.log 2>&1 &

**3. Open the interface** at `http://<pi-ip>:8080`.

**4. Check that it answers:**

    curl -s http://<pi-ip>:8080/estado | head -c 300

🔴 **To stop it, kill by the PID of the Python process, not with `pkill -f`.**
`pgrep -f "main.py"` also catches the bash wrapper and **the ssh command
itself**, so a `kill` that seems to work leaves the server alive — and two
servers fighting for the port give false measurements. The right way:

    ssh user@<pi-ip> 'for p in $(pgrep -f "python3 main[.]py"); do kill -9 $p; done'
    ssh user@<pi-ip> 'fuser /dev/ttyUSB0'        # must come out empty

---

## 3. Uploading code over WiFi

### To the ESP32 (OTA, without touching the car)

    cd firmware
    pio run -e wifi -t upload

It takes about 70 s. The ESP has a fixed IP and `platformio.ini` already sets a
fixed listening port (33531) and a 90 s timeout, because the network at the
site loses packets in bursts.

🔴 **Before uploading, stop the Pi server.** With the serial port busy the
upload fails with *"Error response from device"*. Checked today: it failed
with the server alive and went in at the first attempt with the port free.

🔴 **The first time it has to be flashed by cable** (`pio run -t upload`, at
460800). The OTA only exists once this firmware is inside.

The first thing the firmware does when an upload starts is brake and disable
the motors. Even so: the car on the floor and clear.

If it fails with *"No response from device"*, in order: check that the car is
on, that `ping <esp-ip>` answers, and that no turn is in progress (it is
blocking, up to 8 s).

### To the Pi

    PI=user@<pi-ip> ./tools/deploy_interface.sh

or copy individual files with `scp` into `~/raspberry_pi/` and **restart the
server** so it picks them up.

---

## 4. Driving the robot by hand

From the interface, **Manual control** panel:

- **↑ ↓** or the arrow keys: 5-second driving push.
- **← →**: 90° turns closed by the IMU.
- **Space bar** or STOP: cuts the push.

(In the Sep 15 version of the interface there were also sideways translation
buttons, keys **A / D**, and an **advance to the wall** button.)

🔑 **The translation rises by itself to vy 120** even if the slider is lower. It
is not a whim: below that the robot does not translate, it twists (see §6).

Through the API, without the interface:

    # move: vx forward, vy to the LEFT, w turn; ms = sustained push
    curl -X POST -H "Content-Type: application/json" \
      -d '{"accion":"manual_pwm","vx":102,"vy":0,"w":0,"ms":700}' \
      http://<pi-ip>:8080/orden

    # primitives: L and R turn 90, T half turn, 5 advances one cell,
    #             A advances to the wall, 0 stops
    curl -X POST -H "Content-Type: application/json" \
      -d '{"accion":"manual","letra":"L"}' http://<pi-ip>:8080/orden

    # bench: E motors on/off, C heading to zero, B factory take-offs,
    #        Z set heading, G save take-offs
    curl -X POST -H "Content-Type: application/json" \
      -d '{"accion":"banco","letra":"E"}' http://<pi-ip>:8080/orden

    # live gains (they do NOT go through /orden)
    curl -X POST -H "Content-Type: application/json" \
      -d '{"nombre":"tolgiro","valor":4.4}' http://<pi-ip>:8080/parametro

---

## 5. The three experiments that work

All in `bench_tests/`, all launched with `python3 <script>.py` from their
folder and stopped with **Ctrl+C** (not with Esc), which brakes the robot
before quitting. Each script has `API = "http://<pi-ip>:8080"` at the top.

| Folder | What it does | Best result |
|---|---|---|
| `14_square_circuit/best_2026-09-15_1433/` | square turning left | two laps |
| `14_square_circuit/best_right_2026-09-15_1547/` | square turning right | **two laps, 0.21° of error** |
| `15_back_and_forth_180/best_2026-09-15_1610/` | runs turning **towards the free side** | nine turns, 0.42° of error |
| `16_lateral_translation/` | sideways translation bench | see §6 |

Each `best_*` folder has its `README.md` with the figures and the reason for
each constant. **Always start there before touching anything.**

---

## 6. 🔑 The most important thing learned today

### The sideways translation has its own minimum speed

| vy | difference between wheels | twist |
|---|---|---|
| 40 | **41%** | **−8.5°** |
| 80 | 36% | +2.1° |
| **120** | **14%** | **−0.5°** |
| **160** | **11%** | **+0.5°** |

**Below vy 120 the robot does not translate, it twists.** The cause is the
front right wheel: it has a different gearbox — it reaches 365 rpm where the
other three stay at 160-175 — and below about 90 rpm its reading is not
repeatable.

🔴 **This applies to any lateral correction.** The circuit corrected at vy 40
while moving and vy 65 while stopped, the two worst points: each tap twisted
the heading 0.46° in median, with a maximum of 13.73°.

### The behavior matrices of the wheels are already measured

**They do not need to be measured again. They are in:**

- `bench_tests/11_encoder_noise/README.md` — noise of each encoder, per-sector
  error, and how it falls when averaging longer windows.
- `bench_tests/12_pwm_sweep/README.md` — what PWM each wheel needs to start and
  to keep going, its rpm cap and its PWM↔rpm line.
- `bench_tests/16_lateral_translation/README.md` — the same for the sideways
  translation.

**For any wheel motion problem, look at those three first.**

---

## 7. What should NOT be tried again

🔴 **The AGC does not judge the quality of an encoder.** It says how far the
magnet is from the chip, not whether it is centered. The rear right has its
AGC at the top of the range and it is the one that measures best. Today that
mistake was made again.

🔴 **Centering the magnets by hand does not work.** The scale one has to hit is
tenths of a millimeter (tried on Sep 14).

🔴 **Tightening `tolgiro` below 4.4 does not remove the turn bias, it adds
bounces.** With 3, the firmware retries the whole turn: one took 13.1 s and
bounced four times.

🔴 **Anticipating the braking by measuring the speed with the ToF does not
work.** The sensor refreshes every 160 ms and the loop reads every 220,
unsynchronized: two consecutive readings often fall in the same refresh and
give zero. It is computed with the **requested** speed.

🔴 **Inflating the braking margin when the network is slow does not work
either.** A one-second peak asked for 300 mm of margin and the robot stopped
halfway to the wall.

🔴 **A small `vy` does not move the robot.** Below 40 out of 255 the wheels do
not break free. The force stays at the executable minimum and **what is
modulated is how often it is applied**, not how hard.

🔴 **Send pushes (`ms` > 0), never pulses.** The pulse carries a 600 ms dead
man's switch and a loop from the Pi takes ~230 ms per iteration: with pulses
the robot goes in jerks.

🔴 **Do not retry commands that are not idempotent.** Retrying a turn leaves
another one queued on the ESP and it is executed later, in the middle of a
straight stretch.

---

## 8. The three layers of parameters

1. The firmware `.cpp`.
2. **`PARAMS_CALIBRADOS` in `raspberry_pi/main.py`, which is sent on every start
   and OVERRIDES what the firmware has written.** If a parameter is touched in
   the `.cpp`, it has to be touched there too.
3. The live adjustments through `/parametro`, which override both.

🔴 After a session of sweeps, check the parameters in force in `/estado` before
taking anything as good.

---

## 9. What remains open

- **The take-off PWMs have been against the 170 ceiling** the whole session,
  and they are saved in flash: resting the motors does not lower them. They
  are reset with the `B` command.
- **The heading is limited by the motor temperature**: with the same code,
  +0.77° of mean error per turn cold and +1.92° eight minutes later.
- **In open field there is nothing to center against** and the heading drifts
  up to 11°.
- **The circuits do not use the `'A'` command yet**, which would take the front
  braking off the network. It is the natural next step.
- **Raise `VY_MIN_EFECT` and `VY_PEGAR` to 120-140** and shorten the pulses
  proportionally.

---

## 10. New interface (Sep 16)

`raspberry_pi/web/` is the new interface. Header with START / FORCED STOP
(space bar) / MANUAL GOAL (key F); full neural system as a heat map (ToF →
Naka-Rushton → gaussians → memory/reverse + MLP → votes → softmax); decision
and camera in the center; maze reconstructed with continuous odometry
(`config.CELDA_CM = 24`); five-button manual control; ToF / gaussian / left PWM
/ right PWM plots; telemetry block.

Pi side: `navigator.decidir` publishes on every decision `_instantanea_neuro`
(features, h1, h2, q, breakdown, score, probs, candidates, chosen, T, β)
through the `on_decision` callback, which `main.py` points to
`tablero.publicar_neuro`; `/estado` exposes it in `neuro`, `decision` and
`celda_cm`. Validated with `test_headless.py` and 579 serializable snapshots.

Without the robot: open `raspberry_pi/web/index.html` with a double click (or
serve it with `?dummy=1`): `simulator.js` runs the real equations on a random
maze.

🔑 `config.CELDA_CM` (map) and the `celda` parameter of `PARAMS_CALIBRADOS`
(27, what the ESP travels for each `5`) must match the real cell; otherwise the
robot drifts a fraction of a cell per step and the map shows it.
