# Square circuit: it advances along the corridor, stops when it sees 60 mm
# ahead, turns -90 (to the LEFT, command 'L'), and repeats. It centers with
# the side sensors.
#
# 🔴 THE HEADING REALLY STARTS AT ZERO. The 'C' command re-anchors the yaw
# (yaw = yawMedido = yawRef = 0) besides measuring the gyroscope drift at
# rest. It leaves the motors off, so they have to be re-enabled afterwards.
#
# 🔴 CENTERING WITH A SINGLE WALL. At the beginning of the circuit one side is
# free. With both it centers by the difference; with only one it keeps THAT
# one at its setpoint. The setpoints are NOT 60: the firmware correction is
# measured between 150 and 350 mm and fails below that, so the values each
# sensor really publishes with the wall at 60 mm, measured on Sep 15, are
# used.
#
# 🔴 POSITIVE vy MOVES TO THE LEFT. Measured, not assumed.

import json, os, time, urllib.request

API = "http://11.11.41.17:8080"
REG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "square_%s.jsonl" % time.strftime("%Y-%m-%d_%H%M"))

TRAMOS       = 8       # two laps of the square
# 🔴 THE GOAL IS TO END 7 cm FROM THE WALL, measured with tape. With the wall at
# 70 real mm the firmware publishes 40.9 (measured on Sep 15: raw it reads
# 83.1, and its long line falls 29 mm short because it is only calibrated
# from 150 to 350). The typical overshoot is about 25 of the published units,
# so the trigger goes 25 above the target so that the END falls at 41.
PARADA       = 63.0    # 2 mm closer to the front
# 🔴 SOFT START. Asking for the full cruise speed when leaving the turn made
# the robot lean to the right and then stick to the left compensating. The
# speed is ramped up during the first second and a half of the stretch.
T_RAMPA_INI  = 1.5     # s until the full cruise speed is requested
# 🔴 PROGRESSIVE BRAKING, NOT TWO STEPS. Before, it went at cruise until it saw
# LENTO and there it jumped to V_LENTA at once; now the speed goes down
# continuously from EMPIEZA_FRENAR to PARADA, so it reaches the threshold
# already slowly.
EMPIEZA_FRENAR = 330.0
V_MINIMA     = 30      # out of 255 -> 0.035 m/s, the slowest it is asked to go
# 🔑 AND IT STOPS BY ANTICIPATION. The overshoot is not inertia alone: between
# the ToF refreshing (160 ms) and this script reading and sending (about 230
# ms) the robot has already advanced. So it brakes when the distance
# PREDICTED for that time from now falls below the threshold, not when the
# current one does.
LATENCIA     = 0.35    # s of total loop delay
# 🔴 AND THE APPROACH SPEED IS NOT MEASURED BETWEEN TWO READINGS. The ToF
# refreshes every 160 ms and this loop reads every 220, unsynchronized: two
# consecutive readings often fall within the same refresh and give ZERO, and
# the next one gives double. Measured on the 14:25 run: the robot approaches
# at 120 mm/s for real and the instantaneous estimate swept from 0 to 295,
# i.e. the anticipation margin jumped between 0 and 103 mm WITHIN THE SAME
# STRETCH. That is why the stop came out between 4 and 67 mm with the same
# target of 41: the spread came from the estimator, not from the brake. The
# two stretches that ended up touching the wall are exactly the ones that
# fired with the estimate at zero. It is measured with the slope of the last
# 0.6 s, which cover almost four sensor refreshes.
VENTANA_VEL  = 0.6     # s of history used to estimate the approach
# published mm per second the robot approaches for each unit of requested vx.
# Measured on the 14:25 run in the stretch that matters (vx from 45 to 85,
# which is where the stop fires): 1.45 to 1.58.
MMS_POR_VX   = 1.5
# 🔴 And in the last centimeters it does not translate, to arrive straight at
# the turn. Careful not to overdo it: with 300 mm the robot went half the
# stretch without centering and came out diagonally at 12-14 degrees. It is
# the last centimeters, not half the corridor.
SIN_TRASLADAR = 150.0
V_CRUCERO    = 102     # 0.12 m/s
V_LENTA      = 35      # 0.041 m/s
KP_DOS       = 0.9     # with both walls in view
KP_UNA       = 0.6     # with only one, against its setpoint
# 🔴 A SMALL vy DOES NOT MOVE THE ROBOT. With an error of 27 mm the loop asked
# for vy=13 out of 255, i.e. 0.016 m/s of translation: far below what these
# wheels can sustain (the front right does not turn repeatably below 70 rpm).
# The robot advanced straight, stuck to a wall, "correcting" without moving.
# So the correction has a FLOOR: if it is worth correcting, something
# executable is sent; and if not, zero. What is modulated is the time, not
# the magnitude.
VY_MIN_EFECT = 40      # below this the robot does not translate
BANDA_LAT    = 5.0     # mm of error considered centered
# 🔴 SAFETY LIMIT ON THE LEFT. Chasing the right-hand setpoint the robot got
# too close to the other side and touched it several times: a one-sided
# setpoint knows nothing about the other side. Below this, moving away
# overrides everything else.
MIN_LATERAL  = 32.0
# 🔴 WHILE MOVING, ONLY SOFT TAPS. The shoves of 60 arrived on time but were
# abrupt: the robot did not go straight, it leaned towards a wall and on top
# of that each large translation twisted its heading. The big fix is done by
# the stopped re-centering after the turn; while moving it is only touched up.
VY_URGENTE   = 45      # ONLY to move away from a wall that is coming too close
ERR_GRANDE   = 9999.0  # disabled: no strong shoves while moving
SESGO_LAT    = 3.8     # dE-dO when truly centered: the sensors do not read the same
# 🔴 THE CORRIDOR IS NARROW: NO ABRUPT MOVES. Correcting with a fixed vy of 40
# every cycle, the robot threw itself towards the other wall and touched it.
# But lowering the magnitude does not work either: below 40 the wheels do
# not break free and nothing moves (measured).
# 🔑 So the force stays at the minimum executable and what is modulated is HOW
# OFTEN it is applied: with a small error a tap is given every now and then,
# and with a large error on every cycle. It is the same idea that fixed the
# re-centering after the turn, where the pulse time was modulated instead of
# the force.
# 🔴 CORRECT SOONER. With ERR_PLENO at 30 and a band of 10, a small offset took
# long to fill the accumulator and the robot corrected at the end of the
# stretch, when it had practically left. Lowering both, the same error
# produces taps sooner and more often, WITHOUT raising the force of each tap.
ERR_PLENO    = 18.0    # error at which it already corrects on every cycle
TOPE_VY      = 40      # never more than the executable minimum: no shoves
# 🔴 THE SETPOINTS COME FROM THE MEASURED CORRIDOR, NOT FROM ASSUMING 60 mm.
# With both walls in view the sum dE+dO gives 121.7 mm (measured on Sep 15
# over 32 samples), so the center leaves about 61 on each side. The previous
# setpoint was 47.7 and the robot read 74: the loop believed it was far from
# the right wall and pushed it against it the WHOLE stretch, which came out
# as going diagonally by 2 to 4 degrees.
# 🔑 And the two side sensors do NOT publish the same at the same real
# distance: at 60 mm the right one gives 47.7 and the left one 43.9. They
# differ by 3.8, so WHEN CENTERED the difference is not zero, it is 3.8.
# 🔴 THE REFERENCE IS THE RIGHT WALL, NOT THE CENTER. Measured with the robot
# placed by hand where it should go: the right one publishes 57.5 (59.8 raw).
# That is the same target for both things, the re-centering after the turn
# and the motion, so the robot does not change criterion halfway through the
# stretch.
# 🔑 And looking for the right wall instead of the center takes the left
# sensor out of the picture, which is exactly the one that goes blind at the
# gap in the wall.
CONSIGNA_E   = 52.5   # 5 mm closer to the right: it came out of the turn scraping the left
HUECO_LIBRE  = 95.5    # dE+dO measured in that position
CONSIGNA_O   = HUECO_LIBRE - CONSIGNA_E   # 38.0, only if the right one does not see
TOPE_TRAMO   = 45.0
TOPE_GIRO    = 12.0    # goes up: with tolgiro at 3 the firmware retries more
# 🔴 TURN TOLERANCE: BACK TO 4.4. With 3 the firmware RETRIES the whole turn,
# and on Sep 15 what that does was measured: turn 1 took 13.1 s instead of 4
# and its heading bounced four times (93.4 -> 80.5 -> 99.3 -> 87.5 -> 97.0),
# ending 6 degrees past. Mean turn error 2.84 against 0.88 with 4.4.
# Tightening the criterion does not remove the bias, it only adds bounces.
TOLGIRO      = 4.4
# 🔴 THE TURN ALWAYS OVERSHOOTS TO THE SAME SIDE: +1.4 to +2.5 degrees in the
# four turns, and the ESP KNOWS it (it reports it is 1.6-2.6 short) but
# accepts it because it fits within the tolerance. A constant bias is not
# fixed by tightening the stopping criterion: it is compensated by arriving
# more slowly. gradfreno is the degrees before the target at which it starts
# braking.
GRADFRENO    = 45.0    # the one validated on the floor; 60 did not improve anything
# 🔴 AFTER EACH TURN, STICK TO 6 cm FROM THE RIGHT WALL BEFORE ADVANCING. The
# turn does not leave the robot centered in the new corridor, and if it
# starts like that it goes into the left wall. 6 REAL cm of the right sensor
# are 47.7 of what it publishes (measured on Sep 15 with the wall at 60 mm).
# 8 cm from the right wall. The right sensor publishes 47.7 with the wall at
# 60 real mm (measured) and about 150 at 150, so at 80 real mm it publishes
# something around 70.
# ⚠️ It is an extrapolation: the front and side sensors are not calibrated
# at short range.
# 7 cm from the right wall (one centimeter closer than before). Measured
# references: 47.7 published with the wall at 60 real mm, and ~70 at 80 mm.
OBJETIVO_DER = 52.5
BANDA_DER    = 5.0     # mm considered good
# 🔴 WHEN THE PULSE IS CUT, THE ROBOT KEEPS SLIDING SIDEWAYS. Measured on Sep 15:
# asking for 48 mm it ended at 29, 30 and 33 the three times, about 17 too
# many and ALWAYS towards the same side. A constant bias is anticipated, like
# the front stop: it is cut when what remains is exactly what it will slide.
# Measured again on the afternoon of Sep 15: with 17 the re-centering stayed
# between 12 and 20 mm short of the target, so it now slides less than in the
# morning.
INERCIA_LAT  = 12.0
VY_PEGAR     = 65      # 🔑 STOPPED, a LARGE translation is needed: with +-27 it does not break free
VY_PEGAR_MIN = 45      # below this it does not move while still
MS_PEGAR     = 250     # short pushes so as not to overshoot
TOPE_PEGAR   = 9.0
PERIODO      = 0.10
MS_TIRADA    = 700
GIRO         = "L"     # -90 according to Sebas = to the left; in yaw it goes up +90
# 🔴 ABSOLUTE HEADING, NOT INDEPENDENT TURNS. Each turn overshot by ~1 degree
# and nothing compensated it, so the lap closed 7-8 degrees off. Now the
# heading it SHOULD have is tracked (n x 90 from zero) and whatever is left
# over from the previous turn is discounted in the next stretch.
#
# 🔑 And it is NOT corrected by turning while stopped: that was already tried
# and discarded on Sep 2, because the smallest motion this car can make is
# larger than the error (correcting 2 degrees it overshot by 5 to 7). It is
# corrected WHILE ROLLING, taking advantage of the firmware moving its own
# reference with the turn rate it is sent: 'yawRef += pedidoW * 180/PI * dt'.
# A short w pulse at the start of the stretch shifts the reference by the
# missing degrees, and the loop takes the robot there while it advances,
# which is where it does have authority.
PASO_GIRO    = 90.0    # what each turn should add to the heading
# 🔴 THE PULSE TIME CANNOT BE COMPUTED BLINDLY. The firmware JUMPS to
# W_ARRANQUE (36 degrees/s) as soon as any w is requested, because otherwise
# the short-gearbox wheel does not break free:
#     if(|oW|>0 && |pedidoW|<eps) pedidoW = ±W_ARRANQUE;
# Asking for 8.8 degrees/s the pulse moved four times more than planned, and
# the compensation oscillated with growing amplitude: -1.87, +2.75, -4.44.
# 🔑 So the loop is closed on the firmware's OWN reference, which travels in
# the telemetry as yaw_ref: short pulses are given and it stops when it is
# already where it should be. Nothing has to be modeled.
W_CORRIGE    = 25      # out of 255
BANDA_YAW    = 0.8     # degrees considered good
MS_PULSO_W   = 90      # short pulse: with the start jump it is ~3 degrees
# 🔴 With 8 it ran out of budget: in the stretches that carried more than 3
# degrees it used up the pulses and 2.04 and 1.78 were still uncorrected.
MAX_PULSOS_W = 14

reg = open(REG, "w", encoding="utf-8")

# 🔴 THE NETWORK AT THIS SITE LOSES PACKETS IN BURSTS (the platformio.ini
# already warned about it with its 45 s timeout). On Sep 15 a cut of a few
# seconds killed the script in the middle of turn 2 and took the stop command
# down with it. Nothing that talks to the Pi can assume the network answers.
# The underlying safety is still the push: if this dies, the ESP brakes by
# itself within 700 ms. But the script does not have to die.
REINTENTOS = 3

def _pedir(url, datos=None, timeout=5):
    ultimo = None
    for i in range(REINTENTOS):
        try:
            if datos is None:
                with urllib.request.urlopen(url, timeout=timeout) as r:
                    return r.read().decode()
            r = urllib.request.Request(url, data=datos,
                                       headers={"Content-Type": "application/json"})
            return urllib.request.urlopen(r, timeout=timeout).read().decode()
        except Exception as e:
            ultimo = e
            time.sleep(0.25 * (i + 1))
    raise ultimo

def post(c):
    return _pedir(API + "/orden", json.dumps(c).encode())

def post_una_vez(c):
    """🔴 For commands that CANNOT be repeated. Retrying a READING is harmless,
    but retrying a TURN can leave another one queued on the ESP and executed
    later, in the middle of a straight stretch. On Sep 15 stretch 4 got a
    94-degree turn that nobody requested at that moment.
    If it fails, it fails: better to lose the command than to send it twice."""
    d = json.dumps(c).encode()
    r = urllib.request.Request(API + "/orden", data=d,
                               headers={"Content-Type": "application/json"})
    return urllib.request.urlopen(r, timeout=8).read().decode()

def parametro(nombre, valor):
    """Live adjustments go to /parametro, NOT to /orden."""
    return _pedir(API + "/parametro",
                  json.dumps({"nombre": nombre, "valor": valor}).encode())


def telem():
    return json.loads(_pedir(API + "/estado"))["telem"]

def mover(vx, vy):
    post({"accion": "manual_pwm", "vx": int(vx), "vy": int(vy), "w": 0,
          "ms": MS_TIRADA})

def parar():
    """Stopping can NEVER fail because of a network cut: it insists and stays quiet."""
    for _ in range(4):
        try:
            post({"accion": "manual_pwm", "vx": 0, "vy": 0, "w": 0})
            return
        except Exception:
            time.sleep(0.4)
    try:
        post({"accion": "paro"})
    except Exception:
        print("    (the stop could not be sent: the push expires by itself in 700 ms)")

def anota(t, etiqueta, vx, vy, modo=""):
    reg.write(json.dumps({
        "etiqueta": etiqueta, "t": round(time.time() - T0, 2), "modo": modo,
        "dN": t["dN"], "dE": t["dE"], "dS": t["dS"], "dO": t["dO"],
        "yaw": t["yaw"], "yaw_ref": t["yaw_ref"], "vx": vx, "vy": vy,
        "estado": t["estado"], "cmd": t["cmd"], "ev": t["ev"],
        "imu_ok": t["imu_ok"],
        "rpm": [t["rpm_DI"], t["rpm_DD"], t["rpm_TI"], t["rpm_TD"]],
        "cortadas": t["cortadas"]}) + "\n")
    reg.flush()

# 🔴 A SIDE READING IS ONLY VALID IF IT FITS IN THE CORRIDOR. On Sep 15 the loop
# believed a left reading of 580 mm while the right one read 54: the sensor
# was looking through a side opening, not at the wall. It computed 526 mm of
# offset, sent the translation to the cap and dragged 15.6 degrees of heading
# in two seconds.
# In a 250 mm corridor with the robot measuring ~130 between sensors, a side
# sensor that sees ITS wall cannot exceed ~120. Anything beyond the limit is
# something else -- a side opening, a gap -- and is not used for centering.
LIMITE_LATERAL = 140.0
# And when the wall is recovered one has to wait: the firmware's 5-sample
# average filter drags the old value for ~0.8 s and publishes a ramp that is
# not any real distance.
CUARENTENA = 5
_cuar = {"dE": 0, "dO": 0}

def ve(t, k):
    if t[k] >= 599 or t[k] > LIMITE_LATERAL:
        _cuar[k] = CUARENTENA
        return False
    if _cuar[k] > 0:
        _cuar[k] -= 1
        return False
    return True

# distribution of the taps: how much "is needed" is accumulated and it only
# corrects when the accumulator fills up. This way the FREQUENCY is
# proportional to the error.
_acum = [0.0]

def _toque(err, sentido):
    """Returns vy: either zero, or a tap of the minimum executable size.

    `sentido` is +1 if correcting requires going to the left (vy>0).
    """
    _acum[0] += min(1.0, abs(err) / ERR_PLENO)
    if _acum[0] < 1.0:
        return 0.0
    _acum[0] -= 1.0
    return float(VY_MIN_EFECT * sentido)

def correccion(t):
    """Returns (vy, error, mode). vy>0 goes to the left.

    🔴 WITH BOTH WALLS, THE TARGET IS THE AVERAGE OF THE TWO MEASUREMENTS, i.e.
    the center of the gap the robot sees RIGHT THERE. It is a dynamic target:
    it adapts to the corridor not measuring the same in every stretch, which a
    fixed setpoint cannot do. Chasing only the right wall, the robot met that
    setpoint and ran into the opposite wall.
    """
    d, i = ve(t, "dE"), ve(t, "dO")

    # first of all: move away if it is right on top of a wall
    if i and t["dO"] < MIN_LATERAL:
        return float(-VY_URGENTE), t["dO"] - MIN_LATERAL, "huye_izq"
    if d and t["dE"] < MIN_LATERAL:
        return float(+VY_URGENTE), MIN_LATERAL - t["dE"], "huye_der"

    if d and i:
        # error against the center of the gap seen now; the bias is there
        # because the two sensors do not publish the same at the same real
        # distance
        err = ((t["dE"] - t["dO"]) - SESGO_LAT) / 2.0
        if abs(err) <= BANDA_LAT:
            return 0.0, err, "centro"
        if abs(err) >= ERR_GRANDE:
            return float(-VY_URGENTE if err > 0 else +VY_URGENTE), err, "centro_fuerte"
        return _toque(err, -1 if err > 0 else +1), err, "centro"

    if d:
        err = t["dE"] - CONSIGNA_E        # >0 = far from the right
        if abs(err) <= BANDA_LAT:
            return 0.0, err, "derecha"
        if abs(err) >= ERR_GRANDE:
            return float(-VY_URGENTE if err > 0 else +VY_URGENTE), err, "derecha_fuerte"
        return _toque(err, -1 if err > 0 else +1), err, "derecha"
    if i:
        err = t["dO"] - CONSIGNA_O        # >0 = far from the left
        if abs(err) <= BANDA_LAT:
            return 0.0, err, "izquierda"
        return _toque(err, +1 if err > 0 else -1), err, "izquierda"
    return 0.0, None, "ciego"

def alineacion(muestras):
    """Angle of the robot with respect to the wall it sees to the side, in degrees.

    If it goes parallel, the side reading does not change while it advances.
    If it goes tilted, it changes in proportion to the advance: the angle is
    atan(d_lateral / d_advance). The advance is measured with the REAR sensor
    while it does not saturate, which at the start of the stretch sees the
    wall behind.

    🔑 This is what tells whether the heading zero was taken with the robot
    tilted with respect to the corridor: the loop defends its reference well,
    but if the reference is not parallel to the walls the robot goes straight
    DIAGONALLY.
    """
    import math
    pares = []
    for x in muestras:
        lat = x["dE"] if x["modo"] == "derecha" else (x["dO"] if x["modo"] == "izquierda" else None)
        if lat is None or not (10 < lat < 140): continue
        if not (60 < x["dS"] < 550): continue
        pares.append((x["dS"], lat))
    if len(pares) < 5:
        return None, len(pares)
    n = len(pares)
    sx = sum(a for a, _ in pares); sy = sum(b for _, b in pares)
    sxx = sum(a*a for a, _ in pares); sxy = sum(a*b for a, b in pares)
    den = n*sxx - sx*sx
    if abs(den) < 1e-6: return None, n
    m = (n*sxy - sx*sy) / den          # mm of lateral per mm of advance
    # the sign depends on which wall is looked at: the magnitude and the side
    # are reported
    return math.degrees(math.atan(m)), n


def pegarse_a_la_derecha(etiqueta):
    """Translates sideways until it is 6 cm from the right wall, stopped.

    🔑 This DOES work stopped, even though the soft centering of Sep 15 did
    not: what fails with the robot still is the SMALL translation (with a vy
    of +-27 it did not move in eight seconds). With a large vy it breaks free
    -- measured, +60 for 300 ms moved it 22 mm.
    """
    t0 = time.time()
    t = telem()
    if t["dE"] >= 599 or t["dE"] > 200:
        print("    I do not see the right wall (%.0f): not sticking" % t["dE"])
        return None
    print("    sticking to the right (now %.0f, target %.0f)..."
          % (t["dE"], OBJETIVO_DER))
    while time.time() - t0 < TOPE_PEGAR:
        t = telem()
        if t["dE"] >= 599 or t["dE"] > 200:
            break
        err = t["dE"] - OBJETIVO_DER      # >0 = far from the right
        # what will be left after it slides: that is what has to be cancelled
        restante = err - (INERCIA_LAT if err > 0 else -INERCIA_LAT)
        if err * restante <= 0 or abs(restante) <= BANDA_DER:
            break
        # vy>0 goes to the LEFT, so moving away from the right asks for vy>0
        # 🔑 The force is fixed (below it does not break free) and what is
        # modulated is the TIME: with a 250 ms pulse it moves ~18 mm, more than
        # the band of 5, so with a small error it has to push for less time or
        # it never converges.
        vy = -VY_PEGAR if err > 0 else +VY_PEGAR
        # the last pulse, shorter: with a minimum of 60 ms the robot overshot
        # the target right at the end, which is where fine-tuning is needed
        ms = int(max(35, min(MS_PEGAR, abs(err) * 6)))
        post({"accion": "manual_pwm", "vx": 0, "vy": int(vy), "w": 0, "ms": ms})
        anota(t, etiqueta, 0, vy, "pegar")
        time.sleep(0.30)
    parar(); time.sleep(0.8)
    t = telem()
    print("      ends at %.0f (target %.0f, %+.0f mm) in %.1f s"
          % (t["dE"], OBJETIVO_DER, t["dE"] - OBJETIVO_DER, time.time() - t0))
    return t["dE"]


def desenvolver(seq):
    """Removes the +-360 jumps so the heading can be accumulated."""
    out = [seq[0]]
    for x in seq[1:]:
        d = x - out[-1]
        while d > 180: d -= 360
        while d < -180: d += 360
        out.append(out[-1] + d)
    return out

def acercamiento(hist):
    # least-squares slope of dN against t, in mm/s and towards the wall
    if len(hist) < 3: return 0.0
    ts = [h[0] - hist[0][0] for h in hist]
    ds = [h[1] for h in hist]
    if ts[-1] - ts[0] < 0.3: return 0.0
    n = len(ts)
    mt = sum(ts) / n; md = sum(ds) / n
    den = sum((x - mt) ** 2 for x in ts)
    if den <= 0.0: return 0.0
    pend = sum((x - mt) * (y - md) for x, y in zip(ts, ds)) / den
    return max(0.0, min(400.0, -pend))


def tramo(n, objetivo_yaw=None):
    t = telem()
    # discount right away what was left over from the previous turn: the
    # firmware reference is shifted with a w pulse, and the loop takes it there
    # while rolling
    pendiente = 0.0
    if objetivo_yaw is not None:
        pendiente = objetivo_yaw - t["yaw"]
        if abs(pendiente) <= BANDA_YAW:
            pendiente = 0.0
        else:
            print("    carries %+.2f degrees from the previous turn: compensated in this stretch"
                  % -pendiente)
    print("\n  STRETCH %d: starts with front = %.0f mm, yaw %+.2f" % (n, t["dN"], t["yaw"]))
    if t["dN"] <= PARADA:
        print("    already closer than %.0f mm: not advancing" % PARADA)
        return "sin sitio", 0.0
    t0 = time.time()
    errs = []; modos = {}; vistas = []
    yaws = [t["yaw"]]
    hist = []
    pulsos_w = [0]
    while True:
        t = telem()
        ahora = time.time()
        # speed proportional to what is left: continuous, no step
        d = t["dN"]
        if d >= EMPIEZA_FRENAR:
            v = V_CRUCERO
        else:
            frac = max(0.0, (d - PARADA) / (EMPIEZA_FRENAR - PARADA))
            v = V_MINIMA + (V_CRUCERO - V_MINIMA) * frac
        # and at the start, whatever the ramp asks if it is less than that
        transcurrido = time.time() - t0
        if transcurrido < T_RAMPA_INI:
            tope = V_MINIMA + (V_CRUCERO - V_MINIMA) * (transcurrido / T_RAMPA_INI)
            v = min(v, tope)
        # 🔑 THE ANTICIPATION IS COMPUTED WITH THE REQUESTED SPEED, NOT WITH THE
        # SENSOR. It is the number this script has just decided: exact and
        # noise-free. The ToF can only INCREASE the margin if it sees it
        # approaching faster than requested; never reduce it. This way a zero
        # from the sensor no longer leaves the robot without margin, which is
        # what smashed stretches 6 and 8 into the wall.
        hist.append((ahora, d))
        while len(hist) > 2 and ahora - hist[0][0] > VENTANA_VEL: hist.pop(0)
        vel = max(v * MMS_POR_VX, acercamiento(hist))
        if d - vel * LATENCIA <= PARADA: motivo = "sensor"; break
        if time.time() - t0 > TOPE_TRAMO: motivo = "TIEMPO"; break
        if t["cortadas"] > 0: motivo = "RUEDA CORTADA"; break
        # 🔴 the firmware should not be turning in the middle of a straight stretch
        if t["estado"] == 2: motivo = "GIRO INESPERADO"; break
        vy, err, modo = correccion(t)
        # 🔴 IN THE FINAL APPROACH IT DOES NOT TRANSLATE. The heading loop
        # defends its reference very well WHILE MOVING (mean error 0.02
        # degrees), but the last lateral translation leaves the robot tilted
        # right when it brakes, and once stopped there is nothing to
        # straighten it with: the stretch ended with 4.4 degrees of deviation
        # even though during the motion the heading was nailed. Arriving
        # straight matters more than arriving centered, because what comes
        # next is a turn.
        if d <= SIN_TRASLADAR:
            vy = 0.0
            modo += "_recto"
        modos[modo] = modos.get(modo, 0) + 1
        if err is not None and modo == "dos": errs.append(err)
        yaws.append(t["yaw"])
        vistas.append({"dE": t["dE"], "dO": t["dO"], "dS": t["dS"], "modo": modo})
        if pendiente != 0.0:
            # loop closed on yaw_ref: one pulse, look, decide
            falta = objetivo_yaw - t["yaw_ref"]
            if abs(falta) <= BANDA_YAW or pulsos_w[0] >= MAX_PULSOS_W:
                if pulsos_w[0]:
                    print("      reference corrected in %d pulses, %+.2f degrees left"
                          % (pulsos_w[0], falta))
                pendiente = 0.0
            else:
                w = int(W_CORRIGE if falta > 0 else -W_CORRIGE)
                post({"accion": "manual_pwm", "vx": int(v), "vy": int(vy), "w": w,
                      "ms": MS_PULSO_W})
                anota(t, "tramo%d" % n, v, vy, modo + "_yaw")
                pulsos_w[0] += 1
                time.sleep(MS_PULSO_W / 1000.0 + 0.12)
                continue
        mover(v, vy); anota(t, "tramo%d" % n, v, vy, modo)
        time.sleep(PERIODO)
    parar(); time.sleep(1.2)
    t = telem()
    yaws = desenvolver(yaws + [t["yaw"]])
    print("    stops by %s in %.1f s, front = %.0f mm" % (motivo, time.time()-t0, t["dN"]))
    print("    centering: %s" % ", ".join("%s %d%%" % (k, 100*v//sum(modos.values())) for k, v in modos.items()))
    if errs:
        print("    with both walls: mean difference |%.1f| mm, maximum %.0f"
              % ((sum(e*e for e in errs)/len(errs))**0.5, max(abs(e) for e in errs)))
    print("    heading during the stretch: %+.2f -> %+.2f (drift %+.2f degrees)"
          % (yaws[0], yaws[-1], yaws[-1]-yaws[0]))
    ang, npares = alineacion(vistas)
    if ang is None:
        print("    alignment with the wall: could not be measured (%d useful samples)" % npares)
    else:
        print("    ALIGNMENT with the wall: %+.2f degrees over %d samples" % (ang, npares))
        if abs(ang) > 3.0:
            print("      ⚠️ it goes diagonally: the heading zero is not parallel to the corridor")
    return motivo, yaws[-1]-yaws[0]

def giro(n):
    a = telem()
    t0 = time.time()
    post_una_vez({"accion": "manual", "letra": GIRO})
    visto = False
    while time.time() - t0 < TOPE_GIRO:
        t = telem()
        anota(t, "giro%d" % n, 0, 0, "giro")
        if t["estado"] != 0: visto = True
        elif visto and time.time()-t0 > 0.6: break
        time.sleep(0.08)
    time.sleep(1.0)
    b = telem()
    d = b["yaw"] - a["yaw"]
    while d > 180: d -= 360
    while d < -180: d += 360
    print("    TURN %d: %+.2f degrees in %.1f s (ESP err_giro %+.2f)"
          % (n, d, time.time()-t0, b["err_giro"]))
    return d

T0 = time.time()
print("=== SQUARE CIRCUIT, %d stretches ===" % TRAMOS)
print("setting the heading to zero (command C, about 2 s)...")
post({"accion": "banco", "letra": "C"})
time.sleep(4.0)
t = telem()
print("  yaw = %+.2f   ref = %+.2f   imu_ok = %d" % (t["yaw"], t["yaw_ref"], t["imu_ok"]))
parametro("tolgiro", TOLGIRO)
parametro("gradfreno", GRADFRENO)
print("  turn: tolerance %.1f degrees (was 4.4), braking from %.0f (was 45)"
      % (TOLGIRO, GRADFRENO))
if int(t["hab"]) == 0:
    post({"accion": "banco", "letra": "E"}); time.sleep(1.0)
print("  motors:", int(telem()["hab"]))
print("\nstarting: %.0f from the right, %.0f from the left" % (t["dE"], t["dO"]))
pegarse_a_la_derecha("inicio")

derivas = []; giros = []
try:
    objetivo = 0.0
    for n in range(1, TRAMOS + 1):
        motivo, der = tramo(n, objetivo)
        if motivo not in ("sensor",):
            print("\n  cut: stretch %d ended by %s" % (n, motivo)); break
        derivas.append(der)
        giros.append(giro(n))
        objetivo += PASO_GIRO
        t = telem()
        print("    heading: %+.2f, should be %+.2f  ->  %+.2f degrees left over"
              % (t["yaw"], objetivo, t["yaw"] - objetivo))
        # 🔴 IT RE-CENTERS STOPPED, AS IN THE 12:44 VERSION. Correcting it while
        # moving forces large lateral shoves right when leaving the turn, and a
        # 20 mm translation twists the heading 1.3 degrees: that is why the yaw
        # degraded when this was removed. The turn leaves the robot 22 mm
        # shifted to the left on average (measured), which is too much to fix
        # without abruptness.
        print("      leaves at %.0f from the right" % t["dE"])
        pegarse_a_la_derecha("pegar%d" % n)
except KeyboardInterrupt:
    print("\ninterrupted")
except Exception as e:
    print("\n🔴 cut because of a failure talking to the Pi: %s" % e)
finally:
    parar()

print("\n=== SUMMARY ===")
print("  complete stretches: %d   turns: %d" % (len(derivas), len(giros)))
if giros:
    print("  turns: %s" % "  ".join("%+.1f" % g for g in giros))
    print("  mean turn %+.2f degrees, mean error %+.2f, worst %+.2f"
          % (sum(giros)/len(giros), sum(g-90 for g in giros)/len(giros),
             max(giros, key=lambda g: abs(g-90))-90))
if derivas:
    print("  heading drift in the straight stretches: mean %+.2f, worst %+.2f"
          % (sum(derivas)/len(derivas), max(derivas, key=abs)))
    print("  ACCUMULATED (how much the whole square deviates): %+.2f degrees"
          % (sum(derivas) + sum(g-90 for g in giros)))
t = telem()
print("  final yaw: %+.2f" % t["yaw"])
print("\nlog:", REG)
reg.close()
