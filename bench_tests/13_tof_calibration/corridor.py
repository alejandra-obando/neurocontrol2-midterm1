# Back and forth in a corridor: it advances until the front sensor sees
# PARADA mm, backs up until the rear one sees PARADA, and repeats. It centers
# with the two side sensors by translating sideways (mecanum), without
# turning: the robot sweeps 263 mm when turning and the corridor is 250.
#
# 🔴 THE CONTROL GOES AS A PUSH ('#t=' with ms), NOT AS A PULSE. With the '#m='
# pulse the ESP brakes by itself after 600 ms without a message, and between
# reading the status and sending the command the real cycle averaged 230 ms
# WITH PEAKS that exceeded the deadline: the robot advanced in jerks, 0.023
# m/s real against 0.12 requested.
# The push is sustained by the ESP on its own, and each new command renews
# the deadline. Safety is not lost: if this script dies, the push ends by
# itself.
#
# 🔴 POSITIVE vy MOVES TO THE LEFT. Measured on Sep 15, not assumed.

import json, os, math, sys, time, urllib.request

API = "http://11.11.41.17:8080"
REG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "corridor_%s.jsonl" % time.strftime("%Y-%m-%d_%H%M"))

PASADAS      = 5
PARADA       = 60.0    # mm the sensor in the direction of motion must see to stop
LENTO        = 200.0   # mm: below this it goes slowly
V_CRUCERO    = 102     # out of 255 -> 0.12 m/s
V_LENTA      = 35      # out of 255 -> 0.041 m/s, so the overshoot is short
KP_LAT       = 0.6     # vy per mm of difference between side sensors
TOPE_VY      = 50      # out of 255 -> 0.059 m/s
BANDA_CENTRO = 6.0     # mm of difference considered centered
TOPE_PASADA  = 45.0    # s per pass before aborting
PERIODO      = 0.10    # s between commands (the ESP brakes after 600 ms without one)

reg = open(REG, "w", encoding="utf-8")

def post(c):
    d = json.dumps(c).encode()
    r = urllib.request.Request(API + "/orden", data=d,
                               headers={"Content-Type": "application/json"})
    return urllib.request.urlopen(r, timeout=5).read().decode()

def telem():
    with urllib.request.urlopen(API + "/estado", timeout=5) as r:
        return json.load(r)["telem"]

MS_TIRADA = 700   # what the ESP sustains without news; renewed every loop

def mover(vx, vy):
    post({"accion": "manual_pwm", "vx": int(vx), "vy": int(vy), "w": 0,
          "ms": MS_TIRADA})

def parar():
    post({"accion": "manual_pwm", "vx": 0, "vy": 0, "w": 0})

def anota(t, etiqueta, vx, vy):
    f = {"etiqueta": etiqueta, "t": round(time.time() - T0, 2),
         "dN": t["dN"], "dE": t["dE"], "dS": t["dS"], "dO": t["dO"],
         "yaw": t["yaw"], "vx": vx, "vy": vy,
         "rpm": [t["rpm_DI"], t["rpm_DD"], t["rpm_TI"], t["rpm_TD"]],
         "cortadas": t["cortadas"]}
    reg.write(json.dumps(f) + "\n"); reg.flush()

# 🔴 QUARANTINE AFTER LOSING THE WALL. Measured on Sep 15: when a ToF sees
# again after a stretch at 600, the firmware's 5-sample average filter drags
# that 600 and publishes a FALSE ramp -- 525, 421, 313, 203, 109 in one second
# -- that does not correspond to any real distance. The loop reacted to that
# ghost and sent vy to the cap. Discarding the 600 is not enough: the
# following samples must be discarded too, until the filter has been fully
# renewed.
MUESTRAS_CUARENTENA = 5      # the 5 of the filter, ~0.8 s
_cuarentena = {"dE": 0, "dO": 0}

def lateral_fiable(t):
    """Updates the quarantine and says whether the two side sensors can be trusted."""
    ok = True
    for k in ("dE", "dO"):
        if t[k] >= 599:
            _cuarentena[k] = MUESTRAS_CUARENTENA
            ok = False
        elif _cuarentena[k] > 0:
            _cuarentena[k] -= 1
            ok = False
    return ok

def correccion(t):
    """vy to center. error>0 = farther from the right = stuck to the
    left -> it has to go right -> NEGATIVE vy."""
    if not lateral_fiable(t):
        return 0.0, None
    err = t["dE"] - t["dO"]
    vy = -KP_LAT * err
    return max(-TOPE_VY, min(TOPE_VY, vy)), err

# 🔴 THERE IS NO STATIC CENTERING. Measured on Sep 15: stopped, eight seconds
# of vy did not move the robot sideways (the four wheels have to break free
# at the same time at low speed); while advancing, the same loop took it from
# 45 mm off-center to 9. It is the same nuance as the turn: a wheel does not
# START slowly, but in motion it does sustain a slow speed. So the centering
# goes inside the motion.
def estado_lateral(etiqueta=""):
    t = telem()
    print("  %sright %.0f  left %.0f  (difference %+.0f mm)"
          % (etiqueta, t["dE"], t["dO"], t["dE"] - t["dO"]))
    return t

def pasada(n, sentido):
    campo = "dN" if sentido > 0 else "dS"
    nombre = "IDA" if sentido > 0 else "VUELTA"
    t = telem()
    print("\n  %s %d: starts with %s = %.0f mm" % (nombre, n, campo, t[campo]))
    t0 = time.time()
    errs = []
    yaw0 = t["yaw"]
    while True:
        t = telem()
        d = t[campo]
        if d <= PARADA:
            motivo = "sensor"; break
        if time.time() - t0 > TOPE_PASADA:
            motivo = "TIEMPO"; break
        if t["cortadas"] > 0:
            motivo = "RUEDA CORTADA"; break
        v = V_CRUCERO if d > LENTO else V_LENTA
        vy, err = correccion(t)
        if err is not None:
            errs.append(err)
        mover(sentido * v, vy)
        anota(t, nombre.lower() + str(n), sentido * v, vy)
        time.sleep(PERIODO)
    parar()
    time.sleep(1.2)
    t = telem()
    dur = time.time() - t0
    desv = (sum(e * e for e in errs) / len(errs)) ** 0.5 if errs else float("nan")
    print("    stops by %s in %.1f s   %s final = %.0f mm" % (motivo, dur, campo, t[campo]))
    print("    centering while moving: mean difference |%.1f| mm, maximum %.0f"
          % (desv, max((abs(e) for e in errs), default=0)))
    print("    heading: %+.2f -> %+.2f  (drift %+.2f degrees)" % (yaw0, t["yaw"], t["yaw"] - yaw0))
    return motivo

T0 = time.time()
print("=== BACK AND FORTH IN CORRIDOR, %d passes ===" % PASADAS)
t = telem()
if int(t["hab"]) == 0:
    post({"accion": "banco", "letra": "E"}); time.sleep(1.0)
print("motors:", int(telem()["hab"]))
estado_lateral("starting: ")

try:
    for n in range(1, PASADAS + 1):
        if pasada(n, +1) not in ("sensor",): break
        if pasada(n, -1) not in ("sensor",): break
except KeyboardInterrupt:
    print("\ninterrupted")
finally:
    parar()
    print("\nstopped. log:", REG)
    reg.close()
