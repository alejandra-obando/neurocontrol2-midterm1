# Lateral translation sweep: test 12 applied to the Y axis.
#
# WHAT IT TESTS. It raises the translation speed in steps and measures, at
# each one, at what rpm each wheel goes and how much it wobbles. The question
# it answers is at what speed the four go THE SAME: as long as they do not go
# the same the robot does not translate cleanly, it twists and drifts forward
# or backward.
#
# 🔴 WHY IT IS NEEDED. Test 12 of Sep 14 measured this while ADVANCING and with
# the wheels in the air, and found that the front right has a different
# gearbox: it reaches 365 rpm where the other three stay at 160-175, and its
# reading is only repeatable above PWM 100, about 90 rpm. The translation we
# tried today asks for 30 rpm, i.e. far below that. Here it is measured with
# the robot ON THE FLOOR and translating, which is the real condition: the
# mecanum rollers do not roll the same sideways as forward.
#
# 🔑 IT GOES BACK AND FORTH. Each step is given to one side and the next to the
# other, so the robot oscillates around its spot instead of going into a wall.
#
# 🔴 POSITIVE vy MOVES TO THE LEFT. Measured, not assumed.

import json, os, time, urllib.request

API = "http://11.11.41.17:8080"
REG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sweep_%s.jsonl" % time.strftime("%Y-%m-%d_%H%M"))

ESCALONES  = [40, 60, 80, 100, 120, 140, 160, 180]
DURACION   = 1.6     # s per step
PAUSA      = 1.0     # s stopped between steps, to separate the measurements
MS_TIRADA  = 700
PERIODO    = 0.10
REINTENTOS = 3
MIN_LIBRE  = 180.0   # mm: below this on the side it is going to, it is skipped

reg = open(REG, "w", encoding="utf-8")


def _pedir(url, datos=None, timeout=5):
    ultimo = None
    for i in range(REINTENTOS):
        try:
            if datos is None:
                with urllib.request.urlopen(url, timeout=timeout) as r:
                    return json.loads(r.read().decode())
            cuerpo = json.dumps(datos).encode()
            req = urllib.request.Request(url, data=cuerpo,
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read().decode())
        except Exception as e:
            ultimo = e; time.sleep(0.15)
    raise ultimo


def telem():
    return _pedir(API + "/estado")["telem"]


def mover(vy):
    _pedir(API + "/orden", {"accion": "manual_pwm", "vx": 0, "vy": int(vy),
                            "w": 0, "ms": MS_TIRADA})


def parar():
    for i in range(4):
        try:
            _pedir(API + "/orden", {"accion": "manual_pwm", "vx": 0, "vy": 0,
                                    "w": 0, "ms": 1}, timeout=3)
            return
        except Exception:
            time.sleep(0.2)


def escalon(vy):
    """One step: sends a fixed vy and returns what each wheel did."""
    a = telem()
    hueco = a["dO"] if vy > 0 else a["dE"]
    if 10 < hueco < MIN_LIBRE:
        print("    vy %+4d: only %.0f mm on that side, skipping it" % (vy, hueco))
        return None
    muestras = []
    yaws = [a["yaw"]]
    t0 = time.time()
    while time.time() - t0 < DURACION:
        t = telem()
        mover(vy)
        r = [t["rpm_DI"], t["rpm_DD"], t["rpm_TI"], t["rpm_TD"]]
        if max(abs(x) for x in r) > 3:
            muestras.append({"rpm": r,
                             "pwm": [t["pwm_DI"], t["pwm_DD"], t["pwm_TI"], t["pwm_TD"]],
                             "obj": [t["obj_DI"], t["obj_DD"], t["obj_TI"], t["obj_TD"]]})
        yaws.append(t["yaw"])
        reg.write(json.dumps({"vy": vy, "t": round(time.time() - T0, 2),
                              "rpm": r, "yaw": t["yaw"], "corr": t["corr"],
                              "dE": t["dE"], "dO": t["dO"]}) + "\n")
        reg.flush()
        time.sleep(PERIODO)
    parar(); time.sleep(PAUSA)
    b = telem()
    d = b["yaw"] - yaws[0]
    while d > 180: d -= 360
    while d < -180: d += 360
    if not muestras:
        print("    vy %+4d: NO WHEEL TURNED" % vy)
        return None
    n = len(muestras)
    med = [sum(abs(m["rpm"][i]) for m in muestras) / n for i in range(4)]
    sd = [(sum((abs(m["rpm"][i]) - med[i]) ** 2 for m in muestras) / n) ** 0.5 for i in range(4)]
    pwm = [sum(abs(m["pwm"][i]) for m in muestras) / n for i in range(4)]
    dispersion = 100.0 * (max(med) - min(med)) / max(med) if max(med) else 0.0
    print("    vy %+4d | rpm %s | noise %s | PWM %s | between wheels %4.1f%% | yaw %+6.2f"
          % (vy,
             " ".join("%5.1f" % x for x in med),
             " ".join("%4.1f" % x for x in sd),
             " ".join("%3.0f" % x for x in pwm),
             dispersion, d))
    return {"vy": vy, "med": med, "sd": sd, "pwm": pwm,
            "dispersion": dispersion, "yaw": d, "n": n}


T0 = time.time()
print("=== LATERAL TRANSLATION SWEEP ===")
print("  vy steps: %s, %.1f s each, alternating side" % (ESCALONES, DURACION))
t = telem()
if int(t["hab"]) == 0:
    _pedir(API + "/orden", {"accion": "banco", "letra": "E"}); time.sleep(1.0)
print("  motors %d | room: right %.0f, left %.0f\n"
      % (int(telem()["hab"]), t["dE"], t["dO"]))
print("             wheel:    DI    DD    TI    TD")

filas = []
try:
    for k, v in enumerate(ESCALONES):
        r = escalon(-v if k % 2 == 0 else +v)     # alternates right / left
        if r: filas.append(r)
except KeyboardInterrupt:
    print("\ninterrupted")
except Exception as e:
    print("\n🔴 cut because of a failure talking to the Pi: %s" % e)
finally:
    parar()

print("\n=== SUMMARY ===")
if filas:
    print("  vy | spread between wheels | noise of worst wheel | yaw")
    for f in filas:
        peor = max(100.0 * f["sd"][i] / f["med"][i] if f["med"][i] else 0 for i in range(4))
        print("  %+4d |        %5.1f%%         |      %5.1f%%      | %+6.2f"
              % (f["vy"], f["dispersion"], peor, f["yaw"]))
    mejor = min(filas, key=lambda f: f["dispersion"])
    print("\n  🔑 the four are most even at vy %+d: %.1f%% difference between the"
          % (mejor["vy"], mejor["dispersion"]))
    print("     fastest and the slowest, and the robot twisted %+.2f degrees"
          % mejor["yaw"])
print("\nlog:", REG)
reg.close()
