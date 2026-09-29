# Calibration of the sideways motion: it translates to the RIGHT for a few
# seconds, stops, and comes back to the LEFT by the same amount. It is used
# to see what the mecanum combination really does when asked for a pure
# translation.
#
# 🔴 POSITIVE vy MOVES TO THE LEFT. Measured, not assumed. So going to the
# right uses a negative vy.
#
# 🔴 PUSHES ARE SENT, NOT PULSES. The pulse carries a 600 ms dead man's switch
# and a loop from the Pi takes ~230 ms per iteration: with pulses the robot
# goes in jerks.
#
# 🔑 WHAT TO LOOK AT IN THE RESULT. In a pure translation the four wheels
# should turn at the SAME magnitude and with alternating signs in an X
# pattern. If one falls short, the robot does not translate: it describes an
# arc and twists.

import json, os, time, urllib.request

API = "http://11.11.41.17:8080"
REG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lateral_%s.jsonl" % time.strftime("%Y-%m-%d_%H%M"))

# --- what gets tuned ----------------------------------------------------------
VY         = 80      # out of 255. The measured executable minimum is 40; when stopped, 65
SEGUNDOS   = 2.5    # each direction
PAUSA      = 2.0     # between going and coming back, so it settles
VUELVE     = True    # False to see only the outbound move

MS_TIRADA  = 700
PERIODO    = 0.10
REINTENTOS = 3

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
            ultimo = e
            time.sleep(0.15)
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


def anota(t, etiqueta, vy):
    fila = {"etiqueta": etiqueta, "t": round(time.time() - T0, 2), "vy": vy,
            "dE": t["dE"], "dO": t["dO"], "dN": t["dN"], "dS": t["dS"],
            "yaw": t["yaw"], "yaw_ref": t["yaw_ref"], "corr": t["corr"],
            "rpm": [t["rpm_DI"], t["rpm_DD"], t["rpm_TI"], t["rpm_TD"]],
            "obj": [t["obj_DI"], t["obj_DD"], t["obj_TI"], t["obj_TD"]],
            "pwm": [t["pwm_DI"], t["pwm_DD"], t["pwm_TI"], t["pwm_TD"]],
            "cortadas": t["cortadas"]}
    reg.write(json.dumps(fila) + "\n")
    reg.flush()


def desenvolver(seq):
    out = [seq[0]]
    for x in seq[1:]:
        d = x - out[-1]
        while d > 180: d -= 360
        while d < -180: d += 360
        out.append(out[-1] + d)
    return out


def tanda(nombre, vy):
    a = telem()
    print("\n  %s: vy = %+d for %.1f s" % (nombre, vy, SEGUNDOS))
    print("    leaves from:  right %.0f   left %.0f   yaw %+.2f"
          % (a["dE"], a["dO"], a["yaw"]))
    yaws = [a["yaw"]]; rpms = []; corrs = []; objs = []
    t0 = time.time()
    while time.time() - t0 < SEGUNDOS:
        t = telem()
        mover(vy)
        anota(t, nombre, vy)
        yaws.append(t["yaw"])
        r = [t["rpm_DI"], t["rpm_DD"], t["rpm_TI"], t["rpm_TD"]]
        if max(abs(x) for x in r) > 5:
            rpms.append(r); corrs.append(t["corr"])
            objs.append([t["obj_DI"], t["obj_DD"], t["obj_TI"], t["obj_TD"]])
        time.sleep(PERIODO)
    parar(); time.sleep(1.2)
    b = telem()
    yaws = desenvolver(yaws + [b["yaw"]])
    print("    arrives at:   right %.0f   left %.0f   yaw %+.2f"
          % (b["dE"], b["dO"], b["yaw"]))
    # the displacement is read with the side sensor that still sees a wall
    for campo, lado in (("dE", "derecha"), ("dO", "izquierda")):
        if 10 < a[campo] < 200 and 10 < b[campo] < 200:
            print("    it moved %+.0f mm according to the %s side sensor" % (b[campo] - a[campo], lado))
    print("    it twisted %+.2f degrees" % (yaws[-1] - yaws[0]))
    if rpms:
        n = len(rpms)
        med = [sum(r[i] for r in rpms) / n for i in range(4)]
        print("    mean rpm  DI %+6.1f   DD %+6.1f   TI %+6.1f   TD %+6.1f" % tuple(med))
        mags = [abs(x) for x in med]
        print("    magnitudes: %s   the weakest stays at %d%% of the strongest"
              % (" ".join("%.0f" % m for m in mags), 100 * min(mags) / max(mags)))
        # 🔑 corr is the rpm the heading loop adds to one side and subtracts from
        # the other. If it lives stuck at its cap, the loop wants to correct
        # more than it is allowed to.
        c = [abs(x) for x in corrs]
        print("    heading correction: mean %.1f rpm, maximum %.1f  (typical cap 12)"
              % (sum(c)/len(c), max(c)))
        # 🔴 IN A PURE TRANSLATION THE ALGEBRAIC SUM OF THE FOUR MUST BE ZERO.
        # Whatever is left over is parasitic forward or backward motion: the
        # robot does not translate cleanly, it also drifts forward or backward.
        sumas = [sum(r) for r in rpms]
        print("    sum of the four (0 = clean translation): mean %+.1f rpm, worst %+.1f"
              % (sum(sumas)/len(sumas), max(sumas, key=abs)))
        # and the same with what the firmware REQUESTS, to know whose fault it is
        objm = [sum(o[i] for o in objs) / len(objs) for i in range(4)]
        print("    target      DI %+6.1f   DD %+6.1f   TI %+6.1f   TD %+6.1f   (sum %+.1f)"
              % tuple(objm + [sum(objm)]))
        segui = [med[i] - objm[i] for i in range(4)]
        print("    reaches     DI %+6.1f   DD %+6.1f   TI %+6.1f   TD %+6.1f   rpm of the target"
              % tuple(segui))
        if abs(sum(objm)) > 1.0:
            print("      -> the firmware ALREADY ASKS for a dirty translation: the split is to blame")
        else:
            print("      -> the firmware asks for a clean translation: the wheels do not follow it")
    else:
        print("    🔴 NO WHEEL TURNED: vy = %d does not break the inertia" % vy)
    return b


T0 = time.time()
print("=== LATERAL TRANSLATION: vy %d, %.1f s per direction ===" % (VY, SEGUNDOS))
t = telem()
if int(t["hab"]) == 0:
    _pedir(API + "/orden", {"accion": "banco", "letra": "E"}); time.sleep(1.0)
print("  motors: %d   starting yaw: %+.2f" % (int(telem()["hab"]), t["yaw"]))

try:
    tanda("derecha", -VY)
    if VUELVE:
        time.sleep(PAUSA)
        tanda("izquierda", +VY)
except KeyboardInterrupt:
    print("\ninterrupted")
except Exception as e:
    print("\n🔴 cut because of a failure talking to the Pi: %s" % e)
finally:
    parar()

t = telem()
print("\n=== SUMMARY ===")
print("  ends at: right %.0f   left %.0f   yaw %+.2f" % (t["dE"], t["dO"], t["yaw"]))
print("\nlog:", REG)
reg.close()
