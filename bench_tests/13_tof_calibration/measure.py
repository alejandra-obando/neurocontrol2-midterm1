# Takes a calibration point of a ToF sensor: you tell it which sensor and at
# what real distance the board is, and it saves the mean of a window of
# readings.
#
#   python3 measure.py N 150      -> front, board at 150 mm
#   python3 measure.py E 250      -> right, board at 250 mm
#   python3 measure.py --ajuste   -> fits the line with what was measured so far
#
# The distance is measured FROM THE SENSOR MODULE, not from the chassis.

import csv, json, os, statistics, sys, time, urllib.request

API = "http://11.11.41.17:8080"
AQUI = os.path.dirname(os.path.abspath(__file__))
DATOS = os.path.join(AQUI, "points.csv")
CAMPO = {"N": "dN", "E": "dE", "S": "dS", "O": "dO"}
# 🔴 Since Sep 15 the firmware ALREADY corrects the readings (BLOCK 04b), so
# /estado returns real distance, not the sensor reading. Calibrating needs the
# raw value, and it is rebuilt by undoing the same line the firmware applies:
# 'read = c + m * corrected'. It is exact as long as there is no saturation,
# i.e. everywhere except the 600 cap.
FIRMWARE_C = {"N": 46.7, "E": 0.0, "S": 15.6, "O": 24.9}
FIRMWARE_M = {"N": 0.890, "E": 1.040, "S": 1.032, "O": 1.043}

def a_bruto(sensor, corregido):
    return FIRMWARE_C[sensor] + FIRMWARE_M[sensor] * corregido
NOMBRE = {"N": "frente", "E": "derecha", "S": "atras", "O": "izquierda"}
ESPERA_FILTRO = 1.2   # the firmware averages the last 5: 5 x 160 ms
VENTANA = 8.0

def telem():
    with urllib.request.urlopen(API + "/estado", timeout=6) as r:
        return json.load(r)["telem"]

def medir(sensor, real):
    campo = CAMPO[sensor]
    print("sensor %s (%s), board at %d mm" % (sensor, NOMBRE[sensor], real))
    print("  waiting %.1f s for the 5-sample average filter to renew..." % ESPERA_FILTRO)
    time.sleep(ESPERA_FILTRO)
    print("  measuring %.0f s..." % VENTANA)
    t0 = time.time()
    v = []
    corr = []
    while time.time() - t0 < VENTANA:
        try:
            c = telem()[campo]
            corr.append(c)
            v.append(a_bruto(sensor, c))
        except Exception:
            pass
        time.sleep(0.08)
    media = statistics.mean(v)
    desv = statistics.pstdev(v)
    distintos = len(set(v))
    print()
    print("  readings: %d   distinct values: %d" % (len(v), distintos))
    print("  mean: %.1f mm   spread: +-%.1f   min %.0f  max %.0f"
          % (media, desv, min(v), max(v)))
    print("  RAW ERROR: %+.1f mm with respect to the real %d" % (media - real, real))
    mc = statistics.mean(corr)
    print("  what the firmware publishes today: %.1f  (error %+.1f)" % (mc, mc - real))
    if media >= 599.0 and desv == 0.0:
        print("  WARNING: stuck at the cap without variation = the sensor does NOT see the board")
    if distintos == 1:
        print("  WARNING: a single value in the whole window, suspicious")
    nuevo = not os.path.exists(DATOS)
    with open(DATOS, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if nuevo:
            w.writerow(["fecha", "sensor", "real_mm", "lee_mm", "desv_mm",
                        "n", "distintos"])
        w.writerow([time.strftime("%Y-%m-%d %H:%M"), sensor, real,
                    round(media, 1), round(desv, 1), len(v), distintos])
    print("  saved in points.csv")

def ajuste():
    if not os.path.exists(DATOS):
        print("no measured points yet"); return
    filas = list(csv.DictReader(open(DATOS, encoding="utf-8")))
    for s in ("N", "E", "S", "O"):
        p = [(float(f["real_mm"]), float(f["lee_mm"])) for f in filas if f["sensor"] == s]
        if len(p) < 2:
            continue
        n = len(p)
        sx = sum(a for a, _ in p); sy = sum(b for _, b in p)
        sxx = sum(a * a for a, _ in p); sxy = sum(a * b for a, b in p)
        m = (n * sxy - sx * sy) / (n * sxx - sx * sx)
        c = (sy - m * sx) / n
        print()
        print("sensor %s (%s), %d points" % (s, NOMBRE[s], n))
        print("  read = %.1f + %.3f * real" % (c, m))
        print("  to correct:  real = (read - %.1f) / %.3f" % (c, m))
        for real, lee in sorted(p):
            pred = c + m * real
            print("    real %4.0f  ->  read %6.1f   (line %6.1f, residual %+.1f)"
                  % (real, lee, pred, lee - pred))

if len(sys.argv) == 2 and sys.argv[1] == "--ajuste":
    ajuste()
elif len(sys.argv) == 3 and sys.argv[1] in CAMPO:
    medir(sys.argv[1], int(sys.argv[2]))
else:
    print(__doc__ or "usage: measure.py [N|E|S|O] <mm>   |   measure.py --ajuste")
