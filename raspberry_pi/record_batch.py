#!/usr/bin/env python3
"""Records the raw telemetry of a batch of tests, for diagnosis.

    python3 record_batch.py                      # records until Ctrl+C
    python3 record_batch.py --name forward_1     # with a custom name

WHY THE SERIAL PORT IS NOT READ HERE: main.py holds it exclusively. This
script asks the interface itself (localhost) for the telemetry, which
already has it parsed and fresh. Being local it does not go through WiFi, so
it inherits neither the latency nor the losses of the network.

HOW IT NEITHER LOSES NOR DUPLICATES SAMPLES: it polls FASTER than the ESP
sends (every 20 ms against the 50 of the CSV) and discards whatever comes
with the same `t_ms`, which is the ESP's own clock. This way each line of
the file is a distinct line of the ESP, and if one is missing it shows:
`salto_ms` says how much time passed since the previous one, and with the
CSV at 50 ms any value above ~80 is a sample that did not arrive.

WHAT TO LOOK AT AFTERWARDS, by column:
    yaw, corr           -- where it faces and how much the heading is being corrected
    rpm_*  vs  obj_*    -- what each wheel does against what is asked of it
    pwm_*               -- what the loop is giving it
    hab, estado, cmd    -- whether the motors are enabled and in which mode
    cortadas            -- bit mask: 0 means none cut off
    salto_ms            -- telemetry gaps (see above)
"""

import argparse
import csv
import json
import os
import sys
import time
import urllib.request

URL = "http://127.0.0.1:8080/estado"
PERIODO_CONSULTA_S = 0.020      # faster than the CSV, on purpose
SALTO_SOSPECHOSO_MS = 80        # with the CSV at 50 ms


def leer_estado(url, timeout=1.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--name", "--nombre", dest="nombre", default="", help="label for the file")
    p.add_argument("--url", default=URL)
    p.add_argument("--dir", default="batches", help="folder to save into")
    args = p.parse_args()

    os.makedirs(args.dir, exist_ok=True)
    sello = time.strftime("%Y-%m-%d_%H%M%S")
    nombre = "{}_{}.csv".format(sello, args.nombre) if args.nombre else sello + ".csv"
    ruta = os.path.join(args.dir, nombre)

    # Wait for the first sample to know which fields the firmware publishes:
    # this way the file comes out with whatever columns there are, without a
    # hand-written list.
    print("waiting for telemetry from {} ...".format(args.url))
    while True:
        try:
            d = leer_estado(args.url)
            if d.get("telem"):
                break
        except Exception as e:
            print("  no answer ({}), retrying".format(e))
        time.sleep(0.5)

    campos = list(d["telem"].keys())
    f = open(ruta, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["t_pc_s", "salto_ms"] + campos)

    print("RECORDING to {}".format(ruta))
    print("  {} columns, one row per ESP line".format(len(campos) + 2))
    print("  Ctrl+C to close\n")

    t0 = time.time()
    ultimo_tms = None
    n = 0
    huecos = 0
    try:
        while True:
            try:
                d = leer_estado(args.url)
            except Exception:
                time.sleep(PERIODO_CONSULTA_S)
                continue
            t = d.get("telem")
            if not t:
                time.sleep(PERIODO_CONSULTA_S)
                continue
            tms = t.get("t_ms")
            if tms is None or tms == ultimo_tms:
                time.sleep(PERIODO_CONSULTA_S)
                continue          # the same as before: not a new sample

            salto = "" if ultimo_tms is None else int(tms - ultimo_tms)
            if salto != "" and salto > SALTO_SOSPECHOSO_MS:
                huecos += 1
            ultimo_tms = tms
            w.writerow([round(time.time() - t0, 3), salto] +
                       [t.get(c, "") for c in campos])
            n += 1
            if n % 100 == 0:
                f.flush()
                print("  {} samples  ({:.1f} s)  gaps: {}".format(
                    n, time.time() - t0, huecos), end="\r", flush=True)
            time.sleep(PERIODO_CONSULTA_S)
    except KeyboardInterrupt:
        pass
    finally:
        f.flush()
        f.close()

    dur = time.time() - t0
    print("\n\nclosed: {} samples in {:.1f} s".format(n, dur))
    if n:
        print("  one every {:.0f} ms on average".format(dur * 1000.0 / n))
    print("  gaps longer than {} ms: {}".format(SALTO_SOSPECHOSO_MS, huecos))
    print("  file: {}".format(os.path.abspath(ruta)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
