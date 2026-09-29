#!/usr/bin/env python3
"""Headless test of data_logger.py, without robot and without cable.

    python3 test_logger.py

WHAT IT TESTS
    1. That the CSV is written with the same header declared by CAMPOS_CSV
       and that it respects the throttle (`periodo_s`): sending telemetry at
       10 Hz writes fewer rows than were observed.
    2. That the diagnosis triggers the right alert with data made by hand
       for each case (drifting wheel, AGC out of range, hung ToF, uncorrected
       yaw, stalls, cut-off wheel) -- and that it triggers NONE with healthy
       data.
    3. That the report (.md) is rewritten and contains the expected
       sections.

WHAT IT DOES **NOT** TEST
    The real serial link or the firmware: it uses hand-made Telemetria
    objects, as test_headless.py does with its ESPFalsa.
"""

import os
import sys
import tempfile

_AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _AQUI)

import serial_link as en                     # noqa: E402
import data_logger as reg_mod                # noqa: E402

fallos = []


def revisar(cond, texto):
    print("  {}  {}".format("ok  " if cond else "FAIL ", texto))
    if not cond:
        fallos.append(texto)


def campos_base(**over):
    """A healthy telemetry sample: ToF far from walls, wheels following
    their target, yaw glued to the reference. `over` overrides whatever is
    needed to build a specific case."""
    v = {"t_ms": 0, "ev": en.EV_NADA, "imu_ok": 1, "hab": 1,
         "estado": 1, "cmd": 5, "err_giro": 0.0,
         "yaw": 0.0, "yaw_ref": 0.0, "corr": 0.0, "cuad": 0.0, "ecen": 0.0,
         "roll": 0.0, "pitch": 0.0, "cm_tramo": 0.0, "celdas": 0,
         "ret": 0.0, "mem": 0.0, "ciclo_ms": 7, "vivas": 15, "cortadas": 0}
    for d in en.DIRS4:
        v["d" + d] = 500.0
        v["p" + d] = 0
    for i in range(4):
        v["nr%d" % i] = 0.5
        v["mr%d" % i] = 0.5
    for n in en.RUEDAS:
        v["rpm_" + n] = 75.0
        v["obj_" + n] = 75.0
        v["pwm_" + n] = 150.0
        v["desp_" + n] = 90.0
        v["agc_" + n] = 128.0
    v.update(over)
    faltan = set(en.CAMPOS_POR_DEFECTO) - set(v)
    assert not faltan, "missing fields {}".format(faltan)
    return v


GANANCIAS = {"kp": 0.25, "ki": 0.20, "kd": 0.0, "kpy": 3.6, "kiy": 0.0,
            "kdy": 0.7, "corrmax": 30.0, "signoyaw": 1.0,
            "crucero": 0.228, "vgiro": 60.0, "celda": 27.0, "odom": 1.0}


def alimentar(reg, filas, ganancias=GANANCIAS):
    for campos in filas:
        reg.observar(en.Telemetria(dict(campos)), ganancias)


with tempfile.TemporaryDirectory() as tmp:
    ruta_csv = os.path.join(tmp, "sensor_log.csv")
    ruta_md = os.path.join(tmp, "sensor_log.md")

    # --- 1. CSV throttle ------------------------------------------------
    print("CSV throttle:")
    reg = reg_mod.Registro(ruta_csv, ruta_md, periodo_s=1e6,  # never expires
                           informe_cada=1000, ventana=50)
    alimentar(reg, [campos_base(t_ms=i * 100) for i in range(30)])
    with open(ruta_csv, encoding="utf-8") as f:
        lineas = f.readlines()
    revisar(lineas[0].strip() == ",".join(reg_mod.CAMPOS_CSV),
            "the CSV header matches CAMPOS_CSV")
    # with a huge periodo_s only the FIRST row is written (the rest arrive
    # before the throttle expires), even though 30 samples were observed.
    revisar(len(lineas) == 2, "30 samples at 10 Hz with periodo_s=1e6 "
            "write a single row (got {})".format(len(lineas) - 1))
    reg.cerrar()

    # --- 2. diagnosis, one case at a time --------------------------------
    print("\ndiagnosis:")

    def solo_diagnostico(*filas, periodo_s=0.0):
        r = reg_mod.Registro(ruta_csv + ".d", ruta_md + ".d",
                             periodo_s=periodo_s, informe_cada=1000, ventana=50)
        alimentar(r, filas)
        d = r.diagnostico()
        r.cerrar()
        os.remove(ruta_csv + ".d")
        return d

    def hay_alerta(diagnostico, pista):
        return any(nivel == "alerta" and pista in texto
                   for nivel, texto in diagnostico)

    # healthy robot: nothing at all. The ToF carry a bit of jitter (i % 3) on
    # purpose -- a PERFECTLY constant reading is exactly the signature of a
    # hung sensor that the alert below is designed to catch, so faking
    # "healthy" with a fixed value would trigger that very alert.
    d = solo_diagnostico(*[campos_base(t_ms=i * 100,
                                       dN=500.0 + (i % 3), dE=500.0 + (i % 2),
                                       dS=500.0 + ((i + 1) % 3),
                                       dO=500.0 + ((i + 2) % 2))
                          for i in range(20)])
    revisar(all(nivel == "info" for nivel, _ in d),
            "healthy data (with realistic jitter) triggers no alert")

    # wheel DD stuck: asks 75, measures 20 (55 rpm of error, above the
    # threshold of 25).
    filas = [campos_base(t_ms=i * 100, **{"rpm_DD": 20.0}) for i in range(20)]
    d = solo_diagnostico(*filas)
    revisar(hay_alerta(d, "wheel DD"), "wheel DD with 55 rpm of error triggers "
            "the drifting wheel alert")

    # AGC of TI outside 80-220 (badly placed magnet).
    filas = [campos_base(t_ms=i * 100, **{"agc_TI": 250.0}) for i in range(10)]
    d = solo_diagnostico(*filas)
    revisar(hay_alerta(d, "wheel TI") and "AGC" in " ".join(t for _, t in d),
            "AGC of TI at 250 triggers the badly placed magnet alert")

    # East ToF hung: 25 IDENTICAL readings in a row, robot MOVING FORWARD
    # (estado=1, the default value of campos_base).
    filas = [campos_base(t_ms=i * 100, dE=444.0) for i in range(25)]
    d = solo_diagnostico(*filas)
    revisar(hay_alerta(d, "ToF E"), "25 identical readings on E, moving forward, "
            "trigger the hung sensor alert")

    # The same constant reading, but with the robot IDLE (estado=0): it is
    # not a hung sensor, it is what is expected with the robot stopped. It
    # must NOT trigger anything.
    filas = [campos_base(t_ms=i * 100, dE=444.0, estado=0) for i in range(25)]
    d = solo_diagnostico(*filas)
    revisar(not hay_alerta(d, "ToF E"), "the same constant reading with the "
            "robot IDLE does not trigger the hung sensor alert")

    # yaw 40 degrees away from the reference and uncorrected.
    filas = [campos_base(t_ms=i * 100, yaw=40.0, yaw_ref=0.0) for i in range(15)]
    d = solo_diagnostico(*filas)
    revisar(hay_alerta(d, "signoyaw"), "yaw 40 degrees from the reference "
            "triggers the heading alert")

    # 3 stalls in the session.
    filas = [campos_base(t_ms=i * 100, ev=(en.EV_BLOQUEO if i < 3 else en.EV_NADA))
             for i in range(10)]
    d = solo_diagnostico(*filas)
    revisar(hay_alerta(d, "stalls"), "3 stalls trigger the alert of "
            "odometry not advancing")

    # 1 cut-off wheel.
    filas = [campos_base(t_ms=0, ev=en.EV_RUEDA_CORTADA)]
    d = solo_diagnostico(*filas)
    revisar(hay_alerta(d, "wheel cut-off(s)"), "1 cut-off wheel event "
            "triggers the measure-directions alert")

    # --- 3. the report is written and has the sections --------------------
    print("\nreport (.md):")
    reg = reg_mod.Registro(ruta_csv + ".i", ruta_md + ".i", periodo_s=0.0,
                           informe_cada=5, ventana=50)
    alimentar(reg, [campos_base(t_ms=i * 100, **{"rpm_DI": 10.0}) for i in range(12)])
    reg.cerrar()
    with open(ruta_md + ".i", encoding="utf-8") as f:
        md = f.read()
    for seccion in ("# Sensor log", "## Diagnosis", "## Wheels",
                    "## ToF", "## Heading", "## Session events", "## Gains"):
        revisar(seccion in md, "the report has the section {!r}".format(seccion))
    revisar("wheel DI" in md, "the report mentions the wheel that drifted")
    os.remove(ruta_csv + ".i")

print("\n{}".format("ALL GOOD" if not fallos else "{} FAILED: {}".format(
    len(fallos), fallos)))
sys.exit(1 if fallos else 0)
