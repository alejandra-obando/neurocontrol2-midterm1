"""Sensor logger: continuous data capture to know what to tune.

WHAT IT IS AND WHY IT IS NOT `RUTA_LOG`
    `RUTA_LOG` (main.py) is one row per RL DECISION: a cell, a direction, a
    reward. It is used to understand the POLICY.

    This is one row per TELEMETRY sample -- at a low rate, not at the 10 Hz
    of the cable -- with ALL the raw sensor numbers and the gains in force
    at that instant. It is used to understand the HARDWARE: why a wheel
    falls short, why a ToF is useless, whether the heading correction is
    doing anything. It is the "what to modify" question of the exam, not
    "why did it choose that cell".

TWO OUTPUTS, AND WHY THERE ARE TWO
    - `sensor_log.csv`: one row every `periodo_s`, with a header.
      It is OPENED IN APPEND MODE and not truncated between sessions --
      unlike RUTA_LOG, which main.abrir_log() rewrites on every start -- so
      one session can be compared with the previous one in a spreadsheet.
    - `sensor_log.md`: a READABLE document that is REWRITTEN ENTIRELY every
      `informe_cada` rows. It is not a growing history: it is the state of
      the moving window RIGHT NOW, which is what is useful to decide what to
      touch at this instant, not what happened half an hour ago.

🔑 THE STATISTICS ARE ALWAYS UPDATED, THE DISK IS TOUCHED RARELY. Every
`observar()` that arrives (10 Hz, from the `_bombear` thread of the link)
puts its value into a moving window in RAM, which is free. Only the write to
disk --CSV and, more expensive, the .md-- is throttled. This way the live
diagnosis the interface queries (`diagnostico()`) is always fresh, without
the Pi writing a file 10 times per second.

HOW TO READ THE DIAGNOSIS: each finding states the suspicion AND the number
that supports it ("34 rpm of mean error"), never just "something is wrong"
-- so that deciding is reading, not guessing. The thresholds (UMBRAL_*) are
reasonable starting points, not measured on THIS robot: if a finding shows
up with a healthy robot, raise the threshold instead of ignoring the alert.
"""

import csv
import os
import threading
import time
from collections import deque

import serial_link as en

RUEDAS = en.RUEDAS   # ("DI", "DD", "TI", "TD")
DIRS = en.DIRS4       # ("N", "E", "S", "O") -- chassis frame

# Names of the gains logged together with the telemetry: they are the same
# as en.PARAMETROS, so the report can say with which gain each sample was
# taken without having to cross two files by hand.
GANANCIAS = [p[0] for p in en.PARAMETROS]

# CSV fields. The order is fixed (first row = header); new fields go AT THE
# END, just like in the ESP CSV.
CAMPOS_CSV = (
    ["t", "estado", "cmd", "ev", "err_giro"]
    + ["d" + d for d in DIRS] + ["p" + d for d in DIRS]
    + ["yaw", "yaw_ref", "corr"]
    + ["rpm_" + r for r in RUEDAS] + ["obj_" + r for r in RUEDAS]
    + ["pwm_" + r for r in RUEDAS] + ["desp_" + r for r in RUEDAS]
    + ["agc_" + r for r in RUEDAS]
    + ["vivas", "cortadas"]
    + GANANCIAS
)

# Diagnosis thresholds. Reasonable starting points (see the file header),
# not measured on the physical robot: adjust here if a finding shows up with
# a healthy robot, or if one that should trigger does not.
UMBRAL_ERROR_RUEDA_RPM = 25.0     # |measured rpm - requested rpm| on average
UMBRAL_AGC_BAJO, UMBRAL_AGC_ALTO = 80, 220   # outside this, badly placed magnet
UMBRAL_YAW_DESVIO_DEG = 15.0      # mean gap between yaw and its reference
TOF_ATASCADO_MUESTRAS = 20        # IDENTICAL readings in a row = hung sensor
BLOQUEOS_PARA_ALERTA = 3
CORTES_PARA_ALERTA = 1


class Registro:
    """Samples the telemetry at a low rate and keeps statistics per sensor.

    A single lock: the data it protects is small (one moving window per
    field) and it is written more than it is read, so splitting the lock
    would save nothing and would add ways to deadlock.
    """

    def __init__(self, ruta_csv, ruta_md, periodo_s=0.5, informe_cada=30,
                 ventana=400, activo=True):
        self.ruta_csv = ruta_csv
        self.ruta_md = ruta_md
        self.periodo_s = periodo_s
        self.informe_cada = max(1, informe_cada)
        self.activo = activo
        self._lock = threading.Lock()
        self._t_ultima_fila = 0.0
        self._muestras = 0          # total observed (10 Hz)
        self._filas_csv = 0         # total written to the csv (throttled)
        self._ventanas = {c: deque(maxlen=ventana)
                          for c in CAMPOS_CSV if c not in ("t", "estado", "cmd")}
        # [count, last_value] per direction, to detect a hung ToF.
        self._tof_repetidas = {d: [0, None] for d in DIRS}
        self._eventos = {}          # event_name -> count (whole session)
        self._f = None
        self._w = None
        self._t_informe = None      # when the last .md was written
        if self.activo:
            self._abrir_csv()

    def _abrir_csv(self):
        nuevo = not os.path.exists(self.ruta_csv)
        self._f = open(self.ruta_csv, "a", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        if nuevo:
            self._w.writerow(CAMPOS_CSV)
            self._f.flush()

    # ------------------------------------------------------------------
    def observar(self, telem, ganancias):
        """Call with EVERY telemetry sample that arrives (used by the
        `on_telemetria` callback of the link). `ganancias` is the dict of
        values in force -- the same one the interface uses for the sliders.
        """
        if not self.activo:
            return
        campos = telem.campos
        nombre_evento = en.NOMBRE_EVENTO.get(telem.evento, "")

        with self._lock:
            for c, ventana in self._ventanas.items():
                if c in campos:
                    ventana.append(campos[c])
                elif c in ganancias:
                    ventana.append(ganancias[c])
            # 🔴 IT ONLY COUNTS WHILE THE ROBOT MOVES FORWARD OR TURNS. With the
            # robot stopped (IDLE, or the 10 s wait of the SQUARE figure) the
            # filtered ToF gives the SAME reading for seconds by design --
            # that is not a hung sensor, it is expected. Counting there too
            # triggered a false alarm every time the robot stopped, which is
            # exactly when the diagnosis needs to be credible (discovered
            # with the headless test: a "healthy" fixture with a constant ToF
            # triggered this very alert).
            en_movimiento = telem.estado in (1, 2, 6)   # moving forward, turning, manual
            for d in DIRS:
                v = campos.get("d" + d)
                cont, ultimo = self._tof_repetidas[d]
                if not en_movimiento:
                    self._tof_repetidas[d] = [0, v]
                else:
                    self._tof_repetidas[d] = ([cont + 1, v] if v == ultimo
                                              else [1, v])
            if nombre_evento and nombre_evento != "nada":
                self._eventos[nombre_evento] = self._eventos.get(nombre_evento, 0) + 1
            self._muestras += 1
            muestras = self._muestras

        ahora = time.time()
        if ahora - self._t_ultima_fila < self.periodo_s:
            return
        self._t_ultima_fila = ahora
        self._escribir_fila(ahora, telem, ganancias)
        if self._filas_csv % self.informe_cada == 0:
            self.escribir_informe()

    def _escribir_fila(self, ahora, telem, ganancias):
        c = telem.campos
        fila = [round(ahora, 2), telem.estado, telem.cmd]
        for nombre in CAMPOS_CSV[3:]:
            if nombre in c:
                fila.append(c[nombre])
            elif nombre in ganancias:
                fila.append(ganancias[nombre])
            else:
                fila.append("")
        with self._lock:
            if self._w is None:
                return
            self._w.writerow(fila)
            self._f.flush()
            self._filas_csv += 1

    # ---------------------------------------------------------- diagnosis
    def _stats(self, nombre):
        with self._lock:
            datos = [v for v in self._ventanas.get(nombre, ()) if v is not None]
        if not datos:
            return None
        return {"n": len(datos), "min": min(datos), "max": max(datos),
                "media": sum(datos) / len(datos)}

    def diagnostico(self):
        """List of (level, text). level is 'alerta' (alert) or 'info'. It is
        recomputed with what is in the window RIGHT NOW -- cheap, it can be
        called on every interface query without touching the disk."""
        hallazgos = []

        for r in RUEDAS:
            rpm, obj = self._stats("rpm_" + r), self._stats("obj_" + r)
            if rpm and obj and obj["media"] > 5:
                error = abs(rpm["media"] - obj["media"])
                if error > UMBRAL_ERROR_RUEDA_RPM:
                    hallazgos.append(("alerta",
                        "wheel {}: {:.0f} rpm of mean error (requested {:.0f}, "
                        "measured {:.0f}) -- check take-off (take-off PWM "
                        "in the wheel table) or the sign of its encoder"
                        .format(r, error, obj["media"], rpm["media"])))
            agc = self._stats("agc_" + r)
            if agc and agc["n"] > 5 and not (UMBRAL_AGC_BAJO <= agc["media"] <= UMBRAL_AGC_ALTO):
                hallazgos.append(("alerta",
                    "wheel {}: mean AGC {:.0f} outside {}-{} -- AS5600 "
                    "magnet off-center or at the wrong distance from the chip"
                    .format(r, agc["media"], UMBRAL_AGC_BAJO, UMBRAL_AGC_ALTO)))

        for d in DIRS:
            cont, _ = self._tof_repetidas[d]
            if cont >= TOF_ATASCADO_MUESTRAS:
                hallazgos.append(("alerta",
                    "ToF {}: {} IDENTICAL readings in a row -- hung sensor, "
                    "or wrongly assigned mux channel (config.TOF_REMAPA / BLOCK 03)"
                    .format(d, cont)))

        yaw, ref = self._stats("yaw"), self._stats("yaw_ref")
        if yaw and ref and yaw["n"] > 10:
            desvio = abs(yaw["media"] - ref["media"])
            if desvio > UMBRAL_YAW_DESVIO_DEG:
                hallazgos.append(("alerta",
                    "yaw drifts {:.1f}° from the reference on average -- "
                    "if the robot always veers to the same side, check signoyaw"
                    .format(desvio)))

        if self._eventos.get("bloqueo", 0) >= BLOQUEOS_PARA_ALERTA:
            hallazgos.append(("alerta",
                "{} stalls in the session: odometry not advancing with an active "
                "setpoint (wheel slipping or dead encoder)"
                .format(self._eventos["bloqueo"])))
        if self._eventos.get("rueda_cortada", 0) >= CORTES_PARA_ALERTA:
            hallazgos.append(("alerta",
                "{} wheel cut-off(s) by the controller: measure encoder "
                "directions ('V', robot on its stand with the wheels in the air)"
                .format(self._eventos["rueda_cortada"])))
        if self._eventos.get("err_giro", 0) >= 1:
            hallazgos.append(("alerta",
                "{} turn(s) that did not close (err_giro): check the IMU or raise "
                "TIMEOUT_GIRO_MS / lower VEL_GIRO_MIN_DPS"
                .format(self._eventos["err_giro"])))

        if not hallazgos:
            hallazgos.append(("info", "no suspicions with the current data"))
        return hallazgos

    def instantanea(self):
        """What /estado queries: cheap, called 5 times per second."""
        with self._lock:
            muestras, filas = self._muestras, self._filas_csv
        return {
            "activo": self.activo, "muestras": muestras, "filas_csv": filas,
            "diagnostico": self.diagnostico(),
            "ruta_csv": self.ruta_csv, "ruta_md": self.ruta_md,
            "informe_escrito": self._t_informe,
        }

    # ------------------------------------------------------------- report
    def escribir_informe(self):
        """Rewrites `sensor_log.md` ENTIRELY with the moving window of NOW.
        It is not a history: each call erases the previous one."""
        ahora = time.strftime("%Y-%m-%d %H:%M:%S")
        with self._lock:
            muestras, filas = self._muestras, self._filas_csv
        L = ["# Sensor log", "",
             "Generated: {}  ·  samples seen: {}  ·  rows in the CSV: {}"
             .format(ahora, muestras, filas), "",
             "## Diagnosis", ""]
        for nivel, texto in self.diagnostico():
            L.append("- {} {}".format("**ALERT**" if nivel == "alerta" else "info", texto))

        L += ["", "## Wheels (moving window average)", "",
              "| wheel | requested rpm | measured rpm | mean error | PWM | take-off | magnet AGC |",
              "|---|---|---|---|---|---|---|"]
        for r in RUEDAS:
            obj = self._stats("obj_" + r) or {}
            rpm = self._stats("rpm_" + r) or {}
            pwm = self._stats("pwm_" + r) or {}
            desp = self._stats("desp_" + r) or {}
            agc = self._stats("agc_" + r) or {}
            error = abs(rpm.get("media", 0) - obj.get("media", 0))
            L.append("| {} | {:.0f} | {:.0f} | {:.0f} | {:.0f} | {:.0f} | {:.0f} |".format(
                r, obj.get("media", 0), rpm.get("media", 0), error,
                pwm.get("media", 0), desp.get("media", 0), agc.get("media", 0)))

        L += ["", "## ToF, in mm (chassis frame: N = front of the robot)", "",
              "| sensor | minimum | mean | maximum | repeated readings in a row |",
              "|---|---|---|---|---|"]
        for d in DIRS:
            st = self._stats("d" + d) or {}
            cont, _ = self._tof_repetidas[d]
            L.append("| {} | {:.0f} | {:.0f} | {:.0f} | {} |".format(
                d, st.get("min", 0), st.get("media", 0), st.get("max", 0), cont))

        yaw, ref, corr = (self._stats("yaw") or {}, self._stats("yaw_ref") or {},
                         self._stats("corr") or {})
        L += ["", "## Heading", "",
              "- yaw: {:.1f} to {:.1f}° (mean {:.1f}°)".format(
                  yaw.get("min", 0), yaw.get("max", 0), yaw.get("media", 0)),
              "- reference (yaw_ref): mean {:.1f}°".format(ref.get("media", 0)),
              "- heading correction: {:.1f} to {:.1f} rpm (mean {:.1f})".format(
                  corr.get("min", 0), corr.get("max", 0), corr.get("media", 0)),
              "", "## Session events", ""]
        if self._eventos:
            for nombre, cuenta in sorted(self._eventos.items(), key=lambda kv: -kv[1]):
                L.append("- {}: {}".format(nombre, cuenta))
        else:
            L.append("- no events yet")

        L += ["", "## Gains in force when this report was written", ""]
        with self._lock:
            for k in GANANCIAS:
                v = self._ventanas.get(k)
                if v:
                    L.append("- {} = {}".format(k, v[-1]))

        contenido = "\n".join(L) + "\n"
        with open(self.ruta_md, "w", encoding="utf-8") as f:
            f.write(contenido)
        self._t_informe = time.time()
        return self.ruta_md

    def cerrar(self):
        if self._f is None:
            return
        try:
            self.escribir_informe()
        except OSError:
            pass
        with self._lock:
            self._f.close()
            self._f = None
