"""Serial link with the ESP32: the ONLY file that knows what the CSV looks like.

WHAT IT DOES
    - Sends one character per primitive and '#param=value' lines per gain.
    - Reads 'D,...' lines and turns them into a Telemetria object.
    - Blocks waiting for the closing event of each primitive.

🔑 THE PARSER BUILDS ITSELF. At startup (and when sent 'N') the firmware
publishes a 'C,t_ms,dN,dE,...' line with the NAMES of the fields in order.
This class stores it and uses it to index the 'D,...' lines. This way,
adding a field in the firmware does NOT force touching this file: it shows
up by itself in `telem.campos` and in the interface. If for some reason the
header does not arrive (the startup was missed), CAMPOS_POR_DEFECTO is used,
which is the same list copied by hand -- and if the firmware sends more
fields than that list, a warning is raised instead of silently mis-parsing.

WATCH THE FRAME: dN/pN are in the CHASSIS FRAME ("N" = front of the robot).
Converting them to the absolute frame is the job of whoever consumes them,
with agente.lecturas_a_absoluto(). Nothing is converted here: if it were,
this file would have to know the orientation of the agent and would stop
being a translator of the cable.
"""

import queue
import threading
import time

import config

# --- commands (BLOCK 01 of the firmware) ------------------------------------
CMD_PARAR = "0"
CMD_AVANZAR = "5"
CMD_CENTRAR = "9"
CMD_GIRO_DER = "R"
CMD_GIRO_IZQ = "L"
CMD_GIRO_180 = "T"
# moves forward until it sees the wall in front, without the maze cell limit.
# 🔑 The stopping loop lives on the ESP: it reads the front ToF directly every
# 40 ms, without the filter. Closing it from here over WiFi the delay
# reached 1.3 s.
CMD_HASTA_PARED = "A"
# Align with the heading grid (Sep 15 afternoon): turns whatever is missing to
# sit on the corresponding multiple of 90. It closes like a turn (7 or 8).
CMD_ALINEAR = "Y"
# Reverse ONE cell (Sep 15 night): in a dead end there is no half turn, the
# robot backs up with the same motion and the same stopping criterion, backwards.
CMD_REVERSA = "D"
CMD_HALT = "H"
CMD_CSV_YA = "?"
# --- bench commands -----------------------------------------------------------
CMD_ESTADO = "S"
CMD_CALIBRAR_BIAS = "C"
CMD_FIJAR_RUMBO = "Z"
CMD_MEDIR_SENTIDOS = "V"
CMD_MOTORES = "E"          # toggles enabled / disabled
CMD_GUARDAR = "G"
CMD_DESPEGUES_FABRICA = "B"   # back to the take-off PWM values of the table
CMD_CABECERA = "N"

# --- events -------------------------------------------------------------------
EV_NADA = 0
EV_ACK = 1
EV_NACK = 2
EV_FIN_CELDA = 3
EV_PARED = 4
EV_CAMPO_ABIERTO = 5
EV_BLOQUEO = 6
EV_FIN_GIRO = 7
EV_ERR_GIRO = 8
EV_RUEDA_CORTADA = 9
# CMD_AVANZAR rejected because the heading is far from the multiple of 90: it
# is NOT a wall. The Pi aligns (Y) and asks again.
EV_DESALINEADO = 10

NOMBRE_EVENTO = {
    EV_NADA: "nada", EV_ACK: "ack", EV_NACK: "nack",
    EV_FIN_CELDA: "fin_celda", EV_PARED: "pared",
    EV_CAMPO_ABIERTO: "campo_abierto", EV_BLOQUEO: "bloqueo",
    EV_FIN_GIRO: "fin_giro", EV_ERR_GIRO: "err_giro",
    EV_RUEDA_CORTADA: "rueda_cortada", EV_DESALINEADO: "desalineado",
}
NOMBRE_ESTADO = {0: "idle", 1: "avanzando", 2: "girando", 3: "centrando",
                 4: "bloqueado", 5: "calibrando", 6: "manual"}

# Events that CLOSE each primitive. They are listed explicitly so that adding
# a new one in the firmware forces deciding here what it means.
EVENTOS_FIN_AVANCE = (EV_FIN_CELDA, EV_PARED, EV_NACK,
                      EV_BLOQUEO, EV_CAMPO_ABIERTO)
EVENTOS_FIN_GIRO = (EV_FIN_GIRO, EV_ERR_GIRO)

DIRS4 = ("N", "E", "S", "O")   # chassis frame: N = front of the robot
RUEDAS = ("DI", "DD", "TI", "TD")   # front-left, front-right, rear-left, rear-right

# Gains the firmware accepts live (BLOCK 21, PARAMS table). The third and
# fourth values are the limits the firmware itself applies; they are repeated
# here so the interface draws sliders with the same range.
PARAMETROS = [
    ("kp",       "Speed KP",      0.0,   5.0,  0.01),
    ("ki",       "Speed KI",      0.0,   5.0,  0.01),
    ("kd",       "Speed KD",      0.0,   5.0,  0.01),
    ("kpy",      "Heading KP",          0.0,  20.0,  0.1),
    ("kiy",      "Heading KI",          0.0,  10.0,  0.01),
    ("kdy",      "Heading KD",          0.0,  10.0,  0.05),
    ("corrmax",  "Correction cap",   0.0,  80.0,  1.0),
    ("signoyaw", "Yaw sign",    -1.0,   1.0,  2.0),
    ("crucero",  "Cruise speed", 0.03,  0.45, 0.005),
    # Start ramp. Raise the number and the car picks up speed sooner; lower it
    # if the four wheels do not grip at the same time and the start jerks.
    ("acel",     "Start ramp m/s2",0.05,  1.50, 0.01),
    # ONLY the turn on the spot. Lower it if when turning one wheel starts late
    # and the car drifts instead of turning on itself; raise it if the turn is
    # slow to start. It does not affect forward/backward.
    # The heading correction, with its own ramp and its own cap. If the car
    # jerks sideways, lower "Correction ramp"; if it does not manage to
    # straighten, raise it. "Max imbalance" is how much one side can differ
    # from the other.
    ("acelcorr", "Correction ramp",   2.0, 200.0,  1.0),
    ("fraccorr", "Max imbalance",     0.02,  0.60, 0.01),
    # MINIMUM heading authority, in rpm. At low speed the percentage above
    # falls short and the correction spends its time saturated without
    # managing to straighten; this floor is what prevents it.
    ("corrmin",  "Min heading authority", 0.0,  40.0,  1.0),
    # Degrees before the target at which the turn starts braking. Raise it if
    # the turn overshoots; lower it if it takes long to close the last stretch.
    # --- ONLY THE TURN ON THE SPOT -----------------------------------------
    ("vgiro",    "Turn: speed (deg/s)", 10.0, 180.0, 5.0),
    ("vgiroder", "Turn: right speed",    10.0, 180.0, 5.0),
    ("vgiromin", "Turn: minimum speed",     10.0,  90.0, 1.0),
    ("acelgiro", "Turn: ramp (rad/s2)",        0.2,   6.0, 0.1),
    ("gradfreno","Turn: braking degrees",     5.0,  90.0, 1.0),
    # The two that close the 90-degree turn. The tolerance has to be larger
    # than the finest step the chassis can make (~6 degrees: the fastest
    # wheel does not go below 54 rpm) and the minimum has to be above the
    # take-off of all four, or the loop asks for a speed nobody delivers.
    ("tolgiro",  "Turn tolerance",   1.0,  10.0,  0.5),
    # Heading grid (Sep 15 afternoon). "Align" is the band within which 'Y'
    # accepts the heading before moving forward; "Advance: cap" is how tilted
    # the robot may be for '5' not to reject it (misaligned).
    ("tolalin",  "Align: band",    1.0,  10.0,  0.5),
    ("tolavance","Advance: heading cap",1.0,  30.0,  0.5),
    ("parada",   "Front stop mm",  30.0, 150.0, 1.0),
    ("arrimar",  "A stop (crossing center) mm", 50.0, 200.0, 1.0),
    ("paradaS",  "Rear stop (reverse) mm", 30.0, 150.0, 1.0),
    ("segN",     "Safety front mm",   5.0, 120.0, 1.0),
    ("segE",     "Safety right mm",  5.0, 120.0, 1.0),
    ("segS",     "Safety rear mm",    5.0, 120.0, 1.0),
    ("segO",     "Safety left mm",5.0, 120.0, 1.0),
    ("fracrev",  "Reverse / cruise",     0.2, 1.0, 0.05),
    ("umbralN",  "Wall N if < mm",   100.0, 500.0, 5.0),
    ("umbralE",  "Wall E if < mm",   100.0, 500.0, 5.0),
    ("umbralS",  "Wall S if < mm",   100.0, 500.0, 5.0),
    ("umbralO",  "Wall W if < mm",   100.0, 500.0, 5.0),
    # Forward motion and centering by taps, on the ESP, with the numbers of
    # square_right.py (Sep 15 15:47). Same names as the script.
    ("centrar",  "Taps: active (0/1)",   0.0, 1.0, 1.0),
    ("vmin",     "V_MIN m/s",           0.01, 0.20, 0.005),
    ("rampaini", "T_START_RAMP s",          0.0, 5.0, 0.1),
    ("frenadesde","START_BRAKING mm",     100.0, 600.0, 10.0),
    ("latfreno", "Brake LATENCY s",       0.0, 1.0, 0.05),
    ("vytoque",  "VY_MIN_EFFECTIVE m/s",       0.01, 0.20, 0.005),
    ("bandalat", "LAT_BAND mm",           0.0, 60.0, 1.0),
    ("errpleno", "FULL_ERR mm",           1.0, 100.0, 1.0),
    ("sesgolat", "LAT_BIAS mm",          -30.0, 30.0, 0.1),
    ("minlat",   "MIN_LATERAL mm",         0.0, 100.0, 1.0),
    ("vyurg",    "VY_URGENT m/s",         0.01, 0.20, 0.005),
    ("consE",    "SETPOINT_E mm",          20.0, 120.0, 0.5),
    ("consO",    "SETPOINT_W mm",          20.0, 120.0, 0.5),
    ("limlat",   "LATERAL_LIMIT mm",      60.0, 400.0, 5.0),
    ("sintras",  "NO_TRANSLATE mm",       0.0, 400.0, 5.0),
    ("pegar",    "Stick after turn (0/1)", 0.0, 1.0, 1.0),
    ("vypegar",  "VY_STICK m/s",           0.02, 0.20, 0.005),
    ("inerlat",  "LAT_INERTIA mm",         0.0, 40.0, 1.0),
    ("bandapegar","RIGHT_BAND mm",          0.0, 30.0, 1.0),
    ("topepegar","STICK_CAP s",           0.0, 20.0, 0.5),
    # Only for the RIGHT turn, which is the one that oscillates (the left one
    # closes cleanly and uses "Turn speed" and its usual gain).
    # The caps of the manual control: how fast the car goes with the slider
    # at maximum. They were in the firmware PARAMS table from the start, but
    # not in this list, so there was no way to touch them without reflashing
    # -- exactly the opposite of what is needed to tune the control by hand.
    ("vmanual",  "Manual drive cap",0.05,  0.45, 0.005),
    ("wmanual",  "Manual turn cap",  10.0, 180.0, 5.0),
    ("celda",    "Cell step cm",  5.0,  60.0,  0.5),
    ("odom",     "Odometry factor",  0.5,   2.0,  0.01),
    # PWM threshold per wheel: how much a wheel may exceed what is asked of
    # it. Lower them if one shoots up to compensate; raise them if the control
    # falls short and does not reach the requested speed.
    ("factecho", "Wheel threshold (factor)", 1.0, 5.0, 0.1),
    ("margtecho","Wheel threshold (margin)",0.0,120.0, 5.0),
    # From the measure-directions routine (key V), not from the control.
    ("calpwm",   "Calibration: PWM",  60.0, 255.0,  5.0),
    ("calms",    "Calibration: ms",  200.0,3000.0, 50.0),
]

# Copy of CAMPOS_CSV from the firmware. Only used if the header did not arrive.
CAMPOS_POR_DEFECTO = [
    "t_ms",
    "dN", "dE", "dS", "dO",
    "pN", "pE", "pS", "pO",
    "nr0", "nr1", "nr2", "nr3",
    "mr0", "mr1", "mr2", "mr3",
    "ret", "mem",
    "roll", "pitch", "yaw", "yaw_ref", "corr", "cuad", "ecen",
    "cm_tramo", "celdas",
    "estado", "cmd", "ev", "err_giro", "imu_ok", "hab",
    "rpm_DI", "rpm_DD", "rpm_TI", "rpm_TD",
    "obj_DI", "obj_DD", "obj_TI", "obj_TD",
    "pwm_DI", "pwm_DD", "pwm_TI", "pwm_TD",
    "desp_DI", "desp_DD", "desp_TI", "desp_TD",
    "agc_DI", "agc_DD", "agc_TI", "agc_TD",
    "vivas", "cortadas", "ciclo_ms",
]


class Telemetria:
    """One already parsed CSV line. Only data, no logic.

    `campos` is the raw dictionary name -> float, as it came. The named
    attributes are those used by the state machine; everything else (rpm,
    PWM, AGC...) is read from `campos` and goes straight to the interface.
    """

    __slots__ = ("campos", "distancias", "paredes", "gauss", "ret", "mem",
                 "yaw", "yaw_ref", "corr", "cm_tramo", "celdas", "estado",
                 "cmd", "evento", "err_giro", "imu_ok", "habilitado", "t_ms")

    def __init__(self, campos):
        # ToF swap measured on the robot (config.TOF_REMAPA). It is fixed in
        # `campos`, the source, so the interface, map and RL see the same.
        # It is not a frame conversion -- it is still the chassis frame --
        # it is a wiring fix that does not justify reflashing the ESP.
        if config.TOF_REMAPA:
            for prefijo in ("d", "p"):
                orig = {esp: campos.get(prefijo + esp) for esp in config.TOF_REMAPA}
                for esp, real in config.TOF_REMAPA.items():
                    if orig[esp] is not None:
                        campos[prefijo + real] = orig[esp]
        self.campos = campos
        g = campos.get
        self.t_ms = int(g("t_ms", 0))
        self.distancias = {d: g("d" + d, 0.0) for d in DIRS4}
        self.paredes = {d: bool(g("p" + d, 0)) for d in DIRS4}
        self.gauss = [g("mr%d" % i, 0.0) for i in range(4)]
        self.ret = g("ret", 0.0)
        self.mem = g("mem", 0.0)
        self.yaw = g("yaw", 0.0)
        self.yaw_ref = g("yaw_ref", 0.0)
        self.corr = g("corr", 0.0)
        self.cm_tramo = g("cm_tramo", 0.0)
        self.celdas = int(g("celdas", 0))
        self.estado = int(g("estado", 0))
        self.cmd = int(g("cmd", 0))
        self.evento = int(g("ev", 0))
        self.err_giro = g("err_giro", 0.0)
        self.imu_ok = bool(g("imu_ok", 0))
        self.habilitado = bool(g("hab", 0))

    def rueda(self, i):
        """Dictionary with the data of one wheel, for the interface."""
        n = RUEDAS[i]
        return {
            "nombre": n,
            "rpm": self.campos.get("rpm_" + n, 0.0),
            "objetivo": self.campos.get("obj_" + n, 0.0),
            "pwm": self.campos.get("pwm_" + n, 0.0),
            "despegue": self.campos.get("desp_" + n, 0.0),
            "agc": self.campos.get("agc_" + n, 0.0),
            "viva": bool(int(self.campos.get("vivas", 0)) & (1 << i)),
            "cortada": bool(int(self.campos.get("cortadas", 0)) & (1 << i)),
        }

    def __repr__(self):
        return "Telemetria(t={} F={:.0f} ev={} yaw={:.1f})".format(
            self.t_ms, self.distancias["N"],
            NOMBRE_EVENTO.get(self.evento, self.evento), self.yaw)


class ErrorEnlace(RuntimeError):
    """The cable dropped, or the ESP stopped answering."""


class EnlaceESP:
    """Serial port + parser. Used as a context manager.

    🔑 ONLY ONE THREAD READS THE PORT. There are two consumers of the
    telemetry (the state machine, which waits for events, and the web
    interface, which ALWAYS wants the latest data, also while the robot is
    stopped). If both read from the port, they would steal lines from each
    other: the interface would eat the event the state machine is waiting
    for and the robot would hang. So a `pump` reads in its own thread and
    distributes:
        - `self.ultima`  -> the latest data, for whoever wants to look.
        - `self._eventos` -> event queue, which is what esperar_evento()
          consumes. Events are queued and not lost even if they arrive
          while nobody was looking.
        - `on_telemetria` -> optional callback, for the interface.
    Writing is also locked: an 'H' from the stop button must not sneak into
    the middle of a '#kp=0.3'.
    """

    def __init__(self, puerto, baudios, timeout_telemetria=3.0, verboso=False,
                 on_telemetria=None):
        try:
            import serial
        except ImportError:
            raise ErrorEnlace("pyserial is missing: pip3 install pyserial")
        self._serial = serial.Serial(puerto, baudios, timeout=0.2)
        self._lock = threading.RLock()
        self.puerto = puerto
        self.verboso = verboso
        self.timeout_telemetria = timeout_telemetria
        self.campos = list(CAMPOS_POR_DEFECTO)
        self.cabecera_recibida = False
        self.ultima = None
        self.lineas_malas = 0
        self.avisos = []            # READY, OK:, ERR:, CAL:... for the interface
        # STOP from the interface: the navigator checks it before sending each
        # primitive and inside the long waits, and sends nothing else.
        self.paro_pedido = False
        self.ultimo_comando = ""    # the last thing sent (for the interface)
        self.t_ultimo_comando = 0.0
        self.on_telemetria = on_telemetria
        self._eventos = queue.Queue()
        self._hay_nueva = threading.Event()
        self._vivo = True
        time.sleep(2.0)             # the ESP32 resets when the port is opened
        self._serial.reset_input_buffer()
        self._bomba = threading.Thread(target=self._bombear, daemon=True)
        self._bomba.start()
        self.enviar(CMD_CABECERA)   # in case the startup already happened

    def _bombear(self):
        """The only thread that reads from the port.

        🔴 readline() is NOT used: with `timeout=0.2`, pyserial returns
        whatever it has when the deadline expires, WITH OR WITHOUT a newline.
        Every time the ESP took more than 200 ms between the beginning and
        the end of a CSV line (2026-09-12: it happened ten times in a row),
        the Pi received half a line -- the tail fell into `avisos` as garbage
        and the head was discarded as a "split line". Here bytes are
        accumulated and only newline-terminated lines are delivered, whether
        they come in one piece or in ten.
        """
        resto = b""
        while self._vivo:
            try:
                # Everything available (or 1 blocking byte until the timeout).
                trozo = self._serial.read(max(1, self._serial.in_waiting))
            except Exception:
                break
            if not trozo:
                continue
            resto += trozo
            if len(resto) > 65536:          # 64 KB without a newline: it is not CSV
                resto = resto[-4096:]
            while b"\n" in resto:
                cruda, resto = resto.split(b"\n", 1)
                self._entregar(cruda.decode("ascii", errors="replace").strip())

    def _entregar(self, linea):
        telem = self._procesar(linea)
        if telem is None:
            return
        self._hay_nueva.set()
        if telem.evento != EV_NADA:
            self._eventos.put((telem.evento, telem))
        if self.on_telemetria:
            try:
                self.on_telemetria(telem)
            except Exception:
                pass    # the interface can never bring the link down

    # -- life cycle ---------------------------------------------------------
    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.cerrar()

    def cerrar(self):
        """Brakes the robot BEFORE releasing the port. If it is closed without
        this and the robot was moving forward, it keeps moving by itself."""
        try:
            self.enviar(CMD_HALT)
            time.sleep(0.2)
        except Exception:
            pass
        self._vivo = False
        self._bomba.join(timeout=1.0)
        self._serial.close()

    # -- output -------------------------------------------------------------
    def enviar(self, comando):
        with self._lock:
            self._serial.write(comando.encode("ascii"))
            self._serial.flush()
            # The last thing asked of the ESP, to show it in the interface
            # next to what the ESP says it is doing.
            self.ultimo_comando = comando.strip()
            self.t_ultimo_comando = time.time()
        if self.verboso:
            print("  -> {}".format(comando))

    def ajustar(self, nombre, valor):
        """Changes a gain LIVE. This is how the original control was
        calibrated: moving the numbers from the interface without uploading
        anything again."""
        self.enviar("#{}={}\n".format(nombre, valor))

    # -- input --------------------------------------------------------------
    def _procesar(self, linea):
        """Returns a Telemetria, or None if the line was not a CSV."""
        if linea.startswith("C,"):
            nombres = linea.split(",")[1:]
            if nombres:
                self.campos = nombres
                self.cabecera_recibida = True
            return None
        if not linea.startswith("D,"):
            if linea:
                self.avisos.append(linea)
                del self.avisos[:-40]
                if self.verboso:
                    print("  <- {}".format(linea))
            return None

        partes = linea.split(",")[1:]
        if len(partes) != len(self.campos):
            # Split line (startup, full buffer) or firmware and parser out of
            # sync. The header is requested: if the firmware changed, the
            # next line is already parsed correctly by itself.
            self.lineas_malas += 1
            if self.lineas_malas in (5, 50):
                self.enviar(CMD_CABECERA)
            return None
        try:
            valores = {n: float(v) for n, v in zip(self.campos, partes)}
        except ValueError:
            self.lineas_malas += 1
            return None
        self.ultima = Telemetria(valores)
        return self.ultima

    def leer_telemetria(self, timeout=None):
        """Blocks until the pump delivers a new 'D,...' line."""
        espera = timeout or self.timeout_telemetria
        self._hay_nueva.clear()      # what matters is the NEXT one, not the
                                     # one that was there when it was asked
        if not self._hay_nueva.wait(espera):
            raise ErrorEnlace(
                "no telemetry for {:.1f}s (bad lines: {})".format(
                    espera, self.lineas_malas))
        return self.ultima

    def esperar_evento(self, eventos, timeout):
        """Blocks until one of `eventos` arrives. Returns (ev, telem).

        Events that arrive and are not among the expected ones are NOT thrown
        away: they are recorded in `avisos` (a wheel cut off while waiting
        for the end of a cell is exactly what one needs to see afterwards).
        If the timeout expires the robot is braked and ErrorEnlace is raised:
        to keep sending commands to an ESP that does not answer is the sure
        way to crash it.
        """
        limite = time.time() + timeout
        while True:
            restante = limite - time.time()
            if restante <= 0:
                break
            if self.paro_pedido:
                # STOP from the interface: nothing else is awaited, the robot is
                # already braked and with the motors off (H).
                raise ErrorEnlace("stop from the interface")
            try:
                ev, telem = self._eventos.get(timeout=min(restante, 0.5))
            except queue.Empty:
                continue
            if ev in eventos:
                return ev, telem
            self.avisos.append("unexpected event: {}".format(
                NOMBRE_EVENTO.get(ev, ev)))
            del self.avisos[:-40]
        self.enviar(CMD_HALT)
        raise ErrorEnlace("none of the events {} arrived within {:.1f}s".format(
            [NOMBRE_EVENTO.get(e, e) for e in eventos], timeout))

    def esperar_ack(self, timeout=4.0):
        """ACK of the non-blocking primitives. Turns do NOT send an ACK: they
        are blocking and only emit their closing event."""
        # Sep 16: if the primitive closes in the same cycle in which it is
        # accepted (the 'A' with the wall already there), the ESP emits the
        # closing and the ACK is lost. A closing counts as ACK+closing: the
        # caller tells them apart.
        return self.esperar_evento((EV_ACK, EV_NACK, EV_DESALINEADO) + tuple(EVENTOS_FIN_AVANCE), timeout)

    def purgar(self):
        """Throws away old events. Called at the start of a run so as not to
        drag along the ACK of the previous one."""
        while True:
            try:
                self._eventos.get_nowait()
            except queue.Empty:
                break
