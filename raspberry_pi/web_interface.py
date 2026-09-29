"""The interface: web server on the Pi.

WHAT IT DOES
    Serves a page at http://<the-pi>:8080 with the live telemetry, the map
    the robot is discovering, the control gains editable without uploading
    the firmware again, and the START / FINISHED / STOP buttons.

WHERE IT COMES FROM
    From `carro_v08/estacion/`, which did the same but over WiFi against the
    ESP32. 🔑 HERE THE ESP HAS NO WiFi: that load is taken off it and carried
    by the Pi, which is already connected through the USB-C cable and is
    also the one that holds the map and the RL. The ESP only publishes its
    CSV over serial.

    What is kept from that interface, because it proved useful while
    calibrating: the live gains (that is how the whole control was
    calibrated), the state of the four wheels (requested vs measured rpm,
    PWM, learned take-off PWM and magnet health) and the plots of the last
    twenty seconds.

    What is added: the maze map, and the start buttons. In the old
    motors-only test the robot started by pressing EN on the ESP -- that is
    no longer needed.

WHY THE STANDARD LIBRARY AND NOT FLASK
    So as not to add one more dependency to install on the Pi before the
    exam. A threaded `http.server` is more than enough for a page that polls
    five times per second.

SECURITY: it listens on all interfaces so it can be opened from the laptop
    over the lab network. There is no password: it is a lab network and the
    robot is in plain sight. Do not expose it to the internet.
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial_link as en

_AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ_WEB = os.path.join(_AQUI, "web")

TIPOS = {".html": "text/html; charset=utf-8",
         ".js": "application/javascript; charset=utf-8",
         ".css": "text/css; charset=utf-8"}


class Tablero:
    """The state shared by the navigation loop and the web server.

    A single lock for everything. Both sides write little and read a lot,
    and the structures are small: splitting it into several locks would only
    add ways to deadlock.
    """

    def __init__(self, lado, meta):
        self._lock = threading.Lock()
        self.lado = lado
        self.meta = tuple(meta)
        # --- what the navigation loop publishes ---
        self.telem = {}              # latest telemetry, as a dictionary
        self.ruedas = []
        self.avisos = []
        self.corrida = {"n": 0, "paso": 0, "celda": [0, 0], "orientacion": 0,
                        "heading": "N", "evento": "", "motivo": "",
                        "figura": "", "en_meta": False, "modo": "",
                        "temperatura": 0.0, "corridas": 0, "exitos": 0,
                        "recompensa": None}
        self.mapa = {}               # "x,y" -> {"N":1,"E":0,...}
        self.traza = []              # cells travelled in this run
        # Neural snapshot of the LAST decision (navigator.on_decision) and
        # the decision itself. The page uses it to draw the Pi side of the
        # network (MLP, votes, softmax); if it is None, that part stays off.
        self.neuro = None
        self.decision = {}
        self.celda_cm = None         # cell size for the map
        self.estado_pi = "esperando" # esperando | corriendo | terminado | error
        self.mensaje = "Press START to begin"
        # Port of the camera preview, or None if it is off. The page uses it
        # only to show the link: the video does NOT go through here, it has
        # its own server (see camera_master.py).
        self.camara_puerto = None
        # --- what the interface requests ---
        self._ordenes = []           # queue of commands from the web browser
        self.correr = False          # the START/FINISHED button

    # -- navigation side ----------------------------------------------------
    def publicar_telemetria(self, telem):
        with self._lock:
            self.telem = dict(telem.campos)
            self.ruedas = [telem.rueda(i) for i in range(4)]

    def publicar_corrida(self, **kw):
        with self._lock:
            self.corrida.update(kw)

    def publicar_mapa(self, mapa_obj):
        with self._lock:
            self.mapa = {"{},{}".format(*c): {d: (1 if p[d] else 0)
                                              for d in ("N", "E", "S", "O")}
                         for c, p in mapa_obj.paredes.items()}
            self.traza = [list(c) for c in mapa_obj.historial_reciente]

    def publicar_neuro(self, direccion, motivo, neuro):
        with self._lock:
            self.neuro = neuro
            self.decision = {"direccion": direccion, "motivo": motivo or ""}

    def publicar_estado(self, estado, mensaje=None):
        with self._lock:
            self.estado_pi = estado
            if mensaje is not None:
                self.mensaje = mensaje

    def tomar_ordenes(self):
        """Returns and empties the queue. The navigation loop calls it between
        steps: this way a command never lands in the middle of a turn."""
        with self._lock:
            ordenes, self._ordenes = self._ordenes, []
        return ordenes

    def quiere_correr(self):
        with self._lock:
            return self.correr

    # -- web side -----------------------------------------------------------
    def encolar(self, orden):
        with self._lock:
            self._ordenes.append(orden)

    def poner_correr(self, valor):
        with self._lock:
            self.correr = bool(valor)

    def instantanea(self):
        with self._lock:
            return {
                "telem": self.telem,
                "ruedas": self.ruedas,
                "corrida": dict(self.corrida),
                "mapa": self.mapa,
                "traza": self.traza,
                "estado_pi": self.estado_pi,
                "mensaje": self.mensaje,
                "avisos": self.avisos[-12:],
                "lado": self.lado,
                "meta": list(self.meta),
                "correr": self.correr,
                "camara_puerto": self.camara_puerto,
                "neuro": self.neuro,
                "decision": dict(self.decision),
                "celda_cm": self.celda_cm,
                "t": time.time(),
            }


def crear_servidor(tablero, link, params_actuales, registro=None, puerto=8080):
    """Returns an already assembled ThreadingHTTPServer (not started).

    `link` is used ONLY for the things that do not go through the state
    machine: the emergency stop, the bench commands and the gains.
    Everything that affects navigation (start, finished) goes through the
    board command queue, so that it is handled between steps.

    `registro` is the `data_logger.Registro` that records the sensor data
    (see data_logger.py). It may be None (logging disabled, or while the
    interface is being tested alone): then the diagnostic panel of the page
    stays empty and /informe.md answers 404, without anything else failing.
    """

    class Manejador(BaseHTTPRequestHandler):
        # The default log prints one line per request: with the page polling
        # 5 times per second, it floods the robot console.
        def log_message(self, *_):
            pass

        def _responder(self, codigo, cuerpo, tipo="application/json"):
            datos = cuerpo if isinstance(cuerpo, bytes) else cuerpo.encode("utf-8")
            self.send_response(codigo)
            self.send_header("Content-Type", tipo)
            self.send_header("Content-Length", str(len(datos)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(datos)

        def do_GET(self):
            ruta = self.path.split("?")[0]
            if ruta == "/estado":
                inst = tablero.instantanea()
                inst["avisos"] = link.avisos[-12:]
                inst["ultimo_cmd"] = link.ultimo_comando
                inst["hace_cmd"] = round(time.time() - link.t_ultimo_comando, 1)
                t = link.ultima
                inst["estado_esp"] = (en.NOMBRE_ESTADO.get(t.estado, t.estado)
                                      if t else "--")
                inst["params"] = params_actuales
                inst["registro"] = registro.instantanea() if registro else None
                self._responder(200, json.dumps(inst))
                return
            if ruta == "/parametros":
                self._responder(200, json.dumps({
                    "definicion": en.PARAMETROS,
                    "valores": params_actuales,
                }))
                return
            if ruta == "/informe.md":
                # The calibration document, served as plain text: it opens in
                # a new tab and is read or saved from there, without setting
                # up any Markdown viewer.
                if registro is None or not os.path.exists(registro.ruta_md):
                    self._responder(404, "there is no report yet: press "
                                    "'Generate report' or wait for the first "
                                    "automatic write.", "text/plain; charset=utf-8")
                    return
                with open(registro.ruta_md, "rb") as f:
                    self._responder(200, f.read(), "text/plain; charset=utf-8")
                return
            if ruta == "/":
                ruta = "/index.html"
            destino = os.path.normpath(os.path.join(RAIZ_WEB, ruta.lstrip("/")))
            if not destino.startswith(RAIZ_WEB) or not os.path.isfile(destino):
                self._responder(404, "not found", "text/plain")
                return
            ext = os.path.splitext(destino)[1]
            with open(destino, "rb") as f:
                self._responder(200, f.read(), TIPOS.get(ext, "text/plain"))

        def do_POST(self):
            largo = int(self.headers.get("Content-Length", 0))
            try:
                cuerpo = json.loads(self.rfile.read(largo) or b"{}")
            except ValueError:
                self._responder(400, json.dumps({"error": "unreadable json"}))
                return
            ruta = self.path.split("?")[0]

            if ruta == "/orden":
                accion = cuerpo.get("accion", "")
                # The STOP does not wait its turn: it goes to the cable
                # immediately. It is the only thing in the interface that
                # skips the command queue, on purpose -- a stop button that
                # waits for the current turn to finish is not a stop button.
                if accion in ("paro", "final"):
                    # 🔴 FORCED (Sep 15 night): 'H' brakes AND turns off the
                    # motors on the ESP (idempotent: it can be repeated without
                    # toggling anything), it is handled even inside a turn, and
                    # the Pi immediately stops waiting for events and sending
                    # primitives. START turns the motors on again.
                    # "final" = "I reached the end of the maze": it also closes
                    # the run as a SUCCESS (exit declared by hand).
                    link.paro_pedido = True
                    link.enviar(en.CMD_HALT)
                    link.enviar(en.CMD_HALT)          # in case the first one landed in a turn
                    tablero.poner_correr(False)
                    if accion == "final":
                        tablero.encolar({"accion": "final"})
                        tablero.publicar_estado("terminado", "END declared by hand: "
                                                "robot braked, motors off")
                    else:
                        tablero.publicar_estado("esperando", "STOP from the interface: "
                                                "motors off (START turns them on)")
                elif accion == "start":
                    tablero.poner_correr(True)
                    tablero.encolar({"accion": "start"})
                elif accion == "finished":
                    tablero.poner_correr(False)
                    tablero.encolar({"accion": "finished"})
                elif accion == "limpiar":
                    # Erase runs and map. Only with the robot stopped: it is
                    # handled by main.esperar_start(), which is the one that
                    # has the agent and the map at hand.
                    if tablero.quiere_correr():
                        self._responder(409, json.dumps(
                            {"error": "a run is in progress: FINISHED first"}))
                        return
                    tablero.encolar({"accion": "limpiar", "todo": bool(cuerpo.get("todo"))})
                elif accion == "manual_pwm":
                    # Manual control: '#m=vx,vy,w' goes straight to the ESP,
                    # which passes it through the SAME speed and heading loop
                    # as navigation (each field is -255..255, a fraction of the
                    # control cap). The page repeats it every 250 ms while the
                    # button is held; the ESP brakes by itself if 600 ms pass
                    # without a line. Only with the robot stopped.
                    if tablero.quiere_correr():
                        self._responder(409, json.dumps(
                            {"error": "a run is in progress: FINISHED first"}))
                        return
                    try:
                        vx = int(cuerpo.get("vx", 0)); vy = int(cuerpo.get("vy", 0))
                        w = int(cuerpo.get("w", 0))
                    except (TypeError, ValueError):
                        self._responder(400, json.dumps({"error": "vx/vy/w not numeric"}))
                        return
                    tope = lambda v: max(-255, min(255, v))
                    # A PUSH is sustained by the ESP on its own until the time
                    # is up: the message does not need repeating and the
                    # network can no longer cut the motion halfway. Without
                    # "ms" the classic pulsed control is sent, which is still
                    # valid for everything else (and for which the ESP brakes
                    # after 600 ms).
                    try:
                        ms = int(cuerpo.get("ms", 0))
                    except (TypeError, ValueError):
                        ms = 0
                    if ms > 0:
                        link.enviar("#t={},{},{},{}\n".format(
                            tope(vx), tope(vy), tope(w), max(0, min(10000, ms))))
                    else:
                        link.enviar("#m={},{},{}\n".format(tope(vx), tope(vy), tope(w)))
                elif accion == "rearmar":
                    # The wheels cut off by the controller (cortada=1) are
                    # re-armed by turning the motors off and on: 'E' toggles,
                    # and when turning on the ESP gives them "another chance".
                    if link.ultima and link.ultima.habilitado:
                        link.enviar(en.CMD_MOTORES)
                        time.sleep(0.15)
                    link.enviar(en.CMD_MOTORES)
                elif accion == "manual":
                    # Move by hand from the page, to check wiring and
                    # directions. It goes straight to the cable, like the STOP,
                    # but ONLY with the robot stopped: in the middle of a run
                    # the navigator is waiting for the event of ITS command and
                    # a foreign one desynchronizes it.
                    letra = str(cuerpo.get("letra", ""))[:1]
                    if tablero.quiere_correr():
                        self._responder(409, json.dumps(
                            {"error": "a run is in progress: FINISHED first"}))
                        return
                    if letra in (en.CMD_AVANZAR, en.CMD_GIRO_IZQ, en.CMD_GIRO_DER,
                                 en.CMD_GIRO_180, en.CMD_PARAR,
                                 en.CMD_HASTA_PARED, en.CMD_ALINEAR,
                                 en.CMD_REVERSA):
                        link.enviar(letra)
                    else:
                        self._responder(400, json.dumps({"error": "letter not allowed"}))
                        return
                elif accion == "informe":
                    # Forces the report to be rewritten NOW, instead of waiting
                    # for the next multiple of informe_cada rows. Useful right
                    # before closing a session or changing a gain.
                    if registro is None or not registro.activo:
                        self._responder(400, json.dumps(
                            {"error": "the logger is off"}))
                        return
                    ruta = registro.escribir_informe()
                    self._responder(200, json.dumps({"ok": True, "ruta": ruta}))
                    return
                elif accion == "banco":
                    # One-character commands: motors, calibrate bias,
                    # measure directions, save, set heading.
                    letra = str(cuerpo.get("letra", ""))[:1]
                    if letra in (en.CMD_MOTORES, en.CMD_CALIBRAR_BIAS,
                                 en.CMD_FIJAR_RUMBO, en.CMD_MEDIR_SENTIDOS,
                                 en.CMD_GUARDAR, en.CMD_DESPEGUES_FABRICA,
                                 en.CMD_ESTADO, en.CMD_CABECERA):
                        link.enviar(letra)
                    else:
                        self._responder(400, json.dumps({"error": "letter not allowed"}))
                        return
                else:
                    self._responder(400, json.dumps({"error": "unknown action"}))
                    return
                self._responder(200, json.dumps({"ok": True}))
                return

            if ruta == "/parametro":
                nombre = cuerpo.get("nombre", "")
                try:
                    valor = float(cuerpo.get("valor"))
                except (TypeError, ValueError):
                    self._responder(400, json.dumps({"error": "value not numeric"}))
                    return
                if nombre not in [p[0] for p in en.PARAMETROS]:
                    self._responder(400, json.dumps({"error": "unknown parameter"}))
                    return
                link.ajustar(nombre, valor)
                params_actuales[nombre] = valor
                self._responder(200, json.dumps({"ok": True, nombre: valor}))
                return

            self._responder(404, json.dumps({"error": "not found"}))

    servidor = ThreadingHTTPServer(("0.0.0.0", puerto), Manejador)
    servidor.daemon_threads = True
    return servidor


def arrancar_en_hilo(tablero, link, params_actuales, registro=None, puerto=8080):
    servidor = crear_servidor(tablero, link, params_actuales, registro, puerto)
    hilo = threading.Thread(target=servidor.serve_forever, daemon=True)
    hilo.start()
    return servidor
