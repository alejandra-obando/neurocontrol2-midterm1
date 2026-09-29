#!/usr/bin/env python3
"""The robot camera, ready for main, with a live preview.

    python3 camera_master.py            standalone, to calibrate in the maze
    python3 camera_master.py --port 8081

    and from the PC:  http://<pi-ip>:8081

WHAT THIS IS AND WHY IT EXISTS
    `test_camera.py` showed that classical vision is enough. `perception.py`
    has the slot where the navigator expects it. This file is the bridge:
    the SAME detection as test_camera, wrapped with the three things it was
    missing to live inside the robot.

    1. A SINGLE CAMERA READER. /dev/video0 is not shared: if the navigation
       loop and the preview server each open the camera on their own, the
       second one gets "Device is busy". Here a dedicated thread is the only
       one that touches the camera; everybody else reads the last frame that
       thread left.

    2. A BURST INSTEAD OF A SINGLE FRAME. 🔴 THIS WAS A REAL BUG.
       `LectorEstable` requires seeing the same figure in FRAMES_ESTABLES
       CONSECUTIVE frames, but `navigator.observar()` calls `camara.leer()`
       ONCE per cell. With the perception.py camera that means the three
       consecutive frames are three consecutive CELLS: the robot would have
       to see the same triangle at three different stops to obey it, and by
       then it has already gone past. Here `leer()` consumes a burst of
       consecutive frames from the reader thread -- the robot is stopped,
       taking them costs ~120 ms -- and requires the figure to repeat
       WITHIN that burst. The stability rule is met, but at the moment of
       the decision and not spread across the maze.

    3. SEEING WHAT THE ROBOT SEES, WHILE IT RUNS. The page of this file
       serves the annotated video and the raw detection numbers. It is not
       a luxury: it is the only way to know why it obeyed or why it did not.

THE THREE GATES OF A DECISION (and where each one lives)
    DISTANCE    the front ToF below dist_decision. Set by the navigator,
                which is the one that has the telemetry, and it arrives here
                as an argument of leer(). A figure seen from afar belongs to
                another cell.
    AREA        the figure fills at least area_decision of the frame. A large
                figure painted far away fools the area alone, that is why
                both are needed.
    CONFIDENCE  the geometry of the contour is clean enough. 🔴 THIS IS THE
                ONE TO CALIBRATE IN THE MAZE, see CONFIANZA_MINIMA.

    🔑 AND HERE IS THE DIFFERENCE WITH test_camera.py: there the detector is
    built with area_minima=AREA_MINIMA and therefore DISCARDS everything far
    away, so one never sees how the area grows when approaching. Here the
    detection uses a WIDE threshold (AREA_MIRAR) and the decision a NARROW
    one. The page shows both figures and whether each gate passes; without
    that, calibrating is guessing.

HOW IT PLUGS INTO main.py -- FOUR CHANGES, NONE OF THEM TOUCHES THE LOGIC
    It replaces `perception.Camara` without the navigator noticing: it
    exposes `disponible()`, `leer(dist_frontal_mm)` and `cerrar()`, the three
    methods navigator.py uses, with the same signature and the same contract
    (it returns a name from perception.FIGURAS, or None).

    (1) next to the other imports of main.py:
            import camera_master as camara_master    # noqa: E402

    (2) where it used to say `camara = percepcion.Camara(activa=args.camara)`:
            camara = camara_master.CamaraMaster(
                activa=args.camara, indice=args.indice_camara)
            if args.camara and not args.sin_interfaz:
                camara_master.arrancar_en_hilo(camara, args.puerto_camara)
                print("[init] camera at http://localhost:{}".format(
                    args.puerto_camara))

    (3) in main(), two new arguments:
            p.add_argument("--camera-index", type=int, default=0)
            p.add_argument("--camera-port", type=int,
                           default=camara_master.PUERTO_PREVIA)

    (4) 🔴 at the end of correr(), where the log is closed:
            camara.cerrar()
        main.py used NOT to close the camera. With perception.Camara it did
        not matter because the process ended, but here there is a live reader
        thread: if it is not closed, the next start finds /dev/video0 taken
        -- exactly the error that cost half an afternoon. It goes in a
        `finally`, because a ctrl-c also has to release the camera.

    perception.py is NOT touched. It is still the skeleton that documents
    the contract, and with --camera off the robot behaves as before.
"""

import argparse
import csv
import json
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import perception as percepcion

# test_camera imports cv2 when loaded, and main.py ALWAYS imports this file
# (even with the camera off). Without this guard, a Pi without OpenCV
# installed could not even start the robot without a camera -- which is
# exactly the mode in which everything has been tested so far. With OpenCV
# present the constants of test_camera are used, which is the source;
# without it, the same numbers by hand and the camera simply does not turn on.
try:
    from test_camera import AREA_MINIMA, FRAMES_ESTABLES
    HAY_OPENCV = True
except ImportError:                                  # pragma: no OpenCV
    AREA_MINIMA, FRAMES_ESTABLES = 0.045, 3
    HAY_OPENCV = False

_AQUI = os.path.dirname(os.path.abspath(__file__))

# Its own port, different from the 8080 of web_interface.py. Separate on
# purpose: MJPEG leaves a connection open forever, and mixing it with the
# status queries of the interface is asking for one to block the other.
PUERTO_PREVIA = 8081

# WIDE detection threshold: with this one the robot LOOKS, it does not
# decide. Low on purpose, to be able to see on the page the figure at the end
# of the corridor and how its area grows as the robot approaches. That is the
# data used to choose AREA_DECISION.
AREA_MIRAR = 0.004

# NARROW threshold: with this one the robot DECIDES. It starts at the test_camera value.
AREA_DECISION = AREA_MINIMA

# 🔴 THE NUMBERS TO CALIBRATE IN THE MAZE, one per figure.
# At 0.0 they filter nothing: the robot obeys any detection that passes the
# area and distance gates, which is how test_camera behaves today.
#
# They are per figure and not a single one because confidence does NOT mean
# the same thing for each, and comparing them against the same number would
# be a mistake:
#   triangle   how clear it is where the vertex points. It is the only one
#              that can really stay low, and the only one that really
#              matters: a > mistaken for a < sends the robot the other way.
#   square     the solidity of the contour. The detector already discards
#              below 0.85, so here it always arrives between 0.85 and 1.0.
#   circle     the circularity. The detector already requires > 0.78.
#
# HOW TO GET THEM, with this file running standalone in the maze:
#   1. robot against the wall, figure in front. Record samples with the page
#      button, moving the robot to the poses that really happen.
#   2. same walk but WITHOUT a figure on the wall, and with the figure of the
#      next cell in view: these are the false positives, the ones that must
#      be left out.
#   3. open camera_calibration.csv and set the threshold between the lowest
#      confidence of (1) and the highest of (2). If those two ranges
#      overlap, no threshold separates them and the lighting or the size of
#      the figure must be fixed, not the number.
CONFIANZA_MINIMA = {
    percepcion.FIGURA_TRIANGULO_IZQ: 0.0,
    percepcion.FIGURA_TRIANGULO_DER: 0.0,
    percepcion.FIGURA_CIRCULO: 0.0,
    percepcion.FIGURA_CUADRADO: 0.0,
}

# Consecutive frames that must match within the burst.
FRAMES_RAFAGA = FRAMES_ESTABLES
# Minimum confidence for a figure seen from afar to trigger the ALARM
# (vista()). Sep 15 night: raised from ~1% (0.0 = everything passes) to 5%,
# empirical test.
CONFIANZA_ALARMA = 0.0     # Sep 16: detecting IS acting (the real confidence is ~1%)

# Maximum wait for the reader thread to deliver the burst. At 25 frames per
# second, three frames are 120 ms; half a second is plenty of margin and
# prevents a hung camera from freezing the run.
ESPERA_RAFAGA_S = 0.6

RUTA_CALIBRACION = os.path.join(_AQUI, "data", "camera_calibration.csv")
CABECERA_CALIBRACION = ["t", "figura", "confianza", "area_frac",
                        "dist_frontal_mm", "decidiria", "nota"]


class CamaraMaster:
    """The robot camera. One thread reads; everybody else looks.

    With `activa=False` it opens nothing and `leer()` always returns None,
    like perception.Camara: the robot runs without a camera and without
    OpenCV installed.
    """

    def __init__(self, activa=False, indice=0,
                 dist_decision=percepcion.DIST_DECISION_MM,
                 area_mirar=AREA_MIRAR, area_decision=AREA_DECISION,
                 confianza_minima=None, ancho=320, alto=240):
        # 320x240 since Sep 16 (afternoon): 4 times fewer pixels per frame,
        # so detection runs on EVERY frame (25-30 fps) with less CPU than
        # before at 640x480 on every other frame. The gates are fractional
        # (area_frac), they do not change with the resolution.
        self.activa = activa
        self.indice = indice
        self.dist_decision = dist_decision
        self.area_mirar = area_mirar
        self.area_decision = area_decision
        self.confianza_minima = dict(CONFIANZA_MINIMA)
        if isinstance(confianza_minima, dict):
            self.confianza_minima.update(confianza_minima)
        elif confianza_minima is not None:      # a single number for all
            self.confianza_minima = {f: float(confianza_minima)
                                     for f in percepcion.FIGURAS}

        self.ultima_figura = None      # the last one OBEYED
        self.evaluada = None           # last result of evaluar()
        # 🔴 SIGHTING ALARM (Sep 15 afternoon). On the floor the figure is
        # detected with ~1% confidence and from afar: with the three gates it
        # never got to decide. Now the mere fact of DETECTING it stably (a
        # whole burst with the same figure) triggers this alarm; the
        # navigator uses it to approach the figure node and obey it there.
        # No distance, area or confidence gate.
        self.avistada = None           # the figure sighted in the last query
        self.lecturas = 0
        self.ultima_distancia = None   # the front ToF of the last query

        self._cam = None
        self._detector = None
        self._cv2 = None
        self._jpeg = None
        self._jpeg_n = 0               # encoded frames (the MJPEG only sends new ones)
        self._muestras = deque(maxlen=64)   # (sequence, detection)
        self._secuencia = 0
        self._lock = threading.Lock()
        self._vivo = False
        self._hilo = None
        self._stats = {}               # figure -> {n, conf_min, conf_max, ...}

        if activa:
            if not HAY_OPENCV:
                raise RuntimeError(
                    "--camera was requested but there is no OpenCV. On the Pi: "
                    "`sudo apt install python3-opencv`, and if `import cv2` "
                    "points to .local/lib it is the pip one shadowing it -- that "
                    "build has no V4L and the webcam does not open.")
            # Imported here, not at the top: without a mounted camera it makes
            # no sense to require OpenCV to run the rest of the robot. Same
            # criterion as perception.Camara.
            import cv2
            from test_camera import CamaraReal, DetectorFiguras
            self._cv2 = cv2
            # 🔴 Sep 16 afternoon: after a voltage drop the webcam re-enumerates
            # (video0 -> video1) and the startup died with "could not open
            # camera 0". The requested index is tried and, if not, 0..3 in order.
            self._cam = None
            ultimo_error = None
            for idx in [indice] + [i for i in range(4) if i != indice]:
                if not os.path.exists("/dev/video{}".format(idx)) and os.name != "nt":
                    continue
                try:
                    self._cam = CamaraReal(idx, ancho, alto)
                    if idx != indice:
                        print("[camera] /dev/video{} did not open: using /dev/video{}".format(indice, idx))
                    break
                except RuntimeError as e:
                    ultimo_error = e
            if self._cam is None:
                raise ultimo_error or RuntimeError("there is no camera /dev/video0..3")
            self._detector = DetectorFiguras(area_minima=area_mirar)
            self._vivo = True
            self._hilo = threading.Thread(target=self._bucle, daemon=True)
            self._hilo.start()

    # -- the only one that touches the camera ------------------------------
    def _bucle(self):
        cv2 = self._cv2
        from test_camera import dibujar
        # 🔴 CPU (Sep 16): at 640x480 detecting on every frame kept the Pi's
        # python at 121%. Now at 320x240 detection runs on EVERY frame (the
        # detection follows the motion and the 3-frame bursts come out in
        # ~100 ms) and the preview is encoded at ~12 fps with quality 55.
        t_jpeg = 0.0
        while self._vivo:
            cuadro = self._cam.leer()
            if cuadro is None:
                time.sleep(0.005)
                continue
            d = self._detector.detectar(cuadro)
            with self._lock:
                self._secuencia += 1
                self._muestras.append((self._secuencia, d))
                if d is not None:
                    s = self._stats.setdefault(
                        d.figura, {"n": 0, "conf_min": 1.0, "conf_max": 0.0,
                                   "area_min": 1.0, "area_max": 0.0})
                    s["n"] += 1
                    s["conf_min"] = min(s["conf_min"], d.confianza)
                    s["conf_max"] = max(s["conf_max"], d.confianza)
                    s["area_min"] = min(s["area_min"], d.area_frac)
                    s["area_max"] = max(s["area_max"], d.area_frac)
                dist = self.ultima_distancia
            # Drawing happens outside the lock: encoding a JPEG takes
            # milliseconds and there is no reason to make the navigator wait
            # with the lock taken meanwhile.
            if time.time() - t_jpeg < 0.08:
                continue
            t_jpeg = time.time()
            lienzo = dibujar(cuadro, d, self._orden_de(d, dist))
            self._anotar_puertas(lienzo, d, dist)
            ok, buf = cv2.imencode(".jpg", lienzo,
                                   [cv2.IMWRITE_JPEG_QUALITY, 55])
            if ok:
                with self._lock:
                    self._jpeg = buf.tobytes()
                    self._jpeg_n += 1

    def _anotar_puertas(self, lienzo, d, dist):
        """On top of the frame, the three gates and whether they pass. It is
        what makes the page useful: seeing the figure is not enough, one has
        to see why it was obeyed or why not."""
        cv2 = self._cv2
        verde, rojo, gris = (80, 255, 140), (80, 80, 255), (170, 170, 170)
        estado = {"y": lienzo.shape[0] - 74}

        def linea(txt, ok):
            color = gris if ok is None else (verde if ok else rojo)
            cv2.putText(lienzo, txt, (10, estado["y"]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            estado["y"] += 18

        if dist is None:
            linea("ToF: no data (standalone camera)", None)
        else:
            linea("ToF {:.0f} < {:.0f} mm".format(dist, self.dist_decision),
                  dist <= self.dist_decision)
        if d is None:
            linea("area: no figure", None)
            linea("conf: no figure", None)
            return
        linea("area {:.3f} >= {:.3f}".format(d.area_frac, self.area_decision),
              d.area_frac >= self.area_decision)
        umbral = self.confianza_minima.get(d.figura, 0.0)
        linea("conf {:.2f} >= {:.2f}".format(d.confianza, umbral),
              d.confianza >= umbral)

    def _orden_de(self, d, dist):
        """Only for the drawing: what would be seen as an order if this were
        the moment to decide. It decides nothing; deciding is leer()'s job."""
        return d.figura if self._pasa_puertas(d, dist) else None

    def _pasa_puertas(self, d, dist):
        if d is None:
            return False
        if dist is not None and dist > self.dist_decision:
            return False
        if d.area_frac < self.area_decision:
            return False
        return d.confianza >= self.confianza_minima.get(d.figura, 0.0)

    # -- what the navigator calls ------------------------------------------
    def disponible(self):
        return self.activa

    def leer(self, dist_frontal_mm=None):
        """A figure from perception.FIGURAS, or None. Same signature as
        perception.Camara.leer(), so the navigator does not notice the change.

        It consumes a BURST of consecutive frames and requires the figure to
        repeat in all of them. See point 2 of the header: with a single
        frame, the stability was spread across different cells and was
        useless.
        """
        self.lecturas += 1
        with self._lock:
            self.ultima_distancia = dist_frontal_mm
        if not self.activa:
            return None
        # The distance gate first: if the wall is not right there, the burst
        # is not even awaited. Saves ~120 ms in every open corridor cell.
        if dist_frontal_mm is not None and dist_frontal_mm > self.dist_decision:
            return None

        rafaga = self._tomar_rafaga(FRAMES_RAFAGA, ESPERA_RAFAGA_S)
        if len(rafaga) < FRAMES_RAFAGA:
            return None                # slow or hung camera: nothing is made up
        if any(d is None for d in rafaga):
            return None                # flicker: not stable
        if len({d.figura for d in rafaga}) != 1:
            return None                # two different figures in the burst
        if not all(self._pasa_puertas(d, dist_frontal_mm) for d in rafaga):
            return None
        self.ultima_figura = rafaga[0].figura
        return self.ultima_figura

    def vista(self):
        """The figure seen NOW, with the lowest gate there is: a whole burst
        with the same figure, without looking at distance, area or confidence
        (the detector already requires area >= AREA_MIRAR to see it). Leaves
        the result in `avistada` for the page. None if there is none."""
        if not self.activa:
            self.avistada = None
            return None
        rafaga = self._tomar_rafaga(FRAMES_RAFAGA, ESPERA_RAFAGA_S)
        fig = None
        if (len(rafaga) == FRAMES_RAFAGA and all(d is not None for d in rafaga)
                and len({d.figura for d in rafaga}) == 1
                and all(d.confianza >= CONFIANZA_ALARMA for d in rafaga)):
            fig = rafaga[0].figura
        with self._lock:
            self.avistada = fig
        return fig

    def evaluar(self, duracion_s=1.0):
        """🔴 EVALUATION WITH THE ROBOT STOPPED (Sep 16): gathers all the
        frames of `duracion_s` and decides by MAJORITY which figure it is,
        where it is in the frame (left / center / right, by the centroid of
        the contour) and how large it looks. It is what is STORED to execute
        later, when the robot reaches the node, even if up close it no longer
        frames it.
        Returns a dict {figura, votos, cuadros, lado, x, area, confianza} or
        None if there was no clear majority (more than half of the frames)."""
        if not self.activa:
            return None
        with self._lock:
            desde = self._secuencia
        time.sleep(duracion_s)
        with self._lock:
            nuevos = [d for s, d in self._muestras if s > desde]
        if not nuevos:
            return None
        votos = {}
        for d in nuevos:
            if d is not None:
                votos[d.figura] = votos.get(d.figura, 0) + 1
        if not votos:
            return None
        figura, n = max(votos.items(), key=lambda kv: kv[1])
        if n * 2 <= len(nuevos):
            return None                     # no majority: nothing is stored
        xs, areas, confs = [], [], []
        for d in nuevos:
            if d is None or d.figura != figura:
                continue
            areas.append(d.area_frac); confs.append(d.confianza)
            try:
                m = self._cv2.moments(d.contorno)
                if m["m00"] > 0:
                    xs.append(m["m10"] / m["m00"] / float(self.ancho))
            except Exception:
                pass
        x = sum(xs) / len(xs) if xs else 0.5
        lado = "left" if x < 0.4 else "right" if x > 0.6 else "center"
        res = {"figura": figura, "votos": n, "cuadros": len(nuevos), "lado": lado,
               "x": round(x, 2), "area": round(sum(areas) / len(areas), 4),
               "confianza": round(sum(confs) / len(confs), 3)}
        with self._lock:
            self.evaluada = res
        return res

    def _tomar_rafaga(self, cuantos, espera_s):
        """The `cuantos` frames the reader thread produces FROM NOW ON. Not
        the last ones in the buffer: those may be from before the robot
        braked."""
        with self._lock:
            desde = self._secuencia
        limite = time.time() + espera_s
        nuevos = []
        while time.time() < limite:
            with self._lock:
                nuevos = [d for s, d in self._muestras if s > desde]
            if len(nuevos) >= cuantos:
                return nuevos[:cuantos]
            time.sleep(0.01)
        return nuevos

    def cerrar(self):
        self._vivo = False
        if self._hilo:
            self._hilo.join(timeout=1.0)
            self._hilo = None
        if self._cam:
            self._cam.cerrar()
            self._cam = None

    # -- what the page queries -----------------------------------------------
    def jpeg(self):
        with self._lock:
            return self._jpeg

    def datos(self):
        with self._lock:
            muestra = self._muestras[-1][1] if self._muestras else None
            dist = self.ultima_distancia
            stats = {f: dict(s) for f, s in self._stats.items()}
        d = {"activa": self.activa, "lecturas": self.lecturas,
             "ultima_figura": self.ultima_figura,
             "dist_frontal": dist, "dist_decision": self.dist_decision,
             "area_decision": self.area_decision,
             "confianza_minima": self.confianza_minima,
             "stats": stats, "figura": None,
             "avistada": self.avistada, "alarma": self.avistada is not None}
        if muestra is not None:
            umbral = self.confianza_minima.get(muestra.figura, 0.0)
            d.update(figura=muestra.figura,
                     confianza=round(muestra.confianza, 3),
                     area_frac=round(muestra.area_frac, 4),
                     vertices=len(muestra.vertices),
                     puerta_area=muestra.area_frac >= self.area_decision,
                     puerta_conf=muestra.confianza >= umbral,
                     puerta_dist=(dist is None or dist <= self.dist_decision),
                     decidiria=self._pasa_puertas(muestra, dist))
        return d

    def guardar_muestra(self, nota=""):
        """One row of the calibration CSV. It is the page button."""
        d = self.datos()
        nueva = not os.path.exists(RUTA_CALIBRACION)
        with open(RUTA_CALIBRACION, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if nueva:
                w.writerow(CABECERA_CALIBRACION)
            w.writerow([round(time.time(), 2), d.get("figura") or "",
                        d.get("confianza", ""), d.get("area_frac", ""),
                        "" if d.get("dist_frontal") is None else d["dist_frontal"],
                        1 if d.get("decidiria") else 0, nota])
        return d


# ======================================================================
# The page
# ======================================================================
PAGINA = """<!doctype html><meta charset=utf-8>
<title>what the robot sees</title>
<style>
 body{margin:0;background:#0e1116;color:#d7dde5;font:14px system-ui,sans-serif;
      display:flex;flex-wrap:wrap;gap:16px;padding:16px}
 img{background:#000;border-radius:6px;max-width:min(100%,720px)}
 #panel{min-width:280px;flex:1}
 h2{font-size:13px;text-transform:uppercase;letter-spacing:.08em;
    color:#7f8b9c;margin:18px 0 6px}
 table{border-collapse:collapse;width:100%}
 td{padding:3px 6px;border-bottom:1px solid #1c222b}
 td:last-child{text-align:right;font-variant-numeric:tabular-nums}
 .si{color:#5fe08c} .no{color:#ff7b7b} .na{color:#69737f}
 input,button{font:inherit;background:#1a212b;color:#d7dde5;
   border:1px solid #2b3543;border-radius:5px;padding:6px 10px}
 button{cursor:pointer} button:hover{background:#243040}
 #guardado{color:#5fe08c;min-height:1em;font-size:12px;margin-top:6px}
</style>
<img src="/camara.mjpg" alt="camera">
<div id=panel>
  <h2>right now</h2><table id=vivo></table>
  <h2>gates</h2><table id=puertas></table>
  <h2>ranges seen (for the threshold)</h2><table id=stats></table>
  <h2>calibration</h2>
  <input id=nota placeholder="pose, e.g. against wall + right triangle" size=26>
  <button onclick="guardar()">save sample</button>
  <div id=guardado></div>
</div>
<script>
var F = function (v, n) { return v == null ? '--' : (+v).toFixed(n == null ? 2 : n); };
var OK = function (v) {
  if (v == null) { return '<span class=na>--</span>'; }
  return v ? '<span class=si>yes</span>' : '<span class=no>NO</span>';
};
var fila = function (k, v) { return '<tr><td>' + k + '</td><td>' + v + '</td></tr>'; };

function tic() {
  fetch('/datos').then(function (r) { return r.json(); }).then(function (d) {
    document.getElementById('vivo').innerHTML =
      fila('figure', d.figura || '<span class=na>none</span>') +
      fila('confidence', F(d.confianza)) +
      fila('frame area', d.area_frac == null ? '--'
           : (100 * d.area_frac).toFixed(2) + '%') +
      fila('ToF frontal', d.dist_frontal == null
           ? '<span class=na>standalone</span>' : F(d.dist_frontal, 0) + ' mm') +
      fila('last obeyed', d.ultima_figura || '<span class=na>none</span>') +
      fila('navigator reads', d.lecturas);

    document.getElementById('puertas').innerHTML =
      fila('distance below ' + F(d.dist_decision, 0) + ' mm', OK(d.puerta_dist)) +
      fila('area above ' + (100 * d.area_decision).toFixed(1) + '%', OK(d.puerta_area)) +
      fila('confidence above threshold', OK(d.puerta_conf)) +
      fila('<b>would obey</b>', OK(d.decidiria));

    var claves = Object.keys(d.stats);
    document.getElementById('stats').innerHTML = claves.length
      ? claves.map(function (f) {
          var s = d.stats[f];
          return fila(f + ' <span class=na>(' + s.n + ')</span>',
            'conf ' + F(s.conf_min) + '-' + F(s.conf_max) +
            ' / area ' + (100 * s.area_min).toFixed(1) + '-' +
            (100 * s.area_max).toFixed(1) + '%');
        }).join('')
      : fila('<span class=na>no figures yet</span>', '');
  }).catch(function () { /* the Pi restarting: the next tick retries */ });
}

function guardar() {
  var nota = document.getElementById('nota').value;
  fetch('/muestra', { method: 'POST', body: JSON.stringify({ nota: nota }) })
    .then(function (r) { return r.json(); })
    .then(function (d) {
      document.getElementById('guardado').textContent =
        'saved: ' + (d.figura || 'no figure') + ' conf ' + F(d.confianza) +
        ' -> camera_calibration.csv';
    });
}

setInterval(tic, 200);
tic();
</script>
"""


def crear_servidor(camara, puerto=PUERTO_PREVIA):
    """Already assembled ThreadingHTTPServer (not started). Same pattern as
    web_interface.crear_servidor, so it reads the same way."""

    class Manejador(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass            # the console belongs to the robot, not to the server

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
            if ruta in ("/", "/index.html"):
                self._responder(200, PAGINA, "text/html; charset=utf-8")
                return
            if ruta == "/datos":
                self._responder(200, json.dumps(camara.datos()))
                return
            if ruta != "/camara.mjpg":
                self._responder(404, json.dumps({"error": "not found"}))
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=cuadro")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                visto = -1
                while True:
                    with camara._lock:
                        n, jpeg = camara._jpeg_n, camara._jpeg
                    if jpeg is None or n == visto:
                        time.sleep(0.01)        # only a NEW frame is sent
                        continue
                    visto = n
                    self.wfile.write(b"--cuadro\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        ("Content-Length: %d\r\n\r\n" % len(jpeg)).encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass        # the tab was closed; it is not an error

        def do_POST(self):
            if self.path.split("?")[0] != "/muestra":
                self._responder(404, json.dumps({"error": "not found"}))
                return
            largo = int(self.headers.get("Content-Length", 0))
            try:
                cuerpo = json.loads(self.rfile.read(largo) or b"{}")
            except ValueError:
                cuerpo = {}
            self._responder(200, json.dumps(
                camara.guardar_muestra(str(cuerpo.get("nota", ""))[:120])))

    servidor = ThreadingHTTPServer(("0.0.0.0", puerto), Manejador)
    servidor.daemon_threads = True
    return servidor


def arrancar_en_hilo(camara, puerto=PUERTO_PREVIA):
    """The one main.py calls. Returns the server in case it has to be closed."""
    servidor = crear_servidor(camara, puerto)
    threading.Thread(target=servidor.serve_forever, daemon=True).start()
    return servidor


def main():
    p = argparse.ArgumentParser(
        description="The robot camera, standalone, to calibrate in the maze")
    p.add_argument("--camera", "--camara", dest="camara", type=int, default=0, help="/dev/video* index")
    p.add_argument("--width", "--ancho", dest="ancho", type=int, default=640)
    p.add_argument("--height", "--alto", dest="alto", type=int, default=480)
    p.add_argument("--port", "--puerto", dest="puerto", type=int, default=PUERTO_PREVIA)
    p.add_argument("--look-area", "--area-mirar", dest="area_mirar", type=float, default=AREA_MIRAR,
                   help="wide threshold: used to LOOK (default: %(default)s)")
    p.add_argument("--decision-area", "--area-decision", dest="area_decision", type=float, default=AREA_DECISION,
                   help="narrow threshold: used to DECIDE (default: %(default)s)")
    p.add_argument("--confidence", "--confianza", dest="confianza", type=float, default=None,
                   help="confidence threshold for all figures")
    args = p.parse_args()

    cam = CamaraMaster(activa=True, indice=args.camara,
                       area_mirar=args.area_mirar,
                       area_decision=args.area_decision,
                       confianza_minima=args.confianza,
                       ancho=args.ancho, alto=args.alto)
    # Standalone there is no ToF: the distance gate stays open and the page
    # says so ("standalone"). Inside the robot the navigator closes it.
    servidor = crear_servidor(cam, args.puerto)
    print("camera ready. open http://<pi-ip>:{}  (ctrl-c to quit)"
          .format(args.puerto), flush=True)
    print("samples go to {}".format(RUTA_CALIBRACION), flush=True)
    try:
        servidor.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cam.cerrar()        # 🔴 otherwise /dev/video0 stays taken
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
