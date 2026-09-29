#!/usr/bin/env python3
"""Camera test: recognizing the maze figures with plain OpenCV.

    python3 test_camera.py                      live camera, with a window
    python3 test_camera.py --no-window          same, printing to the console
    python3 test_camera.py --photo wall.jpg     on a saved photo
    python3 test_camera.py --selftest           no camera: generated figures

Requires OpenCV:  pip3 install opencv-python   (on the Pi, the system
python3-opencv is faster:  sudo apt install python3-opencv)

WHAT HAS TO BE DECIDED HERE, AND IT IS THE ONLY THING THAT MATTERS
    Whether classical vision is enough. The acid test is the TRIANGLE: it is
    not enough to see "a triangle", one has to say where it points -- a >
    and a < are the same figure rotated and mean opposite things. That is
    solved with contour geometry, without training anything: find the
    vertex farthest from the center and look at which side it falls. If
    this works in the real corridor, the CNN is unnecessary.

THE RULES THE PROJECT ASKED FOR
    1. THE FIGURE OVERRIDES THE RL. It is not one more vote: it is an
       instruction. This file only detects; the one that obeys is
       navigator.decidir(), which already calls agente.forzar() when there
       is a figure.
    2. DECIDE ONLY WHEN CLOSE TO THE WALL. A figure seen from afar belongs
       to another cell, and obeying it six cells early sends the robot
       anywhere. Here that is solved with TWO gates, and both are needed:
         - AREA: the figure has to fill at least AREA_MINIMA of the frame.
           A figure at the end of the corridor fills a tiny fraction.
         - DISTANCE: the front ToF has to be below DIST_DECISION_MM. This
           gate is set by the navigator, which is the one that has the
           telemetry; here only the number and the slot are left.
       The area alone is not enough: a large figure painted far away can
       fill as much as a small one up close. The ToF alone is not enough
       either: it says there is a wall, not that there is a figure ON that
       wall.
    3. And a third one, common sense: STABILITY. The same figure has to be
       seen in FRAMES_ESTABLES consecutive frames. A single frame with an
       odd reflection cannot change the route.

HOW IT PLUGS IN LATER (it is already designed for that)
    `perception.Camara` has the same `leer()` as this class: it returns one
    of the four figures or None. To integrate it:
        from test_camera import DetectorFiguras, CamaraReal
    and delegate in `Camara.leer()`. The figure names are EXACTLY those of
    perception.FIGURAS, so nothing has to be translated along the way.
"""

import argparse
import sys
import time

import cv2
import numpy as np

# The same names as perception.py. Do not change them without changing them there.
FIGURA_TRIANGULO_IZQ = "triangulo_izq"
FIGURA_TRIANGULO_DER = "triangulo_der"
FIGURA_CIRCULO = "circulo"
FIGURA_CUADRADO = "cuadrado"

# ---------------------------------------------------------------- settings
# Fraction of the frame the figure has to fill to be obeyed. With the camera
# ~15 cm from the wall an 8 cm figure fills quite a lot more than this; at
# one meter it does not even come close. ADJUST by looking at what the
# calibration key prints with the robot placed where it will really decide.
AREA_MINIMA = 0.045          # 4.5% of the frame
AREA_MAXIMA = 0.80           # above this, the camera is covered
# Distance gate, for when this is integrated: the navigator only looks at
# the camera if the front ToF is below this.
DIST_DECISION_MM = 160.0
FRAMES_ESTABLES = 3          # consecutive frames with the same figure

# A triangle whose vertex points almost up or almost down is not a left or
# right instruction: it is a badly seen or badly stuck triangle, and it is
# discarded instead of guessing.
COS_MINIMO_HORIZONTAL = 0.55


class Deteccion:
    __slots__ = ("figura", "area_frac", "contorno", "vertices", "confianza")

    def __init__(self, figura, area_frac, contorno, vertices, confianza):
        self.figura = figura
        self.area_frac = area_frac
        self.contorno = contorno
        self.vertices = vertices
        self.confianza = confianza

    def __repr__(self):
        return "{} ({:.0f}% of the frame, conf {:.2f})".format(
            self.figura, 100 * self.area_frac, self.confianza)


class DetectorFiguras:
    """Classical vision: adaptive threshold, contours and geometry.

    No training, no model and no GPU. It runs easily on a Pi.
    """

    def __init__(self, area_minima=AREA_MINIMA, area_maxima=AREA_MAXIMA):
        self.area_minima = area_minima
        self.area_maxima = area_maxima

    # -- the previous step: leave the figure as a clean blob ----------------
    def _binarizar(self, bgr):
        gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gris = cv2.GaussianBlur(gris, (5, 5), 0)
        # ADAPTIVE threshold and not a fixed one: the light in the maze is not
        # uniform and a global threshold loses the figure as soon as a lamp
        # hits it from the side. The large block (31) prevents the paper
        # texture from turning into contours itself.
        binaria = cv2.adaptiveThreshold(
            gris, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 31, 8)
        # Closing: joins the pieces of a line cut by a glare.
        nucleo = np.ones((5, 5), np.uint8)
        return cv2.morphologyEx(binaria, cv2.MORPH_CLOSE, nucleo)

    # -- from a contour to a figure -----------------------------------------
    def _clasificar(self, contorno, area_cuadro):
        area = cv2.contourArea(contorno)
        if area <= 0:
            return None
        area_frac = area / area_cuadro
        if not (self.area_minima <= area_frac <= self.area_maxima):
            return None

        perimetro = cv2.arcLength(contorno, True)
        if perimetro <= 0:
            return None
        aprox = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
        n = len(aprox)
        # Circularity: 1 is a perfect circle, a square gives ~0.79 and a
        # triangle ~0.60. It is what separates the circle from a polygon with
        # many vertices because of edge noise.
        circularidad = 4 * np.pi * area / (perimetro * perimetro)

        # How solid it is: contour area against its convex hull. A painted
        # figure is solid; a reflection or a shadow is not.
        envolvente = cv2.convexHull(contorno)
        area_env = cv2.contourArea(envolvente)
        solidez = area / area_env if area_env > 0 else 0.0
        if solidez < 0.85:
            return None

        if n == 3:
            figura, conf = self._triangulo(aprox)
            if figura is None:
                return None
            return Deteccion(figura, area_frac, contorno, aprox, conf)

        if n == 4:
            x, y, w, h = cv2.boundingRect(aprox)
            razon = w / float(h) if h else 0
            if 0.75 <= razon <= 1.33:
                return Deteccion(FIGURA_CUADRADO, area_frac, contorno, aprox,
                                 min(1.0, solidez))
            return None

        if n >= 5 and circularidad > 0.78:
            return Deteccion(FIGURA_CIRCULO, area_frac, contorno, aprox,
                             min(1.0, circularidad))
        return None

    def _triangulo(self, aprox):
        """Where it points. Returns (figure, confidence).

        🔑 THE APEX IS THE VERTEX FARTHEST FROM THE CENTER. In an isosceles
        triangle pointing to one side, the two base vertices are at the same
        distance from the centroid and the tip one is farther. There is no
        need to know the camera orientation or the size of the figure: only
        three distances have to be compared.
        """
        pts = aprox.reshape(3, 2).astype(float)
        centro = pts.mean(axis=0)
        distancias = np.linalg.norm(pts - centro, axis=1)
        apice = pts[int(np.argmax(distancias))]

        v = apice - centro
        norma = np.linalg.norm(v)
        if norma < 1e-6:
            return None, 0.0
        # Cosine of the angle with the horizontal: 1 = points sideways, 0 =
        # points up or down. If the tip is not clearly lateral, no decision.
        cos_h = abs(v[0]) / norma
        if cos_h < COS_MINIMO_HORIZONTAL:
            return None, 0.0

        # In image coordinates x grows towards the RIGHT of what the camera
        # sees, which is the right of the robot: the robot and the camera face
        # the same way. A ">" points to the right.
        figura = FIGURA_TRIANGULO_DER if v[0] > 0 else FIGURA_TRIANGULO_IZQ
        return figura, float(cos_h)

    # -- the public entry point ----------------------------------------------
    def detectar(self, bgr):
        """Returns the largest Deteccion in the frame, or None.

        The largest and not the first one: in the corridor the figure of the
        next cell may be seen out of the corner of the eye, and the one that
        rules is the one in front.
        """
        alto, ancho = bgr.shape[:2]
        area_cuadro = float(alto * ancho)
        binaria = self._binarizar(bgr)
        contornos, _ = cv2.findContours(binaria, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        mejor = None
        for c in contornos:
            d = self._clasificar(c, area_cuadro)
            if d and (mejor is None or d.area_frac > mejor.area_frac):
                mejor = d
        return mejor


class LectorEstable:
    """Requires seeing the same figure for several consecutive frames.

    A single frame with an odd reflection cannot change the robot's route.
    """

    def __init__(self, detector, frames=FRAMES_ESTABLES):
        self.detector = detector
        self.frames = frames
        self._ultima = None
        self._cuenta = 0
        self.deteccion = None       # the last raw one, to draw it

    def leer(self, bgr):
        d = self.detector.detectar(bgr)
        self.deteccion = d
        figura = d.figura if d else None
        if figura == self._ultima:
            self._cuenta += 1
        else:
            self._ultima = figura
            self._cuenta = 1
        if figura is not None and self._cuenta >= self.frames:
            return figura
        return None

    def reiniciar(self):
        self._ultima = None
        self._cuenta = 0


class CamaraReal:
    """The USB webcam. It is opened ONCE: opening it per frame costs hundreds of ms.

    The Logitech delivers MJPG; asking for it explicitly prevents the driver
    from delivering YUYV, which at 640x480 halves the frames per second.
    """

    def __init__(self, indice=0, ancho=640, alto=480):
        # Explicit V4L2: otherwise OpenCV tries GStreamer and even drivers of
        # cameras we do not have before giving up, and 13 seconds are lost on
        # every start. On Linux the USB webcam is always V4L2.
        self.cap = cv2.VideoCapture(indice, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                "could not open camera {}. If `ls /dev/video*` sees it, "
                "it is usually taken by another process: `sudo fuser -v "
                "/dev/video0` tells which one.".format(indice))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        # Buffer of 1: otherwise frames from half a second ago are read and
        # the robot decides with what it saw before braking.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def leer(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def cerrar(self):
        self.cap.release()


# ====================================================================
# Drawing and test modes
# ====================================================================
COLORES = {FIGURA_TRIANGULO_IZQ: (80, 200, 255),
           FIGURA_TRIANGULO_DER: (80, 255, 140),
           FIGURA_CIRCULO: (255, 170, 80),
           FIGURA_CUADRADO: (200, 140, 255)}


def dibujar(frame, deteccion, estable):
    if deteccion is not None:
        color = COLORES.get(deteccion.figura, (200, 200, 200))
        cv2.drawContours(frame, [deteccion.contorno], -1, color, 2)
        cv2.putText(frame, "{} {:.0f}%".format(deteccion.figura,
                                               100 * deteccion.area_frac),
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cerca = deteccion.area_frac >= AREA_MINIMA
        cv2.putText(frame, "close" if cerca else "FAR: not obeyed",
                    (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (80, 255, 140) if cerca else (80, 80, 255), 2)
    else:
        cv2.putText(frame, "no figure", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 2)
    if estable:
        cv2.putText(frame, "ORDER: " + estable, (10, frame.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return frame


def figura_sintetica(clase, lado=480, escala=0.35, giro=0.0):
    """Draws a black figure on white, to test without a camera."""
    img = np.full((lado, lado, 3), 245, np.uint8)
    c = lado // 2
    r = int(lado * escala)
    if clase == "circulo":
        cv2.circle(img, (c, c), r, (20, 20, 20), -1)
    elif clase == "cuadrado":
        cv2.rectangle(img, (c - r, c - r), (c + r, c + r), (20, 20, 20), -1)
    else:
        signo = 1 if clase == "triangulo_der" else -1
        pts = np.array([[c + signo * r, c],
                        [c - signo * r, c - r],
                        [c - signo * r, c + r]], np.float32)
        if giro:
            m = cv2.getRotationMatrix2D((c, c), giro, 1.0)
            pts = cv2.transform(pts.reshape(1, -1, 2), m).reshape(-1, 2)
        cv2.fillPoly(img, [pts.astype(np.int32)], (20, 20, 20))
    return img


def autoprueba():
    """The acid test, without hardware: triangles to one side and the other,
    rotated a few degrees, plus circle and square."""
    det = DetectorFiguras()
    casos = []
    for clase in ("triangulo_der", "triangulo_izq"):
        for giro in (0, 8, -8, 15, -15):
            casos.append((clase, giro))
    casos += [("circulo", 0), ("cuadrado", 0)]

    aciertos = 0
    for clase, giro in casos:
        img = figura_sintetica(clase, giro=giro)
        d = det.detectar(img)
        obtenido = d.figura if d else "nothing"
        ok = obtenido == clase
        aciertos += ok
        print("  {:<16} rot {:+3}  ->  {:<16} {}".format(
            clase, giro, obtenido, "ok" if ok else "FAIL"))

    # And the case that must NOT be obeyed: a triangle pointing up.
    img = figura_sintetica("triangulo_der", giro=90)
    d = det.detectar(img)
    bien = d is None
    print("  {:<16} rot +90 ->  {:<16} {}".format(
        "points up", d.figura if d else "nothing",
        "ok (no decision)" if bien else "FAIL: it decided"))
    aciertos += bien

    # And the proximity gate: the same figure, small = far.
    lejos = figura_sintetica("triangulo_der", escala=0.06)
    d = det.detectar(lejos)
    bien = d is None
    print("  {:<16} far      ->  {:<16} {}".format(
        "triangulo_der", d.figura if d else "nothing",
        "ok (discarded by area)" if bien else "FAIL: it would obey from afar"))
    aciertos += bien

    total = len(casos) + 2
    print("\n{}/{} cases correct".format(aciertos, total))
    return 0 if aciertos == total else 1


def en_vivo(args):
    det = DetectorFiguras(area_minima=args.area_minima)
    lector = LectorEstable(det, frames=args.estables)

    if args.foto:
        frame = cv2.imread(args.foto)
        if frame is None:
            sys.exit("could not read " + args.foto)
        estable = None
        for _ in range(args.estables):        # the photo does not change
            estable = lector.leer(frame)
        print("detection: {}".format(lector.deteccion))
        print("stable order: {}".format(estable))
        if not args.sin_ventana:
            cv2.imshow("figure", dibujar(frame, lector.deteccion, estable))
            cv2.waitKey(0)
        return 0

    cam = CamaraReal(args.camara, args.ancho, args.alto)
    print("camera open. q to quit, c to dump the calibration numbers")
    ultimo_aviso = 0.0
    try:
        while True:
            frame = cam.leer()
            if frame is None:
                print("frame lost")
                continue
            estable = lector.leer(frame)
            d = lector.deteccion

            if args.sin_ventana:
                ahora = time.time()
                if estable or (d and ahora - ultimo_aviso > 0.5):
                    ultimo_aviso = ahora
                    print("{}  order={}".format(d, estable))
                if estable:
                    lector.reiniciar()
                continue

            cv2.imshow("camera test", dibujar(frame, d, estable))
            tecla = cv2.waitKey(1) & 0xFF
            if tecla == ord("q"):
                break
            if tecla == ord("c") and d:
                print("area_frac={:.4f}  vertices={}  conf={:.2f}".format(
                    d.area_frac, len(d.vertices), d.confianza))
    finally:
        cam.cerrar()
        cv2.destroyAllWindows()
    return 0


def main():
    p = argparse.ArgumentParser(description="Camera and figures test")
    p.add_argument("--camera", "--camara", dest="camara", type=int, default=0, help="/dev/video* index")
    p.add_argument("--width", "--ancho", dest="ancho", type=int, default=640)
    p.add_argument("--height", "--alto", dest="alto", type=int, default=480)
    p.add_argument("--photo", "--foto", dest="foto", help="test on a saved image")
    p.add_argument("--selftest", "--autoprueba", dest="autoprueba", action="store_true",
                   help="no camera: generated figures, including the acid test")
    p.add_argument("--no-window", "--sin-ventana", dest="sin_ventana", action="store_true",
                   help="no window (the Pi over SSH); prints to the console")
    p.add_argument("--min-area", "--area-minima", dest="area_minima", type=float, default=AREA_MINIMA,
                   help="fraction of the frame to consider the figure close")
    p.add_argument("--stable", "--estables", dest="estables", type=int, default=FRAMES_ESTABLES,
                   help="consecutive frames with the same figure")
    args = p.parse_args()

    if args.autoprueba:
        return autoprueba()
    return en_vivo(args)


if __name__ == "__main__":
    sys.exit(main())
