#!/usr/bin/env python3
"""Integration of camera_master + navigator, WITHOUT a physical camera.

    python3 test_camera_integration.py

It is the third headless test of this project, next to test_headless.py
(the Pi stack) and test_camera.py --selftest (the vision). This one covers
what neither of the two sees: that the camera, already mounted inside the
robot, OVERRIDES the RL at the right moment and only at that moment.

CamaraReal is replaced by one that returns synthetic figures, and the
navigator runs against the fake ESP of test_headless. What is checked:

  1. With a wall in front (ToF 80 mm) and a ">" in view, the navigator's
     decision is the figure and NOT the RL, and the direction is the one to
     the right of the heading.
  2. With the corridor free (ToF 900 mm) the camera is not even looked at:
     the RL rules.
  3. The reader thread closes and does not leave the camera taken.
"""
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)

import rl                                    # noqa: E402
import test_camera as pc                     # noqa: E402
import perception as percepcion             # noqa: E402


class CamaraFalsa:
    """Always returns the same synthetic frame. Same API as CamaraReal,
    including the three positional arguments: camera_master builds it with
    (indice, ancho, alto), and if the signature does not match this blows
    up here and not with the robot in hand."""

    def __init__(self, indice=0, ancho=640, alto=480,
                 clase="triangulo_der", escala=0.35):
        self.cuadro = pc.figura_sintetica(clase, escala=escala)
        self.cerrada = False

    def leer(self):
        return self.cuadro.copy()

    def cerrar(self):
        self.cerrada = True


pc.CamaraReal = CamaraFalsa                  # before importing camera_master

import camera_master as camara_master        # noqa: E402
import serial_link as en                     # noqa: E402
import maze_map as mapa_mod                  # noqa: E402
import navigator as nav_mod                  # noqa: E402
from test_headless import ESPFalsa           # noqa: E402

fallos = []


def revisar(cond, texto):
    print("  {}  {}".format("ok  " if cond else "FAIL ", texto))
    if not cond:
        fallos.append(texto)


# --- 1. the camera alone, with the three gates ------------------------
print("camera_master, gates:")
cam = camara_master.CamaraMaster(activa=True, indice=0)
revisar(cam.disponible(), "disponible() with activa=True")
revisar(cam.leer(dist_frontal_mm=900.0) is None,
        "with the corridor free (900 mm) the camera is not looked at")
figura = cam.leer(dist_frontal_mm=80.0)
revisar(figura == percepcion.FIGURA_TRIANGULO_DER,
        "close to the wall (80 mm) returns triangulo_der, not {}".format(figura))
d = cam.datos()
revisar(d["decidiria"] and d["puerta_area"] and d["puerta_conf"],
        "the three gates pass and the page reports it")

# the same figure small = far: it passes the distance gate but not the area
# one, which is exactly the case the two gates together cover
cam.cerrar()
lejos = camara_master.CamaraMaster(activa=True, indice=0)
lejos._cam = CamaraFalsa(escala=0.05)
revisar(lejos.leer(dist_frontal_mm=80.0) is None,
        "small figure (far) does not decide even if the ToF says there is a wall")
lejos.cerrar()

# --- 2. inside the navigator --------------------------------------------
print("\nnavigator with the camera on:")
paredes = rl.generar_laberinto(lado=8, semilla=1)
esp = ESPFalsa(paredes, 8, (7, 7))
agente = rl.AgenteRL(lado=8, meta=(7, 7),
                     ruta_estado=os.path.join(AQUI, "_estado_integracion.json"),
                     semilla=1)
el_mapa = mapa_mod.Mapa(8, (0, 0), 6)
cam2 = camara_master.CamaraMaster(activa=True, indice=0)
nav = nav_mod.Navegador(agente, el_mapa, esp, camara=cam2, meta=(7, 7))
nav.preparar_corrida()

con_figura = sin_figura = 0
mando_la_figura = None
for _ in range(25):
    obs, telem, fig = nav.observar()
    hay_pared_delante = telem.distancias["N"] <= percepcion.DIST_DECISION_MM
    if hay_pared_delante:
        con_figura += fig is not None
        if fig and mando_la_figura is None:
            direccion, motivo = nav.decidir(obs, fig)
            esperada = rl.absoluta_desde_relativa(1, nav.heading)   # right
            mando_la_figura = (direccion == esperada
                               and motivo.startswith("figura:"),
                               direccion, motivo)
    else:
        sin_figura += fig is None
    nav.un_paso()

revisar(mando_la_figura and mando_la_figura[0],
        "with a wall in front the figure rules, and it goes right of the heading"
        + ("" if not mando_la_figura else ": {} ({})".format(*mando_la_figura[1:])))

revisar(con_figura > 0, "a cell with a wall in front and a figure was reached")
revisar(sin_figura > 0, "in an open corridor the camera returned None")

cam2.cerrar()
revisar(cam2._cam is None, "cerrar() releases the camera (it does not stay taken)")
for sufijo in (".json", "_red.npz"):
    ruta = os.path.join(AQUI, "_estado_integracion" + sufijo)
    if os.path.exists(ruta):
        os.remove(ruta)

print("\n{}".format("ALL GOOD" if not fallos else "{} FAILED: {}".format(
    len(fallos), fallos)))
sys.exit(1 if fallos else 0)
