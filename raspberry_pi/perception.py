"""Camera + CNN: the four states still to be implemented.

STATUS: SKELETON ON PURPOSE. Today `Camara.leer()` always returns None
("I see no figure") and the navigator behaves exactly as if this file did
not exist. Plugging in the real CNN means filling ONE method; the state
machine and the link do not need to be touched.
(The figures ended up being recognized with classical OpenCV contour
analysis in test_camera.py / camera_master.py; the CNN was never plugged in.)

WHAT EACH FIGURE MEANS (INSTRUCTIONS IN THE MAZE, 2026-09-12; replaces what
was in PROMPT_migracion_mapa_a_pi.txt):
    right triangle   -> turn right 90
    left triangle    -> turn left 90
    circle           -> U-turn (180): go back
    square           -> STOP 10 s and RE-EVALUATE: the wall in front may or
                        may not disappear. It does not impose a direction:
                        after the wait the ToF are read again and the RL
                        decides (navigator.py, config.ESPERA_CUADRADO_S).

WHY THE FIGURES ARE "INTENTIONS" AND NOT DIRECTIONS: a figure says "left"
relative to HOW THE ROBOT IS FACING, not relative to the North of the maze.
The translation to an absolute direction needs the orientation of the
agent, and that is why it lives in navigator.py (intencion_a_direccion), not
here. This file only recognizes drawings.
"""


# --- the four intentions ----------------------------------------------------
# They are numbered like the relative indices of rl.py (0=front, 1=right,
# 2=back, 3=left) where it makes sense, so rl.absoluta_desde_relativa() can
# be used without intermediate tables.
FIGURA_TRIANGULO_IZQ = "triangulo_izq"
FIGURA_TRIANGULO_DER = "triangulo_der"
FIGURA_CIRCULO = "circulo"
FIGURA_CUADRADO = "cuadrado"

FIGURAS = (FIGURA_TRIANGULO_IZQ, FIGURA_TRIANGULO_DER,
           FIGURA_CIRCULO, FIGURA_CUADRADO)

# Index relative to the heading imposed by each figure. None = it imposes no
# direction (the square is "wait 10 s and look again", and then the RL
# decides; the circle is handled separately because the agent's permission
# to go back must also be lifted).
REL_DE_FIGURA = {
    FIGURA_TRIANGULO_IZQ: 3,     # left
    FIGURA_TRIANGULO_DER: 1,     # right
    FIGURA_CIRCULO: 2,           # back (U-turn)
    FIGURA_CUADRADO: None,       # wait and re-evaluate (navigator.py)
}


# 🔴 THE DISTANCE GATE. A figure seen from afar belongs to ANOTHER cell:
# obeying it six cells early sends the robot anywhere. The camera is only
# looked at when the front ToF says the wall is right there. The second gate
# (the figure must fill a minimum of the frame) lives in
# test_camera.DetectorFiguras, and both are needed: the area alone can be
# fooled by a large figure painted far away, and the ToF alone only says
# there is a wall, not that there is a figure ON that wall.
DIST_DECISION_MM = 160.0


class Camara:
    """Minimal interface the navigator expects.

    With `activa=False` (the normal case until the camera is mounted) it
    always returns None and the navigator behaves as if this file did not
    exist. With `activa=True` it delegates to test_camera.py, which is where
    the vision lives and where it is tested alone with `--autoprueba`.

    The navigator calls it ONLY with the robot stopped, in the OBSERVE
    state: a blurry photo is worth nothing, and the robot already stops at
    the end of each cell.
    """

    def __init__(self, activa=False, indice=0, dist_decision=DIST_DECISION_MM):
        self.activa = activa
        self.dist_decision = dist_decision
        self.ultima_figura = None
        self.lecturas = 0
        self._cam = None
        self._lector = None
        if activa:
            # Imported here and not at the top: without a mounted camera it
            # makes no sense to require OpenCV to run the rest.
            from test_camera import CamaraReal, DetectorFiguras, LectorEstable
            self._cam = CamaraReal(indice)
            self._lector = LectorEstable(DetectorFiguras())

    def disponible(self):
        return self.activa

    def leer(self, dist_frontal_mm=None):
        """Returns one of FIGURAS, or None.

        `dist_frontal_mm` is the front ToF. If it is given and is above
        `dist_decision`, the camera is not even looked at: no deciding from
        afar.
        """
        self.lecturas += 1
        if not self.activa:
            return None
        if dist_frontal_mm is not None and dist_frontal_mm > self.dist_decision:
            self._lector.reiniciar()   # what was seen from afar does not count
            return None
        cuadro = self._cam.leer()
        if cuadro is None:
            return None
        figura = self._lector.leer(cuadro)
        if figura:
            self.ultima_figura = figura
            self._lector.reiniciar()   # a figure is obeyed ONCE
        return figura

    def cerrar(self):
        if self._cam:
            self._cam.cerrar()
