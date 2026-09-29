"""
========================================================================
RASPBERRY PI RL BLOCK -- CELL-LEVEL reinforcement learning
========================================================================

WHAT IT DOES
    Decides which direction to take in each cell and learns from every
    punishment and every reward, online, with no separate training phase.
    What it learns survives a power-off (state file) and makes the SECOND
    run of the same maze much shorter than the first.

    It works at the cell level, not at the physics level: one RL step = one
    command sent to the ESP32 + the event the ESP sends back. The ESP does
    not know this module exists.

THREE MEMORIES THAT LEARN AT THE SAME TIME (this is the central idea)
    1. Q TABLE WITH TRACES -- EXACT memory of THIS maze, indexed by cell.
       With TD(lambda) eligibility traces, the goal reward propagates
       backwards along the WHOLE route travelled at the very moment the
       goal is reached. This is what makes run 2 fast.
    2. Q NETWORK (8-32-32-4) -- GENERAL memory, indexed by continuous
       features (distances, vector to the goal, memory/reverse neurons of
       the neurocontroller). It generalizes to never-seen cells and to other
       mazes. Same architecture and same input order as the old firmware,
       so it can start from the already trained weights in
       q_network_weights.h.
    3. PATTERN MEMORY -- STRUCTURAL memory, indexed by the local signature
       (wall pattern + goal sector) expressed IN THE HEADING FRAME, with
       relative actions (front/right/back/left). Being invariant to position
       and orientation, it is the one that really "understands the pattern":
       it learns things like "if there is a wall in front and the goal is to
       the right, turn right", and applies it in a corner it never visited.

    The three vote in a single score per direction; each one is updated
    with its own TD error (mixing the targets destabilizes learning).

PLUS ONE MEMORY THAT DOES NOT LEARN, IT ONLY REMEMBERS
    4. MASTER ROUTE -- the best trajectory to the goal, with loops removed.
       In later runs it is followed in exploitation mode, checking in each
       cell that the walls match; if they do not, it falls back to the
       learned policy. It is the direct shortcut to "already knows what to
       do".

COMPUTATIONAL COST (Raspberry Pi, per cell decision)
    Q table + patterns: microseconds (small dictionaries).
    Q network: two 32x32 matmuls in numpy, ~50 us. The update (forward +
    backward) ~150 us. Since it runs ONCE PER CELL (not per camera frame or
    per CSV line), the RL cost is negligible compared with the CNN.
    Memory: the Q table grows as (visited cells) x 4 floats.

IMPORTANCE
    CRITICAL for the goal of the exam. It is the only block that improves
    by itself between runs. It can be disabled by setting PESO_TABLA=PESO_RED=
    PESO_PATRON=0: the robot then decides only with Tremaux + neuro, which
    works as an A/B test that the RL really contributes.

CONVENTIONS (identical to the firmware -- do not "fix" them)
    DIRS4 = N,E,S,O in CLOCKWISE order (O = West). dx: E=+1, O=-1.
    dy: S=+1, N=-1. The Y axis grows towards the SOUTH.
    Relative index: 0=front, 1=right, 2=back, 3=left.
"""

from __future__ import annotations

import json
import math
import os
import random
import re
from collections import defaultdict

import numpy as np

# ######################################################################
# BLOCK 1 -- DIRECTION CONVENTIONS
# WHAT IT DOES: the same heading arithmetic as the firmware, in Python.
# COST: none.
# IMPORTANCE: HIGH -- an inverted sign here invalidates everything learned.
# ######################################################################
DIRS4 = ("N", "E", "S", "O")
IDX_DIR = {d: i for i, d in enumerate(DIRS4)}
DX = {"N": 0, "E": 1, "S": 0, "O": -1}
DY = {"N": -1, "E": 0, "S": 1, "O": 0}
CONTRARIA = {"N": "S", "S": "N", "E": "O", "O": "E"}

# Codes the ESP32 understands (BLOCK 01 of firmware/src/main.cpp).
#
# 🔴 THE CHASSIS NO LONGER TRANSLATES SIDEWAYS OR BACKWARDS. The set of
# digits 1..4 (go to absolute North, East, ...) and 6..8 (to the right,
# back, to the left without turning) belonged to the mecanum chassis; the
# firmware RETIRED them and today answers an explicit NACK to any of them,
# so that an outdated link is noticed immediately instead of failing
# silently. The only thing that translates is "5", one cell forward; to go
# anywhere else the robot must turn first, with the IMU-closed turn
# primitives.
CMD_AVANZAR   = "5"     # one cell forward (the only translation)
CMD_GIRO_DER  = "R"     # +90 degrees
CMD_GIRO_IZQ  = "L"     # -90 degrees
CMD_GIRO_180  = "T"     # half turn, in ONE command

# By index relative to the front: 0=front, 1=right, 2=back, 3=left.
# "back" is no longer a translation: it is the half turn.
CMD_RELATIVO = {0: CMD_AVANZAR, 1: CMD_GIRO_DER, 2: CMD_GIRO_180, 3: CMD_GIRO_IZQ}

# Absolute motion does not exist in the protocol: the ESP does not know
# where it is facing. The name is kept because `elegir(relativo=False)`
# returns it, but NO caller uses that value (they all do
# `direccion, _cmd = elegir(obs)`) and sending it down the cable would be a
# NACK.
CMD_ABSOLUTO = {d: None for d in DIRS4}


def rotar(direccion: str, pasos90: int) -> str:
    """+1 = right, -1 = left (DIRS4 is in clockwise order)."""
    return DIRS4[(IDX_DIR[direccion] + pasos90) % 4]


def indice_relativo(direccion: str, heading: str) -> int:
    """0=front, 1=right, 2=back, 3=left, relative to the heading."""
    return (IDX_DIR[direccion] - IDX_DIR[heading]) % 4


def absoluta_desde_relativa(rel: int, heading: str) -> str:
    return DIRS4[(IDX_DIR[heading] + rel) % 4]


# ######################################################################
# BLOCK 2 -- REWARDS AND PUNISHMENTS
# WHAT IT DOES: translates each ESP32 event into a number. It is THE piece
#   that defines which behaviour is reinforced; everything else is
#   machinery.
# COST: none.
# IMPORTANCE: CRITICAL. Design rules that were respected:
#   - The per-step cost (R_PASO) is what pushes the route to get shorter.
#     Without it, going around in circles is free.
#   - R_PROGRESO uses the DIFFERENCE of Manhattan distance to the goal
#     (potential shaping). Being a potential difference, it speeds up
#     learning without changing which policy is optimal: it is safe to
#     raise it.
#   - Getting closer and getting farther cancel out exactly, so reward
#     CANNOT be farmed by oscillating between two cells.
#   - Reaching the goal pays a speed bonus: beating one's own best time is
#     worth more than just arriving. That is what keeps refining the route
#     after the first successful run.
# ######################################################################
R_PASO = -0.05          # cost of existing: pushes towards shorter routes
R_PROGRESO = 0.00       # Sep 16: the exit is NOT at META (random maze);
                        # with 0.30 the robot was biased towards corner (15,15)
# Sep 15 afternoon: the robot went back and forth between known cells.
# Curiosity raised (0.20 -> 0.60) and revisit penalty raised (-0.10 -> -0.40),
# and a dedicated punishment added for TURNING BACK (moving opposite to the
# heading).
R_CELDA_NUEVA = 0.60    # curiosity: rewards exploring in run 1
R_REVISITA = -0.80      # discourages going in circles (Sep 16: was -0.40)
R_RETROCESO = -3.00     # going back the way it came: almost impossible (Sep 15 night)
R_PARED = -0.50         # stopped by a wall before completing the cell (ev=4)
R_RECHAZO = -0.50       # command rejected, there was a wall (ev=2)
R_BLOQUEO = -1.00       # got stuck, encoders stopped (ev=6)
R_CAMPO_ABIERTO = -1.00 # left the maze (ev=5)
R_META = 10.0           # arriving
R_PRIMA_RAPIDEZ = 5.0   # extra, proportional to how much the record improves

# Learning hyperparameters. ALPHA and GAMMA are kept from the firmware and
# from rl/agente1_refuerzo_neurosimbolico/q_red.py so that the results are
# comparable with the simulation.
ALPHA_TABLA = 0.30      # Q table rate (one cell does not drag its neighbours)
LAMBDA_TRAZA = 0.85     # TD(lambda): 0 = last step only, 1 = Monte Carlo
GAMMA = 0.90            # discount
TASA_RED = 0.05         # actual network lr (ALPHA=0.30 makes it diverge)
NORMA_MAX_GRAD = 1.0    # gradient clipping
TASA_PATRON = 0.15      # moving average rate of the pattern memory

# Weights with which each memory votes in the final score.
PESO_TABLA = 1.00
PESO_RED_BASE = 0.10    # grows with the runs (see beta_actual)
PESO_RED_PASO = 0.05
PESO_PATRON = 0.60
PESO_MOMENTUM_SEGUIR = 0.25   # from the firmware: inertia, avoids zigzag
PESO_MOMENTUM_LATERAL = 0.19
PESO_MOMENTUM_ATRAS = -3.00   # turning back already weighs AGAINST it when deciding: almost impossible
PESO_NEURO = 0.40             # vote of the ESP neurocontroller

TEMP_INICIAL = 0.60
TEMP_PASO = 0.06
TEMP_MINIMA = 0.20      # Sep 16: 0.10 kept repeating the same bad path in exploitation
TRAZA_MINIMA = 1e-3     # below this the trace is discarded


# ######################################################################
# BLOCK 3 -- Q NETWORK IN NUMPY (8-32-32-4)
# WHAT IT DOES: MLP with tanh, forward and backward written by hand. Same
#   architecture, same input order and same weight layout as
#   q_network_weights.h, in order to (a) start from the weights already
#   trained over 4555 simulation runs and (b) re-export to C++ if one day
#   the network is to be moved back to the ESP.
# COST: forward ~50 us, forward+backward ~150 us per cell on a Pi 4.
# IMPORTANCE: HIGH -- it is the part that generalizes. The Q table alone
#   knows nothing about a cell it never stepped on; the network does have
#   an opinion.
# NOTE: numpy is used instead of torch on purpose -- torch on the Pi means
#   hundreds of MB and several seconds of import, for a network with 1.3k
#   parameters.
# ######################################################################
DIM_ESTADO = 8
OCULTO = 32
DIM_SALIDA = 4


class RedQ:
    """MLP Linear(8,32)-Tanh-Linear(32,32)-Tanh-Linear(32,4)."""

    def __init__(self, semilla: int = 0):
        rng = np.random.default_rng(semilla)
        esc1 = math.sqrt(1.0 / DIM_ESTADO)
        esc2 = math.sqrt(1.0 / OCULTO)
        self.W1 = rng.uniform(-esc1, esc1, (OCULTO, DIM_ESTADO))
        self.b1 = np.zeros(OCULTO)
        self.W2 = rng.uniform(-esc2, esc2, (OCULTO, OCULTO))
        self.b2 = np.zeros(OCULTO)
        self.W3 = rng.uniform(-esc2, esc2, (DIM_SALIDA, OCULTO))
        self.b3 = np.zeros(DIM_SALIDA)

    def adelante(self, x):
        """Returns (q, cache) -- the cache is reused in the backward pass."""
        z1 = self.W1 @ x + self.b1
        h1 = np.tanh(z1)
        z2 = self.W2 @ h1 + self.b2
        h2 = np.tanh(z2)
        q = self.W3 @ h2 + self.b3
        return q, (x, h1, h2)

    def predecir(self, x):
        return self.adelante(x)[0]

    def paso_sgd(self, cache, accion_idx: int, error: float, tasa: float = TASA_RED):
        """One descent step on 0.5*(Q(s,a)-target)^2.
        `error` = Q(s,a) - target (the gradient of the loss at the output).
        On a table this is exactly Q += alpha*(target - Q); on a network the
        norm has to be clipped, or a single large transition (the first time
        the goal is reached) sends the weights to NaN."""
        x, h1, h2 = cache
        gq = np.zeros(DIM_SALIDA)
        gq[accion_idx] = error

        gW3 = np.outer(gq, h2)
        gb3 = gq
        gh2 = self.W3.T @ gq
        gz2 = gh2 * (1.0 - h2 * h2)
        gW2 = np.outer(gz2, h1)
        gb2 = gz2
        gh1 = self.W2.T @ gz2
        gz1 = gh1 * (1.0 - h1 * h1)
        gW1 = np.outer(gz1, x)
        gb1 = gz1

        norma = math.sqrt(sum(float(np.sum(g * g)) for g in (gW1, gb1, gW2, gb2, gW3, gb3)))
        escala = 1.0 if norma <= NORMA_MAX_GRAD else NORMA_MAX_GRAD / (norma + 1e-12)
        k = tasa * escala

        self.W1 -= k * gW1; self.b1 -= k * gb1
        self.W2 -= k * gW2; self.b2 -= k * gb2
        self.W3 -= k * gW3; self.b3 -= k * gb3

    # --- persistence ----------------------------------------------------
    def guardar_npz(self, ruta: str):
        np.savez(ruta, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2,
                 W3=self.W3, b3=self.b3)

    def cargar_npz(self, ruta: str) -> bool:
        if not os.path.exists(ruta):
            return False
        d = np.load(ruta)
        self.W1, self.b1 = d["W1"], d["b1"]
        self.W2, self.b2 = d["W2"], d["b2"]
        self.W3, self.b3 = d["W3"], d["b3"]
        return True

    def cargar_header_cpp(self, ruta_h: str) -> bool:
        """Imports q_network_weights.h (the one the old firmware already
        uses, exported after 4555 simulation runs). This way the Pi does not
        start from scratch."""
        if not os.path.exists(ruta_h):
            return False
        txt = open(ruta_h, "r", encoding="utf-8", errors="ignore").read()

        def bloque(nombre):
            m = re.search(nombre + r"\s*(?:\[[^\]]*\])+\s*=\s*\{(.*?)\};", txt, re.S)
            if not m:
                return None
            return [float(v) for v in re.findall(r"-?\d+\.?\d*(?:e-?\d+)?f?", m.group(1).replace("f", ""))]

        try:
            w1 = np.array(bloque("RED_Q_W1")).reshape(OCULTO, DIM_ESTADO)
            b1 = np.array(bloque("RED_Q_B1"))
            w2 = np.array(bloque("RED_Q_W2")).reshape(OCULTO, OCULTO)
            b2 = np.array(bloque("RED_Q_B2"))
            w3 = np.array(bloque("RED_Q_W3")).reshape(DIM_SALIDA, OCULTO)
            b3 = np.array(bloque("RED_Q_B3"))
        except Exception:
            return False
        self.W1, self.b1, self.W2, self.b2, self.W3, self.b3 = w1, b1, w2, b2, w3, b3
        return True


# ######################################################################
# BLOCK 4 -- PATTERN MEMORY (the pattern detector proper)
# WHAT IT DOES: stores, for each LOCAL SIGNATURE, how good each RELATIVE
#   action turned out to be. The signature is invariant to position and
#   orientation:
#       (walls in the heading frame, sector where the goal lies)
#   That is why "dead end with an exit to the right" is the SAME signature
#   in the four corners of the maze and in any rotation, and what is learned
#   in one is applied in the other three without visiting them.
# COST: one dictionary lookup. At most 16 wall patterns x 8 sectors = 128
#   entries: fits easily in cache.
# IMPORTANCE: HIGH -- it is what the user asked for as "that it understands
#   the pattern". Confidence grows with visits: a pattern seen twice barely
#   votes, one seen 50 times votes with its full weight.
# ######################################################################
class MemoriaPatrones:
    def __init__(self):
        self.valores = defaultdict(lambda: np.zeros(4))   # signature -> 4 relative actions
        self.visitas = defaultdict(lambda: np.zeros(4))

    @staticmethod
    def firma(paredes: dict, heading: str, dx_meta: int, dy_meta: int) -> str:
        """paredes: {"N":bool,...}. Everything is expressed in the heading frame."""
        bits = 0
        for rel in range(4):
            if paredes[absoluta_desde_relativa(rel, heading)]:
                bits |= (1 << rel)
        # Goal sector, also relative to the heading: 8 sectors of 45 degrees.
        if dx_meta == 0 and dy_meta == 0:
            sector = 8
        else:
            ang = math.degrees(math.atan2(-dy_meta, dx_meta))      # 0 = East
            ang_heading = {"N": 90.0, "E": 0.0, "S": 270.0, "O": 180.0}[heading]
            rel_ang = (ang - ang_heading) % 360.0
            sector = int(rel_ang // 45.0)
        return f"{bits:02d}_{sector}"

    def opinar(self, firma: str) -> np.ndarray:
        """Value per relative action, attenuated by confidence n/(n+5)."""
        if firma not in self.valores:
            return np.zeros(4)
        n = self.visitas[firma]
        confianza = n / (n + 5.0)
        return self.valores[firma] * confianza

    def actualizar(self, firma: str, rel: int, objetivo: float):
        v = self.valores[firma]
        v[rel] += TASA_PATRON * (objetivo - v[rel])
        self.visitas[firma][rel] += 1

    def a_json(self):
        return {k: {"v": self.valores[k].tolist(), "n": self.visitas[k].tolist()}
                for k in self.valores}

    def desde_json(self, d):
        for k, val in d.items():
            self.valores[k] = np.array(val["v"], dtype=float)
            self.visitas[k] = np.array(val["n"], dtype=float)


# ######################################################################
# BLOCK 5 -- OBSERVATION THAT ENTERS THE AGENT
# WHAT IT DOES: data contract between the rest of the Pi and this module.
#   All the fields come from the ESP32 CSV or from the map kept by
#   maze_map.py; this module does NOT read the serial port nor keep the map.
# COST: none.
# IMPORTANCE: HIGH -- keeps the RL decoupled from the link and the map.
# ######################################################################
class Observacion:
    __slots__ = ("celda", "heading", "paredes", "distancias", "candidatas",
                 "mem", "ret", "neuro", "celda_nueva")

    def __init__(self, celda, heading, paredes, distancias=None, candidatas=None,
                 mem=0.0, ret=0.0, neuro=None, celda_nueva=False):
        self.celda = tuple(celda)              # (x, y)
        self.heading = heading or "N"          # last direction of motion
        self.paredes = dict(paredes)           # {"N":bool,"E":bool,"S":bool,"O":bool}
        # ToF distances in mm; if there are none, they are synthesized from
        # the walls so the network receives something coherent (useful when
        # testing without the robot).
        self.distancias = (dict(distancias) if distancias else
                           {d: (80.0 if paredes[d] else 900.0) for d in DIRS4})
        # Directions that Tremaux considers legal. None = all free ones.
        self.candidatas = None if candidatas is None else list(candidatas)
        self.mem = float(mem)
        self.ret = float(ret)
        # Gaussian activations of the neurocontroller (4, DIRS4 order).
        self.neuro = np.zeros(4) if neuro is None else np.asarray(neuro, dtype=float)
        self.celda_nueva = bool(celda_nueva)


# ######################################################################
# BLOCK 6 -- RL AGENT
# WHAT IT DOES: brings together the three memories that learn, the master
#   route and the exploration policy. It is the only class the rest of the
#   Pi uses.
# USAGE CYCLE (one loop = one cell):
#       direccion, comando = agente.elegir(obs)
#       ... `comando` is sent to the ESP and its event is awaited ...
#       agente.aprender(evento, obs_siguiente)
#   and at the end of each run:
#       agente.fin_corrida(exito=True/False)
# COST: ~0.2 ms per cell. Nothing compared with the CNN.
# IMPORTANCE: CRITICAL.
# ######################################################################
class AgenteRL:
    def __init__(self, lado=8, meta=(7, 7), ruta_estado="rl_state.json",
                 pesos_iniciales_h=None, semilla=0):
        self.lado = lado
        self.meta = tuple(meta)
        self.ruta_estado = ruta_estado
        self.rng = random.Random(semilla)

        self.tabla = defaultdict(lambda: np.zeros(4))   # (x,y) -> 4 absolute actions
        self.trazas = {}                                 # ((x,y),a) -> eligibility
        self.red = RedQ(semilla)
        self.patrones = MemoriaPatrones()

        self.corridas = 0
        self.exitos = 0
        self.mejor_pasos = None          # record of steps to the goal
        self.ruta_maestra = {}           # "x,y" -> direction, loop-free route
        self.modo_explotacion = False    # activated once there is a good route

        # State of the ongoing transition (between `elegir` and `aprender`).
        self._pend = None
        self.pasos_corrida = 0
        self.traza_corrida = []          # [(cell, direction)] of this run
        self.visitas_corrida = defaultdict(int)
        self.ultimo_motivo = ""          # for telemetry/debugging

        if not self.cargar() and pesos_iniciales_h:
            # No previous state: start from the weights trained in simulation.
            self.red.cargar_header_cpp(pesos_iniciales_h)

    # ------------------------------------------------------------------
    # 6.1 Features and values
    # ------------------------------------------------------------------
    def _rasgos_red(self, obs: Observacion) -> np.ndarray:
        """Same 8-vector as construirEstadoRed() in the firmware, in the
        same order -- that is why the weights in q_network_weights.h still
        work."""
        x, y = obs.celda
        e = np.empty(DIM_ESTADO)
        for i, d in enumerate(DIRS4):
            e[i] = min(max(obs.distancias[d] / 1200.0, 0.0), 1.0)
        e[4] = (self.meta[0] - x) / float(self.lado)
        e[5] = (self.meta[1] - y) / float(self.lado)
        e[6] = obs.mem
        e[7] = obs.ret
        return e

    def _momentum(self, direccion: str, heading: str) -> float:
        if not heading:
            return 0.0
        if direccion == heading:
            return PESO_MOMENTUM_SEGUIR
        if direccion == CONTRARIA[heading]:
            return PESO_MOMENTUM_ATRAS
        return PESO_MOMENTUM_LATERAL

    def beta_actual(self) -> float:
        """How much the network weighs. It rises with experience, as in the
        firmware: at first Tremaux/neuro rule, later what was learned
        rules."""
        return min(1.0, PESO_RED_BASE + PESO_RED_PASO * self.corridas)

    def temperatura(self) -> float:
        """Decreasing exploration. In exploitation mode it drops to the
        minimum: that is what keeps run 2 from wasting time trying things
        again."""
        if self.modo_explotacion:
            return TEMP_MINIMA
        return max(TEMP_MINIMA, TEMP_INICIAL - TEMP_PASO * self.corridas)

    def _candidatas(self, obs: Observacion):
        libres = [d for d in DIRS4 if not obs.paredes[d]]
        if obs.candidatas:
            cand = [d for d in obs.candidatas if not obs.paredes[d]]
            if cand:
                return cand
        return libres

    def puntajes(self, obs: Observacion):
        """Score per absolute direction = sum of the votes. The breakdown is
        returned too and goes to telemetry: without it, it is impossible to
        debug why the robot chose what it chose."""
        q_tab = self.tabla[obs.celda]
        q_red, cache = self.red.adelante(self._rasgos_red(obs))
        firma = MemoriaPatrones.firma(obs.paredes, obs.heading,
                                      self.meta[0] - obs.celda[0],
                                      self.meta[1] - obs.celda[1])
        v_pat_rel = self.patrones.opinar(firma)

        beta = self.beta_actual()
        score = np.zeros(4)
        desglose = {}
        for i, d in enumerate(DIRS4):
            rel = indice_relativo(d, obs.heading)
            partes = {
                "tabla": PESO_TABLA * q_tab[i],
                "red": beta * float(q_red[i]),
                "patron": PESO_PATRON * float(v_pat_rel[rel]),
                "momentum": self._momentum(d, obs.heading),
                "neuro": PESO_NEURO * float(obs.neuro[i]) if obs.neuro.size == 4 else 0.0,
            }
            score[i] = sum(partes.values())
            desglose[d] = partes
        return score, desglose, cache, firma

    # ------------------------------------------------------------------
    # 6.2 Decision
    # ------------------------------------------------------------------
    def elegir(self, obs: Observacion, relativo=False):
        """Returns (absolute_direction, command_for_the_ESP).
        `relativo=True` returns the 5..8 code instead of 1..4 -- useful
        when the detected figure orders a relative turn."""
        candidatas = self._candidatas(obs)
        if not candidatas:
            self.ultimo_motivo = "sin_candidatas"
            self._pend = None
            return None, "0"

        # (a) Master route: if this cell is already on the best known route
        # and that direction is still free, it is taken directly. This is the
        # shortcut that makes run 2 short. If the walls do not match
        # (different maze, or the route was bad), it is ignored and the
        # normal decision is made.
        clave = f"{obs.celda[0]},{obs.celda[1]}"
        if self.modo_explotacion and clave in self.ruta_maestra:
            d = self.ruta_maestra[clave]
            if not obs.paredes[d]:
                self.ultimo_motivo = "ruta_maestra"
                return self._comprometer(obs, d, relativo)
            del self.ruta_maestra[clave]   # the route is no longer valid here

        score, _, _, _ = self.puntajes(obs)

        # (b) Softmax restricted to the candidates, same formula as
        # elegirPorSoftmax() in the firmware.
        temp = self.temperatura()
        idx = [IDX_DIR[d] for d in candidatas]
        s = np.array([score[i] for i in idx])
        s = s - s.max()
        exps = np.exp(s / max(temp, 1e-3))
        probs = exps / exps.sum()
        elegida = candidatas[int(self.rng.choices(range(len(candidatas)), weights=probs)[0])]
        self.ultimo_motivo = f"softmax(T={temp:.2f})"
        return self._comprometer(obs, elegida, relativo)

    def forzar(self, obs: Observacion, direccion: str, relativo=False):
        """The figure seen by the camera rules: that direction is executed
        but the transition IS learned. A decision coming from outside is no
        reason not to learn from its outcome -- on the contrary, they are
        the most informative data there is."""
        self.ultimo_motivo = "figura"
        return self._comprometer(obs, direccion, relativo)

    def _comprometer(self, obs: Observacion, direccion: str, relativo: bool):
        """Stores everything needed to learn when the ESP event arrives."""
        score, _, cache, firma = self.puntajes(obs)
        self._pend = {
            "celda": obs.celda,
            "heading": obs.heading,
            "dir": direccion,
            "idx": IDX_DIR[direccion],
            "rel": indice_relativo(direccion, obs.heading),
            "cache": cache,
            "q_red": float(self.red.adelante(self._rasgos_red(obs))[0][IDX_DIR[direccion]]),
            "firma": firma,
            "dist_meta": self._dist_meta(obs.celda),
        }
        if relativo:
            return direccion, CMD_RELATIVO[indice_relativo(direccion, obs.heading)]
        return direccion, CMD_ABSOLUTO[direccion]

    def _dist_meta(self, celda):
        return abs(self.meta[0] - celda[0]) + abs(self.meta[1] - celda[1])

    # ------------------------------------------------------------------
    # 6.3 Learning -- one call per ESP event
    # ------------------------------------------------------------------
    def aprender(self, evento: int, obs_siguiente: Observacion, en_meta=False):
        """evento: the `ev` field of the CSV (0..6). Returns the reward that
        was applied, so it can be plotted.

        This is where the system "updates dynamically with every punishment
        or reward": there is no batch, no deferred episode; every event
        moves the weights immediately."""
        if self._pend is None:
            return 0.0
        p = self._pend

        # --- 1. reward ----------------------------------------------------
        r = R_PASO
        avanzo = (evento == 3)   # EV_FIN_CELDA
        if avanzo:
            self.pasos_corrida += 1
            nueva_dist = self._dist_meta(obs_siguiente.celda)
            r += R_PROGRESO * (p["dist_meta"] - nueva_dist)
            if obs_siguiente.celda_nueva:
                r += R_CELDA_NUEVA
            self.visitas_corrida[obs_siguiente.celda] += 1
            if self.visitas_corrida[obs_siguiente.celda] > 1:
                r += R_REVISITA * (self.visitas_corrida[obs_siguiente.celda] - 1)
            if p.get("heading") and p["dir"] == CONTRARIA[p["heading"]]:
                r += R_RETROCESO
            self.traza_corrida.append((p["celda"], p["dir"]))
        elif evento == 4:
            r += R_PARED
        elif evento == 2:
            r += R_RECHAZO
        elif evento == 6:
            r += R_BLOQUEO
        elif evento == 5:
            r += R_CAMPO_ABIERTO

        terminal = bool(en_meta)
        if terminal:
            r += R_META
            if self.mejor_pasos is not None and self.pasos_corrida < self.mejor_pasos:
                ahorro = (self.mejor_pasos - self.pasos_corrida) / float(self.mejor_pasos)
                r += R_PRIMA_RAPIDEZ * ahorro

        # --- 2. bootstrap: how much the reached state is worth ------------
        if terminal:
            v_tab_sig = 0.0
            v_red_sig = 0.0
        else:
            libres = [IDX_DIR[d] for d in DIRS4 if not obs_siguiente.paredes[d]]
            q_sig = self.tabla[obs_siguiente.celda]
            v_tab_sig = float(max(q_sig[i] for i in libres)) if libres else 0.0
            v_red_sig = float(self.red.predecir(self._rasgos_red(obs_siguiente)).max())

        # --- 3. Q table with eligibility traces --------------------------
        # delta is spread backwards over the WHOLE recent route. It is the
        # mechanism by which a single success reorders the entire run,
        # instead of improving only the last cell.
        q_sa = self.tabla[p["celda"]][p["idx"]]
        delta = r + GAMMA * v_tab_sig - q_sa
        self.trazas[(p["celda"], p["idx"])] = 1.0        # replacing trace
        muertas = []
        for clave, e in self.trazas.items():
            celda, a = clave
            self.tabla[celda][a] += ALPHA_TABLA * delta * e
            nueva = e * GAMMA * LAMBDA_TRAZA
            if nueva < TRAZA_MINIMA:
                muertas.append(clave)
            else:
                self.trazas[clave] = nueva
        for clave in muertas:
            del self.trazas[clave]

        # --- 4. Q network: TD(0) with its own target -----------------------
        objetivo_red = r + GAMMA * v_red_sig
        self.red.paso_sgd(p["cache"], p["idx"], p["q_red"] - objetivo_red)

        # --- 5. pattern memory -------------------------------------------
        self.patrones.actualizar(p["firma"], p["rel"], r + GAMMA * v_tab_sig)

        self._pend = None
        return r

    # ------------------------------------------------------------------
    # 6.4 End of run
    # ------------------------------------------------------------------
    def fin_corrida(self, exito: bool, guardar=True):
        """Closes the episode: clears the traces, compresses the trajectory
        into a loop-free route if it was the best so far, increases the
        counter (which lowers the temperature) and saves to disk."""
        self.corridas += 1
        if exito:
            self.exitos += 1
            ruta = self._comprimir_ruta(self.traza_corrida)
            if self.mejor_pasos is None or len(ruta) < self.mejor_pasos:
                self.mejor_pasos = len(ruta)
                self.ruta_maestra = {f"{c[0]},{c[1]}": d for c, d in ruta}
                self.modo_explotacion = True

        self.trazas.clear()
        self.traza_corrida = []
        self.visitas_corrida = defaultdict(int)
        self.pasos_corrida = 0
        self._pend = None
        if guardar:
            self.guardar()

    @staticmethod
    def _comprimir_ruta(traza):
        """Removes the loops: if a cell repeats, everything between the two
        visits is deleted. Turns a tortuous exploration into the clean route
        that will be followed next time."""
        salida = []
        indice = {}
        for celda, direccion in traza:
            if celda in indice:
                corte = indice[celda]
                for c, _ in salida[corte:]:
                    indice.pop(c, None)
                salida = salida[:corte]
            indice[celda] = len(salida)
            salida.append((celda, direccion))
        return salida

    # ------------------------------------------------------------------
    # 6.5 Persistence and telemetry
    # ------------------------------------------------------------------
    def guardar(self):
        datos = {
            "corridas": self.corridas,
            "exitos": self.exitos,
            "mejor_pasos": self.mejor_pasos,
            "ruta_maestra": self.ruta_maestra,
            "modo_explotacion": self.modo_explotacion,
            "lado": self.lado,
            "meta": list(self.meta),
            "tabla": {f"{k[0]},{k[1]}": v.tolist() for k, v in self.tabla.items()},
            "patrones": self.patrones.a_json(),
        }
        tmp = self.ruta_estado + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(datos, f)
        os.replace(tmp, self.ruta_estado)   # atomic write: a power cut does
        self.red.guardar_npz(self._ruta_pesos())   # not leave the state half-written

    def _ruta_pesos(self):
        return os.path.splitext(self.ruta_estado)[0] + "_red.npz"

    def cargar(self) -> bool:
        if not os.path.exists(self.ruta_estado):
            return False
        with open(self.ruta_estado, "r", encoding="utf-8") as f:
            d = json.load(f)
        self.corridas = d.get("corridas", 0)
        self.exitos = d.get("exitos", 0)
        self.mejor_pasos = d.get("mejor_pasos")
        self.ruta_maestra = d.get("ruta_maestra", {})
        self.modo_explotacion = d.get("modo_explotacion", False)
        for k, v in d.get("tabla", {}).items():
            x, y = k.split(",")
            self.tabla[(int(x), int(y))] = np.array(v, dtype=float)
        self.patrones.desde_json(d.get("patrones", {}))
        self.red.cargar_npz(self._ruta_pesos())
        return True

    def olvidar_laberinto(self):
        """Erases what is specific to THIS maze (table, route) and keeps the
        general knowledge (network and patterns). This is what must be
        called if the exam is run on a maze different from the practice one:
        what the robot understood about the world is reused without dragging
        along a map that no longer applies."""
        self.tabla = defaultdict(lambda: np.zeros(4))
        self.trazas.clear()
        self.ruta_maestra = {}
        self.mejor_pasos = None
        self.modo_explotacion = False

    def diagnostico(self) -> dict:
        return {
            "corridas": self.corridas,
            "exitos": self.exitos,
            "mejor_pasos": self.mejor_pasos,
            "temperatura": round(self.temperatura(), 3),
            "beta_red": round(self.beta_actual(), 3),
            "celdas_en_tabla": len(self.tabla),
            "patrones_aprendidos": len(self.patrones.valores),
            "ruta_maestra": len(self.ruta_maestra),
            "modo": "explotacion" if self.modo_explotacion else "exploracion",
            "ultimo_motivo": self.ultimo_motivo,
        }


# ######################################################################
# BLOCK 7 -- TEST BENCH (not used on the robot)
# WHAT IT DOES: perfect maze generated by DFS + a simulator that imitates
#   exactly the semantics of the ESP32 (one command = one cell, wall =
#   event 4). It is used to verify that the agent really LEARNS before
#   uploading it to the Pi, and to measure how much run 2 gets shorter.
# COST: only when this file is executed directly.
# IMPORTANCE: HIGH for the report -- the steps-per-run plot that
#   demonstrates learning comes from here.
# ######################################################################
def generar_laberinto(lado=8, semilla=0):
    """Returns paredes[(x,y)] = {"N":bool,...}. Perfect maze (no cycles)
    built by DFS with backtracking."""
    rng = random.Random(semilla)
    paredes = {(x, y): {d: True for d in DIRS4} for x in range(lado) for y in range(lado)}
    visto = {(0, 0)}
    pila = [(0, 0)]
    while pila:
        x, y = pila[-1]
        vecinos = []
        for d in DIRS4:
            nx, ny = x + DX[d], y + DY[d]
            if 0 <= nx < lado and 0 <= ny < lado and (nx, ny) not in visto:
                vecinos.append((d, nx, ny))
        if not vecinos:
            pila.pop()
            continue
        d, nx, ny = rng.choice(vecinos)
        paredes[(x, y)][d] = False
        paredes[(nx, ny)][CONTRARIA[d]] = False
        visto.add((nx, ny))
        pila.append((nx, ny))
    return paredes


def _observacion(paredes, celda, heading, nuevas):
    return Observacion(celda=celda, heading=heading, paredes=paredes[celda],
                       celda_nueva=(celda not in nuevas))


def correr_episodio(agente, paredes, lado=8, max_pasos=400):
    """Imitates the real Pi loop: choose -> command -> event -> learn."""
    celda, heading = (0, 0), "N"
    vistas = {(0, 0)}
    pasos = 0
    while pasos < max_pasos:
        obs = _observacion(paredes, celda, heading, vistas)
        direccion, _cmd = agente.elegir(obs)
        if direccion is None:
            break
        pasos += 1
        if paredes[celda][direccion]:
            evento = 4                       # EV_PARED: did not move
            siguiente = celda
        else:
            evento = 3                       # EV_FIN_CELDA
            siguiente = (celda[0] + DX[direccion], celda[1] + DY[direccion])
            heading = direccion
        obs_sig = _observacion(paredes, siguiente, heading, vistas)
        vistas.add(siguiente)
        en_meta = (siguiente == agente.meta)
        agente.aprender(evento, obs_sig, en_meta=en_meta)
        celda = siguiente
        if en_meta:
            agente.fin_corrida(exito=True, guardar=False)
            return pasos, True
    agente.fin_corrida(exito=False, guardar=False)
    return pasos, False


if __name__ == "__main__":
    LADO = 8
    laberinto = generar_laberinto(LADO, semilla=7)
    agente = AgenteRL(lado=LADO, meta=(LADO - 1, LADO - 1),
                      ruta_estado="/tmp/_rl_demo.json", semilla=3)

    print("run      steps  success mode         patterns  best")
    for i in range(12):
        pasos, exito = correr_episodio(agente, laberinto, LADO)
        d = agente.diagnostico()
        print(f"{i+1:^7} {pasos:^6} {str(exito):^6} {d['modo']:<13} "
              f"{d['patrones_aprendidos']:^8} {str(d['mejor_pasos']):^5}")

    print("\nGeneralization: same agent, NEW maze never seen before.")
    print("The memory of THIS maze is erased; the network and patterns are kept.")
    agente.olvidar_laberinto()
    otro = generar_laberinto(LADO, semilla=99)
    for i in range(3):
        pasos, exito = correr_episodio(agente, otro, LADO)
        print(f"  new maze, run {i+1}: steps={pasos} success={exito}")

    virgen = AgenteRL(lado=LADO, meta=(LADO - 1, LADO - 1),
                      ruta_estado="/tmp/_rl_virgen.json", semilla=3)
    p0, e0 = correr_episodio(virgen, otro, LADO)
    print(f"  agent WITHOUT previous experience, 1st run: steps={p0} success={e0}")


# ######################################################################
# BLOCK 8 -- TURNING ON THE SPOT  (ADDED 2026-09-04, NOT FROM ALEJO)
# ----------------------------------------------------------------------
# WHY THIS BLOCK EXISTS
#     The PRuebaRLgiroeje package (ejecutor.py, main.py) calls four things
#     that THIS copy of rl.py does not have:
#         agente.orientacion          (DIRS4 index of the current front)
#         agente.plan_giro_hacia(dir) -> (commands, pasos90)
#         agente.confirmar_giro(n)
#         diagnostico()["orientacion"] and ["giros"]
#     That is: the copy on disk is OLDER than the version Alejo used to
#     record the demo. This block rebuilds that interface from outside,
#     without editing a single line of the original file above, so that the
#     demo can run. If the good rl.py shows up, this whole block is deleted
#     and no trace is left.
#
# CONVENTION (the file's own, BLOCK 1): DIRS4 = N,E,S,O in CLOCKWISE
#     order, so +1 step of 90 degrees = turn RIGHT (clockwise) and
#     -1 = left. 2 = half turn.
# ######################################################################

R_GIRO_90 = -0.08     # turning costs real time: soft penalty
R_GIRO_180 = -0.20    # the half turn costs twice as much and it also
                      # usually means the robot took the wrong branch

_init_original = AgenteRL.__init__
_aprender_original = AgenteRL.aprender
_fin_corrida_original = AgenteRL.fin_corrida
_olvidar_original = AgenteRL.olvidar_laberinto
_diagnostico_original = AgenteRL.diagnostico


def _init_con_giro(self, *args, **kwargs):
    self.orientacion = 0        # 0 = "N": every run starts facing North
    self.giros = 0              # how many 90-degree turns so far (telemetry)
    self._costo_giro = 0.0      # pending penalty to apply
    _init_original(self, *args, **kwargs)


def _plan_giro_hacia(self, direccion_absoluta):
    """How many 90-degree steps are needed to go from `self.orientacion`
    to `direccion_absoluta`, and the equivalent relative ESP commands.
    Returns (commands, pasos90) with pasos90 in {-1, 0, 1, 2}."""
    d = (IDX_DIR[direccion_absoluta] - self.orientacion) % 4
    pasos90 = -1 if d == 3 else d
    if pasos90 == 0:
        comandos = []
    elif pasos90 == 1:
        comandos = [CMD_GIRO_DER]
    elif pasos90 == -1:
        comandos = [CMD_GIRO_IZQ]
    else:
        # 🔴 ONE SINGLE COMMAND, NOT TWO 90-DEGREE TURNS. Whoever executes
        # this calls confirmar_giro(pasos90) ONCE PER COMMAND (see
        # navigator.girar_hacia): with two commands and pasos90=2 the model
        # would add four quarter turns and the orientation would end up
        # rotated 360 degrees with respect to the chassis. Besides, the ESP
        # closes the half turn with the IMU in one go, which is more precise
        # than chaining two.
        comandos = [CMD_GIRO_180]
    return comandos, pasos90


def _confirmar_giro(self, pasos90):
    """Called by whoever EXECUTED the turn, with the 90-degree steps that
    actually happened (not the planned ones). Updates the front and records
    the cost so that the next `aprender` charges it."""
    n = abs(int(pasos90))
    self.orientacion = (self.orientacion + int(pasos90)) % 4
    self.giros += n
    if n == 1:
        self._costo_giro += R_GIRO_90
    elif n >= 2:
        self._costo_giro += R_GIRO_180


def _aprender_con_giro(self, evento, obs_siguiente, en_meta=False):
    """The turn cost has to enter the SAME r that feeds the TD error (if it
    were only subtracted from the returned value, the agent would never
    learn it). `aprender` starts with `r = R_PASO` reading the module
    global, so the cost is added there for the duration of the call and
    restored afterwards."""
    global R_PASO
    if self._pend is None:
        return _aprender_original(self, evento, obs_siguiente, en_meta=en_meta)
    base = R_PASO
    try:
        R_PASO = base + self._costo_giro
        return _aprender_original(self, evento, obs_siguiente, en_meta=en_meta)
    finally:
        R_PASO = base
        self._costo_giro = 0.0


def _fin_corrida_con_giro(self, exito, guardar=True):
    salida = _fin_corrida_original(self, exito, guardar=guardar)
    self.orientacion = 0        # the new run spawns facing North again
    self.giros = 0
    self._costo_giro = 0.0
    return salida


def _olvidar_con_giro(self):
    salida = _olvidar_original(self)
    self.orientacion = 0
    self.giros = 0
    self._costo_giro = 0.0
    return salida


def _diagnostico_con_giro(self):
    d = _diagnostico_original(self)
    d["orientacion"] = DIRS4[self.orientacion]
    d["giros"] = self.giros
    return d


def _lecturas_a_absoluto(self, lecturas):
    """Converts a dict of readings from the CHASSIS FRAME to the ABSOLUTE
    frame.

    The ESP does not know where it is facing: in its CSV, "N" is always the
    front of the robot, "E" its right, and so on. The one that knows the
    orientation is this agent, and that is why the conversion lives here and
    not in serial_link.py, which is a translator of the cable and must not
    know the state of the agent.

    With the robot facing East, what the ESP calls "N" (its front) is the
    absolute East; what it calls "E" (its right) is the South. In general,
    the relative reading `rel` falls on absoluta_desde_relativa(rel, front).

    Works for walls (bool), distances (mm) and gaussian activations: it
    only relabels the keys, it does not touch the values. Returns a new
    dict.
    """
    frente = DIRS4[self.orientacion]
    return {absoluta_desde_relativa(rel, frente): lecturas[DIRS4[rel]]
            for rel in range(4)}


AgenteRL.__init__ = _init_con_giro
AgenteRL.plan_giro_hacia = _plan_giro_hacia
AgenteRL.confirmar_giro = _confirmar_giro
AgenteRL.lecturas_a_absoluto = _lecturas_a_absoluto
AgenteRL.aprender = _aprender_con_giro
AgenteRL.fin_corrida = _fin_corrida_con_giro
AgenteRL.olvidar_laberinto = _olvidar_con_giro
AgenteRL.diagnostico = _diagnostico_con_giro
