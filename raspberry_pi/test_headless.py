#!/usr/bin/env python3
"""Headless test of the WHOLE Pi stack, without robot and without cable.

    python3 test_headless.py
    python3 test_headless.py --mazes 6 --runs 3

WHAT IT TESTS
    1. That the serial_link.py parser and the firmware CSV match field by
       field (the fake ESP builds the line with the SAME format as BLOCK 17
       and it is parsed with the real parser: if someone adds a field on one
       side and not on the other, this blows up here and not in the lab).
    2. That the frames do not get mixed up: the fake ESP ALWAYS publishes in
       the chassis frame and only obeys turns; if the navigator converts
       wrongly, the robot crashes into walls the map says do not exist.
    3. That the prohibition of short loops (maze_map.candidatas) does its
       job: it counts how many runs end because of MAX_PASOS instead of
       reaching the exit.

WHAT IT DOES **NOT** TEST
    Anything about the physics: here turning and moving forward always work.
    For that there is the MuJoCo demo (Robot/PRuebaRLgiroeje) and the field
    tests.

This is the headless version this project requires before touching rewards
or believing that something works (see the rules at the end of
PROMPT_portar_fix_bucles_y_castigos.md).
"""

import argparse
import os
import sys

_AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _AQUI)

import rl                                    # noqa: E402

import config                                # noqa: E402
import serial_link as en                     # noqa: E402
import maze_map as mapa_mod                  # noqa: E402
import navigator as nav_mod                  # noqa: E402
import perception as percepcion             # noqa: E402


class ESPFalsa:
    """Imitates the firmware in firmware/ just enough to exercise the Pi.

    It implements the same contract: primitives per character, telemetry in
    the CHASSIS frame, one event per action. It does not simulate physics:
    turns always close with event 7 and forward moves with 3 or 2.

    Sep 15: the maze has an EXIT. A border wall is opened in a random cell
    and everything outside the board is open field: the four ToF read far
    away. Also, inside, a free direction reads far only if the next TWO
    cells are free too (like the real sensor, which two cells away already
    returns its cap): this way long crossings produce the same false "exit"
    alarm as in the lab and the navigator's confirmation is really
    exercised. The manual push ('#t=...') moves one cell forward if there is
    no wall.
    """

    LEJOS = 900.0     # above config.UMBRAL_SALIDA_MM
    CERCA = 80.0      # wall right there
    MEDIO = 300.0     # free, but the wall of the next cell is visible

    def __init__(self, paredes, lado, meta, semilla=0):
        self.paredes = paredes          # ABSOLUTE frame, like the generator
        self.lado = lado
        self.meta = meta
        # The exit: a border cell, chosen at random, with the outer wall
        # open. It can be on any of the four sides.
        import random
        rnd = random.Random(semilla)
        lado_salida = rnd.choice(rl.DIRS4)
        k = rnd.randrange(lado)
        self.salida = {"N": ((k, 0), "N"), "S": ((k, lado - 1), "S"),
                       "O": ((0, k), "O"), "E": ((lado - 1, k), "E")}[lado_salida]
        celda, d = self.salida
        self.paredes[celda][d] = False
        self.reiniciar()

    def reiniciar(self):
        self.celda = (0, 0)
        self.orientacion = 0            # 0=N, 1=E, 2=S, 3=O(W) (faces North)
        self.pendientes = []            # queue of events to deliver
        self.t_ms = 0
        self.avances = 0

    # -- the fake world -----------------------------------------------------
    def dentro(self, c):
        return 0 <= c[0] < self.lado and 0 <= c[1] < self.lado

    def hay_pared(self, celda, d):
        """Outside the board there are no walls anywhere."""
        if not self.dentro(celda):
            return False
        return bool(self.paredes[celda][d])

    def distancia(self, celda, d):
        if self.hay_pared(celda, d):
            return self.CERCA
        sig = (celda[0] + rl.DX[d], celda[1] + rl.DY[d])
        if not self.hay_pared(sig, d):
            return self.LEJOS         # two free cells: the sensor sees nothing
        return self.MEDIO

    def _mover_frente(self):
        frente = rl.DIRS4[self.orientacion]
        self.celda = (self.celda[0] + rl.DX[frente],
                      self.celda[1] + rl.DY[frente])

    # -- what the navigator uses from the real link ------------------------
    def enviar(self, comando):
        # 'Z' (set heading) is a bench command: the real ESP does not answer
        # with an event. preparar_corrida() sends it since Sep 12.
        if comando in (en.CMD_CSV_YA, en.CMD_FIJAR_RUMBO):
            return
        if comando.startswith("#t="):
            # Manual push: moves one cell if there is no wall. No event, like
            # the real ESP (it answers with the text OK:tirada).
            frente = rl.DIRS4[self.orientacion]
            if not self.hay_pared(self.celda, frente):
                self._mover_frente()
            return
        if comando in (en.CMD_PARAR, en.CMD_HALT):
            self.pendientes.append(en.EV_ACK)
            return
        if comando in (en.CMD_ALINEAR, en.CMD_CENTRAR):
            self.pendientes.append(en.EV_FIN_GIRO)
            return
        if comando == en.CMD_REVERSA:
            # Reverse one cell: opposite to the front, if it is free.
            atras = rl.CONTRARIA[rl.DIRS4[self.orientacion]]
            if self.hay_pared(self.celda, atras):
                self.pendientes.append(en.EV_NACK)
                return
            self.pendientes.append(en.EV_ACK)
            self.celda = (self.celda[0] + rl.DX[atras], self.celda[1] + rl.DY[atras])
            self.pendientes.append(en.EV_FIN_CELDA)
            return
        if comando == en.CMD_HASTA_PARED:
            # Move up to the wall: here, one cell, and it closes with PARED
            # like the firmware (the navigator translates it to end of cell).
            frente = rl.DIRS4[self.orientacion]
            if self.hay_pared(self.celda, frente):
                self.pendientes.append(en.EV_NACK)
                return
            self.pendientes.append(en.EV_ACK)
            self._mover_frente()
            self.pendientes.append(en.EV_PARED)
            return
        if comando == en.CMD_GIRO_DER:
            self.orientacion = (self.orientacion + 1) % 4
            self.pendientes.append(en.EV_FIN_GIRO)
            return
        if comando == en.CMD_GIRO_IZQ:
            self.orientacion = (self.orientacion - 1) % 4
            self.pendientes.append(en.EV_FIN_GIRO)
            return
        if comando == en.CMD_GIRO_180:
            self.orientacion = (self.orientacion + 2) % 4
            self.pendientes.append(en.EV_FIN_GIRO)
            return
        if comando == en.CMD_AVANZAR:
            frente = rl.DIRS4[self.orientacion]
            if self.hay_pared(self.celda, frente):
                self.pendientes.append(en.EV_NACK)   # the ESP does not even try
                return
            self.avances += 1
            self.pendientes.append(en.EV_ACK)
            self._mover_frente()
            self.pendientes.append(en.EV_FIN_CELDA)
            return
        raise AssertionError("the fake ESP does not know the command " + repr(comando))

    def _campos(self, evento):
        """Builds the fields EXACTLY as BLOCK 22 of the firmware names them,
        and passes them through the real parser (en.Telemetria). If someone
        adds a field on one side and not on the other, this blows up here
        and not in the lab."""
        self.t_ms += 100
        # From absolute to chassis: the "N" sensor faces wherever the robot faces.
        v = {"t_ms": self.t_ms, "ev": evento, "imu_ok": 1, "hab": 1}
        for d in rl.DIRS4:
            d_abs = rl.rotar(d, self.orientacion)
            v["p" + d] = 1 if self.hay_pared(self.celda, d_abs) else 0
            v["d" + d] = self.distancia(self.celda, d_abs)
        for i in range(4):
            v["nr%d" % i] = 0.5
            v["mr%d" % i] = 0.5
        for n in en.RUEDAS:
            v["rpm_" + n] = 0.0
            v["obj_" + n] = 0.0
            v["pwm_" + n] = 0.0
            v["desp_" + n] = 90.0
            v["agc_" + n] = 128.0
        for k in ("ret", "mem", "roll", "pitch", "yaw", "yaw_ref", "corr", "cuad", "ecen",
                  "cm_tramo", "celdas", "estado", "cmd", "err_giro",
                  "vivas", "cortadas", "ciclo_ms"):
            v.setdefault(k, 0.0)
        v["vivas"] = 15
        faltan = set(en.CAMPOS_POR_DEFECTO) - set(v)
        assert not faltan, "the fake ESP does not build the fields {}".format(faltan)
        return v

    def leer_telemetria(self, timeout=None):
        ev = self.pendientes.pop(0) if self.pendientes else en.EV_NADA
        return en.Telemetria(self._campos(ev))

    def esperar_evento(self, eventos, timeout=None):
        for _ in range(50):
            t = self.leer_telemetria()
            if t.evento in eventos:
                return t.evento, t
        raise en.ErrorEnlace("the fake ESP did not deliver {}".format(eventos))

    def esperar_ack(self, timeout=2.0):
        return self.esperar_evento((en.EV_ACK, en.EV_NACK, en.EV_DESALINEADO))

    def purgar(self):
        self.pendientes = []


def una_corrida(nav, esp, max_pasos):
    esp.reiniciar()
    nav.preparar_corrida()
    for paso in range(1, max_pasos + 1):
        res = nav.un_paso()
        if res.direccion is None:
            return False, paso, "sin_candidatas"
        if res.en_meta:
            # What counts is the physical EXIT: the navigator marks it in the
            # reason, and the fake ESP must have the robot OUTSIDE.
            assert "salida" in (res.motivo or ""), res.motivo
            assert not esp.dentro(esp.celda), \
                "exit declared with the robot inside, at {}".format(esp.celda)
            return True, paso, "salida"
        # And the other way round: if the robot ended up outside, the Pi had to say so.
        assert esp.dentro(esp.celda), \
            "the robot is outside at {} and the Pi did not declare the exit".format(esp.celda)
    return False, max_pasos, "max_pasos"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mazes", "--laberintos", dest="laberintos", type=int, default=6)
    p.add_argument("--runs", "--corridas", dest="corridas", type=int, default=3)
    p.add_argument("--size", "--lado", dest="lado", type=int, default=8)
    p.add_argument("--max-steps", "--max-pasos", dest="max_pasos", type=int, default=400)
    args = p.parse_args()

    # The run ends at the physical EXIT, not at the goal: as in config.py,
    # the goal remains only as a network feature.
    config.FIN_POR_META = False
    config.SALIDA_TIRADA_ESPERA_S = 0.0     # the fake ESP has no ramp
    config.SALIDA_TIRADA_MS = 0
    meta = (args.lado - 1, args.lado - 1)
    # The Pi map is larger than the maze and the robot starts in the
    # center, as in config.py: the exit can be on any side and the border of
    # the Pi board must not get in the way.
    margen = 4
    lado_pi = args.lado + 2 * margen
    inicio_pi = (margen, margen)
    total = exitos = 0
    cortes = 0
    pasos_meta = []

    for semilla in range(args.laberintos):
        paredes = rl.generar_laberinto(lado=args.lado, semilla=semilla)
        esp = ESPFalsa(paredes, args.lado, meta, semilla=semilla)
        print("  maze {}: exit at cell {} through {}".format(
            semilla, esp.salida[0], esp.salida[1]))
        # Fresh agent per maze: the stack is tested, not the accumulated
        # memory. ruta_estado in a temporary file that is not kept.
        agente = rl.AgenteRL(lado=lado_pi, meta=(lado_pi - 1, lado_pi - 1),
                             ruta_estado=os.path.join(_AQUI,
                                                      "_estado_prueba.json"),
                             semilla=semilla)
        el_mapa = mapa_mod.Mapa(lado_pi, inicio_pi, 6)
        nav = nav_mod.Navegador(agente, el_mapa, esp,
                                camara=percepcion.Camara(activa=False),
                                meta=(lado_pi - 1, lado_pi - 1), verboso=False)

        for c in range(args.corridas):
            ok, pasos, motivo = una_corrida(nav, esp, args.max_pasos)
            total += 1
            if ok:
                exitos += 1
                pasos_meta.append(pasos)
            if motivo == "max_pasos":
                cortes += 1
            agente.fin_corrida(exito=ok, guardar=False)
            print("  maze {} run {}: {:>4} steps  {}".format(
                semilla, c + 1, pasos, motivo))

    print("\n{}/{} runs EXITED the maze ({} cut by max_pasos)"
          .format(exitos, total, cortes))
    if pasos_meta:
        print("steps to the exit: min={} mean={:.1f} max={}".format(
            min(pasos_meta), sum(pasos_meta) / len(pasos_meta), max(pasos_meta)))
    tmp = os.path.join(_AQUI, "_estado_prueba.json")
    for sufijo in ("", "_red.npz"):
        ruta = tmp.replace(".json", sufijo + ".json") if sufijo == "" else \
            tmp.replace(".json", sufijo)
        if os.path.exists(ruta):
            os.remove(ruta)
    return 0 if exitos == total else 1


if __name__ == "__main__":
    sys.exit(main())
