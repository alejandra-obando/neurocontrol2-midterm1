"""State machine of the Pi: one decision = turn + forward move + learning.

THIS IS THE FILE TO READ TO UNDERSTAND THE ROBOT.

One complete step, which is what `un_paso()` does:

    OBSERVE   request a fresh CSV -> convert to the absolute frame -> record
              walls -> (camera: hook) -> build the Observacion with
              `candidatas`
    DECIDE    agente.elegir(obs), or agente.forzar(...) if a figure rules
    TURN      plan_giro_hacia -> send 'R'/'L'/'T' -> wait for event 7
              -> confirmar_giro (ONLY if the ESP confirmed)
    ADVANCE   send '5' -> wait for event 3/4/2/5/6. If the direction is
              ordered by a FIGURE and the ESP rejects the '5', it insists
              with 'A' (up to the wall): the figure leaves no other option.
              (There is no stopped alignment after the turn -- Sep 15
              afternoon, requested on the floor: the turn aims at the
              absolute grid and whatever is left over is compensated WHILE
              ROLLING, as in the square circuits.)
    LEARN     move the position if there was FIN_CELDA, agente.aprender(evento)
    EXIT      if the ESP said campo_abierto, or the four ToF read far away
              with the robot stopped, it is CONFIRMED (manual push + look
              again) and the run ends as a success. See config.SALIDA_*.

SPLIT OF RESPONSIBILITIES (do not break it):
    rl.py            the policy and the three memories. Source of truth, it
                     is IMPORTED, never copied.
    maze_map.py      where the robot is and which walls it knows.
    serial_link.py   what the cable looks like.
    here             the order of things.

WHAT THE ESP DOES **NOT** KNOW: where it is facing. The orientation lives in
AgenteRL.orientacion and moves ONLY with confirmar_giro(), and only when the
ESP confirmed the turn with event 7. If the turn fails, the model keeps
believing the right thing instead of being 90 degrees off -- which is the
kind of bug that looks like a learning error and is not.
"""

import time

import rl

import config
import serial_link as en
import perception as percepcion


class ResultadoPaso:
    """Telemetry of ONE decision, for the log and for the console."""

    __slots__ = ("direccion", "evento", "en_meta", "recompensa", "celda",
                 "celda_previa", "giro90", "motivo", "figura", "yaw",
                 "err_giro")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))


class Navegador:
    def __init__(self, agente, mapa, link, camara=None, meta=(7, 7),
                 verboso=True):
        self.agente = agente
        self.mapa = mapa
        self.link = link
        self.camara = camara or percepcion.Camara(activa=False)
        self.meta = tuple(meta)
        self.verboso = verboso
        # Last direction of MOTION (not of gaze). It is the `heading` that
        # rl.py uses for momentum and for the figures; it starts at "N"
        # because the robot is placed facing North at the start of each run.
        self.heading = "N"
        self.pasos = 0
        # 🔴 FIGURE SPOTTED FROM AFAR (Sep 15 afternoon). The camera sees the
        # figure from the end of the corridor, with ~1% confidence, and the
        # three gates never let it decide. Now: as soon as it is SPOTTED
        # stably, it is stored here, the robot goes straight towards it
        # (cell by cell, without asking the RL) and when it reaches its node
        # -- wall in front -- it obeys what the figure asks. It is cleared
        # when obeyed and at the start of each run.
        self.figura_pendiente = None
        self.figura_vista = None
        self.figura_guardada = None
        self.solo_atras = False
        # Absolute direction through which the current cell was ENTERED (the
        # opposite of the last forward move), or None at startup: there can
        # be no wall there even if the rear sensor says otherwise.
        self.entro_por = None
        # 🔴 REJECTIONS IN THIS CELL (Sep 16): directions in which the ESP
        # already said NACK or braked for a wall WITHOUT the robot changing
        # cell. They are removed from the menu until it moves: on the morning
        # of Sep 16 it sent '5' towards O at (6,8) 21 times in a row with a
        # nack every time.
        self.rechazadas_aqui = set()

    # ==================================================================
    # OBSERVE
    # ==================================================================
    def _telemetria_fresca(self):
        """Requests an immediate CSV instead of waiting for the periodic one.

        It matters: between the ESP braking and the Pi deciding, the 100 ms
        of the period can pass, and deciding with the distances from BEFORE
        braking is deciding with the robot somewhere else.
        """
        self.link.enviar(en.CMD_CSV_YA)
        return self.link.leer_telemetria()

    def observar(self):
        """Returns (Observacion, telemetry, figure).

        Everything that comes through the cable is in the chassis frame and
        is converted to absolute HERE, only once per decision (see
        rl.lecturas_a_absoluto: anything that enters without going through
        it makes the agent learn a false world).
        """
        telem = self._telemetria_fresca()

        paredes = self.agente.lecturas_a_absoluto(telem.paredes)
        distancias = self.agente.lecturas_a_absoluto(telem.distancias)
        gauss_chasis = {d: telem.gauss[i] for i, d in enumerate(rl.DIRS4)}
        # 🔴 THE FRONT NEURON INHIBITS THE REAR ONE (Sep 16): the gaussian
        # grows with the free distance, and in a long corridor the rear one
        # exceeds the front one as soon as the robot has advanced a few
        # cells; without this, when the rear cell became a candidate again
        # (second pass), "back" won and the robot turned back for no reason.
        # In the chassis frame "N" is the front and "S" the back.
        gauss_chasis["S"] = max(0.0, gauss_chasis["S"] - gauss_chasis["N"])
        gauss = self.agente.lecturas_a_absoluto(gauss_chasis)

        # WATCH THE ORDER: anotar_paredes() marks the cell as seen, so the
        # curiosity (celda_nueva) must be measured BEFORE. With the order
        # reversed, celda_nueva is always False and R_CELDA_NUEVA is never
        # paid.
        celda_nueva = self.mapa.celda not in self.mapa.vistas
        # 🔴 THERE IS NO WALL WHERE IT JUST CAME IN (Sep 15 night, measured at
        # (13,7)): the rear sensor marked a wall behind, the cell ended up
        # walled on all four sides and the robot stood still in a dead end
        # whose only exit was going back. If the cell was reached moving
        # forward, the opposite of the heading is free by definition.
        if self.entro_por is not None:
            paredes = dict(paredes)
            paredes[self.entro_por] = False
        if self.rechazadas_aqui:
            paredes = dict(paredes)
            for d in self.rechazadas_aqui:
                paredes[d] = True
        self.mapa.anotar_paredes(paredes)
        atras = rl.CONTRARIA.get(self.heading)
        # 🔴 FOUR WALLS IS IMPOSSIBLE (Sep 16, runs 3-6 at 09:57): the robot
        # is inside, so it entered from somewhere. The rear ToF reads too
        # little (<100 mm 72% of the time) and in the exit cell, or after
        # reversing into a cell with no known entrance, it walled off the
        # only exit and the run was aborted "without candidates". If nothing
        # is left free, the back is free (unless the ESP already refused the
        # reverse).
        if not self.mapa.libres() and atras and atras not in self.rechazadas_aqui:
            self.mapa.paredes[self.mapa.celda][atras] = False
            print("  [cell] {} walled on all 4 sides: the back ({}) is taken as free".format(
                self.mapa.celda, atras))
        # 🔴 TURNING BACK, ALMOST IMPOSSIBLE (Sep 15 night): the opposite of
        # the heading only enters the menu if it is the ONLY candidate. On top
        # of that rl.py punishes it with R_RETROCESO and PESO_MOMENTUM_ATRAS
        # (-3 both).
        candidatas = self.mapa.candidatas()
        # 🔴 THE FREE FRONT IS ALWAYS A CANDIDATE (Sep 16): the recent-history
        # prohibition discarded the cell ahead (the robot had just been there
        # because of a back-and-forth) and left the back as the only option,
        # with the wall at 66 mm. No memory rule can prohibit what the ToF see
        # free ahead; the anti-loop only removes sides and back.
        frente = self.heading
        if frente in self.mapa.libres() and frente not in candidatas:
            candidatas = [frente] + [d for d in candidatas if d != frente]
        if atras in candidatas and len(candidatas) > 1:
            candidatas = [d for d in candidatas if d != atras]
        # 🔑 THE DEAD-END EXCEPTION: if no candidate is left and the only free
        # direction is the one it came in through, that is the decision. The
        # rl.py punishment stays as it is; here we only guarantee that the
        # option exists.
        self.solo_atras = False
        if not candidatas:
            libres = self.mapa.libres()
            if libres and (libres == [self.entro_por] or libres == [atras]):
                candidatas = list(libres)
                self.solo_atras = True     # un_paso checks it by moving closer

        # Camera: it is only looked at if the front ToF says the wall is
        # right there (see perception.DIST_DECISION_MM). It is read with the
        # robot STOPPED, which is exactly where it is when it gets here. With
        # the camera off this returns None and the RL rules.
        figura = self.camara.leer(dist_frontal_mm=telem.distancias.get("N"))
        # And the reading without gates: only "a stable figure is seen". It is
        # the alarm that arms figura_pendiente (see un_paso).
        vista = getattr(self.camara, "vista", None)
        self.figura_vista = vista() if vista else None

        obs = rl.Observacion(
            celda=self.mapa.celda,
            heading=self.heading,
            paredes=self.mapa.paredes_actuales(),
            distancias=distancias,
            candidatas=candidatas,
            mem=telem.mem,
            ret=telem.ret,
            neuro=[gauss[d] for d in rl.DIRS4],
            celda_nueva=celda_nueva,
        )
        return obs, telem, figura

    # ==================================================================
    # DECIDE (RL, or figure if the camera sees one)
    # ==================================================================
    # Callback for the interface: called after EACH decision with
    # (direction, reason, neuro). main.py points it to tablero.publicar_neuro.
    on_decision = None

    def decidir(self, obs, figura):
        """Returns (absolute_direction, reason) and publishes the neural
        snapshot of the decision (see _instantanea_neuro). The snapshot can
        never bring down navigation: it runs in its own try."""
        direccion, motivo = self._decidir(obs, figura)
        if self.on_decision is not None:
            try:
                self.on_decision(direccion, motivo, self._instantanea_neuro(obs, direccion, motivo))
            except Exception as e:      # the interface never brings the robot down
                print("  [interface] neuro snapshot failed: {}".format(e))
        return direccion, motivo

    def _instantanea_neuro(self, obs, direccion, motivo):
        """What the page draws in the Pi-side 'Neural system' block: input
        features, h1/h2 activations and q output of the MLP, the breakdown
        of votes per direction (table, network, pattern, momentum, neuro),
        the score and the softmax probabilities over the candidates. It is a
        RE-READING of what the agent already computed (puntajes() is pure);
        it costs two 32x32 matmuls, nothing."""
        import math
        ag = self.agente
        score, desglose, cache, _firma = ag.puntajes(obs)
        x, h1, h2 = cache
        q, _ = ag.red.adelante(x)
        temp = ag.temperatura()
        cand = list(obs.candidatas) if obs.candidatas else [
            d for d in rl.DIRS4 if not obs.paredes[d]]
        probs = {d: 0.0 for d in rl.DIRS4}
        if cand:
            vals = [float(score[rl.IDX_DIR[d]]) for d in cand]
            m = max(vals)
            ex = [math.exp((v - m) / max(temp, 1e-3)) for v in vals]
            tot = sum(ex) or 1.0
            for d, e in zip(cand, ex):
                probs[d] = e / tot
        return {
            "rasgos": [float(v) for v in x],
            "h1": [float(v) for v in h1], "h2": [float(v) for v in h2],
            "q": [float(v) for v in q],
            "desglose": {d: {k: float(v) for k, v in partes.items()}
                         for d, partes in desglose.items()},
            "score": [float(v) for v in score],
            "probs": probs, "candidatas": cand, "elegida": direccion,
            "temperatura": float(temp), "beta": float(ag.beta_actual()),
            "motivo": motivo or "", "heading": self.heading,
            "tabla": [float(v) for v in ag.tabla[obs.celda]] if hasattr(ag, "tabla") else [],
        }

    def _decidir(self, obs, figura):
        """Returns (absolute_direction, reason).

        With a figure, the camera OVERRIDES the RL: that is the project
        statement, not a heuristic. The agent still learns from the
        transition, because forzar() leaves the same pending state as
        elegir() -- the only difference is who chose.
        """
        rel = percepcion.REL_DE_FIGURA.get(figura) if figura else None

        if figura == percepcion.FIGURA_CIRCULO:
            # Turning back is punished by default (R_RETROCESO and
            # PENALIZA_RETROCESO_SCORE). The circle is the only order that
            # lifts that punishment, and only for the next decision.
            # (the slim rl.py on the Pi does not have it: the circle is
            # executed as a one-cell REVERSE in un_paso, like the dead end;
            # Sep 16)
            if hasattr(self.agente, "autorizar_retroceso"):
                self.agente.autorizar_retroceso()

        if rel is not None:
            direccion = rl.absoluta_desde_relativa(rel, self.heading)
            self.agente.forzar(obs, direccion)
            return direccion, "figura:{}".format(figura)

        # 🔑 THE "SHORTCUT" (Sep 15 night): in a cell WITHOUT experience (its
        # Q table row is still at zero) there is no softmax draw: the robot
        # goes in the direction of HIGHEST NEURAL ACTIVATION among the
        # candidates -- the gaussian layer of the neurocontroller, which rises
        # the more open that side is. It comes "learned" from the factory and
        # saves the steps the RL used to spend figuring out the corner. It is
        # forced like a figure, so the agent learns the transition and on the
        # next visit already decides with its table.
        if config.DECIDIR_POR_NEURO and obs.candidatas:
            fila = self.agente.tabla.get(obs.celda) if hasattr(self.agente, "tabla") else None
            sin_experiencia = fila is None or not any(abs(float(v)) > 1e-9 for v in fila)
            if sin_experiencia and len(obs.neuro) == 4:
                # back only if it is the only candidate: the neural decision
                # is forwards or sideways
                atras = rl.CONTRARIA.get(self.heading)
                cands = [d for d in obs.candidatas if d != atras] or list(obs.candidatas)
                mejor = max(cands, key=lambda d: float(obs.neuro[rl.IDX_DIR[d]]))
                self.agente.forzar(obs, mejor)
                return mejor, "neuro_max({})".format(
                    ",".join("{}={:.2f}".format(d, float(obs.neuro[rl.IDX_DIR[d]]))
                             for d in obs.candidatas))

        direccion, _cmd = self.agente.elegir(obs)
        return direccion, self.agente.ultimo_motivo

    # ==================================================================
    # TURN
    # ==================================================================
    def girar_hacia(self, direccion_absoluta):
        """Leaves the chassis facing `direccion_absoluta`. (pasos90, ok).

        ok=False means the ESP could not close the turn (event 8): it did
        not converge, or it lost the IMU. The caller treats it as a STALL
        and does NOT move forward -- moving forward with an uncertain
        orientation is exactly how the robot ended up stuck at a weird angle
        on 2026-09-03.
        """
        cmds, pasos90 = self.agente.plan_giro_hacia(direccion_absoluta)
        if not cmds:
            return 0, True
        if getattr(self.link, "paro_pedido", False):
            return 0, False

        for cmd in cmds:
            self.link.enviar(cmd)
            # Turns do NOT send an ACK: they are blocking on the ESP and only
            # emit the closing event. During that radio silence not a single
            # CSV line arrives, which is why the timeout is so generous.
            try:
                evento, _telem = self.link.esperar_evento(
                    en.EVENTOS_FIN_GIRO, timeout=self.link_timeout_giro)
            except en.ErrorEnlace as e:
                # Same criterion as in avanzar(): a turn that does not close
                # is a stall of this step, not the end of the program.
                self.link.enviar(en.CMD_PARAR)
                self.link.avisos.append("turn without closing: {}".format(e))
                print("  [turn] {} -> treated as a STALL".format(e))
                return 0, False
            if evento != en.EV_FIN_GIRO:
                return 0, False
            # The model orientation moves AFTER the confirmation, never
            # before.
            self.agente.confirmar_giro(pasos90)
        return pasos90, True

    # ==================================================================
    # ALIGN with the heading grid
    # ==================================================================
    def alinear(self):
        """Leaves the chassis on its corresponding multiple of 90. True if
        the ESP confirmed it (event 7). The ESP does not move anything if it
        is already within tolerance, so calling it always is cheap."""
        self.link.enviar(en.CMD_ALINEAR)
        try:
            evento, _telem = self.link.esperar_evento(
                en.EVENTOS_FIN_GIRO, timeout=self.link_timeout_giro)
        except en.ErrorEnlace as e:
            self.link.enviar(en.CMD_PARAR)
            self.link.avisos.append("alignment without closing: {}".format(e))
            print("  [align] {} -> treated as a STALL".format(e))
            return False
        return evento == en.EV_FIN_GIRO

    def acomodar(self):
        """BEFORE EACH DECISION (Sep 16): command '9' = the ESP aligns the
        heading to its corresponding multiple of 90 (like 'Y') and then
        sticks to the side wall while stopped (pegarseALaPared, the one from
        square_right), all blocking. Nothing else is decided or sent until
        it closes. True if the ESP confirmed it (event 7)."""
        if getattr(self.link, "paro_pedido", False):
            return False
        self.link.enviar(en.CMD_CENTRAR)
        try:
            evento, _telem = self.link.esperar_evento(
                en.EVENTOS_FIN_GIRO, timeout=self.link_timeout_giro + 10.0)
        except en.ErrorEnlace as e:
            self.link.enviar(en.CMD_PARAR)
            self.link.avisos.append("settling without closing: {}".format(e))
            print("  [settle] {} -> continuing without settling".format(e))
            return False
        return evento == en.EV_FIN_GIRO

    # ==================================================================
    # REVERSE one cell (command 'D'): the dead end
    # ==================================================================
    def retroceder(self):
        """Backs up one cell without turning. Returns the closing event
        (FIN_CELDA / PARED / NACK / BLOQUEO), like avanzar()."""
        self.link.enviar(en.CMD_REVERSA)
        try:
            evento, _ = self.link.esperar_ack()
            if evento in en.EVENTOS_FIN_AVANCE:
                return evento               # closed immediately
            if evento != en.EV_ACK:
                return en.EV_NACK
            evento, _ = self.link.esperar_evento(
                en.EVENTOS_FIN_AVANCE, timeout=self.link_timeout_evento)
            return evento
        except en.ErrorEnlace as e:
            self.link.enviar(en.CMD_PARAR)
            self.link.avisos.append("reverse without closing: {}".format(e))
            print("  [reverse] {} -> treated as a STALL".format(e))
            return en.EV_BLOQUEO

    # ==================================================================
    # MOVE CLOSER to the wall in front (command 'A')
    # ==================================================================
    def arrimarse(self):
        """Moves forward up to DIST_STOP_N with 'A'. It does not touch the
        map: it is an adjustment within the same cell. If the ESP rejects it
        (it is already close) or it does not close, it carries on anyway:
        what matters is turning from the crossing."""
        self.link.enviar(en.CMD_HASTA_PARED)
        try:
            evento, _ = self.link.esperar_ack()
            if evento != en.EV_ACK:
                return                      # refused, or closed immediately
            self.link.esperar_evento(en.EVENTOS_FIN_AVANCE,
                                     timeout=self.link_timeout_evento * 3)
        except en.ErrorEnlace as e:
            self.link.enviar(en.CMD_PARAR)
            self.link.avisos.append("move-closer without closing: {}".format(e))

    # ==================================================================
    # ADVANCE
    # ==================================================================
    def avanzar(self, insistir=False):
        """Sends one cell forward. Returns the closing event.
        With a STOP requested it sends nothing and returns BLOQUEO.

        insistir=True (direction ordered by a figure): if the '5' is
        rejected -- the front sensor sees something closer than UMBRAL_N,
        which after a turn in a 250 mm corridor happens even when the path
        is free -- 'A' is sent (move up to the wall, which is only refused
        closer than DIST_STOP_N) and it counts as an advanced cell. The
        robot moves no matter what towards where the figure points.

        The ESP may reject the command immediately (NACK) if its front ToF
        sees a wall: that happens when the moving-average filter of the Pi
        and the one of the ESP disagree, or when the turn left the robot
        facing something that was not on the map. It is treated as one more
        event, not as an error.
        """
        if getattr(self.link, "paro_pedido", False):
            return en.EV_BLOQUEO
        self.link.enviar(en.CMD_AVANZAR)
        try:
            evento, _telem = self.link.esperar_ack()
            if evento in en.EVENTOS_FIN_AVANCE:
                return evento               # closed immediately
            if evento in (en.EV_NACK, en.EV_DESALINEADO):
                if not insistir:
                    return evento
                print("  [advance] '5' rejected ({}) while a figure rules: "
                      "insisting with 'A' (up to the wall)".format(
                          en.NOMBRE_EVENTO.get(evento, evento)))
                self.link.enviar(en.CMD_HASTA_PARED)
                evento, _telem = self.link.esperar_ack()
                if evento == en.EV_ACK:
                    evento, _telem = self.link.esperar_evento(
                        en.EVENTOS_FIN_AVANCE, timeout=self.link_timeout_evento * 3)
                elif evento not in en.EVENTOS_FIN_AVANCE:
                    return en.EV_NACK       # it is already against the wall
                # 'A' closes with PARED when it arrives: for the map it is a
                # travelled cell, not a discovered wall.
                return en.EV_FIN_CELDA if evento == en.EV_PARED else evento
            evento, _telem = self.link.esperar_evento(
                en.EVENTOS_FIN_AVANCE, timeout=self.link_timeout_evento)
            return evento
        except en.ErrorEnlace as e:
            # 🔴 A forward move that does not close does NOT bring down the
            # run. On Sep 12 the ESP accepted the '5', the wheels were cut
            # (old encoder signs) and fin_celda did not arrive within 6 s:
            # the whole main closed because of an ErrorEnlace. That is a
            # STALL -- the robot is in the same cell, nobody knows where
            # within it -- and the agent already knows how to punish it. It
            # brakes, warns, and continues.
            self.link.enviar(en.CMD_PARAR)
            self.link.avisos.append("advance without closing: {}".format(e))
            print("  [advance] {} -> treated as a STALL".format(e))
            return en.EV_BLOQUEO

    # ==================================================================
    # MAZE EXIT: the four ToF far away
    # ==================================================================
    @staticmethod
    def campo_abierto(telem, estricto=False):
        """True if the ToF (chassis frame, already corrected by the ESP) say
        there is nothing around.

        With estricto=False it is the CANDIDATE: front and both sides above
        config.UMBRAL_SALIDA_MM and the rear above
        config.UMBRAL_SALIDA_TRASERA_MM, which is lower on purpose: right
        after crossing the door, the rear sensor still sees the end of the
        corridor it came out of (about 300 mm), and with all four at the
        same threshold the robot would have to move two cells away before
        noticing.
        With estricto=True (the confirmation, already farther) all four must
        exceed UMBRAL_SALIDA_MM, which is the EV_CAMPO_ABIERTO criterion.
        """
        d = telem.distancias
        # 🔴 The rear sensor only counts if config asks something of it
        # (> 0): the one on this robot reads <100 mm 72% of the time and
        # with it the exit was never confirmed (Sep 15 night: it got out and
        # had to be stopped by hand).
        lados = ("N", "E", "O")
        ok = all(d[k] >= config.UMBRAL_SALIDA_MM for k in lados)
        if config.UMBRAL_SALIDA_TRASERA_MM > 0:
            ok = ok and d["S"] >= (config.UMBRAL_SALIDA_MM if estricto
                                   else config.UMBRAL_SALIDA_TRASERA_MM)
        return ok

    def confirmar_salida(self, telem):
        """Returns True if the robot is really outside the maze.

        🔴 A crossing with long arms also leaves the four sensors at their
        cap: that is why the first reading is not trusted. The robot moves a
        stretch in MANUAL mode (a '#t=' push: it goes through the same speed
        and heading loop, but the ESP does not evaluate open field in that
        state, so it travels the whole requested distance) and looks again.
        Inside a crossing the walls of the arm reappear after a few
        centimeters; outside, there is still nothing. It is repeated
        config.SALIDA_CONFIRMACIONES times.
        """
        if not self.campo_abierto(telem):
            return False
        for i in range(config.SALIDA_CONFIRMACIONES):
            if getattr(self.link, "paro_pedido", False):
                return False
            if True:
                print("  [exit] ToF far away ({}); confirmation {}/{}".format(
                    {d: int(telem.distancias[d]) for d in en.DIRS4},
                    i + 1, config.SALIDA_CONFIRMACIONES))
            self.link.enviar("#t={},0,0,{}\n".format(config.SALIDA_TIRADA_VX,
                                                     config.SALIDA_TIRADA_MS))
            # The ESP sustains the push by itself; the Pi only waits for it
            # to end and for the ramp to brake (fin_tirada arrives as text,
            # not as an event, so it waits by time with a margin).
            time.sleep(config.SALIDA_TIRADA_MS / 1000.0 + config.SALIDA_TIRADA_ESPERA_S)
            self.link.enviar(en.CMD_PARAR)
            time.sleep(0.3)
            telem = self._telemetria_fresca()
            if not self.campo_abierto(telem, estricto=True):
                if True:
                    print("  [exit] a wall reappeared {}: it was NOT the exit".format(
                        {d: int(telem.distancias[d]) for d in en.DIRS4}))
                return False
        return True

    # ==================================================================
    # SQUARE: stop 10 s and re-evaluate (INSTRUCTIONS IN THE MAZE)
    # ==================================================================
    def esperar_cuadrado(self):
        """SQUARE (Sep 16): the robot stays still in the cell for up to
        config.ESPERA_CUADRADO_S watching the front ToF. If the wall in front
        (the one with the square) DISAPPEARS before that -- three readings
        in a row without a wall -- it returns True immediately: it must move
        forward. If after the 10 s it is still there, it returns False: half
        turn and go back.
        Returns (disappeared, telem)."""
        self.link.enviar(en.CMD_PARAR)
        print("  [square] still for up to {:.0f} s watching the front wall".format(
            config.ESPERA_CUADRADO_S))
        limite = time.time() + config.ESPERA_CUADRADO_S
        libres_seguidas = 0
        telem = self._telemetria_fresca()
        while time.time() < limite:
            if getattr(self.link, "paro_pedido", False):
                return False, telem
            time.sleep(0.25)
            telem = self._telemetria_fresca()
            if not telem.paredes.get("N"):
                libres_seguidas += 1
                if libres_seguidas >= 3:
                    print("  [square] the wall disappeared after {:.1f} s (front {:.0f} mm): "
                          "moving forward".format(config.ESPERA_CUADRADO_S - (limite - time.time()),
                                                  telem.distancias.get("N", 0)))
                    return True, telem
            else:
                libres_seguidas = 0
        print("  [square] the wall is still there (front {:.0f} mm): half turn and go back".format(
            telem.distancias.get("N", 0)))
        return False, telem

    def _media_vuelta(self, etiqueta):
        """180 with 'T' (like the circle): the heading becomes the new
        front, the robot settles and observes again from there. Returns
        (ok, obs, telem, pasos90)."""
        nuevo = rl.CONTRARIA[self.heading]
        print("  [camera] {}: half turn towards {}".format(etiqueta, nuevo))
        pasos90, giro_ok = self.girar_hacia(nuevo)
        if not giro_ok:
            self.link.enviar(en.CMD_PARAR)
            return False, None, None, pasos90
        self.heading = nuevo
        self.entro_por = None
        self.rechazadas_aqui = set()
        self.acomodar()
        self.mapa.paredes.pop(self.mapa.celda, None)   # re-recorded from the new front
        obs, telem, _ = self.observar()
        return True, obs, telem, pasos90

    # ==================================================================
    # ONE COMPLETE STEP
    # ==================================================================
    def un_paso(self):
        # 🔴 First settle (heading + stick to the wall), and only then look
        # and decide: the observation is taken from a well-placed robot.
        self.acomodar()
        obs, telem, figura = self.observar()
        celda_previa = self.mapa.celda

        # --- already outside at the start of the step (e.g. the previous step closed oddly) --
        if self.pasos > 0 and self.campo_abierto(telem) and self.confirmar_salida(telem):
            r = self.agente.aprender(en.EV_FIN_CELDA, obs, en_meta=True)
            return ResultadoPaso(direccion=self.heading, evento=en.EV_FIN_CELDA,
                                 en_meta=True, recompensa=r, celda=self.mapa.celda,
                                 celda_previa=celda_previa, giro90=0,
                                 motivo="salida", figura=figura, yaw=telem.yaw,
                                 err_giro=telem.err_giro)

        # --- figure spotted from afar: get closer and obey at its node ------
        pared_frente = bool(telem.paredes.get("N"))
        if self.figura_vista and not figura and not self.figura_pendiente:
            # 🔴 LOGIC OF Sep 16: detect -> (already stopped) -> evaluate
            # calmly which figure it is and where -> STORE it -> approach
            # head-on until the N ToF threshold -> there execute what was
            # stored.
            self.link.enviar(en.CMD_PARAR)
            evaluar = getattr(self.camara, "evaluar", None)
            ev = evaluar(1.0) if evaluar else None
            if ev is None:
                ev = {"figura": self.figura_vista, "votos": 0, "cuadros": 0,
                      "lado": "?", "x": 0.5, "area": 0.0, "confianza": 0.0}
            self.figura_pendiente = ev["figura"]
            self.figura_guardada = dict(ev, celda=self.mapa.celda,
                                        heading=self.heading,
                                        frente_mm=int(telem.distancias.get("N", 0)))
            print("  [camera] FIGURE STORED: {} ({}/{} frames, {} of the frame, "
                  "x={}, area={}, conf={}) with the front at {} mm -> order: {}. "
                  "Approaching the wall, it will be executed there".format(
                      ev["figura"], ev["votos"], ev["cuadros"], ev["lado"], ev["x"],
                      ev["area"], ev["confianza"], self.figura_guardada["frente_mm"],
                      {3: "turn left", 1: "turn right", 2: "go back",
                       None: "wait and re-evaluate"}.get(
                          percepcion.REL_DE_FIGURA.get(ev["figura"]), "?")))
            if hasattr(self.link, "avisos"):
                self.link.avisos.append("figure stored: {} ({} of the frame)".format(
                    ev["figura"], ev["lado"]))
        acercandose = False
        if self.figura_pendiente and not figura:
            if pared_frente:
                # It reached the figure node: it obeys what it asks, even if
                # up close the camera no longer frames it. 🔴 BUT FIRST, MOVE
                # CLOSER: the '5' stops up to 28 cm before the wall, and
                # turning there leaves the robot facing the corridor wall
                # instead of the branch (12285 s: turned with the front at
                # 192 mm and ended up 113 mm from a wall). 'A' takes it to
                # DIST_STOP_N, inside the crossing.
                self.arrimarse()
                figura = self.figura_pendiente
                print("  [camera] at the node of {}: obeying".format(figura))
            else:
                acercandose = True
        if figura:
            self.figura_pendiente = None

        if figura == percepcion.FIGURA_CIRCULO:
            # 🔴 CIRCLE = HALF TURN (Sep 16): turn 180 with 'T', settle, and
            # from the new front the candidates are evaluated again and the
            # RL decides (no reverse: that is only for the dead end).
            # The heading becomes the new front: going back the way it came
            # is now "keep going straight", which is what the figure asks.
            ok, obs2, telem2, pasos90 = self._media_vuelta("circle")
            if not ok:
                obs_sig, telem3, _ = self.observar()
                r = self.agente.aprender(en.EV_BLOQUEO, obs_sig, en_meta=False)
                return ResultadoPaso(direccion=rl.CONTRARIA[self.heading], evento=en.EV_BLOQUEO,
                                     en_meta=False, recompensa=r, celda=self.mapa.celda,
                                     celda_previa=celda_previa, giro90=pasos90,
                                     motivo="figura:circulo|giro_fallido", figura=figura,
                                     yaw=telem3.yaw, err_giro=telem3.err_giro)
            obs, telem = obs2, telem2
            pared_frente = bool(telem.paredes.get("N"))
            direccion, motivo = self.decidir(obs, None)
            motivo = "circulo180+" + (motivo or "")
            print("  [camera] after the half turn: candidates {} -> {}".format(
                obs.candidatas, direccion))
        elif figura == percepcion.FIGURA_CUADRADO:
            # 🔴 SQUARE (Sep 16): still in the cell for up to 10 s watching
            # the front wall. If it disappears before -> move FORWARD. If it
            # is still there after 10 s -> half turn (like the circle) and go
            # back the way it came.
            desaparecio, telem = self.esperar_cuadrado()
            if desaparecio:
                self.mapa.paredes.pop(self.mapa.celda, None)   # the wall is gone
                obs, telem, _ = self.observar()
                pared_frente = bool(telem.paredes.get("N"))
                direccion = rl.DIRS4[self.agente.orientacion % 4]
                self.agente.forzar(obs, direccion)
                motivo = "cuadrado_libre"
            else:
                ok, obs2, telem2, pasos90 = self._media_vuelta("square")
                if not ok:
                    obs_sig, telem3, _ = self.observar()
                    r = self.agente.aprender(en.EV_BLOQUEO, obs_sig, en_meta=False)
                    return ResultadoPaso(direccion=rl.CONTRARIA[self.heading], evento=en.EV_BLOQUEO,
                                         en_meta=False, recompensa=r, celda=self.mapa.celda,
                                         celda_previa=celda_previa, giro90=pasos90,
                                         motivo="figura:cuadrado|giro_fallido", figura=figura,
                                         yaw=telem3.yaw, err_giro=telem3.err_giro)
                obs, telem = obs2, telem2
                pared_frente = bool(telem.paredes.get("N"))
                # going back = keep going straight after the half turn; if
                # for some reason the front is closed, the RL decides
                if self.heading in self.mapa.libres():
                    direccion = self.heading
                    self.agente.forzar(obs, direccion)
                    motivo = "cuadrado180"
                else:
                    direccion, motivo = self.decidir(obs, None)
                    motivo = "cuadrado180+" + (motivo or "")
                print("  [camera] after the half turn of the square -> {}".format(direccion))
        elif acercandose:
            # Straight towards the figure: the absolute direction of the
            # chassis front, which is where the camera looks. It is forced
            # (like a figure) so the agent learns the transition anyway.
            direccion = rl.DIRS4[self.agente.orientacion % 4]
            self.agente.forzar(obs, direccion)
            motivo = "acercandose:{}".format(self.figura_pendiente)
        else:
            direccion, motivo = self.decidir(obs, figura)
            if figura and direccion in self.rechazadas_aqui:
                # The figure's order was already rejected by the ESP in this
                # cell (real wall on that side): decide without it instead of
                # insisting in a loop (Sep 16, circle simulation).
                print("  [camera] the order of {} ({}) was already rejected here: the RL decides".format(
                    figura, direccion))
                direccion, motivo = self.decidir(obs, None)
                motivo = "figura_rechazada+" + (motivo or "")
        # 🔴 "NO EXITS" OR "BACK ONLY" WITH THE FRONT WALL STILL FAR AWAY: the
        # '5' stops up to 28 cm before the wall, and from there the sides
        # still see the corridor walls, not the corner opening: front wall,
        # side walls, back free. That is NOT a dead end (Sep 15 night: it
        # reversed in normal corners). Before believing it, it moves closer
        # to the wall with 'A' and looks again: from there the sides are
        # already inside the opening. If it is still back only, then it IS a
        # dead end.
        # Only if the ToF really sees the front wall: if the front is closed
        # by a VIRTUAL wall (walled-off dead end, rejection), 'A' took it back
        # into the dead end and the map drifted (Sep 16).
        if ((direccion is None or self.solo_atras) and pared_frente
                and telem.distancias.get("N", 0) > config.ARRIMAR_SI_FRENTE_MM):
            print("  [cell] {} with the front at {:.0f} mm: moving closer to the wall "
                  "and looking again".format(
                      "no exits" if direccion is None else "back only",
                      telem.distancias["N"]))
            self.arrimarse()
            self.mapa.paredes.pop(self.mapa.celda, None)   # re-recorded entirely
            obs, telem, figura = self.observar()
            direccion, motivo = self.decidir(obs, figura)
        if direccion is None:
            # rl.py does not return None unless there is NO direction at all
            # (cell walled on all four sides). It is reported and the run is
            # cut: insisting changes nothing.
            return ResultadoPaso(direccion=None, motivo="sin_candidatas",
                                 celda=celda_previa, celda_previa=celda_previa,
                                 evento=None, en_meta=False, giro90=0,
                                 figura=figura, yaw=telem.yaw,
                                 err_giro=telem.err_giro)

        # 🔴 GOING BACK = REVERSE, NOT A HALF TURN (Sep 15 night). In the dead
        # end the robot backs up one cell without turning, with the same
        # forward move and the same stopping criterion applied to the rear
        # sensor, and re-evaluates from there. The half turn does not fit in
        # the corridor.
        if direccion == rl.CONTRARIA.get(self.heading):
            # REAL dead end = according to the sensors the only free thing is
            # the back. If it goes back for another reason (everything free is
            # in the recent history and the RL chose back) NOTHING is walled
            # off: walling there closed good corridors (test_headless, Sep 16).
            callejon_real = self.mapa.libres() == [direccion]
            print("  [reverse] {}: backing up one cell".format(
                "dead end" if callejon_real else "back because of history"))
            evento = self.retroceder()
            if evento in (en.EV_FIN_CELDA, en.EV_PARED):
                self.mapa.avanzar(direccion)
                self.rechazadas_aqui = set()
                # 🔴 The front is still the same and leads to the dead end it
                # just left: it is WALLED OFF for the rest of the run (Sep 16;
                # before, it was marked as "entro_por", i.e. forced free, and
                # the robot went back in: back-and-forth). Behind it, the way
                # it first entered this cell stays free, even if the rear ToF
                # says wall.
                if callejon_real:
                    self.mapa.tapiar(self.heading)
                    print("  [reverse] walled off {} from {}; candidates now: {}".format(
                        self.heading, self.mapa.celda, self.mapa.candidatas()))
                self.entro_por = self.mapa.entradas.get(self.mapa.celda)
                evento = en.EV_FIN_CELDA
            else:
                # The reverse did not happen (nack: the rear sees a wall, or a
                # stall): back leaves the menu while it stays in this cell.
                # Without this the RL chose back again and the ESP refused
                # again, in a loop (test_headless, Sep 16).
                self.rechazadas_aqui.add(direccion)
                if evento == en.EV_DESALINEADO:
                    evento = en.EV_BLOQUEO
            en_meta = False
            obs_sig, telem2, _ = self.observar()
            r = self.agente.aprender(evento, obs_sig, en_meta=False)
            self.pasos += 1
            return ResultadoPaso(direccion=direccion, evento=evento,
                                 en_meta=False, recompensa=r,
                                 celda=self.mapa.celda, celda_previa=celda_previa,
                                 giro90=0, motivo=(motivo or "") + "|reversa",
                                 figura=figura, yaw=telem2.yaw,
                                 err_giro=telem2.err_giro)

        # 🔴 TURN ONLY AT THE WALL THRESHOLD (square_right: PARADA). If there
        # is a wall ahead but still far away (the '5' stops up to 28 cm
        # before), first move closer with 'A' and turn from there. A turn in
        # the middle of a cell leaves the robot outside the crossing.
        if (direccion != self.heading and pared_frente
                and telem.distancias.get("N", 0) > config.ARRIMAR_SI_FRENTE_MM):
            self.arrimarse()

        pasos90, giro_ok = self.girar_hacia(direccion)
        if not giro_ok:
            # Same treatment as in the physical demo: event 6 (BLOQUEO), which
            # agente.aprender() already knows how to punish (R_BLOQUEO), and
            # no forward move.
            self.link.enviar(en.CMD_PARAR)
            obs_sig, telem2, _ = self.observar()
            r = self.agente.aprender(en.EV_BLOQUEO, obs_sig, en_meta=False)
            return ResultadoPaso(direccion=direccion, evento=en.EV_BLOQUEO,
                                 en_meta=False, recompensa=r,
                                 celda=self.mapa.celda,
                                 celda_previa=celda_previa, giro90=pasos90,
                                 motivo=motivo, figura=figura,
                                 yaw=telem2.yaw, err_giro=telem2.err_giro)

        # With a figure (or approaching one) the forward move is mandatory.
        evento = self.avanzar(insistir=bool(figura) or acercandose)
        # 🔴 TILTED MORE THAN tolavance (12 degrees): the ESP does not move
        # forward. It aligns with 'Y' on the multiple of 90 and retries once.
        # It is the safety net against the diagonal of Sep 15 night: a crash
        # or a failed turn left the robot rotated and it advanced like that.
        if evento == en.EV_DESALINEADO:
            print("  [align] the ESP rejected the advance because of heading (err {:+.1f}): "
                  "aligning with Y and retrying".format(
                      self.link.ultima.err_giro if self.link.ultima else 0.0))
            evento = self.avanzar(insistir=bool(figura) or acercandose) if self.alinear() else en.EV_BLOQUEO
        if evento == en.EV_DESALINEADO:
            evento = en.EV_BLOQUEO

        # --- effects of each event on the map ---------------------------------
        if evento == en.EV_FIN_CELDA:
            self.mapa.avanzar(direccion)
            self.heading = direccion
            self.entro_por = rl.CONTRARIA[direccion]
            self.mapa.entradas.setdefault(self.mapa.celda, self.entro_por)
            self.rechazadas_aqui = set()
        elif evento in (en.EV_PARED, en.EV_NACK):
            # The position does not change, but a new wall WAS learned: that
            # direction leaves the menu while the robot stays in this cell
            # (see rechazadas_aqui).
            self.mapa.anotar_pared_descubierta(direccion)
            self.rechazadas_aqui.add(direccion)
        # EV_BLOQUEO and EV_CAMPO_ABIERTO do not touch the map: nobody knows
        # where the robot ended up within the cell, but the cell is the same.

        en_meta = config.FIN_POR_META and self.mapa.en_meta(self.meta)
        obs_sig, telem2, _ = self.observar()

        # --- maze exit ------------------------------------------------------
        # The ESP sees it while moving (EV_CAMPO_ABIERTO); the Pi sees it
        # stopped (campo_abierto), WITH ANY EVENT (Sep 15 night: it was only
        # checked after fin_celda, and the last stretch when leaving usually
        # closes with a wall or a nack against the outer border: the robot,
        # already outside, was still "inside").
        salida = False
        if evento == en.EV_CAMPO_ABIERTO or self.campo_abierto(telem2):
            print("  [exit] candidate: N {:.0f} E {:.0f} O {:.0f} (threshold {:.0f})".format(
                telem2.distancias["N"], telem2.distancias["E"], telem2.distancias["O"],
                config.UMBRAL_SALIDA_MM))
            salida = self.confirmar_salida(telem2)
            if salida:
                if evento == en.EV_CAMPO_ABIERTO:
                    # The robot left the board halfway through a stretch: for
                    # the map it counts as the next cell, which is where it
                    # ended up (outside, and exactly where no longer matters).
                    self.mapa.avanzar(direccion)
                    self.heading = direccion
                # It is learned as reaching the goal, not with the open-field
                # punishment: getting out IS the goal of this exam.
                evento = en.EV_FIN_CELDA
                en_meta = True
                motivo = (motivo or "") + "|salida"
            else:
                # False alarm (a crossing): carry on with what happened, and
                # observe again because the confirmation moved the robot.
                obs_sig, telem2, _ = self.observar()

        r = self.agente.aprender(evento, obs_sig, en_meta=en_meta)

        self.pasos += 1
        return ResultadoPaso(direccion=direccion, evento=evento,
                             en_meta=en_meta, recompensa=r,
                             celda=self.mapa.celda, celda_previa=celda_previa,
                             giro90=pasos90, motivo=motivo, figura=figura,
                             yaw=telem2.yaw, err_giro=telem2.err_giro)

    # ==================================================================
    # RUN
    # ==================================================================
    def preparar_corrida(self):
        """Leaves robot, map and agent ready to start from scratch.

        It does not recalibrate the gyroscope: the ESP does that at startup,
        with the robot still. If the yaw is seen to drift between runs, the
        bench command 'C' repeats it (see BLOCK 19 of the firmware).
        """
        self.link.paro_pedido = False
        self.link.enviar(en.CMD_PARAR)
        # The robot has just been placed BY HAND: the heading it has NOW is
        # the reference. Without this, the heading loop defends the reference
        # of the previous run and drags that error into the first stretch
        # (up to RUMBO_ARRASTRE_MAX=25 degrees without discarding it).
        self.link.enviar(en.CMD_FIJAR_RUMBO)
        self.link.purgar()
        self.mapa.reiniciar_corrida()
        self.heading = "N"
        self.pasos = 0
        self.figura_pendiente = None
        self.figura_vista = None
        self.figura_guardada = None
        self.entro_por = None
        self.rechazadas_aqui = set()
        # "N" is simply WHERE THE ROBOT FACES AT STARTUP: the map is relative
        # to that pose ('Z' sets the yaw to 0 there). There is no physical
        # North to respect.
        self.agente.orientacion = 0

    # Timeouts: set by main.py from config.py. They are left as attributes
    # and not as constants of this module so they can be raised in the lab
    # without touching the logic.
    link_timeout_evento = 6.0
    link_timeout_giro = 25.0
