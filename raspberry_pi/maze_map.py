"""Maze map and hard prohibition of short loops.

WHAT IT DOES
    - Stores the discovered walls, in the ABSOLUTE FRAME, cell by cell.
    - Keeps the position (x, y), which only changes with EV_FIN_CELDA.
    - Builds the list of `candidatas` consumed by rl.py.

COORDINATES (those of the firmware, do not "fix"): E=+1, O=-1 (West) in x;
S=+1, N=-1 in y. The Y axis grows towards the SOUTH.

WHERE `candidatas` COMES FROM -- 2026-09-05, see
Robot/PROMPT_portar_fix_bucles_y_castigos.md: the robot kept going back and
forth at crossings. Raising R_REVISITA / R_RETROCESO and lowering the
temperature was NOT enough, because a punishment subtracts points but never
takes the softmax probability to ZERO. The solution that did work was a
PROHIBITION: not even offering the directions that lead to one of the last
occupied cells. rl.py already had the mechanism ready
(Observacion.candidatas / AgenteRL._candidatas(), designed for Tremaux),
only nobody filled it.

If the prohibition leaves the list empty (short dead end, cell surrounded by
recently visited places) it is returned empty on purpose: rl.py falls back
by itself to "all the free ones" (`if cand: return cand` else
`return libres`), so the agent is never left without options.
"""

import collections

import rl


class Mapa:
    def __init__(self, lado, celda_inicial, historial_reciente):
        self.lado = lado
        self.celda_inicial = tuple(celda_inicial)
        self.celda = tuple(celda_inicial)
        # paredes[(x,y)] = {"N":bool,...} in the ABSOLUTE frame. It only has
        # the cells already visited: the map is discovered, not known in
        # advance.
        self.paredes = {}
        # Walls DISCOVERED by the ESP (nack / braking for a wall): they are
        # ALWAYS added to what the ToF say. Sep 15 night: anotar_paredes()
        # rewrote the cell on every observation and erased the wall the
        # rejection had just discovered -> 19 nacks in a row in the same
        # direction.
        self.descubiertas = {}
        # 🔴 WALLED-OFF DEAD ENDS (Sep 16): when the robot leaves a dead end in
        # reverse, the direction that leads to it is walled off in the cell
        # from which it is seen. It is a virtual wall ADDED to the ToF on
        # every observation: without it the front (free for the sensors, the
        # dead end is right there) became a candidate again and the robot
        # went in and out of the dead end back and forth.
        # callejones[(x,y)] = {"N",...}
        self.callejones = {}
        # Where each cell was ENTERED the first time (absolute frame). After
        # a reverse, it is what is still free behind the robot even if the
        # rear ToF (which reads too little) says wall.
        self.entradas = {}
        self.vistas = set()
        # Sliding window of the last cells occupied IN THIS RUN.
        self.historial_reciente = collections.deque(maxlen=historial_reciente)

    # ------------------------------------------------------------------
    def reiniciar_corrida(self):
        """Goes back to the starting point. The wall MAP is NOT erased (what
        was discovered is still true), but the loop history is: it is a
        window of the current run, not of the maze."""
        self.celda = self.celda_inicial
        self.historial_reciente.clear()
        self.vistas = set()
        # dead ends are walled off per run: the map drifts a fraction of a
        # cell from one run to the next and an old virtual wall could block
        # a good corridor
        self.callejones = {}
        self.entradas = {}

    def olvidar_todo(self):
        """New maze: what was discovered is erased too."""
        self.paredes = {}
        self.descubiertas = {}
        self.callejones = {}
        self.entradas = {}
        self.reiniciar_corrida()

    # ------------------------------------------------------------------
    def dentro(self, celda):
        x, y = celda
        return 0 <= x < self.lado and 0 <= y < self.lado

    def vecina(self, celda, direccion):
        return (celda[0] + rl.DX[direccion], celda[1] + rl.DY[direccion])

    def anotar_paredes(self, paredes_absolutas):
        """Stores what the ToF see from the current cell.

        The borders of the maze are recorded as walls even if the sensor
        does not see them: leaving the board is not a legal option, and
        without this the agent could choose a direction that takes it off
        the map.
        """
        p = dict(paredes_absolutas)
        for d in rl.DIRS4:
            if not self.dentro(self.vecina(self.celda, d)):
                p[d] = True
        # Sep 16: the "discovered" walls are NO LONGER added: with the map
        # shifted by a fraction of a cell they blocked a front the ToF saw
        # free (298 mm) and the robot turned back. The sensors rule.
        # The walled-off dead ends ARE added (Sep 16): they belong to this run.
        for d in self.callejones.get(self.celda, ()):
            p[d] = True
        self.paredes[self.celda] = p
        self.vistas.add(self.celda)

    def tapiar(self, direccion):
        """Virtual wall in the current cell towards `direccion`: there is an
        already travelled dead end there. Valid for the whole run."""
        self.callejones.setdefault(self.celda, set()).add(direccion)
        if self.celda in self.paredes:
            self.paredes[self.celda][direccion] = True

    def anotar_pared_descubierta(self, direccion):
        """The ESP braked for a wall (ev=4) or rejected the command (ev=2):
        there is a wall the ToF filter had not confirmed yet."""
        self.descubiertas.setdefault(self.celda, set()).add(direccion)
        if self.celda in self.paredes:
            self.paredes[self.celda][direccion] = True

    def avanzar(self, direccion):
        """Only called with EV_FIN_CELDA. It is the ONLY place where the
        position changes: if this is called too often, the Pi map gets out
        of sync with the real world and there is no way to notice until the
        robot is already lost."""
        self.celda = self.vecina(self.celda, direccion)
        self.vistas.add(self.celda)
        self.historial_reciente.append(self.celda)

    # ------------------------------------------------------------------
    def paredes_actuales(self):
        return self.paredes.get(self.celda, {d: False for d in rl.DIRS4})

    def libres(self):
        p = self.paredes_actuales()
        return [d for d in rl.DIRS4 if not p[d]]

    def candidatas(self):
        """Free directions whose destination is NOT in the recent history,
        and if any of them leads to a cell NEVER seen in this run, ONLY
        those.

        Sep 15 (afternoon): the robot went back and forth between already
        explored cells while having new corridors next to it. A punishment
        (R_REVISITA) subtracts points but never takes the probability to
        zero; this is a prohibition, like the history one: as long as there
        is something to explore from here, going back is not on the menu.
        When nothing new is left around (dead end, exhausted area) everything
        free and not recent is offered again, and the RL chooses among that.

        It may return [] on purpose: see the header of the file.
        """
        cand = [d for d in self.libres()
                if self.vecina(self.celda, d) not in self.historial_reciente]
        nuevas = [d for d in cand if self.vecina(self.celda, d) not in self.vistas]
        return nuevas if nuevas else cand

    def en_meta(self, meta):
        return self.celda == tuple(meta)
