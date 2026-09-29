"""Configuration of the Raspberry Pi side.

A single file with everything that gets touched in the lab, so there is no
need to open the logic to change a port or the size of the maze.
"""

import os

# --- serial link with the ESP32 (USB-C cable between the two boards) ------
# /dev/ttyUSB0 with a CP2102/CH340 adapter, /dev/ttyACM0 if the ESP32 exposes
# native CDC. If it is ever wired through the GPIO pins: /dev/serial0.
PUERTO_SERIE = os.environ.get("PUERTO_ESP", "/dev/ttyUSB0")
BAUDIOS = 115200

# Maximum wait for the closing event of each primitive.
# The turn is BLOCKING on the ESP and has its own TIMEOUT_GIRO_MS=6000 with
# up to 3 passes: that is why its wait is the longest of all. If it expires
# here, the problem is the link, not the motion.
# 🔴 20 s (Sep 15 night): with the square cruise speed (0.12 m/s), the 1.5 s
# ramp and the progressive braking, one cell can take more than 6 s; with 6
# the Pi sent H halfway through a cell and logged it as a stall (10 times in
# a row).
TIMEOUT_EVENTO_S = 20.0
TIMEOUT_GIRO_S = 25.0
TIMEOUT_TELEMETRIA_S = 3.0     # not a single CSV line in this time = link down

# Port of the web interface served by the Pi (web_interface.py). It listens
# on all interfaces so it can be opened from the laptop over the lab
# network: http://<pi-ip>:8080
PUERTO_WEB = 8080

# --- ToF: which sensor is which ---------------------------------------------
# 🔴 MEASURED ON THE ROBOT (2026-09-12): the mux channels the firmware assigns
# to each ToF (BLOCK 03 of the .cpp, marked [ADJUST]) do not match how they
# were mounted. What the ESP calls N faces WEST, what it calls O faces NORTH,
# and E/S are swapped. serial_link.Telemetria applies this dictionary to fix
# it on the Pi without reflashing the ESP: key = what the ESP says, value =
# where it really faces. Empty = touch nothing.
# It is an emergency patch: the ESP uses ITS "N" to refuse to move against a
# wall, and that can only be fixed in the firmware (BLOCK 03, CANAL_*).
# Sep 12: fixed in the firmware (CANAL_N=5 E=4 S=2 O=3), so it stays empty
# here. If a firmware older than Sep 12 is flashed, put back
#     {"N": "O", "O": "N", "E": "S", "S": "E"}
TOF_REMAPA = {}

# --- maze -------------------------------------------------------------------
# 🔴 Sep 15 (afternoon): THE MAZE IS RANDOM AND THE EXIT CAN BE ON ANY SIDE.
# The only fixed thing is that the robot starts facing NORTH.
# That is why the grid is large and the start is in the CENTER: the Pi map
# marks the border of the board as a wall (maze_map.anotar_paredes), and if
# the start were at (0,0) an exit to the North or the West would be
# impossible to represent. With 16x16 and the robot at (8,8) there is room
# for 8 cells in every direction, more than any maze in the lab.
LADO = 16
# Real size of a maze cell, in cm. Used ONLY by the interface to rebuild the
# map with odometry (continuous position and ToF hits). The distance the ESP
# travels for each '5' is the `celda` parameter of PARAMS_CALIBRADOS
# (main.py): if the two do not match, the robot drifts a fraction of a cell
# per step and the map shows it.
CELDA_CM = 24.0
CELDA_INICIAL = (8, 8)
# META is no longer where the run ends: the exit is detected by the sensors
# (see SALIDA_* below). It is kept because rl.py uses it as a network feature
# and in the pattern signature (vector to the goal), and it has to be given
# some value. It is placed in a corner so that the R_PROGRESO bias pushes
# exploration outwards and not back to the start.
META = (15, 15)
# False = reaching META does not end the run; only the physical exit of the
# maze ends it (all ToF far away). That is the exam: the robot has to GET
# OUT, not reach a coordinate.
FIN_POR_META = False

# --- maze exit ----------------------------------------------------------------
# The ESP already emits EV_CAMPO_ABIERTO (5) when, WHILE MOVING FORWARD, the
# four ToF exceed UMBRAL_FUERA (478.7 real mm) three loops in a row. Here the
# same criterion is replicated with the robot STOPPED, on the telemetry that
# arrives after each step, because the event is only emitted in the middle
# of a forward move and a robot that stopped right when crossing the door
# would never emit it.
# 🔴 300 since Sep 15 night (was 470): with 470 the robot, already outside,
# was still "inside" because of false readings. The two confirmation pushes
# are enough.
UMBRAL_SALIDA_MM = 500.0     # 50 cm, no confirmation (Sep 15 night)
# The rear one, only for the CANDIDATE: right after crossing the door it still
# sees the end of the exit corridor (~300 mm). Confirmation requires all four.
# 🔴 Sep 15 night: the rear sensor reads <100 mm 72% of the day and 195 with
# seven free cells behind it (it is seeing the floor or part of the chassis).
# Until it is repositioned, little is asked of it: 150. With 250 the exit
# would never be confirmed.
UMBRAL_SALIDA_TRASERA_MM = 0.0     # 0 = the rear sensor does NOT count (badly mounted)
# 🔴 CONFIRMATION BEFORE ACCEPTING IT. A crossing with four long arms also
# leaves the four ToF above the threshold (2 cells away the sensor already
# returns its 600 cap). To tell them apart, the navigator moves forward in
# manual mode (a push, which does NOT trigger open field on the ESP) and
# looks again: at a crossing the side walls of the arm reappear; outside the
# maze there is still nothing. 0 = no confirmation.
# 🔴 0 since Sep 15 night: the confirmation pushes took the robot 50 cm out,
# and there the front sensor found the wall of the room (171 mm) and
# discarded it. Outside the maze there is no room to "confirm by moving
# forward": the exit is declared as soon as front and sides exceed
# UMBRAL_SALIDA_MM.
SALIDA_CONFIRMACIONES = 0
SALIDA_TIRADA_MS = 1500          # ~25-30 cm at 187/255 of vmanual (0.22 m/s), with ramp
SALIDA_TIRADA_VX = 187
SALIDA_TIRADA_ESPERA_S = 1.2     # margin after the push so the ramp can brake

# "No exits" with the front wall farther than this: move closer with 'A' and
# look again before aborting (see navigator.un_paso).
# Sep 16: 'A' stops at 110 mm (param `arrimar`, center of the crossing cell);
# if it is already closer than 120 there is nothing to move closer to.
ARRIMAR_SI_FRENTE_MM = 120.0

# SQUARE figure (INSTRUCTIONS IN THE MAZE, Sep 12): "stop for 10 s and
# re-evaluate (the wall may or may not disappear)". The navigator waits this
# long with the robot still and reads the ToF again before deciding.
ESPERA_CUADRADO_S = 10.0

# --- run policy -------------------------------------------------------------
# "Shortcut" of Sep 15 night: in a cell without experience (Q table at zero)
# the candidate with the HIGHEST neurocontroller activation (most open side)
# is chosen, without softmax. The RL learns from that transition anyway.
DECIDIR_POR_NEURO = False   # Sep 16: disabled, the RL decides as before

# Same as main.py of PRuebaRLgiroeje and simulador_entrenamiento.py: without
# this cap, a run that never reaches the goal wanders FOREVER, because the
# softmax temperature only drops when fin_corrida() increments
# self.corridas. See PROMPT_portar_fix_bucles_y_castigos.md.
MAX_PASOS_CORRIDA = 400

# Window of the hard prohibition of short loops: a direction whose
# destination is among the last N cells occupied IN THIS RUN is not chosen.
# THIS IS THE FIX THAT SOLVED THE INFINITE LOOP (2026-09-05), ported as is
# from PRuebaRLgiroeje/ejecutor.py. It is not one more punishment: it is a
# prohibition, and that is why it works where raising R_REVISITA did not.
HISTORIAL_RECIENTE = 6

# --- persistence --------------------------------------------------------------
_AQUI = os.path.dirname(os.path.abspath(__file__))
RUTA_ESTADO = os.path.join(_AQUI, "data", "rl_state_robot.json")
# Starting weights: 4555 simulation runs of the old firmware. Better that
# than random noise in the first real run.
RUTA_PESOS_H = os.path.join(_AQUI, "q_network_weights.h")

# --- log ----------------------------------------------------------------------
# One row per decision, overwritten on every start and flushed on every row
# so nothing is lost if the process is cut halfway. Same pattern as
# prueba2mascastigos/PRuebaRLgiroeje/main.py.
RUTA_LOG = os.path.join(_AQUI, "data", "movement_log.csv")
LOG_ACTIVO = True

# --- sensor log (to know what to tune, not to reconstruct the RL) -------------
# WHAT IT IS AND HOW IT DIFFERS FROM RUTA_LOG: RUTA_LOG is one row per RL
# DECISION (one cell). This is one row per TELEMETRY sample (at a low rate,
# not at the 10 Hz of the cable) with ALL the sensor numbers and the gains in
# force at that instant -- what is needed to answer "why is this wheel
# drifting" or "why is this ToF useless" without rereading the ESP CSV by
# hand.
# It is NOT erased between runs (unlike RUTA_LOG, which abrir_log()
# truncates on every start): it is used to compare one session with the
# previous one.
RUTA_REGISTRO_CSV = os.path.join(_AQUI, "data", "sensor_log.csv")
# The report IS rewritten entirely every time (see data_logger.py): it is a
# summary of the current moving window, not an ever-growing history.
RUTA_REGISTRO_MD = os.path.join(_AQUI, "data", "sensor_log.md")
REGISTRO_ACTIVO = True
REGISTRO_PERIODO_S = 0.5        # at most one CSV row every half second
REGISTRO_INFORME_CADA_N = 30    # rewrites the .md every 30 rows (~15 s)
REGISTRO_VENTANA = 400          # samples averaged by the statistics (~200 s)
