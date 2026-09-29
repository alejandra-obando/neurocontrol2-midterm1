#!/usr/bin/env python3
"""Raspberry Pi main program.

    python3 main.py --port /dev/ttyUSB0
    python3 main.py --port /dev/ttyUSB0 --runs 5
    python3 main.py --port /dev/ttyUSB0 --forget        # new maze
    python3 main.py --no-interface                      # console only

Requires pyserial (pip3 install pyserial) and numpy (for rl.py).

HOW THE ROBOT STARTS: from the interface, at http://<the-pi>:8080. Open it,
enable the motors and press START. There is no longer any need to press EN
on the ESP as in the old motors-only test: the physical button stops being
the start switch, which is exactly what we wanted to remove.

WHAT THIS FILE DOES AND WHAT IT DOES NOT
    It assembles the pieces, runs the loop of runs and writes the log. The
    logic of one decision lives in navigator.py; the policy, in rl.py; the
    page, in web_interface.py.

rl.py lives next to this file and is the single source of truth for the
policy.
"""

import argparse
import csv
import json
import os
import sys
import time

_AQUI = os.path.dirname(os.path.abspath(__file__))
_RUTA_RL = _AQUI
sys.path.insert(0, _AQUI)

if not os.path.exists(os.path.join(_RUTA_RL, "rl.py")):
    sys.exit("[FATAL] rl.py not found in {}\n"
             "  Copy the whole raspberry_pi/ folder to the Pi.".format(_RUTA_RL))

import rl                                    # noqa: E402  (after the sys.path)

import camera_master as camara_master        # noqa: E402
import config                                # noqa: E402
import serial_link as en                     # noqa: E402
import web_interface as ui                   # noqa: E402
import maze_map as mapa_mod                  # noqa: E402
import navigator as nav_mod                  # noqa: E402
import data_logger as reg_mod                # noqa: E402
# perception.py is no longer imported here: the robot camera is
# camera_master.CamaraMaster, which exposes the same three methods the
# navigator uses (disponible / leer / cerrar) and also solves what
# perception.Camara was missing -- a single reader of /dev/video0 and a
# burst of frames at the moment of deciding. perception.py is still the
# skeleton that documents the contract, and camera_master imports it.


CABECERA_LOG = [
    "paso_global", "corrida", "paso_corrida", "celda_antes", "celda_despues",
    "heading_antes", "direccion", "giro90", "evento", "en_meta", "motivo",
    "figura", "recompensa", "orientacion", "temperatura", "modo", "corridas",
    "yaw", "err_giro", "sesion",
]
SESION = time.strftime("%Y-%m-%d_%H%M%S")

# Starting values of the gains, the same ones written in the firmware. The
# interface uses them for the "back to calibrated values" button, so if they
# are changed in the .cpp they must be changed here too.
#
# 🔴 AND IT IS NOT ONLY FOR THAT BUTTON: THE Pi SENDS THEM TO THE ESP AT
# STARTUP, so whatever is here OVERRIDES what the firmware has written. On
# Sep 14 kp, ki, crucero and vmanual were calibrated in the .cpp, this table
# kept the old ones, and the first push on the floor came out at 38.5 rpm
# instead of 77: half. If a parameter in this list is touched in the .cpp,
# it must ALSO be touched here, or it is useless.
PARAMS_CALIBRADOS = {
    "kp": 0.40, "ki": 0.10, "kd": 0.0,
    "kpy": 3.6, "kiy": 0.0, "kdy": 0.7,    # those of carro_v08, as they were
    "corrmax": 30.0, "signoyaw": 1.0,
    # cruise 0.12 = V_CRUCERO 102/255 of square_right.py (was 0.22)
    # Sep 16: back to 0.12 / ramp 1.5 / brake from 330 (those of
    # PRUEBA1509, which did not crash); with 0.15 it crashed
    "crucero": 0.12, "gradfreno": 45.0, "vgiro": 60.0, "vgiroder": 60.0,
    "vgiromin": 25.0, "acelgiro": 1.8, "acel": 0.22, "acelcorr": 25.0, "fraccorr": 0.22, "corrmin": 13.0,
    # tolavance 12: whatever is left over from a turn (< 4.4) is compensated
    # while rolling; more than 12 degrees is a crash or a failed turn and the
    # ESP does not move forward until the Pi aligns it with Y (Sep 15 night:
    # diagonal at the end of the run).
    "tolgiro": 4.4, "tolalin": 4.0, "tolavance": 12.0,
    # stop at 61 in corners; in a T (two open sides) the ESP stops at
    # `arrimar` (110), the center of the crossing. paradaS: rear in reverse.
    # paradaS 10 and segS 5 (Sep 16 afternoon): the rear ToF reads 16-41 mm
    # in open space; with any useful threshold it refused to reverse. Only
    # the one-cell encoder cap remains. CHECK THE SENSOR.
    "parada": 61.0, "paradaS": 10.0,
    # per-side safety (Sep 16): hard brake if a ToF goes below this, in any
    # motion; low so they do not trigger in the corridor (setpoints 43-52)
    "segN": 35.0, "segE": 22.0, "segS": 5.0, "segO": 22.0,
    "fracrev": 0.6,   # reverse at 60% of the cruise speed
    # arrimar 110: where 'A' stops = robot centered in the crossing cell (measured Sep 16)
    "arrimar": 110.0,
    # wall thresholds per sensor (mm): sides at 200 since Sep 15 night
    # umbralN 230 (Sep 16): the back wall of the crossing counts as seen at 230
    "umbralN": 230.0, "umbralE": 200.0, "umbralS": 200.0, "umbralO": 200.0,
    # forward motion and centering by taps: the numbers of square_right.py
    "centrar": 1.0, "vmin": 0.035, "rampaini": 1.5, "frenadesde": 330.0,
    "latfreno": 0.20, "vytoque": 0.047, "bandalat": 5.0, "errpleno": 18.0,
    "sesgolat": 3.8, "minlat": 32.0, "vyurg": 0.053, "consE": 52.5, "consO": 43.0,
    "limlat": 140.0, "sintras": 150.0,
    "pegar": 1.0, "vypegar": 0.076, "inerlat": 12.0, "bandapegar": 5.0, "topepegar": 9.0,
    "vmanual": 0.30, "wmanual": 90.0,
    "factecho": 1.8, "margtecho": 30.0,
    "celda": 27.0, "odom": 1.09,
    "calpwm": 150.0, "calms": 600.0,
}


def abrir_log(ruta, activo):
    """Decision log. 🔴 Since Sep 15 (night) it is OPENED IN APPEND mode:
    every start of the program erased the previous runs and the session data
    was lost. Now it accumulates (the `sesion` column tells which start each
    row belongs to) and, in addition, each run is copied whole to data/runs/
    when it ends (see _correr_con_camara)."""
    if not activo:
        return None, None
    nuevo = not os.path.exists(ruta) or os.path.getsize(ruta) == 0
    f = open(ruta, "a", newline="", encoding="utf-8")
    w = csv.writer(f)
    if nuevo:
        w.writerow(CABECERA_LOG)
        f.flush()
    return f, w


def fila_log(w, f, paso_global, corrida, paso_corrida, heading_antes, res, agente):
    if w is None:
        return
    w.writerow([
        paso_global, corrida, paso_corrida,
        "{},{}".format(*res.celda_previa) if res.celda_previa else "",
        "{},{}".format(*res.celda) if res.celda else "",
        heading_antes, res.direccion or "", res.giro90,
        en.NOMBRE_EVENTO.get(res.evento, res.evento),
        1 if res.en_meta else 0, res.motivo or "", res.figura or "",
        "" if res.recompensa is None else round(res.recompensa, 4),
        agente.orientacion, round(agente.temperatura(), 3),
        "explotacion" if agente.modo_explotacion else "exploracion",
        agente.corridas,
        "" if res.yaw is None else round(res.yaw, 2),
        "" if res.err_giro is None else round(res.err_giro, 2),
        SESION,
    ])
    f.flush()   # a row may be the last one if the process is cut


def esperar_start(tablero, link, timeout=None, al_limpiar=None):
    """Blocks until someone presses START in the interface.

    Meanwhile telemetry keeps arriving and the page looks alive: the link
    pump runs in its own thread, not in this one. The only command handled
    while waiting is 'limpiar' (erase runs and map).
    """
    tablero.publicar_estado("esperando", "Place the robot at the start "
                            "(facing anywhere: the map is relative to that "
                            "pose; you can move it with the manual control), "
                            "enable the motors and press START")
    limite = None if timeout is None else time.time() + timeout
    while not tablero.quiere_correr():
        if limite and time.time() > limite:
            return False
        for o in tablero.tomar_ordenes():
            if o.get("accion") == "limpiar" and al_limpiar:
                al_limpiar(bool(o.get("todo")))
                tablero.publicar_estado("esperando", "Runs erased{}. Place the robot "
                                        "at the start and press START".format(
                                            " and network reset" if o.get("todo") else ""))
        time.sleep(0.1)
    tablero.tomar_ordenes()          # commands received while waiting are discarded
    return True


def correr(args):
    pesos_h = config.RUTA_PESOS_H if os.path.exists(config.RUTA_PESOS_H) else None
    if pesos_h is None:
        print("[warning] q_network_weights.h not found: the network starts at random")

    agente = rl.AgenteRL(lado=config.LADO, meta=config.META,
                         ruta_estado=config.RUTA_ESTADO,
                         pesos_iniciales_h=pesos_h)
    el_mapa = mapa_mod.Mapa(config.LADO, config.CELDA_INICIAL,
                            config.HISTORIAL_RECIENTE)

    def limpiar_corridas(todo=False):
        # todo=True (interface button, Sep 16): the network and the pattern
        # memory are ALSO reset -- the network goes back to the factory
        # weights (q_network_weights.h) if they exist, or to random. It is
        # for tests without any bias from previous runs.
        if todo:
            agente.red = rl.RedQ(0)
            if pesos_h:
                agente.red.cargar_header_cpp(pesos_h)
            agente.patrones = rl.MemoriaPatrones()
        # Different maze (or runs that contributed nothing): the Q table, the
        # master route and the discovered walls are erased; the network and
        # the patterns (structural knowledge) are kept. olvidar_laberinto()
        # does not touch `corridas`, and the softmax temperature comes from
        # it: after 14 bad runs (Sep 12) it was at its minimum and the robot
        # started greedy on an empty table. Exploration is restarted too.
        agente.olvidar_laberinto()
        el_mapa.olvidar_todo()
        agente.corridas = 0
        agente.exitos = 0
        agente.guardar()             # write it to disk NOW, not at the end
        print("[init] maze memory erased, network {}; "
              "exploration restarted (T={:.2f})".format(
                  "RESET (everything from scratch)" if todo else "kept",
                  agente.temperatura()))

    # 🔴 GOAL DIFFERENT FROM THE ONE IN THE SAVED STATE (Sep 12: the exit went
    # from (7,7) to (7,1)). rl.cargar() does not compare the goal, and the Q
    # table, master route and learned traces point to the old goal: running
    # with that is running with the map of another maze. It is treated as
    # --forget.
    meta_guardada = None
    try:
        with open(config.RUTA_ESTADO, "r", encoding="utf-8") as f:
            meta_guardada = json.load(f).get("meta")
    except (OSError, ValueError):
        pass
    if meta_guardada is not None and tuple(meta_guardada) != tuple(config.META):
        print("[init] the saved state belongs to goal {} and config.META is {}: "
              "the maze memory is erased".format(
                  tuple(meta_guardada), tuple(config.META)))
        limpiar_corridas()
    elif args.olvidar:
        limpiar_corridas()

    print("[init] agent: runs={} successes={} best={} mode={}".format(
        agente.corridas, agente.exitos, agente.mejor_pasos,
        "explotacion" if agente.modo_explotacion else "exploracion"))

    tablero = ui.Tablero(config.LADO, config.META)

    def limpiar_desde_interfaz(todo=False):
        limpiar_corridas(todo)
        tablero.publicar_mapa(el_mapa)
        tablero.publicar_corrida(corridas=0, exitos=0, paso=0,
                                 temperatura=agente.temperatura())
        tablero.publicar_estado("esperando", "Runs and map erased "
                                "(the network is kept). Press START.")
    f_log, w_log = abrir_log(config.RUTA_LOG, config.LOG_ACTIVO)
    paso_global = 0

    # The camera is opened BEFORE the link and closed in the `finally` below.
    # 🔴 It has a live reader thread: if it is not closed, /dev/video0 stays
    # taken and the next start fails with "Device is busy" -- and that
    # already cost half an afternoon once. With --camera off it opens
    # nothing and the robot behaves exactly as without a camera.
    camara = camara_master.CamaraMaster(activa=args.camara,
                                        indice=args.indice_camara)
    # The sensor log (see data_logger.py) is independent of the camera and
    # the link: it starts and closes the same way, whether a run happens or
    # not, so whole sessions can be compared.
    registro = reg_mod.Registro(config.RUTA_REGISTRO_CSV, config.RUTA_REGISTRO_MD,
                                periodo_s=config.REGISTRO_PERIODO_S,
                                informe_cada=config.REGISTRO_INFORME_CADA_N,
                                ventana=config.REGISTRO_VENTANA,
                                activo=config.REGISTRO_ACTIVO)
    try:
        _correr_con_camara(args, agente, el_mapa, tablero, camara, registro,
                           f_log, w_log, limpiar_desde_interfaz)
    finally:
        camara.cerrar()
        registro.cerrar()          # rewrites the report one last time
        if f_log:
            f_log.close()
    print("\n[end] state saved in {}".format(config.RUTA_ESTADO))
    if config.LOG_ACTIVO:
        print("[end] log in {}".format(config.RUTA_LOG))
    if config.REGISTRO_ACTIVO:
        print("[end] sensor log in {}".format(config.RUTA_REGISTRO_CSV))
        print("[end] diagnostic report in {}".format(config.RUTA_REGISTRO_MD))


def _correr_con_camara(args, agente, el_mapa, tablero, camara, registro, f_log, w_log,
                       limpiar_desde_interfaz=None):
    """The body of the run. It is split from correr() only so that closing
    the camera sits in a `finally` that covers EVERYTHING, including a
    ctrl-c."""
    paso_global = 0
    # `params` is created BEFORE opening the link because `on_telemetria`
    # captures it by closure: the logger needs the gains in force on EVERY
    # telemetry line, not only those present at startup.
    params = dict(PARAMS_CALIBRADOS)

    def on_telemetria(telem):
        tablero.publicar_telemetria(telem)
        registro.observar(telem, params)

    # 🔴 Sep 16 afternoon: after a voltage drop (Undervoltage) the USB resets
    # and the ESP re-enumerates (ttyUSB0 -> ttyUSB1). If the requested port
    # does not exist, the first /dev/ttyUSB* available is used.
    import glob
    if not os.path.exists(args.puerto):
        otros = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        if otros:
            print("[init] {} does not exist: using {}".format(args.puerto, otros[0]))
            args.puerto = otros[0]
    with en.EnlaceESP(args.puerto, config.BAUDIOS,
                      timeout_telemetria=config.TIMEOUT_TELEMETRIA_S,
                      verboso=args.verboso,
                      on_telemetria=on_telemetria) as link:

        if not args.sin_interfaz:
            ui.arrancar_en_hilo(tablero, link, params, registro, args.puerto_web)
            print("[init] interface at http://localhost:{}  "
                  "(or http://<pi-ip>:{} from another machine)".format(
                      args.puerto_web, args.puerto_web))

        # The camera preview goes on ITS OWN port: MJPEG leaves a connection
        # open forever and mixing it with the interface status queries is
        # asking for one to block the other.
        if args.camara and not args.sin_interfaz:
            camara_master.arrancar_en_hilo(camara, args.puerto_camara)
            tablero.camara_puerto = args.puerto_camara
            print("[init] camera at http://localhost:{}  "
                  "(what the robot sees, with the three gates of the "
                  "decision)".format(args.puerto_camara))

        nav = nav_mod.Navegador(agente, el_mapa, link, camara=camara,
                                meta=config.META, verboso=args.verboso)
        nav.link_timeout_evento = config.TIMEOUT_EVENTO_S
        nav.link_timeout_giro = config.TIMEOUT_GIRO_S
        # The interface draws the neural network with the snapshot of each
        # decision (see navigator._instantanea_neuro) and the map with the
        # real maze cell.
        nav.on_decision = tablero.publicar_neuro
        tablero.celda_cm = config.CELDA_CM

        telem = link.leer_telemetria(timeout=10.0)
        print("[init] link alive. imu={} yaw={:.1f}".format(
            "ok" if telem.imu_ok else "NO IMU (turns will fail)",
            telem.yaw))

        # 🔴 The gains live in the ESP RAM, and the ESP RESETS every time this
        # main opens the port (pyserial toggles DTR). Everything tuned from
        # the interface goes back to the factory values of the flashed
        # firmware -- on Sep 12 that put signoyaw back to +1 and the robot
        # turned until the timeout. The Pi is the source of truth of the
        # calibrated values: all of them are sent at startup, always.
        for nombre, valor in params.items():
            link.ajustar(nombre, valor)
            time.sleep(0.02)        # one '#k=v' line at a time, without flooding
        print("[init] calibrated gains sent to the ESP (signoyaw={:+.0f})"
              .format(params["signoyaw"]))

        for corrida in range(1, args.corridas + 1):
            if not args.sin_interfaz:
                if not esperar_start(tablero, link, al_limpiar=limpiar_desde_interfaz):
                    break
            nav.preparar_corrida()
            # 🔴 WHEN START IS PRESSED THE ROBOT IS IN THE START CELL FACING
            # "N" (Sep 16 afternoon): the map already puts it there
            # (reiniciar_corrida), but the page kept showing the last cell of
            # the previous run until the first step closed, and the robot
            # seemed to "appear somewhere else". The starting pose is
            # published immediately.
            el_mapa.celda = el_mapa.celda_inicial
            tablero.publicar_mapa(el_mapa)
            tablero.publicar_corrida(
                n=corrida, paso=0, celda=list(el_mapa.celda),
                orientacion=0, heading="N", evento="", motivo="inicio",
                figura="", en_meta=False, recompensa=None,
                modo="explotacion" if agente.modo_explotacion else "exploracion",
                temperatura=agente.temperatura(),
                corridas=agente.corridas, exitos=agente.exitos)
            # The motors start disabled on every ESP power-up; if they stay
            # disabled, every motion command returns NACK and the robot stays
            # still without saying why.
            if not (link.ultima and link.ultima.habilitado):
                link.enviar(en.CMD_MOTORES)
                time.sleep(0.3)

            # 🔴 Cut-off wheels = no run is worth anything. The controller cuts
            # them when the encoder sees them turning opposite to what was
            # requested, and that ALWAYS happens when the polarity or the
            # wiring changes without measuring the directions again ('V').
            # Before, the run started anyway and the robot "did not want to
            # move forward". Here it is reported and re-armed (motors off/on)
            # once; if they are still cut, the directions must be measured.
            t = link.ultima
            cortadas = [n for i, n in enumerate(en.RUEDAS)
                        if t and int(t.campos.get("cortadas", 0)) & (1 << i)]
            if cortadas:
                print("  [warning] wheels cut off by the controller: {}. "
                      "Re-arming; if they are cut again, put the robot on its "
                      "stand and 'Measure directions' (V).".format(", ".join(cortadas)))
                link.enviar(en.CMD_MOTORES); time.sleep(0.2)
                link.enviar(en.CMD_MOTORES); time.sleep(0.3)

            tablero.publicar_estado("corriendo", "Run {} in progress".format(corrida))
            print("\n=== run {}/{} (T={:.2f}, {}) ===".format(
                corrida, args.corridas, agente.temperatura(),
                "explotacion" if agente.modo_explotacion else "exploracion"))
            t0 = time.time()
            exito = False
            parado = False

            for paso in range(1, config.MAX_PASOS_CORRIDA + 1):
                # Interface commands are handled BETWEEN steps: this way none
                # of them lands in the middle of a turn, which is blocking.
                final_manual = False
                for o in tablero.tomar_ordenes():
                    if o.get("accion") == "finished":
                        parado = True
                    if o.get("accion") == "final":
                        final_manual = True
                if final_manual:
                    # "I reached the end of the maze": success declared by hand.
                    exito = True
                    print("  END declared from the interface: the run counts as an exit")
                    break
                if parado or not tablero.quiere_correr():
                    if not args.sin_interfaz:
                        print("  stopped from the interface")
                        parado = True
                        break

                heading_antes = nav.heading
                res = nav.un_paso()
                paso_global += 1
                fila_log(w_log, f_log, paso_global, corrida, paso,
                         heading_antes, res, agente)
                tablero.publicar_mapa(el_mapa)
                tablero.publicar_corrida(
                    n=corrida, paso=paso, celda=list(el_mapa.celda),
                    orientacion=agente.orientacion, heading=nav.heading,
                    evento=en.NOMBRE_EVENTO.get(res.evento, ""),
                    motivo=res.motivo or "", figura=res.figura or "",
                    en_meta=bool(res.en_meta), recompensa=res.recompensa,
                    modo="explotacion" if agente.modo_explotacion else "exploracion",
                    temperatura=agente.temperatura(),
                    corridas=agente.corridas, exitos=agente.exitos)

                if args.verboso or res.evento != en.EV_FIN_CELDA:
                    print("  [{:3d}] {} -> {} ev={} r={} {}".format(
                        paso, res.celda_previa, res.direccion,
                        en.NOMBRE_EVENTO.get(res.evento, res.evento),
                        "-" if res.recompensa is None
                        else "{:+.2f}".format(res.recompensa),
                        res.motivo or ""))
                # If the controller cut wheels in this step, show it on the
                # console right away: it is the number one cause of "it does
                # not move forward".
                t = link.ultima
                if t and int(t.campos.get("cortadas", 0)):
                    print("  [warning] cortadas={} after step {}: the "
                          "encoder sees those wheels reversed -> 'Measure directions'".format(
                              [n for i, n in enumerate(en.RUEDAS)
                               if int(t.campos.get("cortadas", 0)) & (1 << i)], paso))

                if res.direccion is None:
                    print("  run aborted: the cell has no exits")
                    break
                if res.en_meta:
                    exito = True
                    salio = "salida" in (res.motivo or "")
                    print("  {} in {} steps, {:.1f}s".format(
                        "MAZE EXIT (the 4 ToF far away, confirmed)"
                        if salio else "GOAL", paso, time.time()-t0))
                    break
            else:
                # Step cap. Without it the softmax temperature would never
                # drop, because it only drops when fin_corrida() increments
                # self.corridas, and a run that does not end never calls it.
                print("  cut by MAX_PASOS_CORRIDA={}".format(
                    config.MAX_PASOS_CORRIDA))

            for o in tablero.tomar_ordenes():
                if o.get("accion") == "final":
                    exito = True
                    print("  END declared from the interface: the run counts as an exit")
            # 🔴 Stop TWICE, with a pause: the first one may land while the
            # ESP is still braking the confirmation push, and the second one
            # guarantees it leaves manual mode and stays IDLE with setpoint 0.
            link.enviar(en.CMD_PARAR)
            time.sleep(0.2)
            link.enviar(en.CMD_PARAR)
            tablero.poner_correr(False)
            if not parado:
                agente.fin_corrida(exito=exito)
            d = agente.diagnostico()
            tablero.publicar_estado(
                "terminado",
                "Run {} finished: {}. Press START for the next one.".format(
                    corrida, "EXITED THE MAZE" if exito else "did not exit"))
            print("  end: success={} runs={} successes={} best={}".format(
                exito, d.get("corridas"), d.get("exitos"), d.get("mejor_pasos")))
            # 🔑 Every run is archived separately: its log rows and a copy of
            # the learned state (table, master route, patterns) as it was
            # after learning from it.
            try:
                carpeta = os.path.join(_AQUI, "data", "runs")
                os.makedirs(carpeta, exist_ok=True)
                nombre = "{}_run{:02d}_{}_{}steps".format(
                    SESION, corrida, "EXIT" if exito else "no_exit", paso)
                if f_log:
                    f_log.flush()
                    with open(config.RUTA_LOG, "r", encoding="utf-8") as src, open(os.path.join(carpeta, nombre + ".csv"), "w",
                              encoding="utf-8", newline="") as dst:
                        cab = src.readline(); dst.write(cab)
                        for linea in src:
                            campos = linea.rstrip("\r\n").split(",")
                            if (len(campos) >= 3 and campos[1] == str(corrida)
                                    and campos[-1] == SESION):
                                dst.write(linea)
                if os.path.exists(config.RUTA_ESTADO):
                    import shutil
                    shutil.copy(config.RUTA_ESTADO,
                                os.path.join(carpeta, nombre + "_state.json"))
                print("  archived in data/runs/{}".format(nombre))
            except OSError as e:
                print("  [warning] the run could not be archived: {}".format(e))

            if args.sin_interfaz and corrida < args.corridas:
                input("  place the robot at the start and press Enter...")


def main():
    p = argparse.ArgumentParser(description="Raspberry Pi main program of the maze robot")
    p.add_argument("--port", "--puerto", dest="puerto", default=config.PUERTO_SERIE,
                   help="ESP32 serial port (default: %(default)s)")
    p.add_argument("--runs", "--corridas", dest="corridas", type=int, default=1,
                   help="how many runs in a row (default: %(default)s)")
    p.add_argument("--forget", "--olvidar", dest="olvidar", action="store_true",
                   help="erase the maze memory (not the network)")
    p.add_argument("--camera", "--camara", dest="camara", action="store_true",
                   help="enable the camera: the figures OVERRIDE the RL")
    p.add_argument("--camera-index", "--indice-camara", dest="indice_camara", type=int, default=0,
                   help="/dev/video* index (default: %(default)s)")
    p.add_argument("--camera-port", "--puerto-camara", dest="puerto_camara", type=int, default=camara_master.PUERTO_PREVIA,
                   help="preview port (default: %(default)s)")
    p.add_argument("--no-interface", "--sin-interfaz", dest="sin_interfaz", action="store_true",
                   help="do not start the web server; start immediately")
    p.add_argument("--web-port", "--puerto-web", dest="puerto_web", type=int, default=config.PUERTO_WEB,
                   help="interface port (default: %(default)s)")
    p.add_argument("--verbose", "--verboso", dest="verboso", action="store_true",
                   help="print the dialogue with the ESP")
    args = p.parse_args()

    try:
        correr(args)
    except KeyboardInterrupt:
        print("\n[ctrl-c] interrupted by the user")
    except en.ErrorEnlace as e:
        print("\n[LINK] {}".format(e))
        print("  check: USB cable, that no serial monitor is open,")
        print("  and that the firmware is the one in firmware/")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
