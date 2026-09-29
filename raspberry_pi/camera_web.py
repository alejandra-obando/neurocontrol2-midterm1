#!/usr/bin/env python3
"""View the Pi camera from the PC, in the browser.

    python3 camera_web.py                 serves at http://<pi-ip>:8000
    python3 camera_web.py --port 8080

It exists because `test_camera.py` opens its window with cv2.imshow, and
that window shows up on the Pi screen: over SSH there is no display. Instead
of fighting with X11, the Pi sends the already drawn frames as MJPEG and any
browser can show them. The detection is EXACTLY the same: it is imported
from test_camera, nothing is reimplemented here.

A single thread reads the camera and keeps the last frame; the browsers that
connect copy that one. If each client read on its own, two open tabs would
fight over /dev/video0.
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from test_camera import (AREA_MINIMA, FRAMES_ESTABLES, CamaraReal,
                           DetectorFiguras, LectorEstable, dibujar)

PAGINA = b"""<!doctype html><meta charset=utf-8>
<title>pi camera</title>
<style>body{margin:0;background:#111;display:grid;place-items:center;
height:100vh}img{max-width:100%;image-rendering:pixelated}</style>
<img src="/video.mjpg">
"""


class Vigia:
    """Reads the camera in its own thread and keeps the last drawn frame."""

    def __init__(self, args):
        self.cam = CamaraReal(args.camara, args.ancho, args.alto)
        self.lector = LectorEstable(
            DetectorFiguras(area_minima=args.area_minima), args.estables)
        self.jpeg = None
        self.lock = threading.Lock()
        self.vivo = True
        threading.Thread(target=self._bucle, daemon=True).start()

    def _bucle(self):
        ultimo_aviso = 0.0
        while self.vivo:
            frame = self.cam.leer()
            if frame is None:
                time.sleep(0.01)
                continue
            estable = self.lector.leer(frame)
            d = self.lector.deteccion
            # The console is still useful: the browser may be closed.
            ahora = time.time()
            if estable or (d and ahora - ultimo_aviso > 0.5):
                ultimo_aviso = ahora
                print("{}  order={}".format(d, estable), flush=True)
            if estable:
                self.lector.reiniciar()
            ok, buf = cv2.imencode(".jpg", dibujar(frame, d, estable),
                                   [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()

    def ultimo(self):
        with self.lock:
            return self.jpeg

    def cerrar(self):
        self.vivo = False
        time.sleep(0.1)
        self.cam.cerrar()


def hacer_handler(vigia):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass                    # no noise: the console is for the figures

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(PAGINA)))
                self.end_headers()
                self.wfile.write(PAGINA)
                return
            if self.path != "/video.mjpg":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=cuadro")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    jpeg = vigia.ultimo()
                    if jpeg is None:
                        time.sleep(0.02)
                        continue
                    self.wfile.write(b"--cuadro\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        ("Content-Length: %d\r\n\r\n" % len(jpeg)).encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.04)        # ~25 fps cap
            except (BrokenPipeError, ConnectionResetError):
                pass                # the tab was closed, it is not an error
    return Handler


def main():
    p = argparse.ArgumentParser(description="Pi camera in the browser")
    p.add_argument("--camera", "--camara", dest="camara", type=int, default=0)
    p.add_argument("--width", "--ancho", dest="ancho", type=int, default=640)
    p.add_argument("--height", "--alto", dest="alto", type=int, default=480)
    p.add_argument("--port", "--puerto", dest="puerto", type=int, default=8000)
    p.add_argument("--min-area", "--area-minima", dest="area_minima", type=float, default=AREA_MINIMA)
    p.add_argument("--stable", "--estables", dest="estables", type=int, default=FRAMES_ESTABLES)
    args = p.parse_args()

    vigia = Vigia(args)
    servidor = ThreadingHTTPServer(("0.0.0.0", args.puerto),
                                   hacer_handler(vigia))
    print("open http://<pi-ip>:{} on the PC. ctrl-c to quit"
          .format(args.puerto), flush=True)
    try:
        servidor.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        vigia.cerrar()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
