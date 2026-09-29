#!/usr/bin/env bash
# Uploads the web interface and the Pi-side code that feeds it, and restarts
# the server on the Raspberry Pi.
#
#   PI=user@<pi-ip> ./tools/deploy_interface.sh
#
# Run it from a machine with passwordless SSH access to the Pi (key-based).
# The Pi keeps the same layout as this repository: ~/raspberry_pi/...
set -e
PI="${PI:?set PI=user@<pi-ip>}"
REMOTE_DIR="${REMOTE_DIR:-~/raspberry_pi}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE/raspberry_pi"

echo "== 1. files"
ssh "$PI" "mkdir -p $REMOTE_DIR/web $REMOTE_DIR/data"
scp *.py q_network_weights.h "$PI:$REMOTE_DIR/"
scp web/index.html web/app.js web/style.css web/simulator.js "$PI:$REMOTE_DIR/web/"

echo "== 2. stop the server (by anchored PID, not pkill -f)"
ssh "$PI" 'for p in $(pgrep -f "^python3 (-u )?main[.]py"); do kill $p; done; sleep 4; for p in $(pgrep -f "^python3 (-u )?main[.]py"); do kill -9 $p; done; fuser /dev/ttyUSB0 || true'

echo "== 3. restart (with camera and preview on :8081)"
ssh "$PI" "cd $REMOTE_DIR && setsid nohup python3 main.py --port /dev/ttyUSB0 --web-port 8080 --camera > /tmp/server.log 2>&1 < /dev/null & sleep 3; curl -s http://localhost:8080/estado | head -c 200; echo"
echo "== done: http://<pi-ip>:8080"
