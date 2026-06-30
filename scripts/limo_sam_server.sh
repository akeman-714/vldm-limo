#!/usr/bin/env bash
# MobileSAM server (port 12183) for the Limo/Nav2 G2/G3 object-navigate path.
#
# Runs in the vlfm_ros312 venv. That venv's torch is CPU-only, so SAM runs on
# CPU here (fine for acceptance; for speed host it in a CUDA env instead).
#
# ONE-TIME install:
#   source /home/asong/vlfm/scripts/source_limo_ros_env.sh
#   pip install --no-deps "git+https://github.com/ChaoningZhang/MobileSAM.git" timm
#
# Usage: bash scripts/limo_sam_server.sh [start|stop|status]
set -uo pipefail

CMD="${1:-start}"
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENV="${VLFM_ROS_VENV:-/home/asong/venvs/vlfm_ros312}"
PORT="${SAM_PORT:-12183}"
export MOBILE_SAM_CHECKPOINT="${MOBILE_SAM_CHECKPOINT:-${REPO_DIR}/data/mobile_sam.pt}"
RUN_DIR="${REPO_DIR}/logs/dgx_vlm"; mkdir -p "$RUN_DIR"
PIDF="${RUN_DIR}/mobile_sam.pid"; LOGF="${RUN_DIR}/mobile_sam.log"

_alive() { [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null; }
_health() {
  local code; code=$(curl -s -o /dev/null --max-time 2 -w '%{http_code}' "http://127.0.0.1:${PORT}/" || true)
  [ -n "$code" ] && [ "$code" != "000" ] && echo "up (HTTP $code)" || echo "DOWN"
}

case "$CMD" in
  start)
    if _alive; then echo "mobile_sam already running (pid $(cat "$PIDF"))"; exit 0; fi
    if [ ! -f "${MOBILE_SAM_CHECKPOINT}" ]; then echo "missing checkpoint: ${MOBILE_SAM_CHECKPOINT}"; exit 1; fi
    if ! "${VENV}/bin/python" -c "import mobile_sam" 2>/dev/null; then
      echo "mobile_sam not installed in ${VENV}. One-time install (run yourself):"
      echo "  source ${REPO_DIR}/scripts/source_limo_ros_env.sh"
      echo "  pip install --no-deps \"git+https://github.com/ChaoningZhang/MobileSAM.git\" timm"
      exit 1
    fi
    echo "starting mobile_sam on :${PORT} (CPU, venv=${VENV})"
    PYTHONPATH="${REPO_DIR}" MOBILE_SAM_CHECKPOINT="${MOBILE_SAM_CHECKPOINT}" \
      setsid nohup "${VENV}/bin/python" -m vlfm.vlm.sam --port "${PORT}" > "${LOGF}" 2>&1 < /dev/null &
    echo $! > "$PIDF"
    for _ in $(seq 1 30); do [ "$(_health)" != DOWN ] && break; sleep 2; done
    echo "  :${PORT} -> $(_health)   (log: ${LOGF})"
    ;;
  stop)
    if _alive; then echo "stopping mobile_sam (pid $(cat "$PIDF"))"; kill "$(cat "$PIDF")" 2>/dev/null; fi
    rm -f "$PIDF"
    ;;
  status) echo "  mobile_sam :${PORT} -> $(_health) (pidfile $(_alive && echo alive || echo none))" ;;
  *) echo "usage: $0 [start|stop|status]"; exit 2 ;;
esac
