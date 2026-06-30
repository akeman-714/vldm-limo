#!/usr/bin/env bash
# DGX Spark (GB10 / Blackwell, aarch64) VLM service launcher.
#
# Minimal subset for the ROS2 navigation integration on this box: only the two
# services the value-map loop needs for a COCO target — YOLO26 (detection) and
# SigLIP2 (image-text similarity). GroundingDINO / SAM / BLIP2 are intentionally
# NOT started here (no vlfm_pip env on this box; GDINO needs a CUDA-op build).
#
# torch/.pt mode by default (no TensorRT): siglip2 runs the HF model, yolo26
# loads the .pt. Once Blackwell TensorRT engines are re-exported, point
# YOLO_MODEL at the .engine and set SIGLIP_VISION_ENGINE/SIGLIP_TEXT_* to switch.
#
# Usage:
#   bash scripts/dgx/launch_vlm_servers_dgx.sh [start|stop|status|restart]
set -uo pipefail

CMD="${1:-start}"
REPO_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
CONDA_SH="${CONDA_SH:-$HOME/miniforge3/etc/profile.d/conda.sh}"
GPU_ID="${GPU_ID:-0}"

SIGLIP_ENV="${SIGLIP_ENV:-siglip2_itm}"
YOLO_ENV="${YOLO_ENV:-yolo_trt}"
SIGLIP_PORT="${SIGLIP_PORT:-12182}"
YOLO_PORT="${YOLO_PORT:-12184}"
SIGLIP_MODEL_ID="${SIGLIP_MODEL_ID:-$HOME/siglip2-base-patch16-384}"
YOLO_MODEL="${YOLO_MODEL:-data/yolo26l_960.pt}"
YOLO_IMGSZ="${YOLO_IMGSZ:-960}"

RUN_DIR="${REPO_DIR}/logs/dgx_vlm"
mkdir -p "${RUN_DIR}"

_pidfile() { echo "${RUN_DIR}/$1.pid"; }
_logfile() { echo "${RUN_DIR}/$1.log"; }

_alive() { local f; f="$(_pidfile "$1")"; [ -f "$f" ] && kill -0 "$(cat "$f")" 2>/dev/null; }

_health() {  # name port -> prints up/down
  local code
  code=$(curl -s -o /dev/null --max-time 2 -w '%{http_code}' "http://127.0.0.1:$2/" || true)
  if [ -n "$code" ] && [ "$code" != "000" ]; then echo "up (HTTP $code)"; else echo "DOWN"; fi
}

start_one() {  # name env port "extra-exports" module-args...
  local name="$1" env="$2" port="$3" exports="$4"; shift 4
  if _alive "$name"; then echo "  $name already running (pid $(cat "$(_pidfile "$name")"))"; return; fi
  echo "  starting $name on :$port (env=$env, GPU $GPU_ID)"
  nohup bash -c "
    source '${CONDA_SH}' && conda activate '${env}' \
    && cd '${REPO_DIR}' \
    && export PYTHONPATH='${REPO_DIR}' CUDA_VISIBLE_DEVICES='${GPU_ID}' HF_HUB_OFFLINE=1 ${exports} \
    && exec python -m $*
  " > "$(_logfile "$name")" 2>&1 &
  echo $! > "$(_pidfile "$name")"
}

stop_one() {
  local name="$1" f; f="$(_pidfile "$name")"
  if _alive "$name"; then echo "  stopping $name (pid $(cat "$f"))"; kill "$(cat "$f")" 2>/dev/null; rm -f "$f"
  else echo "  $name not running"; rm -f "$f"; fi
}

case "$CMD" in
  start|restart)
    [ "$CMD" = restart ] && { stop_one siglip2; stop_one yolo26; }
    echo ">>> launching DGX VLM services (torch/.pt mode)"
    # SigLIP2 torch mode: ensure no engine/table env so it loads the HF model.
    start_one siglip2 "$SIGLIP_ENV" "$SIGLIP_PORT" \
      "SIGLIP_MODEL_ID='${SIGLIP_MODEL_ID}' && unset SIGLIP_VISION_ENGINE SIGLIP_TEXT_ENGINE SIGLIP_TEXT_TABLE" \
      "vlfm.vlm.siglip2itm --port ${SIGLIP_PORT}"
    start_one yolo26 "$YOLO_ENV" "$YOLO_PORT" "true" \
      "vlfm.vlm.yolo_trt --port ${YOLO_PORT} --model ${YOLO_MODEL} --imgsz ${YOLO_IMGSZ}"
    echo ">>> waiting for health (model load ~10-20s) ..."
    for _ in $(seq 1 40); do
      s=$(_health siglip2 "$SIGLIP_PORT"); y=$(_health yolo26 "$YOLO_PORT")
      [ "${s%% *}" = up ] && [ "${y%% *}" = up ] && break
      sleep 2
    done
    echo "  siglip2 :$SIGLIP_PORT -> $(_health siglip2 "$SIGLIP_PORT")"
    echo "  yolo26  :$YOLO_PORT -> $(_health yolo26 "$YOLO_PORT")"
    echo ">>> logs: ${RUN_DIR}/{siglip2,yolo26}.log  |  stop: bash $0 stop"
    ;;
  stop) echo ">>> stopping DGX VLM services"; stop_one siglip2; stop_one yolo26 ;;
  status)
    echo "  siglip2 :$SIGLIP_PORT -> $(_health siglip2 "$SIGLIP_PORT") (pidfile $(_alive siglip2 && echo alive || echo none))"
    echo "  yolo26  :$YOLO_PORT -> $(_health yolo26 "$YOLO_PORT") (pidfile $(_alive yolo26 && echo alive || echo none))"
    ;;
  *) echo "usage: $0 [start|stop|status|restart]"; exit 2 ;;
esac
