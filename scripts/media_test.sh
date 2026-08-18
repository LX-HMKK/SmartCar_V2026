#!/usr/bin/env bash
# Isolated Aurora RGB media check: bash media_test.sh qr|vlm
set -uo pipefail

WORKSPACE=/home/sunrise/ros2_ws
SOURCE_ENV="$WORKSPACE/scripts/source_env.sh"
RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp}
QR_LOG_DIR="$RUNTIME_DIR/smartcar_media"
MEDIA_STATE_DIR=/tmp/smartcar_media
QR_LOG="$QR_LOG_DIR/qr.log"
QR_PROBE_LOG="$QR_LOG_DIR/qr_probe.log"
RGB_LOG="$QR_LOG_DIR/rgb_imshow.log"
OUTPUT_LOG="$QR_LOG_DIR/competition_output.log"

usage() {
  echo "用法: bash /home/sunrise/ros2_ws/scripts/media_test.sh qr|vlm"
}

if [[ $# -ne 1 ]]; then
  usage
  exit 2
fi

if [[ ! -r "$SOURCE_ENV" ]]; then
  echo "缺少 $SOURCE_ENV；先执行 sync_to_rdk.py setup" >&2
  exit 2
fi

# source_env.sh may reference unset TROS variables.
set +u
source "$SOURCE_ENV"
set -u
cd "$WORKSPACE"

configure_display() {
  export DISPLAY="${DISPLAY:-:0}"
  if [[ -n "${XAUTHORITY:-}" && -r "$XAUTHORITY" ]]; then
    return
  fi
  if [[ -r /var/run/lightdm/root/:0 ]]; then
    export XAUTHORITY=/var/run/lightdm/root/:0
  elif [[ -r /root/.Xauthority ]]; then
    export XAUTHORITY=/root/.Xauthority
  fi
}

stop_pid() {
  local pid=${1:-}
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

record_media_pid() {
  local name=$1
  local pid=$2
  printf '%s\n' "$pid" > "$MEDIA_STATE_DIR/$name.pid"
}

clear_media_state() {
  rm -f -- \
    "$MEDIA_STATE_DIR/qr_launch.pid" \
    "$MEDIA_STATE_DIR/vlm_launch.pid" \
    "$MEDIA_STATE_DIR/rgb_imshow.pid" \
    "$MEDIA_STATE_DIR/output_display.pid" \
    "$MEDIA_STATE_DIR/qr_probe.pid"
  rmdir "$MEDIA_STATE_DIR" 2>/dev/null || true
}

start_rgb_imshow() {
  local label=$1
  ros2 run smartcar_tools rgb_imshow --ros-args \
    -p image_topic:=/aurora/rgb/image_raw \
    -p window_title:="Aurora RGB - $label" >"$RGB_LOG" 2>&1 &
  rgb_pid=$!
  record_media_pid rgb_imshow "$rgb_pid"
}

configure_display
mkdir -p "$QR_LOG_DIR"
mkdir -p "$MEDIA_STATE_DIR"

case "$1" in
  qr)
    launch_pid=""
    rgb_pid=""
    output_pid=""
    probe_pid=""
    cleanup_qr() {
      stop_pid "$probe_pid"
      stop_pid "$output_pid"
      stop_pid "$rgb_pid"
      stop_pid "$launch_pid"
      clear_media_state
    }
    trap cleanup_qr EXIT

    # QR subscribes only to Aurora RGB. Do not override Aurora depth, IR, or
    # point-cloud settings here; this isolated check must not alter them.
    : > "$QR_LOG"
    : > "$QR_PROBE_LOG"
    : > "$RGB_LOG"
    : > "$OUTPUT_LOG"
    ros2 launch smartcar_tools qr_test.launch.py \
      input_source:=camera \
      aurora_resolution_mode_index:=2 >"$QR_LOG" 2>&1 &
    launch_pid=$!
    record_media_pid qr_launch "$launch_pid"
    start_rgb_imshow QR
    ros2 run smartcar_tools competition_output_display --ros-args \
      -p output_topic:=/smartcar/output/text \
      -p window_title:="QR 比赛输出" >"$OUTPUT_LOG" 2>&1 &
    output_pid=$!
    record_media_pid output_display "$output_pid"

    # Keep requesting fresh barcode messages while the text UI is open. This
    # lets the operator adjust distance and framing after the first attempt.
    ros2 run smartcar_tools qr_probe \
      --service-wait-sec 15 --timeout-sec 3 --continuous \
      --interval-sec 0.25 --output-topic /smartcar/output/qr \
      >"$QR_PROBE_LOG" 2>&1 &
    probe_pid=$!
    record_media_pid qr_probe "$probe_pid"
    wait "$output_pid" || true
    exit 0
    ;;
  vlm)
    rgb_pid=""
    launch_pid=""
    cleanup_vlm() {
      stop_pid "$rgb_pid"
      stop_pid "$launch_pid"
      clear_media_state
    }
    trap cleanup_vlm EXIT
    # VLM subscribes only to Aurora RGB. Do not override Aurora depth, IR, or
    # point-cloud settings here; this isolated check must not alter them.
    : > "$RGB_LOG"
    start_rgb_imshow VLM
    ros2 launch smartcar_tools vlm_test.launch.py \
      input_source:=camera auto_request:=true &
    launch_pid=$!
    record_media_pid vlm_launch "$launch_pid"
    wait "$launch_pid"
    exit $?
    ;;
  *)
    usage
    exit 2
    ;;
esac
