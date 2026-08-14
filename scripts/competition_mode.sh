#!/usr/bin/env bash
# Prepare the semantic competition stack, then trigger it manually at the line.
set -uo pipefail

SOURCE_ENV=/root/source_env.sh
WORKSPACE=/home/sunrise/ros2_ws
WAYPOINTS_FILE="$WORKSPACE/src/smartcar_nav2/config/waypoints/default_waypoints.yaml"
VISION_CONFIG="$WORKSPACE/src/smartcar_vision/config/vision_volcengine.yaml"
VLM_CREDENTIALS_FILE="$WORKSPACE/config/volcengine_ark.local.yaml"
STATUS_TOOL=/root/nav_status.py
BUILD_FINGERPRINT="$WORKSPACE/.smartcar_nav_prepare_fingerprint"
STATE_DIR=/tmp/smartcar_competition
RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp}
LOG_DIR="$RUNTIME_DIR/smartcar_competition"
LAUNCH_LOG="$LOG_DIR/bringup.log"
UI_LOG="$LOG_DIR/output_ui.log"
LAUNCH_PID_FILE="$STATE_DIR/launch.pid"
UI_PID_FILE="$STATE_DIR/output_ui.pid"
QR_READER_MODE_FILE="$STATE_DIR/qr_reader_preloaded"

usage() {
  cat <<'EOF'
用法:
  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh prepare [--lazy-qr]
  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh start --confirm
  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh status
  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh stop

prepare 只启动 Aurora、深度避障、Nav2、QR/VLM 服务和比赛输出 UI，并锁存软件急停。
start 只在已准备的栈上执行定位复位、解除软件急停和任务 Trigger。
EOF
}

source_fingerprint() {
  find "$WORKSPACE/src" \
    -type f \
    ! -path '*/.git/*' \
    ! -path '*/__pycache__/*' \
    ! -name '*.pyc' \
    ! -name '*.bak' \
    ! -name '*.orig' \
    -print0 \
    | sort -z \
    | xargs -0r sha256sum \
    | sha256sum \
    | awk '{print $1}'
}

require_prepared_workspace() {
  local prepared current
  if [[ ! -r "$BUILD_FINGERPRINT" ]]; then
    echo "missing $BUILD_FINGERPRINT; run /root/nav_prepare.sh" >&2
    exit 2
  fi
  prepared=$(tr -d '[:space:]' < "$BUILD_FINGERPRINT")
  current=$(source_fingerprint)
  if [[ -z "$prepared" || "$prepared" != "$current" ]]; then
    echo "RDK source differs from the prepared build; run /root/nav_prepare.sh" >&2
    exit 2
  fi
}

require_vlm_credentials() {
  if [[ ! -s "$VLM_CREDENTIALS_FILE" ]]; then
    echo "missing nonempty $VLM_CREDENTIALS_FILE; configure the RDK-local VLM credential" >&2
    exit 2
  fi
}

configure_display() {
  local desktop_session desktop_user desktop_home
  export DISPLAY="${DISPLAY:-:0}"
  if [[ -n "${XAUTHORITY:-}" && -r "$XAUTHORITY" ]]; then
    return
  fi
  if [[ "$DISPLAY" == :0 ]] && command -v loginctl >/dev/null 2>&1; then
    desktop_session=$(loginctl show-seat seat0 -p ActiveSession --value \
      2>/dev/null || true)
    desktop_user=$(loginctl show-session "$desktop_session" -p Name --value \
      2>/dev/null || true)
    desktop_home=$(getent passwd "$desktop_user" 2>/dev/null | cut -d: -f6)
    if [[ -n "$desktop_home" && -r "$desktop_home/.Xauthority" ]]; then
      export XAUTHORITY="$desktop_home/.Xauthority"
      return
    fi
  fi
  if [[ -r /var/run/lightdm/root/:0 ]]; then
    export XAUTHORITY=/var/run/lightdm/root/:0
  elif [[ -r /root/.Xauthority ]]; then
    export XAUTHORITY=/root/.Xauthority
  fi
}

pid_is_running() {
  local pid_file=$1
  local pid
  [[ -r "$pid_file" ]] || return 1
  pid=$(tr -d '[:space:]' < "$pid_file")
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" 2>/dev/null
}

read_preloaded_qr_reader_mode() {
  local mode
  if [[ ! -r "$QR_READER_MODE_FILE" ]]; then
    echo "missing $QR_READER_MODE_FILE; prepare the competition stack again" >&2
    return 1
  fi
  mode=$(tr -d '[:space:]' < "$QR_READER_MODE_FILE")
  case "$mode" in
    true|false)
      printf '%s\n' "$mode"
      ;;
    *)
      echo "invalid QR reader mode in $QR_READER_MODE_FILE" >&2
      return 1
      ;;
  esac
}

assert_no_competing_ros_stack() {
  local node_list node existing
  if ! node_list=$(timeout 5s ros2 node list 2>/dev/null); then
    echo "cannot inspect the ROS graph; refuse to prepare over an unknown stack" >&2
    exit 1
  fi
  existing=""
  while IFS= read -r node; do
    case "$node" in
      /task_node|/safety_node|/safety_node_cpp|/direction_guard|\
      /direction_guard_node|/origincar_base|/ekf_filter_node|\
      /controller_server|/planner_server|/behavior_server|/bt_navigator|\
      /smoother_server|/velocity_smoother|/lifecycle_manager_navigation|\
      /lifecycle_manager|/aurora930_node|/depth_pointcloud_relay|\
      /depth_pointcloud_to_laserscan|/vision_node|/barcode_reader|\
      /competition_output_display)
        existing+="${existing:+,}$node"
        ;;
    esac
  done <<< "$node_list"
  if [[ -n "$existing" ]]; then
    echo "existing SmartCar stack detected ($existing); stop it before prepare" >&2
    exit 1
  fi
}

set_software_emergency_stop() {
  local enabled=$1
  local result
  if ! result=$(timeout 12s ros2 service call \
      /smartcar/safety/emergency_stop std_srvs/srv/SetBool \
      "{data: $enabled}" 2>&1); then
    echo "$result" >&2
    return 1
  fi
  if [[ "$result" != *"success=True"* ]]; then
    echo "$result" >&2
    return 1
  fi
  return 0
}

request_task_stop() {
  timeout 5s ros2 service call /smartcar/task/stop std_srvs/srv/Trigger \
    "{}" >/dev/null 2>&1 || true
}

hold_after_start_failure() {
  local reason=$1
  echo "比赛发车失败: $reason" >&2
  if ! set_software_emergency_stop true; then
    echo "无法确认软件急停；请立即使用物理急停" >&2
  fi
  request_task_stop
  exit 1
}

handle_start_interruption() {
  local signal=$1
  trap - INT TERM HUP
  echo "比赛发车被 $signal 中断；正在重新锁存软件急停" >&2
  if ! set_software_emergency_stop true; then
    echo "无法确认软件急停；请立即使用物理急停" >&2
  fi
  # The Trigger request may have reached task_node before its response was
  # interrupted. Cancel the pending mission after the command path is latched.
  request_task_stop
  exit 128
}

prepare() {
  local preload_qr=true
  if [[ $# -eq 1 ]]; then
    if [[ "$1" != "--lazy-qr" ]]; then
      usage >&2
      exit 2
    fi
    preload_qr=false
  elif [[ $# -ne 0 ]]; then
    usage >&2
    exit 2
  fi

  require_prepared_workspace
  [[ -r "$WAYPOINTS_FILE" ]] || {
    echo "missing $WAYPOINTS_FILE" >&2
    exit 2
  }
  [[ -r "$VISION_CONFIG" ]] || {
    echo "missing $VISION_CONFIG" >&2
    exit 2
  }
  require_vlm_credentials
  [[ -x "$STATUS_TOOL" ]] || {
    echo "missing $STATUS_TOOL; run /root/nav_prepare.sh" >&2
    exit 2
  }
  if pid_is_running "$LAUNCH_PID_FILE"; then
    echo "competition stack is already prepared; use status, start, or stop" >&2
    exit 1
  fi
  assert_no_competing_ros_stack

  configure_display
  mkdir -p "$STATE_DIR" "$LOG_DIR"
  : > "$LAUNCH_LOG"
  : > "$UI_LOG"
  rm -f "$LAUNCH_PID_FILE" "$UI_PID_FILE" "$QR_READER_MODE_FILE"

  nohup ros2 launch smartcar_bringup smartcar_system.launch.py \
    use_base:=true use_lidar:=false use_laser_odometry:=false \
    localization_profile:=wheel_imu \
    use_imu_filter:=false use_robot_description:=false \
    use_safety:=true use_nav:=true nav_autostart:=true \
    use_camera:=true use_vision:=true camera_driver:=aurora \
    use_depth_camera:=true aurora_rgb_fps:=10 aurora_ir_fps:=10 \
    use_task:=true use_visualization:=false use_speech:=false \
    autostart_mission:=false safety_emergency_stop_on_start:=true \
    supervised_competition_mode:=true continue_after_qr_failure:=true \
    c_zone_direction:=counterclockwise \
    preload_qr_reader:="$preload_qr" \
    waypoints_calibrated:=true extrinsics_calibrated:=true \
    steering_calibrated:=true emergency_stop_ready:=true operator_approved:=true \
    vision_config_file:="$VISION_CONFIG" waypoints_file:="$WAYPOINTS_FILE" \
    >> "$LAUNCH_LOG" 2>&1 &
  local launch_pid=$!
  printf '%s\n' "$launch_pid" > "$LAUNCH_PID_FILE"
  printf '%s\n' "$preload_qr" > "$QR_READER_MODE_FILE"

  local status_args=(
    --depth-camera --vision-services --timeout 75 --launch-pid "$launch_pid"
  )
  if [[ "$preload_qr" == true ]]; then
    status_args+=(--preloaded-qr-reader)
  fi
  if ! python3 "$STATUS_TOOL" "${status_args[@]}"; then
    tail -80 "$LAUNCH_LOG" >&2 || true
    echo "系统未就绪；软件急停保持锁存" >&2
    exit 1
  fi
  if ! set_software_emergency_stop true; then
    tail -80 "$LAUNCH_LOG" >&2 || true
    echo "无法确认软件急停已锁存；请立即使用物理急停" >&2
    exit 1
  fi

  nohup ros2 run smartcar_tools competition_output_display --ros-args \
    -p fullscreen:=true -p window_title:="比赛输出" \
    -p initial_state:="急停锁存，等待远程发车" \
    -p remote_start_enabled:=true \
    -p remote_start_command:="$WORKSPACE/scripts/competition_mode.sh" \
    > "$UI_LOG" 2>&1 &
  local ui_pid=$!
  printf '%s\n' "$ui_pid" > "$UI_PID_FILE"
  sleep 0.2
  if ! kill -0 "$ui_pid" 2>/dev/null; then
    tail -40 "$UI_LOG" >&2 || true
    echo "比赛输出 UI 启动失败；软件急停保持锁存" >&2
    exit 1
  fi

  echo "比赛栈已就绪：急停锁存，车辆不会自动发车。"
  echo "裁判发令后，在确认车辆位于 P 原点、车头朝 +X 且物理急停可用时执行："
  echo "  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh start --confirm"
}

start() {
  if [[ $# -ne 1 || "$1" != "--confirm" ]]; then
    echo "start requires the explicit --confirm acknowledgement" >&2
    exit 2
  fi
  require_prepared_workspace
  require_vlm_credentials
  if ! pid_is_running "$LAUNCH_PID_FILE"; then
    echo "competition stack is not prepared; run prepare first" >&2
    exit 1
  fi
  if ! pid_is_running "$UI_PID_FILE"; then
    echo "competition output UI is not running; aborting before motion" >&2
    exit 1
  fi
  local launch_pid qr_reader_mode
  launch_pid=$(tr -d '[:space:]' < "$LAUNCH_PID_FILE")
  if ! qr_reader_mode=$(read_preloaded_qr_reader_mode); then
    exit 1
  fi
  local status_args=(
    --depth-camera --vision-services --timeout 15 --launch-pid "$launch_pid"
  )
  if [[ "$qr_reader_mode" == true ]]; then
    status_args+=(--preloaded-qr-reader)
  fi
  if ! python3 "$STATUS_TOOL" "${status_args[@]}"; then
    echo "competition stack health check failed; software emergency stop remains latched" >&2
    exit 1
  fi

  local result
  if ! result=$(timeout 12s ros2 service call /smartcar/task/reset \
      std_srvs/srv/Trigger "{}" 2>&1); then
    echo "$result" >&2
    exit 1
  fi
  if [[ "$result" != *"success=True"* ]]; then
    echo "$result" >&2
    exit 1
  fi
  trap 'handle_start_interruption INT' INT
  trap 'handle_start_interruption TERM' TERM
  trap 'handle_start_interruption HUP' HUP
  if ! set_software_emergency_stop false; then
    hold_after_start_failure "解除软件急停失败"
  fi
  if ! result=$(timeout 12s ros2 service call /smartcar/task/start \
      std_srvs/srv/Trigger "{}" 2>&1); then
    echo "$result" >&2
    hold_after_start_failure "任务 Trigger 调用失败"
  fi
  if [[ "$result" != *"success=True"* ]]; then
    echo "$result" >&2
    hold_after_start_failure "任务未获准启动"
  fi
  trap - INT TERM HUP
  echo "比赛任务已启动。"
}

status() {
  require_prepared_workspace
  if ! pid_is_running "$LAUNCH_PID_FILE"; then
    echo "competition stack is not running" >&2
    exit 1
  fi
  local launch_pid qr_reader_mode
  launch_pid=$(tr -d '[:space:]' < "$LAUNCH_PID_FILE")
  if ! qr_reader_mode=$(read_preloaded_qr_reader_mode); then
    exit 1
  fi
  local status_args=(
    --depth-camera --vision-services --timeout 5 --launch-pid "$launch_pid"
  )
  if [[ "$qr_reader_mode" == true ]]; then
    status_args+=(--preloaded-qr-reader)
  fi
  python3 "$STATUS_TOOL" "${status_args[@]}"
}

stop() {
  set_software_emergency_stop true || true
  bash "$WORKSPACE/scripts/ros_cleanup.sh"
}

if [[ ! -r "$SOURCE_ENV" ]]; then
  echo "missing $SOURCE_ENV; run sync_to_rdk.py setup" >&2
  exit 2
fi
set +u
source "$SOURCE_ENV"
set -u
cd "$WORKSPACE"

case "${1:-}" in
  prepare)
    shift
    prepare "$@"
    ;;
  start)
    shift
    start "$@"
    ;;
  status)
    shift
    [[ $# -eq 0 ]] || { usage >&2; exit 2; }
    status
    ;;
  stop)
    shift
    [[ $# -eq 0 ]] || { usage >&2; exit 2; }
    stop
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
