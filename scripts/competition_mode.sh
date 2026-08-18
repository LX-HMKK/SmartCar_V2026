#!/usr/bin/env bash
# Prepare the semantic competition stack, then trigger it manually at the line.
set -uo pipefail

WORKSPACE=/home/sunrise/ros2_ws
SOURCE_ENV="$WORKSPACE/scripts/source_env.sh"
WAYPOINTS_FILE="$WORKSPACE/src/smartcar_nav2/config/waypoints/default_waypoints.yaml"
VISION_CONFIG="$WORKSPACE/src/smartcar_vision/config/vision.yaml"
VLM_CREDENTIALS_FILE="$WORKSPACE/config/volcengine_ark.local.yaml"
STATUS_TOOL="$WORKSPACE/scripts/nav_status.py"
COMPETITION_LAUNCHER="$WORKSPACE/scripts/competition_launch.py"
COMPETITION_STACK_PROBE="$WORKSPACE/install/smartcar_nav2/bin/competition_stack_probe"
STATE_DIR=/tmp/smartcar_competition
RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp}
LOG_DIR="$RUNTIME_DIR/smartcar_competition"
LAUNCH_LOG="$LOG_DIR/bringup.log"
UI_LOG="$LOG_DIR/output_ui.log"
LAUNCH_PID_FILE="$STATE_DIR/launch.pid"
UI_PID_FILE="$STATE_DIR/output_ui.pid"
QR_READER_MODE_FILE="$STATE_DIR/qr_reader_preloaded"
ARMED_LAUNCH_PID_FILE="$STATE_DIR/armed_launch.pid"
START_LOCK_DIR="$STATE_DIR/start.lock"
UI_START_ENV_MARKER=SMARTCAR_COMPETITION_UI_START_ENV
SCRIPT_STARTED_AT_MS=$(awk '{printf "%d\n", $1 * 1000}' /proc/uptime)
STARTUP_STARTED_AT_MS=$SCRIPT_STARTED_AT_MS

usage() {
  cat <<'EOF'
用法:
  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh prepare [--lazy-qr]
  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh arm
  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh start --confirm
  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh status
  bash /home/sunrise/ros2_ws/scripts/competition_mode.sh stop

prepare 启动比赛栈，并在急停锁存下完成健康检查和 P 点定位复位。
arm 对已准备栈重新执行上述预置；车辆必须已人工放在 P 原点、车头朝 +X。
start 只接受当前启动进程的已预置标记，解除软件急停并触发任务。
EOF
}

startup_mark() {
  local step=$1
  local now_ms elapsed_ms
  now_ms=$(awk '{printf "%d\n", $1 * 1000}' /proc/uptime)
  elapsed_ms=$((now_ms - STARTUP_STARTED_AT_MS))
  printf '[competition-startup] elapsed_ms=%s step=%s\n' \
    "$elapsed_ms" "$step" >> "$LAUNCH_LOG"
}

require_vlm_credentials() {
  [[ -n "${ARK_API_KEY:-}" ]] && return
  [[ -s "$VLM_CREDENTIALS_FILE" ]] && return
  echo "missing ARK_API_KEY and nonempty $VLM_CREDENTIALS_FILE" >&2
  exit 2
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

read_launch_pid() {
  local launch_pid
  if ! pid_is_running "$LAUNCH_PID_FILE"; then
    echo "competition stack is not prepared; run prepare first" >&2
    return 1
  fi
  launch_pid=$(tr -d '[:space:]' < "$LAUNCH_PID_FILE")
  printf '%s\n' "$launch_pid"
}

process_start_ticks() {
  local pid=$1
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/stat" ]] || return 1
  awk '{print $22}' "/proc/$pid/stat"
}

write_armed_launch_identity() {
  local launch_pid=$1
  local start_ticks
  local temporary_file="$ARMED_LAUNCH_PID_FILE.$$.tmp"
  if ! start_ticks=$(process_start_ticks "$launch_pid"); then
    return 1
  fi
  printf '%s %s\n' "$launch_pid" "$start_ticks" > "$temporary_file"
  mv -f "$temporary_file" "$ARMED_LAUNCH_PID_FILE"
}

stack_is_armed() {
  local launch_pid=$1
  local armed_pid armed_start_ticks current_start_ticks
  [[ -r "$ARMED_LAUNCH_PID_FILE" ]] || return 1
  read -r armed_pid armed_start_ticks < "$ARMED_LAUNCH_PID_FILE"
  [[ "$armed_pid" == "$launch_pid" && -n "$armed_start_ticks" ]] || return 1
  if ! current_start_ticks=$(process_start_ticks "$launch_pid"); then
    return 1
  fi
  [[ "$armed_start_ticks" == "$current_start_ticks" ]]
}

release_start_lock() {
  rmdir "$START_LOCK_DIR" 2>/dev/null || true
}

assert_no_competing_ros_stack() {
  local node_list node existing probe_output probe_status
  if [[ -x "$COMPETITION_STACK_PROBE" ]]; then
    probe_output=$(timeout 2s "$COMPETITION_STACK_PROBE" \
      --timeout-sec 0.5 2>&1)
    probe_status=$?
    case "$probe_status" in
      0)
        return
        ;;
      1)
        echo "existing SmartCar stack detected ($probe_output); stop it before prepare" >&2
        exit 1
        ;;
      *)
        # Do not treat a failed graph probe as an empty graph. The established
        # ROS CLI check below remains the conservative fallback.
        echo "competition graph probe unavailable; using ROS CLI fallback" >&2
        ;;
    esac
  fi
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

reset_task_origin() {
  local result
  if ! result=$(timeout 12s ros2 service call /smartcar/task/reset \
      std_srvs/srv/Trigger "{}" 2>&1); then
    echo "$result" >&2
    return 1
  fi
  if [[ "$result" != *"success=True"* ]]; then
    echo "$result" >&2
    return 1
  fi
  return 0
}

verify_stack_health() {
  local launch_pid=$1
  local qr_reader_mode=$2
  local timeout_sec=$3
  local prearm=${4:-false}
  local status_args=(
    --depth-camera --vision-services --timeout "$timeout_sec" \
    --launch-pid "$launch_pid" --timeline-log "$LAUNCH_LOG"
  )
  if [[ "$qr_reader_mode" == true ]]; then
    status_args+=(--preloaded-qr-reader)
  fi
  if [[ "$prearm" == true ]]; then
    status_args+=(--prearm)
  fi
  python3 "$STATUS_TOOL" "${status_args[@]}"
}

arm_prepared_stack() {
  local launch_pid=$1
  local qr_reader_mode=$2
  local timeout_sec=$3
  rm -f "$ARMED_LAUNCH_PID_FILE"
  echo "正在完成赛前健康检查..."
  startup_mark "health_check_started"
  if ! verify_stack_health \
      "$launch_pid" "$qr_reader_mode" "$timeout_sec" true; then
    echo "比赛栈健康检查失败；软件急停保持锁存" >&2
    return 1
  fi
  startup_mark "prearm_ready"
  if ! write_armed_launch_identity "$launch_pid"; then
    echo "无法写入比赛栈预置标记；软件急停保持锁存" >&2
    return 1
  fi
  startup_mark "stack_armed"
  echo "比赛栈已预置：软件急停锁存，按下发车将立即触发任务。"
}

hold_after_start_failure() {
  local reason=$1
  echo "比赛发车失败: $reason" >&2
  if ! set_software_emergency_stop true; then
    echo "无法确认软件急停；请立即使用物理急停" >&2
  fi
  request_task_stop
  release_start_lock
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
  rm -f "$ARMED_LAUNCH_PID_FILE"
  release_start_lock
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
    echo "missing $STATUS_TOOL; run bash $WORKSPACE/scripts/nav_prepare.sh" >&2
    exit 2
  }
  [[ -r "$COMPETITION_LAUNCHER" ]] || {
    echo "missing $COMPETITION_LAUNCHER; run bash $WORKSPACE/scripts/nav_prepare.sh" >&2
    exit 2
  }
  if pid_is_running "$LAUNCH_PID_FILE"; then
    echo "competition stack is already prepared; use status, start, or stop" >&2
    exit 1
  fi
  configure_display
  mkdir -p "$STATE_DIR" "$LOG_DIR"
  : > "$LAUNCH_LOG"
  : > "$UI_LOG"
  startup_mark "prepare_started"
  assert_no_competing_ros_stack
  startup_mark "ros_graph_clear"
  rm -f "$LAUNCH_PID_FILE" "$UI_PID_FILE" "$QR_READER_MODE_FILE" \
    "$ARMED_LAUNCH_PID_FILE"
  release_start_lock

  nohup python3 "$COMPETITION_LAUNCHER" \
    use_base:=true \
    localization_profile:=wheel_imu \
    use_imu_filter:=false use_robot_description:=false \
    use_safety:=true use_nav:=true nav_autostart:=true \
    nav2_lifecycle_manager_delay_sec:=0.0 \
    use_camera:=true use_vision:=true \
    use_depth_camera:=true aurora_rgb_fps:=10 aurora_ir_fps:=10 \
    use_task:=true use_visualization:=false \
    autostart_mission:=false safety_emergency_stop_on_start:=true \
    supervised_competition_mode:=true \
    c_zone_direction:=counterclockwise \
    preload_qr_reader:="$preload_qr" \
    steering_calibrated:=true emergency_stop_ready:=true operator_approved:=true \
    vision_config_file:="$VISION_CONFIG" waypoints_file:="$WAYPOINTS_FILE" \
    >> "$LAUNCH_LOG" 2>&1 &
  local launch_pid=$!
  printf '%s\n' "$launch_pid" > "$LAUNCH_PID_FILE"
  printf '%s\n' "$preload_qr" > "$QR_READER_MODE_FILE"
  startup_mark "launch_spawned"

  if ! arm_prepared_stack "$launch_pid" "$preload_qr" 75; then
    tail -80 "$LAUNCH_LOG" >&2 || true
    echo "系统未完成预置；软件急停保持锁存" >&2
    exit 1
  fi

  # The UI process inherits this fully sourced ROS environment. Its start
  # child passes the marker back to this script, avoiding a second 4-5 second
  # source_env.sh pass after the referee presses the button.
  nohup env "$UI_START_ENV_MARKER=1" \
    ros2 run smartcar_tools competition_output_display --ros-args \
    -p fullscreen:=true -p window_title:="比赛输出" \
    -p initial_state:="预置完成，急停锁存，等待远程发车" \
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
  startup_mark "ui_ready"

  echo "比赛栈已预置：急停锁存，车辆不会自动发车。"
  echo "裁判发令后，在比赛输出 UI 单击“发车”。"
}

arm() {
  if [[ $# -ne 0 ]]; then
    usage >&2
    exit 2
  fi
  require_vlm_credentials
  local launch_pid qr_reader_mode
  if ! launch_pid=$(read_launch_pid); then
    exit 1
  fi
  if ! qr_reader_mode=$(read_preloaded_qr_reader_mode); then
    exit 1
  fi
  arm_prepared_stack "$launch_pid" "$qr_reader_mode" 15
}

start() {
  if [[ $# -ne 1 || "$1" != "--confirm" ]]; then
    echo "start requires the explicit --confirm acknowledgement" >&2
    exit 2
  fi
  local launch_pid
  if ! launch_pid=$(read_launch_pid); then
    exit 1
  fi
  if ! stack_is_armed "$launch_pid"; then
    echo "competition stack is not pre-armed; place the vehicle at P and run arm" >&2
    exit 1
  fi
  if ! pid_is_running "$UI_PID_FILE"; then
    echo "competition output UI is not running; aborting before motion" >&2
    exit 1
  fi
  if ! mkdir "$START_LOCK_DIR" 2>/dev/null; then
    echo "competition start is already in progress" >&2
    exit 1
  fi
  startup_mark "start_requested"
  trap 'handle_start_interruption INT' INT
  trap 'handle_start_interruption TERM' TERM
  trap 'handle_start_interruption HUP' HUP
  echo "正在解除软件急停..."
  if ! set_software_emergency_stop false; then
    hold_after_start_failure "解除软件急停失败"
  fi
  startup_mark "software_stop_released"
  echo "正在触发比赛任务..."
  local result
  if ! result=$(timeout 12s ros2 service call /smartcar/task/start \
      std_srvs/srv/Trigger "{}" 2>&1); then
    echo "$result" >&2
    rm -f "$ARMED_LAUNCH_PID_FILE"
    hold_after_start_failure "任务 Trigger 调用失败"
  fi
  if [[ "$result" != *"success=True"* ]]; then
    echo "$result" >&2
    rm -f "$ARMED_LAUNCH_PID_FILE"
    hold_after_start_failure "任务未获准启动"
  fi
  startup_mark "task_trigger_accepted"
  rm -f "$ARMED_LAUNCH_PID_FILE"
  release_start_lock
  trap - INT TERM HUP
  echo "比赛任务已启动。"
}

status() {
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
  rm -f "$ARMED_LAUNCH_PID_FILE"
  release_start_lock
  bash "$WORKSPACE/scripts/ros_cleanup.sh"
}

if [[ "${SMARTCAR_COMPETITION_UI_START_ENV:-}" != "1" ]]; then
  if [[ ! -r "$SOURCE_ENV" ]]; then
    echo "missing $SOURCE_ENV; run sync_to_rdk.py setup" >&2
    exit 2
  fi
  export SMARTCAR_COMPETITION_FAST_ENV=1
  set +u
  source "$SOURCE_ENV"
  set -u
elif ! command -v ros2 >/dev/null 2>&1 || [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "UI start did not inherit the prepared ROS environment" >&2
  exit 2
fi
cd "$WORKSPACE"

case "${1:-}" in
  prepare)
    shift
    prepare "$@"
    ;;
  arm)
    shift
    arm "$@"
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
