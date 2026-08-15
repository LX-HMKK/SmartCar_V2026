#!/bin/bash
# ============================================================
# SmartCar 纯导航安全测试（DUBIN 全正向路线 + 方向门）
# 用法: bash /home/sunrise/ros2_ws/scripts/nav_test.sh [--go] [--wheel-only] [--cw]
# --cw: C 区使用顺时针镜像路线；也兼容 --c-zone-direction=clockwise。
# --go: 健康检查后复位并发车；默认只启动并保持急停。
# ============================================================
set -uo pipefail

WORKSPACE=/home/sunrise/ros2_ws
SOURCE_ENV="$WORKSPACE/scripts/source_env.sh"
WP="$WORKSPACE/src/smartcar_nav2/config/waypoints/nav_only.yaml"
STATUS_TOOL="$WORKSPACE/scripts/nav_status.py"
BUILD_FINGERPRINT="$WORKSPACE/.smartcar_nav_prepare_fingerprint"
STATE_DIR=/tmp/smartcar_nav
RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp}
LOG_DIR="$RUNTIME_DIR/smartcar_nav"
LOG="$LOG_DIR/bringup.log"
RVIZ_LOG="$LOG_DIR/rviz.log"
LAUNCH_PID_FILE="$STATE_DIR/launch.pid"
RVIZ_PID_FILE="$STATE_DIR/rviz.pid"
LAUNCH_PID=""
RVIZ_PID=""

# TROS humble setup.bash has unbound AMENT_TRACE_SETUP_FILES — disable
# nounset around the source to avoid script exit under set -uo pipefail.
set +u
source "$SOURCE_ENV"
set -u
cd "$WORKSPACE"

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
    echo "missing $BUILD_FINGERPRINT; run bash $WORKSPACE/scripts/nav_prepare.sh"
    exit 2
  fi
  prepared=$(tr -d '[:space:]' < "$BUILD_FINGERPRINT")
  current=$(source_fingerprint)
  if [[ -z "$prepared" || "$prepared" != "$current" ]]; then
    echo "RDK source differs from the prepared build; run bash $WORKSPACE/scripts/nav_prepare.sh"
    exit 2
  fi
}

# Do not launch against stale install artifacts after a source-only sync.
require_prepared_workspace

# ---- 参数解析 ----
GO=false
WHEEL_ONLY=false
C_ZONE_DIRECTION=counterclockwise
for arg in "$@"; do
  case "$arg" in
    --go) GO=true ;;
    --wheel-only) WHEEL_ONLY=true ;;
    --cw) C_ZONE_DIRECTION=clockwise ;;
    --c-zone-direction)
      echo "请使用 --c-zone-direction=clockwise 或 --c-zone-direction=counterclockwise"
      exit 1
      ;;
    --c-zone-direction=clockwise) C_ZONE_DIRECTION=clockwise ;;
    --c-zone-direction=counterclockwise) C_ZONE_DIRECTION=counterclockwise ;;
    *)
      echo "未知选项: $arg"
      echo "用法: bash /home/sunrise/ros2_ws/scripts/nav_test.sh [--go] [--wheel-only] [--cw]"
      exit 1
      ;;
  esac
done

# Use the active seat0 desktop session's X11 credentials. This also covers a
# root SSH shell: root's own cookie is not authorized for sunrise's desktop.
if [[ -z "${DISPLAY:-}" ]]; then
  export DISPLAY=:0
fi
if [[ "$DISPLAY" == :0 ]] && command -v loginctl >/dev/null 2>&1; then
  DESKTOP_SESSION=$(loginctl show-seat seat0 -p ActiveSession --value 2>/dev/null || true)
  DESKTOP_USER=$(loginctl show-session "$DESKTOP_SESSION" -p Name --value 2>/dev/null || true)
  DESKTOP_HOME=$(getent passwd "$DESKTOP_USER" 2>/dev/null | cut -d: -f6)
  if [[ -n "$DESKTOP_HOME" && -r "$DESKTOP_HOME/.Xauthority" ]]; then
    export XAUTHORITY="$DESKTOP_HOME/.Xauthority"
  fi
fi
if [[ -z "${XAUTHORITY:-}" || ! -r "$XAUTHORITY" ]]; then
  if [[ -r "$HOME/.Xauthority" ]]; then
    export XAUTHORITY="$HOME/.Xauthority"
  elif [[ -r /var/run/lightdm/root/:0 ]]; then
    export XAUTHORITY=/var/run/lightdm/root/:0
  fi
fi

banner() { echo ""; echo "=== $* ==="; }

hold_started_stack() {
  echo "✗ $*"
  echo "  急停保持锁存；已启动的 ROS 栈和 RViz 保留供现场诊断。"
  exit 1
}

set_software_emergency_stop() {
  local enabled=$1
  local result
  if ! result=$(timeout 12s ros2 service call \
      /smartcar/safety/emergency_stop std_srvs/srv/SetBool \
      "{data: $enabled}" 2>&1); then
    echo "$result"
    return 1
  fi
  if [[ "$result" != *"success=True"* ]]; then
    echo "$result"
    return 1
  fi
  echo "  ✓ 软件急停 data=$enabled"
  return 0
}

re_latch_after_transition_failure() {
  local reason=$1
  echo "✗ $reason"
  echo "  正在重新锁存软件急停..."
  if ! set_software_emergency_stop true; then
    echo "  ✗ 无法确认软件急停已重新锁存；立即使用物理急停"
  fi
  hold_started_stack "运动转换失败；急停保持锁存"
}

start_rviz() {
  nohup rviz2 -d "$WORKSPACE/src/smartcar_tools/rviz/navigation.rviz" \
    >"$RVIZ_LOG" 2>&1 &
  RVIZ_PID=$!
  printf '%s\n' "$RVIZ_PID" > "$RVIZ_PID_FILE"
  sleep 0.2
  kill -0 "$RVIZ_PID" 2>/dev/null || hold_started_stack "RViz launch failed"
  echo "  ✓ RViz PID: $RVIZ_PID"
}

# nav_prepare.sh owns cleanup and build. This entry verifies that preparation,
# then starts the prepared stack, checks it, and optionally starts the route.
test -x "$STATUS_TOOL" || {
  echo "missing $STATUS_TOOL; run bash $WORKSPACE/scripts/nav_prepare.sh"
  exit 2
}

EXTRA_ARGS="autostart_mission:=false safety_emergency_stop_on_start:=true"
LOCALIZATION_PROFILE_ARG="localization_profile:=wheel_imu"
C_ZONE_DIRECTION_ARG="c_zone_direction:=$C_ZONE_DIRECTION"
if $WHEEL_ONLY; then
  LOCALIZATION_PROFILE_ARG="localization_profile:=wheel_only"
fi
BANNER_MSG="DUBIN + Aurora 深度障碍感知，急停锁存"
if $WHEEL_ONLY; then
  BANNER_MSG="$BANNER_MSG + 纯轮式里程计诊断"
fi
if [[ "$C_ZONE_DIRECTION" == clockwise ]]; then
  BANNER_MSG="$BANNER_MSG + C 区顺时针镜像"
fi
if $GO; then
  EXTRA_ARGS="$EXTRA_ARGS supervised_full_route:=true waypoints_calibrated:=true extrinsics_calibrated:=true steering_calibrated:=true emergency_stop_ready:=true operator_approved:=true"
fi
banner "[1/2] 启动系统 ($BANNER_MSG)"
mkdir -p "$LOG_DIR"
: > "$LOG"
mkdir -p "$STATE_DIR"
nohup ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=true use_lidar:=false \
  use_laser_odometry:=false use_safety:=true use_nav:=true \
  nav_autostart:=true use_camera:=false use_vision:=false \
  camera_driver:=aurora use_depth_camera:=true aurora_ir_fps:=10 aurora_rgb_fps:=10 \
  use_task:=true \
  $EXTRA_ARGS $LOCALIZATION_PROFILE_ARG $C_ZONE_DIRECTION_ARG use_visualization:=true waypoints_file:="$WP" \
  >> "$LOG" 2>&1 &
LAUNCH_PID=$!
printf '%s\n' "$LAUNCH_PID" > "$LAUNCH_PID_FILE"
echo "  Launch PID: $LAUNCH_PID"

banner "[2/2] 等待状态"
STATUS_ARGS=(--timeout 60)
STATUS_ARGS+=(--launch-pid "$LAUNCH_PID")
STATUS_ARGS+=(--depth-camera)
if ! python3 "$STATUS_TOOL" "${STATUS_ARGS[@]}"; then
  tail -80 "$LOG" || true
  hold_started_stack "startup health did not become ready; emergency stop remains latched"
fi
start_rviz

if $GO; then
  banner "复位并发车"
  RESET_RESULT=$(timeout 12s ros2 service call /smartcar/task/reset \
    std_srvs/srv/Trigger "{}" 2>&1) || {
      echo "$RESET_RESULT"
      hold_started_stack "定位复位调用失败；急停保持锁存"
    }
  if [[ "$RESET_RESULT" != *"success=True"* ]]; then
    echo "$RESET_RESULT"
    hold_started_stack "定位复位未通过原点验证；急停保持锁存"
  fi
  echo "  ✓ 已将人工放置的 P 点当前位置和航向写入定位原点"
  if ! set_software_emergency_stop false; then
    re_latch_after_transition_failure "解除软件急停失败"
  fi
  START_RESULT=""
  if ! START_RESULT=$(timeout 12s ros2 service call /smartcar/task/start \
      std_srvs/srv/Trigger "{}" 2>&1); then
    echo "$START_RESULT"
    re_latch_after_transition_failure "任务启动调用失败"
  fi
  if [[ "$START_RESULT" != *"success=True"* ]]; then
    echo "$START_RESULT"
    re_latch_after_transition_failure "任务启动未获准"
  fi
  echo "  ✓ 任务已启动"
  banner "导航运行中"
  echo "  监控: ros2 topic echo /smartcar/task/state"
  echo "  日志: tail -f $LOG"
  exit 0
fi

banner "等待人工发车"

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  系统已就绪，急停仍锁存，车辆不会自动发车    ║"
echo "║  本脚本不授予运动门禁                          ║"
echo "║  监控: ros2 topic echo /smartcar/task/state  ║"
echo "║  日志: tail -f $LOG"
echo "║  急停: ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool '{data: true}' ║"
echo "╚══════════════════════════════════════════════╝"
