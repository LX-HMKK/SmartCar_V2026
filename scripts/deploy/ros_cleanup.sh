#!/bin/bash
# ============================================================
# SmartCar ROS2 全量清理脚本 (v2)
# 部署: scp 本文件到 RDK → cp 到 /usr/local/bin/ros_cleanup
# 用法: bash /usr/local/bin/ros_cleanup
#
# 改进点:
#   - 清理 FastRTPS/CycloneDDS 共享内存
#   - 清理后验证残留进程与串口占用
#   - 干净重置 ros2 daemon（含 kill -9 兜底）
#   - 两级终止: SIGTERM (优雅) → 等待 → SIGKILL (强制)
# ============================================================
set -euo pipefail

echo ""
echo "=== SmartCar ROS2 全量清理 ==="
echo ""

# 跳过自身 PID 和父 PID，防止自杀
KILL_SAFE() {
    local sig="$1"; local pid="$2"
    [ "$pid" = "$$" ] && return 0
    [ "$pid" = "$PPID" ] && return 0
    kill "-${sig}" "$pid" 2>/dev/null || true
}

KILLALL_SAFE() {
    local sig="$1"; shift
    local count=0
    for pattern in "$@"; do
        local pids
        pids=$(ps aux | grep "$pattern" | grep -v grep | awk '{print $2}' 2>/dev/null || true)
        for pid in $pids; do
            KILL_SAFE "$sig" "$pid" && ((count=count+1)) || true
        done
    done
    echo "$count"
}

COUNT_REMAINING() {
    ps aux | grep -E "$1" | grep -v grep | wc -l
}

# ── 1/6: SIGTERM 优雅终止 ────────────────────────────────────
echo "[1/6] 发送 SIGTERM ..."

ROS_TARGETS=(
    'ekf_node'
    'safety_node'
    'controller_server'
    'planner_server'
    'behavior_server'
    'bt_navigator'
    'waypoint_follower'
    'velocity_smoother'
    'lifecycle_manager'
    'task_node'
    'origincar_base'
    'ydlidar_ros2_driver'
    'static_transform_publisher'
    'rviz2'
    'zbar_ros'
    'barcode_reader'
    'hobot_usb_cam'
    'field_reference'
    'waypoint_viz'
    'waypoint_editor'
    'image_replay'
    'ros2 launch'
    'ros2.*daemon'
)

TERMED=$(KILLALL_SAFE 15 "${ROS_TARGETS[@]}" || echo 0)
echo "  TERM: $TERMED 进程"

sleep 2

# ── 2/6: SIGKILL 强制终止 ────────────────────────────────────
echo "[2/6] 发送 SIGKILL (强制) ..."
KILLED=$(KILLALL_SAFE 9 "${ROS_TARGETS[@]}" || echo 0)
echo "  KILL: $KILLED 进程"

# 补充: 杀 python 解释器中残留的 smartcar/ros2/rclpy/nav2 模块
for pycmd in python3 python; do
    pids=$(ps aux | grep "$pycmd" | grep -E "(smartcar|rclpy|nav2|origincar)" | grep -v grep | awk '{print $2}' 2>/dev/null || true)
    for pid in $pids; do
        KILL_SAFE 9 "$pid" || true
    done
done

sleep 1

# ── 3/6: 清理 DDS 共享内存 ───────────────────────────────────
echo "[3/6] 清理 DDS 共享内存 ..."
SHM_CLEANED=0
for pattern in fastrtps dds ros2 eprosima cyclone iceoryx rmw; do
    for item in /dev/shm/*"${pattern}"*; do
        [ -e "$item" ] || continue
        rm -rf "$item" 2>/dev/null && ((SHM_CLEANED=SHM_CLEANED+1)) || true
    done
done
echo "  删除共享内存: $SHM_CLEANED 项"

SHM_LEFT=$(ls /dev/shm/ 2>/dev/null | wc -l)
if [ "$SHM_LEFT" -gt 0 ]; then
    echo "  /dev/shm 残留: $SHM_LEFT 项 (非 ROS 相关可忽略)"
else
    echo "  /dev/shm 已清空"
fi

# ── 4/6: 重置 ros2 daemon ────────────────────────────────────
echo "[4/6] 重置 ros2 daemon ..."
ros2 daemon stop 2>/dev/null || true
sleep 1

# 杀残留 daemon 进程 (stop 可能无响应)
DAEMON_PIDS=$(ps aux | grep -E "ros2.*daemon" | grep -v grep | awk '{print $2}' 2>/dev/null || true)
for pid in $DAEMON_PIDS; do
    KILL_SAFE 9 "$pid" || true
done
sleep 1

ros2 daemon start 2>/dev/null || {
    echo "  ! 首次 daemon start 失败，重试中..."
    sleep 2
    ros2 daemon start 2>/dev/null || {
        echo "  ! ros2 daemon 启动失败"
        echo "  提示: 检查网络接口 (ifconfig) 和 CycloneDDS 配置"
    }
}
echo "  Daemon 已重置"

# ── 5/6: 验证清理结果 ────────────────────────────────────────
echo "[5/6] 验证清理结果 ..."

REMAINING_FILTER="ekf_node|safety_node|controller_server|planner_server|navigator|ydlidar|rviz|zbar|task_node|origincar_base|field_reference|waypoint_viz|waypoint_editor|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|lifecycle_manager|barcode_reader|hobot_usb_cam"

REMAINING=$(COUNT_REMAINING "$REMAINING_FILTER")
if [ "$REMAINING" -gt 0 ]; then
    echo "  ! 仍有 $REMAINING 个 ROS 进程存活:"
    ps aux | grep -E "$REMAINING_FILTER" | grep -v grep | head -10
else
    echo "  OK: 无 ROS 进程残留"
fi

# 串口占用检查 (通过 /proc, 不依赖 lsof)
SERIAL_USED=0
for fd_link in /proc/*/fd/*; do
    [ -e "$fd_link" ] || continue
    target=$(readlink "$fd_link" 2>/dev/null || true)
    case "$target" in
        */ttyUSB*) ((SERIAL_USED=SERIAL_USED+1)) ;;
    esac
done
if [ "$SERIAL_USED" -gt 0 ]; then
    echo "  ! 仍有进程持有串口设备 (ttyUSB)"
else
    echo "  OK: 无串口设备占用"
fi

# ── 6/6: 诊断报告 ────────────────────────────────────────────
echo "[6/6] 诊断报告"
rmw_impl=$(ros2 doctor --report 2>/dev/null | grep "middleware" || echo "  RMW: (无法获取)")
echo "  $rmw_impl"

echo ""
echo "=== 清理完成, 系统就绪 ==="
