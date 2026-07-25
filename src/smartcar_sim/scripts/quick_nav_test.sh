#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export DISPLAY=:0
export IGN_GAZEBO_RESOURCE_PATH=/root/ros2_ws/install/smartcar_sim/share/smartcar_sim/models:$IGN_GAZEBO_RESOURCE_PATH

pkill -9 ign ros2 rviz2 2>/dev/null || true
sleep 2

echo "========================================="
echo "  SmartCar 仿真导航测试"
echo "========================================="

echo "[1/4] 启动仿真（GUI模式）..."
ros2 launch smartcar_sim sim.launch.py headless:=false use_rviz:=false > /tmp/nav_final.log 2>&1 &
SIM_PID=$!

echo "[2/4] 等待 Nav2 就绪..."
READY=0
for i in $(seq 1 60); do
    sleep 2
    if ros2 node list 2>/dev/null | grep -q controller_server; then
        echo "  controller_server 出现 (${i}x2s)"
        READY=1
        break
    fi
done
[ $READY -eq 0 ] && { echo "超时!"; kill $SIM_PID 2>/dev/null; exit 1; }

sleep 8
echo "  节点列表:"
ros2 node list 2>/dev/null

echo "[3/4] 发送导航目标: P -> QR..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom_combined}, pose: {position: {x: 3.127, y: 0.977, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.2588, w: 0.9659}}}}" \
  --feedback 2>&1 | tee /tmp/goal_result.txt &
GOAL_PID=$!

echo "[4/4] 观察 40 秒..."
for i in $(seq 1 20); do
    sleep 2
    POS=$(ros2 topic echo /odom_combined --once 2>/dev/null | grep "x:" | head -1 | tr -d " " || echo "?")
    echo "  t=$((i*2))s $POS"
done

echo ""
echo "=== 目标结果 ==="
cat /tmp/goal_result.txt 2>/dev/null || echo "(无)"
echo ""
echo "=== 关键日志 ==="
grep -iE "circle|ccc|兜|replan|reverse|abort|succeed|reached" /tmp/nav_final.log 2>/dev/null | tail -10 || echo "(无关键日志)"
echo ""
echo "=== 错误 ==="
grep -iE "error|fatal" /tmp/nav_final.log 2>/dev/null | grep -v "gui-system\|SHM\|fastrtps" | tail -5 || echo "(无严重错误)"

kill $GOAL_PID $SIM_PID 2>/dev/null || true
pkill -9 ign ros2 2>/dev/null || true
echo "=== 测试完成 ==="
