#!/usr/bin/env python3
"""
交互式导航参数调参工具（仿真版）

用法：
    python3 tune_params.py                        # 交互模式
    python3 tune_params.py --sweep turning_radius # 扫参模式
    python3 tune_params.py --restore run_xxx      # 恢复某次运行的参数

参数速查（实测验证过的调整方向）：
    minimum_turning_radius: 0.45→0.55→0.65 (越小越灵活但可能绕圈)
    yaw_goal_tolerance:     0.25→0.50→0.75 (越大越容易"到达"但精度差)
    xy_goal_tolerance:      0.12→0.25→0.35
    lookahead_dist:         0.5→0.8→1.2 (前视距离，影响跟踪精度)
    inflation_radius:       0.35→0.55→0.65 (影响避障)
    desired_linear_vel:     0.10→0.15→0.20
    curvature_tolerance:    0.15→0.20→0.30 (倒车 BT 节点参数)
"""

import os
import sys
import yaml
import shutil
import argparse
from datetime import datetime
from pathlib import Path

# 参数文件路径（仿真环境）
WS = Path(os.environ.get("ROS2_WS", os.path.expanduser("~/ros2_ws")))
PARAMS_FILE = WS / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"
BACKUP_DIR = WS / "tune_logs" / "param_backups"


def load_params() -> dict:
    with open(PARAMS_FILE) as f:
        return yaml.safe_load(f)


def save_params(data: dict):
    with open(PARAMS_FILE, "w") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)


def backup_params(run_id: str = None):
    if run_id is None:
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    dst = BACKUP_DIR / f"{run_id}.yaml"
    shutil.copy(PARAMS_FILE, dst)
    print(f"Backup saved: {dst}")
    return run_id


def restore_params(run_id: str):
    src = BACKUP_DIR / f"{run_id}.yaml"
    if not src.exists():
        print(f"Backup not found: {src}")
        print(f"Available: {list(BACKUP_DIR.glob('*.yaml'))}")
        return False
    shutil.copy(src, PARAMS_FILE)
    print(f"Restored params from: {src}")
    return True


def get_nested(data: dict, path: str):
    """Get nested dict value by dot-separated path."""
    keys = path.split(".")
    for k in keys:
        if isinstance(data, dict):
            data = data.get(k, {})
        else:
            return None
    return data if not isinstance(data, dict) else None


def set_nested(data: dict, path: str, value):
    """Set nested dict value by dot-separated path."""
    keys = path.split(".")
    for k in keys[:-1]:
        if k not in data:
            data[k] = {}
        data = data[k]
    data[keys[-1]] = value


# 可调参数注册表（路径, 描述, 默认值, 范围）
TUNABLE_PARAMS = {
    "1": {
        "path": "planner_server.ros__parameters.GridBased.minimum_turning_radius",
        "desc": "最小转弯半径 (m)",
        "default": 0.55,
        "range": (0.35, 0.80),
        "step": 0.05,
    },
    "2": {
        "path": "controller_server.ros__parameters.goal_checker.yaw_goal_tolerance",
        "desc": "目标朝向容差 (rad)",
        "default": 0.50,
        "range": (0.15, 1.0),
        "step": 0.05,
    },
    "3": {
        "path": "controller_server.ros__parameters.goal_checker.xy_goal_tolerance",
        "desc": "目标位置容差 (m)",
        "default": 0.25,
        "range": (0.10, 0.50),
        "step": 0.05,
    },
    "4": {
        "path": "controller_server.ros__parameters.FollowPath.lookahead_dist",
        "desc": "RPP 前视距离 (m)",
        "default": 0.8,
        "range": (0.3, 2.0),
        "step": 0.1,
    },
    "5": {
        "path": "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius",
        "desc": "局部膨胀半径 (m)",
        "default": 0.55,
        "range": (0.30, 0.80),
        "step": 0.05,
    },
    "6": {
        "path": "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius",
        "desc": "全局膨胀半径 (m)",
        "default": 0.65,
        "range": (0.40, 1.0),
        "step": 0.05,
    },
    "7": {
        "path": "controller_server.ros__parameters.FollowPath.desired_linear_vel",
        "desc": "期望线速度 (m/s)",
        "default": 0.15,
        "range": (0.05, 0.30),
        "step": 0.05,
    },
    "8": {
        "path": "planner_server.ros__parameters.GridBased.curvature_tolerance",
        "desc": "曲面容差 (Smac, 越大越宽松)",
        "default": 0.20,
        "range": (0.05, 0.50),
        "step": 0.05,
    },
}


def interactive():
    """交互式调参模式"""
    data = load_params()
    run_id = backup_params()
    print(f"\n{'='*60}")
    print("SmartCar 仿真参数调参工具")
    print(f"{'='*60}")

    while True:
        print("\n当前参数值:")
        for key, info in TUNABLE_PARAMS.items():
            val = get_nested(data, info["path"])
            marker = ""
            if val != info["default"]:
                marker = f" ← 默认={info['default']}"
            print(f"  [{key}] {info['desc']}: {val}{marker}")

        print(f"\n操作:")
        print("  1-8  修改参数")
        print("  s    保存并提示构建")
        print("  r    恢复默认值")
        print("  l    列出备份")
        print("  q    退出（不保存）")
        choice = input("> ").strip()

        if choice == "q":
            print("退出（参数未保存）")
            break
        elif choice == "s":
            save_params(data)
            print(f"参数已保存到: {PARAMS_FILE}")
            print("运行: bash sim_tune.sh  (或 --no-build 如果只改了YAML)")
            break
        elif choice == "r":
            for info in TUNABLE_PARAMS.values():
                set_nested(data, info["path"], info["default"])
            print("已恢复所有默认值")
        elif choice == "l":
            backups = sorted(BACKUP_DIR.glob("*.yaml"))
            if backups:
                for b in backups:
                    print(f"  {b.stem}")
            else:
                print("  无备份")
        elif choice in TUNABLE_PARAMS:
            info = TUNABLE_PARAMS[choice]
            cur = get_nested(data, info["path"])
            print(f"{info['desc']}: 当前={cur}, 范围={info['range']}, 步长={info['step']}")
            val = input(f"新值 [{cur}]: ").strip()
            if val:
                try:
                    new_val = float(val)
                    if info["range"][0] <= new_val <= info["range"][1]:
                        set_nested(data, info["path"], new_val)
                        print(f"  {info['desc']}: {cur} → {new_val}")
                    else:
                        print(f"  超出范围 {info['range']}")
                except ValueError:
                    print("  无效数值")
        else:
            print("  无效选项")


def sweep(param_key: str):
    """扫参模式：对指定参数遍历范围，每次生成一个参数快照"""
    if param_key not in TUNABLE_PARAMS:
        print(f"未知参数: {param_key}")
        print(f"可选: {list(TUNABLE_PARAMS.keys())}")
        return

    info = TUNABLE_PARAMS[param_key]
    data = load_params()
    lo, hi = info["range"]
    step = info["step"]

    print(f"扫参: {info['desc']} [{lo} → {hi}], 步长={step}")
    values = []
    v = lo
    while v <= hi + 0.0001:
        values.append(round(v, 3))
        v += step

    for val in values:
        set_nested(data, info["path"], val)
        run_id = f"sweep_{param_key}_{val}"
        save_params(data)
        backup_params(run_id)
        print(f"  [{run_id}] {info['desc']} = {val}")

    print(f"\n生成了 {len(values)} 个参数快照")
    print("逐个运行: for backup in tune_logs/param_backups/sweep_*.yaml; do")
    print("  python3 tune_params.py --restore $(basename $backup .yaml)")
    print("  bash sim_tune.sh --no-build")
    print("done")


def main():
    parser = argparse.ArgumentParser(description="SmartCar 仿真参数调参工具")
    parser.add_argument("--sweep", help="扫参模式，指定参数名")
    parser.add_argument("--restore", help="恢复指定 run_id 的参数")
    parser.add_argument("--list", action="store_true", help="列出所有备份")
    args = parser.parse_args()

    if args.list:
        backups = sorted(BACKUP_DIR.glob("*.yaml"))
        for b in backups:
            print(b.stem)
        return

    if args.restore:
        restore_params(args.restore)
        return

    if args.sweep:
        sweep(args.sweep)
        return

    # 默认：交互模式
    interactive()


if __name__ == "__main__":
    main()
