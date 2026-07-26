#!/usr/bin/env python3
"""Edit navigation tuning parameters without reformatting nav2_params.yaml."""

import argparse
import copy
from contextlib import contextmanager
import math
import os
import shutil
from datetime import datetime
from pathlib import Path

import yaml
from yaml.nodes import MappingNode, ScalarNode

try:
    import fcntl
except ImportError:  # pragma: no cover - Windows-only test environment.
    fcntl = None


WS = Path(os.environ.get("SMARTCAR_WS", "/root/ros2_ws"))
REPO_ROOT = Path(os.environ.get(
    "SMARTCAR_REPO_ROOT", "/mnt/d/StudyWorks/3.2/SmartCar"))
SOURCE_ROOT = Path(os.environ.get("SMARTCAR_SRC", str(REPO_ROOT / "src")))
PARAMS_FILE = SOURCE_ROOT / "smartcar_nav2" / "config" / "nav2_params.yaml"
BACKUP_DIR = Path(os.environ.get(
    "SMARTCAR_TUNE_BACKUP_DIR",
    str(WS / "tune_logs" / "param_backups"),
))
LOCK_FILE = Path(os.environ.get(
    "SMARTCAR_TUNE_LOCK_FILE", str(WS / ".sim_tune.lock")))


TUNABLE_PARAMS = {
    "1": {
        "name": "minimum_turning_radius",
        "path": "planner_server.ros__parameters.GridBased.minimum_turning_radius",
        "desc": "最小转弯半径 (m)",
        "default": 0.55,
        "range": (0.35, 0.80),
        "step": 0.05,
    },
    "2": {
        "name": "precise_yaw_goal_tolerance",
        "path": "controller_server.ros__parameters.precise_goal_checker.yaw_goal_tolerance",
        "desc": "QR 精确目标朝向容差 (rad)",
        "default": 0.15,
        "range": (0.10, 0.30),
        "step": 0.05,
    },
    "3": {
        "name": "precise_xy_goal_tolerance",
        "path": "controller_server.ros__parameters.precise_goal_checker.xy_goal_tolerance",
        "desc": "QR 精确目标位置容差 (m)",
        "default": 0.12,
        "range": (0.08, 0.25),
        "step": 0.01,
    },
    "4": {
        "name": "yaw_goal_tolerance",
        "path": "controller_server.ros__parameters.goal_checker.yaw_goal_tolerance",
        "desc": "普通目标朝向容差 (rad)",
        "default": 0.50,
        "range": (0.25, 1.00),
        "step": 0.05,
    },
    "5": {
        "name": "xy_goal_tolerance",
        "path": "controller_server.ros__parameters.goal_checker.xy_goal_tolerance",
        "desc": "普通目标位置容差 (m)",
        "default": 0.25,
        "range": (0.10, 0.50),
        "step": 0.05,
    },
    "6": {
        "name": "lookahead_dist",
        "path": "controller_server.ros__parameters.FollowPath.lookahead_dist",
        "desc": "RPP 前视距离 (m)",
        "default": 0.80,
        "range": (0.30, 2.00),
        "step": 0.10,
    },
    "7": {
        "name": "local_inflation_radius",
        "path": "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius",
        "desc": "局部膨胀半径 (m)",
        "default": 0.55,
        "range": (0.30, 0.80),
        "step": 0.05,
    },
    "8": {
        "name": "global_inflation_radius",
        "path": "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius",
        "desc": "全局膨胀半径 (m)",
        "default": 0.65,
        "range": (0.40, 1.00),
        "step": 0.05,
    },
    "9": {
        "name": "desired_linear_vel",
        "path": "controller_server.ros__parameters.FollowPath.desired_linear_vel",
        "desc": "期望线速度 (m/s)",
        "default": 0.15,
        "range": (0.05, 0.30),
        "step": 0.05,
    },
    "10": {
        "name": "reverse_handoff_desired_linear_vel",
        "path": "controller_server.ros__parameters.ReverseHandoff.desired_linear_vel",
        "desc": "倒车交接期望线速度 (m/s)",
        "default": 0.09,
        "range": (0.05, 0.15),
        "step": 0.01,
    },
    "11": {
        "name": "reverse_handoff_lookahead_dist",
        "path": "controller_server.ros__parameters.ReverseHandoff.lookahead_dist",
        "desc": "倒车交接固定前视距离 (m)",
        "default": 0.25,
        "range": (0.15, 0.45),
        "step": 0.05,
    },
}


def load_params(path=None):
    path = Path(path) if path is not None else PARAMS_FILE
    if not path.is_file():
        raise FileNotFoundError(
            f"authoritative nav2 params not found: {path}")
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def get_nested(data, path):
    current = data
    for key in path.split("."):
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def set_nested(data, path, value):
    keys = path.split(".")
    current = data
    for key in keys[:-1]:
        if not isinstance(current, dict) or key not in current:
            raise KeyError(f"missing parameter path: {path}")
        current = current[key]
    if not isinstance(current, dict) or keys[-1] not in current:
        raise KeyError(f"missing parameter path: {path}")
    current[keys[-1]] = value


def _mapping_value(node, key):
    if not isinstance(node, MappingNode):
        raise KeyError(key)
    for key_node, value_node in node.value:
        if isinstance(key_node, ScalarNode) and key_node.value == key:
            return value_node
    raise KeyError(key)


def _scalar_node(document, path):
    node = document
    for key in path.split("."):
        node = _mapping_value(node, key)
    if not isinstance(node, ScalarNode):
        raise TypeError(f"parameter is not scalar: {path}")
    return node


def _format_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return format(value, ".12g")
    raise TypeError(f"unsupported scalar value: {value!r}")


@contextmanager
def _tuning_lock():
    if fcntl is None:
        yield
        return
    LOCK_FILE.parent.mkdir(parents=True, exist_ok=True)
    with LOCK_FILE.open("a+") as stream:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(
                f"another simulation/tuning run holds {LOCK_FILE}") from error
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def render_params(data, source_text):
    original = yaml.safe_load(source_text)
    document = yaml.compose(source_text)
    replacements = []
    for info in TUNABLE_PARAMS.values():
        path = info["path"]
        old_value = get_nested(original, path)
        new_value = get_nested(data, path)
        if old_value == new_value:
            continue
        if new_value is None:
            raise KeyError(f"missing parameter path: {path}")
        node = _scalar_node(document, path)
        replacements.append((
            node.start_mark.index,
            node.end_mark.index,
            _format_scalar(new_value),
        ))

    rendered = source_text
    for start, end, value in sorted(replacements, reverse=True):
        rendered = rendered[:start] + value + rendered[end:]
    yaml.safe_load(rendered)
    return rendered


def save_params(data):
    with _tuning_lock():
        source_text = PARAMS_FILE.read_text(encoding="utf-8")
        temporary = PARAMS_FILE.with_name(f".{PARAMS_FILE.name}.tmp")
        try:
            temporary.write_text(
                render_params(data, source_text), encoding="utf-8")
            os.replace(temporary, PARAMS_FILE)
        finally:
            temporary.unlink(missing_ok=True)


def backup_params(run_id=None, source=None):
    source = Path(source) if source is not None else PARAMS_FILE
    run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    destination = BACKUP_DIR / f"{run_id}.yaml"
    shutil.copy(source, destination)
    print(f"Backup saved: {destination}")
    return run_id


def restore_params(run_id):
    backup_root = BACKUP_DIR.resolve()
    source = (backup_root / f"{run_id}.yaml").resolve()
    if source.parent != backup_root:
        print(f"Invalid backup id: {run_id}")
        return False
    if not source.is_file():
        print(f"Backup not found: {source}")
        return False
    backup_data = yaml.safe_load(source.read_text(encoding="utf-8"))
    current_data = load_params()
    for info in TUNABLE_PARAMS.values():
        value = get_nested(backup_data, info["path"])
        if value is None:
            raise KeyError(f"missing backup parameter path: {info['path']}")
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            raise ValueError(
                f"invalid backup value for {info['name']}: {value!r}")
        minimum, maximum = info["range"]
        if not minimum <= float(value) <= maximum:
            raise ValueError(
                f"backup value out of range for {info['name']}: {value}")
        set_nested(current_data, info["path"], float(value))
    save_params(current_data)
    print(f"Restored params from: {source}")
    print("Run sim_tune.sh to rebuild nav2_params_fixed.yaml before testing.")
    return True


def resolve_param(selector):
    if selector in TUNABLE_PARAMS:
        return selector, TUNABLE_PARAMS[selector]
    for key, info in TUNABLE_PARAMS.items():
        if selector == info["name"]:
            return key, info
    choices = ", ".join(
        info["name"] for info in TUNABLE_PARAMS.values())
    raise ValueError(f"unknown parameter {selector!r}; choose: {choices}")


def interactive():
    original = load_params()
    data = copy.deepcopy(original)
    backup_params()
    print("\nSmartCar 仿真参数调参工具")
    print(f"参数源: {PARAMS_FILE}")

    while True:
        print("\n当前参数值:")
        for key, info in TUNABLE_PARAMS.items():
            value = get_nested(data, info["path"])
            marker = (
                f" <- 默认={info['default']}"
                if value != info["default"] else ""
            )
            print(f"  [{key}] {info['desc']}: {value}{marker}")

        print("\n操作: 参数编号修改, s 保存, r 恢复默认值, l 列出备份, q 退出")
        choice = input("> ").strip()
        if choice == "q":
            print("退出，参数文件未修改")
            return
        if choice == "s":
            save_params(data)
            print(f"参数已保存到: {PARAMS_FILE}")
            print("运行: bash sim_tune.sh")
            return
        if choice == "r":
            for info in TUNABLE_PARAMS.values():
                set_nested(data, info["path"], info["default"])
            continue
        if choice == "l":
            for backup in sorted(BACKUP_DIR.glob("*.yaml")):
                print(f"  {backup.stem}")
            continue
        if choice not in TUNABLE_PARAMS:
            print("无效选项")
            continue

        info = TUNABLE_PARAMS[choice]
        current = get_nested(data, info["path"])
        print(
            f"{info['desc']}: 当前={current}, "
            f"范围={info['range']}, 步长={info['step']}"
        )
        raw_value = input(f"新值 [{current}]: ").strip()
        if not raw_value:
            continue
        try:
            value = float(raw_value)
        except ValueError:
            print("无效数值")
            continue
        if not info["range"][0] <= value <= info["range"][1]:
            print(f"超出范围 {info['range']}")
            continue
        set_nested(data, info["path"], value)


def sweep(selector):
    key, info = resolve_param(selector)
    data = load_params()
    source_text = PARAMS_FILE.read_text(encoding="utf-8")
    low, high = info["range"]
    value = low
    values = []
    while value <= high + 1.0e-9:
        values.append(round(value, 6))
        value += info["step"]

    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    for value in values:
        snapshot = copy.deepcopy(data)
        set_nested(snapshot, info["path"], value)
        run_id = f"sweep_{key}_{info['name']}_{value:g}"
        destination = BACKUP_DIR / f"{run_id}.yaml"
        destination.write_text(
            render_params(snapshot, source_text), encoding="utf-8")
        print(f"  {run_id}: {info['desc']} = {value:g}")

    print(f"Generated {len(values)} snapshots; source file was not changed.")
    print("Restore one snapshot, then run sim_tune.sh to build and test it.")


def main():
    parser = argparse.ArgumentParser(
        description="SmartCar 仿真参数调参工具")
    parser.add_argument(
        "--sweep", help="参数编号或名称，例如 precise_yaw_goal_tolerance")
    parser.add_argument("--restore", help="恢复指定 run_id")
    parser.add_argument(
        "--list", action="store_true", help="列出所有备份")
    args = parser.parse_args()

    if args.list:
        for backup in sorted(BACKUP_DIR.glob("*.yaml")):
            print(backup.stem)
        return
    if args.restore:
        raise SystemExit(0 if restore_params(args.restore) else 1)
    if args.sweep:
        try:
            sweep(args.sweep)
        except ValueError as error:
            parser.error(str(error))
        return
    interactive()


if __name__ == "__main__":
    main()
