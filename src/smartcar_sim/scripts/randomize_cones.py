#!/usr/bin/env python3
"""A区锥桶随机摆放——每次运行生成不同的障碍物布局。
用法：python3 randomize_cones.py [world_file] [seed]
输出：覆盖 world 文件中的锥桶 <pose>。
"""

import sys
import random
import re
from pathlib import Path

# A区边界 (x_min, x_max, y_min, y_max)
ZONE_A = (0.3, 3.0, 0.05, 1.0)

# 锥桶数量范围
MIN_CONES = 4
MAX_CONES = 10

# 锥桶尺寸
CONE_SIZE = 0.3

# 最小间距（锥桶之间、锥桶与墙之间）
MIN_SEPARATION = 0.5


def generate_positions(n: int, seed: int = None):
    """生成 n 个不重叠的锥桶位置"""
    if seed is not None:
        random.seed(seed)
    positions = []
    attempts = 0
    while len(positions) < n and attempts < 500:
        x = round(random.uniform(ZONE_A[0], ZONE_A[1]), 2)
        y = round(random.uniform(ZONE_A[2], ZONE_A[3]), 2)
        yaw = round(random.uniform(-3.14, 3.14), 2)

        # 检查与已有锥桶的距离
        ok = True
        for px, py, _ in positions:
            if ((x - px) ** 2 + (y - py) ** 2) < MIN_SEPARATION ** 2:
                ok = False
                break
        if ok:
            positions.append((x, y, yaw))
        attempts += 1
    return positions


def update_world(world_path: Path, positions: list):
    """更新 world 文件中的锥桶 pose"""
    text = world_path.read_text()

    # 找到并更新每个 cone_a{N} 的 <pose>
    cone_pattern = re.compile(r'(<model name="cone_a\d+"><static>true</static>)<pose>[^<]*</pose>')

    updated = text
    for i, (x, y, yaw) in enumerate(positions):
        new_pose = f'<pose>{x} {y} 0.15 0 0 {yaw}</pose>'
        # 替换第 i 个锥桶的 pose
        match_count = 0

        def replace_pose(m):
            nonlocal match_count
            if match_count == i:
                match_count += 1
                return m.group(1) + new_pose
            match_count += 1
            return m.group(0)

        updated = cone_pattern.sub(replace_pose, updated, count=i + 1)

    world_path.write_text(updated)
    return len(positions)


def main():
    world_file = sys.argv[1] if len(sys.argv) > 1 else None
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else random.randint(0, 99999)

    if world_file is None:
        # Default: find the installed world
        candidates = list(Path("/root/ros2_ws").rglob("track.world"))
        if not candidates:
            print("ERROR: track.world not found")
            sys.exit(1)
        world_file = str(candidates[0])

    world_path = Path(world_file)
    if not world_path.exists():
        print(f"ERROR: {world_path} not found")
        sys.exit(1)

    random.seed(seed)
    n_cones = random.randint(MIN_CONES, MAX_CONES)
    positions = generate_positions(n_cones, seed)

    # Backup original
    backup = world_path.with_suffix(".world.bak")
    if not backup.exists():
        backup.write_text(world_path.read_text())

    update_world(world_path, positions)

    print(f"Seed: {seed}")
    print(f"Cones: {len(positions)}")
    for i, (x, y, yaw) in enumerate(positions):
        print(f"  cone_a{i + 1}: ({x:.2f}, {y:.2f}, {yaw:.2f})")
    print(f"World updated: {world_path}")
    print(f"Restore: cp {backup} {world_path}")


if __name__ == "__main__":
    main()
