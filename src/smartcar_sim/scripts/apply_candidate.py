#!/usr/bin/env python3
"""Apply geometry scan candidates to nav_only.yaml waypoints file."""
import argparse, json, math, sys
from pathlib import Path
import yaml


def yaw_to_quat(yaw_deg: float):
    """Convert yaw in degrees to ROS quaternion (z, w)."""
    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def apply_candidate(waypoints_path: str, candidate: dict, dry_run: bool = False):
    """Update waypoints from candidate dict."""
    path = Path(waypoints_path)
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    waypoints = doc["waypoints"]

    updates = {}
    for key in ["c_corner_2", "c_corner_3", "c_corner_4",
                "b_corridor_return_enter"]:
        wp_data = candidate.get(key)
        if wp_data is None:
            continue
        x, y, yaw_deg = wp_data["x"], wp_data["y"], wp_data["yaw_deg"]
        z, w = yaw_to_quat(yaw_deg)[2], yaw_to_quat(yaw_deg)[3]
        updates[key] = {
            "x": round(x, 3), "y": round(y, 3),
            "z": round(z, 6), "w": round(w, 6),
            "yaw_deg": yaw_deg,
        }

    if dry_run:
        print("Would update:")
        for wid, data in updates.items():
            print(f"  {wid}: ({data['x']:.3f}, {data['y']:.3f}), "
                  f"yaw {data['yaw_deg']:.0f} deg")
        return

    for wp in waypoints:
        wid = wp.get("id")
        if wid in updates:
            data = updates[wid]
            wp["pose"]["position"]["x"] = data["x"]
            wp["pose"]["position"]["y"] = data["y"]
            wp["pose"]["orientation"]["z"] = data["z"]
            wp["pose"]["orientation"]["w"] = data["w"]
            print(f"  Updated {wid}: ({data['x']:.3f}, {data['y']:.3f}), "
                  f"yaw {data['yaw_deg']:.0f} deg")

    path.write_text(
        yaml.dump(doc, default_flow_style=False, allow_unicode=True,
                  sort_keys=False),
        encoding="utf-8")
    print(f"\nSaved: {path}")


def main():
    parser = argparse.ArgumentParser(
        description="Apply geometry scan candidate to waypoints file")
    parser.add_argument("waypoints", help="Path to waypoints YAML")
    parser.add_argument("--candidate", type=str, required=True,
                        help="Path to candidate JSON or candidate index (0-based)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show changes without writing")
    parser.add_argument("--index", type=int, default=0,
                        help="Candidate index in JSON file (0-based)")
    args = parser.parse_args()

    candidate_path = Path(args.candidate)
    if candidate_path.suffix == ".json" and candidate_path.is_file():
        candidates = json.loads(candidate_path.read_text(encoding="utf-8"))
        if args.index >= len(candidates):
            print(f"Error: index {args.index} >= {len(candidates)} candidates",
                  file=sys.stderr)
            return 1
        candidate = candidates[args.index]
    else:
        print(f"Error: {args.candidate} is not a valid JSON file", file=sys.stderr)
        return 1

    print(f"Applying candidate #{args.index} (score={candidate['score']}, "
          f"total={candidate['total_path_m']}m)")
    apply_candidate(args.waypoints, candidate, dry_run=args.dry_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
