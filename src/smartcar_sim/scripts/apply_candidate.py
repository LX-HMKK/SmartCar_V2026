#!/usr/bin/env python3
"""Apply movable geometry scan candidates without locking transit headings."""
import argparse
import json
import sys
from pathlib import Path
import yaml


MOVABLE_POSITION_IDS = (
    "b_corridor_gate",
    "b_corridor_enter",
    "b_corridor_return_enter",
)
TRANSIT_CANDIDATE_IDS = (
    "c_corner_2",
    "c_corner_3",
    "c_corner_4",
    *MOVABLE_POSITION_IDS,
)


def apply_candidate(waypoints_path: str, candidate: dict, dry_run: bool = False):
    """Update permitted transit positions and leave every transit yaw free.

    The legacy geometry scan includes C-corner pose candidates.  C-corner
    positions are fixed competition constraints and ordinary transit headings
    are resolved by the runtime free-heading planner, so neither is written
    into the semantic waypoint document.
    """
    path = Path(waypoints_path)
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    waypoints = doc["waypoints"]

    updates = {}
    for key in MOVABLE_POSITION_IDS:
        wp_data = candidate.get(key)
        if wp_data is None:
            continue
        updates[key] = {
            "x": round(wp_data["x"], 3),
            "y": round(wp_data["y"], 3),
        }

    if dry_run:
        print("Would update:")
        for wid, data in updates.items():
            print(f"  {wid}: ({data['x']:.3f}, {data['y']:.3f})")
        ignored_corners = [
            key
            for key in TRANSIT_CANDIDATE_IDS
            if key not in MOVABLE_POSITION_IDS and key in candidate
        ]
        ignored_yaws = [
            key
            for key in TRANSIT_CANDIDATE_IDS
            if candidate.get(key, {}).get("yaw_deg") is not None
        ]
        if ignored_corners:
            print("  Ignored fixed C-corner candidates: " + ", ".join(ignored_corners))
        if ignored_yaws:
            print("  Ignored ordinary transit yaws: " + ", ".join(ignored_yaws))
        return

    for wp in waypoints:
        wid = wp.get("id")
        if wid in updates:
            data = updates[wid]
            wp["pose"]["position"]["x"] = data["x"]
            wp["pose"]["position"]["y"] = data["y"]
            # All candidate targets are ordinary transit points.  Remove a
            # legacy authored quaternion instead of turning it into a Nav2
            # hard goal yaw.
            wp["pose"].pop("orientation", None)
            print(f"  Updated {wid}: ({data['x']:.3f}, {data['y']:.3f})")
        elif wid in TRANSIT_CANDIDATE_IDS:
            # Candidate JSON can come from a pre-free-heading scan.  Strip
            # that stale yaw even when this fixed point has no position update.
            wp["pose"].pop("orientation", None)

    path.write_text(
        yaml.dump(doc, default_flow_style=False, allow_unicode=True,
                  sort_keys=False),
        encoding="utf-8")
    print(f"\nSaved: {path}")


def main():
    parser = argparse.ArgumentParser(
        description="Apply movable geometry candidates without locking transit headings")
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

    score = candidate.get("score", "n/a")
    total_path_m = candidate.get(
        "total_path_m", candidate.get("total_m", candidate.get("total"))
    )
    summary = f"score={score}"
    if total_path_m is not None:
        summary += f", total={total_path_m}m"
    print(f"Applying candidate #{args.index} ({summary})")
    apply_candidate(args.waypoints, candidate, dry_run=args.dry_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
