#!/usr/bin/env python3
"""Generate field_map.pgm + field_map.yaml from wall geometry.

The script encodes B-zone and C-zone permanent walls as occupied cells in a
100×100 PGM (0.05 m/px, 5×5 m field). Wall coordinates are derived from
track.world and field_geometry.yaml.  The output is consumed by
nav2_map_server → static_layer → Nav2 costmaps.

PGM pixel convention (negate: 1):
  pixel   0 (black) → OCCUPIED (wall)
  pixel 254 (white) → FREE     (open space)
  pixel 205 (grey)  → UNKNOWN  (out of field / unclassified)

P5 PGM is row-major top-to-bottom. Row 0 = y=5.0 (top of field),
row 99 = y=0.0 (bottom).  The Y-axis is flipped so the field origin
(bottom-left) appears at the bottom of the image.
"""

import math
import os
import struct
import sys
from pathlib import Path

# ── field geometry (metres, origin south-west) ──
FIELD_SIZE = 5.0           # both axes
RESOLUTION = 0.05          # m/px
PIXELS = int(FIELD_SIZE / RESOLUTION)  # 100

# Occupancy values (PGM pixel bytes)
OCCUPIED = 0               # black → wall (negate:1 flips to occupied)
FREE = 254                 # near-white → open
UNKNOWN = 205              # grey → unclassified

# Wall definitions: (x_min, x_max, y_min, y_max) in metres
WALLS = [
    # B-zone band at y∈[2.0, 2.5]; corridor opening x∈[2.0, 3.0]
    ("b_zone_left",   0.0, 2.0,   2.0, 2.5),
    ("b_zone_right",  3.0, 5.0,   2.0, 2.5),
    # C-zone inner fill blocks the centre of the ring
    ("c_zone_inner",  1.0, 4.0,   3.25, 3.90),
    # C-zone outer boundaries
    ("c_zone_north",  0.0, 5.0,   4.4, 5.0),
    ("c_zone_west",   0.0, 0.5,   2.75, 4.40),
    ("c_zone_east",   4.5, 5.0,   2.75, 4.40),
]

# Field border: 1-cell rim around the entire field
BORDERS = [
    ("border_south", 0.0, 5.0,   0.0, 0.0),   # touching y=0
    ("border_north", 0.0, 5.0,   5.0, 5.0),   # touching y=5
    ("border_west",  0.0, 0.0,   0.0, 5.0),   # touching x=0
    ("border_east",  5.0, 5.0,   0.0, 5.0),   # touching x=5
]

# Corridor centre-line (decorative guide only — NOT an obstacle)
CORRIDOR_X = 2.5
CORRIDOR_HALF = 0.5


def fill_rect(grid: bytearray, x0: float, x1: float, y0: float, y1: float,
              value: int) -> None:
    """Mark every pixel whose centre falls inside [x0,x1)×[y0,y1).

    PGM rows go top→bottom: row 0 = field y = FIELD_SIZE, row 99 = field y = 0.
    This function computes the pixel row range from field y-coordinates,
    then fills every pixel in that range.
    """
    if x0 >= x1 or y0 >= y1:
        return
    # column range (x → col is straightforward: col = x / resolution)
    col0 = max(0, int(math.floor(x0 / RESOLUTION)))
    col1 = min(PIXELS, int(math.ceil(x1 / RESOLUTION)))
    if col0 >= col1:
        return
    # row range (y → row is inverted: row = (FIELD_SIZE - y) / resolution)
    # row_top    = row index of the top    edge (y = y1), smaller row number
    # row_bottom = row index of the bottom edge (y = y0), larger  row number
    row_top = int(math.floor((FIELD_SIZE - y1) / RESOLUTION))
    row_bottom = int(math.ceil((FIELD_SIZE - y0) / RESOLUTION)) - 1
    # clamp
    row_top = max(0, row_top)
    row_bottom = min(PIXELS - 1, row_bottom)
    if row_top > row_bottom:
        return
    for row in range(row_top, row_bottom + 1):
        base = row * PIXELS
        for col in range(col0, col1):
            grid[base + col] = value


def draw_corridor_marks(grid: bytearray) -> None:
    """Light dash marks along the corridor opening edges (NOT obstacles)."""
    col_center = int(CORRIDOR_X / RESOLUTION)
    half_px = int(CORRIDOR_HALF / RESOLUTION)
    # B-zone band: y ∈ [2.0, 2.5]
    row_top = int((FIELD_SIZE - 2.5) / RESOLUTION)
    row_bottom = int((FIELD_SIZE - 2.0) / RESOLUTION) - 1
    for row in range(row_top, row_bottom + 1, 3):  # every 3rd row → dashed
        left = max(0, col_center - half_px)
        right = min(PIXELS - 1, col_center + half_px)
        if 0 <= left < PIXELS:
            grid[row * PIXELS + left] = 150
        if 0 <= right < PIXELS:
            grid[row * PIXELS + right] = 150


def make_grid() -> bytearray:
    """Build the 100×100 occupancy grid."""
    grid = bytearray([FREE]) * (PIXELS * PIXELS)

    # 1. Paint wall rectangles as OCCUPIED
    for _name, x0, x1, y0, y1 in WALLS:
        fill_rect(grid, x0, x1, y0, y1, OCCUPIED)

    # 2. Paint field border rim (1 cell)
    for _name, x0, x1, y0, y1 in BORDERS:
        # expand degenerate edges to 1-cell thickness
        if abs(x1 - x0) < 1e-6:
            x0 -= RESOLUTION / 2
            x1 += RESOLUTION / 2
        if abs(y1 - y0) < 1e-6:
            y0 -= RESOLUTION / 2
            y1 += RESOLUTION / 2
        fill_rect(grid, x0, x1, y0, y1, OCCUPIED)

    # 3. Light corridor edge dashes (visual only, still free)
    draw_corridor_marks(grid)

    # 4. Mark pixels outside 5×5 as UNKNOWN (fringe gutter)
    #    Actually the grid IS exactly 5×5, so just mark a 1px rim unknown.
    #    No — keep it simple, the grid covers the field exactly.

    return grid


def write_pgm(path: Path, grid: bytearray) -> None:
    """Write P5 binary PGM."""
    header = f"P5\n{PIXELS} {PIXELS}\n255\n".encode("ascii")
    path.write_bytes(header + bytes(grid))


def write_yaml(path: Path, image_name: str) -> None:
    """Write the nav2 map YAML descriptor."""
    content = (
        f"image: {image_name}\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION}\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 1\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    path.write_text(content, encoding="utf-8")


def main() -> None:
    maps_dir = Path(__file__).resolve().parent.parent / "maps"
    maps_dir.mkdir(parents=True, exist_ok=True)

    pgm_path = maps_dir / "field_map.pgm"
    yaml_path = maps_dir / "field_map.yaml"

    grid = make_grid()

    # Quick stats
    occupied_px = sum(1 for b in grid if b == OCCUPIED)
    free_px = sum(1 for b in grid if b == FREE)
    print(
        f"Generated {PIXELS}×{PIXELS} PGM: "
        f"{occupied_px} occupied, {free_px} free pixels"
    )

    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, "field_map.pgm")

    print(f"Wrote {pgm_path}")
    print(f"Wrote {yaml_path}")

    # Verify round-trip: YAML file is readable
    if not yaml_path.exists():
        print("ERROR: YAML file not found after write", file=sys.stderr)
        sys.exit(1)

    pgm_size = pgm_path.stat().st_size
    expected = len(header_bytes()) + PIXELS * PIXELS
    if pgm_size != expected:
        print(
            f"WARNING: PGM size {pgm_size} != expected {expected} "
            f"(header {len(header_bytes())} + {PIXELS*PIXELS} data)",
            file=sys.stderr,
        )


def header_bytes() -> bytes:
    return f"P5\n{PIXELS} {PIXELS}\n255\n".encode("ascii")


if __name__ == "__main__":
    main()
