# Static Forbidden Zones for Gazebo Simulation (2026-07-27)

## Problem

Gazebo `gpu_lidar` in headless mode (`-s`) returns all rays at minimum range
(0.1 m), producing an empty costmap. The six wall models in `track.world`
(2 × B-zone, 4 × C-zone) physically exist and block the robot in Gazebo, but
Nav2 cannot see them during planning.

Goal: make B-zone and C-zone permanent walls visible as forbidden zones in
RViz and constrain the Smac Hybrid DUBIN planner, without depending on LiDAR.

## Solution

**Static PGM occupancy map → `nav2_map_server` → `static_layer` in both
local and global costmaps.**

A Python script generates `field_map.pgm` and `field_map.yaml` from wall
coordinates. `map_server` publishes `/map`. Both costmaps consume it through
the already-configured `static_layer` plugin.

Non-goals (deferred): A-zone cones, dynamic obstacles.

## Field geometry (all coordinates in metres, origin at south-west)

| Feature | Region |
|---------|--------|
| Field | 0 ≤ x ≤ 5, 0 ≤ y ≤ 5 |
| A-zone | y ∈ [0, 2.0] |
| B-zone band | y ∈ [2.0, 2.5], corridor gap x ∈ [2.0, 3.0] |
| C-zone | y ∈ [2.5, 5.0] |
| C-zone ring outer | 4.0 × 1.65, centre (2.5, 3.575) → y ∈ [2.75, 4.40] |
| C-zone ring inner | 3.0 × 0.65, centre (2.5, 3.575) → y ∈ [3.25, 3.90] |

## Wall occupancy regions (painted black in PGM)

| Wall | Occupied region |
|------|----------------|
| b_zone_left | x ∈ [0, 2.0], y ∈ [2.0, 2.5] |
| b_zone_right | x ∈ [3.0, 5.0], y ∈ [2.0, 2.5] |
| c_zone_inner | x ∈ [1.0, 4.0], y ∈ [3.25, 3.90] |
| c_zone_north | x ∈ [0, 5.0], y ∈ [4.4, 5.0] |
| c_zone_west | x ∈ [0, 0.5], y ∈ [2.75, 4.40] |
| c_zone_east | x ∈ [4.5, 5.0], y ∈ [2.75, 4.40] |
| field_border (4 edges) | 1-cell rim: x=0, x=5, y=0, y=5 |

### PGM pixel mapping

Trinary mode: `pixel/255 < free_thresh → FREE`, `pixel/255 > occ_thresh →
OCCUPIED`, else UNKNOWN. With `negate: 1` this is flipped so the image looks
natural (black = obstacle, white = free):

| Role | Pixel value | Visual |
|------|------------|--------|
| Occupied (walls) | 0 | Black |
| Free (open space) | 254 | Near-white |
| Unknown | 205 | Grey |

## Map parameters

- Format: PGM binary (P5), 100 × 100 pixels
- Resolution: 0.05 m/pixel (5 m field → 100 px)
- Origin: (0, 0, 0) — bottom-left corner of the field
- YAML:
  ```yaml
  image: field_map.pgm
  mode: trinary
  resolution: 0.05
  origin: [0.0, 0.0, 0.0]
  negate: 1
  occupied_thresh: 0.65
  free_thresh: 0.196
  ```

## Launch changes

`sim.launch.py`:

1. Remove the broken `ExecuteProcess` map_server block.
2. Add a `Node` action:

```python
map_server = Node(
    package="nav2_map_server",
    executable="map_server",
    name="map_server",
    parameters=[{
        "use_sim_time": True,
        "yaml_filename": map_file,
    }],
)
```

3. Move `map_server` into `start_after_cleanup` ahead of `nav2_launch`, with its
   own `TimerAction(period=6.0, ...)` so `/clock` is publishing before the
   lifecycle node configures.

## Costmap configuration (no changes needed)

`nav2_params.yaml` already has `static_layer` in both costmaps:

```yaml
plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: true
  enabled: true
```

## Files

| File | Action |
|------|--------|
| `src/smartcar_sim/scripts/generate_field_map.py` | **New** — wall→PGM generator |
| `src/smartcar_sim/maps/field_map.pgm` | **New** — generated PGM |
| `src/smartcar_sim/maps/field_map.yaml` | **New** — generated YAML |
| `src/smartcar_sim/launch/sim.launch.py` | **Edit** — map_server Node |
| `src/smartcar_sim/rviz/sim_nav.rviz` | **Edit** — add Map display for /map |
| `src/smartcar_nav2/config/nav2_params.yaml` | No change |

## Verification

1. `sim_start.sh --headless --rviz` → `/map` topic has data
   (`ros2 topic echo /map --once`).
2. RViz shows the Map display with occupied cells matching wall locations.
3. RViz also shows waypoint markers (`/smartcar/waypoints/markers`) and field
   reference geometry (`/smartcar/field_reference/markers`). These are
   independent MarkerArray layers — already configured in `sim_nav.rviz` and
   launched via `waypoint_viz` + `field_reference` nodes.
4. Local and global costmap topics (`/local_costmap/costmap`,
   `/global_costmap/costmap`) include the static walls plus inflation.
5. `auto_train.py` run: planner routes through the B-zone corridor opening
   (x ∈ [2.0, 3.0]) and C-zone ring track, avoiding occupied cells.
