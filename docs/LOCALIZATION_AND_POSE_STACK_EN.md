# Where the Robot Thinks It Is — A Detailed Guide (Localization / Pose / TF)

This document explains **systematically** all the components that affect the robot’s pose estimate (where it “thinks” it is): LiDAR, SLAM (Cartographer), Nav2, TF, odometry, IMU, and also **ultrasonic sensors** (what they *should* affect, and what they should *not*).

> Goal: When you see messages like `Robot is out of bounds of the costmap!` or `planner_server worldToMap failed`, you’ll know exactly **where to look** and which subsystem could be responsible.

---

## 1) Core Concepts: Pose, Map, TF

### Pose
A “pose” is the robot’s position and orientation. Typically written as $(x, y, \theta)$ in a chosen coordinate frame.

### TF (Transform Tree)
In ROS 2, TF maintains a **tree of transforms** between coordinate frames. Each transform defines how to convert data between frames.

In most Nav2 + SLAM setups, the common TF chain is:

- `map` → `odom` → `base_link` → `base_laser` / sensors

Meaning:
- `map`: a global, relatively stable map frame (SLAM / Map Server)
- `odom`: a locally smooth frame that drifts over time (odometry)
- `base_link`: the robot body frame

**Who determines where the robot is on the map?**
- Effectively the TF `map -> base_link` (usually composed from `map->odom` and `odom->base_link`).

---

## 2) Who Produces What? (Data Sources)

### 2.1 LiDAR (`/scan`)
LiDAR provides range measurements (LaserScan). It is used for:
- SLAM (Cartographer) to build the map and correct drift
- Nav2 costmaps to mark obstacles (obstacle layer)
- sometimes localization against a prebuilt map (AMCL / scan-matching)

LiDAR does **not** directly provide pose; it provides observations that are aligned to the map by an algorithm (SLAM / localization).

### 2.2 Odometry (`/odom`) — wheels / rf2o / fusion
Odometry estimates relative motion:
- “we moved 0.2 m forward”
- “we rotated 10 degrees”

Odometry may come from:
- wheel encoders (wheel odom)
- LiDAR odometry (e.g., `rf2o_laser_odometry`)
- IMU + wheel fusion (EKF)

It’s usually good short-term but tends to **drift** long-term.

Important practical note (this workspace): “odom” can mean two different things
- the **`odom` TF frame** (used for transforms like `odom -> base_link`)
- the **`/odom` topic** (a `nav_msgs/Odometry` message used by some Nav2 components, especially for velocity feedback)

You can have a system where TF exists but `/odom` topic does not (or vice-versa). Also, if *two different nodes* publish `odom -> base_link` TF, the robot pose can appear to “jump”.

Quick identification commands:
```bash
export ROS_DOMAIN_ID=42

# Who publishes the /odom topic (if it exists)
ros2 topic info --verbose /odom | sed -n '1,80p'

# Who publishes odom->base_link TF (authority and stability)
ros2 run tf2_ros tf2_monitor odom base_link
```

### 2.3 IMU
An IMU provides angular velocities / accelerations.
- improves heading stability, especially during turns
- does not build a map
- alone is not sufficient for global localization

### 2.4 Ultrasonic sensors
Ultrasonic sensors provide distance-to-obstacle along narrow beams. In this stack they’re typically converted into PointCloud2 (e.g., `/low_obstacles`) and used for:
- **avoidance** through the **local_costmap**

Important:
- they **should not move the pose**
- they **should not affect `/map`**
- they **should not feed Cartographer**

If ultrasonics are affecting pose in practice, that implies you are fusing them into localization (e.g., EKF with absolute measurements), which is not the intended architecture here.

---

## 3) Cartographer: How It Places the Robot on the Map

Cartographer (2D) roughly does:
1. receives `/scan` plus the LiDAR-to-base TF (e.g., `base_link -> laser`)
2. uses odometry/IMU (if configured) as a motion prior
3. performs **scan matching** against submaps
4. refines the pose estimate and updates TF

In typical setups:
- `cartographer_node` publishes **TF `map -> odom`**
- an odometry source publishes **TF `odom -> base_link`**
- together that yields `map -> base_link`

Additionally:
- `cartographer_occupancy_grid_node` publishes `/map` (OccupancyGrid)

### 3.1 Why can “out of bounds” happen with Cartographer?
It happens when the pose (`map->base_link`) or a goal ends up outside:
- the OccupancyGrid bounds (`/map.info.origin + width/height/resolution`)
- or the active costmap window (global/local)

Classic example:
- the map is still small / only partially explored
- the robot or the goal is near the edge
- planners sample around the path/goal and step outside by a few cells → `worldToMap failed`

This is **not** because “Nav2 can’t handle a live-updating map” in general. Nav2 can operate with updating maps; the failure is usually edge goals, inconsistent TF, or multiple map sources.

---

## 4) Nav2: How It Uses Pose and Map

Nav2 does not perform SLAM by default. It assumes:
- `/map` (OccupancyGrid)
- a valid TF chain `map -> odom -> base_link`
- sensor data for costmaps (e.g., `/scan`)

### 4.1 Global costmap
The global costmap typically uses `global_frame: map`.
It uses `/map` as a base layer plus obstacle layers.

Key parameters:
- `rolling_window`: if True, the costmap window follows the robot (helps low RAM/CPU, but can break far goals)
- `width/height`: window size in meters
- `resolution`: cell size in meters
- `origin_x/origin_y`: the window origin

If you get `worldToMap failed` where `mx,my` exceed `size_x,size_y`, the planner is trying to query a cell that does not exist in the current costmap.

### 4.2 Local costmap
The local costmap often uses `global_frame: odom` (or base) and is a small window around the robot.
This is the correct place to include:
- `/scan`
- `/low_obstacles` (ultrasonics as PointCloud2)

### 4.3 Planner / Controller
- Planner (NavFn / Smac, etc.) plans on the global costmap
- Controller (DWB / RPP) tracks the path and outputs `/cmd_vel`

Errors like `planner_server worldToMap failed` are *almost always*:
- a goal outside the map/costmap window
- TF `map->base_link` is wrong / jumping
- or the `/map` source is inconsistent (e.g., multiple publishers)

### 4.4 The planner “start pose” (what TF Nav2 actually uses)
Nav2 does not pick an arbitrary “start” pose. It asks the costmap for the robot pose, and the costmap answers by looking up TF.

Conceptually, the global planner uses:
- `global_costmap.global_frame` → `global_costmap.robot_base_frame`

Typical values:
- global costmap: `global_frame: map`, `robot_base_frame: base_link` (or `base_footprint`)
- local costmap: `global_frame: odom`, `robot_base_frame: base_link`

So when people say “planner start pose comes from TF `map -> base_link`”, that’s usually true for the global planner, but the *more precise* statement is:
- the start pose is the TF transform from the active costmap’s `global_frame` into the robot base frame

Why this matters for out-of-bounds:
- even if `/map` itself is large, the *active global costmap window* can be smaller
- if TF jumps (especially `map->odom`, or `odom->base_link`), the computed start $(x,y)$ can instantly move outside the current costmap bounds, and any planner checks that convert world→grid (`worldToMap`) can fail

Quick checks:
```bash
export ROS_DOMAIN_ID=42

# What frames the global costmap is actually using
ros2 param get /global_costmap/global_costmap global_frame
ros2 param get /global_costmap/global_costmap robot_base_frame

# Then echo that exact transform (replace values if they differ)
ros2 run tf2_ros tf2_echo map base_link

# Monitor for dropouts / large jumps / bad rates
ros2 run tf2_ros tf2_monitor map base_link
```

---

## 5) Critical Rule: There Must Not Be Two Publishers on `/map`

If more than one node publishes `/map`, Nav2 can effectively see “switching maps”:
- map A: origin/size
- map B: different origin/size

That commonly causes:
- `worldToMap failed`
- `Robot is out of bounds of the costmap`

**Healthy state:**
- `Publisher count: 1`
- publisher is `cartographer_occupancy_grid_node`

Command:
```bash
export ROS_DOMAIN_ID=42
ros2 daemon stop; ros2 daemon start
ros2 topic info --verbose /map | sed -n '1,80p'
```

---

## 6) ROS_DOMAIN_ID and “Why I Don’t See Nodes”

If `ros2 node list` shows only part of the graph, two common causes are:
1. you are in the wrong domain (`ROS_DOMAIN_ID` mismatch)
2. `ros2 daemon` has stale discovery cache

Fix:
```bash
export ROS_DOMAIN_ID=42
ros2 daemon stop
ros2 daemon start
```

Then re-check:
```bash
ros2 node list | head
ros2 topic list | egrep 'map|tf'
```

---

## 7) Practical Debug: “Where Does the Robot Think It Is?”

### 7.1 Check map bounds
Compare pose to OccupancyGrid bounds:
- X bounds:
  - $x \in [origin_x, origin_x + width \cdot resolution]$
- Y bounds:
  - $y \in [origin_y, origin_y + height \cdot resolution]$

To view metadata without printing the full `data` array:
```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /map --no-arr --once | sed -n '1,80p'
```

### 7.2 Check TF of robot on the map
Verify the transform exists:
```bash
export ROS_DOMAIN_ID=42
ros2 run tf2_ros tf2_echo map base_link
```

If TF is missing, Nav2 cannot plan in `map`.

### 7.3 Inspect global_costmap parameters at runtime
```bash
export ROS_DOMAIN_ID=42
ros2 param get /global_costmap/global_costmap rolling_window
ros2 param get /global_costmap/global_costmap width
ros2 param get /global_costmap/global_costmap height
ros2 param get /global_costmap/global_costmap resolution
ros2 param get /global_costmap/global_costmap origin_x
ros2 param get /global_costmap/global_costmap origin_y
```

### 7.4 How to interpret `worldToMap failed`
Example:
```
worldToMap failed: mx,my: 175,60, size_x,size_y: 160,61
```
Meaning:
- conversion produced cell col=175 but the costmap width is only 0..159
- so the queried point (often a sample near the goal/path) lies outside the current window

#### Convert it to meters (why this works)
Costmaps (and `/map`) are **grids**. A grid cell index is just “meters divided by resolution”.
The typical conversion used by `worldToMap()` is:

$$
mx = \left\lfloor \frac{x - origin_x}{resolution} \right\rfloor,\quad
my = \left\lfloor \frac{y - origin_y}{resolution} \right\rfloor
$$

Where:
- `origin_x, origin_y` are the world coordinates of cell `(0, 0)` of that grid
- `resolution` is meters-per-cell
- `size_x, size_y` are the grid dimensions in cells

So you can estimate how far “out of bounds” you are by computing how many cells exceed the valid range and multiplying by `resolution`.

For the example above:
- valid `mx` range is `[0, size_x-1] = [0, 159]`
- actual `mx` is `175`
- overflow cells: `175 - 159 = 16 cells`

If the costmap resolution is `0.05 m/cell` (common in Nav2), then overflow distance is:

$$16\,cells \cdot 0.05\,\frac{m}{cell} = 0.8\,m$$

So the planner is sampling a point roughly **0.8 m beyond the costmap’s X boundary**.

Notice also:
- valid `my` range is `[0, 60]` and `my=60`, so Y is **exactly on the last row** (right at the boundary).

This is why edge goals/frontiers are a common trigger: even if the goal cell is barely inside, planners often sample a neighborhood around it (or check footprints), and that neighborhood can step outside by a few cells.

#### What are the “physical” map/costmap sizes?
The **physical size** of the grid is:

$$width_{meters} = size_x \cdot resolution,\quad height_{meters} = size_y \cdot resolution$$

So in the example log line:
- `size_x = 160` with `resolution = 0.05 m/cell` ⇒ $160 \cdot 0.05 = 8.0\,m$
- `size_y = 61` with `resolution = 0.05 m/cell` ⇒ $61 \cdot 0.05 = 3.05\,m$

That means the grid being queried at that moment was roughly **8.0 m × 3.05 m**.

> If you expected a 30 m × 30 m global costmap but see sizes like 8 m × 3 m in the log, verify the *runtime* costmap metadata via `/global_costmap/costmap` (below). It’s the quickest way to see what Nav2 is actually using.

#### What does “resolution” refer to?
In this context, `resolution` is the resolution of the **Nav2 costmap grid** being queried by `worldToMap()`.
It is **not** the LiDAR resolution, and it may differ from the `/map` resolution coming from Cartographer.
Nav2 can resample the static map into its own costmap resolution.

#### Recover the approximate world coordinate (meters) from `mx,my`
If you know the grid `origin_x/origin_y` and `resolution`, you can estimate which **world** point triggered the failure.
For the **cell corner**:

$$x \approx origin_x + mx \cdot resolution,\quad y \approx origin_y + my \cdot resolution$$

For the **cell center** (often more intuitive):

$$x_{center} \approx origin_x + (mx + 0.5) \cdot resolution,\quad y_{center} \approx origin_y + (my + 0.5) \cdot resolution$$

So if `resolution=0.05` and (for example) `origin_x=-15`, then `mx=175` corresponds to approximately:

$$x_{center} \approx -15 + (175.5)\cdot 0.05 = -6.225\,m$$

Do the same for Y using `origin_y`.

Most common causes:
- frontier/goal chosen on the map edge
- map has not expanded into that region yet
- costmap window is too small / origin is wrong

---

### 7.5 “Who publishes it?” vs “Who subscribes to it?” (what to check)

The `worldToMap failed ...` line is **not a ROS topic message** — it’s a log printed by the **Nav2 `planner_server`** process.
However, that log is produced *based on data coming from these topics / TF frames*:

#### `/map` (nav_msgs/OccupancyGrid)
- **Publisher (healthy setup):** `cartographer_occupancy_grid_node`
- **Typical subscribers:** `global_costmap` (Nav2), mission logic / web UI (for visualization), RViz

Check:
```bash
export ROS_DOMAIN_ID=42
ros2 topic info --verbose /map | sed -n '1,120p'
```

#### `/tf` and `/tf_static`
- **Publishers:** usually `robot_state_publisher` (robot model frames) and SLAM/localization/odometry nodes (map/odom)
- **Subscribers:** Nav2 servers, mission nodes, RViz, any TF listeners

Check:
```bash
export ROS_DOMAIN_ID=42
ros2 topic info --verbose /tf | sed -n '1,120p'
ros2 topic info --verbose /tf_static | sed -n '1,120p'
```

#### `/global_costmap/costmap` and `/global_costmap/costmap_updates`
- **Publisher:** the `global_costmap` node (Nav2 costmap_2d)
- **Typical subscribers:** RViz and any diagnostic/monitoring tools; (Nav2 planner typically uses the costmap internally, not by subscribing to this topic)

Check:
```bash
export ROS_DOMAIN_ID=42
ros2 topic info --verbose /global_costmap/costmap | sed -n '1,120p'
```

To see the *actual runtime grid size/origin/resolution* (this is the “truth” that matches `size_x,size_y`):
```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /global_costmap/costmap --no-arr --once | sed -n '1,120p'
```

You can do the same for local costmap:
```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /local_costmap/costmap --no-arr --once | sed -n '1,120p'
```

> If `/map` shows `Publisher count: 2`, fix that first. Multiple `/map` publishers can cause costmap origin/size to “jump”, which also triggers `worldToMap failed`.

---

## 8) Where Ultrasonics *Should* Affect the Stack

Ultrasonics should feed **local_costmap only**:
- marking (and optionally clearing, depending on noise)
- to avoid low obstacles

They should not:
- publish TF
- publish `/map`
- feed Cartographer
- move pose (`map->base_link`)

If you see “pose shifts” and you suspect ultrasonics, it’s usually actually:
- TF / odometry / LiDAR alignment issues
- or inconsistent map sources
- or edge goals causing planner errors

---

## 9) Common Scenarios and Symptoms

### Scenario A: TF missing or unstable
Symptoms:
- Nav2 doesn’t move
- `tf2_echo map base_link` fails
- logs about missing transforms

Also common when TF is *jumping* (not missing):
- `tf2_echo map base_link` shows discontinuous position changes (sudden meters of movement with the robot stationary)
- `worldToMap failed` appears “randomly”, including when navigating to nearby goals

Fix ideas:
- ensure `robot_state_publisher` is running
- ensure `odom->base_link` is being published
- ensure Cartographer publishes `map->odom`
- ensure there is only one logical source of `map->odom` (two different localization sources publishing `map->odom` can look like a pose jump)

### Scenario B: Two publishers on `/map`
Symptoms:
- `Publisher count: 2`
- `worldToMap failed` / out-of-bounds starts “randomly”

Fix:
- keep a single `/map` publisher (Cartographer)

### Scenario C: Exploration goals on the map edge
Symptoms:
- `worldToMap failed` spam during exploration/frontiers
- happens especially near the edge of the explored region

Fix:
- avoid goals too close to map edges (margin)
- ensure goal is in free space

---

## 10) Command Cheat Sheet

```bash
# Use the correct domain
export ROS_DOMAIN_ID=42

# Refresh discovery
ros2 daemon stop; ros2 daemon start

# Who publishes /map
ros2 topic info --verbose /map | sed -n '1,80p'

# Map metadata
ros2 topic echo /map --no-arr --once | sed -n '1,80p'

# Robot TF on the map
ros2 run tf2_ros tf2_echo map base_link

# Global costmap parameters
ros2 param get /global_costmap/global_costmap width
ros2 param get /global_costmap/global_costmap height
ros2 param get /global_costmap/global_costmap rolling_window
```

---

## 11) Project-Specific Notes (This Workspace)

- The healthy `/map` source is `cartographer_occupancy_grid_node`.
- Nav2 uses `/map` + TF for planning.
- Ultrasonics (`/low_obstacles`) should affect **local_costmap only**.
- If `worldToMap failed` appears during exploration, it is often not “live map updates” but simply goals too close to the edge. In that case, harden goal selection with a margin and free-space validation.

---

If you want, I can add a section that maps this to your *exact* runtime nodes/frames (who publishes which TF) — just paste:
- `ros2 topic info --verbose /tf | sed -n '1,80p'`
- `ros2 topic info --verbose /tf_static | sed -n '1,80p'`
- and optionally a TF tree screenshot from RViz.
