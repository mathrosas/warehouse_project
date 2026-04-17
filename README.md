# Checkpoint 11 - Autonomous Navigation

Full **Nav2 navigation stack** for an **RB1 mobile robot** operating in a warehouse environment. The robot builds a map with **Cartographer SLAM**, localizes itself with **AMCL**, and autonomously navigates to goals using global/local costmaps, a **DWB local planner**, a **NavfnPlanner** global planner, and a custom **behavior tree** with replanning and recovery actions. Supports both **simulation** and **real robot** environments via a single parameterized launch command.

Part of the [ROS & ROS 2 Developer Master Class](https://www.theconstructsim.com/) certification (Phase 2).

<p align="center">
  <img src="media/rb1_planning.gif" alt="RB1 robot autonomously navigating with global and local costmaps and path planning in RViz" width="650"/>
</p>

## How It Works

<p align="center">
  <img src="media/particles.gif" alt="AMCL particle cloud converging around the RB1 robot during localization" width="650"/>
</p>

### Task 1 - Mapping (Cartographer SLAM)

1. `cartographer_node` subscribes to `/scan` and `/odom`, builds a 2D occupancy grid
2. `occupancy_grid_node` publishes the evolving map on `/map` at 1 Hz
3. User teleops the robot around the warehouse to generate the full map
4. Map saved as `warehouse_map_sim.yaml/.pgm` (and `warehouse_map_real.*` for real robot)
5. `map_server` package reloads the saved map for downstream nodes

### Task 2 - Localization (AMCL)

1. `map_server` serves the saved warehouse map
2. `amcl` initializes a particle cloud (2000-80000 particles) and estimates robot pose on the map
3. Laser scans align with the map as the robot moves — particles converge to the true pose
4. `lifecycle_manager_localization` sequences map_server → amcl activation
5. The `/attach_service_server` is also launched here (reused from Checkpoint 9/10 for shelf attachment)

### Task 3 - Navigation (Nav2)

1. `planner_server` (NavfnPlanner / Dijkstra) computes global paths on the global costmap
2. `controller_server` (DWB local planner) tracks the path while avoiding obstacles on the local costmap
3. `bt_navigator` orchestrates navigation with a custom behavior tree (`navigate_w_replanning_and_recovery.xml`):
   - Replans global path at 5 Hz
   - On failure: clears costmaps → spins → waits → retries (up to 6 retries)
4. `behavior_server` executes recovery primitives: `spin`, `backup`, `wait`
5. `costmap_filter_info_server` + `filter_mask_server` apply **keepout zones** to exclude forbidden areas from planning
6. `lifecycle_manager_pathplanner` activates all Nav2 nodes in the correct order

## Tasks Breakdown

### Task 1 - Mapping (`cartographer_slam`, `map_server`)

- Created `cartographer_slam` package with `cartographer.launch.py` launching `cartographer_node` + `occupancy_grid_node` + RViz
- `cartographer_sim.lua` / `cartographer_real.lua` configuration files (tracking_frame, published_frame, odom_frame, 2D trajectory builder, scan matching)
- `mapping.rviz` with Map, TF, LaserScan displays
- `use_sim_time` launch argument switches config between sim and real
- Saved maps: `warehouse_map_sim.yaml/.pgm` and `warehouse_map_real.yaml/.pgm`
- Created `map_server` package with `map_server.launch.py` launching `nav2_map_server::map_server` + `lifecycle_manager` + RViz
- `map_file` launch argument selects which map YAML to load
- `map_display.rviz` for standalone map visualization

### Task 2 - Localization (`localization_server`)

- Created `localization_server` package with `localization.launch.py` launching `map_server` + `amcl` + `lifecycle_manager_localization` + RViz
- `amcl_config_sim.yaml` / `amcl_config_real.yaml` parameters (likelihood_field laser model, differential motion model, 60 max beams, 2000-80000 particles)
- Auto-selects sim/real AMCL config based on `map_file` suffix
- TF frames: `map` → `odom` → `robot_base_footprint`
- `localization_config.rviz` with ParticleCloud display
- `attach_service_server` node also launched (from CP9/10 shelf attachment)

### Task 3 - Navigation (`path_planner_server`)

- Created `path_planner_server` package with `pathplanner.launch.py` launching:
  - `controller_server` (DWB local planner)
  - `planner_server` (NavfnPlanner)
  - `behavior_server` (spin, backup, wait recoveries)
  - `bt_navigator` (behavior tree executor)
  - `filter_mask_server` + `costmap_filter_info_server` (keepout zones)
  - `lifecycle_manager_pathplanner`
  - `rviz2`
- `controller_sim.yaml` / `controller_real.yaml` - DWB params + local costmap (voxel + inflation + keepout layers, 3x3m rolling window, 0.05m resolution, 0.25m footprint)
- `planner_sim.yaml` / `planner_real.yaml` - NavfnPlanner + global costmap (static + obstacle + inflation + keepout layers, 0.02m resolution)
- `recoveries_sim.yaml` / `recoveries_real.yaml` - recovery behavior params
- `bt_navigator_sim.yaml` / `bt_navigator_real.yaml` - BT plugin library list
- `filters_sim.yaml` / `filters_real.yaml` - keepout mask topic and YAML path
- `navigate_w_replanning_and_recovery.xml` - custom behavior tree with replanning pipeline and recovery sequence
- `pathplanning.rviz` - full Nav2 visualization (robot model, TF, laser, odom, particles, global/local costmaps, global/local paths, global/local footprints)
- `use_sim_time` launch argument switches all configs and remaps `/cmd_vel` between sim (`/diffbot_base_controller/cmd_vel_unstamped`) and real (`/cmd_vel`)

## Simulation vs Real Robot

Every package supports both environments through a single launch argument. The launch files use `PythonExpression` substitutions to dynamically select the correct config YAML and topic remapping at launch time.

| Environment | Map file | `cmd_vel` topic | Controller freq |
|---|---|---|---|
| Simulation | `warehouse_map_sim.yaml` | `/diffbot_base_controller/cmd_vel_unstamped` | 5.0 Hz |
| Real Robot | `warehouse_map_real.yaml` | `/cmd_vel` | 5.0 Hz |

## ROS 2 Interface

| Name | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` (sub) | Laser scanner data |
| `/odom` | `nav_msgs/Odometry` (sub) | Robot odometry |
| `/cmd_vel` | `geometry_msgs/Twist` (pub) | Velocity commands (remapped per environment) |
| `/map` | `nav_msgs/OccupancyGrid` (pub) | Map served by `map_server` |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` (pub) | Localized pose estimate |
| `/particle_cloud` | `nav2_msgs/ParticleCloud` (pub) | AMCL particle distribution |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` (pub) | Global costmap (map + inflation + keepout) |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` (pub) | Local costmap (voxel + inflation + keepout) |
| `/plan` | `nav_msgs/Path` (pub) | Global path from planner_server |
| `/local_plan` | `nav_msgs/Path` (pub) | Local path from controller_server |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` (action) | Send a nav goal to the BT navigator |
| `/attach_service` | `GoToLoading` (service) | Triggers shelf detection and final approach |
| `/elevator_up` | `std_msgs/String` (pub) | Elevator lift command |
| `map` → `odom` → `robot_base_footprint` | TF tree | Full navigation TF chain |

## Project Structure

```
warehouse_project/
├── cartographer_slam/              # Task 1a: SLAM mapping
│   ├── launch/cartographer.launch.py
│   ├── config/
│   │   ├── cartographer_sim.lua
│   │   ├── cartographer_real.lua
│   │   ├── warehouse_map_sim.{yaml,pgm}
│   │   └── warehouse_map_real.{yaml,pgm}
│   └── rviz/mapping.rviz
├── map_server/                     # Task 1b: Map serving
│   ├── launch/map_server.launch.py
│   ├── config/
│   │   ├── warehouse_map_{sim,real}.{yaml,pgm}
│   │   └── warehouse_map_keepout_{sim,real}.{yaml,pgm}
│   └── rviz/map_display.rviz
├── localization_server/            # Task 2: AMCL localization
│   ├── launch/localization.launch.py
│   ├── config/
│   │   ├── amcl_config_sim.yaml
│   │   └── amcl_config_real.yaml
│   └── rviz/localization_config.rviz
├── path_planner_server/            # Task 3: Nav2 navigation
│   ├── launch/pathplanner.launch.py
│   ├── config/
│   │   ├── controller_{sim,real}.yaml        # DWB local planner
│   │   ├── planner_{sim,real}.yaml           # NavfnPlanner global planner
│   │   ├── bt_navigator_{sim,real}.yaml      # Behavior tree navigator
│   │   ├── recoveries_{sim,real}.yaml        # spin/backup/wait behaviors
│   │   ├── filters_{sim,real}.yaml           # Keepout zone filters
│   │   └── navigate_w_replanning_and_recovery.xml
│   └── rviz/pathplanning.rviz
├── attach_service/                 # Shelf attach service (reused from CP9/10)
│   ├── src/attach_service_server.cpp
│   └── srv/GoToLoading.srv
├── nav2_apps/                      # Simple Commander API (Checkpoint 12 preview)
│   └── scripts/move_shelf_to_ship.py
└── media/
```

## How to Use

### Prerequisites

- ROS 2 Humble
- Gazebo Classic 11
- `cartographer_ros`, `nav2_*` packages
- RB1 robot simulation (`warehouse_rb1.launch.xml` from `the_construct_office_gazebo`)

### Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Task 1 - Mapping

```bash
# Launch simulation
ros2 launch the_construct_office_gazebo warehouse_rb1.launch.xml

# Simulation
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=True
# Real robot
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=False

# Teleop to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

# Save the map (ros2 cli)
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_map_sim

# Load the saved map
ros2 launch map_server map_server.launch.py map_file:=warehouse_map_sim.yaml
```

### Task 2 - Localization

```bash
# Simulation
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml
# Real robot
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml
```

### Task 3 - Autonomous Navigation

```bash
# Terminal 1 - Localization
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml

# Terminal 2 - Navigation stack
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=True

# In RViz, click "2D Nav Goal" and pick a point on the map.
# The robot plans + navigates autonomously with recovery on failure.
```

### Bonus - Full Warehouse Workflow (Simple Commander API)

```bash
# After launching localization + pathplanner:
python3 ~/ros2_ws/src/warehouse_project/nav2_apps/scripts/move_shelf_to_ship.py
```

## Git Branches

| Branch | Task | Description |
|---|---|---|
| `main` | All | Latest working code (default) |
| `task_1` | Task 1 | Mapping with Cartographer + map_server |
| `task_2` | Task 2 | AMCL localization |

## Key Concepts Covered

- **Cartographer SLAM**: 2D trajectory builder, online scan matching, pose graph optimization
- **AMCL localization**: likelihood-field laser model, differential motion model, particle filter convergence
- **Nav2 stack**: planner server, controller server, behavior server, bt_navigator, lifecycle manager
- **Global planner**: NavfnPlanner (Dijkstra) on global costmap with keepout zones
- **Local planner**: DWB (Dynamic Window Approach) with trajectory critics (PathAlign, GoalAlign, ObstacleFootprint, RotateToGoal)
- **Costmaps**: static layer, obstacle layer, voxel layer, inflation layer, keepout filter
- **Behavior trees**: custom XML with PipelineSequence, RecoveryNode, RateController, ClearEntireCostmap
- **Recovery behaviors**: spin, backup, wait primitives triggered on planning/following failure
- **Lifecycle management**: `lifecycle_manager` sequencing Nav2 node activation
- **Keepout zones**: costmap filter with mask topic excluding forbidden areas from planning
- **Sim/real parity**: parameterized launch files with `PythonExpression` config selection and `cmd_vel` remapping

## Technologies

- ROS 2 Humble
- Nav2 (planner_server, controller_server, bt_navigator, behavior_server, lifecycle_manager)
- Cartographer ROS
- AMCL (`nav2_amcl`)
- Gazebo Classic 11
- BehaviorTree.CPP (XML trees)
- C++ 17 / Python 3
