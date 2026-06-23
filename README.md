# isaac-husky-visual-slam-nav

RGB-D visual SLAM and autonomous navigation for a [Clearpath Husky](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/) in [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim).

The Husky carries 4x Intel RealSense D435 RGB-D cameras. Isaac Sim provides
physics, rendering, wheel odometry, and the camera streams; everything above the
simulator is ROS 2:

```
Isaac Sim ─ (ROS 2 bridge) ─> RTAB-Map ─> /map + map→odom TF ─> Nav2 ─> /cmd_vel ─> Isaac Sim
```

- **Mapping:** RTAB-Map fuses 3 cameras (front/left/right) with wheel
  odometry as the motion prior; loop closures (multi-camera PnP via OpenGV)
  correct drift. Output: a `.db` map + a 2D occupancy grid. Only 3 of the 4
  on-board cameras are used to keep GPU load manageable in simulation; on real
  hardware this can be switched to all 4.
- **Localization:** RTAB-Map reloads the `.db` and publishes `map→odom`.
- **Navigation:** Nav2 (NavFn global planner + DWB controller) consumes
  RTAB-Map's `/map` and obstacle clouds.

---

## Table of contents
1. [Prerequisites](#prerequisites)
2. [Repository layout](#repository-layout)
3. [Installation](#installation)
4. [Environment sourcing](#environment-source-this-in-every-terminal)
5. [Mapping](#mapping)
6. [Navigation](#navigation)
7. [Appendix: regenerating the Husky USD](#appendix-regenerating-the-husky-usd)

---

## Prerequisites

| Component | Notes |
|---|---|
| [Ubuntu 22.04](https://releases.ubuntu.com/22.04/) | Host OS |
| [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) | `ros-humble-desktop` (brings `xacro`, `colcon`, RViz) |
| [Nav2](https://docs.nav2.org/) | `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` |
| [RTAB-Map](https://github.com/introlab/rtabmap) | Built from source with OpenGV (see [Installation](#installation) step 3) |
| [NVIDIA Isaac Sim 4.5](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html) | Workstation install, assumed at `~/isaacsim/` (run via `~/isaacsim/python.sh`) |
| `teleop_twist_keyboard` | `sudo apt install ros-humble-teleop-twist-keyboard` |

---

## Repository layout

Only this project's own code is tracked. The Husky/RealSense/RTAB-Map repos are
cloned during setup (see `.gitignore`), and the large `maps/*.db` are generated
by mapping — never committed.

```
isaac-husky-visual-slam-nav/
├── assets/
│   ├── warehouse.usd
│   ├── husky_custom.urdf.xacro
│   ├── husky.urdf
│   └── husky/
├── scripts/
│   ├── husky_sim.py
│   ├── husky_scene.py
│   ├── drive_graph.py
│   └── camera_graph.py
├── husky_ws/
│   └── src/
│       ├── husky_custom/
│       │   ├── launch/
│       │   │   ├── slam.launch.py
│       │   │   └── navigation.launch.py
│       │   ├── config/
│       │   │   ├── nav2_params.yaml
│       │   │   ├── navigation.rviz
│       │   │   └── mapping.rviz
│       │   └── maps/
│       ├── husky/                     # to be cloned
│       └── realsense-ros/             # to be cloned
└── README.md
```

---

## Installation

### 1. Clone the repo

```bash
git clone https://github.com/vaibhavparekh9/isaac-husky-visual-slam-nav.git ~/isaac-husky-visual-slam-nav
```

### 2. Clone the robot/camera descriptions

These provide the **URDF/xacro + meshes** referenced by `assets/` (Husky body,
D435 camera). They go inside this repo's `husky_ws/src/`:

```bash
cd ~/isaac-husky-visual-slam-nav/husky_ws/src
git clone -b humble-devel      https://github.com/husky/husky.git
git clone -b ros2-development   https://github.com/IntelRealSense/realsense-ros.git
```

### 3. Build RTAB-Map

The stock binary RTAB-Map lacks OpenGV, so multi-camera loop closures could silently
fail. Build it in its own workspace with OpenGV compiled in:

```bash
mkdir -p ~/rtabmap_ws/src && cd ~/rtabmap_ws/src
git clone            https://github.com/introlab/rtabmap.git
git clone -b ros2    https://github.com/introlab/rtabmap_ros.git

cd ~/rtabmap_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y    

colcon build --packages-select rtabmap \
  --cmake-args -DWITH_OPENGV=ON -DBUILD_OPENGV=ON --cmake-force-configure

colcon build --packages-skip rtabmap
```

### 4. Build the project package

```bash
cd ~/isaac-husky-visual-slam-nav/husky_ws
source /opt/ros/humble/setup.bash
source ~/rtabmap_ws/install/setup.bash
colcon build --symlink-install --packages-select husky_custom
```

---

## Environment (source this in every terminal)

All terminals must share the same DDS (Fast DDS) and domain (0) so the Isaac
bridge and ROS nodes see each other.

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDDS_URI

source /opt/ros/humble/setup.bash
source ~/rtabmap_ws/install/setup.bash
source ~/isaac-husky-visual-slam-nav/husky_ws/install/setup.bash
```

---

## Mapping

Build a map by driving the Husky around manually.

| Terminal | Command |
|---|---|
| **T1** (Isaac Sim) | `cd ~/isaac-husky-visual-slam-nav/scripts && ~/isaacsim/python.sh husky_sim.py` | 
| **T2** (Mapping) | `ros2 launch husky_custom slam.launch.py` | 
| **T3** (Teleop) | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | 

Drive around, revisiting places so loop closures fire and the occupancy grid
fills in. Watch the map build in RViz. Once done, `Ctrl-C` T2; map will be saved to `~/.ros/rtabmap.db`.

### Save the map

RTAB-Map writes its database to `~/.ros/rtabmap.db`. Copy it into the repo's
`maps/` folder so localization can load it. The `.db` file is git-ignored because of its size:

```bash
mkdir -p ~/isaac-husky-visual-slam-nav/husky_ws/src/husky_custom/maps
cp ~/.ros/rtabmap.db \
   ~/isaac-husky-visual-slam-nav/husky_ws/src/husky_custom/maps/rtabmap.db
```

https://github.com/user-attachments/assets/089acb42-f9d9-4ccb-963a-708aa3763c8d

---

## Navigation

Localize against the saved map and navigate autonomously.

| Terminal | Command |
|---|---|
| **T1** (Isaac Sim) | `cd ~/isaac-husky-visual-slam-nav/scripts && ~/isaacsim/python.sh husky_sim.py` |
| **T2** (Localization) | `ros2 launch husky_custom slam.launch.py localization_only:=true rviz:=false database_path:=$HOME/isaac-husky-visual-slam-nav/husky_ws/src/husky_custom/maps/rtabmap.db` | 
| **T3** (Navigation) | `ros2 launch husky_custom navigation.launch.py` |

In the RViz window;
1. If the robot's pose looks off, use **2D Pose Estimate** to seed it.
2. Use **2D Goal Pose** to send a goal — the Husky plans and drives to it.

https://github.com/user-attachments/assets/3670df5e-895c-4f11-9951-710e962d86b7

---

## Appendix: regenerating the Husky USD

A ready-to-use robot USD is committed under `assets/husky/`. To regenerate it
from the source-of-truth xacro (`assets/husky_custom.urdf.xacro` = bare Husky +
4× D435 camera frames/meshes):
1. Expand the xacro → URDF.
2. Import the URDF (`assets/husky.urdf`) in Isaac Sim.
Save to `assets/husky/husky.usd`. 
