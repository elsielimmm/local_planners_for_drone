# ⚡ Quick Start - EGO-Swarm với X500 + OpenVINS

## 🎯 Mục tiêu

Chạy **EGO-Swarm planning** với **X500 Gazebo** và **OpenVINS VIO** thay vì simulator cũ.

---

## ✅ Đã Fix

Bạn đã sửa trong `single_run_in_sim.launch.py`:

1. ✅ **Typo dòng 18**: `ov_mscfkf/odomimu` → `ov_msckf/odomimu`
2. ✅ **Camera topics (dòng 105-107)**: Đã update từ `pcl_render_node/*` sang `gz/drone1/*`

---

## 📋 Các Topic Mapping

| EGO-Swarm cần | Bạn có sẵn | Giải pháp |
|---------------|------------|-----------|
| `drone_0_visual_slam/odom` | `/ov_msckf/odomimu` | ✅ Bridge node `odom_converter` |
| `drone_0_camera/pose` | TF: `world->drone1/camera_link` | ✅ Bridge node `camera_pose_publisher` |
| `drone_0_*_render_node/depth` | `/gz/drone1/camera_depth/image_raw` | ✅ Đã remap trong launch |
| `drone_0_*_render_node/cloud` | `/gz/drone1/camera_depth/points` | ✅ Đã remap trong launch |
| `/fmu/in/trajectory_setpoint` | `/drone_0_planning/pos_cmd` | ✅ Bridge node `x500_cmd_bridge` |

---

## 🛠️ Installation Steps

### Bước 1: Tạo Bridge Packages

```bash
cd ~/ego_ws/src/ego-planner-swarm/src/uav_simulator/Utils

# 1. Odometry Bridge
ros2 pkg create --build-type ament_cmake odom_bridge \
  --dependencies rclcpp nav_msgs

# 2. Camera Bridge
ros2 pkg create --build-type ament_cmake camera_bridge \
  --dependencies rclcpp geometry_msgs tf2_ros tf2_geometry_msgs

# 3. X500 Command Bridge (nếu chưa có)
ros2 pkg create --build-type ament_cmake x500_bridge \
  --dependencies rclcpp quadrotor_msgs px4_msgs geometry_msgs nav_msgs
```

### Bước 2: Copy Source Code

#### odom_bridge/src/odom_converter.cpp

```bash
mkdir -p odom_bridge/src
nano odom_bridge/src/odom_converter.cpp
```

Paste code từ `TOPIC_REMAP_GUIDE.md` section "Bridge Node 1".

#### camera_bridge/src/camera_pose_publisher.cpp

```bash
mkdir -p camera_bridge/src
nano camera_bridge/src/camera_pose_publisher.cpp
```

Paste code từ `TOPIC_REMAP_GUIDE.md` section "Bridge Node 2".

#### x500_bridge/src/x500_cmd_bridge.cpp

```bash
mkdir -p x500_bridge/src
nano x500_bridge/src/x500_cmd_bridge.cpp
```

Paste code từ `HOW_EGO_CONTROLS_DRONE.md` hoặc `GAZEBO_X500_OPENVINS_INTEGRATION.md`.

### Bước 3: Update CMakeLists.txt và package.xml

Cho mỗi package, copy CMakeLists.txt và package.xml từ documents.

### Bước 4: Build

```bash
cd ~/ego_ws

# Build all bridge packages
colcon build --packages-select odom_bridge camera_bridge x500_bridge

# Source
source install/setup.bash
```

---

## 🚀 Running the System

### Terminal 1: Gazebo X500

```bash
# Launch your X500 Gazebo simulation
# (Thay command này bằng launch file X500 của bạn)
ros2 launch your_x500_package x500_gazebo.launch.py
```

**Verify:**
```bash
# Check X500 topics exist
ros2 topic list | grep -E "fmu|gz/drone"
```

### Terminal 2: OpenVINS

```bash
# Launch OpenVINS VIO
ros2 launch ov_msckf your_openvins_config.launch.py
```

**Verify:**
```bash
# Check OpenVINS publishing odometry
ros2 topic hz /ov_msckf/odomimu
# Expected: 30-60 Hz

# Check odometry quality
ros2 topic echo /ov_msckf/odomimu --field pose.pose.position
```

### Terminal 3: Bridge Nodes

```bash
cd ~/ego_ws
source install/setup.bash

# Option A: Launch all bridges together
ros2 launch ego_planner x500_bridges.launch.py

# Option B: Launch individually
ros2 run odom_bridge odom_converter &
ros2 run camera_bridge camera_pose_publisher &
ros2 run x500_bridge x500_cmd_bridge &
```

**Verify:**
```bash
# Check odom converter
ros2 topic hz /drone_0_visual_slam/odom  # Should match OpenVINS rate

# Check camera pose
ros2 topic hz /drone_0_camera/pose  # Should be ~30 Hz

# Check command bridge ready
ros2 node list | grep x500_cmd_bridge
```

### Terminal 4: EGO-Swarm Planning

```bash
cd ~/ego_ws
source install/setup.bash

ros2 launch ego_planner single_run_in_sim.launch.py \
  odom_topic:=visual_slam/odom \
  drone_id:=0 \
  use_mockamap:=true \
  map_size_x:=30.0 \
  map_size_y:=30.0 \
  map_size_z:=3.0
```

**Verify:**
```bash
# Check planner node
ros2 node list | grep ego_planner

# Check if receiving odometry
ros2 topic echo /drone_0_visual_slam/odom --once

# Check if depth camera working
ros2 topic hz /gz/drone1/camera_depth/image_raw
```

### Terminal 5: RViz Visualization

```bash
rviz2
```

**Add these topics:**
- `/drone_0_planning/bspline` - Trajectory
- `/drone_0_plan_vis/*` - Planning visualization
- `/drone_0_vis/path` - Path
- `/map_generator/global_cloud` - Map
- `/drone_0_grid/grid_map/occupancy_inflate` - Occupancy grid
- `/ov_msckf/pathimu` - VIO path

---

## 🎯 Setting Goal

### Option 1: RViz 2D Nav Goal

1. Open RViz
2. Click "2D Nav Goal" button in toolbar
3. Click and drag on map

### Option 2: Command Line

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, 
    pose: {position: {x: 5.0, y: 3.0, z: 1.5}}}"
```

### Option 3: Interactive Script

```bash
# Tạo script set_goal_interactive.sh
nano ~/ego_ws/scripts/set_goal_interactive.sh
```

```bash
#!/bin/bash
echo "Enter goal position:"
read -p "X: " x
read -p "Y: " y
read -p "Z: " z

ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world', stamp: {sec: 0, nanosec: 0}}, 
    pose: {position: {x: $x, y: $y, z: $z}, 
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

echo "Goal set to [$x, $y, $z]"
```

```bash
chmod +x ~/ego_ws/scripts/set_goal_interactive.sh
~/ego_ws/scripts/set_goal_interactive.sh
```

---

## ✅ Verification Checklist

### 1. OpenVINS Working

```bash
# Odometry publishing
ros2 topic hz /ov_msckf/odomimu  # > 20 Hz

# VIO tracking
ros2 topic echo /ov_msckf/odomimu --field pose.pose.position
# Position should change as drone moves
```

### 2. Bridge Nodes Working

```bash
# Odom converter
ros2 topic hz /drone_0_visual_slam/odom  # Should match OpenVINS rate

# Camera pose
ros2 topic hz /drone_0_camera/pose  # ~30 Hz

# Nodes running
ros2 node list | grep -E "odom_converter|camera_pose|x500_cmd"
```

### 3. EGO-Swarm Receiving Data

```bash
# Check node info
ros2 node info /drone_0_ego_planner_node

# Should show subscribers:
#   /drone_0_visual_slam/odom
#   /gz/drone1/camera_depth/image_raw
#   /drone_0_camera/pose

# Check if planning active
ros2 topic hz /drone_0_planning/bspline
# Will show Hz when trajectory generated
```

### 4. PX4 Receiving Commands

```bash
# After setting goal and trajectory generated:
ros2 topic hz /fmu/in/trajectory_setpoint  # ~30 Hz

# Check PX4 mode
ros2 topic echo /fmu/out/vehicle_status --field nav_state
# Should be 14 (OFFBOARD) when x500_cmd_bridge arms it

# Check position changing
ros2 topic echo /fmu/out/vehicle_local_position --field xy
```

---

## 🐛 Troubleshooting

### Problem: "No odom received"

**Cause:** Bridge node not running or topic mismatch

**Fix:**
```bash
# Check topics
ros2 topic list | grep odom

# Restart bridge
ros2 run odom_bridge odom_converter \
  --ros-args -p input_topic:=/ov_msckf/odomimu \
              -p output_topic:=/drone_0_visual_slam/odom

# Check remapping in ego_planner_node
ros2 node info /drone_0_ego_planner_node | grep -A 20 Subscriptions
```

### Problem: "Cannot get camera pose"

**Cause:** TF not available or camera_pose_publisher not running

**Fix:**
```bash
# Check TF
ros2 run tf2_tools view_frames
# Open frames.pdf - should show: world -> drone1 -> camera_link

# Check camera pose publishing
ros2 topic echo /drone_0_camera/pose --once

# Restart camera bridge
ros2 run camera_bridge camera_pose_publisher \
  --ros-args -p world_frame:=world \
              -p camera_frame:=drone1/camera_link
```

### Problem: "X500 not flying"

**Cause:** Not armed or not in offboard mode

**Fix:**
```bash
# Check current status
ros2 topic echo /fmu/out/vehicle_status

# Manually arm
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{command: 400, param1: 1.0}"

# Set offboard
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{command: 176, param1: 1.0, param2: 6.0}"

# Check if x500_cmd_bridge is running
ros2 node list | grep x500
```

### Problem: "Planning but drone flying wrong direction"

**Cause:** Frame conversion error (ENU vs NED)

**Fix:**
- Check `x500_cmd_bridge.cpp` frame conversion code
- Verify: `traj.position[0] = msg->position.y` (North = Y_enu)
- Verify: `traj.position[2] = -msg->position.z` (Down = -Z_enu)

### Problem: "Collision with obstacles that don't exist"

**Cause:** Depth camera calibration or FOV mismatch

**Fix:**
```bash
# Check camera intrinsics in launch file match actual camera
ros2 topic echo /gz/drone1/camera_left/color/camera_info

# Update in single_run_in_sim.launch.py:
# 'cx': str(actual_cx_from_camera_info)
# 'fx': str(actual_fx_from_camera_info)
```

---

## 📊 Expected Topic Rates

| Topic | Rate | Description |
|-------|------|-------------|
| `/ov_msckf/odomimu` | 30-60 Hz | OpenVINS odometry |
| `/drone_0_visual_slam/odom` | 30-60 Hz | Converted odometry |
| `/gz/drone1/camera_depth/image_raw` | 10-30 Hz | Depth images |
| `/drone_0_camera/pose` | 30 Hz | Camera pose |
| `/drone_0_planning/bspline` | ~1 Hz | Trajectory updates |
| `/drone_0_planning/pos_cmd` | 30 Hz | Position commands |
| `/fmu/in/trajectory_setpoint` | 30 Hz | PX4 commands |
| `/fmu/out/vehicle_odometry` | 50 Hz | PX4 state estimate |

---

## 🎬 Complete Launch Sequence (Single Script)

Tạo một script để launch tất cả:

```bash
nano ~/ego_ws/launch_full_system.sh
```

```bash
#!/bin/bash

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting X500 + OpenVINS + EGO-Swarm System${NC}"

# Terminal 1: Gazebo
echo -e "${YELLOW}[1/5] Launching Gazebo X500...${NC}"
gnome-terminal -- bash -c "ros2 launch your_x500_package x500_gazebo.launch.py; exec bash"
sleep 5

# Terminal 2: OpenVINS
echo -e "${YELLOW}[2/5] Launching OpenVINS...${NC}"
gnome-terminal -- bash -c "ros2 launch ov_msckf your_config.launch.py; exec bash"
sleep 3

# Wait for OpenVINS to initialize
echo "Waiting for OpenVINS initialization..."
while ! ros2 topic hz /ov_msckf/odomimu --once 2>/dev/null; do
    sleep 1
done
echo -e "${GREEN}OpenVINS ready!${NC}"

# Terminal 3: Bridge nodes
echo -e "${YELLOW}[3/5] Launching bridge nodes...${NC}"
gnome-terminal -- bash -c "source ~/ego_ws/install/setup.bash && \
  ros2 launch ego_planner x500_bridges.launch.py; exec bash"
sleep 2

# Terminal 4: EGO-Swarm
echo -e "${YELLOW}[4/5] Launching EGO-Swarm planner...${NC}"
gnome-terminal -- bash -c "source ~/ego_ws/install/setup.bash && \
  ros2 launch ego_planner single_run_in_sim.launch.py \
    odom_topic:=visual_slam/odom drone_id:=0 use_mockamap:=true; exec bash"
sleep 3

# Terminal 5: RViz
echo -e "${YELLOW}[5/5] Launching RViz...${NC}"
gnome-terminal -- bash -c "rviz2; exec bash"

echo -e "${GREEN}System launched successfully!${NC}"
echo -e "Set goal using: ros2 topic pub --once /goal_pose ..."
```

```bash
chmod +x ~/ego_ws/launch_full_system.sh
```

---

## 🎯 Summary

**Cần làm:**
1. ✅ Tạo 3 bridge packages (odom_bridge, camera_bridge, x500_bridge)
2. ✅ Copy source code vào từ docs
3. ✅ Build packages
4. ✅ Launch system theo thứ tự: Gazebo → OpenVINS → Bridges → Planning
5. ✅ Set goal và test

**Documents reference:**
- Topic mapping chi tiết: `TOPIC_REMAP_GUIDE.md`
- Control mechanism: `HOW_EGO_CONTROLS_DRONE.md`
- Full integration: `GAZEBO_X500_OPENVINS_INTEGRATION.md`
- Mermaid diagrams: `MERMAID_DIAGRAMS.md`

**Next:** Hãy bắt đầu với bước 1 - tạo bridge packages!
