# 🚁 Hướng dẫn Setup EGO-Planner với Gazebo

## 📋 Checklist - Bạn đang thiếu gì?

### ✅ Đã có:
- [x] Gazebo chạy với drone model
- [x] Offboard control node (`px4_offboard_path`)
- [x] Remappings trong launch files
- [x] Goal topic (`/goal_pose`)

### ❌ Còn thiếu:
1. **Gazebo Bridge** - Chưa chạy!
2. **Topic remapping** cho `grid_map/cloud`
3. **Map generator** hoặc publish static map

---

## 🔧 Các bước setup

### 1. Terminal 1: Gazebo
```bash
cd ~/ego_ws
source install/setup.bash
gz sim your_world.sdf
```

### 2. Terminal 2: **GAZEBO BRIDGE** ⚠️ QUAN TRỌNG!

Đây là phần **BẠN ĐANG THIẾU**!

```bash
cd ~/ego_ws
source install/setup.bash

# Chạy bridge script
python3 scripts/ego_gz_bridge.py

# HOẶC chạy thủ công:
ros2 run ros_gz_bridge parameter_bridge \
  /model/x500_mono_cam_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  --ros-args \
  -r /model/x500_mono_cam_0/odometry:=/model/x500_mono_cam_0/odom \
  -r /depth_camera/points:=/depth_camera/points
```

**Bridge này làm gì?**
- Bridge `/model/x500_mono_cam_0/odometry` (GZ) → `/model/x500_mono_cam_0/odom` (ROS2)
- Bridge `/depth_camera/points` (GZ PointCloud) → `/depth_camera/points` (ROS2)

### 3. Terminal 3: Map Generator (hoặc Static Map)

**Option A: Map Generator (random forest)**
```bash
cd ~/ego_ws
source install/setup.bash

ros2 run map_generator random_forest \
  --ros-args \
  -p map/x_size:=50.0 \
  -p map/y_size:=50.0 \
  -p map/z_size:=10.0 \
  -p map/resolution:=0.1 \
  -p sensing/radius:=5.0 \
  -p sensing/rate:=10.0
```

**Option B: Publish Static Map từ file .ply**
```bash
cd ~/ego_ws
source install/setup.bash

python3 scripts/publish_static_map.py \
  --ros-args \
  -p map_file:=/path/to/your/map.ply \
  -p downsample_voxel_size:=0.1
```

**Map phải được publish vào:**
- Topic: `/map_generator/global_cloud`
- Type: `sensor_msgs/msg/PointCloud2`
- Frame: `world`

### 4. Terminal 4: EGO Planner
```bash
cd ~/ego_ws
source install/setup.bash

ros2 launch ego_planner gazebo_integration.launch.py \
  drone_id:=0 \
  map_size_x:=50.0 \
  map_size_y:=50.0 \
  map_size_z:=10.0
```

### 5. Terminal 5: Offboard Control
```bash
cd ~/ego_ws
source install/setup.bash

ros2 run offboard_control px4_offboard_path \
  --ros-args \
  -p use_pos_cmd:=true \
  -p pos_cmd_topic:=/drone_0_planning/pos_cmd \
  -p use_test_path:=false
```

### 6. Terminal 6: RViz (Optional)
```bash
cd ~/ego_ws
source install/setup.bash

ros2 launch ego_planner rviz.launch.py
```

---

## 🔍 Kiểm tra các topic

```bash
# Kiểm tra bridge đang chạy
ros2 topic list | grep -E "(odom|cloud|depth)"

# Phải thấy:
# /model/x500_mono_cam_0/odom ✅
# /depth_camera/points ✅

# Kiểm tra map
ros2 topic echo /map_generator/global_cloud --once

# Kiểm tra planner output
ros2 topic echo /drone_0_planning/pos_cmd

# Kiểm tra goal
ros2 topic echo /goal_pose
```

---

## 🎯 Test với Goal

### Set goal bằng RViz:
1. Mở RViz
2. Click "2D Nav Goal"
3. Click và kéo trên map để set goal

### Set goal bằng terminal:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{
  header: {frame_id: 'world'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 1.5},
    orientation: {w: 1.0}
  }
}"
```

---

## ⚠️ Troubleshooting

### Lỗi: "No odom received"
- **Nguyên nhân**: Bridge chưa chạy hoặc Gazebo chưa publish
- **Giải pháp**: 
  ```bash
  ros2 topic list | grep odom
  ros2 topic echo /model/x500_mono_cam_0/odom
  ```

### Lỗi: "GridMap has no cloud data"
- **Nguyên nhân**: `grid_map/cloud` topic chưa remap đúng
- **Giải pháp**: Đã fix trong `advanced_param.launch.py` line 109:
  ```python
  ('grid_map/cloud', '/depth_camera/points'),  # ✅ FIXED
  ```

### Lỗi: "No planning output"
- **Nguyên nhân**: Planner chưa nhận được odom hoặc map
- **Giải pháp**: Check topics:
  ```bash
  ros2 topic hz /model/x500_mono_cam_0/odom      # Phải ~100Hz
  ros2 topic hz /depth_camera/points              # Phải ~30Hz
  ros2 topic hz /map_generator/global_cloud      # Phải publish
  ```

### Drone không bay
- **Kiểm tra**: `/drone_0_planning/pos_cmd` có publish không?
  ```bash
  ros2 topic echo /drone_0_planning/pos_cmd
  ```
- **Kiểm tra**: Offboard control có nhận được `pos_cmd`?
  ```bash
  ros2 topic echo /fmu/in/trajectory_setpoint
  ```

---

## 📊 Flow Diagram

```
Gazebo World
    │
    ├─> Bridge ──> /model/x500_mono_cam_0/odom ─┐
    │                                             │
    └─> Bridge ──> /depth_camera/points ────────┼─> EGO Planner ─> /drone_0_planning/pos_cmd ─> Offboard Control ─> /fmu/in/trajectory_setpoint
                                                  │
Map Generator ─> /map_generator/global_cloud ─────┘
                                                  │
User ─> /goal_pose ──────────────────────────────┘
```

---

## 🎉 Sau khi setup xong

1. ✅ Bridge đang chạy
2. ✅ Map được publish
3. ✅ EGO Planner đang chạy
4. ✅ Offboard control đang chạy
5. Set goal và xem drone bay!

---

**Good luck!** 🚀
