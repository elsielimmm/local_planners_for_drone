# Quick Start: Tích hợp X500 + OpenVINS vào EGO-Swarm

## TÓM TẮT

Thay thế simulator giả lập bằng Gazebo X500 thực tế và dùng OpenVINS cho VIO.

---

## BƯỚC 1: Tạo Bridge Package (5 phút)

```bash
cd ~/ego_ws/src/ego-planner-swarm/src/uav_simulator/Utils

# Tạo package
ros2 pkg create --build-type ament_cmake x500_bridge \
  --dependencies rclcpp nav_msgs quadrotor_msgs px4_msgs geometry_msgs

cd x500_bridge
mkdir src
```

---

## BƯỚC 2: Tạo 2 file C++ chính (10 phút)

### File 1: `src/odom_converter.cpp`

Copy code từ `GAZEBO_X500_OPENVINS_INTEGRATION.md` phần 4.2

**Chức năng**: Convert OpenVINS odometry → EGO-Swarm format

### File 2: `src/x500_cmd_bridge.cpp`

Copy code từ `GAZEBO_X500_OPENVINS_INTEGRATION.md` phần 4.1

**Chức năng**: Convert EGO-Swarm commands → PX4 X500 format

---

## BƯỚC 3: Sửa CMakeLists.txt (2 phút)

File: `x500_bridge/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(x500_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Odom Converter
add_executable(odom_converter src/odom_converter.cpp)
ament_target_dependencies(odom_converter
  rclcpp nav_msgs px4_msgs
)

# Command Bridge
add_executable(x500_cmd_bridge src/x500_cmd_bridge.cpp)
ament_target_dependencies(x500_cmd_bridge
  rclcpp quadrotor_msgs px4_msgs geometry_msgs
)

install(TARGETS
  odom_converter
  x500_cmd_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## BƯỚC 4: Build (3 phút)

```bash
cd ~/ego_ws

# Build chỉ package mới
colcon build --packages-select x500_bridge

# Build toàn bộ (nếu có lỗi dependencies)
colcon build

source install/setup.bash
```

---

## BƯỚC 5: Test Topics Mapping (10 phút)

### 5.1 Kiểm tra X500 Topics

```bash
# Terminal 1: Khởi động X500
ros2 launch <your_px4_package> x500.launch.py

# Terminal 2: List topics
ros2 topic list | grep -E "odom|cmd|trajectory"

# Ghi chú lại:
# - Odometry topic: /fmu/out/vehicle_odometry
# - Command topic: /fmu/in/trajectory_setpoint
```

### 5.2 Kiểm tra OpenVINS Topics

```bash
# Terminal 1: X500 (giữ nguyên)

# Terminal 2: OpenVINS
ros2 launch ov_msckf <your_openvins_launch>

# Terminal 3: Check
ros2 topic list | grep odom
# Ghi chú: /ov_msckf/odometry_imu
```

---

## BƯỚC 6: Chạy Bridge Nodes (5 phút)

### Terminal 1: X500 Gazebo
```bash
ros2 launch <your_px4_package> x500.launch.py
```

### Terminal 2: OpenVINS
```bash
ros2 launch ov_msckf <your_config>.launch.py
```

### Terminal 3: Odometry Converter
```bash
ros2 run x500_bridge odom_converter \
  --ros-args \
  -p input_topic:=/ov_msckf/odometry_imu \
  -p output_topic:=/drone_0_visual_slam/odom \
  -p use_px4_odom:=false
```

### Terminal 4: Command Bridge
```bash
ros2 run x500_bridge x500_cmd_bridge
```

### Terminal 5: Verify
```bash
# Check odometry
ros2 topic echo /drone_0_visual_slam/odom --once

# Check Hz
ros2 topic hz /drone_0_visual_slam/odom
```

---

## BƯỚC 7: Launch EGO-Planner (không simulator) (5 phút)

### Tạo launch file đơn giản

File: `src/planner/plan_manage/launch/ego_with_x500.launch.py`

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Chỉ launch phần planning, KHÔNG launch simulator
    ego_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ego_planner'),
                'launch',
                'advanced_param.launch.py'
            ])
        ]),
        launch_arguments={
            'drone_id': '0',
            'map_size_x_': '50.0',
            'map_size_y_': '25.0',
            'map_size_z_': '5.0',
            'odometry_topic': '/drone_0_visual_slam/odom',
            'depth_topic': '/x500/camera/depth',  # Từ X500
            'cloud_topic': 'pcl_render_node/cloud',
        }.items()
    )
    
    return LaunchDescription([ego_planner])
```

### Chạy
```bash
ros2 launch ego_planner ego_with_x500.launch.py
```

---

## BƯỚC 8: Test Full System (10 phút)

### Scenario 1: Single Waypoint

```bash
# Terminal 1: X500
ros2 launch <px4> x500.launch.py

# Terminal 2: OpenVINS
ros2 launch ov_msckf <config>.launch.py

# Terminal 3: Bridges
ros2 run x500_bridge odom_converter --ros-args -p input_topic:=/ov_msckf/odometry_imu -p output_topic:=/drone_0_visual_slam/odom
# Mở terminal khác
ros2 run x500_bridge x500_cmd_bridge

# Terminal 4: Map
ros2 run mockamap mockamap_node

# Terminal 5: EGO-Planner
ros2 launch ego_planner ego_with_x500.launch.py

# Terminal 6: RViz
ros2 launch ego_planner rviz.launch.py

# Terminal 7: Set goal
ros2 topic pub --once /goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 5.0, y: 5.0, z: 1.0}}}"
```

---

## CHECKLIST DEBUG

Nếu không hoạt động, kiểm tra:

### ✓ Odometry
```bash
ros2 topic echo /drone_0_visual_slam/odom --once
# Phải thấy position, velocity, orientation
```

### ✓ Planning
```bash
ros2 topic echo /drone_0_planning/bspline --once
# Phải thấy trajectory khi set goal
```

### ✓ Command
```bash
ros2 topic echo /fmu/in/trajectory_setpoint --once
# Phải thấy position, velocity commands
```

### ✓ X500 Moving
```bash
# Trong Gazebo, X500 phải di chuyển
# Kiểm tra bằng mắt hoặc:
ros2 topic echo /fmu/out/vehicle_odometry --field position
```

---

## LƯU Ý QUAN TRỌNG

### 1. Frame Convention
- **PX4**: NED (North-East-Down)
- **ROS2**: ENU (East-North-Up)
- **OpenVINS**: Thường là ENU, nhưng cần verify

### 2. Topic Remapping
Đảm bảo topic names khớp với EGO-Swarm expectations:
- `odom_world` hoặc `/drone_X_visual_slam/odom`
- `planning/pos_cmd`
- `depth` hoặc camera depth topic

### 3. Timestamp
OpenVINS và PX4 có thể có clock khác nhau. Nếu có warning về time, dùng:
```cpp
out_msg.header.stamp = this->now();  // ROS2 clock
```

### 4. Depth Camera
Nếu X500 không có depth camera:
- **Option 1**: Thêm Gazebo depth plugin (xem GAZEBO_X500_OPENVINS_INTEGRATION.md)
- **Option 2**: Dùng `pcl_render_node` của EGO-Swarm

---

## COMMON ERRORS

### Error 1: "No odom received"
**Fix**: Check topic name mapping trong `odom_converter` parameters

### Error 2: "X500 not moving"
**Fix**: 
1. Check `/fmu/in/offboard_control_mode` được publish
2. Check PX4 trong offboard mode: `commander offboard`

### Error 3: "Planning fails"
**Fix**: Check map topic `/map_generator/global_cloud` có data

### Error 4: "Transform error"
**Fix**: Publish static transform:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
```

---

## NEXT STEPS

Sau khi single drone hoạt động:

1. **Multi-drone Swarm**: Duplicate và remap topics cho drone_1, drone_2...
2. **Real X500**: Thay Gazebo bằng hardware real
3. **Tune Parameters**: Điều chỉnh planner params cho phù hợp với X500 dynamics

---

## TÀI LIỆU THAM KHẢO

- Chi tiết đầy đủ: `GAZEBO_X500_OPENVINS_INTEGRATION.md`
- EGO-Swarm topics: `TOPIC_GOAL_DETAILS.md`
- PX4 offboard: https://docs.px4.io/main/en/flight_modes/offboard.html
