# 🚀 Quick Start: EGO-Planner + Gazebo + Map .ply

## Các bước thực hiện nhanh

### 1. Cài đặt dependencies

```bash
# Gazebo Harmonic
sudo apt-get install ros-humble-ros-gz

# Python packages
pip3 install open3d trimesh numpy-stl
```

### 2. Convert map .ply sang .dae

```bash
cd ~/ego_ws/scripts
chmod +x convert_ply_to_dae.py

# Ví dụ: Convert và giảm xuống 10000 faces
python3 convert_ply_to_dae.py \
  /path/to/your/map.ply \
  ~/ego_ws/maps/map.dae \
  --simplify 10000
```

### 3. Tạo thư mục maps

```bash
mkdir -p ~/ego_ws/maps
mkdir -p ~/ego_ws/models
mkdir -p ~/ego_ws/worlds
```

### 4. Tạo Gazebo world file

Tạo file: `~/ego_ws/worlds/my_map.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="my_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- YOUR MAP - Thay đổi path ở đây -->
    <model name="static_map">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file:///home/quangsang/ego_ws/maps/map.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file:///home/quangsang/ego_ws/maps/map.dae</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 5. Test Gazebo world

```bash
gz sim ~/ego_ws/worlds/my_map.sdf
```

Bạn nên thấy map của mình trong Gazebo!

### 6. Setup scripts

```bash
cd ~/ego_ws/scripts
chmod +x publish_static_map.py
chmod +x position_cmd_to_gazebo.py
```

### 7. Chạy hệ thống

**Terminal 1**: Gazebo
```bash
source ~/ego_ws/install/setup.bash
gz sim ~/ego_ws/worlds/my_map.sdf
```

**Terminal 2**: Publish map cho EGO-Planner
```bash
source ~/ego_ws/install/setup.bash
python3 ~/ego_ws/scripts/publish_static_map.py \
  --ros-args \
  -p map_file:=/path/to/your/map.ply \
  -p downsample_voxel_size:=0.1
```

**Terminal 3**: Bridge Gazebo odometry
```bash
source ~/ego_ws/install/setup.bash

# Cần tạo drone model trước (xem hướng dẫn chi tiết)
# Sau đó bridge topics:
ros2 run ros_gz_bridge parameter_bridge \
  /model/quadrotor/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  --ros-args \
  -r /model/quadrotor/odometry:=/drone_0_visual_slam/odom
```

**Terminal 4**: EGO-Planner (disable simulator)
```bash
source ~/ego_ws/install/setup.bash

# Sửa launch file để comment simulator nodes
# Hoặc chỉ chạy planner node
ros2 run ego_planner ego_planner_node \
  --ros-args \
  -p drone_id:=0 \
  -p max_vel:=2.0 \
  -p max_acc:=3.0
```

**Terminal 5**: RViz
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner rviz.launch.py
```

---

## ⚠️ Lưu ý quan trọng

### Map quá lớn?
```bash
# Giảm số faces khi convert
python3 convert_ply_to_dae.py map.ply map_light.dae --simplify 5000

# Hoặc downsample nhiều hơn
python3 publish_static_map.py \
  --ros-args -p downsample_voxel_size:=0.2  # 20cm thay vì 10cm
```

### Gazebo chậm?
- Giảm số faces của mesh (--simplify)
- Tắt shadows trong .sdf file
- Giảm real_time_factor xuống 0.5

### Drone không bay?
- Cần tạo drone model với motor plugin (xem GAZEBO_INTEGRATION_GUIDE.md)
- Hoặc dùng simple velocity controller

---

## 🎯 Alternative: Cách đơn giản hơn

Nếu chỉ muốn test EGO-Planner với map tĩnh (không cần Gazebo physics):

**Terminal 1**: Publish map
```bash
python3 ~/ego_ws/scripts/publish_static_map.py \
  --ros-args -p map_file:=/path/to/map.ply
```

**Terminal 2**: Fake odometry (drone tĩnh tại vị trí bắt đầu)
```bash
ros2 topic pub /drone_0_visual_slam/odom nav_msgs/msg/Odometry \
"{
  header: {frame_id: 'world'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 1.0},
      orientation: {w: 1.0}
    }
  }
}" --rate 10
```

**Terminal 3**: EGO-Planner
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner single_run_in_sim.launch.py
# (Nhớ comment map_generator trong launch file)
```

**Terminal 4**: RViz
```bash
ros2 launch ego_planner rviz.launch.py
```

Bây giờ bạn có thể visualize map và test path planning!

---

## 📋 Checklist

- [ ] Convert .ply → .dae thành công
- [ ] Map hiển thị trong Gazebo
- [ ] PointCloud được publish tới /map_generator/global_cloud
- [ ] RViz nhận được map
- [ ] Odometry được publish
- [ ] EGO-Planner chạy không lỗi

---

Xem hướng dẫn đầy đủ tại: `GAZEBO_INTEGRATION_GUIDE.md`
