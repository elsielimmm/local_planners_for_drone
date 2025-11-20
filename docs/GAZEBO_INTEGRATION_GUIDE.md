# 🚁 Hướng dẫn Tích hợp EGO-Planner với Gazebo + Map Tĩnh (.ply)

## 📋 Tổng quan

Bạn muốn:
- ✅ Load map tĩnh từ file `.ply` (đã quét sẵn)
- ✅ Chạy drone trong **Gazebo** (thay vì simulator đơn giản)
- ✅ Sử dụng EGO-Planner để navigate trong Gazebo

## 🎯 Kiến trúc tổng thể

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Gazebo        │────▶│  Topic Bridge    │────▶│  EGO-Planner    │
│  - Drone model  │     │  - Odometry      │     │  - Path plan    │
│  - Physics      │     │  - Depth camera  │     │  - Collision    │
│  - Sensors      │     │  - Commands      │     │  - Optimization │
└─────────────────┘     └──────────────────┘     └─────────────────┘
         ▲                                                │
         │                                                │
         └────────────────────────────────────────────────┘
                        Position commands
                        
┌─────────────────────────────────────────────────────────┐
│  Static Map (.ply)                                      │
│  - Load vào Gazebo world (as mesh)                     │
│  - Publish PointCloud2 cho EGO-Planner                 │
└─────────────────────────────────────────────────────────┘
```

---

## 🔧 BƯỚC 1: Cài đặt Dependencies

### 1.1. Cài Gazebo Harmonic (cho ROS 2 Humble)

```bash
# Cài Gazebo Harmonic
sudo apt-get update
sudo apt-get install ros-humble-ros-gz

# Kiểm tra
gz sim --version
```

### 1.2. Cài các tools cần thiết

```bash
# PCL tools để xử lý pointcloud
sudo apt-get install pcl-tools

# MeshLab (optional - để preview/edit mesh)
sudo apt-get install meshlab

# Python dependencies
pip3 install open3d trimesh numpy-stl
```

---

## 📦 BƯỚC 2: Convert Map .ply sang Format Gazebo

Gazebo có thể load map theo **3 cách**:

### **Cách 1: Mesh (.dae, .stl, .obj) - Khuyến nghị**

Convert `.ply` → `.dae` (COLLADA format):

```python
#!/usr/bin/env python3
# File: convert_ply_to_dae.py

import open3d as o3d
import trimesh

# Load PLY
print("Loading .ply file...")
mesh = trimesh.load('/path/to/your/map.ply')

# Downsample nếu quá nặng
# mesh = mesh.simplify_quadric_decimation(face_count=50000)

# Export sang .dae (COLLADA)
output_path = '/path/to/output/map.dae'
mesh.export(output_path)
print(f"Saved to {output_path}")

# Hoặc export sang .stl
# mesh.export('/path/to/output/map.stl')
```

Chạy:
```bash
python3 convert_ply_to_dae.py
```

**Lưu ý**: Nếu file quá lớn (>100MB), cần downsample:
```python
mesh = mesh.simplify_quadric_decimation(face_count=10000)
```

### **Cách 2: Heightmap (cho địa hình phẳng)**

Nếu map của bạn chủ yếu là mặt đất/địa hình:

```python
#!/usr/bin/env python3
# File: convert_ply_to_heightmap.py

import open3d as o3d
import numpy as np
from PIL import Image

pcd = o3d.io.read_point_cloud('/path/to/your/map.ply')
points = np.asarray(pcd.points)

# Tạo heightmap (grid 2D với giá trị Z)
resolution = 0.1  # 10cm per pixel
x_min, y_min = points[:, :2].min(axis=0)
x_max, y_max = points[:, :2].max(axis=0)

width = int((x_max - x_min) / resolution)
height = int((y_max - y_min) / resolution)

heightmap = np.zeros((height, width))

for p in points:
    xi = int((p[0] - x_min) / resolution)
    yi = int((p[1] - y_min) / resolution)
    if 0 <= xi < width and 0 <= yi < height:
        heightmap[yi, xi] = max(heightmap[yi, xi], p[2])

# Normalize về 0-255
heightmap_norm = ((heightmap - heightmap.min()) / 
                  (heightmap.max() - heightmap.min()) * 255).astype(np.uint8)

# Save as PNG
Image.fromarray(heightmap_norm).save('/path/to/output/heightmap.png')
print("Heightmap saved!")
```

### **Cách 3: PointCloud trực tiếp (cho EGO-Planner)**

Không cần convert, dùng script đã tạo ở file trước.

---

## 🌍 BƯỚC 3: Tạo Gazebo World với Map

### 3.1. Tạo file world

Tạo file: `~/ego_ws/src/ego-planner-swarm/worlds/custom_map.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="custom_map_world">
    
    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground plane (optional) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- CUSTOM MAP - Thay đổi path tới file .dae của bạn -->
    <model name="custom_map">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file:///home/quangsang/maps/map.dae</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file:///home/quangsang/maps/map.dae</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
```

**Thay đổi**: 
- Dòng 52, 58: Đổi path `file:///home/quangsang/maps/map.dae` thành path file của bạn
- Scale: Nếu map quá lớn/nhỏ, đổi `<scale>1 1 1</scale>`

### 3.2. Test Gazebo world

```bash
gz sim ~/ego_ws/src/ego-planner-swarm/worlds/custom_map.sdf
```

Bạn sẽ thấy map của mình trong Gazebo!

---

## 🚁 BƯỚC 4: Tạo Drone Model cho Gazebo

### 4.1. Download quadrotor model

```bash
cd ~/ego_ws/src
git clone https://github.com/PX4/PX4-gazebo-models.git

# Hoặc dùng model đơn giản hơn
mkdir -p ~/ego_ws/models
cd ~/ego_ws/models
```

### 4.2. Tạo drone model đơn giản

Tạo file: `~/ego_ws/models/simple_quadrotor/model.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="simple_quadrotor">
    <pose>0 0 0.5 0 0 0</pose>
    
    <!-- Base link -->
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.02</izz>
        </inertia>
      </inertial>
      
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.1</size>
          </box>
        </geometry>
      </collision>
      
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
      
      <!-- Depth camera sensor -->
      <sensor name="depth_camera" type="depth_camera">
        <pose>0.15 0 0 0 0 0</pose>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R_FLOAT32</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10.0</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <visualize>true</visualize>
      </sensor>
      
      <!-- IMU sensor -->
      <sensor name="imu" type="imu">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
      </sensor>
    </link>
    
    <!-- Plugin for control -->
    <plugin
      filename="gz-sim-multicopter-motor-model-system"
      name="gz::sim::systems::MulticopterMotorModel">
      <robotNamespace>drone_0</robotNamespace>
      <jointName>rotor_0_joint</jointName>
      <linkName>rotor_0</linkName>
      <turningDirection>ccw</turningDirection>
      <motorNumber>0</motorNumber>
    </plugin>
    
  </model>
</sdf>
```

---

## 🔗 BƯỚC 5: Bridge Gazebo ↔ ROS 2

### 5.1. Tạo launch file cho Gazebo + Bridge

Tạo file: `~/ego_ws/src/ego-planner-swarm/launch/gazebo_ego_planner.launch.py`

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Paths
    world_file = '/home/quangsang/ego_ws/src/ego-planner-swarm/worlds/custom_map.sdf'
    
    # Launch arguments
    drone_id = LaunchConfiguration('drone_id', default='0')
    init_x = LaunchConfiguration('init_x', default='0.0')
    init_y = LaunchConfiguration('init_y', default='0.0')
    init_z = LaunchConfiguration('init_z', default='1.0')
    
    return LaunchDescription([
        
        # 1. Start Gazebo with custom world
        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '-r'],
            output='screen'
        ),
        
        # 2. Spawn drone model
        ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/world/custom_map_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req',
                f'sdf_filename: "/home/quangsang/ego_ws/models/simple_quadrotor/model.sdf", '
                f'pose: {{position: {{x: {init_x}, y: {init_y}, z: {init_z}}}}}'
            ],
            output='screen'
        ),
        
        # 3. Bridge Gazebo topics to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Odometry
                '/model/simple_quadrotor/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                # Depth camera
                '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloud2',
                # Commands (ROS -> Gazebo)
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            output='screen',
            remappings=[
                ('/model/simple_quadrotor/odometry', '/drone_0_visual_slam/odom'),
                ('/depth_camera/points', '/drone_0_pcl_render_node/cloud'),
            ]
        ),
        
        # 4. Publish static map pointcloud
        Node(
            package='ego_planner',
            executable='publish_static_map.py',
            name='static_map_publisher',
            output='screen',
            parameters=[{
                'map_file': '/home/quangsang/maps/map.ply',
                'publish_rate': 1.0
            }]
        ),
        
        # 5. Include EGO-Planner
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ego_planner'),
                    'launch',
                    'advanced_param.launch.py'
                )
            ),
            launch_arguments={
                'drone_id': drone_id,
                'odometry_topic': 'visual_slam/odom',
                'cloud_topic': 'pcl_render_node/cloud',
                'map_size_x_': '50.0',
                'map_size_y_': '50.0',
                'map_size_z_': '5.0',
            }.items()
        ),
        
        # 6. Trajectory server
        Node(
            package='ego_planner',
            executable='traj_server',
            name='traj_server',
            output='screen',
            remappings=[
                ('position_cmd', '/drone_0_planning/pos_cmd'),
            ]
        ),
        
        # 7. Controller adapter (convert EGO commands to Gazebo)
        Node(
            package='ego_planner',
            executable='position_cmd_to_gazebo.py',
            name='cmd_adapter',
            output='screen',
            remappings=[
                ('position_cmd', '/drone_0_planning/pos_cmd'),
                ('cmd_vel', '/cmd_vel'),
            ]
        ),
    ])
```

### 5.2. Tạo adapter chuyển commands

Tạo file: `~/ego_ws/src/ego-planner-swarm/scripts/position_cmd_to_gazebo.py`

```python
#!/usr/bin/env python3
"""
Convert EGO-Planner position commands to Gazebo velocity commands
"""
import rclpy
from rclpy.node import Node
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import Twist
import numpy as np

class PositionCmdAdapter(Node):
    def __init__(self):
        super().__init__('position_cmd_adapter')
        
        self.sub = self.create_subscription(
            PositionCommand,
            'position_cmd',
            self.callback,
            10
        )
        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Simple PD controller gains
        self.kp = 1.0
        self.kd = 0.5
        
        self.last_pos = None
        self.last_time = None
        
        self.get_logger().info('Position command adapter started')
    
    def callback(self, msg):
        """Convert PositionCommand to Twist"""
        twist = Twist()
        
        # Extract target velocity (EGO-Planner already provides velocity)
        twist.linear.x = msg.velocity.x
        twist.linear.y = msg.velocity.y
        twist.linear.z = msg.velocity.z
        
        # Yaw rate
        twist.angular.z = msg.yaw_dot
        
        self.pub.publish(twist)

def main():
    rclpy.init()
    node = PositionCmdAdapter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
chmod +x ~/ego_ws/src/ego-planner-swarm/scripts/position_cmd_to_gazebo.py
```

### 5.3. Tạo publisher cho static map

Tạo file: `~/ego_ws/src/ego-planner-swarm/scripts/publish_static_map.py`

```python
#!/usr/bin/env python3
"""
Publish static map pointcloud from .ply file
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')
        
        self.declare_parameter('map_file', '')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('downsample_voxel_size', 0.1)
        
        map_file = self.get_parameter('map_file').value
        rate = self.get_parameter('publish_rate').value
        voxel_size = self.get_parameter('downsample_voxel_size').value
        
        # Publisher
        self.pub = self.create_publisher(
            PointCloud2,
            '/map_generator/global_cloud',
            10
        )
        
        # Load and process pointcloud
        self.get_logger().info(f'Loading map: {map_file}')
        pcd = o3d.io.read_point_cloud(map_file)
        
        # Downsample để giảm kích thước
        if voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            self.get_logger().info(f'Downsampled to {len(pcd.points)} points')
        
        self.points = np.asarray(pcd.points)
        self.get_logger().info(f'Loaded {len(self.points)} points')
        
        # Timer
        self.timer = self.create_timer(1.0/rate, self.publish)
    
    def publish(self):
        header = self.get_clock().now().to_msg()
        header.frame_id = 'world'
        
        msg = pc2.create_cloud_xyz32(header, self.points)
        self.pub.publish(msg)
        
        self.get_logger().info('Published static map', once=True)

def main():
    rclpy.init()
    node = StaticMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
chmod +x ~/ego_ws/src/ego-planner-swarm/scripts/publish_static_map.py
```

---

## 🚀 BƯỚC 6: Chạy Hệ Thống

### 6.1. Setup môi trường

```bash
cd ~/ego_ws
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ego_ws/models
```

### 6.2. Chạy từng phần (Debug mode)

**Terminal 1**: Gazebo với map
```bash
source ~/ego_ws/install/setup.bash
gz sim ~/ego_ws/src/ego-planner-swarm/worlds/custom_map.sdf
```

**Terminal 2**: Spawn drone
```bash
gz service -s /world/custom_map_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/home/quangsang/ego_ws/models/simple_quadrotor/model.sdf", pose: {position: {x: 0, y: 0, z: 1}}'
```

**Terminal 3**: Bridge topics
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/simple_quadrotor/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  --ros-args -r /model/simple_quadrotor/odometry:=/drone_0_visual_slam/odom
```

**Terminal 4**: Publish static map
```bash
python3 ~/ego_ws/src/ego-planner-swarm/scripts/publish_static_map.py \
  --ros-args -p map_file:=/home/quangsang/maps/map.ply
```

**Terminal 5**: EGO-Planner
```bash
ros2 launch ego_planner advanced_param.launch.py drone_id:=0
```

**Terminal 6**: RViz
```bash
ros2 launch ego_planner rviz.launch.py
```

### 6.3. Hoặc chạy tất cả cùng lúc

```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner gazebo_ego_planner.launch.py \
  init_x:=0.0 \
  init_y:=0.0 \
  init_z:=1.0
```

---

## 🎮 BƯỚC 7: Điều khiển Drone

### Cách 1: Qua RViz (như trước)
- Click "2D Nav Goal" và chọn điểm đích

### Cách 2: Publish goal qua terminal
```bash
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 5.0, y: 3.0, z: 1.5}}}"
```

### Cách 3: Điều khiển thủ công Gazebo (test)
```bash
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 1.0}"
```

---

## 📊 BƯỚC 8: Kiểm tra Topics

```bash
# Xem tất cả topics
ros2 topic list

# Kiểm tra odometry từ Gazebo
ros2 topic echo /drone_0_visual_slam/odom

# Kiểm tra map được publish
ros2 topic echo /map_generator/global_cloud --no-arr

# Kiểm tra lệnh từ planner
ros2 topic echo /drone_0_planning/pos_cmd
```

---

## ⚙️ BƯỚC 9: Tối ưu hóa

### 9.1. Giảm kích thước mesh

Nếu Gazebo chậm:

```python
import trimesh
mesh = trimesh.load('map.ply')
# Giảm xuống 10k faces
mesh_simplified = mesh.simplify_quadric_decimation(10000)
mesh_simplified.export('map_light.dae')
```

### 9.2. Tăng tốc rendering

Trong file `.sdf`, bỏ bớt chi tiết visual:

```xml
<visual name="visual">
  <geometry>
    <mesh>
      <uri>file://map_light.dae</uri>
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
  <cast_shadows>false</cast_shadows>  <!-- Tắt shadow -->
</visual>
```

### 9.3. Dùng Octomap

Thay vì load toàn bộ map, dùng Octomap để chỉ load vùng gần drone:

```bash
sudo apt install ros-humble-octomap-server
```

---

## 🐛 Xử lý lỗi thường gặp

### Lỗi 1: "Cannot find mesh file"
```bash
# Kiểm tra path
ls -lh /home/quangsang/maps/map.dae

# Hoặc dùng absolute path trong .sdf
file:///home/quangsang/maps/map.dae
```

### Lỗi 2: Gazebo crash khi load mesh lớn
```python
# Downsample trước
import trimesh
mesh = trimesh.load('huge_map.ply')
mesh = mesh.simplify_quadric_decimation(5000)
mesh.export('small_map.dae')
```

### Lỗi 3: Drone không bay
```bash
# Kiểm tra bridge topics
ros2 topic list | grep odom
ros2 topic list | grep cmd

# Test thủ công
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0, z: 0}}"
```

### Lỗi 4: Map không hiện trong RViz
```bash
# Kiểm tra frame_id
ros2 topic echo /map_generator/global_cloud | grep frame_id

# Phải là "world", nếu không, sửa trong script
```

---

## 📋 Checklist Hoàn thành

- [ ] Cài Gazebo Harmonic
- [ ] Convert .ply → .dae thành công
- [ ] Tạo Gazebo world file
- [ ] Test load world trong Gazebo
- [ ] Tạo drone model
- [ ] Spawn drone trong Gazebo
- [ ] Setup topic bridge
- [ ] Publish static map PointCloud2
- [ ] Chạy EGO-Planner
- [ ] Test điều khiển drone
- [ ] Verify collision avoidance

---

## 🎯 Workflow Tổng Kết

```bash
# 1. Convert map
python3 convert_ply_to_dae.py

# 2. Copy vào đúng folder
cp map.dae ~/ego_ws/maps/

# 3. Chỉnh path trong custom_map.sdf
nano ~/ego_ws/src/ego-planner-swarm/worlds/custom_map.sdf

# 4. Chạy Gazebo + EGO-Planner
ros2 launch ego_planner gazebo_ego_planner.launch.py

# 5. Set goal và bay!
```

---

## 📚 Tài liệu tham khảo

- Gazebo documentation: https://gazebosim.org/docs
- ROS-Gazebo bridge: https://github.com/gazebosim/ros_gz
- Open3D: https://www.open3d.org/docs/
- Trimesh: https://trimsh.org/

---

**Chúc bạn thành công!** 🚁✨

Nếu cần hỗ trợ thêm, hãy cho tôi biết:
- Đường dẫn file .ply của bạn
- Kích thước map (số điểm, dung lượng file)
- Loại môi trường (indoor/outdoor, phẳng/phức tạp)
