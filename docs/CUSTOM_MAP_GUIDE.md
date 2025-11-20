# 🗺️ Hướng dẫn Load Map Tùy Chỉnh và Cài Đặt Điểm Bắt Đầu/Kết Thúc

## 📋 TÓM TẮT ĐẦU VÀO CỦA HỆ THỐNG

EGO-Planner-Swarm cần **3 đầu vào chính**:

### 1. **Odometry (Vị trí drone)** 
- **Topic**: `/drone_0_visual_slam/odom` (hoặc tùy chỉnh)
- **Type**: `nav_msgs/msg/Odometry`
- **Nguồn**: Mô phỏng hoặc SLAM thực tế
- **Nội dung**: Position (x, y, z), Velocity, Orientation

### 2. **Map (Bản đồ môi trường)**
- **Topic**: `/map_generator/global_cloud`
- **Type**: `sensor_msgs/msg/PointCloud2`
- **Nguồn**: 
  - `map_generator` (random forest) - mặc định
  - `mockamap` (maze/perlin noise)
  - **File PCD/PLY của bạn** ← Đây là cách load map tùy chỉnh!

### 3. **Goal Point (Điểm đích)**
- **Topic**: `/goal` 
- **Type**: `geometry_msgs/msg/PoseStamped`
- **Nguồn**: Click chuột trong RViz (tool "2D Nav Goal")

---

## 🎯 PHẦN 1: ĐẶT ĐIỂM BẮT ĐẦU (Start Position)

Điểm bắt đầu của drone được đặt trong **launch file**.

### Cách 1: Sửa trực tiếp trong launch file

Mở file: `src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py`

Tìm dòng 168-170:
```python
'init_x_': str(-15.0),
'init_y_': str(0.0),
'init_z_': str(0.1),
```

**Thay đổi thành tọa độ bạn muốn:**
```python
'init_x_': str(5.0),    # Vị trí X bắt đầu (mét)
'init_y_': str(3.0),    # Vị trí Y bắt đầu (mét)
'init_z_': str(1.5),    # Độ cao bắt đầu (mét)
```

### Cách 2: Truyền tham số khi chạy (Linh hoạt hơn)

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  init_x_:=5.0 \
  init_y_:=3.0 \
  init_z_:=1.5
```

**Lưu ý quan trọng:**
- Tọa độ phải nằm trong **không gian trống** (không có chướng ngại vật)
- `init_z` nên >= 0.5m để tránh va chạm với mặt đất
- Hệ tọa độ: Trung tâm map ở (0, 0, ground_height)

---

## 🗺️ PHẦN 2: LOAD MAP TÙY CHỈNH (PCD/PLY File)

Bạn có **3 tùy chọn** để cung cấp map:

### ✅ TÙYCHỌN 1: Publish PointCloud từ file (Khuyến nghị)

Tạo một node đơn giản publish PointCloud từ file PCD/PLY của bạn.

#### **Bước 1: Tạo script Python publish map**

Tạo file: `/home/quangsang/ego_ws/src/ego-planner-swarm/scripts/publish_custom_map.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class CustomMapPublisher(Node):
    def __init__(self):
        super().__init__('custom_map_publisher')
        
        # Khai báo parameter cho đường dẫn file
        self.declare_parameter('map_file', '/path/to/your/map.pcd')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        map_file = self.get_parameter('map_file').value
        rate = self.get_parameter('publish_rate').value
        
        # Publisher
        self.publisher = self.create_publisher(
            PointCloud2, 
            '/map_generator/global_cloud', 
            10
        )
        
        # Load point cloud
        self.get_logger().info(f'Loading map from: {map_file}')
        try:
            pcd = o3d.io.read_point_cloud(map_file)
            self.points = np.asarray(pcd.points)
            self.get_logger().info(f'Loaded {len(self.points)} points')
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            raise
        
        # Timer để publish định kỳ
        self.timer = self.create_timer(1.0/rate, self.publish_map)
        
    def publish_map(self):
        # Convert numpy array to PointCloud2
        header = self.get_clock().now().to_msg()
        header.frame_id = 'world'
        
        cloud_msg = pc2.create_cloud_xyz32(header, self.points)
        self.publisher.publish(cloud_msg)
        self.get_logger().info('Published custom map', once=True)

def main(args=None):
    rclpy.init(args=args)
    node = CustomMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### **Bước 2: Cài đặt Open3D**

```bash
pip3 install open3d
```

#### **Bước 3: Tạo launch file riêng**

Tạo file: `/home/quangsang/ego_ws/src/ego-planner-swarm/launch/custom_map.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ego_planner',  # Hoặc tạo package riêng
            executable='publish_custom_map.py',
            name='custom_map_publisher',
            output='screen',
            parameters=[{
                'map_file': '/home/quangsang/my_maps/warehouse.pcd',  # ĐỔI PATH
                'publish_rate': 1.0
            }]
        )
    ])
```

#### **Bước 4: Chạy hệ thống**

**Terminal 1**: Publish map của bạn
```bash
cd ~/ego_ws
source install/setup.bash
ros2 run ego_planner publish_custom_map.py --ros-args \
  -p map_file:=/home/quangsang/my_maps/warehouse.pcd
```

**Terminal 2**: RViz
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner rviz.launch.py
```

**Terminal 3**: Planner (TẮT map generator mặc định)
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  use_mockamap:=False \
  init_x_:=0.0 \
  init_y_:=0.0 \
  init_z_:=1.0
```

Sau đó **comment/disable node `map_generator_node`** trong launch file!

---

### ✅ TÙY CHỌN 2: Sửa map_generator để load từ file

Đơn giản hơn nhưng cần sửa code C++.

#### **File cần sửa**: 
`src/ego-planner-swarm/src/uav_simulator/map_generator/src/random_forest.cpp`

Thêm code sau vào hàm `main()`:

```cpp
// Thêm vào đầu file
#include <pcl/io/pcd_io.h>

// Trong main(), sau khi init node:
std::string map_file;
nh.param("map_file", map_file, std::string(""));

if (!map_file.empty()) {
    ROS_INFO("Loading custom map from: %s", map_file.c_str());
    
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(map_file, cloud) == -1) {
        ROS_ERROR("Failed to load PCD file!");
        return -1;
    }
    
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    
    // Publish cloud
    // ... (code publish)
}
```

Sau đó rebuild:
```bash
colcon build --packages-select map_generator
```

---

### ✅ TÙY CHỌN 3: Dùng ROS bag (Nếu có sẵn)

Nếu bạn đã có ROS bag chứa PointCloud:

```bash
ros2 bag play your_map.bag --topics /map_generator/global_cloud
```

---

## 🎯 PHẦN 3: ĐẶT ĐIỂM KẾT THÚC (Goal Point)

Có **2 cách** đặt điểm đích:

### Cách 1: Click trong RViz (Interactive - Khuyến nghị)

1. Mở RViz
2. Chọn tool **"2D Nav Goal"** trên thanh công cụ
3. Click và kéo trên bản đồ để đặt điểm đích
4. Drone sẽ tự động bay đến đó!

**Lưu ý**: Để thay đổi độ cao (Z), bạn cần dùng tool "Publish Point" hoặc publish trực tiếp qua terminal.

### Cách 2: Publish trực tiếp qua terminal

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
"{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'world'
  },
  pose: {
    position: {x: 10.0, y: 5.0, z: 2.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### Cách 3: Preset waypoints (Auto tour)

Sửa trong launch file `advanced_param.launch.py`:

```python
# Số waypoints
'point_num': str(3),

# Waypoint 0
'point0_x': str(5.0),
'point0_y': str(0.0),
'point0_z': str(1.0),

# Waypoint 1
'point1_x': str(10.0),
'point1_y': str(5.0),
'point1_z': str(1.5),

# Waypoint 2
'point2_x': str(-5.0),
'point2_y': str(-3.0),
'point2_z': str(1.0),
```

Sau đó chạy với `flight_type:=1` (preset waypoints).

---

## 📐 PHẦN 4: ĐIỀU CHỈNH KHU VỰC BẢN ĐỒ

Đảm bảo kích thước bản đồ khớp với map của bạn:

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  map_size_x:=50.0 \    # Kích thước X (mét)
  map_size_y:=50.0 \    # Kích thước Y (mét)
  map_size_z:=5.0 \     # Độ cao (mét)
  init_x_:=0.0 \
  init_y_:=0.0 \
  init_z_:=1.0
```

**Lưu ý**: 
- Map origin mặc định ở `(-map_size_x/2, -map_size_y/2, ground_height)`
- Nếu map của bạn có origin khác, cần transform pointcloud trước

---

## 🔧 PHẦN 5: KIỂM TRA VÀ DEBUG

### Kiểm tra topic đang hoạt động:

```bash
# Xem danh sách topics
ros2 topic list

# Kiểm tra map đang được publish
ros2 topic echo /map_generator/global_cloud --no-arr

# Kiểm tra odometry
ros2 topic echo /drone_0_visual_slam/odom

# Kiểm tra goal
ros2 topic echo /goal
```

### Visualize trong RViz:

1. Add display **"PointCloud2"**
2. Topic: `/map_generator/global_cloud`
3. Fixed Frame: `world`
4. Add **"Odometry"**: `/drone_0_visual_slam/odom`
5. Add **"Path"**: `/drone_0_planning/path` (nếu có)

---

## 📝 VÍ DỤ HOÀN CHỈNH: Load map PCD + Set start/goal

### Bước 1: Chuẩn bị map
```bash
# Giả sử bạn có file: /home/quangsang/maps/warehouse.pcd
ls -lh /home/quangsang/maps/warehouse.pcd
```

### Bước 2: Tạo script publisher (dùng code Python ở trên)
```bash
nano ~/ego_ws/src/ego-planner-swarm/scripts/publish_custom_map.py
chmod +x ~/ego_ws/src/ego-planner-swarm/scripts/publish_custom_map.py
```

### Bước 3: Sửa single_run_in_sim.launch.py

Comment dòng `map_generator_node`:
```python
# ld.add_action(map_generator_node)  # COMMENT dòng này
# ld.add_action(mockamap_node)       # COMMENT dòng này
```

### Bước 4: Chạy hệ thống

**Terminal 1**: Publish map
```bash
source ~/ego_ws/install/setup.bash
python3 ~/ego_ws/src/ego-planner-swarm/scripts/publish_custom_map.py \
  --ros-args -p map_file:=/home/quangsang/maps/warehouse.pcd
```

**Terminal 2**: RViz
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner rviz.launch.py
```

**Terminal 3**: Planner
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  init_x_:=2.0 \
  init_y_:=3.0 \
  init_z_:=1.0 \
  map_size_x:=40.0 \
  map_size_y:=40.0 \
  map_size_z:=5.0
```

**Terminal 4**: Set goal (hoặc dùng RViz)
```bash
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 15.0, y: 10.0, z: 1.5}}}"
```

---

## ⚠️ LƯU Ý QUAN TRỌNG

### 1. **Định dạng PointCloud**
- Hệ thống chỉ cần **XYZ coordinates**
- Không cần RGB/Normal/Intensity
- Đơn vị: **mét**

### 2. **Hệ tọa độ**
- Frame ID phải là **"world"**
- Z-axis hướng lên (up)
- Origin của map: `(-map_size_x/2, -map_size_y/2, ground_height)`

### 3. **Độ phân giải (Resolution)**
- Map grid resolution mặc định: **0.1m** (10cm)
- Nếu map của bạn quá dày đặc → downsample trước
- Nếu quá thưa → có thể bị lỗ hổng

```python
# Downsample với Open3D
import open3d as o3d
pcd = o3d.io.read_point_cloud("dense_map.pcd")
pcd_down = pcd.voxel_down_sample(voxel_size=0.1)  # 10cm
o3d.io.write_point_cloud("sparse_map.pcd", pcd_down)
```

### 4. **Collision checking**
- Planner sẽ inflate obstacles thêm **obstacles_inflation** (mặc định ~0.2m)
- Đảm bảo map không có "vách quá mỏng" < 0.5m

### 5. **Performance**
- Map quá lớn (>100k points) → chậm
- Nên giới hạn trong vùng cần bay
- Dùng local map update để tối ưu

---

## 🚀 CÁCH NHANH NHẤT: Tất cả trong 1 lệnh

Tạo file `my_custom_mission.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Publish custom map
        Node(
            package='ego_planner',
            executable='publish_custom_map.py',
            parameters=[{'map_file': '/home/quangsang/maps/warehouse.pcd'}]
        ),
        
        # 2. RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 
                           'launch', 'rviz.launch.py')
            )
        ),
        
        # 3. Planner với custom start point
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 
                           'launch', 'single_run_in_sim.launch.py')
            ),
            launch_arguments={
                'init_x_': '2.0',
                'init_y_': '3.0',
                'init_z_': '1.0',
                'map_size_x': '40.0',
                'map_size_y': '40.0',
                'map_size_z': '5.0'
            }.items()
        )
    ])
```

Chạy:
```bash
ros2 launch ego_planner my_custom_mission.launch.py
```

---

## 📚 TÀI LIỆU THAM KHẢO

- Topic `/map_generator/global_cloud`: Là nơi planner subscribe map
- Topic `/goal`: Nơi planner subscribe target
- Topic `/drone_0_visual_slam/odom`: Nơi planner subscribe vị trí drone
- File config: `src/planner/plan_env/src/grid_map.cpp` (line 168)

**Chúc bạn thành công!** 🎉
