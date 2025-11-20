# Hướng dẫn tích hợp X500 Gazebo và OpenVINS vào EGO-Swarm

## Tổng quan

Tài liệu này hướng dẫn cách thay thế:
1. **Simulator hiện tại** (so3_quadrotor_simulator) → **Gazebo với X500**
2. **Odometry giả lập** → **OpenVINS (VIO thực tế)**

---

## Phần 1: Phân tích Kiến trúc Hiện tại

### 1.1 Topics mà EGO-Swarm Subscribe
Từ source code, các module chính subscribe các topics sau:

#### Planning Module (`ego_replan_fsm.cpp`)
```cpp
// Topic CHÍNH cần có
"odom_world"  → nav_msgs/Odometry  // Vị trí, vận tốc, góc quay drone
```

#### Local Sensing Module (`pcl_render_node.cpp`, `pointcloud_render_node.cpp`)
```cpp
"odometry"    → nav_msgs/Odometry  // Vị trí drone để render depth
"global_map"  → sensor_msgs/PointCloud2  // Bản đồ môi trường
```

#### Drone Detector Module (`drone_detector.cpp`)
```cpp
"odometry"    → nav_msgs/Odometry  // Vị trí drone của mình
"/others_odom" → nav_msgs/Odometry  // Vị trí các drone khác (swarm)
"depth"       → sensor_msgs/Image   // Depth image để xóa drone khác
```

### 1.2 Topics mà EGO-Swarm Publish

#### Control Commands
```cpp
"planning/pos_cmd"  → quadrotor_msgs/PositionCommand  // Lệnh điều khiển
"so3_cmd"          → quadrotor_msgs/SO3Command       // (nếu dùng dynamics)
```

---

## Phần 2: Chuẩn bị X500 Gazebo

### 2.1 Kiểm tra X500 Topics

Chạy X500 trong Gazebo và kiểm tra topics:
```bash
# Terminal 1: Khởi động Gazebo với X500
cd ~/px4_ws  # hoặc workspace chứa X500
ros2 launch px4_sitl_gazebo x500.launch.py  # Tên launch file của bạn

# Terminal 2: Kiểm tra topics
ros2 topic list
ros2 topic info /fmu/out/vehicle_odometry  # Hoặc topic tương tự
ros2 topic echo /fmu/out/vehicle_odometry  # Xem format
```

### 2.2 Xác định Command Topic của X500

```bash
# Tìm topic nhận lệnh điều khiển
ros2 topic list | grep -i cmd
ros2 topic info /fmu/in/offboard_control_mode  # Ví dụ
```

### 2.3 Kiểm tra Camera/Depth trên X500

```bash
# Kiểm tra X500 có camera không
ros2 topic list | grep -i camera
ros2 topic list | grep -i depth
ros2 topic list | grep -i image
```

**LƯU Ý**: Nếu X500 chưa có depth camera trong Gazebo, bạn cần:
- Thêm depth camera plugin vào file URDF/SDF của X500
- Hoặc sử dụng local_sensing từ EGO-Swarm để render depth từ map

---

## Phần 3: Chuẩn bị OpenVINS

### 3.1 Kiểm tra OpenVINS Output

```bash
# Terminal 1: Chạy OpenVINS
ros2 launch ov_msckf <your_openvins_launch_file>

# Terminal 2: Kiểm tra output
ros2 topic list | grep -i odom
ros2 topic echo /ov_msckf/odometry_imu  # Hoặc topic tương tự
```

### 3.2 Xác định Format của OpenVINS Odometry

OpenVINS thường publish:
```
Topic: /ov_msckf/odometry_imu
Type: nav_msgs/Odometry
```

Kiểm tra frame_id:
```bash
ros2 topic echo /ov_msckf/odometry_imu --once | grep frame_id
```

---

## Phần 4: Tạo Bridge Nodes

### 4.1 Tạo X500 Command Bridge

File: `src/ego-planner-swarm/src/uav_simulator/Utils/x500_bridge/src/x500_cmd_bridge.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

class X500CmdBridge : public rclcpp::Node
{
public:
    X500CmdBridge() : Node("x500_cmd_bridge")
    {
        // Subscribe EGO-Swarm command
        cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "planning/pos_cmd", 10,
            std::bind(&X500CmdBridge::cmdCallback, this, std::placeholders::_1));
        
        // Publish to PX4
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        
        traj_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        
        // Timer to publish offboard mode
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&X500CmdBridge::publishOffboardMode, this));
    }

private:
    void cmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
    {
        // Convert EGO-Swarm PositionCommand to PX4 TrajectorySetpoint
        px4_msgs::msg::TrajectorySetpoint traj_msg;
        
        // Position (NED in PX4, ENU in ROS2)
        traj_msg.position[0] = msg->position.x;   // North (X)
        traj_msg.position[1] = msg->position.y;   // East (Y)
        traj_msg.position[2] = -msg->position.z;  // Down (-Z, đảo dấu)
        
        // Velocity
        traj_msg.velocity[0] = msg->velocity.x;
        traj_msg.velocity[1] = msg->velocity.y;
        traj_msg.velocity[2] = -msg->velocity.z;
        
        // Acceleration
        traj_msg.acceleration[0] = msg->acceleration.x;
        traj_msg.acceleration[1] = msg->acceleration.y;
        traj_msg.acceleration[2] = -msg->acceleration.z;
        
        traj_msg.yaw = msg->yaw;
        traj_msg.timestamp = this->now().nanoseconds() / 1000;
        
        traj_setpoint_pub_->publish(traj_msg);
    }
    
    void publishOffboardMode()
    {
        px4_msgs::msg::OffboardControlMode msg;
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->now().nanoseconds() / 1000;
        
        offboard_mode_pub_->publish(msg);
    }

    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<X500CmdBridge>());
    rclcpp::shutdown();
    return 0;
}
```

### 4.2 Tạo Odometry Converter

File: `src/ego-planner-swarm/src/uav_simulator/Utils/x500_bridge/src/odom_converter.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class OdomConverter : public rclcpp::Node
{
public:
    OdomConverter() : Node("odom_converter")
    {
        // Subscribe từ OpenVINS hoặc PX4
        this->declare_parameter("input_topic", "/ov_msckf/odometry_imu");
        this->declare_parameter("output_topic", "odom_world");
        this->declare_parameter("use_px4_odom", false);
        
        std::string input_topic, output_topic;
        bool use_px4;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("output_topic", output_topic);
        this->get_parameter("use_px4_odom", use_px4);
        
        if (use_px4)
        {
            // Subscribe PX4 odometry
            px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                input_topic, 10,
                std::bind(&OdomConverter::px4OdomCallback, this, std::placeholders::_1));
        }
        else
        {
            // Subscribe OpenVINS odometry
            vins_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                input_topic, 10,
                std::bind(&OdomConverter::vinsOdomCallback, this, std::placeholders::_1));
        }
        
        // Publish to EGO-Swarm format
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
    }

private:
    void vinsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // OpenVINS đã là nav_msgs/Odometry, chỉ cần forward
        // Có thể cần đổi frame_id
        nav_msgs::msg::Odometry out_msg = *msg;
        out_msg.header.frame_id = "world";
        out_msg.child_frame_id = "body";
        
        odom_pub_->publish(out_msg);
    }
    
    void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Convert PX4 VehicleOdometry to nav_msgs/Odometry
        nav_msgs::msg::Odometry out_msg;
        
        out_msg.header.stamp = this->now();
        out_msg.header.frame_id = "world";
        out_msg.child_frame_id = "body";
        
        // Position (NED to ENU)
        out_msg.pose.pose.position.x = msg->position[0];
        out_msg.pose.pose.position.y = msg->position[1];
        out_msg.pose.pose.position.z = -msg->position[2];  // Down to Up
        
        // Quaternion (NED to ENU) - cần rotate
        // Simplified: giả sử quaternion đã đúng frame
        out_msg.pose.pose.orientation.w = msg->q[0];
        out_msg.pose.pose.orientation.x = msg->q[1];
        out_msg.pose.pose.orientation.y = msg->q[2];
        out_msg.pose.pose.orientation.z = msg->q[3];
        
        // Velocity (NED to ENU)
        out_msg.twist.twist.linear.x = msg->velocity[0];
        out_msg.twist.twist.linear.y = msg->velocity[1];
        out_msg.twist.twist.linear.z = -msg->velocity[2];
        
        odom_pub_->publish(out_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vins_odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomConverter>());
    rclcpp::shutdown();
    return 0;
}
```

---

## Phần 5: Tạo Launch File Tích hợp

File: `src/ego-planner-swarm/src/planner/plan_manage/launch/gazebo_x500_openvins.launch.py`

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Parameters
    drone_id = LaunchConfiguration('drone_id', default='0')
    map_size_x = LaunchConfiguration('map_size_x', default='50.0')
    map_size_y = LaunchConfiguration('map_size_y', default='25.0')
    map_size_z = LaunchConfiguration('map_size_z', default='5.0')
    
    # OpenVINS config
    openvins_topic = LaunchConfiguration('openvins_topic', 
                                         default='/ov_msckf/odometry_imu')
    use_openvins = LaunchConfiguration('use_openvins', default='true')
    
    # X500 Gazebo config
    x500_namespace = LaunchConfiguration('x500_namespace', default='x500')
    
    # 1. Launch Gazebo với X500
    gazebo_x500 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('px4_sitl_gazebo'),
                'launch',
                'x500.launch.py'  # Thay bằng launch file X500 của bạn
            ])
        ]),
        launch_arguments={
            'namespace': x500_namespace,
        }.items()
    )
    
    # 2. Launch OpenVINS
    openvins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ov_msckf'),
                'launch',
                'subscribe.launch.py'  # Thay bằng launch file OpenVINS của bạn
            ])
        ])
    )
    
    # 3. Odometry Converter (OpenVINS -> EGO-Swarm)
    odom_converter = Node(
        package='x500_bridge',  # Package mới tạo
        executable='odom_converter',
        name='odom_converter',
        output='screen',
        parameters=[{
            'input_topic': openvins_topic,
            'output_topic': 'odom_world',
            'use_px4_odom': False  # True nếu dùng PX4 odom thay vì OpenVINS
        }]
    )
    
    # 4. Command Bridge (EGO-Swarm -> X500)
    cmd_bridge = Node(
        package='x500_bridge',
        executable='x500_cmd_bridge',
        name='x500_cmd_bridge',
        output='screen',
        remappings=[
            ('planning/pos_cmd', ['drone_', drone_id, '_planning/pos_cmd'])
        ]
    )
    
    # 5. Map Generator (giữ nguyên từ EGO-Swarm)
    map_generator = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[{
            'seed': 127,
            'update_freq': 0.5,
            'resolution': 0.1,
            'x_length': map_size_x,
            'y_length': map_size_y,
            'z_length': map_size_z,
            'type': 1,
            'complexity': 0.05,
            'fill': 0.12,
            'fractal': 1,
            'attenuation': 0.1
        }]
    )
    
    # 6. Include EGO-Planner (không dùng simulator nữa)
    ego_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ego_planner'),
                'launch',
                'advanced_param.launch.py'
            ])
        ]),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': 'odom_world',  # Từ odom_converter
            'camera_pose_topic': '/x500/camera_pose',  # Từ X500
            'depth_topic': '/x500/depth',  # Từ X500 depth camera
            'cloud_topic': 'pcl_render_node/cloud',
        }.items()
    )
    
    # 7. Depth Render (nếu X500 không có depth camera)
    # Uncomment nếu cần render depth từ map
    # depth_render = Node(...)
    
    ld = LaunchDescription()
    
    # Add all nodes
    ld.add_action(gazebo_x500)
    ld.add_action(openvins)
    ld.add_action(odom_converter)
    ld.add_action(cmd_bridge)
    ld.add_action(map_generator)
    ld.add_action(ego_planner)
    
    return ld
```

---

## Phần 6: CMakeLists và Package.xml

### 6.1 Tạo Package cho Bridge Nodes

```bash
cd ~/ego_ws/src/ego-planner-swarm/src/uav_simulator/Utils
ros2 pkg create --build-type ament_cmake x500_bridge \
  --dependencies rclcpp nav_msgs quadrotor_msgs px4_msgs geometry_msgs
```

### 6.2 CMakeLists.txt

File: `src/uav_simulator/Utils/x500_bridge/CMakeLists.txt`

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

## Phần 7: Quy trình Build và Test

### 7.1 Build

```bash
cd ~/ego_ws
colcon build --packages-select x500_bridge
source install/setup.bash
```

### 7.2 Test Riêng từng Component

#### Test 1: X500 Gazebo
```bash
ros2 launch px4_sitl_gazebo x500.launch.py
```

#### Test 2: OpenVINS
```bash
# Terminal 1: Gazebo + X500
ros2 launch px4_sitl_gazebo x500.launch.py

# Terminal 2: OpenVINS
ros2 launch ov_msckf subscribe.launch.py

# Terminal 3: Check output
ros2 topic hz /ov_msckf/odometry_imu
```

#### Test 3: Odom Converter
```bash
# Terminal 1-2: như trên

# Terminal 3: Converter
ros2 run x500_bridge odom_converter \
  --ros-args \
  -p input_topic:=/ov_msckf/odometry_imu \
  -p output_topic:=odom_world

# Terminal 4: Verify
ros2 topic echo odom_world
```

#### Test 4: Full Integration
```bash
ros2 launch ego_planner gazebo_x500_openvins.launch.py
```

---

## Phần 8: Xử lý Vấn đề Thường gặp

### 8.1 Depth Camera trên X500

**Vấn đề**: X500 không có depth camera mặc định

**Giải pháp 1**: Thêm depth camera plugin vào X500 SDF

File: `x500_depth.sdf` (thêm vào model X500)

```xml
<sensor name="camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>x500</namespace>
      <remapping>depth/image_raw:=depth</remapping>
      <remapping>depth/camera_info:=depth/camera_info</remapping>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

**Giải pháp 2**: Sử dụng pcl_render_node của EGO-Swarm để render depth từ map

### 8.2 Frame Transformation (NED vs ENU)

PX4 dùng **NED** (North-East-Down), ROS2 dùng **ENU** (East-North-Up)

Cần convert:
- X(ENU) = X(NED)
- Y(ENU) = Y(NED)  
- Z(ENU) = -Z(NED)

### 8.3 Timestamp Synchronization

OpenVINS và PX4 có thể có clock khác nhau.

Giải pháp:
```cpp
// Trong odom_converter
out_msg.header.stamp = this->now();  // Dùng ROS2 clock hiện tại
```

---

## Phần 9: Checklist Tích hợp

- [ ] X500 chạy được trong Gazebo
- [ ] OpenVINS publish odometry chính xác
- [ ] Odom converter chuyển đổi đúng format
- [ ] EGO-Swarm nhận được odom_world topic
- [ ] Command bridge gửi lệnh đến X500
- [ ] X500 di chuyển theo trajectory
- [ ] Depth camera hoặc render hoạt động
- [ ] Planning tránh chướng ngại vật
- [ ] Test với single drone thành công
- [ ] Test với swarm (nhiều X500)

---

## Phần 10: Tài liệu Tham khảo

1. PX4 ROS2 Interface: https://docs.px4.io/main/en/ros/ros2_comm.html
2. OpenVINS Documentation: https://docs.openvins.com/
3. EGO-Swarm Paper: https://arxiv.org/abs/2011.04800

---

## Hỗ trợ

Nếu gặp vấn đề, kiểm tra:
1. `ros2 topic list` - xem tất cả topics
2. `ros2 topic hz <topic>` - kiểm tra frequency
3. `ros2 topic echo <topic>` - xem nội dung
4. `ros2 node info <node>` - kiểm tra node connections
