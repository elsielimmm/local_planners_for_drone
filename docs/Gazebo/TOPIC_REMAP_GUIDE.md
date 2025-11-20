# 🔗 Topic Remapping Guide - X500 + OpenVINS + EGO-Swarm

## 📋 Tổng quan

Bạn đã **sửa typo thành công** trong `single_run_in_sim.launch.py`:
- ✅ `ov_mscfkf/odomimu` → `ov_msckf/odomimu`

## 🗺️ Topic Mapping Table

| Mục đích | Topic EGO-Swarm cần | Topic bạn có | Cần bridge? |
|----------|---------------------|--------------|-------------|
| **Odometry** | `drone_0_visual_slam/odom` hoặc `odom_world` | `/ov_msckf/odomimu` | ✅ CẦN |
| **Depth Image** | `drone_0_pcl_render_node/depth` | `/gz/drone1/camera_depth/image_raw` | ⚠️ Remap trong launch |
| **Point Cloud** | `drone_0_pcl_render_node/cloud` | `/gz/drone1/camera_depth/points` | ⚠️ Remap trong launch |
| **Camera Pose** | `drone_0_pcl_render_node/camera_pose` | *(không có)* | ✅ CẦN tạo |
| **Control Output** | `/drone_0_planning/pos_cmd` | `/fmu/in/trajectory_setpoint` | ✅ CẦN |

---

## ✅ ĐÃ SỬA trong `single_run_in_sim.launch.py`

### 1. Fix Typo Odometry Topic (Dòng 18)

```python
# BEFORE:
odom_topic = LaunchConfiguration('odom_topic', default = 'ov_mscfkf/odomimu')  # ❌ SAI

# AFTER:
odom_topic = LaunchConfiguration('odom_topic', default = 'ov_msckf/odomimu')  # ✅ ĐÚNG
```

### 2. Update Camera Topics (Dòng 105-107)

```python
# BEFORE: Topics cho simulator cũ
'camera_pose_topic': 'pcl_render_node/camera_pose',
'depth_topic': 'pcl_render_node/depth',
'cloud_topic': 'pcl_render_node/cloud',

# AFTER: Topics cho X500 Gazebo
'camera_pose_topic': 'drone_0_camera/pose',           # Sẽ publish bởi bridge node
'depth_topic': 'gz/drone1/camera_depth/image_raw',    # Từ Gazebo
'cloud_topic': 'gz/drone1/camera_depth/points',       # Từ Gazebo
```

---

## 🔧 CẦN TẠO: Bridge Nodes

### Architecture Diagram

```
┌─────────────┐         ┌──────────────────┐         ┌────────────────┐
│ OpenVINS    │         │  odom_converter  │         │  EGO-Swarm     │
│             │────────▶│                  │────────▶│  Planning      │
│/ov_msckf/   │ odomimu │  Frame convert   │ odom_   │                │
│  odomimu    │         │  Message type    │  world  │                │
└─────────────┘         └──────────────────┘         └────────────────┘

┌─────────────┐         ┌──────────────────┐         ┌────────────────┐
│ EGO-Swarm   │         │ x500_cmd_bridge  │         │   PX4 SITL     │
│ Planning    │────────▶│                  │────────▶│                │
│/drone_0_    │ pos_cmd │  PositionCommand │ Traj    │  /fmu/in/      │
│planning/    │         │  → Trajectory    │ Setpoint│  trajectory    │
│pos_cmd      │         │  ENU → NED       │         │  _setpoint     │
└─────────────┘         └──────────────────┘         └────────────────┘

┌─────────────┐         ┌──────────────────┐         ┌────────────────┐
│ Gazebo X500 │         │ camera_pose_pub  │         │  EGO-Swarm     │
│             │         │                  │────────▶│  Mapping       │
│TF: drone1/  │────────▶│  TF Listener     │ camera_ │                │
│camera_link  │   TF    │  → PoseStamped   │  pose   │                │
└─────────────┘         └──────────────────┘         └────────────────┘
```

---

## 🛠️ Bridge Node 1: `odom_converter`

### Chức năng:
- Subscribe: `/ov_msckf/odomimu` (nav_msgs/Odometry từ OpenVINS)
- Publish: `/drone_0_visual_slam/odom` (nav_msgs/Odometry cho EGO-Swarm)
- Convert: Có thể cần đổi frame_id, coordinate frame nếu cần

### Code Implementation:

```cpp
// File: src/ego-planner-swarm/src/uav_simulator/Utils/odom_bridge/src/odom_converter.cpp

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomConverter : public rclcpp::Node
{
public:
    OdomConverter() : Node("odom_converter")
    {
        // Parameters
        this->declare_parameter("input_topic", "ov_msckf/odomimu");
        this->declare_parameter("output_topic", "drone_0_visual_slam/odom");
        this->declare_parameter("output_frame_id", "world");
        
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        output_frame_id_ = this->get_parameter("output_frame_id").as_string();
        
        // Subscribe to OpenVINS odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic, 10,
            std::bind(&OdomConverter::odomCallback, this, std::placeholders::_1));
        
        // Publish to EGO-Swarm
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), 
            "Odometry Converter started: %s -> %s", 
            input_topic.c_str(), output_topic.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Simple pass-through with frame_id change
        nav_msgs::msg::Odometry output_odom = *msg;
        
        // Change frame_id to match EGO-Swarm expectation
        output_odom.header.frame_id = output_frame_id_;
        output_odom.child_frame_id = "drone_0";
        
        // Publish
        odom_pub_->publish(output_odom);
        
        // Debug every 1 second
        static auto last_log = this->now();
        if ((this->now() - last_log).seconds() > 1.0) {
            RCLCPP_INFO(this->get_logger(), 
                "Odom: pos=[%.2f, %.2f, %.2f]",
                output_odom.pose.pose.position.x,
                output_odom.pose.pose.position.y,
                output_odom.pose.pose.position.z);
            last_log = this->now();
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::string output_frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomConverter>());
    rclcpp::shutdown();
    return 0;
}
```

### CMakeLists.txt:

```cmake
# Add to: src/ego-planner-swarm/src/uav_simulator/Utils/odom_bridge/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(odom_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(odom_converter src/odom_converter.cpp)
ament_target_dependencies(odom_converter rclcpp nav_msgs)

install(TARGETS odom_converter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### package.xml:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>odom_bridge</name>
  <version>1.0.0</version>
  <description>Odometry topic converter for OpenVINS to EGO-Swarm</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>nav_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 🛠️ Bridge Node 2: `camera_pose_publisher`

### Chức năng:
- Subscribe: TF từ Gazebo (`world` -> `drone1/camera_link`)
- Publish: `/drone_0_camera/pose` (geometry_msgs/PoseStamped)

### Code Implementation:

```cpp
// File: src/ego-planner-swarm/src/uav_simulator/Utils/camera_bridge/src/camera_pose_publisher.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CameraPosePublisher : public rclcpp::Node
{
public:
    CameraPosePublisher() : Node("camera_pose_publisher")
    {
        // Parameters
        this->declare_parameter("world_frame", "world");
        this->declare_parameter("camera_frame", "drone1/camera_link");
        this->declare_parameter("output_topic", "drone_0_camera/pose");
        this->declare_parameter("publish_rate", 30.0);  // Hz
        
        world_frame_ = this->get_parameter("world_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        double rate = this->get_parameter("publish_rate").as_double();
        
        // TF listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic, 10);
        
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&CameraPosePublisher::publishPose, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "Camera Pose Publisher started: %s -> %s at %.1f Hz", 
            camera_frame_.c_str(), output_topic.c_str(), rate);
    }

private:
    void publishPose()
    {
        try {
            // Lookup transform
            auto transform = tf_buffer_->lookupTransform(
                world_frame_, camera_frame_, tf2::TimePointZero);
            
            // Convert to PoseStamped
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = world_frame_;
            
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            
            pose.pose.orientation = transform.transform.rotation;
            
            // Publish
            pose_pub_->publish(pose);
            
        } catch (tf2::TransformException& ex) {
            // Suppress errors until TF is available
            static auto last_warn = this->now();
            if ((this->now() - last_warn).seconds() > 5.0) {
                RCLCPP_WARN(this->get_logger(), 
                    "Could not get transform %s -> %s: %s",
                    world_frame_.c_str(), camera_frame_.c_str(), ex.what());
                last_warn = this->now();
            }
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string world_frame_;
    std::string camera_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### CMakeLists.txt:

```cmake
# src/ego-planner-swarm/src/uav_simulator/Utils/camera_bridge/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(camera_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(camera_pose_publisher src/camera_pose_publisher.cpp)
ament_target_dependencies(camera_pose_publisher 
  rclcpp geometry_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS camera_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

---

## 🛠️ Bridge Node 3: `x500_cmd_bridge`

### (Đã có code đầy đủ trong `GAZEBO_X500_OPENVINS_INTEGRATION.md`)

Subscribe: `/drone_0_planning/pos_cmd`  
Publish: `/fmu/in/trajectory_setpoint`, `/fmu/in/offboard_control_mode`

---

## 📝 Cách Build và Chạy

### 1. Tạo các package bridge

```bash
cd ~/ego_ws/src/ego-planner-swarm/src/uav_simulator/Utils

# Tạo odom_bridge package
ros2 pkg create --build-type ament_cmake odom_bridge \
  --dependencies rclcpp nav_msgs

# Tạo camera_bridge package  
ros2 pkg create --build-type ament_cmake camera_bridge \
  --dependencies rclcpp geometry_msgs tf2_ros tf2_geometry_msgs

# Copy code vào từ document này
```

### 2. Build

```bash
cd ~/ego_ws
colcon build --packages-select odom_bridge camera_bridge
source install/setup.bash
```

### 3. Chạy toàn bộ hệ thống

#### Terminal 1: Launch X500 Gazebo + OpenVINS
```bash
# Your existing X500 + OpenVINS launch
ros2 launch your_package x500_openvins.launch.py
```

#### Terminal 2: Launch Bridge Nodes
```bash
# Launch các bridge nodes
ros2 run odom_bridge odom_converter \
  --ros-args \
  -p input_topic:=/ov_msckf/odomimu \
  -p output_topic:=/drone_0_visual_slam/odom

ros2 run camera_bridge camera_pose_publisher \
  --ros-args \
  -p world_frame:=world \
  -p camera_frame:=drone1/camera_link \
  -p output_topic:=/drone_0_camera/pose

ros2 run x500_bridge x500_cmd_bridge
```

#### Terminal 3: Launch EGO-Swarm Planning
```bash
cd ~/ego_ws
source install/setup.bash

ros2 launch ego_planner single_run_in_sim.launch.py \
  odom_topic:=visual_slam/odom \
  drone_id:=0 \
  use_mockamap:=true
```

---

## ✅ Verification Checklist

### 1. Check Odometry Topic
```bash
# Check OpenVINS publishing
ros2 topic hz /ov_msckf/odomimu
# Expected: ~30-60 Hz

# Check bridge converting
ros2 topic hz /drone_0_visual_slam/odom
# Expected: same rate as input

# Check message content
ros2 topic echo /drone_0_visual_slam/odom --once
```

### 2. Check Camera Topics
```bash
# Check depth from Gazebo
ros2 topic hz /gz/drone1/camera_depth/image_raw
# Expected: ~10-30 Hz

# Check camera pose bridge
ros2 topic hz /drone_0_camera/pose
# Expected: ~30 Hz

# Check point cloud
ros2 topic hz /gz/drone1/camera_depth/points
```

### 3. Check Planning Output
```bash
# Check if planner is running
ros2 topic hz /drone_0_planning/pos_cmd
# Expected: ~30 Hz when trajectory executing

# Check bspline trajectory
ros2 topic hz /drone_0_planning/bspline
```

### 4. Check PX4 Receiving Commands
```bash
# Check x500_cmd_bridge converting
ros2 topic hz /fmu/in/trajectory_setpoint
# Expected: ~30 Hz when planning active

# Check PX4 status
ros2 topic echo /fmu/out/vehicle_status --once
# nav_state should be 14 (OFFBOARD)
# arming_state should be 2 (ARMED)
```

---

## 🐛 Common Issues & Solutions

### Issue 1: "No odometry received"

**Symptom:**
```
[ego_planner_node] Warning: No odom received!
```

**Debug:**
```bash
# Check if OpenVINS is publishing
ros2 topic list | grep ov_msckf

# Check if bridge is running
ros2 node list | grep odom_converter

# Check topic remapping
ros2 node info /drone_0_ego_planner_node | grep odom
```

**Fix:**
- Verify `odom_converter` bridge is running
- Check topic names match exactly
- Verify OpenVINS is initialized and tracking

### Issue 2: "No camera pose available"

**Symptom:**
```
[grid_map] Cannot get camera pose
```

**Debug:**
```bash
# Check TF tree
ros2 run tf2_tools view_frames
# Should show: world -> drone1 -> drone1/camera_link

# Check camera_pose_publisher
ros2 topic echo /drone_0_camera/pose --once
```

**Fix:**
- Ensure Gazebo is publishing TF
- Verify `camera_pose_publisher` is running
- Check frame names in TF tree match config

### Issue 3: "Depth image not received"

**Debug:**
```bash
# Check Gazebo camera plugin
ros2 topic list | grep camera_depth

# Check image message type
ros2 topic info /gz/drone1/camera_depth/image_raw
```

**Fix:**
- Verify Gazebo camera plugin is loaded
- Check camera topics are being remapped correctly in launch file
- Adjust camera frame rate if too slow

### Issue 4: "X500 not responding to commands"

**Debug:**
```bash
# Check bridge publishing
ros2 topic hz /fmu/in/trajectory_setpoint

# Check PX4 mode
ros2 topic echo /fmu/out/vehicle_status --field nav_state
# Should be 14 (OFFBOARD)

# Check armed
ros2 topic echo /fmu/out/vehicle_status --field arming_state
# Should be 2 (ARMED)
```

**Fix:**
- Ensure `x500_cmd_bridge` is running
- Manually arm and set offboard mode
- Check PX4 parameters allow offboard

---

## 📊 Final Topic Flow Summary

```
OpenVINS
  /ov_msckf/odomimu
      ↓
  [odom_converter]
      ↓
  /drone_0_visual_slam/odom
      ↓
  EGO-Swarm Planner
      ↓
  /drone_0_planning/pos_cmd
      ↓
  [x500_cmd_bridge]
      ↓
  /fmu/in/trajectory_setpoint
      ↓
  PX4 SITL
      ↓
  Gazebo X500
      ↓
  Camera/IMU sensors
      ↓
  OpenVINS (closes the loop)

Gazebo TF
  world -> drone1 -> camera_link
      ↓
  [camera_pose_publisher]
      ↓
  /drone_0_camera/pose
      ↓
  EGO-Swarm Grid Map

Gazebo Sensors
  /gz/drone1/camera_depth/image_raw
      ↓
  (direct remap in launch file)
      ↓
  EGO-Swarm Grid Map
```

---

## 🎯 Next Steps

1. ✅ **Đã sửa**: Typo trong `single_run_in_sim.launch.py`
2. ✅ **Đã update**: Camera topics mapping
3. ⏳ **Cần làm**: Tạo 3 bridge nodes (odom, camera_pose, x500_cmd)
4. ⏳ **Cần test**: Run full integration
5. ⏳ **Cần tune**: PX4 controller parameters nếu cần

**Hãy bắt đầu với bridge node `odom_converter` trước vì đây là quan trọng nhất!**
