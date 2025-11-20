# Tích hợp X500 Gazebo và OpenVINS vào EGO-Swarm

## 📋 Tổng quan

Hướng dẫn này giúp bạn thay thế:
- **Simulator giả lập** (so3_quadrotor_simulator) → **Gazebo với X500 (PX4)**
- **Odometry giả lập** → **OpenVINS (Visual-Inertial Odometry)**

## 🎯 Mục tiêu

1. Sử dụng drone X500 thực tế trong Gazebo thay vì quadrotor simulator
2. Tích hợp OpenVINS để có VIO với noise/drift thực tế
3. Chuẩn bị cho việc deploy lên hardware thực

## 📁 Cấu trúc Tài liệu

### 1. **QUICK_START_X500_INTEGRATION.md** ⭐ BẮT ĐẦU TỪ ĐÂY
- Hướng dẫn từng bước nhanh (30-45 phút)
- Checklist để debug
- Commands cụ thể để copy-paste

### 2. **GAZEBO_X500_OPENVINS_INTEGRATION.md** 📖 CHI TIẾT ĐẦY ĐỦ
- Giải thích từng module
- Source code đầy đủ cho bridge nodes
- Troubleshooting chi tiết
- Xử lý vấn đề frame transformation (NED vs ENU)

### 3. **ARCHITECTURE_X500_INTEGRATION.md** 🏗️ KIẾN TRÚC HỆ THỐNG
- Sơ đồ kiến trúc mới
- Luồng dữ liệu giữa các module
- So sánh trước/sau tích hợp
- Topic mapping table

## 🚀 Quick Start (Tóm tắt)

### Bước 1: Tạo Bridge Package
```bash
cd ~/ego_ws/src/ego-planner-swarm/src/uav_simulator/Utils
ros2 pkg create --build-type ament_cmake x500_bridge \
  --dependencies rclcpp nav_msgs quadrotor_msgs px4_msgs geometry_msgs
```

### Bước 2: Tạo Bridge Nodes
Tạo 2 files:
1. `x500_bridge/src/odom_converter.cpp` - Convert OpenVINS → EGO-Swarm format
2. `x500_bridge/src/x500_cmd_bridge.cpp` - Convert EGO-Swarm → PX4 format

(Chi tiết code xem trong **GAZEBO_X500_OPENVINS_INTEGRATION.md** phần 4)

### Bước 3: Build
```bash
cd ~/ego_ws
colcon build --packages-select x500_bridge
source install/setup.bash
```

### Bước 4: Test
```bash
# Terminal 1: X500 Gazebo
ros2 launch <your_px4_pkg> x500.launch.py

# Terminal 2: OpenVINS
ros2 launch ov_msckf <your_config>.launch.py

# Terminal 3: Odom Bridge
ros2 run x500_bridge odom_converter

# Terminal 4: Command Bridge
ros2 run x500_bridge x500_cmd_bridge

# Terminal 5: EGO-Planner
ros2 launch ego_planner ego_with_x500.launch.py

# Terminal 6: Set goal
ros2 topic pub --once /goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 5.0, y: 5.0, z: 1.0}}}"
```

## 🔧 Components Cần Thiết

### Hardware/Software Requirements
- ✅ ROS2 Humble
- ✅ Gazebo (Garden hoặc Classic)
- ✅ PX4 SITL với X500 model
- ✅ OpenVINS (đã compile)
- ✅ EGO-Swarm workspace này

### Dependencies cần cài thêm
```bash
sudo apt install ros-humble-px4-msgs
```

## 📊 Kiến trúc Hệ thống Mới

```
┌──────────────┐
│ Gazebo X500  │ ← Simulator thực tế thay vì so3_quadrotor
└──────┬───────┘
       │ Sensors (Camera, IMU)
       ↓
┌──────────────┐
│  OpenVINS    │ ← VIO thực tế thay vì odometry giả lập
└──────┬───────┘
       │ /ov_msckf/odometry_imu
       ↓
┌──────────────┐
│odom_converter│ ← Bridge mới (convert topics)
└──────┬───────┘
       │ odom_world
       ↓
┌──────────────┐
│ EGO-Planner  │ ← Module chính (giữ nguyên)
└──────┬───────┘
       │ planning/pos_cmd
       ↓
┌──────────────┐
│x500_cmd_     │ ← Bridge mới (convert commands)
│   bridge     │
└──────┬───────┘
       │ /fmu/in/trajectory_setpoint
       ↓
┌──────────────┐
│  PX4 + X500  │ ← Thực thi lệnh điều khiển
└──────────────┘
```

## 🔍 Topics Quan trọng

### Topics mà EGO-Swarm cần (INPUT)
| Topic | Type | From |
|-------|------|------|
| `odom_world` hoặc `/drone_X_visual_slam/odom` | nav_msgs/Odometry | odom_converter |
| `depth` | sensor_msgs/Image | X500 camera hoặc pcl_render |
| `/map_generator/global_cloud` | sensor_msgs/PointCloud2 | map_generator |

### Topics mà EGO-Swarm publish (OUTPUT)
| Topic | Type | To |
|-------|------|-----|
| `/drone_X_planning/pos_cmd` | quadrotor_msgs/PositionCommand | x500_cmd_bridge |
| `/drone_X_planning/swarm_trajs` | traj_utils/MultiBsplines | Other drones |

## 🎓 Quy trình Học tập Đề xuất

### Level 1: Hiểu cơ bản (1-2 giờ)
1. Đọc **ARCHITECTURE_X500_INTEGRATION.md**
2. Xem sơ đồ kiến trúc
3. Hiểu luồng dữ liệu

### Level 2: Thực hành cơ bản (2-3 giờ)
1. Làm theo **QUICK_START_X500_INTEGRATION.md**
2. Test từng component riêng lẻ
3. Chạy full system với single drone

### Level 3: Nâng cao (3-5 giờ)
1. Đọc chi tiết **GAZEBO_X500_OPENVINS_INTEGRATION.md**
2. Hiểu code của bridge nodes
3. Modify parameters cho phù hợp với setup của bạn
4. Test với swarm (nhiều drones)

### Level 4: Deploy thực tế (1-2 tuần)
1. Thay Gazebo bằng hardware X500 thực
2. Tune OpenVINS cho môi trường thực
3. Calibrate camera và IMU
4. Flight test

## ⚠️ Lưu ý Quan trọng

### 1. Frame Convention
- **PX4/Gazebo**: NED (North-East-Down)
- **ROS2/EGO-Swarm**: ENU (East-North-Up)
- **OpenVINS**: Thường ENU, nhưng cần verify

→ Bridge nodes phải convert: Z(ENU) = -Z(NED)

### 2. Topic Names
Đảm bảo topic names match:
```bash
# Kiểm tra
ros2 topic list | grep odom
ros2 topic list | grep planning
```

### 3. Timestamp Sync
OpenVINS và PX4 có thể dùng clock khác nhau:
```cpp
out_msg.header.stamp = this->now();  // Dùng ROS2 clock
```

### 4. Depth Camera
Nếu X500 không có depth camera sẵn:
- **Option 1**: Thêm Gazebo depth plugin (xem hướng dẫn trong docs)
- **Option 2**: Dùng `pcl_render_node` của EGO-Swarm

## 🐛 Troubleshooting

### "No odom received"
```bash
# Check topic
ros2 topic hz /drone_0_visual_slam/odom
# Fix: Kiểm tra odom_converter parameters
```

### "X500 not moving"
```bash
# Check PX4 mode
ros2 topic echo /fmu/out/vehicle_status --field nav_state
# Fix: Enable offboard mode trong PX4
```

### "Planning fails"
```bash
# Check map
ros2 topic echo /map_generator/global_cloud --once
# Fix: Đảm bảo map_generator chạy
```

### "Transform errors"
```bash
# Publish static TF
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
```

## 📚 Tài liệu Tham khảo

- **EGO-Swarm Paper**: https://arxiv.org/abs/2011.04800
- **PX4 Documentation**: https://docs.px4.io/
- **OpenVINS Docs**: https://docs.openvins.com/
- **ROS2 Humble**: https://docs.ros.org/en/humble/

## 🤝 Đóng góp

Nếu bạn tìm thấy bug hoặc có cải tiến, hãy:
1. Document lại vấn đề
2. Test fix của bạn
3. Update tài liệu này

## 📞 Hỗ trợ

Khi gặp vấn đề:
1. Check **QUICK_START_X500_INTEGRATION.md** phần "Common Errors"
2. Xem **GAZEBO_X500_OPENVINS_INTEGRATION.md** phần "Troubleshooting"
3. Debug bằng `ros2 topic` commands
4. Check ROS2 logs: `ros2 run rqt_console rqt_console`

## ✅ Testing Checklist

- [ ] X500 spawn trong Gazebo
- [ ] OpenVINS chạy và publish odom (>30Hz)
- [ ] odom_converter convert topics
- [ ] EGO-Swarm nhận odom_world
- [ ] Map generator publish map
- [ ] Planning tạo trajectory khi set goal
- [ ] x500_cmd_bridge gửi commands
- [ ] X500 di chuyển trong Gazebo
- [ ] Collision avoidance hoạt động
- [ ] Swarm coordination (multi-drone)

## 🎉 Kết quả Mong đợi

Sau khi hoàn thành, bạn sẽ có:
1. ✅ X500 Gazebo bay theo trajectory của EGO-Swarm
2. ✅ OpenVINS cung cấp odometry với noise thực tế
3. ✅ Planner tránh chướng ngại vật
4. ✅ Multi-drone swarm coordination
5. ✅ Nền tảng sẵn sàng cho deploy hardware thực

---

**Good luck! 🚁**
