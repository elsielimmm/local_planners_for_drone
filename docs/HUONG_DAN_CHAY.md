# Hướng dẫn chạy EGO-Planner-Swarm ROS2

## ✅ Cài đặt đã hoàn thành thành công!

Workspace đã được build thành công với 24 packages.

## 🚀 Cách chạy demo

### Bước 1: Source môi trường (Quan trọng!)
Mỗi terminal mới cần chạy:
```bash
cd ~/ego_ws
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**Hoặc** tự động hóa bằng cách thêm vào `~/.bashrc`:
```bash
echo "source ~/ego_ws/install/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### Bước 2: Chạy simulation

#### Demo đơn máy bay (single drone):

**Terminal 1** - RViz (visualization):
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner rviz.launch.py
```

**Terminal 2** - Planner:
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner single_run_in_sim.launch.py
```

#### Demo swarm (nhiều máy bay):

**Terminal 1** - RViz:
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner rviz.launch.py
```

**Terminal 2** - Swarm planner:
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner swarm.launch.py
```

#### Demo swarm lớn:
```bash
ros2 launch ego_planner swarm_large.launch.py
```

### Bước 3: Điều khiển trong RViz

Sau khi RViz mở:
1. Click vào **"2D Nav Goal"** trên thanh công cụ RViz
2. Click và kéo trên bản đồ để đặt điểm đích cho drone
3. Drone sẽ tự động lập kế hoạch và bay đến đích

## ⚙️ Tùy chọn nâng cao

### Chọn loại bản đồ:
```bash
# Dùng Random Forest (mặc định)
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=False

# Dùng Mockamap
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True
```

### Bật/tắt mô phỏng động lực học:
```bash
# Không dùng dynamic model (mặc định, nhanh hơn)
ros2 launch ego_planner single_run_in_sim.launch.py use_dynamic:=False

# Bật dynamic model (mô phỏng thực tế hơn)
ros2 launch ego_planner single_run_in_sim.launch.py use_dynamic:=True
```

### Kết hợp tùy chọn:
```bash
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=True
```

## 🐛 Xử lý sự cố

### Lỗi: "no Qt platform plugin" / RViz không hiển thị
- **Nguyên nhân**: Đang dùng SSH hoặc không có X server
- **Giải pháp**: 
  1. Chạy trực tiếp trên máy có GUI, hoặc
  2. Dùng X11 forwarding: `ssh -X user@host`, hoặc
  3. Dùng VNC/Remote Desktop

### Lỗi: Simulation chạy chậm/lag
- **Kiểm tra DDS**: 
  ```bash
  ros2 doctor --report | grep "RMW middleware"
  ```
  Phải hiển thị `rmw_cyclonedds_cpp`

- **Tăng hiệu suất CPU** (tùy chọn):
  ```bash
  sudo apt install cpufrequtils
  sudo cpufreq-set -g performance
  ```

### Lỗi: "package 'ego_planner' not found"
- Source lại workspace:
  ```bash
  source ~/ego_ws/install/setup.bash
  ```

### Build lại nếu cần:
```bash
cd ~/ego_ws
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 📦 Thông tin các gói đã cài

- **ego_planner**: Planner chính cho swarm
- **plan_env**: Environment mapping và obstacle detection
- **path_searching**: A* search algorithm
- **bspline_opt**: B-spline trajectory optimization
- **local_sensing**: Sensor simulation (depth camera/pointcloud)
- **mockamap**: Map generator cho testing
- **so3_control**: Quadrotor controller
- **perception_pcl**: PCL bindings cho ROS2 (mình đã thêm)
- **pcl_msgs**: PCL message definitions (mình đã thêm)

## 📚 Tài liệu tham khảo

- GitHub repo: https://github.com/ZJU-FAST-Lab/ego-planner-swarm
- Branch ROS2: https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version
- Paper: "EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System" (ICRA 2021)

## 💡 Lưu ý

- **Warnings trong build**: Các deprecation warnings là bình thường, code vẫn hoạt động tốt
- **CycloneDDS**: Bắt buộc phải dùng để tránh lag với FastDDS
- **Môi trường**: Luôn nhớ `source ~/ego_ws/install/setup.bash` khi mở terminal mới
- **Performance**: Nếu chạy swarm lớn, tắt dynamics (`use_dynamic:=False`) để tăng tốc

---
**Cài đặt hoàn thành bởi GitHub Copilot - October 14, 2025**
