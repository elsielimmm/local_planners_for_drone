# 📚 TÓM TẮT: Sử dụng Map .ply với EGO-Planner trong Gazebo

## 🎯 Bạn có gì:
- ✅ Map tĩnh đã quét sẵn (file .ply)
- ✅ EGO-Planner-Swarm (ROS 2)
- ❓ Muốn: Chạy drone trong Gazebo với map này

## 🛠️ Tôi đã tạo cho bạn:

### 📁 Scripts (~/ego_ws/scripts/)
1. **convert_ply_to_dae.py** - Convert .ply sang format Gazebo
2. **publish_static_map.py** - Publish map cho EGO-Planner  
3. **position_cmd_to_gazebo.py** - Adapter commands

### 📖 Hướng dẫn
1. **GAZEBO_INTEGRATION_GUIDE.md** - Hướng dẫn đầy đủ, chi tiết
2. **QUICK_START_GAZEBO.md** - Quick start guide
3. **CUSTOM_MAP_GUIDE.md** - Load map không dùng Gazebo

---

## ⚡ Quick Start (3 phút)

### Bước 1: Cài đặt
```bash
# Gazebo
sudo apt-get install ros-humble-ros-gz

# Python packages
pip3 install open3d trimesh
```

### Bước 2: Convert map
```bash
cd ~/ego_ws/scripts

# Thay /path/to/your/map.ply bằng đường dẫn thực
python3 convert_ply_to_dae.py \
  /path/to/your/map.ply \
  ~/ego_ws/maps/map.dae \
  --simplify 10000
```

### Bước 3: Test trong Gazebo

Tạo file world đơn giản:
```bash
mkdir -p ~/ego_ws/worlds
cat > ~/ego_ws/worlds/test.sdf << 'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="test">
    <model name="map">
      <static>true</static>
      <link name="link">
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
EOF

# Test
gz sim ~/ego_ws/worlds/test.sdf
```

**Nếu thấy map → Thành công! ✓**

### Bước 4: Publish map cho EGO-Planner

```bash
source ~/ego_ws/install/setup.bash

python3 ~/ego_ws/scripts/publish_static_map.py \
  --ros-args \
  -p map_file:=/path/to/your/map.ply \
  -p downsample_voxel_size:=0.1
```

### Bước 5: Chạy EGO-Planner

Mở file launch, comment map generator:
```bash
nano ~/ego_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py
```

Tìm và comment dòng ~195:
```python
# ld.add_action(map_generator_node)  # <-- Comment dòng này
```

Sau đó chạy:
```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner single_run_in_sim.launch.py
```

---

## 🎮 2 Tùy Chọn:

### **Option A: Chỉ test Path Planning (Không cần Gazebo)**

Nếu bạn chỉ muốn:
- Visualize map
- Test path planning algorithm  
- Không cần physics simulation

👉 Đọc: **CUSTOM_MAP_GUIDE.md**

**Ưu điểm**: Đơn giản, nhanh, không cần setup Gazebo  
**Nhược điểm**: Không có dynamics, không có sensor simulation

---

### **Option B: Full Gazebo Simulation**

Nếu bạn muốn:
- Simulation đầy đủ với physics
- Sensor simulation (depth camera, IMU)
- Dynamics thực tế của drone

👉 Đọc: **GAZEBO_INTEGRATION_GUIDE.md**

**Ưu điểm**: Realistic, có thể test control  
**Nhược điểm**: Phức tạp hơn, cần setup nhiều

---

## 📊 So sánh

| Feature | Option A (Không Gazebo) | Option B (Gazebo) |
|---------|------------------------|-------------------|
| Map tĩnh .ply | ✅ | ✅ |
| Path planning | ✅ | ✅ |
| Visualization | ✅ | ✅ |
| Physics simulation | ❌ | ✅ |
| Sensor simulation | ❌ | ✅ |
| Drone dynamics | ❌ | ✅ |
| Setup complexity | Dễ (5 phút) | Trung bình (30 phút) |

---

## 🤔 Bạn nên chọn gì?

### Chọn Option A nếu:
- Chỉ muốn test thuật toán path planning
- Muốn visualize map và trajectory
- Map đã có sẵn, chỉ cần load

### Chọn Option B nếu:
- Cần test với drone thật sau này
- Muốn có sensor data (depth, IMU)
- Cần dynamics và control loop đầy đủ

---

## 🚀 Bắt đầu ngay

### Cho Option A (Recommended để bắt đầu):
```bash
# Terminal 1: Publish map
cd ~/ego_ws
python3 scripts/publish_static_map.py \
  --ros-args -p map_file:=/your/map.ply

# Terminal 2: RViz
ros2 launch ego_planner rviz.launch.py

# Terminal 3: Planner  
ros2 launch ego_planner single_run_in_sim.launch.py
```

### Cho Option B (Advanced):
```bash
# Đọc GAZEBO_INTEGRATION_GUIDE.md
# Làm theo từng bước
```

---

## 🆘 Cần giúp đỡ?

1. **Map không hiện trong RViz?**
   ```bash
   # Kiểm tra topic
   ros2 topic list | grep global_cloud
   ros2 topic echo /map_generator/global_cloud --no-arr
   ```

2. **File .dae quá lớn?**
   ```bash
   # Convert lại với simplify cao hơn
   python3 convert_ply_to_dae.py map.ply map_light.dae --simplify 5000
   ```

3. **Python import error?**
   ```bash
   pip3 install open3d trimesh numpy
   ```

4. **Cần thay đổi vị trí bắt đầu?**
   ```bash
   ros2 launch ego_planner single_run_in_sim.launch.py \
     init_x_:=2.0 \
     init_y_:=3.0 \
     init_z_:=1.5
   ```

---

## 📝 Next Steps

1. ✅ Test với Option A trước
2. ✅ Verify map được load đúng
3. ✅ Test path planning với goal đơn giản
4. ⏭️ Nếu cần: Chuyển sang Option B (Gazebo)

---

**Chúc bạn thành công!** 🎉

Nếu cần hỗ trợ, hãy cung cấp:
- Đường dẫn file .ply của bạn
- Kích thước file (số điểm, MB)
- Loại môi trường (indoor/outdoor)
- Lỗi gặp phải (nếu có)
