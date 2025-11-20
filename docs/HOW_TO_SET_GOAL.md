# 🎯 Hướng dẫn Set Goal Point cho EGO-Planner

## 📋 Hiện tại bạn gặp vấn đề gì?

Khi chạy `single_run_in_sim.launch.py`, drone **tự động bay đến điểm preset** thay vì đợi bạn click goal trong RViz.

## 🔍 Nguyên nhân

Trong file launch, có tham số `flight_type` quyết định cách drone hoạt động:

```python
flight_type = LaunchConfiguration('flight_type', default=2)
```

**Giá trị của `flight_type`:**
- `flight_type = 1`: **MANUAL_CTRL** - Đợi bạn click goal trong RViz ✅
- `flight_type = 2`: **PRESET_WAYPOINTS** - Tự động bay theo waypoints đã set ❌
- `flight_type = 3`: **KINO_REPLAN** - Advanced mode

---

## ✅ GIẢI PHÁP 1: Thay đổi flight_type (Khuyến nghị)

### Cách 1A: Truyền tham số khi chạy (Nhanh nhất)

```bash
source ~/ego_ws/install/setup.bash

ros2 launch ego_planner single_run_in_sim.launch.py \
  flight_type:=1
```

Bây giờ drone sẽ **đợi bạn click goal trong RViz**!

---

### Cách 1B: Sửa file launch (Permanent)

Mở file:
```bash
nano ~/ego_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py
```

Tìm dòng ~128 (trong phần `launch_arguments`):
```python
'flight_type': str(2),  # ← Thay đổi dòng này
```

**Đổi thành:**
```python
'flight_type': str(1),  # Manual control via RViz
```

Sau đó rebuild:
```bash
cd ~/ego_ws
colcon build --packages-select ego_planner --symlink-install
source install/setup.bash
```

---

## ✅ GIẢI PHÁP 2: Set Goal trong Code (Nếu muốn hardcode)

Nếu bạn muốn drone tự động bay đến **một điểm cụ thể của bạn**, giữ `flight_type=2` nhưng đổi waypoints:

### Cách 2A: Truyền tham số khi chạy

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  flight_type:=2 \
  point_num:=1 \
  point0_x:=5.0 \
  point0_y:=3.0 \
  point0_z:=1.5
```

Drone sẽ tự động bay đến `(5.0, 3.0, 1.5)`.

**Nhiều waypoints:**
```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  flight_type:=2 \
  point_num:=3 \
  point0_x:=5.0 point0_y:=0.0 point0_z:=1.0 \
  point1_x:=10.0 point1_y:=5.0 point1_z:=1.5 \
  point2_x:=-5.0 point2_y:=-3.0 point2_z:=1.0
```

Drone sẽ bay: Start → Point0 → Point1 → Point2 → Point0 (loop)

---

### Cách 2B: Sửa trong launch file

Mở file:
```bash
nano ~/ego_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py
```

Tìm dòng ~120-133:
```python
'point_num': str(4),           # Số waypoints
'point0_x': str(15.0),         # Waypoint 0
'point0_y': str(0.0),
'point0_z': str(1.0),

'point1_x': str(-15.0),        # Waypoint 1
'point1_y': str(0.0),
'point1_z': str(1.0),

'point2_x': str(15.0),         # Waypoint 2
'point2_y': str(0.0),
'point2_z': str(1.0),
```

**Ví dụ thay đổi:**
```python
'point_num': str(2),           # Chỉ 2 waypoints
'point0_x': str(5.0),          # Điểm đầu tiên của bạn
'point0_y': str(3.0),
'point0_z': str(1.5),

'point1_x': str(-5.0),         # Điểm thứ hai
'point1_y': str(-3.0),
'point1_z': str(1.0),
```

---

## ✅ GIẢI PHÁP 3: Set Goal qua RViz (Interactive)

Nếu đã set `flight_type=1`:

### Bước 1: Chạy hệ thống

**Terminal 1**: Publish map (nếu dùng map tùy chỉnh)
```bash
source ~/ego_ws/install/setup.bash
ros2 run ego_planner publish_static_map.py \
  --ros-args -p map_file:=/path/to/your/map.ply
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
  flight_type:=1
```

### Bước 2: Click goal trong RViz

1. Trong RViz, tìm tool **"2D Nav Goal"** trên thanh công cụ (hoặc press `G`)
2. Click vào map tại vị trí bạn muốn drone bay đến
3. Kéo chuột để set hướng (yaw)
4. Drone sẽ tự động lập kế hoạch và bay!

**Lưu ý**: Tool "2D Nav Goal" chỉ set X, Y. Để set Z (độ cao), dùng cách 4.

---

## ✅ GIẢI PHÁP 4: Publish Goal qua Terminal

Nếu muốn control qua command line:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
"{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'world'
  },
  pose: {
    position: {x: 5.0, y: 3.0, z: 1.5},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

**Tạo script để dễ dùng:**

```bash
cat > ~/ego_ws/scripts/set_goal.sh << 'EOF'
#!/bin/bash
# Usage: ./set_goal.sh <x> <y> <z>

X=${1:-0.0}
Y=${2:-0.0}
Z=${3:-1.0}

ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
"{
  header: {frame_id: 'world'},
  pose: {
    position: {x: $X, y: $Y, z: $Z},
    orientation: {w: 1.0}
  }
}"

echo "Goal set to: ($X, $Y, $Z)"
EOF

chmod +x ~/ego_ws/scripts/set_goal.sh
```

**Sử dụng:**
```bash
~/ego_ws/scripts/set_goal.sh 5.0 3.0 1.5
```

---

## 📊 So sánh các cách

| Cách | Ưu điểm | Nhược điểm | Khi nào dùng |
|------|---------|------------|--------------|
| **1A. flight_type=1 (tham số)** | Nhanh, không cần rebuild | Phải gõ mỗi lần | Test, thay đổi thường xuyên |
| **1B. Sửa launch file** | Permanent, tự động | Cần rebuild | Setup lần đầu |
| **2A. Preset waypoints (tham số)** | Tự động, lặp lại | Không linh hoạt | Demo, testing tự động |
| **2B. Sửa waypoints trong code** | Permanent preset | Cần rebuild | Fixed mission |
| **3. Click RViz** | Interactive, trực quan | Cần GUI | Development, testing |
| **4. Publish terminal** | Script được, automation | Hơi dài dòng | Automation, ROS bag |

---

## 🎯 KHUYẾN NGHỊ CHO BẠN

### Cho development và testing:

```bash
# Chạy với flight_type=1
ros2 launch ego_planner single_run_in_sim.launch.py \
  flight_type:=1 \
  init_x_:=0.0 \
  init_y_:=0.0 \
  init_z_:=1.0
```

Sau đó click goal trong RViz hoặc dùng script `set_goal.sh`.

### Cho demo/presentation:

Sửa launch file với preset waypoints đẹp:

```python
'flight_type': str(2),
'point_num': str(3),
'point0_x': str(5.0),
'point0_y': str(0.0),
'point0_z': str(1.5),
# ... các waypoints khác
```

---

## 🔧 Debug: Kiểm tra goal đang set

```bash
# Xem topic goal
ros2 topic list | grep goal

# Echo goal hiện tại
ros2 topic echo /goal

# Xem thông tin FSM state
ros2 topic echo /drone_0_planning/data_display
```

---

## 📝 Tóm tắt Quick Commands

### Cho Interactive Control (Click RViz):
```bash
ros2 launch ego_planner single_run_in_sim.launch.py flight_type:=1
```

### Cho Auto Mission (1 điểm):
```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  flight_type:=2 point_num:=1 \
  point0_x:=5.0 point0_y:=3.0 point0_z:=1.5
```

### Cho Auto Tour (nhiều điểm):
```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  flight_type:=2 point_num:=3 \
  point0_x:=5.0 point0_y:=0.0 point0_z:=1.0 \
  point1_x:=0.0 point1_y:=5.0 point1_z:=1.5 \
  point2_x:=-5.0 point2_y:=0.0 point2_z:=1.0
```

### Set goal qua terminal:
```bash
~/ego_ws/scripts/set_goal.sh 5.0 3.0 1.5
```

---

**Chúc bạn thành công!** 🎉

Nếu cần thêm, hãy cho tôi biết:
- Bạn muốn dùng cách nào?
- Map của bạn có kích thước bao nhiêu?
- Bạn cần set bao nhiêu waypoints?
