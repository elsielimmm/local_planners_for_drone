# 🎯 QUICK REFERENCE: Set Goal cho EGO-Planner

## TL;DR - Giải pháp nhanh nhất

### Muốn click goal trong RViz? (Khuyến nghị)

```bash
ros2 launch ego_planner single_run_in_sim.launch.py flight_type:=1
```

Sau đó click tool "2D Nav Goal" trong RViz và click vào map!

---

### Muốn drone tự bay đến 1 điểm cố định?

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  flight_type:=2 \
  point_num:=1 \
  point0_x:=5.0 \
  point0_y:=3.0 \
  point0_z:=1.5
```

---

### Muốn set goal qua terminal?

```bash
~/ego_ws/scripts/set_goal.sh 5.0 3.0 1.5
```

---

## Chi tiết flight_type

- `flight_type:=1` → Đợi bạn click goal trong RViz
- `flight_type:=2` → Tự động bay theo waypoints preset
- `flight_type:=3` → Advanced kinodynamic mode

---

## Map của bạn

Thông tin từ log:
```
X: [-9.11, 9.34]   (kích thước: ~18m)
Y: [-22.65, 25.25] (kích thước: ~48m)
Z: [-0.12, 5.05]   (độ cao: ~5m)
Points: 169,104 (sau downsample)
```

**Gợi ý waypoints phù hợp:**
- Start: `(0, 0, 1)` - Trung tâm map
- Goal examples:
  - `(5, 10, 1.5)` - Phía đông bắc
  - `(-5, -10, 2.0)` - Phía tây nam
  - `(0, 20, 1.0)` - Phía bắc

---

## Full Command với Map của bạn

```bash
# Terminal 1: Publish map
source ~/ego_ws/install/setup.bash
ros2 run ego_planner publish_static_map.py \
  --ros-args -p map_file:=~/ego_ws/scripts/urbanroom.ply

# Terminal 2: RViz
ros2 launch ego_planner rviz.launch.py

# Terminal 3: Planner (manual control)
ros2 launch ego_planner single_run_in_sim.launch.py \
  flight_type:=1 \
  init_x_:=0.0 \
  init_y_:=0.0 \
  init_z_:=1.0 \
  map_size_x:=20.0 \
  map_size_y:=50.0 \
  map_size_z:=6.0

# Terminal 4: Set goal
~/ego_ws/scripts/set_goal.sh 5.0 10.0 1.5
```

---

## Troubleshooting

### Drone không bay?
- Kiểm tra `flight_type:=1` đã set chưa
- Publish goal: `~/ego_ws/scripts/set_goal.sh X Y Z`
- Xem log: `ros2 topic echo /drone_0_planning/data_display`

### Map không hiện?
```bash
ros2 topic echo /map_generator/global_cloud --no-arr
```

### Python error "rclpy._rclpy_pybind11"?
Dùng `ros2 run` thay vì `python3`:
```bash
ros2 run ego_planner publish_static_map.py --ros-args -p map_file:=...
```

---

**Đọc chi tiết**: `HOW_TO_SET_GOAL.md`
