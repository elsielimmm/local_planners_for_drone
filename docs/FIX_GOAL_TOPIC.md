# 📍 TÓM TẮT: Topic Goal trong EGO-Planner

## ❓ Vấn đề

**RViz2** publish goal tới topic: `/goal_pose`  
**EGO-Planner** subscribe từ topic: `/move_base_simple/goal` (ROS 1 legacy)

→ **Không khớp!** Goal từ RViz không đến được planner.

---

## ✅ Giải pháp nhanh (1 lệnh)

```bash
~/ego_ws/scripts/fix_goal_topic.sh
```

Script này sẽ:
1. ✓ Đổi topic: `/move_base_simple/goal` → `/goal_pose`
2. ✓ Sửa Z coordinate: dùng giá trị từ RViz thay vì hardcode 1.0
3. ✓ Backup file gốc
4. ✓ Rebuild ego_planner

---

## 📂 Chi tiết code

**File**: `src/planner/plan_manage/src/ego_replan_fsm.cpp`

**Dòng 118** - Subscribe topic:
```cpp
// BEFORE:
"/move_base_simple/goal",  // ROS 1 topic ❌

// AFTER:
"/goal_pose",  // ROS 2 topic ✅
```

**Dòng 245** - Lấy tọa độ Z:
```cpp
// BEFORE:
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0); // ❌

// AFTER:
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z); // ✅
```

---

## 🧪 Test sau khi fix

```bash
# Terminal 1: RViz
ros2 launch ego_planner rviz.launch.py

# Terminal 2: Planner (manual mode)
ros2 launch ego_planner single_run_in_sim.launch.py flight_type:=1

# Trong RViz:
# 1. Click "2D Goal Pose" tool (hoặc nhấn G)
# 2. Click vào map
# 3. Xem terminal planner - sẽ thấy "Triggered!"
```

---

## 🔍 Debug nếu không hoạt động

```bash
# Kiểm tra topic có đúng không
ros2 topic list | grep goal

# Echo goal từ RViz
ros2 topic echo /goal_pose

# Kiểm tra planner có subscribe không
ros2 node info /drone_0_ego_planner_node | grep -A 10 Subscriptions
```

---

## 🔙 Restore nếu cần

```bash
# Restore file gốc
cp ~/ego_ws/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp.backup \
   ~/ego_ws/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp

# Rebuild
cd ~/ego_ws
colcon build --packages-select ego_planner --symlink-install
```

---

## 📚 Đọc thêm

- **TOPIC_GOAL_DETAILS.md** - Chi tiết đầy đủ
- **HOW_TO_SET_GOAL.md** - Cách set goal khác nhau
- **QUICK_GOAL_REFERENCE.md** - Quick cheat sheet

---

## 🎯 Flow hoàn chỉnh

```
RViz Click "2D Goal Pose" 
    ↓
Publish to /goal_pose
    ↓
EGO-Planner subscribe /goal_pose  ← (Sau khi fix)
    ↓
waypointCallback() nhận PoseStamped
    ↓
Extract (x, y, z) coordinates  ← (Sau khi fix Z)
    ↓
planNextWaypoint(end_wp)
    ↓
Drone bay đến goal! ✅
```

---

**Chạy ngay**: `~/ego_ws/scripts/fix_goal_topic.sh`
