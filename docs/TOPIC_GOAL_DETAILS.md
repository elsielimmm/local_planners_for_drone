# 🎯 Chi tiết Topic Goal trong EGO-Planner

## 📍 Vị trí Subscribe Goal

**File**: `/home/quangsang/ego_ws/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp`

**Dòng 117-123**: Subscribe topic goal

```cpp
waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/move_base_simple/goal",  // ← TOPIC GOAL
    1,
    [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
    {
        this->waypointCallback(msg);
    });
```

**Callback function** (Dòng 235-246):
```cpp
void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
{
    if (msg->pose.position.z < -0.1)
        return;

    cout << "Triggered!" << endl;

    init_pt_ = odom_pos_;

    // LẤY TỌA ĐỘ MỤC TIÊU
    Eigen::Vector3d end_wp(msg->pose.position.x, 
                           msg->pose.position.y, 
                           1.0);  // ← Z luôn = 1.0 (hardcoded!)

    planNextWaypoint(end_wp);
}
```

---

## 🔑 Thông tin quan trọng

### 1. Topic Name

**Hiện tại code subscribe:**
```
/move_base_simple/goal
```

**RViz2 mặc định publish:**
```
/goal_pose
```

→ **Vấn đề**: Topic names không khớp! ❌

---

## ✅ Giải pháp

### **Option A: Sửa code C++ để subscribe đúng topic RViz2**

Sửa file: `src/planner/plan_manage/src/ego_replan_fsm.cpp`

**Dòng 118**, đổi:
```cpp
"/move_base_simple/goal",  // ROS 1 topic
```

Thành:
```cpp
"/goal_pose",  // ROS 2 topic
```

Sau đó rebuild:
```bash
cd ~/ego_ws
colcon build --packages-select ego_planner --symlink-install
source install/setup.bash
```

---

### **Option B: Remap topic khi launch (Không cần rebuild)**

Trong launch file, thêm remap:

```python
ego_planner_node = Node(
    package='ego_planner',
    executable='ego_planner_node',
    remappings=[
        ('/move_base_simple/goal', '/goal_pose'),  # ← Thêm dòng này
        # ... các remapping khác
    ]
)
```

Hoặc khi chạy:
```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  --ros-args -r /move_base_simple/goal:=/goal_pose
```

---

### **Option C: Remap RViz topic (Đơn giản nhất)**

Sửa file RViz config: `src/planner/plan_manage/launch/default.rviz`

Tìm dòng có:
```yaml
Value: /goal_pose
```

Đổi thành:
```yaml
Value: /move_base_simple/goal
```

---

## 🎯 Khuyến nghị: Sửa code C++ (Option A)

Vì ROS 2 dùng `/goal_pose` là chuẩn, nên sửa code để subscribe đúng topic ROS 2.

### Các bước thực hiện:

**1. Mở file:**
```bash
nano ~/ego_ws/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp
```

**2. Tìm dòng 118 (hoặc search `/move_base_simple/goal`):**
```cpp
"/move_base_simple/goal",
```

**3. Đổi thành:**
```cpp
"/goal_pose",
```

**4. Rebuild:**
```bash
cd ~/ego_ws
colcon build --packages-select ego_planner --symlink-install
source install/setup.bash
```

**5. Test:**
```bash
# Terminal 1: RViz
ros2 launch ego_planner rviz.launch.py

# Terminal 2: Planner
ros2 launch ego_planner single_run_in_sim.launch.py flight_type:=1

# Trong RViz, click "2D Goal Pose" và click vào map
```

---

## ⚠️ Vấn đề với Z coordinate

**Lưu ý quan trọng trong code (dòng 245):**
```cpp
Eigen::Vector3d end_wp(msg->pose.position.x, 
                       msg->pose.position.y, 
                       1.0);  // ← Z luôn = 1.0!
```

**Vấn đề**: Code **hardcode Z = 1.0**, bỏ qua giá trị Z từ RViz!

### Sửa để dùng Z từ RViz:

**Tìm dòng 245:**
```cpp
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
```

**Đổi thành:**
```cpp
Eigen::Vector3d end_wp(msg->pose.position.x, 
                       msg->pose.position.y, 
                       msg->pose.position.z);  // Dùng Z từ msg
```

Hoặc với giá trị min/max:
```cpp
double z = std::max(0.5, std::min(msg->pose.position.z, 3.0)); // Clamp [0.5, 3.0]
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, z);
```

---

## 🧪 Test Topic

### Kiểm tra topic đang hoạt động:

```bash
# List tất cả topics
ros2 topic list | grep goal

# Echo topic từ RViz
ros2 topic echo /goal_pose

# Echo topic mà code subscribe
ros2 topic echo /move_base_simple/goal

# Publish test thủ công
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
"{
  header: {frame_id: 'world'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 1.5},
    orientation: {w: 1.0}
  }
}"
```

### Kiểm tra subscriber:

```bash
# Xem ai đang subscribe
ros2 topic info /move_base_simple/goal

# Xem ai đang publish
ros2 topic info /goal_pose
```

---

## 📝 Summary: Thay đổi cần thiết

### **Thay đổi 1: Topic name**
```cpp
// File: ego_replan_fsm.cpp, dòng 118
// BEFORE:
"/move_base_simple/goal",

// AFTER:
"/goal_pose",
```

### **Thay đổi 2: Z coordinate (Optional)**
```cpp
// File: ego_replan_fsm.cpp, dòng 245
// BEFORE:
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);

// AFTER:
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
```

### **Rebuild:**
```bash
cd ~/ego_ws
colcon build --packages-select ego_planner --symlink-install
source install/setup.bash
```

---

## 🎬 Workflow sau khi sửa

```bash
# Terminal 1: RViz
ros2 launch ego_planner rviz.launch.py

# Terminal 2: Planner (manual mode)
ros2 launch ego_planner single_run_in_sim.launch.py flight_type:=1

# Trong RViz:
# 1. Click tool "2D Goal Pose" (hoặc press G)
# 2. Click vào map tại vị trí muốn drone bay đến
# 3. Drone sẽ nhận goal và bắt đầu planning!
```

---

## 🔍 Debug

Nếu goal không hoạt động:

```bash
# 1. Kiểm tra topic có được publish không
ros2 topic echo /goal_pose

# 2. Kiểm tra planner có subscribe không
ros2 node info /drone_0_ego_planner_node | grep Subscriptions

# 3. Xem log của planner
ros2 launch ego_planner single_run_in_sim.launch.py flight_type:=1 2>&1 | grep -i "trigger\|goal"
```

---

**File code chính**: `src/planner/plan_manage/src/ego_replan_fsm.cpp`
**Dòng subscribe**: 117-123
**Callback**: 235-246
**Topic cần sửa**: `/move_base_simple/goal` → `/goal_pose`
