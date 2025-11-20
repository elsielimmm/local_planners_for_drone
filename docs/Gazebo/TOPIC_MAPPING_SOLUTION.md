# 🔗 Topic Mapping Solution cho X500 + OpenVINS + EGO-Swarm

## 📊 Phân tích Topics hiện tại

### Topics của bạn đang có:

#### 1. **OpenVINS Odometry**

```
/ov_msckf/odomimu          ← Odometry chính từ OpenVINS
/ov_msckf/poseimu          ← Pose data
/ov_msckf/pathimu          ← Path visualization
```

#### 2. **Gazebo X500 Sensors**

```
/gz/drone1/camera_depth/image_raw       ← Depth image từ camera
/gz/drone1/camera_depth/points          ← Point cloud từ depth
/gz/drone1/camera_left/color/image_raw  ← RGB image
/gz/drone1/camera_left/color/camera_info ← Camera parameters
/gz/drone1/sensors/imu                  ← IMU data
```

#### 3. **PX4 Control Topics**

```
/fmu/in/trajectory_setpoint    ← Input commands cho PX4
/fmu/in/offboard_control_mode  ← Offboard mode control
/fmu/in/vehicle_command        ← Vehicle commands (arm, mode)
/fmu/out/vehicle_odometry      ← PX4 position estimate
/fmu/out/vehicle_status        ← Vehicle status
```

#### 4. **EGO-Swarm Planning Output**

```
/drone_0_planning/pos_cmd      ← Position commands từ planner
/drone_0_planning/bspline      ← B-spline trajectory
/drone_0_visual_slam/odom      ← Odometry input cho planner
```

---

## ❌ Vấn đề trong `single_run_in_sim.launch.py`

### Dòng 18 - TYPO:
```python
odom_topic = LaunchConfiguration('odom_topic', default = 'ov_mscfkf/odomimu')
#                                                              ^^^^^^
#                                                              SAI! Phải là msckf
```

### Topics simulation không khớp với X500:

File launch này được thiết kế cho **simulator cũ** với topics:
```
pcl_render_node/depth        ← Simulator cũ
pcl_render_node/camera_pose  ← Simulator cũ
visual_slam/odom             ← Simulator cũ
```

Nhưng bạn có **X500 Gazebo + OpenVINS**:
```
/gz/drone1/camera_depth/image_raw  ← X500 thật
/ov_msckf/odomimu                  ← OpenVINS thật
```

---

## ✅ Giải pháp: 2 Options

### **Option 1: Topic Remapping trong Launch File** (Khuyến nghị)

Tạo các remapping trực tiếp trong launch file, không cần sửa code C++.

### **Option 2: Bridge Nodes**

Tạo các bridge nodes riêng biệt để convert topics.

---

## 🔧 Option 1: Fix Launch File với Remapping

### Bước 1: Sửa typo trong `single_run_in_sim.launch.py`

<function_calls>
<invoke name="replace_string_in_file">
<parameter name="filePath">/home/quangsang/ego_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py