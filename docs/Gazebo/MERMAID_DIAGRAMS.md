# Sơ đồ Mermaid - Tích hợp X500 + OpenVINS + EGO-Swarm

## 1. Sơ đồ Tổng quan Kiến trúc Hệ thống

```mermaid
graph TB
    subgraph Gazebo["🎮 GAZEBO SIMULATION"]
        X500["X500 Drone<br/>📡 IMU<br/>📷 Camera<br/>📏 Depth Cam<br/>🛰️ GPS"]
        PX4["PX4 SITL<br/>Flight Controller"]
        X500 -->|Sensor Data| PX4
    end

    subgraph OpenVINS["👁️ OPENVINS"]
        VIO["Visual-Inertial<br/>Odometry"]
        VIO_OUT["/ov_msckf/odometry_imu<br/>(nav_msgs/Odometry)"]
        VIO --> VIO_OUT
    end

    subgraph Bridge["🌉 BRIDGE NODES (Mới tạo)"]
        ODOM_CONV["odom_converter<br/>📊 Frame Convert<br/>⏱️ Time Sync"]
        CMD_BRIDGE["x500_cmd_bridge<br/>🎯 Command Convert<br/>📡 Offboard Mode"]
    end

    subgraph EgoSwarm["🚁 EGO-SWARM PLANNER"]
        subgraph Sensing["SENSING"]
            PCL["PCL Render"]
            DETECT["Drone Detector"]
        end
        
        subgraph Mapping["MAPPING"]
            GRIDMAP["Grid Map<br/>(Occupancy)"]
        end
        
        subgraph Planning["PLANNING"]
            FSM["ego_replan_fsm"]
            PATH["Path Search<br/>(A*, JPS)"]
            OPT["B-spline<br/>Optimization"]
            FSM --> PATH
            PATH --> OPT
        end
    end

    subgraph MapGen["🗺️ MAP GENERATOR"]
        MOCKA["Mockamap /<br/>Random Forest"]
    end

    %% Sensor Flow
    PX4 -->|/x500/camera/image<br/>/x500/imu| VIO

    %% Odometry Flow
    VIO_OUT --> ODOM_CONV
    ODOM_CONV -->|/drone_0_visual_slam/odom| Sensing
    ODOM_CONV -->|/drone_0_visual_slam/odom| Mapping
    ODOM_CONV -->|odom_world| Planning

    %% Map Flow
    MOCKA -->|/map_generator/global_cloud| PCL
    PCL -->|depth, point cloud| DETECT
    DETECT -->|clean_depth| GRIDMAP
    GRIDMAP -->|local_map| Planning

    %% Planning Flow
    OPT -->|/drone_0_planning/pos_cmd| CMD_BRIDGE
    CMD_BRIDGE -->|/fmu/in/trajectory_setpoint<br/>/fmu/in/offboard_control_mode| PX4

    %% Swarm Communication
    OPT -.->|/drone_X_planning/swarm_trajs<br/>(broadcast)| Planning

    style Gazebo fill:#e1f5ff
    style OpenVINS fill:#fff4e1
    style Bridge fill:#ffe1e1
    style EgoSwarm fill:#e1ffe1
    style MapGen fill:#f0e1ff
```

## 2. Luồng Dữ liệu Chi tiết

### 2.1 Odometry Flow

```mermaid
sequenceDiagram
    participant X500 as 🚁 X500 Drone
    participant PX4 as ⚙️ PX4 SITL
    participant OV as 👁️ OpenVINS
    participant OCONV as 🌉 odom_converter
    participant EGO as 🧠 EGO-Planner

    X500->>PX4: IMU + Camera Data
    PX4->>OV: /x500/imu<br/>/x500/camera/image_raw
    OV->>OV: Visual-Inertial<br/>Odometry Estimation
    OV->>OCONV: /ov_msckf/odometry_imu<br/>(nav_msgs/Odometry)
    
    Note over OCONV: Frame Convert (NED↔ENU)<br/>Timestamp Sync<br/>Topic Remap
    
    OCONV->>EGO: /drone_0_visual_slam/odom<br/>or odom_world
    EGO->>EGO: Planning + Mapping<br/>+ Collision Check
```

### 2.2 Control Flow

```mermaid
sequenceDiagram
    participant USER as 👤 User
    participant EGO as 🧠 EGO-Planner
    participant CBRIDGE as 🌉 x500_cmd_bridge
    participant PX4 as ⚙️ PX4 SITL
    participant X500 as 🚁 X500 Gazebo

    USER->>EGO: /goal<br/>(PoseStamped)
    EGO->>EGO: 1. Path Search (A*)
    EGO->>EGO: 2. B-spline Optimization
    EGO->>EGO: 3. Collision Check
    
    EGO->>CBRIDGE: /drone_0_planning/pos_cmd<br/>(PositionCommand)
    
    Note over CBRIDGE: Convert to PX4 format<br/>ENU → NED<br/>Enable Offboard Mode
    
    CBRIDGE->>PX4: /fmu/in/trajectory_setpoint<br/>/fmu/in/offboard_control_mode
    PX4->>X500: Motor Commands
    X500->>X500: Execute Trajectory
```

### 2.3 Perception & Mapping Flow

```mermaid
graph LR
    subgraph MapSource["Map Source"]
        MOCK["Mockamap/<br/>Random Forest"]
    end

    subgraph Render["Depth Rendering"]
        PCL["PCL Render Node"]
        ODOM1["Odometry"]
    end

    subgraph Detection["Drone Detection"]
        DEPTH["Depth Image"]
        OTHERS["Others Odom"]
        DETECTOR["Drone Detector"]
    end

    subgraph Mapping["Grid Mapping"]
        CLEAN["Clean Depth"]
        GRID["Grid Map"]
    end

    subgraph Planning["Planning"]
        PLANNER["EGO-Planner"]
    end

    MOCK -->|/map_generator/global_cloud| PCL
    ODOM1 -->|Position| PCL
    PCL -->|Rendered Depth| DEPTH
    DEPTH --> DETECTOR
    OTHERS -->|/others_odom| DETECTOR
    DETECTOR -->|Remove Other Drones| CLEAN
    CLEAN --> GRID
    GRID -->|Occupancy Map| PLANNER

    style MapSource fill:#f0e1ff
    style Render fill:#e1f5ff
    style Detection fill:#ffe1e1
    style Mapping fill:#fff4e1
    style Planning fill:#e1ffe1
```

## 3. Swarm Communication (Multi-Drone)

```mermaid
graph TB
    subgraph Drone0["🚁 Drone 0"]
        P0["Planner 0"]
        O0["Odom 0"]
    end

    subgraph Drone1["🚁 Drone 1"]
        P1["Planner 1"]
        O1["Odom 1"]
    end

    subgraph Drone2["🚁 Drone 2"]
        P2["Planner 2"]
        O2["Odom 2"]
    end

    subgraph Network["📡 Broadcast Network"]
        TRAJ0["/drone_0_planning/swarm_trajs"]
        TRAJ1["/drone_1_planning/swarm_trajs"]
        TRAJ2["/drone_2_planning/swarm_trajs"]
    end

    %% Trajectory Broadcasting
    P0 -->|Publish| TRAJ0
    P1 -->|Publish| TRAJ1
    P2 -->|Publish| TRAJ2

    %% Cross-subscription for collision avoidance
    TRAJ1 -.->|Subscribe| P0
    TRAJ2 -.->|Subscribe| P0
    TRAJ0 -.->|Subscribe| P1
    TRAJ2 -.->|Subscribe| P1
    TRAJ0 -.->|Subscribe| P2
    TRAJ1 -.->|Subscribe| P2

    %% Odometry for drone detection
    O0 -->|/others_odom| P1
    O0 -->|/others_odom| P2
    O1 -->|/others_odom| P0
    O1 -->|/others_odom| P2
    O2 -->|/others_odom| P0
    O2 -->|/others_odom| P1

    style Drone0 fill:#e1f5ff
    style Drone1 fill:#ffe1e1
    style Drone2 fill:#e1ffe1
    style Network fill:#fff4e1
```

## 4. Bridge Nodes Chi tiết

### 4.1 odom_converter

```mermaid
graph LR
    subgraph Input["INPUT"]
        VINS["/ov_msckf/odometry_imu<br/>nav_msgs/Odometry<br/>(ENU frame)"]
    end

    subgraph Process["PROCESSING"]
        CHECK{"Frame<br/>Check"}
        CONVERT["Frame Convert<br/>NED ↔ ENU"]
        SYNC["Timestamp<br/>Sync"]
        REMAP["Topic<br/>Remap"]
    end

    subgraph Output["OUTPUT"]
        OUT["/drone_0_visual_slam/odom<br/>or odom_world<br/>(EGO-Swarm format)"]
    end

    VINS --> CHECK
    CHECK -->|Need Convert| CONVERT
    CHECK -->|Already ENU| SYNC
    CONVERT --> SYNC
    SYNC --> REMAP
    REMAP --> OUT

    style Input fill:#e1f5ff
    style Process fill:#fff4e1
    style Output fill:#e1ffe1
```

### 4.2 x500_cmd_bridge

```mermaid
graph LR
    subgraph Input["INPUT"]
        CMD["/drone_X_planning/pos_cmd<br/>quadrotor_msgs/PositionCommand<br/>(ENU frame)"]
    end

    subgraph Process["PROCESSING"]
        EXTRACT["Extract<br/>Position/Velocity/Accel"]
        CONVERT["Frame Convert<br/>ENU → NED"]
        FORMAT["Format to<br/>TrajectorySetpoint"]
        MODE["Generate<br/>OffboardControlMode"]
    end

    subgraph Output["OUTPUT"]
        TRAJ["/fmu/in/trajectory_setpoint<br/>(PX4 format, NED)"]
        OFF["/fmu/in/offboard_control_mode<br/>(Enable offboard)"]
    end

    CMD --> EXTRACT
    EXTRACT --> CONVERT
    CONVERT --> FORMAT
    CONVERT --> MODE
    FORMAT --> TRAJ
    MODE --> OFF

    style Input fill:#e1f5ff
    style Process fill:#fff4e1
    style Output fill:#ffe1e1
```

## 5. Topic Mapping Diagram

```mermaid
graph TB
    subgraph Sources["Data Sources"]
        OV_TOPIC["/ov_msckf/odometry_imu"]
        EGO_CMD["/drone_X_planning/pos_cmd"]
        X500_DEPTH["/x500/camera/depth"]
        MAP_TOPIC["/map_generator/global_cloud"]
    end

    subgraph Bridges["Bridge Layer"]
        OCONV["odom_converter"]
        CBRIDGE["cmd_bridge"]
    end

    subgraph EgoTopics["EGO-Swarm Expected Topics"]
        ODOM_WORLD["odom_world<br/>/drone_X_visual_slam/odom"]
        DEPTH["depth"]
        GLOBAL_MAP["global_map"]
    end

    subgraph PX4Topics["PX4 Topics"]
        TRAJ_SET["/fmu/in/trajectory_setpoint"]
        OFF_MODE["/fmu/in/offboard_control_mode"]
    end

    OV_TOPIC -->|Convert| OCONV
    OCONV --> ODOM_WORLD
    
    EGO_CMD -->|Convert| CBRIDGE
    CBRIDGE --> TRAJ_SET
    CBRIDGE --> OFF_MODE

    X500_DEPTH -.->|Direct or<br/>pcl_render| DEPTH
    MAP_TOPIC -.->|Keep same| GLOBAL_MAP

    style Sources fill:#e1f5ff
    style Bridges fill:#ffe1e1
    style EgoTopics fill:#e1ffe1
    style PX4Topics fill:#fff4e1
```

## 6. State Machine - Planning FSM

```mermaid
stateDiagram-v2
    [*] --> INIT
    
    INIT --> WAIT_TARGET: Have Odom
    WAIT_TARGET --> GEN_NEW_TRAJ: Receive Goal
    
    GEN_NEW_TRAJ --> REPLAN_TRAJ: Planning Success
    GEN_NEW_TRAJ --> WAIT_TARGET: Planning Failed
    
    REPLAN_TRAJ --> EXEC_TRAJ: Trajectory Valid
    REPLAN_TRAJ --> REPLAN_TRAJ: Need Replan
    
    EXEC_TRAJ --> REPLAN_TRAJ: Collision Detected
    EXEC_TRAJ --> REPLAN_TRAJ: Trajectory Timeout
    EXEC_TRAJ --> WAIT_TARGET: Goal Reached
    
    state GEN_NEW_TRAJ {
        [*] --> PathSearch
        PathSearch --> BSplineOpt
        BSplineOpt --> CollisionCheck
        CollisionCheck --> [*]
    }
    
    state REPLAN_TRAJ {
        [*] --> CheckSwarmTrajs
        CheckSwarmTrajs --> ReOptimize
        ReOptimize --> ValidateTrajectory
        ValidateTrajectory --> [*]
    }
```

## 7. Deployment Comparison

```mermaid
graph TB
    subgraph Before["BEFORE: Simulation Only"]
        SIM_OLD["so3_quadrotor_simulator<br/>Perfect Odometry<br/>No noise"]
        POS2ODOM["poscmd_2_odom<br/>Direct conversion"]
        EGO_OLD["EGO-Planner"]
        
        EGO_OLD -->|pos_cmd| POS2ODOM
        POS2ODOM -->|perfect odom| EGO_OLD
        SIM_OLD -->|perfect odom| EGO_OLD
        
        style SIM_OLD fill:#ffcccc
        style POS2ODOM fill:#ffcccc
    end

    subgraph After["AFTER: Realistic Simulation"]
        X500_NEW["X500 Gazebo<br/>+ PX4 SITL<br/>Realistic dynamics"]
        OV_NEW["OpenVINS<br/>Real VIO<br/>Noise + Drift"]
        BRIDGE_NEW["Bridge Nodes<br/>Topic conversion"]
        EGO_NEW["EGO-Planner<br/>(No changes)"]
        
        X500_NEW -->|sensors| OV_NEW
        OV_NEW -->|vio odom| BRIDGE_NEW
        BRIDGE_NEW -->|converted odom| EGO_NEW
        EGO_NEW -->|pos_cmd| BRIDGE_NEW
        BRIDGE_NEW -->|px4 cmd| X500_NEW
        
        style X500_NEW fill:#ccffcc
        style OV_NEW fill:#ccffcc
        style BRIDGE_NEW fill:#ccffcc
    end

    Before -.->|Migration| After
```

## 8. Data Flow Timeline

```mermaid
gantt
    title Typical Planning Cycle (100ms)
    dateFormat X
    axisFormat %L ms

    section Sensing
    Camera/IMU capture    :0, 5
    OpenVINS processing   :5, 25
    Odom conversion       :25, 28
    
    section Mapping
    Depth rendering       :28, 38
    Drone detection       :38, 42
    Grid map update       :42, 50
    
    section Planning
    Collision check       :50, 55
    Path search           :55, 65
    B-spline optimize     :65, 85
    
    section Control
    Command conversion    :85, 88
    PX4 processing        :88, 95
    Motor actuation       :95, 100
```

## Cách sử dụng

1. **Copy code Mermaid** vào GitHub README hoặc documentation
2. **Render online** tại: https://mermaid.live/
3. **Export** sang PNG/SVG để dùng trong presentations
4. **Edit** trực tiếp trong VS Code với extension: Markdown Preview Mermaid Support

## Notes

- Màu sắc:
  - 🔵 Xanh dương (#e1f5ff): Gazebo/Simulation
  - 🟡 Vàng (#fff4e1): OpenVINS/VIO
  - 🔴 Đỏ nhạt (#ffe1e1): Bridge nodes
  - 🟢 Xanh lá (#e1ffe1): EGO-Swarm
  - 🟣 Tím (#f0e1ff): Map Generator

- Icons:
  - 🚁 Drone
  - 👁️ Vision/VIO
  - 🌉 Bridge
  - 🧠 Brain/Planning
  - 📡 Communication
  - 🗺️ Map
  - 🎮 Simulation
