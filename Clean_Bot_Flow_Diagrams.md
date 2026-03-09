# 🤖 Clean Bot - System Flow Diagrams

Complete visual documentation of the autonomous cleaning robot project.

---

## 1. Launch File Hierarchy

Shows which launch files include which other files and what nodes they start.

```mermaid
flowchart TD
    subgraph ENTRY["🚀 ENTRY POINT"]
        A["cleaning_mission.launch.py<br/><i>clean_bot_mission/launch/</i>"]
    end
    
    subgraph HARDWARE["🔧 HARDWARE LAYER"]
        B["robot_bringup.launch.py<br/><i>clean_bot_hardware/launch/</i>"]
    end
    
    subgraph EXTERNAL["📦 EXTERNAL PACKAGES"]
        C["navigation_launch.py<br/><i>nav2_bringup/launch/</i>"]
    end
    
    subgraph NODES["🖥️ LAUNCHED NODES"]
        N1["robot_state_publisher"]
        N2["arduino_driver"]
        N3["sllidar_node"]
        N4["imu_publisher"]
        N5["imu_filter_madgwick"]
        N6["low_obstacle_detector"]
        N7["emergency_stop"]
        N8["scan_throttler"]
        N9["rf2o_laser_odometry"]
        N10["slam_toolbox"]
        N11["Nav2 Stack"]
        N12["full_mission_controller"]
    end
    
    A -->|includes| B
    A -->|launches| N12
    B -->|includes| C
    B -->|launches| N1
    B -->|launches| N2
    B -->|launches| N3
    B -->|launches| N4
    B -->|launches| N5
    B -->|launches| N6
    B -->|launches| N7
    B -->|launches| N8
    B -->|launches| N9
    B -->|launches| N10
    C -->|launches| N11
```

---

## 2. Sensor Input Layer

How physical sensors connect to ROS2 driver nodes.

```mermaid
flowchart TB
    subgraph SENSORS["🔌 PHYSICAL SENSORS"]
        HW_LIDAR["RPLidar A1"]
        HW_IMU["Grove IMU 9DOF"]
        HW_ULTRA["Ultrasonic HC-SR04"]
        HW_MOTORS["DC Motors"]
    end

    subgraph DRIVERS["📟 DRIVER NODES"]
        sllidar["sllidar_node<br/><i>sllidar_ros2</i>"]
        imu_pub["imu_publisher<br/><i>imu_publisher_node.py</i>"]
        arduino["arduino_driver<br/><i>arduino_driver.py</i>"]
    end
    
    subgraph TOPICS_RAW["📡 RAW TOPICS"]
        T_SCAN["/scan"]
        T_IMU_RAW["/imu/data_raw"]
        T_IMU_MAG["/imu/mag"]
        T_ULTRA["/ultrasonic_range"]
    end
    
    HW_LIDAR --> sllidar
    HW_IMU --> imu_pub
    HW_ULTRA --> arduino
    
    sllidar --> T_SCAN
    imu_pub --> T_IMU_RAW
    imu_pub --> T_IMU_MAG
    arduino --> T_ULTRA
```

---

## 3. Sensor Processing Pipeline

How raw sensor data is processed into usable information.

```mermaid
flowchart TB
    subgraph INPUT["📥 INPUT TOPICS"]
        T_SCAN["/scan"]
        T_IMU_RAW["/imu/data_raw"]
        T_ULTRA["/ultrasonic_range"]
    end
    
    subgraph PROCESSING["⚙️ PROCESSING NODES"]
        throttle["scan_throttler<br/><i>topic_tools/throttle</i>"]
        madgwick["imu_filter_madgwick<br/><i>imu_filter_madgwick</i>"]
        obstacle["low_obstacle_detector<br/><i>low_obstacle_detector.py</i>"]
        rf2o["rf2o_laser_odometry<br/><i>rf2o_laser_odometry</i>"]
        slam["slam_toolbox<br/><i>async_slam_toolbox_node</i>"]
    end
    
    subgraph OUTPUT["📤 OUTPUT TOPICS"]
        T_SCAN_T["/scan_throttled"]
        T_IMU["/imu/data"]
        T_LOW["/low_obstacles"]
        T_ODOM["/odom"]
        T_MAP["/map"]
    end
    
    subgraph TF["🔗 TF TRANSFORMS"]
        TF_ODOM["odom → base_link"]
        TF_MAP["map → odom"]
    end
    
    T_SCAN --> throttle
    T_SCAN --> slam
    throttle --> T_SCAN_T
    T_SCAN_T --> rf2o
    
    T_IMU_RAW --> madgwick
    madgwick --> T_IMU
    
    T_ULTRA --> obstacle
    obstacle --> T_LOW
    
    rf2o --> T_ODOM
    rf2o --> TF_ODOM
    
    slam --> T_MAP
    slam --> TF_MAP
```

---

## 4. Navigation to Motor Output Flow

How perception feeds into navigation and ultimately controls the motors.

```mermaid
flowchart TB
    subgraph PERCEPTION["👁️ PERCEPTION INPUT"]
        T_SCAN["/scan"]
        T_MAP["/map"]
        T_LOW["/low_obstacles"]
        T_ODOM["/odom"]
    end
    
    subgraph NAV2["🧭 NAV2 NAVIGATION STACK"]
        costmap_g["Global Costmap"]
        costmap_l["Local Costmap"]
        planner["Global Planner<br/><i>NavFn/Smac</i>"]
        controller["Local Controller<br/><i>DWB/MPPI</i>"]
        bt["BT Navigator"]
    end
    
    subgraph SAFETY["🛡️ SAFETY LAYER"]
        estop["emergency_stop<br/><i>emergency_stop.py</i>"]
    end
    
    subgraph TOPICS_VEL["📡 VELOCITY TOPICS"]
        T_CMD_NAV["/cmd_vel_nav"]
        T_CMD["/cmd_vel"]
        T_ULTRA["/ultrasonic_range"]
    end
    
    subgraph OUTPUT["🔌 MOTOR OUTPUT"]
        arduino["arduino_driver<br/><i>arduino_driver.py</i>"]
        motors["DC Motors"]
    end
    
    T_SCAN --> costmap_l
    T_MAP --> costmap_g
    T_LOW --> costmap_l
    T_ODOM --> controller
    
    costmap_g --> planner
    costmap_l --> controller
    planner --> bt
    bt --> controller
    controller --> T_CMD_NAV
    
    T_CMD_NAV --> estop
    T_ULTRA --> estop
    estop --> T_CMD
    
    T_CMD --> arduino
    arduino --> motors
```

---

## 5. Mission State Machine

The state transitions in `full_mission.py`.

```mermaid
stateDiagram-v2
    [*] --> WAITING_FOR_SCAN: System Start
    
    WAITING_FOR_SCAN --> EXPLORING: start_scan
    EXPLORING --> WAITING_FOR_CLEAN: stop_scan / exploration_complete
    EXPLORING --> EXPLORING: exploring...
    
    WAITING_FOR_CLEAN --> COVERAGE: start_clean
    WAITING_FOR_CLEAN --> EXPLORING: start_scan
    
    COVERAGE --> WAITING_FOR_CLEAN: stop_clean
    COVERAGE --> RETURNING: coverage_complete
    COVERAGE --> COVERAGE: cleaning...
    
    RETURNING --> COMPLETE: reached_home
    COMPLETE --> WAITING_FOR_SCAN: reset
    
    WAITING_FOR_SCAN --> PAUSED: pause
    EXPLORING --> PAUSED: pause
    COVERAGE --> PAUSED: pause
    PAUSED --> WAITING_FOR_SCAN: resume
    PAUSED --> EXPLORING: resume
    PAUSED --> COVERAGE: resume
```

**State Descriptions:**
| State | Description | Active Module |
|-------|-------------|---------------|
| WAITING_FOR_SCAN | Idle, waiting for start command | None |
| EXPLORING | Mapping the room | `frontier_explorer.py` |
| WAITING_FOR_CLEAN | Map complete, waiting for clean command | None |
| COVERAGE | Executing cleaning path | `adaptive_coverage.py` |
| RETURNING | Going back to start position | Nav2 |
| COMPLETE | Mission finished | None |

---

## 6. Mission Control Communication

How the mission controller communicates with other components.

```mermaid
flowchart LR
    subgraph EXTERNAL["🌐 EXTERNAL CONTROL"]
        telegram["Telegram Bot<br/><i>telegram_bridge.py</i><br/>(runs on PC)"]
    end
    
    subgraph TOPICS_CMD["📥 COMMAND TOPICS"]
        T_MISSION_CMD["/mission_command"]
        T_EXPL_CTRL["/exploration_control"]
        T_COV_CTRL["/coverage_control"]
        T_ARDUINO_CMD["/arduino_command"]
    end
    
    subgraph MISSION["🎯 MISSION CONTROL"]
        full_mission["full_mission_controller<br/><i>full_mission.py</i>"]
    end
    
    subgraph MODULES["📦 MISSION MODULES"]
        explorer["FrontierExplorer<br/><i>frontier_explorer.py</i>"]
        coverage["AdaptiveCoveragePlanner<br/><i>adaptive_coverage.py</i>"]
    end
    
    subgraph HARDWARE["🔧 HARDWARE"]
        arduino["arduino_driver<br/><i>arduino_driver.py</i>"]
    end
    
    subgraph TOPICS_STATUS["📤 STATUS TOPICS"]
        T_MISSION_STATE["/mission_state"]
        T_EXPL_COMPLETE["/exploration_complete"]
        T_COV_COMPLETE["/coverage_complete"]
    end
    
    telegram --> T_MISSION_CMD
    T_MISSION_CMD --> full_mission
    
    full_mission --> T_EXPL_CTRL
    full_mission --> T_COV_CTRL
    full_mission --> T_ARDUINO_CMD
    full_mission --> T_MISSION_STATE
    
    T_EXPL_CTRL --> explorer
    T_COV_CTRL --> coverage
    T_ARDUINO_CMD --> arduino
    
    explorer --> T_EXPL_COMPLETE
    coverage --> T_COV_COMPLETE
    
    T_EXPL_COMPLETE --> full_mission
    T_COV_COMPLETE --> full_mission
    
    T_MISSION_STATE --> telegram
```

---

## 7. File Dependencies

Which files import/include which other files.

```mermaid
flowchart TD
    subgraph PKG_DESC["clean_bot_description"]
        robot_xacro["robot.urdf.xacro"]
        robot_core["robot_core.xacro"]
        lidar_xacro["lidar.xacro"]
        imu_xacro["imu.xacro"]
        ultra_xacro["ultrasonic.xacro"]
        inertial["inertial_macros.xacro"]
        gazebo_ctrl["gazebo_control.xacro"]
    end
    
    subgraph PKG_HW["clean_bot_hardware"]
        arduino_py["arduino_driver.py"]
        imu_pub_py["imu_publisher_node.py"]
        simple_imu["simple_imu_driver.py"]
        low_obs_py["low_obstacle_detector.py"]
        estop_py["emergency_stop.py"]
        bringup_launch["robot_bringup.launch.py"]
        slam_yaml["mapper_params_online_async.yaml"]
        nav2_yaml["nav2_params.yaml"]
    end
    
    subgraph PKG_MISSION["clean_bot_mission"]
        full_py["full_mission.py"]
        frontier_py["frontier_explorer.py"]
        adaptive_py["adaptive_coverage.py"]
        simple_cov["simple_coverage.py"]
        mission_launch["cleaning_mission.launch.py"]
        telegram_py["telegram_bridge.py"]
    end
    
    robot_xacro --> robot_core
    robot_xacro --> lidar_xacro
    robot_xacro --> imu_xacro
    robot_xacro --> ultra_xacro
    robot_xacro --> gazebo_ctrl
    robot_core --> inertial
    
    imu_pub_py --> simple_imu
    full_py --> frontier_py
    full_py --> adaptive_py
    
    mission_launch --> bringup_launch
    bringup_launch --> robot_xacro
    bringup_launch --> slam_yaml
    bringup_launch --> nav2_yaml
    
    telegram_py -.->|ROS2 topics| full_py
```

---

## 8. Complete System Architecture

The full end-to-end data flow.

```mermaid
flowchart TB
    subgraph HW["🔌 HARDWARE"]
        LIDAR["RPLidar A1"]
        IMU["Grove IMU"]
        ULTRA["Ultrasonic"]
        MOTORS["DC Motors"]
    end
    
    subgraph URDF["📐 ROBOT DESCRIPTION"]
        xacro["robot.urdf.xacro"]
        core["robot_core.xacro"]
        lidar_x["lidar.xacro"]
        imu_x["imu.xacro"]
        ultra_x["ultrasonic.xacro"]
    end
    
    subgraph CONFIG["⚙️ CONFIG FILES"]
        slam_cfg["mapper_params_online_async.yaml"]
        nav2_cfg["nav2_params.yaml"]
    end
    
    subgraph DRIVERS["📟 DRIVERS"]
        sllidar["sllidar_node"]
        imu_pub["imu_publisher_node.py"]
        arduino["arduino_driver.py"]
        rsp["robot_state_publisher"]
    end
    
    subgraph PROC["⚙️ PROCESSING"]
        throttle["throttle"]
        madgwick["madgwick"]
        low_obs["low_obstacle_detector.py"]
        rf2o["rf2o"]
        slam["slam_toolbox"]
    end
    
    subgraph SAFETY["🛡️ SAFETY"]
        estop["emergency_stop.py"]
    end
    
    subgraph NAV["🧭 NAV2"]
        nav2["Nav2 Stack"]
    end
    
    subgraph MISSION["🎯 MISSION"]
        full["full_mission.py"]
        frontier["frontier_explorer.py"]
        adaptive["adaptive_coverage.py"]
    end
    
    subgraph EXT["🌐 EXTERNAL"]
        telegram["telegram_bridge.py"]
    end
    
    LIDAR --> sllidar
    IMU --> imu_pub
    ULTRA --> arduino
    
    xacro --> core
    xacro --> lidar_x
    xacro --> imu_x
    xacro --> ultra_x
    xacro --> rsp
    
    slam_cfg --> slam
    nav2_cfg --> nav2
    
    sllidar -->|/scan| throttle
    sllidar -->|/scan| slam
    throttle -->|/scan_throttled| rf2o
    imu_pub -->|/imu/data_raw| madgwick
    arduino -->|/ultrasonic_range| low_obs
    arduino -->|/ultrasonic_range| estop
    
    rf2o -->|/odom + TF| nav2
    slam -->|/map + TF| nav2
    madgwick -->|/imu/data| nav2
    low_obs -->|/low_obstacles| nav2
    
    nav2 -->|/cmd_vel_nav| estop
    estop -->|/cmd_vel| arduino
    arduino --> MOTORS
    
    telegram -->|/mission_command| full
    full -->|control| frontier
    full -->|control| adaptive
    frontier -->|/navigate_to_pose| nav2
    adaptive -->|/navigate_to_pose| nav2
    full -->|/mission_state| telegram
```

---

## 9. Complete ROS Topic Map

All topics with their publishers and subscribers.

```mermaid
flowchart LR
    subgraph PUB["📤 PUBLISHERS"]
        sllidar["sllidar_node"]
        imu_pub["imu_publisher"]
        arduino["arduino_driver"]
        throttle["throttle"]
        madgwick["madgwick"]
        low_obs["low_obstacle_detector"]
        rf2o["rf2o"]
        slam["slam_toolbox"]
        nav2["Nav2"]
        estop["emergency_stop"]
        full["full_mission"]
    end
    
    subgraph TOPICS["📡 ROS TOPICS"]
        scan["/scan"]
        scan_t["/scan_throttled"]
        imu_raw["/imu/data_raw"]
        imu_mag["/imu/mag"]
        imu_data["/imu/data"]
        ultra["/ultrasonic_range"]
        low_cloud["/low_obstacles"]
        odom["/odom"]
        map["/map"]
        cmd_nav["/cmd_vel_nav"]
        cmd["/cmd_vel"]
        mission_cmd["/mission_command"]
        mission_state["/mission_state"]
        goal["/navigate_to_pose"]
    end
    
    subgraph SUB["📥 SUBSCRIBERS"]
        throttle2["throttle"]
        madgwick2["madgwick"]
        low_obs2["low_obstacle_detector"]
        rf2o2["rf2o"]
        slam2["slam_toolbox"]
        nav2_2["Nav2"]
        estop2["emergency_stop"]
        arduino2["arduino_driver"]
        full2["full_mission"]
    end
    
    sllidar --> scan
    arduino --> ultra
    imu_pub --> imu_raw
    imu_pub --> imu_mag
    throttle --> scan_t
    madgwick --> imu_data
    low_obs --> low_cloud
    rf2o --> odom
    slam --> map
    nav2 --> cmd_nav
    estop --> cmd
    full --> mission_state
    
    scan --> throttle2
    scan --> slam2
    scan_t --> rf2o2
    imu_raw --> madgwick2
    ultra --> low_obs2
    ultra --> estop2
    odom --> nav2_2
    map --> nav2_2
    low_cloud --> nav2_2
    cmd_nav --> estop2
    cmd --> arduino2
    mission_cmd --> full2
    goal --> nav2_2
```

---

## Topic Reference Table

| Topic | Message Type | Publisher | Subscriber(s) |
|-------|-------------|-----------|---------------|
| `/scan` | sensor_msgs/LaserScan | sllidar_node | throttle, slam_toolbox, Nav2 |
| `/scan_throttled` | sensor_msgs/LaserScan | throttle | rf2o |
| `/imu/data_raw` | sensor_msgs/Imu | imu_publisher | madgwick |
| `/imu/mag` | sensor_msgs/MagneticField | imu_publisher | madgwick |
| `/imu/data` | sensor_msgs/Imu | madgwick | Nav2 |
| `/ultrasonic_range` | sensor_msgs/Range | arduino_driver | low_obstacle_detector, emergency_stop |
| `/low_obstacles` | sensor_msgs/PointCloud2 | low_obstacle_detector | Nav2 costmap |
| `/odom` | nav_msgs/Odometry | rf2o | Nav2 |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox | Nav2, full_mission |
| `/cmd_vel_nav` | geometry_msgs/Twist | Nav2 | emergency_stop |
| `/cmd_vel` | geometry_msgs/Twist | emergency_stop, full_mission | arduino_driver |
| `/mission_command` | std_msgs/String | telegram_bridge | full_mission |
| `/mission_state` | std_msgs/String | full_mission | telegram_bridge |
| `/navigate_to_pose` | nav2_msgs/action | frontier_explorer, adaptive_coverage | Nav2 |

---

## TF Tree

```
map
 └── odom (published by SLAM Toolbox)
      └── base_link (published by rf2o_laser_odometry)
           ├── chassis
           │    ├── laser_frame (LiDAR)
           │    ├── imu_link (IMU)
           │    └── ultrasonic_link (Ultrasonic)
           ├── left_wheel
           ├── right_wheel
           └── caster_wheel
```

---

*Generated: March 2026*
