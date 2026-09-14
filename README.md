# 🏎️ Autonomous Formula Student Ackermann Platform

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-8%20(Harmonic)-orange.svg)](https://gazebosim.org/)
[![Language](https://img.shields.io/badge/C%2B%2B-17%2F20-blue.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.12-yellow.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

An end-to-end autonomous racing stack developed for an **Ackermann-steering Formula Student vehicle** simulated in **Gazebo Sim 8 (Harmonic)** under **ROS 2 Jazzy Jalisco**.

The platform features an integrated perception pipeline combining 2D LiDAR clustering and RGB camera projection for cone detection and color classification, an algorithmic **Delaunay Triangulation** path planner, a curvature-adaptive speed profiler, a non-linear **Stanley steering controller**, and a 214-landmark **Extended Kalman Filter (EKF)** localization system.

---

## 🌟 Key Features

- **Multi-Modal Perception & Sensor Fusion**:
  - LiDAR-based Euclidean clustering for precise 3D cone candidate position extraction.
  - Monocular camera projection using calibrated pinhole geometry ($P = K [R \mid t]$).
  - HSV color space classification distinguishing Left boundary cones (Blue) from Right boundary cones (Yellow).
- **Delaunay Triangulation Track Planner**:
  - Automatic topological dual-graph extraction connecting opposing boundary cones.
  - Midpoint centerline extraction generating smooth, collision-free reference trajectories on unknown circuits without prior global maps.
- **Curvature-Adaptive Speed Profiler**:
  - Dynamically computes local path curvature ($\kappa = \frac{|x' y'' - y' x''|}{(x'^2 + y'^2)^{3/2}}$) and scales velocity targets ($1.2\text{ m/s} \le v \le 2.5\text{ m/s}$) to respect lateral acceleration limits ($a_{lat,max} \le 1.8\text{ m/s}^2$).
- **Stanley Path Tracking Controller**:
  - Front-axle reference tracking combining heading error and non-linear cross-track error:
    $$\delta(t) = \theta_e(t) + \arctan2\left(k \cdot e_{fa}(t), v_x(t) + k_{soft}\right) + k_d \cdot \dot{e}_{fa}(t)$$
- **Extended Kalman Filter (EKF) Localization**:
  - Fuses non-linear Ackermann kinematics, IMU yaw rate, and 214 LiDAR cone landmark observations with Mahalanobis distance gating ($\chi^2 \le 9.21$).
  - Sub-centimeter state covariance tracking and real-time drift elimination.
- **Multi-Controller Benchmarking Harness**:
  - C++ comparison framework implementing PID, Stanley, LQR, and Sliding Mode Control (SMC) on standardized test geometries.
- **Full Gazebo Sim 8 Simulation & RViz2 Setup**:
  - Custom SDF world (`track_with_cones.sdf`) containing 214 Formula Student cones with realistic friction and lighting.
  - Pre-configured RViz2 display profiles for real-time visualization of sensor clouds, triangulated paths, and Kalman states.

---

## 📐 System Architecture

```mermaid
flowchart TD
    subgraph SimLayer["Gazebo Sim 8 (Harmonic)"]
        GZ_WORLD["World Simulation\n(track_with_cones.sdf - 214 Cones)"]
        GZ_ROBOT["Ackermann Vehicle Model\n(URDF / Xacro)"]
        SENSORS["LiDAR (/scan) & Monocular Camera (/camera/image_raw)"]
        GZ_WORLD --> SENSORS
    end

    subgraph BridgeLayer["Hardware Abstraction (ros_gz_bridge & ros2_control)"]
        BRIDGE["ros_gz_bridge"]
        CTRL_MGR["controller_manager"]
        FWD_VEL["forward_velocity_controller"]
        FWD_POS["forward_position_controller"]
        SENSORS --> BRIDGE
        CTRL_MGR --> FWD_VEL
        CTRL_MGR --> FWD_POS
    end

    subgraph LocalizationLayer["State Estimation (EKF SLAM)"]
        EKF["ekf_localization_node.py\n(Ackermann Kinematics + 214-Cone Scan-Matching)"]
        BRIDGE -->|/scan & /odom| EKF
        EKF -->|/ekf/pose, /ekf/path| RVIZ["RViz2 Visualization"]
    end

    subgraph PerceptionPlanningLayer["Perception & Delaunay Planning"]
        PLANNER["cone_fusion_delaunay_planner.py\n1. LiDAR Euclidean Clustering\n2. Pinhole Projection (K [R|t])\n3. HSV Classification (Blue/Yellow)\n4. Delaunay Triangulation & Dual Graph\n5. Curvature Speed Profiler"]
        BRIDGE -->|/scan & /camera/image_raw| PLANNER
        BRIDGE -->|/model/.../odometry| PLANNER
    end

    subgraph ControlLayer["Control Execution"]
        STANLEY["Stanley Steering Controller"]
        PLANNER --> STANLEY
        STANLEY -->|/velocity (v_cmd)| VEH_CTRL["vehicle_controller (C++)\nAckermann Geometry Solver"]
        STANLEY -->|/steering_angle (delta)| VEH_CTRL
        VEH_CTRL -->|Wheel Steering Angles| FWD_POS
        VEH_CTRL -->|Wheel Drive Velocities| FWD_VEL
    end
```

---

## 📁 Repository Structure

```
gokart_ws/
├── docs/                                                 # Comprehensive technical documentation & reports
│   ├── Autonomous_Ackermann_Formula_Student_Master_Report.pdf   # 40+ page Master Thesis / Project Report (PDF)
│   ├── Autonomous_Ackermann_Formula_Student_Master_Report.docx  # Editable Word Master Document
│   └── full_project_master_report.md                            # Complete Markdown Project Reference
├── src/
│   ├── ackermann-path-following-controllers/             # Main autonomous package
│   │   └── src/gazebo_ackermann_steering_vehicle/
│   │       ├── config/                                  # Parameters, bridges, and RViz layouts
│   │       │   ├── ekf_tracking.rviz
│   │       │   ├── parameters.yaml
│   │       │   └── ros_gz_bridge.yaml
│   │       ├── launch/                                  # Master ROS 2 launch files
│   │       │   ├── vehicle.launch.py                    # Gazebo + Controllers + EKF
│   │       │   └── rviz.launch.py                       # RViz2 visualization
│   │       ├── model/                                   # Robot URDF/Xacro description
│   │       │   └── vehicle.xacro
│   │       ├── scripts/                                 # Autonomous Python nodes & tools
│   │       │   ├── cone_fusion_delaunay_planner.py      # Core Perception, Planner & Stanley Controller
│   │       │   ├── ekf_localization_node.py             # 214-Cone EKF Localization Node
│   │       │   ├── evaluate_ekf.py                      # Localization RMSE benchmark suite
│   │       │   └── circle_motion.py                     # Kinematic test utility
│   │       ├── src/                                     # High-performance C++ controllers & kinematics
│   │       │   ├── vehicle_controller.cpp               # Ackermann wheel angle & velocity solver
│   │       │   ├── stanley_controller.cpp               # C++ Stanley front-axle controller
│   │       │   ├── pid_controller.cpp                   # Lateral PID controller
│   │       │   └── sliding_mode_controller.cpp          # Non-linear SMC controller
│   │       └── worlds/
│   │           └── track_with_cones.sdf                 # 214-Cone Formula Student track world
│   └── ros2_controllers/                                # Hardware interface controller packages
│       ├── forward_command_controller/
│       └── joint_state_broadcaster/
├── .gitignore                                           # Ignores build/, install/, log/, and temp files
└── README.md
```

---

## 🚀 Quick Start Guide

### 1. Prerequisites & Dependencies

Ensure you are running **Ubuntu 24.04 LTS** (or compatible) with **ROS 2 Jazzy** and **Gazebo Sim 8 (Harmonic)** installed:

```bash
# Required ROS 2 and Gazebo packages
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-ros2-control \
                    ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster \
                    ros-jazzy-forward-command-controller python3-pip

# Scientific Python dependencies
pip3 install numpy scipy opencv-python matplotlib pandas pyyaml
```

### 2. Building the Workspace

```bash
cd gokart_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launching the Simulation

Open three terminals (sourcing `/opt/ros/jazzy/setup.bash` and `install/setup.bash` in each):

#### Terminal 1: Gazebo World, Vehicle & EKF
```bash
ros2 launch gazebo_ackermann_steering_vehicle vehicle.launch.py
```

#### Terminal 2: RViz2 Visualization
```bash
ros2 launch gazebo_ackermann_steering_vehicle rviz.launch.py
```

#### Terminal 3: Perception, Delaunay Planning & Stanley Controller
```bash
python3 install/gazebo_ackermann_steering_vehicle/lib/gazebo_ackermann_steering_vehicle/cone_fusion_delaunay_planner.py \
  --ros-args -p use_sim_time:=true \
             -p controller_type:=stanley \
             -p target_velocity:=1.8 \
             -p stanley_k:=2.2 \
             -p stanley_ks:=0.4 \
             -p stanley_kd:=0.08
```

---

## 📊 Live ROS 2 Topic Interface

| Topic | Type | Frequency | Description |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | ~10 Hz | 2D LiDAR range measurements |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | ~15 Hz | Monocular RGB camera stream |
| `/model/ackermann_steering_vehicle/odometry` | `nav_msgs/msg/Odometry` | ~20 Hz | Ground truth simulation odometry |
| `/ekf/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | ~20 Hz | Extended Kalman Filter state estimate ($x, y, \theta$) |
| `/planning/delaunay_path` | `nav_msgs/msg/Path` | ~5 Hz | Generated centerline waypoints |
| `/planning/debug_image` | `sensor_msgs/msg/Image` | ~5 Hz | Camera stream with projected 3D cones and color bounding boxes |
| `/steering_angle` | `std_msgs/msg/Float64` | ~20 Hz | Target Ackermann steering angle $\delta$ [rad] |
| `/velocity` | `std_msgs/msg/Float64` | ~20 Hz | Target longitudinal speed $v$ [m/s] |

---

## 📄 Documentation & Master Reports

Detailed mathematical derivations, state-space formulations, sensor fusion projections, and benchmarking analyses are documented in:
- **[Full Project Master Report (PDF)](docs/Autonomous_Ackermann_Formula_Student_Master_Report.pdf)**
- **[Master Document (DOCX)](docs/Autonomous_Ackermann_Formula_Student_Master_Report.docx)**
- **[Technical Architecture Reference (Markdown)](docs/full_project_master_report.md)**

---

## 📜 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
