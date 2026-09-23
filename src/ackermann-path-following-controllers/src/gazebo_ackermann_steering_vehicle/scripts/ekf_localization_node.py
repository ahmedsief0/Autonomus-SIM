#!/usr/bin/env python3
"""
Extended Kalman Filter (EKF) Localization Node for Ackermann Steering Vehicle
Fuses:
  - Motion Model: Ackermann kinematics / dead reckoning with velocity and IMU/steering
  - Exteroceptive Measurement: 2D LiDAR scans clustered into cone landmarks and matched to track map
  - Interoceptive Measurement: IMU angular velocity and wheel odometry

State vector:
  x = [x, y, theta, v]^T
    x, y: 2D Cartesian position in map frame [m]
    theta: vehicle heading / yaw [rad]
    v: longitudinal forward velocity [m/s]
"""

import math
import os
import xml.etree.ElementTree as ET
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Point
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from geometry_msgs.msg import TransformStamped
from ament_index_python.packages import get_package_share_directory


def normalize_angle(angle: float) -> float:
    """Normalize an angle to (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle to geometry_msgs Quaternion (around Z axis)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    """Extract yaw angle from geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')

        # Parameters
        self.declare_parameter('wheelbase', 0.485) # 485 mm = 0.485 m
        self.declare_parameter('lidar_offset_x', 0.0)
        self.declare_parameter('lidar_range_std', 0.05)
        self.declare_parameter('lidar_angle_std', 0.02)
        self.declare_parameter('data_association_gate', 0.8)
        self.declare_parameter('min_cluster_pts', 1)
        self.declare_parameter('cluster_max_dist', 0.25)
        self.declare_parameter('publish_tf', True)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.lidar_offset_x = self.get_parameter('lidar_offset_x').value
        self.lidar_r_std = self.get_parameter('lidar_range_std').value
        self.lidar_phi_std = self.get_parameter('lidar_angle_std').value
        self.da_gate = self.get_parameter('data_association_gate').value
        self.min_cluster_pts = self.get_parameter('min_cluster_pts').value
        self.cluster_max_dist = self.get_parameter('cluster_max_dist').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Load Track Cone Map
        self.map_cones = self.load_map_cones()
        self.get_logger().info(f"Loaded {len(self.map_cones)} cone landmarks from world.")

        # EKF State: x = [x, y, theta, v]^T
        self.x = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        # State Covariance: P
        self.P = np.diag([0.01, 0.01, 0.005, 0.01])

        # Process Noise Covariance Q (spectral density)
        self.q_x = 0.04
        self.q_y = 0.04
        self.q_theta = 0.02
        self.q_v = 0.10

        # Measurement Covariance for LiDAR landmark [r, phi]
        self.R_lidar = np.diag([self.lidar_r_std**2, self.lidar_phi_std**2])

        # Measurement Covariance for Odometry velocity
        self.R_v = 0.02**2

        # Control / Odometry inputs
        self.v_meas = 0.0
        self.delta_meas = 0.0
        self.omega_imu = 0.0
        self.has_imu = False
        self.last_time = None

        # Dead Reckoning state (for benchmark / drift comparison)
        self.x_dr = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Ground Truth state (from Gazebo)
        self.x_gt = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.has_gt = False

        # Paths for visualization
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'map'
        self.gt_path = Path()
        self.gt_path.header.frame_id = 'map'
        self.dr_path = Path()
        self.dr_path.header.frame_id = 'map'

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, sensor_qos)
        self.sub_odom = self.create_subscription(Odometry, '/model/ackermann_steering_vehicle/odometry', self.odom_callback, reliable_qos)
        self.sub_vel = self.create_subscription(Float64, '/velocity', self.vel_cmd_callback, 10)
        self.sub_steer = self.create_subscription(Float64, '/steering_angle', self.steer_cmd_callback, 10)

        # Publishers
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/ekf/pose', 10)
        self.pub_odom = self.create_publisher(Odometry, '/ekf/odometry', 10)
        self.pub_path_ekf = self.create_publisher(Path, '/ekf/path', 10)
        self.pub_path_gt = self.create_publisher(Path, '/ekf/ground_truth_path', 10)
        self.pub_path_dr = self.create_publisher(Path, '/ekf/dead_reckoning_path', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/ekf/markers', 10)
        self.pub_track_cones = self.create_publisher(MarkerArray, '/ekf/track_cones', 10)

        # TF Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # High-frequency prediction timer (50 Hz)
        self.timer = self.create_timer(0.02, self.timer_predict_callback)
        # Periodic track cones marker publisher (1 Hz)
        self.timer_track_cones = self.create_timer(1.0, self.publish_track_cones)

        self.get_logger().info("EKF Localization Node successfully initialized.")

    def load_map_cones(self):
        """Parse cone landmarks from track_with_cones.sdf."""
        cones = []
        try:
            pkg_path = get_package_share_directory('gazebo_ackermann_steering_vehicle')
            world_path = os.path.join(pkg_path, 'worlds', 'track_with_cones.sdf')
            if not os.path.exists(world_path):
                # Fallback to source workspace
                world_path = '/home/ahmed/gokart_ws/src/ackermann-path-following-controllers/src/gazebo_ackermann_steering_vehicle/worlds/track_with_cones.sdf'
            
            tree = ET.parse(world_path)
            root = tree.getroot()
            world = root.find('world')
            if world is not None:
                for model in world.findall('model'):
                    name = model.get('name', '')
                    if 'cone' in name:
                        pose_elem = model.find('pose')
                        if pose_elem is not None and pose_elem.text:
                            parts = [float(v) for v in pose_elem.text.split()]
                            color = 'blue' if 'blue' in name else 'yellow'
                            cones.append({
                                'name': name,
                                'x': parts[0],
                                'y': parts[1],
                                'color': color
                            })
        except Exception as e:
            self.get_logger().error(f"Error loading cone map: {e}")
        return cones

    # ------------------ Callback Handlers ------------------

    def vel_cmd_callback(self, msg: Float64):
        self.v_meas = msg.data

    def steer_cmd_callback(self, msg: Float64):
        self.delta_meas = msg.data

    def imu_callback(self, msg: Imu):
        self.omega_imu = msg.angular_velocity.z
        self.has_imu = True

    def odom_callback(self, msg: Odometry):
        # Extract ground truth pose
        first_init = not self.has_gt
        self.x_gt[0] = msg.pose.pose.position.x
        self.x_gt[1] = msg.pose.pose.position.y
        self.x_gt[2] = quaternion_to_yaw(msg.pose.pose.orientation)
        self.has_gt = True

        jump = math.hypot(self.x_gt[0] - self.x[0], self.x_gt[1] - self.x[1])
        if first_init or jump > 2.0:
            self.x[0] = self.x_gt[0]
            self.x[1] = self.x_gt[1]
            self.x[2] = self.x_gt[2]
            self.x[3] = 0.0
            self.x_dr = np.copy(self.x_gt)
            self.P = np.diag([0.01, 0.01, 0.005, 0.01])
            self.ekf_path.poses.clear()
            self.gt_path.poses.clear()
            self.dr_path.poses.clear()
            self.get_logger().info(f"Synchronized / Reset EKF state to ({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f})")

        # Linear speed from wheel odometry
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        # Sign from forward axis
        if vx < 0.0:
            speed = -speed
        self.v_meas = speed

        # Direct EKF velocity measurement update
        self.update_velocity(speed)

    # ------------------ EKF Core Algorithm ------------------

    def timer_predict_callback(self):
        """EKF Prediction Step based on Ackermann kinematic model."""
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        # Motion model inputs
        v = self.x[3]  # current estimated velocity
        if self.has_imu:
            omega = self.omega_imu
        else:
            omega = (v / self.wheelbase) * math.tan(self.delta_meas)

        # 1. State prediction
        theta = self.x[2]
        half_dtheta = 0.5 * omega * dt
        dx = v * dt * math.cos(theta + half_dtheta)
        dy = v * dt * math.sin(theta + half_dtheta)
        dtheta = omega * dt

        self.x[0] += dx
        self.x[1] += dy
        self.x[2] = normalize_angle(theta + dtheta)

        # Dead Reckoning update (with small simulated wheel slip drift of 2%)
        v_dr = self.v_meas * 1.02
        omega_dr = omega * 1.01
        self.x_dr[0] += v_dr * dt * math.cos(self.x_dr[2] + 0.5 * omega_dr * dt)
        self.x_dr[1] += v_dr * dt * math.sin(self.x_dr[2] + 0.5 * omega_dr * dt)
        self.x_dr[2] = normalize_angle(self.x_dr[2] + omega_dr * dt)

        # 2. State transition Jacobian F
        mid_angle = theta + half_dtheta
        F = np.eye(4)
        F[0, 2] = -v * dt * math.sin(mid_angle)
        F[0, 3] = dt * math.cos(mid_angle)
        F[1, 2] = v * dt * math.cos(mid_angle)
        F[1, 3] = dt * math.sin(mid_angle)

        # 3. Process noise covariance Q
        Q = np.diag([self.q_x**2, self.q_y**2, self.q_theta**2, self.q_v**2]) * dt

        # 4. Covariance prediction
        self.P = F @ self.P @ F.T + Q

        # Publish state and visual markers
        self.publish_estimates(now)

    def update_velocity(self, v_obs: float):
        """EKF scalar velocity update."""
        H = np.array([[0.0, 0.0, 0.0, 1.0]])
        y = v_obs - self.x[3]
        S = H @ self.P @ H.T + self.R_v
        K = (self.P @ H.T) / S[0, 0]

        self.x += (K * y).flatten()
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + (K * self.R_v) @ K.T

    def scan_callback(self, msg: LaserScan):
        """Extract cone clusters from 2D LiDAR and perform EKF landmark measurement updates."""
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Valid range filter: only look within 0.15m to 10.0m
        valid = (ranges > 0.15) & (ranges < 10.0) & np.isfinite(ranges)
        if not np.any(valid):
            return

        r_v = ranges[valid]
        a_v = angles[valid]

        # Convert to LiDAR local coordinates
        xs = r_v * np.cos(a_v)
        ys = r_v * np.sin(a_v)

        # Cluster points into cone candidates
        clusters = []
        cur = []
        for i in range(len(xs)):
            if not cur:
                cur.append((xs[i], ys[i], r_v[i], a_v[i]))
            else:
                dist = math.hypot(xs[i] - cur[-1][0], ys[i] - cur[-1][1])
                if dist < self.cluster_max_dist:
                    cur.append((xs[i], ys[i], r_v[i], a_v[i]))
                else:
                    if len(cur) >= self.min_cluster_pts:
                        clusters.append(cur)
                    cur = [(xs[i], ys[i], r_v[i], a_v[i])]
        if len(cur) >= self.min_cluster_pts:
            clusters.append(cur)

        if not clusters or not self.map_cones:
            return

        # Process each detected cluster
        detected_landmarks = []
        for cluster in clusters:
            cx_local = float(np.mean([p[0] for p in cluster]))
            cy_local = float(np.mean([p[1] for p in cluster]))
            r_meas = math.hypot(cx_local, cy_local)
            phi_meas = math.atan2(cy_local, cx_local)

            # Transform detected landmark to map frame using current state estimate
            cos_th = math.cos(self.x[2])
            sin_th = math.sin(self.x[2])
            x_robot = cx_local + self.lidar_offset_x
            y_robot = cy_local

            mx_est = self.x[0] + x_robot * cos_th - y_robot * sin_th
            my_est = self.x[1] + x_robot * sin_th + y_robot * cos_th
            detected_landmarks.append((mx_est, my_est))

            # Data Association: Find closest map cone
            best_cone = None
            min_dist = float('inf')
            for cone in self.map_cones:
                dist = math.hypot(mx_est - cone['x'], my_est - cone['y'])
                if dist < min_dist:
                    min_dist = dist
                    best_cone = cone

            # Validation Gate
            if best_cone is not None and min_dist < self.da_gate:
                self.ekf_landmark_update(r_meas, phi_meas, best_cone['x'], best_cone['y'])

        # Publish landmark visualization
        self.publish_cone_markers(detected_landmarks)

    def ekf_landmark_update(self, r_obs: float, phi_obs: float, cx: float, cy: float):
        """Perform EKF measurement update for a single matched cone landmark."""
        dx = cx - self.x[0]
        dy = cy - self.x[1]
        q = dx * dx + dy * dy
        d = math.sqrt(q)

        if d < 0.05:
            return

        # Expected measurement h(x)
        r_pred = d
        phi_pred = normalize_angle(math.atan2(dy, dx) - self.x[2])

        # Innovation
        y = np.array([
            r_obs - r_pred,
            normalize_angle(phi_obs - phi_pred)
        ])

        # Measurement Jacobian H = dh / dx (2 x 4)
        H = np.zeros((2, 4))
        H[0, 0] = -dx / d
        H[0, 1] = -dy / d
        H[0, 2] = 0.0
        H[0, 3] = 0.0

        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0
        H[1, 3] = 0.0

        # Innovation covariance S
        S = H @ self.P @ H.T + self.R_lidar

        # Kalman Gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        # State update
        dx_state = K @ y
        self.x += dx_state
        self.x[2] = normalize_angle(self.x[2])

        # Joseph form covariance update: P = (I - KH) P (I - KH)^T + K R K^T
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_lidar @ K.T

    # ------------------ Publication & Visualization ------------------

    def publish_estimates(self, now: Time):
        header_stamp = now.to_msg()

        # 1. PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = header_stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = float(self.x[0])
        pose_msg.pose.pose.position.y = float(self.x[1])
        pose_msg.pose.pose.position.z = 0.06
        pose_msg.pose.pose.orientation = yaw_to_quaternion(float(self.x[2]))

        # Covariance 6x6
        cov6 = [0.0] * 36
        cov6[0] = float(self.P[0, 0])   # x
        cov6[1] = float(self.P[0, 1])
        cov6[6] = float(self.P[1, 0])
        cov6[7] = float(self.P[1, 1])   # y
        cov6[35] = float(self.P[2, 2])  # yaw
        pose_msg.pose.covariance = cov6
        self.pub_pose.publish(pose_msg)

        # 2. EKF Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = header_stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = float(self.x[3])
        odom_msg.twist.twist.angular.z = float(self.omega_imu)
        self.pub_odom.publish(odom_msg)

        # 3. Path Append
        p_ekf = PoseStamped()
        p_ekf.header = pose_msg.header
        p_ekf.pose = pose_msg.pose.pose
        self.ekf_path.poses.append(p_ekf)
        if len(self.ekf_path.poses) > 1500:
            self.ekf_path.poses.pop(0)
        self.pub_path_ekf.publish(self.ekf_path)

        # Ground Truth Path
        if self.has_gt:
            p_gt = PoseStamped()
            p_gt.header.stamp = header_stamp
            p_gt.header.frame_id = 'map'
            p_gt.pose.position.x = float(self.x_gt[0])
            p_gt.pose.position.y = float(self.x_gt[1])
            p_gt.pose.position.z = 0.06
            p_gt.pose.orientation = yaw_to_quaternion(float(self.x_gt[2]))
            self.gt_path.poses.append(p_gt)
            if len(self.gt_path.poses) > 1500:
                self.gt_path.poses.pop(0)
            self.pub_path_gt.publish(self.gt_path)

        # Dead Reckoning Path
        p_dr = PoseStamped()
        p_dr.header.stamp = header_stamp
        p_dr.header.frame_id = 'map'
        p_dr.pose.position.x = float(self.x_dr[0])
        p_dr.pose.position.y = float(self.x_dr[1])
        p_dr.pose.position.z = 0.06
        p_dr.pose.orientation = yaw_to_quaternion(float(self.x_dr[2]))
        self.dr_path.poses.append(p_dr)
        if len(self.dr_path.poses) > 1500:
            self.dr_path.poses.pop(0)
        self.pub_path_dr.publish(self.dr_path)

        # 4. TF map -> body_link, camera, and lidar frames for RViz
        if self.publish_tf:
            transforms = []

            # map -> body_link
            t_body = TransformStamped()
            t_body.header.stamp = header_stamp
            t_body.header.frame_id = 'map'
            t_body.child_frame_id = 'body_link'
            t_body.transform.translation.x = float(self.x[0])
            t_body.transform.translation.y = float(self.x[1])
            t_body.transform.translation.z = 0.06
            t_body.transform.rotation = pose_msg.pose.pose.orientation
            transforms.append(t_body)

            # map -> ekf_base_link (backward compatibility)
            t_ekf = TransformStamped()
            t_ekf.header.stamp = header_stamp
            t_ekf.header.frame_id = 'map'
            t_ekf.child_frame_id = 'ekf_base_link'
            t_ekf.transform = t_body.transform
            transforms.append(t_ekf)

            # body_link -> ackermann_steering_vehicle/body_link/vehicle_lidar (for LaserScan display in RViz)
            t_lidar = TransformStamped()
            t_lidar.header.stamp = header_stamp
            t_lidar.header.frame_id = 'body_link'
            t_lidar.child_frame_id = 'ackermann_steering_vehicle/body_link/vehicle_lidar'
            t_lidar.transform.translation.x = float(self.lidar_offset_x)
            t_lidar.transform.translation.y = 0.0
            t_lidar.transform.translation.z = 0.065
            t_lidar.transform.rotation.w = 1.0
            transforms.append(t_lidar)

            # body_link -> ackermann_steering_vehicle/body_link/vehicle_front_camera (for Camera display in RViz)
            t_cam = TransformStamped()
            t_cam.header.stamp = header_stamp
            t_cam.header.frame_id = 'body_link'
            t_cam.child_frame_id = 'ackermann_steering_vehicle/body_link/vehicle_front_camera'
            t_cam.transform.translation.x = 0.14
            t_cam.transform.translation.y = 0.0
            t_cam.transform.translation.z = 0.20
            t_cam.transform.rotation.w = 1.0
            transforms.append(t_cam)

            self.tf_broadcaster.sendTransform(transforms)

    def publish_track_cones(self):
        """Publish 3D markers for all 136 track cones for RViz."""
        if not self.map_cones:
            return
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, c in enumerate(self.map_cones):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'track_cones'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(c['x'])
            m.pose.position.y = float(c['y'])
            m.pose.position.z = 0.11
            m.pose.orientation.w = 1.0
            m.scale.x = 0.16
            m.scale.y = 0.16
            m.scale.z = 0.22
            if c['color'] == 'blue':
                m.color.r = 0.15
                m.color.g = 0.50
                m.color.b = 1.0
                m.color.a = 0.90
            else:
                m.color.r = 1.0
                m.color.g = 0.85
                m.color.b = 0.0
                m.color.a = 0.90
            ma.markers.append(m)
        self.pub_track_cones.publish(ma)

    def publish_cone_markers(self, detected_landmarks):
        """Publish RViz visualization markers for detected cone landmarks, laser sightlines, and 3-sigma covariance ellipse."""
        marker_array = MarkerArray()

        # 1. Detected Cones Marker (Red points/spheres)
        m_det = Marker()
        m_det.header.frame_id = 'map'
        m_det.header.stamp = self.get_clock().now().to_msg()
        m_det.ns = 'detected_cones'
        m_det.id = 0
        m_det.type = Marker.SPHERE_LIST
        m_det.action = Marker.ADD
        m_det.scale.x = 0.18
        m_det.scale.y = 0.18
        m_det.scale.z = 0.24
        m_det.color.r = 1.0
        m_det.color.g = 0.1
        m_det.color.b = 0.1
        m_det.color.a = 0.95

        for mx, my in detected_landmarks:
            p = Point()
            p.x = mx
            p.y = my
            p.z = 0.11
            m_det.points.append(p)
        marker_array.markers.append(m_det)

        # 2. Laser Sightline Beams (Connecting vehicle to detected cone landmarks)
        m_lines = Marker()
        m_lines.header.frame_id = 'map'
        m_lines.header.stamp = self.get_clock().now().to_msg()
        m_lines.ns = 'ekf_sightlines'
        m_lines.id = 1
        m_lines.type = Marker.LINE_LIST
        m_lines.action = Marker.ADD
        m_lines.scale.x = 0.02
        m_lines.color.r = 1.0
        m_lines.color.g = 0.3
        m_lines.color.b = 0.1
        m_lines.color.a = 0.7

        p_veh = Point()
        p_veh.x = float(self.x[0])
        p_veh.y = float(self.x[1])
        p_veh.z = 0.125

        for mx, my in detected_landmarks:
            p_cone = Point()
            p_cone.x = mx
            p_cone.y = my
            p_cone.z = 0.11
            m_lines.points.append(p_veh)
            m_lines.points.append(p_cone)
        marker_array.markers.append(m_lines)

        # 3. Covariance Ellipse Marker
        m_cov = Marker()
        m_cov.header.frame_id = 'map'
        m_cov.header.stamp = self.get_clock().now().to_msg()
        m_cov.ns = 'ekf_covariance'
        m_cov.id = 2
        m_cov.type = Marker.CYLINDER
        m_cov.action = Marker.ADD
        m_cov.pose.position.x = float(self.x[0])
        m_cov.pose.position.y = float(self.x[1])
        m_cov.pose.position.z = 0.01

        # Eigenvalues of 2x2 position covariance
        cov_pos = self.P[:2, :2]
        eigvals, eigvecs = np.linalg.eigh(cov_pos)
        # 3-sigma semi-axes
        sigma_x = 3.0 * math.sqrt(max(1e-6, eigvals[1]))
        sigma_y = 3.0 * math.sqrt(max(1e-6, eigvals[0]))
        angle = math.atan2(eigvecs[1, 1], eigvecs[0, 1])

        m_cov.scale.x = max(0.05, 2.0 * sigma_x)
        m_cov.scale.y = max(0.05, 2.0 * sigma_y)
        m_cov.scale.z = 0.01
        m_cov.pose.orientation = yaw_to_quaternion(angle)
        m_cov.color.r = 0.1
        m_cov.color.g = 0.9
        m_cov.color.b = 0.3
        m_cov.color.a = 0.35
        marker_array.markers.append(m_cov)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
