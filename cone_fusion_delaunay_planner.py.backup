#!/usr/bin/env python3
"""
Camera-LiDAR Fusion & Delaunay Triangulation Path Planner
For Ackermann Steering Vehicle in Formula Student Cone Track

Pipeline:
  1. 2D LiDAR Clustering -> Extracts metric cone candidate centroids (x, y).
  2. Camera Projection -> Projects metric centroids onto the camera image plane (u, v).
  3. Color Classification -> Samples HSV color masks to classify cones as Blue (Left) or Yellow (Right).
  4. Delaunay Triangulation -> Triangulates perceived cones using scipy.spatial.Delaunay.
  5. Cross-Track Edge Extraction -> Finds edges connecting Blue cones to Yellow cones.
  6. Centerline Midpoint Generation -> Computes midpoints of valid cross-track edges.
  7. Forward Path Ordering & Spline -> Orders midpoints forward from vehicle into a collision-free path.
  8. Pure Pursuit Path Tracking -> Generates /steering_angle and /velocity to drive the vehicle.
  9. Visualization -> Publishes RViz markers, debug camera image, and planned path.
"""

import os
import sys
import time
import math
import numpy as np
import cv2
from scipy.spatial import Delaunay

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Image as RosImage
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64, ColorRGBA


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ConeFusionDelaunayPlanner(Node):
    def __init__(self):
        super().__init__('cone_fusion_delaunay_planner')

        # Parameters
        self.declare_parameter('controller_type', 'stanley')
        self.declare_parameter('target_velocity', 4.1667)  # 15 km/h = 4.1667 m/s
        self.declare_parameter('max_steering_angle', 0.506145)  # 29 deg = 0.506145 rad
        self.declare_parameter('lookahead_distance', 1.45)
        self.declare_parameter('wheelbase', 0.485)  # 485 mm = 0.485 m
        self.declare_parameter('min_track_width', 0.8)
        self.declare_parameter('max_track_width', 3.2)
        self.declare_parameter('planning_horizon', 10.5)
        self.declare_parameter('stanley_k', 2.2)
        self.declare_parameter('stanley_ks', 0.6)
        self.declare_parameter('stanley_kd', 0.10)
        self.declare_parameter('ay_max', 3.5)
        self.declare_parameter('min_velocity', 1.2)

        self.controller_type = self.get_parameter('controller_type').value.lower()
        self.v_target = self.get_parameter('target_velocity').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.min_track_w = self.get_parameter('min_track_width').value
        self.max_track_w = self.get_parameter('max_track_width').value
        self.horizon = self.get_parameter('planning_horizon').value
        self.stanley_k = self.get_parameter('stanley_k').value
        self.stanley_ks = self.get_parameter('stanley_ks').value
        self.stanley_kd = self.get_parameter('stanley_kd').value
        self.ay_max = self.get_parameter('ay_max').value
        self.min_velocity = self.get_parameter('min_velocity').value

        # Camera Intrinsics (640x480, 80 deg FOV = 1.39626 rad)
        fov_rad = 1.3962634
        self.fx = 320.0 / math.tan(fov_rad / 2.0)
        self.fy = self.fx
        self.cx = 320.0
        self.cy = 240.0

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pub_vel = self.create_publisher(Float64, '/velocity', 10)
        self.pub_steer = self.create_publisher(Float64, '/steering_angle', 10)
        self.pub_path = self.create_publisher(Path, '/planning/delaunay_path', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/planning/delaunay_markers', 10)
        self.pub_debug_img = self.create_publisher(RosImage, '/planning/debug_image', 10)

        # Subscribers
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.sub_cam = self.create_subscription(
            RosImage, '/camera/image_raw', self.cam_callback, sensor_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/model/ackermann_steering_vehicle/odometry', self.odom_callback, sensor_qos)

        # Latest states
        self.latest_scan = None
        self.latest_cam_bgr = None
        self.latest_cam_hsv = None
        self.latest_cam_header = None
        self.veh_x = 0.0
        self.veh_y = 0.0
        self.veh_yaw = 0.0
        self.veh_yaw_rate = 0.0
        self.veh_speed = 0.0
        self.has_odom = False

        # Rolling cone memory
        self.known_cones = {}
        self.latest_steer = 0.0

        # Main Planning & Control Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Cone Fusion & Delaunay Triangulation Planner initialized ({self.controller_type.upper()}, target_v={self.v_target}m/s)!")

    def odom_callback(self, msg: Odometry):
        self.veh_x = msg.pose.pose.position.x
        self.veh_y = msg.pose.pose.position.y
        self.veh_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.veh_yaw_rate = msg.twist.twist.angular.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.veh_speed = math.hypot(vx, vy)
        self.has_odom = True

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        self.latest_scan = (ranges, angles, msg.header)

    def cam_callback(self, msg: RosImage):
        try:
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            self.latest_cam_bgr = bgr
            self.latest_cam_hsv = hsv
            self.latest_cam_header = msg.header
        except Exception as e:
            self.get_logger().warn(f"Error decoding camera frame: {e}")

    def extract_and_fuse_cones(self):
        if self.latest_scan is None:
            return [], []

        ranges, angles, _ = self.latest_scan
        valid = (ranges > 0.15) & (ranges < self.horizon) & np.isfinite(ranges)
        if not np.any(valid):
            return [], []

        xs_loc = ranges[valid] * np.cos(angles[valid])
        ys_loc = ranges[valid] * np.sin(angles[valid])

        clusters = []
        cur = []
        for i in range(len(xs_loc)):
            if not cur:
                cur.append((xs_loc[i], ys_loc[i]))
            else:
                d = math.hypot(xs_loc[i] - cur[-1][0], ys_loc[i] - cur[-1][1])
                if d < 0.25:
                    cur.append((xs_loc[i], ys_loc[i]))
                else:
                    if len(cur) >= 1:
                        clusters.append(cur)
                    cur = [(xs_loc[i], ys_loc[i])]
        if len(cur) >= 1:
            clusters.append(cur)

        mask_y = None
        mask_b = None
        mask_r = None
        mask_g = None
        debug_img = None
        if self.latest_cam_bgr is not None:
            debug_img = self.latest_cam_bgr.copy()
        if self.latest_cam_hsv is not None:
            mask_y = cv2.inRange(self.latest_cam_hsv, (15, 60, 60), (35, 255, 255))
            mask_b = cv2.inRange(self.latest_cam_hsv, (95, 60, 60), (135, 255, 255))
            mask_r = cv2.inRange(self.latest_cam_hsv, (0, 70, 50), (10, 255, 255)) | cv2.inRange(self.latest_cam_hsv, (170, 70, 50), (180, 255, 255))
            mask_g = cv2.inRange(self.latest_cam_hsv, (36, 60, 50), (85, 255, 255))

        cos_yaw = math.cos(self.veh_yaw)
        sin_yaw = math.sin(self.veh_yaw)

        classified_cones = []
        noise_obstacles = []
        for cl in clusters:
            cx_l = float(np.mean([p[0] for p in cl]))
            cy_l = float(np.mean([p[1] for p in cl]))
            r = math.hypot(cx_l, cy_l)
            phi = math.atan2(cy_l, cx_l)

            # Skip cones far behind the rear axle
            if cx_l < -0.8:
                continue

            wx = self.veh_x + cx_l * cos_yaw - cy_l * sin_yaw
            wy = self.veh_y + cx_l * sin_yaw + cy_l * cos_yaw

            color = "unknown"
            is_noise = False
            noise_type = ""
            if mask_y is not None and cx_l > 0.25 and abs(phi) < math.radians(38):
                u = int(-self.fx * math.tan(phi) + self.cx)
                if 15 <= u < 625:
                    strip_y = mask_y[130:320, max(0, u-16):min(640, u+16)]
                    strip_b = mask_b[130:320, max(0, u-16):min(640, u+16)]
                    strip_r = mask_r[130:320, max(0, u-16):min(640, u+16)] if mask_r is not None else None
                    strip_g = mask_g[130:320, max(0, u-16):min(640, u+16)] if mask_g is not None else None

                    cnt_y = cv2.countNonZero(strip_y)
                    cnt_b = cv2.countNonZero(strip_b)
                    cnt_r = cv2.countNonZero(strip_r) if strip_r is not None else 0
                    cnt_g = cv2.countNonZero(strip_g) if strip_g is not None else 0

                    # 1. Distractor / Noise Detection (Red or Green 3D Obstacles)
                    if cnt_r >= 6 and cnt_r > cnt_y and cnt_r > cnt_b:
                        is_noise = True
                        noise_type = "RED NOISE"
                    elif cnt_g >= 6 and cnt_g > cnt_y and cnt_g > cnt_b:
                        is_noise = True
                        noise_type = "GREEN NOISE"
                    # 2. Valid Cone Classification (Yellow or Blue)
                    elif cnt_y > cnt_b and cnt_y >= 5:
                        color = "yellow"
                    elif cnt_b > cnt_y and cnt_b >= 5:
                        color = "blue"
                    elif cnt_r >= 4 or cnt_g >= 4:
                        is_noise = True
                        noise_type = "DISTRACTOR"

                    if is_noise:
                        if debug_img is not None:
                            # Draw prominent RED warning box on rejected noise polygon
                            cv2.rectangle(debug_img, (max(0, u-18), 140), (min(640, u+18), 265), (0, 0, 255), 2)
                            cv2.drawMarker(debug_img, (u, 202), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
                            cv2.putText(debug_img, f"{noise_type} {r:.1f}m", (u-30, 132),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 255), 1)
                    else:
                        if debug_img is not None and color != "unknown":
                            dot_col = (0, 255, 255) if color == "yellow" else (255, 120, 30)
                            cv2.rectangle(debug_img, (max(0, u-16), 150), (min(640, u+16), 260), dot_col, 1)
                            cv2.circle(debug_img, (u, 200), 5, dot_col, -1)
                            cv2.putText(debug_img, f"{color[0].upper()} {r:.1f}m", (u-15, 145),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, dot_col, 1)

            if is_noise:
                noise_obstacles.append({
                    "loc_x": cx_l, "loc_y": cy_l,
                    "world_x": wx, "world_y": wy,
                    "type": noise_type, "range": r
                })
            else:
                if color == "unknown":
                    color = "yellow" if cy_l < 0 else "blue"

                classified_cones.append({
                    "loc_x": cx_l, "loc_y": cy_l,
                    "world_x": wx, "world_y": wy,
                    "color": color, "range": r
                })

        if debug_img is not None:
            try:
                # Top HUD Banner with Noise Statistics
                cv2.rectangle(debug_img, (0, 0), (640, 34), (18, 18, 18), -1)
                hud_str = f"V: {self.veh_speed:.2f}m/s | Steer: {math.degrees(self.latest_steer):+.1f} deg | Cones: {len(classified_cones)} | Noise Filtered: {len(noise_obstacles)}"
                cv2.putText(debug_img, hud_str, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 220), 1)

                ros_img = RosImage()
                ros_img.header = self.latest_cam_header
                ros_img.height = debug_img.shape[0]
                ros_img.width = debug_img.shape[1]
                ros_img.encoding = "bgr8"
                ros_img.is_bigendian = 0
                ros_img.step = debug_img.shape[1] * 3
                ros_img.data = debug_img.tobytes()
                self.pub_debug_img.publish(ros_img)
            except Exception:
                pass

        return classified_cones, noise_obstacles

    def plan_delaunay_path(self, cones):
        blue_pts = [np.array([c["world_x"], c["world_y"]]) for c in cones if c["color"] == "blue"]
        yellow_pts = [np.array([c["world_x"], c["world_y"]]) for c in cones if c["color"] == "yellow"]

        if not blue_pts or not yellow_pts:
            return [], [], [], []

        all_pts = np.vstack(blue_pts + yellow_pts)
        colors = ["blue"] * len(blue_pts) + ["yellow"] * len(yellow_pts)

        if len(all_pts) < 3:
            p_b = blue_pts[0]
            p_y = yellow_pts[0]
            d = np.linalg.norm(p_b - p_y)
            if self.min_track_w <= d <= self.max_track_w:
                cross_edges = [(p_b, p_y)]
                midpoints = [0.5 * (p_b + p_y)]
                return midpoints, cross_edges, midpoints, []
            return [], [], [], []

        mesh_edges = []
        cross_edges = []
        midpoints = []
        try:
            tri = Delaunay(all_pts)
            edges = set()
            for simplex in tri.simplices:
                for i in range(3):
                    e = tuple(sorted([simplex[i], simplex[(i+1)%3]]))
                    edges.add(e)

            for i1, i2 in edges:
                p1 = all_pts[i1]
                p2 = all_pts[i2]
                mesh_edges.append((p1, p2))
                c1, c2 = colors[i1], colors[i2]
                if c1 != c2:
                    d = np.linalg.norm(p1 - p2)
                    if self.min_track_w <= d <= self.max_track_w:
                        cross_edges.append((p1, p2))
                        midpoints.append(0.5 * (p1 + p2))
        except Exception:
            pass

        if not midpoints:
            return [], [], [], mesh_edges

        ordered_path = self.order_centerline_midpoints(midpoints)
        return ordered_path, cross_edges, midpoints, mesh_edges

    def order_centerline_midpoints(self, midpoints):
        veh_pt = np.array([self.veh_x, self.veh_y])
        veh_dir = np.array([math.cos(self.veh_yaw), math.sin(self.veh_yaw)])

        remaining = list(midpoints)
        ordered = []
        curr = veh_pt
        curr_dir = veh_dir

        for _ in range(min(20, len(remaining))):
            best_idx = None
            best_score = float("inf")

            for i, pt in enumerate(remaining):
                vec = pt - curr
                dist = np.linalg.norm(vec)
                if dist < 0.25:
                    continue

                dot = np.dot(vec / (dist + 1e-6), curr_dir)
                if dot > -0.1:
                    score = dist + (1.0 - dot) * 2.2
                    if score < best_score:
                        best_score = score
                        best_idx = i

            if best_idx is None:
                break

            next_pt = remaining.pop(best_idx)
            ordered.append(next_pt)
            curr_dir = (next_pt - curr) / (np.linalg.norm(next_pt - curr) + 1e-6)
            curr = next_pt

        return ordered

    def stanley_step(self, path):
        if not path:
            return self.min_velocity, 0.0, None
        if len(path) == 1:
            dx = path[0][0] - self.veh_x
            dy = path[0][1] - self.veh_y
            alpha = normalize_angle(math.atan2(dy, dx) - self.veh_yaw)
            delta = float(np.clip(math.atan2(2.0 * self.wheelbase * math.sin(alpha), 1.0), -self.max_steer, self.max_steer))
            return self.min_velocity, delta, path[0]

        # 1. Front axle location (Stanley control reference point)
        fx = self.veh_x + self.wheelbase * math.cos(self.veh_yaw)
        fy = self.veh_y + self.wheelbase * math.sin(self.veh_yaw)
        front_pt = np.array([fx, fy])

        path_arr = np.array(path)
        dists = np.linalg.norm(path_arr - front_pt, axis=1)
        idx = int(np.argmin(dists))

        # 2. Path tangent and reference heading (classical Stanley)
        if idx < len(path_arr) - 1:
            p_curr = path_arr[idx]
            p_next = path_arr[idx + 1]
        elif idx > 0:
            p_curr = path_arr[idx - 1]
            p_next = path_arr[idx]
        else:
            p_curr = path_arr[0]
            p_next = path_arr[0] + np.array([math.cos(self.veh_yaw), math.sin(self.veh_yaw)])

        tangent = p_next - p_curr
        tan_norm = np.linalg.norm(tangent) + 1e-6
        tangent_u = tangent / tan_norm
        theta_path = math.atan2(tangent[1], tangent[0])

        # 3. Signed cross-track error at front axle
        normal = np.array([-tangent_u[1], tangent_u[0]])
        vec_to_fa = front_pt - p_curr
        e_fa = float(np.dot(vec_to_fa, normal))

        # 4. Heading error
        theta_e = normalize_angle(theta_path - self.veh_yaw)

        # 5. Stanley control law:
        # If e_fa > 0 (vehicle left of track), steer right (delta < 0)
        v_eff = max(0.1, self.veh_speed)
        delta_cte = math.atan2(-self.stanley_k * e_fa, self.stanley_ks + v_eff)

        # 6. Yaw rate damping to suppress high-speed oscillation
        yaw_damping = self.stanley_kd * self.veh_yaw_rate

        delta = theta_e + delta_cte - yaw_damping
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))

        # 7. Anticipatory Curvature-Adaptive Speed Profiling
        min_v_curve = self.v_target
        a_brake = 3.0  # Anticipatory deceleration capacity (m/s^2)
        cum_d = 0.0
        for k in range(max(1, idx), min(len(path_arr) - 1, idx + 14)):
            step_len = float(np.linalg.norm(path_arr[k] - path_arr[k - 1]))
            cum_d += step_len
            v1 = path_arr[k] - path_arr[k - 1]
            v2 = path_arr[k + 1] - path_arr[k]
            th1 = math.atan2(v1[1], v1[0])
            th2 = math.atan2(v2[1], v2[0])
            d_th = abs(normalize_angle(th2 - th1))
            ds = 0.5 * (np.linalg.norm(v1) + np.linalg.norm(v2)) + 1e-6
            kappa = d_th / ds
            v_turn = math.sqrt(self.ay_max / (kappa + 1e-4))
            v_entry_limit = math.sqrt(v_turn * v_turn + 2.0 * a_brake * cum_d)
            if v_entry_limit < min_v_curve:
                min_v_curve = v_entry_limit

        v_cmd = min(self.v_target, min_v_curve)

        # Steering angle speed attenuation
        steer_ratio = abs(delta) / self.max_steer
        v_cmd = min(v_cmd, self.v_target * (1.0 - 0.52 * steer_ratio))
        v_cmd = float(np.clip(v_cmd, self.min_velocity, self.v_target))

        return v_cmd, delta, p_curr

    def pure_pursuit_step(self, path):
        if not path:
            return 0.0, 0.0, None

        veh_pt = np.array([self.veh_x, self.veh_y])

        target_pt = None
        for pt in path:
            d = np.linalg.norm(pt - veh_pt)
            if d >= self.lookahead:
                target_pt = pt
                break

        if target_pt is None:
            target_pt = path[-1]

        dx = target_pt[0] - self.veh_x
        dy = target_pt[1] - self.veh_y
        angle_to_target = math.atan2(dy, dx)
        alpha = normalize_angle(angle_to_target - self.veh_yaw)

        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead)
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))

        steer_ratio = abs(delta) / self.max_steer
        v_cmd = self.v_target * (1.0 - 0.30 * steer_ratio)

        return v_cmd, delta, target_pt

    def control_loop(self):
        if not self.has_odom:
            return

        cones, noise_obstacles = self.extract_and_fuse_cones()
        if not cones and not noise_obstacles:
            return

        path, cross_edges, midpoints, mesh_edges = self.plan_delaunay_path(cones)

        target_pt = None
        if path:
            if self.controller_type == 'stanley':
                v_cmd, delta_cmd, target_pt = self.stanley_step(path)
            else:
                v_cmd, delta_cmd, target_pt = self.pure_pursuit_step(path)
            self.latest_steer = delta_cmd

            v_msg = Float64()
            v_msg.data = float(v_cmd)
            self.pub_vel.publish(v_msg)

            s_msg = Float64()
            s_msg.data = float(delta_cmd)
            self.pub_steer.publish(s_msg)

        self.publish_rviz_path(path)
        self.publish_rviz_markers(cones, cross_edges, midpoints, mesh_edges, target_pt, noise_obstacles)

    def publish_rviz_path(self, path):
        if not path:
            return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        p0 = PoseStamped()
        p0.header = path_msg.header
        p0.pose.position.x = float(self.veh_x)
        p0.pose.position.y = float(self.veh_y)
        p0.pose.position.z = 0.05
        path_msg.poses.append(p0)

        for pt in path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = 0.05
            path_msg.poses.append(ps)

        self.pub_path.publish(path_msg)

    def publish_rviz_markers(self, cones, cross_edges, midpoints, mesh_edges, target_pt, noise_obstacles=None):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        m_cones = Marker()
        m_cones.header.frame_id = "map"
        m_cones.header.stamp = stamp
        m_cones.ns = "fused_cones"
        m_cones.id = 0
        m_cones.type = Marker.SPHERE_LIST
        m_cones.action = Marker.ADD
        m_cones.scale.x = 0.18
        m_cones.scale.y = 0.18
        m_cones.scale.z = 0.22

        for c in cones:
            p = Point()
            p.x = float(c["world_x"])
            p.y = float(c["world_y"])
            p.z = 0.11
            m_cones.points.append(p)
            col = ColorRGBA()
            col.a = 0.95
            if c["color"] == "blue":
                col.r, col.g, col.b = 0.15, 0.50, 1.0
            else:
                col.r, col.g, col.b = 1.0, 0.85, 0.0
            m_cones.colors.append(col)
        ma.markers.append(m_cones)

        m_edges = Marker()
        m_edges.header.frame_id = "map"
        m_edges.header.stamp = stamp
        m_edges.ns = "delaunay_cross_edges"
        m_edges.id = 1
        m_edges.type = Marker.LINE_LIST
        m_edges.action = Marker.ADD
        m_edges.scale.x = 0.025
        m_edges.color.r = 1.0
        m_edges.color.g = 0.25
        m_edges.color.b = 0.25
        m_edges.color.a = 0.75

        for p1, p2 in cross_edges:
            pt1 = Point(x=float(p1[0]), y=float(p1[1]), z=0.08)
            pt2 = Point(x=float(p2[0]), y=float(p2[1]), z=0.08)
            m_edges.points.append(pt1)
            m_edges.points.append(pt2)
        ma.markers.append(m_edges)

        m_mid = Marker()
        m_mid.header.frame_id = "map"
        m_mid.header.stamp = stamp
        m_mid.ns = "delaunay_midpoints"
        m_mid.id = 2
        m_mid.type = Marker.SPHERE_LIST
        m_mid.action = Marker.ADD
        m_mid.scale.x = 0.12
        m_mid.scale.y = 0.12
        m_mid.scale.z = 0.12
        m_mid.color.r = 0.15
        m_mid.color.g = 0.95
        m_mid.color.b = 0.35
        m_mid.color.a = 0.9

        for mp in midpoints:
            pt = Point(x=float(mp[0]), y=float(mp[1]), z=0.06)
            m_mid.points.append(pt)
        ma.markers.append(m_mid)

        if mesh_edges:
            m_mesh = Marker()
            m_mesh.header.frame_id = "map"
            m_mesh.header.stamp = stamp
            m_mesh.ns = "delaunay_mesh"
            m_mesh.id = 4
            m_mesh.type = Marker.LINE_LIST
            m_mesh.action = Marker.ADD
            m_mesh.scale.x = 0.012
            m_mesh.color.r = 0.3
            m_mesh.color.g = 0.65
            m_mesh.color.b = 0.9
            m_mesh.color.a = 0.4
            for p1, p2 in mesh_edges:
                pt1 = Point(x=float(p1[0]), y=float(p1[1]), z=0.07)
                pt2 = Point(x=float(p2[0]), y=float(p2[1]), z=0.07)
                m_mesh.points.append(pt1)
                m_mesh.points.append(pt2)
            ma.markers.append(m_mesh)

        if noise_obstacles:
            m_noise = Marker()
            m_noise.header.frame_id = "map"
            m_noise.header.stamp = stamp
            m_noise.ns = "noise_obstacles_rejected"
            m_noise.id = 99
            m_noise.type = Marker.CUBE_LIST
            m_noise.action = Marker.ADD
            m_noise.scale.x = 0.28
            m_noise.scale.y = 0.28
            m_noise.scale.z = 0.28
            m_noise.color.r = 1.0
            m_noise.color.g = 0.05
            m_noise.color.b = 0.05
            m_noise.color.a = 0.95
            for n in noise_obstacles:
                pt = Point(x=float(n["world_x"]), y=float(n["world_y"]), z=0.14)
                m_noise.points.append(pt)
            ma.markers.append(m_noise)

        if target_pt is not None:
            m_tgt = Marker()
            m_tgt.header.frame_id = "map"
            m_tgt.header.stamp = stamp
            m_tgt.ns = "lookahead_target"
            m_tgt.id = 3
            m_tgt.type = Marker.SPHERE
            m_tgt.action = Marker.ADD
            m_tgt.pose.position.x = float(target_pt[0])
            m_tgt.pose.position.y = float(target_pt[1])
            m_tgt.pose.position.z = 0.12
            m_tgt.scale.x = 0.22
            m_tgt.scale.y = 0.22
            m_tgt.scale.z = 0.22
            m_tgt.color.r = 1.0
            m_tgt.color.g = 0.1
            m_tgt.color.b = 0.8
            m_tgt.color.a = 0.95
            ma.markers.append(m_tgt)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    planner = ConeFusionDelaunayPlanner()
    try:
        rclpy.spin(planner)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            stop_v = Float64(data=0.0)
            stop_s = Float64(data=0.0)
            planner.pub_vel.publish(stop_v)
            planner.pub_steer.publish(stop_s)
        except Exception:
            pass
        try:
            planner.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
