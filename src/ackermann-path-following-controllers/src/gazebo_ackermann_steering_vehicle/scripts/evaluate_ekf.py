#!/usr/bin/env python3
"""
Evaluation and Benchmarking Script for EKF Localization on Cone Track
Drives the vehicle along the cone circuit, logs Ground Truth, Dead Reckoning,
and EKF Estimated poses, calculates RMSE and error metrics, and plots the results.
"""

import time
import math
import os
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64


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


class EKFEvaluator(Node):
    def __init__(self):
        super().__init__('ekf_evaluator')

        # Publishers to drive vehicle
        self.pub_vel = self.create_publisher(Float64, '/velocity', 10)
        self.pub_steer = self.create_publisher(Float64, '/steering_angle', 10)

        # Subscribers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_gt = self.create_subscription(
            Odometry, '/model/ackermann_steering_vehicle/odometry', self.gt_callback, sensor_qos)
        self.sub_ekf = self.create_subscription(
            PoseWithCovarianceStamped, '/ekf/pose', self.ekf_callback, 10)

        # State storage
        self.latest_gt = None
        self.latest_ekf = None

        # Time series logs
        self.timestamps = []
        self.gt_x = []
        self.gt_y = []
        self.gt_yaw = []

        self.dr_x = []
        self.dr_y = []
        self.dr_yaw = []

        self.ekf_x = []
        self.ekf_y = []
        self.ekf_yaw = []
        self.ekf_sigma_x = []
        self.ekf_sigma_y = []

        # Internal Dead Reckoning integration
        self.curr_dr = np.array([0.0, 0.0, 0.0])
        self.last_dr_time = None

        # Load map cones for plotting
        self.blue_cones, self.yellow_cones = self.load_map_cones()

    def load_map_cones(self):
        blue = []
        yellow = []
        world_path = '/home/ahmed/gokart_ws/src/ackermann-path-following-controllers/src/gazebo_ackermann_steering_vehicle/worlds/track_with_cones.sdf'
        if os.path.exists(world_path):
            tree = ET.parse(world_path)
            root = tree.getroot()
            world = root.find('world')
            if world is not None:
                for model in world.findall('model'):
                    name = model.get('name', '')
                    if 'cone' in name:
                        pose = model.find('pose')
                        if pose is not None and pose.text:
                            p = [float(v) for v in pose.text.split()]
                            if 'blue' in name:
                                blue.append((p[0], p[1]))
                            else:
                                yellow.append((p[0], p[1]))
        return blue, yellow

    def gt_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_gt = (t, x, y, yaw, vx, wz)

    def ekf_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        sig_x = math.sqrt(max(1e-6, msg.pose.covariance[0]))
        sig_y = math.sqrt(max(1e-6, msg.pose.covariance[7]))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_ekf = (t, x, y, yaw, sig_x, sig_y)

    def run_evaluation(self, duration_sec=15.0):
        self.get_logger().info(f"Starting EKF Localization evaluation run for {duration_sec}s...")
        start_wall = time.time()
        start_sim = None

        # Wait for initial messages
        while rclpy.ok() and (self.latest_gt is None or self.latest_ekf is None):
            rclpy.spin_once(self, timeout_sec=0.1)

        start_sim = self.latest_gt[0]
        self.curr_dr = np.array([self.latest_gt[1], self.latest_gt[2], self.latest_gt[3]])
        self.last_dr_time = start_sim

        rate_hz = 20.0
        dt = 1.0 / rate_hz

        while rclpy.ok():
            now_sim = self.latest_gt[0]
            elapsed_sim = now_sim - start_sim
            if elapsed_sim >= duration_sec:
                break

            # Driving Profile:
            # First 3s straight, then curve along track straightaway and chicane
            if elapsed_sim < 3.0:
                v_cmd = 0.5
                steer_cmd = 0.0
            elif elapsed_sim < 8.0:
                v_cmd = 0.55
                steer_cmd = 0.08 * math.sin(0.8 * (elapsed_sim - 3.0))
            else:
                v_cmd = 0.5
                steer_cmd = 0.12 * math.sin(0.6 * (elapsed_sim - 8.0))

            v_msg = Float64()
            v_msg.data = float(v_cmd)
            self.pub_vel.publish(v_msg)

            s_msg = Float64()
            s_msg.data = float(steer_cmd)
            self.pub_steer.publish(s_msg)

            # Update Dead Reckoning (simulating 4% odometer scale error + 0.02 rad/s gyro bias)
            dr_dt = now_sim - self.last_dr_time
            if dr_dt > 0.0:
                vx = self.latest_gt[4] * 1.04  # 4% odometry slip
                wz = self.latest_gt[5] + 0.015 # 0.015 rad/s bias
                self.curr_dr[0] += vx * dr_dt * math.cos(self.curr_dr[2] + 0.5 * wz * dr_dt)
                self.curr_dr[1] += vx * dr_dt * math.sin(self.curr_dr[2] + 0.5 * wz * dr_dt)
                self.curr_dr[2] = normalize_angle(self.curr_dr[2] + wz * dr_dt)
                self.last_dr_time = now_sim

                # Record synchronized log
                self.timestamps.append(elapsed_sim)
                self.gt_x.append(self.latest_gt[1])
                self.gt_y.append(self.latest_gt[2])
                self.gt_yaw.append(self.latest_gt[3])

                self.dr_x.append(self.curr_dr[0])
                self.dr_y.append(self.curr_dr[1])
                self.dr_yaw.append(self.curr_dr[2])

                self.ekf_x.append(self.latest_ekf[1])
                self.ekf_y.append(self.latest_ekf[2])
                self.ekf_yaw.append(self.latest_ekf[3])
                self.ekf_sigma_x.append(self.latest_ekf[4])
                self.ekf_sigma_y.append(self.latest_ekf[5])

            rclpy.spin_once(self, timeout_sec=dt)

        # Stop the vehicle
        v_msg = Float64()
        v_msg.data = 0.0
        self.pub_vel.publish(v_msg)
        s_msg = Float64()
        s_msg.data = 0.0
        self.pub_steer.publish(s_msg)

        self.get_logger().info("Finished evaluation trajectory.")
        self.compute_and_plot_results()

    def compute_and_plot_results(self):
        t = np.array(self.timestamps)
        gt_x = np.array(self.gt_x)
        gt_y = np.array(self.gt_y)
        gt_yaw = np.array(self.gt_yaw)

        dr_x = np.array(self.dr_x)
        dr_y = np.array(self.dr_y)
        dr_yaw = np.array(self.dr_yaw)

        ekf_x = np.array(self.ekf_x)
        ekf_y = np.array(self.ekf_y)
        ekf_yaw = np.array(self.ekf_yaw)

        sig_x = np.array(self.ekf_sigma_x)
        sig_y = np.array(self.ekf_sigma_y)

        # Errors
        err_pos_dr = np.hypot(dr_x - gt_x, dr_y - gt_y)
        err_pos_ekf = np.hypot(ekf_x - gt_x, ekf_y - gt_y)

        err_yaw_dr = np.array([abs(normalize_angle(d - g)) for d, g in zip(dr_yaw, gt_yaw)])
        err_yaw_ekf = np.array([abs(normalize_angle(e - g)) for e, g in zip(ekf_yaw, gt_yaw)])

        # Error Metrics
        rmse_pos_dr = np.sqrt(np.mean(err_pos_dr**2))
        rmse_pos_ekf = np.sqrt(np.mean(err_pos_ekf**2))
        max_pos_dr = np.max(err_pos_dr)
        max_pos_ekf = np.max(err_pos_ekf)
        mean_pos_dr = np.mean(err_pos_dr)
        mean_pos_ekf = np.mean(err_pos_ekf)

        rmse_yaw_dr = np.rad2deg(np.sqrt(np.mean(err_yaw_dr**2)))
        rmse_yaw_ekf = np.rad2deg(np.sqrt(np.mean(err_yaw_ekf**2)))

        print("\n" + "="*60)
        print("         EKF LOCALIZATION BENCHMARK RESULTS")
        print("="*60)
        print(f"{'Metric':<25} | {'Dead Reckoning':<15} | {'EKF (2D LiDAR)':<15}")
        print("-"*60)
        print(f"{'Position RMSE [m]':<25} | {rmse_pos_dr:<15.4f} | {rmse_pos_ekf:<15.4f}")
        print(f"{'Mean Position Error [m]':<25} | {mean_pos_dr:<15.4f} | {mean_pos_ekf:<15.4f}")
        print(f"{'Max Position Error [m]':<25} | {max_pos_dr:<15.4f} | {max_pos_ekf:<15.4f}")
        print(f"{'Heading (Yaw) RMSE [deg]':<25} | {rmse_yaw_dr:<15.2f} | {rmse_yaw_ekf:<15.2f}")
        print("="*60)
        improvement = ((rmse_pos_dr - rmse_pos_ekf) / rmse_pos_dr) * 100.0
        print(f"Accuracy Improvement: {improvement:.1f}% reduction in position error!\n")

        # Create Plot
        fig = plt.figure(figsize=(16, 12))
        plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')

        # Subplot 1: 2D Track Trajectory
        ax1 = fig.add_subplot(2, 2, 1)
        if self.blue_cones:
            bx, by = zip(*self.blue_cones)
            ax1.scatter(bx, by, c='#0055ff', s=25, label='Blue Cones (Left Boundary)', zorder=2)
        if self.yellow_cones:
            yx, yy = zip(*self.yellow_cones)
            ax1.scatter(yx, yy, c='#ffcc00', edgecolors='#997700', s=25, label='Yellow Cones (Right Boundary)', zorder=2)

        ax1.plot(gt_x, gt_y, 'b-', linewidth=2.5, label='Ground Truth (Gazebo)', zorder=3)
        ax1.plot(dr_x, dr_y, 'r--', linewidth=1.8, label='Dead Reckoning (Drifting)', zorder=4)
        ax1.plot(ekf_x, ekf_y, 'g-', linewidth=2.0, label='EKF Localization (LiDAR Cones)', zorder=5)

        ax1.scatter(gt_x[0], gt_y[0], c='green', marker='o', s=80, label='Start Point', zorder=6)
        ax1.scatter(gt_x[-1], gt_y[-1], c='purple', marker='X', s=80, label='End Point', zorder=6)

        ax1.set_xlabel('X Position [m]', fontsize=11, fontweight='bold')
        ax1.set_ylabel('Y Position [m]', fontsize=11, fontweight='bold')
        ax1.set_title('Vehicle Trajectory on Track with 2D LiDAR Cones', fontsize=13, fontweight='bold')
        ax1.legend(loc='upper left', frameon=True, fontsize=9)
        ax1.axis('equal')
        ax1.grid(True, linestyle='--', alpha=0.6)

        # Subplot 2: Position Error vs Time
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(t, err_pos_dr, 'r--', linewidth=2.0, label=f'Dead Reckoning (RMSE: {rmse_pos_dr:.3f}m)')
        ax2.plot(t, err_pos_ekf, 'g-', linewidth=2.2, label=f'EKF Localization (RMSE: {rmse_pos_ekf:.3f}m)')
        ax2.set_xlabel('Simulation Time [s]', fontsize=11, fontweight='bold')
        ax2.set_ylabel('Position Error [m]', fontsize=11, fontweight='bold')
        ax2.set_title('Position Error: Dead Reckoning vs. EKF', fontsize=13, fontweight='bold')
        ax2.legend(loc='upper left', frameon=True, fontsize=10)
        ax2.grid(True, linestyle='--', alpha=0.6)

        # Subplot 3: Heading (Yaw) Error vs Time
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(t, np.rad2deg(err_yaw_dr), 'r--', linewidth=2.0, label=f'Dead Reckoning (RMSE: {rmse_yaw_dr:.2f}°)')
        ax3.plot(t, np.rad2deg(err_yaw_ekf), 'g-', linewidth=2.2, label=f'EKF Localization (RMSE: {rmse_yaw_ekf:.2f}°)')
        ax3.set_xlabel('Simulation Time [s]', fontsize=11, fontweight='bold')
        ax3.set_ylabel('Heading Error [deg]', fontsize=11, fontweight='bold')
        ax3.set_title('Heading (Yaw) Error Comparison', fontsize=13, fontweight='bold')
        ax3.legend(loc='upper left', frameon=True, fontsize=10)
        ax3.grid(True, linestyle='--', alpha=0.6)

        # Subplot 4: EKF Consistency & 3-Sigma Bounds
        ax4 = fig.add_subplot(2, 2, 4)
        err_x_ekf = ekf_x - gt_x
        err_y_ekf = ekf_y - gt_y
        ax4.plot(t, err_x_ekf, 'b-', label='EKF Error X [m]', linewidth=1.5)
        ax4.plot(t, 3.0 * sig_x, 'b--', alpha=0.7, label='±3σ X Bound')
        ax4.plot(t, -3.0 * sig_x, 'b--', alpha=0.7)
        ax4.plot(t, err_y_ekf, 'm-', label='EKF Error Y [m]', linewidth=1.5)
        ax4.plot(t, 3.0 * sig_y, 'm--', alpha=0.7, label='±3σ Y Bound')
        ax4.plot(t, -3.0 * sig_y, 'm--', alpha=0.7)
        ax4.set_xlabel('Simulation Time [s]', fontsize=11, fontweight='bold')
        ax4.set_ylabel('Estimation Error & 3σ Bounds [m]', fontsize=11, fontweight='bold')
        ax4.set_title('EKF Filter Consistency (3σ Confidence Bounds)', fontsize=13, fontweight='bold')
        ax4.legend(loc='upper right', frameon=True, fontsize=9)
        ax4.grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()
        out_path = '/home/ahmed/.gemini/antigravity-cli/brain/23ff0724-09c7-4c31-b609-f42998d3655a/ekf_localization_results.png'
        plt.savefig(out_path, dpi=180)
        plt.close()
        print(f"Saved evaluation plot to: {out_path}")


def main(args=None):
    rclpy.init(args=args)
    evaluator = EKFEvaluator()
    try:
        evaluator.run_evaluation(duration_sec=16.0)
    except KeyboardInterrupt:
        pass
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
