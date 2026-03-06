#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import csv
import math


# ===================== CONFIG =====================
# 👉 CHỌN 1 TRONG 2
USE_ODOM = False
USE_AMCL = True
# ================================================


# ===================== UTIL =====================

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


# ===================== NODE =====================

class CsvLoggerNode(Node):

    def __init__(self):
        super().__init__('csv_logger_node')

        if USE_ODOM and USE_AMCL:
            self.get_logger().fatal(
                "Only ONE pose source allowed: ODOM or AMCL"
            )
            raise RuntimeError("Invalid pose source config")

        # ===== CSV =====
        self.csv_path = 'chay_real_amcl_5a4a5b_lan1_co.csv'
        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time',
            'x', 'y', 'yaw',
            'linear_vel', 'angular_vel',
            'error_x', 'error_y',
            'yaw_error'
        ])

        # ===== GOAL =====
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        # ===== QoS =====
        qos_odom = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ===== SUBSCRIPTIONS =====
        if USE_ODOM:
            self.create_subscription(
                Odometry,
                '/odometry/filtered',
                self.odom_callback,
                qos_odom
            )
            self.get_logger().info("Pose source: ODOM")

        if USE_AMCL:
            self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.amcl_callback,
                10
            )
            self.get_logger().info("Pose source: AMCL")

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.get_logger().info(f'CSV Logger STARTED → {self.csv_path}')

    # ===================== CALLBACKS =====================

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = quaternion_to_yaw(msg.pose.orientation)

    def odom_callback(self, msg: Odometry):
        if not USE_ODOM:
            return

        t = self.get_time()

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z

        error_x, error_y, yaw_error = self.compute_error(x, y, yaw)

        self.write_csv(
            t, x, y, yaw,
            linear_vel, angular_vel,
            error_x, error_y, yaw_error
        )

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        if not USE_AMCL:
            return

        t = self.get_time()

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        # AMCL không có velocity
        linear_vel = float('nan')
        angular_vel = float('nan')

        error_x, error_y, yaw_error = self.compute_error(x, y, yaw)

        self.write_csv(
            t, x, y, yaw,
            linear_vel, angular_vel,
            error_x, error_y, yaw_error
        )

    # ===================== HELPERS =====================

    def get_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def compute_error(self, x, y, yaw):
        if self.goal_x is None:
            return float('nan'), float('nan'), float('nan')

        error_x = self.goal_x - x
        error_y = self.goal_y - y
        yaw_error = wrap_to_pi(self.goal_yaw - yaw)

        return error_x, error_y, yaw_error

    def write_csv(self, t, x, y, yaw,
                  linear_vel, angular_vel,
                  error_x, error_y, yaw_error):

        self.writer.writerow([
            f'{t:.6f}',
            f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}',
            f'{linear_vel:.6f}', f'{angular_vel:.6f}',
            f'{error_x:.6f}', f'{error_y:.6f}',
            f'{yaw_error:.6f}'
        ])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


# ===================== MAIN =====================

def main(args=None):
    rclpy.init(args=args)
    node = CsvLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

