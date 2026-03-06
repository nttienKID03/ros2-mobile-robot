import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from PyQt5.QtCore import QThread, pyqtSignal

from tracking_model import TrackingModel


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class RosWorker(QThread):
    signal_snapshot = pyqtSignal(dict)
    signal_mode = pyqtSignal(str)

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.tracking = TrackingModel()
        self.running = True

    def run(self):
        last_emit = time.time()

        while rclpy.ok() and self.running:
            rclpy.spin_once(self.node, timeout_sec=0.01)

            now = time.time()
            if now - last_emit >= 1.0 / 15.0:  # 15 Hz
                self.signal_snapshot.emit(self.tracking.snapshot())
                last_emit = now

    def stop(self):
        self.running = False

    # ---------- ROS callbacks ----------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q)

        self.tracking.update_pose(x, y, yaw)
        self.tracking.update_velocity(
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        )

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        self.tracking.update_velocity(v, w)

        if abs(v) > 0.01 or abs(w) > 0.01:
            self.signal_mode.emit("MOTION")
        else:
            self.signal_mode.emit("IDLE")
