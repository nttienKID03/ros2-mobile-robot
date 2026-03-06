#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')

        # ---- Parameters ----
        self.declare_parameter('input_topic', '/scan ')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('min_radius', 0.05)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.min_radius = self.get_parameter('min_radius').get_parameter_value().double_value

        # ---- Subscribers / Publishers ----
        self.sub_scan = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10
        )
        self.pub_filtered = self.create_publisher(LaserScan, output_topic, 10)

        self.get_logger().info(
            f'Lidar filter active: filtering radius < {self.min_radius} m\n'
            f'Input: {input_topic}\nOutput: {output_topic}'
        )

    def scan_callback(self, scan_msg: LaserScan):
        # Copy header & meta info
        filtered_scan = LaserScan()
        filtered_scan.header = scan_msg.header
        filtered_scan.angle_min = scan_msg.angle_min
        filtered_scan.angle_max = scan_msg.angle_max
        filtered_scan.angle_increment = scan_msg.angle_increment
        filtered_scan.time_increment = scan_msg.time_increment
        filtered_scan.scan_time = scan_msg.scan_time
        filtered_scan.range_min = scan_msg.range_min
        filtered_scan.range_max = scan_msg.range_max

        # ---- Filter ----
        filtered_ranges = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if math.isfinite(r) and r < self.min_radius:
                filtered_ranges.append(float('inf'))
            else:
                filtered_ranges.append(r)
            angle += scan_msg.angle_increment

        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = scan_msg.intensities

        # ---- Publish ----
        self.pub_filtered.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
