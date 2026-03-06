#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import yaml
import os
import time

class GoalMarkerPublisher(Node):
    def __init__(self):
        super().__init__('goal_marker_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        yaml_path = "/home/nttien/ros2_ws/src/mobile/scripts/predefined_goals.yaml"
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"File not found: {yaml_path}")
            return

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        self.goals = data.get('goals', [])
        self.timer = self.create_timer(2.0, self.publish_all_markers)
        self.get_logger().info(f"Loaded {len(self.goals)} goals to visualize.")

    def publish_all_markers(self):
        for i, goal in enumerate(self.goals):
            self.publish_marker(goal, i)

    def publish_marker(self, goal, idx):
        name = goal.get('name', f"Goal {idx+1}")
        pos = goal.get('position', [0, 0, 0])
        frame = goal.get('frame_id', 'map')

        # --- Sphere marker ---
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "predefined_goals"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = 0.1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.1
        self.publisher.publish(marker)

        # --- Text marker ---
        text_marker = Marker()
        text_marker.header.frame_id = frame
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "goal_labels"
        text_marker.id = 100 + idx
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = pos[0]
        text_marker.pose.position.y = pos[1]
        text_marker.pose.position.z = 0.0
        text_marker.scale.z = 0.35
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 0.1
        text_marker.text = name
        self.publisher.publish(text_marker)

def main(args=None):
    rclpy.init(args=args)
    node = GoalMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
