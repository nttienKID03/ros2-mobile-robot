#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math
import time

class MovingObstacle(Node):
    def __init__(self):
        super().__init__('moving_obstacle')
        self.cli = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.get_logger().info("⏳ Waiting for /gazebo/set_model_state service...")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for /gazebo/set_model_state service...")

        self.get_logger().info("✅ Connected to /gazebo/set_model_state")
        self.timer = self.create_timer(0.1, self.move_obstacle)
        self.t0 = time.time()

    def move_obstacle(self):
        t = time.time() - self.t0
        # Dao động giữa -2.5 và 0.5 (biên độ = 1.5, trung tâm = -1)
        x = -1.0 + 1.5 * math.sin(0.3 * t)
        y = 5.0
        z = 0.35

        state = ModelState()
        state.model_name = 'obstacle_moving'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z

        req = SetModelState.Request()
        req.model_state = state
        self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
