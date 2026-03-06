#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm

class PointCloudTF(Node):
    def __init__(self):
        super().__init__('pc_tf_node')

        self.target_frame = 'base_footprint'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ❗ KHÔNG TRUYỀN QoS → dùng mặc định (RELIABLE)
        self.sub = self.create_subscription(
            PointCloud2,
            '/cam_xa_sensor/points',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/cam_xa/points_base',
            10
        )

        self.get_logger().info('PointCloud TF node started')

    def cb(self, msg):
        self.get_logger().error('CALLBACK HIT')

        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )

            out = tf2_sm.do_transform_cloud(msg, trans)
            out.header.frame_id = self.target_frame
            out.header.stamp = msg.header.stamp
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(str(e))


def main():
    rclpy.init()
    node = PointCloudTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
