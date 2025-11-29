#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Time

import numpy as np

class TwistFromPose(Node):
    def __init__(self):
        super().__init__('wamv_twist_from_pose')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/wamv/pose_clean", 10)
        self.twist_pub = self.create_publisher(Twist, "/wamv/estimated_twist", 10)

        # Internal state
        self.prev_pose = None
        self.prev_stamp = None

        # Timer to poll TF at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

        # Frames VRX uses
        self.base_frame = "wamv/base_link"
        self.world_frame = "world"

        self.get_logger().info("TwistFromPose running... listening to TF for WAM-V pose.")

    def timer_callback(self):
        try:
            # Lookup transform world → base_link
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            return  # No TF yet

        # Extract pose
        pose = PoseStamped()
        pose.header.stamp = tf.header.stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation

        # Publish clean pose
        self.pose_pub.publish(pose)

        # Compute twist if previous exists
        current_time = float(tf.header.stamp.sec) + float(tf.header.stamp.nanosec) * 1e-9

        if self.prev_pose is not None:
            dt = current_time - self.prev_stamp
            if dt > 0:

                dx = pose.pose.position.x - self.prev_pose.pose.position.x
                dy = pose.pose.position.y - self.prev_pose.pose.position.y
                dz = pose.pose.position.z - self.prev_pose.pose.position.z

                twist = Twist()
                twist.linear.x = dx / dt
                twist.linear.y = dy / dt
                twist.linear.z = dz / dt

                self.twist_pub.publish(twist)

        # Store new state
        self.prev_pose = pose
        self.prev_stamp = current_time


def main(args=None):
    rclpy.init(args=args)
    node = TwistFromPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
