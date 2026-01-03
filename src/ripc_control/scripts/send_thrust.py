#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # supported by ros_gz_bridge

class RearThrustPublisher(Node):
    def __init__(self):
        super().__init__('diff_thrust_publisher')

        # Publishers for left and right rear thrusters
        self.pub_left = self.create_publisher(Float64, '/ripc_usv/thrusters/left/thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/ripc_usv/thrusters/right/thrust', 10)

        # Example thrust values
        self.thrust_left = 50.0
        self.thrust_right = 50.0

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_thrust)

    def publish_thrust(self):
        self.pub_left.publish(Float64(data=self.thrust_left))
        self.pub_right.publish(Float64(data=self.thrust_right))
        self.get_logger().info(f'Publishing thrusts -> Left: {self.thrust_left}, Right: {self.thrust_right}')

def main(args=None):
    rclpy.init(args=args)
    node = RearThrustPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
