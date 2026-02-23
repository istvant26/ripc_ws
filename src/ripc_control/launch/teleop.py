#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ThrusterMixer(Node):
    def __init__(self):
        super().__init__('thruster_mixer')
        
        # Adjust this value if the boat moves too slow or too fast
        self.thrust_multiplier = 100.0 
        
        # Subscribes to standard teleop
        self.subscription = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.listener_callback, 
            10)
        
        # Publishers for Gazebo thruster plugins
        self.left_pub = self.create_publisher(Float64, '/ripc_usv/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/ripc_usv/thrusters/right/thrust', 10)
        
        self.get_logger().info(f"Thruster Mixer started with multiplier: {self.thrust_multiplier}")

    def listener_callback(self, msg):
        forward = msg.linear.x
        turn = msg.angular.z
        
        # Differential mixing logic
        left_val = (forward - turn) * self.thrust_multiplier
        right_val = (forward + turn) * self.thrust_multiplier
        
        # Create message objects
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = float(left_val)
        right_msg.data = float(right_val)
        
        # Publish
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        
        self.get_logger().info(f"Thrust L: {left_val:.2f} | R: {right_val:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()