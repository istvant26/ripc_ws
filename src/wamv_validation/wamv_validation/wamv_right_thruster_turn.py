#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class WamvRightThrusterTurn(Node):
    def __init__(self, thrust=100.0, on_time=16.25, off_time=10.0):
        super().__init__('wamv_right_thruster_turn')

        # Thruster publishers
        self.left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # Parameters
        self.thrust = thrust
        self.on_time = on_time
        self.off_time = off_time

        # Timing and state
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.state = "ON"  # Start immediately

        # 50 Hz timer
        self.timer = self.create_timer(0.02, self.publish_thrust)
        self.get_logger().info("Right-thruster turn node started. ON/OFF cycle begins immediately.")

    def publish_thrust(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.start_time

        # --- STATE MACHINE ---
        if self.state == "ON" and elapsed > self.on_time:
            self.get_logger().info("Switching thrust OFF.")
            self.state = "OFF"
            self.start_time = now  # reset timer for OFF period
        elif self.state == "OFF" and elapsed > self.off_time:
            self.get_logger().info("ON/OFF cycle complete. Shutting down.")
            self.left_pub.publish(Float64(data=0.0))
            self.right_pub.publish(Float64(data=0.0))
            self.destroy_node()
            return

        # --- PUBLISH THRUST ---
        if self.state == "ON":
            left_cmd = 0.0
            right_cmd = self.thrust
        else:
            left_cmd = 0.0
            right_cmd = 0.0

        # Warn if no subscribers yet
        if self.left_pub.get_subscription_count() == 0 or self.right_pub.get_subscription_count() == 0:
            self.get_logger().warn("Thruster topics not yet subscribed. Publishing anyway.")

        self.left_pub.publish(Float64(data=left_cmd))
        self.right_pub.publish(Float64(data=right_cmd))

def main(args=None):
    rclpy.init(args=args)
    node = WamvRightThrusterTurn(thrust=100.0, on_time=16.25, off_time=10.0)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
