#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ublox_msgs.msg import RxmRTCM
from std_msgs.msg import UInt8MultiArray
import sys

class RTCMBridge(Node):
    def __init__(self):
        super().__init__('rtcm_bridge')
        self.get_logger().info('Bridge Node Initializing...')
        
        self.subscription = self.create_subscription(
            RxmRTCM,
            '/rxmrtcm_raw',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            UInt8MultiArray, 
            '/rtcm_land', 
            10)
        
        self.get_logger().info('Bridge Node Ready: /rxmrtcm_raw -> /rtcm_land')

    def listener_callback(self, msg):
        # This converts the message
        new_msg = UInt8MultiArray()
        new_msg.data = msg.data
        self.publisher.publish(new_msg)

def main(args=None):
    print("Main function started...")
    rclpy.init(args=args)
    try:
        bridge = RTCMBridge()
        print("Spinning node...")
        rclpy.spin(bridge)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Shutting down...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()