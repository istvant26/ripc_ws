#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

class LandBaseSerialNode(Node):
    def __init__(self):
        super().__init__('land_base_serial')
        
        self.port = '/dev/ttyACM0'  # Your base GPS
        self.baud = 115200
        
        self.publisher = self.create_publisher(UInt8MultiArray, '/rtcm_land', 10)
        
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'✓ Connected to base GPS on {self.port}')
        except Exception as e:
            self.get_logger().error(f'✗ Failed to open {self.port}: {e}')
            exit(1)
        
        # Poll at 100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.msg_count = 0
    
    def timer_callback(self):
        if self.ser.in_waiting > 0:
            raw_bytes = self.ser.read(self.ser.in_waiting)
            
            msg = UInt8MultiArray()
            msg.data = list(raw_bytes)
            self.publisher.publish(msg)
            
            self.msg_count += 1
            if self.msg_count % 100 == 0:
                self.get_logger().info(f'Published {self.msg_count} RTCM packets')
    
    def __del__(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = LandBaseSerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()