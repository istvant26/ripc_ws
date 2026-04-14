#!/usr/bin/env python3
"""
Proper RTCM Message Type Decoder
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

class RTCMDecoder(Node):
    def __init__(self):
        super().__init__('rtcm_decoder')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/rtcm_land',
            self.callback,
            10
        )
        self.types_seen = {}
    
    def callback(self, msg):
        data = msg.data
        i = 0
        
        while i < len(data) - 5:
            # Look for RTCM header (0xD3)
            if data[i] == 0xD3:
                # RTCM3 frame structure:
                # Byte 0: 0xD3
                # Byte 1-2: Length (10 bits) + Reserved (6 bits)
                # Byte 3-4: Message Type (12 bits)
                
                if i + 5 > len(data):
                    break
                
                # Extract length (bits 0-9 of bytes 1-2)
                length = ((data[i+1] & 0x03) << 8) | data[i+2]
                
                # Extract message type (12 bits across bytes 3-4)
                msg_type = (data[i+3] << 4) | (data[i+4] >> 4)
                
                if msg_type not in self.types_seen:
                    self.types_seen[msg_type] = 0
                    self.get_logger().info(f'NEW RTCM Type: {msg_type} (length: {length})')
                
                self.types_seen[msg_type] += 1
                
                # Skip to next potential message
                i += 3 + length + 3  # header + payload + CRC
            else:
                i += 1

def main():
    rclpy.init()
    node = RTCMDecoder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nRTCM Message Summary:")
        for msg_type in sorted(node.types_seen.keys()):
            print(f"  Type {msg_type}: {node.types_seen[msg_type]} messages")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()