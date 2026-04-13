#!/usr/bin/env python3
import argparse
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


class RTCMSerialPublisher(Node):
    def __init__(self, port: str, baudrate: int, topic: str):
        super().__init__('rtcm_serial_publisher')
        self._topic = topic
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self._pub = self.create_publisher(UInt8MultiArray, topic, 50)
        self._timer = self.create_timer(0.01, self._poll)
        self.get_logger().info(f'Reading RTCM from {port} at {baudrate} and publishing to {topic}')

    def _poll(self) -> None:
        data = self._ser.read(4096)
        if not data:
            return
        msg = UInt8MultiArray()
        msg.data = list(data)
        self._pub.publish(msg)

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--topic', default='/rtcm_land')
    args = parser.parse_args()

    rclpy.init()
    node = RTCMSerialPublisher(args.port, args.baudrate, args.topic)
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
