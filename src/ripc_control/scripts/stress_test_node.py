import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
import subprocess

class LongDurationStressTest(Node):
    def __init__(self):
        super().__init__('long_stress_test')
        
        # 1. Settings
        self.test_duration = 600.0  
        self.target_amps = 20.0
        self.max_amps_config = 30.0 
        
        # 2. Safety Thresholds
        self.max_esc_temp = 30.0    
        self.max_mot_temp = 90.0    
        
        # Calculate the x value needed to hit 40A
        self.linear_x = self.target_amps / self.max_amps_config

        # Internal tracking for logging
        self.current_rpm = 0.0
        self.current_esc_temp = 0.0

        # 3. Publishers & Subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Float32, 'vesc/starboard/temp_esc', self.esc_temp_callback, 10)
        self.create_subscription(Float32, 'vesc/starboard/temp_motor', self.mot_temp_callback, 10)
        self.create_subscription(Float32, 'vesc/starboard/rpm', self.rpm_callback, 10)

        # 4. Start Rosbag automatically (Added RPM to the list)
        self.bag_process = subprocess.Popen([
            'ros2', 'bag', 'record', '-o', f'long_test_40A_{int(time.time())}',
            '/vesc/starboard/amps', 
            '/vesc/starboard/temp_esc', 
            '/vesc/starboard/temp_motor', 
            '/vesc/starboard/rpm', 
        ])

        self.get_logger().info(f'--- STARTING 10 MINUTE TEST AT {self.target_amps}A ---')
        self.start_time = time.time()
        
        self.timer = self.create_timer(0.1, self.run_test)

    def rpm_callback(self, msg):
        self.current_rpm = msg.data

    def esc_temp_callback(self, msg):
        self.current_esc_temp = msg.data
        if msg.data > self.max_esc_temp:
            self.get_logger().error(f'!!! ESC OVERHEAT: {msg.data}C !!! Emergency Stop.')
            self.stop_motors()

    def mot_temp_callback(self, msg):
        if msg.data > self.max_mot_temp:
            self.get_logger().error(f'!!! MOTOR OVERHEAT: {msg.data}C !!! Emergency Stop.')
            self.stop_motors()

    def run_test(self):
        elapsed = time.time() - self.start_time
        
        if elapsed < self.test_duration:
            msg = Twist()
            msg.linear.x = self.linear_x
            self.publisher.publish(msg)
            
            # Expanded Logging: See RPM and Temp in real-time
            if int(elapsed) % 10 == 0:
                mins, secs = divmod(int(self.test_duration - elapsed), 60)
                self.get_logger().info(
                    f'[{mins:02d}:{secs:02d}] | '
                    f'RPM: {int(self.current_rpm)} | '
                    f'ESC: {self.current_esc_temp:.1f}C'
                )
        else:
            self.get_logger().info('Test Completed Successfully!')
            self.stop_motors()

    def stop_motors(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.publisher.publish(msg)
        self.bag_process.terminate()
        self.get_logger().info('Motors Stopped. Bag Saved.')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LongDurationStressTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_motors()

if __name__ == '__main__':
    main()