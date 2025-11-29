

# scripts/wamv_motion_cmd.py
"""
ROS 2 Python node to command the WAM-V thrusters to execute a predefined
sequence of maneuvers useful for hydrodynamic validation:
1) Surge steady-thrust steps (different thrust levels)
2) Free-decay (thrust=0) to capture damping
3) Turning circle using differential thrust
4) Zig-zag: alternating differential thrust pulses


This node publishes std_msgs/Float64 messages to the bridged thruster topics:
/wamv/thrusters/left/thrust
/wamv/thrusters/right/thrust


Usage:
ros2 run <your_package> wamv_motion_cmd.py


Or run directly (if executable bit set):
python3 scripts/wamv_motion_cmd.py


Note: Ensure the bridge launch is running (so the topics exist in ROS 2).
"""


from platform import node
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time


class WamvMotionCommander(Node):
 def __init__(self):
        super().__init__('wamv_motion_commander')
        self.left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # Parameters (tweak these for your vehicle)
        self.surge_steps = [50.0, 100.0, 200.0] # Newtons (example)
        self.step_duration = 20.0 # seconds at each steady thrust
        self.settle_time = 5.0 # seconds to wait before measurement
        self.free_decay_time = 15.0 # seconds of thrust=0 to capture decay

        self.turn_thrust = 80.0 # differential thrust for turning
        self.turn_duration = 30.0 # seconds for turning circle

        self.zigzag_thrust = 120.0 # magnitude for zig-zag pulses
        self.zigzag_pulse = 4.0 # seconds per pulse
        self.zigzag_switches = 6 # number of pulses per side


        # Start the sequence in a separate thread (non-blocking is fine for simple scripts)
        self.get_logger().info('WAM-V motion commander ready. Starting maneuver sequence...')
        self.execute_sequence()


 def publish_thrust(self, left, right):
   lmsg = Float64()
   rmsg = Float64()
   lmsg.data = float(left)
   rmsg.data = float(right)
   self.left_pub.publish(lmsg)
   self.right_pub.publish(rmsg)
   self.get_logger().info(f'Published thrust L={left:.1f} N, R={right:.1f} N')

def execute_sequence(self):
    # 1) Surge steady-step sequence
    for T in self.surge_steps:
        self.get_logger().info(f'Starting surge step with thrust {T} N')
        t0 = time.time()
        while (time.time() - t0) < self.step_duration:
            self.publish_thrust(T, T)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)


        # Allow settle time and then set thrust to zero briefly to capture free decay
        self.get_logger().info('Settling...')
        time.sleep(self.settle_time)
        self.get_logger().info('Free decay: setting thrust to 0')
        self.publish_thrust(0.0, 0.0)
        tfd = time.time()
        while (time.time() - tfd) < self.free_decay_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1) 

        # 2) Turning circle via differential thrust
        self.get_logger().info('Starting turning circle maneuver')
        t0 = time.time()
        while (time.time() - t0) < self.turn_duration:
            # left forward, right reverse -> yaw
            self.publish_thrust(self.turn_thrust, -self.turn_thrust)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)     

        # stop
        self.publish_thrust(0.0, 0.0)
        time.sleep(2.0)

        # 3) Zig-zag maneuver (alternating differential pulses)
        self.get_logger().info('Starting zig-zag maneuver')
        for i in range(self.zigzag_switches):
            # starboard pulse
            self.publish_thrust(self.zigzag_thrust, -self.zigzag_thrust)
            t0 = time.time()
            while (time.time() - t0) < self.zigzag_pulse:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)

        # port pulse
        self.publish_thrust(-self.zigzag_thrust, self.zigzag_thrust)
        t0 = time.time()
        while (time.time() - t0) < self.zigzag_pulse:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)


        # final stop
        self.publish_thrust(0.0, 0.0)
        self.get_logger().info('Maneuver sequence complete. Node will keep running (ctrl-c to exit).')




def main(args=None):
    rclpy.init(args=args)
    node = WamvMotionCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ensure thrusters are zeroed on shutdown
        node.publish_thrust(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()