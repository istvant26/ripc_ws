import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = [(0,0,0),(8,0,0),(8,5,1.57),(0,5,3.14)]
        self.create_timer(2.0, self._run)

    def _run(self):
        if not self._action_client.server_is_ready():
            self.get_logger().info('Waiting for NavigateToPose action server...')
            return
        self.get_logger().info('Starting mission waypoints...')
        for x,y,yaw in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            q = quaternion_from_euler(0,0,float(yaw))
            pose.pose.orientation.x = q[0]; pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]; pose.pose.orientation.w = q[3]
            goal = NavigateToPose.Goal()
            goal.pose = pose
            send_goal = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal)
            goal_handle = send_goal.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                continue
            self.get_logger().info('Goal accepted, waiting result...')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            res = result_future.result()
            self.get_logger().info('Waypoint reached or finished.')
        self.get_logger().info('Mission complete.')

def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
