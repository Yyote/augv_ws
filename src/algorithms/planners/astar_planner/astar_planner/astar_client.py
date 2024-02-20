import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point
from astar_planner_msgs.srv import GetTrajectory

class AstarClientNode(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.astar_client = self.create_client(GetTrajectory, '/astar/get_trajectory')
        self.sub_goal = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        while not self.astar_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetTrajectory.Request()

    def send_request(self, goal):
        self.req.goal_position = goal
        self.future = self.astar_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def goal_cb(self, goal):
        response = self.send_request(goal=goal)
        print(response)

def main(args=None):
    rclpy.init(args=args)

    node = AstarClientNode()

    while rclpy.ok():
        rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()