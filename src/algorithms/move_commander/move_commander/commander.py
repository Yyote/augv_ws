import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from astar_planner_msgs.msg import GlobalTrajectory


def try_get(fn, default):
    try:
        val = fn().value
        if val is None:
            return default
        else:
            return val
    except:
        return default



class CommanderNode(Node):

    def __init__(self):
        super().__init__('move_commander', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.robot_id = try_get(lambda: self.get_parameter("id"), 1)
        self.robot_name = try_get(lambda: self.get_parameter("name"), "robot")
        self.odom_prefix = try_get(lambda: self.get_parameter("odom_prefix"), "")

        self.sub_odom = self.create_subscription(Odometry, f'/{self.robot_name}{self.robot_id}{self.odom_prefix}/odom', self.odom_cb, 10)

        self.current_odom = Odometry()

    def odom_cb(self, odom: Odometry):
        self.current_odom = odom


def main(args=None):
    rclpy.init(args=args)

    node = CommanderNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
