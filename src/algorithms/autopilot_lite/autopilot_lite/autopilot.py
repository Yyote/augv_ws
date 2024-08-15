import rclpy
from rclpy.node import Node

from astar_planner_msgs.msg import GlobalTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from augv_navigation_msgs.msg import Position

def try_get(fn, default):
    try:
        val = fn().value
        if val is None:
            return default
        else:
            return val
    except:
        return default



class AutopilotLiteNode(Node):
    def __init__(self):
        super().__init__('autopilot_lite_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.robot_id = try_get(lambda: self.get_parameter("id"), 1)
        self.robot_name = try_get(lambda: self.get_parameter("name"), "robot")
        self.odom_prefix = try_get(lambda: self.get_parameter("odom_prefix"), "")
        self.sub_trajectory = self.create_subscription(GlobalTrajectory, f'/{self.robot_name}{self.robot_id}/trajectory', self.trajectory_cb, 10)
        self.sub_pose = self.create_subscription(PoseStamped, f'/{self.robot_name}{self.robot_id}/pose', self.pose_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, f'/{self.robot_name}{self.robot_id}{self.odom_prefix}/odom', self.odom_cb, 10)
        
        self.goal_pub = self.create_publisher(PoseStamped, f'/{self.robot_name}{self.robot_id}/goal_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.curr_wp = None
        self.current_pose = None
        self.current_trajectory = None

    def trajectory_cb(self, trajectory: GlobalTrajectory):
        self.curr_wp = None
        self.current_trajectory = trajectory.waypoints
    
    def pose_cb(self, pose: PoseStamped):
        self.current_pose = pose
    
    def odom_cb(self, odom: Odometry):
        pose = PoseStamped()
        pose.pose = odom.pose
        pose.header = odom.header
        self.current_pose = pose

    def timer_cb(self):
        if self.current_trajectory is not None and len(self.current_trajectory) > 0 and self.current_pose is not None:
            if self.curr_wp is not None:
                dx = self.curr_wp.pose.position.x - self.current_pose.pose.position.x
                dy = self.curr_wp.pose.position.y - self.current_pose.pose.position.y
                dz = self.curr_wp.pose.position.z - self.current_pose.pose.position.z
                
                dr = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
                print(f"dr = {dr}")
                if dr < 1.0:
                    self.curr_wp = self.current_trajectory[0] 
                    self.goal_pub.publish(self.curr_wp) ## Отправить текущую цель
                    del self.current_trajectory[0]
            else:
                self.curr_wp = self.current_trajectory[0] 
                self.goal_pub.publish(self.curr_wp) ## Отправить текущую цель
                del self.current_trajectory[0]


def main(args=None):
    rclpy.init(args=args)

    autopilot_lite_node = AutopilotLiteNode()

    rclpy.spin(autopilot_lite_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    autopilot_lite_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
