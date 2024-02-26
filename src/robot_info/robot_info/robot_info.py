import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from augv_navigation_msgs.msg import RobotInfo, RobotInfoArray
from std_msgs.msg import String




def try_get(fn, default):
    try:
        val = fn().value
        if val is None:
            return default
        else:
            return val
    except:
        return default




class RobotInfoNode(Node):
    def __init__(self):
        super().__init__('robot_info_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.robot_id = try_get(lambda: self.get_parameter("id"), 1)
        self.platform_type = try_get(lambda: self.get_parameter("platform_type"), None)
        
        self.current_pose = None
        self.current_goal = None
        
        if self.platform_type is None:
            raise Exception(f'For robot with id {self.robot_id} platform type was not set!')
        
        self.sub_current_pose = self.create_subscription(PoseStamped, f'/robot{self.robot_id}/pose', self.pose_cb, 10)
        self.sub_goal_pose = self.create_subscription(PoseStamped, f'/robot{self.robot_id}/global_planner/goal_pose', self.goal_cb, 10)
        self.sub_robot_info = self.create_subscription(RobotInfoArray, '/global/robot_info', self.robot_info_cb, 10)
        
        self.pub_robot_info = self.create_publisher(RobotInfoArray, '/global/robot_info', 10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.local_robot_info_array = RobotInfoArray()
        self.local_robot_info  = RobotInfo()
        self.local_robot_info.robot_id = self.robot_id
        
        platform_type_msg = String()
        platform_type_msg.data = self.platform_type
        self.local_robot_info.platform_type = platform_type_msg

    def pose_cb(self, pose):
        self.current_pose = pose
        if self.current_goal is None:
            self.current_goal = self.current_pose

    def goal_cb(self, goal):
        self.current_goal = goal

    def robot_info_cb(self, global_robot_info_array: RobotInfoArray):
        for i in range(len(global_robot_info_array.data)):
            current_robot_id = global_robot_info_array.data[i].robot_id
            found_current_robot = False
            for j in range(len(self.local_robot_info_array.data)):
                if self.local_robot_info_array.data[j].robot_id == current_robot_id:
                    found_current_robot = True
                    
                    current_timestamp = self.local_robot_info_array.data[j].header.stamp.sec
                    global_timestamp = global_robot_info_array.data[i].header.stamp.sec
                    
                    if global_timestamp > current_timestamp:
                        self.local_robot_info_array.data[j] = global_robot_info_array.data[i]
                    
            if not found_current_robot:
                self.local_robot_info_array.data.append(global_robot_info_array.data[i])

    def timer_callback(self):
        if self.current_pose is not None:
            self.local_robot_info.current_pose = self.current_pose
            self.local_robot_info.goal_pose = self.current_goal
            
            found_myself = False
            
            for i in range(len(self.local_robot_info_array.data)):
                if self.local_robot_info_array.data[i].robot_id == self.robot_id:
                    found_myself = True
            if not found_myself:
                self.local_robot_info_array.data.append(self.local_robot_info)
            
            self.pub_robot_info.publish(self.local_robot_info_array)
    
def main(args=None):
    rclpy.init(args=args)

    robot_info_node = RobotInfoNode()

    rclpy.spin(robot_info_node)

    robot_info_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
