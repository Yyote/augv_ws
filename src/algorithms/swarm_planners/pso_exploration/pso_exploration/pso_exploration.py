import numpy as np
from sys import maxsize
import os
import sys
import cv2
import math
from cv2 import CV_8UC1, line

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from augv_navigation_msgs.msg import PsoRobotStatus

np.set_printoptions(threshold=sys.maxsize)


def try_get(fn, default):
    try:
        val = fn().value
        if val is None:
            return default
        else:
            return val
    except:
        return default








class Map:
    def __init__(self):
        self.grid_map_ = OccupancyGrid()

    def occupancygrid_to_numpy(msg):
        data = np.asarray(msg.data, dtype=np.uint8).reshape(
            msg.info.height, msg.info.width
        )
        # print(np.unique(data, axis=0, return_counts=True))

        return np.ma.array(data, mask=data == 0, fill_value=-1)

    def world_to_map(self, x: int, y: int):
        """
        Get map coordinates
        """
        mx = int(
            (x - self.grid_map_.info.origin.position.x) / self.grid_map_.info.resolution
        )
        my = int(
            (y - self.grid_map_.info.origin.position.y) / self.grid_map_.info.resolution
        )
        return [mx, my]

    def get_index(self, x: int, y: int):
        """
        Get index in map
        """
        return x + y * self.grid_map_.info.width

    def get_index_in_map_from_world_coords(self, x, y):
        """
        Returns index of point in world map to occupancy_grid
        """
        xm, ym = self.world_to_map(x, y)
        return self.get_index(xm, ym)

    def get_coords_from_grid_index(self, i):
        """
        Get coords using index in 1D occupancy grid
        """
        y = divmod(i, self.grid_map_.info.width)[0]
        x = i - y * self.grid_map_.info.width

        x = (
            x * self.grid_map_.info.resolution
            + self.grid_map_.info.origin.position.x
            + self.grid_map_.info.resolution / 2.0
        )
        y = (
            y * self.grid_map_.info.resolution
            + self.grid_map_.info.origin.position.y
            + self.grid_map_.info.resolution / 2.0
        )

        return y, x


def check_if_point_in_polygon(vertices, point):
    vertices_num = len(vertices)
    start_sign = 1
    for i in range(vertices_num - 1):
        next_vertices = i + 1
        if i == vertices_num - 1:
            next_vertices = 0

        vector1 = Point(
            vertices[next_vertices].x - vertices[i].x,
            vertices[next_vertices].y - vertices[i].y,
            vertices[next_vertices].z - vertices[i].z,
        )

        vector2 = Point(
            point.x - vertices[i].x, point.y - vertices[i].y, point.z - vertices[i].z
        )

        angle = math.acos(
            (vector1.x * vector2.x + vector1.y * vector2.y)
            / (
                math.sqrt(vector1.x**2 + vector1.y**2)
                * math.sqrt(vector2.x**2 + vector2.y**2)
            )
        )
        if angle == 0:
            return False

        if i == 0:
            start_sign = angle / abs(angle)
        else:
            if (angle / abs(angle) - start_sign) == 0:
                return False
    return True














class ModifiedPSOExplorer(Node):
    def __init__(self):
        self.node_name = "modified_pso_explorer"
        super().__init__(
            self.node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        
        self.robot_id = try_get(lambda: self.get_parameter("id"), 1)
        self.cell_evaluation_radius = try_get(lambda: self.get_parameter("cell_evaluation_radius"), 4)
        self.neighbour_evaluation_radius = try_get(lambda: self.get_parameter("neighbour_evaluation_radius"), 10)
        self.current_point = None # for first summand in pso
        self.current_fitness_value = 0
        self.best_fitness_position = None 
        self.best_fitness_value = 0
        self.best_neighbour_position = None
        self.best_neighbour_fitness_value = 0

        self.velocity_dampening_coefficient = 0.9
        
        self.global_map_ = None
        self.exploration_field_ = None
        self.key_points = None
        self.need_exploration = True
        self.current_pose = None
        self.robot_info_array = None

        goal_pose_topic = f"/robot{self.robot_id}/global_planner/goal_pose"  # self.get_parameter('goal_topic').value

        self.create_subscription(PsoRobotStatus, "/global/pso_status", self.pso_status_cb, 10)
        self.create_subscription(PoseStamped, f'/robot{self.robot_id}/pose', self.pose_cb, 10)
        self.create_subscription(
            OccupancyGrid, "/OccupancyGrid_map", self.global_map_clb, 10
        )  # qos_profile_sensor_data)
        self.create_subscription(
            Bool, "exploration_need", self.exploration_need_clb, 10
        )

        self.pub_countours_points = self.create_publisher(
            MarkerArray, "edges", qos_profile_sensor_data
        )
        self.pub_goal_point = self.create_publisher(PoseStamped, goal_pose_topic, 10)
        self.pso_status_pub = self.create_publisher(PsoRobotStatus, "/global/pso_status", 10)
        
        self.create_timer(5, self.exploration_loop)
        self.create_timer(1, self.publish_pso_status)
        self.create_timer(10, self.zero_neighbour_fitness_value)

    def pso_status_cb(self, status: PsoRobotStatus):
        if status.robot_id == self.robot_id:
            return
        dx = status.current_pose.pose.position.x - self.current_pose.pose.position.x
        dy = status.current_pose.pose.position.y - self.current_pose.pose.position.y
        dr = (dx ** 2 + dy ** 2) ** 0.5
        if dr < self.neighbour_evaluation_radius:
            if status.current_fitness_value > self.best_neighbour_fitness_value:
                self.best_neighbour_position = status.current_pose

    def pose_cb(self, pose):
        self.current_pose = pose

    def exploration_need_clb(self, msg: Bool):
        # print("here")
        # self.need_exploration = msg.data
        pass

    def publish_pso_status(self):
        if self.best_fitness_position is None:
            return
        if self.current_pose is None:
            return
        
        msg = PsoRobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.current_fitness_value = self.current_fitness_value
        msg.robot_id = self.robot_id
        msg.current_pose = self.current_pose
        self.pso_status_pub.publish(msg)

    def zero_neighbour_fitness_value(self):
        self.best_neighbour_fitness_value = 0
        self.best_neighbour_position = None

    def display_countours(self, trajectory: list):

        waypoints = MarkerArray()

        for i in range(len(trajectory)):
            waypoint = Marker()
            waypoint.id = i
            waypoint.type = Marker.CUBE
            waypoint.header.frame_id = "map"
            waypoint.action = Marker.ADD
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.ns = "path"

            waypoint.color.a = 1.0
            waypoint.color.b = 0.8
            waypoint.color.g = 0.7
            waypoint.color.r = 0.5

            waypoint.scale.x = 0.1
            waypoint.scale.y = 0.1
            waypoint.scale.z = 0.1

            # waypoint.pose.position.y = (float(trajectory[i][0]) - float(self.global_map_.info.height) / 2) * self.global_map_.info.resolution - 20.0
            # waypoint.pose.position.x = (float(trajectory[i][1]) - float(self.global_map_.info.width) / 2) * self.global_map_.info.resolution

            waypoint.pose.position.y = (
                (float(trajectory[i][0])) * self.global_map_.info.resolution
                + self.global_map_.info.origin.position.y
            )
            waypoint.pose.position.x = (
                (float(trajectory[i][1])) * self.global_map_.info.resolution
                + self.global_map_.info.origin.position.x
            )
            waypoint.pose.position.z = 1.0
            waypoints.markers.append(waypoint)

        self.pub_countours_points.publish(waypoints)

    def find_countour_with_most_point_intensivity(self, countours: list):
        max_points = 0
        contour_id = None
        for i in range(len(countours)):
            points_in_contour = len(countours[i])
            if points_in_contour > max_points:
                max_points = points_in_contour
                contour_id = i
        return contour_id

    def get_center_of_mass(self, points):
        center_of_mass_x = 0
        center_of_mass_y = 0

        for point in points:
            # print(point[0])
            center_of_mass_x += point[0][0]
            center_of_mass_y += point[0][1]
        center_of_mass_x = center_of_mass_x / len(points)
        center_of_mass_y = center_of_mass_y / len(points)

        return center_of_mass_x, center_of_mass_y

    def global_map_clb(self, map):
        self.global_map_ = map

    def find_dist(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def find_most_lenght_line(self, lines):
        max_lenght = 0.0
        i = 0
        idx = 0
        for line in lines:
            line = line[0]
            dist = self.find_dist(line[0], line[1], line[2], line[3])
            if dist > max_lenght:
                max_lenght = dist
                idx = i
            i = i + 1
        # print("line num", idx)
        return idx

    def find_line_center(self, line):
        x = (line[0] + line[2]) * 0.5 * float(np.random.random(1))
        y = (line[1] + line[3]) * 0.5 * float(np.random.random(1))

        return x, y

    def exploration_loop(self):
        if self.global_map_ == None:
            print("No map")
            return
        if len(self.global_map_.data) < 10:
            print('Small map')
            return
        if self.need_exploration == False:
            print("No exploration needed")
            return
        if self.current_pose == None:
            print("No pose")
            return
        print("start")

        map_image = Map.occupancygrid_to_numpy(self.global_map_)
        img = np.zeros(
            (3, self.global_map_.info.height, self.global_map_.info.width),
            dtype=np.uint8,
        )

        new_image_red, new_image_green, new_image_blue = img
        new_image_blue = (map_image) * 1

        new_rgb = np.dstack([new_image_blue, new_image_green, new_image_red])

        UNKNOWN_RANGE = [(255, 0, 0), (255, 0, 0)]  # Blue in BGR, [(low), (high)].
        KNOWN_RANGE = [(0, 0, 0), (0, 0, 0)]  # Green in BGR, [(low), (high)].
        OBSTACLE_COLOR_RANGE = [(100, 0, 0), (100, 0, 0)]

        # Create masks:
        unknown_mask = cv2.inRange(new_rgb, UNKNOWN_RANGE[0], UNKNOWN_RANGE[1])
        known_mask = cv2.inRange(new_rgb, KNOWN_RANGE[0], KNOWN_RANGE[1])
        obstacle_mask = cv2.inRange(new_rgb, OBSTACLE_COLOR_RANGE[0], OBSTACLE_COLOR_RANGE[1])

        # Adjust according to your adjacency requirement.
        kernel = np.ones((3, 3), dtype=np.uint8)

        # Dilating masks to expand boundary.
        unknown_mask = cv2.erode(unknown_mask, kernel, iterations=1)
        unknown_mask = cv2.dilate(unknown_mask, kernel, iterations=1)
        known_mask = cv2.dilate(known_mask, kernel, iterations=1)
        obstacle_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)

        # Required points now will have both color's mask val as 255.
        common = cv2.bitwise_and(unknown_mask, known_mask)
        common = cv2.bitwise_xor(obstacle_mask, common)
        common = cv2.bitwise_not(obstacle_mask, common)
        common = cv2.bitwise_and(unknown_mask, common)

        contours, hierarchy = cv2.findContours(known_mask, 1, 2)
        greatest_area = 0
        cnt = None
        
        for contour in contours:
            if cv2.contourArea(contour) > greatest_area:
                cnt = contour
                greatest_area = cv2.contourArea(contour)

        moments = cv2.moments(cnt)
        cx = int(moments['m10']/moments['m00'])
        cy = int(moments['m01']/moments['m00'])

        known_center = [0, 0]
        known_center[1] = (
            float(cx) * self.global_map_.info.resolution
            + self.global_map_.info.origin.position.y
        )
        known_center[0] = (
            float(cy) * self.global_map_.info.resolution
            + self.global_map_.info.origin.position.x
        )
        intersection_points = np.where(common > 0)

        # Say you want these points in a list form, then you can do this.
        pts_list = [[r, c] for r, c in zip(*intersection_points)]

        goal = [0, 0]
        
        # best_cost = 9999999999999
            
        # fitness function
        fitness_value = 0
        
        # exploration summand
        greatest_distance = 0
        most_distant_unexplored_point = None
    
        for i in range(len(pts_list)):
            point = [0, 0]
            point[1] = (
                float(pts_list[i][0]) * self.global_map_.info.resolution
                + self.global_map_.info.origin.position.y
            )
            point[0] = (
                float(pts_list[i][1]) * self.global_map_.info.resolution
                + self.global_map_.info.origin.position.x
            )
            # print(f"point = {point}")
            
            local_cost = 0
            
            
            # local_cost += dr * 1
            


            
            # for fitness function
            dx = point[0] - self.current_pose.pose.position.x
            dy = point[1] - self.current_pose.pose.position.y
            dr = (dx ** 2 + dy ** 2) ** 0.5
            
            if dr < self.cell_evaluation_radius:
                fitness_value += 1
            
            # first summand will be evaluated later
            
            # cognitive summand will be evaluated later
            

            # for exploration summand
            
            dx = point[0] - known_center[0]
            dy = point[1] - known_center[1]
            dr = (dx ** 2 + dy ** 2) ** 0.5
            
            if most_distant_unexplored_point is None:
                most_distant_unexplored_point = point
                greatest_distance = dr
            
            if dr > greatest_distance:
                most_distant_unexplored_point = point
                greatest_distance = dr
            
        
        # for coordinate choosing we have to average the coordinates
        divisor = 0
        
        # first summand
        if self.current_point is not None:
            goal[0] += self.velocity_dampening_coefficient * self.current_point[0]
            goal[1] += self.velocity_dampening_coefficient * self.current_point[1]
            divisor += 1
        
        self.current_fitness_value = fitness_value
        # for cognitive summand
        if self.best_fitness_position is not None:
            if self.best_fitness_value < fitness_value:
                self.best_fitness_value = fitness_value
                self.best_fitness_position = self.current_pose
        elif self.best_fitness_position is None:
            self.best_fitness_position = self.current_pose
            self.best_fitness_value = fitness_value
            
            goal[0] += self.best_fitness_position.pose.position.x * 0.2 * float(np.random.random(1))
            goal[1] += self.best_fitness_position.pose.position.y * 0.2 * float(np.random.random(1))
            divisor += 1
        
        # for social summand
        if self.best_neighbour_position is not None:
            goal[0] += self.best_neighbour_position.pose.position.x * 0.5 * float(np.random.random(1))
            goal[1] += self.best_neighbour_position.pose.position.y * 0.5 * float(np.random.random(1))
            divisor += 1
            # if local_cost < best_cost:
            #     goal = point
            #     best_cost = local_cost

        if most_distant_unexplored_point is not None:
            goal[0] += most_distant_unexplored_point[0] * 1 * float(np.random.random(1))
            goal[1] += most_distant_unexplored_point[1] * 1 * float(np.random.random(1))
            divisor += 1

        print(float(np.random.random(1)))

        divisor = 2

        goal[0] /= divisor
        goal[1] /= divisor

        self.current_point = goal

        if goal is None:
            print("Goal is None!")
            print(f"len(pts_list) = {len(pts_list)}")
            return

        goal_msg = PoseStamped()
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]

        # print(goal)
        # print(goal_msg)
        self.pub_goal_point.publish(goal_msg)

        self.display_countours(pts_list)
        cv2.imwrite(os.path.expanduser(os.path.join("~/.", "global_map.png")), common)

        # print(contours)
        self.need_exploration = True
        # print("w", self.global_map_.info.width)
        # print("h", self.global_map_.info.height)


def main():
    rclpy.init(args=None)
    node = ModifiedPSOExplorer()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
