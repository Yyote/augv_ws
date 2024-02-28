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
from augv_navigation_msgs.msg import RobotInfo, RobotInfoArray

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


class FrontierPSOExplorer(Node):
    def __init__(self):
        self.node_name = "frontier_pso_explorer"
        super().__init__(
            self.node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        
        self.robot_id = try_get(lambda: self.get_parameter("id"), 1)
        self.exploration_period = try_get(lambda: self.get_parameter("exploration_period"), 1)
        
        self.current_point = None
        
        self.global_map_ = None
        self.exploration_field_ = None
        self.key_points = None
        self.need_exploration = True
        self.current_pose = None
        self.robot_info_array = None

        goal_pose_topic = f"/robot{self.robot_id}/global_planner/goal_pose"  # self.get_parameter('goal_topic').value

        self.create_subscription(RobotInfoArray, '/global/robot_info', self.robot_info_cb, 10)
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

        self.create_timer(5, self.exploration_loop)

    def robot_info_cb(self, robot_info):
        self.robot_info_array = robot_info

    def pose_cb(self, pose):
        self.current_pose = pose

    def exploration_need_clb(self, msg: Bool):
        # print("here")
        # self.need_exploration = msg.data
        pass

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
        x = (line[0] + line[2]) * 0.5
        y = (line[1] + line[3]) * 0.5

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
        if self.robot_info_array == None:
            print("No robot info")
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
        known_mask = cv2.bitwise_and(known_mask, cv2.bitwise_not(obstacle_mask))
        common = cv2.bitwise_and(unknown_mask, known_mask)

        intersection_points = np.where(common > 0)

        # Say you want these points in a list form, then you can do this.
        pts_list = [[r, c] for r, c in zip(*intersection_points)]

        goal = None
        
        
        cognitive_value = 99999999999
        cognitive_point = None
        
        social_cost = 0
        social_point = None
        
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
            
            # cognitive part
            
            dx = point[0] - self.current_pose.pose.position.x
            dy = point[1] - self.current_pose.pose.position.y
            dr = (dx ** 2 + dy ** 2) ** 0.5
            
            if dr < cognitive_value:
                cognitive_point = point
                cognitive_value = dr
            
            # social part
            average_dr = 0
            
            for i in range(len(self.robot_info_array.data)):
                if self.robot_info_array.data[i].robot_id == self.robot_id:
                    continue
                
                current_robot_pose = self.robot_info_array.data[i].current_pose
                dx = point[0] - current_robot_pose.pose.position.x
                dy = point[1] - current_robot_pose.pose.position.y
                dr = (dx ** 2 + dy ** 2) ** 0.5
                
                average_dr += dr
            
            average_dr /= len(self.robot_info_array.data) - 1
            
            if social_cost < average_dr:
                social_point = point
                social_cost = average_dr
            
            if cognitive_point is not None or social_point is not None:
                goal = [0, 0]
                if cognitive_point is not None:
                    goal[0] += cognitive_point[0] * 1
                    goal[1] += cognitive_point[1] * 1
                if social_point is not None:
                    goal[0] += social_point[0] * 1
                    goal[1] += social_point[1] * 1
                if self.current_point is not None:
                    goal[0] += self.current_point[0] * 0.2
                    goal[1] += self.current_point[1] * 0.2
            
            self.current_point = goal
            
            goal[0] /= 2
            goal[1] /= 2
            
            # if local_cost < best_cost:
            #     goal = point
            #     best_cost = local_cost

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
    node = FrontierPSOExplorer()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
