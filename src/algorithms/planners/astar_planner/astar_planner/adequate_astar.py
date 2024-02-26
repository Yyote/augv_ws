# from astar_planner.path_filter import filtering_path 
import numpy as np
import resource
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
from astar_planner_msgs.msg import GlobalTrajectory
from rclpy.qos import qos_profile_sensor_data

import math
"""
Description
"""

import numpy as np
import math






def try_get(fn, default):
    try:
        val = fn().value
        if val is None:
            return default
        else:
            return val
    except:
        return default









def getAngleBetweenPoints(p1, p2):
    angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    angle = angle * 180 / math.pi
    if angle < 0:
        angle = angle + 2 * 180
    return angle


def getDistanceBetweenPoints(p1, p2):
    """
    Вычисление расстояния между точками
    """
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)


def getDistanceFromPointToLineSegment(p, p1, p2):
    """
    Определение расстояния от точки до отрезка (при различных положениях точки относительно отрезка)
    https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    """
    A = p[0] - p1[0]
    B = p[1] - p1[1]
    C = p2[0] - p1[0]
    D = p2[1] - p1[1]

    dot = A * C + B * D
    len_sq = C * C + D * D
    param = -1
    if len_sq != 0:  # in case of 0 length line
        param = dot / len_sq
    xx = 0
    yy = 0

    # Рассматриваем только 3-ий вариант расположения точки относительно отрезка
    if (param > 0) and (param < 1):  # 3 случай
        xx = p1[0] + param * C
        yy = p1[1] + param * D

    dx = p[0] - xx
    dy = p[1] - yy

    distance = math.sqrt(dx * dx + dy * dy)

    if (param < 0) or (param > 1):  # 1 случай
        distance = -1

    return distance


def filterTrajectoryBySaw(trajectory, saw_size):
    """
    Фильтр ступенек заданного размера
    """
    start_pos_index = -1
    while True:
        angle_joint_1 = -1
        angle_joint_2 = -1
        distance_joint_1 = -1
        distance_joint_2 = -1

        start_pos_index = start_pos_index + 1

        if start_pos_index + 1 <= len(trajectory) - 1:
            angle_joint_1 = getAngleBetweenPoints(
                trajectory[start_pos_index], trajectory[start_pos_index + 1])
            distance_joint_1 = getDistanceBetweenPoints(
                trajectory[start_pos_index], trajectory[start_pos_index + 1])

        if start_pos_index + 2 <= len(trajectory) - 1:
            angle_joint_2 = getAngleBetweenPoints(
                trajectory[start_pos_index + 1], trajectory[start_pos_index + 2])
            distance_joint_2 = getDistanceBetweenPoints(
                trajectory[start_pos_index + 1], trajectory[start_pos_index + 2])

        # Пила под прямым углом
        if (math.fabs(angle_joint_1 - angle_joint_2) == 90) and (distance_joint_1 == distance_joint_2):  # Пила обнаружена
            if distance_joint_1 <= saw_size:
                del trajectory[start_pos_index + 1]
                start_pos_index = 0

        # Пила под косым углом
        if (start_pos_index + 1 <= len(trajectory) - 1) and (start_pos_index + 2 <= len(trajectory) - 1):
            if (distance_joint_1 <= saw_size) and (distance_joint_2 <= saw_size) and (angle_joint_1 != angle_joint_2):
                saw_height = getDistanceFromPointToLineSegment(trajectory[start_pos_index + 1],
                                                               trajectory[start_pos_index],
                                                               trajectory[start_pos_index + 2])
                if saw_height != -1:
                    if saw_height <= saw_size * 0.5:
                        del trajectory[start_pos_index + 1]
                        start_pos_index = 0

        if start_pos_index >= len(trajectory) - 1:
            break

    return trajectory


def filterTrajectoryByAngle(trajectory, angle, distance):
    """
    Фильтр по углу между сочленениями
    """
    start_pos_index = 0

    while True:
        angle_joint_1 = -1
        angle_joint_2 = -1
        distance_joint_1 = -1
        distance_joint_2 = -1

        if start_pos_index + 1 <= len(trajectory) - 1:
            angle_joint_1 = getAngleBetweenPoints(
                trajectory[start_pos_index], trajectory[start_pos_index + 1])
            distance_joint_1 = getDistanceBetweenPoints(
                trajectory[start_pos_index], trajectory[start_pos_index + 1])

        if start_pos_index + 2 <= len(trajectory) - 1:
            angle_joint_2 = getAngleBetweenPoints(
                trajectory[start_pos_index + 1], trajectory[start_pos_index + 2])
            distance_joint_2 = getDistanceBetweenPoints(
                trajectory[start_pos_index + 1], trajectory[start_pos_index + 2])

        if (angle_joint_1 != -1) and (angle_joint_2 != -1):
            if math.fabs(angle_joint_1 - angle_joint_2) == 0:
                del trajectory[start_pos_index + 1]
                start_pos_index = 0
            elif (math.fabs(angle_joint_1 - angle_joint_2) <= angle) and (distance_joint_1 + distance_joint_2 <= distance):
                del trajectory[start_pos_index + 1]
                start_pos_index = 0
            elif (math.fabs(angle_joint_1 - angle_joint_2) <= 15) and (distance_joint_1 <= 5 or distance_joint_2 <= 5):
                del trajectory[start_pos_index + 1]
                start_pos_index = 0
            elif angle_joint_1 == 0 or angle_joint_2 == 0:
                if angle_joint_1 == 0:
                    angle_joint_1 = 360
                if angle_joint_2 == 0:
                    angle_joint_2 = 360
                if (math.fabs(angle_joint_1 - angle_joint_2) <= angle) and (distance_joint_1 + distance_joint_2 <= distance):
                    del trajectory[start_pos_index + 1]
                    start_pos_index = 0
                else:
                    start_pos_index = start_pos_index + 1
            else:
                start_pos_index = start_pos_index + 1
        else:
            break

    return trajectory


def addMediumPoints(trajectory, minJointLength):
    """
    Добавляем промежуточные точки
    minJointLength - если сочленение меньше указанной длинны, то не добавляем промежуточных точек
    """
    start_pos_index = 0

    while True:
        if start_pos_index + 1 <= len(trajectory) - 1:
            distance_joint = getDistanceBetweenPoints(
                trajectory[start_pos_index], trajectory[start_pos_index + 1])
            if distance_joint >= minJointLength / 2.0 and distance_joint > 0.15:
                center_of_joint = np.array(
                    [int((trajectory[start_pos_index][0] + trajectory[start_pos_index + 1][0]) / 2),
                     int((trajectory[start_pos_index][1] + trajectory[start_pos_index + 1][1]) / 2)])
                trajectory.insert(start_pos_index + 1, center_of_joint)
                start_pos_index = 0
            else:
                start_pos_index = start_pos_index + 1
        else:
            break

    return trajectory


def filtering_path(path, angle, dist, saw_dist):
    """
    ## Функция фильтрации пути после D*
    """
    good_points = path
    good_points = list(good_points)

    good_points = filterTrajectoryBySaw(
        good_points, saw_dist)  # Убираем ступеньки
    good_points = filterTrajectoryByAngle(
        good_points, angle, dist)  # Убираем лишние сочленения
    # Добавляем промежуточные точки
    good_points = addMediumPoints(good_points, 50)
    good_points = np.array(good_points)
    print("END FILTERED OF PATH")
    return good_points


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)

        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        open_set_empty = False

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                open_set_empty = True
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        if not open_set_empty:
            rx, ry = self.calc_final_path(goal_node, closed_set)
        else:
            # rx, ry = self.calc_final_paths(start_node, closed_set)
            rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = -50  # round(min(ox))
        self.min_y = -50  # round(min(oy))
        self.max_x = 50  # round(max(ox))
        self.max_y = 50  # round(max(oy))

        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__("astar_global_planner", allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # self.robot_id = self.get_parameter("id").value
        self.robot_id = try_get(lambda: self.get_parameter("id"), 1)
        if self.robot_id == None:
            self.robot_id = 0

        self.trajectory_sended = True

        self.robot_pose_ = None
        self.goal_pose_ = PoseStamped()
        self.trajectory = GlobalTrajectory()

        self.grid_map_ = None
        self.old_map = OccupancyGrid()
        self.obstacle_map = list()

        self.global_path = list()
        self.old_path = list()
        self.old_goal_pose = PoseStamped()

        self.start_pose_grid = ()
        self.goal_pose_grid = ()

        robot_topic_prefix = f"/robot{self.robot_id}"
        # robot_topic_prefix = f"/robot1"

        self.visualize_global_path = self.create_publisher(
            MarkerArray, "global_path", 10)
        self.create_subscription(OccupancyGrid, "/OccupancyGrid_map", self.map_clb, 10)
        self.create_subscription(
            # PoseStamped, robot_topic_prefix + "/pose", self.robot_pose_clb,  qos_profile_sensor_data)
            PoseStamped, robot_topic_prefix + "/pose", self.robot_pose_clb, 10)
        self.create_subscription(
            PoseStamped, robot_topic_prefix + "/global_planner/goal_pose", self.goal_pose_clb, 10)

        self.send_trajectory_pub = self.create_publisher(
            GlobalTrajectory, robot_topic_prefix + "/trajectory", 10)
        self.marker_path_pub = self.create_publisher(
            MarkerArray, "global_path", 10)
        self.marker_obs_pub = self.create_publisher(
            MarkerArray, "obstacles", 10)
        self.create_timer(0.2, self.planner_loop)
        self.create_timer(5, self.replan)

    def replan(self):
        self.trajectory_sended = False

    def map_clb(self, msg: OccupancyGrid):
        """
        """
        self.grid_map_ = msg

    def robot_pose_clb(self, msg: PoseStamped):
        """
        """
        self.robot_pose_ = msg
        if self.goal_pose_ == PoseStamped():
            self.goal_pose_ = msg

    def goal_pose_clb(self, msg: PoseStamped):
        """
        """
        # if self.old_goal_pose.pose.position != msg.pose.position:
        self.goal_pose_ = msg
        self.trajectory_sended = False

    def get_obstacle_map(self):
        """
        """
        for i in range(len(self.grid_map_.data)):
            if self.grid_map_.data[i] == 100:
                obj_exist = i in self.obstacle_map
                if obj_exist == False:
                    self.obstacle_map.append(i)

    def world_to_map(self, x: int, y: int):
        """
        Get map coordinates
        """
        mx = int((x - self.grid_map_.info.origin.position.x) /
                 self.grid_map_.info.resolution)
        my = int((y - self.grid_map_.info.origin.position.y) /
                 self.grid_map_.info.resolution)
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

        # print(self.grid_map_.info)
        # print(y)

        x = x * self.grid_map_.info.resolution + \
            self.grid_map_.info.origin.position.x + self.grid_map_.info.resolution / 2.0
        y = y * self.grid_map_.info.resolution + \
            self.grid_map_.info.origin.position.y + self.grid_map_.info.resolution / 2.0

        return y, x

    def planner_loop(self):
        if (self.robot_pose_ == None):
            print("Robot pose missing")
            return
        if (self.grid_map_ == None):
            print("Grid map missing")
            return
        if (self.trajectory_sended == True):
            print("Trajectory sended")
            return

        obs_x = []
        obs_y = []

        self.get_obstacle_map()
        for i in self.obstacle_map:
            x = self.get_coords_from_grid_index(i)[0]
            y = self.get_coords_from_grid_index(i)[1]
            obs_x.append(x)
            obs_y.append(y)
        #     print(f"OBSTACLE {i} {[x, y]}")
        # print("len ", len(self.grid_map_.data))
        # print("resolution ", self.grid_map_.info.resolution)
        a_star = AStarPlanner(
            obs_x, obs_y, self.grid_map_.info.resolution, 0.5)

        # self.start_pose_grid = [0, 0]
        cur_time = self.get_clock().now().to_msg().nanosec / (10**9) + \
            float(self.get_clock().now().to_msg().sec)
        rx, ry = a_star.planning(self.robot_pose_.pose.position.y, self.robot_pose_.pose.position.x,
                                 self.goal_pose_.pose.position.y, self.goal_pose_.pose.position.x)
        cur_time = cur_time - self.get_clock().now().to_msg().nanosec / (10**9) - \
            float(self.get_clock().now().to_msg().sec)
        # print("whole time = ", cur_time)
        # print("memory usage in kylobytes =", resource.getrusage(
        #     resource.RUSAGE_SELF).ru_maxrss)
        # print("percent_infill", len(obs_x) /
        #       (self.grid_map_.info.width*self.grid_map_.info.height))
        # print("full_map")
        # 0.014475584030151367
        # -0.03199148178100586
        print(rx)
        self.display_path(ry, rx)
        self.display_obs(obs_y, obs_x)

        self.infill_trajectory(ry, rx)
        self.trajectory_sended = True
        self.old_goal_pose = self.goal_pose_

    def display_path(self, x: list, y: list):
        marker_arr = MarkerArray()
        for i in range(len(x)):
            waypoint = Marker()
            waypoint.id = i
            waypoint.type = Marker.SPHERE
            # waypoint.header.frame_id = "odom"
            waypoint.header.frame_id = "/map"
            waypoint.action = Marker.ADD

            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.ns = "path"
            waypoint.color.a = 1.0
            waypoint.color.b = 0.5
            waypoint.color.g = 0.5
            waypoint.color.r = 0.5

            waypoint.scale.x = 0.2
            waypoint.scale.y = 0.2
            waypoint.scale.z = 0.2

            waypoint.pose.position.x = x[i]
            waypoint.pose.position.y = y[i]
            waypoint.pose.position.z = 1.0

            marker_arr.markers.append(waypoint)
        self.marker_path_pub.publish(marker_arr)
        self.trajectory = GlobalTrajectory()

    def display_obs(self, x: list, y: list):
        marker_arr = MarkerArray()
        for i in range(len(x)):
            waypoint = Marker()
            waypoint.id = i
            waypoint.type = Marker.CUBE
            # waypoint.header.frame_id = "odom"
            waypoint.header.frame_id = "/map"
            waypoint.action = Marker.ADD

            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.ns = "path"
            waypoint.color.a = 1.0
            waypoint.color.b = 0.1
            waypoint.color.g = 0.2
            waypoint.color.r = 0.5

            waypoint.scale.x = 0.5
            waypoint.scale.y = 0.5
            waypoint.scale.z = 0.5

            waypoint.pose.position.x = x[i]
            waypoint.pose.position.y = y[i]
            waypoint.pose.position.z = 1.0

            marker_arr.markers.append(waypoint)
        self.marker_obs_pub.publish(marker_arr)

    def infill_trajectory(self, x, y):
        x.reverse()
        y.reverse()

        list_of_np_coords = list()
        for i in range(len(x)):
            point = np.array([x[i], y[i]])
            list_of_np_coords.append(point)
        list_of_np_coords = filtering_path(list_of_np_coords, 3, 25, 1)

        self.trajectory.mode = GlobalTrajectory.USE_UNSTABLE
        for i in range(len(list_of_np_coords)):
            waypoint = PoseStamped()
            waypoint.pose.position.x = list_of_np_coords[i][0]
            waypoint.pose.position.y = list_of_np_coords[i][1]
            self.trajectory.waypoints.append(waypoint)

        del self.trajectory.waypoints[0]

        self.trajectory.max_velocity = 0.6
        self.send_trajectory_pub.publish(self.trajectory)


def main():

    rclpy.init(args=None)
    node = GlobalPlanner()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()