import numpy as np
import math


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