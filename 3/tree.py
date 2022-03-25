import math
import random
from collision import isCollisionFree as collision_checker


class Tree:

    def __init__(self, robot, obstacles, start, goal):

        self.robot = robot
        self.obstacles = obstacles
        self.start = start
        self.goal = goal
        self.node_data = {start: [start, None]}  # name of the node is the key and the list is it's parent,
        # and distance from parent

    def add(self, point1, point2):

        dx = point1[0] - point2[0]
        dy = point1[1] - point2[1]
        angle_dist = self.angular_dist(point2[2], point1[2])
        distance = math.sqrt(pow(dx, 2) + pow(dy, 2) + pow(angle_dist, 2))
        self.node_data[point2] = [point1, distance]

    def exists(self, point):

        return point in self.node_data.keys()

    def parent(self, point):

        return self.node_data[point][0]

    def nearest(self, point):

        distance = math.inf
        rtrn_tuple = None
        for key in self.node_data:
            dx = key[0] - point[0]
            dy = key[1] - point[1]
            angle_dist = self.angular_dist(point[2], key[2])
            temp_distance = math.sqrt(pow(dx, 2) + pow(dy, 2) + pow(angle_dist, 2))

            if temp_distance < distance and key != point:
                distance = temp_distance
                rtrn_tuple = key

        return rtrn_tuple

    def angular_dist(self, angle1, angle2):

        diff = (angle2 - angle1 + round(math.pi, 2)) % (2 * round(math.pi, 2)) - round(math.pi, 2)
        if diff < 0:
            diff += round(math.pi, 2) * 2
        return diff

    def extend_old(self, point1, point2):

        if self.isCollisionFree(point1, point2):
            self.add(point1, point2)
            return True

        return False

    def extend(self, point, n1, n2, dt):

        rand_duration = random.randint(n1, n2)

        control_list = []
        linear_v = random.randrange(-20, 200) / 10  # max of 2 units per second
        anglular_v = .1 * random.randrange(-314, 314) / 100  # max of .5 * pi per second
        for i in range(int(rand_duration/dt)):

            control_list.append((linear_v, anglular_v))

        temp_path = self.robot.propogate(point, control_list, rand_duration, dt)

        path = []

        for temp_point in temp_path:

            self.robot.set_pose(temp_point)
            robot_points = self.robot.transform()
            if temp_point != temp_path[0]:
                temp_point_index = temp_path.index(temp_point)
                if self.isCollisionFree(temp_path[temp_point_index - 1], temp_point) and collision_checker(robot_points, temp_point, self.obstacles):
                    path.append(temp_point)
                    self.add(temp_path[temp_point_index - 1], temp_point)
                else:
                    break

        return path

    def isCollisionFree(self, point1, point2):

        self.robot.set_pose(point1)
        robot_points_1 = self.robot.transform()
        self.robot.set_pose(point2)
        robot_points_2 = self.robot.transform()

        for j in robot_points_1:
            if j[0] > 10 or j[1] > 10 or j[0] < 0 or j[1] < 0:
                return False
        for j in robot_points_2:
            if j[0] > 10 or j[1] > 10 or j[0] < 0 or j[1] < 0:
                return False

        robot_lines_list = []  # holds the list of lines that can be made from the edges of the robot

        for i in range(len(robot_points_1)):
            robot_lines_list.append([robot_points_1[i], robot_points_2[i]])

        for obs in self.obstacles:

            temp_obstacles = []

            for i in range(-1, len(obs) - 1):
                temp_obstacles.append([obs[i], obs[i + 1]])

            for robot_line in robot_lines_list:

                for obs_line in temp_obstacles:

                    intersection = self.isIntersecting(robot_line, obs_line)
                    rob_x = sorted([robot_line[0][0], robot_line[1][0]])
                    obs_x = sorted([obs_line[0][0], obs_line[1][0]])
                    rob_y = sorted([robot_line[0][1], robot_line[1][1]])
                    obs_y = sorted([obs_line[0][1], obs_line[1][1]])

                    if rob_x[0] <= intersection[0] <= rob_x[1] and rob_y[0] <= intersection[1] <= rob_y[1]:  # check
                        # if the intersection is in range of the robot path line
                        if obs_x[0] <= intersection[0] <= obs_x[1] and obs_y[0] <= intersection[1] <= obs_y[1]:
                            return False

        return True

    def isIntersecting(self, line1, line2):  # check if the lines intersect, if they do return the point of intersection

        dx = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        dy = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        div = self.determinant(dx, dy)
        if div == 0:
            return tuple((-1, -1))

        d = (self.determinant(*line1), self.determinant(*line2))
        x = round(self.determinant(d, dx) / div, 4)
        y = round(self.determinant(d, dy) / div, 4)
        rtrn_tuple = (x, y)

        return rtrn_tuple

    def determinant(self, a, b):
        return (a[0] * b[1]) - (a[1] * b[0])

    def nearest_collision_free(self, point):

        distance = math.inf
        rtrn_tuple = None
        for key in self.node_data:
            dx = key[0] - point[0]
            dy = key[1] - point[1]
            angle_dist = self.angular_dist(point[2], key[2])
            temp_distance = math.sqrt(pow(dx, 2) + pow(dy, 2) + pow(angle_dist, 2))
            if temp_distance == 0.0:
                continue
            if temp_distance < distance and self.isCollisionFree(point, key):
                distance = temp_distance
                rtrn_tuple = key

        return rtrn_tuple

    def cost(self, point):

        rtrn_val = 0
        curr_node = point
        while curr_node != self.start:

            rtrn_val += self.node_data[curr_node][1]
            curr_node = self.node_data[curr_node][0]

        return rtrn_val
