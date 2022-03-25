import math


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
        distance = math.sqrt(pow(dx, 2) + pow(dy, 2))  # euclidean distance between point1 and point2
        self.node_data[point2] = [point1, distance]  # add point2 and have point1 as it's parent

    def exists(self, point):

        return point in self.node_data.keys()  # if the point is in the dictionary as a tree, return True

    def parent(self, point):

        return self.node_data[point][0]  # the first index of the list holds the parent of the node

    def nearest(self, point):

        distance = math.inf  # keep track of the shortest distance
        rtrn_tuple = None  # point that has the lowest distance from the point
        for key in self.node_data:
            dx = key[0] - point[0]
            dy = key[1] - point[1]
            temp_distance = math.sqrt(pow(dx, 2) + pow(dy, 2))  # find the distance from the point to key

            if temp_distance < distance and key != point:  # if the distance from current point is lower than the
                # lowest found before and the key is not the point
                distance = temp_distance  # replace the distance with lowest
                rtrn_tuple = key  # replace the key with the shortest distance

        return rtrn_tuple

    def extend(self, point1, point2):

        if self.isCollisionFree(point1, point2):  # if the path from poit1 to poit2 is collision free, extend
            self.add(point1, point2)  # point2 is added with point1 being the parent node
            return True

        return False

    def isCollisionFree(self, point1, point2):

        robot_points_1 = self.translate(point1)
        robot_points_2 = self.translate(point2)

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
                            # print(intersection)
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

    def translate(self, goal):

        location = []

        for i in self.robot:
            location.append((i[0] + goal[0], i[1] + goal[1]))

        return location

    def nearest_collision_free(self, point):

        distance = math.inf
        rtrn_tuple = None
        for key in self.node_data:
            dx = key[0] - point[0]
            dy = key[1] - point[1]
            temp_distance = math.sqrt(pow(dx, 2) + pow(dy, 2))
            if temp_distance == 0.0:
                continue
            if temp_distance < distance and self.isCollisionFree(point, key):
                distance = temp_distance
                rtrn_tuple = key

        return rtrn_tuple

    def get_cost(self, point):

        rtrn_val = 0
        curr_node = point
        while curr_node != self.start:  # while we don't reach the start goal

            rtrn_val += self.node_data[curr_node][1]  # add the cost from current node to parent
            curr_node = self.node_data[curr_node][0]  # replace the current node to it's parent

        return rtrn_val

    def rewire(self, point, r):

        neighbors_list = []  # list of neighbors in the radius r

        for key in self.node_data:

            dx = key[0] - point[0]
            dy = key[1] - point[1]
            distance = math.sqrt(pow(dx, 2) + pow(dy, 2))  # find the distance

            if distance <= r:  # if the distance is less than r, add it to the neighbors list
                neighbors_list.append(key)

        if neighbors_list is None or len(neighbors_list) <= 0:  # if there are no neighbors, return None
            return

        replacement_node = self.start

        while replacement_node:  # to check if a new change was made, if there was then the loop is run again

            for node in neighbors_list:  # for each node in the neighbors list

                if node == self.start:  # if node == start node, we don't run the loop
                    continue
                current_cost = self.get_cost(node)
                shrtst_dst_frm_temp_node = math.inf
                replacement_node = None
                for temp_node in neighbors_list:
                    if temp_node == node or temp_node == self.start:  # if node is start or temp, we don't run the loop
                        continue
                    if self.isCollisionFree(node, temp_node):  # only do the calculations if the points have a path
                        temp_node_cost = self.get_cost(temp_node)

                        dx = node[0] - temp_node[0]
                        dy = node[1] - temp_node[1]
                        temp_node_to_node = math.sqrt(pow(dx, 2) + pow(dy, 2))
                        if current_cost > (temp_node_to_node + temp_node_cost):  # shorter than the current path

                            if (temp_node_to_node + temp_node_cost) < shrtst_dst_frm_temp_node:  # shorter than the
                                # shortest path found before

                                shrtst_dst_frm_temp_node = temp_node_to_node + temp_node_cost  # replace the shortest dist
                                replacement_node = temp_node  # replace the node to new parent of node

                if replacement_node:  # if there is a replacement node, replace it
                    dx = node[0] - replacement_node[0]
                    dy = node[1] - replacement_node[1]
                    temp_dist = math.sqrt(pow(dx, 2) + pow(dy, 2))  # update the distance
                    self.node_data[node][0] = replacement_node  # update the parent node
                    self.node_data[node][1] = temp_dist
                    replacement_node = None
