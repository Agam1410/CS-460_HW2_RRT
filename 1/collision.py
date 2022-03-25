import matplotlib.path as plpath


def isCollisionFree(robot, point, obstacles):

    robot_points = translate(robot, point)

    robot_lines_list = []  # holds the list of lines that can be made from the edges of the robot

    for i in range(-1, len(robot_points) - 1):

        robot_lines_list.append([robot_points[i], robot_points[i+1]])

    for obs in obstacles:

        temp_obstacles = []

        for i in range(-1, len(obs) - 1):

            temp_obstacles.append([obs[i], obs[i + 1]])

        for robot_line in robot_lines_list:

            for obs_line in temp_obstacles:
                intersection = isIntersecting(robot_line, obs_line)
                rob_x = sorted([robot_line[0][0], robot_line[1][0]])
                obs_x = sorted([obs_line[0][0], obs_line[1][0]])
                rob_y = sorted([robot_line[0][1], robot_line[1][1]])
                obs_y = sorted([obs_line[0][1], obs_line[1][1]])

                if rob_x[0] <= intersection[0] <= rob_x[1] and obs_x[0] <= intersection[0] <= obs_x[1]:  # check
                    # if the intersection is in range of the robot path line
                    if rob_y[0] <= intersection[0] <= rob_y[1] and obs_y[0] <= intersection[0] <= obs_y[1]:
                        # print(intersection)
                        return False

    for i in obstacles:  # check if the robot is inside the polygon
        current_obs_path = plpath.Path(i)
        for j in robot_points:
            if current_obs_path.contains_point(j):
                return False
            elif j[0] > 10 or j[1] > 10 or j[0] < 0 or j[1] < 0:
                return False

    return True


def isIntersecting(line1, line2):  # check if the lines intersect, if they do return the point of intersection

    dx = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    dy = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    div = determinant(dx, dy)
    if div == 0:
        return tuple((-1, -1))

    d = (determinant(*line1), determinant(*line2))
    x = round(determinant(d, dx) / div, 4)
    y = round(determinant(d, dy) / div, 4)
    rtrn_tuple = (x, y)

    return rtrn_tuple


def determinant(a, b):  # returns the determinant with the given rows
    return a[0] * b[1] - a[1] * b[0]


def translate(robot, goal):

    location = []

    for i in robot:
        location.append((i[0] + goal[0], i[1] + goal[1]))

    return location
