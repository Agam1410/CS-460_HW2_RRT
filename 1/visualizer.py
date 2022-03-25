from matplotlib.patches import Polygon
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from tree import Tree
from sampler import sample


def visualize_problem(robot, obstacles, start, goal):

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    robot_start = translate(robot, start)  # translates the robot to the given point
    robot_strt_pts = np.array(robot_start)
    robot_p_st = Polygon(robot_strt_pts, color=[0, 1, 0])  # add the robot patch at start point, with green color
    robot_goal = translate(robot, goal)
    robot_goal_pts = np.array(robot_goal)
    robot_p_gl = Polygon(robot_goal_pts, color=[1, 0, 0])  # add the robot patch at end point, with red color
    display = plt.gca()
    display.add_patch(robot_p_st)
    display.add_patch(robot_p_gl)

    for i in obstacles:  # add patches for the obstacles
        obs = Polygon(i, color=[1, 0, 1])
        display.add_patch(obs)

    plt.show()


def translate(robot, goal):

    location = []

    for i in robot:
        location.append((i[0] + goal[0], i[1] + goal[1]))  # since the robot starts at (0, 0), the goal points can be
        # added directs to robot points individually

    return location


def visualize_configuration(robot, obstacles, start, goal):

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    robot_start = translate(robot, start)  # translates the robot to the given point
    robot_strt_pts = np.array(robot_start)
    robot_p_st = Polygon(robot_strt_pts, color=[0, 1, 0])  # add the robot patch at start point, with green color
    robot_goal = translate(robot, goal)
    robot_goal_pts = np.array(robot_goal)
    robot_p_gl = Polygon(robot_goal_pts, color=[1, 0, 0])  # add the robot patch at end point, with red color
    display = plt.gca()
    display.add_patch(robot_p_st)
    display.add_patch(robot_p_gl)
    flipped_robot = []
    for point in robot:  # flip the robot over the y = -x line

        flipped_robot.append((-point[0], -point[1]))

    boundary_list = [(0, 0), (0, 10), (10, 10), (10, 0)]
    obstacles.append(boundary_list)
    for obs in obstacles:

        for i in range(len(obs)):  # for every line of the obstacles, travrese the flipped robot over the line

            start_point = obs[i]
            end_point = obs[i - 1]
            dx = start_point[0] - end_point[0]
            dy = start_point[1] - end_point[1]
            x_step = dx / 200
            y_step = dy / 200
            for j in range(201):  # discritize the line and add a patch of the robot at every point
                node = (end_point[0] + (x_step * j), end_point[1] + (y_step * j))
                points_list = translate(flipped_robot, node)
                plgn = Polygon(np.array(points_list), color=[.5, .5, .5])
                display.add_patch(plgn)

    obstacles.remove(boundary_list)

    for i in obstacles:  # add patches for the obstacles
        obs = Polygon(i, color=[1, 0, 1])
        display.add_patch(obs)

    plt.show()


def visualize_path(robot, obstacles, path):

    if path is None:
        return None

    robot_start = translate(robot, path[0])  # robot points at the start position
    robot_strt_pts = np.array(robot_start)
    robot_p_st = Polygon(robot_strt_pts, color=[0, 1, 0])
    robot_goal = translate(robot, path[-1])
    robot_goal_pts = np.array(robot_goal)  # robot points at the goal position
    robot_p_gl = Polygon(robot_goal_pts, color=[1, 0, 0])
    disp = plt.figure()
    display = disp.gca()
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    robot_path_patch = display.add_patch(robot_p_st)

    for i in obstacles:  # add patches for the obstacles
        obs = Polygon(i, color=[1, 0, 1])
        display.add_patch(obs)
    step = len(path)

    discritized_path = []
    for z in range(step - 1):
        dx = path[z + 1][0] - path[z][0]
        dy = path[z + 1][1] - path[z][1]
        x_step = dx / 40
        y_step = dy / 40
        for j in range(41):
            node = (path[z][0] + (x_step * j), path[z][1] + (y_step * j))  # add the small steps between the 2 points
            discritized_path.append(node)

    step = len(discritized_path)

    def animate(step_num):
        step_size = discritized_path[step_num]
        robot_path_patch.set_xy(translate(robot, step_size))  # translate the robot to given point
        return robot_p_st

    ani = animation.FuncAnimation(disp, animate, frames=step, interval=10, repeat=False)

    x_vals = []
    y_vals = []

    for j in path:  # add circles to the points for better visuals and animate the path line
        temp_pnt = plt.Circle(j, .05)
        display.add_patch(temp_pnt)
        x_vals.append(j[0])
        y_vals.append((j[1]))

    plt.plot(x_vals, y_vals)
    plt.show()


def visualize_points(points, robot, obstacles, start, goal):

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    robot_start = translate(robot, start)  # robot points at the start position
    robot_strt_pts = np.array(robot_start)
    robot_p_st = Polygon(robot_strt_pts, color=[0, 1, 0])
    robot_goal = translate(robot, goal)  # robot points at the goal position
    robot_goal_pts = np.array(robot_goal)
    robot_p_gl = Polygon(robot_goal_pts, color=[1, 0, 0])
    display = plt.gca()

    if points:  # for all the given points, move the robot there and add a patch
        for j in points:
            temp_pnt = Polygon(np.array(translate(robot, j)), color=[0, 0, 0])  # translate the robot to given point
            display.add_patch(temp_pnt)

    display.add_patch(robot_p_st)
    display.add_patch(robot_p_gl)

    for i in obstacles:  # add patches for the obstacles
        obs = Polygon(i, color=[1, 0, 1])
        display.add_patch(obs)

    plt.show()


def visualize_rrt(robot, obstacles, start, goal, itern_n):

    tree_struct = Tree(robot, obstacles, start, goal)  # initialize the tree structure with given data

    for i in range(itern_n):  # run the loop for given n iterations

        sample_point = sample()
        if not tree_struct.exists(sample_point):  # if the sample point is already in the tree don't run
            if len(tree_struct.node_data) == 1:  # if it's the first point add it to the start node

                tree_struct.extend(start, sample_point)

            else:
                nearest_point = tree_struct.nearest(sample_point)  # find the nearest point to the sample point
                tree_struct.extend(nearest_point, sample_point)  # extend from the nearest point to the sampel point

    if not tree_struct.exists(goal):  # if the goal not is not in the tree
        nearest_to_goal = tree_struct.nearest_collision_free(goal)  # find a node that is nearest to the goal and is
        # collision free
        if nearest_to_goal is None:  # if there is no point that is collision free close to goal, return None
            return None
        path_to_goal = tree_struct.extend(nearest_to_goal, goal)  # if there is a node, extend from node to goal
    else:
        path_to_goal = True

    path = [goal]  # add goal to the path list
    if path_to_goal:
        curr_node = goal
        while curr_node != start:  # while we don't reach the start node
            path.append(tree_struct.parent(curr_node))  # keep adding the parent of the current node
            curr_node = path[-1]  # update the current node to be the last node in the path list

        path.reverse()  # reverse the path

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    robot_start = translate(robot, start)  # robot points at the start position
    robot_strt_pts = np.array(robot_start)
    robot_p_st = Polygon(robot_strt_pts, color=[0, 1, 0])
    robot_goal = translate(robot, goal)  # robot points at the goal position
    robot_goal_pts = np.array(robot_goal)
    robot_p_gl = Polygon(robot_goal_pts, color=[1, 0, 0])
    display = plt.gca()
    display.add_patch(robot_p_st)
    display.add_patch(robot_p_gl)
    flipped_robot = []
    for point in robot:  # flip the robot over the y = -x line

        flipped_robot.append((-point[0], -point[1]))
    boundary_list = [(0, 0), (0, 10), (10, 10), (10, 0)]
    obstacles.append(boundary_list)
    for obs in obstacles:

        for i in range(len(obs)):  # for every line of the obstacles, travrese the flipped robot over the line

            start_point = obs[i]
            end_point = obs[i - 1]
            dx = start_point[0] - end_point[0]
            dy = start_point[1] - end_point[1]
            x_step = dx / 200
            y_step = dy / 200
            for j in range(201):  # discritize the line and add a patch of the robot at every point
                node = (end_point[0] + (x_step * j), end_point[1] + (y_step * j))
                points_list = translate(flipped_robot, node)
                plgn = Polygon(np.array(points_list), color=[.5, .5, .5])
                display.add_patch(plgn)

    obstacles.remove(boundary_list)

    for i in obstacles:  # add patches for the obstacles
        obs = Polygon(i, color=[1, 0, 1])
        display.add_patch(obs)

    for j in tree_struct.node_data.keys():
        temp_pnt = plt.Circle(j, .05)  # for all the given points, add a small circle patch
        display.add_patch(temp_pnt)
        x_vals = [j[0], tree_struct.parent(j)[0]]  # add the tree stucture lines
        y_vals = [j[1], tree_struct.parent(j)[1]]
        plt.plot(x_vals, y_vals, color=[0, 0, 0])
        plt.draw()
        plt.pause(.01)  # animate the tree growth

    plt.show()


def visualize_rrt_star(robot, obstacles, start, goal, itern_n):

    tree_struct = Tree(robot, obstacles, start, goal)  # initialize the tree structure with given data

    for i in range(itern_n):  # run the loop for given n iterations

        sample_point = sample()
        if not tree_struct.exists(sample_point):  # if the sample point is already in the tree don't run
            if len(tree_struct.node_data) == 1:  # if it's the first point add it to the start node

                tree_struct.extend(start, sample_point)

            else:
                nearest_point = tree_struct.nearest(sample_point)  # find the nearest point to the sample point
                if tree_struct.extend(nearest_point, sample_point):  # if the sample point was added, rewire
                    tree_struct.rewire(sample_point, 1.5)

    if not tree_struct.exists(goal):  # if the goal not is not in the tree
        nearest_to_goal = tree_struct.nearest_collision_free(goal)  # find a node that is nearest to the goal and is
        # collision free
        if nearest_to_goal is None:  # if there is no point that is collision free close to goal, return None
            visualize_points(tree_struct.node_data.keys(), robot, obstacles, start, goal)
            return None
        path_to_goal = tree_struct.extend(nearest_to_goal, goal)  # if there is a node, extend from node to goal
    else:
        path_to_goal = True

    path = [goal]  # add goal to the path list
    if path_to_goal:
        curr_node = goal
        while curr_node != start:  # while we don't reach the start node
            path.append(tree_struct.parent(curr_node))  # keep adding the parent of the current node
            curr_node = path[-1]  # update the current node to be the last node in the path list

        path.reverse()  # reverse the path

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    robot_start = translate(robot, start)  # robot points at the start position
    robot_strt_pts = np.array(robot_start)
    robot_p_st = Polygon(robot_strt_pts, color=[0, 1, 0])
    robot_goal = translate(robot, goal)  # robot points at the goal position
    robot_goal_pts = np.array(robot_goal)
    robot_p_gl = Polygon(robot_goal_pts, color=[1, 0, 0])
    display = plt.gca()
    display.add_patch(robot_p_st)
    display.add_patch(robot_p_gl)
    flipped_robot = []
    for point in robot:  # flip the robot over the y = -x line

        flipped_robot.append((-point[0], -point[1]))
    boundary_list = [(0, 0), (0, 10), (10, 10), (10, 0)]
    obstacles.append(boundary_list)
    for obs in obstacles:

        for i in range(len(obs)):  # for every line of the obstacles, travrese the flipped robot over the line

            start_point = obs[i]
            end_point = obs[i - 1]
            dx = start_point[0] - end_point[0]
            dy = start_point[1] - end_point[1]
            x_step = dx / 200
            y_step = dy / 200
            for j in range(201):  # discritize the line and add a patch of the robot at every point
                node = (end_point[0] + (x_step * j), end_point[1] + (y_step * j))
                points_list = translate(flipped_robot, node)
                plgn = Polygon(np.array(points_list), color=[.5, .5, .5])
                display.add_patch(plgn)

    obstacles.remove(boundary_list)

    for i in obstacles:  # add patches for the obstacles
        obs = Polygon(i, color=[1, 0, 1])
        display.add_patch(obs)

    for j in tree_struct.node_data.keys():
        temp_pnt = plt.Circle(j, .05)  # for all the given points, add a small circle patch
        display.add_patch(temp_pnt)
        x_vals = [j[0], tree_struct.parent(j)[0]]  # add the tree stucture lines
        y_vals = [j[1], tree_struct.parent(j)[1]]
        plt.plot(x_vals, y_vals, color=[0, 0, 0])
        plt.draw()
        plt.pause(.01)  # animate the tree growth

    plt.show()
