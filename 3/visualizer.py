from matplotlib.patches import Polygon
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import math


def visualize_problem(robot, obstacles, start, goal):

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    robot.set_pose(start)
    robot_start = robot.transform()
    robot_strt_pts = np.array(robot_start)
    robot_p_st = Polygon(robot_strt_pts, color=[0, 1, 0])
    robot.set_pose(goal)
    robot_goal = robot.transform()
    robot_goal_pts = np.array(robot_goal)
    robot_p_gl = Polygon(robot_goal_pts, color=[1, 0, 0])
    display = plt.gca()
    display.add_patch(robot_p_st)
    display.add_patch(robot_p_gl)

    for i in obstacles:
        obs = Polygon(i, color=[1, 0, 1])
        display.add_patch(obs)

    plt.show()

def angular_dist(angle1, angle2):

    diff = (angle2 - angle1 + round(math.pi, 2)) % (2 * round(math.pi, 2)) - round(math.pi, 2)
    if diff < 0:
        diff += round(math.pi, 2) * 2
    return diff


def translate(robot, goal):

    location = []

    for i in robot:
        location.append((i[0] + goal[0], i[1] + goal[1]))  # works becasue the starting point is always (0, 0)

    return location


def visualize_trajectory(robot, obstacles, start, goal, trajectory):

    path = trajectory
    if path is None:
        return None

    robot.set_pose(start)
    robot_start = robot.transform()
    robot_strt_pts = np.array(robot_start)
    robot_p_st = Polygon(robot_strt_pts, color=[0, 1, 0])
    robot.set_pose(goal)
    robot_goal = robot.transform()
    robot_goal_pts = np.array(robot_goal)
    robot_p_gl = Polygon(robot_goal_pts, color=[1, 0, 0])
    disp = plt.figure()
    display = disp.gca()
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    robot_path_patch = display.add_patch(robot_p_st)


    for i in obstacles:
        obs = Polygon(i, color=[1, 0, 1])
        display.add_patch(obs)
    step = len(path)

    discritized_path = []
    for z in range(step - 1):
        dx = path[z + 1][0] - path[z][0]
        dy = path[z + 1][1] - path[z][1]
        ang = path[z][2]
        x_step = dx / 2
        y_step = dy / 2
        ang_step = ang / 2

        for ang_step_temp in range(3):
            node = (path[z][0], path[z][1], path[z][2] + (ang_step * ang_step_temp))
            discritized_path.append(node)

        for j in range(3):
            node = (path[z][0] + (x_step * j), path[z][1] + (y_step * j), ang)
            discritized_path.append(node)

    ang = path[-1][2]
    ang_step = ang / 3
    for ang_step_temp in range(3):
        node = (path[-1][0], path[-1][1], path[-1][2] + (ang_step * ang_step_temp))
        discritized_path.append(node)

    step = len(discritized_path)

    def animate(step_num):
        step_size = discritized_path[step_num]
        robot.set_pose(step_size)
        robot_path_patch.set_xy(robot.transform())

    ani = animation.FuncAnimation(disp, animate, frames=step, interval=40, repeat=False)

    x_vals = []
    y_vals = []

    for j in path:
        temp_pnt = plt.Circle(j, .05)
        display.add_patch(temp_pnt)
        x_vals.append(j[0])
        y_vals.append((j[1]))

    plt.plot(x_vals, y_vals)
    plt.show()
