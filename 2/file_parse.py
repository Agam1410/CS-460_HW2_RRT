from robot import Robot


def parse_problem(world_file, problem_file):

    world_data = open(world_file, "r")
    problem_locaitons = open(problem_file, "r")
    robot = Robot(0, 0)
    obstables = []
    problems = []

    split_data = world_data.readline().split(" ")
    robot.width = float(split_data[0])
    robot.height = float(split_data[1])

    for i in world_data:
        split_data = i.split(" ")
        obstacle_single = []
        for j in range(0, len(split_data)):
            if (j + 1) % 2 == 0:
                obstacle_single.append((float(split_data[j - 1]), float(split_data[j])))

        obstables.append(obstacle_single)

    for i in problem_locaitons:
        split_data = i.split(" ")
        start_goal = []
        for j in range(0, len(split_data)):
            if (j + 1) % 3 == 0:
                start_goal.append((float(split_data[j - 2]), float(split_data[j - 1]), float(split_data[j])))

        problems.append(start_goal)

    rtrn_val = (robot, obstables, problems)
    return rtrn_val

