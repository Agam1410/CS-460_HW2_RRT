def parse_problem(world_file, problem_file):

    world_data = open(world_file, "r")
    problem_locations = open(problem_file, "r")
    robot = []
    obstacles = []
    problems = []

    split_data = world_data.readline().split(" ")  # split the first line by one space

    for j in range(0, len(split_data)):
        if (j + 1) % 2 == 0:
            robot.append((float(split_data[j - 1]), float(split_data[j])))  # populate robot points

    for i in world_data:  # populate the obstacles list, starting from line 2
        split_data = i.split(" ")
        obstacle_single = []
        for j in range(0, len(split_data)):
            if (j + 1) % 2 == 0:
                obstacle_single.append((float(split_data[j - 1]), float(split_data[j])))

        obstacles.append(obstacle_single)

    for i in problem_locations:  # populate the start and goal in pairs to a list
        split_data = i.split(" ")
        start_goal = []
        for j in range(0, len(split_data)):
            if (j + 1) % 2 == 0:
                start_goal.append((float(split_data[j - 1]), float(split_data[j])))

        problems.append(start_goal)

    rtrn_val = (robot, obstacles, problems)
    return rtrn_val  # return the robotm obstacles and problems list
