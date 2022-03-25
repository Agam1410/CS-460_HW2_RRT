from tree import Tree
from sample import sample


def translate(robot, goal):

    location = []

    for i in robot:
        location.append((i[0] + goal[0], i[1] + goal[1]))

    return location


def rrt_star(robot, obstacles, start, goal, itern_n):

    tree_struct = Tree(robot, obstacles, start, goal)

    for i in range(itern_n):
        sample_point = sample()
        if not tree_struct.exists(sample_point):  # if the sample point is already in the tree don't run
            if len(tree_struct.node_data) == 1:  # if it's the first point add it to the start node

                tree_struct.extend(start, sample_point)

            else:
                nearest_point = tree_struct.nearest(sample_point)
                tree_struct.extend(nearest_point, sample_point)
                tree_struct.rewire(sample_point, 4)

    if not tree_struct.exists(goal):
        nearest_to_goal = tree_struct.nearest_collision_free(goal)
        if nearest_to_goal is None:
            return None
        path_to_goal = tree_struct.extend(nearest_to_goal, goal)
    else:
        path_to_goal = True

    path = [goal]
    if path_to_goal:
        curr_node = goal
        while curr_node != start:
            path.append(tree_struct.parent(curr_node))
            curr_node = path[-1]

        path.reverse()
        return path
    return None
