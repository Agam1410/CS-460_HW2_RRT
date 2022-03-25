from tree import Tree
from sampler import sample


def translate(robot, goal):

    location = []

    for i in robot:
        location.append((i[0] + goal[0], i[1] + goal[1]))

    return location


def rrt_star(robot, obstacles, start, goal, itern_n):

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
        return path

    return None
