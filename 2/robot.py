import numpy as np
import math


class Robot:

    def __init__(self, width, height):

        self.width = width
        self.height = height
        self.translation = []
        self.rotation = 0

    def set_pose(self, pose):

        self.translation = (pose[0], pose[1])
        self.rotation = pose[2]

    def transform(self):

        rtrn_list = []
        row_1 = [math.cos(self.rotation), - math.sin(self.rotation)]
        row_2 = [math.sin(self.rotation), math.cos(self.rotation)]
        rotation_matrix = np.array([row_1, row_2])
        point = np.array([self.translation[0], self.translation[1]])
        l_b = (-self.width / 2, -self.height / 2)  # left bottom point
        l_t = (-self.width / 2, self.height / 2)  # left top point
        r_t = (self.width / 2, self.height / 2)  # right top point
        r_b = (self.width / 2, -self.height / 2)  # right bottom point

        start_loc = [l_b, l_t, r_t, r_b]

        for i in start_loc:
            i_array = np.array([i[0], i[1]])
            temp_array = np.dot(rotation_matrix, i_array) + point
            rtrn_list.append([temp_array[0], temp_array[1]])

        return rtrn_list
