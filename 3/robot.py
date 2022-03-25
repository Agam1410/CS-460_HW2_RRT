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

    def kinematics(self, state, control):

        x_new = control[0] * math.cos(state[2] + math.pi/2)
        y_new = control[0] * math.cos(state[2] + math.pi/2)
        rtrn_val = (x_new, y_new, control[1])
        return rtrn_val

    def propogate(self, state, controls, durations, dt):

        if durations == 0:
            return [state]

        rtrn_list = [state]

        start = state

        for i in range(int(durations/dt)):

            temps = start
            temp_control = controls[i]

            temp_kinematics = self.kinematics(temps, temp_control)

            delta_x = temp_kinematics[0] * dt
            delta_y = temp_kinematics[1] * dt
            delta_theta = temp_kinematics[2] * dt
            correct_theta = start[2] + delta_theta

            while correct_theta > math.pi:  # make sure the angle is between -pi and pi
                correct_theta -= 2 * math.pi
            while correct_theta < -math.pi:
                correct_theta += math.pi

            new_state = (start[0] + delta_x, state[1] + delta_y, correct_theta)  # new state after dt

            start = new_state

            rtrn_list.append(new_state)

        return rtrn_list
