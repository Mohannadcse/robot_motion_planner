from planarsim import *
import numpy as np
'''
Code provided by Devin Balkcom
'''
class PlanarTrajectory:

    # controls: a list of tuples of available controls, as xdot, ydot, thetadot tuples
    # sx, sy, stheta: Initial starting configuration of the robot (location and radians.)
    # control sequence: A list of indices into the control list
    # durations: a list of durations for each of the controls applied in sequence.

    def __init__(self, controls, sx, sy, stheta, control_sequence, durations):
        # list of lists. Each sublist contains xdot, ydot, thetadot in frame of robot
        self.controls = controls

        # starting configuration
        self.q = [sx, sy, stheta]

        # a list of indices into the control_list
        self.sequence = control_sequence

        # durations
        self.durations = durations

        self.end_time = sum(durations)

        # transforms after switches. 0th switch is initial configuration
        self.transform = []
        self.compute_switch_transforms()

    def compute_switch_transforms(self):

        current_transform = transform_from_config(self.q)
        self.transform.append(current_transform)

        x, y, theta = config_from_transform(current_transform)

        for i in range(len(self.sequence)):

            u = self.controls[self.sequence[i]]
            t = self.durations[i]

            #print(u)

            control_transform = transform_from_control(u, t)

            current_transform = current_transform @ control_transform

            #print(current_transform)

            self.transform.append(current_transform)



    def most_recent_switch(self, t):
        total_t = 0
        switch_idx = 0

        while total_t <= t:
            total_t += self.durations[switch_idx]
            switch_idx += 1

        switch_idx -= 1

        #print(switch_idx)

        return switch_idx

    def transform_at_t(self, t):
        # find the most recent switch, and apply the most recent control from the switch to find a new tranform.

        si = self.most_recent_switch(t)
        most_recent_transform = self.transform[si]
        current_control_index = self.sequence[si]
        u = self.controls[current_control_index]

        t_last_switch = 0
        for i in range(si):
            t_last_switch += self.durations[i]

        control_transform = transform_from_control(u, t - t_last_switch)

        T = most_recent_transform @ control_transform

        return T

    def config_at_t(self, t):
        T = self.transform_at_t(t)
        return config_from_transform(T)

    # compute a list of active rotation centers
    # up through time t. (For now, assumes trajectory
    # contains only rotations
    def rotation_center_sequence(self, t):

        def rc_at_time(current_t):
            q = self.config_at_t(current_t)
            rc_world = compute_worldframe_rcs(self.controls, q)

            # select the active control and corresponding rc
            control_index = self.sequence[i]
            #print(control_index)
            rc = rc_world[control_index]
            return rc


        rc_sequence = []

        current_t = 0
        i = 0
        while current_t < t:
            rc = rc_at_time(current_t)

            rc_sequence.append(rc)

            current_t += self.durations[i]
            i += 1

        # add last rc for current control
        rc = rc_at_time(t)
        rc_sequence.append(rc)

        return rc_sequence

    def linspace(self, delta, start_t=0, end_t=-1):
        x_array = []
        y_array = []
        theta_array = []

        if end_t == -1:
            end_t = self.end_time

        #print(end_t)

        for t in np.arange(start_t, end_t, delta):
            x, y, theta = self.config_at_t(t)
            x_array.append(x)
            y_array.append(y)
            theta_array.append(theta)


        return x_array, y_array, theta_array

def test_planar_trajectory_1():
    traj = PlanarTrajectory(controls_rs, 0, 0, .5, [0, 3, 5, 3], [1.0, 2.0, 2.0, 4.0])

    x, y, theta = traj.config_at_t(.99)

    #T = traj.transform[2]
    #x, y, theta = config_from_transform(T)

    print(f'x: {x},   y: {y},   theta: {theta}')



    x, y, theta = traj.config_at_t(1.01)
    print(f'x: {x},   y: {y},   theta: {theta}')


if __name__ == "__main__":
    test_planar_trajectory_1()
