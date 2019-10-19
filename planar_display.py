
from planar_trajectory import *
from planarsim import *

from matplotlib import pyplot as plt
from matplotlib import animation
from shapely.geometry import *

from matplotlib.animation import FFMpegWriter
import math
import numpy as np
'''
Code provided by Devin Balkcom
'''

class CarShapes:
    def __init__(self):
        # points in the local coordinate frame
        self.points = [[0, -.25], [.75, -.25], [1, 0], [.75, .25], [0, .25]]
        self.q = [0, 0, 0]    # initial configuration

        self.update_patch()

    def compute_transformed(self):
        T = transform_from_config(self.q)

        transformed_points = []

        for point in self.points:
            homog_point = [point[0], point[1], 1]
            transformed_point = T @ homog_point
            tp = [transformed_point[0], transformed_point[1]]
            transformed_points.append(tp)

        return transformed_points


    def set_configuration(self, q):
        self.q = q
        self.update_patch()

    def update_patch(self):
        transformed_points = self.compute_transformed()
        self.car_poly = plt.Polygon(transformed_points, edgecolor='gray', facecolor="lightgray")

    def draw(self, ax):
        ax.add_patch(self.car_poly)


class TrajectoryView:
    def __init__(self, traj):
        self.traj = traj


    def draw(self, ax, final_t):

        car = CarShapes()
        car.set_configuration(self.traj.config_at_t(final_t))
        car.draw(ax)

        traj_x, traj_y, traj_theta = self.traj.linspace(.03, 0, final_t)

        ax.plot(traj_x, traj_y, color='black')




def draw1():

    traj = PlanarTrajectory(controls_rs, 3, 0, .5, [0, 3, 5, 3], [1.0, 2.0, 2.0, 2.0])

    tview = TrajectoryView(traj)

    ax = plt.axes()

    tview.draw(ax, 6.99)



def animate1():

    time_step = .05
    traj = PlanarTrajectory(controls_rs, 0, 0, .5, [0, 3, 5, 3], [1.0, 2.0, 2.0, 2.0])

    fig = plt.figure(figsize=(8, 4.5))
    fig.tight_layout()

    ax = plt.axes()

    def animate(i):
        ax.clear()
        ax.set_xlim(-1, 10)
        ax.set_ylim(-3, 3)
        ax.set_aspect('equal')

        tview = TrajectoryView(traj)
        tview.draw(ax, i * .05)
        #print(f"hello: {i}")

    num_frames = traj.end_time / time_step
    anim = animation.FuncAnimation(fig, animate, frames=140, interval=20, repeat=False)

    plt.show()


def draw_rcs(ax, controls, q):
    rcs = compute_worldframe_rcs(controls, q)
    #print(rcs)
    for rc in rcs:
        if(rc[2] > 0):
            circle= plt.Circle((rc[0], rc[1]), radius=.1)
            ax.add_patch(circle)


# Animation!
def animate2(st, g, obs):
    time_step = .05
    traj = PlanarTrajectory(controls_rs, st[0], st[1], st[2],
        [2, 2, 0, 2, 4, 4, 5, 0, 3], [.5, .5, .5, .5, .5, .5, .5, .5, .5])

    fig = plt.figure(figsize=(8, 4.5))
    fig.tight_layout()

    ax = plt.axes()

    for ob in obs:
        x, y = ob.exterior.xy
        ax.plot(x, y)

    ax.plot(g[0], g[1], 'go', markersize = 10)
    ax.plot(st[0], st[1], 'bo', markersize = 10)

    def animate(i):
        ax.clear()
        ax.set_xlim(0, 5)
        ax.set_ylim(0, 5)
        ax.set_aspect('equal')

        tview = TrajectoryView(traj)
        tview.draw(ax, i * .05)
        #print(f"hello: {i}")

    num_frames = traj.end_time / time_step
    anim = animation.FuncAnimation(fig, animate, frames=140, interval=20, repeat=False)

    plt.show()

def save_movie(anim, filename):
    writer = FFMpegWriter(fps=30, metadata=dict(artist='Devin Balkcom'), bitrate=1800)
    anim.save(filename, writer=writer)

def gen_obstacles():
    obs = []
    points = [Point(1, .5), Point(1, 2), Point(1.5, 2), Point(2.5, 1)]
    ob = Polygon([[p.x, p.y] for p in points])
    obs.append(ob)
    points = [Point(4, 4.5), Point(4, 5), Point(4.5, 5), Point(4.5, 4.5)]
    ob = Polygon([[p.x, p.y] for p in points])
    obs.append(ob)
    points = [Point(0, 1.5), Point(.5, 3), Point(.25, 2)]
    ob = Polygon([[p.x, p.y] for p in points])
    obs.append(ob)
    return obs

if __name__ == '__main__':

    # standard size plot with 16:9 ratio to match youtube, do
    #  not change
    fig = plt.figure(figsize=(8, 4.5))
    fig.tight_layout()

    st = np.zeros(3)
    st[0] = 2
    st[1] = 2
    st[2] = math.pi/4
    g = np.zeros(3)
    g[0] = 4
    g[1] = 2.5
    g[2] = -math.pi/4

    animate2(st, g, gen_obstacles())

    plt.show()
