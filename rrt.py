# Author: Chris Miller '20
import math
from planar_trajectory import PlanarTrajectory
from shapely.geometry import *
import numpy as np
import random
from sklearn.neighbors import NearestNeighbors
from collections import deque
from configNode import ConfigNode
import matplotlib.pyplot as plt


class RRT:

    def __init__(self, xbounds, ybounds, obs = [], num_samples = 15000,
                xy_ep = .1, t_ep = math.pi/16):
        self.edges = {}
        self.vertices = []
        self.vertex_data = []
        self.num_samples = num_samples
        self.expanded = set()
        self.obs = obs

        self.min_x = xbounds[0]
        self.max_x = xbounds[1]
        self.min_y = ybounds[0]
        self.max_y = ybounds[1]

        self.xs = []
        self.ys = []

        self.theta_scaling = 5
        self.xy_epsilon = xy_ep
        self.theta_epsilon = t_ep
        # From planarsim
        self.controls = np.array([
                    [1, 0, 0],
                    [-1, 0, 0],
                    [1, 0, -1],
                    [1, 0, 1],
                    [-1, 0, -1],
                    [-1, 0, 1]])

#=============================== GRAPH HANDLING ===============================
    # Adds a vertex v1
    def add_vertex(self, vertex):
        self.vertices.append(vertex)
        self.vertex_data.append(vertex.config)

    def backchain(self, sol, config, actions):
        actions.insert(0, config.action)
        sol.insert(0, config)
        config = config.get_parent()

        while (config != None):
            sol.insert(0, config)
            actions.insert(0, config.action)
            config = config.get_parent()


#==============================================================================


#================================ ANN HANDLING ================================

    # Gets nearest neighbors (default 1) of a configuration, training if necessary
    def get_NN(self, config, num_neighbors = 1, train = True, return_distance = False):
        if train:
            self.train_NN_model()
        num_neighbors = min(num_neighbors, len(self.vertex_data))
        return self.model.kneighbors(config.reshape(1, -1), num_neighbors,
                                    return_distance = return_distance)

    # Trains NN model on vertices
    def train_NN_model(self):
        self.model = NearestNeighbors(10, algorithm = 'auto', metric = self.config_dist)
        self.model.fit(np.asarray(self.vertex_data))

#===============================================================================

#=============================== CONFIG HANDLING ===============================
    # Checks if a point is within defined epsilons of the goal
    def check_if_goal(self, new_loc, goal):
        xval = (new_loc[0] < goal[0] + self.xy_epsilon) and \
                (new_loc[0] > goal[0] - self.xy_epsilon)
        yval = (new_loc[1] < goal[1] + self.xy_epsilon) and \
                (new_loc[1] > goal[1] - self.xy_epsilon)
        tval = (new_loc[2] < goal[2] + self.theta_epsilon) and \
                (new_loc[2] > goal[2] - self.theta_epsilon)

        return xval and yval and tval

    # Checks if a point is too close to an existing point and should not be added
    def check_if_redundant(self, new_loc):
        conf = self.get_NN(new_loc, train = False)

        near = self.vertex_data[conf[0][0]]
        xdist = abs(near[0] - new_loc[0])
        ydist = abs(near[1] - new_loc[1])
        tdist = self.ang_dist(near[2], new_loc[2])

        return xdist < self.xy_epsilon and ydist < self.xy_epsilon and tdist < self.theta_epsilon


    # Returns true if the configuration is valid (the point is not within any obstacle)
    def is_valid(self, config):
        carpt = Point(config[0], config[1])

        for obs in self.obs:
            if obs.contains(carpt):
                return False
        return True

    # Return true if the move is valid (the path does not intersect with any obstacle)
    def is_valid_move(self, start, end):

        line = LineString([(start[0], start[1]), (end[0], end[1])])
        for obstacle in self.obs:
            if line.intersects(obstacle):
                return False
        return True


    def config_dist(self, config1, config2):
        xdist = math.pow(config1[0] - config2[0], 2)
        ydist = math.pow(config1[1] - config2[1], 2)
        tdist = self.ang_dist(config1[2], config2[2])

        dist = tdist * self.theta_scaling + math.sqrt(xdist + ydist)

        return dist

    # Computes the minimum angular distance between two angles measured in radians
    def ang_dist(self, ang1, ang2):
        d1 = abs(ang1 - ang2)
        d2 = abs(2 * math.pi - d1)
        return min(d1, d2)

    # Returns a random state within the xy bounds and with theta between -pi and pi
    def get_rand_config(self):
        state = np.zeros(3)
        state[0] = random.uniform(self.min_x, self.max_x) # x coord
        state[1] = random.uniform(self.min_y, self.max_y) # y coord
        state[2] = random.uniform(-math.pi, math.pi) # theta value
        return state

    # Gets a random valid configuration
    def get_rand_valid_config(self):
        config = self.get_rand_config()
        while not self.is_valid(config):
            config = self.get_rand_config()

        return config

#===============================================================================

#================================== RRT CODE ===================================

    # Builds the RRT
    def build_rand_tree(self, start, goal = None, vis = True, stepdist = 1):
        found_goal = False
        goal_node = None
        sol = []
        i = 0

        if vis:
            fig, ax = plt.subplots()

            if goal is not None:
                ax.plot(goal[0], goal[1], 'go', markersize = 4)

            ax.set_ylim([self.min_y, self.max_y])
            ax.set_xlim([self.min_x, self.max_x])
            plt.show(block=False)

            if self.obs:
                for ob in self.obs:
                    x, y = ob.exterior.xy
                    ax.plot(x, y)

        starter = ConfigNode(start)
        self.add_vertex(starter)

        while i < self.num_samples:
            rpoint = self.get_rand_valid_config()
            nearest = self.get_NN(rpoint)[0][0]
            tgt_node = self.vertices[nearest]
            if vis:
                par_pt = (tgt_node.config[0], tgt_node.config[1])


            # Validate that it hasn't been expanded yet
            if str(tgt_node.config) not in self.expanded:
                for j in range(len(self.controls)):
                    new_loc = np.zeros(3)
                    actionseq = [j]
                    durations = [stepdist + .01] # one second action by default - potentially change if near goal?
                    nearSim = PlanarTrajectory(self.controls, tgt_node.config[0], tgt_node.config[1],
                        tgt_node.config[2], actionseq, durations)

                    new_loc[0], new_loc[1], new_loc[2] = nearSim.config_at_t(stepdist)


                    # Only adds if it's not too close to an existing point
                    # And the trajectory is valid
                    if (not self.check_if_redundant(new_loc)) and self.is_valid_move(tgt_node.config, new_loc):
                        new_node = ConfigNode(new_loc, tgt_node, j)
                        self.add_vertex(new_node)
                        i += 1

                        if vis:
                            self.xs.append(par_pt[0])
                            self.ys.append(par_pt[1])
                            self.xs.append(new_loc[0])
                            self.ys.append(new_loc[1])
                            xd = [par_pt[0], new_loc[0]]
                            yd = [par_pt[1], new_loc[1]]
                            plt.plot(xd, yd, '-r', linewidth = .5)

                        if goal is not None:
                             if self.check_if_goal(new_loc, goal):
                                 found_goal = True
                                 goal_node = new_node
                                 break
                if vis:
                    plt.plot(self.xs, self.ys, 'bo', markersize=1)

                    plt.pause(.00001)

            if found_goal:
                break



        if found_goal:
            print("Goal found.")
            actions  = []
            self.backchain(sol, goal_node, actions)
            print(sol)
            fig, ax = plt.subplots()

            ax.plot(goal[0], goal[1], 'go', markersize = 10)
            ax.plot(start[0], start[1], 'bo', markersize = 10)

            if self.obs:
                for ob in self.obs:
                    x, y = ob.exterior.xy
                    ax.plot(x, y)


            xs = []
            ys = []

            for pt in sol:
                conf = pt.config
                xs.append(conf[0])
                ys.append(conf[1])

            ax.plot(xs,ys, '-g', linewidth = 2)
            plt.show()

            print("Actions:")
            print(actions)

        else:
            print("Goal not found. Try increasing number of samples.")


# Helper code to generate some obstacles
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
    random.seed(2)
    xbounds = [0, 5]
    ybounds = [0, 5]

    obs = gen_obstacles()

    rrt = RRT(xbounds, ybounds, obs = obs, num_samples=8000)

    # Start and goal
    st = np.zeros(3)
    st[0] = 2
    st[1] = 2
    st[2] = math.pi/4
    g = np.zeros(3)
    g[0] = 4
    g[1] = 2.5
    g[2] = -math.pi/4
    rrt.build_rand_tree(st, goal = g, vis = False, stepdist=.5)
