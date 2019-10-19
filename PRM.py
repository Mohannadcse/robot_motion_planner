# Code by Chris Miller '20
import numpy as np
import pylab as plt
import math
import random
from collections import deque
from configNode import ConfigNode
from shapely.geometry import *
from sklearn.neighbors import NearestNeighbors
import matplotlib.animation as animation
'''
Author: Chris Miller '20
'''
class PRM:
    def __init__(self, num_links, lengths, obs = None):
        self.num_links = num_links
        self.lengths = lengths
        self.thetas = np.zeros(self.num_links)
        self.pos = [None] * (self.num_links + 1)
        self.links = [None] * (self.num_links)
        self.obs = obs
        self.vertices = []
        self.edges = {}
        self.num_samples = 3000
        self.dist_epsilon = self.num_links*math.pi/16
        self.model = None


#=============================== GRAPH HANDLING ===============================
    # Adds a vertex v1
    def add_vertex(self, vertex):
        self.vertices.append(vertex)
        self.edges[str(vertex)] = []

    # Adds an undirected edge between v1 and v2
    def add_edge(self, v1, v2):
        self.edges[str(v1)].append(v2)
        self.edges[str(v2)].append(v1)

    # Performs a breadth first search, initialized at the starting state of the
    # given problem. Adapted from HW1 BFS code that I wrote.
    def bfs_search(self, start_connect, goal_connect):
        visited = set()
        to_visit = deque()
        num_visited = 0
        start = ConfigNode(start_connect)
        to_visit.append(start)
        found_goal = False
        sol = []

        while (len(to_visit) != 0):
            curr_config = to_visit.popleft()
            num_visited += 1

            if(str(curr_config.config) not in visited):
                if np.array_equal(curr_config.config, goal_connect):
                    self.backchain(sol, curr_config)
                    found_goal = True
                    break

                visited.add(str(curr_config.config))

                for linked_config in self.edges[str(curr_config.config)]:
                    new_node = ConfigNode(linked_config, curr_config)
                    to_visit.append(new_node)
        print("Visited " + str(num_visited) + " nodes in my journey")
        return sol

    def backchain(self, sol, config):
        sol.insert(0, config)
        config = config.get_parent()

        while (config != None):
            sol.insert(0, config)
            config = config.get_parent()

#============================ POSITION & LINK SETUP ============================

    # Returns a tuple (x, y) of vectors of x and y positions for each link,
    # starting with the ground link (0, 0)
    def get_xy_pos(self, thetas):
        x = [0] * (self.num_links + 1)
        y = [0] * (self.num_links + 1)
        pos = [None] * (self.num_links + 1)
        pos[0] = Point(0, 0)
        for i in range(1, self.num_links + 1):
            x = pos[i - 1].x + self.lengths[i - 1] * math.cos(np.sum(thetas[0:i]))
            y = pos[i - 1].y + self.lengths[i - 1] * math.sin(np.sum(thetas[0:i]))
            pos[i] = Point(x, y)

        return pos

    # Updates current xy positions of links
    def set_xy_pos(self):
        self.pos = self.get_xy_pos(self.thetas)

    # Returns linestrings for each arm segment
    def get_links(self, pos):
        links = []
        for i in range(len(pos) - 1):
            l = LineString([pos[i], pos[i+1]])
            links.append(l)
        return links

    # Sets link values based on positions
    def set_links(self):
        self.links = self.get_links(self.pos)


#=============================== VISUALIZATION ================================

    # Visualize a configuration given a set of positions of links
    def vis(self, pos, obs = None):
        xs = []
        ys = []
        for i in range(self.num_links + 1):
            xs.append(pos[i].x)
            ys.append(pos[i].y)

        fig, ax = plt.subplots()
        ax.plot(xs, ys)
        if obs is not None:
            for ob in obs:
                x, y = ob.exterior.xy
                ax.plot(x, y)
            if not self.check_state(self.links):
                ax.set_facecolor((1.0, 0.35, 0.35))
        ax.autoscale()
        plt.show()

    # Plot the current config from a set of thetas
    def vis_from_thetas(self, thetas):
        pos = self.get_xy_pos(thetas)
        self.vis(pos, obs = self.obs)

    # Visualize the current configuration
    def self_vis(self):
        self.vis(self.pos, self.obs)


#================================ ANIMATION CODE ===============================

    # Animation code based on code provided by Matplotlib at
    # https://matplotlib.org/gallery/animation/simple_anim.html
    def animate_sol(self):
        fig, ax = plt.subplots()
        max_len = sum(self.lengths)
        ax.set_xlim(-max_len, max_len)
        ax.set_ylim(-max_len, max_len)
        pos = self.get_xy_pos(self.solution[0].config)
        xs = []
        ys = []
        for i in range(self.num_links + 1):
            xs.append(pos[i].x)
            ys.append(pos[i].y)

        for ob in self.obs:
            x, y = ob.exterior.xy
            ax.plot(x, y)

        self.line, = ax.plot(xs, ys)

        anim = animation.FuncAnimation(fig, self.animate, frames = list(range(len(self.solution))),
            interval = 200, blit = False, save_count = 50)
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
        anim.save('lineer.mp4', writer=writer)
        plt.show()

    # Helper function called by animation.FuncAnimation to update animation
    def animate(self, i):
        pos = self.get_xy_pos(self.solution[i].config)
        xs = []
        ys = []
        for i in range(self.num_links + 1):
            xs.append(pos[i].x)
            ys.append(pos[i].y)

        self.line.set_xdata(xs)
        self.line.set_ydata(ys)
        return self.line


#======================== CONFIG GENERATION/VALIDATION ========================

    # Checks if a given state intersects with obstacles
    def check_state(self, links):
        for link in links:
            for obstacle in self.obs:
                if link.intersects(obstacle):
                    return False

        return True

    # Returns true iff a state is valid (links do not intersect with obstacles)
    def is_valid(self, config):
        return(self.check_state(self.get_links(self.get_xy_pos(config))))

    # Generates a random configuration between -pi and pi for each link
    def get_rand_config(self):
        thetas = np.zeros(self.num_links)
        for i in range(self.num_links):
            thetas[i] = random.uniform(-math.pi, math.pi)
        return thetas

    # Gets a random valid configuration
    def get_rand_valid_config(self):
        config = self.get_rand_config()
        while not self.is_valid(config):
            config = self.get_rand_config()

        return config

#============================== NEIGHBOR HANDLING ==============================

    # Computes the sum of angular distances between two configurations
    def config_dist(self, config1, config2):
        dist = 0
        for i in range(len(config1)):
            dist += self.ang_dist(config1[i], config2[i])
        return dist

    # Computes the minimum angular distance between two angles measured in radians
    def ang_dist(self, ang1, ang2):
        d1 = abs(ang1 - ang2)
        d2 = abs(2 * math.pi - d1)
        return min(d1, d2)

    # Gets up to num_neighbors NNs of a provided config
    def get_NNs(self, config, num_neighbors=15, train=True):
        if train:
            self.train_NN_model()
        num_neighbors = min(num_neighbors, len(self.vertices))
        return self.model.kneighbors(config.reshape(1, -1), num_neighbors)

    # Trains NN model on vertices
    def train_NN_model(self):
        self.model = NearestNeighbors(10, algorithm = 'auto', metric = self.config_dist)
        self.model.fit(np.asarray(self.vertices))


#============================ PROBABILISTIC ROADMAP ============================

    # Build map of configs
    def build_roadmap(self):
        N = self.num_samples
        i = 0
        while i < N:
            config = self.get_rand_valid_config()
            present = False # Deal with check for duplicate later

            if not present:
                if len(self.vertices) > 1:
                    (dists, configs) = self.get_NNs(config)
                    self.add_vertex(config)
                    i += 1
                    distvalid = 0

                    for j in range(len(configs[0])):
                        if dists[0][j] < self.dist_epsilon:
                            distvalid += 1
                            self.add_edge(config, self.vertices[configs[0][j]])
                else:
                    self.add_vertex(config)
                    i += 1

    # Query roadmap for a start and goal
    def query(self, start, goal):
        print("Attempting to connect " + str(start) + " with " + str(goal))
        num_connects = 10
        (sc_dist, sc_conf) = self.get_NNs(start, num_neighbors=num_connects, train=False)
        (gc_dist, gc_conf) = self.get_NNs(goal, num_neighbors=num_connects, train=False)
        start_connections = []
        # = self.vertices[sc_conf[0][0]]
        goal_connections = []
        #self.vertices[gc_conf[0][0]]
        fail = False

        for i in range(num_connects):
            if sc_dist[0][i] < self.dist_epsilon:
                start_connections.append(self.vertices[sc_conf[0][i]])
            if gc_dist[0][i] < self.dist_epsilon:
                goal_connections.append(self.vertices[gc_conf[0][i]])

        # Find the connection to the start and goal with the highest outdegree
        # and use that one to search.
        max_start_deg = -1
        max_starter = None
        for i in range(len(start_connections)):
            out = len(self.edges[str(start_connections[i])])
            if out > max_start_deg:
                max_start_deg = out
                max_starter = start_connections[i]

        max_goal_deg = -1
        max_goal = None
        for i in range(len(goal_connections)):
            out = len(self.edges[str(goal_connections[i])])
            if out > max_goal_deg:
                max_goal_deg = out
                max_goal = goal_connections[i]


        if not start_connections:
            print("Unable to connect start to sample graph. Try using more samples!")
            fail = True
        else:
            print("Linking start to " + str(max_starter))

        if not goal_connections:
            print("Unable to connect goal to sample graph. Try using more samples!")
            fail = True
        else:
            print("Linking goal to " + str(max_goal))

        if not fail:
            self.solution = self.bfs_search(max_starter, max_goal)

            if self.solution:
                print("Found path: " + str(self.solution))
                self.animate_sol()
            else:
                print("Unable to connect the start and goal. Try using more samples!")

# Helper functions to produce polygonal obstacles
def gen_obstacles():
    obs = []

    points = [Point(1, .5), Point(1, 2), Point(1.5, 2), Point(2.5, 1)]
    ob = Polygon([[p.x, p.y] for p in points])
    obs.append(ob)

    points = [Point(4, 4.5), Point(4, 5), Point(4.5, 5), Point(4.5, 4.5)]
    ob = Polygon([[p.x, p.y] for p in points])
    obs.append(ob)

    points = [Point(0, -2), Point(0, -1), Point(1, -1)]
    ob = Polygon([[p.x, p.y] for p in points])
    obs.append(ob)

    points = [Point(-3, 1.5), Point(-3, 2), Point(2, 2), Point(2, 1.5)]
    ob = Polygon([[p.x, p.y] for p in points])
    obs.append(ob)

    return obs

if  __name__ == '__main__':
    random.seed(1)
    obs = []
    num_links = 3
    lengths = [1, 2, 1]
    obs = gen_obstacles()
    perm = PRM(num_links, lengths, obs)
    perm.build_roadmap()
    # Random configuration for testing
    s = perm.get_rand_valid_config()
    g = perm.get_rand_valid_config()

    # Uncomment these and provide theta values to try it out
    # with custom configurations!
    '''
    s = np.zeros(num_links)
    s[0] = 0
    s[1] = 0
    s[2] = 0
    #
    g = np.zeros(num_links)
    g[0] = 0
    g[1] = 0
    g[2] = 0
    '''
    perm.query(s, g)
