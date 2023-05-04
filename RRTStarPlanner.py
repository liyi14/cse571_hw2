from __future__ import absolute_import, print_function
import numpy as np
from RRTTree import RRTTree
import sys
import time
from tqdm import tqdm

class RRTStarPlanner(object):

    def __init__(self, planning_env, bias = 0.05, eta = 1.0, max_iter = 10000):
        self.env = planning_env         # Map Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                # Goal Bias
        self.max_iter = max_iter        # Max Iterations
        self.eta = eta                  # Distance to extend

    def compute_cost(self, node_id):
        root_id = self.tree.GetRootID()

        cost = 0
        node = self.tree.vertices[node_id]
        while node_id != root_id:
            parent_id = self.tree.edges[node_id]
            parent = self.tree.vertices[parent_id]
            cost += self.env.compute_distance(node, parent)

            node_id = parent_id
            node = parent

        return cost

    def Plan(self, start_config, goal_config, rad=10):
        # TODO: YOUR IMPLEMENTATION HERE
        pass

    def extend(self, x_near, x_rand):
        # TODO: YOUR IMPLEMENTATION HERE
        pass

    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal

        return self.env.sample()