import numpy as np
from RRTTree import RRTTree
import time

class RRTPlannerNonholonomic(object):
    def __init__(self, planning_env, bias=0.05, max_iter=10000, num_control_samples=25):
        self.env = planning_env                 # Car Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                        # Goal Bias
        self.max_iter = max_iter                # Max Iterations
        self.num_control_samples = 25           # Number of controls to sample

    def Plan(self, start_config, goal_config):
        plan = []
        plan_time = time.time()
        x_new_id = self.tree.GetRootID()
        # TODO: YOUR IMPLEMENTATION HERE

        # YOUR IMPLEMENTATION END HERE
        cost = 0
        while x_new_id != self.tree.GetRootID():
            cost+=self.tree.costs[x_new_id]
            plan.insert(1, self.tree.vertices[x_new_id])
            x_new_id = self.tree.edges[x_new_id]
        plan_time = time.time() - plan_time
        print("Cost: %f" % cost)
        if len(plan)>0:
            return np.concatenate(plan, axis=1)

    def extend(self, x_near, x_rand):
        """ Extend method for non-holonomic RRT

            Generate n control samples, with n = self.num_control_samples
            Simulate trajectories with these control samples
            Compute the closest closest trajectory and return the resulting state (and cost)
        """
        # TODO: YOUR IMPLEMENTATION HERE
        pass 
                
    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal

        return self.env.sample()