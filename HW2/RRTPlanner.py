import numpy as np
from RRTTree import RRTTree
import random

class RRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.map_shape = self.planning_env.map.shape
        self.bounds = [(0, self.map_shape[0] - 1), (0,self.map_shape[1] - 1)]
        

    def Plan(self, start_config, goal_config, eta=5.0, goal_sample_rate=5, max_iterations=10000):
        
        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)

        # TODO (student): Implement your planner here.
        for i in range(max_iterations):
            x_rand = self.sample(goal_sample_rate)
            v_nearest_id, dist = self.tree.GetNearestVertex(x_rand)
            v_nearest = self.tree.vertices[v_nearest_id]
            v_new = self.extend(v_nearest, x_rand, eta)
            v_new = np.int_(v_new.round())
            if self.planning_env.state_validity_checker(v_new):
                if self.planning_env.edge_validity_checker(v_nearest, v_new):
                    v_new_id = len(self.tree.vertices)
                    self.tree.AddVertex(v_new)
                    self.tree.AddEdge(v_nearest_id, v_new_id)
                else:
                    continue
            else:
                continue

            if tuple(v_new) == tuple(self.planning_env.goal):
                print('goal reached!')
                break

        plan.append(goal_config)
        last_index = len(self.tree.vertices) - 1
        while self.tree.edges[last_index] is not 0:
            plan.append(self.tree.vertices[last_index])
            last_index = self.tree.edges[last_index]
        plan.append(start_config)
        return np.array(plan)

    def extend(self, v_nearest, x_rand, eta):
        extend_length = self.planning_env.compute_distance(x_rand, v_nearest)
        if extend_length > eta:
            return ((x_rand - v_nearest)*eta) / extend_length + v_nearest
        else:
            return x_rand

    def sample(self, goal_sample_rate):
        if random.randint(0, 100) > goal_sample_rate:
            x_rand = [random.uniform(x[0],x[1]) for x in self.bounds]
        else:
            x_rand = self.planning_env.goal
        x_rand = np.array(x_rand)
        return x_rand

    def ShortenPath(self, path):
        # TODO (student): Postprocessing of the plan.
        return path
