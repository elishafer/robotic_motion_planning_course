import numpy as np
from RRTTree import RRTTree
import random
from math import log, e
from time import time

class RRTStarPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.map_shape = self.planning_env.map.shape
        self.bounds = [(0, self.map_shape[0] - 1), (0, self.map_shape[1] - 1)]

    def Plan(self, start_config, goal_config, eta=float(5.0), goal_sample_rate=5, timeout=float(1.0)):

        start_time = time()
        compute_distance = self.planning_env.compute_distance

        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        self.tree.SetCost(0, 0)

        # TODO (student): Implement your planner here.
        while True:
            x_rand = self.sample(goal_sample_rate)
            v_nearest_id, v_nearest = self.tree.GetNearestVertex(x_rand)
            # v_nearest = self.tree.vertices[v_nearest_id]
            v_new = self.extend(v_nearest, x_rand, eta)
            v_new = np.int_(v_new.round())
            if self.planning_env.state_validity_checker(v_new):
                k = int(e*1.5*log(len(self.tree.vertices)))
                knn_ids = list(self.tree.GetKNN(v_new, k))
                if not self.planning_env.edge_validity_checker(v_nearest, v_new):
                    continue
                c_min = self.tree.cost[v_nearest_id] + compute_distance(v_nearest, v_new)


                for j, v_near_id in enumerate(knn_ids):
                    v_near = self.tree.vertices[v_near_id]
                    if not self.planning_env.edge_validity_checker(v_near, v_new):
                        continue
                    c_j = self.tree.cost[v_near_id] + compute_distance(v_near, v_new)

                    if c_j < c_min:
                        c_min = c_j
                        v_nearest_id = v_near_id
                        v_nearest = v_near

                v_new_id = len(self.tree.vertices)
                self.tree.AddVertex(v_new)
                self.tree.AddEdge(v_nearest_id, v_new_id)
                self.tree.SetCost(v_new_id,
                                  self.tree.cost[v_nearest_id] + compute_distance(v_nearest, v_new))

                for j, v_near_id in enumerate(knn_ids):
                    v_near = self.tree.vertices[v_near_id]
                    if not self.planning_env.edge_validity_checker(v_near, v_new):
                        continue
                    c_j = self.tree.cost[v_new_id] + compute_distance(v_near, v_new)
                    if c_j < self.tree.cost[v_near_id]:
                        self.tree.AddEdge(v_new_id, v_near_id)
                        self.tree.SetCost(v_near_id, c_j)

            # self.planning_env.visualize_plan(tree=self.tree)

            if time() - start_time > timeout:
                break

        best_vid = self.get_best_vid(goal_config, 1.0)
        if best_vid is None:
            return None
        print('goal reached!')
        total_cost = self.tree.cost[best_vid]
        plan.append(goal_config)
        last_index = best_vid
        while self.tree.edges[last_index] != 0:
            plan.append(self.tree.vertices[last_index])
            last_index = self.tree.edges[last_index]
        plan.append(start_config)

        return np.array(plan), total_cost, self.tree

    def extend(self, v_nearest, x_rand, eta):
        extend_length = self.planning_env.compute_distance(x_rand, v_nearest)
        if extend_length > eta:
            return ((x_rand - v_nearest) * eta) / extend_length + v_nearest
        else:
            return x_rand

    def sample(self, goal_sample_rate):
        if random.randint(0, 100) > goal_sample_rate:
            x_rand = [random.uniform(x[0], x[1]) for x in self.bounds]
        else:
            x_rand = self.planning_env.goal
        x_rand = np.array(x_rand)
        return x_rand

    def get_best_vid(self, goal_config, r):
        dist_goal_list = [self.planning_env.compute_distance(v, goal_config) for v in self.tree.vertices]
        goal_ids = [dist_goal_list.index(i) for i in dist_goal_list if i <= r]

        if not goal_ids:
            print('goal not reached')
            return None

        mincost = min(self.tree.cost[i] for i in goal_ids)
        for i in goal_ids:
            if self.tree.cost[i] == mincost:
                return i
        return None

    def ShortenPath(self, path):
        # TODO (student): Postprocessing of the plan.
        return path
