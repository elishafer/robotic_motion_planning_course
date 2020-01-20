import sys
import time
import numpy as np

VISUALISE = False

class AStarPlanner(object):    
    def __init__(self, planning_env, w=20.0):
        self.planning_env = planning_env
        self.nodes = dict()
        self.w = w

    def Plan(self, start_config, goal_config):

        plan = []
        start_config = tuple(start_config)
        goal_config = tuple(goal_config)

        # TODO (student): Implement your planner here.
        map_shape = self.planning_env.map.shape
        compute_heuristic = self.planning_env.compute_heuristic
        compute_distance = self.planning_env.compute_distance
        g = np.full(map_shape, float('Inf'))
        g[start_config] = 0
        f = np.full(map_shape, float('Inf'))
        f[start_config] = compute_heuristic(start_config)
        open_set = {start_config: 0}
        came_from = dict()
        j = 0
        while True:
            if VISUALISE and not j%1000:
                self.planning_env.visualize_plan(visited=g)
            j+=1
            current = min(open_set, key=open_set.get)
            if current == goal_config:
                break

            open_set.pop(current)

            neighbours = self.find_neighbours(current, map_shape)

            for neighbour in neighbours:
                if not self.planning_env.state_validity_checker(neighbour):
                    continue
                t_g = g[tuple(current)] + compute_distance(current, neighbour)
                if t_g < g[neighbour]:
                    came_from[neighbour] = current
                    g[neighbour] = t_g
                    f[neighbour] = g[neighbour] + self.w * compute_heuristic(neighbour)
                    if neighbour not in open_set:
                        open_set[neighbour] = f[neighbour]

        plan.append(goal_config)
        current = goal_config
        while True:
            if current == start_config:
                break
            plan.append(came_from[current])
            current = came_from[current]

        expansion_count = np.count_nonzero(~np.isinf(g))
        print('expanded nodes:', expansion_count)
        print(f"Cost of path: {g[goal_config]}")
        # return np.array(plan), g[goal_config], f
        return np.array(plan), g


    def find_neighbours(self, config, map_shape):

        neighbours = []
        if config[0] != 0:
            neighbours.append((config[0] - 1, config[1]))
            if config[1] != 0:
                neighbours.append((config[0] - 1, config[1] - 1))
            if config[1] < map_shape[1] - 1:
                neighbours.append((config[0] - 1, config[1] + 1))

        if config[0] < map_shape[0] - 1:
            neighbours.append((config[0] + 1, config[1]))
            if config[1] != 0:
                neighbours.append((config[0] + 1, config[1] - 1))
            if config[1] < map_shape[1] - 1:
                neighbours.append((config[0] + 1, config[1] + 1))

        if config[1] != 0:
            neighbours.append((config[0], config[1] -1))

        if config[1] < map_shape[1] - 1:
            neighbours.append((config[0], config[1] + 1))

        return neighbours


    def ShortenPath(self, path):

        # TODO (student): Postprocess the planner.
        return path
