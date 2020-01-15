import queue as q
from heapq import heapify
import numpy as np
from matplotlib import pyplot as plt


VISUALISE = True

class MultiHeuristicPlanner(object):
    def __init__(self, planning_env, guidance, w1=20, w2=2.5):
        """

        :param planning_env: The planning environment for the algorithm
        :param guidance: a list of tuples containing the user guidance points
        :param w1: inflation parameter of individual searches
        :param w2: The factor of comparison between anchor heuristic and inadmissible heuristics 
        """
        self.guidance = guidance
        self.planning_env = planning_env
        self.nodes = dict()
        self.n = 2                          # number of heuristics
        self.w1 = w1
        self.w2 = w2

    '''
    This function assume the input start_config and goal_config matches the start and goal
    in the environment object.
    '''

    def Plan(self, start_config, goal_config):
        plan = []
        map_shape = self.planning_env.map.shape
        start_config = tuple(start_config)
        goal_config = tuple(goal_config)
        self.g = np.full(map_shape, float('Inf'))
        self.g[start_config] = 0
        self.g[goal_config] = float('inf')
        self.ancestor_of_g = np.zeros(map_shape, dtype=np.bool)

        self.bp = dict()
        self.bp[start_config] = None
        self.bp[goal_config] = None

        self.o_nodes = [None] * self.n
        for i in range(self.n):
            self.o_nodes[i] = q.PriorityQueue()
            self.o_nodes[i].put((self.key_heuristic(start_config, i),start_config))

        self.c_nodes_anchor = []
        self.c_nodes_inad =[]

        minkey = [float('inf')] * self.n
        bf = False
        j=0
        while True:
            if VISUALISE and not j%1000:
                self.planning_env.visualize_plan(visited=self.g)
            j+=1
            minkey[0] = self.o_nodes[0].queue[0][0]
            for i in range(1,self.n):
                minkey[i] = self.o_nodes[i].queue[0][0]
                if minkey[i] <= self.w2 * minkey[0]:
                    if self.g[goal_config] <= minkey[i]:
                        if self.g[goal_config] < float('inf'):
                            bf = True
                            break
                    else:
                        _, s =self.o_nodes[i].get()
                        self.expand_state(s)
                        self.c_nodes_inad.append(s)
                else:
                    if self.g[goal_config] <= minkey[0]:
                        if self.g[goal_config] < float('inf'):
                            bf = True
                            break
                    else:
                        _, s =self.o_nodes[0].get()
                        self.expand_state(s)
                        self.c_nodes_anchor.append(s)

            if bf is True:
                break

        plan.append(goal_config)
        current = goal_config
        while True:
            if current == start_config:
                break
            plan.append(self.bp[current])
            current = self.bp[current]

        return np.array(plan)

    def key_heuristic(self, s, i):
        if i == 0:
            return self.g[s] + self.w1 * self.planning_env.compute_heuristic(s)
        elif i == 1:
            if self.ancestor_of_g[s]:
                return self.g[s] + self.w1 * self.planning_env.compute_heuristic(s)
            else:
                return self.g[s] + self.w1 * (self.planning_env.compute_distance(s, self.guidance) + self.planning_env.compute_heuristic(self.guidance))

    def expand_state(self, s):
        c = self.planning_env.compute_distance

        for sp in self.find_neighbours(s, self.planning_env.map.shape):
            if not self.planning_env.state_validity_checker(sp):
                continue
            try:
                self.g[sp]
            except:
                self.g[sp] = float('inf')
                self.bp[sp] = None
            if self.g[sp] > self.g[s] + c(s, sp):
                self.g[sp] = self.g[s] + c(s, sp)
                self.bp[sp] = s
                if sp not in self.c_nodes_anchor:
                    self.insert_update(sp, 0)
                    if (s == self.guidance) or self.ancestor_of_g[s]:
                        self.ancestor_of_g[sp] = True
                    if sp not in self.c_nodes_inad:
                        for i in range(1, self.n):
                            if self.key_heuristic(sp, i) <= self.w2 *self.key_heuristic(sp, 0):
                                self.insert_update(sp, i)
                                # TODO update if guidance point is ancestor

    def insert_update(self, s, i):
        new_key = self.key_heuristic(s,i)
        p = [x for x in self.o_nodes[i].queue if x[1] == s]
        if not p:
            self.o_nodes[i].put((new_key, s))
        else:
            self.o_nodes[i].queue = [(new_key, s) if x[1] == s else x for x in self.o_nodes[i].queue]
            heapify(self.o_nodes[i].queue)

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
            neighbours.append((config[0], config[1] - 1))

        if config[1] < map_shape[1] - 1:
            neighbours.append((config[0], config[1] + 1))

        return neighbours