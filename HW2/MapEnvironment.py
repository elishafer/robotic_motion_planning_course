import numpy as np
from IPython import embed
from matplotlib import pyplot as plt
from math import hypot
from skimage import draw

class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal):

        # Obtain the boundary limits.
        # Check if file exists.
        self.goal = goal
        self.map = np.loadtxt(mapfile)
        self.xlimit = [1, np.shape(self.map)[0]] # TODO (avk): Check if this needs to flip.
        self.ylimit = [1, np.shape(self.map)[1]]

        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)

        # Display the map
        plt.imshow(self.map, interpolation='nearest')


    def compute_distance(self, start_config, end_config):

        return hypot(start_config[0] - end_config[0], start_config[1] - end_config[1])


    def state_validity_checker(self, config):

        if self.map[tuple(config)] == 0:
            return True
        else:
            return False


    def edge_validity_checker(self, config1, config2):

        line = draw.line(config1[0], config1[1],
                         config2[0], config2[1])
        a = np.where(self.map[line] >=1)
        if np.size(a) == 0:
            return True
        else:
            return False

    def compute_heuristic(self, config):
        return self.compute_distance(config, self.goal)

    def visualize_plan(self, plan=None, visited=None, tree=None, title=None):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        plt.imshow(self.map, interpolation='nearest', cmap='Greys')
        if visited is not None:
            plt.imshow(visited)
        elif tree is not None:
            nodes = tree.vertices
            for k, v in tree.edges.items():
                plt.plot([nodes[k][1], nodes[v][1]], [
                    nodes[k][0], nodes[v][0]], "-g")
        if plan is not None:
            for i in range(np.shape(plan)[0] - 1):
                x = [plan[i,0], plan[i+1, 0]]
                y = [plan[i,1], plan[i+1, 1]]
                plt.plot(y, x, 'r')
        if title: plt.title(title)
        plt.show()