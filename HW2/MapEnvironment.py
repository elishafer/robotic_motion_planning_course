import numpy
from IPython import embed
from matplotlib import pyplot as plt
from math import hypot
from skimage import draw

class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal):

        # Obtain the boundary limits.
        # Check if file exists.
        self.goal = goal
        self.map = numpy.loadtxt(mapfile)
        self.xlimit = [1, numpy.shape(self.map)[0]] # TODO (avk): Check if this needs to flip.
        self.ylimit = [1, numpy.shape(self.map)[1]]

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
        pass


    def compute_heuristic(self, config):
        return self.compute_distance(config, self.goal)

    def visualize_plan(self, plan):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        plt.imshow(self.map, interpolation='nearest')
        for i in range(numpy.shape(plan)[0] - 1):
            x = [plan[i,0], plan[i+1, 0]]
            y = [plan[i,1], plan[i+1, 1]]
            plt.plot(y, x, 'k')
        plt.show()