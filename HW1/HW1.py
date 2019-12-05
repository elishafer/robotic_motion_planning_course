import argparse
import os
from typing import List, Tuple

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString, orient
from shapely.geometry import Point
from math import atan2
from math import pi
import numpy as np
import heapq


def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """

    # Reorient the polygon so that vertices are in clockwise direction
    original_shape = orient(original_shape, sign=1.0)
    obstacle = np.array(original_shape.exterior.coords)
    obstacle = np.append(obstacle[:-1], obstacle[:2], 0)
    robot = np.array([(0, -r), (r, 0), (0, r), (-r, 0), (0, -r), (r, 0)])
    m = len(obstacle) - 2
    n = 4
    poly_sum = []
    i = 0
    j = 0
    while i < n or j < m:
        poly_sum.append(robot[i] + obstacle[j])
        robot_diff = robot[i + 1] - robot[i]
        obstacle_diff = obstacle[j + 1] - obstacle[j]
        angle_robot = atan2(robot_diff[1], robot_diff[0])

        # Make sure all angles are positive
        if angle_robot < 0:
            angle_robot += 2 * pi
        angle_obs = atan2(obstacle_diff[1], obstacle_diff[0])
        if angle_obs < 0:
            angle_obs += 2 * pi

        # If we complete all vertices on the polygon, we have completed a revolution.
        # All angles thereafter are the angle plus one revolution.
        if i >= n:
            angle_robot += 2 * pi
        if j >= m:
            angle_obs += 2 * pi

        if angle_robot < angle_obs:
            i += 1
        elif angle_robot > angle_obs:
            j += 1
        else:
            i += 1
            j += 1

    return Polygon(poly_sum)


def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    vis_graph = []
    # Create a list of all vertices of polygons
    v_list = [vertex for obstacle in obstacles for vertex in obstacle.exterior.coords[:-1]]
    if source is not None:
        v_list.append(source)
    if dest is not None:
        v_list.append(dest)
    # for each vertice connect to all other vertices and collision check
    for i, v in enumerate(v_list):
        for j, w in enumerate(v_list[i + 1:]):
            crosses = False
            line = LineString([v, w])
            for obstacle in obstacles:
                if line.within(obstacle) or line.crosses(obstacle):
                    crosses = True
                    break
            if not crosses:
                vis_graph.append(line)

    return vis_graph


def dijkstra(lines: List[LineString], start: tuple, goal: tuple):
    X = []   # A list of nodes using Node class. This is a list of visited nodes.
    X_v = [] # A list of vertice coordinates that we've already calculated the cost to.
    # create a list of vertices:
    V = []
    for line in lines:
        if line.coords[0] not in V: V.append(line.coords[0])
        if line.coords[1] not in V: V.append(line.coords[1])
    neighbours = {v: [] for v in V}
    for line in lines:
        neighbours[line.coords[0]] += [(line.coords[1], line.length)]
        neighbours[line.coords[1]] += [(line.coords[0], line.length)]
    h = []  # shortest edges heap
    heapq.heappush(h, (0, (start,None)))    # Initialise heap with node (cost, (coords, parent)
    i = 0
    while i <= len(V):                   # actually loop should end on number of vertices.
        cost, node = heapq.heappop(h)
        node = Node(*node)
        node.cost = cost
        X.append(node)

        if X[-1].x == goal:
            # We found the goal, exit the while loop
            break

        for j, neighbour in enumerate(neighbours[X[-1].x]):
            # The following updates keys in the heap to use the smallest cost obtained after
            # adding the last node to explored.
            neighbour_coords = neighbour[0]
            neighbour_dist   = neighbour[1]
            # The following is to check if we've already calculated a the cost to the neighbour
            for e,v in enumerate(h):
                node = Node(*v[1])
                node.cost = v[0]
                if neighbour_coords == node.x:
                    if X[-1].cost + neighbour_dist < node.cost:
                        node.parent = len(X) - 1
                        node.cost = X[-1].cost + neighbour_dist
                        # The following removes the vertex from the heap:
                        h[e] = h[-1]
                        h.pop()
                        if e < len(h):
                            heapq._siftup(h, e)
                            heapq._siftdown(h, 0, e)
                        heapq.heappush(h, (node.cost, (node.x, node.parent)))

        for j, neighbour in enumerate(neighbours[X[-1].x]): # Gives a total of n heap operations
            neighbour_coords = neighbour[0]
            neighbour_dist = neighbour[1]
            # The following we add nodes that we haven't visited yet to the heap:
            if neighbour_coords not in X_v:                           # check that we've not visited it yet
                heapq.heappush(h, (neighbour_dist + X[-1].cost,
                                   (neighbour_coords, len(X)-1)))     # add to heap
                X_v.append(neighbour_coords)                        # add to list of visited nodes
                # del neighbours[X[-1].x][j]
        i += 1

    path = []
    last_index = len(X) -1
    while X[last_index].parent is not None:
        node = X[last_index]
        path.append(node.x)
        last_index = node.parent
    path.append(start)
    path.reverse()

    return path, X[-1].cost


class Node():

    def __init__(self, x, parent=None, cost=0.0):
        self.x = x
        self.parent = parent
        self.cost = cost


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot",
                        help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            points = [tuple(map(float, t.split(','))) for t in line.replace('\n', '').split(' ')]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()

    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    # TODO: fill in the next line

    shortest_path, cost = dijkstra(lines, source, dest)

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))

    plotter3.show_graph()
