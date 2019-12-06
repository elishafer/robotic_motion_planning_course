import argparse
import os
from typing import List, Tuple, Dict

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString, orient
from shapely.geometry import Point
from math import atan2
from math import pi
import numpy as np

###########################################
# Algorithmic Motion Planning (236610)   ##
# John Noonan and Eli Shafer			 ##
# Homework 1							 ##
# November 2019							 ##
###########################################


def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
	"""
	Get the polygon representing the Minkowsky sum
	:param original_shape: The original obstacle
	:param r: The radius of the rhombus
	:return: The polygon composed from the Minkowsky sums
	"""
	# Reorient the polygon so that vertices are in counter-clockwise direction
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

def get_adjacency_list(lines: List[LineString]) -> Dict[Tuple, List[Tuple]]:
	"""
	Creates an Adjacency List from the list of edges in the visibility graph.
	"""
	adjacency_list = {}
	for line in lines:
		try:
			adjacency_list[line.coords[0]].append(line.coords[1])
		except:
			adjacency_list[line.coords[0]] = [line.coords[1]]
		try:
			adjacency_list[line.coords[1]].append(line.coords[0])
		except:
			adjacency_list[line.coords[1]] = [line.coords[0]]
	return adjacency_list	

def construct_path(parent_map: Dict[Tuple, Tuple], goal: Tuple) -> List[Tuple]:
	"""
	Constructs the path from the goal node to the start node. The start node is 
	known due to its parent being None.
	"""
	path_node = goal
	path = []
	while path_node:
		path.append(path_node)
		path_node = parent_map[path_node]
	path.reverse()
	return path

def dijkstra(lines: List[LineString], start: Tuple, goal: Tuple) -> Tuple[List[Tuple], float]:
	"""
	Performs Dijkstra's Algorithm
	"""
	adjacency_list = get_adjacency_list(lines)
	cost_map, parent_map = {}, {}
	unvisited = set(adjacency_list.keys())
	for node in unvisited:
		cost_map[node] = 0.0 if node == start else float('inf')
		parent_map[node] = None

	while unvisited:
		curr_node = min(unvisited, key = lambda node: cost_map[node])
		curr_cost = cost_map[curr_node]
		unvisited.remove(curr_node)
		adj_nodes = adjacency_list[curr_node]
		for adj_node in adj_nodes:
			cost_curr_adj = curr_cost + float(np.linalg.norm(np.array(curr_node) - np.array(adj_node)))
			prev_cost = cost_map[adj_node]
			if (cost_curr_adj < prev_cost):
				cost_map[adj_node] = cost_curr_adj
				parent_map[adj_node] = curr_node
		if curr_node == goal:
			break

	return construct_path(parent_map, goal), cost_map[goal]


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
			points = [tuple(map(float, t.split(','))) for t in line.replace('\n', '').split(' ') if t != '']
			# Ensure that the first vertex is one which has the minimum y-coordinate
			min_y_ind = np.array(points).argmin(0)[1]
			points = points[min_y_ind:] + points[:min_y_ind]
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
	shortest_path, cost = dijkstra(lines, source, dest)

	plotter3 = Plotter()
	plotter3.add_robot(source, dist)
	plotter3.add_obstacles(workspace_obstacles)
	plotter3.add_robot(dest, dist)
	plotter3.add_visibility_graph(lines)
	plotter3.add_shorterst_path(list(shortest_path))

	plotter3.show_graph()
