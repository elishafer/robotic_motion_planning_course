#!/usr/bin/env python

import argparse
import numpy as np
from time import time
from statistics import mean, stdev

from MapEnvironment import MapEnvironment
from RRTPlanner import RRTPlanner
from RRTStarPlanner import RRTStarPlanner
from AStarPlanner import AStarPlanner


from IPython import embed

def main(planning_env, planner, start, goal, planner_type):

    # Notify.
    input('Press any key to begin planning')

    # Plan.
    if planner_type == 'astar':
        plan, cost, visited = planner.Plan(start, goal)
        print('cost avg: ', cost)
    else:
        cost_list = []
        time_list = []
        for i in range(10):
            time_start = time()
            plan, cost, tree = planner.Plan(start, goal)
            time_list.append(time() - time_start)
            cost_list.append(cost)
            if i < 9:
                planner.tree.ResetTree()
        cost = mean(cost_list)
        print('cost avg: ', cost)
        print('cost stdev:', stdev(cost_list))
        print('time avg', mean(time_list))
        print('time stdev:', stdev(time_list))


    # Shortcut the path.
    # TODO (student): Do not shortcut when comparing the performance of algorithms. 
    # Comment this line out when collecting data over performance metrics.
    plan_short = planner.ShortenPath(plan)

    # Visualize the final path.
    if planner_type == 'astar':
        planning_env.visualize_plan(plan, visited)
    else:
        planning_env.visualize_plan(plan, tree=tree)

    # planning_env.
    embed()


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-m', '--map', type=str, default='map1.txt',
                        help='The environment to plan on')    
    parser.add_argument('-p', '--planner', type=str, default='rrt',
                        help='The planner to run (star, rrt, rrtstar)')
    parser.add_argument('-s', '--start', nargs='+', type=int, required=True)
    parser.add_argument('-g', '--goal', nargs='+', type=int, required=True)

    args = parser.parse_args()

    # First setup the environment and the robot.
    planning_env = MapEnvironment(args.map, args.start, args.goal)

    # Next setup the planner
    if args.planner == 'astar':
        planner = AStarPlanner(planning_env)
    elif args.planner == 'rrt':
        planner = RRTPlanner(planning_env)
    elif args.planner == 'rrtconnect':
        planner = RRTStarPlanner(planning_env)
    else:
        print('Unknown planner option: %s' % args.planner)
        exit(0)

    main(planning_env, planner, args.start, args.goal, args.planner)
