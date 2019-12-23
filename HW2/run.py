#!/usr/bin/env python

import argparse
import numpy as np
from time import time
from statistics import mean, stdev
from copy import deepcopy

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
    elif planner_type == 'rrt':
        biases = [5, 20]
        etas = [float('inf'), 5]
        for bias in biases:
            for eta in etas:  
                cost_list = []
                time_list = []
                for i in range(10):
                    time_start = time()
                    plan, cost, tree = planner.Plan(start, goal, eta=eta, goal_sample_rate=bias)
                    time_list.append(time() - time_start)
                    cost_list.append(cost)
                    if i < 9:
                        planner.tree.ResetTree()
                cost = mean(cost_list)
                print(f"Results for bias={bias} and eta={eta}:")
                print('cost avg: ', cost)
                print('cost stdev:', stdev(cost_list))
                print('time avg', mean(time_list))
                print('time stdev:', stdev(time_list))
                if (eta == float('inf')):
                    title = f"Final State of Tree for E1 with bias probability = {bias/100.0}"
                else:
                    title = rf"Final State of Tree for E2 ($\eta$ = {eta}) with bias probability = {bias/100.0}"
                planning_env.visualize_plan(plan, tree=tree, title=title)

    elif planner_type == 'rrtconnect':
        const_vals = [3, 5, 10, 20, 30]
        for const_val in const_vals:
            cost_dict = dict()
            run_times = [0.1, 0.5, 1.0, 5.0, 7.5, 10.0, 15.0, 20.0]
            sample_times = [0.1, 0.5, 1.0, 5.0, 7.5, 10.0, 15.0, 20.0]
            sample_times.reverse()
            successes = []
            path_lengths = {}
            for run_time in run_times:
                success = 0
                cost_dict[run_time] = []
                for i in range(10):
                    sample_times_copy = deepcopy(sample_times)
                    planner = RRTStarPlanner(planning_env)
                    planner_return = planner.Plan(start, goal, timeout=run_time, sample_times=sample_times_copy, k_type='const', const_val=const_val)
                    if planner_return is not None:
                        success += 1
                        plan, cost, tree, cost_at_time = planner_return
                        cost_dict[run_time].append(cost)
                    else:
                        cost_dict[run_time].append(None)
                        if run_time == run_times[-1]:
                            print('cost at time:', cost_at_time)
                    print(f"k = {const_val}, Run time: {run_time}, iter: {i}")
                successes.append(success)
            print(f"Results for k = {const_val}:")
            print('successes', successes)

        # Shortcut the path.
    # TODO (student): Do not shortcut when comparing the performance of algorithms. 
    # Comment this line out when collecting data over performance metrics.
    # plan_short = planner.ShortenPath(plan)

    # Visualize the final path.
    if planner_type == 'astar':
        planning_env.visualize_plan(plan, visited)
    # elif planner_type == 'rrt':
    #     planning_env.visualize_plan(plan, tree=tree)
    elif planner_type == 'rrtconnect':
        if planner_return is not None:
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
