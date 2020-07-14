import glob
import os
import time
import warnings
from copy import deepcopy

import commonroad_cc
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker

from route_planner.route_planner import RoutePlanner
from commonroad_rp.reactive_planner import ReactivePlanner
from scenario_helpers import *

# Visualization parameters
DRAW_PARAMS = {
    'draw_shape': True,
    'draw_icon': False,
    'draw_bounding_box': True,
    'trajectory_steps': 2,
    'show_label': False,
    'occupancy': {
        'draw_occupancies': 0, 'shape': {
            'rectangle': {
                'opacity': 0.2,
                'facecolor': '#fa0200',
                'edgecolor': '#0066cc',
                'linewidth': 0.5,
                'zorder': 18}}},
    'shape': {'rectangle': {'opacity': 1.0,
                            'facecolor': '#fa0200',
                            'edgecolor': '#831d20',
                            'linewidth': 0.5,
                            'zorder': 20}}}


def plan(scenario, planning_problem, plot_dir):

    problem_init_state = planning_problem.initial_state
    if not hasattr(problem_init_state, 'acceleration'):
        problem_init_state.acceleration = 0.

    try:
        road_boundary_sg, road_boundary_obstacle = create_road_boundary(scenario, draw=False)

        collision_checker_scenario = create_collision_checker(scenario)
        collision_checker_scenario.add_collision_object(road_boundary_sg)

        route_planner = RoutePlanner(scenario.benchmark_id, scenario.lanelet_network, planning_problem)
        # reference_path = route_planner.generate_ref_path()
        DT = scenario.dt
        T_H = 2.5
        if hasattr(planning_problem.goal.state_list[0], 'velocity'):
            if planning_problem.goal.state_list[0].velocity.start != 0:
                desired_velocity = planning_problem.goal.state_list[0].velocity.start
            else:
                desired_velocity = (planning_problem.goal.state_list[0].velocity.start
                                    + planning_problem.goal.state_list[0].velocity.end) / 2
        else:
            desired_velocity = problem_init_state.velocity

        planner = ReactivePlanner(scenario, planning_problem=planning_problem, route_planner=route_planner, dt=DT,
                                  t_h=T_H, N=int(T_H / DT), v_desired=desired_velocity)

        planner.set_desired_velocity(desired_velocity)
        x_0 = deepcopy(problem_init_state)
        planned_states, ref_path_list = planner.re_plan(x_0, collision_checker_scenario)
        plt.figure(figsize=(20, 10))
        draw_object(scenario, draw_params=DRAW_PARAMS)
        draw_object(planning_problem)

        if planned_states is not None:
            print(f"Plan successfully for {scenario.benchmark_id}.")
            trajectory = Trajectory(0, planned_states)
            ego = planner.convert_cr_trajectory_to_object(trajectory)
            planned_x = []
            planned_y = []
            for state in planned_states:
                planned_x.append(state.position[0])
                planned_y.append(state.position[1])
            plt.plot(planned_x, planned_y, color='k', marker='o', markersize=1, zorder=20, linewidth=0.5,
                     label='Planned route')
        else:
            print(f"Plan fails for {scenario.benchmark_id}.")

        for rp in ref_path_list:
            plt.plot(rp[:, 0], rp[:, 1], color='g', marker='*', markersize=1, zorder=19, linewidth=0.5,
                     label='Reference route')
        commonroad_cc.visualization.draw_dispatch.draw_object(road_boundary_sg,
                                                              draw_params={'collision': {'facecolor': 'yellow'}})

        plt.gca().set_aspect('equal')

        # plt.legend(loc=(1.04, 0))
        plt.autoscale()
        plt.savefig(os.path.join(plot_dir, scenario.benchmark_id + '.png'), format='png', dpi=300,
                    bbox_inches='tight')
        plt.close()
    except BaseException as e:
        warnings.warn('Unexpected error during planning:' + str(e), stacklevel=1)


def main(args):
    # plot_dir = os.path.join(args.base_dir, "plot")
    plot_dir = os.path.join('./plots', time.strftime("%Y-%m-%d-%H%M%S"))

    os.makedirs(plot_dir, exist_ok=True)

    scenario_path = os.path.join(args.base_dir, "*.xml")
    files = sorted(glob.glob(scenario_path))

    for f in files:

        crfr = CommonRoadFileReader(f)
        scenario, problem_set = crfr.open()
        planning_problem = list(problem_set.planning_problem_dict.values())[0]

        plan(scenario, planning_problem, plot_dir)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument("-p", "--base_dir", type=str,
                        default='./scenarios')

    args = parser.parse_args()
    main(args)
