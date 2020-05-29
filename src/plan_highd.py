import glob
import os
import numpy as np

import commonroad_cc
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker

from route_planner import generate_ref_path
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
    scenario.translate_rotate(np.array([0., 0.]), 45 / 180 * np.pi)
    planning_problem.translate_rotate(np.array([0., 0.]), 45 / 180 * np.pi)

    problem_init_state = planning_problem.initial_state
    if not hasattr(problem_init_state, 'acceleration'):
        problem_init_state.acceleration = 0.
    
    # road_boundary_sg, road_boundary_obstacle = create_road_boundary(scenario, draw=False)

    collision_checker_scenario = create_collision_checker(scenario)
    # collision_checker_scenario.add_collision_object(road_boundary_sg)


    reference_path = generate_ref_path(scenario, planning_problem)
    
    for num_ref, rp in enumerate(reference_path):
        
        plt.figure(figsize=(20, 10))
        draw_object(scenario, draw_params=DRAW_PARAMS)
        draw_object(planning_problem)

        DT = scenario.dt
        T_H = 3.6
        rp = smoothing_reference_path(rp)

        planner = ReactivePlanner(dt=DT, t_h=T_H, N=int(T_H / DT), v_desired=problem_init_state.velocity)
        planner.set_reference_path(rp)
        planner.set_desired_velocity(problem_init_state.velocity)
        
        x_0 = problem_init_state
        optimal = planner.plan(x_0, collision_checker_scenario)
        if optimal:
            print(f"Plan successfully for {scenario.benchmark_id}.")
            planned_x = []
            planned_y = []
            ego = planner.convert_cr_trajectory_to_object(optimal[0])
            for state in ego.prediction.trajectory.state_list:
                planned_x.append(state.position[0])
                planned_y.append(state.position[1])
            plt.plot(planned_x, planned_y, color='k', marker='o', markersize=1, zorder=20, linewidth=0.5,
                     label='Planned route')
        else:
            print(f"Plan fails for {scenario.benchmark_id}.")

        plt.plot(rp[:, 0], rp[:, 1], color='g', marker='*', markersize=1, zorder=19, linewidth=0.5,
                 label='Reference route')
        # commonroad_cc.visualization.draw_dispatch.draw_object(road_boundary_sg,
        #                                                       draw_params={'collision': {'facecolor': 'yellow'}})

        plt.gca().set_aspect('equal')

        plt.legend(loc=(1.04, 0))
        plt.autoscale()
        plt.savefig(os.path.join(plot_dir, scenario.benchmark_id + '_' + str(num_ref) + '.png'), format='png', dpi=300,
                    bbox_inches='tight')
        plt.close()

def main(args):
    plot_dir = os.path.join(args.base_dir, "plot")

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
