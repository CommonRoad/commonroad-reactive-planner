import glob
import os
import time
import warnings
from copy import deepcopy

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object

from cr2sumo.interface.sumo_interface import SumoInterface
from cr2sumo.rpc.sumo_client import SumoRPCClient
from cr2sumo.visualization.video import create_video
from sumo_config.default import SumoCommonRoadConfig

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


def plan(scenario, planning_problem_set, plot_dir, scenario_folder, make_video: bool = True):
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    problem_init_state = planning_problem.initial_state
    if not hasattr(problem_init_state, 'acceleration'):
        problem_init_state.acceleration = 0.

    try:
        # road_boundary_sg, road_boundary_obstacle = create_road_boundary(scenario, draw=False)
        route_planner = RoutePlanner(scenario.benchmark_id, scenario.lanelet_network, planning_problem)
        # reference_path = route_planner.generate_ref_path()

        conf = SumoCommonRoadConfig()
        conf.scenario_name = scenario.benchmark_id
        sumo_interface = SumoInterface()
        sumo_client: SumoRPCClient = sumo_interface.start_simulator()

        DT = scenario.dt
        T_H = 2.5
        # TODO: It turns out that setting the desired velocity has a big influence of the planner performance.
        #       A more smart way to choose the desired velocity or even better to adjust the desired velocity within
        #       the planning process should be implemented.
        if hasattr(planning_problem.goal.state_list[0], 'velocity'):
            if planning_problem.goal.state_list[0].velocity.start != 0:
                desired_velocity = planning_problem.goal.state_list[0].velocity.start
            else:
                desired_velocity = (planning_problem.goal.state_list[0].velocity.start
                                    + planning_problem.goal.state_list[0].velocity.end) / 2
        else:
            desired_velocity = problem_init_state.velocity

        planner = ReactivePlanner(scenario, scenario_folder=scenario_folder, planning_problem_set=planning_problem_set,
                                  route_planner=route_planner, sumo_client=sumo_client, conf=conf, dt=DT, t_h=T_H,
                                  N=int(T_H / DT), v_desired=desired_velocity)

        planner.set_desired_velocity(desired_velocity)
        planned_states, ref_path_list = planner.re_plan(deepcopy(problem_init_state))

        sumo_client.stop()

        if planned_states is not None:
            print(f"Plan successfully for {scenario.benchmark_id}.")
            planned_scenario = sumo_client.commonroad_scenarios_all_time_steps()
            plt.figure(figsize=(20, 10))
            draw_object(planned_scenario, draw_params=DRAW_PARAMS)
        #     draw_object(planning_problem)
        #     trajectory = Trajectory(0, planned_states)
        #     ego = planner.convert_cr_trajectory_to_object(trajectory)
        #     planned_x = []
        #     planned_y = []
        #     for state in planned_states:
        #         planned_x.append(state.position[0])
        #         planned_y.append(state.position[1])
        #     plt.plot(planned_x, planned_y, color='k', marker='o', markersize=1, zorder=20, linewidth=0.5,
        #              label='Planned route')
        else:
            print(f"Plan fails for {scenario.benchmark_id}.")

        # for rp in ref_path_list:
        #     plt.plot(rp[:, 0], rp[:, 1], color='g', marker='*', markersize=1, zorder=19, linewidth=0.5,
        #              label='Reference route')
        # commonroad_cc.visualization.draw_dispatch.draw_object(road_boundary_sg,
        #                                                       draw_params={'collision': {'facecolor': 'yellow'}})

        plt.gca().set_aspect('equal')

        # plt.legend(loc=(1.04, 0))
        plt.autoscale()
        plt.savefig(os.path.join(plot_dir, scenario.benchmark_id + '.png'), format='png', dpi=300,
                    bbox_inches='tight')
        plt.close()

        if make_video:
            output_folder = "./videos"
            os.makedirs(output_folder, exist_ok=True)
            print("Creating video")
            create_video(sumo_client, conf.video_start, conf.video_end, output_folder)
            print("Video created")

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
        scenario, planning_problem_set = crfr.open()
        scenario_folder = os.path.join('./scenarios', scenario.benchmark_id)

        plan(scenario, planning_problem_set, plot_dir, scenario_folder)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument("-p", "--base_dir", type=str,
                        default='./scenarios')

    args = parser.parse_args()
    main(args)
