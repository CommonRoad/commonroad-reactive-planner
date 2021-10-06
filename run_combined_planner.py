# standard imports
import os
import glob
import time
from copy import deepcopy

# third party
import numpy as np
import matplotlib.pyplot as plt
import pickle

# commonroad-io
from commonroad.common.file_reader import CommonRoadFileReader
# from commonroad.visualization.draw_dispatch_cr import draw_object

# commonroad_dc
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from route_planner import RoutePlanner, ReferenceRouteManager
from scenario_helpers import extrapolate_ref_path, shift_ref_path


# *************************
# Open CommonRoad scenario
# *************************
base_dir = "./example_scenarios"
filename = "ZAM_Over-1_1.xml"

scenario_path = os.path.join(base_dir, filename)
files = sorted(glob.glob(scenario_path))

crfr = CommonRoadFileReader(files[0])
scenario, problem_set = crfr.open()
planning_problem = list(problem_set.planning_problem_dict.values())[0]


# *************************
# Configurations
# *************************
DT = scenario.dt            # planning time step
T_H = 2                     # planning horizon
replanning_frequency = 3    # re-plan every i-th time step
plot = False                # plot results


# *************************
# Initializations
# *************************
# initial state configuration
problem_init_state = planning_problem.initial_state
current_velocity = problem_init_state.velocity
if not hasattr(problem_init_state, 'acceleration'):
    problem_init_state.acceleration = 0.
x_0 = deepcopy(problem_init_state)

# goal state configuration
goal = planning_problem.goal
if hasattr(planning_problem.goal.state_list[0], 'velocity'):
    if planning_problem.goal.state_list[0].velocity.start != 0:
        desired_velocity = (planning_problem.goal.state_list[0].velocity.start + planning_problem.goal.state_list[0].velocity.end) / 2
    else:
        desired_velocity = (planning_problem.goal.state_list[0].velocity.start
                            + planning_problem.goal.state_list[0].velocity.end) / 2
else:
    desired_velocity = problem_init_state.velocity


# initialize collision checker and road boundary
road_boundary_obstacle, road_boundary_sg_triangles = create_road_boundary_obstacle(scenario, method='triangulation')
collision_checker_scenario = create_collision_checker(scenario)
collision_checker_scenario.add_collision_object(road_boundary_sg_triangles)


# initialize reactive planner
planner = ReactivePlanner(dt=DT, t_h=T_H, N=int(T_H / DT), v_desired=desired_velocity)
planner.set_d_sampling_parameters(-3, 3)
planner.set_t_sampling_parameters(0.4, planner.dT, planner.horizon)


# initialize route planner get reference path
route_planner = RoutePlanner(scenario.scenario_id, scenario.lanelet_network, planning_problem)
ref_path = route_planner.generate_ref_path()[0]

# WORKAROUND: extrapolate reference path to avoid CCosy errors
ref_path_mod = extrapolate_ref_path(ref_path, resample_step=0.2)
planner.set_reference_path(ref_path_mod)


# **************************
# Run Planning
# **************************
# initialize state list
record_state_list = list()
record_state_list.append(x_0)
x_cl = None
current_count = 0
planning_times = list()

# create plot directory
if plot:
    plot_path = "./plots/" + filename[:-4]
    os.makedirs(plot_path, exist_ok=True)

# planning
while not goal.is_reached(x_0):
    if plot:
        rnd = MPRenderer()
    current_count = len(record_state_list) - 1
    if current_count % replanning_frequency == 0:
        # new planning cycle -> new optimal trajectory
        # START TIMER
        comp_time_start = time.time()
        # set desired velocity
        current_velocity = x_0.velocity
        planner.set_desired_velocity(desired_velocity)
        optimal = planner.plan(x_0, collision_checker_scenario, cl_states=x_cl, draw_traj_set=plot)
        comp_time_end = time.time()
        # END TIMER

        # store planning times
        planning_times.append(comp_time_end - comp_time_start)

        if not optimal:
            print("not optimal")
            break

        new_state_list = planner.shift_orientation(optimal[0])
        planned_x = []
        planned_y = []
        for state in new_state_list.state_list:
            planned_x.append(state.position[0])
            planned_y.append(state.position[1])
        new_state = new_state_list.state_list[1]
        new_state.time_step = current_count + 1
        record_state_list.append(new_state)

        # update init state and curvilinear state
        x_0 = deepcopy(record_state_list[-1])
        x_cl = (optimal[2][1], optimal[3][1])

        if plot:
            ego_vehicle = planner.convert_cr_trajectory_to_object(optimal[0])
            ego_vehicle.draw(rnd, draw_params={'time_begin': current_count,
                                                  "dynamic_obstacle": {"facecolor": "#E37222", 'edgecolor': '#E37222',
                                                                       "zorder": 30, 'opacity': 0.7}})
    else:
        # continue on optimal trajectory
        temp = current_count % replanning_frequency
        new_state = new_state_list.state_list[1 + temp]
        new_state.time_step = current_count + 1
        record_state_list.append(new_state)

        # update init state and curvilinear state
        x_0 = deepcopy(record_state_list[-1])
        x_cl = (optimal[2][1 + temp], optimal[3][1 + temp])

        if plot:
            ego_vehicle.draw(rnd, draw_params={'time_begin': current_count,
                                                  "dynamic_obstacle": {"facecolor": "#E37222", 'edgecolor': '#E37222',
                                                                       "zorder": 30, 'opacity': 0.7}})

    print(f"current time step: {current_count}")
    # draw scenario + planning solution
    if plot:
        scenario.draw(rnd, draw_params={'time_begin': current_count})
        rnd.render(show=True)
        # draw_object(planning_problem)
        plt.plot(np.array(planned_x), np.array(planned_y), color='k', marker='o', markersize=1, zorder=30,
                 linewidth=1.0, label='planned trajectories')
        plt.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=20, linewidth=0.8,
                 label='reference path')
        plt.gca().set_aspect('equal')
        plt.autoscale()
        plt.savefig(f"{plot_path}/{scenario.scenario_id}_{current_count}.png", format='png', dpi=300,
                    bbox_inches='tight')
        plt.close()
