__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.5"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Beta"

# standard imports
import os
import glob
import time
from copy import deepcopy

# third party
import numpy as np

# commonroad-io
from commonroad.common.file_reader import CommonRoadFileReader

# commonroad_dc
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker

# commonroad-route-planner
from commonroad_route_planner.route_planner import RoutePlanner

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.visualization import visualize_planning_result, visualize_collision_checker


# *************************************
# Open CommonRoad scenario
# *************************************
base_dir = "./example_scenarios"
filename = "ZAM_Over-1_1.xml"

scenario_path = os.path.join(base_dir, filename)
files = sorted(glob.glob(scenario_path))

# open scenario and panning problem
crfr = CommonRoadFileReader(files[0])
scenario, problem_set = crfr.open()
planning_problem = list(problem_set.planning_problem_dict.values())[0]


# *************************************
# Set Global Configurations
# *************************************
DT = scenario.dt            # planning time step
T_H = 2                     # planning horizon
replanning_frequency = 3    # re-plan every i-th time step
plot = False                # plot results


# *************************************
# Initializations
# *************************************
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
_, road_boundary_sg_obb = create_road_boundary_obstacle(scenario)
collision_checker_scenario = create_collision_checker(scenario)
collision_checker_scenario.add_collision_object(road_boundary_sg_obb)


# initialize reactive planner
planner = ReactivePlanner(dt=DT, t_h=T_H, N=int(T_H / DT), v_desired=desired_velocity)
planner.set_d_sampling_parameters(-3, 3)
planner.set_t_sampling_parameters(0.4, planner.dT, planner.horizon)


# initialize route planner get reference path
route_planner = RoutePlanner(scenario, planning_problem)
ref_path = route_planner.plan_routes().retrieve_first_route().reference_path


# set reference path for planner
planner.set_reference_path(ref_path)


# **************************
# Run Planning
# **************************
# initialize some variables
record_state_list = list()
record_state_list.append(x_0)
x_cl = None
current_count = 0
planning_times = list()
positions = None
ego_vehicle = None


# Run the planner
# while not goal.is_reached(x_0):
while current_count <=1:
    current_count = len(record_state_list) - 1
    if current_count % replanning_frequency == 0:
        # new planning cycle -> plan a new optimal trajectory

        # START TIMER
        comp_time_start = time.time()
        # set desired velocity
        current_velocity = x_0.velocity
        planner.set_desired_velocity(desired_velocity)

        # plan trajectory
        optimal = planner.plan(x_0, collision_checker_scenario, cl_states=x_cl, draw_traj_set=plot)     # returns the planned (i.e., optimal) trajectory
        comp_time_end = time.time()
        # END TIMER

        # if the planner fails to find an optimal trajectory -> terminate
        if not optimal:
            print("not optimal")
            break

        # store sampled trajectory bundle for visualization
        if plot:
            sampled_trajectory_bundle = deepcopy(planner.bundle.trajectories)

        # store planning times
        planning_times.append(comp_time_end - comp_time_start)

        # correct orientation angle
        new_state_list = planner.shift_orientation(optimal[0])

        # get positions of optimal trajectory
        positions = np.asarray([state.position for state in new_state_list.state_list])

        new_state = new_state_list.state_list[1]
        new_state.time_step = current_count + 1

        record_state_list.append(new_state)

        # update init state and curvilinear state
        x_0 = deepcopy(record_state_list[-1])
        x_cl = (optimal[2][1], optimal[3][1])

        # create CommonRoad Obstacle for the ego Vehicle
        ego_vehicle = planner.convert_cr_trajectory_to_object(optimal[0])
    else:
        # not a planning cycle -> no trajectories sampled -> set sampled_trajectory_bundle to None
        sampled_trajectory_bundle = None

        # continue on optimal trajectory
        temp = current_count % replanning_frequency
        new_state = new_state_list.state_list[1 + temp]
        new_state.time_step = current_count + 1
        record_state_list.append(new_state)

        # update init state and curvilinear state
        x_0 = deepcopy(record_state_list[-1])
        x_cl = (optimal[2][1 + temp], optimal[3][1 + temp])

    print(f"current time step: {current_count}")
    # draw scenario + planning solution
    if plot:
        visualize_planning_result(scenario=scenario, planning_problem=planning_problem, ego=ego_vehicle,
                                  pos=positions, traj_set=sampled_trajectory_bundle, ref_path=ref_path,
                                  timestep=current_count)
