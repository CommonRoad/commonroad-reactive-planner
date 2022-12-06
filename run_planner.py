__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "0.5"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Beta"


# standard imports
import time
from copy import deepcopy

# third party
import numpy as np

# commonroad-io
from commonroad.scenario.trajectory import State
from commonroad.scenario.state import CustomState, InputState

# commonroad-route-planner
from commonroad_route_planner.route_planner import RoutePlanner

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner, CartesianState
from commonroad_rp.utility.visualization import visualize_planner_at_timestep, plot_final_trajectory, make_gif
from commonroad_rp.utility.evaluation import create_planning_problem_solution, reconstruct_inputs, plot_states, \
    plot_inputs, reconstruct_states
from commonroad_rp.configuration_builder import ConfigurationBuilder

from commonroad_rp.utility.general import load_scenario_and_planning_problem

# *************************************
# Set Configurations
# *************************************
filename = "ZAM_Over-1_1.xml"
# filename = "ZAM_105222-1_1_T-1.xml"
# filename = "ZAM_OpenDrive-123.xml"
# filename = "ZAM_Tjunction-1_42_T-1.xml"
# filename = "C-DEU_B471-2_1.xml"

config = ConfigurationBuilder.build_configuration(filename[:-4])

scenario, planning_problem, planning_problem_set = load_scenario_and_planning_problem(config)

DT = config.planning.dt            # planning time step


# *************************************
# Init and Goal State
# *************************************
# initial state configuration
problem_init_state = planning_problem.initial_state
current_velocity = problem_init_state.velocity
if not hasattr(problem_init_state, 'acceleration'):
    problem_init_state.acceleration = 0.
x_0 = deepcopy(problem_init_state)
delattr(x_0, "slip_angle")

# goal state configuration
goal = planning_problem.goal
if hasattr(planning_problem.goal.state_list[0], 'velocity'):
    if planning_problem.goal.state_list[0].velocity.start > 0:
        desired_velocity = (planning_problem.goal.state_list[0].velocity.start + planning_problem.goal.state_list[0].velocity.end) / 2
    else:
        desired_velocity = (planning_problem.goal.state_list[0].velocity.end)
else:
    desired_velocity = x_0.velocity


# *************************************
# Initialize Planner
# *************************************
# initialize reactive planner
planner = ReactivePlanner(config)
# set sampling parameters
planner.set_d_sampling_parameters(config.sampling.d_min, config.sampling.d_max)
planner.set_t_sampling_parameters(config.sampling.t_min, config.planning.dt, config.planning.planning_horizon)
# set collision checker
planner.set_collision_checker(scenario)
# initialize route planner and set reference path
route_planner = RoutePlanner(scenario, planning_problem)
route = route_planner.plan_routes().retrieve_first_route()
ref_path = route.reference_path

planner.set_reference_path(ref_path)


# **************************
# Run Planning
# **************************
# initialize lists to store states and inputs
record_state_list = list()
record_input_list = list()
x_cl = None
current_count = 0
planning_times = list()
ego_vehicle = None

# convert initial state from planning problem to reactive planner (Cartesian) state type
x_0_cartesian = CartesianState()
x_0 = x_0.convert_state_to_state(x_0_cartesian)
x_0.steering_angle = np.arctan2(config.vehicle.wheelbase * x_0_cartesian.yaw_rate,
                                x_0_cartesian.velocity)
record_state_list.append(x_0)

record_input_state = InputState(
        acceleration=x_0.acceleration,
        time_step=x_0.time_step,
        steering_angle_speed=0.)
record_input_list.append(record_input_state)

# Run planner
while not goal.is_reached(x_0):
    current_count = len(record_state_list) - 1
    if current_count % config.planning.replanning_frequency == 0:
        # new planning cycle -> plan a new optimal trajectory

        # START TIMER
        comp_time_start = time.time()
        # set desired velocity
        current_velocity = x_0.velocity
        planner.set_desired_velocity(desired_velocity)

        # plan trajectory
        optimal = planner.plan(x_0, x_cl)     # returns the planned (i.e., optimal) trajectory
        comp_time_end = time.time()
        # END TIMER

        # store planning times
        planning_times.append(comp_time_end - comp_time_start)
        print(f"***Total Planning Time: {planning_times[-1]}")

        # if the planner fails to find an optimal trajectory -> terminate
        if not optimal:
            print("not optimal")
            break

        # if desired, store sampled trajectory bundle for visualization
        if config.debug.show_plots or config.debug.save_plots:
            sampled_trajectory_bundle = deepcopy(planner.stored_trajectories)

        # correct orientation angle
        new_state_list = planner.shift_orientation(optimal[0])

        # get next state from state list of planned trajectory
        new_state = new_state_list.state_list[1]
        new_state.time_step = current_count + 1

        # add input to recorded input list
        record_input_list.append(InputState(
            acceleration=new_state.acceleration,
            steering_angle_speed=(new_state.steering_angle - record_state_list[-1].steering_angle) / DT,
            time_step=new_state.time_step
        ))

        # add new state to recorded state list
        record_state_list.append(new_state)

        # update init state and curvilinear state
        x_0 = deepcopy(record_state_list[-1])
        x_cl = (optimal[2][1], optimal[3][1])

        # create CommonRoad Obstacle for the ego Vehicle
        if config.debug.show_plots or config.debug.save_plots:
            ego_vehicle = planner.convert_cr_trajectory_to_object(optimal[0])
    else:
        # not a planning cycle -> no trajectories sampled -> set sampled_trajectory_bundle to None
        sampled_trajectory_bundle = None

        # continue on optimal trajectory
        temp = current_count % config.planning.replanning_frequency

        # get next state from state list of planned trajectory
        new_state = new_state_list.state_list[1 + temp]
        new_state.time_step = current_count + 1

        # add input to recorded input list
        record_input_list.append(InputState(
            acceleration=new_state.acceleration,
            steering_angle_speed=(new_state.steering_angle - record_state_list[-1].steering_angle) / DT,
            time_step=new_state.time_step
        ))

        # add new state to recorded state list
        record_state_list.append(new_state)

        # update init state and curvilinear state
        x_0 = deepcopy(record_state_list[-1])
        x_cl = (optimal[2][1 + temp], optimal[3][1 + temp])

    print(f"current time step: {current_count}")
    # draw scenario + planning solution
    if config.debug.show_plots or config.debug.save_plots:
        visualize_planner_at_timestep(scenario=scenario, planning_problem=planning_problem, ego=ego_vehicle,
                                      traj_set=sampled_trajectory_bundle, ref_path=ref_path,
                                      timestep=current_count, config=config)

# plot  final ego vehicle trajectory
plot_final_trajectory(scenario, planning_problem, record_state_list, config)

# make gif
# if goal.is_reached(x_0):
#     make_gif(config, scenario, range(0, current_count+1))

# remove first element
record_input_list.pop(0)

# **************************
# Evaluate results
# **************************
evaluate = True
if evaluate:
    # create CR solution
    solution = create_planning_problem_solution(config, record_state_list, scenario, planning_problem)

    # check feasibility
    # reconstruct inputs (state transition optimizations)
    feasible, reconstructed_inputs = reconstruct_inputs(config, solution.planning_problem_solutions[0])
    # reconstruct states from inputs
    reconstructed_states = reconstruct_states(config, record_state_list, reconstructed_inputs)
    # evaluate
    plot_states(config, record_state_list, reconstructed_states, plot_bounds=False)
    plot_inputs(config, record_input_list, reconstructed_inputs, plot_bounds=True)
