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
from commonroad.scenario.state import InputState

# commonroad-route-planner
from commonroad_route_planner.route_planner import RoutePlanner

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.visualization import visualize_planner_at_timestep, plot_final_trajectory, make_gif, \
    visualize_scenario_and_pp
from commonroad_rp.utility.evaluation import create_planning_problem_solution, reconstruct_inputs, plot_states, \
    plot_inputs, reconstruct_states, create_full_solution_trajectory, check_acceleration
from commonroad_rp.configuration_builder import ConfigurationBuilder

from commonroad_rp.utility.general import load_scenario_and_planning_problem
from commonroad_rp.utility.spot_helpers import spot_setup_scenario, spot_compute_occupancy_prediction, \
    spot_remove_scenario, spot_update_init_state_obstacles

# *************************************
# Set Configurations
# *************************************
filename = "DEU_Test-1_1_T-1.xml"

config = ConfigurationBuilder.build_configuration(filename[:-4])

scenario, planning_problem, planning_problem_set = load_scenario_and_planning_problem(config)

DT = config.planning.dt            # planning time step

# SPOT configurations
spot_update_dict = {'Vehicle': {
    0: {
        'a_max': 4.0,
        'compute_occ_m1': True,
        'compute_occ_m2': True,
        'compute_occ_m3': False,
        'onlyInLane': True
    }}}
spot_scenario_id = 1


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
        desired_velocity = (planning_problem.goal.state_list[0].velocity.end) / 2
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
x_0 = planner.process_initial_state_from_pp(x0_pp=x_0)
record_state_list.append(x_0)

# add initial inputs to recorded input list
record_input_list.append(InputState(
        acceleration=x_0.acceleration,
        time_step=x_0.time_step,
        steering_angle_speed=0.))

# Run planner
while not goal.is_reached(x_0):
    current_count = len(record_state_list) - 1
    if current_count % config.planning.replanning_frequency == 0:
        # new planning cycle -> plan a new optimal trajectory

        # do SPOT prediction for new planning cycle
        scenario_spot = deepcopy(scenario)
        scenario_spot = spot_update_init_state_obstacles(scenario_spot, current_count)
        # Register Spot Scenario
        spot_setup_scenario(scenario_spot, planning_problem, spot_scenario_id, spot_update_dict)
        spot_compute_occupancy_prediction(scenario_spot, spot_scenario_id,
                                          start_time_step=current_count,
                                          end_time_step=current_count + config.planning.time_steps_computation,
                                          dt=config.planning.dt,
                                          num_threads=4)
        # Deregister Spot Scenario
        spot_remove_scenario(spot_scenario_id)

        # update collision checker of planner
        planner.set_collision_checker(scenario_spot)

        # START TIMER
        comp_time_start = time.time()
        # set desired velocity
        current_velocity = x_0.velocity
        planner.set_desired_velocity(desired_velocity, current_velocity)

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
        new_state_list = planner.shift_orientation(optimal[0], interval_start=x_0.orientation-np.pi,
                                                   interval_end=x_0.orientation+np.pi)

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
            ego_vehicle = planner.shift_and_convert_trajectory_to_object(optimal[0])
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
        visualize_planner_at_timestep(scenario=scenario_spot, planning_problem=planning_problem, ego=ego_vehicle,
                                      traj_set=sampled_trajectory_bundle, ref_path=ref_path,
                                      timestep=current_count, config=config)

# make gif
# if goal.is_reached(x_0):
#     make_gif(config, scenario, range(0, current_count+1))

# **************************
# Evaluate results
# **************************
# Optional: solutions can be evaluated via input reconstruction from commonroad_dc.feasibility.feasibility_checker
# For each state transition the inputs are reconstructed. To check feasibility, the reconstructed input is used for
# forward simulation and the error between the forward simulated (reconstructed) state and the planned state is
# evaluated
evaluate = True
if evaluate:
    from commonroad_dc.feasibility.solution_checker import valid_solution
    # create full solution trajectory
    ego_solution_trajectory = create_full_solution_trajectory(config, record_state_list)

    # plot full ego vehicle trajectory
    plot_final_trajectory(scenario, planning_problem, ego_solution_trajectory.state_list, config)

    # create CR solution
    solution = create_planning_problem_solution(config, ego_solution_trajectory, scenario, planning_problem)

    # check feasibility
    # reconstruct inputs (state transition optimizations)
    feasible, reconstructed_inputs = reconstruct_inputs(config, solution.planning_problem_solutions[0])

    # reconstruct states from inputs
    reconstructed_states = reconstruct_states(config, ego_solution_trajectory.state_list, reconstructed_inputs)

    # check acceleration correctness
    check_acceleration(config, ego_solution_trajectory.state_list, plot=True)

    # remove first element from input list
    record_input_list.pop(0)

    # plot
    plot_states(config, ego_solution_trajectory.state_list, reconstructed_states, plot_bounds=False)
    plot_inputs(config, record_input_list, reconstructed_inputs, plot_bounds=False)

    # CR validity check
    print("Feasibility Check Result: ")
    print(valid_solution(scenario, planning_problem_set, solution))
