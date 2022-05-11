__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "0.5"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Beta"


# standard imports
import os
import glob
import time
from copy import deepcopy
from matplotlib import pyplot as plt

# third party
import numpy as np

# commonroad-io
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.trajectory import State

# commonroad-route-planner
from commonroad_route_planner.route_planner import RoutePlanner

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.visualization import visualize_planner_at_timestep, plot_final_trajectory
from commonroad_rp.configuration import build_configuration


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
config = build_configuration(filename[:-4])

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


# initialize reactive planner
planner = ReactivePlanner(config)
planner.set_d_sampling_parameters(-3, 3)
planner.set_t_sampling_parameters(0.4, planner.dT, planner.horizon)

# set collision checker
planner.set_collision_checker(scenario)

# initialize route planner and set reference path
route_planner = RoutePlanner(scenario, planning_problem)
ref_path = route_planner.plan_routes().retrieve_first_route().reference_path
planner.set_reference_path(ref_path)


# **************************
# Run Planning
# **************************
# initialize some variables
record_state_list = list()
x_cl = None
current_count = 0
planning_times = list()
positions = None
ego_vehicle = None

record_state_list.append(x_0)
delattr(record_state_list[0], "slip_angle")
record_state_list[0].steering_angle = np.arctan2(config.vehicle.wheelbase * record_state_list[0].yaw_rate,
                                                 record_state_list[0].velocity)

record_input_list = list()
record_input_state = State(
        steering_angle=np.arctan2(config.vehicle.wheelbase * x_0.yaw_rate, x_0.velocity),
        acceleration=x_0.acceleration,
        time_step=x_0.time_step,
        steering_angle_speed=0.
)
record_input_list.append(record_input_state)



# Run the planner
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
        if plot:
            sampled_trajectory_bundle = deepcopy(planner.stored_trajectories)

        # correct orientation angle
        new_state_list = planner.shift_orientation(optimal[0])

        # get positions of optimal trajectory
        positions = np.asarray([state.position for state in new_state_list.state_list])

        new_state = new_state_list.state_list[1]
        new_state.time_step = current_count + 1

        # TODO recompute acceleration
        new_state.acceleration = (new_state.velocity - record_state_list[-1].velocity) / DT

        record_state_list.append(new_state)

        # TODO: add input to list
        record_input_list.append(State(
            steering_angle=new_state.steering_angle,
            acceleration=new_state.acceleration,
            steering_angle_speed=(new_state.steering_angle - record_input_list[-1].steering_angle) / DT,
            time_step=new_state.time_step
        ))

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

        # TODO recompute acceleration
        new_state.acceleration = (new_state.velocity - record_state_list[-1].velocity) / DT

        record_state_list.append(new_state)

        # TODO: add input to list
        record_input_list.append(State(
            steering_angle=new_state.steering_angle,
            acceleration=new_state.acceleration,
            steering_angle_speed=(new_state.steering_angle - record_input_list[-1].steering_angle) / DT,
            time_step=new_state.time_step
        ))

        # update init state and curvilinear state
        x_0 = deepcopy(record_state_list[-1])
        x_cl = (optimal[2][1 + temp], optimal[3][1 + temp])

    print(f"current time step: {current_count}")
    # draw scenario + planning solution
    if plot:
        visualize_planner_at_timestep(scenario=scenario, planning_problem=planning_problem, ego=ego_vehicle,
                                      pos=positions, traj_set=sampled_trajectory_bundle, ref_path=ref_path,
                                      timestep=current_count)


# ************************************
# Plot and Check CommonRoad Solution
# ************************************
from commonroad.scenario.trajectory import Trajectory
from commonroad.common.solution import Solution, PlanningProblemSolution, VehicleModel, \
    VehicleType, CostFunction
from commonroad_dc.feasibility.solution_checker import valid_solution
from commonroad_dc.feasibility.feasibility_checker import trajectory_feasibility, VehicleDynamics, \
    state_transition_feasibility, position_orientation_objective, position_orientation_feasibility_criteria
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2

veh_params = parameters_vehicle2()

# plot occupancies of final ego vehicle trajectory
plot_final_trajectory(scenario, planning_problem, record_state_list, (config.vehicle.length, config.vehicle.width))

# create solution object
input_solution = False
if input_solution:
    ego_vehicle_trajectory = Trajectory(initial_time_step=record_input_list[0].time_step, state_list=record_input_list)
    pps = PlanningProblemSolution(planning_problem_id=planning_problem.planning_problem_id,
                                  vehicle_type=VehicleType.BMW_320i,
                                  vehicle_model=VehicleModel.KS,
                                  cost_function=CostFunction.JB1,
                                  trajectory=ego_vehicle_trajectory)
    from commonroad_dc.feasibility.solution_checker import _simulate_trajectory_if_input_vector
    _, trajectory = _simulate_trajectory_if_input_vector(problem_set, pps, DT)

    plot_final_trajectory(scenario, planning_problem, trajectory.state_list, (config.vehicle.length, config.vehicle.width))
else:
    ego_vehicle_trajectory = Trajectory(initial_time_step=record_state_list[0].time_step, state_list=record_state_list)
    pps = PlanningProblemSolution(planning_problem_id=planning_problem.planning_problem_id,
                                  vehicle_type=VehicleType.BMW_320i,
                                  vehicle_model=VehicleModel.KS,
                                  cost_function=CostFunction.JB1,
                                  trajectory=ego_vehicle_trajectory)

# create solution object
solution = Solution(scenario.scenario_id, [pps])

# check validity of solution
is_valid, res = valid_solution(scenario, problem_set, solution)

if not input_solution:
    # check kinematic feasibility
    vehicle_dynamics = VehicleDynamics.from_model(pps.vehicle_model, pps.vehicle_type)
    feasible, reconstructed_inputs = trajectory_feasibility(pps.trajectory, vehicle_dynamics, DT)

    feasible_state_list = []
    reconstructed_inputs = []
    for x0, x1 in zip(pps.trajectory.state_list[:-1], pps.trajectory.state_list[1:]):
        feasible_state, reconstructed_input_state = state_transition_feasibility(x0, x1, vehicle_dynamics, DT,
                                                                            position_orientation_objective,
                                                                            position_orientation_feasibility_criteria,
                                                                            1e-8, np.array([2e-2, 2e-2, 3e-2]),
                                                                            4, 100, False)
        feasible_state_list.append(feasible_state)
        reconstructed_inputs.append(reconstructed_input_state)


# ************************
# Evaluation
# ************************
# plot inputs
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(list(range(len(record_input_list))),
         [state.steering_angle_speed for state in record_input_list], color="black", label="planned")
plt.plot([0, len(record_input_list)], [veh_params.steering.v_min, veh_params.steering.v_min],
         color="red", label="bounds")
plt.plot([0, len(record_input_list)], [veh_params.steering.v_max, veh_params.steering.v_max],
         color="red")
plt.legend()
plt.ylabel("v_delta")
plt.subplot(2, 1, 2)
plt.plot(list(range(len(record_input_list))),
         [state.acceleration for state in record_input_list], color="black", label="planned")
plt.plot([0, len(record_input_list)], [-veh_params.longitudinal.a_max, -veh_params.longitudinal.a_max],
         color="red", label="bounds")
plt.plot([0, len(record_input_list)], [veh_params.longitudinal.a_max, veh_params.longitudinal.a_max],
         color="red")
plt.ylabel("a_long")
plt.tight_layout()
plt.show()

# plot states
plt.figure()
plt.subplot(4, 1, 1)
plt.plot(list(range(len(record_state_list))),
         [state.steering_angle for state in record_state_list], color="black", label="planned")
# plt.plot([0, len(record_state_list)], [veh_params.steering.min, veh_params.steering.min],
#          color="red", label="bounds")
# plt.plot([0, len(record_state_list)], [veh_params.steering.max, veh_params.steering.max],
#          color="red")
plt.legend()
plt.ylabel("delta")
plt.subplot(4, 1, 2)
plt.plot(list(range(len(record_state_list))),
         [state.velocity for state in record_state_list], color="black", label="planned")
plt.legend()
plt.ylabel("v")
plt.subplot(4, 1, 3)
plt.plot(list(range(len(record_state_list))),
         [state.orientation for state in record_state_list], color="black", label="planned")
plt.ylabel("theta")
plt.tight_layout()
plt.subplot(4, 1, 4)
plt.plot(list(range(len(record_state_list))),
         [state.yaw_rate for state in record_state_list], color="black", label="planned")
plt.ylabel("theta_dot")
plt.tight_layout()
plt.show()

# check reconstructed inputs
if not input_solution:
    # reconstructed_input_list = reconstructed_inputs.state_list
    reconstructed_input_list = reconstructed_inputs
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(list(range(len(record_input_list))),
             [state.steering_angle_speed for state in record_input_list], color="black", label="planned")
    plt.plot([0, len(record_input_list)], [veh_params.steering.v_min, veh_params.steering.v_min],
             color="red", label="bounds")
    plt.plot([0, len(record_input_list)], [veh_params.steering.v_max, veh_params.steering.v_max],
             color="red")
    plt.plot(list(range(len(reconstructed_input_list))),
             [state.steering_angle_speed for state in reconstructed_input_list], color="blue",
             label="reconstructed")
    plt.legend()
    plt.ylabel("steering angle velocity")
    plt.subplot(2, 1, 2)
    plt.plot(list(range(len(record_input_list))),
             [state.acceleration for state in record_input_list], color="black", label="planned")
    plt.plot(list(range(len(reconstructed_input_list))),
             [state.acceleration for state in reconstructed_input_list], color="blue", label="reconstructed")
    # plt.plot([0, len(record_input_list)], [-veh_params.longitudinal.a_max, -veh_params.longitudinal.a_max],
    #          color="red", label="bounds")
    # plt.plot([0, len(record_input_list)], [veh_params.longitudinal.a_max, veh_params.longitudinal.a_max],
    #          color="red")
    plt.ylabel("acceleration")
    plt.show()

