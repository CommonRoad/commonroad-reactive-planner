import os
import glob

from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
# from commonroad.visualization.draw_dispatch_cr import draw_object

from commonroad_dc.collision.visualization.draw_dispatch import draw_object
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker

from commonroad_rl.gym_commonroad.vehicle import Vehicle
from commonroad_rl.gym_commonroad.constants import VEHICLE_PARAMS

from commonroad_rp.reactive_planner import ReactivePlanner
from route_planner import RoutePlanner, ReferenceRouteManager

base_dir = "./scenarios"
scenario_path = os.path.join(base_dir, "*.xml")
files = sorted(glob.glob(scenario_path))

crfr = CommonRoadFileReader(files[0])
scenario, problem_set = crfr.open()
planning_problem = list(problem_set.planning_problem_dict.values())[0]

# create road boundary 
(road_boundary_obstacle, 
road_boundary_sg_triangles
) = create_road_boundary_obstacle(scenario, method='triangulation')
collision_checker_scenario = create_collision_checker(scenario)
collision_checker_scenario.add_collision_object(road_boundary_sg_triangles)

# initial state configuration
# the attribute "acceleration" is needed for reactive planner
problem_init_state = planning_problem.initial_state
current_velocity = problem_init_state.velocity
if not hasattr(problem_init_state, 'acceleration'):
    problem_init_state.acceleration = 0.
x_0 = deepcopy(problem_init_state)

# init vehicle model
ego_vehicle = Vehicle.create_vehicle(VEHICLE_PARAMS)
ego_vehicle.reset(problem_init_state, scenario.dt)

# goal state configuration
goal = planning_problem.goal
if hasattr(planning_problem.goal.state_list[0], 'velocity'):
    if planning_problem.goal.state_list[0].velocity.start != 0:
        desired_velocity = planning_problem.goal.state_list[0].velocity.start
    else:
        desired_velocity = (planning_problem.goal.state_list[0].velocity.start
                            + planning_problem.goal.state_list[0].velocity.end) / 2
else:
    desired_velocity = problem_init_state.velocity

# route planner
route_planner = RoutePlanner(scenario.benchmark_id, scenario.lanelet_network, planning_problem)
ref_route_manager = ReferenceRouteManager(route_planner)

# reactive planner configuration
DT = scenario.dt
T_H = 2
planner = ReactivePlanner(dt=DT, t_h=T_H, N=int(T_H / DT), v_desired=desired_velocity)
planner.set_d_sampling_parameters(-3.0, 3.0)

# planning
record_state_list = list()
record_state_list.append(x_0)
x_cl = None

# set reference path
ref_path = ref_route_manager.get_ref_path(x_0)
planner.set_reference_path(ref_path)

os.makedirs("./img", exist_ok=True)

while not goal.is_reached(x_0):
    # set desired velocity
    planner.set_desired_velocity(desired_velocity, current_velocity)
    optimal = planner.plan(x_0, collision_checker_scenario)
    current_count = len(record_state_list)
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

    x_0 = deepcopy(record_state_list[-1])
    ego_vehicle._update_state(x_0)
    x_cl = (optimal[2][1], optimal[3][1])

    print(f"current time step: {current_count}")
    plt.figure(figsize=(20, 10))
    # draw scenario
    draw_object(scenario, draw_params={'time_begin': current_count})
    draw_object(planning_problem)
    draw_object(ego_vehicle.collision_object, draw_params={"collision": {"facecolor": "green", "zorder": 30}})
    plt.plot(np.array(planned_x), np.array(planned_y), color='k', marker='o', markersize=1, zorder=20, linewidth=0.5,
            label='planned trajectories')
    plt.plot(ref_path[:, 0], ref_path[:, 1], color='b', marker='.', markersize=1, zorder=20, linewidth=0.8,
            label='reference path')
    plt.gca().set_aspect('equal')
    plt.autoscale()
    plt.savefig(f"./img/{scenario.benchmark_id}_{current_count + 1}.png", format='png', dpi=300, bbox_inches='tight')
    plt.close()
