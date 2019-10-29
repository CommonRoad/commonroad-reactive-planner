__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.5"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Beta"

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_ccosy.geometry.util import chaikins_corner_cutting, resample_polyline
from scenario_helpers import *
import commonroad_cc.visualization.draw_dispatch as crd
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker


import matplotlib.pyplot as plt

#import spot



if __name__ == '__main__':

    scenario = 'scenarios_lg_single_lane_23.xml'
    scenario = 'scenarios_lg_borregas_ave_59.xml'
    #scenario = 'scenarios_borregas_ave_traffic_routing_3_v2_scenario_borregas_ave_844.xml'

    print('Loading scenario {}'.format(scenario))

    # fail-safe parameter for planner
    N_fs = 30
    t_fs = 6.


    # Load example scenario ZAM Over
    crfr = CommonRoadFileReader(scenario)
    scenario, problem = crfr.open()
    ego_original = scenario.obstacle_by_id(999999)
    scenario.remove_obstacle(ego_original)  # remove ego vehicle
    # spot_setup(scenario,next(iter(problem.planning_problem_dict.values())))
    # set_obstacle_occupancy_prediction(scenario,end_time=len(ego_original.prediction.state_list)*scenario.dt+t_fs)
    road_boundary_sg, road_boundary_obstacle = create_road_boundary(scenario, draw=False)
    lanelet_network = scenario.lanelet_network
    lanelets = lanelet_network.lanelets


    # Create figure and draw CommonRoad scneario
    plt.figure(figsize=(25, 10))
    draw_object(scenario)
    #crd.draw_object(road_boundary_sg)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(1)

    # create collision checker for scenario
    collision_checker = create_collision_checker(scenario)
    collision_checker.add_collision_object(road_boundary_sg)

    # get planning problem from CommonRoad file
    prob: PlanningProblem = next(iter(problem.planning_problem_dict.values()))
    ego_initial_state = prob.initial_state
    draw_object(ego_initial_state)

    print('Initial state of ego vehicle is {}'.format(ego_initial_state))

    # create reference path for curvilinear coordinate system
    reference_path = obtain_reference_path(ego_initial_state, scenario)
    reference_path = resample_polyline(reference_path, step = 2)
    # smooth reference
    reference_path = chaikins_corner_cutting(reference_path)
    reference_path = chaikins_corner_cutting(reference_path)
    reference_path = chaikins_corner_cutting(reference_path)

    # plt.plot(reference_path[:,0],reference_path[:,1],'-xk')
    # draw_object(Rectangle(4.5,2.1,center=ego_initial_state.position,orientation=ego_initial_state.orientation),draw_params=draw_parameters_itended)
    # plt.show(block=True)

    # create new instance of reactive planner
    planner: ReactivePlanner = ReactivePlanner(0.2, t_fs, N_fs, factor=10)
    planner.set_reference_path(reference_path)

    # compute TTR for branch point of fail-safe trajectory
    ttr = compute_simplified_ttr(ego_original.prediction.trajectory, collision_checker,planner.coordinate_system(),scenario.dt,planner.constraints)
    print("TTR is {}".format(ttr))

    # obtain initial state for fail-safe planner and draw it
    x_0 = ego_original.prediction.trajectory.state_list[ttr]
    draw_object(x_0)

    # plan trajectory
    optimal = planner.plan(x_0, collision_checker)

    # convert to CR obstacle
    ego = planner.convert_cr_trajectory_to_object(optimal[0])
    # draw intended trajectory
    for occ in ego_original.prediction.occupancy_set:
        draw_object(occ, draw_params=draw_parameters_itended)
    # draw fail-safe trajectory
    for occ in ego.prediction.occupancy_set:
        draw_object(occ, draw_params=draw_parameters_fail_safe)
    plt.pause(0.1)

    print('Done')
    plt.show(block=True)