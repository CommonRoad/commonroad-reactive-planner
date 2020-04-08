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
from commonroad_rp.utils import compute_curvature_from_polyline
from scenario_helpers import *
import commonroad_cc.visualization.draw_dispatch as crd
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker
import matplotlib.pyplot as plt
from commonroad_ccosy.geometry.util import resample_polyline



from scipy import spatial
from scipy.interpolate import splprep, splev

import spot
import os



def smoothing_reference_path(reference_path):
    """
    Smooths a given reference polyline for lower curvature rates. The smoothing is done using splines from scipy.
    :param reference_path: The reference_path to smooth [array]
    :return: The smoothed reference
    """
    transposed_reference_path = reference_path.T
    okay = np.where(np.abs(np.diff(transposed_reference_path[0])) + np.abs(np.diff(transposed_reference_path[1])) > 0)
    xp = np.r_[transposed_reference_path[0][okay], transposed_reference_path[0][-1]]
    yp = np.r_[transposed_reference_path[1][okay], transposed_reference_path[1][-1]]
    tck, u = splprep([xp, yp], s=0)
    # tck, u = splprep(transposed_reference_path)
    u_new = np.linspace(u.min(), u.max(), 1000)
    x_new, y_new = splev(u_new, tck, der=0)

    return np.array([x_new, y_new]).transpose()


if __name__ == '__main__':

    scenario = 'scenarios_lg_single_lane_23.xml'
    # scenario = 'scenarios_lg_borregas_ave_59.xml'
    # scenario = 'scenarios_borregas_ave_traffic_routing_3_v2_scenario_borregas_ave_844.xml'

    print('Loading scenario {}'.format(scenario))

    # fail-safe parameter for planner
    N_fs = 30
    t_fs = 6.


    # Load example scenario ZAM Over
    crfr = CommonRoadFileReader(scenario)
    scenario, problem = crfr.open()
    ego_original = scenario.obstacle_by_id(999999)

    scenario.remove_obstacle(ego_original)  # remove ego vehicle


    update_dict = {
        "obstacles": {
            0: {  # 0 means that all obstacles will be changed
                "a_max": 8.0,
                "a_max_long": 6.0,
                "compute_occ_m1": True,
                "compute_occ_m2": True,
                "compute_occ_m3": True
            }
        },
        "egoVehicle": {
            0: {  # ID is ignored for ego vehicle (which is created based on cr_planning problem)
                "a_max": 1.0,
                "length": 5.0,
                "width": 2.0
            }
        }
    }
    spot_setup(scenario,next(iter(problem.planning_problem_dict.values())), update_dict)
    set_obstacle_occupancy_prediction(scenario,end_time=len(ego_original.prediction.trajectory.state_list)*scenario.dt+t_fs)
    road_boundary_sg, road_boundary_obstacle = create_road_boundary(scenario, draw=False)
    lanelet_network = scenario.lanelet_network
    lanelets = lanelet_network.lanelets


    # Create figure and draw CommonRoad scneario
    plt.figure(figsize=(25, 10))
    draw_object(scenario, draw_params=draw_parameters_scenario)
    draw_object(ego_original, draw_params=draw_parameters_intended)
    #draw_object(ego_original.initial_state)
    #crd.draw_object(road_boundary_sg)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(1)

    # create collision checker for scenario
    collision_checker = create_collision_checker(scenario)
    collision_checker.add_collision_object(road_boundary_sg)

    print('Initial state of ego vehicle is {}'.format(ego_original.initial_state))

    # create reference path for curvilinear coordinate system
    reference_path = obtain_reference_path(ego_original.prediction.trajectory, scenario)
    reference_path = smoothing_reference_path(reference_path)
    #reference_path = resample_polyline(reference_path, step = 1)

    #plt.figure()
    #plt.plot(reference_path[:,0],reference_path[:,1],'k')
    if reference_path.shape[0] < 5:
        reference_path = resample_polyline(reference_path)
    reference_path = smooth_reference(reference_path)
    #plt.plot(reference_path[:,0], reference_path[:,1], 'b')
    #plt.show(block=True)

    print(f'Smoothing done with curvature of {np.max(compute_curvature_from_polyline(reference_path))}')

    # create new instance of reactive planner
    t_h = 3.0 # planning horizon
    desired_speed = 1.5 + ego_original.prediction.trajectory.state_list[0].velocity
    planner: ReactivePlanner = ReactivePlanner(scenario.dt, t_h, int(t_h / scenario.dt), scenario_id=scenario.benchmark_id)
    planner.set_reference_path(reference_path)
    planner.set_desired_velocity(desired_speed)


    # obtain initial state for fail-safe planner and draw it
    x_0 = ego_original.prediction.trajectory.state_list[0]
    x_0.yaw_rate = 0
    x_0.acceleration = 0
    draw_object(x_0)
    plt.plot(x_0.position[0],x_0.position[1],'xk')

    # plan trajectory
    optimal = planner.plan(x_0, collision_checker)


    if optimal is not None:

        # convert to CR obstacle
        ego = planner.convert_cr_trajectory_to_object(optimal[0])
        # draw intended trajectory
        for occ in ego_original.prediction.occupancy_set:
            draw_object(occ, draw_params=draw_parameters_intended)
        # draw fail-safe trajectory
        for occ in ego.prediction.occupancy_set:
            draw_object(occ, draw_params=draw_parameters_fail_safe)

        plt.pause(0.1)

        # scenario.add_objects(ego)
        # plt.figure()
        # draw_object(scenario.lanelet_network)
        # for state in ego.prediction.trajectory.state_list:
        #     print(state.time_step)
        #     draw_object(scenario.occupancies_at_time_step(state.time_step))
        #     plt.axis('equal')
        #     plt.autoscale()
        #     plt.pause(0.5)



    print('Done')
    plt.show(block=True)
    remove_scenario_from_spot()
