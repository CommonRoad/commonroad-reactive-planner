__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.1"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Alpha"

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
from scenario_helpers import *
import commonroad_cc.visualization.draw_dispatch as crd
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker


import matplotlib.pyplot as plt

#import spot



if __name__ == '__main__':

    scenario = 'scenarios_lg_single_lane_23.xml'
    scenario = 'scenarios_lg_borregas_ave_59.xml'

    print('Loading scenario {}'.format(scenario))

    # Load example scenario ZAM Over
    crfr = CommonRoadFileReader(scenario)
    scenario, problem = crfr.open()
    scenario.remove_obstacle(scenario.obstacle_by_id(999999))  # remove ego vehicle
    #spot_setup(scenario,next(iter(problem.planning_problem_dict.values())))
    #set_obstacle_occupancy_prediction(scenario)
    road_boundary_sg, road_boundary_obstacle = create_road_boundary(scenario, draw=False)
    lanelet_network = scenario.lanelet_network
    lanelets = lanelet_network.lanelets



    plt.figure(figsize=(25, 10))
    draw_object(scenario)
    crd.draw_object(road_boundary_sg)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(0.1*10)

    # create collision checker for scenario
    collision_checker = create_collision_checker(scenario)
    collision_checker.add_collision_object(road_boundary_sg)

    # get planning problem from CommonRoad file
    prob: PlanningProblem = next(iter(problem.planning_problem_dict.values()))
    ego_initial_state = prob.initial_state
    draw_object(ego_initial_state)

    print('Initial state of ego vehicle is {}'.format(ego_initial_state))

    # create coordinate system
    ego_lanelet_id = scenario.lanelet_network.find_lanelet_by_position([ego_initial_state.position])[0][0]
    print('Ego vehice is located in lanelet id={}'.format(ego_lanelet_id))
    reference_path = obtain_reference_path(ego_initial_state, scenario)
    curvilinear_cosy = create_coordinate_system_from_polyline(reference_path)



    # create initial state
    x_0 = ego_initial_state

    planner: ReactivePlanner = ReactivePlanner(0.2, 6, 30)
    planner.set_reference_path(reference_path)

    x_cl = None

    optimal = planner.plan(x_0, collision_checker, cl_states=x_cl)
    # convert to CR obstacle
    ego = planner.convert_cr_trajectory_to_object(optimal[0])
    draw_object(ego)
    for occ in ego.prediction.occupancy_set:
        draw_object(occ)
    plt.pause(0.1)

    #for state in optimal[0].state_list:
    #    print(state.orientation)

    x_0 = optimal[0].state_list[1]
    x_cl = (optimal[2][1], optimal[3][1])

        #print("Goal state is: {}".format(optimal[1].state_list[-1]))

    print('Done')
    plt.show(block=True)