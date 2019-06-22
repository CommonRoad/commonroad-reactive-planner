from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.trajectory import State
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
import matplotlib.pyplot as plt
import numpy as np

import xml.etree.ElementTree as ET
from reactive_planner import ReactivePlanner
from route_planner import RoutePlanner
from route_planner import find_reference_path_and_lanelets_leading_to_goal


if __name__ == '__main__':
    print('Creating velocity reaching bundle....')

    # Please set the path to your scenario here
    scenario_path = '/home/julian/Desktop/commonroadlibrary/commonroad-scenarios/hand-crafted/ZAM_Over-1_1.xml'

    # Initialize reactive planner
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
    plt.figure(figsize=(25, 10))
    draw_object(scenario)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(0.1)

    # Find planning problem or select initial state
    if not planning_problem_set:
        root = ET.parse(scenario_path).getroot()
        for lane in root.findall('lanelet'):
            laneletid = lane.get('id')
            break

        print('No Planning Problem specified for this scenario! Lanelet ', laneletid, ' is chosen.')

    else:
        root = ET.parse(scenario_path).getroot()
        for problem in root.findall('planningProblem'):
            problemid = problem.get('id')
            break

        planning_problem = planning_problem_set.find_planning_problem_by_id(int(problemid))

    goal_lanelet = scenario.lanelet_network.lanelets_in_proximity(
        planning_problem.goal.state_list[0].position.center, 100)

    goal_lanelet_id = list(goal_lanelet)[0].lanelet_id

    # find reference path
    route_planner = RoutePlanner(scenario.lanelet_network)
    reference_path, lanelets_leading_to_goal = find_reference_path_and_lanelets_leading_to_goal(
        route_planner=route_planner,
        planning_problem=planning_problem,
        target_lanelet_id=goal_lanelet_id,
        allow_overtaking=False,
        resampling_step_reference_path=1.5,
        max_curvature_reference_path=0.15)

    # create coordinate system
    curvilinear_cosy = create_coordinate_system_from_polyline(reference_path)

    # create collision checker for scenario
    collision_checker = create_collision_checker(scenario)

    # create initial state
    x, y = planning_problem.initial_state.position
    heading = planning_problem.initial_state.orientation
    x_0 = State(**{'position':np.array([x,y]),'orientation':heading, 'velocity':10, 'acceleration':0,'yaw_rate':0})

    planner:ReactivePlanner = ReactivePlanner()
    planner.set_reference_path(reference_path)

    x_cl = None

    for k in range(0, 100):
        optimal = planner.plan(x_0, collision_checker, cl_states=x_cl)
        # convert to CR obstacle
        ego = planner.convert_cr_trajectory_to_object(optimal[0])
        draw_object(ego)
        draw_object(ego.prediction.occupancy_at_time_step(1))
        plt.pause(0.1)

        x_0 = optimal[0].state_list[1]
        x_cl = (optimal[2][1], optimal[3][1])

        print("Goal state is: {}".format(optimal[1].state_list[-1]))

    print('Done')
    plt.show(block=True)