from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.trajectory import State
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
import matplotlib.pyplot as plt
import numpy as np

import xml.etree.ElementTree as ET
from reactive_planner import ReactivePlanner
from parameter import PlanningParameter, VehicleParameter

if __name__ == '__main__':
    print('Creating velocity reaching bundle....')

    file_path = '/home/julian/Desktop/commonroadlibrary/commonroad-scenarios/hand-crafted/ZAM_Over-1_1.xml'

    crfr = CommonRoadFileReader(file_path)
    scenario, PlanningProblemSet = crfr.open()
    plt.figure(figsize=(25, 10))
    draw_object(scenario)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(0.1)
    scenario = ReactivePlanner.add_obstacles_at_lanelet_edges(ReactivePlanner, scenario, file_path)

    if not PlanningProblemSet:
        root = ET.parse(file_path).getroot()
        for lane in root.findall('lanelet'):
            laneletid = lane.get('id')
            break
        print('No Planning Problem specified for this scenario!')
    else:
        root = ET.parse(file_path).getroot()
        for problem in root.findall('planningProblem'):
            problemid = problem.get('id')
            break
        init_state = PlanningProblemSet.find_planning_problem_by_id(int(problemid)).initial_state
        x, y = init_state.position
        heading = init_state.orientation
        x_1, y_1 = np.array([x, y]) + np.array([1, 1 * np.tan(heading)])
        tmp = np.array([[x, y], [x_1, y_1]])
        laneletid = scenario.lanelet_network.find_lanelet_by_position(tmp)[0][0]

    # create coordinate system
    reference_path = scenario.lanelet_network.find_lanelet_by_id(int(laneletid)).center_vertices
    curvilinear_cosy = create_coordinate_system_from_polyline(reference_path)

    # create collision checker for scenario
    collision_checker = create_collision_checker(scenario)

    # create initial state
    if not PlanningProblemSet:
        i_init = 2
        x, y = reference_path[i_init, :]
        x_1, y_1 = reference_path[i_init + 1, :]
        heading = np.arctan2(x_1 - x, y_1 - y)
    else:
        heading = init_state.orientation

    x_0 = State(**{'position': np.array([x, y]), 'orientation': heading, 'velocity': 0, 'acceleration': 0, 'yaw_rate': 0})
    print(x_0.position)

    # Set planning parameters and vehicle parameters
    params_planning = PlanningParameter(velocity_reaching=True)
    params_vehicle = VehicleParameter()

    planner:ReactivePlanner = ReactivePlanner(lanelet_network=scenario.lanelet_network)
    planner.set_parameters(params_planning)
    planner.set_parameters(params_vehicle)
    planner.set_reference_path(reference_path)

    x_cl = None

    for k in range(0, 50):
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
