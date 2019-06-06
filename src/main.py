from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.trajectory import State
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
import matplotlib.pyplot as plt
import numpy as np

import xml.etree.ElementTree as ET
from reactive_planner import ReactivePlanner


if __name__ == '__main__':
    print('Creating velocity reaching bundle....')

    crfr = CommonRoadFileReader('/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Over-1_1.xml')
    scenario, _ = crfr.open()
    plt.figure(figsize=(25, 10))
    draw_object(scenario)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(0.1)

    root = ET.parse('/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Over-1_1.xml').getroot()
    for lane in root.findall('lanelet'):
        laneletid= lane.get('id')
        break

    # create coordinate system
    reference_path = scenario.lanelet_network.find_lanelet_by_id(int(laneletid)).center_vertices
    curvilinear_cosy = create_coordinate_system_from_polyline(reference_path)

    # create collision checker for scenario
    collision_checker = create_collision_checker(scenario)

    # convert coordinates and create initial state
    x, y = curvilinear_cosy.convert_to_cartesian_coords(25, 0)
    x_0 = State(**{'position':np.array([x, y]),'orientation':0.04, 'velocity':10, 'acceleration':0,'yaw_rate':0})

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