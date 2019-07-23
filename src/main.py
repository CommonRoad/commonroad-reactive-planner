from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.visualization.plot_helper import set_non_blocking
from commonroad.scenario.trajectory import State
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
import matplotlib.pyplot as plt
import numpy as np

from reactive_planner import ReactivePlanner
from route_planner import RoutePlanner
from parameter import PlanningParameter, VehicleParameter

if __name__ == '__main__':
    print('Creating velocity reaching bundle....')

    # Finish
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-20_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-25_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-3_1_T-1.xml'

    # Please set the path to your scenario here
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-3_2_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-3_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-1_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_4_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_2_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-2_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_3_T-1.xml'

    # One time Step (2,5)
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-21_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-14_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-17_1_T-1.xml'

    # A few steps (2,30)
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-27_1_T-1.xml'

            # Wird von Auto gerammt
                # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-23_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-2_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-6_1_T-1.xml'
            # Weicht von der Strecke ab..
    scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-1_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-29_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-16_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-9_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-7_1_T-1.xml'

    # No Step (3,30)
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-22_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-10_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-4_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-19_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-12_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-5_1_T-1.xml'
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-8_1_T-1.xml'

    # Doesn't make the lanechange
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-15_1_T-1.xml'

    # Car in front does not get recognized
        # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-24_1_T-1.xml'


    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-11_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-26_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-18_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-13_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/US101/USA_US101-28_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_7_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_8_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_2_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_5_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_12_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_7_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_13_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_9_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_15_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_13_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_12_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_6_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_14_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_3_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_8_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_2_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_6_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_11_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_4_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_10_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_9_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_4_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_3_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_10_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_5_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_11_T-1.xml'

    # viel zu klein-> reference path
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_A99-1_1_T-1.xml'
    # discrete
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/ZAM_Intersect-1_2_S-1.xml'
    # discrete network
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_A9-2_1_T-1.xml'
    # alle, aber müsste lane change einleiten
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Gar-1_1_T-1.xml'
    # zu wenige center vertices -> zu geringe auflösung für discrete/lane change
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Muc-4_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/ZAM_Merge-1_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/ZAM_Over-1_1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Hhr-1_1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Ffb-1_3_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/ZAM_HW-1_1_S-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/ZAM_Intersect-1_1_S-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_A9-1_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Muc-3_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Ffb-1_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Ffb-2_2_S-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_B471-1_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Muc-2_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/ZAM_Urban-1_1_S-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Ffb-2_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Muc-1_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/hand-crafted/DEU_Ffb-1_2_S-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_Lanker-2_3_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-DEU_B471-2_1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_Lanker-2_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_Lanker-2_2_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_US101-30_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-DEU_B471-1_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_Lanker-1_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_US101-32_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_Lanker-2_4_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_US101-31_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_US101-33_1_T-1.xml'
    # scenario_path = '/home/raphaelrg/Desktop/commonroad-scenarios/cooperative/C-USA_Lanker-1_2_T-1.xml'

    # Initialize reactive planner
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()
    plt.figure(figsize=(25, 10))
    draw_object(scenario)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(0.1)
    #scenario = ReactivePlanner.add_obstacles_at_lanelet_edges(ReactivePlanner, scenario, file_path)

    # set reference path
    route_planner = RoutePlanner(scenario.lanelet_network, scenario_path)
    route_planner.create_reference_path_network()
    route_planner.plan_all_reference_paths()

    source_position = route_planner.planning_problem.initial_state.position
    sourcelanelets = route_planner.lanelet_network.find_lanelet_by_position(np.array([source_position]))
    source_lanelet = route_planner.lanelet_network.find_lanelet_by_id(sourcelanelets[0][0])

    reference_path = route_planner.set_reference_lane(0, source_position)

    #reference_path = scenario.lanelet_network.find_lanelet_by_id(int(source_lanelet.lanelet_id)).center_vertices

    # create coordinate system from reference path
    curvilinear_cosy = create_coordinate_system_from_polyline(reference_path)

    # create collision checker for scenario
    collision_checker = create_collision_checker(scenario)

    # create initial state
    x, y = route_planner.planning_problem.initial_state.position
    heading = route_planner.planning_problem.initial_state.orientation
    x_0 = State(**{'position':np.array([x,y]),'orientation':heading, 'velocity':10, 'acceleration':0,'yaw_rate':0})

    # Set planning parameters and vehicle parameters
    params_planning = PlanningParameter(velocity_reaching=True)
    params_vehicle = VehicleParameter()

    planner: ReactivePlanner = ReactivePlanner(lanelet_network=scenario.lanelet_network)
    planner.set_parameters(params_planning)
    planner.set_parameters(params_vehicle)
    planner.set_reference_path(reference_path)

    plt.plot(reference_path[:, 0], reference_path[:, 1], '-*g', linewidth=1, zorder=10)
    x_cl = None

    for k in range(0, 150):
        print(k)
        optimal = planner.plan(x_0, collision_checker.time_slice(k), cl_states=x_cl)
        # convert to CR obstacle
        ego = planner.convert_cr_trajectory_to_object(optimal[0])
        draw_object(scenario, draw_params={'time_begin': k, 'time_end': k})
        #draw_object(planning_problem_set)
        draw_object(ego)
        draw_object(ego.prediction.occupancy_at_time_step(1))
        plt.pause(0.1)

        # TODO: Reste x_0 to reach starting velocity?
        x_0 = optimal[0].state_list[1]
        x_cl = (optimal[2][1], optimal[3][1])

        print("Goal state is: {}".format(optimal[1].state_list[-1]))

        vel_is_too_slow, obstacles_ahead = planner.check_velocity_of_car_ahead_too_slow(scenario, ego, k)

        if vel_is_too_slow:
            changed_velocity, reference_path = planner.check_current_state(route_planner, scenario, ego, reference_path, obstacles_ahead)
            x_0.velocity = changed_velocity
            x_cl = planner._compute_initial_states(x_0)
            # x_cl = planner.create_new_cl_state(x_0, x_cl, changed_velocity)

            planner.set_reference_path(reference_path)
            plt.plot(reference_path[:, 0], reference_path[:, 1], '-*g', linewidth=1, zorder=10)

    print('Done')
    plt.show(block=True)
