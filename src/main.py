from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.trajectory import State, Trajectory
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
from commonroad.common.solution_writer import CommonRoadSolutionWriter, VehicleModel, VehicleType, CostFunction
import matplotlib.pyplot as plt
import numpy as np
import cv2

from reactive_planner import ReactivePlanner
from route_planner import RoutePlanner
from parameter import PlanningParameter, VehicleParameter
from scenario_classes import PlanningProblem, Scenario

if __name__ == '__main__':
    print('Creating velocity reaching bundle....')

    # Finish
    #scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Over-1_1.xml'
    scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-20_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-25_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-3_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-1_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-29_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-21_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-11_1_T-1.xml'
    scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-18_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_US101-32_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_US101-33_1_T-1.xml'

    # A few steps (2,30)
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-27_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-2_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-6_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-16_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-9_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-7_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-14_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-17_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-4_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-19_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-12_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-5_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-26_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-13_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-28_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_A9-1_1_T-1.xml'
            # Wird von Auto gerammt
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-23_1_T-1.xml'
            # Kreuzung!
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Muc-3_1_T-1.xml'
    # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-2_1_T-1.xml'

    # No Step (3,30)
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-22_1_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-10_1_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-8_1_T-1.xml'
    # Doesn't make the lanechange
    scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-15_1_T-1.xml'
    # Car in front does not get recognized
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-24_1_T-1.xml'
    # Do not work at all
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-3_2_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-3_1_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-1_1_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_4_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_2_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_1_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-2_1_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_3_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_7_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_8_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_2_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_1_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_5_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_12_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_7_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_13_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_9_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_15_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_13_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_12_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_6_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_14_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_3_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_8_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_2_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_6_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_11_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_4_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_10_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_9_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_4_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_3_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_10_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_1_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_5_T-1.xml'
        # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_11_T-1.xml'
        # viel zu klein-> reference path
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_A99-1_1_T-1.xml'
        # discrete
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Intersect-1_2_S-1.xml'
        # discrete network
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_A9-2_1_T-1.xml'
        # alle, aber müsste lane change einleiten
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Gar-1_1_T-1.xml'
        # zu wenige center vertices -> zu geringe auflösung für discrete/lane change
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Muc-4_1_T-1.xml'
    #scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Merge-1_1_T-1.xml'
    #scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Over-1_1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Hhr-1_1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-1_3_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_HW-1_1_S-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Intersect-1_1_S-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-1_1_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-2_2_S-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_B471-1_1_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Muc-2_1_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Urban-1_1_S-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Muc-1_1_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-1_2_S-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-2_3_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-DEU_B471-2_1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-2_1_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-2_2_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_US101-30_1_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-DEU_B471-1_1_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-1_1_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-2_4_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_US101-31_1_T-1.xml'
            # scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-1_2_T-1.xml'

    # Initialize reactive planner
    planning_problem = PlanningProblem(scenario_path=scenario_path)
    scenario = Scenario(scenario_path=scenario_path)

    plt.figure('start', figsize=(25, 10))
    draw_object(scenario.scenario_set)
    draw_object(planning_problem.planning_problem_set)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(0.1)

    # create collision checker for scenario.scneario_set
    collision_checker = create_collision_checker(scenario.scenario_set)

    # get initial state
    init_state = planning_problem.planning_problem_set.find_planning_problem_by_id(int(planning_problem.problem_id)).initial_state
    x, y = init_state.position
    heading = init_state.orientation
    velocity = init_state.velocity
    yaw_rate = init_state.yaw_rate
    x_0 = State(**{'position':np.array([x,y]),'orientation':heading, 'velocity':velocity, 'acceleration':0,'yaw_rate':yaw_rate})

    # Set planning parameters and vehicle parameters
    params_planning = PlanningParameter(velocity_reaching=True)
    params_vehicle = VehicleParameter()

    # create reactive planner
    planner: ReactivePlanner = ReactivePlanner(scenario=scenario)
    planner.set_parameters(params_planning)
    planner.set_parameters(params_vehicle)
    planner.set_desired_speed(velocity)

    x_cl = None


    write_solution = True
    solution_path = '/home/fabian/Praktikum/solution_commonroad/'
    solution_model = 'KS'

    video = True
    picture_list = []

    if write_solution:
        radius = init_state.velocity / np.abs(init_state.yaw_rate)
        steering_angle = np.arctan((params_vehicle.length_front + params_vehicle.length_rear) / radius)
        init_state.steering_angle = steering_angle
        state_list = list()
        state_list.append(init_state)
        solution_traj = Trajectory(initial_time_step=0, state_list=state_list)
        solution_step = 1

    for k in range(0, 50):
        optimal = planner.plan(x_0, collision_checker.time_slice(k), cl_states=x_cl, k=k)

        planner.plotting(k, scenario.scenario_set, planning_problem.planning_problem_set, planner._reference, planner._ego, only_new_time_step = False)

        x_0 = optimal[0].state_list[1]
        x_cl = (optimal[2][1], optimal[3][1])

        print("Goal state is: {}".format(optimal[1].state_list[-1]))

        if write_solution:
            state = x_0
            radius = state.velocity / np.abs(state.yaw_rate)
            steering_angle = np.arctan((params_vehicle.length_front + params_vehicle.length_rear) / radius)

            state.steering_angle = steering_angle
            state.time_step = solution_step

            solution_traj.state_list.append(state)
            solution_step += 1

        if video:
            picture_path = ("/home/fabian/Praktikum/solution_commonroad/video/file%d.png" % k)
            plt.savefig(picture_path)
            picture_list.append(picture_path)


    print('Done')
    plt.show(block=False)

    if (solution_model == 'KS') & write_solution:
        solution_writer = CommonRoadSolutionWriter(solution_path, scenario.scenario_set.benchmark_id, params_planning.t_step_size,
                                                   VehicleType.FORD_ESCORT,
                                                   VehicleModel.KS, CostFunction.WX1)
        solution_writer.add_solution_trajectory(trajectory=solution_traj,
                                                planning_problem_id=int(route_planner.planning_problem.planning_problem_id))
        solution_writer.write_to_file(overwrite=True)
        print('Solution stored!')

    elif (solution_model == 'PM') & write_solution:
        for state in solution_traj.state_list:
            state.velocity_y = np.sin(state.orientation) * state.velocity
            state.velocity = np.cos(state.orientation) * state.velocity

        solution_writer = CommonRoadSolutionWriter(solution_path, scenario.scenario_set.benchmark_id, params_planning.t_step_size,
                                                   VehicleType.FORD_ESCORT,
                                                   VehicleModel.PM, CostFunction.WX1)

        solution_writer.add_solution_trajectory(trajectory=solution_traj,
                                                planning_problem_id=int(route_planner.planning_problem.planning_problem_id))
        solution_writer.write_to_file(overwrite=True)
        print('Solution stored!')

    if video:
        img_array = []
        for im in picture_list:
            img = cv2.imread(im)
            height, width, layers = img.shape
            size = (width, height)
            img_array.append(img)

        out = cv2.VideoWriter('/home/fabian/Praktikum/Commonroad/solution_commonroad/video/project.mp4',cv2.VideoWriter_fourcc('m','p','4','v'), 10, size)

        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()

        print('Video created!')