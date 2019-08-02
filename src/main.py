from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.common.solution_writer import CommonRoadSolutionWriter, VehicleModel, VehicleType, CostFunction
import matplotlib.pyplot as plt
import numpy as np

from reactive_planner import ReactivePlanner
from parameter import PlanningParameter, VehicleParameter
from scenario_classes import PlanningProblem, Scenario

if __name__ == '__main__':
    print('Creating velocity reaching bundle....')

    # solution parameter
    write_solution = True
    solution_path = '/home/fabian/Praktikum/solution_commonroad/'
    solution_model = 'KS'

    scenario_path = '/home/fabian/Praktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-15_1_T-1.xml'

    # create scenario object
    scenario = Scenario(scenario_path=scenario_path)

    # create planning problem object
    planning_problem = PlanningProblem(scenario_path=scenario_path)

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
    planner: ReactivePlanner = ReactivePlanner(scenario=scenario, planning_problem=planning_problem)
    planner.set_parameters(params_planning)
    planner.set_parameters(params_vehicle)
    planner.set_desired_speed(velocity)

    # plot initial scenario
    plt.figure('start', figsize=(25, 10))
    draw_object(scenario.scenario_set)
    draw_object(planning_problem.planning_problem_set)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(0.1)

    x_cl = None

    # initial solution
    if write_solution:
        radius = init_state.velocity / np.abs(init_state.yaw_rate)
        steering_angle = np.arctan((params_vehicle.length_front + params_vehicle.length_rear) / radius)
        init_state.steering_angle = steering_angle
        state_list = list()
        state_list.append(init_state)
        solution_traj = Trajectory(initial_time_step=0, state_list=state_list)
        solution_step = 1

    for k in range(0, 20):
        # compute optimal trajectory for given time step
        optimal = planner.plan(x_0, cl_states=x_cl, k=k)

        # compute new states
        ego = planner.convert_cr_trajectory_to_object(optimal[0])
        x_0 = optimal[0].state_list[1]
        x_cl = (optimal[2][1], optimal[3][1])

        # plot new scenario state
        planner.plotting(k, only_new_time_step=False)

        print("Goal state is: {}".format(optimal[1].state_list[-1]))

        # save current trajectory for solution writer
        if write_solution:
            state = x_0
            radius = state.velocity / np.abs(state.yaw_rate)
            steering_angle = np.arctan((params_vehicle.length_front + params_vehicle.length_rear) / radius)

            state.steering_angle = steering_angle
            state.time_step = solution_step

            solution_traj.state_list.append(state)
            solution_step += 1

    print('Done')
    plt.show(block=False)

    # safe solution
    if (solution_model == 'KS') & write_solution:
        solution_writer = CommonRoadSolutionWriter(solution_path, scenario.scenario_set.benchmark_id, params_planning.t_step_size,
                                                   VehicleType.FORD_ESCORT,
                                                   VehicleModel.KS, CostFunction.WX1)
        solution_writer.add_solution_trajectory(trajectory=solution_traj,
                                                planning_problem_id=int(planning_problem.problem_id))
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
                                                planning_problem_id=int(planning_problem.problem_id))
        solution_writer.write_to_file(overwrite=True)
        print('Solution stored!')