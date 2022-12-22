import numpy as np

from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.geometry.shape import Polygon, ShapeGroup
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction, TrajectoryPrediction

import spot


def spot_setup_scenario(scenario: Scenario, planning_problem: PlanningProblem, spot_scenario_id: int = 1,
                        update_dict: dict = None):
    """
    Sets up spot scenario and updates obstacle parameters
    :param scenario: CR Scenario object
    :param planning_problem: CR PlanningProblem object
    :param spot_scenario_id: scenario ID for spot
    :param update_dict: update dict for obstacle parameters
    """
    # default FOV (no occlusions considered)
    field_of_view = np.empty([0, 2], float)
    # register spot scenario
    spot.registerScenario(spot_scenario_id, scenario.lanelet_network, scenario.dynamic_obstacles, [planning_problem],
                          field_of_view)
    # update obstacle parameters
    spot.updateProperties(spot_scenario_id, update_dict)


def spot_compute_occupancy_prediction(scenario: Scenario, spot_scenario_id, start_time_step: int,
                                      end_time_step: int, dt: float, num_threads: int = 4):
    """
    computes set-based occupancy prediction for all dynamic obstacles in scenario and updates the dynamic obstacles in
    CommonRoad scenario
    :param scenario: CR Scenario object
    :param spot_scenario_id: spot scneario id
    :param start_time: initial time for prediction
    :param end_time: predictions horizon
    :param num_threads: number of threads
    """
    # do occupancy prediction
    print('Calling SPOT prediction with ', num_threads, 'threads and for time step interval [', start_time_step, ', ', end_time_step,
          '] with step size of ', dt)
    spot_horizon = (end_time_step - start_time_step) * dt
    obstacles_occupancy = spot.doOccupancyPrediction(spot_scenario_id, 0.0, dt, spot_horizon, num_threads)

    print('Number of predicted obstacles:', len(obstacles_occupancy))
    # iterate over predicted occupancies and copy them into the CommonRoad scenario
    k = 0
    phantom_obstacles_id_begin = 50000000  # to avoid non-unique ids in cr_scenario
    for cpp_obstacle in obstacles_occupancy:
        # initialise
        occupancy_list = []
        for i in range(start_time_step, end_time_step):
            occ = Occupancy(i + 1, ShapeGroup([]))
            occupancy_list.append(occ)

        # all occupancy polygons (cpp_obstacle[1]) are stored in one list; thus, we need to separate each polygon
        i = 0  # iterator over occupancy_list
        for vertices_at_time_step in cpp_obstacle[1]:
            j = 1  # iterator over vertices_at_time_step
            b = 0  # index to select vertices_at_time_step that are the start of a new polygon
            while j < len(vertices_at_time_step[1]):
                compare_vertice = vertices_at_time_step[1][b]  # first vertice of next polygon
                if compare_vertice[0] == vertices_at_time_step[1][j][0] and compare_vertice[1] == \
                        vertices_at_time_step[1][j][1]:
                    if (j + 1) - b < 3:  # polygon has less than 3 vertices
                        print(
                            'Warning: one duplicated vertice skipped when copying predicted occupancies to CommonRoad')
                        b += 1  # try next vertice as first vertice (in case of equal vertices directly after each other)
                    else:
                        shape_obj = Polygon(np.array(vertices_at_time_step[1][b:j + 1]))
                        occupancy_list[i].shape.shapes.append(shape_obj)
                        j += 1
                        b = j
                j += 1
            assert b == j - 1, ('Last polygon not closed (at time_step = ', i, ', b = ', b)
            i += 1

        # set occupancy as prediction of cr_obstacle
        scenario.dynamic_obstacles[k].prediction = SetBasedPrediction(start_time_step + 1, occupancy_list[0:])
        k += 1


def spot_remove_scenario(spot_scenario_id):
    """
    Removes scneario with ID from spot
    """
    spot.removeScenario(spot_scenario_id)


def spot_update_init_state_obstacles(scenario: Scenario, time_step):
    """
    Updates the initial states of the obstacles in the scenario to their new state at the given timestep obtained from
    the ground truth trajectory. Used to compute SPOT prediction from the current obstacle state during cyclic
    re-planning
    """
    if time_step == 0:
        return scenario
    for dyn_obstacle in scenario.dynamic_obstacles:
        assert isinstance(dyn_obstacle.prediction, TrajectoryPrediction), \
            "Prediction must be of type TrajectoryPrediction"
        if dyn_obstacle.initial_state.time_step > time_step:
            continue
        else:
            try:
                pred_init_time_step = dyn_obstacle.prediction.trajectory.initial_time_step
                dyn_obstacle.initial_state = dyn_obstacle.prediction.trajectory.state_list[time_step - pred_init_time_step]
            except IndexError:
                # obstacle doesn't exist in scenario anymore at the given time step -> ignore
                scenario.remove_obstacle(dyn_obstacle)

    return scenario
