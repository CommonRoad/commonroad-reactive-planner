from typing import Tuple, Optional

import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory


def load_scenario_and_planning_problem(path_scenario, idx_planning_problem: Optional[int] = None)\
        -> Tuple[Scenario, PlanningProblem, PlanningProblemSet]:
    """
    Loads a scenario and planning problem from the configuration.
    :param path_scenario: full path to scenario XML file
    :param idx_planning_problem: index of the planning problem (if none provided, first planning problem is returned)
    :return: scenario and planning problem and planning problem set
    """
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
    if idx_planning_problem is not None:
        try:
            planning_problem = planning_problem_set.find_planning_problem_by_id(idx_planning_problem)
        except KeyError:
            raise KeyError(f"<ReactivePlannerConfiguration.update()>:"
                           f"Planning Problem with ID: {idx_planning_problem} does not exist!")
    else:
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    return scenario, planning_problem, planning_problem_set


def retrieve_desired_velocity_from_pp(planning_problem: PlanningProblem):
    """Simple approach which retrieves average velocity from goal configuration"""
    goal = planning_problem.goal
    init_state = planning_problem.initial_state

    if hasattr(goal.state_list[0], 'velocity'):
        if goal.state_list[0].velocity.start > 0:
            desired_velocity = (planning_problem.goal.state_list[0].velocity.start + planning_problem.goal.state_list[
                0].velocity.end) / 2
        else:
            desired_velocity = (planning_problem.goal.state_list[0].velocity.end) / 2
    else:
        desired_velocity = init_state.velocity

    return desired_velocity


def shift_orientation(trajectory: Trajectory, interval_start=-np.pi, interval_end=np.pi):
    for state in trajectory.state_list:
        while state.orientation < interval_start:
            state.orientation += 2 * np.pi
        while state.orientation > interval_end:
            state.orientation -= 2 * np.pi
    return trajectory
