from typing import Tuple

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.scenario.scenario import Scenario


def load_scenario_and_planning_problem(config, idx_planning_problem: int = 0)\
        -> Tuple[Scenario, PlanningProblem, PlanningProblemSet]:
    """
    Loads a scenario and planning problem from the configuration.
    :param config: configuration
    :param idx_planning_problem: index of the planning problem
    :return: scenario and planning problem and planning problem set
    """
    scenario, planning_problem_set = CommonRoadFileReader(config.general.path_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[idx_planning_problem]

    return scenario, planning_problem, planning_problem_set
