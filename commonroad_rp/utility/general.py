from commonroad.common.file_reader import CommonRoadFileReader


def load_scenario_and_planning_problem(config, idx_planning_problem: int = 0):
    scenario, planning_problem_set = CommonRoadFileReader(config.general.path_scenario).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[
        idx_planning_problem
    ]

    return scenario, planning_problem, planning_problem_set
