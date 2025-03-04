import unittest
from pathlib import Path
import os

from commonroad_rp.high_level_planner import HighLevelPlanner
from commonroad_rp.utility.general import load_scenario_and_planning_problem


def run_rpi(config_path: Path, scenario: str, min_ts: int = None):
    rpi = HighLevelPlanner(config_path=config_path, scenario=scenario)
    goal_reached, timestep = rpi.run()
    assert goal_reached is True
    if min_ts:
        # check if rule is actually evaluated and forces car do slow down or stop
        assert timestep > min_ts, f"timestep {timestep} is not greater than {min_ts}"


class MonitorTest(unittest.TestCase):

    def test_intersection(self):
       scenario = "DEU_testRightBeforeLeft-1_1_T-1.pb"
       config_path = Path(__file__).parents[1] / ("configurations/DEU_testRightBeforeLeft"
                                                  "-1_1_T-1.yaml")
       run_rpi(config_path, scenario)

    def test_stop_line(self):
        scenario = "DEU_testStop1Line-1_1_T-1.xml" # use this scenario as the other one has max. velocity of 5 m/s in goal region
        config_path = Path(__file__).parents[1] / "configurations/DEU_testStopLine-1_1_T-1.yaml"
        run_rpi(config_path, scenario, 150)

    def test_traffic_lights(self):
        scenario = "DEU_testTrafficLights-1_1_T-1.pb"
        config_path = Path(__file__).parents[1] / "configurations/DEU_testTrafficLights-1_1_T-1.yaml"
        run_rpi(config_path, scenario, 100)

    def test_interstate(self):
        scenario = "ZAM_3lanes-1_1_T-1.pb"
        config_path = Path(__file__).parents[1] / "configurations/ZAM_3lanes-1_1_T-1.yaml"
        run_rpi(config_path, scenario)

    def test_plan_one_step_hlp(self):
        scenario_name = "ZAM_3lanes-1_1_T-1.pb"
        config_path = Path(__file__).parents[1] / "configurations/ZAM_3lanes-1_1_T-1.yaml"
        path_scenarios = Path(__file__).parents[1] / "example_scenarios"
        path_scenario = os.path.join(path_scenarios, scenario_name[:-9], scenario_name)
        scenario, planning_problem, _ = \
            load_scenario_and_planning_problem(path_scenario)
        rpi = HighLevelPlanner(config_path=config_path, scenario=scenario, planning_problem=planning_problem)
        traj = rpi.plan(rpi.planner.config.scenario, rpi.planner.config.planning_problem)
        assert traj is not None

    def test_plan_one_step_set_based(self):
        scenario_name = "ZAM_Augmentation-1_1_S-1.xml"
        config_path = Path(__file__).parents[1] / "configurations/ZAM_Augmentation-1_1_S-1.yaml"
        path_scenarios = Path(__file__).parents[1] / "example_scenarios"
        path_scenario = os.path.join(path_scenarios, scenario_name)
        scenario, planning_problem, _ = \
            load_scenario_and_planning_problem(path_scenario)
        rpi = HighLevelPlanner(config_path=config_path, scenario=scenario, planning_problem=planning_problem)
        traj = rpi.plan(rpi.planner.config.scenario, rpi.planner.config.planning_problem)
        assert traj is not None

