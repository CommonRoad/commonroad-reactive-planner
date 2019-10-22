import unittest
from commonroad.common.file_reader import CommonRoadFileReader
from removal.utils import lanelet_passed, end_lane_id
from removal.trajectory_tree import TrajectoryTree


class TestLaneletPassed(unittest.TestCase):

    def setUp(self):
        pass

    def test_lanelet_passed(self):
        self.scenario, _ = CommonRoadFileReader('USA_US101-2_1_T-1.xml').open()
        results = [[True, True, False, True, False, True, False, True, False, True],
                   [False, True, False, False, False, False, False, False, False, False],
                   [False, True, True, True, False, True, False, True, False, True],
                   [False, False, False, True, False, False, False, False, False, False],
                   [False, True, False, True, True, True, False, True, False, True],
                   [False, False, False, False, False, True, False, False, False, False],
                   [False, True, False, True, False, True, True, True, False, True],
                   [False, False, False, False, False, False, False, True, False, False],
                   [False, True, False, True, False, True, False, True, True, True],
                   [False, False, False, False, False, False, False, False, False, True]]
        for des_idx, desired_lanelet in enumerate(self.scenario.lanelet_network.lanelets):
            for cur_idx, current_lanelet in enumerate(self.scenario.lanelet_network.lanelets):
                lanelet_passed(self.scenario.lanelet_network, current_lanelet.lanelet_id, desired_lanelet.lanelet_id)
                self.assertEqual(lanelet_passed(self.scenario.lanelet_network, current_lanelet.lanelet_id,
                                                desired_lanelet.lanelet_id), results[des_idx][cur_idx])


    def test_ends_on_lanelet(self):
        scenario, planning_task = CommonRoadFileReader("DEU_A9-1_1_T-1.xml").open()
        planning_problem = planning_task.planning_problem_dict[
            list(planning_task.planning_problem_dict.keys())[0]]
        search_tree = TrajectoryTree(planning_problem, scenario)
        goal_region = planning_problem.goal
        trajectory = [search_tree.frontier[0].trajectory, search_tree.frontier[0].lane]
        optimal_trajectory = search_tree.uniform_cost_search()
        self.assertEqual(end_lane_id(scenario.lanelet_network, trajectory), 48969)

if __name__ == '__main__':
    unittest.main()
