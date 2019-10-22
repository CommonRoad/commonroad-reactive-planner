import unittest
from commonroad.common.file_reader import CommonRoadFileReader
from removal.trajectory_tree import TrajectoryTree


class TestEndLaneId(unittest.TestCase):

    def setUp(self):
            scenario, planning_task = CommonRoadFileReader("DEU_A9-1_1_T-1.xml").open()
            self.scenario = scenario
            self.planning_problem = planning_task.planning_problem_dict[
                list(planning_task.planning_problem_dict.keys())[0]]
            self.search_tree = TrajectoryTree(self.planning_problem, scenario)
            self.goal_region = self.planning_problem.goal
            self.trajectory = [self.search_tree.frontier[0].trajectory, self.search_tree.frontier[0].lane]
            self.optimal_trajectory = self.search_tree.uniform_cost_search()

    def test_ends_not_on_lanelet(self):
        self.assertEqual(True, False)

    def test_long_id(self):
        self.assertEqual(True, False)


class TestReturnSolution(unittest.TestCase):
    def test_returns_valid_solution(self):
        pass


class TestDesiredVelocity(unittest.TestCase):
    def setUp(self):
        pass

    def test_no_velocity_interval(self):
        pass


class TestInitialiseFrontier(unittest.TestCase):
    pass


class TestUniformCostSearch(unittest.TestCase):
    pass


class TestPlanOptimalTrajectory(unittest.TestCase):
    pass


class TestCreateCoordinateSystemsForLaneletId(unittest.TestCase):
    pass


if __name__ == '__main__':
    unittest.main()