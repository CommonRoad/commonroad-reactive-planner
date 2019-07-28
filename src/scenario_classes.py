from commonroad.common.file_reader import CommonRoadFileReader
import numpy as np
from commonroad.common.util import Interval
import xml.etree.ElementTree as ET
from commonroad.scenario.obstacle import ObstacleType, StaticObstacle
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.trajectory import State


class PlanningProblem():
    """
    Class that represents the planning problem
    """
    def __init__(self, scenario_path):
        _, self.planning_problem_set = CommonRoadFileReader(scenario_path).open()
        self.scenario_path = scenario_path


class Scenario(object):
    """
    Class that represents the scenario
    """
    def __init__(self, scenario_path):
        self.scenario_set, _ = CommonRoadFileReader(scenario_path).open()
        self.scenario_path = scenario_path

        
    def _boundary_not_intersecting(self, points, own_ID, total_IDs):
        """
        Helper to check if boundary object is not intersecting/blocking any lanelet
        :param points:      Array of a point and its predecessor describing a boundary segment
        :param own_ID:      Id of the lanelet corresponding to the boundary
        :param total_IDs:   List of all lanelet Ids to check for intersection
        :return:            Boolean
        """

        # number of sections, the distance between the points[0] and points[1] is split
        # the higher the more precise
        n = 50

        total_IDs.remove(own_ID)

        # create list of n-1 points between points[0] and points[1]
        delta_x = points[1][0]-points[0][0]
        delta_y = points[1][1]-points[0][1]
        points = np.array([[points[0][0]+1/n*delta_x, points[0][1]+1/n*delta_y]])
        for i in range(2,n):
            points = np.append(points, np.array([[points[0][0]+i/n*delta_x, points[0][1]+i/n*delta_y]]), axis= 0)

        # checks whether a point between points[0] and points[1] is on other lanelet
        for i in total_IDs:
            if True in self.scenario_set.lanelet_network.find_lanelet_by_id(i).contains_points(points):
                return False
        return True

    def add_obstacles_at_lanelet_edges(self):
        """
        Adds Obstacles to the most outer boundaries of the lanelet network to prevent the planner from leaving the
        lanelet network into empty regions
        :return: pass
        """

        
        # find all lanelet IDs
        tree = ET.parse(self.scenario_path)
        root = tree.getroot()
        IDs = []
        for child in root:
            if child.tag == "lanelet":
                IDs.append(int(child.attrib["id"]))

        # add obstacles
        e = 1e-6  # avoid numerical errors
        for i in IDs:
            if self.scenario_set.lanelet_network.find_lanelet_by_id(i).adj_left is None:
                old = self.scenario_set.lanelet_network.find_lanelet_by_id(i).left_vertices[0];
                for element in self.scenario_set.lanelet_network.find_lanelet_by_id(i).left_vertices:
                    delta_x = element[0] - old[0]
                    delta_y = element[1] - old[1]
                    ang = np.arctan(delta_y / (delta_x + e))
                    len_obj = np.sqrt(delta_x ** 2 + delta_y ** 2)
                    if delta_x >= 0:
                        x_obj = element[0] - 0.5 * len_obj * np.cos(np.arctan(np.abs(delta_y / (delta_x + e))))
                        y_obj = element[1] - 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x + e)))
                    else:
                        x_obj = element[0] + 0.5 * len_obj * np.cos(np.arctan(np.abs(delta_y / (delta_x + e))))
                        y_obj = element[1] + 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x + e)))
                    obj = Rectangle(length=len_obj, width=0.05)
                    state = State(position=np.array([x_obj, y_obj]), orientation=ang, velocity=0,
                                  time_step=Interval(0, 30))
                    o_type = ObstacleType
                    obj = StaticObstacle(obstacle_shape=obj, initial_state=state, obstacle_type=o_type.ROAD_BOUNDARY,
                                         obstacle_id=self.scenario_set.generate_object_id())
                    if self._boundary_not_intersecting(np.array([element, old]), i, IDs[:]):
                        self.scenario_set.add_objects(obj)
                    old = element
            if self.scenario_set.lanelet_network.find_lanelet_by_id(i).adj_right is None:
                old = self.scenario_set.lanelet_network.find_lanelet_by_id(i).right_vertices[0];
                for element in self.scenario_set.lanelet_network.find_lanelet_by_id(i).right_vertices:
                    delta_x = element[0] - old[0]
                    delta_y = element[1] - old[1]
                    ang = np.arctan(delta_y / (delta_x + e))
                    len_obj = np.sqrt(delta_x ** 2 + delta_y ** 2)
                    if delta_x >= 0:
                        x_obj = element[0] - 0.5 * len_obj * np.cos(np.arctan(np.abs(delta_y / (delta_x + e))))
                        y_obj = element[1] - 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x + e)))
                    else:
                        x_obj = element[0] + 0.5 * len_obj * np.cos(np.arctan(np.abs(delta_y / (delta_x + e))))
                        y_obj = element[1] + 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x + e)))
                    # y_obj = element[1] - 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x+e)))
                    obj = Rectangle(length=len_obj, width=-0.05)
                    state = State(position=np.array([x_obj, y_obj]), orientation=ang, velocity=0,
                                  time_step=Interval(0, 30))
                    o_type = ObstacleType
                    obj = StaticObstacle(obstacle_shape=obj, initial_state=state, obstacle_type=o_type.ROAD_BOUNDARY,
                                         obstacle_id=self.scenario_set.generate_object_id())
                    if self._boundary_not_intersecting(np.array([element, old]), i, IDs[:]):
                        self.scenario_set.add_objects(obj)
                    old = element
        pass


