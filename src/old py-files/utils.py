import numpy as np
import scipy.interpolate as si
from typing import List

from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.scenario.trajectory import Trajectory
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline


# TODO: test this function. There seems to be a bug! Currently not included in trajectory_tree.py
def lanelet_passed(lanelet_network: LaneletNetwork, current_lanelet_id: int, desired_lanelet_id: int) -> bool:
    """
    Determine if a desired lanelet id is already passed and cannot be reached on the current lanelet network without
    turning. This is an end criterion for a trajectory node, if the
    desired lanelet id contains the goal region
    :param current_lanelet_id:
    :param desired_lanelet_id:
    :return: True, if desired lanelet is passed, else False.
    """
    if current_lanelet_id == desired_lanelet_id:
        return True

    current_lane = lanelet_network.find_lanelet_by_id(current_lanelet_id)
    for predecessor in current_lane.predecessor:
        if lanelet_passed(lanelet_network, predecessor, desired_lanelet_id):
            return True
    # create a new instance of current lane to change it during the while loop
    lane = lanelet_network.find_lanelet_by_id(current_lanelet_id)
    while True:
        if lane.adj_right is None:
            break
        if lane.adj_right == desired_lanelet_id:
            return False
        right_lane = lanelet_network.find_lanelet_by_id(lane.adj_right)
        for predecessor in right_lane.predecessor:
            if lanelet_passed(lanelet_network, predecessor, desired_lanelet_id):
                return True
        lane = right_lane

    lane = lanelet_network.find_lanelet_by_id(current_lanelet_id)
    while True:
        if lane.adj_left is None:
            break
        if lane.adj_left == desired_lanelet_id:
            return False
        left_lane = lanelet_network.find_lanelet_by_id(lane.adj_left)
        for predecessor in left_lane.predecessor:
            if lanelet_passed(lanelet_network, predecessor, desired_lanelet_id):
                return True
        lane = left_lane

    return False


def end_lane_id(lanelet_network: LaneletNetwork, trajectory_for_lane: [Trajectory, Lanelet]) -> int:
    """
    Determine lanelet id at which the given trajectory will end
    :param trajectory_for_lane: trajectory planned for a given lane and the corresponding lane
    :return: lanelet id
    """
    end_position = np.array([trajectory_for_lane[0].final_state.position])
    end_lanelet_ids = lanelet_network.find_lanelet_by_position(end_position)
    for lanelet_id in end_lanelet_ids[0]:
        # TODO: calculate if lanelet belongs in lane more accurately
        if str(lanelet_id) in str(trajectory_for_lane[1].lanelet_id):
            return lanelet_id
    return -1


class LaneCoordinateSystem:
    """Coordinate system created from lane center vertices. Besides the coordinate system itself, the lane and the
    original lanelet ids are stored"""
    def __init__(self, coordinate_system, lane, lanelet_ids):
        self.coordinate_system = coordinate_system
        self.lane = lane
        self.lanelet_ids = lanelet_ids

    @classmethod
    def create_from_raw_lane(cls, lane: Lanelet, lanelet_ids: List[int]):
        smoothed_lane = cls.smooth_lane_with_b_splines(lane)

        return LaneCoordinateSystem(create_coordinate_system_from_polyline(smoothed_lane),
                                    lane, lanelet_ids)

    @staticmethod
    def smooth_lane_with_b_splines(lane: Lanelet):
        """smoothing center lane with b-splines from scipy"""
        x = lane.center_vertices[:, 0]
        y = lane.center_vertices[:, 1]

        t = range(len(lane.center_vertices))
        ipl_t = np.linspace(0.0, len(lane.center_vertices) - 1, 1000)

        k_x = 5 if len(x) > 5 else len(x) - 1
        k_y = 5 if len(y) > 5 else len(y) - 1
        x_tup = si.splrep(t, x, k=k_x)
        y_tup = si.splrep(t, y, k=k_y)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        x_i = si.splev(ipl_t, x_list)
        y_i = si.splev(ipl_t, y_list)
        return np.stack((x_i, y_i), axis=-1)


def create_coordinate_systems_for_lanelet_id(lanelet_network: LaneletNetwork,
                                             lanelet_id: int, max_iter=20) -> List[LaneCoordinateSystem]:
    """
    Creates a list of curvilinear coordinate systems with one coordinate system for every possible lane starting
    from a given lanelet. The reference curve are the center vertices of the lane
    :param lanelet_network: lanelet network to find succeeding lanelets
    :param lanelet_id: initial reference lanelet
    :param max_iter: maximum iterator for succeeding lanelets
    :return: list of curvilinear coordinate system for a given lane along with the corresponding lane.
    """
    init_lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
    lanes, merged_lanelet_list = init_lanelet.all_lanelets_by_merging_successors_from_lanelet(init_lanelet,
                                                                                              lanelet_network,
                                                                                              max_iter=max_iter)
    coordinate_systems_for_lanes = []
    if not lanes:
        lanes.append(init_lanelet)
        merged_lanelet_list.append(lanelet_id)
    for lane, merged_lanelets in zip(lanes, [merged_lanelet_list]):
        lane_coordinate_system = LaneCoordinateSystem.create_from_raw_lane(lane, merged_lanelets)
        coordinate_systems_for_lanes.append(lane_coordinate_system)
    return coordinate_systems_for_lanes


def interpolate_lane_vertices(lane_vertices):
    """
    Creates an interpolation of a lane.
    :param lane_vertices: The vertices of the lane to be interpolated
    :return: A numpy array with the interpolated vertices
    """
    interpolated_vertices = [lane_vertices[0]]
    for i in range(len(lane_vertices) - 1):
        interpolated_vertices.append((lane_vertices[i] + lane_vertices[i + 1])/2)
        interpolated_vertices.append(lane_vertices[i+1])
    return np.array(interpolated_vertices)


def binary_search_indices(full_range):
    """
    Returns a list of the visited indices in the order they were visited.A recursive function.
    :param full_range: a list containing all the integer numbers of the set.
    :return: list of visited indices
    """
    full_range_list = list(full_range)
    if len(full_range_list) == 2:
        return []
    middle = (full_range_list[0] + full_range_list[-1]) // 2
    result = [middle]
    upper_range = range(middle, full_range_list[-1] + 1)
    lower_range = range(full_range_list[0], middle + 1)
    result = result + binary_search_indices(upper_range)
    result = result + binary_search_indices(lower_range)
    return result
