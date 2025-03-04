import logging
from typing import List, Union

import numpy

from commonroad_route_planner.reference_path import ReferencePath
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from numpy import ndarray


class CloseToStopLine:
    """
    Class to determine if a vehicle is close to a stop line.
    """

    def __init__(self, ref_path: ReferencePath):
        self._lanelets_route: List[int] = ref_path.lanelet_ids
        self._lanelet_network: LaneletNetwork = ref_path.lanelet_network
        self._last_occ_route_lanelet: Lanelet = ref_path.lanelet_network.find_lanelet_by_id(ref_path.lanelet_ids[0])
        self._clcs = CurvilinearCoordinateSystem(ref_path.reference_path)
        self.logger = logging.getLogger("Stop_Line_LOGGER")

    def at_stop_line(self, position: ndarray) -> Union[Lanelet, None]:
        """
        Evaluates if a vehicle is close to stop line (with stop sign)

        :param position: position of interest
        :returns boolean indicating satisfaction
        """

        # check on same lanelet
        if not self._last_occ_route_lanelet.contains_points(
                numpy.array([[position[0], position[1]], [position[0], position[1]]])):
            lanelet = self.get_occupied_lanelet_on_route(position)
            if lanelet is not None:
                self._last_occ_route_lanelet = lanelet
            else:
                return None

        # return lanelet with stop line closer than 50m
        index = self._lanelets_route.index(self._last_occ_route_lanelet.lanelet_id)
        position_l = self._clcs.convert_to_curvilinear_coords(position[0], position[1])[0]
        while index < len(self._lanelets_route) and self.distance_to_end_of_lanelet(position_l, index) < 50.0:
            if self._lanelet_network.find_lanelet_by_id(self._lanelets_route[index]).stop_line:
                return self._lanelet_network.find_lanelet_by_id(self._lanelets_route[index])
            else:
                index += 1
        return None

    def get_occupied_lanelet_on_route(self, position: ndarray) -> Union[Lanelet, None]:
        """
        Traverses route and returns occupied lanelet

        @param position: 2D-position (x,y)
        """

        # find occupied lanelets
        occ = []
        for lanelet in self._lanelet_network.lanelets:
            if lanelet.contains_points(numpy.array(position)):
                occ.append(lanelet)
                if self._lanelets_route.count(lanelet.lanelet_id) > 0:
                    return lanelet

        # checks occupied lanelets for complience with route
        for lanelet in occ:
            left = lanelet.adj_left
            right = lanelet.adj_right
            while left is not None or right is not None:
                if left is not None:
                    if self._lanelets_route.count(left) > 0:
                        return self._lanelet_network.find_lanelet_by_id(left)
                    else:
                        left = self._lanelet_network.find_lanelet_by_id(left).adj_left
                if right is not None:
                    if self._lanelets_route.count(right) > 0:
                        return self._lanelet_network.find_lanelet_by_id(right)
                    else:
                        right = self._lanelet_network.find_lanelet_by_id(right).adj_right
        self.logger.warning("Not on route, couldn't find lanelet on route next to current position")
        return None

    def distance_to_end_of_lanelet(self, position_l: float, index: int):
        """
        Measures longitudinal distance to end of given lanelet.

        @param position_l: 1D longitudinal position on reference path
        @param index: Lanelet Id
        """

        try:
            lanelet = self._lanelet_network.find_lanelet_by_id(self._lanelets_route[index])
            end_of_lanelet_coords = lanelet.center_vertices[-1]
        except:
            lanelet = self._lanelet_network.find_lanelet_by_id(self._lanelets_route[index])
            end_of_lanelet_coords = lanelet.center_vertices[-1]
        end_of_lanelet_l = self._clcs.convert_to_curvilinear_coords(end_of_lanelet_coords[0], end_of_lanelet_coords[1])[0]
        return end_of_lanelet_l - position_l
