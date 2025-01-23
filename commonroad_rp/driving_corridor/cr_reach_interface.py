from abc import ABC, abstractmethod
from typing import Optional, List

from commonroad_rp.driving_corridor.corridor_selector import DrivingCorridorSelector
from commonroad_rp.driving_corridor.parameters import Parameters

try:
    from commonroad_reach.data_structure.reach.driving_corridor import DrivingCorridor
    from commonroad_reach.pycrreach import ReachNode, ReachPolygon
    import commonroad_reach.utility.reach_operation as util_reach_operation
    cr_reach_installed = True
except ImportError:
    DrivingCorridor = None
    ReachPolygon = None
    util_reach_operation = None
    cr_reach_installed = False


class ReachableSetCorridor(DrivingCorridorSelector):
    """
    Wrapper class for CommonRoad-Reach - DrivingCorridor class
    """
    def __init__(self, corridor: DrivingCorridor):
        super().__init__(corridor)
        if not cr_reach_installed:
            raise ImportError("<ReachableSetCorridor>: Please install CommonRoad-Reach to use driving corridor!")
        self._corridor: Optional[DrivingCorridor] = corridor
        self._graph = DrivingCorridor.reach_nodes(corridor)
        self._params : List(Parameters) = list()
        self._velocity_constraints = dict()

    def get_bounding_box_at_step(self):
        params = Parameters()
        for step in self._graph:
            index = DrivingCorridor.reach_nodes_at_step(self._corridor, step)
            for node in index:
                params.p_lat_min = node.polygon_lat.p_min
                params.p_lat_max = node.polygon_lat.p_max
                params.p_lon_min = node.polygon_lon.p_min
                params.p_lon_max = node.polygon_lon.p_max
                params.v_lat_min = node.polygon_lat.v_min
                params.v_lat_max = node.polygon_lat.v_max
                params.v_lon_min = node.polygon_lon.v_min
                params.v_lon_max = node.polygon_lon.v_max
            self._params.append(params)
        return self._params

    def set_velocity_constraints(self):
        for time_idx in self.get_time_step():
            reach_node = self._corridor[time_idx]
            velocity_interval = self.get_lon_velocity_interval(reach_node)
            self._velocity_constraints[time_idx] = [velocity_interval[0], velocity_interval[1]]
        return self._velocity_constraints

    def get_lon_velocity_interval(self, reach_node):
        velocity_interval = util_reach_operation.lon_velocity_interval_connected_set(reach_node)
        return velocity_interval

    def get_time_step(self):
        return list(self._corridor.keys())

    def get_overlapping_nodes_with_lon_pos(self, time_step: int, lon_pos: float):
        reach_node = self._corridor[time_step]
        overlap_nodes = util_reach_operation.determine_overlapping_nodes_with_lon_pos(reach_node, lon_pos)
        return overlap_nodes

    def get_connected_components(self, overlap_nodes: list()):
        lat_connected_sets = util_reach_operation.determine_connected_components(overlap_nodes)
        return lat_connected_sets

    def get_lat_interval(self, reach_node):
        lat_interval = util_reach_operation.lat_interval_connected_set(reach_node)
        return lat_interval