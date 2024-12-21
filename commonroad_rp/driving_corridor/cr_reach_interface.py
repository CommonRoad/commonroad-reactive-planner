from abc import ABC, abstractmethod
from typing import Optional

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
        self._params = Parameters()
        self._velocity_constraints = dict()

    def get_bounding_box_at_step(self):
        for index in self._graph:
            for node in index:
                self._params.a_lat_min = node.polygon_lat.p_min
                self._params.a_lat_max = node.polygon_lat.p_max
                self._params.a_lon_min = node.polygon_lon.p_min
                self._params.a_lon_max = node.polygon_lon.p_max
                self._params.v_lat_min = node.polygon_lat.v_min
                self._params.v_lat_max = node.polygon_lat.v_max
                self._params.v_lon_min = node.polygon_lon.v_min
                self._params.v_lon_max = node.polygon_lon.v_max
        return self._params

    def set_velocity_constraints(self):
        for time_idx, connected_reach_set in self._corridor.items():
            velocity_interval = util_reach_operation.lon_velocity_interval_connected_set(connected_reach_set)
            self._velocity_constraints[time_idx] = [velocity_interval[0], velocity_interval[1]]
        return self._velocity_constraints
