from abc import ABC, abstractmethod
from typing import Optional

from commonroad_rp.driving_corridor.corridor_selector import DrivingCorridorSelector

try:
    from commonroad_reach.data_structure.reach.driving_corridor import DrivingCorridor, ConnectedComponent
    from commonroad_reach.pycrreach import ReachNode, ReachPolygon
    cr_reach_installed = True
except ImportError:
    DrivingCorridor = None
    ReachPolygon = None
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
        self._params = ConnectedComponent()
        #define a new type for params

    def get_bounding_box_at_step(self):
        for list in self._graph:
            for node in list:
                self._params.p_lat_min = node.polygon_lat.p_min
                self._params.p_lat_max = node.polygon_lat.p_max
                self._params.p_lon_min = node.polygon_lon.p_min
                self._params.p_lon_max = node.polygon_lon.p_max
                self._params.v_lat_min = node.polygon_lat.v_min
                self._params.v_lat_max = node.polygon_lat.v_max
                self._params.v_lon_min = node.polygon_lon.v_min
                self._params.v_lon_max = node.polygon_lon.v_max
        return self._params
