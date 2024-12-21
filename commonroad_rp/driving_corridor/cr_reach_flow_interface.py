from abc import ABC, abstractmethod
from typing import Optional

from commonroad_rp.driving_corridor.corridor_selector import DrivingCorridorSelector

try:
    from cr_reach_flow.cr_reach_flow_core.driving_corridor import DynamicDrivingCorridor
    from cr_reach_flow.cr_reach_flow_core.layers.propagation import PointMassParameters
    cr_reach_flow_installed = True
except ImportError:
    DynamicDrivingCorridor = None
    PointMassParameters = None
    cr_reach_flow_installed = False


class ReachFlowCorridor(DrivingCorridorSelector):
    """
    Wrapper class for CommonRoad-Reach-Flow - DrivingCorridor class
    """

    def __init__(self, corridor: DynamicDrivingCorridor):
        super().__init__(corridor)
        if not cr_reach_flow_installed:
            raise ImportError("<ReachFlowCorridor>: Please install CommonRoad-Reach-Flow to use driving corridor!")
        self._corridor: Optional[DynamicDrivingCorridor] = corridor
        self._graph = DynamicDrivingCorridor.reach_graph
        self._params = PointMassParameters()

    def get_bounding_box_at_step(self):
        for step in range(self._graph.initial_step, self._graph.final_step + 1):
            for node in self._graph.get_nodes_at_step(step):
                self._params.a_lon_min = node.set.p_lon_min
                self._params.a_lon_max = node.set.p_lon_max
                self._params.a_lat_min = node.set.p_lat_min
                self._params.a_lat_max = node.set.p_lat_max
                self._params.v_lon_min = node.set.v_lon_min
                self._params.v_lon_max = node.set.v_lon_max
                self._params.v_lat_min = node.set.v_lat_min
                self._params.v_lat_max = node.set.v_lat_max
        return self._params

    def set_velocity_constraints(self):
        pass

