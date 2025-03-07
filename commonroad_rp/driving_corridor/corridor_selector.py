from abc import ABC, abstractmethod
from typing import Optional, Union, Dict, List

import commonroad_rp.driving_corridor as dc

try:
    from commonroad_reach.data_structure.reach.driving_corridor import DrivingCorridor
    from cr_reach_flow.cr_reach_flow_core.driving_corridor import DynamicDrivingCorridor
    cr_reach_installed = True
    cr_reach_flow_installed = True
except ImportError:
    DrivingCorridor = None
    DynamicDrivingCorridor = None
    cr_reach_installed = False
    cr_reach_flow_installed = False

class DrivingCorridorSelector(ABC):
    """
    Abstract class for selecting Driving Corridors.
    """
    def __init__(self, corridor: Union[DrivingCorridor, DynamicDrivingCorridor]):
        if not cr_reach_installed:
            raise ImportError("<ReachableSetCorridor>: Please install CommonRoad-Reach to use driving corridor!")
        if not cr_reach_flow_installed:
            raise ImportError("<ReachFlowCorridor>: Please install CommonRoad-Reach-Flow to use driving corridor!")
        self._corridor: Union[DrivingCorridor, DynamicDrivingCorridor] = corridor
        self._graph = None
        self._params : List = list()
        self._velocity_constraints : Dict = dict()

    @abstractmethod
    def get_bounding_box_at_step(self):
        pass

    @abstractmethod
    def set_velocity_constraints(self):
        pass

    @abstractmethod
    def get_lon_velocity_interval(self, reach_node):
        pass

    @abstractmethod
    def get_time_step(self):
        pass

    @abstractmethod
    def get_overlapping_nodes_with_lon_pos(self, time_step: int, lon_pos: float):
        pass

    @abstractmethod
    def get_connected_components(self, overlap_nodes: list()):
        pass

    @abstractmethod
    def get_lat_interval(self, reach_node):
        pass

    @abstractmethod
    def get_drivable_area(self, time_step: int):
        pass

    @staticmethod
    def select_driving_corridor(corridor: Union[DrivingCorridor, DynamicDrivingCorridor]):
        if isinstance(corridor, DrivingCorridor):
            return dc.cr_reach_interface.ReachableSetCorridor(corridor)
        elif isinstance(corridor, DynamicDrivingCorridor):
            return dc.cr_reach_flow_interface.ReachFlowCorridor(corridor)
        else:
            ValueError("Invalid driving corridor specified")

