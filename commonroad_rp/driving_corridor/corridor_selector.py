from abc import ABC, abstractmethod
from typing import Optional, Union, Dict, List, Tuple

import commonroad_rp.driving_corridor as dc
from commonroad_rp.driving_corridor.parameters import Parameters

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
        """
        Create a new interface for driving corridor wrapper classes.

        :param corridor: Driving corridor from either CommonRoad-Reach or CommonRoad-Reach-Flow.
        """
        if not cr_reach_installed:
            raise ImportError("<ReachableSetCorridor>: Please install CommonRoad-Reach to use driving corridor!")
        if not cr_reach_flow_installed:
            raise ImportError("<ReachFlowCorridor>: Please install CommonRoad-Reach-Flow to use driving corridor!")
        self._corridor: Union[DrivingCorridor, DynamicDrivingCorridor] = corridor
        self._graph = None
        self._params : List = list()
        self._velocity_constraints : Dict = dict()

    @abstractmethod
    def get_bounding_box_at_step(self) -> List[Parameters]:
        """
        Sets bounding box parameters from driving corridor graph.

        :return: List of Parameters.
        """
        pass

    @abstractmethod
    def set_velocity_constraints(self) -> Dict:
        """
        Returns velocity constraints for driving corridors.

        :return: Dictionary with minimum and maximum velocity constraints.
        """
        pass

    @abstractmethod
    def get_lon_velocity_interval(self, reach_node) -> Tuple[float, float]:
        """
        Returns the longitudinal velocity intervals from graph nodes.

        :param reach_node: Graph nodes at a time step.
        :return: Tuple with minimum and maximum longitudinal velocity.
        """
        pass

    @abstractmethod
    def get_time_step(self) -> List[int]:
        """
        Returns the available time steps in the driving corridor.

        :return: List of available time steps.
        """
        pass

    @abstractmethod
    def get_overlapping_nodes_with_lon_pos(self, time_step: int, lon_pos: float) -> List:
        """
        Determines and returns the reach nodes which contain a given longitudinal position.

        :param time_step: the time step.
        :param lon_pos: Longitudinal position.
        :return: List with reach nodes.
        """
        pass

    @abstractmethod
    def get_connected_components(self, overlap_nodes: list()) -> List:
        """
        Determines and returns connected sets in the position domain.

        :param overlap_nodes: List of overlapping reach nodes.
        :return: List with connected sets.
        """
        pass

    @abstractmethod
    def get_lat_interval(self, reach_node) -> Tuple[float, float]:
        """
        Returns the lateral position intervals from graph nodes.

        :param reach_node: Graph nodes at a time step.
        :return: Tuple with minimum and maximum lateral positions.
        """
        pass

    @abstractmethod
    def get_drivable_area(self, time_step: int) -> List[Tuple]:
        """
        Returns the drivable area at a given time step.

        :param time_step: The time step.
        :return: Tuple with drivable area as (min_lon, min_lat, max_lon, max_lat).
        """
        pass

    @staticmethod
    def select_driving_corridor(corridor: Union[DrivingCorridor, DynamicDrivingCorridor]):
        """
        Assigns appropriate Wrapper class based on the Driving Corridor.

        :param corridor: Driving corridor from either CommonRoad-Reach or CommonRoad-Reach-Flow.
        """
        if isinstance(corridor, DrivingCorridor):
            return dc.cr_reach_interface.ReachableSetCorridor(corridor)
        elif isinstance(corridor, DynamicDrivingCorridor):
            return dc.cr_reach_flow_interface.ReachFlowCorridor(corridor)
        else:
            ValueError("Invalid driving corridor specified")

