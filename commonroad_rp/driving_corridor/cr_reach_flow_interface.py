from abc import ABC, abstractmethod
from typing import Optional, List
import numpy as np

from commonroad_rp.driving_corridor.corridor_selector import DrivingCorridorSelector
from commonroad_rp.polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from commonroad_rp.trajectories import TrajectorySample

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
        self._graph = corridor.reach_graph
        self._params : List(PointMassParameters) = list()
        self._velocity_constraints = dict()

    def get_bounding_box_at_step(self):
        params = PointMassParameters()
        for step in range(self._graph.initial_step, self._graph.final_step + 1):
            for node in self._graph.get_nodes_at_step(step):
                params.a_lon_min = node.set.p_lon_min
                params.a_lon_max = node.set.p_lon_max
                params.a_lat_min = node.set.p_lat_min
                params.a_lat_max = node.set.p_lat_max
                params.v_lon_min = node.set.v_lon_min
                params.v_lon_max = node.set.v_lon_max
                params.v_lat_min = node.set.v_lat_min
                params.v_lat_max = node.set.v_lat_max
            self._params.append(params)
        return self._params

    def set_velocity_constraints(self):
        for time_idx, param in enumerate(self._params):
            velocity_interval = self.get_lon_velocity_interval(param)
            self._velocity_constraints[time_idx] = [velocity_interval[0], velocity_interval[1]]
        return self._velocity_constraints

    def get_lon_velocity_interval(self, connected_set):
        min_max_array = np.asarray([connected_set.v_lon_min, connected_set.v_lon_max])
        # get minimum and maximum value for the connected set
        min_connected_set = np.min(min_max_array)
        max_connected_set = np.max(min_max_array)
        return min_connected_set, max_connected_set

    def get_initial_step(self):
        return self._graph.initial_step

    def get_overlapping_nodes_with_lon_pos(self, time_step: int, lon_pos: float):
        overlap_nodes = list()
        param = self._params[time_step]
        if np.greater_equal(round(lon_pos * 10.0 ** 2), np.floor(param.a_lon_min * 10.0 ** 2)) and \
                np.greater_equal(np.ceil(param.a_lon_max * 10.0 ** 2), round(lon_pos * 10.0 ** 2)):
            overlap_nodes.append(param)
        return overlap_nodes

    def get_connected_components(self, overlap_nodes: list()):
        connected_components = list()
        overlap_nodes.sort(key=lambda node: (node.a_lat_min, node.a_lat_max))
        if len(overlap_nodes) == 1:
            connected_components.append(overlap_nodes[0])
            return connected_components

        prec = 1e-9
        added_idx = set()
        for idx in range(len(overlap_nodes)-1):
            if abs(overlap_nodes[idx].a_lat_max - overlap_nodes[idx + 1].a_lat_min) < prec :
                # to ensure each node is added once to connected components
                if idx not in added_idx:
                    connected_components.append(overlap_nodes[idx])
                    added_idx.add(idx)
                connected_components.append(overlap_nodes[idx + 1])
                added_idx.add(idx + 1)
        return connected_components


    def get_lat_interval(self, connected_set):
        min_max_array = np.asarray([connected_set.a_lat_min, connected_set.a_lat_max])
        # get minimum and maximum value for the connected set
        min_connected_set = np.min(min_max_array)
        max_connected_set = np.max(min_max_array)
        return min_connected_set, max_connected_set
