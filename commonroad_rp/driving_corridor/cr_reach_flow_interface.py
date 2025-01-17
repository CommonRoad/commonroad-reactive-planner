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
        self._velocity_constraints = [0,0]
        return self._velocity_constraints

    def generate_trajectories(self, x_0_lon: np.ndarray, x_0_lat: np.ndarray, low: float, up: float,
                              time_samples: set, num_samples: int, horizon: float, dt: float)\
            ->List[TrajectorySample]:
        """
        Implements trajectory generation method for sampling trajectories within dynamic driving corridor
        """
        if self._corridor is None:
            raise AttributeError("<ReachFlowCorridor>: Please set a driving corridor.")

        list_trajectories = list()

        for t in time_samples:
            for v in set(np.linspace(low, up, num_samples)):
                trajectory_long = QuarticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lon), x_d=np.array([v, 0]))
                if trajectory_long.coeffs is not None:
                    trajectory_lat = QuinticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lat), x_d=np.array([v, 0.0, 0.0]))
                    if trajectory_lat.coeffs is not None:
                        trajectory_sample = TrajectorySample(horizon, dt, trajectory_long, trajectory_lat)
                        list_trajectories.append(trajectory_sample)

        return list_trajectories

