from typing import List, Optional, Dict, Iterable, Tuple, Set

import numpy as np

from commonroad_rp.utility.config import ReactivePlannerConfiguration
from ..base.base_sampling_space import SamplingSpace
from .sampling_domain import (
    TimeSampling
)

try:
    from commonroad_reach.data_structure.reach.driving_corridor import DrivingCorridor
    import commonroad_reach.utility.reach_operation as util_reach_operation
    cr_reach_installed = True
except ImportError:
    DrivingCorridor = None
    util_reach_operation = None
    cr_reach_installed = False
    pass


# TODO Handle case if no overlapping reach sets with lon end position
# TODO do we require the possibility to add additional intervals (e.g., for v_min and v_max?)

class CorridorSampling(SamplingSpace):
    """
    Class representing an adaptive sampling space which draws samples from the reachable sets of a precomputed
    collision-free driving corridor.
    NOTE: CommonRoad-Reach is required (https://commonroad.in.tum.de/tools/commonroad-reach)
    """
    def __init__(self, config: ReactivePlannerConfiguration):
        if not cr_reach_installed:
            raise ImportError("CommonRoad-Reach not found.")

        super(CorridorSampling, self).__init__(
            num_sampling_levels=config.sampling.num_sampling_levels,
            dt=config.planning.dt,
            horizon=config.planning.dt * config.planning.time_steps_computation
        )

        # initialize and precompute samples in t domain
        self.samples_t = TimeSampling(config.sampling.t_min,
                                      self.horizon,
                                      self._num_sampling_levels,
                                      self.dt)

        # corridor as Dict over time steps
        self._corridor: Optional[DrivingCorridor] = None

        # longitudinal velocity intervals from corridor
        self._dict_lon_velocity_intervals: Dict = dict()

        # longitudinal position intervals from corridor
        self._dict_lon_position_intervals: Dict = dict()

        # number of samples per level
        self._dict_level_to_num_samples: Dict[int, int] = dict()
        self.set_dict_number_of_samples()

        # external sampling bounds
        self._v_bounds: Tuple[float, float] = (-np.inf, np.inf)
        self._s_bounds: Tuple[float, float] = (-np.inf, np.inf)

    def set_corridor(self, corridor: DrivingCorridor):
        """Interface method to update corridor constraint dict during re-planning"""
        self._corridor = corridor
        for time_idx, connected_reach_set in self._corridor.items():
            position_interval = util_reach_operation.lon_interval_connected_set(connected_reach_set)
            velocity_interval = util_reach_operation.lon_velocity_interval_connected_set(connected_reach_set)
            self._dict_lon_velocity_intervals[time_idx] = [velocity_interval[0], velocity_interval[1]]
            self._dict_lon_position_intervals[time_idx] = [position_interval[0], position_interval[1]]

    def set_t_sampling(self, t_min):
        """Set samples in t domain of sampling space"""
        self.samples_t = TimeSampling(t_min, self.horizon, self._num_sampling_levels, self.dt)

    def set_d_sampling(self, d_min, d_max):
        """Not implemented, d values are obtained from corridor"""
        pass

    def set_v_sampling(self, v_min, v_max):
        """Set external bounds for v_min and v_max"""
        self._v_bounds = (v_min, v_max)

    def set_s_sampling(self, s_min, s_max):
        """Set external bounds for s_min and s_max"""
        self._s_bounds = (s_min, s_max)

    def set_dict_number_of_samples(self, n_min: int = 3, dict_level_to_num_samples: dict = None):
        """
        store number of samples per sampling level
        :param n_min: minimum number of samples in lowest level (default 3)
        :param dict_level_to_num_samples: directly set dictionary with numbers of samples per level
        """
        if dict_level_to_num_samples is not None:
            for level in range(self.num_sampling_levels):
                assert level in dict_level_to_num_samples.keys(), f"input does not contain sampling level:{level}"
        else:
            n = n_min
            for i in range(self.num_sampling_levels):
                self._dict_level_to_num_samples[i] = n
                n = (n * 2) - 1

    def _get_time_samples(self, level_sampling: int, **kwargs) -> Iterable[float]:
        """Returns time samples for the corridor sampling space"""
        return self.samples_t.samples_at_level(level_sampling)

    def _get_lon_samples(self, level_sampling: int, longitudinal_mode: str, **kwargs) -> Iterable[float]:
        """Returns the longitudinal samples for the corridor sampling space"""
        if self._corridor is None:
            raise AttributeError("Corridor not set.")

        # get corridor time step
        t_sample = kwargs["t_sample"]
        time_step = self._get_corridor_time_step(t_sample)

        # num samples for level
        num_samples = self._dict_level_to_num_samples[level_sampling]

        # get longitudinal constraints
        if longitudinal_mode == "velocity_keeping":
            low = self._dict_lon_velocity_intervals[time_step][0]
            up = self._dict_lon_velocity_intervals[time_step][1]
            return np.linspace(low, up, num_samples)
        elif longitudinal_mode == "stopping":
            low = self._dict_lon_position_intervals[time_step][0]
            up = self._dict_lon_position_intervals[time_step][1]
            return np.linspace(low, up, num_samples)
        else:
            raise AttributeError(f"<CorridorSampling>: specified longitudinal mode {longitudinal_mode} is"
                                 f"invalid.")

    def _get_lat_samples(self, level_sampling: int, **kwargs) -> Iterable[float]:
        """Return the lateral samples for the corridor sampling space"""
        traj_long = kwargs["lon_trajectory"]
        t = kwargs["t_sample"]
        time_step = self._get_corridor_time_step(t)
        end_pos_lon = traj_long.calc_position(t, t ** 2, t ** 3, t ** 4, t ** 5)

        # num samples for level
        num_samples = self._dict_level_to_num_samples[level_sampling]

        # get corridor nodes containing longitudinal position
        reachsets_overlap = util_reach_operation.determine_overlapping_nodes_with_lon_pos(
            self._corridor[time_step],
            end_pos_lon
        )

        # TODO Find cause why this happens?
        if len(reachsets_overlap) == 0:
            return set()

        # get connected nodes in position domain among those
        lat_connected_sets = util_reach_operation.determine_connected_components(reachsets_overlap)

        # initialize lat sample set
        d_samples = set()

        for lat_con_set in lat_connected_sets:
            lat_interval = util_reach_operation.lat_interval_connected_set(lat_con_set)

            if lat_interval[0] < 0 < lat_interval[1]:
                # include sample on reference path
                d_samples_set: Set[float] = set(np.linspace(lat_interval[0], lat_interval[1], num_samples)).union({0})
            else:
                d_samples_set: Set[float] = set(np.linspace(lat_interval[0], lat_interval[1], num_samples))

            d_samples.update(d_samples_set)
        return d_samples

    def _get_corridor_time_step(self, t: float) -> float:
        """Returns corresponding time step in corridor (if initial corridor time step is non-zero)"""
        return round(t / self.dt) + min(self._corridor.keys())
