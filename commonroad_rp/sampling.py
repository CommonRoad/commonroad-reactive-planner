__author__ = "Gerald Würsching, Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "1.0"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Beta"

import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, Optional, List

from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from commonroad_rp.trajectories import TrajectorySample

try:
    from commonroad_reach.data_structure.reach.driving_corridor import DrivingCorridor
    import commonroad_reach.utility.reach_operation as util_reach_operation
    cr_reach_installed = True
except ImportError:
    DrivingCorridor = None
    util_reach_operation = None
    cr_reach_installed = False
    pass


class Sampling(ABC):
    """
    Class that represents the sampling parameters for planning
    """

    def __init__(self, low: float, up: float, num_sampling_levels: int):
        # Check validity of input
        assert np.greater_equal(up, low), '<Sampling>: Upper sampling bound is not greater than ' \
                                      'lower bound! up = {} , low = {}'.format(up, low)
        assert isinstance(num_sampling_levels, int) and num_sampling_levels > 0, \
            '<Sampling: number of samples must be positive integer>'

        self.low = low
        self.up = up
        self._n_samples = num_sampling_levels
        self._dict_level_to_sample_set: Dict[int, set] = dict()
        self._sample()

    @abstractmethod
    def _sample(self):
        """
        Abstract method to draw samples for a given range and sampling level
        """
        pass

    def samples_at_level(self, sampling_level: int = 0) -> set:
        """
        Obtain the sampling steps of a given sampling level
        :param sampling_level: The sampling stage to receive (>=0)
        :return: The set of sampling steps for the queried sampling stage
        """
        assert 0 <= sampling_level < self.num_sampling_levels, \
            '<Sampling>: Provided sampling level is incorrect! stage = {}'.format(sampling_level)
        return self._dict_level_to_sample_set[sampling_level]

    @property
    def num_sampling_levels(self) -> int:
        """
        Returns the maximum number of sampling stages
        :return: The maximum number of sampling stages
        """
        return self._n_samples


class VelocitySampling(Sampling):
    """
    Sampling steps object for the velocity domain
    """

    def __init__(self, low: float, up: float, num_sampling_levels: int):
        super(VelocitySampling, self).__init__(low, up, num_sampling_levels)

    def _sample(self):
        n = 3
        for i in range(self.num_sampling_levels):
            self._dict_level_to_sample_set[i] = set(np.linspace(self.low, self.up, n))
            n = (n * 2) - 1


class PositionSampling(Sampling):
    """
    Sampling steps object for the position domain (s or d-coordinate)
    """

    def __init__(self, low: float, up: float, num_sampling_levels: int):
        super(PositionSampling, self).__init__(low, up, num_sampling_levels)

    def _sample(self):
        n = 3
        for i in range(self.num_sampling_levels):
            self._dict_level_to_sample_set[i] = set(np.linspace(self.low, self.up, n))
            n = (n * 2) - 1


class TimeSampling(Sampling):
    """
    Sampling steps object for the time domain
    """

    def __init__(self, low: float, up: float, num_sampling_levels: int, dt: float):
        self.dT = dt
        assert low >= 2 * self.dT, "<TimeSampling: lower bound of time sampling must be greater-equal than the given" \
                                   "time step>"
        super(TimeSampling, self).__init__(low, up, num_sampling_levels)

    def _sample(self):
        for i in range(self.num_sampling_levels):
            step_size = int((1 / (i + 1)) / self.dT)
            samp = set(np.arange(self.low, round(self.up + self.dT, 2), step_size * self.dT))
            samp.discard(round(self.up + self.dT, 2))
            self._dict_level_to_sample_set[i] = samp


class SamplingSpace(ABC):
    """
    Class representing a set of sampling steps for the time, position, and velocity domain
    """

    def __init__(self, num_sampling_levels: int):
        self._num_sampling_levels = num_sampling_levels
        self._samples_t: Optional[TimeSampling] = None
        self._samples_d: Optional[PositionSampling] = None
        self._samples_v: Optional[VelocitySampling] = None
        self._samples_s: Optional[PositionSampling] = None

    @property
    def samples_t(self) -> TimeSampling:
        """Returns the TimeSampling object"""
        return self._samples_t

    @samples_t.setter
    def samples_t(self, time_sampling: TimeSampling):
        self._samples_t = time_sampling

    @property
    def samples_d(self) -> PositionSampling:
        """Returns the PositionSampling object for the d-coordinate"""
        return self._samples_d

    @samples_d.setter
    def samples_d(self, pos_sampling: PositionSampling):
        self._samples_d = pos_sampling

    @property
    def samples_v(self) -> VelocitySampling:
        """Returns the VelocitySampling object"""
        return self._samples_v

    @samples_v.setter
    def samples_v(self, vel_sampling: VelocitySampling):
        self._samples_v = vel_sampling

    @property
    def num_sampling_levels(self) -> int:
        """Returns the maximum number of sampling stages"""
        return self._num_sampling_levels

    @abstractmethod
    def generate_trajectories_at_level(self, level_sampling: int, x_0_lon: np.ndarray, x_0_lat: np.ndarray,
                                       longitudinal_mode: str, low_vel_mode: bool) \
            -> List[TrajectorySample]:
        """
        Abstract method to generate a set of trajectories within the sampling space for a given sampling level.
        Each sampling space implements it's own trajectory generation method.
        This method is called by the reactive planner to generate the trajectory set for the respective sampling space
        configuration.
        """
        pass


class FixedIntervalSampling(SamplingSpace):
    """
    Class representing a sampling space with fixed intervals for sampling in t, v, d or s domain.
    """

    def __init__(self, config: ReactivePlannerConfiguration):
        num_sampling_levels = config.sampling.num_sampling_levels
        super(FixedIntervalSampling, self).__init__(num_sampling_levels)

        config_sampling = config.sampling

        # timestep and horizon
        self.dt = config.planning.dt
        self.horizon = config.planning.dt * config.planning.time_steps_computation

        # longitudinal sampling mode
        self._longitudinal_mode = None

        # initialize and pre-compute samples in t, d, v domains
        self.samples_t = TimeSampling(config_sampling.t_min, self.horizon, num_sampling_levels, self.dt)
        self.samples_d = PositionSampling(config_sampling.d_min, config_sampling.d_max, num_sampling_levels)
        self.samples_v = VelocitySampling(config_sampling.v_min, config_sampling.v_max, num_sampling_levels)
        self.samples_s = PositionSampling(config_sampling.s_min, config_sampling.s_max, num_sampling_levels)

    def generate_trajectories_at_level(self, level_sampling: int, x_0_lon: np.ndarray, x_0_lat: np.ndarray,
                                       longitudinal_mode: str, low_vel_mode: bool) \
            -> List[TrajectorySample]:
        """
        Implements trajectory generation method for sampling trajectories in fixed intervals in t, v, d  or s domain
        """
        # set longitudinal sampling mode
        self._longitudinal_mode = longitudinal_mode

        # initialize trajectory list
        list_trajectories = list()

        # get longitudinal samples (depending if velocity or position sampling is used)
        longitudinal_samples = self._get_lon_samples(level_sampling)

        # Iterate over pre-stored time samples
        for t in self.samples_t.samples_at_level(level_sampling):
            # Iterate over pre-stored longitudinal velocity or position samples
            for lon_sample in longitudinal_samples:
                trajectory_long = self._generate_lon_trajectory(delta_tau=t, x_0=np.array(x_0_lon), lon_sample=lon_sample)

                # Sample lateral end states (add x_0_lat to sampled states)
                if trajectory_long.coeffs is not None:
                    # Iterate over pre-stored lateral positions samples
                    for d in self.samples_d.samples_at_level(level_sampling).union({x_0_lat[0]}):
                        end_state_lat = np.array([d, 0.0, 0.0])
                        # Switch to sampling over s for low velocities
                        if low_vel_mode:
                            s_lon_goal = trajectory_long.evaluate_state_at_tau(t)[0] - x_0_lon[0]
                            if s_lon_goal <= 0:
                                s_lon_goal = t
                            trajectory_lat = self._generate_lat_trajectory(delta_tau=s_lon_goal, x_0=np.array(x_0_lat),
                                                                           x_d=end_state_lat)
                        # Switch to sampling over t for high velocities
                        else:
                            trajectory_lat = self._generate_lat_trajectory(delta_tau=t, x_0=np.array(x_0_lat),
                                                                           x_d=end_state_lat)
                        if trajectory_lat.coeffs is not None:
                            trajectory_sample = TrajectorySample(self.horizon, self.dt, trajectory_long, trajectory_lat)
                            list_trajectories.append(trajectory_sample)
        return list_trajectories

    def _get_lon_samples(self, level_sampling):
        if self._longitudinal_mode == "velocity_keeping":
            return self.samples_v.samples_at_level(level_sampling)
        elif self._longitudinal_mode == "stopping":
            return self.samples_s.samples_at_level(level_sampling)
        else:
            raise AttributeError(f"<FixedIntervalSampling>: specified longitudinal mode {self._longitudinal_mode} is"
                                 f"invalid.")

    def _generate_lon_trajectory(self, delta_tau: float, x_0: np.ndarray, lon_sample: float):
        if self._longitudinal_mode == "velocity_keeping":
            # longitudinal sample is a target velocity
            # Velocity keeping mode: end state acceleration is 0.0
            end_state_lon = np.array([lon_sample, 0.0])
            return QuarticTrajectory(tau_0=0, delta_tau=delta_tau, x_0=x_0, x_d=end_state_lon)
        elif self._longitudinal_mode == "stopping":
            # longitudinal sample is a target position
            # Stopping mode: end state velocity and acceleration are 0.0
            end_state_lon = np.array([lon_sample, 0.0, 0.0])
            return QuinticTrajectory(tau_0=0, delta_tau=delta_tau, x_0=x_0, x_d=end_state_lon)
        else:
            raise AttributeError(f"<FixedIntervalSampling>: specified longitudinal mode {self._longitudinal_mode} is"
                                 f"invalid.")

    @staticmethod
    def _generate_lat_trajectory(delta_tau: float, x_0: np.ndarray, x_d: np.ndarray):
        return QuinticTrajectory(tau_0=0, delta_tau=delta_tau, x_0=x_0, x_d=x_d)


class CorridorSampling(SamplingSpace):
    """
    Class representing an adaptive sampling space which draws samples from the reachable sets of a precomputed
    collision-free driving corridor.
    NOTE: CommonRoad-Reach is required (https://commonroad.in.tum.de/tools/commonroad-reach)
    """
    def __init__(self, config: ReactivePlannerConfiguration):
        if not cr_reach_installed:
            raise ImportError("<CorridorSampling>: Please install CommonRoad-Reach to use adaptive corridor sampling!")

        num_sampling_levels = config.sampling.num_sampling_levels
        super(CorridorSampling, self).__init__(num_sampling_levels)

        # time step and horizon
        self.dt = config.planning.dt
        self.horizon = config.planning.dt * config.planning.time_steps_computation

        # intialize and precompute samples in t domain
        self.samples_t = TimeSampling(config.sampling.t_min, self.horizon, num_sampling_levels, self.dt)

        # driving corridor: needs to be set with setter function
        self._corridor: Optional[DrivingCorridor] = None
        self._velocity_constraints: Dict = dict()

        # number of samples per level
        self._dict_level_to_num_samples: Dict[int, int] = dict()
        self.set_dict_number_of_samples()

    @property
    def driving_corridor(self):
        return self._corridor

    @driving_corridor.setter
    def driving_corridor(self, corridor: DrivingCorridor):
        self._corridor = corridor
        self._velocity_constraints = dict()
        for time_idx, connected_reach_set in self._corridor.items():
            velocity_interval = util_reach_operation.lon_velocity_interval_connected_set(connected_reach_set)
            self._velocity_constraints[time_idx] = [velocity_interval[0], velocity_interval[1]]

    @SamplingSpace.samples_d.setter
    def samples_d(self, pos_sampling: PositionSampling):
        self._d_min = pos_sampling.low
        self._d_max = pos_sampling.up

    @SamplingSpace.samples_v.setter
    def samples_v(self, vel_sampling: VelocitySampling):
        self._v_min = vel_sampling.low
        self._v_max = vel_sampling.up

    def set_dict_number_of_samples(self, n_min: int = 3, dict_level_to_num_samples: dict = None):
        """
        store number of samples per sampling level
        :param n_min: minimum number of samples in lowest level (default 3)
        :param dict_level_to_num_samples: directly set dictionary with numbers of samples per level
        """
        if dict_level_to_num_samples is not None:
            for level in range(self.num_sampling_levels):
                assert level in dict_level_to_num_samples.keys(), f"<SamplingSpace.set_dict_number_of_samples()>:" \
                                                                  f"input dictionary does not contain sampling level:" \
                                                                  f"{level}"
        else:
            n = n_min
            for i in range(self.num_sampling_levels):
                self._dict_level_to_num_samples[i] = n
                n = (n * 2) - 1

    def generate_trajectories_at_level(self, level_sampling: int, x_0_lon: np.ndarray, x_0_lat: np.ndarray,
                                       longitudinal_mode: str, low_vel_mode: bool) \
            -> List[TrajectorySample]:
        """
        Implements trajectory generation method for sampling trajectories within driving corridor
        """
        if self._corridor is None:
            raise AttributeError("<CorridorSampling>: Please set a driving corridor.")

        # initialize trajectory list
        list_trajectories = list()

        # get num samples for level
        num_samples = self._dict_level_to_num_samples[level_sampling]

        # Iterate over pre-stored time samples
        for t in self.samples_t.samples_at_level(level_sampling):
            # get corresponding time step of corridor
            time_step = round(t / self.dt) + min(self._corridor.keys())
            # Set sampling constraints for longitudinal velocity
            # low = max(self._min_v_desired, self._lon_vel_constraints[time_step][0])
            # up = min(self._max_v_desired, self._lon_vel_constraints[time_step][1])
            low = self._velocity_constraints[time_step][0]
            up = self._velocity_constraints[time_step][1]

            # Iterate over velocity samples
            for v in set(np.linspace(low, up, num_samples)):
                trajectory_long = QuarticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lon), x_d=np.array([v, 0]))
                end_pos_lon = trajectory_long.calc_position(t, t ** 2, t ** 3, t ** 4, t ** 5)

                # Sample lateral end states
                if trajectory_long.coeffs is not None:
                    # Determine connected sets containing long end position by projection on longitudinal domain
                    reachsets_overlap = util_reach_operation.determine_overlapping_nodes_with_lon_pos(
                        self._corridor[time_step], end_pos_lon)
                    if len(list(reachsets_overlap)) == 0:
                        continue
                    lat_connected_sets = util_reach_operation.determine_connected_components(list(reachsets_overlap))

                    # Get lateral constraints from base sets
                    for lat_con_set in lat_connected_sets:
                        lat_interval = util_reach_operation.lat_interval_connected_set(lat_con_set)
                        # Sample positions within lateral interval
                        if lat_interval[0] < 0 < lat_interval[1]:
                            # include sample on reference path
                            d_samples = set(np.linspace(lat_interval[0], lat_interval[1], num_samples)).union({0})
                        else:
                            d_samples = set(np.linspace(lat_interval[0], lat_interval[1], num_samples))
                        for d in d_samples:
                            # Switch to position domain for lateral trajectory planning at low velocities
                            # Switch to sampling over t for high velocities
                            trajectory_lat = QuinticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lat),
                                                               x_d=np.array([d, 0.0, 0.0]))
                            if trajectory_lat.coeffs is not None:
                                trajectory_sample = TrajectorySample(self.horizon, self.dt, trajectory_long,
                                                                     trajectory_lat)
                                list_trajectories.append(trajectory_sample)
        return list_trajectories


def sampling_space_factory(config: ReactivePlannerConfiguration):
    """Factory function to select SamplingSpace class"""
    sampling_method = config.sampling.sampling_method
    if sampling_method == 1:
        return FixedIntervalSampling(config)
    elif sampling_method == 2:
        return CorridorSampling(config)
    else:
        ValueError("Invalid sampling method specified")
