from typing import List, Union, Iterable
from abc import ABC, abstractmethod

import numpy as np

from commonroad_rp.trajectories import TrajectorySample
from commonroad_rp.polynomial_trajectory import (
    QuinticTrajectory,
    QuarticTrajectory
)


class SamplingSpace(ABC):
    """
    Base class representing a sampling space for trajectory generation of the reactive planner.
    """

    def __init__(
            self,
            num_sampling_levels: int,
            dt: float,
            horizon: float
    ):
        self._num_sampling_levels = num_sampling_levels
        self.dt = dt
        self.horizon = horizon

    @property
    def num_sampling_levels(self) -> int:
        """Returns the maximum number of sampling stages"""
        return self._num_sampling_levels

    @abstractmethod
    def set_t_sampling(self, t_min):
        """Abstract method to set minimum t of sampling space"""
        pass

    @abstractmethod
    def set_d_sampling(self, d_min, d_max):
        """Abstract method to set d_min, d_max of sampling space"""
        pass

    @abstractmethod
    def set_v_sampling(self, v_min, v_max):
        """Abstract method to set v_min, v_max of sampling space"""
        pass

    @abstractmethod
    def set_s_sampling(self, s_min, s_max):
        """Abstract method to set v_min, v_max of sampling space"""
        pass

    @abstractmethod
    def _get_time_samples(self, level_sampling: int, **kwargs) -> Iterable[float]:
        """Abstract method to return an iterable object of time samples"""
        pass

    @abstractmethod
    def _get_lon_samples(self, level_sampling: int, longitudinal_mode: str, **kwargs) -> Iterable[float]:
        """Abstract method to return an iterable object of longitudinal samples"""
        pass

    @abstractmethod
    def _get_lat_samples(self, level_sampling: int, **kwargs) -> Iterable[float]:
        """Abstract method to return an iterable object of lateral samples"""
        pass

    def generate_trajectories_at_level(
            self,
            level_sampling: int,
            x_0_lon: np.ndarray,
            x_0_lat: np.ndarray,
            longitudinal_mode: str,
            low_vel_mode: bool
    ) -> List[TrajectorySample]:
        """
        Generates a set of trajectories within the sampling space for a given sampling level.
        Interface method to the reactive planner to generate the trajectory set for the respective sampling space
        configuration.
        """
        # initialize trajectory list
        list_trajectories = list()

        # iterate over time samples
        for t in self._get_time_samples(level_sampling):
            # iterate over longitudinal samples
            for lon_sample in self._get_lon_samples(level_sampling,
                                                    longitudinal_mode,
                                                    t_sample=t,
                                                    x_0=x_0_lon):
                # create longitudinal trajectory
                trajectory_long = self._generate_lon_trajectory(
                    delta_tau=t,
                    x_0=np.array(x_0_lon),
                    lon_sample=lon_sample,
                    mode=longitudinal_mode)

                if trajectory_long.coeffs is not None:
                    # iterate over lateral samples
                    for d in self._get_lat_samples(level_sampling,
                                                   x_0_lat=x_0_lat,
                                                   lon_trajectory=trajectory_long,
                                                   t_sample=t):
                        # lateral end state (zero lat velocity & acceleration)
                        end_state_lat = np.array([d, 0.0, 0.0])

                        # switch velocity mode
                        delta_tau = self._switch_vel_mode(
                            low_vel_mode=low_vel_mode,
                            t=t,
                            trajectory_long=trajectory_long,
                            x_0_lon=x_0_lon
                        )

                        # generate lateral trajectory
                        trajectory_lat = self._generate_lat_trajectory(
                            delta_tau=delta_tau,
                            x_0=np.array(x_0_lat),
                            x_d=end_state_lat,
                        )

                        if trajectory_lat.coeffs is not None:
                            # add combined trajectory sample to output list
                            list_trajectories.append(
                                TrajectorySample(self.horizon,
                                                 self.dt,
                                                 trajectory_long,
                                                 trajectory_lat)
                            )
        return list_trajectories

    @staticmethod
    def _generate_lon_trajectory(
            delta_tau: float,
            x_0: np.ndarray,
            lon_sample: float,
            mode: str = "velocity_keeping"
    ) -> Union[QuinticTrajectory, QuarticTrajectory]:
        """Returns a Quartic or Quintic polynomial trajectory"""
        if mode == "velocity_keeping":
            # Longitudinal sample is a target velocity.
            # Velocity keeping mode: end state acceleration is 0.0
            end_state_lon = np.array([lon_sample, 0.0])
            return QuarticTrajectory(tau_0=0, delta_tau=delta_tau, x_0=x_0, x_d=end_state_lon)
        elif mode == "stopping":
            # Longitudinal sample is a target position.
            # Stopping mode: end state velocity and acceleration are 0.0
            end_state_lon = np.array([lon_sample, 0.0, 0.0])
            return QuinticTrajectory(tau_0=0, delta_tau=delta_tau, x_0=x_0, x_d=end_state_lon)
        else:
            raise AttributeError(f"<SamplingSpace>: specified longitudinal mode {mode} is invalid.")

    @staticmethod
    def _generate_lat_trajectory(
            delta_tau: float,
            x_0: np.ndarray,
            x_d: np.ndarray
    ) -> QuinticTrajectory:
        """Returns a Quintic polynomial trajectory"""
        return QuinticTrajectory(tau_0=0, delta_tau=delta_tau, x_0=x_0, x_d=x_d)

    @staticmethod
    def _switch_vel_mode(
            low_vel_mode: bool,
            t: float,
            trajectory_long: Union[QuinticTrajectory, QuarticTrajectory],
            x_0_lon: np.ndarray
    ) -> float:
        """Returns end time or end position for the lateral trajectory depending on high/low velocity mode"""
        if low_vel_mode:
            s_lon_goal = trajectory_long.evaluate_state_at_tau(t)[0] - x_0_lon[0]
            if s_lon_goal <= 0:
                s_lon_goal = t
            return s_lon_goal
        else:
            return t
