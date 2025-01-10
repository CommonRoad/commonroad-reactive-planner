from typing import List, Iterable

from ..base.base_sampling_space import SamplingSpace
from ..implementation.sampling_domain import (
    TimeSampling,
    VelocitySampling,
    PositionSampling
)

from commonroad_rp.utility.config import ReactivePlannerConfiguration


class FixedIntervalSampling(SamplingSpace):
    """
    Class representing a sampling space with fixed intervals for sampling in t, v, d or s domain.
    """

    def __init__(self, config: ReactivePlannerConfiguration):
        super(FixedIntervalSampling, self).__init__(num_sampling_levels=config.sampling.num_sampling_levels,
                                                    dt=config.planning.dt,
                                                    horizon=config.planning.dt * config.planning.time_steps_computation)

        config_sampling = config.sampling

        # initialize and pre-compute samples in t, d, v, s domains
        self.samples_t = TimeSampling(config_sampling.t_min,
                                      self.horizon,
                                      self._num_sampling_levels,
                                      self.dt)
        self.samples_d = PositionSampling(config_sampling.d_min,
                                          config_sampling.d_max,
                                          self._num_sampling_levels,
                                          config_sampling.pos_init_samples)
        self.samples_v = VelocitySampling(config_sampling.v_min,
                                          config_sampling.v_max,
                                          self._num_sampling_levels,
                                          config_sampling.vel_init_samples)
        self.samples_s = PositionSampling(config_sampling.s_min,
                                          config_sampling.s_max,
                                          self._num_sampling_levels,
                                          config_sampling.pos_init_samples)

    def set_t_sampling(self, t_min):
        """Set samples in t domain of sampling space"""
        self.samples_t = TimeSampling(t_min, self.horizon, self._num_sampling_levels, self.dt)

    def set_d_sampling(self, d_min, d_max):
        """Set samples in d domain of sampling space"""
        self.samples_d = PositionSampling(d_min, d_max, self._num_sampling_levels, self.samples_d._num_init_samples)

    def set_v_sampling(self, v_min, v_max):
        """Set samples in v domain of sampling space"""
        self.samples_v = VelocitySampling(v_min, v_max, self._num_sampling_levels, self.samples_v._num_init_samples)

    def set_s_sampling(self, s_min, s_max):
        """Set samples in s domain of sampling space"""
        self.samples_s = PositionSampling(s_min, s_max, self._num_sampling_levels, self.samples_s._num_init_samples)

    def _get_time_samples(self, level_sampling: int, **kwargs) -> Iterable[float]:
        """Returns time samples for the fixed sampling space"""
        return self.samples_t.samples_at_level(level_sampling)

    def _get_lon_samples(self, level_sampling, longitudinal_mode: str, **kwargs):
        """Returns longitudinal samples for the fixed sampling space"""
        if longitudinal_mode == "velocity_keeping":
            return self.samples_v.samples_at_level(level_sampling)
        elif longitudinal_mode == "stopping":
            return self.samples_s.samples_at_level(level_sampling)
        else:
            raise AttributeError(f"<FixedIntervalSampling>: specified longitudinal mode {longitudinal_mode} is"
                                 f"invalid.")

    def _get_lat_samples(self, level_sampling: int, **kwargs) -> Iterable[float]:
        """Returns lateral samples for the fixed sampling space"""
        if "x_0_lat" in kwargs.keys():
            x_0_lat = kwargs["x_0_lat"]
            return self.samples_d.samples_at_level(level_sampling).union({x_0_lat[0]})
        else:
            return self.samples_d.samples_at_level(level_sampling)
