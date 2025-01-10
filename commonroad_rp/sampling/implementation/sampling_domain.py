import numpy as np

from ..base.base_sampling_domain import Sampling


class VelocitySampling(Sampling):
    """
    Sampling steps object for the velocity domain
    """

    def __init__(self, low: float, up: float, num_sampling_levels: int, num_init_samples: int = 3):
        self._num_init_samples = num_init_samples
        super(VelocitySampling, self).__init__(low, up, num_sampling_levels)

    def _sample(self):
        n = self._num_init_samples
        for i in range(self.num_sampling_levels):
            self._dict_level_to_sample_set[i] = set(np.linspace(self.low, self.up, n))
            n = (n * 2) - 1


class PositionSampling(Sampling):
    """
    Sampling steps object for the position domain (s or d-coordinate)
    """

    def __init__(self, low: float, up: float, num_sampling_levels: int, num_init_samples: int = 3):
        self._num_init_samples = num_init_samples
        super(PositionSampling, self).__init__(low, up, num_sampling_levels)

    def _sample(self):
        n = self._num_init_samples
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