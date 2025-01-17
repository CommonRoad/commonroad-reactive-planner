from abc import ABC, abstractmethod
from typing import Dict

import numpy as np


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
