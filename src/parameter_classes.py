from commonroad.common.validity import *
import numpy as np


class VehModelParameters:
    """
    Class that represents the vehicle's constraints
    """
    def __init__(self, a_max, theta_dot_max, kappa_max, kappa_dot_max):
        self.a_max = a_max
        self.theta_dot_max = theta_dot_max
        self.kappa_max = kappa_max
        self.kappa_dot_max = kappa_dot_max


class SamplingParameters(object):
    """
    Class that represents the sampling parameters for planning
    """

    def __init__(self, low: float, up: float, n_samples: int):
        # Check validity of input
        assert is_real_number(low), '<SamplingParameters>: Lower sampling bound not valid! low = {}'.format(low)
        assert is_real_number(up), '<SamplingParameters>: Upper sampling bound not valid! up = {}'.format(up)
        assert np.greater(up,
                          low), '<SamplingParameters>: Upper sampling bound is not greater than lower bound! up = {} , low = {}'.format(
            up, low)
        assert is_positive(n_samples), '<SamplingParameters>: Step size is not valid! step size = {}'.format(step)
        assert isinstance(n_samples, int)
        assert n_samples > 0

        self.low = low
        self.up = up
        self.n_samples = n_samples

    def to_range(self, sampling_factor: int=1) -> np.ndarray:
        """
        Convert to numpy range object as [center-low,center+up] in "step" steps
        :param sampling_factor: Multiplicative factor for number of samples
        :return: The range [low,up] in "n_samples*sampling_factor" steps
        """
        samples = self.n_samples * sampling_factor + 1

        if sampling_factor == 0:
            return np.array([(self.up+self.low)/2])
        else:
            return np.linspace(self.low, self.up, samples)
