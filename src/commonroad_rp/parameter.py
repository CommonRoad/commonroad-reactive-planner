__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.5"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Beta"

from commonroad.common.validity import *

import numpy as np
from abc import ABC, abstractmethod


class Sampling(ABC):
    """
    Class that represents the sampling parameters for planning
    """

    def __init__(self, low: float, up: float, n_samples: int):
        # Check validity of input
        # assert is_real_number(low), '<SamplingParameters>: Lower sampling bound not valid! low = {}'.format(low)
        # assert is_real_number(up), '<SamplingParameters>: Upper sampling bound not valid! up = {}'.format(up)
        assert np.greater_equal(up,
                                low), '<SamplingParameters>: Upper sampling bound is not greater than ' \
                                      'lower bound! up = {} , low = {}'.format(up, low)
        # assert is_positive(n_samples), '<SamplingParameters>: Step size is not valid! step size = {}'.format(n_samples)
        assert isinstance(n_samples, int)
        assert n_samples > 0

        self.low = low
        self.up = up
        self._n_samples = n_samples
        self._db: list = list()
        self._setup()

    @abstractmethod
    def _setup(self):
        """
        Set up the database of sampling steps per sampling stage
        :return:
        """
        pass

    def to_range(self, sampling_stage: int = 0) -> set:
        """
        Obtain the sampling steps of a given sampling stage
        :param sampling_stage: The sampling stage to receive (>=0)
        :return: The set of sampling steps for the queried sampling stage
        """
        assert 0 <= sampling_stage < self.no_of_samples(), '<Sampling/to_range>: Provided sampling stage is' \
                                                           ' incorrect! stage = {}'.format(
            sampling_stage)
        return self._db[sampling_stage]

    def no_of_samples(self) -> int:
        """
        Returns the maximum number of sampling stages
        :return: The maximum number of sampling stages
        """
        return self._n_samples


class VelocitySampling(Sampling):
    """
    Sampling steps object for the velocity domain
    """

    def __init__(self, low: float, up: float, n_samples: int):
        super(VelocitySampling, self).__init__(low, up, n_samples)

    def _setup(self):
        n = 3
        for i in range(self.no_of_samples()):
            self._db.append(set(np.linspace(self.low, self.up, n)))
            n = (n * 2) - 1


class PositionSampling(Sampling):
    """
    Sampling steps object for the position domain (s or d-coordinate)
    """

    def __init__(self, low: float, up: float, n_samples: int):
        super(PositionSampling, self).__init__(low, up, n_samples)

    def _setup(self):
        n = 3
        for i in range(self.no_of_samples()):
            self._db.append(set(np.linspace(self.low, self.up, n)))
            n = (n * 2) - 1


class TimeSampling(Sampling):
    """
    Sampling steps object for the time domain
    """

    def __init__(self, low: float, up: float, n_samples: int, dT: float):
        self.dT = dT
        super(TimeSampling, self).__init__(low, up, n_samples)

    def _setup(self):
        # self._db.append(np.arange(1, int(self.up) + 1, int(1/self.dT)*self.dT))
        for i in range(self.no_of_samples()):
            step_size = int((1 / (i + 1)) / self.dT)
            samp = set(np.arange(self.low, round(self.up + self.dT, 2), step_size * self.dT))
            samp.discard(round(self.up + self.dT, 2))
            self._db.append(samp)
        # samp = set(np.arange(self.low, round(self.up + self.dT,2), self.dT))
        # self._db.append(samp)


class SamplingSet(ABC):
    """
    Class representing a set of sampling steps for the time, position, and velocity domain
    """

    def __init__(self, t_samples: TimeSampling, d_samples: PositionSampling, v_samples: VelocitySampling):
        assert isinstance(t_samples, TimeSampling)
        assert isinstance(d_samples, PositionSampling) and t_samples.no_of_samples() == d_samples.no_of_samples()
        assert isinstance(v_samples, VelocitySampling) and t_samples.no_of_samples() == v_samples.no_of_samples()

        self._t_samples = t_samples
        self._d_samples = d_samples
        self._v_samples = v_samples

    @property
    def t_samples(self) -> TimeSampling:
        """
        Returns the TimeSampling object
        :return:  The TimeSampling object
        """
        return self._t_samples

    @property
    def d_samples(self) -> PositionSampling:
        """
        Returns the PositionSampling object for the d-coordinate
        :return:
        """
        return self._d_samples

    @property
    def v_samples(self) -> VelocitySampling:
        """
        Returns the VelocitySampling object
        :return:
        """
        return self._v_samples

    @property
    def max_iteration(self) -> int:
        """
        Returns the maximum number of sampling stages
        :return:
        """
        return self._t_samples.no_of_samples()


class DefFailSafeSampling(SamplingSet):
    """
    Class representing the default Fail-safe Sampling steps
    """

    def __init__(self):
        t_samples = TimeSampling(0.4, 6, 5, 0.2)
        d_samples = PositionSampling(-3, 3, 5)
        v_samples = VelocitySampling(0, 0, 5)
        super(DefFailSafeSampling, self).__init__(t_samples, d_samples, v_samples)


class DefGymSampling(SamplingSet):
    """
    Class representing the default Sampling steps
    """

    def __init__(self, dt, horizon):
        sampling_level = 4
        t_samples = TimeSampling(0.2, horizon, sampling_level, dt)
        d_samples = PositionSampling(-2.0, 2.0, sampling_level)
        v_samples = VelocitySampling(0., 25.0, sampling_level)
        super(DefGymSampling, self).__init__(t_samples, d_samples, v_samples)


class VehModelParameters:
    """
    Class that represents the vehicle's constraints and parameters a_max=8, 0.2, 0.2, 10)
    """

    def __init__(self, a_max=8.0, theta_dot_max=1.0, kappa_max=0.2, kappa_dot_max=5, veh_length=4.508, veh_width=1.61):
        self.a_max = a_max
        self.theta_dot_max = theta_dot_max
        self.kappa_max = kappa_max
        self.kappa_dot_max = kappa_dot_max
        self.veh_length = veh_length
        self.veh_width = veh_width
