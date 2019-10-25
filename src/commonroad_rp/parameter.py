__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.1"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Alpha"


from commonroad.common.validity import *

import numpy as np
from abc import ABC, abstractmethod

class Sampling(ABC):
    """
    Class that represents the sampling parameters for planning
    """

    def __init__(self, low: float, up: float, n_samples: int):
        # Check validity of input
        assert is_real_number(low), '<SamplingParameters>: Lower sampling bound not valid! low = {}'.format(low)
        assert is_real_number(up), '<SamplingParameters>: Upper sampling bound not valid! up = {}'.format(up)
        assert np.greater_equal(up,
                          low), '<SamplingParameters>: Upper sampling bound is not greater than lower bound! up = {} , low = {}'.format(
            up, low)
        assert is_positive(n_samples), '<SamplingParameters>: Step size is not valid! step size = {}'.format(n_samples)
        assert isinstance(n_samples, int)
        assert n_samples > 0

        self.low = low
        self.up = up
        self._n_samples = n_samples
        self._db: list = list()
        self._setup()

    @abstractmethod
    def _setup(self):
        pass


    def to_range(self, sampling_stage: int=0) -> set:
        """
        Convert to numpy range object as [center-low,center+up] in "step" steps
        :param sampling_factor: Multiplicative factor for number of samples
        :return: The range [low,up] in "n_samples*sampling_factor" steps
        """
        assert 0<=sampling_stage<self.no_of_samples(), '<Sampling/to_range>: Provided sampling stage is incorrect! stage = {}'.format(sampling_stage)
        return self._db[sampling_stage]

    def no_of_samples(self) -> int:
        """
        Returns the number of elements in the range of this sampling parameters object
        :return: The number of elements in the range
        """
        return self._n_samples



class VelocitySampling(Sampling):

    def __init__(self, low: float, up: float, n_samples: int):
        super(VelocitySampling,self).__init__(low, up, n_samples)

    def _setup(self):
        removal = set()
        for i in range(self.no_of_samples()):
            samp = set(np.linspace(self.low,self.up,i+3))
            self._db.append(set(np.linspace(self.low,self.up,i+3)))

class PositionSampling(Sampling):

    def __init__(self, low: float, up: float, n_samples: int):
        super(PositionSampling, self).__init__(low, up, n_samples)

    def _setup(self):
        removal = set()
        for i in range(self.no_of_samples()):
            #samp = set(np.linspace(self.low, self.up, i + 4))
            samp = set(np.arange(self.low, self.up+1/(i+1), 1/(i+1)))
            self._db.append((samp - removal) | {0})
            removal |= samp


class TimeSampling(Sampling):

    def __init__(self, low: float, up: float, n_samples: int, dT: float):
        self.dT = dT
        super(TimeSampling, self).__init__(low, up, n_samples)

    def _setup(self):
        self._db.append(np.arange(1, self.up+1, 1))
        for i in range(1,self.no_of_samples()):
            samp = set(np.arange(self.low, self.up+self.dT, self.dT))
            self._db.append(samp)



class SamplingSet(ABC):

    def __init__(self, t_samples: TimeSampling, d_samples: PositionSampling, v_samples: VelocitySampling):
        assert isinstance(t_samples, TimeSampling)
        assert isinstance(d_samples, PositionSampling) and t_samples.no_of_samples()==d_samples.no_of_samples()
        assert isinstance(v_samples, VelocitySampling) and t_samples.no_of_samples()==v_samples.no_of_samples()

        self._t_samples = t_samples
        self._d_samples = d_samples
        self._v_samples = v_samples

    @property
    def t_samples(self) -> TimeSampling:
        return self._t_samples

    @property
    def d_samples(self) -> PositionSampling:
        return self._d_samples

    @property
    def v_samples(self) -> VelocitySampling:
        return self._v_samples

    @property
    def max_iteration(self) -> int:
        return self._t_samples.no_of_samples()



class DefFailSafeSampling(SamplingSet):

    def __init__(self):
        t_samples = TimeSampling(0.4,6,5, 0.2)
        d_samples = PositionSampling(-3,3,5)
        v_samples = VelocitySampling(0,0,5)
        super(DefFailSafeSampling, self).__init__(t_samples,d_samples,v_samples)

class VehModelParameters:
    """
    Class that represents the vehicle's constraints and parameters a_max=8, 0.2, 0.2, 10)
    """
    def __init__(self, a_max=8, theta_dot_max=0.2, kappa_max=0.2, kappa_dot_max=0.4, veh_length=5.1, veh_width=1.9):
        self.a_max = a_max
        self.theta_dot_max = theta_dot_max
        self.kappa_max = kappa_max
        self.kappa_dot_max = kappa_dot_max
        self.veh_length = veh_length
        self.veh_width = veh_width


# planning parameter
class PlanningParameter:
    k_long = None
    k_lat = None

    k_jerk_lon = None
    k_jerk_lat = None
    k_time = None
    k_distance = None

    prediction_horizon = None
    speed_limit = None

    t_step_size = None
    lat_step_size = None
    long_step_size = None


# vehicle parameter
class VehicleParameter:
    jerk_long_max = None
    jerk_lat_max = None

    acceleration_max = 8  # m/s²
    velocity_max = 150 / 3.6  # m/s

    curvature_max = 0.2

    width = 1.674  # vehicle width [m]
    length = 4.298  # vehicle length [m]

    # parameters for calculating steering angle

    length_front = length / 3
    length_rear = 2 * length / 3
    stiffness_front = 40000  # front tire stiffness [N/rad]
    stiffness_rear = 40000  # rear tire stiffens [N/rad]
    mass = 1500  # mass of vehicle [kg]



def parameter_velocity_reaching():

    params = PlanningParameter()

    params.k_long = .1
    params.k_lat = 5.0

    params.k_jerk_lat = 5.0
    params.k_jerk_lon = 5.0
    params.k_time = 10.0
    params.k_distance = 2.0

    params.prediction_horizon = 3.0     # s
    params.speed_limit = 130.0/3.6      # m/s

    params.t_step_size = .5               # s
    params.lat_step_size = 1
    params.long_step_size = 1

    return params


def parameter_position_reaching():
    params = PlanningParameter()

    params.k_long = 1.0
    params.k_lat = 10.0

    params.k_jerk_lat = 5.0  # 1.0
    params.k_jerk_lon = 5.0
    params.k_time = 10.0
    params.k_distance = 10.0  # 2.0

    params.prediction_horizon = 3.0  # s
    params.speed_limit = 130.0/3.6      # m/s

    params.t_step_size = .5  # s
    params.lat_step_size = 1
    params.long_step_size = 1

    return params


# visualization parameter
basic_shape_parameters_static = {'opacity': 1.0,
                                   'facecolor': '#1d7eea',
                                   'edgecolor': '#0066cc',
                                   'zorder': 20}

basic_shape_parameters_dynamic = {'opacity': 1.0,
                                 'facecolor': '#1d7eea',
                                 'edgecolor': '#0066cc',
                                 'zorder': 100}

draw_params_scenario = {'scenario': {
                        'dynamic_obstacle': {
                            'draw_shape': True,
                            'draw_icon': False,
                            'draw_bounding_box': True,
                            'show_label': False,
                            'trajectory_steps': 25,
                            'zorder': 100,
                            'occupancy': {
                                'draw_occupancies': 1,  # -1= never, 0= if prediction of vehicle is set-based, 1=always
                                'shape': {
                                    'polygon': {
                                    'opacity': 0.2,
                                       'facecolor': '#ff4000',
                                       'edgecolor': '#cc3300',
                                       'zorder': 18,
                                    },
                                    'rectangle': {
                                       'opacity': 0.2,
                                       'facecolor': '#1d7eea',
                                       'edgecolor': '#0066cc',
                                       'zorder': 18,
                                    },
                                    'circle': {
                                       'opacity': 0.2,
                                       'facecolor': '#1d7eea',
                                       'edgecolor': '#0066cc',
                                       'zorder': 18,
                                    }
                                },
                            },
                            'shape': {
                                'polygon': basic_shape_parameters_dynamic,
                                'rectangle': basic_shape_parameters_dynamic,
                                'circle': basic_shape_parameters_dynamic
                            },
                             'trajectory': {'facecolor': '#000000'}
                        },
                        'static_obstacle': {
                           'shape': {
                               'polygon': basic_shape_parameters_static,
                               'rectangle': basic_shape_parameters_static,
                               'circle': basic_shape_parameters_static,
                           }
                        },
                        'lanelet_network': {
                            'lanelet': {'left_bound_color': '#555555',
                                       'right_bound_color': '#555555',
                                       'center_bound_color': '#dddddd',
                                       'draw_left_bound': True,
                                       'draw_right_bound': True,
                                       'draw_center_bound': True,
                                       'draw_border_vertices': False,
                                       'draw_start_and_direction': True,
                                       'show_label': False,
                                       'draw_linewidth': 0.5,
                                       'fill_lanelet': True,
                                       'facecolor': '#e8e8e8'}},
                   },
}
