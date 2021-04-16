__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.5"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Beta"

from typing import Union, List
import numpy as np
from abc import ABC, abstractmethod

from commonroad_rp.polynomial_trajectory import PolynomialTrajectory
import commonroad.common.validity as val


class Sample(ABC):
    """
    Abstract class representing a trajectory ssample in a certain coordinate system
    """

    @abstractmethod
    def length(self) -> int:
        """
        Returns the length of the sample
        :return:
        """
        pass

    @abstractmethod
    def enlarge(self, steps: int, dt: float):
        """
        Enlarges the sample specified by a certain number of steps
        :param steps: The steps for enlarging the sample
        :param dt: The time step of between each step
        :return:
        """
        pass


class CartesianSample(Sample):
    """
    Class representing the cartesian trajectory of a given trajectory sample
    """

    def __init__(self, x: np.ndarray, y: np.ndarray, theta: np.ndarray, v: np.ndarray, a: np.ndarray, kappa: np.ndarray,
                 kappa_dot: np.ndarray):
        # assert val.is_real_number_vector(
        #     x), '<CartesianSample/init>: Provided x positions are not valid! x = {}'.format(x)
        # assert val.is_real_number_vector(
        #     y, length=len(x)), '<CartesianSample/init>: Provided y positions are not valid! y = {}'.format(y)
        # assert val.is_real_number_vector(
        #     theta, length=len(x)), '<CartesianSample/init>: Provided orientations are not valid! theta = {}'.format(
        #     theta)
        # assert val.is_real_number_vector(
        #     v, length=len(x)), '<CartesianSample/init>: Provided velocities are not valid! v = {}'.format(v)
        # assert val.is_real_number_vector(
        #     a, length=len(x)), '<CartesianSample/init>: Provided accelerations are not valid! a = {}'.format(a)
        # assert val.is_real_number_vector(
        #     kappa, length=len(x)), '<CartesianSample/init>: Provided curvatures are not valid! kappa =
        #     {}'.format(kappa)
        # assert val.is_real_number_vector(
        #     kappa_dot,
        #     length=len(x)), '<CartesianSample/init>: Provided curvature changes are not valid! kappa_dot = {}'.format(
        #     kappa_dot)

        self._x = x
        self._y = y
        self._theta = theta
        self._v = v
        self._a = a
        self._kappa = kappa
        self._kappa_dot = kappa_dot

    @property
    def x(self) -> np.ndarray:
        """
        Returns the x positions of the trajectory in Cartesian space
        :return: x positions as numpy array
        """
        return self._x

    @x.setter
    def x(self, x):
        pass

    @property
    def y(self) -> np.ndarray:
        """
        Returns the y positions of the trajectory in Cartesian space
        :return: y positions as numpy array
        """
        return self._y

    @y.setter
    def y(self, y):
        pass

    @property
    def theta(self) -> np.ndarray:
        """
        Returns the orientations of the trajectory in Cartesian space
        :return: Orientations as numpy array
        """
        return self._theta

    @theta.setter
    def theta(self, theta):
        pass

    @property
    def v(self) -> np.ndarray:
        """
        Returns the velocities of the trajectory in Cartesian space
        :return: Velocities as numpy array
        """
        return self._v

    @v.setter
    def v(self, v):
        pass

    @property
    def a(self) -> np.ndarray:
        """
        Returns the accelerations of the trajectory in Cartesian space
        :return: Accelerations as numpy array
        """
        return self._a

    @a.setter
    def a(self, a):
        pass

    @property
    def kappa(self) -> np.ndarray:
        """
        Returns the curvatures of the trajectory in Cartesian space
        :return: Curvatures as numpy array
        """
        return self._kappa

    @kappa.setter
    def kappa(self, kappa):
        pass

    @property
    def kappa_dot(self) -> np.ndarray:
        """
        Returns the curvature change of the trajectory in Cartesian space
        :return: Curvature change as numpy array
        """
        return self._kappa_dot

    @kappa_dot.setter
    def kappa_dot(self, kappa_dot):
        pass

    def length(self) -> int:
        """
        Returns the length of the trajectory
        :return: Length of the trajectory as int
        """
        return len(self.x)

    def enlarge(self, steps: int, dt: float):
        """
        Enlarges the Cartesian sample specified by the number of steps
        :param steps: The number of steps for enlarging the sample
        :param dt: The time step between two consecutive steps
        """
        # assert val.is_positive(steps), '<CartesianSample>: Provided steps is not valid! steps = {}'.format(steps)
        # assert val.is_real_number(dt), '<CartesianSample>: Provided time step is not valid! dt = {}'.format(dt)

        # create time index
        t = np.arange(1, steps + 1, 1) * dt
        # enlarge acceleration values
        self._a = np.append(self.a, np.repeat(self.a[-1], steps))

        # enlarge velocities by considering acceleration
        v_temp = self.v[-1] + t * self.a[-1]
        # remove negative velocities
        v_temp = v_temp * np.greater_equal(v_temp, 0)
        self._v = np.append(self._v, v_temp)

        # enlarge orientations
        self._theta = np.append(self.theta, np.repeat(self.theta[-1], steps))
        # enlarge curvatures
        self._kappa = np.append(self.kappa, np.repeat(self.kappa[-1], steps))
        # enlarge curvature changes
        self._kappa_dot = np.append(self.kappa_dot, np.repeat(self.kappa_dot[-1], steps))

        # enlarge positions
        self._x = np.append(self.x, self.x[-1] + np.cumsum(dt * v_temp * np.cos(self.theta[-1])))
        self._y = np.append(self.y, self.y[-1] + np.cumsum(dt * v_temp * np.sin(self.theta[-1])))


class CurviLinearSample(Sample):
    """
    Class representing the curvilinear trajectory of a given trajectory sample
    """

    def __init__(self, s: np.ndarray, d: np.ndarray, theta: np.ndarray, dd=None, ddd=None, ss=None, sss=None):
        # assert val.is_real_number_vector(
        #     s), '<CurvilinearSample/init>: Provided s positions are not valid! s = {}'.format(s)
        # assert val.is_real_number_vector(
        #     d, length=len(s)), '<CurvilinearSample/init>: Provided d positions are not valid! d = {}'.format(d)
        # assert val.is_real_number_vector(
        #     theta, length=len(s)), '<CurvilinearSample/init>: Provided orientations are not valid! theta = {}'.format(
        #     theta)

        self._s = s
        self._d = d
        self._theta = theta
        # store time derivations
        self._d_dot = dd
        self._d_ddot = ddd
        self._s_dot = ss
        self._s_ddot = sss

    @property
    def s(self) -> np.ndarray:
        """
        Returns the s coordinate of the sample
        :return:
        """
        return self._s

    @s.setter
    def s(self, s):
        pass

    @property
    def d(self) -> np.ndarray:
        """
        Returns the d coordinate of the sample
        :return:
        """
        return self._d

    @d.setter
    def d(self, d):
        pass

    @property
    def theta(self) -> np.ndarray:
        """
        Returns the orientations of the sample
        :return:
        """
        return self._theta

    @theta.setter
    def theta(self, theta):
        pass

    @property
    def d_dot(self) -> np.ndarray:
        """
        Returns the derivation of the d coordinate
        :return:
        """
        return self._d_dot

    @d_dot.setter
    def d_dot(self, d_dot):
        pass

    @property
    def d_ddot(self) -> np.ndarray:
        """
        Returns the second derivation of the d coordinate
        :return:
        """
        return self._d_ddot

    @d_ddot.setter
    def d_ddot(self, d_ddot):
        pass

    @property
    def s_dot(self) -> np.ndarray:
        """
        Returns the derivation of the s coordinate
        :return:
        """
        return self._s_dot

    @s_dot.setter
    def s_dot(self, s_dot):
        pass

    @property
    def s_ddot(self) -> np.ndarray:
        """
        Returns the second derivation of the s coordinate
        :return:
        """
        return self._s_ddot

    @s_ddot.setter
    def s_ddot(self, s_ddot):
        pass

    def length(self) -> int:
        return len(self.s)

    def enlarge(self, steps: int, dt: float):
        """
        Enlarges the curvilinear sample specified by the number of steps
        :param steps: The number of steps for enlarging
        :param dt: The time step between two v steps
        :return:
        """
        # assert val.is_positive(steps), '<CartesianSample>: Provided steps is not valid! steps = {}'.format(steps)
        # assert val.is_real_number(dt), '<CartesianSample>: Provided time step is not valid! dt = {}'.format(dt)

        # create time array
        t = np.arange(1, (steps + 1), 1) * dt
        # enlarge velocities by considering acceleration
        s_dot_temp = self.s_dot[-1] + t * self.s_ddot[-1]
        # remove negative velocities
        s_dot_temp = s_dot_temp * np.greater_equal(s_dot_temp, 0)
        self._s_dot = np.append(self.s_dot, s_dot_temp)

        # enlarge velocities by considering acceleration
        d_dot_temp = self.d_dot[-1] + t * self.d_ddot[-1]
        self._d_dot = np.append(self.d_dot, d_dot_temp)

        # enlarge accelerations
        self._s_ddot = np.append(self.s_ddot, np.repeat(self.s_ddot, steps))
        self._d_ddot = np.append(self.d_ddot, np.repeat(self.d_ddot, steps))

        # enlarge orientations
        self._theta = np.append(self.theta, np.repeat(self.theta[-1], steps))

        # enlarge positions
        self._s = np.append(self.s, self.s[-1] + t * self.s_dot[-1])
        self._d = np.append(self.d, self.d[-1] + t * self.d_dot[-1])


class TrajectorySample(Sample):
    """
    Class representing a TrajectorySample which stores the information about the polynomials
    for the longitudinal and lateral motion
    """

    def __init__(self, horizon: float, dt: float, trajectory_long: PolynomialTrajectory,
                 trajectory_lat: PolynomialTrajectory):
        self.horizon = horizon
        self.dt = dt
        assert isinstance(trajectory_long,
                          PolynomialTrajectory), '<TrajectorySample/init>: Provided longitudinal trajectory ' \
                                                 'is not valid! trajectory = {}'.format(
            trajectory_long)
        self._trajectory_long = trajectory_long
        assert isinstance(trajectory_lat,
                          PolynomialTrajectory), '<TrajectorySample/init>: Provided lateral trajectory ' \
                                                 'is not valid! trajectory = {}'.format(
            trajectory_lat)
        self._trajectory_lat = trajectory_lat

        self._cost = 0
        self._cost_function = None
        self._cartesian: CartesianSample = None
        self._curvilinear: CurviLinearSample = None
        self._ext_cartesian = None
        self._ext_curvilinear = None

    @property
    def trajectory_long(self) -> PolynomialTrajectory:
        """
        Returns the longitudinal polynomial trajectory
        :return: The longitudinal polynomial trajectory
        """
        return self._trajectory_long

    @trajectory_long.setter
    def trajectory_long(self, trajectory_long):
        pass

    @property
    def trajectory_lat(self) -> PolynomialTrajectory:
        """
        Returns the lateral polynomial trajectory
        :return: The lateral polynomial trajectory
        """
        return self._trajectory_lat

    @trajectory_lat.setter
    def trajectory_lat(self, trajectory_lat):
        pass

    @property
    def cost(self) -> float:
        """
        Evaluated cost of the trajectory sample
        :return: The cost of the trajectory sample
        """
        return self._cost

    @cost.setter
    def cost(self, cost_function):
        """
        Sets the cost function for evaluating the costs of the polynomial trajectory
        :param cost_function: The cost function for computing the costs
        """
        self._cost = cost_function.evaluate(self)
        self._cost_function = cost_function

    @property
    def curvilinear(self) -> CurviLinearSample:
        """
        The curvilinear sample of the trajectory sample
        :return: The curvilinear sample
        """
        return self._curvilinear

    @curvilinear.setter
    def curvilinear(self, curvilinear: CurviLinearSample):
        """
        Sets the curvilinear sample of the trajectory sample
        :param curvilinear: The computed curvilinear sample
        """
        assert isinstance(curvilinear, CurviLinearSample)
        self._curvilinear = curvilinear

    @property
    def cartesian(self) -> CartesianSample:
        """
        Returns the Cartesian sample of the trajectory sample
        :return: The Cartesian sample
        """
        return self._cartesian

    @cartesian.setter
    def cartesian(self, cartesian: CartesianSample):
        """
        Sets the Cartesian sample of the trajectory sample
        :param cartesian: The Cartesian sample
        """
        assert isinstance(cartesian, CartesianSample)
        self._cartesian = cartesian

    def length(self) -> int:
        """
        Length of the trajectory sample (number of states)
        :return: The number of states in the trajectory sample
        """
        return self.cartesian.length()

    def enlarge(self, steps: int, dt: float):
        """
        Enlarges the Cartesian and curvilinear sample specified by the given steps
        :param steps: The steps for enlarging
        :param dt: The time step between two consecutive steps
        """
        # assert val.is_positive(steps), '<CartesianSample>: Provided steps is not valid! steps = {}'.format(steps)
        # assert val.is_real_number(dt), '<CartesianSample>: Provided time step is not valid! dt = {}'.format(dt)
        self._cartesian.enlarge(steps, dt)
        self._curvilinear.enlarge(steps, dt)


class TrajectoryBundle:
    """
    Class that represents a collection of trajectories
    """

    def __init__(self, trajectories: List[TrajectorySample], cost_function):
        """
        Initializer of a TrajectoryBundle
        :param trajectories: The list of trajectory samples
        :param cost_function: The cost function for the evaluation
        """
        assert isinstance(trajectories, list) and all([isinstance(t, TrajectorySample) for t in
                                                       trajectories]), '<TrajectoryBundle/init>: ' \
                                                                       'Provided list of trajectory samples is not ' \
                                                                       'valid! List = {}'.format(trajectories)
        self._trajectory_bundle: List[TrajectorySample] = trajectories
        self._cost_function = cost_function
        self._is_sorted = False

    def sort(self):
        """
        Sorts the trajectories within the TrajectoryBundle according to their costs from lowest to highest.
        """
        if not self._is_sorted:
            for trajectory in self._trajectory_bundle:
                trajectory.cost = self._cost_function
            self._trajectory_bundle.sort(key=lambda x: x.cost)
            self._is_sorted = True

    def optimal_trajectory(self) -> Union[TrajectorySample, None]:
        """
        Returns the optimal trajectory (the one with minimal costs) from the TrajectoryBundle
        :return: Trajectory in trajectory_bundle with minimal cost. None, if trajectory bundle is empty.
        """
        if not self._trajectory_bundle or not self._is_sorted:
            return None
        return min(self._trajectory_bundle, key=lambda x: x.total_cost)

    def min_costs(self) -> TrajectorySample:
        """
        Returns the trajectory with the minimal costs.
        :return: The trajectory with the minimal costs
        """
        return self._trajectory_bundle[0] if self._is_sorted else None

    def max_costs(self) -> TrajectorySample:
        """
        Returns the trajectory with the maximal costs.
        :return: The trajectory with the maximal costs
        """
        return self._trajectory_bundle[-1] if self._is_sorted else None

    def get_sorted_list(self) -> list:
        """
        Returns an ordered list of the trajectories. The trajectories are ordered according
        to their costs from lowest to highest.
        :return: List of trajectories sorted from lowest to highest cost
        """
        if not self._is_sorted:
            self.sort()
        return self._trajectory_bundle

    @property
    def trajectories(self) -> List[TrajectorySample]:
        """
        Returns the list of trajectory samples
        :return: The list of trajectory samples
        """
        return self._trajectory_bundle

    @trajectories.setter
    def trajectories(self, trajectories: List[TrajectorySample]):
        """
        Sets the list of trajectory samples
        :param trajectories: The list of trajectory samples
        """
        self._trajectory_bundle = trajectories

    def empty(self) -> bool:
        """
        Checks if the TrajectoryBundle is empty, i.e., does not contain any TrajectorySample
        :return:
        """
        return len(self._trajectory_bundle) == 0
