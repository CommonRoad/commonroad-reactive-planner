import numpy as np
import warnings
from abc import ABC, abstractmethod

"""
NOTE: in this class, time is continuous not in discrete indexes
"""


class PolynomialTrajectory(ABC):
    def __init__(self, t_start_s=0, duration_s=0, desired_horizon=0, target_velocity=0,
                 start_state=np.zeros([3, 1]), end_state=np.zeros([3, 1])):
        super(PolynomialTrajectory, self).__init__()
        self.t_start_s = t_start_s
        self.duration_s = duration_s
        self.desired_horizon = desired_horizon
        self.target_velocity = target_velocity
        self.start_state = start_state
        self.end_state = end_state
        self.cost = None

        self.coeffs = self.calc_coeffs()

    # calculate coefficients
    @abstractmethod
    def calc_coeffs(self):
        """
        In inherited classes, the coefficients of the polynomial has to be determined as
        numpy array of shape [1, 6]. Evaluated during initialization.
        :return:
        """
        pass

    @property
    def cost(self):
        return self.__cost

    @cost.setter
    def cost(self, cost):
        if cost and cost < 0:
            self.__cost = 0
            warnings.warn('Negative cost: set to 0.')
        else:
            self.__cost = cost

    def set_cost(self, jerk_cost, time_cost, distance_cost, k_jerk, k_time, k_distance):
        """
        Sets and calculates the cost from given weights and cost of jerk, time and distance
        :param jerk_cost: jerk cost
        :param time_cost: time cost
        :param distance_cost: distance cost
        :param k_jerk: jerk weight
        :param k_time: time weight
        :param k_distance: distance weight
        :return: none
        """
        self.cost = k_jerk * jerk_cost + k_time * time_cost + k_distance * distance_cost

    def squared_jerk_integral(self, t):
        """
        returns the integral of the squared jerk of a fifth order polynomial in the interval [0, t]
        :param t: time > 0
        :return: evaluated integral
        """
        if self.coeffs is None:
            raise ValueError('Coefficients are not determined')
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t

        integral_squared_jerk = (36 * self.coeffs[3] * self.coeffs[3] * t + 144 * self.coeffs[3] * self.coeffs[4]*t2 +
                                 240 * self.coeffs[3] * self.coeffs[5] * t3 + 192 * self.coeffs[4] * self.coeffs[4]*t3 +
                                 720 * self.coeffs[4] * self.coeffs[5]*t4 + 720 * self.coeffs[5] * self.coeffs[5] * t5)

        return integral_squared_jerk

    def get_velocity_cost(self, desired_velocity):
        """
        Calculates velocity cost for reaching desired_velocity
        :param desired_velocity: desired velocity
        :return: velocity cost
        """
        return (self.calc_velocity_at(self.duration_s) - desired_velocity)**2

    def get_distance_cost(self, desired_distance):
        """
        Calculates velocity cost for reaching desired_velocity
        :param desired_distance: desired distance
        :return: distance cost
        """
        return (self.calc_position_at(self.duration_s) - desired_distance)**2

    # evaluate polynomials
    def evaluate_state_at_time(self, t):
        """
        Returns the position, velocity and acceleration at time t of the polynomial with coefficients self.coeffs
        :param t: point in time between self.t_start and self.duration_s + t_start.
        :return: Numpy array of the form [p, p_dot, p_ddot]
        """
        if self.coeffs is None:
            raise ValueError('Coefficients are not determined')

        delta_t = t - self.t_start_s
        if delta_t < 0:
            t = self.t_start_s
        elif delta_t > self.duration_s:
            t = self.t_start_s + self.duration_s

        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t

        p = (self.coeffs(0) + self.coeffs(1) * t + self.coeffs(2) * t2 + self.coeffs(3) * t3 + self.coeffs(4) * t4 +
             self.coeffs(5) * t5)
        p_d = (self.coeffs(1) + 2 * self.coeffs(2) * t + 3 * self.coeffs(3) * t2 + 4 * self.coeffs(4) * t3 +
               5 * self.coeffs(5) * t4)
        p_dd = 2 * self.coeffs(2) + 6 * self.coeffs(3) * t + 12 * self.coeffs(4) * t2 + 20 * self.coeffs(5) * t3

        return np.array([p, p_d, p_dd])

    def calc_jerk_at(self, t):
        """
        Calculates jerk at time t
        :param t: time
        :return: jerk
        """
        if self.coeffs is None:
            raise ValueError('Coefficients are not determined')
        return 6 * self.coeffs[3] + 24 * self.coeffs[4] * t + 60 * self.coeffs[5] * t * t

    def calc_acceleration(self, t,t2,t3):
        """
        Calculates acceleration at time t
        :param t: time t
        :param t2: t * t
        :param t3: t * t * t
        :return: acceleration
        """
        if self.coeffs is None:
            raise ValueError('Coefficients are not determined')
        return 2 * self.coeffs[2] + 6 * self.coeffs[3] * t + 12 * self.coeffs[4] * t2 + 20 * self.coeffs[5] * t3

    def calc_acceleration_at(self, t):
        """
        Calculates acceleration
        :param t: time
        :return: acceleration
        """
        if self.coeffs is None:
            raise ValueError('Coefficients are not determined')
        t2 = t * t
        t3 = t2 * t
        return 2 * self.coeffs[2] + 6 * self.coeffs[3] * t + 12 * self.coeffs[4] * t2 + 20 * self.coeffs[5] * t3

    def calc_velocity(self, t,t2,t3,t4):
        """
        Calculates velocity
        :param t: time t
        :param t2: t * t
        :param t3: t * t * t
        :param t4: t * t * t * t
        :return: velocity
        """
        if self.coeffs is None:
            raise ValueError('Coefficients are not determined')
        return (self.coeffs[1] + 2. * self.coeffs[2] * t + 3. * self.coeffs[3] * t2 + 4. * self.coeffs[4] * t3 +
                5. * self.coeffs[5] * t4)

    def calc_velocity_at(self, t):
        """
        Calculates velocity at time t
        :param t: time
        :return: velocity
        """
        if self.coeffs is None:
            raise ValueError('Coefficients are not determined')
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        return (self.coeffs[1] + 2. * self.coeffs[2] * t + 3. * self.coeffs[3] * t2 + 4. * self.coeffs[4] * t3 +
                5. * self.coeffs[5] * t4)

    def calc_position(self, t,t2,t3,t4,t5):
        """
        Calculates position
        :param t: time t
        :param t2: t * t
        :param t3: t * t * t
        :param t4: t * t * t * t
        :param t5: t * t * t * t * t
        :return: position
        """
        if self.coeffs is None:
            raise ValueError('Coefficients are not determined')
        return (self.coeffs[0] + self.coeffs[1] * t + self.coeffs[2] * t2 + self.coeffs[3] * t3 +
                self.coeffs[4] * t4 + self.coeffs[5] * t5)

    def calc_position_at(self, t):
        """
        Calculates position at time t
        :param t:
        :return: position
        """
        if self.coeffs is None:
            raise ValueError('Coefficients are not determined')

        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        return (self.coeffs[0] + self.coeffs[1] * t + self.coeffs[2] * t2 + self.coeffs[3] * t3 +
                self.coeffs[4] * t4 + self.coeffs[5] * t5)


class QuinticTrajectory(PolynomialTrajectory):
    def __init__(self, t_start_s=0, duration_s=0, desired_horizon=0, start_state=np.zeros([3, 1]), end_state=np.zeros([3, 1]), desired_velocity=0):
        super(QuinticTrajectory, self).__init__(t_start_s=t_start_s, duration_s=duration_s, desired_horizon=desired_horizon,
                                                start_state=start_state, end_state=end_state)
        self._desired_velocity = desired_velocity

    def calc_coeffs(self):
        """
        calculates coefficients of quintic trajectory for given start state, end state and desired duration
        :return:
        """
        p_init = self.start_state[0]
        p_init_d = self.start_state[1]
        p_init_dd = self.start_state[2]

        p_final = self.end_state[0]
        p_final_d = self.end_state[1]
        p_final_dd = self.end_state[2]

        t2 = self.duration_s * self.duration_s  # s²
        t3 = t2 * self.duration_s               # s³
        t4 = t3 * self.duration_s               # s⁴
        t5 = t4 * self.duration_s               # s⁵

        a = np.array([[t3, t4, t5],
                      [3.*t2, 4.*t3, 5.*t4],
                      [6.*self.duration_s, 12.*t2, 20.*t3]])

        b = np.array([p_final - (p_init + p_init_d * self.duration_s + .5 * p_init_dd * t2),
                     p_final_d - (p_init_d + p_init_dd * self.duration_s),
                     p_final_dd - p_init_dd])

        try:
            x = np.linalg.solve(a, b)
        except Exception as e:
            print(e)
            return None
        return np.array([p_init, p_init_d, .5 * p_init_dd, x[0], x[1], x[2]])



class QuarticTrajectory(PolynomialTrajectory):
    def __init__(self, t_start_s=0, duration_s=0, desired_horizon=0, start_state=np.zeros([3, 1]), target_velocity=0, desired_velocity=0):
        super(QuarticTrajectory, self).__init__(t_start_s=t_start_s, duration_s=duration_s, desired_horizon=desired_horizon,
                                                target_velocity=target_velocity, start_state=start_state)
        self._desired_velocity = desired_velocity

    def calc_coeffs(self):
        p_init = self.start_state[0]
        p_init_d = self.start_state[1]
        p_init_dd = self.start_state[2]

        t2 = self.duration_s * self.duration_s
        t3 = t2 * self.duration_s

        a = np.array([[3. * t2, 4. * t3],
                      [6. * self.duration_s, 12. * t2]])

        b = np.array([self.target_velocity - p_init_d - p_init_dd * self.duration_s,
                      - p_init_dd])

        x = np.linalg.solve(a, b)

        return np.array([p_init, p_init_d, .5 * p_init_dd, x[0], x[1], 0.])

