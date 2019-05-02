from parameter import VehicleParameter
from trajectory_bundle import TrajectoryBundle, TrajectorySample, CartesianSample, CurviLinearSample
from parameter import parameter_velocity_reaching
from polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from parameter import PlanningParameter
from commonroad.common.validity import *
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.trajectory import Trajectory,State
import pycrcc
from pycrccosy import TrapezoidCoordinateSystem
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_object
import commonroad_ccosy.visualization.draw_dispatch
from commonroad_cc.visualization.draw_dispatch import draw_object as draw_cc
from typing import List
import matplotlib.pyplot as plt
import numpy as np
import time, warnings


__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM"]
__version__ = "1.0"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Alpha"


def compute_orientation_from_polyline(polyline: npy.ndarray) -> npy.ndarray:
    """
    Computes the orientations along a given polyline
    :param polyline: The polyline to check
    :return: The orientations along the polyline
    """
    assert isinstance(polyline, npy.ndarray) and len(polyline) > 1 and polyline.ndim == 2 and len(
        polyline[0, :]) == 2, '<Math>: not a valid polyline. polyline = {}'.format(polyline)

    if (len(polyline) < 2):
        raise NameError('Cannot create orientation from polyline of length < 2')

    orientation = [0]
    for i in range(1, len(polyline)):
        pt1 = polyline[i - 1]
        pt2 = polyline[i]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    return orientation


def compute_curvature_from_polyline(polyline: npy.ndarray) -> npy.ndarray:
    """
    Computes the curvature of a given polyline
    :param polyline: The polyline for the curvature computation
    :return: The curvature of the polyline
    """
    assert isinstance(polyline, npy.ndarray) and polyline.ndim == 2 and len(
        polyline[:, 0]) > 2, 'Polyline malformed for curvature computation p={}'.format(polyline)
    x_d = np.gradient(polyline[:, 0])
    x_dd = np.gradient(x_d)
    y_d = np.gradient(polyline[:, 1])
    y_dd = np.gradient(y_d)

    return (x_d * y_dd - x_dd * y_d) / ((x_d ** 2 + y_d ** 2) ** (3. / 2.))


def compute_pathlength_from_polyline(polyline: npy.ndarray) -> npy.ndarray:
    """
    Computes the pathlength of a given polyline
    :param polyline: The polyline
    :return: The pathlength of the polyline
    """
    assert isinstance(polyline, npy.ndarray) and polyline.ndim == 2 and len(
        polyline[:, 0]) > 2, 'Polyline malformed for pathlenth computation p={}'.format(polyline)
    distance = np.zeros((len(polyline),))
    for i in range(1, len(polyline)):
        distance[i] = distance[i - 1] + npy.linalg.norm(polyline[i] - polyline[i - 1])

    return npy.array(distance)


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
        if divmod(self.n_samples*sampling_factor,2) == 0:
            samples = self.n_samples*sampling_factor + 1
        else:
            samples = self.n_samples * sampling_factor + 1

        if sampling_factor == 0:
            return np.array([(self.up+self.low)/2])
        else:
            return np.linspace(self.low, self.up, samples)

    def no_of_samples(self) -> int:
        """
        Returns the number of elements in the range of this sampling parameters object
        :return: The number of elements in the range
        """
        return len(self.to_range())


class VehModelParameters:
    """
    Class that represents the vehicle's constraints
    """
    def __init__(self, a_max, theta_dot_max, kappa_max, kappa_dot_max):
        self.a_max = a_max
        self.theta_dot_max = theta_dot_max
        self.kappa_max = kappa_max
        self.kappa_dot_max = kappa_dot_max


class ReactivePlanner(object):
    def __init__(self, dt: float, t_h: float, N: int, width: float = 2.0, length: float = 5.2,
                 v_desired=14, collision_check_in_cl: bool = False, lanelet_network:LaneletNetwork=None):
        assert is_positive(dt), '<ReactivePlanner>: provided dt is not correct! dt = {}'.format(dt)
        assert is_positive(N) and is_natural_number(N), '<ReactivePlanner>: provided N is not correct! dt = {}'.format(
            N)
        assert is_positive(t_h), '<ReactivePlanner>: provided t_h is not correct! dt = {}'.format(dt)
        assert np.isclose(t_h, N * dt), '<ReactivePlanner>: Provided time horizon information is not correct!'
        assert np.isclose(N*dt,t_h)

        # Set horizon variables
        self.horizon = t_h
        self.N = N
        self.dT = dt

        # Set width and length
        self._width = width
        self._length = length

        # Create default VehModelParameters
        self.constraints = VehModelParameters(VehicleParameter.acceleration_max, 0.2, 0.2, 10)

        # Current State
        self.x_0:State = None

        # Set reference
        self._cosy:TrapezoidCoordinateSystem = None
        self._reference:np.ndarray = None
        self._ref_pos:np.ndarray = None
        self._ref_curv:np.ndarray = None
        self._theta_ref:np.ndarray = None
        self._ref_curv_d:np.ndarray = None

        # lanelet_network
        self.lanelet_network: LaneletNetwork = lanelet_network

        # store feasible trajectories of last run
        self._feasible_trajectories = None
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0
        self._min_cost = None
        self._max_cost = None

        # store collision checker coordinate system information
        self._collision_check_in_cl = collision_check_in_cl

        # sampling levels
        self._sampling_level = 4

        # store desired velocity
        self._desired_speed = v_desired
        self._desired_d = 0.
        self._desired_t = self.horizon

        # Default sampling -> [desired - min,desired + max,initial step]
        self._sampling_d = SamplingParameters(-.5, .5, 5)
        self._sampling_t = SamplingParameters(3 * self.dT, self.horizon, 3)
        self._sampling_v = self.set_desired_speed(v_desired)

        # compute sampling sets
        self._setup_sampling_sets()

    def set_desired_speed(self, v_desired):
        self._desired_speed = v_desired
        if self.x_0 is not None:
            reference_speed = self.x_0.velocity
        else:
            reference_speed = self._desired_speed


        min_v = max(0, reference_speed - (self.horizon) * self.constraints.a_max)
        max_v = max(min_v+1.0,self._desired_speed)
        steps = 4
        self._sampling_v = SamplingParameters(min_v, max_v, steps)
        self._setup_sampling_sets()
        return self._sampling_v

    def set_reference_path(self, reference_path:np.ndarray):
        cosy: TrapezoidCoordinateSystem = create_coordinate_system_from_polyline(reference_path)
        # Set reference
        self._cosy = cosy
        self._reference = reference_path
        self._ref_pos = compute_pathlength_from_polyline(reference_path)
        self._ref_curv = compute_curvature_from_polyline(reference_path)
        theta_ref = compute_orientation_from_polyline(self._reference)
        self._theta_ref = theta_ref
        self._ref_curv_d = np.gradient(self._ref_curv, self._ref_pos)

    def set_reference_lane(self, lane_direction: int) -> None:
        """
        compute new reference path based on relative lane position.
        :param lane_direction: 0=curernt lane >0=right lanes, <0=left lanes
        :return:
        """
        assert self.lanelet_network is not None,\
            'lanelet network must be provided during initialization for using get_reference_of_lane().'

        current_ids = self.lanelet_network.find_lanelet_by_position([self.x_0.position])[0]
        if len(current_ids) > 0:
            current_lanelet = self.lanelet_network.find_lanelet_by_id(current_ids[0])
        else:
            if self._cosy is not None:
                warnings.warn('set_reference_lane: x0 is not located on any lane, use previous reference.', stacklevel=2)
            else:
                raise ValueError('set_reference_lane: x0 is not located on any lane and no previous reference available.')
            return

        # determine target lane
        target_lanelet = None
        if lane_direction==-1:
            if current_lanelet.adj_left_same_direction not in (False,None):
                target_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet.adj_left)
        elif lane_direction == 1:
            if current_lanelet.adj_right_same_direction not in (False,None):
                try:
                    target_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet.adj_right)
                except:
                    i=0
        elif lane_direction == 0:
            target_lanelet = current_lanelet

        if target_lanelet is None:
            warnings.warn('set_reference_lane: No adjacent lane in direction {}, stay in current lane.'.format(lane_direction), stacklevel=2)
            target_lanelet = current_lanelet
        else:
            print('<reactive_planner> Changed reference lanelet from {} to {}.'.format(current_lanelet.lanelet_id, target_lanelet.lanelet_id))

        #TODO: currently only using first lane, integrate high level planner
        lanes = Lanelet.all_lanelets_by_merging_successors_from_lanelet(target_lanelet,self.lanelet_network)
        if len(lanes) > 1:
            reference = lanes[0].center_vertices
        elif len(lanes) > 0:
            reference = lanes[0].center_vertices
        else:
            reference = target_lanelet.center_vertices

        self.set_reference_path(reference)


    def _setup_sampling_sets(self):
        """
        Sets up the sampling sets for planning (needs further refining)
        """

        v_sets = list()
        d_sets = list()
        t_sets = list()
        for i in range(self._sampling_level):
            v_interval = self._sampling_v.to_range(i+1)
            d_interval = self._sampling_d.to_range(i+1) # search with d=0 first
            # if start is self.dT => often results in singularities in linalg
            if i==0:
                t_interval = np.array([self._sampling_t.up])#to_range(i+1)
            else:
                t_interval = self._sampling_t.to_range(i)

            # make sets and compute difference
            vs = set(v_interval.flatten())
            ds = set(d_interval.flatten())
            ts = set(t_interval.flatten())

            # for j in range(i):
            #     # vs = vs - v_sets[j]
            #     ds = ds - d_sets[j]
            #     # ts = ts - t_sets[j]

            v_sets.append(vs)
            d_sets.append(ds)
            t_sets.append(ts)

        self._v_sets = v_sets
        self._d_sets = d_sets
        self._t_sets = t_sets

    def no_of_samples(self, samp_level: int) -> int:
        """
        Returns the number of samples for a given sampling level
        :param samp_level: The sampling level
        :return: Number of trajectory samples for given sampling level
        """
        return len(self._v_sets[samp_level]) * len(self._d_sets[samp_level]) * len(self._t_sets[samp_level])

    def no_of_infeasible_trajectories_collision(self) -> int:
        """
        Returns the number of infeasible trajectories due to collision
        :return: Number of colliding trajectories
        """
        return self._infeasible_count_collision

    def no_of_infeasible_trajectories_kinematics(self) -> int:
        """
        Returns the number of infeasible trajectories due to kinematics
        :return: Number of kinematically infeasible trajectories
        """
        return self._infeasible_count_kinematics

    def _extend_trajectory(self, s, d, s_dot, theta, v, a, duration, dT) -> tuple:
        """
        Extends a trajectory assuming constant motion
        :param s: Longitudinal position
        :param d: Lateral position
        :param s_dot: Longitudinal velocity
        :param theta: Orientation
        :param v: Velocity
        :param a: Acceleration
        :param duration: Duration of extension
        :param dT: Time step of extension
        :return: Tuple (s,d,theta,v,a)
        """
        # compute time array
        t = np.arange(0, duration + dT, dT)
        s_n = s + s_dot * t
        d_n = d + v * np.sin(theta) * t  # np.repeat(d,len(t))
        theta_n = np.repeat(theta, len(t))
        v_n = np.repeat(v, len(t))
        a_n = np.repeat(a, len(t))

        return (s_n, d_n, theta_n, v_n, a_n)

    def _create_trajectory_bundle(self, desired_speed: float, x_0_lon: np.array,
                                  x_0_lat: np.array, samp_level: int) -> TrajectoryBundle:
        """
        Plans trajectory samples that try to reach a certain velocity and samples in this domain.
        Sample in time (duration) and velocity domain. Initial state is given. Longitudinal end state (s) is sampled.
        Lateral end state (d) is always set to 0.
        :param desired_speed: Reference velocity. Deviations from that are penalized in the cost function
        :param x_0_lon: np.array([s, s_dot, s_ddot])
        :param x_0_lat: np.array([d, d_dot, d_ddot])
        :param samp_level: index of the sampling parameter set to use
        :return: trajectory bundle with all sample trajectories.

        NOTE: Here, no collision or feasibility check is done!
        """

        # get parameters for planning
        params = parameter_velocity_reaching()

        # create trajectory bundle object
        trajectory_bundle = TrajectoryBundle(params)

        # reset cost statistic
        self._min_cost = 10 ** 9
        self._max_cost = 0


        for t in self._t_sets[samp_level]:

            # Longitudinal sampling for all possible velocities
            for v in self._v_sets[samp_level]:
                trajectory_long = QuarticTrajectory(t_start_s=0, duration_s=t, desired_horizon=self.horizon,
                                                    start_state=x_0_lon, target_velocity=v,
                                                    desired_velocity=self._desired_speed)
                jerk_cost = trajectory_long.squared_jerk_integral(t) / t
                time_cost = 1.0 / t
                distance_cost = (desired_speed - v) ** 2
                trajectory_long.set_cost(jerk_cost, time_cost, distance_cost,
                                         params.k_jerk_lon, params.k_time, params.k_distance)

                # Sample lateral end states (add x_0_lat to sampled states)
                for d in self._d_sets[samp_level].union({x_0_lat[0]}):
                    end_state_lat = np.array([d, 0.0, 0.0])
                    # SWITCHING TO POSITION DOMAIN FOR LATERAL TRAJECTORY PLANNING
                    s_lon_goal = trajectory_long.calc_position_at(t) - x_0_lon[0]
                    trajectory_lat = QuinticTrajectory(t_start_s=0, duration_s=s_lon_goal, desired_horizon=self.horizon,
                                                       start_state=x_0_lat, end_state=end_state_lat)
                    if trajectory_lat.coeffs is not None:

                        jerk_cost = trajectory_lat.squared_jerk_integral(t) / t
                        time_cost = 0  # 1.0/t
                        distance_cost = d ** 2
                        trajectory_lat.set_cost(jerk_cost, time_cost, distance_cost,
                                                params.k_jerk_lat, params.k_time, 50 * params.k_distance)

                        # Create trajectory sample and add it to trajectory bundle
                        trajectory_cost = params.k_long * trajectory_long.cost + params.k_lat * trajectory_lat.cost

                        # store costs
                        if trajectory_cost < self._min_cost:
                            self._min_cost = trajectory_cost

                        if trajectory_cost > self._max_cost:
                            self._max_cost = trajectory_cost
                        trajectory_sample = TrajectorySample(self.dT, trajectory_long, trajectory_lat, trajectory_cost)
                        trajectory_bundle.add_trajectory(trajectory_sample)

        return trajectory_bundle

    def _check_vehicle_model(self, v, a, theta_dot, kappa, kappa_dot):
        """
        Checks if a given state is feasible according to the kinematic single track model (simplified)
        :param v: Is provided velocity >= 0
        :param a: Is provided abs(acceleration) <= 8
        :param theta: Is provided orientation in range
        :param kappa: Is provided curvature in range
        :param kappa_dot: Is provided curvature change in range
        :return: True if feasible, false otherwise
        """
        _DEBUG = False

        feasible = True

        # Check orientation
        feasible &= np.abs(theta_dot) <= self.constraints.theta_dot_max / self.dT
        if _DEBUG and not feasible:
            print('Theta_dot = {}'.format(theta_dot))
            return False

        # Check curvature
        feasible &= np.abs(kappa) <= self.constraints.kappa_max
        if _DEBUG and not feasible:
            print('Kappa = {}'.format(kappa))
            return False

        # Check kappa_dot
        feasible &= np.abs(kappa_dot) <= self.constraints.kappa_dot_max
        if _DEBUG and not feasible:
            print('KappaDOT = {}'.format(kappa_dot))
            return False

        # Check velocity
        feasible &= v >= -0.0
        if _DEBUG and not feasible:
            print('Velocity = {}'.format(v))
            return False

        # check accelerations
        feasible &= np.abs(a) <= self.constraints.a_max
        if _DEBUG and not feasible:
            print('Acceleration = {}'.format(a))
            return False

        return feasible

    def _check_kinematics(self, trajectory: TrajectorySample) -> bool:
        """
        Checks the kinematics of given trajectory and computes the cartesian trajectory information
        :param trajectory: The trajectory to check
        :return: True if the trajectory is feasible and false otherwise
        """

        # constants
        _LOW_VEL_MODE = True

        # create time array
        t = np.arange(0, trajectory.trajectory_long.duration_s + self.dT, self.dT)

        # precompute time interval information
        t2 = np.square(t)
        t3 = t2 * t
        t4 = np.square(t2)
        t5 = t4 * t
        # compute position, velocity, acceleration
        s = trajectory.trajectory_long.calc_position(t, t2, t3, t4, t5)  # lon pos
        s_velocity = trajectory.trajectory_long.calc_velocity(t, t2, t3, t4)  # lon velocity
        s_acceleration = trajectory.trajectory_long.calc_acceleration(t, t2, t3)  # lon acceleration

        s1 = s - s[0]
        s2 = np.square(s1)
        s3 = s2 * s1
        s4 = np.square(s2)
        s5 = s4 * s1

        # d = trajectory.trajectory_lat.calc_position(t, t2, t3, t4, t5)  # lat pos
        d = trajectory.trajectory_lat.calc_position(s1, s2, s3, s4, s5)  # lat pos
        # d_velocity = trajectory.trajectory_lat.calc_velocity(t, t2, t3, t4)  # lat velocity
        d_velocity = trajectory.trajectory_lat.calc_velocity(s1, s2, s3, s4)  # lat velocity
        # d_acceleration = trajectory.trajectory_lat.calc_acceleration(t, t2, t3)  # lat acceleration
        d_acceleration = trajectory.trajectory_lat.calc_acceleration(s1, s2, s3)  # lat acceleration

        x = list()  # [x_0.position[0]]
        y = list()  # [x_0.position[1]]
        theta_gl = list()  # [x_0.orientation]
        theta_cl = list()  # [x_0.orientation - np.interp(s[0], self._ref_pos, self._theta_ref)]
        v = list()  # [x_0.v]
        a = list()  # [x_0.a]
        kappa_gl = list()  # [x_0.kappa]
        kappa_cl = list()  # [x_0.kappa - np.interp(s[0], self._ref_pos, self._ref_curv)]
        for i in range(0, len(s)):
            # compute Global position
            try:
                pos = self._cosy.convert_to_cartesian_coords(s[i], d[i])
            except ValueError:
                # outside of projection domain
                return False
            x.append(pos[0])
            y.append(pos[1])

            # compute orientations
            dp = None
            dpp = None
            if not _LOW_VEL_MODE:
                if s_velocity[i] > 0.001:
                    dp = d_velocity[i] / s_velocity[i]
                else:
                    if d_velocity[i] > 0.001:
                        dp = None
                    else:
                        dp = 0.
                ddot = d_acceleration[i] - dp * s_acceleration[i]

                if s_velocity[i] > 0.001:
                    d_pp = ddot / (s_velocity[i] ** 2)
                else:
                    if np.abs(ddot) > 0.00003:
                        dpp = None
                    else:
                        dpp = 0.
            else:
                dp = d_velocity[i]
                dpp = d_acceleration[i]

            # add cl orientation
            if s_velocity[i] > 0.005:
                if _LOW_VEL_MODE:
                    theta_cl.append(np.arctan2(dp, 1.0))
                else:
                    theta_cl.append(np.arctan2(d_velocity[i], s_velocity[i]))
            else:
                theta_cl.append(np.interp(s[i],self._ref_pos,self._theta_ref))

            # add global orientation
            theta_gl.append(theta_cl[-1] + np.interp(s[i], self._ref_pos, self._theta_ref))

            # Kappa
            k_r = np.interp(s[i], self._ref_pos, self._ref_curv)
            k_r_d = np.interp(s[i], self._ref_pos, self._ref_curv_d)
            # ref_curv_prime = np.gradient(self._ref_curv, self._ref_pos)
            oneKrD = (1 - k_r * d[i])
            cosTheta = np.cos(theta_cl[-1])
            tanTheta = np.tan(theta_cl[-1])
            kappa = (dpp + k_r * dp * tanTheta) * cosTheta * (cosTheta / oneKrD) ** 2 + (cosTheta / oneKrD) * k_r
            kappa_gl.append(kappa)
            kappa_cl.append(kappa_gl[-1] - k_r)

            # velocity
            v.append(s_velocity[i] * (oneKrD / (np.cos(theta_cl[-1]))))
            if v[-1] <= 10 ** -1:
                v[-1] = 0

            # a.append((v[-1] - v[-2]) / self.dT)

            a.append(s_acceleration[i] * oneKrD / cosTheta + ((s_velocity[i] ** 2) / cosTheta) * (
                        oneKrD * tanTheta * (kappa_gl[-1] * oneKrD / cosTheta - k_r) - (
                            k_r_d * d[i] + k_r * d_velocity[i])))

            # check kinematics to already discard infeasible trajectories
            if not self._check_vehicle_model(v[-1], a[-1],
                                             (theta_gl[-1] - theta_gl[-2]) / self.dT if len(theta_gl) > 1 else 0.,
                                             kappa_gl[-1],
                                             (kappa_gl[-1] - kappa_gl[-2]) / self.dT if len(kappa_gl) > 1 else 0.):
                return False

        # convert to array
        v = np.array(v)
        a = np.array(a)

        # check if trajectories planning horizon is shorter than expected
        if trajectory.trajectory_long.desired_horizon > trajectory.trajectory_long.duration_s:
            # extend trajectory
            s_n, d_n, theta_n, v_n, a_n = self._extend_trajectory(s[-1], d[-1], s_velocity[-1], theta_cl[-1], v[-1],
                                                                  a[-1],
                                                                  trajectory.trajectory_long.desired_horizon - trajectory.trajectory_long.duration_s,
                                                                  self.dT)
            # transform to cartesian
            x_n = list()
            y_n = list()
            for i in range(len(s_n)):
                try:
                    pos = self._cosy.convert_to_cartesian_coords(s_n[i], d_n[i])
                    x_n.append(pos[0])
                    y_n.append(pos[1])
                except Exception as e:
                    print(e)
                    return False

            # store curvilinear extension
            trajectory.ext_curvilinear = CurviLinearSample(s_n, d_n, theta_n)
            # store Cartesian extension
            trajectory.ext_cartesian = CartesianSample(x_n, y_n, np.repeat(theta_gl[-1], len(s_n)), v_n, a_n, 0, 0)

        # store Cartesian trajectory
        trajectory.cartesian = CartesianSample(x, y, theta_gl, v, a, kappa_gl, np.append([0], np.diff(kappa_gl)))

        # store Curvilinear trajectory
        # theta_cl = trajectory.cartesian.theta - np.interp(trajectory.curvilinear.s, self._ref_pos, self._ref_curv)
        trajectory.curvilinear = CurviLinearSample(s, d, theta_cl, ss=s_velocity, sss=s_acceleration, dd=d_velocity,
                                                   ddd=d_acceleration)

        # Trajectory is feasible
        return True

    def draw_trajectory_set(self, trajectory_bundle: List[TrajectorySample], step=2):
        """
        Draws the current feasible trajectory set
        :param step: Choose if you only want to plot every "step" trajectory => default 2
        :return:
        """
        if trajectory_bundle is not None:

            for i in range(0, len(trajectory_bundle), step):
                # trajectory = _compute_cartesian_trajectory(trajectory_bundle[i], self.dT, self._ref_pos,self._ref_curv,self._re)
                color = (trajectory_bundle[i].total_cost - self._min_cost) / (self._max_cost - self._min_cost)
                color = (color, color, color)
                color = 'gray'
                plt.plot(trajectory_bundle[i].cartesian.x, trajectory_bundle[i].cartesian.y,
                         color=color)
                if trajectory_bundle[i].ext_cartesian is not None:
                    plt.plot(trajectory_bundle[i].ext_cartesian.x,
                             trajectory_bundle[i].ext_cartesian.y,
                             color=color)
                # plt.show()

    def _compute_initial_states(self, x_0: State) -> (np.ndarray, np.ndarray):
        """
        Computes the initial states for the polynomial planner based on a TrajPoint object
        :param x_0: The TrajPoint object representing the initial state of the vehicle
        :return: A tuple containing the initial longitudinal and lateral states (lon,lat)
        """

        # compute curvilinear position
        s, d = self._cosy.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])
        # compute orientation in curvilinear coordinate frame
        theta_cl = x_0.orientation - np.interp(s, self._ref_pos, self._theta_ref)
        # compute curvatures
        kr = np.interp(s, self._ref_pos, self._ref_curv)
        kr_d = np.interp(s, self._ref_pos, self._ref_curv_d)

        # compute d prime and d prime prime -> derivation after arclength
        d_p = (1 - kr * d) * np.tan(theta_cl)
        d_pp = -(kr_d * d + kr * d_p) * np.tan(theta_cl) + ((1 - kr * d) / (np.cos(theta_cl) ** 2)) * (
                x_0.yaw_rate * ((1 - kr * d) / (np.cos(theta_cl))) - kr)

        # compute s dot and s dot dot -> derivation after time
        s_d = x_0.velocity * np.cos(theta_cl) / (1 - np.interp(s, self._ref_pos, self._ref_curv) * d)
        s_dd = x_0.acceleration
        s_dd -= (s_d ** 2 / np.cos(theta_cl)) * (
                    (1 - kr * d) * np.tan(theta_cl) * (x_0.yaw_rate * ((1 - kr * d) / (np.cos(theta_cl)) - kr)) - (
                        kr_d * d + kr * d_p))
        s_dd /= ((1 - kr * d) / (np.cos(theta_cl)))

        # d_d =(1-kr*d)*np.tan(theta_cl) #x_0.v * np.sin(theta_cl)

        x_0_lon = [s, s_d, s_dd]
        x_0_lat = [d, d_p, d_pp]

        print('Initial state is {}'.format(x_0))
        print('Initial x_0 lon = {}'.format(x_0_lon))
        print('Initial x_0 lat = {}'.format(x_0_lat))

        return (x_0_lon, x_0_lat)


    def plan(self, x_0: State, cc: object, cl_states=None) -> tuple:
        """
        Plans an optimal trajectory
        :param x_0: Initial state as CR state
        :param cc:  CollisionChecker object
        :param cl_states: Curvilinear state if replanning is used
        :param lane_change_direction: 0=current lane >0=right lanes, <0=left lanes
        :return: Optimal trajectory as tuple
        """
        self.x_0 = x_0

        # compute initial states
        if cl_states is not None:
            x_0_lon, x_0_lat = cl_states
        else:
            x_0_lon, x_0_lat = self._compute_initial_states(x_0)

        print('<Reactive Planner>: initial state is: lon = {} / lat = {}'.format(x_0_lon, x_0_lat))

        # create empty bundle
        bundle = TrajectoryBundle(None)
        # initial index of sampling set to use
        i = 0

        # sample until trajectory has been found or sampling sets are empty
        while bundle.empty() and i < self._sampling_level:
            print('<ReactivePlanner>: Starting at sampling density {} of {}'.format(i + 1, self._sampling_level))
            print('<ReactivePlanner>: Sampling {} trajectories'.format(self.no_of_samples(i)))
            # plan trajectory bundle
            bundle = self._create_trajectory_bundle(self._desired_speed, x_0_lon, x_0_lat, samp_level=i)

            bundle_old = bundle

            # check feasibility
            bundle = self._get_feasible_trajectories(bundle, cc, x_0)

            # print statistics
            print('<ReactivePlanner>: Rejected {} infeasible trajectories due to kinematics'.format(
                self.no_of_infeasible_trajectories_kinematics()))
            print('<ReactivePlanner>: Rejected {} infeasible trajectories due to collisions'.format(
                self.no_of_infeasible_trajectories_collision()))

            i = i + 1
            if bundle.empty() and x_0.velocity <= 0.1:
                # create artifical standstill trajectory
                print('Adding standstill trajectory')
                traj_lon = QuarticTrajectory(t_start_s=0, duration_s=self.horizon, desired_horizon=self.horizon,
                                                    start_state=x_0_lon, target_velocity=v,
                                                    desired_velocity=self._desired_speed)
                traj_lat = QuinticTrajectory(t_start_s=0, duration_s=s_lon_goal, desired_horizon=self.horizon,
                                                       start_state=x_0_lat, end_state=end_state_lat)
                p = TrajectorySample(0, traj_lon, traj_lat, 0)
                p.cartesian = CartesianSample(np.repeat(x_0.position[0], self.N), np.repeat(x_0.position[1], self.N),
                                              np.repeat(x_0.orientation, self.N), np.repeat(0, self.N),
                                              np.repeat(0, self.N), np.repeat(0, self.N), np.repeat(0, self.N))
                p.curvilinear = CurviLinearSample(np.repeat(x_0_lon[0], self.N), np.repeat(x_0_lat[0], self.N),
                                                  np.repeat(x_0.orientation, self.N), np.repeat(x_0_lat[1], self.N),
                                                  np.repeat(x_0_lat[2], self.N), np.repeat(x_0_lon[1], self.N),
                                                  np.repeat(x_0_lon[2], self.N))
                bundle.add_trajectory(p)
                bundle_old = bundle

        # check if feasible trajectory exists -> emergency mode
        if bundle.empty():
            print('<ReactivePlanner>: Could not find any trajectory out of {} trajectories'.format(
                sum([self.no_of_samples(i) for i in range(self._sampling_level)])))
            self._feasible_trajectories = bundle_old.trajectory_bundle
            print('<ReactivePlanner>: Cannot find trajectory with default sampling parameters. '
                  'Switching to emergency mode!')

        # store feasible trajectories
        self._feasible_trajectories = bundle.trajectory_bundle if not bundle.empty() else None

        # find optimal trajectory
        optimal_trajectory = bundle.updated_optimal_trajectory() if not bundle.empty() else None
        if not bundle.empty():
            self._min_cost = bundle_old.min_costs()
            self._max_cost = bundle_old.max_costs()
            # self.draw_trajectory_set(self._feasible_trajectories)
            print('Found optimal trajectory with costs = {}, which corresponds to {} percent of seen costs'.format(
                optimal_trajectory.total_cost,
                ((optimal_trajectory.total_cost - self._min_cost) / (self._max_cost - self._min_cost))))

        return self._compute_trajectory_pair(optimal_trajectory) if not bundle.empty() else None

    def _compute_trajectory_pair(self, trajectory: TrajectorySample) -> tuple:
        """
        Computes the output required for visualizing in CommonRoad framework
        :param trajectory: the optimal trajectory
        :return: (CartesianTrajectory, FrenetTrajectory, lon sample, lat sample)
        """
        # go along state list
        cart_list = list()
        cl_list = list()

        lon_list = list()
        lat_list = list()
        for i in range(len(trajectory.cartesian.x)):

            # create Cartesian state
            cart_states = dict()
            cart_states['time_step'] = i
            cart_states['position'] = np.array([trajectory.cartesian.x[i],trajectory.cartesian.y[i]])
            cart_states['velocity'] = trajectory.cartesian.v[i]
            cart_states['acceleration'] = trajectory.cartesian.a[i]
            cart_states['orientation'] = trajectory.cartesian.theta[i]
            cart_states['yaw_rate'] = trajectory.cartesian.kappa[i]
            cart_list.append(State(**cart_states))

            # create curvilinear state
            cl_states = dict()
            cl_states['time_step'] = i
            cl_states['position'] = np.array([trajectory.curvilinear.s[i], trajectory.curvilinear.d[i]])
            cl_states['velocity'] = trajectory.cartesian.v[i]
            cl_states['acceleration'] = trajectory.cartesian.a[i]
            cl_states['orientation'] = trajectory.cartesian.theta[i]
            cl_states['yaw_rate'] = trajectory.cartesian.kappa[i]
            cl_list.append(State(**cl_states))

            lon_list.append([trajectory.curvilinear.s[i], trajectory.curvilinear.ss[i], trajectory.curvilinear.sss[i]])
            lat_list.append([trajectory.curvilinear.d[i], trajectory.curvilinear.dd[i], trajectory.curvilinear.ddd[i]])

        cartTraj = Trajectory(0, cart_list)
        freTraj = Trajectory(0, cl_list)

        return (cartTraj, freTraj, lon_list, lat_list)

    def _get_feasible_trajectories(self, trajectory_bundle: TrajectoryBundle, cc: object,
                                   x_0: State) -> TrajectoryBundle:
        """
        Checks the feasibility of the trajectories within a trajectory bundle
        :param trajectory_bundle: The trajectory bundle to check
        :return: The set of feasible trajectories
        """

        trajectories = trajectory_bundle.trajectory_bundle

        # Create new trajectory bundle
        feasible_trajectories = TrajectoryBundle(trajectory_bundle.params)

        print('<ReactivePlanner>: Checking {} trajectories for feasibility!'.format(len(trajectories)))

        # reset statistics
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0


        t0 = time.time()

        # Check feasibility of trajectories
        for traj in trajectories:
            if self._feasible_trajectory(traj, cc):
                feasible_trajectories.add_trajectory(traj)

        t1 = time.time()
        total = t1 - t0

        print('<ReactivePlanner>: Found {} feasible trajectories in {} s!'.format(
            len(feasible_trajectories.trajectory_bundle), total))

        return feasible_trajectories

    def _feasible_trajectory(self, trajectory: TrajectorySample, cc: object) -> bool:
        """
        Checks the validity of a trajectory, i.e., it is collision-free and respects kinematic constraints
        :return:
        """

        feas = True

        # check kinematic constraints
        feas &= self._check_kinematics(trajectory)
        if not feas:
            self._infeasible_count_kinematics += 1

        # only check for collisions (expensive operation) if trajectory is kinematically feasible
        if feas:
            # check trajectory for collisions
            feas &= self._check_for_collisions(trajectory, cc)
            if not feas:
                self._infeasible_count_collision += 1

        return feas

    def _check_for_collisions(self, trajectory: TrajectorySample, cc: object) -> bool:
        """
        Checks a given trajectory for collisions
        :param trajectory: The trajectory to check
        :param cc: The CR collision checker object
        :return: True if collision-free, False otherwise
        """
        # create a collision object using the trajectory prediction of the ego vehicle
        co = pycrcc.TimeVariantCollisionObject(0)

        pos1 = trajectory.curvilinear.s if self._collision_check_in_cl else trajectory.cartesian.x
        pos2 = trajectory.curvilinear.d if self._collision_check_in_cl else trajectory.cartesian.y
        theta = trajectory.curvilinear.theta if self._collision_check_in_cl else trajectory.cartesian.theta

        for i in range(len(pos1)):
            co.append_obstacle(pycrcc.RectOBB(0.5 * self._length, 0.5 * self._width, theta[i], pos1[i], pos2[i]))

        # check if trajectory has been extended and check that for collisions as well:
        if trajectory.ext_cartesian is not None:
            pos1 = trajectory.ext_curvilinear.s if self._collision_check_in_cl else trajectory.ext_cartesian.x
            pos2 = trajectory.ext_curvilinear.d if self._collision_check_in_cl else trajectory.ext_cartesian.y
            theta = trajectory.ext_curvilinear.theta if self._collision_check_in_cl else trajectory.ext_cartesian.theta

            for i in range(len(pos1)):
                co.append_obstacle(pycrcc.RectOBB(0.5 * self._length, 0.5 * self._width, theta[i], pos1[i], pos2[i]))

        return not cc.collide(co)

    def convert_cr_trajectory_to_object(self, trajectory: Trajectory):
        """
        Converts a CR trajectory to a CR dynamic obstacle with given dimensions
        :param trajectory: The trajectory of the vehicle
        :param length: The length of the vehicle's rectangular shape
        :param width: The width of the vehicle's rectangular shape
        :return:
        """

        # get shape of vehicle
        shape = Rectangle(self._length,self._width)

        # get trajectory prediction
        prediction = TrajectoryPrediction(trajectory,shape)

        return DynamicObstacle(42, ObstacleType.CAR,shape, trajectory.state_list[0],prediction)




if __name__ == '__main__':
    print('Creating velocity reaching bundle....')

    crfr = CommonRoadFileReader('/home/klischat/GIT_REPOS/testCaseOpt/scenarios_border_obstacles/2018b/ZAM_Over-1_1_border_obstacles_shell.xml')
    scenario, _ = crfr.open()
    plt.figure(figsize=(25, 10))
    draw_object(scenario)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(0.1)

    # create coordinate system
    reference_path = scenario.lanelet_network.find_lanelet_by_id(1000).center_vertices
    curvilinear_cosy = create_coordinate_system_from_polyline(reference_path)

    # create collision checker for scenario
    collision_checker = create_collision_checker(scenario)

    # convert coordinates and create initial state
    x, y = curvilinear_cosy.convert_to_cartesian_coords(25, 0)
    x_0 = State(**{'position':np.array([x,y]),'orientation':0.04, 'velocity':10, 'acceleration':0,'yaw_rate':0})

    planner:ReactivePlanner = ReactivePlanner(0.2, 6, 30)
    planner.set_reference_path(reference_path)

    x_cl = None

    for k in range(0, 18):
        optimal = planner.plan(x_0, collision_checker, cl_states=x_cl)
        # convert to CR obstacle
        ego = planner.convert_cr_trajectory_to_object(optimal[0])
        draw_object(ego)
        draw_object(ego.prediction.occupancy_at_time_step(1))
        plt.pause(0.1)

        x_0 = optimal[0].state_list[1]
        x_cl = (optimal[2][1], optimal[3][1])

        print("Goal state is: {}".format(optimal[1].state_list[-1]))

    print('Done')
    plt.show(block=True)
