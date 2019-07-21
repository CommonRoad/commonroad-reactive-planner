from parameter import VehicleParameter, PlanningParameter
from trajectory_bundle import TrajectoryBundle, TrajectorySample, CartesianSample, CurviLinearSample
from polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from commonroad.common.validity import *
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.trajectory import Trajectory,State
import pycrcc
from pycrccosy import TrapezoidCoordinateSystem
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
from typing import List
import matplotlib.pyplot as plt
import numpy as np
import time, warnings
from parameter_classes import VehModelParameters, SamplingParameters
from polyline import compute_curvature_from_polyline, compute_orientation_from_polyline, compute_pathlength_from_polyline
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, StaticObstacle


__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM"]
__version__ = "1.0"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Alpha"



class ReactivePlanner(object):
    def __init__(self, v_desired=14, collision_check_in_cl: bool = False, lanelet_network:LaneletNetwork=None):

        params = PlanningParameter(velocity_reaching=True)
        self.set_parameters(params)

        vehicle_params = VehicleParameter()
        self.set_parameters(vehicle_params)

        # Set time sampling variables
        self.horizon = params.prediction_horizon
        self.dT = params.t_step_size
        self.N = params.t_N

        assert is_positive(self.dT), '<ReactivePlanner>: provided dt is not correct! dt = {}'.format(self.dT)
        assert is_positive(self.horizon), '<ReactivePlanner>: provided t_h is not correct! dt = {}'.format(self.dT)

        assert is_positive(self.N) and is_natural_number(self.N), '<ReactivePlanner>: provided N is not correct! dt = {}'.format(
            self.N)
        #assert np.isclose(self.horizon, self.N * self.dT), '<ReactivePlanner>: Provided time horizon information is not correct!'
        #assert np.isclose(self.N * self.dT, self.horizon)

        # Set direction sampling options
        self.d_deviation = params.d_deviation
        self.d_N = params.d_N

        # Set width and length
        self._width = vehicle_params.width
        self._length = vehicle_params.length

        # Create default VehModelParameters
        self.constraints = VehModelParameters(vehicle_params.acceleration_max, vehicle_params.acceleration_dot_max, vehicle_params.kappa_max, vehicle_params.kappa_dot_max)

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
        self._infeasible_count_behavior = 0
        self._infeasible_count_kinematics = 0
        self._min_cost = None
        self._max_cost = None

        # store collision checker coordinate system information
        self._collision_check_in_cl = collision_check_in_cl

        # sampling levels
        self._sampling_level = 4

        # store desired velocity
        self._desired_speed = v_desired
        self._desired_d = 0
        self._desired_t = self.horizon

        # Default sampling -> [desired - min,desired + max,initial step]
        self._sampling_d = SamplingParameters(-self.d_deviation, self.d_deviation, self.d_N)
        self._sampling_t = SamplingParameters(params.t_min, self.horizon, self.N)
        self._sampling_v = self.set_desired_speed(v_desired)

        # compute sampling sets
        self._setup_sampling_sets()

        # switch between low and high velocity mode
        self._velocity_threshold = vehicle_params.velocity_threshold

    def set_parameters(self, parameters):
        """
        Set vehicle and planning parameters of the planner to plan trajectories for different vehicle and sampling settings
        :param parameters: VehicleParameter or PlanningParameter Object
        """
        if type(parameters).__name__ == PlanningParameter:
            self.params = parameters

        elif type(parameters).__name__ == VehicleParameter:
            self.params_vehicle = parameters


    def set_desired_speed(self, v_desired):
        """
        Sets desired velocity and calculates velocity for each sample
        :param v_desired: velocity in m/s
        :return: velocity in m/s
        """
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
        """
        Sets internal parameters for reference path
        :param reference_path: reference path
        :return: none
        """
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
        params = PlanningParameter(velocity_reaching = True)

        # create empty trajectory bundle object
        trajectory_bundle = TrajectoryBundle()

        # reset cost statistic
        self._min_cost = 10 ** 9
        self._max_cost = 0

        for t in self._t_sets[samp_level]:

            # Quintic longitudinal sampling over all possible velocities
            for v in self._v_sets[samp_level]:

                trajectory_long = QuinticTrajectory(t_start_s=0, duration_s=t, desired_horizon=self.horizon,
                                                     start_state=x_0_lon, end_state=end_state_lon,
                                                    desired_velocity=self._desired_speed)
                # set costs for sampled longitudinal trajectory sample
                time_cost = 1.0 / t
                distance_cost = (desired_speed - v) ** 2
                end_state_lon = np.array([t * v + x_0_lon[0], v, 0.0])
                jerk_cost = trajectory_long.squared_jerk_integral(t) / t
                trajectory_long.set_cost(jerk_cost, time_cost, distance_cost,
                                         params.k_jerk_lon, params.k_time, params.k_distance)

                # Sample lateral end states (add x_0_lat to sampled states)
                for d in self._d_sets[samp_level].union({x_0_lat[0]}):
                    end_state_lat = np.array([d, 0.0, 0.0])

                    # Switch to sampling over s for low velocities (non-holonomic behaviour)
                    if trajectory_long.calc_velocity_at(trajectory_long.duration_s) < self._velocity_threshold:
                        s_lon_goal = trajectory_long.calc_position_at(t) - x_0_lon[0]
                        trajectory_lat = QuinticTrajectory(t_start_s=0, duration_s=s_lon_goal,
                                                           desired_horizon=self.horizon,
                                                           start_state=x_0_lat, end_state=end_state_lat)
                    # Switch to sampling over t for high velocities
                    else:
                        trajectory_lat = QuinticTrajectory(t_start_s=0, duration_s=t,
                                                           desired_horizon=self.horizon,
                                                           start_state=x_0_lat, end_state=end_state_lat)

                    if trajectory_lat.coeffs is not None:

                        # set costs for sampled lateral trajectory sample
                        jerk_cost = trajectory_lat.squared_jerk_integral(t) / t
                        time_cost = 0  # 1.0/t
                        distance_cost = d ** 2
                        trajectory_lat.set_cost(jerk_cost, time_cost, distance_cost,
                                                params.k_jerk_lat, params.k_time, 50 * params.k_distance)

                        # Create trajectory sample and add it to trajectory bundle
                        trajectory_cost = params.k_long * trajectory_long.cost + params.k_lat * trajectory_lat.cost

                        # store all costs
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
    '''
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
                    dpp = ddot / (s_velocity[i] ** 2)
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
    '''

    def _check_kinematics(self, trajectory: TrajectorySample) -> bool:
        """
        Checks the kinematics of given trajectory
        :param trajectory: The trajectory to check with cartesian information
        :return: True if the trajectory is feasible and false otherwise
        """
        v = trajectory.cartesian.v
        a = trajectory.cartesian.a
        theta_gl = trajectory.cartesian.theta
        kappa_gl = trajectory.cartesian.kappa
        s = trajectory.curvilinear.s
        for i in range(0, len(s)):
            if not self._check_vehicle_model(v[i], a[i],
                                             (theta_gl[i] - theta_gl[i - 1]) / self.dT if i + 1 > 1 else 0.,
                                             kappa_gl[i],
                                             (kappa_gl[i] - kappa_gl[i - 1]) / self.dT if i + 1 > 1 else 0.):
                return False
        return True

    def _compute_cartesian_trajectory(self, trajectory: TrajectorySample):
        """
        Computes the cartesian trajectory information
        :param trajectory: The trajectory to compute
        :return: Computed trajectory
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
        for i in range(0, len(s)):
            # compute Global position
            try:
                pos = self._cosy.convert_to_cartesian_coords(s[i], d[i])
            except ValueError:
                # outside of projection domain
                return None
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
                    dpp = ddot / (s_velocity[i] ** 2)
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
                theta_cl.append(np.interp(s[i], self._ref_pos, self._theta_ref))

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
            # kappa_cl.append(kappa_gl[-1] - k_r)

            # velocity
            v.append(s_velocity[i] * (oneKrD / (np.cos(theta_cl[-1]))))
            if v[-1] <= 10 ** -1:
                v[-1] = 0

            # a.append((v[-1] - v[-2]) / self.dT)

            a.append(s_acceleration[i] * oneKrD / cosTheta + ((s_velocity[i] ** 2) / cosTheta) * (
                    oneKrD * tanTheta * (kappa_gl[-1] * oneKrD / cosTheta - k_r) - (
                    k_r_d * d[i] + k_r * d_velocity[i])))

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
                    # print(e)
                    return None
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
        return trajectory

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
        :param x_0: Initial state as CR state (CR = cartesian)
        :param cc:  CollisionChecker object
        :param cl_states: Curvilinear state if replanning is used
        :return: Optimal trajectory as tuple or None, if no trajectory is found.
        """
        self.x_0 = x_0

        # compute initial states
        if cl_states is not None:
            x_0_lon, x_0_lat = cl_states
        else:
            x_0_lon, x_0_lat = self._compute_initial_states(x_0)

        print('<Reactive Planner>: initial state is: lon = {} / lat = {}'.format(x_0_lon, x_0_lat))

        # create empty bundle
        bundle = TrajectoryBundle()
        # initial index of sampling set to use
        i = 0

        # sample until trajectory has been found or sampling_level exceeds its limit
        while i < self._sampling_level:
            print('<ReactivePlanner>: Starting at sampling density {} of {}'.format(i + 1, self._sampling_level))
            print('<ReactivePlanner>: Sampling {} trajectories'.format(self.no_of_samples(i)))

            bundle = self._create_trajectory_bundle(self._desired_speed, x_0_lon, x_0_lat, samp_level=i)
            bundle_old = bundle
            val = TrajectoryBundle()

            i = i+1
            # store only valid trajectories with computed cartesian coordinates
            if not bundle.empty():
                for traj in bundle.trajectory_bundle:
                    tmp = self._compute_cartesian_trajectory(traj)
                    if tmp is not None:
                        tmp.reevaluate_costs()
                        val.add_trajectory(tmp)

            # sort valid trajectories by their costs
            val.trajectory_bundle = sorted(val.trajectory_bundle, key=lambda traj: traj.total_cost)

            # iterate over valid trajectories and return first feasible trajectory
            for traj in val.trajectory_bundle:
                if self._feasible_trajectory(traj, cc):
                    self._min_cost=bundle_old.min_costs()
                    self._max_cost=bundle_old.max_costs()
                    print(
                        'Found optimal trajectory with costs = {}, which corresponds to {} percent of seen costs'.format(
                            traj.total_cost,
                            ((traj.total_cost - self._min_cost) / (self._max_cost - self._min_cost))))
                    return self._compute_trajectory_pair(traj)

            # add standstill trajectory
            bundle = TrajectoryBundle()
            if x_0.velocity <= 0.1:
                # create artifical standstill trajectory
                print('Adding standstill trajectory')
                traj_lon = QuarticTrajectory(t_start_s=0, duration_s=self.horizon, desired_horizon=self.horizon,
                                                    start_state=x_0_lon,
                                                    desired_velocity=self._desired_speed)
                traj_lat = QuinticTrajectory(t_start_s=0, desired_horizon=self.horizon,
                                                       start_state=x_0_lat)
                p = TrajectorySample(0, traj_lon, traj_lat, 0)
                p.cartesian = CartesianSample(np.repeat(x_0.position[0], self.N), np.repeat(x_0.position[1], self.N),
                                              np.repeat(x_0.orientation, self.N), np.repeat(0, self.N),
                                              np.repeat(0, self.N), np.repeat(0, self.N), np.repeat(0, self.N))
                p.curvilinear = CurviLinearSample(np.repeat(x_0_lon[0], self.N), np.repeat(x_0_lat[0], self.N),
                                                  np.repeat(x_0.orientation, self.N), np.repeat(x_0_lat[1], self.N),
                                                  np.repeat(x_0_lat[2], self.N), np.repeat(x_0_lon[1], self.N),
                                                  np.repeat(x_0_lon[2], self.N))
                p = self._compute_cartesian_trajectory(p)
                p.reevaluate_costs()
                bundle.add_trajectory(p)
                bundle_old = bundle
                if self._feasible_trajectory(p, cc):
                    self._min_cost=bundle_old.min_costs()
                    self._max_cost=bundle_old.max_costs()
                    print(
                        'Found optimal trajectory with costs = {}, which corresponds to {} percent of seen costs'.format(
                            p.total_cost,
                            ((p.total_cost - self._min_cost) / (self._max_cost - self._min_cost))))
                    return self._compute_trajectory_pair(p)

        # check if feasible trajectory exists -> emergency mode
        if bundle.empty():
            print('<ReactivePlanner>: Could not find any trajectory out of {} trajectories'.format(
                sum([self.no_of_samples(i) for i in range(self._sampling_level)])))
            self._feasible_trajectories = bundle_old.trajectory_bundle
            print('<ReactivePlanner>: Cannot find trajectory with default sampling parameters. '
                  'Switching to emergency mode!')

        # store feasible trajectories (and trajectories with higher cost than optimal trajectory)
        # Problem: not all trajectories are checked for feasibility
        #self._feasible_trajectories = bundle.trajectory_bundle if not bundle.empty() else None

        return None

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
        Checks the feasibility of the trajectories within a trajectory bundle for collisions and vehicle kinematics
        :param trajectory_bundle: The trajectory bundle to check
        :return: The set of feasible trajectories
        """

        trajectories = trajectory_bundle.trajectory_bundle

        # Create new trajectory bundle
        feasible_trajectories = TrajectoryBundle()

        print('<ReactivePlanner>: Checking {} trajectories for feasibility!'.format(len(trajectories)))

        # reset statistics
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0
        self._infeasible_count_behavior = 0


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

        if feas:
            # check trajectory for traffic rule violation
            feas &= self._check_driving_behavior(trajectory)
            if not feas:
                self._infeasible_count_behavior += 1

        # only check for collisions (expensive operation) if trajectory is kinematically feasible
        if feas:
            # check trajectory for collisions
            feas &= self._check_for_collisions(trajectory, cc)
            if not feas:
                self._infeasible_count_collision += 1

        return feas

    def _check_driving_behavior(self, trajectory: TrajectorySample) -> bool:
        """
        Checks a given trajectory for violation of traffic rules and behavior (e.g. roadkeeping, velcoity)
        :param trajectory: The trajectory to check
        :return: True if appropriate driving behavior, False otherwise
        """

        assert self.lanelet_network is not None, \
            'lanelet network must be provided during initialization for using get_reference_of_lane().'

        violation = False

        # Pos to check leaving lanelet in cartesian space
        traj_pos_cart = np.column_stack((trajectory.cartesian.x, trajectory.cartesian.y))
        tray_in_any_lanelet = np.full(len(traj_pos_cart), False)

        v_max = np.max(trajectory.cartesian.v)
        cart_course = trajectory.cartesian.theta
        adj_candidate = list(self.lanelet_network.lanelets_in_proximity(traj_pos_cart[0, :], 10))

        # Check if every point of trajectory is inside any lanelet
        for k in range(len(adj_candidate)):
            tray_in_any_lanelet = np.logical_or(tray_in_any_lanelet, adj_candidate[k].contains_points(traj_pos_cart))
        boundary_crossed = not all(tray_in_any_lanelet)


        for k in range(len(adj_candidate)):
            if v_max > adj_candidate[k].speed_limit * 1.1:
                print(v_max)
                print('Please slow down!')

        violation = boundary_crossed | violation

        return not violation

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

    def _boundary_not_intersecting(self, scenario, p, own_ID, total_IDs):
        """
        Helper to check if boundary object is not intersecting/blocking any lanelet
        :param scenario:    Scenario object
        :param p:           Array of a point and its predecessor describing a boundary segment
        :param own_ID:      Id of the lanelet corresponding to the boundary
        :param total_IDs:   List of all lanelet Ids to check for intersection
        :return:            Boolean
        """
        n = 50

        total_IDs.remove(own_ID)
        #create list of n-1 points between p[0] and p[1]
        delta_x = p[1][0]-p[0][0]
        delta_y = p[1][1]-p[0][1]
        points = np.array([[p[0][0]+1/n*delta_x, p[0][1]+1/n*delta_y]])
        for i in range(2,n):
            points = np.append(points, np.array([[p[0][0]+i/n*delta_x, p[0][1]+i/n*delta_y]]), axis= 0)

        # checks whether a point between p[0] and p[1] is on other lanelet
        for i in total_IDs:
            if True in scenario.lanelet_network.find_lanelet_by_id(i).contains_points(points):
                return False
        return True


    def add_obstacles_at_lanelet_edges(self, scenario, xml_file):
        """
        Adds Obstacles to the most outer boundaries of the lanelet network to prevent the planner from leaving the
        lanelet network into empty regions
        :param scenario: Scenario object to which lanelet boundaries obstacles should be added
        :param xml_file: Path to the xml File in which the scenario is described
        :return: scenario with boundary obstacles
        """

        from commonroad.common.util import Interval
        import xml.etree.ElementTree as ET
        # find all lanelet IDs
        tree = ET.parse(xml_file)
        root = tree.getroot()
        IDs = []
        for child in root:
            if child.tag == "lanelet":
                IDs.append(int(child.attrib["id"]))
        # add obstacles
        e = 1e-6  # avoid numerical errors
        for i in IDs:
            if scenario.lanelet_network.find_lanelet_by_id(i).adj_left is None:
                old = scenario.lanelet_network.find_lanelet_by_id(i).left_vertices[0];
                for element in scenario.lanelet_network.find_lanelet_by_id(i).left_vertices:
                    delta_x = element[0] - old[0]
                    delta_y = element[1] - old[1]
                    ang = np.arctan(delta_y / (delta_x + e))
                    len_obj = np.sqrt(delta_x ** 2 + delta_y ** 2)
                    if delta_x >= 0:
                        x_obj = element[0] - 0.5 * len_obj * np.cos(np.arctan(np.abs(delta_y / (delta_x + e))))
                        y_obj = element[1] - 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x + e)))
                    else:
                        x_obj = element[0] + 0.5 * len_obj * np.cos(np.arctan(np.abs(delta_y / (delta_x + e))))
                        y_obj = element[1] + 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x + e)))
                    obj = Rectangle(length=len_obj, width=0.05)
                    state = State(position=np.array([x_obj, y_obj]), orientation=ang, velocity=0,
                                  time_step=Interval(0, 30))
                    o_type = ObstacleType
                    obj = StaticObstacle(obstacle_shape=obj, initial_state=state, obstacle_type=o_type.ROAD_BOUNDARY,
                                         obstacle_id=scenario.generate_object_id())
                    if self._boundary_not_intersecting(self, scenario, np.array([element, old]), i, IDs[:]):
                        scenario.add_objects(obj)
                    old = element
            if scenario.lanelet_network.find_lanelet_by_id(i).adj_right is None:
                old = scenario.lanelet_network.find_lanelet_by_id(i).right_vertices[0];
                for element in scenario.lanelet_network.find_lanelet_by_id(i).right_vertices:
                    delta_x = element[0] - old[0]
                    delta_y = element[1] - old[1]
                    ang = np.arctan(delta_y / (delta_x + e))
                    len_obj = np.sqrt(delta_x ** 2 + delta_y ** 2)
                    if delta_x >= 0:
                        x_obj = element[0] - 0.5 * len_obj * np.cos(np.arctan(np.abs(delta_y / (delta_x + e))))
                        y_obj = element[1] - 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x + e)))
                    else:
                        x_obj = element[0] + 0.5 * len_obj * np.cos(np.arctan(np.abs(delta_y / (delta_x + e))))
                        y_obj = element[1] + 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x + e)))
                    # y_obj = element[1] - 0.5 * len_obj * np.sin(np.arctan(delta_y / (delta_x+e)))
                    obj = Rectangle(length=len_obj, width=-0.05)
                    state = State(position=np.array([x_obj, y_obj]), orientation=ang, velocity=0,
                                  time_step=Interval(0, 30))
                    o_type = ObstacleType
                    obj = StaticObstacle(obstacle_shape=obj, initial_state=state, obstacle_type=o_type.ROAD_BOUNDARY,
                                         obstacle_id=scenario.generate_object_id())
                    if self._boundary_not_intersecting(self, scenario, np.array([element, old]), i, IDs[:]):
                        scenario.add_objects(obj)
                    old = element

        return scenario