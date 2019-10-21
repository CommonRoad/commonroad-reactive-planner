__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.1"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Alpha"

# commonroad imports
from commonroad_rp.parameter import SamplingParameters, VehModelParameters
from commonroad_rp.trajectories import TrajectoryBundle, TrajectorySample, CartesianSample, CurviLinearSample,DefaultCostFunction
from commonroad_rp.parameter import parameter_velocity_reaching
from commonroad_rp.polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from commonroad_rp.utils import *


from commonroad.common.validity import *
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.trajectory import Trajectory, State

import pycrcc
from pycrccosy import TrapezoidCoordinateSystem
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline

# python packages
from typing import List
import matplotlib.pyplot as plt
import numpy as np
import time, warnings
import os
import cProfile


class ReactivePlanner(object):
    def __init__(self, dt: float, t_h: float, N: int, v_desired=14, collision_check_in_cl: bool = False,
                 lanelet_network: LaneletNetwork = None):

        assert is_positive(dt), '<ReactivePlanner>: provided dt is not correct! dt = {}'.format(dt)
        assert is_positive(N) and is_natural_number(N), '<ReactivePlanner>: provided N is not correct! dt = {}'.format(
            N)
        assert is_positive(t_h), '<ReactivePlanner>: provided t_h is not correct! dt = {}'.format(dt)
        assert np.isclose(t_h, N * dt), '<ReactivePlanner>: Provided time horizon information is not correct!'
        assert np.isclose(N * dt, t_h)

        # Set horizon variables
        self.horizon = t_h
        self.N = N
        self.dT = dt

        # Create default VehModelParameters
        self.constraints = VehModelParameters()
        # Set width and length
        self._width = self.constraints.veh_width
        self._length = self.constraints.veh_length

        # Current State
        self.x_0: State = None

        # Set reference
        self._cosy: TrapezoidCoordinateSystem = None
        self._reference: np.ndarray = None
        self._ref_pos: np.ndarray = None
        self._ref_curv: np.ndarray = None
        self._theta_ref: np.ndarray = None
        self._ref_curv_d: np.ndarray = None

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
        max_v = max(min_v + 1.0, self._desired_speed)
        steps = 4
        self._sampling_v = SamplingParameters(min_v, max_v, steps)
        self._setup_sampling_sets()
        return self._sampling_v

    def set_reference_path(self, reference_path: np.ndarray):
        self._co: CoordinateSystem = CoordinateSystem(reference_path)
        #cosy: TrapezoidCoordinateSystem = create_coordinate_system_from_polyline(reference_path)
        # Set reference
        #self._cosy = cosy
        #self._reference = reference_path
        #self._ref_pos = compute_pathlength_from_polyline(reference_path)
        #self._ref_curv = compute_curvature_from_polyline(reference_path)
        #theta_ref = compute_orientation_from_polyline(self._reference)
        #self._theta_ref = theta_ref
        #self._ref_curv_d = np.gradient(self._ref_curv, self._ref_pos)

    def set_reference_lane(self, lane_direction: int) -> None:
        """
        compute new reference path based on relative lane position.
        :param lane_direction: 0=curernt lane >0=right lanes, <0=left lanes
        :return:
        """
        assert self.lanelet_network is not None, \
            'lanelet network must be provided during initialization for using get_reference_of_lane().'

        current_ids = self.lanelet_network.find_lanelet_by_position([self.x_0.position])[0]
        if len(current_ids) > 0:
            current_lanelet = self.lanelet_network.find_lanelet_by_id(current_ids[0])
        else:
            if self._cosy is not None:
                warnings.warn('set_reference_lane: x0 is not located on any lane, use previous reference.',
                              stacklevel=2)
            else:
                raise ValueError(
                    'set_reference_lane: x0 is not located on any lane and no previous reference available.')
            return

        # determine target lane
        target_lanelet = None
        if lane_direction == -1:
            if current_lanelet.adj_left_same_direction not in (False, None):
                target_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet.adj_left)
        elif lane_direction == 1:
            if current_lanelet.adj_right_same_direction not in (False, None):
                try:
                    target_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet.adj_right)
                except:
                    i = 0
        elif lane_direction == 0:
            target_lanelet = current_lanelet

        if target_lanelet is None:
            warnings.warn(
                'set_reference_lane: No adjacent lane in direction {}, stay in current lane.'.format(lane_direction),
                stacklevel=2)
            target_lanelet = current_lanelet
        else:
            print('<reactive_planner> Changed reference lanelet from {} to {}.'.format(current_lanelet.lanelet_id,
                                                                                       target_lanelet.lanelet_id))

        # TODO: currently only using first lane, integrate high level planner
        lanes = Lanelet.all_lanelets_by_merging_successors_from_lanelet(target_lanelet, self.lanelet_network)
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
            v_interval = self._sampling_v.to_range(i + 1)
            d_interval = self._sampling_d.to_range(i + 1)  # search with d=0 first
            # if start is self.dT => often results in singularities in linalg
            if i == 0:
                t_interval = np.array([self._sampling_t.up])  # to_range(i+1)
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

        # reset cost statistic
        self._min_cost = 10 ** 9
        self._max_cost = 0

        trajectories = list()
        for t in self._t_sets[samp_level]:

            # Longitudinal sampling for all possible velocities
            for v in self._v_sets[samp_level]:
                trajectory_long = QuarticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lon), x_d=np.array([v, 0]))
                jerk_cost = trajectory_long.squared_jerk_integral(t) / t
                time_cost = 1.0 / t
                distance_cost = (desired_speed - v) ** 2
                trajectory_long.set_cost(jerk_cost, time_cost, distance_cost,
                                         params.k_jerk_lon, params.k_time, params.k_distance)

                # Sample lateral end states (add x_0_lat to sampled states)
                for d in self._d_sets[samp_level].union({x_0_lat[0]}):
                    end_state_lat = np.array([d, 0.0, 0.0])
                    # SWITCHING TO POSITION DOMAIN FOR LATERAL TRAJECTORY PLANNING
                    s_lon_goal = trajectory_long.evaluate_state_at_tau(t)[0] - x_0_lon[0]
                    trajectory_lat = QuinticTrajectory(tau_0=0, delta_tau=s_lon_goal, x_0=np.array(x_0_lat),
                                                       x_d=end_state_lat)
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
                        trajectory_sample = TrajectorySample(self.horizon, self.dT, trajectory_long, trajectory_lat)
                        trajectories.append(trajectory_sample)

        # perform pre check and order trajectories according their cost
        trajectory_bundle = TrajectoryBundle(trajectories, params, cost_function=DefaultCostFunction(self._desired_speed))

        return trajectory_bundle



    def draw_trajectory_set(self, trajectory_bundle: List[TrajectorySample], step=2):
        """
        Draws the current feasible trajectory set
        :param step: Choose if you only want to plot every "step" trajectory => default 2
        :return:
        """
        if trajectory_bundle is not None:

            for i in range(0, len(trajectory_bundle), step):
                # trajectory = _compute_cartesian_trajectory(trajectory_bundle[i], self.dT, self._ref_pos,self._ref_curv,self._re)
                #color = (trajectory_bundle[i].total_cost - self._min_cost) / (self._max_cost - self._min_cost)
                #color = (color, color, color)
                color = 'gray'
                plt.plot(trajectory_bundle[i].cartesian.x, trajectory_bundle[i].cartesian.y,
                         color=color)
                # plt.show()

    def _compute_initial_states(self, x_0: State) -> (np.ndarray, np.ndarray):
        """
        Computes the initial states for the polynomial planner based on a TrajPoint object
        :param x_0: The TrajPoint object representing the initial state of the vehicle
        :return: A tuple containing the initial longitudinal and lateral states (lon,lat)
        """

        # compute curvilinear position
        s, d = self._co.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])
        # compute orientation in curvilinear coordinate frame
        theta_cl = x_0.orientation - np.interp(s, self._co.ref_pos(), self._co.ref_theta())
        # compute curvatures
        kr = np.interp(s, self._co.ref_pos(), self._co.ref_curv())
        kr_d = np.interp(s, self._co.ref_pos(), self._co.ref_curv_d())

        # compute d prime and d prime prime -> derivation after arclength
        d_p = (1 - kr * d) * np.tan(theta_cl)
        d_pp = -(kr_d * d + kr * d_p) * np.tan(theta_cl) + ((1 - kr * d) / (np.cos(theta_cl) ** 2)) * (
                x_0.yaw_rate * ((1 - kr * d) / (np.cos(theta_cl))) - kr)

        # compute s dot and s dot dot -> derivation after time
        s_d = x_0.velocity * np.cos(theta_cl) / (1 - np.interp(s, self._co.ref_pos(), self._co.ref_curv()) * d)
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

        # optimal trajectory dummy
        optimal_trajectory = None

        # initial index of sampling set to use
        i = 0

        # sample until trajectory has been found or sampling sets are empty
        while optimal_trajectory is None and i < self._sampling_level:
            print('<ReactivePlanner>: Starting at sampling density {} of {}'.format(i + 1, self._sampling_level))
            print('<ReactivePlanner>: Sampling {} trajectories'.format(self.no_of_samples(i)))
            # plan trajectory bundle
            bundle = self._create_trajectory_bundle(self._desired_speed, x_0_lon, x_0_lat, samp_level=i)

            # get optimal trajectory
            t0 = time.time()
            #profiler = cProfile.Profile()
            #profiler.enable()
            optimal_trajectory = self._get_optimal_trajectory(bundle, cc)
            print('Checked trajectories in {} seconds'.format(time.time()-t0))
            #profiler.disable()
            #profiler.print_stats("time")
            #self.draw_trajectory_set(bundle.trajectories)
            #plt.pause(50)
            # print statistics
            print('<ReactivePlanner>: Rejected {} infeasible trajectories due to kinematics'.format(
                self.no_of_infeasible_trajectories_kinematics()))
            print('<ReactivePlanner>: Rejected {} infeasible trajectories due to collisions'.format(
                self.no_of_infeasible_trajectories_collision()))

            i = i + 1
        if optimal_trajectory is None and x_0.velocity <= 0.1:
            optimal_trajectory = self._compute_standstill_trajectory(x_0,x_0_lon,x_0_lat)

        # check if feasible trajectory exists -> emergency mode
        if optimal_trajectory is None:
            print('<ReactivePlanner>: Could not find any trajectory out of {} trajectories'.format(
                sum([self.no_of_samples(i) for i in range(self._sampling_level)])))
            print('<ReactivePlanner>: Cannot find trajectory with default sampling parameters. '
                  'Switching to emergency mode!')
        else:
            print('Found optimal trajectory with costs = {}, which corresponds to {} percent of seen costs'.format(
                optimal_trajectory.cost,
                ((optimal_trajectory.cost - bundle.min_costs().cost) / (bundle.max_costs().cost - bundle.min_costs().cost))))


        return self._compute_trajectory_pair(optimal_trajectory) if not bundle.empty() else None

    def _compute_standstill_trajectory(self, x_0, x_0_lon, x_0_lat) -> TrajectorySample:
        # create artifical standstill trajectory
        print('Adding standstill trajectory')
        traj_lon = QuarticTrajectory(tau_0=0,delta_tau=self.horizon,x_0=x_0_lon,x_d=np.array([self._desired_speed, 0]))
        traj_lat = QuinticTrajectory(tau_0=0,delta_tau=self.horizon,x_0=x_0_lat,x_d=np.array([x_0_lat[0],0,0]))
        p = TrajectorySample(self.horizon,self.dT,traj_lon,traj_lat)
        p.cartesian = CartesianSample(np.repeat(x_0.position[0], self.N), np.repeat(x_0.position[1], self.N),
                                      np.repeat(x_0.orientation, self.N), np.repeat(0, self.N),
                                      np.repeat(0, self.N), np.repeat(0, self.N), np.repeat(0, self.N))
        p.curvilinear = CurviLinearSample(np.repeat(x_0_lon[0], self.N), np.repeat(x_0_lat[0], self.N),
                                          np.repeat(x_0.orientation, self.N), np.repeat(x_0_lat[1], self.N),
                                          np.repeat(x_0_lat[2], self.N), np.repeat(x_0_lon[1], self.N),
                                          np.repeat(x_0_lon[2], self.N))
        return p

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
            cart_states['position'] = np.array([trajectory.cartesian.x[i], trajectory.cartesian.y[i]])
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

            lon_list.append([trajectory.curvilinear.s[i], trajectory.curvilinear.s_dot[i], trajectory.curvilinear.s_ddot[i]])
            lat_list.append([trajectory.curvilinear.d[i], trajectory.curvilinear.d_dot[i], trajectory.curvilinear.d_ddot[i]])

        cartTraj = Trajectory(0, cart_list)
        freTraj = Trajectory(0, cl_list)

        return (cartTraj, freTraj, lon_list, lat_list)

    def check_kinematics(self, trajectory: TrajectorySample) -> bool:
        """
        Checks the kinematics of given trajectory and computes the cartesian trajectory information
        :param trajectory: The trajectory to check
        :return: True if the trajectory is feasible and false otherwise
        """

        # constants
        _LOW_VEL_MODE = True



        #time_array = list()
        #time_array.append(np.arange(0, self.horizon+self.dT, self.dT))
        #time_array.append(np.square(time_array[0]))
        #time_array.append(time_array[1] * time_array[0])
        #time_array.append(np.square(time_array[1]))
        #time_array.append(time_array[3] * time_array[0])

        # lookup
        #idx = np.argmin(time_array[0] < trajectory.trajectory_long.delta_tau)

        # create time array and precompute time interval information
        t = np.arange(0, trajectory.trajectory_long.delta_tau + self.dT, self.dT)
        t2 = np.square(t)
        t3 = t2 * t
        t4 = np.square(t2)
        t5 = t4 * t

        # compute position, velocity, acceleration from trajectory sample
        s = trajectory.trajectory_long.calc_position(t, t2, t3, t4, t5)  # lon pos
        s_velocity = trajectory.trajectory_long.calc_velocity(t, t2, t3, t4)  # lon velocity
        s_acceleration = trajectory.trajectory_long.calc_acceleration(t, t2, t3)  # lon acceleration

        # At low speeds, we have to sample the lateral motion over the travelled distance rather than time.
        if _LOW_VEL_MODE:
            # compute normalized travelled distance for low velocity mode of lateral planning
            s1 = s - s[0]
            s2 = np.square(s1)
            s3 = s2 * s1
            s4 = np.square(s2)
            s5 = s4 * s1

            d = trajectory.trajectory_lat.calc_position(s1, s2, s3, s4, s5)  # lat pos
            d_velocity = trajectory.trajectory_lat.calc_velocity(s1, s2, s3, s4)  # lat velocity
            d_acceleration = trajectory.trajectory_lat.calc_acceleration(s1, s2, s3)  # lat acceleration
        else:
            d = trajectory.trajectory_lat.calc_position(t, t2, t3, t4, t5)  # lat pos
            d_velocity = trajectory.trajectory_lat.calc_velocity(t, t2, t3, t4)  # lat velocity
            d_acceleration = trajectory.trajectory_lat.calc_acceleration(t, t2, t3)  # lat acceleration

        # Compute cartesian information of trajectory
        x = np.zeros(len(s))  # [x_0.position[0]]
        y = np.zeros(len(s))  # [x_0.position[1]]
        theta_gl = np.zeros(len(s))  # [x_0.orientation]
        theta_cl = np.zeros(len(s))  # [x_0.orientation - np.interp(s[0], self._ref_pos, self._theta_ref)]
        v = np.zeros(len(s))  # [x_0.v]
        a = np.zeros(len(s))  # [x_0.a]
        kappa_gl = np.zeros(len(s))  # [x_0.kappa]
        kappa_cl = np.zeros(len(s))  # [x_0.kappa - np.interp(s[0], self._ref_pos, self._ref_curv)]
        for i in range(0, len(s)):
            # compute Global position
            try:
                pos = self._co.convert_to_cartesian_coords(s[i], d[i])
            except ValueError:
                # outside of projection domain
                return False
            x[i] = pos[0]
            y[i] = pos[1]

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


            s_idx = np.argmin(np.abs(self._co.ref_pos() - s[i]))
            if self._co.ref_pos()[s_idx] < s[i]:
                s_idx += 1
            s_lambda = (self._co.ref_pos()[s_idx]-s[i])/(self._co.ref_pos()[s_idx+1]-self._co.ref_pos()[s_idx])



            # add cl and gl orientation
            if s_velocity[i] > 0.005:
                if _LOW_VEL_MODE:
                    theta_cl[i] = np.arctan2(dp, 1.0)
                else:
                    theta_cl[i] = np.arctan2(d_velocity[i], s_velocity[i])
                # add global orientation
                #theta_gl.append(theta_cl[-1] + np.interp(s[i], self._co.ref_pos(), self._co.ref_theta()))
                theta_gl[i] = theta_cl[i] + (self._co.ref_theta()[s_idx+1]-self._co.ref_theta()[s_idx])*s_lambda+self._co.ref_theta()[s_idx]
                #print("Orig: = {}".format(np.interp(s[i], self._co.ref_pos(), self._co.ref_theta())))
                #print("New: = {}".format((self._co.ref_theta()[s_idx+1]-self._co.ref_theta()[s_idx])*s_lambda+self._co.ref_theta()[s_idx]))

            else:
                #theta_cl.append(np.interp(s[i], self._co.ref_pos(), self._co.ref_theta()))
                theta_cl[i] = (self._co.ref_theta()[s_idx+1]-self._co.ref_theta()[s_idx])*s_lambda+self._co.ref_theta()[s_idx]
                theta_gl[i] = theta_cl[i]

            # Compute curvature of reference at current position
            #k_r = np.interp(s[i], self._co.ref_pos(), self._co.ref_curv())
            k_r = (self._co.ref_curv()[s_idx+1]-self._co.ref_curv()[s_idx])*s_lambda+self._co.ref_curv()[s_idx]
            #k_r_d = np.interp(s[i], self._co.ref_pos(), self._co.ref_curv_d())
            k_r_d = (self._co.ref_curv_d()[s_idx+1]-self._co.ref_curv_d()[s_idx])*s_lambda+self._co.ref_curv_d()[s_idx]
            # ref_curv_prime = np.gradient(self._ref_curv, self._ref_pos)
            # compute global curvature based on appendix A of Moritz Werling's PhD thesis
            oneKrD = (1 - k_r * d[i])
            cosTheta = np.cos(theta_cl[i])
            tanTheta = np.tan(theta_cl[i])
            kappa = (dpp + k_r * dp * tanTheta) * cosTheta * (cosTheta / oneKrD) ** 2 + (cosTheta / oneKrD) * k_r
            kappa_gl[i] = kappa
            kappa_cl[i] = kappa_gl[i] - k_r

            # velocity
            v[i] = s_velocity[i] * (oneKrD / (np.cos(theta_cl[i])))
            # account for numerical issues #todo: nevertheless, the position might move
            #if v[i] <= 10 ** -1:
            #    v[i] = 0

            # compute acceleration
            a[i] = s_acceleration[i] * oneKrD / cosTheta + ((s_velocity[i] ** 2) / cosTheta) * (
                    oneKrD * tanTheta * (kappa_gl[i] * oneKrD / cosTheta - k_r) - (
                    k_r_d * d[i] + k_r * d_velocity[i]))

            # check kinematics to already discard infeasible trajectories
            if not self._check_vehicle_model(v[i], a[i], kappa_gl[i], (kappa_gl[i] - kappa_gl[i-1]) / self.dT if i > 0 else 0.):
                return False

        # store Cartesian trajectory
        trajectory.cartesian = CartesianSample(x, y, theta_gl, v, a, kappa_gl, np.append([0], np.diff(kappa_gl)))

        # store Curvilinear trajectory
        # theta_cl = trajectory.cartesian.theta - np.interp(trajectory.curvilinear.s, self._ref_pos, self._ref_curv)
        trajectory.curvilinear = CurviLinearSample(s, d, theta_cl, ss=s_velocity, sss=s_acceleration, dd=d_velocity,
                                             ddd=d_acceleration)


        # check if trajectories planning horizon is shorter than expected and extend if necessary # ToDo: int conversion may be wrong
        if self.horizon > trajectory.trajectory_long.delta_tau:
            trajectory.enlarge(int((self.horizon - trajectory.trajectory_long.delta_tau)/self.dT), self.dT)

        return True

    def _check_vehicle_model(self, v: float, a: float, kappa: float, kappa_dot: float):
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

        # Check curvature
        feasible &= abs(kappa) <= self.constraints.kappa_max
        if not feasible:
            if _DEBUG:
                print('Kappa = {}'.format(kappa))
            return False

        # Check kappa_dot
        feasible &= abs(kappa_dot) <= self.constraints.kappa_dot_max
        if not feasible:
            if _DEBUG:
                print('KappaDOT = {}'.format(kappa_dot))
            return False

        # Check velocity
        feasible &= v >= -0.1
        if not feasible:
            if _DEBUG:
                print('Velocity = {}'.format(v))
            return False

        # check accelerations
        feasible &= abs(a) <= self.constraints.a_max
        if _DEBUG :
            print('Acceleration = {}'.format(a))

        return feasible

    def _get_optimal_trajectory(self, trajectory_bundle: TrajectoryBundle, cc: object) -> TrajectorySample:

        # reset statistics
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0

        # check kinematics of each trajectory
        trajectory_list = list()
        #profiler = cProfile.Profile()
        #profiler.enable()
        for trajectory in trajectory_bundle.trajectories:
            # check kinematics of trajectory
            if self.check_kinematics(trajectory):
                trajectory_list.append(trajectory)
            else:
                # write statistics
                self._infeasible_count_kinematics += 1

        #profiler.disable()
        #profiler.print_stats("time")

        # set feasible trajectories in bundle
        trajectory_bundle.trajectories = trajectory_list
        # sort trajectories according to their costs
        trajectory_bundle.sort()

        # go through sorted list of trajectories
        for trajectory in trajectory_bundle.get_sorted_list():
            if self._is_collision_free_trajectory(trajectory, cc):
                return trajectory
            else:
                self._infeasible_count_collision += 1

        return None

    def _is_collision_free_trajectory(self, trajectory: TrajectorySample, cc: object) -> bool:
        """
        Checks a given trajectory for collisions
        :param trajectory: The trajectory to check
        :param cc: The CR collision checker object
        :return: True if collision-free, False otherwise
        """
        pos1 = trajectory.curvilinear.s if self._collision_check_in_cl else trajectory.cartesian.x
        pos2 = trajectory.curvilinear.d if self._collision_check_in_cl else trajectory.cartesian.y
        theta = trajectory.curvilinear.theta if self._collision_check_in_cl else trajectory.cartesian.theta

        # check each pose for collisions
        for i in range(len(pos1)):
            if cc.collide(pycrcc.RectOBB(0.5 * self._length, 0.5 * self._width, theta[i], pos1[i], pos2[i])):
                return False

        return True


    def convert_cr_trajectory_to_object(self, trajectory: Trajectory):
        """
        Converts a CR trajectory to a CR dynamic obstacle with given dimensions
        :param trajectory: The trajectory of the vehicle
        :param length: The length of the vehicle's rectangular shape
        :param width: The width of the vehicle's rectangular shape
        :return:
        """

        # get shape of vehicle
        shape = Rectangle(self._length, self._width)

        # get trajectory prediction
        prediction = TrajectoryPrediction(trajectory, shape)

        return DynamicObstacle(42, ObstacleType.CAR, shape, trajectory.state_list[0], prediction)
