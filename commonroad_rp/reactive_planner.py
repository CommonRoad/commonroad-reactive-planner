__author__ = "Gerald Würsching, Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.5"
__maintainer__ = "Gerald Würsching"
__email__ = "gerald.wuersching@tum.de"
__status__ = "Beta"

# python packages
import math
import time
import numpy as np
from typing import List
import multiprocessing
from multiprocessing.context import Process

# commonroad-io
from commonroad.common.validity import *
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.scenario.scenario import Scenario

# commonroad_dc
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker, \
    create_collision_object
from commonroad_dc.collision.trajectory_queries.trajectory_queries import trajectory_preprocess_obb_sum, OBBSumException

# commonroad_rp imports
from commonroad_rp.cost_function import DefaultCostFunction
from commonroad_rp.parameter import DefGymSampling, TimeSampling, VelocitySampling, PositionSampling
from commonroad_rp.polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from commonroad_rp.trajectories import TrajectoryBundle, TrajectorySample, CartesianSample, CurviLinearSample
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem, interpolate_angle
from commonroad_rp.configuration import Configuration, VehicleConfiguration


# TODO: introduce mode parameter for longitudinal planning (point following, velocity following, stopping)
# TODO: acceleration-based sampling
# TODO: use logging instead of print statements


class ReactivePlanner(object):
    """
    Reactive planner class that plans trajectories in a sampling-based fashion
    """

    def __init__(self, config: Configuration):
        """
        Constructor of the reactive planner
        : param config: Configuration object holding all planner-relevant configurations
        """
        # Set horizon variables
        self.horizon = config.planning.planning_horizon
        self.dT = config.planning.dt
        self.N = int(self.horizon / self.dT)
        self._factor = config.planning.factor
        self._check_valid_settings()

        # get vehicle parameters from config file
        self.vehicle_params: VehicleConfiguration = config.vehicle

        # Initial State
        self.x_0 = None

        # store sampled trajectory set of last run
        self.stored_trajectories = None

        # initialize internal variables
        # coordinate system & collision checker
        self._co = None
        self._cc = None
        # statistics
        self._total_count = 0
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0
        self._optimal_cost = 0
        self._continuous_cc = config.planning.continuous_cc
        self._collision_check_in_cl = config.planning.collision_check_in_cl
        # desired speed, d and t
        self._desired_speed = None
        self._desired_d = 0.
        self._desired_t = self.horizon
        # Default sampling TODO: Improve initialization of Sampling Set
        fs_sampling = DefGymSampling(self.dT, self.horizon)
        self._sampling_d = fs_sampling.d_samples
        self._sampling_t = fs_sampling.t_samples
        self._sampling_v = fs_sampling.v_samples
        # sampling levels
        self._sampling_level = fs_sampling.max_iteration
        # threshold for low velocity mode
        self._low_vel_mode_threshold = config.planning.low_vel_mode_threshold
        self._LOW_VEL_MODE = False
        # workers for multiprocessing
        self._num_workers = config.debug.num_workers
        # visualize trajectory set
        self._draw_traj_set = config.debug.draw_traj_set
        # Debug mode
        self._DEBUG = config.debug.debug_mode

    def _check_valid_settings(self):
        """Checks validity of provided dt and horizon"""
        assert is_positive(self.dT), '<ReactivePlanner>: provided dt is not correct! dt = {}'.format(self.dT)
        assert is_positive(self.N) and is_natural_number(self.N), '<ReactivePlanner>: N is not correct!'
        assert is_positive(self.horizon), '<ReactivePlanner>: provided t_h is not correct! dt = {}'.format(self.horizon)

    @property
    def collision_checker(self) -> pycrcc.CollisionChecker:
        return self._cc

    @property
    def coordinate_system(self) -> CoordinateSystem:
        return self._co

    @property
    def reference_path(self):
        return self._co.reference

    @property
    def infeasible_count_collision(self):
        """Number of colliding trajectories"""
        return self._infeasible_count_collision

    @property
    def infeasible_count_kinematics(self):
        """Number of kinematically infeasible trajectories"""
        return self._infeasible_count_kinematics

    def set_collision_checker(self, scenario: Scenario):
        """
        Creates the road boundary and creates a collision checker object for a given scenario
        :param scenario: CommonRoad Scenario object
        """
        cc_scenario = pycrcc.CollisionChecker()
        for co in scenario.static_obstacles:
            obs = create_collision_object(co)
            cc_scenario.add_collision_object(obs)
        for co in scenario.dynamic_obstacles:
            tvo = create_collision_object(co)
            if self._continuous_cc:
                tvo, err = trajectory_preprocess_obb_sum(tvo)
                if err == -1:
                    raise OBBSumException("Invalid input for trajectory_preprocess_obb_sum: dynamic "
                                          "obstacle elements overlap")
            cc_scenario.add_collision_object(tvo)
        _, road_boundary_sg_obb = create_road_boundary_obstacle(scenario)
        cc_scenario.add_collision_object(road_boundary_sg_obb)
        self._cc: pycrcc.CollisionChecker = cc_scenario

    def set_reference_path(self, reference_path: np.ndarray):
        """
        Sets the reference path and automatically creates a coordinate system
        :param reference_path: reference path as polyline
        """
        self._co: CoordinateSystem = CoordinateSystem(reference_path)

    def set_t_sampling_parameters(self, t_min, dt, horizon):
        """
        Sets sample parameters of time horizon
        :param t_min: minimum of sampled time horizon
        :param dt: length of each sampled step
        :param horizon: sampled time horizon
        """
        self._sampling_t = TimeSampling(t_min, horizon, self._sampling_level, dt)
        self.N = int(round(horizon / dt))
        self.horizon = horizon

    def set_d_sampling_parameters(self, delta_d_min, delta_d_max):
        """
        Sets sample parameters of lateral offset
        :param delta_d_min: lateral distance lower than reference
        :param delta_d_max: lateral distance higher than reference
        """
        self._sampling_d = PositionSampling(delta_d_min, delta_d_max, self._sampling_level)

    def set_v_sampling_parameters(self, v_min, v_max):
        """
        Sets sample parameters of sampled velocity interval
        :param v_min: minimal velocity sample bound
        :param v_max: maximal velocity sample bound
        """
        self._sampling_v = VelocitySampling(v_min, v_max, self._sampling_level)

    def set_desired_velocity(self, desired_velocity: float, current_speed: float = None, stopping: bool = False):
        """
        Sets desired velocity and calculates velocity for each sample
        :param desired_velocity: velocity in m/s
        :param current_speed: velocity in m/s
        :param stopping
        :return: velocity in m/s
        """
        self._desired_speed = desired_velocity
        if not stopping:
            if current_speed is not None:
                reference_speed = current_speed
            else:
                reference_speed = self._desired_speed

            min_v = max(0, reference_speed - (0.125 * self.horizon * self.vehicle_params.a_max))
            # max_v = max(min_v + 5.0, reference_speed + (0.25 * self.horizon * self.constraints.a_max))
            max_v = max(min_v + 5.0, reference_speed + 2)
            self._sampling_v = VelocitySampling(min_v, max_v, self._sampling_level)
        else:
            self._sampling_v = VelocitySampling(self._desired_speed, self._desired_speed, self._sampling_level)
        if self._DEBUG:
            print('<Reactive_planner>: Sampled interval of velocity: {} m/s - {} m/s'.format(min_v, max_v))

    def _get_no_of_samples(self, samp_level: int) -> int:
        """
        Returns the number of samples for a given sampling level
        :param samp_level: The sampling level
        :return: Number of trajectory samples for given sampling level
        """
        return len(self._sampling_v.to_range(samp_level)) * len(self._sampling_d.to_range(samp_level)) * len(
            self._sampling_t.to_range(samp_level))

    def _create_trajectory_bundle(self, x_0_lon: np.array, x_0_lat: np.array, samp_level: int) -> TrajectoryBundle:
        """
        Plans trajectory samples that try to reach a certain velocity and samples in this domain.
        Sample in time (duration) and velocity domain. Initial state is given. Longitudinal end state (s) is sampled.
        Lateral end state (d) is always set to 0.
        :param x_0_lon: np.array([s, s_dot, s_ddot])
        :param x_0_lat: np.array([d, d_dot, d_ddot])
        :param samp_level: index of the sampling parameter set to use
        :return: trajectory bundle with all sample trajectories.

        NOTE: Here, no collision or feasibility check is done!
        """
        # reset cost statistic
        self._min_cost = 10 ** 9
        self._max_cost = 0

        trajectories = list()
        for t in self._sampling_t.to_range(samp_level):
            # Longitudinal sampling for all possible velocities
            for v in self._sampling_v.to_range(samp_level):
                # end_state_lon = np.array([t * v + x_0_lon[0], v, 0.0])
                # trajectory_long = QuinticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lon), x_d=end_state_lon)
                trajectory_long = QuarticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lon), x_d=np.array([v, 0]))

                # Sample lateral end states (add x_0_lat to sampled states)
                if trajectory_long.coeffs is not None:
                    for d in self._sampling_d.to_range(samp_level).union({x_0_lat[0]}):
                        end_state_lat = np.array([d, 0.0, 0.0])
                        # SWITCHING TO POSITION DOMAIN FOR LATERAL TRAJECTORY PLANNING
                        if self._LOW_VEL_MODE:
                            s_lon_goal = trajectory_long.evaluate_state_at_tau(t)[0] - x_0_lon[0]
                            if s_lon_goal <= 0:
                                s_lon_goal = t
                            trajectory_lat = QuinticTrajectory(tau_0=0, delta_tau=s_lon_goal, x_0=np.array(x_0_lat),
                                                               x_d=end_state_lat)

                        # Switch to sampling over t for high velocities
                        else:
                            trajectory_lat = QuinticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lat),
                                                               x_d=end_state_lat)
                        if trajectory_lat.coeffs is not None:
                            trajectory_sample = TrajectorySample(self.horizon, self.dT, trajectory_long, trajectory_lat)
                            trajectories.append(trajectory_sample)

        # perform pre check and order trajectories according their cost
        trajectory_bundle = TrajectoryBundle(trajectories, cost_function=DefaultCostFunction(self._desired_speed, desired_d=0))
        self._total_count = len(trajectory_bundle._trajectory_bundle)
        if self._DEBUG:
            print('<ReactivePlanner>: %s trajectories sampled' % len(trajectory_bundle._trajectory_bundle))
        return trajectory_bundle

    def _compute_initial_states(self, x_0: State) -> (np.ndarray, np.ndarray):
        """
        Computes the initial states for the polynomial planner based on a CommonRoad state
        :param x_0: The CommonRoad state object representing the initial state of the vehicle
        :return: A tuple containing the initial longitudinal and lateral states (lon,lat)
        """
        # compute curvilinear position
        try:
            s, d = self._co.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])
        except ValueError:
            print('<Reactive_planner>: Value Error for curvilinear transformation')
            tmp = np.array([x_0.position])
            print(x_0.position)
            if self._co._reference[0][0] > x_0.position[0]:
                reference_path = np.concatenate((tmp, self._co._reference), axis=0)
            else:
                reference_path = np.concatenate((self._co._reference, tmp), axis=0)
            self.set_reference_path(reference_path)
            s, d = self._co.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])

        # compute orientation in curvilinear coordinate frame
        ref_theta = np.unwrap(self._co.ref_theta)
        theta_cl = x_0.orientation - np.interp(s, self._co.ref_pos, ref_theta)

        # compute curvatures
        kr = np.interp(s, self._co.ref_pos, self._co.ref_curv)
        kr_d = np.interp(s, self._co.ref_pos, self._co.ref_curv_d)

        # compute d prime and d prime prime -> derivation after arclength
        d_p = (1 - kr * d) * np.tan(theta_cl)
        d_pp = -(kr_d * d + kr * d_p) * np.tan(theta_cl) + ((1 - kr * d) / (math.cos(theta_cl) ** 2)) * (
                x_0.yaw_rate * (1 - kr * d) / math.cos(theta_cl) - kr)

        # compute s dot and s dot dot -> derivation after time
        s_d = x_0.velocity * math.cos(theta_cl) / (1 - kr * d)
        if s_d < 0:
            raise Exception(
                "Initial state or reference incorrect! Curvilinear velocity is negative which indicates that the "
                "ego vehicle is not driving in the same direction as specified by the reference")

        s_dd = x_0.acceleration
        s_dd -= (s_d ** 2 / math.cos(theta_cl)) * (
                (1 - kr * d) * np.tan(theta_cl) * (x_0.yaw_rate * (1 - kr * d) / (math.cos(theta_cl)) - kr) -
                (kr_d * d + kr * d_p))
        s_dd /= ((1 - kr * d) / (math.cos(theta_cl)))

        d_d = x_0.velocity * math.sin(theta_cl)
        d_dd = s_dd * d_p + s_d ** 2 * d_pp

        x_0_lon: List[float] = [s, s_d, s_dd]
        x_0_lat: List[float] = [d, d_d, d_dd]

        if self._DEBUG:
            print("<ReactivePlanner>: Starting planning with: \n#################")
            print(f'Initial state for planning is {x_0}')
            print(f'Initial x_0 lon = {x_0_lon}')
            print(f'Initial x_0 lat = {x_0_lat}')
            print("#################")

        return x_0_lon, x_0_lat

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
            cart_states['time_step'] = self.x_0.time_step+self._factor*i
            cart_states['position'] = np.array([trajectory.cartesian.x[i], trajectory.cartesian.y[i]])
            cart_states['velocity'] = trajectory.cartesian.v[i]
            cart_states['acceleration'] = trajectory.cartesian.a[i]
            cart_states['orientation'] = trajectory.cartesian.theta[i]
            if i > 0:
                cart_states['yaw_rate'] = (trajectory.cartesian.theta[i] - trajectory.cartesian.theta[i-1]) / self.dT
            else:
                cart_states['yaw_rate'] = self.x_0.yaw_rate
            cart_states['steering_angle'] = np.arctan2(self.vehicle_params.wheelbase * cart_states['yaw_rate'],
                                                       cart_states['velocity'])
            cart_list.append(State(**cart_states))

            # create curvilinear state
            # TODO: This is not correct
            cl_states = dict()
            cl_states['time_step'] = self.x_0.time_step+self._factor*i
            cl_states['position'] = np.array([trajectory.curvilinear.s[i], trajectory.curvilinear.d[i]])
            cl_states['velocity'] = trajectory.cartesian.v[i]
            cl_states['acceleration'] = trajectory.cartesian.a[i]
            cl_states['orientation'] = trajectory.cartesian.theta[i]
            cl_states['yaw_rate'] = trajectory.cartesian.kappa[i]
            cl_list.append(State(**cl_states))

            lon_list.append(
                [trajectory.curvilinear.s[i], trajectory.curvilinear.s_dot[i], trajectory.curvilinear.s_ddot[i]])
            lat_list.append(
                [trajectory.curvilinear.d[i], trajectory.curvilinear.d_dot[i], trajectory.curvilinear.d_ddot[i]])

        # make Cartesian and Curvilinear Trajectory
        cartTraj = Trajectory(self.x_0.time_step, cart_list)
        cvlnTraj = Trajectory(self.x_0.time_step, cl_list)

        return cartTraj, cvlnTraj, lon_list, lat_list

    def plan(self, x_0: State, cl_states=None) -> tuple:
        """
        Plans an optimal trajectory
        :param x_0: Initial state as CR state
        :param cl_states: Curvilinear initial states if re-planning is used
        :return: Optimal trajectory as tuple
        """
        # set Cartesian initial state
        self.x_0 = x_0

        # check for low velocity mode
        if self.x_0.velocity < self._low_vel_mode_threshold:
            self._LOW_VEL_MODE = True
        else:
            self._LOW_VEL_MODE = False

        # compute curvilinear initial states
        if cl_states is not None:
            x_0_lon, x_0_lat = cl_states
        else:
            x_0_lon, x_0_lat = self._compute_initial_states(x_0)

        if self._DEBUG:
            print('<Reactive Planner>: initial state is: lon = {} / lat = {}'.format(x_0_lon, x_0_lat))
            print('<Reactive Planner>: desired velocity is {} m/s'.format(self._desired_speed))

        # initialize optimal trajectory dummy
        optimal_trajectory = None

        # initial index of sampling set to use
        i = 1  # Time sampling is not used. To get more samples, start with level 1.

        # sample until trajectory has been found or sampling sets are empty
        while optimal_trajectory is None and i < self._sampling_level:
            if self._DEBUG:
                print('<ReactivePlanner>: Starting at sampling density {} of {}'.format(i + 1, self._sampling_level))

            # sample trajectory bundle
            bundle = self._create_trajectory_bundle(x_0_lon, x_0_lat, samp_level=i)

            # get optimal trajectory
            t0 = time.time()
            optimal_trajectory = self._get_optimal_trajectory(bundle)
            if self._DEBUG:
                print('<ReactivePlanner>: Checked trajectories in {} seconds'.format(time.time() - t0))

            if self._DEBUG:
                print('<ReactivePlanner>: Rejected {} infeasible trajectories due to kinematics'.format(
                    self.infeasible_count_kinematics))
                print('<ReactivePlanner>: Rejected {} infeasible trajectories due to collisions'.format(
                    self.infeasible_count_collision))

            # increase sampling level (i.e., density) if no optimal trajectory could be found
            i = i + 1

        if optimal_trajectory is None and x_0.velocity <= 0.1:
            print('<ReactivePlanner>: planning standstill for the current scenario')
            optimal_trajectory = self._compute_standstill_trajectory(x_0, x_0_lon, x_0_lat)

        # check if feasible trajectory exists -> emergency mode
        if optimal_trajectory is None:
            if self._DEBUG:
                print('<ReactivePlanner>: Could not find any trajectory out of {} trajectories'.format(
                    sum([self._get_no_of_samples(i) for i in range(self._sampling_level)])))
                print('<ReactivePlanner>: Cannot find trajectory with default sampling parameters. '
                      'Switching to emergency mode!')
        else:
            if self._DEBUG:
                print('<ReactivePlanner>: Found optimal trajectory with costs = {}, which '
                      'corresponds to {} percent of seen costs'.format(
                        optimal_trajectory.cost, ((optimal_trajectory.cost - bundle.min_costs().cost) / (
                                bundle.max_costs().cost - bundle.min_costs().cost))))

            self._optimal_cost = optimal_trajectory.cost

        return self._compute_trajectory_pair(optimal_trajectory) if optimal_trajectory is not None else None

    def _compute_standstill_trajectory(self, x_0, x_0_lon, x_0_lat) -> TrajectorySample:
        """
        Computes a standstill trajectory if the vehicle is already at velocity 0
        :param x_0: The current state of the ego vehicle
        :param x_0_lon: The longitudinal state in curvilinear coordinates
        :param x_0_lat: The lateral state in curvilinear coordinates
        :return: The TrajectorySample for a standstill trajectory
        """
        # TODO: Function needs to be updated
        # create artificial standstill trajectory
        if self._DEBUG:
            print('Adding standstill trajectory')
            print("x_0 is {}".format(x_0))
            print("x_0_lon is {}".format(x_0_lon))
            print("x_0_lon is {}".format(type(x_0_lon)))
            for i in x_0_lon:
                print("The element {} of format {} is a real number? {}".format(i, type(i), is_real_number(i)))
            print("x_0_lat is {}".format(x_0_lat))
        traj_lon = QuarticTrajectory(tau_0=0, delta_tau=self.horizon, x_0=np.asarray(x_0_lon),
                                     x_d=np.array([self._desired_speed, 0]))
        traj_lat = QuinticTrajectory(tau_0=0, delta_tau=self.horizon, x_0=np.asarray(x_0_lat),
                                     x_d=np.array([x_0_lat[0], 0, 0]))
        p = TrajectorySample(self.horizon, self.dT, traj_lon, traj_lat)
        p.cartesian = CartesianSample(np.repeat(x_0.position[0], self.N), np.repeat(x_0.position[1], self.N),
                                      np.repeat(x_0.orientation, self.N), np.repeat(0, self.N),
                                      np.repeat(0, self.N), np.repeat(0, self.N), np.repeat(0, self.N))
        p.curvilinear = CurviLinearSample(np.repeat(x_0_lon[0], self.N), np.repeat(x_0_lat[0], self.N),
                                          np.repeat(x_0.orientation, self.N), np.repeat(x_0_lat[1], self.N),
                                          np.repeat(x_0_lat[2], self.N), np.repeat(x_0_lon[1], self.N),
                                          np.repeat(x_0_lon[2], self.N))
        return p

    def _check_kinematics(self, trajectories: List[TrajectorySample], queue_1=None, queue_2=None):
        """
        Checks the kinematics of given trajectories in a bundle and computes the cartesian trajectory information
        Lazy evaluation, only kinematically feasible trajectories are evaluated further

        :param trajectories: The list of trajectory samples to check
        :param queue_1: Multiprocessing.Queue() object for storing feasible trajectories
        :param queue_2: Multiprocessing.Queue() object for storing infeasible trajectories (only vor visualization)
        :return: The list of output trajectories
        """

        # TODO: Remove this Debug parameter
        DEBUG = False

        # initialize lists for output trajectories
        # infeasible trajectory list is only used for visualization when self._draw_traj_set is True
        feasible_trajectories = list()
        infeasible_trajectories = list()

        # loop over list of trajectories
        for trajectory in trajectories:
            # create time array and precompute time interval information
            t = np.arange(0, np.round(trajectory.trajectory_long.delta_tau + self.dT, 5), self.dT)
            t2 = np.square(t)
            t3 = t2 * t
            t4 = np.square(t2)
            t5 = t4 * t

            # initialize long. (s) and lat. (d) state vectors
            s = np.zeros(self.N + 1)
            s_velocity = np.zeros(self.N + 1)
            s_acceleration = np.zeros(self.N + 1)
            d = np.zeros(self.N + 1)
            d_velocity = np.zeros(self.N + 1)
            d_acceleration = np.zeros(self.N + 1)

            # length of the trajectory sample (i.e., number of time steps. can be smaller than planning horizon)
            traj_len = len(t)

            # compute longitudinal position, velocity, acceleration from trajectory sample
            s[:traj_len] = trajectory.trajectory_long.calc_position(t, t2, t3, t4, t5)  # lon pos
            s_velocity[:traj_len] = trajectory.trajectory_long.calc_velocity(t, t2, t3, t4)  # lon velocity
            s_acceleration[:traj_len] = trajectory.trajectory_long.calc_acceleration(t, t2, t3)  # lon acceleration

            # At low speeds, we have to sample the lateral motion over the travelled distance rather than time.
            if not self._LOW_VEL_MODE:
                d[:traj_len] = trajectory.trajectory_lat.calc_position(t, t2, t3, t4, t5)  # lat pos
                d_velocity[:traj_len] = trajectory.trajectory_lat.calc_velocity(t, t2, t3, t4)  # lat velocity
                d_acceleration[:traj_len] = trajectory.trajectory_lat.calc_acceleration(t, t2, t3)  # lat acceleration
            else:
                # compute normalized travelled distance for low velocity mode of lateral planning
                s1 = s - s[0]
                s2 = np.square(s1)
                s3 = s2 * s1
                s4 = np.square(s2)
                s5 = s4 * s1

                # compute lateral position, velocity, acceleration from trajectory sample
                d[:traj_len] = trajectory.trajectory_lat.calc_position(s1, s2, s3, s4, s5)  # lat pos
                # in LOW_VEL_MODE d_velocity is actually d' (see Diss. Moritz Werling  p.124)
                d_velocity[:traj_len] = trajectory.trajectory_lat.calc_velocity(s1, s2, s3, s4)  # lat velocity
                d_acceleration[:traj_len] = trajectory.trajectory_lat.calc_acceleration(s1, s2, s3)  # lat acceleration

            # Initialize trajectory state vectors
            # (Global) Cartesian positions x, y
            x = np.zeros(self.N + 1)
            y = np.zeros(self.N + 1)
            # (Global) Cartesian velocity v and acceleration a
            v = np.zeros(self.N + 1)
            a = np.zeros(self.N + 1)
            # Orientation theta: Cartesian (gl) and Curvilinear (cl)
            theta_gl = np.zeros(self.N + 1)
            theta_cl = np.zeros(self.N + 1)
            # Curvature kappa : Cartesian (gl) and Curvilinear (cl)
            kappa_gl = np.zeros(self.N + 1)
            kappa_cl = np.zeros(self.N + 1)

            # Initialize Feasibility boolean
            feasible = True

            if not self._draw_traj_set:
                # pre-filter with quick underapproximative check for feasibility
                if np.any(np.abs(s_acceleration) > self.vehicle_params.a_max):
                    if DEBUG:
                        print(f"Acceleration {np.max(np.abs(s_acceleration))}")
                    feasible = False
                    continue
                if np.any(s_velocity < -0.1):
                    if DEBUG:
                        print(f"Velocity {min(s_velocity)} at step")
                    feasible = False
                    continue

            for i in range(0, traj_len):
                # compute orientations
                # see Appendix A.1 of Moritz Werling's PhD Thesis for equations
                if not self._LOW_VEL_MODE:
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

                s_idx = np.argmax(self._co.ref_pos > s[i])
                if s_idx + 1 >= len(self._co.ref_pos):
                    feasible = False
                    break

                # factor for interpolation
                s_lambda = (self._co.ref_pos[s_idx] - s[i]) / (
                        self._co.ref_pos[s_idx + 1] - self._co.ref_pos[s_idx])

                # compute curvilinear and (global) Cartesian orientation
                if s_velocity[i] > 0.005:
                    # compute curvilinear orientation
                    if self._LOW_VEL_MODE:
                        theta_cl[i] = np.arctan2(dp, 1.0)
                    else:
                        theta_cl[i] = np.arctan2(d_velocity[i], s_velocity[i])
                    theta_gl[i] = theta_cl[i] + interpolate_angle(
                        s[i],
                        self._co.ref_pos[s_idx],
                        self._co.ref_pos[s_idx + 1],
                        self._co.ref_theta[s_idx],
                        self._co.ref_theta[s_idx + 1]
                    )
                else:
                    theta_cl[i] = interpolate_angle(
                        s[i],
                        self._co.ref_pos[s_idx],
                        self._co.ref_pos[s_idx + 1],
                        self._co.ref_theta[s_idx],
                        self._co.ref_theta[s_idx + 1]
                    )
                    theta_gl[i] = theta_cl[i]

                # Interpolate curvature of reference path k_r at current position
                k_r = (self._co.ref_curv[s_idx + 1] - self._co.ref_curv[s_idx]) * s_lambda + self._co.ref_curv[
                    s_idx]
                # Interpolate curvature rate of reference path k_r_d at current position
                k_r_d = (self._co.ref_curv_d[s_idx + 1] - self._co.ref_curv_d[s_idx]) * s_lambda + \
                        self._co.ref_curv_d[s_idx]

                # compute global curvature (see appendix A of Moritz Werling's PhD thesis)
                oneKrD = (1 - k_r * d[i])
                cosTheta = math.cos(theta_cl[i])
                tanTheta = np.tan(theta_cl[i])
                kappa_gl[i] = (dpp + k_r * dp * tanTheta) * cosTheta * (cosTheta / oneKrD) ** 2 + (
                        cosTheta / oneKrD) * k_r
                kappa_cl[i] = kappa_gl[i] - k_r

                # compute (global) Cartesian velocity
                v[i] = s_velocity[i] * (oneKrD / (math.cos(theta_cl[i])))

                # compute (global) Cartesian acceleration
                a[i] = s_acceleration[i] * oneKrD / cosTheta + ((s_velocity[i] ** 2) / cosTheta) * (
                        oneKrD * tanTheta * (kappa_gl[i] * oneKrD / cosTheta - k_r) - (
                        k_r_d * d[i] + k_r * d_velocity[i]))

                # CHECK KINEMATIC CONSTRAINTS (remove infeasible trajectories)
                # velocity constraint
                if abs(v[i]) < -0.1:
                    feasible = False
                    if not self._draw_traj_set:
                        break
                # curvature constraint
                kappa_max = np.tan(self.vehicle_params.delta_max) / self.vehicle_params.wheelbase
                if abs(kappa_gl[i]) > kappa_max:
                    feasible = False
                    if not self._draw_traj_set:
                        break
                # yaw rate (orientation change) constraint
                yaw_rate = (theta_gl[i] - theta_gl[i-1]) / self.dT if i > 0 else 0.
                theta_dot_max = kappa_max * v[i]
                if abs(yaw_rate) > theta_dot_max:
                    feasible = False
                    if not self._draw_traj_set:
                        break
                # curvature rate constraint
                steering_angle = np.arctan2(self.vehicle_params.wheelbase * yaw_rate, v[i])
                kappa_dot_max = self.vehicle_params.v_delta_max / self.vehicle_params.wheelbase * \
                                math.cos(steering_angle) ** 2
                if abs((kappa_gl[i] - kappa_gl[i - 1]) / self.dT if i > 0 else 0.) > kappa_dot_max:
                    feasible = False
                    if not self._draw_traj_set:
                        break
                # acceleration constraint (considering switching velocity, see vehicle models documentation)
                v_switch = self.vehicle_params.v_switch
                a_max = self.vehicle_params.a_max * v_switch / v[i] if v[i] > v_switch else self.vehicle_params.a_max
                a_min = -self.vehicle_params.a_max
                if not a_min <= a[i] <= a_max:
                    feasible = False
                    if not self._draw_traj_set:
                        break

            # if selected polynomial trajectory is feasible, store it's Cartesian and Curvilinear trajectory
            if feasible or self._draw_traj_set:
                for i in range(0, traj_len):
                    # compute (global) Cartesian position
                    pos: np.ndarray = self._co.convert_to_cartesian_coords(s[i], d[i])
                    if pos is not None:
                        x[i] = pos[0]
                        y[i] = pos[1]
                    else:
                        feasible = False
                        if DEBUG:
                            print("Out of projection domain")
                        break

                if feasible:
                    # store Cartesian trajectory
                    trajectory.cartesian = CartesianSample(x, y, theta_gl, v, a, kappa_gl,
                                                           kappa_dot=np.append([0], np.diff(kappa_gl)),
                                                           current_time_step=traj_len)

                    # store Curvilinear trajectory
                    trajectory.curvilinear = CurviLinearSample(s, d, theta_gl,
                                                               ss=s_velocity, sss=s_acceleration,
                                                               dd=d_velocity, ddd=d_acceleration,
                                                               current_time_step=traj_len)

                    # check if trajectories planning horizon is shorter than expected and extend if necessary
                    if self.N + 1 > trajectory.cartesian.current_time_step:
                        trajectory.enlarge(self.dT)

                    assert self.N + 1 == trajectory.cartesian.current_time_step == len(trajectory.cartesian.x) == \
                           len(trajectory.cartesian.y) == len(trajectory.cartesian.theta), \
                        '<ReactivePlanner/kinematics>:  Lenghts of state variables is not equal.'
                    # store trajectory in feasible trajectories list
                    feasible_trajectories.append(trajectory)

                if not feasible and self._draw_traj_set:
                    # store Cartesian trajectory
                    trajectory.cartesian = CartesianSample(x, y, theta_gl, v, a, kappa_gl,
                                                           kappa_dot=np.append([0], np.diff(kappa_gl)),
                                                           current_time_step=traj_len)

                    # store Curvilinear trajectory
                    trajectory.curvilinear = CurviLinearSample(s, d, theta_gl,
                                                               ss=s_velocity, sss=s_acceleration,
                                                               dd=d_velocity, ddd=d_acceleration,
                                                               current_time_step=traj_len)

                    # check if trajectories planning horizon is shorter than expected and extend if necessary
                    if self.N + 1 > trajectory.cartesian.current_time_step:
                        trajectory.enlarge(self.dT)

                    # store trajectory in infeasible trajectories list
                    infeasible_trajectories.append(trajectory)

        if self._DEBUG:
            print('<ReactivePlanner>: Kinematic check of %s trajectories done' % len(trajectories))

        # store feasible trajectories in Queue 1
        queue_1.put(feasible_trajectories)
        # if visualization is required: store infeasible trajectories in Queue 1
        if self._draw_traj_set:
            queue_2.put(infeasible_trajectories)

    def _get_optimal_trajectory(self, trajectory_bundle: TrajectoryBundle) -> Union[TrajectorySample, None]:
        """
        Computes the optimal trajectory from a given trajectory bundle
        :param trajectory_bundle: The trajectory bundle
        :return: The optimal trajectory if exists (otherwise None)
        """
        # reset statistics
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0

        # check kinematics of each trajectory
        # divide trajectory_bundle.trajectories into chunks
        chunk_size = math.ceil(len(trajectory_bundle.trajectories) / self._num_workers)
        chunks = [trajectory_bundle.trajectories[ii * chunk_size: min(len(trajectory_bundle.trajectories),
                                                                      (ii+1)*chunk_size)] for ii in range(0, self._num_workers)]

        # initialize list of Processes and Queues
        list_processes = []
        feasible_trajectories = []
        queue_1 = multiprocessing.Queue()
        infeasible_trajectories = []
        queue_2 = multiprocessing.Queue()
        for chunk in chunks:
            p = Process(target=self._check_kinematics, args=(chunk, queue_1, queue_2))
            list_processes.append(p)
            p.start()

        # get return values from queue
        for p in list_processes:
            feasible_trajectories.extend(queue_1.get())
            if self._draw_traj_set:
                infeasible_trajectories.extend(queue_2.get())

        # wait for all processes to finish
        for p in list_processes:
            p.join()

        # update number of infeasible trajectories
        self._infeasible_count_kinematics = len(trajectory_bundle.trajectories) - len(feasible_trajectories)

        # for visualization store all trajectories
        if self._draw_traj_set:
            self.stored_trajectories = feasible_trajectories + infeasible_trajectories

        # set feasible trajectories in bundle
        trajectory_bundle.trajectories = feasible_trajectories
        # sort trajectories according to their costs
        trajectory_bundle.sort()

        # go through sorted list of trajectories and check for collisions
        for trajectory in trajectory_bundle.get_sorted_list():
            # compute position and orientation
            pos1 = trajectory.curvilinear.s if self._collision_check_in_cl else trajectory.cartesian.x
            pos2 = trajectory.curvilinear.d if self._collision_check_in_cl else trajectory.cartesian.y
            theta = trajectory.curvilinear.theta if self._collision_check_in_cl else trajectory.cartesian.theta
            # check each pose for collisions
            collide = False
            for i in range(len(pos1)):
                ego = pycrcc.TimeVariantCollisionObject(self.x_0.time_step + i * self._factor)
                ego.append_obstacle(pycrcc.RectOBB(0.5 * self.vehicle_params.length, 0.5 * self.vehicle_params.width,
                                                   theta[i], pos1[i], pos2[i]))
                if self._cc.collide(ego):
                    self._infeasible_count_collision += 1
                    collide = True
                    break
                # TODO add OBB Sum hull for ego
            if not collide:
                return trajectory
        return None

    def convert_cr_trajectory_to_object(self, trajectory: Trajectory):
        """
        Converts a CR trajectory to a CR dynamic obstacle with given dimensions
        :param trajectory: The trajectory of the vehicle
        :return: CR dynamic obstacles representing the ego vehicle
        """
        # get shape of vehicle
        shape = Rectangle(self.vehicle_params.length, self.vehicle_params.width)
        # get trajectory prediction
        prediction = TrajectoryPrediction(trajectory, shape)
        return DynamicObstacle(42, ObstacleType.CAR, shape, trajectory.state_list[0], prediction)

    @staticmethod
    def shift_orientation(trajectory: Trajectory, interval_start=-np.pi, interval_end=np.pi):
        for state in trajectory.state_list:
            while state.orientation < interval_start:
                state.orientation += 2 * np.pi
            while state.orientation > interval_end:
                state.orientation -= 2 * np.pi
        return trajectory


# TODO remove?
def shift_angle_to_interval(angle_list, interval_start=-np.pi, interval_end=np.pi):
    new_angle_list = np.zeros(angle_list.shape)
    for idx, angle in enumerate(angle_list):
        while angle < interval_start:
            angle += 2 * np.pi
        while angle > interval_end:
            angle -= 2 * np.pi
        new_angle_list[idx] = angle
    return new_angle_list
