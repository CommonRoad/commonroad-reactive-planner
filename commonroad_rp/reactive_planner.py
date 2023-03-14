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
from typing import List, Union
from dataclasses import dataclass
import multiprocessing
from multiprocessing.context import Process
import logging

# commonroad-io
from commonroad.common.validity import is_positive, is_real_number, is_natural_number
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import KSState, FloatExactOrInterval, CustomState, InitialState
from commonroad.scenario.scenario import Scenario

# commonroad_dc
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object
from commonroad_dc.collision.trajectory_queries.trajectory_queries import trajectory_preprocess_obb_sum

# commonroad_rp imports
from commonroad_rp.cost_function import DefaultCostFunction
from commonroad_rp.parameter import DefGymSampling, TimeSampling, VelocitySampling, PositionSampling
from commonroad_rp.polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from commonroad_rp.trajectories import TrajectoryBundle, TrajectorySample, CartesianSample, CurviLinearSample
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem, interpolate_angle
from commonroad_rp.configuration import Configuration, VehicleConfiguration


logger = logging.getLogger(__name__)


@dataclass(eq=False)
class ReactivePlannerState(KSState):
    """
    State class used for output trajectory of reactive-planner: Extends KS State attributes by acceleration and
    yaw rate
    """
    def __repr__(self):
        return f"(time_step={self.time_step}, position={self.position},steering_angle={self.steering_angle}, " \
               f"velocity={self.velocity}, orientation={self.orientation}, acceleration={self.acceleration}, " \
               f"yaw_rate = {self.yaw_rate})"

    acceleration: FloatExactOrInterval = None
    yaw_rate: FloatExactOrInterval = None


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
        self.N = config.planning.time_steps_computation
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
        self._continuous_cc = config.planning.continuous_collision_check
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
        # multiprocessing
        self._multiproc = config.debug.multiproc
        self._num_workers = config.debug.num_workers
        # visualize trajectory set
        self._draw_traj_set = config.debug.draw_traj_set and (config.debug.save_plots or config.debug.save_plots)
        # Debug mode
        self.debug_mode = config.debug.debug_mode

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

    def set_collision_checker(self, scenario: Scenario = None, collision_checker: pycrcc.CollisionChecker = None):
        """
        Sets the collision checker used by the planner using either of the two options:
        If a collision_checker object is passed, then it is used directly by the planner.
        If no collision checker object is passed, then a CommonRoad scenario must be provided from which the collision
        checker is created and set.
        :param scenario: CommonRoad Scenario object
        :param collision_checker: pycrcc.CollisionChecker object
        """
        if collision_checker is None:
            assert scenario is not None, '<set collision checker>: Please provide a CommonRoad scenario OR a ' \
                                         'CollisionChecker object to the planner.'
            cc_scenario = pycrcc.CollisionChecker()
            for co in scenario.static_obstacles:
                obs = create_collision_object(co)
                cc_scenario.add_collision_object(obs)
            for co in scenario.dynamic_obstacles:
                tvo = create_collision_object(co)
                if self._continuous_cc:
                    tvo, err = trajectory_preprocess_obb_sum(tvo)
                    if err == -1:
                        raise Exception("Invalid input for trajectory_preprocess_obb_sum: dynamic "
                                        "obstacle elements overlap")
                cc_scenario.add_collision_object(tvo)
            _, road_boundary_sg_obb = create_road_boundary_obstacle(scenario)
            cc_scenario.add_collision_object(road_boundary_sg_obb)
            self._cc: pycrcc.CollisionChecker = cc_scenario
        else:
            assert scenario is None, '<set collision checker>: Please provide a CommonRoad scenario OR a ' \
                                     'CollisionChecker object to the planner.'
            self._cc: pycrcc.CollisionChecker = collision_checker

    def set_reference_path(self, reference_path: np.ndarray = None, coordinate_system: CoordinateSystem = None):
        """
        Automatically creates a curvilinear coordinate system from a given reference path or sets a given
        curvilinear coordinate system for the planner to use
        :param reference_path: reference path as polyline
        :param coordinate_system: given CoordinateSystem object which is used by the planner
        """
        if coordinate_system is None:
            assert reference_path is not None, '<set reference path>: Please provide a reference path OR a ' \
                                               'CoordinateSystem object to the planner.'
            self._co: CoordinateSystem = CoordinateSystem(reference_path)
        else:
            assert reference_path is None, '<set reference path>: Please provide a reference path OR a ' \
                                           'CoordinateSystem object to the planner.'
            self._co: CoordinateSystem = coordinate_system

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
        if self.debug_mode >= 1:
            logger.info("Sampled interval of velocity: {} m/s - {} m/s".format(min_v, max_v))

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
        if self.debug_mode >= 1:
            logger.info("===== Sampling trajectories =====")
            logger.info(f"Sampling density {samp_level + 1} of {self._sampling_level}")

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

        if self.debug_mode >= 1:
            logger.info(f"Number of trajectory samples: {len(trajectory_bundle._trajectory_bundle)}")
        return trajectory_bundle

    def _compute_initial_states(self, x_0: ReactivePlannerState) -> (np.ndarray, np.ndarray):
        """
        Computes the curvilinear initial states for the polynomial planner based on a Cartesian CommonRoad state
        :param x_0: The CommonRoad state object representing the initial state of the vehicle
        :return: A tuple containing the initial longitudinal and lateral states (lon,lat)
        """
        # compute curvilinear position
        try:
            s, d = self._co.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])
        except ValueError:
            print('<Reactive_planner>: Value Error for curvilinear transformation')
            # TODO: Remove this fix from the reactive planner -> fix in CCosy
            tmp = np.array([x_0.position])
            print(x_0.position)
            if self._co.reference[0][0] > x_0.position[0]:
                reference_path = np.concatenate((tmp, self._co.reference), axis=0)
            else:
                reference_path = np.concatenate((self._co.reference, tmp), axis=0)
            self.set_reference_path(reference_path)
            s, d = self._co.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])

        # factor for interpolation
        s_idx = np.argmax(self._co.ref_pos > s) - 1
        s_lambda = (s - self._co.ref_pos[s_idx]) / (
                self._co.ref_pos[s_idx + 1] - self._co.ref_pos[s_idx])

        # compute orientation in curvilinear coordinate frame
        ref_theta = np.unwrap(self._co.ref_theta)
        theta_cl = x_0.orientation - interpolate_angle(s, self._co.ref_pos[s_idx], self._co.ref_pos[s_idx + 1],
                                                       ref_theta[s_idx], ref_theta[s_idx + 1])

        # compute reference curvature
        kr = (self._co.ref_curv[s_idx + 1] - self._co.ref_curv[s_idx]) * s_lambda + self._co.ref_curv[
                    s_idx]
        # compute reference curvature change
        kr_d = (self._co.ref_curv_d[s_idx + 1] - self._co.ref_curv_d[s_idx]) * s_lambda + \
                        self._co.ref_curv_d[s_idx]

        # compute initial ego curvature from initial steering angle
        kappa_0 = np.tan(x_0.steering_angle) / self.vehicle_params.wheelbase

        # compute d' and d'' -> derivation after arclength (s): see Eq. (A.3) and (A.5) in Diss. Werling
        d_p = (1 - kr * d) * np.tan(theta_cl)
        d_pp = -(kr_d * d + kr * d_p) * np.tan(theta_cl) + ((1 - kr * d) / (math.cos(theta_cl) ** 2)) * (
                kappa_0 * (1 - kr * d) / math.cos(theta_cl) - kr)

        # compute s dot (s_velocity) and s dot dot (s_acceleration) -> derivation after time
        s_velocity = x_0.velocity * math.cos(theta_cl) / (1 - kr * d)
        if s_velocity < 0:
            raise Exception("Initial state or reference incorrect! Curvilinear velocity is negative which indicates"
                            "that the ego vehicle is not driving in the same direction as specified by the reference")

        s_acceleration = x_0.acceleration
        s_acceleration -= (s_velocity ** 2 / math.cos(theta_cl)) * (
                (1 - kr * d) * np.tan(theta_cl) * (kappa_0 * (1 - kr * d) / (math.cos(theta_cl)) - kr) -
                (kr_d * d + kr * d_p))
        s_acceleration /= ((1 - kr * d) / (math.cos(theta_cl)))

        # compute d dot (d_velocity) and d dot dot (d_acceleration)
        if self._LOW_VEL_MODE:
            # in LOW_VEL_MODE: d_velocity and d_acceleration are derivatives w.r.t arclength (s)
            d_velocity= d_p
            d_acceleration = d_pp
        else:
            # in HIGH VEL MODE: d_velocity and d_acceleration are derivatives w.r.t time
            d_velocity = x_0.velocity * math.sin(theta_cl)
            d_acceleration = s_acceleration * d_p + s_velocity ** 2 * d_pp

        x_0_lon: List[float] = [s, s_velocity, s_acceleration]
        x_0_lat: List[float] = [d, d_velocity, d_acceleration]

        if self.debug_mode >= 1:
            logger.info(f"Initial state: {x_0}")

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
            cart_states['orientation'] = trajectory.cartesian.theta[i]
            cart_states['velocity'] = trajectory.cartesian.v[i]
            cart_states['acceleration'] = trajectory.cartesian.a[i]
            if i > 0:
                cart_states['yaw_rate'] = (trajectory.cartesian.theta[i] - trajectory.cartesian.theta[i-1]) / self.dT
            else:
                cart_states['yaw_rate'] = self.x_0.yaw_rate
            # TODO Check why computation with yaw rate was faulty ??
            cart_states['steering_angle'] = np.arctan2(self.vehicle_params.wheelbase *
                                                       trajectory.cartesian.kappa[i], 1.0)
            cart_list.append(ReactivePlannerState(**cart_states))

            # create curvilinear state
            # TODO: This is not correct
            cl_states = dict()
            cl_states['time_step'] = self.x_0.time_step+self._factor*i
            cl_states['position'] = np.array([trajectory.curvilinear.s[i], trajectory.curvilinear.d[i]])
            cl_states['velocity'] = trajectory.cartesian.v[i]
            cl_states['acceleration'] = trajectory.cartesian.a[i]
            cl_states['orientation'] = trajectory.cartesian.theta[i]
            cl_states['yaw_rate'] = trajectory.cartesian.kappa[i]
            cl_list.append(CustomState(**cl_states))

            lon_list.append(
                [trajectory.curvilinear.s[i], trajectory.curvilinear.s_dot[i], trajectory.curvilinear.s_ddot[i]])
            lat_list.append(
                [trajectory.curvilinear.d[i], trajectory.curvilinear.d_dot[i], trajectory.curvilinear.d_ddot[i]])

        # make Cartesian and Curvilinear Trajectory
        cartTraj = Trajectory(self.x_0.time_step, cart_list)
        cvlnTraj = Trajectory(self.x_0.time_step, cl_list)

        return cartTraj, cvlnTraj, lon_list, lat_list

    def plan(self, x_0: ReactivePlannerState, cl_states=None) -> tuple:
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

        if self.debug_mode >= 1:
            logger.info("=================== Starting Planning Cycle ===================")
            logger.info(f"Initial state lon: {x_0_lon}")
            logger.info(f"Initial state lat: {x_0_lat}")
            logger.info(f"Desired velocity: {self._desired_speed} m/s")

        # initialize optimal trajectory dummy
        optimal_trajectory = None

        # initial index of sampling set to use
        i = 1  # Time sampling is not used. To get more samples, start with level 1.

        # sample until trajectory has been found or sampling sets are empty
        while optimal_trajectory is None and i < self._sampling_level:
            # sample trajectory bundle
            bundle = self._create_trajectory_bundle(x_0_lon, x_0_lat, samp_level=i)

            # find optimal trajectory (kinematic check/sorting/collision check)
            t0 = time.time()
            optimal_trajectory = self._get_optimal_trajectory(bundle)
            if self.debug_mode >= 1:
                logger.info("===== Planning result =====")
                logger.info(f"Total checking time: {time.time() - t0:.7f}")
                logger.info(f"Rejected {self.infeasible_count_kinematics} infeasible trajectories due to kinematics")
                logger.info(f"Rejected {self.infeasible_count_collision} infeasible trajectories due to collisions")

            # increase sampling level (i.e., density) if no optimal trajectory could be found
            i = i + 1

        if optimal_trajectory is None and x_0.velocity <= 0.1:
            logger.info("Planning standstill for the current scenario")
            optimal_trajectory = self._compute_standstill_trajectory(x_0, x_0_lon, x_0_lat)

        # check if feasible trajectory exists -> emergency mode
        if optimal_trajectory is None:
            if self.debug_mode >= 1:
                logger.info(f"Could not find any trajectory out of "
                            f"{sum([self._get_no_of_samples(i) for i in range(self._sampling_level)])} trajectories")
        else:
            self._optimal_cost = optimal_trajectory.cost
            if self.debug_mode >= 1:
                relative_costs = None
                if bundle:
                    relative_costs = ((optimal_trajectory.cost - bundle.min_costs().cost) /
                                      (bundle.max_costs().cost - bundle.min_costs().cost))
                logger.info(f"Found optimal trajectory with costs = {self._optimal_cost:.3f} "
                            f"({relative_costs:.3f} of seen costs)")

        return self._compute_trajectory_pair(optimal_trajectory) if optimal_trajectory is not None else None

    def _compute_standstill_trajectory(self, x_0, x_0_lon, x_0_lat) -> TrajectorySample:
        """
        Computes a standstill trajectory if the vehicle is already at velocity 0
        :param x_0: The current state of the ego vehicle
        :param x_0_lon: The longitudinal state in curvilinear coordinates
        :param x_0_lat: The lateral state in curvilinear coordinates
        :return: The TrajectorySample for a standstill trajectory
        """
        # create artificial standstill trajectory
        if self.debug_mode >= 1:
            print('Adding standstill trajectory')
            print("x_0 is {}".format(x_0))
            print("x_0_lon is {}".format(x_0_lon))
            print("x_0_lon is {}".format(type(x_0_lon)))
            for i in x_0_lon:
                print("The element {} of format {} is a real number? {}".format(i, type(i), is_real_number(i)))
            print("x_0_lat is {}".format(x_0_lat))
        # create lon and lat polynomial
        traj_lon = QuarticTrajectory(tau_0=0, delta_tau=self.horizon, x_0=np.asarray(x_0_lon),
                                     x_d=np.array([self._desired_speed, 0]))
        traj_lat = QuinticTrajectory(tau_0=0, delta_tau=self.horizon, x_0=np.asarray(x_0_lat),
                                     x_d=np.array([x_0_lat[0], 0, 0]))

        # create Cartesian and Curvilinear trajectory
        p = TrajectorySample(self.horizon, self.dT, traj_lon, traj_lat)
        p.cartesian = CartesianSample(np.repeat(x_0.position[0], self.N), np.repeat(x_0.position[1], self.N),
                                      np.repeat(x_0.orientation, self.N), np.repeat(0, self.N),
                                      np.repeat(0, self.N), np.repeat(0, self.N), np.repeat(0, self.N),
                                      current_time_step=self.N)

        p.curvilinear = CurviLinearSample(np.repeat(x_0_lon[0], self.N), np.repeat(x_0_lat[0], self.N),
                                          np.repeat(x_0.orientation, self.N), dd=np.repeat(x_0_lat[1], self.N),
                                          ddd=np.repeat(x_0_lat[2], self.N), ss=np.repeat(x_0_lon[1], self.N),
                                          sss=np.repeat(x_0_lon[2], self.N), current_time_step=self.N)
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
                s1 = s[:traj_len] - s[0]
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
                    if self.debug_mode >= 2:
                        print(f"Acceleration {np.max(np.abs(s_acceleration))}")
                    feasible = False
                    continue
                if np.any(s_velocity < -0.1):
                    if self.debug_mode >= 2:
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
                        if abs(d_velocity[i]) > 0.001:
                            dp = None
                        else:
                            dp = 0.
                    # see Eq. (A.8) from Moritz Werling's Diss
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

                # factor for interpolation
                s_idx = np.argmax(self._co.ref_pos > s[i]) - 1
                if s_idx + 1 >= len(self._co.ref_pos):
                    feasible = False
                    break
                s_lambda = (s[i] - self._co.ref_pos[s_idx]) / (self._co.ref_pos[s_idx + 1] - self._co.ref_pos[s_idx])

                # compute curvilinear (theta_cl) and global Cartesian (theta_gl) orientation
                if s_velocity[i] > 0.001:
                    # LOW VELOCITY MODE: dp = d_velocity[i]
                    # HIGH VELOCITY MODE: dp = d_velocity[i]/s_velocity[i]
                    theta_cl[i] = np.arctan2(dp, 1.0)

                    theta_gl[i] = theta_cl[i] + interpolate_angle(
                        s[i],
                        self._co.ref_pos[s_idx],
                        self._co.ref_pos[s_idx + 1],
                        self._co.ref_theta[s_idx],
                        self._co.ref_theta[s_idx + 1])
                else:
                    if self._LOW_VEL_MODE:
                        # dp = velocity w.r.t. to travelled arclength (s)
                        theta_cl[i] = np.arctan2(dp, 1.0)

                        theta_gl[i] = theta_cl[i] + interpolate_angle(
                            s[i],
                            self._co.ref_pos[s_idx],
                            self._co.ref_pos[s_idx + 1],
                            self._co.ref_theta[s_idx],
                            self._co.ref_theta[s_idx + 1])
                    else:
                        # in stillstand (s_velocity~0) and High velocity mode: assume vehicle keeps global orientation
                        theta_gl[i] = self.x_0.orientation if i == 0 else theta_gl[i-1]

                        theta_cl[i] = theta_gl[i] - interpolate_angle(
                            s[i],
                            self._co.ref_pos[s_idx],
                            self._co.ref_pos[s_idx + 1],
                            self._co.ref_theta[s_idx],
                            self._co.ref_theta[s_idx + 1])

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
                kappa_gl[i] = (dpp + (k_r * dp + k_r_d * d[i]) * tanTheta) * cosTheta * (cosTheta / oneKrD) ** 2 + (
                        cosTheta / oneKrD) * k_r
                kappa_cl[i] = kappa_gl[i] - k_r

                # compute (global) Cartesian velocity
                v[i] = abs(s_velocity[i] * (oneKrD / (math.cos(theta_cl[i]))))

                # compute (global) Cartesian acceleration
                a[i] = s_acceleration[i] * oneKrD / cosTheta + ((s_velocity[i] ** 2) / cosTheta) * (
                        oneKrD * tanTheta * (kappa_gl[i] * oneKrD / cosTheta - k_r) - (
                        k_r_d * d[i] + k_r * dp))

                # CHECK KINEMATIC CONSTRAINTS (remove infeasible trajectories)
                # velocity constraint
                if v[i] < -0.1:
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
                # TODO: chck if kappa_gl[i-1] ??
                steering_angle = np.arctan2(self.vehicle_params.wheelbase * kappa_gl[i], 1.0)
                kappa_dot_max = self.vehicle_params.v_delta_max / (self.vehicle_params.wheelbase *
                                                                   math.cos(steering_angle) ** 2)
                kappa_dot = (kappa_gl[i] - kappa_gl[i - 1]) / self.dT if i > 0 else 0.
                if abs(kappa_dot) > kappa_dot_max:
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
                        if self.debug_mode >= 2:
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

        if self._multiproc:
            # store feasible trajectories in Queue 1
            queue_1.put(feasible_trajectories)
            # if visualization is required: store infeasible trajectories in Queue 1
            if self._draw_traj_set:
                queue_2.put(infeasible_trajectories)
        else:
            return feasible_trajectories, infeasible_trajectories

    def _get_optimal_trajectory(self, trajectory_bundle: TrajectoryBundle) -> Union[TrajectorySample, None]:
        """
        Computes the optimal trajectory from a given trajectory bundle
        :param trajectory_bundle: The trajectory bundle
        :return: The optimal trajectory if exists (otherwise None)
        """
        logger.info("===== Checking trajectories =====")
        # reset statistics
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0

        # ==== Kinematic checking
        # check kinematics of each trajectory
        t0 = time.time()
        if self._multiproc:
            # with multiprocessing
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
        else:
            # without multiprocessing
            feasible_trajectories, infeasible_trajectories = self._check_kinematics(trajectory_bundle.trajectories)

        logger.info(f"Kinematic checks took:  \t{time.time()-t0:.7f}s")

        # update number of infeasible trajectories
        self._infeasible_count_kinematics = len(trajectory_bundle.trajectories) - len(feasible_trajectories)

        # for visualization store all trajectories
        if self._draw_traj_set:
            self.stored_trajectories = feasible_trajectories + infeasible_trajectories

        # set feasible trajectories in bundle
        trajectory_bundle.trajectories = feasible_trajectories

        # ==== Sorting
        # sort trajectories according to their costs
        t0 = time.time()
        trajectory_bundle.sort()
        logger.info(f"Sort trajectories took:  \t{time.time() - t0:.7f}s")

        # ==== Collision checking
        # go through sorted list of trajectories and check for collisions
        t0 = time.time()
        for trajectory in trajectory_bundle.get_sorted_list():
            # compute position and orientation
            pos1 = trajectory.cartesian.x + self.vehicle_params.wb_rear_axle * np.cos(trajectory.cartesian.theta)
            pos2 = trajectory.cartesian.y + self.vehicle_params.wb_rear_axle * np.sin(trajectory.cartesian.theta)
            theta = trajectory.cartesian.theta

            collide = False
            # check each pose for collisions
            for i in range(len(pos1)):
                ego = pycrcc.TimeVariantCollisionObject(self.x_0.time_step + i * self._factor)
                ego.append_obstacle(pycrcc.RectOBB(0.5 * self.vehicle_params.length, 0.5 * self.vehicle_params.width,
                                                   theta[i], pos1[i], pos2[i]))
                if self._cc.collide(ego):
                    self._infeasible_count_collision += 1
                    collide = True
                    break

            # continuous collision check if not collision has been detected before already
            if self._continuous_cc and not collide:
                ego_tvo = pycrcc.TimeVariantCollisionObject(self.x_0.time_step)
                [ego_tvo.append_obstacle(pycrcc.RectOBB(0.5 * self.vehicle_params.length, 0.5 * self.vehicle_params.width,
                                                   theta[i], pos1[i], pos2[i])) for i in range(len(pos1))]
                ego_tvo, err = trajectory_preprocess_obb_sum(ego_tvo)
                if self._cc.collide(ego_tvo):
                    self._infeasible_count_collision += 1
                    collide = True

            if not collide:
                logger.info(f"Collision checks took:  \t{time.time() - t0:.7f}s")
                return trajectory
        return None

    def shift_and_convert_trajectory_to_object(self, trajectory: Trajectory):
        """
        Converts a CR trajectory to a CR dynamic obstacle with given dimensions
        :param trajectory: The trajectory of the vehicle
        :return: CR dynamic obstacles representing the ego vehicle
        """
        # shift trajectory positions to center
        new_state_list = list()
        for state in trajectory.state_list:
            new_state_list.append(state.translate_rotate(np.array([self.vehicle_params.wb_rear_axle * np.cos(state.orientation),
                                                                   self.vehicle_params.wb_rear_axle * np.sin(state.orientation)]), 0.0))

        new_trajectory = Trajectory(initial_time_step=new_state_list[0].time_step, state_list=new_state_list)
        # get shape of vehicle
        shape = Rectangle(self.vehicle_params.length, self.vehicle_params.width)
        # get trajectory prediction
        prediction = TrajectoryPrediction(new_trajectory, shape)
        return DynamicObstacle(42, ObstacleType.CAR, shape, new_trajectory.state_list[0], prediction)

    @staticmethod
    def shift_orientation(trajectory: Trajectory, interval_start=-np.pi, interval_end=np.pi):
        for state in trajectory.state_list:
            while state.orientation < interval_start:
                state.orientation += 2 * np.pi
            while state.orientation > interval_end:
                state.orientation -= 2 * np.pi
        return trajectory

    def process_initial_state_from_pp(self, x0_pp: InitialState):
        """
        Function converts the initial state from the CommonRoad planning problem to the reactive planner state:
        - initial positions (x, y) from planning problem are shifted from vehicle center to rear axle
        - initial steering angle is computed from initial yaw rate and velocity
        """
        # shift initial position to rear axle
        x0_shifted = x0_pp.translate_rotate(np.array([-self.vehicle_params.wb_rear_axle * np.cos(x0_pp.orientation),
                                                      -self.vehicle_params.wb_rear_axle * np.sin(x0_pp.orientation)])
                                            , 0.0)

        # create initial state for planner and compute initial steering angle
        x0_planner = ReactivePlannerState()
        x0_planner = x0_shifted.convert_state_to_state(x0_planner)
        x0_planner.steering_angle = np.arctan2(self.vehicle_params.wheelbase * x0_planner.yaw_rate, x0_planner.velocity)
        return x0_planner
