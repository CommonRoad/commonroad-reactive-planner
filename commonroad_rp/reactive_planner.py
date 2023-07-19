__author__ = "Gerald Würsching, Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "1.0"
__maintainer__ = "Gerald Würsching"
__email__ = "gerald.wuersching@tum.de"
__status__ = "Beta"

# python packages
import math
import time
import numpy as np
from typing import List, Union, Optional, Tuple, Type, Dict
import multiprocessing
from multiprocessing.context import Process
import logging

# commonroad-io
from commonroad.common.validity import is_real_number
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import CustomState, InputState
from commonroad.scenario.scenario import Scenario

# commonroad_dc
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object
from commonroad_dc.collision.trajectory_queries.trajectory_queries import trajectory_preprocess_obb_sum

# commonroad_rp imports
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.cost_function import CostFunction, DefaultCostFunction
from commonroad_rp.sampling import TimeSampling, VelocitySampling, PositionSampling, SamplingSpace, \
    sampling_space_factory
from commonroad_rp.polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from commonroad_rp.trajectories import TrajectoryBundle, TrajectorySample, CartesianSample, CurviLinearSample, \
    FeasibilityStatus
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem, interpolate_angle
from commonroad_rp.utility.general import shift_orientation, retrieve_desired_velocity_from_pp
from commonroad_rp.utility.config import ReactivePlannerConfiguration, VehicleConfiguration

# get logger
logger = logging.getLogger("RP_LOGGER")

# precision value
_EPS = 1e-5


class ReactivePlanner(object):
    """
    Reactive planner class that plans trajectories in a sampling-based fashion
    """

    def __init__(self, config: ReactivePlannerConfiguration):
        """
        Constructor of the reactive planner
        : param config: Configuration object holding all planner-relevant configurations
        """
        # Set horizon variables
        self.dt: float = config.planning.dt
        self.N: int = config.planning.time_steps_computation
        self.horizon: float = config.planning.dt * config.planning.time_steps_computation

        # get vehicle parameters from config file
        self.vehicle_params: VehicleConfiguration = config.vehicle

        # planner initial states (cartesian and curvilinear)
        self.x_0: Optional[ReactivePlannerState] = None
        self.x_0_cl: Optional[Tuple[List, List]] = None

        # coordinate system & collision checker
        self._co: Optional[CoordinateSystem] = None
        self._cc: Optional[pycrcc.CollisionChecker] = None

        # statistics
        self._infeasible_count_collision: int = 0
        self._infeasible_count_kinematics: int = 0
        self._infeasible_reason_dict: Dict = dict()
        self._optimal_cost: float = 0.0
        self._planning_times_list: List = list()
        self._record_state_list: List[ReactivePlannerState] = list()
        self._record_input_list: List[InputState] = list()

        # store sampled trajectory set of last run
        self.stored_trajectories: Optional[List[TrajectorySample]] = None

        # desired speed
        self._desired_speed: Optional[float] = None
        self._desired_lon_position: Optional[float] = None
        # threshold for low velocity mode
        self._low_vel_mode = False

        # Debug setting: visualize trajectory set
        self._draw_traj_set = config.debug.draw_traj_set and (config.debug.save_plots or config.debug.save_plots)

        # set/reset configuration
        self.config: Optional[ReactivePlannerConfiguration] = None
        self.reset(config)

        # set sampling space
        self.sampling_space: Optional[Type[SamplingSpace]] = None
        self.set_sampling_space()
        self.sampling_level = config.sampling.num_sampling_levels

        # set cost function
        self.cost_function: Optional[Type[CostFunction]] = None
        self.set_cost_function()

        # set standstill lookahead
        self._standstill_lookahead = config.planning.standstill_lookahead

    @property
    def collision_checker(self) -> pycrcc.CollisionChecker:
        return self._cc

    @property
    def coordinate_system(self) -> CoordinateSystem:
        return self._co

    @property
    def reference_path(self) -> np.ndarray:
        return self._co.reference

    @property
    def infeasible_count_collision(self) -> float:
        """Number of colliding trajectories"""
        return self._infeasible_count_collision

    @property
    def infeasible_count_kinematics(self) -> float:
        """Number of kinematically infeasible trajectories"""
        return self._infeasible_count_kinematics

    @property
    def infeasible_reason_dict(self) -> dict:
        """Dictionary to store number of kinematically infeasible trajectories for each checked kinematic constraint"""
        return self._infeasible_reason_dict

    @property
    def optimal_cost(self) -> float:
        """Cost of the optimal trajectory"""
        return self._optimal_cost

    @property
    def planning_times(self) -> List:
        """List of planning computation times"""
        return self._planning_times_list

    @property
    def record_state_list(self) -> List:
        """List of recorded planner states"""
        return self._record_state_list

    @property
    def record_input_list(self) -> List:
        """List of recorded planner control inputs"""
        return self._record_input_list

    def goal_reached(self) -> bool:
        """Checks if the currently set initial state of the planner is within the goal configuration"""
        # shift ReactivePlannerState to center for goal check
        x_0_shifted = self.x_0.shift_positions_to_center(self.vehicle_params.wb_rear_axle)
        if self.config.planning_problem.goal.is_reached(x_0_shifted):
            logger.info("Goal of planning problem reached")
            return True
        else:
            return False

    def reset(self, config: ReactivePlannerConfiguration = None,
              initial_state_cart: ReactivePlannerState = None,
              initial_state_curv: Tuple[List, List] = None,
              collision_checker: pycrcc.CollisionChecker = None,
              coordinate_system: CoordinateSystem = None):
        """
        Initializes/resets configuration of the planner for re-planning purposes
        """
        # set updated config
        if config is not None:
            self.config = config
        else:
            assert self.config is not None, "<ReactivePlanner.reset(). No Configuration object provided>"

        # reset statistics
        self._reset_statistics()

        # reset collision checker
        if collision_checker is None:
            # create new collision checker from updated scenario
            self.set_collision_checker(scenario=self.config.scenario)
        else:
            # use passed collision checker object
            self.set_collision_checker(collision_checker=collision_checker)

        # reset ref path and CoSys
        if coordinate_system is None:
            # create new CoSys from reference path
            self.set_reference_path(reference_path=self.config.planning.reference_path)
        else:
            # use passed CoSys object
            self.set_reference_path(coordinate_system=coordinate_system)

        # if planner init state is empty: Convert cartesian initial state from planning problem
        if self.x_0 is None and initial_state_cart is None:
            self.x_0 = ReactivePlannerState.create_from_initial_state(self.config.planning_problem.initial_state,
                                                                      self.vehicle_params.wheelbase,
                                                                      self.vehicle_params.wb_rear_axle)
        else:
            self.x_0 = initial_state_cart if initial_state_cart is not None else self.x_0

        # set low velocity mode given initial velocity in self.x_0
        self._low_vel_mode = True if self.x_0.velocity < self.config.planning.low_vel_mode_threshold else False

        # convert Cartesian initial state or pass given curvilinear initial state
        self.x_0_cl = initial_state_curv if initial_state_curv is not None else self._compute_initial_states(self.x_0)

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
            assert scenario is not None, '<ReactivePlanner.set collision checker>: Please provide a CommonRoad ' \
                                         'scenario OR a ' \
                                         'CollisionChecker object to the planner.'
            cc_scenario = pycrcc.CollisionChecker()
            for co in scenario.static_obstacles:
                obs = create_collision_object(co)
                cc_scenario.add_collision_object(obs)
            for co in scenario.dynamic_obstacles:
                tvo = create_collision_object(co)
                if self.config.planning.continuous_collision_check:
                    tvo, err = trajectory_preprocess_obb_sum(tvo)
                    if err == -1:
                        raise Exception("Invalid input for trajectory_preprocess_obb_sum: dynamic "
                                        "obstacle elements overlap")
                cc_scenario.add_collision_object(tvo)
            _, road_boundary_sg_obb = create_road_boundary_obstacle(scenario)
            cc_scenario.add_collision_object(road_boundary_sg_obb)
            self._cc: pycrcc.CollisionChecker = cc_scenario
        else:
            assert scenario is None, '<ReactivePlanner.set collision checker>: Please provide a CommonRoad scenario ' \
                                     'OR a ' \
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

    def set_t_sampling_parameters(self, t_min):
        """
        Sets sample parameters of time domain. Only t_min is set; maximum time sample is given by planner horizon
        :param t_min: minimum of sampled time horizon
        """
        self.sampling_space.samples_t = TimeSampling(t_min, self.horizon, self.sampling_level, self.dt)
        logger.debug("Sampled interval of time: {} s - {} s".format(t_min, self.horizon))

    def set_d_sampling_parameters(self, delta_d_min, delta_d_max):
        """
        Sets sample parameters of lateral offset
        :param delta_d_min: lateral distance lower than reference
        :param delta_d_max: lateral distance higher than reference
        """
        self.sampling_space.samples_d = PositionSampling(delta_d_min, delta_d_max, self.sampling_level)
        logger.debug("Sampled interval of lateral position: {} m - {} m".format(delta_d_min, delta_d_max))

    def set_v_sampling_parameters(self, v_min, v_max):
        """
        Sets sample parameters of sampled velocity interval
        :param v_min: minimal velocity sample bound
        :param v_max: maximal velocity sample bound
        """
        self.sampling_space.samples_v = VelocitySampling(v_min, v_max, self.sampling_level)
        logger.info("Sampled interval of velocity: {} m/s - {} m/s".format(v_min, v_max))

    def set_s_sampling_parameters(self, s_min, s_max):
        """
        Sets sample parameters of longitudinal position
        :param s_min minimum lon position sample bound
        :param s_max maximum lon position sample bound
        """
        self.sampling_space.samples_s = PositionSampling(s_min, s_max, self.sampling_level)
        logger.info("Sampled interval of longitudinal position: {} m - {} m".format(s_min, s_max))

    def set_desired_velocity(self, desired_velocity: float = None, current_speed: float = None, stopping: bool = False):
        """
        Sets desired velocity and re-calculates velocity samples
        :param desired_velocity: velocity in m/s
        :param current_speed: velocity in m/s
        :param stopping
        :return: velocity in m/s
        """
        # set desired lon position to None if in velocity following mode
        self._desired_lon_position = None

        if desired_velocity is None and self._desired_speed is None:
            self._desired_speed = retrieve_desired_velocity_from_pp(self.config.planning_problem)
        else:
            self._desired_speed = desired_velocity if desired_velocity is not None else self._desired_speed

        # check if desired speed is valid
        assert self._desired_speed >= 0.0, f"<ReactivePlanner.set_desired_velocity(): desired speed has to be " \
                                           f"positive. Provided speed{self._desired_speed}>"

        if not stopping:
            reference_speed = current_speed if current_speed is not None else self._desired_speed

            min_v = max(0, reference_speed - (0.125 * self.horizon * self.vehicle_params.a_max))
            # max_v = max(min_v + 5.0, reference_speed + (0.25 * self.horizon * self.constraints.a_max))
            max_v = max(min_v + 5.0, reference_speed + 2)
            self.set_v_sampling_parameters(min_v, max_v)
        else:
            self.set_v_sampling_parameters(v_min=self._desired_speed, v_max=self._desired_speed)

        # Update desired velocity in cost function
        if hasattr(self.cost_function, "desired_speed"):
            self.cost_function.desired_speed = self._desired_speed
        # update acceleration weight in cost function
        if hasattr(self.cost_function, "w_a"):
            self.cost_function.w_a = 5
        # set desired_s in cost function to None
        if hasattr(self.cost_function, "desired_s"):
            self.cost_function.desired_s = self._desired_lon_position

    def set_desired_lon_position(self, lon_position: float,
                                 delta_s_min: Optional[float] = None, delta_s_max: Optional[float] = None):
        """
        Sets a desired longitudinal position for stopping and re-calculates s position samples
        NOTE: Currently, the desired longitudinal position is only considered for stopping, target velocity and
              acceleration are set to 0.0
        :param lon_position: the desired longitudinal position (used in cost function)
        :param delta_s_min: longitudinal distance behind lon_position
        :param delta_s_max: longitudinal distance in front of lon_position
        """
        # set target longitudinal position for stopping
        self._desired_lon_position = lon_position
        # set target velocity for stopping
        self._desired_speed = 0.0
        # set up sampling interval for longitudinal position
        if delta_s_min is None and delta_s_max is None:
            delta_s_min = self.config.sampling.s_min
            delta_s_max = self.config.sampling.s_max
        self.set_s_sampling_parameters(s_min=lon_position + delta_s_min, s_max=lon_position + delta_s_max)

        # Update cost function
        if hasattr(self.cost_function, "desired_s"):
            self.cost_function.desired_s = self._desired_lon_position
        if hasattr(self.cost_function, "desired_speed"):
            self.cost_function.desired_speed = self._desired_speed
        # update acceleration weight in cost function
        if hasattr(self.cost_function, "w_a"):
            self.cost_function.w_a = 1

    def set_cost_function(self, cost_function: Type[CostFunction] = None):
        if cost_function:
            self.cost_function = cost_function
        else:
            self.cost_function = DefaultCostFunction(self._desired_speed, desired_d=0.0,
                                                     desired_s=self._desired_lon_position)

    def set_sampling_space(self, sampling_space: Type[SamplingSpace] = None):
        if sampling_space:
            self.sampling_space = sampling_space
        else:
            self.sampling_space = sampling_space_factory(self.config)

    def record_state_and_input(self, state: ReactivePlannerState):
        """
        Adds state to list of recorded states
        Adds control inputs to list of recorded inputs
        """
        # append state to state list
        self.record_state_list.append(state)

        # compute control inputs and append to input list
        if len(self.record_state_list) > 1:
            steering_angle_speed = (state.steering_angle - self.record_state_list[-2].steering_angle) / self.dt
        else:
            steering_angle_speed = 0.0

        input_state = InputState(time_step=state.time_step,
                                 acceleration=state.acceleration,
                                 steering_angle_speed=steering_angle_speed)
        self.record_input_list.append(input_state)

    def _reset_statistics(self):
        """resets internal statistics/counter variables used for debugging at the start of each planning cycle"""
        # set all counters to zero
        self._optimal_cost = 0
        self._infeasible_count_kinematics = 0
        self._infeasible_count_collision = 0

        # reset dictionary for counting infeasible trajectories for each constraint
        for constraint in self.config.planning.constraints_to_check:
            self._infeasible_reason_dict[constraint] = 0

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
        logger.info("===== Sampling trajectories ... =====")
        logger.info(f"Sampling density {samp_level + 1} of {self.sampling_level}")

        trajectories = self.sampling_space.generate_trajectories_at_level(samp_level, x_0_lon, x_0_lat,
                                                                          self.config.sampling.longitudinal_mode,
                                                                          self._low_vel_mode)

        # create trajectory bundle
        trajectory_bundle = TrajectoryBundle(trajectories, cost_function=self.cost_function)

        logger.info(f"Number of trajectory samples: {len(trajectory_bundle.trajectories)}")
        return trajectory_bundle

    def _compute_initial_states(self, x_0: ReactivePlannerState) -> (np.ndarray, np.ndarray):
        """
        Computes the curvilinear initial states for the polynomial planner based on the Cartesian initial state
        :param x_0: The Cartesion state object representing the initial state of the vehicle
        :return: A tuple containing the initial longitudinal and lateral states (lon,lat)
        """
        # compute curvilinear position
        try:
            s, d = self._co.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])
        except ValueError:
            logger.critical("Initial state could not be transformed.")
            raise ValueError("Initial state could not be transformed.")

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
        kr_d = (self._co.ref_curv_d[s_idx + 1] - self._co.ref_curv_d[s_idx]) * s_lambda + self._co.ref_curv_d[s_idx]

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
        if self._low_vel_mode:
            # in LOW_VEL_MODE: d_velocity and d_acceleration are derivatives w.r.t arclength (s)
            d_velocity = d_p
            d_acceleration = d_pp
        else:
            # in HIGH VEL MODE: d_velocity and d_acceleration are derivatives w.r.t time
            d_velocity = x_0.velocity * math.sin(theta_cl)
            d_acceleration = s_acceleration * d_p + s_velocity ** 2 * d_pp

        x_0_lon: List[float] = [s, s_velocity, s_acceleration]
        x_0_lat: List[float] = [d, d_velocity, d_acceleration]

        return x_0_lon, x_0_lat

    def _compute_trajectory_pair(self, trajectory: TrajectorySample) -> Tuple[Trajectory, Trajectory, List, List]:
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

        scaling_factor = self.config.planning.factor
        for i in range(len(trajectory.cartesian.x)):
            # create Cartesian state
            cart_states = dict()
            cart_states['time_step'] = self.x_0.time_step + scaling_factor * i
            cart_states['position'] = np.array([trajectory.cartesian.x[i], trajectory.cartesian.y[i]])
            cart_states['orientation'] = trajectory.cartesian.theta[i]
            cart_states['velocity'] = trajectory.cartesian.v[i]
            cart_states['acceleration'] = trajectory.cartesian.a[i]
            if i > 0:
                cart_states['yaw_rate'] = (trajectory.cartesian.theta[i] - trajectory.cartesian.theta[i - 1]) / self.dt
            else:
                cart_states['yaw_rate'] = self.x_0.yaw_rate
            cart_states['steering_angle'] = np.arctan2(self.vehicle_params.wheelbase *
                                                       trajectory.cartesian.kappa[i], 1.0)
            cart_list.append(ReactivePlannerState(**cart_states))

            # create curvilinear state
            # TODO: This is not correct
            cl_states = dict()
            cl_states['time_step'] = self.x_0.time_step + scaling_factor * i
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

        # correct orientations of cartesian output trajectory
        cartTraj_corrected = shift_orientation(cartTraj, interval_start=self.x_0.orientation - np.pi,
                                               interval_end=self.x_0.orientation + np.pi)

        return cartTraj_corrected, cvlnTraj, lon_list, lat_list

    def plan(self, current_sampling_level: int = None) -> tuple:
        """
        Plans an optimal trajectory
        :param current_sampling_level: A specific sampling level to evaluate (no iteration)
        :return: Optimal trajectory as tuple
        """
        # start timer
        planning_start_time = time.time()

        # check if initial states are provided
        assert self.x_0 is not None, "<ReactivePlanner.plan(): Planner Cartesian initial state is empty!>"
        assert self.x_0_cl is not None, "<ReactivePlanner.plan(): Planner curvilinear initial state is empty!>"

        # get curvilinear initial states
        x_0_lon, x_0_lat = self.x_0_cl

        logger.info("===============================================================")
        logger.info("=================== Starting Planning Cycle ===================")
        logger.info(f"==== Initial state Cartesian ====")
        logger.info(f"time_step={self.x_0.time_step}")
        logger.info(
            f"position={self.x_0.position}, steering_angle={self.x_0.steering_angle}, velocity={self.x_0.velocity}")
        logger.info(
            f"orientation={self.x_0.orientation}, acceleration={self.x_0.acceleration}, yaw_rate={self.x_0.yaw_rate}")
        logger.info(f"==== Initial state Curvilinear ====")
        logger.info(f"longitudinal state = {x_0_lon}")
        logger.info(f"lateral state = {x_0_lat}")
        logger.info(f"==== Target states ====")
        logger.info(f"longitudinal driving mode: {self.config.sampling.longitudinal_mode}")
        logger.info(f"desired velocity: {self._desired_speed} m/s")
        logger.info(f"desired longitudinal position: {self._desired_lon_position} m")

        # initialize optimal trajectory dummy
        optimal_trajectory = None

        # initial index of sampling set to use
        i = 1 if current_sampling_level is None else current_sampling_level

        while optimal_trajectory is None and i <= self.sampling_level:
            # sample trajectory bundle
            bundle = self._create_trajectory_bundle(x_0_lon, x_0_lat, samp_level=i)

            # find optimal trajectory (kinematic check/sorting/collision check)
            t0 = time.time()
            optimal_trajectory = self._get_optimal_trajectory(bundle)

            logger.info("===== Planning result =====")
            logger.info(f"Total checking time: {time.time() - t0:.7f}")
            logger.info(f"Rejected {self.infeasible_count_kinematics} infeasible trajectories due to kinematics")
            for constraint in self.config.planning.constraints_to_check:
                logger.debug(f"\tInfeasible {constraint}: {self._infeasible_reason_dict[constraint]}")
            logger.info(f"Rejected {self.infeasible_count_collision} infeasible trajectories due to collisions")

            if current_sampling_level is not None:
                break
            else:
                i += 1

        if (optimal_trajectory is None or optimal_trajectory.cartesian.v[self._standstill_lookahead] <= 0.05) \
                and self.x_0.velocity <= 0.05:
            logger.info("Planning standstill for the current scenario")
            optimal_trajectory = self._compute_standstill_trajectory()

            # check if feasible trajectory exists -> emergency mode
            if optimal_trajectory is None and current_sampling_level == self.sampling_level:
                logger.warning(f"Could not find a valid trajectory")
            else:
                self._optimal_cost = optimal_trajectory.cost
                relative_costs = None
                if bundle is not None:
                    relative_costs = ((optimal_trajectory.cost - bundle.min_costs().cost) /
                                      (bundle.max_costs().cost - bundle.min_costs().cost))
                logger.info(f"Found optimal trajectory with costs = {self._optimal_cost:.3f} "
                            f"({relative_costs:.3f} of seen costs)")

        # compute output
        planning_result = self._compute_trajectory_pair(optimal_trajectory) if optimal_trajectory is not None else None

        # end timer
        self._planning_times_list.append(time.time() - planning_start_time)
        logger.info(f"Total planning time: {self.planning_times[-1]:.7f}")

        if planning_result is None:
            logger.warning(f"Planner failed to find an optimal trajectory with given sampling configuration!")

        return planning_result

    def _compute_standstill_trajectory(self) -> TrajectorySample:
        """
        Computes a standstill trajectory if the vehicle is already at velocity 0
        :return: The TrajectorySample for a standstill trajectory
        """
        # current planner initial state
        x_0 = self.x_0
        x_0_lon, x_0_lat = self.x_0_cl

        # create artificial standstill trajectory
        logger.info("Adding standstill trajectory")
        logger.info(f"Initial state: x_0 is {x_0}")
        logger.info(f"Longitudinal initial state is x_0_lon is {x_0_lon}")
        logger.info(f"Lateral initial state is x_0_lat is {x_0_lat}")

        # create lon and lat polynomial
        traj_lon = QuarticTrajectory(tau_0=0, delta_tau=self.horizon, x_0=np.asarray(x_0_lon),
                                     x_d=np.array([0, 0]))
        traj_lat = QuinticTrajectory(tau_0=0, delta_tau=self.horizon, x_0=np.asarray(x_0_lat),
                                     x_d=np.array([x_0_lat[0], 0, 0]))

        # compute initial ego curvature (global coordinates) from initial steering angle
        kappa_0 = np.tan(x_0.steering_angle) / self.vehicle_params.wheelbase

        # create Trajectory sample
        p = TrajectorySample(self.horizon, self.dt, traj_lon, traj_lat)

        # create Cartesian trajectory sample
        a = np.repeat(0.0, self.N)
        a[1] = - self.x_0.velocity / self.dt
        p.cartesian = CartesianSample(np.repeat(x_0.position[0], self.N), np.repeat(x_0.position[1], self.N),
                                      np.repeat(x_0.orientation, self.N), np.repeat(0.0, self.N),
                                      a, np.repeat(kappa_0, self.N), np.repeat(0.0, self.N),
                                      current_time_step=self.N)

        # create Curvilinear trajectory sample
        # compute orientation in curvilinear coordinate frame
        s_idx = np.argmax(self._co.ref_pos > x_0_lon[0]) - 1
        ref_theta = np.unwrap(self._co.ref_theta)
        theta_cl = x_0.orientation - interpolate_angle(x_0_lon[0], self._co.ref_pos[s_idx], self._co.ref_pos[s_idx + 1],
                                                       ref_theta[s_idx], ref_theta[s_idx + 1])

        p.curvilinear = CurviLinearSample(np.repeat(x_0_lon[0], self.N), np.repeat(x_0_lat[0], self.N),
                                          np.repeat(theta_cl, self.N), dd=np.repeat(x_0_lat[1], self.N),
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
            t = np.arange(0, np.round(trajectory.trajectory_long.delta_tau + self.dt, 5), self.dt)
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
            if not self._low_vel_mode:
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

            # precision for near zero velocities from evaluation of polynomial coefficients
            # set small velocities to zero
            s_velocity[np.abs(s_velocity) < _EPS] = 0.0
            d_velocity[np.abs(d_velocity) < _EPS] = 0.0

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
                    self._infeasible_reason_dict["acceleration"] += 1
                    feasible = False
                    continue
                if np.any(s_velocity < -_EPS):
                    self._infeasible_reason_dict["velocity"] += 1
                    feasible = False
                    continue

            for i in range(0, traj_len):
                # compute orientations
                # see Appendix A.1 of Moritz Werling's PhD Thesis for equations
                if not self._low_vel_mode:
                    if s_velocity[i] > 0.001:
                        dp = d_velocity[i] / s_velocity[i]
                    else:
                        # TODO Find better way to handle this
                        if abs(d_velocity[i]) > 0.001:
                            dp = 0.
                        else:
                            dp = 0.
                    # see Eq. (A.8) from Moritz Werling's Diss
                    ddot = d_acceleration[i] - dp * s_acceleration[i]

                    if s_velocity[i] > 0.001:
                        dpp = ddot / (s_velocity[i] ** 2)
                    else:
                        # TODO Find better way to handle this
                        if np.abs(ddot) > 0.00003:
                            dpp = 0.
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
                    if self._low_vel_mode:
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
                        theta_gl[i] = self.x_0.orientation if i == 0 else theta_gl[i - 1]

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
                v[i] = s_velocity[i] * (oneKrD / (math.cos(theta_cl[i])))

                # compute (global) Cartesian acceleration
                a[i] = s_acceleration[i] * oneKrD / cosTheta + ((s_velocity[i] ** 2) / cosTheta) * (
                        oneKrD * tanTheta * (kappa_gl[i] * oneKrD / cosTheta - k_r) - (
                        k_r_d * d[i] + k_r * dp))

                # CHECK KINEMATIC CONSTRAINTS (remove infeasible trajectories)
                if feasible:
                    feasible = self._check_constraints(v, kappa_gl, theta_gl, a, i)
                if not feasible:
                    trajectory.feasibility_label = FeasibilityStatus.INFEASIBLE_KINEMATIC
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
                        logger.debug("<ReactivePlanner.check_kinematics()> Out of projection domain")
                        break

                if feasible:
                    trajectory.feasibility_label = FeasibilityStatus.FEASIBLE
                    # store Cartesian trajectory
                    trajectory.cartesian = CartesianSample(x, y, theta_gl, v, a, kappa_gl,
                                                           kappa_dot=np.append([0], np.diff(kappa_gl)),
                                                           current_time_step=traj_len)

                    # store Curvilinear trajectory
                    trajectory.curvilinear = CurviLinearSample(s, d, theta_cl,
                                                               ss=s_velocity, sss=s_acceleration,
                                                               dd=d_velocity, ddd=d_acceleration,
                                                               current_time_step=traj_len)

                    # check if trajectories planning horizon is shorter than expected and extend if necessary
                    if self.N + 1 > trajectory.cartesian.current_time_step:
                        trajectory.enlarge(self.dt)

                    assert self.N + 1 == trajectory.cartesian.current_time_step == len(trajectory.cartesian.x) == \
                           len(trajectory.cartesian.y) == len(trajectory.cartesian.theta), \
                        '<ReactivePlanner/kinematics>:  Lenghts of state variables is not equal.'
                    # store trajectory in feasible trajectories list
                    feasible_trajectories.append(trajectory)

                if not feasible and self._draw_traj_set:
                    trajectory.feasibility_label = FeasibilityStatus.INFEASIBLE_KINEMATIC
                    # store Cartesian trajectory
                    trajectory.cartesian = CartesianSample(x, y, theta_gl, v, a, kappa_gl,
                                                           kappa_dot=np.append([0], np.diff(kappa_gl)),
                                                           current_time_step=traj_len)

                    # store Curvilinear trajectory
                    trajectory.curvilinear = CurviLinearSample(s, d, theta_cl,
                                                               ss=s_velocity, sss=s_acceleration,
                                                               dd=d_velocity, ddd=d_acceleration,
                                                               current_time_step=traj_len)

                    # check if trajectories planning horizon is shorter than expected and extend if necessary
                    if self.N + 1 > trajectory.cartesian.current_time_step:
                        trajectory.enlarge(self.dt)

                    # store trajectory in infeasible trajectories list
                    infeasible_trajectories.append(trajectory)

        if self.config.debug.multiproc:
            # store feasible trajectories in Queue 1
            queue_1.put(feasible_trajectories)
            # if visualization is required: store infeasible trajectories in Queue 1
            if self._draw_traj_set:
                queue_2.put(infeasible_trajectories)
        else:
            return feasible_trajectories, infeasible_trajectories

    def _check_constraints(self, v: np.ndarray, kappa_gl: np.ndarray, theta_gl: np.ndarray, a: np.ndarray, i: int) \
            -> bool:
        """
        Checks kinematic constraints for a sampled trajectory at time index i
        Constraints which should be checked can be specified in config.planning.constraints_to_check
        :return: Boolean stating feasibility for the given constraint set
        """
        # velocity constraint
        if "velocity" in self.config.planning.constraints_to_check:
            if v[i] < -_EPS:
                self._infeasible_reason_dict["velocity"] += 1
                return False

        # curvature constraint
        kappa_max = np.tan(self.vehicle_params.delta_max) / self.vehicle_params.wheelbase
        if "kappa" in self.config.planning.constraints_to_check:
            if abs(kappa_gl[i]) > kappa_max:
                self._infeasible_reason_dict["kappa"] += 1
                return False

        # yaw rate (orientation change) constraint
        if "yaw_rate" in self.config.planning.constraints_to_check:
            yaw_rate = (theta_gl[i] - theta_gl[i - 1]) / self.dt if i > 0 else 0.
            theta_dot_max = kappa_max * v[i]
            if abs(round(yaw_rate, 5)) > theta_dot_max:
                self._infeasible_reason_dict["yaw_rate"] += 1
                return False

        # curvature rate constraint
        if "kappa_dot" in self.config.planning.constraints_to_check:
            steering_angle = np.arctan2(self.vehicle_params.wheelbase * kappa_gl[i], 1.0)
            kappa_dot_max = self.vehicle_params.v_delta_max / (self.vehicle_params.wheelbase *
                                                               math.cos(steering_angle) ** 2)
            kappa_dot = (kappa_gl[i] - kappa_gl[i - 1]) / self.dt if i > 0 else 0.
            if abs(kappa_dot) > kappa_dot_max:
                self._infeasible_reason_dict["kappa_dot"] += 1
                return False

        # acceleration constraint (considering switching velocity, see vehicle models documentation)
        if "acceleration" in self.config.planning.constraints_to_check:
            v_switch = self.vehicle_params.v_switch
            a_max = self.vehicle_params.a_max * v_switch / v[i] if v[i] > v_switch else self.vehicle_params.a_max
            a_min = -self.vehicle_params.a_max
            if not a_min <= a[i] <= a_max:
                self._infeasible_reason_dict["acceleration"] += 1
                return False
        return True

    def _check_collisions(self, trajectory_bundle: TrajectoryBundle) -> Union[TrajectorySample, None]:
        """
        Lazy check: Iterates over the sorted list of trajectory samples and returns the first non-colliding sample.
        If all samples collide, returns None.
        :param trajectory_bundle: (Sorted) Bundle of trajectory samples
        """
        # collision object dimensions
        half_length = 0.5 * self.vehicle_params.length
        half_width = 0.5 * self.vehicle_params.width

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
                ego = pycrcc.TimeVariantCollisionObject(self.x_0.time_step + i * self.config.planning.factor)
                ego.append_obstacle(pycrcc.RectOBB(half_length, half_width, theta[i], pos1[i], pos2[i]))
                if self._cc.collide(ego):
                    self._infeasible_count_collision += 1
                    collide = True
                    trajectory.feasibility_label = FeasibilityStatus.INFEASIBLE_COLLISION
                    break

            # additional continuous collision check if no collision has been detected before already
            if self.config.planning.continuous_collision_check and not collide:
                ego_tvo = pycrcc.TimeVariantCollisionObject(self.x_0.time_step)
                [ego_tvo.append_obstacle(
                    pycrcc.RectOBB(half_length, half_width, theta[i], pos1[i], pos2[i])) for i in range(len(pos1))]
                ego_tvo, err = trajectory_preprocess_obb_sum(ego_tvo)
                if self._cc.collide(ego_tvo):
                    self._infeasible_count_collision += 1
                    collide = True
                    trajectory.feasibility_label = FeasibilityStatus.INFEASIBLE_COLLISION
                    break

            if not collide:
                logger.info(f"Collision checks took:  \t{time.time() - t0:.7f}s")
                return trajectory
        return None

    def _get_optimal_trajectory(self, trajectory_bundle: TrajectoryBundle) -> Union[TrajectorySample, None]:
        """
        Computes the optimal trajectory from a given trajectory bundle
        :param trajectory_bundle: The trajectory bundle
        :return: The optimal trajectory if exists (otherwise None)
        """
        logger.info("===== Checking trajectories ... =====")
        # reset statistics
        self._reset_statistics()

        # Filter trajectories backwards if the car is in the goal region
        if self.config.sampling.longitudinal_mode == 'stopping':
            trajectory_bundle.filter_goals_behind()

        num_workers = self.config.debug.num_workers

        # ==== Kinematic checking
        # check kinematics of each trajectory
        t0 = time.time()
        if self.config.debug.multiproc:
            # with multiprocessing
            # divide trajectory_bundle.trajectories into chunks
            chunk_size = math.ceil(len(trajectory_bundle.trajectories) / num_workers)
            chunks = [trajectory_bundle.trajectories[ii * chunk_size: min(len(trajectory_bundle.trajectories),
                                                                          (ii + 1) * chunk_size)] for ii in
                      range(0, num_workers)]

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

        logger.info(f"Kinematic checks took:  \t{time.time() - t0:.7f}s")

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
        collision_free_trajectory: Optional[TrajectorySample] = self._check_collisions(trajectory_bundle)
        return collision_free_trajectory

    def convert_state_list_to_commonroad_object(self, state_list: List[ReactivePlannerState], obstacle_id: int = 42):
        """
        Converts a CR trajectory to a CR dynamic obstacle with given dimensions
        :param state_list: trajectory state list of reactive planner
        :param obstacle_id: [optional] ID of ego vehicle dynamic obstacle
        :return: CR dynamic obstacle representing the ego vehicle
        """
        # shift trajectory positions to center
        new_state_list = list()
        for state in state_list:
            new_state_list.append(state.shift_positions_to_center(self.vehicle_params.wb_rear_axle))

        trajectory = Trajectory(initial_time_step=new_state_list[0].time_step, state_list=new_state_list)
        # get shape of vehicle
        shape = Rectangle(self.vehicle_params.length, self.vehicle_params.width)
        # get trajectory prediction
        prediction = TrajectoryPrediction(trajectory, shape)
        return DynamicObstacle(obstacle_id, ObstacleType.CAR, shape, trajectory.state_list[0], prediction)
