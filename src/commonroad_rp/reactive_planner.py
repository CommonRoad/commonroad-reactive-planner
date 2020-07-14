__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.5"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Beta"

# python packages
from typing import List, Tuple
import cProfile
import pycrcc
import time
from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.validity import *
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory, State

# commonroad imports
from commonroad_rp.parameter import DefFailSafeSampling, VehModelParameters, DefGymSampling, TimeSampling, VelocitySampling
from commonroad_rp.polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from commonroad_rp.trajectories import TrajectoryBundle, TrajectorySample, CartesianSample, CurviLinearSample
from commonroad_rp.cost_function import DefaultCostFunctionFailSafe, DefaultCostFunction
from commonroad_rp.utils import CoordinateSystem, interpolate_angle

from route_planner.route_planner import RoutePlanner

_LOW_VEL_MODE = False


class ReactivePlanner(object):
    """
    Reactive planner class that plans trajectories in a sampling-based fashion
    """

    def __init__(self, scenario, planning_problem, route_planner: RoutePlanner, dt: float, t_h: float, N: int, v_desired=14, collision_check_in_cl: bool = False,
                 factor: int = 1, replanning_cycle_steps: int = 2):
        """
        Constructor of the reactive planner
        :param dt: The time step of planned trajectories
        :param t_h: The time horizon of planned trajectories
        :param N: The number of steps of planned trajectories (int)
        :param v_desired: The desired velocity of for planning trajectories
        :param collision_check_in_cl: Boolean if the collision checker has been provided in curvilinear
        coordinates (default=False)
        :param factor: Factor when performing collision checks and the scenario is given in a different time
        step, e.g., 0.02 for scenario and 0.2 for planner results in a factor of 10.
        :param replanning_cycle_steps: replanning should be done every this number of time steps.
        """

        assert is_positive(dt), '<ReactivePlanner>: provided dt is not correct! dt = {}'.format(dt)
        assert is_positive(N) and is_natural_number(N), '<ReactivePlanner>: provided N is not correct! dt = {}'.format(
            N)
        assert is_positive(t_h), '<ReactivePlanner>: provided t_h is not correct! dt = {}'.format(dt)
        assert np.isclose(t_h, N * dt), '<ReactivePlanner>: Provided time horizon information is not correct!'
        # assert np.isclose(N * dt, t_h)

        # Set horizon variables
        self.horizon = t_h
        self.N = N
        self.dT = dt
        self._factor = factor

        # Create default VehModelParameters
        self.constraints = VehModelParameters()
        # Set width and length
        self._width = self.constraints.veh_width
        self._length = self.constraints.veh_length

        # Current State
        self.x_0: State = None

        # store feasible trajectories of last run
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0

        # store collision checker coordinate system information
        self._collision_check_in_cl = collision_check_in_cl

        # store desired velocity
        self._desired_speed = v_desired
        self._desired_d = 0.
        self._desired_t = self.horizon

        # Default sampling -> [desired - min,desired + max,initial step]
        fs_sampling = DefGymSampling(dt, t_h)
        self._sampling_d = fs_sampling.d_samples
        self._sampling_t = fs_sampling.t_samples
        self._sampling_v = fs_sampling.v_samples

        # sampling levels
        self._sampling_level = fs_sampling.max_iteration

        # coordinate system
        self._co = None

        # DEBUG Mode
        self._DEBUG = False

        # planning problem
        self.planningProblem = planning_problem
        self.scenario = scenario
        self.route_planner = route_planner
        self.ref_route_manager = ReferenceRouteManager(self.route_planner)
        self.replanning_cycle_steps = replanning_cycle_steps
        self.laneChanging = False

    def set_t_sampling_parameters(self, t_min, dt, horizon):
        self._sampling_t = TimeSampling(t_min, horizon, self._sampling_level, dt)
        self.N = int(round(horizon / dt))
        self.horizon = horizon

    def set_reference_path(self, reference_path: np.ndarray):
        """
        Sets the reference path and automatically creates a coordinate system
        :param reference_path: The reference path
        """
        self._co: CoordinateSystem = CoordinateSystem(reference_path)

    def set_desired_velocity(self, desired_velocity: float, current_speed: float=None, stopping: bool=False):
        """
        Sets desired velocity and calculates velocity for each sample
        :param desired_velocity: velocity in m/s
        :return: velocity in m/s
        """
        self._desired_speed = desired_velocity
        if not stopping:
            if current_speed is not None:
                reference_speed = current_speed
            else:
                reference_speed = self._desired_speed

            min_v = max(0, reference_speed - (0.125 * self.horizon * self.constraints.a_max))
            # max_v = max(min_v + 5.0, reference_speed + (0.25 * self.horizon * self.constraints.a_max))
            max_v = max(min_v + 5.0, reference_speed + 2)
            self._sampling_v = VelocitySampling(min_v, max_v, self._sampling_level)
        else:
            self._sampling_v = VelocitySampling(self._desired_speed, self._desired_speed, self._sampling_level)
        return self._sampling_v

    def coordinate_system(self) -> CoordinateSystem:
        """
        Returns the created coordinate system of the reactive planner
        :return: The coordinate system
        """
        return self._co

    def no_of_samples(self, samp_level: int) -> int:
        """
        Returns the number of samples for a given sampling level
        :param samp_level: The sampling level
        :return: Number of trajectory samples for given sampling level
        """
        # return len(self._v_sets[samp_level]) * len(self._d_sets[samp_level]) * len(self._t_sets[samp_level])
        return len(self._sampling_v.to_range(samp_level)) * len(self._sampling_d.to_range(samp_level)) * len(
            self._sampling_t.to_range(samp_level))

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

        # reset cost statistic
        self._min_cost = 10 ** 9
        self._max_cost = 0

        trajectories = list()
        # for t in self._sampling_t.to_range(samp_level):
        t = self.horizon
            # Longitudinal sampling for all possible velocities
        for v in self._sampling_v.to_range(samp_level):
            #end_state_lon = np.array([t * v + x_0_lon[0], v, 0.0])
            #trajectory_long = QuinticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lon), x_d=end_state_lon)
            trajectory_long = QuarticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lon), x_d=np.array([v, 0]))

            # Sample lateral end states (add x_0_lat to sampled states)
            if trajectory_long.coeffs is not None:
                for d in self._sampling_d.to_range(samp_level).union({x_0_lat[0]}):

                    end_state_lat = np.array([d, 0.0, 0.0])
                    # SWITCHING TO POSITION DOMAIN FOR LATERAL TRAJECTORY PLANNING
                    if _LOW_VEL_MODE:
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
        trajectory_bundle = TrajectoryBundle(trajectories, cost_function=DefaultCostFunction(self._desired_speed))

        return trajectory_bundle

    def draw_trajectory_set(self, trajectory_bundle: List[TrajectorySample], step=2):
        """
        Draws the current feasible trajectory set
        :param step: Choose if you only want to plot every "step" trajectory => default 2
        :return:
        """
        if trajectory_bundle is not None:
            for i in range(0, len(trajectory_bundle), step):
                color = 'gray'
                plt.plot(trajectory_bundle[i].cartesian.x, trajectory_bundle[i].cartesian.y,
                         color=color)

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
            print(self.scenario.benchmark_id)
            # print(self._co._reference[0])
            # print(self._co._reference[-1])
            if self._co._reference[0][0] > x_0.position[0]:
                reference_path = np.concatenate((tmp, self._co._reference), axis=0)
            else:
                reference_path = np.concatenate((self._co._reference, tmp), axis=0)
            self.set_reference_path(reference_path)
            s, d = self._co.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])

        # compute orientation in curvilinear coordinate frame
        ref_theta = self._co.ref_theta()
        for i in range(len(ref_theta)):
            if ref_theta[i] > np.pi:
                ref_theta[i] -= 2 * np.pi
            if ref_theta[i] < -np.pi:
                ref_theta[i] += 2 * np.pi
        theta_cl = x_0.orientation - np.interp(s, self._co.ref_pos(), ref_theta)
        # compute curvatures
        kr = np.interp(s, self._co.ref_pos(), self._co.ref_curv())
        kr_d = np.interp(s, self._co.ref_pos(), self._co.ref_curv_d())

        # compute d prime and d prime prime -> derivation after arclength
        d_p = (1 - kr * d) * np.tan(theta_cl)
        d_pp = -(kr_d * d + kr * d_p) * np.tan(theta_cl) + ((1 - kr * d) / (np.cos(theta_cl) ** 2)) * (
                x_0.yaw_rate * (1 - kr * d) / np.cos(theta_cl) - kr)

        # compute s dot and s dot dot -> derivation after time
        s_d = x_0.velocity * np.cos(theta_cl) / (1 - kr * d)
        if s_d < 0:
            # print(x_0.velocity)
            # print(x_0.time_step)
            # print(x_0.position)
            # print(x_0.orientation)
            # print(s)
            # print(self._co.ref_pos())
            # print(self._co.ref_theta())
            # print(self._co.ref_curv())
            # print(self._co._reference)
            print(self.scenario.benchmark_id)
            # print(theta_cl)
            # print(kr)
            # print(d)
            # print(s_d)
            raise Exception(
                "Initial state or reference incorrect! Curvilinear velocity is negative which indicates that the "
                "ego vehicle is not driving in the same direction as specified by the reference")

        s_dd = x_0.acceleration
        s_dd -= (s_d ** 2 / np.cos(theta_cl)) * (
                (1 - kr * d) * np.tan(theta_cl) * (x_0.yaw_rate * (1 - kr * d) / (np.cos(theta_cl)) - kr) -
                (kr_d * d + kr * d_p))
        s_dd /= ((1 - kr * d) / (np.cos(theta_cl)))

        d_d = x_0.velocity * np.sin(theta_cl)
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
            # cart_states['time_step'] = self.x_0.time_step+self._factor*i
            cart_states['time_step'] = self._factor * i
            cart_states['position'] = np.array([trajectory.cartesian.x[i], trajectory.cartesian.y[i]])
            cart_states['velocity'] = trajectory.cartesian.v[i]
            cart_states['acceleration'] = trajectory.cartesian.a[i]
            cart_states['orientation'] = trajectory.cartesian.theta[i]
            cart_states['yaw_rate'] = trajectory.cartesian.kappa[i]
            cart_states['slip_angle'] = self.x_0.slip_angle
            cart_list.append(State(**cart_states))

            # create curvilinear state
            cl_states = dict()
            # cl_states['time_step'] = self.x_0.time_step+self._factor*i
            cl_states['time_step'] = self._factor * i
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

        # cartTraj = Trajectory(self.x_0.time_step, cart_list)
        # freTraj = Trajectory(self.x_0.time_step, cl_list)
        cartTraj = Trajectory(0, cart_list)
        freTraj = Trajectory(0, cl_list)

        return (cartTraj, freTraj, lon_list, lat_list)

    def plan(self, x_0: State, cc: object, cl_states=None) -> tuple:
        """
        Plans an optimal trajectory
        :param x_0: Initial state as CR state
        :param cc:  CollisionChecker object
        :param cl_states: Curvilinear state if re-planning is used
        :return: Optimal trajectory as tuple
        """
        self.x_0 = x_0

        # compute initial states
        if cl_states is not None:
            x_0_lon, x_0_lat = cl_states
        else:
            x_0_lon, x_0_lat = self._compute_initial_states(x_0)

        if self._DEBUG:
            print('<Reactive Planner>: initial state is: lon = {} / lat = {}'.format(x_0_lon, x_0_lat))
            print('<Reactive Planner>: desired velocity is {} m/s'.format(self._desired_speed))

        # optimal trajectory dummy
        optimal_trajectory = None

        # initial index of sampling set to use
        i = 0

        # sample until trajectory has been found or sampling sets are empty
        while optimal_trajectory is None and i < self._sampling_level:
            if self._DEBUG:
                print('<ReactivePlanner>: Starting at sampling density {} of {}'.format(i + 1, self._sampling_level))
                print('<ReactivePlanner>: Sampling {} trajectories'.format(self.no_of_samples(i)))

            # plan trajectory bundle
            bundle = self._create_trajectory_bundle(self._desired_speed, x_0_lon, x_0_lat, samp_level=i)

            # get optimal trajectory
            t0 = time.time()
            # profiler = cProfile.Profile()
            # profiler.enable()
            optimal_trajectory = self._get_optimal_trajectory(bundle, cc)
            if self._DEBUG:
                print('<ReactivePlanner>: Checked trajectories in {} seconds'.format(time.time() - t0))
            # profiler.disable()
            # profiler.print_stats("time")
            # self.draw_trajectory_set(bundle.trajectories)
            # plt.pause(50)
            # print statistics
            if self._DEBUG:
                print('<ReactivePlanner>: Rejected {} infeasible trajectories due to kinematics'.format(
                    self.no_of_infeasible_trajectories_kinematics()))
                print('<ReactivePlanner>: Rejected {} infeasible trajectories due to collisions'.format(
                    self.no_of_infeasible_trajectories_collision()))

            i = i + 1
        if optimal_trajectory is None and x_0.velocity <= 0.1:
            # for x_i in x_0_lon:
            #     print('<ReactivePlanner>: x_lon list for planning standstill with x_i type {}'.format(type(x_i)))
            print('<ReactivePlanner>: planning standstill for scenario {}'.format(self.scenario.benchmark_id))
            optimal_trajectory = self._compute_standstill_trajectory(x_0, x_0_lon, x_0_lat)

        # check if feasible trajectory exists -> emergency mode
        if optimal_trajectory is None:
            if self._DEBUG:
                print('<ReactivePlanner>: Could not find any trajectory out of {} trajectories'.format(
                    sum([self.no_of_samples(i) for i in range(self._sampling_level)])))
                print('<ReactivePlanner>: Cannot find trajectory with default sampling parameters. '
                      'Switching to emergency mode!')
        else:
            if self._DEBUG:
                print(
                    '<ReactivePlanner>: Found optimal trajectory with costs = {}, which '
                    'corresponds to {} percent of seen costs'.format(
                        optimal_trajectory.cost,
                        ((optimal_trajectory.cost - bundle.min_costs().cost) / (
                                bundle.max_costs().cost - bundle.min_costs().cost))))

        return self._compute_trajectory_pair(optimal_trajectory) if optimal_trajectory is not None else None

    def re_plan(self, x_0: State, cc: object) -> Tuple:
        """
        Re-plans after each given time horizon.
        :param x_0: Initial state as CR state
        :param cc:  CollisionChecker object
        :return: Optimal trajectory as list
        """
        self.x_0 = deepcopy(x_0)

        # initialization
        planned_states = list()
        planned_states.append(self.x_0)
        cl_states = list()
        counter = 0
        ref_path = list()
        ref_path.append(self.ref_route_manager.get_ref_path())
        self.set_reference_path(ref_path[0])
        s_0, d_0 = self._co.convert_to_curvilinear_coords(x_0.position[0], x_0.position[1])
        x_0.position[0] = s_0
        x_0.position[1] = d_0
        cl_states.append(x_0)

        # for i in range(60):
        for i in range(self.planningProblem.goal.state_list[0].time_step.end):
            # optimal = self.plan(self.x_0, cc, cl_states)
            if i < counter:
                continue
            optimal = self.plan(self.x_0, cc)
            if optimal:
                for j in range(self.replanning_cycle_steps):
                    planned_state = self.shift_orientation(optimal[0]).state_list[j + 1]
                    planned_state.time_step = i + j + 1
                    planned_states.append(planned_state)
                    cl_states.append(optimal[1].state_list[j + 1])

                ref_lanelet_id = self.check_ref_lanelet_id(planned_states[-1])

                if self.laneChanging and abs(cl_states[-1].position[1]) < 0.5:
                    new_ref = self.ref_route_manager.switch_ref_path(ref_lanelet_id)
                    self.set_reference_path(new_ref)

            else:
                ref_lanelet_id = self.check_ref_lanelet_id(planned_states[-1])
                new_ref = self.ref_route_manager.switch_ref_path(ref_lanelet_id, cl_state=cl_states[-1])
                self.set_reference_path(new_ref)
                optimal = self.plan(self.x_0, cc)
                if optimal:
                    for j in range(self.replanning_cycle_steps):
                        planned_state = self.shift_orientation(optimal[0]).state_list[j + 1]
                        planned_state.time_step = i + j + 1
                        planned_states.append(planned_state)
                        cl_states.append(optimal[1].state_list[j + 1])
                    new_ref = self.ref_route_manager.switch_ref_path(ref_lanelet_id, cl_state=cl_states[-1])
                    self.set_reference_path(new_ref)
                else:
                    break
            # Shift the initial state of the planning problem to run the next cycle.
            counter = len(planned_states) - 1
            self.x_0 = planned_states[-1]

        return planned_states, ref_path

    def check_ref_lanelet_id(self, current_state):
        current_lanelet_id = self.route_planner.lanelet_network.find_lanelet_by_position(
            list([current_state.position]))[0][0]
        ref_lanelet_id = current_lanelet_id
        if current_lanelet_id in self.ref_route_manager.route:
            self.laneChanging = self.ref_route_manager.lane_change_request[current_lanelet_id]
        else:
            current_lanelet = self.route_planner.lanelet_network.find_lanelet_by_id(current_lanelet_id)
            if current_lanelet.adj_left is not None and current_lanelet.adj_left in self.ref_route_manager.route:
                self.laneChanging = self.ref_route_manager.lane_change_request[current_lanelet.adj_left]
                ref_lanelet_id = current_lanelet.adj_left
            else:
                self.laneChanging = self.ref_route_manager.lane_change_request[current_lanelet.adj_right]
                ref_lanelet_id = current_lanelet.adj_right

        return ref_lanelet_id

    def _compute_standstill_trajectory(self, x_0, x_0_lon, x_0_lat) -> TrajectorySample:
        """
        Computes a standstill trajectory if the vehicle is already at velocity 0
        :param x_0: The current state of the ego vehicle
        :param x_0_lon: The longitudinal state in curvilinear coordinates
        :param x_0_lat: The lateral state in curvilinear coordinates
        :return: The TrajectorySample for a standstill trajectory
        """
        # create artifical standstill trajectory
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
        traj_lat = QuinticTrajectory(tau_0=0, delta_tau=self.horizon, x_0=np.asarray(x_0_lat), x_d=np.array([x_0_lat[0], 0, 0]))
        p = TrajectorySample(self.horizon, self.dT, traj_lon, traj_lat)
        p.cartesian = CartesianSample(np.repeat(x_0.position[0], self.N), np.repeat(x_0.position[1], self.N),
                                      np.repeat(x_0.orientation, self.N), np.repeat(0, self.N),
                                      np.repeat(0, self.N), np.repeat(0, self.N), np.repeat(0, self.N))
        p.curvilinear = CurviLinearSample(np.repeat(x_0_lon[0], self.N), np.repeat(x_0_lat[0], self.N),
                                          np.repeat(x_0.orientation, self.N), np.repeat(x_0_lat[1], self.N),
                                          np.repeat(x_0_lat[2], self.N), np.repeat(x_0_lon[1], self.N),
                                          np.repeat(x_0_lon[2], self.N))
        return p

    def check_kinematics(self, trajectory_bundle: TrajectoryBundle) -> List[TrajectorySample]:
        """
        Checks the kinematics of given trajectories in a bundle and computes the cartesian trajectory information
        :param trajectory_bundle: The trajectory bundle to check
        :return: The list of trajectories which are kinematically feasible
        """

        feasible_trajectories = list()
        for trajectory in trajectory_bundle.trajectories:
            # create time array and precompute time interval information
            t = np.arange(0, np.round(trajectory.trajectory_long.delta_tau + self.dT, 5), self.dT)
            t2 = np.square(t)
            t3 = t2 * t
            t4 = np.square(t2)
            t5 = t4 * t

            # compute position, velocity, acceleration from trajectory sample
            s = trajectory.trajectory_long.calc_position(t, t2, t3, t4, t5)  # lon pos
            s_velocity = trajectory.trajectory_long.calc_velocity(t, t2, t3, t4)  # lon velocity
            s_acceleration = trajectory.trajectory_long.calc_acceleration(t, t2, t3)  # lon acceleration

            # At low speeds, we have to sample the lateral motion over the travelled distance rather than time.
            if not _LOW_VEL_MODE:
                d = trajectory.trajectory_lat.calc_position(t, t2, t3, t4, t5)  # lat pos
                d_velocity = trajectory.trajectory_lat.calc_velocity(t, t2, t3, t4)  # lat velocity
                d_acceleration = trajectory.trajectory_lat.calc_acceleration(t, t2, t3)  # lat acceleration

            else:
                # compute normalized travelled distance for low velocity mode of lateral planning
                s1 = s - s[0]
                s2 = np.square(s1)
                s3 = s2 * s1
                s4 = np.square(s2)
                s5 = s4 * s1

                d = trajectory.trajectory_lat.calc_position(s1, s2, s3, s4, s5)  # lat pos
                d_velocity = trajectory.trajectory_lat.calc_velocity(s1, s2, s3, s4)  # lat velocity
                d_acceleration = trajectory.trajectory_lat.calc_acceleration(s1, s2, s3)  # lat acceleration

            # Compute cartesian information of trajectory
            s_length = len(s)
            x = np.zeros(s_length)
            y = np.zeros(s_length)
            theta_gl = np.zeros(s_length)
            theta_cl = np.zeros(s_length)
            v = np.zeros(s_length)
            a = np.zeros(s_length)
            kappa_gl = np.zeros(s_length)
            kappa_cl = np.zeros(s_length)
            feasible = True
            for i in range(0, s_length):
                # compute Global position
                pos: np.ndarray = self._co.convert_to_cartesian_coords(s[i], d[i])

                if pos is not None:
                    x[i] = pos[0]
                    y[i] = pos[1]
                else:
                    feasible = False
                    break

                # compute orientations
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

                if s_idx + 1 >= len(self._co.ref_pos()):
                    feasible = False
                    break

                s_lambda = (self._co.ref_pos()[s_idx] - s[i]) / (
                            self._co.ref_pos()[s_idx + 1] - self._co.ref_pos()[s_idx])

                # add cl and gl orientation
                if s_velocity[i] > 0.005:
                    if _LOW_VEL_MODE:
                        theta_cl[i] = np.arctan2(dp, 1.0)
                    else:
                        theta_cl[i] = np.arctan2(d_velocity[i], s_velocity[i])
                    theta_gl[i] = theta_cl[i] + interpolate_angle(
                            s[i],
                            self._co.ref_pos()[s_idx],
                            self._co.ref_pos()[s_idx + 1],
                            self._co.ref_theta()[s_idx],
                            self._co.ref_theta()[s_idx + 1]
                        )
                    if theta_gl[i] < -np.pi:
                        theta_gl[i] += 2 * np.pi
                    if theta_gl[i] > np.pi:
                        theta_gl[i] -= 2 * np.pi
                    # theta_gl[i] = theta_cl[i] + (
                    #         self._co.ref_theta()[s_idx + 1] - self._co.ref_theta()[s_idx]) * s_lambda + \
                    #               self._co.ref_theta()[s_idx]

                else:
                    # theta_cl.append(np.interp(s[i], self._co.ref_pos(), self._co.ref_theta()))
                    # theta_cl[i] = (self._co.ref_theta()[s_idx + 1] - self._co.ref_theta()[s_idx]) * s_lambda + \
                    #               self._co.ref_theta()[s_idx]
                    theta_cl[i] = interpolate_angle(
                            s[i],
                            self._co.ref_pos()[s_idx],
                            self._co.ref_pos()[s_idx + 1],
                            self._co.ref_theta()[s_idx],
                            self._co.ref_theta()[s_idx + 1]
                        )
                    if theta_cl[i] < -np.pi:
                        theta_cl[i] += 2 * np.pi
                    if theta_cl[i] > np.pi:
                        theta_cl[i] -= 2 * np.pi
                    theta_gl[i] = theta_cl[i]

                # Compute curvature of reference at current position
                k_r = (self._co.ref_curv()[s_idx + 1] - self._co.ref_curv()[s_idx]) * s_lambda + self._co.ref_curv()[
                    s_idx]
                k_r_d = (self._co.ref_curv_d()[s_idx + 1] - self._co.ref_curv_d()[s_idx]) * s_lambda + \
                        self._co.ref_curv_d()[s_idx]

                # compute global curvature based on appendix A of Moritz Werling's PhD thesis
                oneKrD = (1 - k_r * d[i])
                cosTheta = np.cos(theta_cl[i])
                tanTheta = np.tan(theta_cl[i])
                kappa_gl[i] = (dpp + k_r * dp * tanTheta) * cosTheta * (cosTheta / oneKrD) ** 2 + (
                        cosTheta / oneKrD) * k_r
                kappa_cl[i] = kappa_gl[i] - k_r

                # velocity
                v[i] = s_velocity[i] * (oneKrD / (np.cos(theta_cl[i])))

                # compute acceleration
                a[i] = s_acceleration[i] * oneKrD / cosTheta + ((s_velocity[i] ** 2) / cosTheta) * (
                        oneKrD * tanTheta * (kappa_gl[i] * oneKrD / cosTheta - k_r) - (
                        k_r_d * d[i] + k_r * d_velocity[i]))

                DEBUG = False

                # check kinematics to already discard infeasible trajectories
                if abs(kappa_gl[i] > self.constraints.kappa_max):
                    if DEBUG:
                        print(f"Kappa {kappa_gl[i]}")
                    feasible = False
                    break
                if abs((kappa_gl[i] - kappa_gl[i - 1]) / self.dT if i > 0 else 0.) > self.constraints.kappa_dot_max:
                    if DEBUG:
                        print(f"KappaDOT {abs((kappa_gl[i] - kappa_gl[i - 1]) / self.dT if i > 0 else 0.)}")
                    feasible = False
                    break
                if abs(a[i]) > self.constraints.a_max:
                    if DEBUG:
                        print(f"Acceleration {a[i]}")
                    feasible = False
                    break
                if abs(v[i]) < -0.1:
                    if DEBUG:
                        print(f"Velocity {v[i]}")
                    feasible = False
                    break

                if abs((theta_gl[i - 1] - theta_gl[i]) / self.dT if i > 0 else 0.) > self.constraints.theta_dot_max:
                    if DEBUG:
                        print(f"Theta_dot {(theta_gl[i - 1] - theta_gl[i]) / self.dT if i > 0 else 0.}")
                    feasible = False
                    break

            if feasible:
                # store Cartesian trajectory
                trajectory.cartesian = CartesianSample(x, y, theta_gl, v, a, kappa_gl,
                                                       np.append([0], np.diff(kappa_gl)))

                # store Curvilinear trajectory
                trajectory.curvilinear = CurviLinearSample(s, d, theta_gl, ss=s_velocity, sss=s_acceleration,
                                                           dd=d_velocity,
                                                           ddd=d_acceleration)

                # check if trajectories planning horizon is shorter than expected and extend if necessary
                # if self.horizon > trajectory.trajectory_long.delta_tau:
                if self.N + 1 > len(trajectory.cartesian.x):
                    trajectory.enlarge(self.N + 1 - len(trajectory.cartesian.x), self.dT)

                if self.N + 1 != len(trajectory.cartesian.x):
                    print(self.N)
                    print(self.horizon)
                    print(self.dT)
                    print(self.horizon/self.dT)
                    print(len(trajectory.cartesian.x))
                    print(self._sampling_t._db)
                    print(t)
                assert self.N + 1 == len(trajectory.cartesian.x) == len(trajectory.cartesian.y) == len(
                    trajectory.cartesian.theta), '<ReactivePlanner/kinematics>: Trajectory computation is incorrect!' \
                                                 ' Lenghts of state variables is not equal.'
                feasible_trajectories.append(trajectory)
        return feasible_trajectories

    def _get_optimal_trajectory(self, trajectory_bundle: TrajectoryBundle, cc: pycrcc.CollisionChecker) \
            -> TrajectorySample:
        """
        Computes the optimal trajectroy from a given trajectory bundle
        :param trajectory_bundle: The trajectory bundle
        :param cc: The collision checker object for the collision-checks
        :return: The optimal trajectory if exists (otherwise None)
        """

        # reset statistics
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0

        # check kinematics of each trajectory
        if self._DEBUG:
            profiler = cProfile.Profile()
            profiler.enable()

        feasible_trajectories = self.check_kinematics(trajectory_bundle)
        self._infeasible_count_kinematics = len(trajectory_bundle.trajectories) - len(feasible_trajectories)

        # set feasible trajectories in bundle
        trajectory_bundle.trajectories = feasible_trajectories
        # sort trajectories according to their costs
        trajectory_bundle.sort()

        # self.draw_trajectory_set(feasible_trajectories)
        # plt.show(block=True)

        # go through sorted list of trajectories and check for collisions
        for trajectory in trajectory_bundle.get_sorted_list():

            pos1 = trajectory.curvilinear.s if self._collision_check_in_cl else trajectory.cartesian.x
            pos2 = trajectory.curvilinear.d if self._collision_check_in_cl else trajectory.cartesian.y
            theta = trajectory.curvilinear.theta if self._collision_check_in_cl else trajectory.cartesian.theta

            # check each pose for collisions
            collide = False
            for i in range(len(pos1)):
                ego = pycrcc.TimeVariantCollisionObject(self.x_0.time_step + i * self._factor)
                ego.append_obstacle(pycrcc.RectOBB(0.5 * self._length, 0.5 * self._width, theta[i], pos1[i], pos2[i]))
                if cc.collide(ego):
                    self._infeasible_count_collision += 1
                    collide = True
                    break
            if not collide:
                if self._DEBUG:
                    profiler.disable()
                    profiler.print_stats("time")
                return trajectory

        if self._DEBUG:
            profiler.disable()
            profiler.print_stats("time")
        return None

    def convert_cr_trajectory_to_object(self, trajectory: Trajectory):
        """
        Converts a CR trajectory to a CR dynamic obstacle with given dimensions
        :param trajectory: The trajectory of the vehicle
        :return: CR dynamic obstacles representing the ego vehicle
        """

        # get shape of vehicle
        shape = Rectangle(self._length, self._width)

        # get trajectory prediction
        prediction = TrajectoryPrediction(trajectory, shape)

        return DynamicObstacle(42, ObstacleType.CAR, shape, trajectory.state_list[0], prediction)

    def shift_orientation(self, trajectory: Trajectory, interval_start=-np.pi, interval_end=np.pi):
        for state in trajectory.state_list:
            while state.orientation < interval_start:
                state.orientation += 2 * np.pi
            while state.orientation > interval_end:
                state.orientation -= 2 * np.pi

        return trajectory


def shift_angle_to_interval(angle_list, interval_start=-np.pi, interval_end=np.pi):
    new_angle_list = np.zeros(angle_list.shape)
    for idx, angle in enumerate(angle_list):
        while angle < interval_start:
            angle += 2*np.pi
        while angle > interval_end:
            angle -= 2*np.pi
        new_angle_list[idx] = angle

    return new_angle_list


class ReferenceRouteManager(object):
    """
    Reference route manager that switches reference routes when performing lane changes. Center lines of the lanelets
    are used as reference routes. If a lane change signal is received, the lane change should be done as early as
    possible. When no feasible samples can be found during a planning cycle, the reference path will also be changed
    temporarily.
    """

    def __init__(self, route_planner: RoutePlanner):
        """
        Constructor of the reference route manager
        :param route_planner: route planner used for generating reference path
        """

        self.lane_change_request = None
        self.route_planner = route_planner
        self.ref_path = None
        self.route = None
        self.instruction = None
        self.ref_path_dict = None
        self.successor = None
        self.predecessor = None
        self.ref_path_id = None

    def get_ref_path(self):
        routes = self.route_planner.search_alg()

        if routes is not None:
            # self.ref_path = ref_path_list[0]
            self.route = routes[0]
            if len(routes) > 1:
                for i in range(1, len(routes)):
                    if self.route_planner.goal_lanelet_ids:
                        goal_lanelet_id = self.route_planner.goal_lanelet_ids[-1]
                        if goal_lanelet_id in routes[i]:
                            if len(routes[i]) < len(self.route) or goal_lanelet_id not in self.route:
                                self.route = routes[i]

        self.check_lane_change_request()
        self.check_lanelet_connection()
        self.split_route()
        self.ref_path = self.ref_path_dict[self.route[0]]
        self.ref_path_id = self.route[0]

        return self.ref_path

    def check_lane_change_request(self):
        self.instruction = self.route_planner.get_instruction_from_route(self.route)
        self.lane_change_request = {}
        for i in range(len(self.route)):
            self.lane_change_request[self.route[i]] = self.instruction[i]

        return self.lane_change_request

    def check_lanelet_connection(self):
        """
        Check the predecessors and successors of the lanelets in the route.
        """
        self.successor = {}
        self.predecessor = {}
        for i in range(len(self.route)):
            successor = self.route_planner.lanelet_network.find_lanelet_by_id(self.route[i]).successor
            predecessor = self.route_planner.lanelet_network.find_lanelet_by_id(self.route[i]).predecessor
            if len(successor) != 0:
                self.successor[self.route[i]] = successor[0]
            else:
                self.successor[self.route[i]] = 0

            if len(predecessor) != 0:
                self.predecessor[self.route[i]] = predecessor[0]
            else:
                self.predecessor[self.route[i]] = 0

    def split_route(self):
        """
        Split the route into sub-routes, inner which there is no lane change request. Sub-routes are extended by
        successors and predecessors. When performing lane change, the reference path is switched among the paths
        generated correspondingly from two sub-routes.
        Example:
            lanelets:
            _________________________________________________________
            12              | 16                | 20
            _________________________________________________________
            13              | 17                | 21
            _________________________________________________________
            14              | 18                | 22
            _________________________________________________________
            15              | 19                | 23
            _________________________________________________________

            route = [12, 16, 17, 18, 22, 23]
            instructor = [0, 1, 1, 0, 1, 0]
            successor = {12: 16         |       predecessor = {12: 0
                         16: 20         |                      16: 12
                         17: 21         |                      17: 13
                         18: 22         |                      18: 14
                         22: 0          |                      22: 18
                         23: 0}         |                      23: 19}

            Then sub-routes will be:
                sub_route_0 = [12, 16]
                sub_route_1 = [17]
                sub_route_2 = [18, 22]
                sub_route_3 = [23]

            ref_path_dict will be: (dictionary whose keys are lanelet ids and values are ref_route or pointers)
            {12: ref_path_0 of lanelets 12, 16, 20 (ndarray, extended with successor of lanelet 16)
             16: [12, 17] (list of lanelet ids pointing to the first lanelet of the current and next routes)
             17: ref_path_1 of lanelet 13, 17, 21 (ndarray, extended with predecessor and successor of lanelet 17)
             18: ref_path_2 of lanelets 14, 18, 22 (ndarray, extended with predecessor of lanelet 18)
             22: [18, 23] (list)
             23: ref_path_3 of lanelet 19, 23 (ndarray, extended with predecessor of lanelet 23)}
        """
        self.ref_path_dict = {}
        k = 0
        sub_routes = {k: list([self.route[0]])}
        pointer = self.route[0]
        for i in range(len(self.route) - 1):
            if pointer != self.route[-1]:
                for j in range(self.route.index(pointer), len(self.route) - 1):
                    if not self.instruction[j]:
                        sub_routes[k].append(self.route[j + 1])
                        pointer = self.route[j + 1]
                    else:
                        k += 1
                        sub_routes[k] = list([self.route[j + 1]])
                        pointer = self.route[j + 1]
                        break
            else:
                break

        for num in range(k + 1):
            sub_route = sub_routes[num][:]
            if len(sub_route) == 1:
                ref_lanelet = self.route_planner.lanelet_network.find_lanelet_by_id(sub_route[0])
                self.ref_path_dict[sub_route[0]] = self.route_planner.smooth_reference(ref_lanelet.center_vertices)
            else:
                sub_route_extended = sub_route[:]
                if num != k and self.successor[sub_route[-1]] != 0:
                    sub_route_extended.append(self.successor[sub_route[-1]])
                if num != 0 and self.predecessor[sub_route[0]] != 0:
                    sub_route_extended.insert(0, self.predecessor[sub_route[0]])
                ref_path = self.route_planner.get_ref_path_from_route(sub_route_extended)
                self.ref_path_dict[sub_route[0]] = self.route_planner.smooth_reference(ref_path)
                for i in range(1, len(sub_route)):
                    self.ref_path_dict[sub_route[i]] = list([sub_route[0]])
                    if num < k:
                        self.ref_path_dict[sub_route[i]].append(sub_routes[num + 1][0])
                    else:
                        self.ref_path_dict[sub_route[i]].append(0)

    def switch_ref_path(self, ref_lanelet_id: int, cl_state=None):
        """
        Switch reference path when no feasible sample can be found.
        """
        if cl_state is None:
            if type(self.ref_path_dict[ref_lanelet_id]) == list and self.ref_path_dict[ref_lanelet_id][1] != 0:
                self.ref_path = self.ref_path_dict[self.ref_path_dict[ref_lanelet_id][1]]
                self.ref_path_id = self.ref_path_dict[ref_lanelet_id][1]
            else:
                self.ref_path = self.ref_path_dict[self.route[self.route.index(ref_lanelet_id) + 1]]
                self.ref_path_id = self.route[self.route.index(ref_lanelet_id) + 1]

        else:
            if ref_lanelet_id != self.ref_path_id:
                if type(self.ref_path_dict[ref_lanelet_id]) == list:
                    self.ref_path = self.ref_path_dict[self.ref_path_dict[ref_lanelet_id][0]]
                    self.ref_path_id = ref_lanelet_id
                else:
                    self.ref_path = self.ref_path_dict[ref_lanelet_id]
                    self.ref_path_id = ref_lanelet_id
            else:
                current_ref_lanelet = self.route_planner.lanelet_network.find_lanelet_by_id(self.ref_path_id)
                new_ref_lanelet_id = self.ref_path_id
                if cl_state.position[1] >= 0 and current_ref_lanelet.adj_left is not None:
                    new_ref_lanelet_id = current_ref_lanelet.adj_left
                elif current_ref_lanelet.adj_right is not None:
                    new_ref_lanelet_id = current_ref_lanelet.adj_right
                new_ref_lanelet = self.route_planner.lanelet_network.find_lanelet_by_id(new_ref_lanelet_id)
                self.ref_path = self.route_planner.smooth_reference(new_ref_lanelet.center_vertices)
                self.ref_path_id = new_ref_lanelet_id

        return self.ref_path

