__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.1"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Alpha"


# python packages
from typing import List
import cProfile
import pycrcc
import time

import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.validity import *
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory, State

# commonroad imports
from commonroad_rp.parameter import DefFailSafeSampling, VehModelParameters
from commonroad_rp.parameter import parameter_velocity_reaching
from commonroad_rp.polynomial_trajectory import QuinticTrajectory, QuarticTrajectory
from commonroad_rp.trajectories import TrajectoryBundle, TrajectorySample, CartesianSample, CurviLinearSample
from commonroad_rp.cost_function import DefaultCostFunctionFailSafe
from commonroad_rp.utils import CoordinateSystem


class ReactivePlanner(object):
    def __init__(self, dt: float, t_h: float, N: int, v_desired=14, collision_check_in_cl: bool = False):

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
        fs_sampling = DefFailSafeSampling()
        self._sampling_d = fs_sampling.d_samples
        self._sampling_t = fs_sampling.t_samples
        self._sampling_v = fs_sampling.v_samples

        # sampling levels
        self._sampling_level = fs_sampling.max_iteration

        # coordinate system
        self._co = None

    def set_reference_path(self, reference_path: np.ndarray):
        self._co: CoordinateSystem = CoordinateSystem(reference_path)


    def coordinate_system(self):
        return self._co

    def no_of_samples(self, samp_level: int) -> int:
        """
        Returns the number of samples for a given sampling level
        :param samp_level: The sampling level
        :return: Number of trajectory samples for given sampling level
        """
        #return len(self._v_sets[samp_level]) * len(self._d_sets[samp_level]) * len(self._t_sets[samp_level])
        return len(self._sampling_v.to_range(samp_level)) * len(self._sampling_d.to_range(samp_level)) * len(self._sampling_t.to_range(samp_level))

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
        for t in self._sampling_t.to_range(samp_level):

            # Longitudinal sampling for all possible velocities
            for v in self._sampling_v.to_range(samp_level):
                trajectory_long = QuarticTrajectory(tau_0=0, delta_tau=t, x_0=np.array(x_0_lon), x_d=np.array([v, 0]))

                # Sample lateral end states (add x_0_lat to sampled states)
                if trajectory_long.coeffs is not None:
                    for d in self._sampling_d.to_range(samp_level).union({x_0_lat[0]}):
                        end_state_lat = np.array([d, 0.0, 0.0])
                        # SWITCHING TO POSITION DOMAIN FOR LATERAL TRAJECTORY PLANNING
                        s_lon_goal = trajectory_long.evaluate_state_at_tau(t)[0] - x_0_lon[0]
                        trajectory_lat = QuinticTrajectory(tau_0=0, delta_tau=s_lon_goal, x_0=np.array(x_0_lat),
                                                           x_d=end_state_lat)
                        if trajectory_lat.coeffs is not None:
                            trajectory_sample = TrajectorySample(self.horizon, self.dT, trajectory_long, trajectory_lat)
                            trajectories.append(trajectory_sample)

        # perform pre check and order trajectories according their cost
        trajectory_bundle = TrajectoryBundle(trajectories, params,
                                             cost_function=DefaultCostFunctionFailSafe())

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
        if s_d < 0:
            raise Exception(
                "Initial state or reference incorrect! Curvilinear velocity is negative which indicates that the ego vehicle is not driving in the same direction as specified by the reference")

        s_dd = x_0.acceleration
        s_dd -= (s_d ** 2 / np.cos(theta_cl)) * (
                (1 - kr * d) * np.tan(theta_cl) * (x_0.yaw_rate * ((1 - kr * d) / (np.cos(theta_cl)) - kr)) - (
                kr_d * d + kr * d_p))
        s_dd /= ((1 - kr * d) / (np.cos(theta_cl)))

        # d_d =(1-kr*d)*np.tan(theta_cl) #x_0.v * np.sin(theta_cl)

        x_0_lon = [s, s_d, s_dd]
        x_0_lat = [d, d_p, d_pp]

        print("<ReactivePlanner>: Starting planning with: \n#################")
        print('Initial state for planning is {}'.format(x_0))
        print('Initial x_0 lon = {}'.format(x_0_lon))
        print('Initial x_0 lat = {}'.format(x_0_lat))
        print("#################")

        return (x_0_lon, x_0_lat)

    def plan(self, x_0: State, cc: object, cl_states=None) -> tuple:
        """
        Plans an optimal trajectory
        :param x_0: Initial state as CR state
        :param cc:  CollisionChecker object
        :param cl_states: Curvilinear state if replanning is used
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
            # profiler = cProfile.Profile()
            # profiler.enable()
            optimal_trajectory = self._get_optimal_trajectory(bundle, cc)
            print('<ReactivePlanner>: Checked trajectories in {} seconds'.format(time.time() - t0))
            # profiler.disable()
            # profiler.print_stats("time")
            # self.draw_trajectory_set(bundle.trajectories)
            # plt.pause(50)
            # print statistics
            print('<ReactivePlanner>: Rejected {} infeasible trajectories due to kinematics'.format(
                self.no_of_infeasible_trajectories_kinematics()))
            print('<ReactivePlanner>: Rejected {} infeasible trajectories due to collisions'.format(
                self.no_of_infeasible_trajectories_collision()))

            i = i + 1
        if optimal_trajectory is None and x_0.velocity <= 0.1:
            optimal_trajectory = self._compute_standstill_trajectory(x_0, x_0_lon, x_0_lat)

        # check if feasible trajectory exists -> emergency mode
        if optimal_trajectory is None:
            print('<ReactivePlanner>: Could not find any trajectory out of {} trajectories'.format(
                sum([self.no_of_samples(i) for i in range(self._sampling_level)])))
            print('<ReactivePlanner>: Cannot find trajectory with default sampling parameters. '
                  'Switching to emergency mode!')
        else:
            print('<ReactivePlanner>: Found optimal trajectory with costs = {}, which corresponds to {} percent of seen costs'.format(
                optimal_trajectory.cost,
                ((optimal_trajectory.cost - bundle.min_costs().cost) / (
                            bundle.max_costs().cost - bundle.min_costs().cost))))

        return self._compute_trajectory_pair(optimal_trajectory) if not bundle.empty() else None

    def _compute_standstill_trajectory(self, x_0, x_0_lon, x_0_lat) -> TrajectorySample:
        # create artifical standstill trajectory
        print('Adding standstill trajectory')
        traj_lon = QuarticTrajectory(tau_0=0, delta_tau=self.horizon, x_0=x_0_lon,
                                     x_d=np.array([self._desired_speed, 0]))
        traj_lat = QuinticTrajectory(tau_0=0, delta_tau=self.horizon, x_0=x_0_lat, x_d=np.array([x_0_lat[0], 0, 0]))
        p = TrajectorySample(self.horizon, self.dT, traj_lon, traj_lat)
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

            lon_list.append(
                [trajectory.curvilinear.s[i], trajectory.curvilinear.s_dot[i], trajectory.curvilinear.s_ddot[i]])
            lat_list.append(
                [trajectory.curvilinear.d[i], trajectory.curvilinear.d_dot[i], trajectory.curvilinear.d_ddot[i]])

        cartTraj = Trajectory(0, cart_list)
        freTraj = Trajectory(0, cl_list)

        return (cartTraj, freTraj, lon_list, lat_list)

    def check_kinematics(self, trajectory_bundle: TrajectoryBundle) -> List[TrajectorySample]:
        """
        Checks the kinematics of given trajectory and computes the cartesian trajectory information
        :param trajectory: The trajectory to check
        :return: True if the trajectory is feasible and false otherwise
        """

        # constants
        _LOW_VEL_MODE = True

        feasible_trajectories = list()
        for trajectory in trajectory_bundle.trajectories:
            # create time array and precompute time interval information
            t = np.arange(0, np.round(trajectory.trajectory_long.delta_tau + self.dT,5), self.dT)
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
                try:
                    pos = self._co.convert_to_cartesian_coords(s[i], d[i])
                except ValueError:
                    # outside of projection domain
                    return False
                x[i] = pos[0]
                y[i] = pos[1]

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
                s_lambda = (self._co.ref_pos()[s_idx] - s[i]) / (self._co.ref_pos()[s_idx + 1] - self._co.ref_pos()[s_idx])

                # add cl and gl orientation
                if s_velocity[i] > 0.005:
                    if _LOW_VEL_MODE:
                        theta_cl[i] = np.arctan2(dp, 1.0)
                    else:
                        theta_cl[i] = np.arctan2(d_velocity[i], s_velocity[i])
                    theta_gl[i] = theta_cl[i] + (self._co.ref_theta()[s_idx + 1] - self._co.ref_theta()[s_idx]) * s_lambda + \
                                  self._co.ref_theta()[s_idx]

                else:
                    # theta_cl.append(np.interp(s[i], self._co.ref_pos(), self._co.ref_theta()))
                    theta_cl[i] = (self._co.ref_theta()[s_idx + 1] - self._co.ref_theta()[s_idx]) * s_lambda + \
                                  self._co.ref_theta()[s_idx]
                    theta_gl[i] = theta_cl[i]

                # Compute curvature of reference at current position
                k_r = (self._co.ref_curv()[s_idx + 1] - self._co.ref_curv()[s_idx]) * s_lambda + self._co.ref_curv()[s_idx]
                k_r_d = (self._co.ref_curv_d()[s_idx + 1] - self._co.ref_curv_d()[s_idx]) * s_lambda + \
                        self._co.ref_curv_d()[s_idx]
                # compute global curvature based on appendix A of Moritz Werling's PhD thesis
                oneKrD = (1 - k_r * d[i])
                cosTheta = np.cos(theta_cl[i])
                tanTheta = np.tan(theta_cl[i])
                kappa_gl[i] = (dpp + k_r * dp * tanTheta) * cosTheta * (cosTheta / oneKrD) ** 2 + (cosTheta / oneKrD) * k_r
                kappa_cl[i] = kappa_gl[i] - k_r

                # velocity
                v[i] = s_velocity[i] * (oneKrD / (np.cos(theta_cl[i])))

                # compute acceleration
                a[i] = s_acceleration[i] * oneKrD / cosTheta + ((s_velocity[i] ** 2) / cosTheta) * (
                        oneKrD * tanTheta * (kappa_gl[i] * oneKrD / cosTheta - k_r) - (
                        k_r_d * d[i] + k_r * d_velocity[i]))

                # check kinematics to already discard infeasible trajectories
                if abs(kappa_gl[i] > self.constraints.kappa_max):
                    #print("Kappa")
                    feasible = False
                    break
                if abs((kappa_gl[i] - kappa_gl[i - 1]) / self.dT if i > 0 else 0.) > self.constraints.kappa_dot_max:
                    #print("KappaDOT")
                    feasible = False
                    break
                if abs(a[i]) > self.constraints.a_max:
                    #print("Acceleration")
                    feasible = False
                    break
                if abs(v[i]) < -0.1:
                    #print("Velocity")
                    feasible = False
                    break
                if abs((theta_gl[i-1]-theta_gl[i])/self.dT if i > 0 else 0.) > 0.4:
                    #print("Theta")
                    feasible = False
                    break

            if feasible:
                # store Cartesian trajectory
                trajectory.cartesian = CartesianSample(x, y, theta_gl, v, a, kappa_gl, np.append([0], np.diff(kappa_gl)))

                # store Curvilinear trajectory
                trajectory.curvilinear = CurviLinearSample(s, d, theta_cl, ss=s_velocity, sss=s_acceleration, dd=d_velocity,
                                                           ddd=d_acceleration)

                # check if trajectories planning horizon is shorter than expected and extend if necessary
                #if self.horizon > trajectory.trajectory_long.delta_tau:
                if self.N+1 > len(trajectory.cartesian.x):
                    trajectory.enlarge(self.N+1-len(trajectory.cartesian.x), self.dT)

                assert self.N+1 == len(trajectory.cartesian.x) == len(trajectory.cartesian.y) == len(
                    trajectory.cartesian.theta), '<ReactivePlanner/kinematics>: Trajectory computation is incorrect! Lenghts of state variables is not equal.'
                feasible_trajectories.append(trajectory)
        return feasible_trajectories



    def _get_optimal_trajectory(self, trajectory_bundle: TrajectoryBundle, cc: object) -> TrajectorySample:

        # reset statistics
        self._infeasible_count_collision = 0
        self._infeasible_count_kinematics = 0

        # check kinematics of each trajectory
        profiler = cProfile.Profile()
        profiler.enable()
        feasible_trajectories = self.check_kinematics(trajectory_bundle)
        self._infeasible_count_kinematics = len(trajectory_bundle.trajectories) - len(feasible_trajectories)

        # set feasible trajectories in bundle
        trajectory_bundle.trajectories = feasible_trajectories
        # sort trajectories according to their costs
        trajectory_bundle.sort()

        #self.draw_trajectory_set(feasible_trajectories)
        #plt.show(block=True)

        # go through sorted list of trajectories and check for collisions
        for trajectory in trajectory_bundle.get_sorted_list():

            pos1 = trajectory.curvilinear.s if self._collision_check_in_cl else trajectory.cartesian.x
            pos2 = trajectory.curvilinear.d if self._collision_check_in_cl else trajectory.cartesian.y
            theta = trajectory.curvilinear.theta if self._collision_check_in_cl else trajectory.cartesian.theta

            # check each pose for collisions
            collide = False
            for i in range(len(pos1)):
                ego = pycrcc.TimeVariantCollisionObject(self.x_0.time_step+i)
                ego.append_obstacle(pycrcc.RectOBB(0.5 * self._length, 0.5 * self._width, theta[i], pos1[i], pos2[i]))
                if cc.collide(ego):
                    self._infeasible_count_collision += 1
                    collide = True
                    break
            if not collide:
                profiler.disable()
                profiler.print_stats("time")
                return trajectory

        profiler.disable()
        profiler.print_stats("time")
        return None


    def convert_cr_trajectory_to_object(self, trajectory: Trajectory):
        """
        Converts a CR trajectory to a CR dynamic obstacle with given dimensions
        :param trajectory: The trajectory of the vehicle
        :return:
        """

        # get shape of vehicle
        shape = Rectangle(self._length, self._width)

        # get trajectory prediction
        prediction = TrajectoryPrediction(trajectory, shape)

        return DynamicObstacle(42, ObstacleType.CAR, shape, trajectory.state_list[0], prediction)
