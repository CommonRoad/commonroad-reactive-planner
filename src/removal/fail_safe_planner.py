import numpy as np
from typing import Union
import time
import timeit
import os
import multiprocessing
import signal
from removal.my_process import MyProcess

from removal.combined_trajectory import CombinedPolynomialTrajectory
from removal.create_trajectory_bundle import position_reaching_bundle
from removal.sampling_area import SamplingArea
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker, create_collision_object
from commonroad.prediction.spot_prediction import SpotPrediction
from commonroad.geometry.shape import Rectangle
from removal.utils import create_coordinate_systems_for_lanelet_id, binary_search_indices
from parameter import VehicleParameter


class FailSafePlanner:
    def __init__(self, scenario, desired_trajectory):
        self.start_time = timeit.default_timer()
        self.scenario = scenario
        self.desired_trajectory = desired_trajectory
        self.__set_obstacle_occupancy_prediction(desired_trajectory.initial_time_idx, len(self.desired_trajectory.
                                                                                          state_list))
        self.__update_collision_checker()
        self.finished_slaves_counter = 0

    def __set_obstacle_occupancy_prediction(self, t0: float, steps: int):
        """
        creates occupancy prediction for all dynamic obstacles in scenario
        :param t0: initial time
        :param steps: number of steps
        :return:
        """
        self.spot_prediction = SpotPrediction(t0, steps, self.scenario.dt)
        self.spot_prediction.createRoadNetwork(self.scenario.lanelet_network.lanelets)
        self.spot_prediction.setObstacles(self.scenario)
        self.spot_prediction.setComputeOccDynamicBasedOfPedestrians(True)
        self.spot_prediction.setComputeOccRuleBasedOfPedestrians(True)
        self.spot_prediction.setAmaxOfPedestrians(0.6)
        self.spot_prediction.setAstopOfPedestrians(0.6)
        self.spot_prediction.setVmaxOfPedestrians(2)
        self.spot_prediction.setAccelerationOfPedestrians(1.0)
        self.spot_prediction.setVelocityOfPedestrians([1.5])
        self.spot_prediction.setNumLanesOfPedestrians(5)
        self.spot_prediction.setLaneWidthOfPedestrians(3)
        self.spot_prediction.setbCrossOfPedestrians(True)
        self.spot_prediction.setbStopOfPedestrians(False)
        num_threads = 1
        if self.spot_prediction.calcOccupancies(num_threads, 1) != 0:
            print('No dynamic obstacles included! Fail safe plan is not necessary')

    def __update_collision_checker(self):
        """
        Update the collision checker for the current scenario
        :return:
        """
        self.scenario_cc = create_collision_checker(self.scenario)

    def check_collision_time_with_spot(self, trajectory: Trajectory):
        """
        Checks if there is a collision with other obstacles for the given trajectory of the ego vehicle and returns the
        time step when the collision happened.
        :param trajectory: The trajectory of the ego vehicle to be checked for collision
        :return: The time step when the collision happened or -1 for no collision
        """
        ego_trajectory_prediction = TrajectoryPrediction(trajectory.initial_time_idx,
                                                         trajectory, Rectangle(VehicleParameter.length,
                                                                               VehicleParameter.width))
        ego_collision_object = create_collision_object(ego_trajectory_prediction)
        collision_time_idx = self.scenario_cc.collision_time(ego_collision_object)
        # be careful that you check for the correct time interval
        return collision_time_idx

    def check_collision_with_spot(self, trajectory: Trajectory):
        """
        Checks if there is a collision with other obstacles for the given trajectory of the ego vehicle and returns a
        True or False.
        :param trajectory:
        :return: True if there is a collision. Otherwise returns False.
        """
        ego_trajectory_prediction = TrajectoryPrediction(trajectory.initial_time_idx,
                                                         trajectory,
                                                         Rectangle(VehicleParameter.length, VehicleParameter.width))
        ego_collision_object = create_collision_object(ego_trajectory_prediction)
        collision = self.scenario_cc.collide(ego_collision_object)
        # be careful that you check for the correct time interval
        return collision

    def find_last_safe_time_idx(self) -> int:
        """
        Checks for a collision and returns the last safe time without collision
        :return: Returns the time index before the first collision
        """
        collision_time_idx = self.check_collision_time_with_spot(self.desired_trajectory)
        if collision_time_idx < 0:
            print('Desired trajectory does not collide with any occupancies. No fail-safe trajectory needed.')
        return collision_time_idx - 1  # return the time idx before first collision

    def fail_safe_trajectory_multi_threaded(self) -> list:
        """
        Plans a fail-safe trajectory from a state of the main trajectory until a precalculated safe region. Uses binary
        search in the space of time to find the last time at which a fail-safe trajectory can be found.
        :return: A fail-safe trajectory or None if there isn't one.
        """
        last_time_idx = self.find_last_safe_time_idx()
        if last_time_idx == -2:
            return self.desired_trajectory
        print("last:", last_time_idx)
        num_processes = os.cpu_count()
        processes = []

        # create job list
        st = timeit.default_timer()
        middle = (self.desired_trajectory.initial_time_idx + last_time_idx) // 2
        upper_range = range(middle, last_time_idx + 1)
        lower_range = range(self.desired_trajectory.initial_time_idx, middle)
        full_range_list = [middle, last_time_idx] + binary_search_indices(upper_range)
        full_range_list = full_range_list + [middle - 1, self.desired_trajectory.initial_time_idx] + \
                          binary_search_indices(lower_range)
        print("list created after: %f" % (timeit.default_timer() - st))

        # create queues and lock for processes
        queue_lock = multiprocessing.Lock()
        work_queue = multiprocessing.Queue(len(full_range_list))
        return_queue = multiprocessing.Queue()

        # fill job queue
        queue_lock.acquire()
        for time_idx in full_range_list:
            work_queue.put(time_idx)
        queue_lock.release()

        # create processes
        p_name_base = "Process_"
        time.sleep(1 / 1000000.0)
        start = timeit.default_timer()
        for i in range(num_processes - 2):
            process = MyProcess(i, p_name_base + str(i), work_queue, return_queue, queue_lock, num_processes,
                                os.getpid(), self)
            process.start()
            processes.append(process)

        # wait for processes to finish
        trajectories = []
        finished_slaves = []
        while self.finished_slaves_counter != 2:
            signal_set = {signal.SIGUSR1, 31}
            siginfo = signal.sigwaitinfo(signal_set)
            # process found a trajectory
            if siginfo.si_signo == signal.SIGUSR1:
                finished_slaves.append(siginfo.si_pid)
                break
            # process finished all its jobs
            elif siginfo.si_signo == 31:
                self.finished_slaves_counter += 1
                finished_slaves.append(siginfo.si_pid)

        # if process found a trajectory signal other processes to stop
        for p in processes:
            if p.pid not in finished_slaves:
                os.kill(p.pid, signal.SIGUSR1)

        # join all processes
        for p in processes:
            p.join()
        print("Time for all samples: %f" % (timeit.default_timer() - start))

        # get results from all processes
        while not return_queue.empty():
            trajectories.append(return_queue.get())
        return trajectories

    def fail_safe_trajectory(self) -> Union[None, Trajectory]:
        """
        Plans a fail-safe trajectory from a state of the main trajectory until a precalculated safe region. Uses binary
        search in the space of time to find the last time at which a fail-safe trajectory can be found.
        :return: A fail-safe trajectory or None if there isn't one.
        """
        last_time_idx = self.find_last_safe_time_idx()
        if last_time_idx == -1:
            return self.desired_trajectory

        # perform binary search to find last point at which a fail safe trajectory can be found
        first = 0
        last = last_time_idx
        trajectory_found = None
        start_time = timeit.default_timer()
        while first <= last:
            self.start_time = timeit.default_timer()
            time_idx = (first + last) // 2
            init_x = self.desired_trajectory.state_list[time_idx].position[0]
            init_y = self.desired_trajectory.state_list[time_idx].position[1]
            init_velocity = self.desired_trajectory.state_list[time_idx].velocity
            init_acceleration = self.desired_trajectory.state_list[time_idx].acceleration
            start = timeit.default_timer()
            trajectory = self.fail_safe_trajectory_from_state(init_x, init_y, init_velocity, init_acceleration,
                                                              time_idx)
            print('Fail-safe trajectory found at time_idx %d: %f s' %
              (time_idx, timeit.default_timer() - start))
            if trajectory:
                first = time_idx + 1
                trajectory_found = trajectory
                print('Fail-safe trajectory found at time_idx %d: %f s' %
                      (time_idx, timeit.default_timer() - self.start_time))
                break
            else:
                last = time_idx - 1
                print('No trajectory found at time_idx %d: %f s' % (time_idx, timeit.default_timer() - self.start_time))
        print("Time for all samples: %f" % (timeit.default_timer() - start_time))
        return trajectory_found

    def fail_safe_trajectory_from_state(self, init_x, init_y, init_velocity, init_acceleration,
                                        init_time_idx) -> Union[None, Trajectory]:
        """
        Checks if all maneuver types are possible in the context of the emergency trajectory from current position.
        Transforms position into curvilinear coordinates. Finds optimal trajectory for this maneuver according to
        Werling M et al. "Optimal trajectory generation for dynamic street scenarios in a frenet frame". In: IEEE
        International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.
        :param init_x: current x position in cartesian coordinates
        :param init_y: current y position in cartesian coordinates
        :param init_velocity: current velocity in m/s
        :param init_acceleration: current acceleration in m/s²
        :param init_time_idx: time index at which fail safe trajectory starts
        :return: optimal trajectories and lane for given maneuver type
        """
        position = np.array([init_x, init_y])
        initial_lane_id = self.scenario.lanelet_network.find_lanelet_by_position(np.array([position]))[0][0]
        initial_lane = self.scenario.lanelet_network.find_lanelet_by_id(initial_lane_id)
        adjacent_right = initial_lane.adj_right
        adjacent_left = initial_lane.adj_left

        adjacent_right_coordinate_systems = []
        adjacent_left_coordinate_systems = []
        if adjacent_right:
            adjacent_right_coordinate_systems = create_coordinate_systems_for_lanelet_id(self.scenario.lanelet_network,
                                                                                         adjacent_right, max_iter=3)
        if adjacent_left:
            adjacent_left_coordinate_systems = create_coordinate_systems_for_lanelet_id(self.scenario.lanelet_network,
                                                                                        adjacent_left, max_iter=3)
        init_coordinate_systems = create_coordinate_systems_for_lanelet_id(self.scenario.lanelet_network,
                                                                           initial_lane_id, max_iter=3)

        coordinate_systems = (init_coordinate_systems + adjacent_left_coordinate_systems +
                              adjacent_right_coordinate_systems)

        optimal_trajectory = None
        for lane_coordinate_system in coordinate_systems:
            safe_region = SamplingArea.safe_region_for_coordinate_system(lane_coordinate_system.lanelet_ids)

            if not safe_region:
                continue

            # TODO currently the desired velocity is the init velocity. This has to be adapted according to safe regions
            s, d = lane_coordinate_system.coordinate_system.convert_to_curvilinear_coords(init_x, init_y)
            current_state_long = np.array([s, init_velocity, init_acceleration])
            current_state_lat = np.array([d, 0.0, 0.0])
            tb = position_reaching_bundle(self.scenario.dt, safe_region.s_min, safe_region.s_max, 0, 0,
                                          init_velocity, current_state_long, current_state_lat, init_time_idx)
            optimal_trajectory = None
            while True:
                optimal_trajectory_sample = tb.optimal_trajectory()

                if optimal_trajectory_sample is None:
                    break

                duration = int(optimal_trajectory_sample.trajectory_long.duration_s / self.scenario.dt)
                x, y, v, a, theta, steering_angle, partial_trajectory = CombinedPolynomialTrajectory.\
                    calc_cartesian_state(self.scenario.dt, duration, optimal_trajectory_sample.trajectory_long,
                                         optimal_trajectory_sample.trajectory_lat, lane_coordinate_system)
                # for t in range(self.desired_trajectory.initial_time_idx, init_time_idx):
                #     x.insert(t, self.desired_trajectory.state_list[t].position[0])
                #     y.insert(t, self.desired_trajectory.state_list[t].position[1])
                #      v = np.insert(v, t, self.desired_trajectory.state_list[t].velocity)
                #      a = np.insert(a, t, self.desired_trajectory.state_list[t].acceleration)
                #     theta = np.insert(theta, t, self.desired_trajectory.state_list[t].orientation)
                t_list = np.arange(init_time_idx,
                                   init_time_idx + duration + 1)

                cartesian_state_list = CombinedPolynomialTrajectory.create_ks_state_list(x, y, v, a, theta,
                                                                                         steering_angle,
                                                                                         t_list)

                optimal_trajectory = CombinedPolynomialTrajectory(init_time_idx,
                                                                  cartesian_state_list,
                                                                  optimal_trajectory_sample.total_cost,
                                                                  partial_trajectory)

                if not optimal_trajectory.check_feasibility():
                    tb.remove_trajectory(tb.optimal_trajectory_idx())
                    continue

                if self.check_collision_with_spot(optimal_trajectory):
                    tb.remove_trajectory(tb.optimal_trajectory_idx())
                    continue
                break
                # TODO: check if safe region cannot be reached anymore -> break

            if not optimal_trajectory_sample:
                optimal_trajectory = None
                continue

            if optimal_trajectory:
                break
        return optimal_trajectory
