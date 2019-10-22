import numpy as np
from heapq import heappush, heappop
from enum import Enum, unique
from typing import List, Union
from functools import total_ordering

from removal.create_trajectory_bundle import velocity_reaching_bundle
from removal.combined_trajectory import CombinedPolynomialTrajectory
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker
from commonroad.common.util import Interval
from removal.utils import create_coordinate_systems_for_lanelet_id, end_lane_id


@unique
class ManeuverType(Enum):
    GO_STRAIGHT = 0
    LANE_CHANGE_RIGHT = 1
    LANE_CHANGE_LEFT = 2


@total_ordering
class Node:
    def __init__(self, trajectory: CombinedPolynomialTrajectory, end_lane_id_, lane, cost_from_root=0, parent=None):
        self.trajectory = trajectory
        self.parent = parent
        self.end_lane_id = end_lane_id_
        self.cost_from_root = cost_from_root + trajectory.cost
        self.lane = lane

    # define comparison function for priority queue
    def __eq__(self, other_node):
        if isinstance(other_node, Node):
            return self.cost_from_root == other_node.cost_from_root
        return NotImplemented

    def __lt__(self, other_node):
        """
        we prefer the node with the fewer total cost from the root node.
        :param other_node:
        :return:
        """
        if isinstance(other_node, Node):
            return self.cost_from_root < other_node.cost_from_root
        return NotImplemented


class TrajectoryTree:

    def __init__(self, planning_problem, scenario):
        self.initial_state = planning_problem.initial_state
        self.goal_region = planning_problem.goal
        self.scenario = scenario
        self.scenario_cc = create_collision_checker(scenario)
        self.desired_velocity = self.__desired_velocity()

        initial_lane_id = scenario.lanelet_network.find_lanelet_by_position(np.array(
            [self.initial_state.position]))[0][0]
        self.initial_lane = scenario.lanelet_network.find_lanelet_by_id(initial_lane_id)

        # self.explored = [] not needed, since no two trajectories are the same
        self.frontier = []  # heapq
        self.__initialize_frontier()

    def __initialize_frontier(self):
        """
        from initial position add one node for every maneuver possibility (see ManeuverType) to the frontier
        :return:
        """
        for maneuver in ManeuverType:
            init_acceleration = 0.0
            if hasattr(self.initial_state, 'acceleration'):
                init_acceleration = self.initial_state.acceleration
            optimal_trajectories_for_lanes = self.plan_optimal_trajectory(maneuver, self.initial_state.position[0],
                                                                          self.initial_state.position[1],
                                                                          self.initial_state.velocity,
                                                                          init_acceleration,
                                                                          self.initial_lane.lanelet_id, 0)

            if optimal_trajectories_for_lanes is not None:
                for optimal_trajectory_for_lane in optimal_trajectories_for_lanes:
                    end_lane_id_ = end_lane_id(self.scenario.lanelet_network, optimal_trajectory_for_lane)
                    if end_lane_id_ > -1 and not optimal_trajectory_for_lane[0].partial_trajectory:
                        # inside of lane network
                        child = Node(optimal_trajectory_for_lane[0], end_lane_id_, optimal_trajectory_for_lane[1])
                        heappush(self.frontier, child)

    def uniform_cost_search(self) -> Union[List[CombinedPolynomialTrajectory], None]:
        """
        perform uniform cost search from self.initial_state to self.goal_region. From each node, the maneuvers from enum
        ManeuverType are planned. The node with the lowest cost from initial state to current node is extended first.

        :return: List of successive trajectories or None, if no feasible trajectory from initial state to goal region
            can be found
        """
        while self.frontier:
            node = heappop(self.frontier)

            # update the objects' positions to the end time of the trajectory
            t0 = node.trajectory.duration

            # Check if goal region is reached
            for state in node.trajectory.state_list:
                if node.end_lane_id < 0:    # if position is not on lanelet network, do not check
                    if self.scenario.lanelet_network.find_lanelet_by_position(np.array([state.position]))[0][0] < 0:
                        continue
                if self.goal_region.is_reached(state):
                    return self.return_solution(node)

            if node.trajectory.partial_trajectory:  # trajectory ends outside of lane network
                continue

            # TODO: calculate whe the goal region is passed
            # if we passed the goal region already, throw node away.
            # if isinstance(self.goal_region.state_list[0].position[0], Lanelet):
            #     if type(self.goal_region.state_list[0].position) == list:       # Lanelet
            #         if lanelet_passed(node.end_lane_id, self.goal_region.state_list[0].position[0].lanelet_id):
            #             continue
            if node.end_lane_id < 0:
                # TODO: calculate more precisely
                continue

            # plan from end state of parent trajectory
            node_end_state = node.trajectory.final_state

            for maneuver in ManeuverType:
                optimal_trajectories_for_lanes = self.plan_optimal_trajectory(maneuver, node_end_state.position[0],
                                                                              node_end_state.position[1],
                                                                              node_end_state.velocity,
                                                                              node_end_state.acceleration,
                                                                              node.end_lane_id, t0)

                if optimal_trajectories_for_lanes is not None:
                    for optimal_trajectory_for_lane in optimal_trajectories_for_lanes:
                        child = Node(optimal_trajectory_for_lane[0], end_lane_id(self.scenario.lanelet_network,
                                                                                 optimal_trajectory_for_lane),
                                     optimal_trajectory_for_lane[1], node.cost_from_root, node)
                        heappush(self.frontier, child)
        return None     # no trajectory found

    def return_solution(self, node: Node, output_trajectory=None) -> List[CombinedPolynomialTrajectory]:
        """
        returns recursively the solution from the trajectory that reaches the goal region to the initial trajectory
        :param node: Node that reaches the goal region
        :param output_trajectory: can be omitted. Necessary for recursion
        :return: list of successive trajectories
        """
        if output_trajectory is None:
            output_trajectory = []

        output_trajectory.insert(0, node.trajectory)

        if node.parent:
            self.return_solution(node.parent, output_trajectory)
        return output_trajectory

    def __desired_velocity(self) -> float:
        """
        determine desired velocity in current planning problem. If goal region has a velocity interval, the desired
        velocity is set to the mean value. Else the ego vehicle tries to keep the current velocity
        :return: desired velocity
        """
        desired_velocity_list = []
        for state in self.goal_region.state_list:
            if hasattr(state, 'velocity'):
                if isinstance(state.velocity, Interval):
                    goal_velocity = (state.velocity.start + state.velocity.end)/2
                else:
                    goal_velocity = state.velocity

                desired_velocity_list.append(goal_velocity)
        if len(desired_velocity_list) == 0:
            return self.initial_state.velocity
        else:
            return desired_velocity_list[0]

    def plan_optimal_trajectory(self, maneuver_type: ManeuverType, current_x: float, current_y: float,
                                current_velocity: float, current_acceleration: float, initial_lane_id: int,
                                t0: int) -> Union[list, None]:
        """
        Checks if maneuver type is possible from current position. Transforms position into curvilinear coordinates. The
        reference curve is always the center of the goal lane. Finds optimal trajectory for this maneuver according to
        Werling M et al. "Optimal trajectory generation for dynamic street scenarios in a frenet frame". In: IEEE
        International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.
        :param maneuver_type: desired maneuver type -> determines reference curve used for coordinate transformation
        :param current_x: current x position in cartesian coordinates
        :param current_y: current y position in cartesian coordinates
        :param current_velocity: current velocity in m/s
        :param current_acceleration: current acceleration in m/s²
        :param initial_lane_id: lane id of ego vehicle
        :param t0: discrete time step at the beginning
        :return: optimal trajectory and lane for given maneuver type
        """
        initial_lane = self.scenario.lanelet_network.find_lanelet_by_id(initial_lane_id)

        # lane change is only allowed on lanes in same direction
        if (maneuver_type == ManeuverType.LANE_CHANGE_RIGHT and
                (initial_lane.adj_right_same_direction is None or initial_lane.adj_right_same_direction is False)):
            return None
        if (maneuver_type == ManeuverType.LANE_CHANGE_LEFT and
                (initial_lane.adj_left_same_direction is None or initial_lane.adj_left_same_direction is False)):
            return None

        # determine curvilinear coordinate systems for goal lane
        if maneuver_type == ManeuverType.LANE_CHANGE_RIGHT:
            lane_coordinate_systems = create_coordinate_systems_for_lanelet_id(self.scenario.lanelet_network,
                                                                                    initial_lane.adj_right)
        elif maneuver_type == ManeuverType.LANE_CHANGE_LEFT:
            lane_coordinate_systems = create_coordinate_systems_for_lanelet_id(self.scenario.lanelet_network,
                                                                                    initial_lane.adj_left)
        else:   # maneuver_type == ManeuverType.go_straight:
            lane_coordinate_systems = create_coordinate_systems_for_lanelet_id(self.scenario.lanelet_network,
                                                                                    initial_lane_id)

        optimal_trajectories_for_lanes = []
        # determine position in curvilinear coordinates
        for lane_coordinate_system in lane_coordinate_systems:
            s, d = lane_coordinate_system.coordinate_system.convert_to_curvilinear_coords(current_x, current_y)
            current_s = np.array([s, current_velocity, current_acceleration])
            current_d = np.array([d, 0.0, 0.0])

            # calculate best trajectory for given maneuver
            tb = velocity_reaching_bundle(self.scenario.dt, self.desired_velocity, current_s, current_d, t0)
            optimal_trajectory = None
            while True:
                optimal_trajectory_sample = tb.optimal_trajectory()

                if optimal_trajectory_sample is None:
                    break

                # re-transform to Cartesian coordinates
                duration = int(optimal_trajectory_sample.trajectory_long.duration_s / self.scenario.dt)
                x, y, v, a, theta, steering_angle, partial_trajectory = CombinedPolynomialTrajectory.\
                    calc_cartesian_state(self.scenario.dt, duration, optimal_trajectory_sample.trajectory_long,
                                         optimal_trajectory_sample.trajectory_lat, lane_coordinate_system)
                t_list = np.arange(t0, t0 + duration + 1)

                # TODO calculate correct steering_angle
                cartesian_state_list = CombinedPolynomialTrajectory.create_ks_state_list(x, y, v, a, theta,
                                                                                         steering_angle, t_list)
                optimal_trajectory = CombinedPolynomialTrajectory(t0, cartesian_state_list,
                                                                  optimal_trajectory_sample.total_cost,
                                                                  partial_trajectory)

                if not optimal_trajectory.check_feasibility():
                    tb.remove_trajectory(tb.optimal_trajectory_idx())
                    continue

                if optimal_trajectory.check_collision(self.scenario_cc):
                    tb.remove_trajectory(tb.optimal_trajectory_idx())
                    continue
                break

            if optimal_trajectory_sample is not None:
                optimal_trajectory_for_lane = [optimal_trajectory, lane_coordinate_system.lane]
                optimal_trajectories_for_lanes.append(optimal_trajectory_for_lane)

        return optimal_trajectories_for_lanes
