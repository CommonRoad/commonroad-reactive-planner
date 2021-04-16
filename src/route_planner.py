import itertools
from typing import List, Tuple

import networkx as nx
import numpy as np
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad_ccosy.geometry.util import resample_polyline
from networkx import NetworkXNoPath
from scenario_helpers import smooth_reference


class RoutePlanner:
    def __init__(self, benchmark_id: int, lanelet_network: LaneletNetwork, planning_problem: PlanningProblem):
        self.benchmark_id = benchmark_id
        self.planning_problem = planning_problem
        self.lanelet_network = lanelet_network
        self.create_graph_from_lanelet_network()
        self.goal_lanelet_ids = self.get_goal_lanelet()

    def create_graph_from_lanelet_network(self):
        """
        Creates a DiGraph for current lanelet network, only adjacency with same directions is considered.
        """
        self.graph = nx.DiGraph()
        nodes = list()
        edges = list()

        for lanelet in self.lanelet_network.lanelets:
            nodes.append(lanelet.lanelet_id)
            if lanelet.successor:
                for successor in lanelet.successor:
                    l = self.lanelet_network.find_lanelet_by_id(successor)
                    edges.append((lanelet.lanelet_id, l.lanelet_id, {'weight': lanelet.distance[-1]}))
            if lanelet.adj_left:
                l = self.lanelet_network.find_lanelet_by_id(lanelet.adj_left)
                edges.append((lanelet.lanelet_id, l.lanelet_id,
                              {'weight': 0, 'same_dir': lanelet.adj_left_same_direction}))
            if lanelet.adj_right:
                l = self.lanelet_network.find_lanelet_by_id(lanelet.adj_right)
                edges.append((lanelet.lanelet_id, l.lanelet_id,
                              {'weight': 0, 'same_dir': lanelet.adj_right_same_direction}))

        self.graph.add_nodes_from(nodes)
        self.graph.add_edges_from(edges)

    def get_goal_lanelet(self) -> List[int]:
        """
        Gets a list of lanelet ids that contains the goal position;
        if no position is specified, goal lanelet is created by naively concatenating successors of initial lanelet
        :return:
        """
        if self.planning_problem.goal.lanelets_of_goal_position is None:
            if hasattr(self.planning_problem.goal.state_list[0], 'position'):
                goal_lanelet_id_list = self.lanelet_network.find_lanelet_by_position(
                    [self.planning_problem.goal.state_list[0].position.center])[0]
            else:
                print("<RoutePlanner: get_goal_lanelet> No position provided for goal state, "
                      "using successors of initial position as goal lanelet.")

                # naively concatenate successors, TODO: use some heuristic?
                l_id = self.lanelet_network.find_lanelet_by_position(
                    [self.planning_problem.initial_state.position])[0][0]
                current_lanelet = self.lanelet_network.find_lanelet_by_id(l_id)
                min_dist = self.planning_problem.initial_state.velocity * \
                           self.planning_problem.goal.state_list[0].time_step.end + 200
                dist = current_lanelet.distance[-1]
                while dist < min_dist and current_lanelet is not None:
                    l_id = current_lanelet.successor[0] if len(current_lanelet.successor) > 0 else None
                    current_lanelet = self.lanelet_network.find_lanelet_by_id(l_id) if l_id is not None else None
                    if current_lanelet is not None:
                        dist += current_lanelet.distance[-1]

                goal_lanelet_id_list = [current_lanelet.lanelet_id]
        else:
            goal_lanelet_id_list = list(self.planning_problem.goal.lanelets_of_goal_position.values())
            assert isinstance(goal_lanelet_id_list, list), "goal_lanelet_id_list should be a list of ids!"

        return goal_lanelet_id_list

    def find_all_shortest_paths(self, source_lanelet_id: int, target_lanelet_id: int) -> List[List[int]]:
        """
        Finds shortest paths from source lanelet to target lanelet
        :param source_lanelet_id: source lanelet id
        :param target_lanelet_id: target lanelet id
        :return:
        """
        return list(nx.all_shortest_paths(self.graph,
                                          source=source_lanelet_id,
                                          target=target_lanelet_id))

    def get_instruction_from_route(self, route: List[int]) -> List[int]:
        """
        Get lane change instruction for planned shortest path.
        If the next lanelet is the successor of the previous lanelet, no lane change is needed and the instruction is
        0.
        If the next lanelet is the adjacent lanelet of the previous lanelet, lane change is necessary therefore the
        instruction is 1.
        """
        instruction = []
        for idx, lanelet_id in enumerate(route):
            if idx < len(route) - 1:
                if route[idx + 1] in self.lanelet_network.find_lanelet_by_id(lanelet_id).successor:
                    instruction.append(0)
                else:
                    instruction.append(1)
            else:
                instruction.append(0)
        return instruction

    def get_split_factor(self, instruction_list: List) -> Tuple[List[float], List[float]]:
        """
        Get the split factor for lane change. From the lane change instruction list, the split factor for lane change
        is calculated.
        For example, if there are three consecutive lane change (assuming three lanes are all parallel),
        then the split factor for first lane should be 1/3 and 2/3 for the second lane.
        """
        consecutive = [list(v) for k, v in itertools.groupby(instruction_list)]
        end_idx_list = []
        start_idx_list = [0.]
        for c in consecutive:
            for idx, element in enumerate(c):
                if element == 0:
                    end_idx = 1.
                else:
                    end_idx = (idx + 1) / (len(c) + 1)
                end_idx_list.append(end_idx)
        assert len(end_idx_list) == len(instruction_list)
        if len(end_idx_list) > 1:
            for idx in range(1, len(end_idx_list)):
                if np.isclose(end_idx_list[idx - 1], 1.):
                    start_idx_list.append(0.)
                else:
                    start_idx_list.append(end_idx_list[idx - 1])
        assert len(start_idx_list) == len(instruction_list)
        return start_idx_list, end_idx_list

    def get_ref_path_from_route(self, route: List[int]) -> np.ndarray:
        """
        Converts specified route (lanelet ids) to polylines
        :param route: a list of lanelet ids
        :return: converted reference path
        """
        ref_path = None
        instruction = self.get_instruction_from_route(route)
        start_idx_list, end_idx_list = self.get_split_factor(instruction)
        for idx, lanelet_id in enumerate(route):
            lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
            # resample the center vertices to prevent too few vertices with too large distance
            center_vertices_resampled = resample_polyline(lanelet.center_vertices)
            # we use transition_vertice_idx of vertices of previous and current lanelet to construct lane change
            transition_vertice_idx_max = 5
            # define the proportion that transition_vertices_index occupies in the entire lanelet
            transition_proportion = 5
            transition_vertice_idx = min(len(center_vertices_resampled) // transition_proportion + 1,
                                         transition_vertice_idx_max)

            if ref_path is None:
                ref_path = center_vertices_resampled[
                           int(start_idx_list[idx] * len(center_vertices_resampled)):int(end_idx_list[idx] * len(
                               center_vertices_resampled)) - transition_vertice_idx, :]
            else:
                ref_path = np.concatenate((ref_path, center_vertices_resampled[
                                                     int(start_idx_list[idx] * len(
                                                         center_vertices_resampled)) + transition_vertice_idx:
                                                     int(end_idx_list[idx] * len(
                                                         center_vertices_resampled)) - transition_vertice_idx, :]
                                           ), axis=0)
        return ref_path

    def search_alg(self, current_state):
        # problem_init_state = self.planning_problem.initial_state
        problem_init_state = current_state
        initial_lanelet_id_list = self.lanelet_network.find_lanelet_by_position([problem_init_state.position])[0]
        for goal_lanelet_id in self.goal_lanelet_ids:
            for initial_lanelet in initial_lanelet_id_list:
                try:
                    all_route = self.find_all_shortest_paths(initial_lanelet, goal_lanelet_id)
                    return all_route
                except TypeError:
                    all_route = self.find_all_shortest_paths(initial_lanelet, goal_lanelet_id[0])
                    return all_route
                except NetworkXNoPath:
                    pass

    def smooth_reference(self, ref_path):
        if ref_path.shape[0] < 5:
            ref_path = resample_polyline(ref_path)
        ref_path = smooth_reference(ref_path)
        return ref_path

    def generate_ref_path(self) -> List[np.ndarray]:
        """
        Calculates a reference path for a given lanelet network
        :return:
        """
        goal_lanelet_id_list = self.goal_lanelet_ids

        problem_init_state = self.planning_problem.initial_state
        initial_lanelet_id_list = self.lanelet_network.find_lanelet_by_position([problem_init_state.position])[0]

        ref_path_list = []
        for goal_lanelet_id in goal_lanelet_id_list:
            for initial_lanelet in initial_lanelet_id_list:
                try:
                    all_route = self.find_all_shortest_paths(initial_lanelet, goal_lanelet_id)
                    # TODO: return all routes that were found
                    # route = all_route[0]
                    for route in all_route:
                        ref_path = self.get_ref_path_from_route(route)
                        if ref_path.shape[0] < 5:
                            ref_path = resample_polyline(ref_path)
                        ref_path = smooth_reference(ref_path)
                        ref_path_list.append(ref_path)
                except NetworkXNoPath:
                    pass
        return ref_path_list


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

    def get_ref_path(self, x_0):
        """
        Return the center line of the initial lanelet as the reference route.
        """
        routes = self.route_planner.search_alg(x_0)

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
                ref_path_sub_route = ref_lanelet.center_vertices
                try:
                    if len(ref_lanelet.center_vertices) < 5:
                        ref_path_sub_route = self.route_planner.resample_polyline(ref_path_sub_route)
                    self.ref_path_dict[sub_route[0]] = self.route_planner.smooth_reference(ref_path_sub_route)
                except:
                    self.ref_path_dict[sub_route[0]] = ref_path_sub_route
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
                new_ref_path = new_ref_lanelet.center_vertices

                try:
                    if len(new_ref_path) < 5:
                        new_ref_path = self.route_planner.resample_polyline(new_ref_path)
                    self.ref_path = self.route_planner.smooth_reference(new_ref_path)
                except:
                    self.ref_path = new_ref_path

                self.ref_path_id = new_ref_lanelet_id

        return self.ref_path

    def check_ref_lanelet_id(self, current_state):
        """
        Check the reference lanelet id for reference route switching.
        This id will be used to identify next reference route
        """
        current_lanelet_id_list = self.route_planner.lanelet_network.find_lanelet_by_position(
            list([current_state.position]))[0]
        current_lanelet_id = current_lanelet_id_list[0]
        if len(current_lanelet_id_list) > 1:
            for lanelet_id in current_lanelet_id_list:
                if lanelet_id in self.route:
                    current_lanelet_id = lanelet_id
                    break

        ref_lanelet_id = current_lanelet_id
        if current_lanelet_id in self.route:
            laneChanging = self.lane_change_request[current_lanelet_id]
        else:
            current_lanelet = self.route_planner.lanelet_network.find_lanelet_by_id(current_lanelet_id)
            if current_lanelet.adj_left is not None and current_lanelet.adj_left in self.ref_route_manager.route:
                laneChanging = self.lane_change_request[current_lanelet.adj_left]
                ref_lanelet_id = current_lanelet.adj_left
            else:
                laneChanging = self.lane_change_request[current_lanelet.adj_right]
                ref_lanelet_id = current_lanelet.adj_right

        return laneChanging, ref_lanelet_id