import itertools
from typing import List, Tuple

import numpy as np
import networkx as nx

from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad_ccosy.geometry.util import resample_polyline
from networkx import NetworkXNoPath

from scenario_helpers import smooth_reference


class RoutePlanner:
    def __init__(self, lanelet_network: LaneletNetwork, planning_problem: PlanningProblem):
        self.planning_problem = planning_problem
        self.lanelet_network = lanelet_network
        self.create_graph_from_lanelet_network()

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
            goal_lanelet_id_list = list(self.planning_problem.goal.lanelets_of_goal_position.keys())
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
            idx = len(lanelet.center_vertices) // 5 + 1
            # print(len(lanelet.center_vertices), lanelet.distance[-1], np.abs(lanelet.distance-))
            if ref_path is None:
                ref_path = lanelet.center_vertices[
                           int(start_idx_list[idx] * len(lanelet.center_vertices)):int(end_idx_list[idx] * len(
                               lanelet.center_vertices)) - idx, :]
            else:
                ref_path = np.concatenate((ref_path, lanelet.center_vertices[
                                                     int(start_idx_list[idx] * len(lanelet.center_vertices)) + idx:
                                                     int(end_idx_list[idx] * len(lanelet.center_vertices)) - idx, :]
                                           ), axis=0)
        return ref_path

    def generate_ref_path(self) -> List[np.ndarray]:
        """
        Calculates a reference path for a given lanelet network
        :return:
        """
        goal_lanelet_id_list = self.get_goal_lanelet()

        problem_init_state = self.planning_problem.initial_state
        initial_lanelet_id_list = self.lanelet_network.find_lanelet_by_position([problem_init_state.position])[0]

        ref_path_list = []
        for goal_lanelet_id in goal_lanelet_id_list:
            for initial_lanelet in initial_lanelet_id_list:
                try:
                    all_route = self.find_all_shortest_paths(initial_lanelet, goal_lanelet_id)
                    route = all_route[0]
                    ref_path = self.get_ref_path_from_route(route)
                    if ref_path.shape[0] < 5:
                        ref_path = resample_polyline(ref_path)
                    ref_path = smooth_reference(ref_path)
                    ref_path_list.append(ref_path)
                except NetworkXNoPath:
                    pass
        return ref_path_list
