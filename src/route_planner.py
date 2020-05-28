import itertools
from typing import List, Tuple

import networkx as nx
import numpy as np
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad_ccosy.geometry.util import resample_polyline
from networkx import NetworkXNoPath

from scenario_helpers import smooth_reference


def create_graph_from_lanelet_network(lanelet_network: LaneletNetwork) -> nx.DiGraph:
    graph = nx.DiGraph()
    nodes = list()
    edges = list()

    for lanelet in lanelet_network.lanelets:
        nodes.append(lanelet.lanelet_id)

        if lanelet.successor:
            for successor_id in lanelet.successor:
                edges.append((lanelet.lanelet_id, successor_id, {'weight': lanelet.distance[-1]}))
        if lanelet.adj_right_same_direction:
            edges.append((lanelet.lanelet_id, lanelet.adj_right, {'weight': lanelet.distance[-1]}))
        if lanelet.adj_left_same_direction:
            edges.append((lanelet.lanelet_id, lanelet.adj_left, {'weight': lanelet.distance[-1]}))
    graph.add_nodes_from(nodes)
    graph.add_edges_from(edges)
    return graph


def get_goal_lanelet(scenario: Scenario, planning_problem: PlanningProblem):
    if planning_problem.goal.lanelets_of_goal_position is None:
        if hasattr(planning_problem.goal.state_list[0], 'position'):
            goal_lanelet_id_list = scenario.lanelet_network.find_lanelet_by_position(
                [planning_problem.goal.state_list[0].position.center])[0]
        else:
            # naively concatenate successors, TODO: use some neuristic?
            l_id = scenario.lanelet_network.find_lanelet_by_position([planning_problem.initial_state.position])[0][0]
            current_lanelet = scenario.lanelet_network.find_lanelet_by_id(l_id)
            min_dist = planning_problem.initial_state.velocity * planning_problem.goal.state_list[0].time_step.end + 200
            dist = current_lanelet.distance[-1]
            while dist < min_dist and current_lanelet is not None:
                l_id = current_lanelet.successor[0] if len(current_lanelet.successor) > 0 else None
                current_lanelet = scenario.lanelet_network.find_lanelet_by_id(l_id) if l_id is not None else None
                if current_lanelet is not None:
                    dist += current_lanelet.distance[-1]

            goal_lanelet_id_list = [current_lanelet.lanelet_id]
    else:
        goal_lanelet_id_list = list(planning_problem.goal.lanelets_of_goal_position.keys())
        assert isinstance(goal_lanelet_id_list, list), "goal_lanelet_id_list should be a list of ids!"
    return goal_lanelet_id_list


def find_all_shortest_paths(graph: nx.DiGraph, source_lanelet_id: int, target_lanelet_id: int) -> List[List[int]]:
    return list(nx.all_shortest_paths(graph,
                                      source=source_lanelet_id,
                                      target=target_lanelet_id))


def get_instruction_from_route(lanelet_network: LaneletNetwork, route: List[int]) -> List[int]:
    instruction = []
    for idx, lanelet_id in enumerate(route):
        if idx < len(route) - 1:
            if route[idx + 1] in lanelet_network.find_lanelet_by_id(lanelet_id).successor:
                instruction.append(0)
            else:
                instruction.append(1)
        else:
            instruction.append(0)
    return instruction


def get_split_factor(instruction_list: List) -> Tuple[List[float], List[float]]:
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


def get_ref_path_from_route(lanelet_network: LaneletNetwork, route: List[int]) -> np.ndarray:
    ref_path = None
    instruction = get_instruction_from_route(lanelet_network, route)
    start_idx_list, end_idx_list = get_split_factor(instruction)
    for idx, lanelet_id in enumerate(route):
        lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
        if ref_path is None:
            ref_path = lanelet.center_vertices[
                       int(start_idx_list[idx] * len(lanelet.center_vertices)):int(end_idx_list[idx] * len(
                           lanelet.center_vertices)) - 5, :]
        else:
            ref_path = np.concatenate((ref_path, lanelet.center_vertices[
                                                 int(start_idx_list[idx] * len(lanelet.center_vertices)) + 5:int(
                                                     end_idx_list[idx] * len(
                                                         lanelet.center_vertices)) - 5, :]), axis=0)
    return ref_path


def generate_ref_path(scenario: Scenario, planning_problem: PlanningProblem) -> List[np.ndarray]:
    goal_lanelet_id_list = get_goal_lanelet(scenario, planning_problem)
    problem_init_state = planning_problem.initial_state
    initial_lanelet_id_list = scenario.lanelet_network.find_lanelet_by_position([problem_init_state.position])[0]
    graph_with_adj = create_graph_from_lanelet_network(scenario.lanelet_network)
    ref_path_list = []
    for goal_lanelet_id in goal_lanelet_id_list:
        for initial_lanelet in initial_lanelet_id_list:
            try:
                all_route = find_all_shortest_paths(graph_with_adj, initial_lanelet, goal_lanelet_id)
                route = all_route[0]
                ref_path = get_ref_path_from_route(scenario.lanelet_network, route)
                if ref_path.shape[0] < 5:
                    ref_path = resample_polyline(ref_path)
                ref_path = smooth_reference(ref_path)
                ref_path_list.append(ref_path)
            except NetworkXNoPath:
                pass
    return ref_path_list
