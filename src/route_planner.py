import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.visualization.draw_dispatch_cr import draw_object


class RoutePlanner:
    def __init__(self, lanelet_network):
        self.lanelet_network = lanelet_network
        self.create_graph_from_lanelet_network()

    def create_graph_from_lanelet_network(self):
        """ Build a graph from the lanelet network. The length of a lanelet is assigned as weight to
            its outgoing edges as in Bender P., Ziegler J., Stiller C., "Lanelets: Efficient Map
            Representation for Autonomous Driving",  IEEE Intelligent Vehicles Symposium, 2014.
            The edge weight between adjacent lanelets is set to zero.
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

    def find_all_shortest_paths(self, source_lanelet_id, target_lanelet_id):
        return list(nx.all_shortest_paths(self.graph,
                                          source=source_lanelet_id,
                                          target=target_lanelet_id))

    def find_all_simple_paths(self, source_lanelet_id, target_lanelet_id):
        return list(nx.all_simple_paths(self.graph,
                                        source=source_lanelet_id,
                                        target=target_lanelet_id))

    def find_all_lanelets_leading_to_goal(self, source_lanelet_id, target_lanelet_id, allow_overtaking=True):
        lanelet_ids_leading_to_goal = set()
        if source_lanelet_id == target_lanelet_id:
            cur_lanelet = self.lanelet_network.find_lanelet_by_id(source_lanelet_id)
            lanelet_ids_leading_to_goal.add(source_lanelet_id)
            if cur_lanelet.adj_left:
                if (cur_lanelet.adj_left_same_direction or
                        (not cur_lanelet.adj_left_same_direction and allow_overtaking)):
                    lanelet_ids_leading_to_goal.add(cur_lanelet.adj_left)
            if cur_lanelet.adj_right and cur_lanelet.adj_right_same_direction:
                lanelet_ids_leading_to_goal.add(cur_lanelet.adj_right)
        else:
            simple_paths = self.find_all_simple_paths(source_lanelet_id, target_lanelet_id)
            for p in simple_paths:
                flag = True
                pre_lanelet = self.lanelet_network.find_lanelet_by_id(p[0])
                for i, l_id in enumerate(p[1:-1]):
                    cur_lanelet = self.lanelet_network.find_lanelet_by_id(l_id)
                    if ((l_id == pre_lanelet.adj_left and
                             not pre_lanelet.adj_left_same_direction) or
                            (l_id == pre_lanelet.adj_right and
                                 not pre_lanelet.adj_right_same_direction)):
                        if p[i + 2] in cur_lanelet.successor:
                            flag = False
                            break
                    pre_lanelet = cur_lanelet
                if flag:
                    lanelet_ids_leading_to_goal = lanelet_ids_leading_to_goal.union(set(p))
            if allow_overtaking:
                overtaking_lanelets = set()
                for lanelet_id in lanelet_ids_leading_to_goal:
                    lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
                    if lanelet.adj_left and not pre_lanelet.adj_left_same_direction:
                        overtaking_lanelets.add(lanelet.adj_left)
                lanelet_ids_leading_to_goal = lanelet_ids_leading_to_goal.union(overtaking_lanelets)
        return lanelet_ids_leading_to_goal

    def find_reference_path_to_goal(self, source_lanelet_id, target_lanelet_id):
        """ Not working in many situations."""
        shortest_path = self.find_all_shortest_paths(source_lanelet_id,
                                                     target_lanelet_id)
        # take the first shortest path (there might be more than one)
        shortest_path = shortest_path[0]
        reference_lanelets = [self.lanelet_network.find_lanelet_by_id(shortest_path[0])]
        for i, id in enumerate(shortest_path[1:]):
            lanelet = self.lanelet_network.find_lanelet_by_id(id)
            preceding_lanelet = self.lanelet_network.find_lanelet_by_id(shortest_path[i])

            adjacent_lanelets = set()
            if preceding_lanelet.adj_left:
                adjacent_lanelets.add(preceding_lanelet.adj_left)
            if preceding_lanelet.adj_right:
                adjacent_lanelets.add(preceding_lanelet.adj_right)

            if id in adjacent_lanelets:
                del reference_lanelets[-1]
                reference_lanelets.append(lanelet)
            else:
                reference_lanelets.append(lanelet)

        center_vertices = reference_lanelets[0].center_vertices
        for i in range(1, len(reference_lanelets)):
            if np.isclose(center_vertices[-1],
                          reference_lanelets[i].center_vertices[0]).all():
                idx = 1
            else:
                idx = 0
            center_vertices = np.concatenate((center_vertices,
                                              reference_lanelets[i].center_vertices[idx:]))
        return center_vertices


def compute_curvature_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the curvature of a given polyline
    :param polyline: The polyline for the curvature computation
    :return: The curvature of the polyline
    """
    assert isinstance(polyline, np.ndarray) and polyline.ndim == 2 and len(polyline[:,0]) > 2, \
        'Polyline malformed for curvature computation p={}'.format(polyline)
    x_d = np.gradient(polyline[:,0])
    x_dd = np.gradient(x_d)
    y_d = np.gradient(polyline[:,1])
    y_dd = np.gradient(y_d)

    # compute curvature
    curvature = (x_d*y_dd - x_dd*y_d) / ((x_d**2 + y_d**2)**(3./2.))

    return curvature


def chaikins_corner_cutting(polyline):
    new_polyline = list()
    new_polyline.append(polyline[0])
    for i in range(0, len(polyline)-1):
        new_polyline.append((3/4)*polyline[i] + (1/4)*polyline[i+1])
        new_polyline.append((1/4)*polyline[i] + (3/4)*polyline[i+1])
    new_polyline.append(polyline[-1])
    return new_polyline


def resample_polyline(polyline, step=2.0):
    new_polyline = [polyline[0]]
    current_position = 0 + step
    current_length = np.linalg.norm(polyline[0] - polyline[1])
    current_idx = 0
    while current_idx < len(polyline) - 1:
        if current_position >= current_length:
            current_position = current_position - current_length
            current_idx += 1
            if current_idx > len(polyline) - 2:
                break
            current_length = np.linalg.norm(polyline[current_idx + 1]
                                            - polyline[current_idx])
        else:
            rel = current_position/current_length
            new_polyline.append((1-rel) * polyline[current_idx] +
                                rel * polyline[current_idx + 1])
            current_position += step
    new_polyline.append(polyline[-1])
    return np.array(new_polyline)


def find_reference_path_and_lanelets_leading_to_goal(
        route_planner: RoutePlanner, planning_problem: PlanningProblem,
        target_lanelet_id: int, allow_overtaking: bool,
        resampling_step_reference_path: float = 0.5,
        max_curvature_reference_path: float = 0.2):
    source_lanelet = route_planner.lanelet_network.lanelets_in_proximity(
        planning_problem.initial_state.position, 100)

    if len(source_lanelet) < 1:
        raise ValueError('Expected exactly one source lanelet. Found no source lanelet.')
    else:
        source_lanelet = source_lanelet[0]

    start_lanelet = source_lanelet
    if source_lanelet.predecessor:
        start_lanelet = route_planner.lanelet_network.find_lanelet_by_id(source_lanelet.predecessor[0])
    reference_path = route_planner.find_reference_path_to_goal(
        start_lanelet.lanelet_id, target_lanelet_id)
    lanelets_leading_to_goal = route_planner.find_all_lanelets_leading_to_goal(
        start_lanelet.lanelet_id, target_lanelet_id, allow_overtaking)

    # smooth reference path until curvature is smaller or equal max_curvature_reference_path
    max_curvature = max_curvature_reference_path + 0.2
    while max_curvature > max_curvature_reference_path:
        reference_path = np.array(chaikins_corner_cutting(reference_path))
        reference_path = resample_polyline(reference_path, resampling_step_reference_path)
        max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))
    return reference_path, lanelets_leading_to_goal


if __name__ == '__main__':
    scenario_path = "/home/fabian/Praktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Over-1_1.xml"
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    route_planner = RoutePlanner(scenario.lanelet_network)
    reference_path, lanelets_leading_to_goal = find_reference_path_and_lanelets_leading_to_goal(
        route_planner=route_planner,
        planning_problem=planning_problem_set.find_planning_problem_by_id(11425),
        target_lanelet_id=3492,
        allow_overtaking=False,
        resampling_step_reference_path=1.5,
        max_curvature_reference_path=0.15)

    draw_object(scenario.lanelet_network, draw_params={'lanelet_network': {'lanelet': {'show_label': True}}})
    draw_object(planning_problem_set)

    for id in lanelets_leading_to_goal:
        l = scenario.lanelet_network.find_lanelet_by_id(id)
        draw_object(l, draw_params={'lanelet': {
            'left_bound_color': 'yellow',
            'right_bound_color': 'yellow',
            'center_bound_color': '#dddddd',
            'draw_left_bound': False,
            'draw_right_bound': False,
            'draw_center_bound': False,
            'draw_border_vertices': False,
            'draw_start_and_direction': False,
            'show_label': False,
            'draw_linewidth': 4,
            'fill_lanelet': True,
            'facecolor': 'yellow',
            'zorder': 45}})

        plt.plot(reference_path[:, 0], reference_path[:, 1], '-*g', linewidth=4, zorder=50)

    plt.axes().autoscale()
    plt.gca().set_aspect('equal')
    plt.axis()
    plt.show(block=True)