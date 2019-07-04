import networkx as nx
import numpy as np
import xml.etree.ElementTree as ET
from commonroad.common.file_reader import CommonRoadFileReader
from polyline import compute_curvature_from_polyline, chaikins_corner_cutting, resample_polyline
from commonroad.visualization.draw_dispatch_cr import draw_object
import matplotlib.pyplot as plt


class RoutePlanner:
    def __init__(self, lanelet_network, scenario_path):

        self.scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

        # Find planning problem or select initial state
        if not planning_problem_set:
            root = ET.parse(scenario_path).getroot()
            for lane in root.findall('lanelet'):
                laneletid = lane.get('id')
                print("Lanelet ID" + laneletid)
                break

            print('No Planning Problem specified for this scenario! Lanelet ', laneletid, ' is chosen.')

        else:
            root = ET.parse(scenario_path).getroot()
            for problem in root.findall('planningProblem'):
                problemid = problem.get('id')
                print("Planning Prob ID" + problemid)
                break

            self.planning_problem = planning_problem_set.find_planning_problem_by_id(int(problemid))

        # find goal region
        if hasattr(self.planning_problem.goal.state_list[0], 'position'):
            if self.planning_problem.goal.lanelets_of_goal_position:
                goal_lanelet = self.planning_problem.goal.lanelets_of_goal_position
                print ("Goal lanelets" + str(goal_lanelet))
                self.goal_lanelet_id = goal_lanelet[0][0]
            else:
                goal_lanelet = self.scenario.lanelet_network.lanelets_in_proximity(
                    self.planning_problem.goal.state_list[0].position.center, 10)
                self.goal_lanelet_id = list(goal_lanelet)[-1].lanelet_id
        else:
            # set initial lanelet as goal lanelet if not other specified
            goal_lanelet = self.scenario.lanelet_network.lanelets_in_proximity(
                self.planning_problem.initial_state.position, 10)
            self.goal_lanelet_id = list(goal_lanelet)[0].lanelet_id
            print('No Goal Region defined: Driving on initial lanelet.')

        print("goal lanelet ID" + str(self.goal_lanelet_id))

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

    def find_all_lanelets_leading_to_goal(self, source_lanelet_id, allow_overtaking=True):
        lanelet_ids_leading_to_goal = set()
        if source_lanelet_id == self.goal_lanelet_id:
            cur_lanelet = self.lanelet_network.find_lanelet_by_id(source_lanelet_id)
            lanelet_ids_leading_to_goal.add(source_lanelet_id)
            if cur_lanelet.adj_left:
                if (cur_lanelet.adj_left_same_direction or
                        (not cur_lanelet.adj_left_same_direction and allow_overtaking)):
                    lanelet_ids_leading_to_goal.add(cur_lanelet.adj_left)
            if cur_lanelet.adj_right and cur_lanelet.adj_right_same_direction:
                lanelet_ids_leading_to_goal.add(cur_lanelet.adj_right)
        else:
            simple_paths = self.find_all_simple_paths(source_lanelet_id, self.goal_lanelet_id)
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

    def find_reference_path_to_goal(self, source_lanelet_id):
        """ Not working in many situations."""

        obstacle_map = self.lanelet_network.map_obstacles_to_lanelets(self.scenario.dynamic_obstacles)
        shortest_paths = self.find_all_shortest_paths(source_lanelet_id, self.goal_lanelet_id)
        # take the first shortest path (there might be more than one)

        obstacles = []
        shortest_path = shortest_paths[0]
        for path in shortest_path:
            if path in obstacle_map:
                for elem in obstacle_map[path]:
                    obstacles.append(elem)

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

    def find_reference_path_and_lanelets_leading_to_goal(
            self,
            allow_overtaking: bool,
            source_position=None,
            resampling_step_reference_path: float = 0.5,
            max_curvature_reference_path: float = 0.2):

        if source_position is None:
            source_position = self.planning_problem.initial_state.position
        sourcelanelets = self.lanelet_network.find_lanelet_by_position(np.array([source_position]))
        source_lanelet = self.lanelet_network.find_lanelet_by_id(sourcelanelets[0][0])

        start_lanelet = source_lanelet
        if source_lanelet.predecessor:
            start_lanelet = self.lanelet_network.find_lanelet_by_id(source_lanelet.predecessor[0])
        reference_path = self.find_reference_path_to_goal(
            start_lanelet.lanelet_id)
        lanelets_leading_to_goal = self.find_all_lanelets_leading_to_goal(
            start_lanelet.lanelet_id, allow_overtaking)

        # smooth reference path until curvature is smaller or equal max_curvature_reference_path
        max_curvature = max_curvature_reference_path + 0.2
        while max_curvature > max_curvature_reference_path:
            reference_path = np.array(chaikins_corner_cutting(reference_path))
            reference_path = resample_polyline(reference_path, resampling_step_reference_path)
            max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))
        return reference_path, lanelets_leading_to_goal


if __name__ == '__main__':
    scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_2_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Urban-1_1_S-1.xml'
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    route_planner = RoutePlanner(scenario.lanelet_network, scenario_path)
    reference_path, lanelets_leading_to_goal = route_planner.find_reference_path_and_lanelets_leading_to_goal(
        allow_overtaking=False,
        resampling_step_reference_path=1.5,
        max_curvature_reference_path=0.15,
        source_position = None)

    draw_object(scenario.lanelet_network, draw_params={'lanelet_network': {'lanelet': {'show_label': True}}})
    draw_object(planning_problem_set)

    for id in lanelets_leading_to_goal:
        l = scenario.lanelet_network.find_lanelet_by_id(id)
        draw_object(l, draw_params={'lanelet': {
            'left_bound_color': 'yellow',
            'right_bound_color': 'yellow',
            'center_bound_color': '#dddddd',
            'draw_left_bound': True,
            'draw_right_bound': False,
            'draw_center_bound': False,
            'draw_border_vertices': False,
            'draw_start_and_direction': False,
            'show_label': False,
            'draw_linewidth': 4,
            'fill_lanelet': True,
            'facecolor': 'yellow',
            'zorder': 45}
       })
        #draw_object(scenario)

        plt.plot(reference_path[:, 0], reference_path[:, 1], '-*g', linewidth=4, zorder=50)

    plt.axes().autoscale()
    plt.gca().set_aspect('equal')
    plt.axis()
    plt.show(block=True)