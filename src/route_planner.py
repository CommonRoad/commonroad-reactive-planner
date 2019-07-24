import networkx as nx
import numpy as np
import xml.etree.ElementTree as ET
from commonroad.common.file_reader import CommonRoadFileReader
from polyline import compute_curvature_from_polyline, chaikins_corner_cutting, resample_polyline
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
import matplotlib.pyplot as plt
import cProfile
import time, warnings
from scipy import spatial


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
        self.goal_lanelet_position = None
        if hasattr(self.planning_problem.goal.state_list[0], 'position'):
            if self.planning_problem.goal.lanelets_of_goal_position:
                self.goal_lanes = self.planning_problem.goal.lanelets_of_goal_position
                print ("Goal lanelets" + str(self.goal_lanes))
                self.goal_lanelet_id = self.goal_lanes[0][0]
            else:
                self.goal_lanelet_position = self.planning_problem.goal.state_list[0].position.center
                self.goal_lanes = self.scenario.lanelet_network.find_lanelet_by_position(np.array(self.goal_lanelet_position, ndmin=2))
                self.goal_lanelet_id = self.goal_lanes[0][0]
        else:
            # set initial lanelet as goal lanelet if not other specified
            goal_lanelet = self.scenario.lanelet_network.find_lanelet_by_position(
                np.array(self.planning_problem.initial_state.position, ndmin=2))
            self.goal_lanelet_id = goal_lanelet[0][0]
            print('No Goal Region defined: Driving on initial lanelet.')

        if self.goal_lanelet_position is None:
            goal_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(self.goal_lanelet_id)
            self.goal_lanelet_position = goal_lanelet.center_vertices[-5]

        print("goal lanelet ID" + str(self.goal_lanelet_id))

        self.lanelet_network = self.scenario.lanelet_network
        self.graph = self.create_graph_from_lanelet_network()
        self.reference_paths = {}
        for lanelet in self.lanelet_network.lanelets:
            self.reference_paths[lanelet.lanelet_id] = lanelet.center_vertices


    def create_discrete_graph_from_lanelet_network(self, lanelet_network = None, allow_overtaking: bool = False):
        """ Build a graph from the lanelet network. The length of a lanelet is assigned as weight to
            its outgoing edges as in Bender P., Ziegler J., Stiller C., "Lanelets: Efficient Map
            Representation for Autonomous Driving",  IEEE Intelligent Vehicles Symposium, 2014.
            The edge weight between adjacent lanelets is set to zero.
        """

        if lanelet_network is None:
            lanelet_network = self.lanelet_network
        graph = nx.DiGraph()
        nodes = list()
        edges = list()
        for lanelet in lanelet_network.lanelets:
            nodes.append(lanelet.lanelet_id)
            if lanelet.successor:
                for successor in lanelet.successor:
                    l = lanelet_network.find_lanelet_by_id(successor)
                    edges.append((lanelet.lanelet_id, l.lanelet_id, {'weight': lanelet.distance[-1]}))
                    if l.adj_left:
                        left = lanelet_network.find_lanelet_by_id(l.adj_left)
                        edges.append((lanelet.lanelet_id, left.lanelet_id,
                                      {'weight': 0, 'same_dir': lanelet.adj_left_same_direction}))
                    if l.adj_right:
                        right = lanelet_network.find_lanelet_by_id(l.adj_right)
                        edges.append((lanelet.lanelet_id, right.lanelet_id,
                                      {'weight': 0, 'same_dir': lanelet.adj_right_same_direction}))
            if lanelet.adj_right:
                l_right = lanelet_network.find_lanelet_by_id(lanelet.adj_right)
                if l_right.successor:
                    for successor in l_right.successor:
                        if (lanelet.lanelet_id, successor,{'weight': 0, 'same_dir': lanelet.adj_right_same_direction}) not in edges:
                            edges.append((lanelet.lanelet_id, successor,
                                      {'weight': 0, 'same_dir': lanelet.adj_right_same_direction}))
            if lanelet.adj_left:
                l_left = lanelet_network.find_lanelet_by_id(lanelet.adj_left)
                if l_left.successor:
                    for successor in l_left.successor:
                        if (lanelet.lanelet_id, successor,{'weight': 0, 'same_dir': lanelet.adj_left_same_direction}) not in edges:
                            edges.append((lanelet.lanelet_id, successor,
                                      {'weight': 0, 'same_dir': lanelet.adj_left_same_direction}))
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def create_graph_from_lanelet_network(self, lanelet_network = None, allow_overtaking: bool = False):
        """ Build a graph from the lanelet network. The length of a lanelet is assigned as weight to
            its outgoing edges as in Bender P., Ziegler J., Stiller C., "Lanelets: Efficient Map
            Representation for Autonomous Driving",  IEEE Intelligent Vehicles Symposium, 2014.
            The edge weight between adjacent lanelets is set to zero.
        """

        if lanelet_network is None:
            lanelet_network = self.lanelet_network
        graph = nx.DiGraph()
        nodes = list()
        edges = list()
        for lanelet in lanelet_network.lanelets:
            nodes.append(lanelet.lanelet_id)
            if lanelet.successor:
                for successor in lanelet.successor:
                    l = lanelet_network.find_lanelet_by_id(successor)
                    edges.append((lanelet.lanelet_id, l.lanelet_id, {'weight': lanelet.distance[-1]}))
            if lanelet.adj_left:
                l = lanelet_network.find_lanelet_by_id(lanelet.adj_left)
                edges.append((lanelet.lanelet_id, l.lanelet_id,
                              {'weight': 0, 'same_dir': lanelet.adj_left_same_direction}))
            if lanelet.adj_right:
                l = lanelet_network.find_lanelet_by_id(lanelet.adj_right)
                edges.append((lanelet.lanelet_id, l.lanelet_id,
                              {'weight': 0, 'same_dir': lanelet.adj_right_same_direction}))
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def create_reference_path_network(self):

        lanelet_network = self.lanelet_network

        lanelet_mapping = {}
        goal_lanelet = lanelet_network.find_lanelet_by_id((self.goal_lanelet_id))
        distance, goal_index = spatial.KDTree(goal_lanelet.center_vertices).query(self.goal_lanelet_position)

        left_lanes, right_lanes, max_length = self.get_adjacent_lanelets_list(self.goal_lanelet_id)

        # compute split factor (subtract 1, because source lanelet is in list)
        split_factor = max(len(left_lanes), len(right_lanes))
        split_factor = max(1, split_factor)
        goal_split_lanelets = []

        remove_goal_lanelets = set()
        if goal_index < (split_factor+1) * 2:
            # goal lanelet part has not enough center vertices -> choose preceding one if possible
            if goal_lanelet.predecessor:
                goal_lanelet = lanelet_network.find_lanelet_by_id(goal_lanelet.predecessor[0])
                goal_index = len(goal_lanelet.center_vertices[:]) - 1
                left_lanes, right_lanes, max_length = self.get_adjacent_lanelets_list(goal_lanelet.lanelet_id)

                # compute split factor (subtract 1, because source lanelet is in list)
                split_factor = max(len(left_lanes), len(right_lanes)) - 2
                split_factor = max(1, split_factor)

                # we still need to split the goal lanelets
                goal_left, goal_right, max_length = self.get_adjacent_lanelets_list(self.goal_lanelet_id)
                goal_split_lanelets = (np.concatenate((goal_left, goal_right), axis=0)).tolist()
                goal_split_lanelets.append(self.goal_lanelet_id)
                goal_split_lanelets = list(set(goal_split_lanelets))
                goal_split_lanelets = [int(l) for l in goal_split_lanelets]

                remove_goal_lanelets = set(goal_split_lanelets)
                set2 = {self.goal_lanelet_id, goal_lanelet.adj_left, goal_lanelet.adj_right}
                remove_goal_lanelets.difference(set2)

        split_lanelets = (np.concatenate((left_lanes, right_lanes), axis=0)).tolist()
        split_lanelets.append(goal_lanelet.lanelet_id)
        split_lanelets = list(set(split_lanelets))
        split_lanelets = [int(l) for l in split_lanelets]

        map_ids = list(range(0, (split_factor + 2) * len(lanelet_network.lanelets)))
        for lanelet in lanelet_network.lanelets:
            if lanelet.lanelet_id in map_ids:
                map_ids.remove(lanelet.lanelet_id)

        num = 0
        for lanelet in split_lanelets:
            if lanelet == self.goal_lanelet_id:
                lanelet_mapping[lanelet] = map_ids[num: num + split_factor + 1]
                num += split_factor + 1
            else:
                lanelet_mapping[lanelet] = map_ids[num: num + split_factor + 1]
                num += split_factor + 1

        for lanelet in goal_split_lanelets:
            if lanelet == self.goal_lanelet_id:
                lanelet_mapping[lanelet] = map_ids[num: num + 2]
                num += 2
            else:
                lanelet_mapping[lanelet] = map_ids[num: num + 2]
                num += 2

        other_lanelets = []
        # get all other lanelets
        for lanelet in lanelet_network.lanelets:
            if lanelet.lanelet_id in split_lanelets:
                continue
            elif lanelet.lanelet_id in goal_split_lanelets:
                continue
            else:
                temp_pred = []
                temp_suc = []
                temp_r = None
                temp_l = None
                new = False
                # check if left in lanelet_mapping
                if lanelet.predecessor:
                    if lanelet.predecessor[0] in lanelet_mapping:
                        temp_pred.append(lanelet_mapping[lanelet.predecessor[0]][-1])
                        new = True
                    else:
                        temp_pred = lanelet.predecessor
                if lanelet.successor:
                    if lanelet.successor[0] in lanelet_mapping:
                        temp_suc.append(lanelet_mapping[lanelet.successor[0]][0])
                        new = True
                    else:
                        temp_suc = lanelet.successor
                if lanelet.adj_right:
                    if lanelet.adj_right in lanelet_mapping:
                        temp_r = lanelet_mapping[lanelet.adj_right][0]
                        new = True
                if lanelet.adj_left:
                    if lanelet.adj_left in lanelet_mapping:
                        temp_l = lanelet_mapping[lanelet.adj_left][0]
                        new = True
                if new:
                    temp_lane = Lanelet(left_vertices=lanelet.left_vertices, center_vertices=lanelet.center_vertices,
                                        right_vertices=lanelet.right_vertices,
                                        lanelet_id=lanelet.lanelet_id,
                                        predecessor=temp_pred, successor=temp_suc,
                                        adjacent_left=temp_l, adjacent_left_same_direction=lanelet.adj_left_same_direction,
                                        adjacent_right=temp_r,
                                        adjacent_right_same_direction=lanelet.adj_right_same_direction,
                                        speed_limit=lanelet.speed_limit)
                    other_lanelets.append(temp_lane)
                else:
                    other_lanelets.append(lanelet)

        created_lanelets, end_id, lanelet_mapping= self.create_split_lanelets(split_lanelets, split_factor, lanelet_mapping, map_ids)
        # append all not split lanelets
        all_lanelets = (np.concatenate((created_lanelets, other_lanelets), axis=0)).tolist()

        goal_created_lanelets = []
        if goal_split_lanelets != []:
            # split_factor = 2
            goal_created_lanelets, end_id, lanelet_mapping = self.create_split_lanelets(lanelets=goal_split_lanelets, split_factor=1,
                                                                       lanelet_mapping=lanelet_mapping, map_ids=map_ids,
                                                                       start_id=end_id)

        all_lanelets = (np.concatenate((all_lanelets, goal_created_lanelets), axis=0)).tolist()

        # create new lanelet network for graph
        new_network = LaneletNetwork()
        new_network = new_network.create_from_lanelet_list(all_lanelets)

        # goal = lanelet_mapping[self.goal_lanelet_id][-1]

        return new_network, lanelet_mapping, remove_goal_lanelets

    def create_split_lanelets(self, lanelets, split_factor, lanelet_mapping, map_ids, start_id=0):
        lanelet_network = self.lanelet_network

        created_lanelets = []
        lanes = []
        id = start_id
        # split each lanelet
        for lanelet_id in lanelets:
            lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
            dis, lane_index = spatial.KDTree(lanelet.center_vertices).query(self.goal_lanelet_position)
            lanelet_length = len(lanelet.center_vertices[0:lane_index + 2])

            split_array = np.arange(lanelet_length - (2 * split_factor), lanelet_length, 2)

            lanes.clear()
            # split lanelet into sublanelets
            lanes = np.split(np.array(lanelet.center_vertices[0:lanelet_length]), split_array)

            if len(lanes[-1]) <= 1:
                lanes = lanes[0:len(lanes)-1]
                map_ids.remove((lanelet_mapping[lanelet_id])[-1])
                del (lanelet_mapping[lanelet_id])[-1]

            # get predecessor for first new lanelet
            p_list = lanelet.predecessor
            pred_list = []
            for pred in p_list:
                if pred in lanelet_mapping:
                    if len(lanelet_mapping[pred]) > 1:
                        pred_list.append(lanelet_mapping[pred][-1])
                    else:
                        pred_list.append(lanelet_mapping[pred][0])
                else:
                    pred_list.append(pred)

            if pred_list == []:
                pred_list = None

            for i in range(len(lanes)):
                temp_right = None
                temp_left = None
                suc_list = []
                if not(lanelet_id == self.goal_lanelet_id and i == len(lanes)-1):
                    # get successor from lanelet_mapping
                    if i == split_factor and lanelet_id == self.goal_lanelet_id:
                        suc_list.append(map_ids[id + 1])
                    if i == split_factor:
                        s_list = lanelet.successor
                        for suc in s_list:
                            if suc in lanelet_mapping:
                                suc_list.append(lanelet_mapping[suc][0])
                            else:
                                suc_list.append(suc)
                    else:
                        suc_list.append(map_ids[id + 1])
                    # if suc_list is empty set it to None, otherwise problems with new lanelet
                    if suc_list == []:
                        suc_list = None

                    # get left and right adjacent from lanelet_mapping (attention: (not) same direction)
                    if lanelet.adj_right:
                        if lanelet.adj_right in lanelet_mapping:
                            right_lanelets = lanelet_mapping[lanelet.adj_right]
                            if lanelet.adj_right_same_direction:
                                temp_right = right_lanelets[i]
                            else:
                                if len(right_lanelets) == 1:
                                    temp_right = right_lanelets[0]
                                else:
                                    temp_right = right_lanelets[-1 - i]
                    if lanelet.adj_left:
                        if lanelet.adj_left in lanelet_mapping:
                            left_lanelets = lanelet_mapping[lanelet.adj_left]
                            if lanelet.adj_left_same_direction:
                                temp_left = left_lanelets[i]
                            else:
                                if len(left_lanelets) == 1:
                                    temp_left = left_lanelets[0]
                                else:
                                    temp_left = left_lanelets[-1 - i]

                # create new lanelet: set successor, adjacent_left, adjacent_right are set according to lanelet_mapping
                # in the graph generation we don't care about left and right_vertices: just assign center_vertices
                new_lanelet = Lanelet(left_vertices=lanes[i], center_vertices=lanes[i], right_vertices=lanes[i],
                                      lanelet_id=map_ids[id],
                                      predecessor=pred_list, successor=suc_list,
                                      adjacent_left=temp_left,
                                      adjacent_left_same_direction=lanelet.adj_left_same_direction,
                                      adjacent_right=temp_right,
                                      adjacent_right_same_direction=lanelet.adj_right_same_direction,
                                      speed_limit=lanelet.speed_limit)
                id += 1
                # store new lanelet
                created_lanelets.append(new_lanelet)
                # reset pred_list for next lanelet
                pred_list = []
                # append current lanelet as pred for next lanelet
                pred_list.append(new_lanelet.lanelet_id)

        return created_lanelets, id, lanelet_mapping

    def get_adjacent_lanelets_list(self, lanelet_id):
        """
        Recursively gets adj_left and adj_right lanelets of current lanelet id
        :param lanelet: current lanelet id
        :return: List of adjacent lanelets, empty list if there are none
        """
        list_left = []
        list_right = []
        left = True
        right = True
        temp_id = lanelet_id
        max_length = 0
        while left:
            current_lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
            if len(current_lanelet.center_vertices[:]) > max_length:
                max_length = len(current_lanelet.center_vertices[:])
            if (current_lanelet.adj_left is not None) and current_lanelet.adj_left_same_direction:
                list_left.append(current_lanelet.adj_left)
                lanelet_id = current_lanelet.adj_left
            else:
                left = False
        # reset lanelet_id for right while loop
        lanelet_id = temp_id
        while right:
            current_lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
            if len(current_lanelet.center_vertices[:]) > max_length:
                max_length = len(current_lanelet.center_vertices[:])
            if (current_lanelet.adj_right is not None) and current_lanelet.adj_right_same_direction:
                list_right.append(current_lanelet.adj_right)
                lanelet_id = current_lanelet.adj_right
            else:
                right = False
        return list_left, list_right, max_length

    def find_all_shortest_paths(self, graph, source_lanelet_id, target_lanelet_id):
        return list(nx.all_shortest_paths(graph,
                                          source=source_lanelet_id,
                                          target=target_lanelet_id))

    def find_all_simple_paths(self, graph, source_lanelet_id, target_lanelet_id):
        return list(nx.all_simple_paths(graph,
                                        source=source_lanelet_id,
                                        target=target_lanelet_id))

    def find_all_lanelets_leading_to_goal(self, source_lanelet_id, allow_overtaking=True, graph=None):
        """
        Finds all lanelets connecting source to goal lanelet
        :param source_lanelet_id: lanelet to start searching
        :param allow_overtaking: True if overtaking on left opposite lanelets is allowed
        :param graph: graph created from lanelet network to search
        :return: list of lanelet ids
        """
        if graph is None:
            graph = self.graph
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
            simple_paths = self.find_all_simple_paths(graph, source_lanelet_id, self.goal_lanelet_id)
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

    def find_reference_path_to_goal(self, source_lanelet_id, goal_lanelet_id=None, lanelet_network=None, graph=None):
        """ Not working in many situations."""

        if graph is None:
            graph = self.graph

        if lanelet_network is None:
            lanelet_network = self.lanelet_network

        if goal_lanelet_id is None:
            goal_lanelet_id = self.goal_lanelet_id

        shortest_paths = self.find_all_shortest_paths(graph, source_lanelet_id, goal_lanelet_id)
        # take the first shortest path (there might be more than one)
        if shortest_paths == []:
            return None
        shortest_path = shortest_paths[0]
        #print(shortest_path)

        reference_lanelets = [lanelet_network.find_lanelet_by_id(int(shortest_path[0]))]
        for i, id in enumerate(shortest_path[1:]):
            lanelet = lanelet_network.find_lanelet_by_id(id)
            preceding_lanelet = lanelet_network.find_lanelet_by_id(int(shortest_path[i]))

            adjacent_lanelets = set()
            if preceding_lanelet.lanelet_id == source_lanelet_id:
                if preceding_lanelet.adj_left:
                    adjacent_lanelets.add(preceding_lanelet.adj_left)
                if preceding_lanelet.adj_right:
                    adjacent_lanelets.add(preceding_lanelet.adj_right)
                if id not in adjacent_lanelets:
                    reference_lanelets.append(lanelet)
            else:
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
        """
        Search for path from lanelet leading to goal + find all lanelets connecting start to goal
        :param allow_overtaking:True if overtaking on left opposite lanelets is allowed
        :param source_position: np.array containing vehicle position
        :param resampling_step_reference_path: resampling step
        :param max_curvature_reference_path: max curvature of reference path
        :return:
        """

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


    def plan_all_reference_paths(self, source_position=None,
                       resampling_step_reference_path: float = 1.5,
                       max_curvature_reference_path: float = 0.2):
        """
        Finds a new reference path to perform a lane change
        :param source_position: np.array[x,y]: current vehicle position
        :param resampling_step_reference_path: resampling step
        :param max_curvature_reference_path: max curvature of reference path
        :return:
        """

        if source_position is None:
            source_position = self.planning_problem.initial_state.position
        sourcelanelets = self.lanelet_network.find_lanelet_by_position(np.array([source_position]))
        source_lanelet = self.lanelet_network.find_lanelet_by_id(sourcelanelets[0][0])

        lanelets_leading_to_goal = set()
        for lanelet in self.lanelet_network.lanelets:
            if lanelet.lanelet_id != self.goal_lanelet_id:
                _lanelets = self.find_all_lanelets_leading_to_goal(lanelet.lanelet_id, allow_overtaking=False)
                lanelets_leading_to_goal |= _lanelets

        lanelets_leading_to_goal = list(set(lanelets_leading_to_goal))

        new_network, lanelet_mapping, not_leading_to_goal = self.create_reference_path_network()
        graph = self.create_discrete_graph_from_lanelet_network(lanelet_network=new_network)

        if self.goal_lanelet_id in lanelet_mapping:
            goal = lanelet_mapping[self.goal_lanelet_id][-1]
        else:
            goal = self.goal_lanelet_id

        reference_paths = {}

        for start in lanelets_leading_to_goal:
            start_id = start
            if start_id in not_leading_to_goal:
                continue
            if start in lanelet_mapping:
                start_id = lanelet_mapping[start][0]

            reference_path = self.find_reference_path_to_goal(
                start_id, goal, lanelet_network= new_network, graph=graph)
            if reference_path is None:
                reference_path = source_lanelet.center_vertices
            # smooth reference path until curvature is smaller or equal max_curvature_reference_path
            max_curvature = max_curvature_reference_path + 0.2
            while max_curvature > max_curvature_reference_path:
                reference_path = np.array(chaikins_corner_cutting(reference_path))
                reference_path = resample_polyline(reference_path, resampling_step_reference_path)
                max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))

            reference_paths[start] = reference_path

        self.reference_paths = reference_paths
        return reference_paths, lanelets_leading_to_goal

    def set_reference_lane(self, lane_direction: int, position) -> None:
        """
        compute new reference path based on relative lane position.
        :param lane_direction: 0=curernt lane >0=right lanes, <0=left lanes
        :return: new reference lane vertices
        """
        assert self.lanelet_network is not None,\
            'lanelet network must be provided during initialization for using get_reference_of_lane().'

        current_ids = self.lanelet_network.find_lanelet_by_position(np.array([position]))[0]
        if len(current_ids) > 0:
            current_lanelet = self.lanelet_network.find_lanelet_by_id(current_ids[0])
        else:
            #return self.reference_paths[current_lanelet.lanelet_id]
            raise ValueError('set_reference_lane: x0 is not located on any lane and no previous reference available.')

        # determine target lane
        target_lanelet = None
        if lane_direction==-1:
            if current_lanelet.adj_left_same_direction not in (False,None):
                target_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet.adj_left)
        elif lane_direction == 1:
            if current_lanelet.adj_right_same_direction not in (False,None):
                target_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet.adj_right)
        elif lane_direction == 0:
            target_lanelet = current_lanelet

        if target_lanelet is None:
            warnings.warn('set_reference_lane: No adjacent lane in direction {}, stay in current lane.'.format(lane_direction), stacklevel=2)
            target_lanelet = current_lanelet
        else:
            print('<reactive_planner> Changed reference lanelet from {} to {}.'.format(current_lanelet.lanelet_id, target_lanelet.lanelet_id))

        distance, end_index = spatial.KDTree(self.reference_paths[current_lanelet.lanelet_id]).query(position)
        distance, start_index = spatial.KDTree(self.reference_paths[target_lanelet.lanelet_id]).query(position)
        tmp = np.array([position])
        temp1 = np.concatenate((self.reference_paths[current_lanelet.lanelet_id][0:end_index], tmp), axis=0)
        reference_path = np.concatenate((temp1, self.reference_paths[target_lanelet.lanelet_id][start_index+5:]), axis=0)
        # smooth reference path until curvature is smaller or equal max_curvature_reference_path
        resampling_step_reference_path = 1.5
        max_curvature_reference_path = 0.1
        max_curvature = max_curvature_reference_path + 0.2
        while max_curvature > max_curvature_reference_path:
            reference_path = np.array(chaikins_corner_cutting(reference_path))
            reference_path = resample_polyline(reference_path, resampling_step_reference_path)
            max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))

        return reference_path, self.reference_paths[current_lanelet.lanelet_id][0:end_index]


if __name__ == '__main__':
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-3_2_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-3_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_4_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_2_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-2_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Peachtree/USA_Peach-4_3_T-1.xml'

    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-22_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-23_1_T-1.xml'
    scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-7_1_T-1.xml'

    # fixed
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-9_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-21_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-27_1_T-1.xml'
#
    # probl
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-1_1_T-1.xml'

    # evtl verlängerung nötig?
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-24_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-6_1_T-1.xml'

    # falscher Wechsel?
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-19_1_T-1.xml'

    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-12_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-29_1_T-1.xml'

    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-15_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-25_1_T-1.xml'

    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-10_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-4_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-20_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-3_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-2_1_T-1.xml'

    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-14_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-17_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-16_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-5_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-8_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-11_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-26_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-18_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-13_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-28_1_T-1.xml'

    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_A99-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_A9-2_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_A9-1_1_T-1.xml'

    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_B471-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-DEU_B471-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_7_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_8_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_2_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_5_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_12_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_7_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_13_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_9_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_15_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_13_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_12_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_6_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_14_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_3_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_8_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_2_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_6_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_11_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_4_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_10_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_9_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_4_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_3_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_10_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_5_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-2_11_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Intersect-1_2_S-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Gar-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Muc-4_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Merge-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Over-1_1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Hhr-1_1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-1_3_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_HW-1_1_S-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Intersect-1_1_S-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Muc-3_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-2_2_S-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Muc-2_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Urban-1_1_S-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-2_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Muc-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Ffb-1_2_S-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-2_3_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-DEU_B471-2_1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-2_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-2_2_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_US101-30_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_US101-32_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-2_4_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_US101-31_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_US101-33_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-1_2_T-1.xml'
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    pr = cProfile.Profile()
    pr.enable()

    route_planner = RoutePlanner(scenario.lanelet_network, scenario_path)
    #lanelet = scenario.lanelet_network.find_lanelet_by_id(18)
    #print("lanelet " + str(len(lanelet.center_vertices[:])))
    #for i in np.arange(0.1, 5.5, 0.1):
    #    #print (i)
    #    reference_path = np.array(chaikins_corner_cutting(lanelet.center_vertices))
    #    reference_path1 = resample_polyline(reference_path, i)
    #    print(str(i) + " Ref " + str(len(reference_path[:]))+ "Ref " + str(len(reference_path1[:])))

    #reference_path, lanelets_leading_to_goal = route_planner.find_reference_path_and_lanelets_leading_to_goal(
    #    allow_overtaking=True,
    #    resampling_step_reference_path=1.5,
    #    max_curvature_reference_path=0.15,
    #    source_position=None)
    plt.figure(figsize=(25, 10))
    draw_object(scenario.lanelet_network, draw_params={'lanelet_network': {'lanelet': {'show_label': True}}})
    draw_object(planning_problem_set)

    reference_path0, lanelets_leading_to_goal0 = route_planner.plan_all_reference_paths(
        resampling_step_reference_path=1.5,
        max_curvature_reference_path=0.1)

    print("Paths")

    #lane = scenario.lanelet_network.find_lanelet_by_id(28)

    part1, part2 = route_planner.set_reference_lane(-1, route_planner.planning_problem.initial_state.position)

    start = time.time()
    #draw_object(scenario.lanelet_network, draw_params={'lanelet_network': {'lanelet': {'show_label': False}}})




    #for path in reference_path0.values():
    #    plt.plot(path[:, 0], path[:, 1], '-*b', linewidth=4, zorder=50)
    plt.plot(part2[:, 0], part2[:, 1], '-b', linewidth=7, zorder=50)
    plt.plot(part1[:, 0], part1[:, 1], '-k', linewidth=3, zorder=50)

    #plt.plot(lane.center_vertices[:, 0], lane.center_vertices[:, 1], '-r', linewidth=7, zorder=50)
    #plt.plot(part2[:, 0], part2[:, 1], '-b', linewidth=7, zorder=50)

    #for id in lanelets_leading_to_goal:
    #    l = scenario.lanelet_network.find_lanelet_by_id(id)
    #    draw_object(l, draw_params={'lanelet': {
    #        'left_bound_color': 'yellow',
    #        'right_bound_color': 'yellow',
    #        'center_bound_color': '#dddddd',
    #        'draw_left_bound': True,
    #        'draw_right_bound': False,
    #        'draw_center_bound': False,
    #        'draw_border_vertices': False,
    #        'draw_start_and_direction': False,
    #        'show_label': False,
    #        'draw_linewidth': 4,
    #        'fill_lanelet': True,
    #        'facecolor': 'yellow',
    #        'zorder': 45}
    #   })



        #plt.plot(reference_path[:, 0], reference_path[:, 1], '-*g', linewidth=4, zorder=50)

    print(time.time() - start)
    pr.disable()
    #pr.print_stats(sort='time')


    plt.axes().autoscale()
    plt.gca().set_aspect('equal')
    plt.axis()
    print('Done')
    plt.show(block=True)