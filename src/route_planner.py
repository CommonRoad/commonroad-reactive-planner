import networkx as nx
import numpy as np
import xml.etree.ElementTree as ET
from commonroad.common.file_reader import CommonRoadFileReader
from polyline import compute_curvature_from_polyline, chaikins_corner_cutting, resample_polyline
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
import matplotlib.pyplot as plt
import warnings
from scipy import spatial


class RoutePlanner:
    def __init__(self, scenario_path):
        """
        Initialize route planner
        :param scenario_path: path to scenario xml file
        """

        self.scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

        # Find planning problem
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
                problem_id = problem.get('id')
                print("Planning Prob ID" + problem_id)
                break
            self.planning_problem = planning_problem_set.find_planning_problem_by_id(int(problem_id))

        # Find goal region
        self.goal_lanelet_position = None
        if hasattr(self.planning_problem.goal.state_list[0], 'position'):
            if self.planning_problem.goal.lanelets_of_goal_position:
                self.goal_lanes = self.planning_problem.goal.lanelets_of_goal_position
                print("Goal lanelets" + str(self.goal_lanes))
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
            self.goal_lanelet_position = goal_lanelet.center_vertices[-2]
        print("goal lanelet ID" + str(self.goal_lanelet_id))

        self.lanelet_network = self.scenario.lanelet_network
        self.graph = self.create_graph_from_lanelet_network()
        self.reference_paths = {}
        for lanelet in self.lanelet_network.lanelets:
            self.reference_paths[lanelet.lanelet_id] = lanelet.center_vertices

        self.plan_all_reference_paths()

    def create_graph_from_lanelet_network_lane_change(self, lanelet_network: LaneletNetwork=None) -> nx.DiGraph():
        """
        Build a graph from the lanelet network. The length of a lanelet is assigned as weight to
            its outgoing edges as in Bender P., Ziegler J., Stiller C., "Lanelets: Efficient Map
            Representation for Autonomous Driving",  IEEE Intelligent Vehicles Symposium, 2014.
            No adjacent lanelets are taken into account, only diagonal lanelets needed for a lane change
        :param lanelet_network: new lanelet network to create a graph from. If None default lanelet network of scenario
                                is taken
        :return: created graph from lanelet network
        """

        if lanelet_network is None:
            lanelet_network = self.lanelet_network
        graph = nx.DiGraph()
        nodes = list()
        edges = list()
        for lanelet in lanelet_network.lanelets:
            nodes.append(lanelet.lanelet_id)
            if lanelet.successor:
                lane = lanelet_network.find_lanelet_by_id(lanelet.successor[0])
                edges.append((lanelet.lanelet_id, lane.lanelet_id, {'weight': lanelet.distance[-1]}))
                if lane.adj_left:
                    edges.append((lanelet.lanelet_id, lane.adj_right,
                                  {'weight': 0, 'same_dir': lanelet.adj_left_same_direction}))
                if lane.adj_right:
                    edges.append((lanelet.lanelet_id, lane.adj_right,
                                  {'weight': 0, 'same_dir': lanelet.adj_right_same_direction}))
            if lanelet.adj_right:
                l_right = lanelet_network.find_lanelet_by_id(lanelet.adj_right)
                if l_right.successor:
                    if (lanelet.lanelet_id, l_right.successor[0],{'weight': 0, 'same_dir': lanelet.adj_right_same_direction}) not in edges:
                        edges.append((lanelet.lanelet_id, l_right.successor[0],
                                    {'weight': 0, 'same_dir': lanelet.adj_right_same_direction}))
            if lanelet.adj_left:
                l_left = lanelet_network.find_lanelet_by_id(lanelet.adj_left)
                if l_left.successor:
                    if (lanelet.lanelet_id, l_left.successor[0],{'weight': 0, 'same_dir': lanelet.adj_left_same_direction}) not in edges:
                        edges.append((lanelet.lanelet_id, l_left.successor[0],
                                    {'weight': 0, 'same_dir': lanelet.adj_left_same_direction}))
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def create_graph_from_lanelet_network(self, lanelet_network=None):
        """
        Build a graph from the lanelet network. The length of a lanelet is assigned as weight to
            its outgoing edges as in Bender P., Ziegler J., Stiller C., "Lanelets: Efficient Map
            Representation for Autonomous Driving",  IEEE Intelligent Vehicles Symposium, 2014.
            The edge weight between adjacent lanelets is set to zero.
        :param lanelet_network: new lanelet network to create a graph from. If None default lanelet network of scenario
                                is taken
        :return: created graph from lanelet network
        """

        if lanelet_network is None:
            lanelet_network = self.lanelet_network
        graph = nx.DiGraph()
        nodes = list()
        edges = list()
        for lanelet in lanelet_network.lanelets:
            nodes.append(lanelet.lanelet_id)
            if lanelet.successor:
                edges.append((lanelet.lanelet_id, lanelet.successor[0], {'weight': lanelet.distance[-1]}))
            if lanelet.adj_left:
                edges.append((lanelet.lanelet_id, lanelet.adj_left,
                              {'weight': 0, 'same_dir': lanelet.adj_left_same_direction}))
            if lanelet.adj_right:
                edges.append((lanelet.lanelet_id, lanelet.adj_right,
                              {'weight': 0, 'same_dir': lanelet.adj_right_same_direction}))
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def create_reference_path_network(self):
        """
        Splits up lanelets adjacent to goal lanelet and creates a new lanelet network
        lanelet network: newly created lanelet network
        lanelet mapping: dict of old lanlet ids mapping to new ids of split lanelets
        :return: lanelet network, lanelet mapping, remove goal lanelets
        """
        lanelet_mapping = {}

        # get goal lanelet and index of nearest center vertice to goal position
        goal_lanelet = self.lanelet_network.find_lanelet_by_id((self.goal_lanelet_id))
        distance, goal_index = spatial.KDTree(goal_lanelet.center_vertices).query(self.goal_lanelet_position)

        # get adjacent lanes that should be split up
        left_lanes, right_lanes = self.get_adjacent_lanelets_list(self.goal_lanelet_id)

        # compute split factor (subtract 1, because source lanelet is in list)
        split_factor = max(len(left_lanes), len(right_lanes)) + 1
        split_factor = max(1, split_factor)

        # initialize array to hold split lanelets
        goal_split_lanelets = []
        remove_goal_lanelets = set()

        if goal_index < (split_factor+1) * 2:
            # goal lanelet part has not enough center vertices -> choose preceding one if possible
            if goal_lanelet.predecessor:

                # get adjactent lanelets of predecessing lanelet
                pred_goal_lanelet = self.lanelet_network.find_lanelet_by_id(goal_lanelet.predecessor[0])

                # we still want to split the goal lanelets into 2
                goal_split_lanelets = (np.concatenate((left_lanes, right_lanes), axis=0)).tolist()
                goal_split_lanelets.append(self.goal_lanelet_id)
                goal_split_lanelets = list(set(goal_split_lanelets))
                goal_split_lanelets = [int(l) for l in goal_split_lanelets]

                # overwrite left_lanes and right_lanes -> later left_lanes and right_lanes of predecessor
                # will be treated same as if they were left and right lanes of goal lanelet
                left_lanes, right_lanes = self.get_adjacent_lanelets_list(pred_goal_lanelet.lanelet_id)

                # compute new split factor for predecessing lanelet
                # (subtract 2 because there is adjacent goal lanelets will be split into 2)
                split_factor = max(len(left_lanes), len(right_lanes)) - 2
                split_factor = max(1, split_factor)

                # save lanelets we want to remove from split lanelets
                remove_goal_lanelets = set(goal_split_lanelets)
                set2 = {self.goal_lanelet_id, goal_lanelet.adj_left, goal_lanelet.adj_right}
                remove_goal_lanelets.difference(set2)

        # get all lanelets we want to split up 'normally'
        split_lanelets = (np.concatenate((left_lanes, right_lanes), axis=0)).tolist()
        split_lanelets.append(goal_lanelet.lanelet_id)
        split_lanelets = list(set(split_lanelets))
        split_lanelets = [int(l) for l in split_lanelets]

        # we need unique lanlet IDs for new lanelets
        # just generate list with ascending numbers and remove already existing ones
        map_ids = list(range(0, (split_factor + 2) * len(self.lanelet_network.lanelets)))
        for lanelet in self.lanelet_network.lanelets:
            if lanelet.lanelet_id in map_ids:
                # id already exists -> remove from list
                map_ids.remove(lanelet.lanelet_id)

        # map existing IDs to IDs of new lanelets
        num = 0
        # Increase number in array with unique IDs according to split factor
        for lanelet in split_lanelets:
            lanelet_mapping[lanelet] = map_ids[num: num + split_factor + 1]
            num += split_factor + 1

        # if we have values in goal_split_lanes, adjacent goal lanes are only split into 2 parts
        # the predecessing lanelets are split into the rest (split_lanes)
        for lanelet in goal_split_lanelets:
            lanelet_mapping[lanelet] = map_ids[num: num + 2]
            num += 2

        # get other lanelets
        current_lanelets = (np.concatenate((left_lanes, right_lanes), axis=0)).tolist()
        other_lanelets = self.convert_old_lanelets(split_lanelets=current_lanelets, lanelet_mapping=lanelet_mapping)

        # split lanelets up
        created_lanelets, end_id, lanelet_mapping= self.create_split_lanelets(split_lanelets, split_factor, lanelet_mapping, map_ids)
        # append all not split lanelets
        all_lanelets = (np.concatenate((created_lanelets, other_lanelets), axis=0)).tolist()

        # if we have goal_split_lanelets split them up into 2 parts
        goal_created_lanelets = []
        if goal_split_lanelets != []:
            # split_factor = 2
            goal_created_lanelets, end_id, lanelet_mapping = self.create_split_lanelets(lanelets=goal_split_lanelets, split_factor=1,
                                                                       lanelet_mapping=lanelet_mapping, map_ids=map_ids,
                                                                       start_id=end_id)
        # add them to the rest
        all_lanelets = (np.concatenate((all_lanelets, goal_created_lanelets), axis=0)).tolist()

        # create new lanelet network for graph
        new_network = LaneletNetwork()
        new_network = new_network.create_from_lanelet_list(all_lanelets)

        return new_network, lanelet_mapping, remove_goal_lanelets

    def convert_old_lanelets(self, split_lanelets, lanelet_mapping: dict):
        """
        Converts lanelet information (successor, predecessor, adj_left, adj_right) according to lanelet_mapping
        information from old lanelet IDs to new lanelet IDs
        :param split_lanelets: lanelets that are going to be split
        :param lanelet_mapping: dict mapping old lanelet IDs to new IDs of new split lanelets
        :return:
        """

        other_lanelets= []
        # get all other lanelets (not split up) we need to also save them in new lanelet network
        for lanelet in self.lanelet_network.lanelets:
            if lanelet.lanelet_id in split_lanelets:
                continue
            else:
                # check in lanelet mapping if we need to change existing adj_left, adj_right, predecessor or successor
                # values according to new lanelet IDs
                temp_pred = []
                temp_suc = []
                temp_r = None
                temp_l = None
                new = False
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
                    # take new lanelet IDs into account from lanelet mapping
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
                    # nothing changes -> just save lanelet as it is
                    other_lanelets.append(lanelet)
        return other_lanelets

    def create_split_lanelets(self, lanelets, split_factor, lanelet_mapping, map_ids, start_id=0):
        """
        Split lanelets up into split_factor x sublanelets
        :param lanelets: lanelets to be split up
        :param split_factor: in how many sublanelets lanelets need to be split up
        :param lanelet_mapping: mapping of IDs of current lanelet to IDs of new split lanelets
        :param map_ids: list with all new lanelet IDs
        :param start_id: index to start from in map_ids to assign new IDs
        :return: created lanelets, index of last assigned element in map_ids, lanelet_mapping
        """
        created_lanelets = []
        lanes = []
        id = start_id
        # split each lanelet
        for lanelet_id in lanelets:
            lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
            dis, lane_index = spatial.KDTree(lanelet.center_vertices).query(self.goal_lanelet_position)
            lanelet_length = len(lanelet.center_vertices[0:lane_index + 2])

            # array with indices to split current lanelet up
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
            # convert for valid initialization for lanelet
            if pred_list == []:
                pred_list = None

            # create from split up center vertices new lanelets
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
        :return: list of left_lanelets, list of right lanelets
                 empty lists if there are none
        """
        list_left = []
        list_right = []
        left = True
        right = True
        temp_id = lanelet_id
        while left:
            current_lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
            # only append lanelets from the same direction
            if current_lanelet.adj_left_same_direction is True:
                list_left.append(current_lanelet.adj_left)
                lanelet_id = current_lanelet.adj_left
            else:
                left = False
        # reset lanelet_id for right while loop
        lanelet_id = temp_id
        while right:
            current_lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
            # only append lanelets from the same direction
            if current_lanelet.adj_right_same_direction is True:
                list_right.append(current_lanelet.adj_right)
                lanelet_id = current_lanelet.adj_right
            else:
                right = False
        return list_left, list_right

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
            source_lanelet = lanelet_network.find_lanelet_by_id(source_lanelet_id)
            return source_lanelet.center_vertices
        shortest_path = shortest_paths[0]
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

        adjacent_left, adjacent_right = self.get_adjacent_lanelets_list(source_lanelet.lanelet_id)
        adjacent = (np.concatenate((adjacent_left, adjacent_right), axis=0)).tolist()

        flag = False
        if source_lanelet.successor:
            suc = self.lanelet_network.find_lanelet_by_id(source_lanelet.successor[0])
            if suc.adj_left_same_direction is True or suc.adj_right_same_direction is True:
                flag = True

        from_source = set()
        # if we have highway scenarios, we need also to consider all lanelets reachable from source
        if adjacent != [] or flag is True:
            from_source = nx.descendants(self.graph, source_lanelet.lanelet_id)

        adjacent.append(source_lanelet.lanelet_id)
        adjacent = list(set(adjacent).union(from_source))
        adjacent = [int(l) for l in adjacent]
        lanelets_leading_to_goal = set()
        lanelets_leading_to_goal.add(self.goal_lanelet_id)
        for lanelet in adjacent:
            if lanelet not in lanelets_leading_to_goal:
                _lanelets = self.find_all_lanelets_leading_to_goal(lanelet, allow_overtaking=False)
                lanelets_leading_to_goal |= _lanelets

        lanelets_leading_to_goal = list(set(lanelets_leading_to_goal))

        new_network, lanelet_mapping, not_leading_to_goal = self.create_reference_path_network()
        graph = self.create_graph_from_lanelet_network_lane_change(lanelet_network=new_network)

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

    def set_reference_lane(self, lane_direction: int, position):
        """
        compute new reference path based on relative lane position.
        :param lane_direction: 0=curernt lane >0=right lanes, <0=left lanes
        :param position: current position
        :return: new reference lane vertices
        """
        assert self.lanelet_network is not None,\
            'lanelet network must be provided during initialization for using get_reference_of_lane().'

        # get current lanelet
        current_ids = self.lanelet_network.find_lanelet_by_position(np.array([position]))[0]
        if len(current_ids) > 0:
            current_lanelet = self.lanelet_network.find_lanelet_by_id(current_ids[0])
        else:
            raise ValueError('set_reference_lane: x0 is not located on any lane and no previous reference available.')

        # determine target lane
        target_lanelet = None
        if lane_direction == -1:
            if current_lanelet.adj_left_same_direction not in (False, None):
                target_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet.adj_left)
        elif lane_direction == 1:
            if current_lanelet.adj_right_same_direction not in (False, None):
                target_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet.adj_right)
        elif lane_direction == 0:
            target_lanelet = current_lanelet

        # inform user if lane change is possible
        if target_lanelet is None:
            warnings.warn('set_reference_lane: No adjacent lane in direction {}, stay in current lane.'.format(lane_direction), stacklevel=2)
            target_lanelet = current_lanelet
        else:
            print('<reactive_planner> Changed reference lanelet from {} to {}.'.format(current_lanelet.lanelet_id, target_lanelet.lanelet_id))

        # get indices where to cut new and old reference lanelets center vertices
        distance, end_index = spatial.KDTree(self.reference_paths[current_lanelet.lanelet_id]).query(position)
        distance, start_index = spatial.KDTree(self.reference_paths[target_lanelet.lanelet_id]).query(position)

        # concatenate new and old reference paths
        tmp = np.array([position])
        temp1 = np.concatenate((self.reference_paths[current_lanelet.lanelet_id][0:end_index], tmp), axis=0)
        reference_path = np.concatenate((temp1, self.reference_paths[target_lanelet.lanelet_id][start_index+3:]), axis=0)

        # smooth reference path until curvature is smaller or equal max_curvature_reference_path
        resampling_step_reference_path = 1.5
        max_curvature_reference_path = 0.1
        max_curvature = max_curvature_reference_path + 0.2
        while max_curvature > max_curvature_reference_path:
            reference_path = np.array(chaikins_corner_cutting(reference_path))
            reference_path = resample_polyline(reference_path, resampling_step_reference_path)
            max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))

        return reference_path


# Example: usage of route planner
if __name__ == '__main__':
    scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/US101/USA_US101-23_1_T-1.xml'
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    plt.figure(figsize=(25, 10))
    draw_object(scenario.lanelet_network, draw_params={'lanelet_network': {'lanelet': {'show_label': True}}})
    draw_object(planning_problem_set)

    # initialize route planner
    route_planner = RoutePlanner(scenario_path)

    # reference path network generation
    reference_paths, lanelets_leading_to_goal = route_planner.plan_all_reference_paths(
        resampling_step_reference_path=2,
        max_curvature_reference_path=0.1)

    # plot reference path
    for path in reference_paths.values():
        plt.plot(path[:, 0], path[:, 1], '-*b', linewidth=4, zorder=50)
    # plot all lanelets leading to goal
    for id in lanelets_leading_to_goal:
        lane = scenario.lanelet_network.find_lanelet_by_id(id)
        draw_object(lane, draw_params={'lanelet': {
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
            'zorder': 45}})

    # lane change
    lane_change = route_planner.set_reference_lane(-1, route_planner.planning_problem.initial_state.position)
    # plot new reference lane after lane change
    plt.plot(lane_change[:, 0], lane_change[:, 1], '-*k', linewidth=4, zorder=50)

    plt.axes().autoscale()
    plt.gca().set_aspect('equal')
    plt.axis()
    print('Done')
    plt.show(block=True)