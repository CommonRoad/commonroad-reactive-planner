import networkx as nx
import numpy as np
import xml.etree.ElementTree as ET
from commonroad.common.file_reader import CommonRoadFileReader
from polyline import compute_curvature_from_polyline, chaikins_corner_cutting, resample_polyline
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
import matplotlib.pyplot as plt
import cProfile
import time
from scipy import spatial
import math


# TODO:
#  c) evtl zukünftig: ob rechts/links lane change?
#  d) über verschiedene Ziel lanelets iterieren
#  f) evtl zukünftig über verschiedene Faktoren iterieren, falls kein Pfad gefunden?
#  g) fix successor/predecessor for revert_lane
# DONE:
# b) Problem lane change mit Anfang ist Ziel Lanelet -> kein Pfad gefunden!
# Lösung: Pfad zu rechter/linker Linie festlegen
# a) start lanelet besser festlegen -> in find_reference_path nicht für adj_left/right ersetzen lassen
# e) goal state: left/right vertices definieren?! -> nicht mehr gebraucht, da goal anderweitig gefunden!

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

        print("goal lanelet ID" + str(self.goal_lanelet_id))

        self.lanelet_network = self.scenario.lanelet_network
        self.graph = self.create_graph_from_lanelet_network()

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

    def get_adjacent_lanelets(self, lanelet_id, right: bool = True, left: bool = True):
        """
        Recursively gets adj_left and adj_right lanelets of current lanelet id
        :param lanelet: current lanelet id
        :return: List of adjacent lanelets, empty list if there are none
        """
        adj_list = []
        current_lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
        if (current_lanelet.adj_left is not None)  and (left is True):
            adj_list.append(current_lanelet.adj_left)
            if current_lanelet.adj_left_same_direction:
                temp_left = self.get_adjacent_lanelets(current_lanelet.adj_left, right=False, left=True)
                if temp_left != []:
                    for elem in temp_left:
                        adj_list.append(elem)
                    return adj_list
            else:
                temp_left = self.get_adjacent_lanelets(current_lanelet.adj_left, right=True, left=False)
                if temp_left != []:
                    for elem in temp_left:
                        adj_list.append(elem)
                    return adj_list
        if (current_lanelet.adj_right is not None) and (right is True):
            adj_list.append(current_lanelet.adj_right)
            if current_lanelet.adj_right_same_direction:
                temp_right = self.get_adjacent_lanelets(current_lanelet.adj_right, right=True, left=False)
                if temp_right != []:
                    for elem in temp_right:
                        adj_list.append(elem)
                    return adj_list
            else:
                temp_right = self.get_adjacent_lanelets(current_lanelet.adj_right, right=False, left=True)
                if temp_right != []:
                    for elem in temp_right:
                        adj_list.append(elem)
                    return adj_list
        return adj_list

    def discrete_network_lane_change(self, factor: int=1, lanelet_list: list=None, current_pos=None,
                                     current_id: int = -1, allow_overtaking: bool = False):
        """
        Create a new discrete lanelet network from current scenario network to enable lane changing
        :param factor: how many new lanelets to split current lanelets
        :param lanelet_list: list of lanelets to split (lanelets leading to goal)
        :param current_pos: current vehicle position
        :param current_id: current lanelet id of vehicle position
        :return: tuple: (new_lanelet_network, start, goal)
        """
        created_lanelets=[]
        lanelet_mapping = {}
        id = 0

        if current_pos is None:
            current_pos = self.planning_problem.initial_state.position

        # map new discrete lanelets to scenario lanelets
        num = 0
        for lanelet in lanelet_list:
            if lanelet.center_vertices.shape[0]/factor <= 1:
                # factor is too big, we need min 2 center vertices per lanelet
                # TODO trigger new factor
                factor = max(1, math.ceil(lanelet.center_vertices.shape[0] / 2))
                break

        # get all adjacent lanelets
        adjacent_lanelets = self.get_adjacent_lanelets(current_id)

        # calculation how to split current lane and adjacent lanes (should all have same length)
        current_lanelet = self.lanelet_network.find_lanelet_by_id(current_id)

        # if we allow overtaking, check for all lanelets to the left, if they drive in same direction
        # TODO: also consider predecessors/ successors?!
        revert_direction = []
        if allow_overtaking:
            if current_lanelet.adj_left:
                if not current_lanelet.adj_left_same_direction:
                    revert_direction.append(current_lanelet.adj_left)

        lanelet_ids = []
        for lanelet in lanelet_list:
            lanelet_ids.append(lanelet.lanelet_id)

        # only consider adjacent lanlets that are in lanelet_list
        adjacent_lanelets = set(adjacent_lanelets).intersection(set(lanelet_ids))

        # create dict to map new to current lanelet ids
        for lanelet in lanelet_list:
            # special case: we only save fist part of current lanelet
            if lanelet.lanelet_id == current_id:
                lanelet_mapping[lanelet.lanelet_id] = list(range(num, num + 1))
                # save new start lanelet id
                start = num
                num += 1
            # special case: for lane change we split adj lanelets in min 2 parts
            elif factor == 1 and lanelet.lanelet_id in adjacent_lanelets:
                lanelet_mapping[lanelet.lanelet_id] = list(range(num, num + 2))
                num += 2
            else:
                lanelet_mapping[lanelet.lanelet_id] = list(range(num, num + factor))
                num += factor

        # compute start position: nearest center vertice to current position
        distance, start_index = spatial.KDTree(current_lanelet.center_vertices).query(current_pos)
        offset = len(current_lanelet.center_vertices[start_index:]) % factor
        split = int(len(current_lanelet.center_vertices[start_index:]) / factor)
        if offset == 1 and split == 1:
            # len < 2 -> just replan without splitting, so return original network
            return self.lanelet_network
        elif factor == 1:
            split_array = np.arange(2, split, split)
        else:
            #split_array = np.arange(split, factor*split, split)
            split_array = np.arange(2, (factor-1) * split, split)
        lanes = []
        # split each lanelet
        for lanelet in lanelet_list:
            lanes.clear()
            if lanelet.lanelet_id == current_id:
                # only add fist lanelet part
                # TODO: check if index + split out of bound!
                lanes.append(lanelet.center_vertices[start_index: start_index + 2])
            elif lanelet.lanelet_id in adjacent_lanelets:
                if lanelet.lanelet_id in revert_direction:
                    temp_l = list(lanelet.center_vertices[start_index:])
                    temp_l.reverse()
                    lanes = np.split(temp_l, split_array)
                else:
                    temp_l = np.array(lanelet.center_vertices[start_index:])
                    lanes = np.split(temp_l, split_array)
            else:
                lanelet_offset = len(lanelet.center_vertices[:]) % factor
                lanelet_split = int(len(lanelet.center_vertices[:]) / factor)
                # we only split the other lanes, if factor >= 2
                if factor == 1 or (lanelet_offset == 1 and lanelet_split == 1):
                    # we just add complete lane
                    lanes.append(lanelet.center_vertices)
                elif lanelet_offset == 0:
                    # TODO FIX split array
                    split_array = np.full((1, factor), lanelet_split)
                else:
                    # TODO FIX split array
                    split_array = np.full((1, factor - 1), split)
                    list(split_array).append(split + offset)
                    lanes = np.split(np.array(lanelet.center_vertices), split_array)

            # get predecessor for first new lanelet
            p_list = lanelet.predecessor
            pred_list = []
            for pred in p_list:
                if pred in lanelet_mapping:
                    for p in lanelet_mapping[pred]:
                        pred_list.append(p)
            if pred_list == []:
                pred_list = None

            for i in range(len(lanes)):
                # get successor from lanelet_mapping
                s_list = lanelet.successor
                suc_list = []
                if i < (factor-1) and (lanelet.lanelet_id != current_id):
                    suc_list.append(id + 1)
                elif i == 0 and factor == 1 and lanelet.lanelet_id in adjacent_lanelets:
                    suc_list.append(id+1)
                elif i == (factor - 1):
                    for suc in s_list:
                        if suc in lanelet_mapping:
                            for s in lanelet_mapping[suc]:
                                suc_list.append(s)
                else:
                    suc_list = None
                # if suc_list is empty set it to None, otherwise problems with new lanelet
                if suc_list == []:
                    suc_list = None

                # get left and right adjacent from lanelet_mapping (attention: (not) same direction)
                temp_right = None
                temp_left = None
                if lanelet.adj_right:
                    if lanelet.adj_right in lanelet_mapping:
                        right_lanelets = lanelet_mapping[lanelet.adj_right]
                        if lanelet.adj_right_same_direction:
                            if lanelet.adj_right == current_id and i == 0:
                                temp_right = right_lanelets[0]
                            elif lanelet.adj_right != current_id:
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
                            if lanelet.adj_left == current_id and i == 0:
                                temp_left = left_lanelets[0]
                            elif lanelet.adj_left != current_id:
                                temp_left = left_lanelets[i]
                        elif lanelet.adj_left in revert_direction:
                            if lanelet.adj_left == current_id:
                                temp_left = left_lanelets[0]
                            elif lanelet.adj_left != current_id:
                                temp_left = left_lanelets[i]
                        else:
                            if len(left_lanelets) == 1:
                                temp_left = left_lanelets[0]
                            else:
                                temp_left = left_lanelets[-1 - i]

                # create new lanelet: set successor, adjacent_left, adjacent_right are set according to lanelet_mapping
                # in the graph generation we don't care about left and right_vertices: just assign center_vertices
                if lanelet.adj_left in revert_direction:
                    alsd = True
                else:
                    alsd = lanelet.adj_left_same_direction

                new_lanelet = Lanelet(left_vertices=lanes[i], center_vertices=lanes[i], right_vertices=lanes[i],
                 lanelet_id=id,
                 predecessor=pred_list, successor=suc_list,
                 adjacent_left=temp_left, adjacent_left_same_direction=alsd,
                 adjacent_right=temp_right, adjacent_right_same_direction=lanelet.adj_right_same_direction,
                 speed_limit=lanelet.speed_limit)
                id += 1
                # store new lanelet
                created_lanelets.append(new_lanelet)
                # reset pred_list for next lanelet
                pred_list = []
                # append current lanelet as pred for next lanelet
                pred_list.append(new_lanelet.lanelet_id)

        # create new lanelet network for graph
        new_network= LaneletNetwork()
        new_network = new_network.create_from_lanelet_list(created_lanelets)

        if self.goal_lanelet_position is not None and self.goal_lanelet_id != current_id:
            goal_lanelet = self.lanelet_network.find_lanelet_by_id(self.goal_lanelet_id)
            distance, goal_index = spatial.KDTree(goal_lanelet.center_vertices).query(self.goal_lanelet_position)
            split = int(len(goal_lanelet.center_vertices[:]) / factor)
            goal = lanelet_mapping[self.goal_lanelet_id][int(goal_index / split)]
        elif self.goal_lanelet_id == current_id:
            if current_lanelet.adj_left:
                if current_lanelet.adj_left_same_direction:
                    goal = lanelet_mapping[current_lanelet.adj_left][-1]
                elif current_lanelet.adj_left in revert_direction:
                    goal = lanelet_mapping[current_lanelet.adj_left][-1]
                elif current_lanelet.adj_left not in lanelet_mapping:
                    goal = lanelet_mapping[current_id][0]
                else:
                    goal = lanelet_mapping[current_lanelet.adj_left][0]
            elif current_lanelet.adj_right:
                if current_lanelet.adj_right_same_direction:
                    goal = lanelet_mapping[current_lanelet.adj_right][-1]
                else:
                    goal = lanelet_mapping[current_lanelet.adj_right][0]
            else:
                goal = lanelet_mapping[current_id][0]
        else:
            # get new goal lanelet id (last element of split goal lanelet)
            goal = lanelet_mapping[self.goal_lanelet_id][-1]

        return new_network, start, goal

    def plan_with_discrete_network(self, factor: int=2, lanelet_list: list=None, current_pos=None,
                                     current_id: int = -1, allow_overtaking: bool = False):
        """
        Create a new discrete lanelet network from current scenario network to enable lane changing
        :param factor: how many new lanelets to split current lanelets
        :param lanelet_list: list of lanelets to split (lanelets leading to goal)
        :param current_pos: current vehicle position
        :param current_id: current lanelet id of vehicle position
        :return: tuple: (new_lanelet_network, start, goal)
        """
        created_lanelets=[]
        lanelet_mapping = {}
        id = 0

        if current_pos is None:
            current_pos = self.planning_problem.initial_state.position

        # map new discrete lanelets to scenario lanelets
        num = 0
        for lanelet in lanelet_list:
            if lanelet.center_vertices.shape[0]/factor <= 1:
                # factor is too big, we need min 2 center vertices per lanelet
                # TODO trigger new factor
                factor = max(1, math.ceil(lanelet.center_vertices.shape[0] / 2))
                break

        # calculation how to split current lane and adjacent lanes (should all have same length)
        current_lanelet = self.lanelet_network.find_lanelet_by_id(current_id)

        lanelet_ids = []
        for lanelet in lanelet_list:
            lanelet_ids.append(lanelet.lanelet_id)

        # create dict to map new to current lanelet ids
        for lanelet in lanelet_list:
            # special case: we only save fist part of current lanelet
            if lanelet.lanelet_id == current_id:
                # save new start lanelet id
                start = num
            lanelet_mapping[lanelet.lanelet_id] = list(range(num, num + factor))
            num += factor

        lanes = []
        # split each lanelet
        for lanelet in lanelet_list:
            lanes.clear()
            lanelet_offset = len(lanelet.center_vertices[:]) % factor
            lanelet_split = int(len(lanelet.center_vertices[:]) / factor)
            split_array = np.arange(lanelet_split, factor * lanelet_split, lanelet_split)
            # we only split the other lanes, if factor >= 2
            if (lanelet_offset == 1 and lanelet_split == 1):
                # we just add complete lane
                lanes.append(lanelet.center_vertices)
            else:
                lanes = np.split(np.array(lanelet.center_vertices), split_array)

            # get predecessor for first new lanelet
            p_list = lanelet.predecessor
            pred_list = []
            for pred in p_list:
                if pred in lanelet_mapping:
                    for p in lanelet_mapping[pred]:
                        pred_list.append(p)
            if pred_list == []:
                pred_list = None

            for i in range(len(lanes)):
                # get successor from lanelet_mapping
                s_list = lanelet.successor
                suc_list = []
                if i < (factor-1) and (lanelet.lanelet_id != current_id):
                    suc_list.append(id + 1)
                elif i == (factor - 1):
                    for suc in s_list:
                        if suc in lanelet_mapping:
                            for s in lanelet_mapping[suc]:
                                suc_list.append(s)
                else:
                    suc_list = None
                # if suc_list is empty set it to None, otherwise problems with new lanelet
                if suc_list == []:
                    suc_list = None

                # get left and right adjacent from lanelet_mapping (attention: (not) same direction)
                temp_right = None
                temp_left = None
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
                 lanelet_id=id,
                 predecessor=pred_list, successor=suc_list,
                 adjacent_left=temp_left, adjacent_left_same_direction=lanelet.adj_left_same_direction,
                 adjacent_right=temp_right, adjacent_right_same_direction=lanelet.adj_right_same_direction,
                 speed_limit=lanelet.speed_limit)
                id += 1
                # store new lanelet
                created_lanelets.append(new_lanelet)
                # reset pred_list for next lanelet
                pred_list = []
                # append current lanelet as pred for next lanelet
                pred_list.append(new_lanelet.lanelet_id)

        # create new lanelet network for graph
        new_network= LaneletNetwork()
        new_network = new_network.create_from_lanelet_list(created_lanelets)

        if self.goal_lanelet_position is not None and self.goal_lanelet_id != current_id:
            goal_lanelet = self.lanelet_network.find_lanelet_by_id(self.goal_lanelet_id)
            distance, goal_index = spatial.KDTree(goal_lanelet.center_vertices).query(self.goal_lanelet_position)
            split = int(len(goal_lanelet.center_vertices[:])/factor)
            goal = lanelet_mapping[self.goal_lanelet_id][int(goal_index/split)]
        elif self.goal_lanelet_id == current_id:
            if current_lanelet.adj_left:
                if current_lanelet.adj_left_same_direction:
                    goal = lanelet_mapping[current_lanelet.adj_left][-1]
                elif current_lanelet.adj_left not in lanelet_mapping:
                    goal = lanelet_mapping[current_id][0]
                else:
                    goal = lanelet_mapping[current_lanelet.adj_left][0]
            elif current_lanelet.adj_right:
                if current_lanelet.adj_right_same_direction:
                    goal = lanelet_mapping[current_lanelet.adj_right][-1]
                else:
                    goal = lanelet_mapping[current_lanelet.adj_right][0]
            else:
                goal = lanelet_mapping[current_id][0]
        else:
            # get new goal lanelet id (last element of split goal lanelet)
            goal = lanelet_mapping[self.goal_lanelet_id][-1]

        return new_network, start, goal

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

        reference_lanelets = [lanelet_network.find_lanelet_by_id(shortest_path[0])]
        for i, id in enumerate(shortest_path[1:]):
            lanelet = lanelet_network.find_lanelet_by_id(id)
            preceding_lanelet = lanelet_network.find_lanelet_by_id(shortest_path[i])

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
        #lanelets_leading_to_goal = self.find_all_lanelets_leading_to_goal(
        #    start_lanelet.lanelet_id, allow_overtaking)

        # smooth reference path until curvature is smaller or equal max_curvature_reference_path
        max_curvature = max_curvature_reference_path + 0.2
        while max_curvature > max_curvature_reference_path:
            reference_path = np.array(chaikins_corner_cutting(reference_path))
            reference_path = resample_polyline(reference_path, resampling_step_reference_path)
            max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))
        #return reference_path, lanelets_leading_to_goal
        return reference_path, None

    def change_lanelet(self, allow_overtaking: bool = True,
                       source_position=None,
                       resampling_step_reference_path: float = 1.5,
                       max_curvature_reference_path: float = 0.2,
                       factor: int = 1):
        """
        Finds a new reference path to perform a lane change
        :param allow_overtaking:True if overtaking on left opposite lanelets is allowed
        :param source_position: np.array[x,y]: current vehicle position
        :param resampling_step_reference_path: resampling step
        :param max_curvature_reference_path: max curvature of reference path
        :param factor: (optional) in how many parts lanelet should be split in
        :return:
        """

        if source_position is None:
            source_position = self.planning_problem.initial_state.position
        sourcelanelets = self.lanelet_network.find_lanelet_by_position(np.array([source_position]))
        source_lanelet = self.lanelet_network.find_lanelet_by_id(sourcelanelets[0][0])

        laneletids_leading_to_goal = self.find_all_lanelets_leading_to_goal(
            source_lanelet.lanelet_id, allow_overtaking)
        lanelets_leading_to_goal = []
        for id in laneletids_leading_to_goal:
            lanelets_leading_to_goal.append(self.lanelet_network.find_lanelet_by_id(id))

        if lanelets_leading_to_goal == []:
            lanelets_leading_to_goal = self.lanelet_network.lanelets

        new_network, start, goal = self.discrete_network_lane_change(lanelet_list= lanelets_leading_to_goal,
                                                                     current_id=source_lanelet.lanelet_id,
                                                                     factor=factor,
                                                                     current_pos=source_position,
                                                                     allow_overtaking=allow_overtaking)
        graph = self.create_graph_from_lanelet_network(lanelet_network=new_network)

        reference_path = self.find_reference_path_to_goal(
            start, goal, lanelet_network= new_network, graph= graph)
        if reference_path is None:
            reference_path = source_lanelet.center_vertices

        # smooth reference path until curvature is smaller or equal max_curvature_reference_path
        max_curvature = max_curvature_reference_path + 0.2
        while max_curvature > max_curvature_reference_path:
            reference_path = np.array(chaikins_corner_cutting(reference_path))
            reference_path = resample_polyline(reference_path, resampling_step_reference_path)
            max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))
        return reference_path, lanelets_leading_to_goal

    def plan_discrete(self, allow_overtaking: bool = True,
                       source_position=None,
                       resampling_step_reference_path: float = 1.5,
                       max_curvature_reference_path: float = 0.2,
                       factor: int = 1):
        """
        Finds a new reference path to perform a lane change
        :param allow_overtaking:True if overtaking on left opposite lanelets is allowed
        :param source_position: np.array[x,y]: current vehicle position
        :param resampling_step_reference_path: resampling step
        :param max_curvature_reference_path: max curvature of reference path
        :param factor: (optional) in how many parts lanelet should be split in
        :return:
        """

        if source_position is None:
            source_position = self.planning_problem.initial_state.position
        sourcelanelets = self.lanelet_network.find_lanelet_by_position(np.array([source_position]))
        source_lanelet = self.lanelet_network.find_lanelet_by_id(sourcelanelets[0][0])

        laneletids_leading_to_goal = self.find_all_lanelets_leading_to_goal(
            source_lanelet.lanelet_id, allow_overtaking)
        lanelets_leading_to_goal = []
        for id in laneletids_leading_to_goal:
            lanelets_leading_to_goal.append(self.lanelet_network.find_lanelet_by_id(id))

        if lanelets_leading_to_goal == []:
            lanelets_leading_to_goal = self.lanelet_network.lanelets

        new_network, start, goal = self.plan_with_discrete_network(lanelet_list=lanelets_leading_to_goal,
                                                                     current_id=source_lanelet.lanelet_id,
                                                                     factor=factor,
                                                                     current_pos=source_position,
                                                                     allow_overtaking=allow_overtaking)
        graph = self.create_graph_from_lanelet_network(lanelet_network=new_network)

        reference_path = self.find_reference_path_to_goal(
            start, goal, lanelet_network= new_network, graph= graph)
        if reference_path is None:
            reference_path = source_lanelet.center_vertices

        # smooth reference path until curvature is smaller or equal max_curvature_reference_path
        max_curvature = max_curvature_reference_path + 0.2
        while max_curvature > max_curvature_reference_path:
            reference_path = np.array(chaikins_corner_cutting(reference_path))
            reference_path = resample_polyline(reference_path, resampling_step_reference_path)
            max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))
        return reference_path, lanelets_leading_to_goal

if __name__ == '__main__':
    scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/NGSIM/Lankershim/USA_Lanker-1_2_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/cooperative/C-USA_Lanker-2_1_T-1.xml'
    scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/ZAM_Urban-1_1_S-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_Gar-1_1_T-1.xml'
    #scenario_path = '/home/friederike/Masterpraktikum/Commonroad/commonroad-scenarios/hand-crafted/DEU_A9-2_1_T-1.xml'
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    pr = cProfile.Profile()
    pr.enable()

    route_planner = RoutePlanner(scenario.lanelet_network, scenario_path)
    reference_path, lanelets_leading_to_goal = route_planner.find_reference_path_and_lanelets_leading_to_goal(
        allow_overtaking=False,
        resampling_step_reference_path=1.5,
        max_curvature_reference_path=0.15,
        source_position=None)
    path_change2, lanelets2 = route_planner.change_lanelet(
        allow_overtaking=True,
        resampling_step_reference_path=1.5,
        max_curvature_reference_path=0.1,
        source_position=None,
        factor=1)
    start = time.time()
    draw_object(scenario.lanelet_network, draw_params={'lanelet_network': {'lanelet': {'show_label': False}}})
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
        #plt.plot(path_change[:,0], path_change[:,1], '-*r', linewidth=4, zorder=50)
        plt.plot(path_change2[:,0], path_change2[:,1], '-*b', linewidth=4, zorder=50)

    print(time.time() - start)
    pr.disable()
    pr.print_stats(sort='time')


    plt.axes().autoscale()
    plt.gca().set_aspect('equal')
    plt.axis()
    plt.show(block=True)