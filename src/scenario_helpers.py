import construction
import triangle_builder
import pycrcc
from pycrcc import *
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
import numpy as np
import commonroad.geometry.shape as shp
from commonroad.scenario.trajectory import State
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from commonroad_rp.utils import compute_pathlength_from_polyline
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import Lanelet

#import spot
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_object

def create_road_boundary(scenario: Scenario, draw=False) -> StaticObstacle:
    #method =('triangulation', {'call_options': 'a100q'}) # 'shell' # 'triangulation'
    method = 'triangulation' # 'shell' # 'triangulation'
    #build = ['section_triangles','triangulation']
    build = ['section_triangles', ('triangulation', {'max_area': '1'})]
    if draw:
        draw = ['triangulation']
        boundary = construction.construct(scenario, build, draw)
    else:
        boundary = construction.construct(scenario, build, [], [])

    initial_state = State(position=np.array([0, 0]), orientation=0.0, time_step=0)

    road_boundary_shape_list = list()

    for r in boundary[method].unpack():
        p = shp.Polygon(np.array(r.vertices()))
        road_boundary_shape_list.append(p)
    road_boundary_obstacle = StaticObstacle(obstacle_id=scenario.generate_object_id(), obstacle_type=ObstacleType.ROAD_BOUNDARY,
                                   obstacle_shape=shp.ShapeGroup(road_boundary_shape_list), initial_state=initial_state)
    return boundary[method], road_boundary_obstacle

def lanelet_rep_setup(lanelets):
    lanelet_rep = []
    for lanelet in lanelets:
        triangles = pycrcc.ShapeGroup()
        triangle_builder.build_simple_triangles([lanelet], triangles)
        polyline = np.concatenate((lanelet.left_vertices, np.flipud(lanelet.right_vertices)))
        polygon = Polygon(polyline)
        lanelet_rep.append((polygon, triangles))
    return lanelet_rep

def obtain_reference_path(state: State, scenario: Scenario) -> np.ndarray:
    # create coordinate system
    ego_lanelet_id = scenario.lanelet_network.find_lanelet_by_position([state.position])[0][0]
    print('Ego vehice is located in lanelet id={}'.format(ego_lanelet_id))
    ego_lanelet = scenario.lanelet_network.find_lanelet_by_id(ego_lanelet_id)
    reference_path = ego_lanelet.center_vertices
    # check if reference path is long enough
    length = compute_pathlength_from_polyline(reference_path)[-1]
    new_lanelet = ego_lanelet
    temp = ego_lanelet
    while length < 30 and len(temp.successor):
        temp = scenario.lanelet_network.find_lanelet_by_id(temp.successor[0])
        new_lanelet = Lanelet.merge_lanelets(new_lanelet,temp)
        length = compute_pathlength_from_polyline(new_lanelet.center_vertices)[-1]

    return new_lanelet.center_vertices



def spot_setup(scenario: Scenario, planning_problem: PlanningProblem):
    # Register scenario input id, lanelets, dynamic obstacle, problem_set
    spot.registerScenario(1, scenario.lanelet_network.lanelets, scenario.dynamic_obstacles, [planning_problem],np.empty([0, 2], float))

def compute_lanelet_polygons(lanelets) -> pycrcc.ShapeGroup:
    def lanelet_rep_setup(lanelets):
        lanelet_rep = []
        for lanelet in lanelets:
            triangles = pycrcc.ShapeGroup()
            triangle_builder.build_simple_triangles([lanelet], triangles)
            polyline = np.concatenate((lanelet.left_vertices, np.flipud(lanelet.right_vertices)))
            polygon = shp.Polygon(polyline)
            lanelet_rep.append((polygon, triangles))
        return lanelet_rep


    lanelet_rep = lanelet_rep_setup(lanelets)
    lanelet_polygons = pycrcc.ShapeGroup()
    for poly, tri in lanelet_rep:
        lanelet_polygons.add_shape(create_collision_object(poly))
    return lanelet_polygons

def set_obstacle_occupancy_prediction(scenario: Scenario, update_dict=None, end_time=10.0, num_threads=2):
    """
    creates occupancy prediction for all dynamic obstacles in scenario
    :param update_dict: Changes the parameters for the occupancy prediction
    :param end_time: Time horizon for which occupancy should be created of type float
    :param num_threads: Number of threads on which occupancy prediction should be done
    :return:
    """
    start_time = 0.0
    if update_dict:
        spot.updateProperties(1, update_dict)
    obstacles_occupancy = spot.doOccupancyPrediction(1, start_time, scenario.dt, end_time, num_threads)

    k = 0
    # Write computed occupancies in dynamic obstacles of the scenario
    for cpp_obstacle in obstacles_occupancy:
        print('Output for obstacle with ID: ', cpp_obstacle[0])
        obs = scenario.dynamic_obstacles[k]

        cr_occupancy_list = []

        for i in range(int(end_time / scenario.dt) + 1):
            occ = Occupancy(i + 1, ShapeGroup([]))
            cr_occupancy_list.append(occ)

            # print(len(occupancy_list))
        i = 0
        for vertices_at_time_step in cpp_obstacle[1]:

            j = 1  # iterator over vertices_at_time_step
            b = 0  # index to select vertices_at_time_step that are the start of a new polygon
            while j < len(vertices_at_time_step[1]):
                compare_vertex = vertices_at_time_step[1][b]  # first vertex of next polygon
                if compare_vertex[0] == vertices_at_time_step[1][j][0] and compare_vertex[1] == \
                        vertices_at_time_step[1][j][1]:
                    if (j + 1) - b < 3:  # polygon less than 3 vertices
                        print(
                            'Warning: one duplicated vertex skipped when copying predicted occupancies to CommonRoad')
                        b += 1  # try next vertex as first vertex (in case of equal vertices directly after each other)
                    else:
                        shape_obj = Polygon(np.array(vertices_at_time_step[1][b:j + 1]))
                        cr_occupancy_list[i].shape.shapes.append(shape_obj)
                        j += 1
                        b = j
                j += 1

            assert b == j - 1, ('Last polygon not closed (at time_step = ', i, ', b = ', b)
            i += 1

        scenario.dynamic_obstacles[k].prediction = SetBasedPrediction(1, cr_occupancy_list[0:])
        k += 1

