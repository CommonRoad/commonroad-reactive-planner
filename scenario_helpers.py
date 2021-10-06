# import construction
# import triangle_builder
from typing import List
import copy

import numpy as np
from scipy.interpolate import splprep, splev

# commonroad_dc
from commonroad_dc.boundary import construction, triangle_builder
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.pycrcc import *
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object

# commonroad-io
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
import commonroad.geometry.shape as shp
from commonroad.scenario.trajectory import State, Trajectory
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork

# commonroad-ccosy
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.geometry.util import resample_polyline

# commonroad_rp
from commonroad_rp.utils import compute_pathlength_from_polyline, compute_orientation_from_polyline
from commonroad_rp.parameter import VehModelParameters
from commonroad_rp.utils import CoordinateSystem

# Recently not use the spot 
# import spot

draw_parameters_intended = {}
draw_parameters_fail_safe = {}
draw_parameters_ego = {}
draw_parameters_scenario = {}


def update_draw_params():
    # intended
    draw_parameters_intended.update({'facecolor': '#000000'})
    draw_parameters_intended.update({'edgecolor': '#000000'})
    draw_parameters_intended.update({'dynamic_obstacle': {'trajectory_steps': 200}})
    draw_parameters_intended.update({'time_end': 200})
    draw_parameters_intended.update({'trajectory': {'facecolor': 'r'}})
    draw_parameters_intended.update({'static_obstacle': {'shape': {'rectangle': {'facecolor': '##1d7eea'}}}})
    draw_parameters_intended.update({'static_obstacle': {'shape': {'rectangle': {'edgecolor': '#0066cc'}}}})

    # fail-safe
    draw_parameters_fail_safe.update({'dynamic_obstacle': {'trajectory_steps': 200}})
    draw_parameters_fail_safe.update({'edgecolor': '#FF0000'})
    draw_parameters_fail_safe.update({'facecolor': '#FF0000'})
    draw_parameters_fail_safe.update({'time_end': 200})

    # scenario
    draw_parameters_scenario.update({'time_end': 200})
    draw_parameters_scenario.update({'dynamic_obstacle': {'trajectory_steps': 200}})

    # static obstacles
    # draw_parameters_intended['scenario']['static_obstacle']['shape']['rectangle']['facecolor'] = '#1d7eea'
    # draw_parameters_intended['scenario']['static_obstacle']['shape']['rectangle']['edgecolor'] = '#0066cc'

    # # ego initial shape
    # draw_parameters_ego['shape']['rectangle']['facecolor'] = '#000000'
    # draw_parameters_ego['shape']['rectangle']['edgecolor'] = '#000000'


update_draw_params()


def create_road_boundary(scenario: Scenario, draw=False) -> StaticObstacle:
    #method =('triangulation', {'call_options': 'a100q'}) # 'shell' # 'triangulation'
    method = 'triangulation' # 'shell' # 'triangulation'
    #build = ['section_triangles','triangulation']
    build = ['section_triangles', ('triangulation', {'max_area': '1'})]
    if draw:
        draw = ['triangulation']
        boundary = construction.construct(scenario, build, draw)
    else:
        # boundary = construction.construct(scenario, build, [], [])
        boundary = construction.construct(scenario, build, [])

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


def _obtain_correct_lanelet_for_pose(state: State, lanelet_network: LaneletNetwork, lanes: List[int]):

    best = 1000
    best_lane = None
    for lane_id in lanes:
        lane = lanelet_network.find_lanelet_by_id(lane_id)
        pos = compute_pathlength_from_polyline(lane.center_vertices)
        orientation = compute_orientation_from_polyline(lane.center_vertices)
        # cosys = create_coordinate_system_from_polyline(lane.center_vertices)
        cosys = CurvilinearCoordinateSystem(lane.center_vertices)
        s,d = cosys.convert_to_curvilinear_coords(state.position[0],state.position[1])
        theta = np.interp(s,pos,orientation)
        diff = abs(state.orientation - theta)
        if diff < best:
            best = diff
            best_lane = lane
    return best_lane


def obtain_reference_path(trajectory: Trajectory, scenario: Scenario) -> np.ndarray:
    # create coordinate system
    state = trajectory.state_list[0]
    ego_lanelet_id = scenario.lanelet_network.find_lanelet_by_position([state.position])[0]
    if len(ego_lanelet_id) > 1:
        ego_lanelet = _obtain_correct_lanelet_for_pose(state, scenario.lanelet_network, ego_lanelet_id)
        ego_lanelet_id = ego_lanelet.lanelet_id
    else:
        ego_lanelet_id = ego_lanelet_id[0]
        ego_lanelet = scenario.lanelet_network.find_lanelet_by_id(ego_lanelet_id)
    print('Ego vehice is located in lanelet id={}'.format(ego_lanelet_id))

    # check if reference path is long enough
    new_lanelet = ego_lanelet
    temp = ego_lanelet
    visited = set()
    visited.add(temp.lanelet_id)
    # get all positions from trajectory
    positions = list()
    for state in trajectory.state_list:
        positions.append(state.position)
    positions = np.array(positions)
    while len(temp.successor) and any(temp.successor[i] not in visited for i in range(len(temp.successor))): #( if len(temp.successor) else False):
        # find matching successor for trajectory
        candidate = scenario.lanelet_network.find_lanelet_by_id(temp.successor[0])
        points = 0
        for i in range(len(temp.successor)):
            lane = scenario.lanelet_network.find_lanelet_by_id(temp.successor[i])
            matches = sum(lane.contains_points(positions))
            if matches > points:
                candidate = lane
                points = matches

        temp = candidate
        visited.add(temp.lanelet_id)
        new_lanelet = Lanelet.merge_lanelets(new_lanelet,temp)
    return new_lanelet.center_vertices


def spot_setup(scenario: Scenario, planning_problem: PlanningProblem, update_dict: dict):
    # Register scenario input id, lanelets, dynamic obstacle, problem_set
    spot.registerScenario(1, scenario.lanelet_network.lanelets, scenario.dynamic_obstacles, [planning_problem],np.empty([0, 2], float))

    spot.updateProperties(1, update_dict)


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
            occ = Occupancy(i + 1, shp.ShapeGroup([]))
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
                        shape_obj = shp.Polygon(np.array(vertices_at_time_step[1][b:j + 1]))
                        cr_occupancy_list[i].shape.shapes.append(shape_obj)
                        j += 1
                        b = j
                j += 1

            assert b == j - 1, ('Last polygon not closed (at time_step = ', i, ', b = ', b)
            i += 1

        scenario.dynamic_obstacles[k].prediction = SetBasedPrediction(1, cr_occupancy_list[0:])
        k += 1


def remove_scenario_from_spot():

    spot.removeScenario(1)


def _compute_braking_maneuver(x0: State, cosys: CoordinateSystem, dT: float, params: VehModelParameters, t_react=0.3):
    # transform into curvilinear coordinate system
    s0, d0 = cosys.convert_to_curvilinear_coords(x0.position[0],x0.position[1])

    # compute braking maneuver
    v0 = x0.velocity
    t_brake = abs(v0/params.a_max)
    t = np.arange(0,t_brake+dT,dT)
    if t[-1] > t_brake:
        t = t[:-1]
    s_brake = s0 + v0*(t + t_react) -0.5*params.a_max*np.square(t)

    # transform back to Cartesian coordinate system
    x_brake = list()
    y_brake = list()
    for s in s_brake:
        res = cosys.convert_to_cartesian_coords(s, 0)
        if res is not None:
            x_brake.append(res[0])
            y_brake.append(res[1])

    # compute orientation
    theta_brake = np.interp(s_brake,cosys.ref_pos(),cosys.ref_theta())

    # compute list of rectangles
    shapes = list()
    for i in range(len(x_brake)):
        ego = pycrcc.TimeVariantCollisionObject(x0.time_step + i)
        ego.append_obstacle(pycrcc.RectOBB(0.5 * params.veh_length, 0.5 * params.veh_width, theta_brake[i], x_brake[i], y_brake[i]))
        shapes.append(ego)
    return shapes


def compute_simplified_ttr(intended: Trajectory, cc, cosys: CoordinateSystem, dT: float, params: VehModelParameters) -> int:
    # initial ttr
    ttr = -1
    # go through states
    for i,x in enumerate(intended.state_list):
        shapes = _compute_braking_maneuver(x, cosys, dT, params)
        collide = False

        for shape in shapes:
            if cc.collide(shape):
                collide = True
                break

        if not collide:
            if ttr == i - 1:
                ttr = i
            else:
                break

    return ttr if ttr >= 0 else 0


def smooth_reference(reference: np.ndarray) -> np.ndarray:
    """
    Smooths a given reference polyline for lower curvature rates. The smoothing is done using splines from scipy.
    :param reference: The reference to smooth
    :return: The smoothed reference
    """
    tck, u = splprep(reference.T, u=None, s=0.0)  # , per=1)
    u_new = np.linspace(u.min(), u.max(), 1000)
    x_new, y_new = splev(u_new, tck, der=0)

    return np.array([x_new, y_new]).transpose()


def smoothing_reference_path(reference_path):
    """
    Smooths a given reference polyline for lower curvature rates. The smoothing is done using splines from scipy.
    :param reference_path: The reference_path to smooth [array]
    :return: The smoothed reference
    """
    transposed_reference_path = reference_path.T
    okay = np.where(np.abs(np.diff(transposed_reference_path[0])) + np.abs(np.diff(transposed_reference_path[1])) > 0)
    xp = np.r_[transposed_reference_path[0][okay], transposed_reference_path[0][-1]]
    yp = np.r_[transposed_reference_path[1][okay], transposed_reference_path[1][-1]]
    tck, u = splprep([xp, yp], s=0)
    # tck, u = splprep(transposed_reference_path)
    u_new = np.linspace(u.min(), u.max(), 1000)
    x_new, y_new = splev(u_new, tck, der=0)

    return np.array([x_new, y_new]).transpose()


def extrapolate_ref_path(reference_path: np.ndarray, resample_step: float = 2.0) -> np.ndarray:
    """
    Function to extrapolate the end of the reference path in order to avoid CCosy errors and/or invalid trajectory
    samples when the reference path is too short.
    :param reference_path: original reference path
    :param resample_step: interval for resampling
    :return extrapolated reference path
    """
    p = np.poly1d(np.polyfit(reference_path[-2:, 0], reference_path[-2:, 1], 1))
    x = 2.3*reference_path[-1, 0] - reference_path[-2, 0]
    new_polyline = np.concatenate((reference_path, np.array([[x, p(x)]])), axis=0)
    return resample_polyline(new_polyline, step=resample_step)


def shift_ref_path(reference_path: np.ndarray, shift: np.ndarray, scale: float) -> np.ndarray:
    """
    Function to shift ref path along a given slope specified by shift
    """
    ref_path_shifted = reference_path - scale*shift
    return ref_path_shifted



