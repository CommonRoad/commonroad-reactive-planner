__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.1"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Alpha"

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.trajectory import State
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
import commonroad.geometry.shape as shp

from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker

import construction
import lanelet_bounds
import triangle_builder
import pycrcc
from pycrcc import *


import commonroad_cc.visualization.draw_dispatch as crd
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_checker,\
    create_collision_object


import matplotlib.pyplot as plt
import numpy as np

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


if __name__ == '__main__':
    print('Creating velocity reaching bundle....')

    # Load example scenario ZAM Over
    crfr = CommonRoadFileReader('../unit_tests/scenarios/ZAM_Over-1_1.xml')
    scenario, _ = crfr.open()
    road_boundary_sg, road_boundary_obstacle = create_road_boundary(scenario, draw=False)
    lanelet_network = scenario.lanelet_network
    lanelets = lanelet_network.lanelets


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


    plt.figure(figsize=(25, 10))
    draw_object(scenario)
    crd.draw_object(road_boundary_sg)
    #crd.draw_object(lanelet_polygons)
    plt.axis('equal')
    plt.show(block=False)
    plt.pause(0.1*10)

    # create coordinate system
    reference_path = scenario.lanelet_network.find_lanelet_by_id(1000).center_vertices
    curvilinear_cosy = create_coordinate_system_from_polyline(reference_path)

    # create collision checker for scenario
    collision_checker = create_collision_checker(scenario)
    collision_checker.add_collision_object(road_boundary_sg)

    # convert coordinates and create initial state
    x, y = curvilinear_cosy.convert_to_cartesian_coords(25, 0)
    x_0 = State(**{'position':np.array([x,y]),'orientation':-0.2, 'velocity':10, 'acceleration':0,'yaw_rate':0})

    planner: ReactivePlanner = ReactivePlanner(0.2, 6, 30)
    planner.set_reference_path(reference_path)

    x_cl = None

    for k in range(0, 18):
        optimal = planner.plan(x_0, collision_checker, cl_states=x_cl)
        # convert to CR obstacle
        ego = planner.convert_cr_trajectory_to_object(optimal[0])
        draw_object(ego)
        draw_object(ego.prediction.occupancy_at_time_step(1))
        plt.pause(0.1)

        x_0 = optimal[0].state_list[1]
        x_cl = (optimal[2][1], optimal[3][1])

        #print("Goal state is: {}".format(optimal[1].state_list[-1]))

    print('Done')
    plt.show(block=True)