
# planning parameter
class PlanningParameter:
    k_long = None
    k_lat = None

    k_jerk_lon = None
    k_jerk_lat = None
    k_time = None
    k_distance = None

    prediction_horizon = None
    speed_limit = None

    t_step_size = None
    lat_step_size = None
    long_step_size = None


# vehicle parameter
class VehicleParameter:
    jerk_long_max = None
    jerk_lat_max = None

    acceleration_max = 8  # m/s²
    velocity_max = 150 / 3.6  # m/s

    curvature_max = 0.2

    width = 1.674  # vehicle width [m]
    length = 4.298  # vehicle length [m]

    # parameters for calculating steering angle

    length_front = length / 3
    length_rear = 2 * length / 3
    stiffness_front = 40000  # front tire stiffness [N/rad]
    stiffness_rear = 40000  # rear tire stiffens [N/rad]
    mass = 1500  # mass of vehicle [kg]



def parameter_velocity_reaching():

    params = PlanningParameter()

    params.k_long = .1
    params.k_lat = 5.0

    params.k_jerk_lat = 5.0
    params.k_jerk_lon = 5.0
    params.k_time = 10.0
    params.k_distance = 2.0

    params.prediction_horizon = 3.0     # s
    params.speed_limit = 130.0/3.6      # m/s

    params.t_step_size = .5               # s
    params.lat_step_size = 1
    params.long_step_size = 1

    return params


def parameter_position_reaching():
    params = PlanningParameter()

    params.k_long = 1.0
    params.k_lat = 10.0

    params.k_jerk_lat = 5.0  # 1.0
    params.k_jerk_lon = 5.0
    params.k_time = 10.0
    params.k_distance = 10.0  # 2.0

    params.prediction_horizon = 3.0  # s
    params.speed_limit = 130.0/3.6      # m/s

    params.t_step_size = .5  # s
    params.lat_step_size = 1
    params.long_step_size = 1

    return params


# visualization parameter
basic_shape_parameters_static = {'opacity': 1.0,
                                   'facecolor': '#1d7eea',
                                   'edgecolor': '#0066cc',
                                   'zorder': 20}

basic_shape_parameters_dynamic = {'opacity': 1.0,
                                 'facecolor': '#1d7eea',
                                 'edgecolor': '#0066cc',
                                 'zorder': 100}

draw_params_scenario = {'scenario': {
                        'dynamic_obstacle': {
                            'draw_shape': True,
                            'draw_icon': False,
                            'draw_bounding_box': True,
                            'show_label': False,
                            'trajectory_steps': 25,
                            'zorder': 100,
                            'occupancy': {
                                'draw_occupancies': 1,  # -1= never, 0= if prediction of vehicle is set-based, 1=always
                                'shape': {
                                    'polygon': {
                                    'opacity': 0.2,
                                       'facecolor': '#ff4000',
                                       'edgecolor': '#cc3300',
                                       'zorder': 18,
                                    },
                                    'rectangle': {
                                       'opacity': 0.2,
                                       'facecolor': '#1d7eea',
                                       'edgecolor': '#0066cc',
                                       'zorder': 18,
                                    },
                                    'circle': {
                                       'opacity': 0.2,
                                       'facecolor': '#1d7eea',
                                       'edgecolor': '#0066cc',
                                       'zorder': 18,
                                    }
                                },
                            },
                            'shape': {
                                'polygon': basic_shape_parameters_dynamic,
                                'rectangle': basic_shape_parameters_dynamic,
                                'circle': basic_shape_parameters_dynamic
                            },
                             'trajectory': {'facecolor': '#000000'}
                        },
                        'static_obstacle': {
                           'shape': {
                               'polygon': basic_shape_parameters_static,
                               'rectangle': basic_shape_parameters_static,
                               'circle': basic_shape_parameters_static,
                           }
                        },
                        'lanelet_network': {
                            'lanelet': {'left_bound_color': '#555555',
                                       'right_bound_color': '#555555',
                                       'center_bound_color': '#dddddd',
                                       'draw_left_bound': True,
                                       'draw_right_bound': True,
                                       'draw_center_bound': True,
                                       'draw_border_vertices': False,
                                       'draw_start_and_direction': True,
                                       'show_label': False,
                                       'draw_linewidth': 0.5,
                                       'fill_lanelet': True,
                                       'facecolor': '#e8e8e8'}},
                   },
}
