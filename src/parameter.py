# planning parameter
class PlanningParameter:
    def __init__(self, velocity_reaching=True):

        if velocity_reaching:
            # weightening factors for cost calculation
            self.k_long = .1
            self.k_lat = 1.0

            self.k_jerk_lat = 5.0
            self.k_jerk_lon = 5.0
            self.k_time = 10.0
            self.k_distance = 15.0

            # time planning parameters: t_N * t_step_size = prediction_horizon
            self.prediction_horizon = 3.0  # s
            self.t_N = 3
            self.t_step_size = .2  # s
            self.t_min = 0.1

            # direction planning parameter
            self.d_deviation = 2
            self.d_N = 10

        else:
            # Values for position reaching
            self.k_long = 1.0
            self.k_lat = 10.0

            self.k_jerk_lat = 5.0  # 1.0
            self.k_jerk_lon = 5.0
            self.k_time = 10.0
            self.k_distance = 10.0  # 2.0

            # time planning parameters: t_N * t_step_size = prediction_horizon
            self.prediction_horizon = 3.0  # s
            self.t_N = 30
            self.t_step_size = .2  # s
            self.t_min = 0.6

            # direction planning parameter
            self.d_deviation = 2
            self.d_N = 5

# vehicle parameter
class VehicleParameter:
    def __init__(self):
        self.jerk_long_max = None
        self.jerk_lat_max = None

        self.acceleration_max = 8  # m/s²
        self.acceleration_dot_max = 0.2
        self.velocity_max = 150 / 3.6  # m/s

        self.curvature_max = 0.2

        self.kappa_max = 0.2
        self.kappa_dot_max = 10

        self.width = 1.674  # vehicle width [m]
        self.length = 4.298  # vehicle length [m]

        # parameters for calculating steering angle

        self.length_front = self.length / 3
        self.length_rear = 2 * self.length / 3
        self.stiffness_front = 40000  # front tire stiffness [N/rad]
        self.stiffness_rear = 40000  # rear tire stiffens [N/rad]
        self.mass = 1500  # mass of vehicle [kg]


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
