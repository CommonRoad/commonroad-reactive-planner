import os
from scipy.spatial import ConvexHull
from trajectory_params_bundle import TrajectoryParamsBundle
from polynomial_trajectory import QuinticTrajectory
from parameter import parameter_velocity_keeping
import numpy as np
from combined_trajectory import CombinedPolynomialTrajectory
from shapely.geometry import Polygon
from commonroad_ccosy.geometry.trapezoid_coordinate_system import create_coordinate_system_from_polyline
import scipy.interpolate as si
import shapely.affinity

class Matlab:
    def __init__(self, filename, eng):
        self.filename = filename
        self.eng = eng
        self.map = None
        self.__get_occupancy_map_object()

    def __get_occupancy_map_object(self):
        perception = self.eng.globalPck.Perception(self.filename)
        self.eng.workspace['matlab_perception'] = perception
        self.map = self.eng.eval("matlab_perception.map")


class EmergencyTrajectory:
    def __init__(self, matlab_map, eng, scenario, planning_problem):
        self.scenario = scenario
        self.len_obstacles = len(self.scenario._obstacles)
        self.params = parameter_velocity_keeping()
        self.matlab_map = matlab_map
        self.eng = eng
        self.start_time = None
        self.end_time = None
        self.planning_problem = planning_problem
        self.current_vertices = {}

    def create_trajectory_params_bundle(self, current_s, current_d, current_time):
        pass

    def calculate_emergency_trajectory(self, current_x, current_y, current_acceleration, current_velocity):
        """
        Checks if all maneuver types are possible in the context of the emergency trajectory from current position.
        Transforms position into curvilinear coordinates. Finds optimal trajectory for this maneuver according to
        Werling M et al. "Optimal trajectory generation for dynamic street scenarios in a frenet frame". In: IEEE
        International Conference on Robotics and Automation, Anchorage, Alaska, 987–993.
        :param current_x: current x position in cartesian coordinates
        :param current_y: current y position in cartesian coordinates
        :param current_velocity: current velocity in m/s
        :param current_acceleration: current acceleration in m/s²
        :return: optimal trajectories and lane for given maneuver type
        """
        position = np.array([current_x, current_y])
        current_lanelet = self.scenario.lanelet_network.find_lanelet_by_position(np.array([position]))
        coordinate_system_for_lane = self.create_coordinate_systems_for_lanelet_id(current_lanelet)
        optimal_trajectories_for_lanes = []
        # determine position in curvilinear coordinates
        for coordinate_system in coordinate_system_for_lane:
            s, d = coordinate_system[0].convert_to_curvilinear_coords(current_x, current_y)
            current_s = np.array([s, current_velocity, current_acceleration])
            current_d = np.array([d, 0.0, 0.0])
            # calculate best trajectory for given maneuver
            v = EmergencyVelocityKeeping(self.scenario.dt, self.planning_problem, current_x, current_y)
            tb = v.create_trajectory_params_bundle(current_s, current_d, 0)  # TODO: change time?
            while True:
                optimal_trajectory_params = tb.optimal_trajectory_params()

                if optimal_trajectory_params is None:
                    break

                trajectory_long, trajectory_lat, dt, params, cost = optimal_trajectory_params
                optimal_trajectory = CombinedPolynomialTrajectory(trajectory_long, trajectory_lat, dt,
                                                                  coordinate_system, params)
                optimal_trajectory.set_cost(cost)
                if optimal_trajectory.check_collision(self.scenario):
                    tb.remove_trajectory_params(tb.optimal_trajectory_params_idx())
                    continue

                if not optimal_trajectory.check_feasibility():
                    tb.remove_trajectory_params(tb.optimal_trajectory_params_idx())
                    continue
                break
            if optimal_trajectory_params is not None:
                optimal_trajectory_for_lane = [optimal_trajectory, coordinate_system[1]]
                optimal_trajectories_for_lanes.append(optimal_trajectory_for_lane)
        return optimal_trajectories_for_lanes

    def create_coordinate_systems_for_lanelet_id(self, lanelet_id: int) -> list:
        """
        Creates a list of curvilinear coordinate systems with one coordinate system for every possible lane starting
        from a given lanelet. The reference curve are the center vertices of the lane
        :param lanelet_id: initial reference lanelet
        :return: list of curvilinear coordinate system for a given lane along with the corresponding lane.
        """
        init_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        lanes = init_lanelet.all_lanelets_by_merging_successors_from_lanelet(init_lanelet, self.scenario.lanelet_network)
        coordinate_systems_for_lanes = []
        if not lanes:
            lanes.append(init_lanelet)
        for lane in lanes:
            """smoothing center lane with b-splines from scipy"""
            x = lane.center_vertices[:, 0]
            y = lane.center_vertices[:, 1]

            t = range(len(lane.center_vertices))
            ipl_t = np.linspace(0.0, len(lane.center_vertices) - 1, 1000)

            k_x = 5 if len(x) > 5 else len(x) - 1
            k_y = 5 if len(y) > 5 else len(y) - 1
            x_tup = si.splrep(t, x, k=k_x)
            y_tup = si.splrep(t, y, k=k_y)

            x_list = list(x_tup)
            xl = x.tolist()
            x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

            y_list = list(y_tup)
            yl = y.tolist()
            y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

            x_i = si.splev(ipl_t, x_list)
            y_i = si.splev(ipl_t, y_list)
            smoothed_reference_curve = np.stack((x_i, y_i), axis=-1)
            coordinate_system_for_lane = [create_coordinate_system_from_polyline(smoothed_reference_curve), lane]
            coordinate_systems_for_lanes.append(coordinate_system_for_lane)
        return coordinate_systems_for_lanes

    def get_occupancy_set(self, matlab_map, matlab_eng):
        """
        returns the occupancy set from start point ts_prediction to end point tf_prediction for every time step dt
        :param matlab_map: A SPOT map object
        :param matlab_eng: The matlab-python engine object
        :return: four-dimensional list: [id of obstacle, timestep, lane,vertices(x,y)] with [0,x,x,x] being the ego vehicle
        NOTE: when no vertices in a specific timestep/lane array is filled solely with 0s #TODO: is this ok?
        """
        #  set projection time
        time_interval_prediction = matlab_eng.globalPck.TimeInterval(self.start_time, self.end_time, self.end_time)   #TODO:check if logical?
        # Create Matlab objects
        perception = matlab_eng.globalPck.Perception(matlab_map)  # eventually map object possible->performance reasons
        matlab_eng.addpath(matlab_eng.genpath(os.path.join(os.path.dirname(__file__), os.pardir, 'spot')))
        matlab_eng.workspace['matlab_perception'] = perception
        matlab_eng.workspace['matlab_timeInterval_prediction'] = time_interval_prediction
        matlab_eng.eval("matlab_perception.computeOccupancyGlobal(matlab_timeInterval_prediction)", nargout=0)

        # Get timesteps, lanes from matlab
        lanes_timesteps_length = matlab_eng.eval("size(matlab_perception.map.obstacles(1,1).occupancy)")
        len_timesteps = 1  #(self.start_time-self.start_time+self.frequency)/self.frequency
        len_lanes = lanes_timesteps_length[0][0]

        # Create array for the extracted data
        all_vertices_by_id = {}
        all_vertices_ego = np.zeros((int(len_timesteps), int(len_lanes), 2, 1000))

        # Fill in vertices for ever obstacle, timestep, and lane split by x and y coordinates
        for obstacle in range(int(self.len_obstacles)):
            all_vertices_obstacle = np.zeros((int(len_timesteps), int(len_lanes), 2, 1000))
            for timestep in range(int(len_timesteps)):
                for lane in range(int(len_lanes)):
                    #  get vertices once for the ego vehicle
                    if obstacle == 0:
                        vertices_ego = matlab_eng.eval("matlab_perception.map.egoVehicle.occupancy(" + str(lane + 1)
                                                + "," + str(timestep + 1) + ").vertices")
                        if vertices_ego:
                            for i, (entry1, entry2) in enumerate(zip(vertices_ego[0], vertices_ego[1])):
                                all_vertices_ego[timestep, lane, 0, i] = entry1
                                all_vertices_ego[timestep, lane, 1, i] = entry2

                    vertices_obstacles = matlab_eng.eval("matlab_perception.map.obstacles(1,"+str(obstacle + 1)+").occupancy("
                                                  + str(lane + 1)+","+str(timestep + 1)+").vertices")
                    #  get the vertices for the obstacles
                    if vertices_obstacles:
                        for i, (entry1, entry2) in enumerate(zip(vertices_obstacles[0], vertices_obstacles[1])):
                            all_vertices_obstacle[timestep, lane, 0, i] = entry1
                            all_vertices_obstacle[timestep, lane, 1, i] = entry2
            if obstacle == 0:
                all_vertices_by_id[0] = all_vertices_ego
            all_vertices_by_id[int(matlab_eng.eval("matlab_perception.map.obstacles(1," +
                                            str(obstacle + 1)+").id"))] = all_vertices_obstacle

        self.current_vertices = all_vertices_by_id

    def get_convex_hull_by_id_timestep(self, car_id, timestep):
        """
        returns the convex hull for a specific car at a specific timestep
        :param car_id: id of car for which convex hull should be calculated
        :param timestep: timestep for which the convex hull should be calculated
        :return: two-dimensional list: [x-coordinate, y-coordinate] of convex-hull
        """
        x_vertices_of_all_lanes = []
        y_vertices_of_all_lanes = []
        for lane in self.current_vertices[car_id][timestep]:
            for x, y in zip(lane[0], lane[1]):
                if x and y is not 0:  #TODO: check if possible to do so
                    x_vertices_of_all_lanes.append(x)
                    y_vertices_of_all_lanes.append(y)
        vertices = [x_vertices_of_all_lanes, y_vertices_of_all_lanes]
        vertices = np.transpose(vertices)
        hull = ConvexHull(vertices)
        #transform back to array
        convex_hull = np.zeros((len(hull.vertices), 2))
        for i, index in enumerate(hull.vertices):
            convex_hull[i] = vertices[index]

        return convex_hull

    def find_sampling_area(self):
        """
        Finds the sampling area. Sampling area is the area where the occupancy set of the ego vehicle does not collide with the occupancy set of the other vehicles
        :return: Polygon Object from shapely which contains x and y coordinates of the sampling area
        """
        sampling_area = Polygon(self.get_convex_hull_by_id_timestep(0, 0))
        for car_id, _ in self.current_vertices.items():
            if car_id is not 0:
                sampling_area = sampling_area.difference(Polygon(self.get_convex_hull_by_id_timestep(car_id, 0)))
        return sampling_area


class EmergencyVelocityKeeping:
    def __init__(self, dt, planning_problem,current_x, current_y):
        """
        VelocityKeeping object plans trajectories that try to reach a certain velocity.
        :param dt: time resolution of trajectory
        :param desired_velocity: Reference velocity. Deviations from that are penalized in the cost function
        """
        self.dt = dt
        self.params = parameter_velocity_keeping()
        self.planning_problem = planning_problem
        self.current_x = current_x
        self.current_y = current_y

    def create_trajectory_params_bundle(self, current_s, current_d, current_time):
        """
        sample in time (duration). Initial state is given. Longitudinal end state (s) is sampled for s[0] in the goal region. s[1],s[2] stay same.
        Lateral end state (d) is sampled between the goal region .
        :param current_s: np.array([s, s_dot, s_ddot])
        :param current__d: np.array([d, d_dot, d_ddot])
        :param current_time: discrete time index
        :return: trajectory bundle with all sample trajectories.
        NOTE: Here, no collision or feasibility check is done!
        """
        #get coordinates of goal region -> more effective way?
        r_x = self.planning_problem.goal.state_list[0].position.r_x()
        r_y = self.planning_problem.goal.state_list[0].position.r_y()
        c_x = self.planning_problem.goal.state_list[0].position.center()[0]
        c_y = self.planning_problem.goal.state_list[0].position.center()[1]
        goal_area = Polygon([(c_x + r_x, c_y+r_y), (c_x - r_x, c_y-r_y), (c_x - r_x, c_y+r_y),(c_x + r_x, c_y - r_y)])
        goal_area = shapely.affinity.rotate(goal_area, 2.14859173, use_radians=False) #TODO: how to get orienation

        # get trajectory params_bundle with sampled parameters
        trajectory_params_bundle = TrajectoryParamsBundle()
        for T in range(2, 80, 1): #TODO: time range?
            for delta_d in range(int(current_d[0]+(goal_area.bounds[1] - self.current_y)), int(current_d[0]+(goal_area.bounds[3] - self.current_y)), 1):
                end_state_d = np.array([delta_d, 0.0, 0.0])
                trajectory_lat = QuinticTrajectory(t_start_s=current_time * self.dt, duration_s=T * self.dt,
                                                   start_state=current_d, end_state=end_state_d)
                trajectory_lat.jerk_cost = trajectory_lat.squared_jerk_integral(T * self.dt) / (T * self.dt)
                trajectory_lat.time_cost = 1.0 / (T * self.dt)
                trajectory_lat.distance_cost = 0
                trajectory_lat.set_cost(self.params.k_jerk_lon, self.params.k_time, 0)
                for delta_s in range(int(int(current_s[0]) + (goal_area.bounds[0]-self.current_x)), int(current_s[0]+(goal_area.bounds[2]-self.current_x)),1):
                    trajectory_long = QuinticTrajectory(t_start_s=current_time * self.dt,
                                                        duration_s=T * self.dt,
                                                        start_state=current_s, end_state=[delta_s, current_s[1], current_s[2]]) #TODO: input for acceleration and velocity

                    trajectory_long.jerk_cost = trajectory_long.squared_jerk_integral(T * self.dt) / (T * self.dt)
                    trajectory_long.time_cost = 1.0 / (T * self.dt)
                    trajectory_long.distance_cost = 0
                    trajectory_long.set_cost(self.params.k_jerk_lon, self.params.k_time,self.params.k_distance)
                    trajectory_cost = self.params.k_long * trajectory_long.cost + self.params.k_lat * trajectory_lat.cost
                    trajectory_params = (trajectory_long, trajectory_lat, self.dt, self.params, trajectory_cost)
                    trajectory_params_bundle.add_trajectory_params(trajectory_params)
        return trajectory_params_bundle