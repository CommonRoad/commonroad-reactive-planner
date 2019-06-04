import numpy as np
import warnings

from commonroad.scenario.trajectory import Trajectory, State
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad_ccosy.geometry.util import compute_curvature_from_polyline, calculate_orientation_from_polyline
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_object
from parameter import VehicleParameter
from utils import LaneCoordinateSystem, interpolate_lane_vertices


class CombinedPolynomialTrajectory(Trajectory):
    def __init__(self, t0, state_list, cost, partial_trajectory=False):
        assert(hasattr(state_list[-1], 'time_idx')), \
            '<CombinedTrajectory/__init__> time_idx attribute in state_list is mandatory'
        super(CombinedPolynomialTrajectory, self).__init__(t0, state_list)
        self.cost = cost
        self.duration = state_list[-1].time_idx
        self.partial_trajectory = partial_trajectory

    @property
    def cost(self):
        return self._cost

    @cost.setter
    def cost(self, cost):
        if cost < 0:
            self._cost = 0
            warnings.warn('Negative cost: set to 0.')
        else:
            self._cost = cost

    def check_feasibility(self) -> bool:
        """
        feasibility check contains currently the compliance with max/min velocities and max/min accelerations
        :return: True, if trajectory is feasible. Else False.
        """
        feasibility = True

        if (abs(max(self.state_list, key=lambda x: abs(x.acceleration)).acceleration) >
                VehicleParameter.acceleration_max):
            feasibility = False

        if (max(self.state_list, key=lambda x: x.velocity).velocity > VehicleParameter.velocity_max or
                min(self.state_list, key=lambda x: x.velocity).velocity < 0):
            feasibility = False

        # TODO: check max. jerk

        return feasibility

    def check_collision(self, scenario_cc) -> bool:
        """
        checks collision between trajectory and the obstacles' trajectories in obstacle list
        :param scenario_cc: scenario collision checker (pyfvks.collision.collision_checker object)
        :return: True, if trajectory collides with any collision relevant object in the scenario. Else False.
        """
        occ = TrajectoryPrediction(self.initial_time_idx, self, Rectangle(5, 2))
        ego_collision_object = create_collision_object(occ)
        return scenario_cc.collide(ego_collision_object)

        # import pyfvks
        # ego_bv = ego_collision_object.get_bounding_volume()
        # r_x = 0.5 * (ego_bv.max_x() - ego_bv.min_x())
        # r_y = 0.5 * (ego_bv.max_y() - ego_bv.min_y())
        #
        # cc_time = cc.window_query(pyfvks.collision.RectAABB(r_x, r_y, ego_bv.min_x() + r_x, ego_bv.min_y() + r_y))

        # return cc_time.collide(ego_collision_object)

    @staticmethod
    def create_ks_state_list(position_x, position_y, velocity, acceleration, orientation, steering_angle, time_idx):
        """
        Zip corresponding values in x,y -position, acceleration, orientation, steering angle and time lists to a state
        and create a list of such states
        :param position_x: list of x positions of the vehicle for each time step
        :param position_y: list of y positions of the vehicle for each time step
        :param velocity: numpy array with velocity of the vehicle for each time step
        :param acceleration: numpy array with acceleration of the vehicle for each time step
        :param orientation: list with orientations of the vehicle for each time step
        :param steering_angle: list with steering angles of the vehicle for each time step
        :param time_idx: numpy array with the indexes of the time steps.
        :return: list of states where each state has the x ,y - positions, velocity, acceleration, orientation and
        steering angle of a vehicle for a specific time step
        """
        state_list = list()
        for x, y, v, a, theta, phi, time in zip(position_x, position_y, velocity, acceleration, orientation,
                                                steering_angle, time_idx):
            state_elements = {'position': [x, y], 'velocity': v, 'acceleration': a, 'orientation': theta,
                              'time_idx': int(time), 'steering_angle': phi}
            state = State(**state_elements)
            state_list.append(state)
        return state_list

    @staticmethod
    def calc_cartesian_state(dt, duration, trajectory_long, trajectory_lat,
                             lane_coordinate_system: LaneCoordinateSystem):
        """
        Re-transformation from curvilinear coordinates to cartesian coordinates

        :param dt: time resolution of the trajectories
        :param duration:
        :param trajectory_long:
        :param trajectory_lat:
        :param lane_coordinate_system:
        :return: cartesian state lists for whole trajectory with resolution dt (x position, y position, velocity,
                        acceleration, orientation)

        Note: points outside of lanelet network are removed
        """

        # we always plan from t0=0 and shift in time later
        t = np.arange(0, duration * dt + dt, dt)

        s = trajectory_long.calc_position_at(t)
        d = trajectory_lat.calc_position_at(t)
        s_velocity = trajectory_long.calc_velocity_at(t)
        d_velocity = trajectory_lat.calc_velocity_at(t)
        s_acceleration = trajectory_long.calc_acceleration_at(t)
        d_acceleration = trajectory_lat.calc_acceleration_at(t)
        theta = [np.arctan2(d_vel, s_vel) for (s_vel, d_vel) in zip(s_velocity, d_velocity)]
        partial_trajectory = False
        if lane_coordinate_system.coordinate_system:
            x = [lane_coordinate_system.coordinate_system.convert_to_cartesian_coords(s_, d_)[0] for (s_, d_) in
                 zip(s, d) if lane_coordinate_system.coordinate_system.find_segment_index(s_) >= 0]
            y = [lane_coordinate_system.coordinate_system.convert_to_cartesian_coords(s_, d_)[1] for (s_, d_) in
                 zip(s, d) if lane_coordinate_system.coordinate_system.find_segment_index(s_) >= 0]

            center_vertices = lane_coordinate_system.lane.center_vertices
            if len(center_vertices) == 2:  # if lane has only two points, add one in between
                center_vertices = interpolate_lane_vertices(center_vertices)

            curvatures = compute_curvature_from_polyline(center_vertices)
            # TODO: calculate_orientation_from_polyline always returns a list with a zero as first element
            # Possible refactoring of this function.
            orientations = calculate_orientation_from_polyline(center_vertices)
            trajectory_vertices = np.array(list(zip(x, y)))
            if len(trajectory_vertices) == 2:
                trajectory_vertices = interpolate_lane_vertices(trajectory_vertices)
            trajectory_curvatures = np.abs(compute_curvature_from_polyline(trajectory_vertices))
            reference_curvatures = np.zeros(len(d))
            reference_orientations = np.zeros(len(d))
            distance_sum = 0
            closest_point_indices = [-1] * len(d)
            end = -1
            out_of_bounds = False
            for j in range(0, len(d)):
                for i in range(0, len(center_vertices) - 1):
                    distance_sum = distance_sum + \
                                   np.linalg.norm(center_vertices[i] -
                                                  center_vertices[i + 1])
                    if s[j] - distance_sum < 0:
                        closest_point_indices[j] = i + 1
                        break
                if closest_point_indices[j] > -1:
                    reference_curvatures[j] = curvatures[closest_point_indices[j]]
                    reference_orientations[j] = orientations[closest_point_indices[j]]
                elif out_of_bounds is False:
                    end = j
                    out_of_bounds = True
                distance_sum = 0
            v = np.sqrt(np.square(1 - reference_curvatures * d) * np.square(s_velocity) + np.square(d_velocity))
            for i in range(0, len(v)):
                theta[i] += reference_orientations[i]
            cos_curv = np.cos(reference_curvatures)
            tan_curv = np.tan(reference_curvatures)
            a = (s_acceleration * (1 - reference_curvatures * d) / cos_curv + (np.square(s_velocity) / cos_curv) *
                 ((1 - reference_curvatures * d) * tan_curv * (
                             np.gradient(theta) * (1 - reference_curvatures * d) / cos_curv - reference_curvatures) -
                  (np.gradient(reference_curvatures) * d + d_velocity * reference_curvatures)))
            for k in range(len(trajectory_curvatures), len(d)):
                trajectory_curvatures = np.append(trajectory_curvatures, 0)
            steering_angle = VehicleParameter.length * trajectory_curvatures + \
                              (VehicleParameter.mass / VehicleParameter.length) * \
                              (VehicleParameter.length_rear / VehicleParameter.stiffness_front -
                               VehicleParameter.length_front / VehicleParameter.stiffness_rear) * d_acceleration
            if end != -1:
                v = v[0:end]
                a = a[0:end]
                theta = theta[0:end]
                steering_angle[0:end]
                partial_trajectory = True
        else:  # if we plan in Cartesian coordinates
            x = s
            y = d
            v = np.sqrt(s_velocity * s_velocity + d_velocity * d_velocity)
            a = np.sqrt(s_acceleration * s_acceleration + d_acceleration * d_acceleration)
            trajectory_curvatures = np.abs(compute_curvature_from_polyline(np.array(list(zip(x, y)))))
            steering_angle = VehicleParameter.length * trajectory_curvatures + \
                              (VehicleParameter.mass / VehicleParameter.length) * \
                              (VehicleParameter.length_rear / VehicleParameter.stiffness_front -
                               VehicleParameter.length_front / VehicleParameter.stiffness_rear) * d_acceleration

        return x, y, v, a, theta, steering_angle, partial_trajectory
