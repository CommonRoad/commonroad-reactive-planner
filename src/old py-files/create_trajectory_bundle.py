import numpy as np

from trajectory_bundle import TrajectoryBundle, TrajectorySample
from sampling_area import SamplingArea
from parameter import parameter_velocity_reaching, parameter_position_reaching
from polynomial_trajectory import QuinticTrajectory, QuarticTrajectory


def velocity_reaching_bundle(dt: float, desired_velocity: float, current_state_s: np.array, current_state_d: np.array,
                             current_time_idx: int=0) -> TrajectoryBundle:
    """
    Plans trajectory samples that try to reach a certain velocity and samples in this domain.
    Sample in time (duration) and velocity domain. Initial state is given. Longitudinal end state (s) is sampled.
    Lateral end state (d) is always set to 0.
    :param dt: time resolution of trajectory
    :param desired_velocity: Reference velocity. Deviations from that are penalized in the cost function
    :param current_state_s: np.array([s, s_dot, s_ddot])
    :param current_state_d: np.array([d, d_dot, d_ddot])
    :param current_time_idx: discrete time index
    :return: trajectory bundle with all sample trajectories.

    NOTE: Here, no collision or feasibility check is done!
    """
    params = parameter_velocity_reaching()

    trajectory_bundle = TrajectoryBundle(params)

    for T in range(int(params.t_step_size/dt),
                   int(params.prediction_horizon/dt) + int(params.t_step_size/dt),
                   int(params.t_step_size/dt)):

        # Lateral end state is fixed. Therefore only one sample for each duration
        end_state_d = np.array([0.0, 0.0, 0.0])
        trajectory_lat = QuinticTrajectory(t_start_s=current_time_idx*dt, duration_s=T*dt,
                                           start_state=current_state_d, end_state=end_state_d)
        jerk_cost = trajectory_lat.squared_jerk_integral(T*dt)/(T*dt)
        time_cost = 1.0/(T*dt)
        distance_cost = 0
        trajectory_lat.set_cost(jerk_cost, time_cost, distance_cost, params.k_jerk_lat, params.k_time, 0)

        # Longitudinal sampling for all possible velocities
        # TODO: sample with params.long_step_size
        for v in range(int(params.speed_limit)):
            trajectory_long = QuarticTrajectory(t_start_s=current_time_idx*dt, duration_s=T*dt,
                                                start_state=current_state_s, target_velocity=v)
            jerk_cost = trajectory_long.squared_jerk_integral(T*dt)/(T*dt)
            time_cost = 1.0/(T*dt)
            distance_cost = (v - desired_velocity)**2
            trajectory_long.set_cost(jerk_cost, time_cost, distance_cost,
                                     params.k_jerk_lon, params.k_time, params.k_distance)

            trajectory_cost = params.k_long * trajectory_long.cost + params.k_lat * trajectory_lat.cost
            trajectory_sample = TrajectorySample(dt, trajectory_long, trajectory_lat, trajectory_cost)
            trajectory_bundle.add_trajectory(trajectory_sample)

    return trajectory_bundle


def position_reaching_bundle(dt: float, s_min: float, s_max: float, d_min: float, d_max: float, desired_v: float,
                             current_state_long: np.array, current_state_lat: np.array, current_time_idx: int):
    """
    Plans trajectories that try to reach a certain position and samples in this domain.
    Sample in time (duration) and velocity domain. Initial state is given. Longitudinal end state (s) is sampled.
    Lateral end state (d) is always set to 0.

    :param dt: time resolution of trajectory
    :param s_min:
    :param s_max:
    :param d_min:
    :param d_max:
    :param desired_v:
    :param current_state_long: np.array([s, s_dot, s_ddot])
    :param current_state_lat: np.array([d, d_dot, d_ddot])
    :param current_time_idx: discrete time index
    :return: trajectory bundle with all sample trajectories.

    NOTE: Here, no collision or feasibility check is done!
    """

    params = parameter_position_reaching()

    trajectory_bundle = TrajectoryBundle(params)

    desired_s = (s_min + s_max) / 2.0
    desired_d = (d_min + d_max) / 2.0

    if current_state_long[0] > s_min:
        s_min = current_state_long[0] + params.long_step_size

    for T in range(int(params.t_step_size / dt),
                   int(params.prediction_horizon / dt) + int(params.t_step_size / dt),
                   int(params.t_step_size / dt)):
        t = T*dt
        for d in np.arange(d_min, d_max + params.lat_step_size, params.lat_step_size):

            end_state_y = np.array([d, 0.0, 0.0])
            trajectory_lat = QuinticTrajectory(t_start_s=current_time_idx*dt, duration_s=t,
                                               start_state=current_state_lat, end_state=end_state_y)
            jerk_cost = trajectory_lat.squared_jerk_integral(t)/t
            time_cost = 0  # 1.0/t
            distance_cost = (d - desired_d)**2
            trajectory_lat.set_cost(jerk_cost, time_cost, distance_cost,
                                    params.k_jerk_lat, params.k_time, 50*params.k_distance)

            for s in np.arange(s_min, s_max + params.long_step_size, params.long_step_size):
                end_state_x = np.array([s, desired_v, 0.0])
                trajectory_long = QuinticTrajectory(t_start_s=current_time_idx*dt, duration_s=t,
                                                    start_state=current_state_long, end_state=end_state_x)
                jerk_cost = trajectory_long.squared_jerk_integral(t)/t
                time_cost = 1.0/t
                distance_cost = (s - desired_s)**2
                trajectory_long.set_cost(jerk_cost, time_cost, distance_cost,
                                         params.k_jerk_lon, params.k_time, params.k_distance)

                trajectory_cost = params.k_long * trajectory_long.cost + params.k_lat * trajectory_lat.cost
                trajectory_sample = TrajectorySample(dt, trajectory_long, trajectory_lat, trajectory_cost)
                trajectory_bundle.add_trajectory(trajectory_sample)

    return trajectory_bundle
