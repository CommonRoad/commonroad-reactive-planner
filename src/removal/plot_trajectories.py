import os
import ntpath
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet

from parameter import draw_params_scenario
from typing import List, Union

def plot_scenario_at_time_idx(time_idx: int, scenario: Scenario):
    """
    Plots a scenario at a specific point in time.
    :param time_idx: The time point for which the scenario will be plotted
    :param scenario: The scenario to be plotted
    :return:
    """
    draw_params_scenario['time_begin'] = time_idx
    draw_params_scenario['time_end'] = time_idx
    draw_params_scenario['scenario']['dynamic_obstacle']['shape']['rectangle']['facecolor'] = '#ff4000'
    draw_params_scenario['scenario']['dynamic_obstacle']['shape']['rectangle']['edgecolor'] = '#cc3300'
    draw_params_scenario['scenario']['dynamic_obstacle']['occupancy']['shape']['polygon']['opacity'] = .1
    draw_object(scenario, draw_params=draw_params_scenario)


def plot_vehicle_at_time_idx(time_idx: int, trajectory: Trajectory):
    """
    Plots the occupancy of a vehicle at a specific time point given its trajectory.
    :param time_idx: The time point for which the occupancy will be plotted
    :param trajectory: The trajectory of the vehicle whose occupancy will be plotted
    :return:
    """
    ego_occupancy = TrajectoryPrediction(0, trajectory, Rectangle(5, 2)).occupancy_at_time_idx(time_idx)
    draw_params_scenario['scenario']['dynamic_obstacle']['occupancy']['shape']['rectangle']['opacity'] = 1
    draw_object(ego_occupancy, draw_params=draw_params_scenario['scenario']['dynamic_obstacle'])


def create_video(out_path: str, scenario: Scenario, ego_trajectory: Trajectory,
                 planning_problem_set: PlanningProblemSet=None):
    """
    Creates a video of the solution for a specific planning problem.
    :param out_path: The path where the video will be saved.
    :param scenario: The scenario of the planning problem that was solved.
    :param ego_trajectory: The trajectory of the ego vehicle that solves the planning problem
    :param planning_problem_set: The planning problem set in which the planning problem belongs.
    :return:
    """
    assert(os.path.isdir(os.path.dirname(os.path.abspath(out_path)))), \
        'Directory %s does not exist' % os.path.dirname(os.path.abspath(out_path))
    filename = ntpath.basename(out_path)

    ffmpeg_writer = animation.writers['ffmpeg']
    metadata = dict(title=filename, artist='Matplotlib')
    writer = ffmpeg_writer(fps=10, metadata=metadata)

    fig = plt.figure(figsize=(25, 10))
    plt.xlabel('[m]')
    plt.ylabel('[m]')
    plt.title(filename)

    # find figure size
    x = [x for lanelet in scenario.lanelet_network.lanelets for x in lanelet.center_vertices[:, 0]]
    y = [y for lanelet in scenario.lanelet_network.lanelets for y in lanelet.center_vertices[:, 1]]
    x_min = min(x) - 10
    y_min = min(y) - 10
    x_max = max(x) + 5
    y_max = max(y) + 5

    if os.path.isfile(out_path):
        os.remove(out_path)
    with writer.saving(fig, out_path, dpi=150):
        for t in [state.time_idx for state in ego_trajectory.state_list]:
            plt.cla()
            plot_scenario_at_time_idx(t, scenario)
            draw_object(planning_problem_set)
            plot_vehicle_at_time_idx(t, ego_trajectory)
            plt.gca().set_aspect('equal')
            plt.gca().set_xlim([x_min, x_max])
            plt.gca().set_ylim([y_min, y_max])
            writer.grab_frame()

def create_video_replanning(out_path: str, scenario: Scenario, ego_trajectory: Trajectory,
                            desired_trajectories: List[Trajectory], fail_safe_trajectories: Union[None,List[Trajectory]],
                            planning_problem_set: PlanningProblemSet=None, plot_planning_problem: bool=False):
    """
    Creates a video of the solution for a specific planning problem.
    :param out_path: The path where the video will be saved.
    :param scenario: The scenario of the planning problem that was solved.
    :param ego_trajectory: actually driven trajectory of ego vehicle
    :param desired_trajectories: desired trajectories from current states till goal state (one for every every time step, where replanning started)
    :param fail_safe_trajectories: List of fail safe trajectories (one for every every time step, where replanning started), set to None if should not be plotted
    :param planning_problem_set: The planning problem set in which the planning problem belongs.
    :param plot_planning_problem: defines if planning problem is plotted
    :return:
    """
    assert(os.path.isdir(os.path.dirname(os.path.abspath(out_path)))), \
        'Directory %s does not exist' % os.path.dirname(os.path.abspath(out_path))
    filename = ntpath.basename(out_path)

    ffmpeg_writer = animation.writers['ffmpeg']
    metadata = dict(title=filename, artist='Matplotlib')
    writer = ffmpeg_writer(fps=10, metadata=metadata)

    fig = plt.figure(figsize=(25, 10))
    plt.xlabel('[m]')
    plt.ylabel('[m]')
    plt.title(filename)

    # find figure size
    x = [x for lanelet in scenario.lanelet_network.lanelets for x in lanelet.center_vertices[:, 0]]
    y = [y for lanelet in scenario.lanelet_network.lanelets for y in lanelet.center_vertices[:, 1]]
    x_min = min(x) - 10
    y_min = min(y) - 10
    x_max = max(x) + 5
    y_max = max(y) + 5

    if os.path.isfile(out_path):
        os.remove(out_path)
    with writer.saving(fig, out_path, dpi=150):
        for t, desired_traj_t in enumerate(desired_trajectories):
            plt.cla()
            plt.clf()
            plot_scenario_at_time_idx(t, scenario)
            if plot_planning_problem is True:
                draw_object(planning_problem_set)
            # draw vehicle at current time step
            plot_vehicle_at_time_idx(t, ego_trajectory)
            # draw desired trajectory to goal
            draw_object(desired_traj_t)
            # draw current fail-safe trajectory
            draw_object(fail_safe_trajectories[t])

            plt.gca().set_aspect('equal')
            plt.gca().set_xlim([x_min, x_max])
            plt.gca().set_ylim([y_min, y_max])
            writer.grab_frame()

def plot_sequence(out_dir: str, scenario: Scenario, planning_problem: PlanningProblem,
                  ego_trajectory: Trajectory=None, fail_safe_trajectory: Trajectory=None):
    """
    Plots the main trajectory and the fail-safe trajectory combined together.
    :param out_dir: The path where the plot will be saved.
    :param scenario: The scenario for which the trajectories where calculated.
    :param planning_problem: The planning problem that the trajectories solve.
    :param ego_trajectory: The main trajectory of the ego vehicle.
    :param fail_safe_trajectory: The fail-safe trajectory that was calculated for the ego vehicle.
    :return:
    """
    assert(os.path.isdir(out_dir)), 'Directory %s does not exist' % out_dir
    # plot ego trajectory
    if ego_trajectory:
        for t in [state.time_idx for state in ego_trajectory.state_list]:
            plt.figure(figsize=(25, 10))
            plot_scenario_at_time_idx(t, scenario)
            draw_object(planning_problem)
            plot_vehicle_at_time_idx(t, ego_trajectory)

            plt.gca().set_aspect('equal')
            plt.savefig(os.path.join(out_dir, 'ego_trajectory_' + str(t) + '.svg'))  # for eps:, dpi=300, bbox_inches='tight'
            plt.clf()

    # plot fail-safe trajectory
    if fail_safe_trajectory:
        for t in [state.time_idx for state in fail_safe_trajectory.state_list]:
            plt.figure(figsize=(25, 10))
            plot_scenario_at_time_idx(t, scenario)
            draw_object(planning_problem)
            plot_vehicle_at_time_idx(t, fail_safe_trajectory)

            plt.gca().set_aspect('equal')
            plt.savefig(os.path.join(out_dir, 'fail_safe_trajectory_' + str(t) + '.svg'))
            plt.clf()
