__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "1.0"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Beta"


# standard imports
from typing import List, Union
import os
import logging

# third party
import matplotlib.pyplot as plt
import numpy as np
import imageio

# commonroad-io
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.state import CustomState
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.visualization.mp_renderer import MPRenderer, DynamicObstacleParams, ShapeParams
from commonroad.geometry.shape import Rectangle

# commonroad_dc
from commonroad_dc import pycrcc

# commonroad-rp
from commonroad_rp.trajectories import TrajectorySample, FeasibilityStatus
from commonroad_rp.utility.config import ReactivePlannerConfiguration


logger = logging.getLogger("RP_LOGGER")
logging.getLogger('PIL').setLevel(logging.ERROR)
logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)

# color dict of trajectories
_dict_traj_status_to_color = {
    FeasibilityStatus.FEASIBLE.name: 'blue',
    FeasibilityStatus.INFEASIBLE_KINEMATIC.name: 'blue',
    FeasibilityStatus.INFEASIBLE_COLLISION.name: 'blue'
}


def visualize_scenario_and_pp(scenario: Scenario, planning_problem: PlanningProblem, cosy=None):
    """Visualizes scenario, planning problem and (optionally) the reference path"""
    plot_limits = None
    ref_path = None
    if cosy is not None:
        ref_path = cosy.reference
        x_min = np.min(ref_path[:, 0]) - 50
        x_max = np.max(ref_path[:, 0]) + 50
        y_min = np.min(ref_path[:, 1]) - 50
        y_max = np.max(ref_path[:, 1]) + 50
        plot_limits = [x_min, x_max, y_min, y_max]

    rnd = MPRenderer(figsize=(20, 10), plot_limits=plot_limits)
    rnd.draw_params.time_begin = 0
    scenario.draw(rnd)
    planning_problem.draw(rnd)
    rnd.render()
    if ref_path is not None:
        rnd.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19,
                    linewidth=0.8, label='reference path')
        proj_domain_border = np.array(cosy.ccosy.projection_domain())
        rnd.ax.plot(proj_domain_border[:, 0], proj_domain_border[:, 1], color="orange", linewidth=0.8)
    plt.show(block=True)


def visualize_collision_checker(scenario: Scenario, cc: pycrcc.CollisionChecker):
    """
    Visualizes the collision checker, i.e., all collision objects and, if applicable, the road boundary.
    :param scenario CommonRoad scenario object
    :param cc pycrcc.CollisionChecker object
    """
    rnd = MPRenderer(figsize=(20, 10))
    scenario.lanelet_network.draw(rnd)
    cc.draw(rnd)
    rnd.render(show=True)


def visualize_planner_at_timestep(scenario: Scenario, planning_problem: PlanningProblem, ego: DynamicObstacle,
                                  timestep: int, config: ReactivePlannerConfiguration, traj_set: List[TrajectorySample] = None,
                                  ref_path: np.ndarray = None, rnd: MPRenderer = None,
                                  plot_limits: Union[List[Union[int, float]], None] = None):
    """
    Function to visualize planning result from the reactive planner for a given time step
    :param scenario: CommonRoad scenario object
    :param planning_problem CommonRoad Planning problem object
    :param ego: Ego vehicle as CommonRoad DynamicObstacle object
    :param timestep: current time step of scenario to plot
    :param config: Configuration object for plot/save settings
    :param traj_set: List of sampled trajectories (optional)
    :param ref_path: Reference path for planner as polyline [(nx2) np.ndarray] (optional)
    :param rnd: MPRenderer object (optional: if none is passed, the function creates a new renderer object; otherwise it
    will visualize on the existing object)
    :param plot_limits: x, y axis limits for plotting
    """
    # create renderer object (if no existing renderer is passed)
    if rnd is None:
        rnd = MPRenderer(figsize=(20, 10), plot_limits=plot_limits)

    # set renderer draw params
    rnd.draw_params.time_begin = timestep
    rnd.draw_params.dynamic_obstacle.draw_icon = config.debug.draw_icons
    rnd.draw_params.planning_problem.initial_state.state.draw_arrow = False
    rnd.draw_params.planning_problem.initial_state.state.radius = 0.5

    # set ego vehicle draw params
    ego_params = DynamicObstacleParams()
    ego_params.time_begin = timestep
    ego_params.draw_icon = config.debug.draw_icons
    ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"
    ego_params.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
    ego_params.vehicle_shape.occupancy.shape.zorder = 50
    ego_params.vehicle_shape.occupancy.shape.opacity = 1

    # visualize scenario, planning problem, ego vehicle
    scenario.draw(rnd)
    if config.debug.draw_planning_problem:
        planning_problem.draw(rnd)
    ego.draw(rnd, draw_params=ego_params)
    rnd.render()

    # visualize optimal trajectory
    pos = np.asarray([state.position for state in ego.prediction.trajectory.state_list])
    rnd.ax.plot(pos[:, 0], pos[:, 1], color='k', marker='x', markersize=1.5, zorder=21, linewidth=1.5,
                label='optimal trajectory')

    # visualize sampled trajectory bundle
    step = 1  # draw every trajectory (step=2 would draw every second trajectory)
    if traj_set is not None:
        for i in range(0, len(traj_set), step):
            color = _dict_traj_status_to_color[traj_set[i].feasibility_label.name]
            plt.plot(traj_set[i].cartesian.x, traj_set[i].cartesian.y,
                     color=color, zorder=20, linewidth=0.1, alpha=1.0)

    # visualize reference path
    if ref_path is not None and config.debug.draw_ref_path:
        rnd.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19, linewidth=0.8,
                    label='reference path')

    # save as .png file
    if config.debug.save_plots:
        os.makedirs(os.path.join(config.general.path_output, str(scenario.scenario_id)),
                    exist_ok=True)
        plot_dir = os.path.join(config.general.path_output, str(scenario.scenario_id))
        plt.savefig(f"{plot_dir}/{scenario.scenario_id}_{timestep}.png", format='png', dpi=300,
                    bbox_inches='tight')

    # show plot
    if config.debug.show_plots:
        plt.show(block=True)


def plot_final_trajectory(scenario: Scenario, planning_problem: PlanningProblem, state_list: List[CustomState],
                          config: ReactivePlannerConfiguration, ref_path: np.ndarray = None):
    """
    Function plots occupancies for a given CommonRoad trajectory (of the ego vehicle)
    :param scenario: CommonRoad scenario object
    :param planning_problem CommonRoad Planning problem object
    :param state_list: List of trajectory States
    :param config: Configuration object for plot/save settings
    :param ref_path: Reference path as [(nx2) np.ndarray] (optional)
    :param save_path: Path to save plot as .png (optional)
    """
    # create renderer object (if no existing renderer is passed)
    rnd = MPRenderer(figsize=(20, 10))

    # set renderer draw params
    rnd.draw_params.time_begin = 0
    rnd.draw_params.planning_problem.initial_state.state.draw_arrow = False
    rnd.draw_params.planning_problem.initial_state.state.radius = 0.5

    # set occupancy shape params
    occ_params = ShapeParams()
    occ_params.facecolor = '#E37222'
    occ_params.edgecolor = '#9C4100'
    occ_params.opacity = 1.0
    occ_params.zorder = 51

    # visualize scenario
    scenario.draw(rnd)
    # visualize planning problem
    if config.debug.draw_planning_problem:
        planning_problem.draw(rnd)
    # visualize occupancies of trajectory
    for i in range(len(state_list)):
        state = state_list[i]
        occ_pos = Rectangle(length=config.vehicle.length, width=config.vehicle.width, center=state.position,
                            orientation=state.orientation)
        if i >= 1:
            occ_params.opacity = 0.3
            occ_params.zorder = 50
        occ_pos.draw(rnd, draw_params=occ_params)
    # render scenario and occupancies
    rnd.render()

    # visualize trajectory
    pos = np.asarray([state.position for state in state_list])
    rnd.ax.plot(pos[:, 0], pos[:, 1], color='k', marker='x', markersize=3.0, markeredgewidth=0.4, zorder=21,
                linewidth=0.8)

    # visualize reference path
    if ref_path is not None and config.debug.draw_ref_path:
        rnd.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19, linewidth=0.8,
                    label='reference path')

    # save as .png file
    if config.debug.save_plots:
        os.makedirs(os.path.join(config.general.path_output, str(scenario.scenario_id)),
                    exist_ok=True)
        plot_dir = os.path.join(config.general.path_output, str(scenario.scenario_id))
        plt.savefig(f"{plot_dir}/{scenario.scenario_id}_final_trajectory.png", format='png', dpi=300,
                    bbox_inches='tight')

    # show plot
    if True:
        plt.show(block=True)


def make_gif(config: ReactivePlannerConfiguration, time_steps: Union[range, List[int]], duration: float = 0.1):
    """
    Function to create GIF from single images of planning results at each time step
    Images are saved in output path specified in config.general.path_output
    :param config Configuration object
    :param scenario CommonRoad scenario object
    :param time_steps list or range of time steps to create the GIF
    :param duration
    """
    if not config.debug.save_plots:
        # only create GIF when saving of plots is enabled
        print("...GIF not created: Enable config.debug.save_plots to generate GIF.")
        pass
    else:
        print("...Generating GIF")
        images = []
        filenames = []

        scenario_id = config.scenario.scenario_id

        # directory, where single images are outputted (see visualize_planner_at_timestep())
        path_images = os.path.join(config.general.path_output, str(scenario_id))

        for step in time_steps:
            im_path = os.path.join(path_images, str(scenario_id) + "_{}.png".format(step))
            filenames.append(im_path)

        for filename in filenames:
            images.append(imageio.imread(filename))

        imageio.mimsave(os.path.join(config.general.path_output, str(scenario_id) + ".gif"),
                        images, duration=duration)
