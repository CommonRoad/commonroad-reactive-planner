__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2025.1"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Beta"

import multiprocessing
from copy import deepcopy
# standard imports
from typing import List, Union, Optional, Tuple
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
from commonroad.visualization.draw_params import OccupancyParams
from commonroad.visualization.mp_renderer import MPRenderer, DynamicObstacleParams, ShapeParams
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import Occupancy

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
    FeasibilityStatus.INFEASIBLE_COLLISION.name: 'red',
    FeasibilityStatus.INFEASIBLE_RULE.name: 'red'
}


def visualize_scenario_and_pp(
        scenario: Scenario,
        planning_problem: PlanningProblem,
        cosy=None
) -> None:
    """Visualizes scenario, planning problem and (optionally) the reference path"""
    plot_limits = None
    ref_path = None
    if cosy is not None:
        ref_path = cosy.reference
        x_min = np.min(ref_path[:, 0]) - 20
        x_max = np.max(ref_path[:, 0]) + 20
        y_min = np.min(ref_path[:, 1]) - 20
        y_max = np.max(ref_path[:, 1]) + 20
        plot_limits = [x_min, x_max, y_min, y_max]

    rnd = MPRenderer(figsize=(20, 10), plot_limits=plot_limits)
    rnd.draw_params.time_begin = 0
    rnd.draw_params.dynamic_obstacle.draw_icon = True
    scenario.draw(rnd)
    planning_problem.draw(rnd)
    rnd.render()
    if ref_path is not None:
        rnd.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=100,
                    linewidth=0.8, label='reference path')
        proj_domain_border = np.array(cosy.ccosy.projection_domain())
        rnd.ax.plot(proj_domain_border[:, 0], proj_domain_border[:, 1], color="orange", linewidth=0.8, zorder=100)
    plt.show(block=True)


def visualize_collision_checker(
        scenario: Scenario,
        cc: pycrcc.CollisionChecker
) -> None:
    """
    Visualizes the collision checker, i.e., all collision objects and, if applicable, the road boundary.
    :param scenario CommonRoad scenario object
    :param cc pycrcc.CollisionChecker object
    """
    rnd = MPRenderer(figsize=(20, 10))
    scenario.lanelet_network.draw(rnd)
    cc.draw(rnd)
    rnd.render(show=True)


def visualize_planner_at_timestep(
        scenario: Scenario,
        planning_problem: PlanningProblem,
        ego: DynamicObstacle,
        timestep: int,
        config: ReactivePlannerConfiguration,
        traj_set: List[TrajectorySample] = None,
        ref_path: np.ndarray = None,
        rnd: MPRenderer = None,
        plot_limits: Union[List[Union[int, float]], None] = None
) -> None:
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
    :param plot_limits: x, y-axis limits for plotting
    """
    # get plot limits from ref path
    if plot_limits is None and ref_path is not None:
        x_min = np.min(ref_path[:, 0]) - 20
        x_max = np.max(ref_path[:, 0]) + 20
        y_min = np.min(ref_path[:, 1]) - 20
        y_max = np.max(ref_path[:, 1]) + 20
        plot_limits = [x_min, x_max, y_min, y_max]

    # create renderer object (if no existing renderer is passed)
    if rnd is None:
        rnd = MPRenderer(figsize=(20, 10), plot_limits=plot_limits)
    else:
        rnd.plot_limits = plot_limits

    # set renderer draw params
    rnd.draw_params.time_begin = timestep
    rnd.draw_params.dynamic_obstacle.draw_icon = config.debug.draw_icons
    rnd.draw_params.dynamic_obstacle.trajectory.draw_trajectory = False
    rnd.draw_params.planning_problem.initial_state.state.draw_arrow = False
    rnd.draw_params.planning_problem.initial_state.state.radius = 0.5

    # set ego vehicle draw params
    ego_params = DynamicObstacleParams()
    ego_params.time_begin = timestep
    ego_params.time_end = 1000
    ego_params.draw_icon = config.debug.draw_icons
    ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"
    ego_params.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
    ego_params.vehicle_shape.occupancy.shape.zorder = 50
    ego_params.vehicle_shape.occupancy.shape.opacity = 1
    ego_params.trajectory.draw_trajectory = False

    # visualize scenario, planning problem, ego vehicle
    scenario.draw(rnd)
    if config.debug.draw_planning_problem:
        planning_problem.draw(rnd)
    ego.draw(rnd, draw_params=ego_params)
    rnd.render()

    # visualize optimal trajectory
    pos = np.asarray([state.position for state in ego.prediction.trajectory.state_list])
    rnd.ax.plot(pos[:, 0], pos[:, 1], color='k', marker='x', markersize=2.5, zorder=21, linewidth=2.0,
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
        rnd.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19, linewidth=1.8,
                    label='reference path')

    plt.xlabel("x [m]", fontsize=22)
    plt.ylabel("y [m]", fontsize=22)
    plt.xticks(fontsize=22)
    plt.yticks(fontsize=22)

    # save as image
    if config.debug.save_plots:
        os.makedirs(os.path.join(config.general.path_output, str(scenario.scenario_id)),
                    exist_ok=True)
        plot_dir = os.path.join(config.general.path_output, str(scenario.scenario_id))
        plt.savefig(f"{plot_dir}/{scenario.scenario_id}_{timestep}.{config.debug.plots_file_format}",
                    format=config.debug.plots_file_format, dpi=300,
                    bbox_inches='tight')

    # show plot
    if config.debug.show_plots:
        plt.show(block=True)


def plot_final_trajectory(
        scenario: Scenario,
        planning_problem: PlanningProblem,
        state_list: List[CustomState],
        config: ReactivePlannerConfiguration,
        ref_path: np.ndarray = None,
        plot_limits: Optional[List[Union[int, float]]] = None,
        ego_vehicle: Optional[DynamicObstacle] = None,
        time_step: int = 0,
        rnd: Optional[MPRenderer] = None
) -> None:
    """
    Function plots occupancies for a given CommonRoad trajectory (of the ego vehicle)
    :param scenario: CommonRoad scenario object
    :param planning_problem CommonRoad Planning problem object
    :param state_list: List of trajectory States
    :param config: Configuration object for plot/save settings
    :param ref_path: Reference path as [(nx2) np.ndarray] (optional)
    :param plot_limits: limits of map plotting
    :param ego_vehicle: Ego Vehicle as CR Dynamic Obstacle type
    :param time_step: the time step to visualize the scenario (default 0)
    :param rnd: MPRenderer (optional: if none is passed, the function creates a new renderer object; otherwise it
    will visualize on the existing object)
    """
    # get plot limits from trajectory
    if plot_limits is None:
        position_array = np.array([state.position for state in state_list])
        x_min = np.min(position_array[:, 0]) - 20
        x_max = np.max(position_array[:, 0]) + 20
        y_min = np.min(position_array[:, 1]) - 20
        y_max = np.max(position_array[:, 1]) + 20
        plot_limits = [x_min, x_max, y_min, y_max]

    # create renderer object (if no existing renderer is passed)
    if rnd is None:
        rnd = MPRenderer(figsize=(20, 10), plot_limits=plot_limits)
    else:
        rnd.plot_limits = plot_limits

    # set renderer draw params
    rnd.draw_params.time_begin = time_step
    rnd.draw_params.planning_problem.initial_state.state.draw_arrow = False
    rnd.draw_params.planning_problem.initial_state.state.radius = 0.5
    rnd.draw_params.dynamic_obstacle.draw_icon = config.debug.draw_icons
    rnd.draw_params.dynamic_obstacle.trajectory.draw_trajectory = False

    # set ego trajectory occupancy shape params
    occ_params = OccupancyParams()
    occ_params.shape.facecolor = '#E37222'
    occ_params.shape.edgecolor = '#9C4100'
    occ_params.shape.opacity = 0.2
    occ_params.shape.zorder = 20

    # set ego vehicle draw params
    ego_params = DynamicObstacleParams()
    ego_params.draw_icon = config.debug.draw_icons
    ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"
    ego_params.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
    ego_params.vehicle_shape.direction.zorder = 55
    ego_params.trajectory.draw_trajectory = False
    ego_params.time_begin = time_step

    # visualize scenario
    scenario.draw(rnd)
    # visualize planning problem
    if config.debug.draw_planning_problem:
        planning_problem.draw(rnd)

    # visualize ego trajectory occupancies
    if ego_vehicle is not None:
        # draw occupancies of ego vehicle trajectory
        [occ.draw(rnd, draw_params=occ_params) for occ in ego_vehicle.prediction.occupancy_set]

        # visualize ego vehicle at specified time step
        ego_vehicle.draw(rnd, draw_params=ego_params)
    else:
        # visualize trajectory occupancies from state list directly
        for i in range(0, len(state_list)):
            # Create occupancy from state
            state = state_list[i]
            shape_rect = Rectangle(
                length=config.vehicle.length,
                width=config.vehicle.width,
                center=state.position,
                orientation=state.orientation
            )
            occ = Occupancy(time_step=i, shape=shape_rect)

            # draw occupancies
            if i == 0:
                occ_params.shape.opacity = 1.0
                occ_params.shape.zorder = 55
            else:
                occ_params.shape.opacity = 0.2
                occ_params.shape.zorder = 20
            occ.draw(rnd, draw_params=occ_params)

    # visualize occupancies of other trajectories
    if config.debug.draw_occupancies_other:
        dyn_obs_params = OccupancyParams()
        dyn_obs_params.shape.opacity = 0.1
        for obs in scenario.dynamic_obstacles:
            for o in obs.prediction.occupancy_set:
                o.draw(rnd, draw_params=dyn_obs_params)

    # render scenario and occupancies
    rnd.render()

    # visualize ego trajectory states
    pos = np.asarray([state.position for state in state_list])
    rnd.ax.plot(pos[:, 0], pos[:, 1], color='#9C4100', marker='x', markersize=3.0, markeredgewidth=0.4, zorder=21,
                linewidth=4.0)

    # visualize reference path
    if ref_path is not None and config.debug.draw_ref_path:
        rnd.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19, linewidth=0.8,
                    label='reference path')

    # save as image
    if config.debug.save_plots:
        os.makedirs(os.path.join(config.general.path_output, str(scenario.scenario_id)),
                    exist_ok=True)
        plot_dir = os.path.join(config.general.path_output, str(scenario.scenario_id))
        plt.savefig(
            f"{plot_dir}/{scenario.scenario_id}_final_trajectory.{config.debug.plots_file_format}",
            format=config.debug.plots_file_format,
            dpi=300,
            bbox_inches='tight'
        )

    # show plot
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    if config.debug.show_plots:
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
            im_path = os.path.join(path_images, str(scenario_id) + f"_{step}.{config.debug.plots_file_format}")
            filenames.append(im_path)

        for filename in filenames:
            images.append(imageio.v2.imread(filename))

        imageio.mimsave(os.path.join(config.general.path_output, str(scenario_id) + ".gif"),
                        images, duration=duration)


def worker(task_queue):
    """
    Worker function for multiprocessing.

    :param task_queue Queue
    """
    while True:
        task: VisualizationTask = task_queue.get()  # Blocking until a task is available
        if task is None:
            break
        visualize(task)


def visualize(task):
    """
    Function to visualize planning result from the reactive planner for a given time step

    :param task: VisualizationTask including all relevant info
    """
    # get plot limits from ref path
    if task.plot_limits is None and task.ref_path is not None:
         x_min = np.min(task.ref_path[:, 0]) - 20
         x_max = np.max(task.ref_path[:, 0]) + 20
         y_min = np.min(task.ref_path[:, 1]) - 20
         y_max = np.max(task.ref_path[:, 1]) + 20
         plot_limits = [x_min, x_max, y_min, y_max]

    # create renderer object (if no existing renderer is passed)
    if task.rnd is None:
        task.rnd = MPRenderer(figsize=(20, 10), plot_limits=task.plot_limits)

    # set renderer draw params
    task.rnd.draw_params.time_begin = task.timestep
    task.rnd.draw_params.dynamic_obstacle.draw_icon = task.draw_icons
    task.rnd.draw_params.planning_problem.initial_state.state.draw_arrow = False
    task.rnd.draw_params.planning_problem.initial_state.state.radius = 0.5

    # set ego vehicle draw params
    ego_params = DynamicObstacleParams()
    ego_params.time_begin = task.timestep
    ego_params.time_end = 1000
    ego_params.draw_icon = task.draw_icons
    ego_params.vehicle_shape.occupancy.shape.facecolor = "#E37222"
    ego_params.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
    ego_params.vehicle_shape.occupancy.shape.zorder = 50
    ego_params.vehicle_shape.occupancy.shape.opacity = 1

    # visualize scenario, planning problem, ego vehicle
    task.scenario.draw(task.rnd)
    if task.draw_planning_problem:
        task.planning_problem.draw(task.rnd)
    task.ego_vehicle.draw(task.rnd, draw_params=ego_params)
    task.rnd.render()

    # visualize optimal trajectory
    pos = np.asarray([state.position for state in task.ego_vehicle.prediction.trajectory.state_list])
    task.rnd.ax.plot(pos[:, 0], pos[:, 1], color='k', marker='x', markersize=1.5, zorder=21, linewidth=1.5,
                     label='optimal trajectory')

    # visualize sampled trajectory bundle
    step = 1  # draw every trajectory (step=2 would draw every second trajectory)
    if task.traj_set is not None:
        for i in range(0, len(task.traj_set), step):
            color = _dict_traj_status_to_color[task.traj_set[i].feasibility_label.name]
            plt.plot(task.traj_set[i].cartesian.x, task.traj_set[i].cartesian.y,
                     color=color, zorder=20, linewidth=0.1, alpha=1.0)

    # visualize reference path
    if task.ref_path is not None and task.draw_ref_path:
        task.rnd.ax.plot(task.ref_path[:, 0], task.ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19,
                         linewidth=1.8,
                         label='reference path')

    plt.xlabel("x [m]", fontsize=22)
    plt.ylabel("y [m]", fontsize=22)
    plt.xticks(fontsize=22)
    plt.yticks(fontsize=22)

    # save as image file
    if task.save_plots:
        os.makedirs(os.path.join(task.path_output, str(task.scenario.scenario_id)),
                    exist_ok=True)
        plot_dir = os.path.join(task.path_output, str(task.scenario.scenario_id))
        plt.savefig(f"{plot_dir}/{task.scenario.scenario_id}_{task.timestep}.{task.file_format}",
                    format=task.file_format, dpi=300,
                    bbox_inches='tight')

    # show plot
    if task.show_plots:
        plt.show(block=True)

class VisualizationTask:
    """
    Class to collect all relevant info to visualize planning result from the reactive planner for a given time step
    """
    def __init__(self, scenario: Scenario, planning_problem: PlanningProblem, ego: DynamicObstacle,
                                  timestep: int, config: ReactivePlannerConfiguration, traj_set: List[TrajectorySample] = None,
                                  monitor_set: Tuple[List[TrajectorySample], List[bool]] = None,
                                  ref_path: np.ndarray = None, rnd: MPRenderer = None,
                                  plot_limits: Union[List[Union[int, float]], None] = None,
                                  medium_goal: np.ndarray = None):
        self.scenario = deepcopy(scenario)
        self.planning_problem = deepcopy(planning_problem)
        self.ego_vehicle = deepcopy(ego)
        self.traj_set = deepcopy(traj_set)
        self.monitor_set = deepcopy(monitor_set)
        self.ref_path = deepcopy(ref_path)
        self.timestep = timestep
        self.rnd = deepcopy(rnd)
        self.medium_goal = deepcopy(medium_goal)
        self.plot_limits = plot_limits

        # config
        self.draw_icons = config.debug.draw_icons
        self.draw_planning_problem = config.debug.draw_planning_problem
        self.draw_ref_path = config.debug.draw_ref_path
        self.save_plots = config.debug.save_plots
        self.path_output = config.general.path_output
        self.show_plots = config.debug.show_plots
        self.file_format = config.debug.plots_file_format

class VisualizationHandler:
    """
    Clss responsible for handling (multiprocessing) all visualization tasks
    """

    def __init__(self, config: ReactivePlannerConfiguration):
        self.task_queue = multiprocessing.Queue(maxsize=config.debug.max_queue_size)
        self.num_workers = config.debug.num_workers_viz
        if self.num_workers < 0:
            raise ValueError('num_workers_viz must be greater than -1. Choose a value greater than 0 to activate multiprocessing')
        self.workers = []
        for i in range(self.num_workers):
            p = multiprocessing.Process(target=worker, args=(self.task_queue,))
            p.start()
            self.workers.append(p)

    def add_task(self, scenario: Scenario, planning_problem: PlanningProblem,
                 ego_vehicle: DynamicObstacle, traj_set: List[TrajectorySample],
                 ref_path: np.ndarray, timestep: int,
                 config: ReactivePlannerConfiguration, medium_goal: np.ndarray = None,
                 monitor_set: Tuple[List[TrajectorySample], List[bool]] = None):
        """
        add a task to the queue for visualization

        :param scenario: Scenario
        :param planning_problem: PlanningProblem
        :param ego_vehicle: DynamicObstacle
        :param traj_set: TrajectorySample
        :param monitor_set: TrajectorySample
        :param ref_path: np.ndarray
        :param timestep: int
        :param config: ReactivePlannerConfiguration
        :param medium_goal: ndarray
        """
        if self.num_workers != 0:
            task = VisualizationTask(scenario=scenario,
                                     planning_problem=planning_problem,
                                     ego=ego_vehicle,
                                     timestep=timestep,
                                     config=config,
                                     traj_set=traj_set,
                                     monitor_set=monitor_set,
                                     ref_path=ref_path,
                                     medium_goal=medium_goal)
            self.task_queue.put(task, block=False)
        else:
            visualize(VisualizationTask(scenario=scenario,
                                     planning_problem=planning_problem,
                                     ego=ego_vehicle,
                                     timestep=timestep,
                                     config=config,
                                     traj_set=traj_set,
                                     monitor_set=monitor_set,
                                     ref_path=ref_path,
                                     medium_goal=medium_goal))

    def __del__(self):
        for i in range(self.num_workers):
            self.task_queue.put(None)

        for p in self.workers:
            p.join()
