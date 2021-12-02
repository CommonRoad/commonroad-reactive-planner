__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "0.5"
__maintainer__ = "Gerald Würsching"
__email__ = "gerald.wuersching@tum.de"
__status__ = "Beta"


# standard imports
from typing import List

# third party
import matplotlib.pyplot as plt
import numpy as np

# commonroad-io
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.visualization.mp_renderer import MPRenderer

# commonroad-rp
from commonroad_rp.trajectories import TrajectorySample


def visualize_planning_result(scenario: Scenario, planning_problem: PlanningProblem, ego: DynamicObstacle,
                              pos: np.ndarray, timestep: int, traj_set: List[TrajectorySample] = None,
                              ref_path: np.ndarray = None, save_path: str = None):
    """
    Function to visualize complete planning result from the reactive planner for a given time step
    :param scenario: CommonRoad scenario object
    :param: planning_problem: CommonRoad PlanningProblem object
    :param ego: Ego vehicle as CommonRoad DynamicObstacle object
    :param pos: positions of planned trajectory [(nx2) np.ndarray]
    :param timestep: current time step of scenario to plot
    :param traj_set: List of sampled trajectories (optional)
    :param ref_path: Reference path for planner as polyline [(nx2) np.ndarray] (optional)
    :param save_path: Path to save plot as .png (optional)
    """
    # create renderer object
    rnd = MPRenderer(figsize=(20, 10))
    # visualize scenario
    scenario.draw(rnd, draw_params={'time_begin': timestep})
    # visualize planning problem
    planning_problem.draw(rnd, draw_params={"initial_state": {"state": {"draw_arrow": False}}})
    # visualize ego vehicle
    ego.draw(rnd, draw_params={"time_begin": timestep,
                               "dynamic_obstacle": {
                                   "vehicle_shape": {
                                       "occupancy": {
                                           "shape": {
                                               "rectangle": {
                                                   "facecolor": "#E37222",
                                                   "edgecolor": '#E37222',
                                                   "zorder": 50,
                                                   "opacity": 1
                                               }
                                           }
                                       }
                                   }
                               }
                               })
    # render scenario and ego vehicle
    rnd.render()

    # visualize optimal trajectory
    rnd.ax.plot(pos[:, 0], pos[:, 1], color='k', marker='x', markersize=1.5, zorder=21, linewidth=1.5,
                label='optimal trajectory')

    # visualize sampled trajectory bundle
    step = 1  # draw every trajectory (step=2 would draw every second trajectory)
    if traj_set is not None:
        for i in range(0, len(traj_set), step):
            color = 'blue'
            plt.plot(traj_set[i].cartesian.x, traj_set[i].cartesian.y,
                     color=color, zorder=20, linewidth=0.1, alpha=1.0)

    # visualize reference path
    if ref_path is not None:
        rnd.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19, linewidth=0.8,
                    label='reference path')

    # show plot
    plt.show(block=True)

    # save as .png file
    if save_path is not None:
        plt.savefig(f"{save_path}/{scenario.scenario_id}_{timestep}.png", format='png', dpi=300,
                    bbox_inches='tight')
