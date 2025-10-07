# standard imports
import logging
from copy import deepcopy
from pathlib import Path
from typing import Union, Literal, List, Tuple, Optional

import numpy as np
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InputState

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.utility.visualization import visualize_planner_at_timestep
from commonroad_rp.utility.evaluation import run_evaluation
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.logger import initialize_logger
from commonroad_rp.utility.utils_coordinate_system import (
    CoordinateSystem,
    create_initial_ref_path,
    create_coordinate_system
)


def run_planner_from_rp_config(
        config: ReactivePlannerConfiguration,
        custom_reference_path: Optional[np.ndarray] = None,
        max_planning_loop_iteration: int = 200,
        do_evaluation: bool = True,
        plot_after_every_step: bool = False
) -> Tuple[List[ReactivePlannerState], List[InputState]]:
    """
    Run reactive planner from reactive planner config
    :param config: reactive planner config
    :param custom_reference_path: custom reference path as (n,2) np.ndarray. If None, CR Global Planner is used as default.
    :param output_save_path: path to save the output to
    :param do_evaluation: if True, evaluates trajectory for drivability and reconstructs inputs
    :param plot_after_every_step: plot trajectory after each time steps
    :return: Tuple of state trajectory and Input trajectory
    """
    # reference path init
    ref_path_orig = create_initial_ref_path(
        config.scenario.lanelet_network,
        config.planning_problem
    ) if custom_reference_path is None else custom_reference_path
    rp_cosys: CoordinateSystem = create_coordinate_system(ref_path_orig)

    # planner init
    planner = ReactivePlanner(config)
    planner.set_reference_path(coordinate_system=rp_cosys)

    # planning
    planner.record_state_and_input(planner.x_0)
    sampling_iteration_in_planner = True
    cnt: int = 0

    while not planner.goal_reached() and cnt < max_planning_loop_iteration + 1:
        current_count = len(planner.record_state_list) - 1
        plan_new_trajectory = current_count % config.planning.replanning_frequency == 0
        if plan_new_trajectory:
            planner.set_desired_velocity(current_speed=planner.x_0.velocity)
            if sampling_iteration_in_planner:
                optimal = planner.plan()
            else:
                optimal = None
                i = 1
                while optimal is None and i <= planner.sampling_level:
                    optimal = planner.plan(i)
                    i += 1

            if not optimal:
                break

            planner.record_state_and_input(optimal[0].state_list[1])

            planner.reset(initial_state_cart=planner.record_state_list[-1],
                          initial_state_curv=(optimal[1][1], optimal[2][1]),
                          collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)

            if config.debug.show_plots or config.debug.save_plots or plot_after_every_step:
                ego_vehicle = planner.convert_state_list_to_commonroad_object(optimal[0].state_list)
                sampled_trajectory_bundle = None
                if config.debug.draw_traj_set:
                    sampled_trajectory_bundle = deepcopy(planner.stored_trajectories)
        else:
            sampled_trajectory_bundle = None
            temp = current_count % config.planning.replanning_frequency
            planner.record_state_and_input(optimal[0].state_list[1 + temp])
            planner.reset(initial_state_cart=planner.record_state_list[-1],
                          initial_state_curv=(optimal[1][1 + temp], optimal[2][1 + temp]),
                          collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)

        if config.debug.show_plots or config.debug.save_plots or plot_after_every_step:
            visualize_planner_at_timestep(scenario=config.scenario, planning_problem=config.planning_problem,
                                          ego=ego_vehicle, traj_set=sampled_trajectory_bundle,
                                          ref_path=planner.reference_path, timestep=current_count, config=config)

        if cnt == max_planning_loop_iteration:
            logger = logging.getLogger("REACTIVE-PLANNER")
            logger.warning(f"Reached maximum time steps of max_planning_loop_iteration={max_planning_loop_iteration}")

    if do_evaluation:
        _, _ = run_evaluation(planner.config, planner.record_state_list,
                              planner.record_input_list)

    return planner.record_state_list, planner.record_input_list


def run_planner_from_scenario_and_planning_problem(
        scenario: Scenario,
        planning_problem: PlanningProblem,
        output_save_path: Union[Path, str],
        config: ReactivePlannerConfiguration = ReactivePlannerConfiguration(),
        custom_reference_path: Optional[np.ndarray] = None,
        do_evaluation: bool = True,
        logging_level: Literal["DEBUG", "ERROR", "WARNING", "INFO"] = "WARNING",
        max_planning_loop_iteration: int = 200,
        plot_after_every_step: bool = False
) -> Tuple[List[ReactivePlannerState], List[InputState]]:
    """
    Work in Progress!! Run reactive planner
    :param scenario: cr scenario object,
    :param planning_problem: cr planning problem object
    :param config: reactive planner config, generates one per defuault
    :param custom_reference_path: custom reference path as (n,2) np.ndarray. If None, CR Global Planner is used as default.
    :param output_save_path: path to save the output to
    :param do_evaluation: if True, evaluates trajectory for drivability and reconstructs inputs
    :param logging_level: logging level
    :param max_planning_loop_iteration: break planning loop after this many iterations
    :param plot_after_every_step: plot trajectory after each time steps
    :return: Tuple of state trajectory and Input trajectory
    """
    raise NotImplementedError("Currently not implemented fully")
    # Config init
    config.general.path_output = output_save_path
    config.update(scenario=scenario, planning_problem=planning_problem)
    initialize_logger(config)
    config.debug.logging_level = logging_level

    return run_planner_from_rp_config(
        config=config,
        custom_reference_path=custom_reference_path,
        do_evaluation=do_evaluation,
        max_planning_loop_iteration=max_planning_loop_iteration,
        plot_after_every_step=plot_after_every_step
    )


def run_planner(
    scenario_xml_path: Union[Path, str],
    config_path: Union[Path, str],
    output_save_path: Union[Path, str],
    custom_reference_path: Optional[np.ndarray] = None,
    do_evaluation: bool = True,
    logging_level: Literal["DEBUG", "ERROR", "WARNING", "INFO"] = "WARNING",
    max_planning_loop_iteration: int = 200,
    plot_after_every_step: bool = False
) -> Tuple[List[ReactivePlannerState], List[InputState]]:
    """
    Run reactive planner
    :param scenario_xml_name: name of scenario + .xml at the end, not a path
    :param config_path: path to Reactive Planner config
    :param custom_reference_path: custom reference path as (n,2) np.ndarray. If None, CR Global Planner is used as default.
    :param output_save_path: path to save the output to
    :param do_evaluation: if True, evaluates trajectory for drivability and reconstructs inputs
    :param logging_level: logging level
    :param max_planning_loop_iteration: break planning loop after this many iterations
    :param plot_after_every_step: plot trajectory after each time steps
    :return: Tuple of state trajectory and Input trajectory
    """
    # Config init
    config = ReactivePlannerConfiguration()
    config = config.load_from_xml_and_yaml(scenario_xml_path=scenario_xml_path, config_yaml_path=config_path)
    config.update()
    initialize_logger(config)
    config.general.path_output = output_save_path
    config.debug.logging_level = logging_level

    return run_planner_from_rp_config(
        config=config,
        custom_reference_path=custom_reference_path,
        do_evaluation=do_evaluation,
        max_planning_loop_iteration=max_planning_loop_iteration,
        plot_after_every_step=plot_after_every_step
    )
