__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "2024.1"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Beta"


# standard imports
from copy import deepcopy
import logging
from pathlib import Path

# commonroad-route-planner
import commonroad_route_planner.fast_api.fast_api as rfapi
import crmonitor
from commonroad_route_planner.reference_path import ReferencePath

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.visualization import visualize_planner_at_timestep, make_gif
from commonroad_rp.utility.evaluation import run_evaluation
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from commonroad_rp.utility.logger import initialize_logger


def main(
    config: ReactivePlannerConfiguration
) -> None:
    # initialize and get logger
    initialize_logger(config)
    logger = logging.getLogger("RP_LOGGER")

    # *************************************
    # Initialize Planner
    # *************************************
    # run route planner and add reference path to config
    reference_path: ReferencePath = rfapi.generate_reference_path_from_lanelet_network_and_planning_problem(
        lanelet_network=config.scenario.lanelet_network,
        planning_problem=config.planning_problem
    )

    # initialize reactive planner
    planner = ReactivePlanner(config)

    # set reference path for curvilinear coordinate system
    planner.set_reference_path(reference_path.reference_path)

    # **************************
    # Run Planning
    # **************************
    # Add first state to recorded state and input list
    planner.record_state_and_input(planner.x_0)

    SAMPLING_ITERATION_IN_PLANNER = True

    while not planner.goal_reached():
        current_count = len(planner.record_state_list) - 1

        # check if planning cycle or not
        plan_new_trajectory = current_count % config.planning.replanning_frequency == 0
        if plan_new_trajectory:
            # new planning cycle -> plan a new optimal trajectory
            planner.set_desired_velocity(current_speed=planner.x_0.velocity)
            if SAMPLING_ITERATION_IN_PLANNER:
                optimal = planner.plan()
            else:
                optimal = None
                i = 1
                while optimal is None and i <= planner.sampling_level:
                    optimal = planner.plan(i)

            if not optimal:
                break

            # record state and input
            planner.record_state_and_input(optimal[0].state_list[1])

            # reset planner state for re-planning
            planner.reset(initial_state_cart=planner.record_state_list[-1],
                          initial_state_curv=(optimal[1][1], optimal[2][1]),
                          collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)

            # visualization: create ego Vehicle for planned trajectory and store sampled trajectory set
            if config.debug.show_plots or config.debug.save_plots:
                ego_vehicle = planner.convert_state_list_to_commonroad_object(optimal[0].state_list)
                sampled_trajectory_bundle = None
                if config.debug.draw_traj_set:
                    sampled_trajectory_bundle = deepcopy(planner.stored_trajectories)
        else:
            # simulate scenario one step forward with planned trajectory
            sampled_trajectory_bundle = None

            # continue on optimal trajectory
            temp = current_count % config.planning.replanning_frequency

            # record state and input
            planner.record_state_and_input(optimal[0].state_list[1 + temp])

            # reset planner state for re-planning
            planner.reset(initial_state_cart=planner.record_state_list[-1],
                          initial_state_curv=(optimal[1][1 + temp], optimal[2][1 + temp]),
                          collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)

        print(f"current time step: {current_count}")

        # visualize the current time step of the simulation
        if config.debug.show_plots or config.debug.save_plots:
            visualize_planner_at_timestep(scenario=config.scenario, planning_problem=config.planning_problem,
                                          ego=ego_vehicle, traj_set=sampled_trajectory_bundle,
                                          ref_path=planner.reference_path, timestep=current_count, config=config)

        if planner.config.monitor.trace_reset_option_val is crmonitor.TraceResetOptions.filter:
            planner.config.rule_monitor.propagate_trace()  # we use filter option to keep computed props of other traffic participants within cycle
        planner.config.rule_monitor.get_world().propagate(ego=False)  # ego needs to be propagated inside planner since otherwise invalid planned trajectory of ego is executed
        planner.prepare_initial_state_monitor()

    # make gif
    # make_gif(config, range(0, planner.record_state_list[-1].time_step))

    # **************************
    # Evaluate results
    # **************************
    evaluate = True
    if evaluate:
        run_evaluation(planner.config, planner.record_state_list, planner.record_input_list)


# *************************************
# Run planning
# *************************************
if __name__ == "__main__":
    scenario = "DEU_testStopLine-1_1_T-1.pb"
    config_path = Path(__file__).parents[0] / "configurations/DEU_testStopLine-1_1_T-1.yaml"

    # Build config object
    rp_config = ReactivePlannerConfiguration.load(config_path, scenario)
    rp_config.update()

    main(
        config=rp_config
    )
