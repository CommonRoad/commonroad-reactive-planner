import logging
import os
import shutil
import unittest
from copy import deepcopy
from pathlib import Path

from commonroad_route_planner.route_planner import RoutePlanner

from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.logger import initialize_logger
from commonroad_rp.utility.visualization import VisualizationHandler


class VisualizationTests(unittest.TestCase):

    def test_multiprocessing(self):
        filename = "ZAM_Over-1_1.xml"
        path_config = Path(__file__).parents[1] / "configurations/ZAM_OverMultiP-1_1.yaml"
        config = ReactivePlannerConfiguration.load(path_config, filename)
        config.general.path_output = Path(os.path.abspath(Path(__file__).parents[0])) / "test_output"
        config.update()

        vis_handler = VisualizationHandler(config)

        # initialize and get logger
        initialize_logger(config)
        logger = logging.getLogger("RP_LOGGER")

        # *************************************
        # Initialize Planner
        # *************************************
        # run route planner and add reference path to config
        route_planner = RoutePlanner(config.scenario.lanelet_network, config.planning_problem)
        route = route_planner.plan_routes().retrieve_first_route()

        # initialize reactive planner
        planner = ReactivePlanner(config)

        # set reference path for curvilinear coordinate system
        planner.set_reference_path(route.reference_path)

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
                        i += 1

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
                vis_handler.add_task(scenario=config.scenario, planning_problem=config.planning_problem,
                                              ego_vehicle=ego_vehicle, traj_set=sampled_trajectory_bundle,
                                              ref_path=planner.reference_path, timestep=current_count, config=config)

        del(vis_handler)
        _, _, files = next(os.walk(Path(__file__).parent / "test_output/ZAM_Over-1_1"))
        self.assertTrue(len(files) == 27)
        shutil.rmtree(Path(__file__).parent / "test_output")

