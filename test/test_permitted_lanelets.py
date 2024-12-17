import unittest
import os.path
from pathlib import Path

# commonroad-route-planner
from commonroad_route_planner.route_planner import RoutePlanner

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.evaluation import run_evaluation
from commonroad_rp.utility.config import ReactivePlannerConfiguration



class TestPermittedLanelets(unittest.TestCase):

    def test_permitted(self):

        filename = "ZAM_TestPermittedLanes-1_1_T-1.xml"

        # Build config object
        path_config = Path(__file__).parents[1] / "configurations" / f"{filename[:-4]}.yaml"
        config = ReactivePlannerConfiguration.load(path_config, filename)
        config.general.path_output = Path(os.path.abspath(Path(__file__).parents[1])) / "output"
        config.update()

        # route planner
        route_planner = RoutePlanner(config.scenario.lanelet_network, config.planning_problem)
        route = route_planner.plan_routes().retrieve_first_route()

        # Reactive planner
        planner = ReactivePlanner(config)
        planner.set_permitted_lanelet_ids([7, 22, 8, 12, 10, 20, 17, 19, 11])
        planner.set_reference_path(route.reference_path)


        # Run planner
        planner.record_state_and_input(planner.x_0)

        SAMPLING_ITERATION_IN_PLANNER = True

        while not planner.goal_reached():
            current_count = len(planner.record_state_list) - 1

            plan_new_trajectory = current_count % config.planning.replanning_frequency == 0
            if plan_new_trajectory:
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

                planner.record_state_and_input(optimal[0].state_list[1])
                planner.reset(initial_state_cart=planner.record_state_list[-1],
                              initial_state_curv=(optimal[1][1], optimal[2][1]),
                              collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)
            else:
                temp = current_count % config.planning.replanning_frequency
                planner.record_state_and_input(optimal[0].state_list[1 + temp])
                planner.reset(initial_state_cart=planner.record_state_list[-1],
                              initial_state_curv=(optimal[1][1 + temp], optimal[2][1 + temp]),
                              collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)

            print(f"current time step: {current_count}")


        # Evalute throws error, thus killing the test anyways
        evaluate = True
        if evaluate:
            cr_solution, feasibility_list = run_evaluation(
                planner.config,
                planner.record_state_list,
                planner.record_input_list
            )
