__author__ = "Gerald Würsching"
__copyright__ = "TUM Cyber-Physical Systems Group"
__version__ = "1.0"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Beta"


# standard imports
from copy import deepcopy
import logging

# commonroad-io
from commonroad.visualization.mp_renderer import MPRenderer

# commonroad-route-planner
from commonroad_route_planner.route_planner import RoutePlanner

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.visualization import visualize_planner_at_timestep, make_gif
from commonroad_rp.utility.evaluation import run_evaluation
from commonroad_rp.utility.config import ReactivePlannerConfiguration
import commonroad_rp.utility.logger as util_logger_rp

# commonroad-reach
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder as ReachConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
import commonroad_reach.utility.visualization as util_visual


# *************************************
# Set Configurations
# *************************************
# filename = "ZAM_Over-1_1.xml"
filename = "DEU_Test-1_1_T-1.xml"

# Build planner config object
config_planner = ReactivePlannerConfiguration.load(f"configurations/{filename[:-4]}.yaml", filename)
config_planner.update()

# Build reach config object
path_reach = "/home/gerald/Documents/CommonRoad/cps/commonroad-reachable-set"
config_reach = ReachConfigurationBuilder.build_configuration(filename[:-4], path_root=path_reach)

# initialize and get logger
util_logger_rp.initialize_logger(config_planner)
logger = logging.getLogger("RP_LOGGER")


# *************************************
# Initialize Planner
# *************************************
# run route planner and add reference path to config
route_planner = RoutePlanner(config_planner.scenario, config_planner.planning_problem)
route = route_planner.plan_routes().retrieve_first_route()
config_planner.planning.route = route
config_planner.planning.reference_path = route.reference_path

# initialize reactive planner
planner = ReactivePlanner(config_planner)


# *************************************
# Initialize Reach Interface
# *************************************
# update reach config with planner attributes
config_reach.update()
config_reach.planning.steps_computation = config_planner.planning.time_steps_computation
config_reach.planning_problem = planner.config.planning_problem
config_reach.print_configuration_summary()
reach_interface = ReachableSetInterface(config_reach)


# **************************
# Run Planning
# **************************
# Add first state to recorded state and input list
planner.record_state_and_input(planner.x_0)

while not planner.goal_reached():
    current_count = len(planner.record_state_list) - 1

    # check if planning cycle or not
    plan_new_trajectory = current_count % config_planner.planning.replanning_frequency == 0
    if plan_new_trajectory:
        # reset reach interface at start of each re-planning step
        config_reach.update(scenario=planner.config.scenario,
                            state_initial=planner.x_0.shift_positions_to_center(planner.vehicle_params.wb_rear_axle),
                            CLCS=planner.coordinate_system.ccosy)
        config_reach.planning_problem = planner.config.planning_problem
        reach_interface.reset(config_reach)

        # compute reachable sets and get corridor for new planning cycle
        reach_interface.compute_reachable_sets()
        corridor = reach_interface.extract_driving_corridors(to_goal_region=False)[0]

        # new planning cycle -> plan a new optimal trajectory
        planner.sampling_space.driving_corridor = corridor
        planner.set_desired_velocity(current_speed=planner.x_0.velocity)
        optimal = planner.plan()
        if not optimal:
            break

        # record state and input
        planner.record_state_and_input(optimal[0].state_list[1])

        # reset planner state for re-planning
        planner.reset(initial_state_cart=planner.record_state_list[-1],
                      initial_state_curv=(optimal[2][1], optimal[3][1]),
                      collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)

        # visualization: create ego Vehicle for planned trajectory and store sampled trajectory set
        if config_planner.debug.show_plots or config_planner.debug.save_plots:
            ego_vehicle = planner.convert_state_list_to_commonroad_object(optimal[0].state_list)
            sampled_trajectory_bundle = None
            if config_planner.debug.draw_traj_set:
                sampled_trajectory_bundle = deepcopy(planner.stored_trajectories)
    else:
        # simulate scenario one step forward with planned trajectory
        sampled_trajectory_bundle = None

        # continue on optimal trajectory
        temp = current_count % config_planner.planning.replanning_frequency

        # record state and input
        planner.record_state_and_input(optimal[0].state_list[1 + temp])

        # reset planner state for re-planning
        planner.reset(initial_state_cart=planner.record_state_list[-1],
                      initial_state_curv=(optimal[2][1 + temp], optimal[3][1 + temp]),
                      collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)

    print(f"current time step: {current_count}")

    # visualize the current time step of the simulation
    if config_planner.debug.show_plots or config_planner.debug.save_plots:
        renderer = MPRenderer(figsize=(20, 10))
        util_visual.draw_driving_corridor_2d(corridor, 0, reach_interface, rnd=renderer)
        visualize_planner_at_timestep(scenario=config_planner.scenario, planning_problem=config_planner.planning_problem,
                                      ego=ego_vehicle, traj_set=sampled_trajectory_bundle,
                                      ref_path=planner.reference_path, timestep=current_count, config=config_planner,
                                      rnd=renderer)

# make gif
# make_gif(config_planner, range(0, planner.record_state_list[-1].time_step))


# **************************
# Evaluate results
# **************************
evaluate = True
if evaluate:
    cr_solution, feasibility_list = run_evaluation(planner.config, planner.record_state_list, planner.record_input_list)
