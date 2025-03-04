import copy
import logging
import math
from copy import deepcopy

import crmonitor
import numpy as np
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object
from numpy import ndarray
from pathlib import Path

from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

import commonroad_route_planner.fast_api.fast_api as fast_api
from commonroad_route_planner.reference_path import ReferencePath

from commonroad_rp.close_to_stop_line import CloseToStopLine
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.evaluation import run_evaluation
from commonroad_rp.utility.visualization import VisualizationHandler

from commonroad_velocity_planner.configuration.configuration_builder import ConfigurationBuilder
from commonroad_velocity_planner.velocity_planner_interface import IVelocityPlanner
from commonroad_velocity_planner.velocity_planning_problem import VppBuilder

logger = logging.getLogger("HLP_LOGGER")

class HighLevelPlanner:
    """
    Class for higher logic planning
    """

    def __init__(self, config_path: Path, scenario: str | Scenario = None, planning_problem: PlanningProblem = None):
        if isinstance(scenario, str):
            self.config = ReactivePlannerConfiguration.load(config_path, scenario)
            self.config.update()
        else:
            self.config = ReactivePlannerConfiguration.load(config_path)
            self.config.update(scenario, planning_problem)


        # Initialize Planner
        self.config.planning.reference_path: ReferencePath = fast_api.generate_reference_path_from_lanelet_network_and_planning_problem(
            lanelet_network=self.config.scenario.lanelet_network,
            planning_problem=self.config.planning_problem
        )
        self.planner = ReactivePlanner(self.config)

        # set reference path for curvilinear coordinate system
        self.planner.set_reference_path(self.config.planning.reference_path.reference_path)

        # Add first state to recorded state and input list
        self.planner.record_state_and_input(self.planner.x_0)

        # Close to stop line
        self.ctsl = CloseToStopLine(self.config.planning.reference_path)
        self.intermediate_goal = None
        self.stop_line = None
        self.recently_stopped = False

        self.optimal_traj = None
        self.ego_vehicle = None
        self.sampled_trajectory_bundle = None

        # velocity planning
        self.global_trajectory = IVelocityPlanner().plan_velocity(
            reference_path=self.config.planning.reference_path,
            planner_config=ConfigurationBuilder().get_predefined_configuration(),
            velocity_planning_problem=VppBuilder().build_vpp(
                reference_path=self.config.planning.reference_path,
                planning_problem=self.config.planning_problem,
                default_goal_velocity=self.config.planning_problem.initial_state.velocity
            )
        )

        # collision checker
        self._cc = self.planner.collision_checker

        self._wb_rear_axle = self.planner.config.vehicle.wb_rear_axle

        # init visualization
        self.visualization_handler = VisualizationHandler(self.config)

    def plan(self, sc: Scenario, pp: PlanningProblem, steering_angle: float = 0.0) -> Trajectory | None:
        """
        Performs trajectory planning of reactive planner for one step.

        :param sc: CommonRoad scenario.
        :param pp: CommonRoad planning problem.
        :param steering_angle: Steering angle of ego vehicle
        :return: CommonRoad trajectory.
        """
        self.config.scenario = sc
        self.config.planning_problem = pp

        # self._planner.set_collision_checker(sc)
        cc_scenario = copy.deepcopy(self._cc)
        for co in sc.static_obstacles:
            obs = create_collision_object(co)
            cc_scenario.add_collision_object(obs)
        for co in sc.dynamic_obstacles:
            tvo = create_collision_object(co)
            cc_scenario.add_collision_object(tvo)
        self.planner.set_collision_checker(None, cc_scenario)

        # convert to ReactivePlannerState
        x0_planner_cart = ReactivePlannerState()
        orientation = pp.initial_state.orientation
        initial_state_shifted = pp.initial_state.translate_rotate(
            np.array([-self._wb_rear_axle * np.cos(orientation), -self._wb_rear_axle * np.sin(orientation)]), 0.0)
        x0_planner_cart = initial_state_shifted.convert_state_to_state(x0_planner_cart)

        # add steering angle
        x0_planner_cart.steering_angle = steering_angle

        self.planner.x_0 = None
        self.planner.reset(
            initial_state_cart=x0_planner_cart,
            initial_state_curv=None,
            collision_checker=self.planner.collision_checker,
            coordinate_system=self.planner.coordinate_system,
        )

        timestep = len(self.planner.record_state_list) - 1

        # call plan function and generate trajectory
        if self.plan_new_trajectory(timestep):
            return self.convert_from_rear_to_middle(self.optimal_traj[0])
        else:
            return None

    def convert_from_rear_to_middle(self, traj: Trajectory) -> Trajectory:
        shifted_traj = []
        for state in traj.state_list:
            shifted_traj.append(ReactivePlannerState.shift_state_to_center(state, self._wb_rear_axle))
        return Trajectory(traj.initial_time_step, shifted_traj)


    def run(self):
        """
        run planner
        """

        # start planning
        timestep = len(self.planner.record_state_list) - 1
        goal_reached = True
        while not self.planner.goal_reached():
            timestep = len(self.planner.record_state_list) - 1

            # check if planning cycle or not
            plan_new_trajectory = timestep % self.config.planning.replanning_frequency == 0

            if plan_new_trajectory:
                if not self.plan_new_trajectory(timestep):
                    goal_reached = False
                    break
            else:
                if self.optimal_traj:
                    self.simulate_step(timestep)
            if self.config.monitor.trace_reset_option_val is crmonitor.TraceResetOptions.filter:
                self.config.rule_monitor.propagate_trace()  # we use filter option to keep computed props of other traffic participants within cycle
            self.config.rule_monitor.get_world().propagate(ego=False) # ego needs to be propagated inside planner since otherwise invalid planned trajectory of ego is executed
            self.planner.prepare_initial_state_monitor()

        # make gif
        #make_gif(self.config, range(0, self.planner.record_state_list[-1].time_step))

        # Evaluate results
        evaluate = False
        if evaluate:
            run_evaluation(self.planner.config, self.planner.record_state_list, self.planner.record_input_list)

        return goal_reached, timestep

    def plan_new_trajectory(self, timestep) -> bool:
        """
        Compute new trajectory fpr current timestep

        @param timestep: Current timestep
        """

        stop_line_lanelet = self.ctsl.at_stop_line(self.planner.x_0.position)
        if stop_line_lanelet:
            self.stop_line = stop_line_lanelet.center_vertices[-1]
            self.intermediate_goal = self.tf_goal_to_rear(stop_line_lanelet.center_vertices)

            if not self.standing_close_to_stopline() and not self.recently_stopped:
                self.optimal_traj = None
                i = 1
                while self.optimal_traj is None and i <= self.planner.sampling_level:

                    # plan to intermediate goal
                    self.call_planner(i, self.intermediate_goal)

                    stop_optimal = copy.copy(self.optimal_traj)

                    # plan without intermediate goal
                    self.call_planner(i)

                    i += 1
                    if stop_optimal is None and self.optimal_traj is None:
                        continue
                    elif stop_optimal is None:
                        logger.warning("Not stopping!")
                        break
                    elif self.optimal_traj is None:
                        self.optimal_traj = stop_optimal
                        logger.warning("Stopping")
                        break

                    dist_stop = stop_optimal[2][-1][0]
                    dist_continue = self.optimal_traj[2][-1][0]

                    # compare optimal trajectories
                    if dist_continue < dist_stop:
                        self.optimal_traj = stop_optimal
                        logger.info("Stopping")
                    else:
                        logger.info("Not stopping!")
            else:
                self.call_planner()
                self.recently_stopped = True
        else:
            self.call_planner()
            self.recently_stopped = False

        if self.optimal_traj:
            self.save_and_visualize_planning_step(timestep)
            return True
        else:
            return False


    def call_planner(self, sampling_level: int = -1, goal: ndarray = None):
        """
        new planning cycle -> plan a new optimal trajectory

        @param sampling_level: (optional) Level for longitudinal and lateral sampling density
        @param goal: (optional) intermediate goal
        """

        # set sampling strategy
        if goal is None:
            self.config.sampling.longitudinal_mode = 'velocity_keeping'
            desired_speed: float = self.global_trajectory.get_velocity_at_position_with_lookahead(
                position=self.planner.x_0.position,
                lookahead_s=self.config.planning.planning_horizon
            )
            self.planner.set_desired_velocity(desired_velocity=desired_speed)
        else:
            self.config.sampling.longitudinal_mode = 'stopping'
            self.planner.set_sampling_space()
            self.planner.set_desired_lon_position(
                self.planner.coordinate_system.convert_to_curvilinear_coords(goal[0], goal[1])[0])

        # plan until optimal trajectory is found with increasing sampling density or only with given sampling density
        if sampling_level == -1:
            self.optimal_traj = self.planner.plan()
        else:
            self.optimal_traj = self.planner.plan(sampling_level)

    def save_and_visualize_planning_step(self, timestep):
        # record state and input
        self.planner.record_state_and_input(self.optimal_traj[0].state_list[1])

        # reset planner state for re-planning
        self.planner.reset(initial_state_cart=self.planner.record_state_list[-1],
                           initial_state_curv=(self.optimal_traj[1][1], self.optimal_traj[2][1]),
                           collision_checker=self.planner.collision_checker,
                           coordinate_system=self.planner.coordinate_system)

        # visualization: create ego Vehicle for planned trajectory and store sampled trajectory set
        if self.config.debug.show_plots or self.config.debug.save_plots:
            self.ego_vehicle = self.planner.convert_state_list_to_commonroad_object(self.optimal_traj[0].state_list)
            self.sampled_trajectory_bundle = None
            if self.config.debug.draw_traj_set:
                self.sampled_trajectory_bundle = deepcopy(self.planner.stored_trajectories)

            self.visualize(timestep)

    def simulate_step(self, timestep):
        """
        Simulate timestep based on already computed trajectory
        """
        # simulate scenario one step forward with planned trajectory
        self.sampled_trajectory_bundle = None

        # continue on optimal trajectory
        temp = timestep % self.config.planning.replanning_frequency

        # record state and input
        self.planner.record_state_and_input(self.optimal_traj[0].state_list[1 + temp])

        # reset planner state for re-planning
        self.planner.reset(initial_state_cart=self.planner.record_state_list[-1],
                           initial_state_curv=(self.optimal_traj[1][1 + temp], self.optimal_traj[2][1 + temp]),
                           collision_checker=self.planner.collision_checker,
                           coordinate_system=self.planner.coordinate_system)

        if self.config.debug.show_plots or self.config.debug.save_plots:
            self.visualize(timestep)

    def tf_goal_to_rear(self, center_vertices):
        """
        transform a goal point to the rear axis of the vehicle based on its dimensions

        @param center_vertices: center vertices of a given lanelet
        """
        back = center_vertices[-1]
        min_dist = self.planner.config.vehicle.length / 2 + self.planner.config.vehicle.wb_rear_axle
        i = -2
        while math.dist(back, center_vertices[i]) < min_dist:
            i -= 1
        return center_vertices[i]

    def visualize(self, timestep):
        self.visualization_handler.add_task(scenario=self.config.scenario,
                                      planning_problem=self.config.planning_problem,
                                      ego_vehicle=self.ego_vehicle, traj_set=self.sampled_trajectory_bundle,
                                      ref_path=self.planner.reference_path, timestep=timestep,
                                      config=self.config, medium_goal=self.intermediate_goal)

    def standing_close_to_stopline(self):
        """
        Deside if vehicles front is closer than 1m to stop line and in standstill
        """
        dist = math.dist(self.planner.x_0.position, self.stop_line)
        max_dist = 1.0 + self.planner.config.vehicle.length / 2 + self.planner.config.vehicle.wb_rear_axle
        if dist < max_dist and self.planner.x_0.velocity == 0.0:
            return True
        else:
            return False
