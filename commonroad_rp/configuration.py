import os
from pathlib import Path
from typing import Union
from omegaconf import OmegaConf, ListConfig, DictConfig
import numpy as np

# commonroad-io
from commonroad.common.solution import VehicleType

# commonroad-dc
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping


class Configuration:
    """
    Main Configuration class holding all planner-relevant configurations
    """
    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.name_scenario = config.general.name_scenario

        # initialize subclasses
        self.general: GeneralConfiguration = GeneralConfiguration(config.general)
        self.planning: PlanningConfiguration = PlanningConfiguration(config.planning)
        self.vehicle: VehicleConfiguration = VehicleConfiguration(config.vehicle)
        self.sampling: SamplingConfiguration = SamplingConfiguration(config.sampling)
        self.debug: DebugConfiguration = DebugConfiguration(config.debug)


class ConfigurationBase:
    def to_dict(self):
        dict_config = dict()
        for key, val in self.__dict__.items():
            if isinstance(val, np.float64):
                val = float(val)

            if isinstance(val, (str, int, float, bool)):
                dict_config[key] = val

        return dict_config


class GeneralConfiguration(ConfigurationBase):
    def __init__(self, config: Union[ListConfig, DictConfig]):
        name_scenario = config.name_scenario

        self.path_scenarios = config.path_scenarios
        self.path_scenario = config.path_scenarios + name_scenario + ".xml"
        self.path_output = config.path_output
        self.path_logs = config.path_logs
        self.path_pickles = config.path_pickles
        self.path_offline_data = config.path_offline_data


class PlanningConfiguration(ConfigurationBase):
    """Class to store all planning configurations"""
    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.dt = config.dt
        self.time_steps_computation = config.time_steps_computation
        self.planning_horizon = config.dt * config.time_steps_computation
        self.replanning_frequency = config.replanning_frequency
        self.mode = config.mode
        self.continuous_collision_check = config.continuous_collision_check
        self.factor = config.factor
        self.low_vel_mode_threshold = config.low_vel_mode_threshold


class VehicleConfiguration(ConfigurationBase):
    """Class to store vehicle configurations"""
    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.id_type_vehicle = config.id_type_vehicle

        # get vehicle parameters from CommonRoad vehicle models given cr_vehicle_id
        vehicle_parameters = VehicleParameterMapping.from_vehicle_type(VehicleType(self.id_type_vehicle))

        # get dimensions from given vehicle ID
        self.length = vehicle_parameters.l
        self.width = vehicle_parameters.w

        # distances front/rear axle to vehicle center
        self.wb_front_axle = vehicle_parameters.a
        self.wb_rear_axle = vehicle_parameters.b

        # get constraints from given vehicle ID
        self.a_max = vehicle_parameters.longitudinal.a_max
        self.v_switch = vehicle_parameters.longitudinal.v_switch
        self.delta_min = vehicle_parameters.steering.min
        self.delta_max = vehicle_parameters.steering.max
        self.v_delta_min = vehicle_parameters.steering.v_min
        self.v_delta_max = vehicle_parameters.steering.v_max

        # overwrite parameters given by vehicle ID if they are explicitly provided in the *.yaml file
        for key, value in config.items():
            if value is not None:
                setattr(self, key, value)

        # wheelbase
        self.wheelbase = self.wb_front_axle + self.wb_rear_axle


class SamplingConfiguration(ConfigurationBase):
    """Class to store sampling configurations"""
    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.t_min = config.t_min
        self.v_min = config.v_min
        self.v_max = config.v_max
        self.d_min = config.d_min
        self.d_max = config.d_max


class DebugConfiguration(ConfigurationBase):
    """Class to store debug configurations"""
    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.save_plots = config.save_plots
        self.save_config = config.save_config
        self.show_plots = config.show_plots
        self.draw_ref_path = config.draw_ref_path
        self.draw_planning_problem = config.draw_planning_problem
        self.draw_icons = config.draw_icons
        self.draw_traj_set = config.draw_traj_set
        self.debug_mode = config.debug_mode
        self.multiproc = config.multiproc
        self.num_workers = config.num_workers
