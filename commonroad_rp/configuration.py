import os
from pathlib import Path
from typing import Union
from omegaconf import OmegaConf, ListConfig, DictConfig

# commonroad-io
from commonroad.common.solution import VehicleType

# commonroad-dc
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping


class Configuration:
    """
    Main Configuration class holding all planner-relevant configurations
    """
    def __init__(self, config: Union[ListConfig, DictConfig]):
        # initialize subclasses
        self.planning: PlanningConfiguration = PlanningConfiguration(config.planning)
        self.vehicle: VehicleConfiguration = VehicleConfiguration(config.vehicle)
        self.sampling: SamplingConfiguration = SamplingConfiguration(config.sampling)
        self.debug: DebugConfiguration = DebugConfiguration(config.debug)


class PlanningConfiguration:
    """Class to store all planning configurations"""
    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.dt = config.dt
        self.planning_horizon = config.planning_horizon
        self.replanning_frequency = config.replanning_frequency
        self.mode = config.mode
        self.continuous_cc = config.continuous_cc
        self.collision_check_in_cl = config.collision_check_in_cl
        self.factor = config.factor
        self.low_vel_mode_threshold = config.low_vel_mode_threshold


class VehicleConfiguration:
    """Class to store vehicle configurations"""
    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.cr_vehicle_id = config.cr_vehicle_id

        # get vehicle parameters from CommonRoad vehicle models given cr_vehicle_id
        vehicle_parameters = VehicleParameterMapping.from_vehicle_type(VehicleType(config.cr_vehicle_id))

        # get dimensions from given vehicle ID
        self.length = vehicle_parameters.l
        self.width = vehicle_parameters.w
        self.wheelbase = vehicle_parameters.a + vehicle_parameters.b

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


class SamplingConfiguration:
    """Class to store sampling configurations"""
    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.t_min = config.t_min
        self.v_min = config.v_min
        self.v_max = config.v_max
        self.d_min = config.d_min
        self.d_max = config.d_max


class DebugConfiguration:
    """Class to store debug configurations"""
    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.show_plots = config.show_plots
        self.save_plots = config.save_plots
        self.draw_icons = config.draw_icons
        self.draw_traj_set = config.draw_traj_set
        self.debug_mode = config.debug_mode
        self.multiproc = config.multiproc
        self.num_workers = config.num_workers


def build_configuration(name_scenario: str = None, dir_config: str = "configurations") -> Configuration:
    """
    Builds configuration object by merging default, scenario-specific and commandline (CLI) configurations.
    :param name_scenario
    :param dir_config
    """
    conf_default = OmegaConf.load(Path(__file__).parents[1] / "configurations" / "default.yaml")

    path_scenario_config = dir_config + f"/{name_scenario}.yaml"
    if os.path.exists(path_scenario_config):
        conf_scenario = OmegaConf.load(path_scenario_config)
    else:
        conf_scenario = OmegaConf.create()
        print("No scenario-specific config file provided ... Using Default Configuration")

    conf_cli = OmegaConf.from_cli()

    config_merged = OmegaConf.merge(conf_default, conf_scenario, conf_cli)
    return Configuration(config_merged)

