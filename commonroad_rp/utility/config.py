import numpy as np
import dataclasses
import inspect
import os.path
from dataclasses import dataclass, field, fields
from typing import Union, Any, Optional, Dict, List, Callable
from pathlib import Path

from omegaconf import OmegaConf
from omegaconf.dictconfig import DictConfig
import warnings

from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping
from commonroad.common.solution import VehicleType
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from vehiclemodels.vehicle_parameters import VehicleParameters

from commonroad_rp.utility.general import load_scenario_and_planning_problem


def _dict_to_params(dict_params: Dict[str, Any], cls: Any) -> Any:
    """
    Converts dictionary to parameter class.

    :param dict_params: Dictionary containing parameters.
    :param cls: Parameter dataclass to which dictionary should be converted to.
    :return: Parameter class.
    """
    fields = dataclasses.fields(cls)
    cls_map = {f.name: f.type for f in fields}
    kwargs = {}
    for k, v in cls_map.items():
        if k not in dict_params:
            continue
        if inspect.isclass(v) and issubclass(v, BaseConfiguration):
            kwargs[k] = _dict_to_params(dict_params[k], cls_map[k])
        else:
            kwargs[k] = dict_params[k]
    obj = cls(**kwargs)
    return obj


@dataclass
class BaseConfiguration:
    """Reactive planner base parameters."""

    __initialized: bool = field(init=False, default=False, repr=False)

    def __post_init__(self):
        """Post initialization of base parameter class."""
        # pylint: disable=unused-private-member
        self.__initialized = True
        # Make sure that the base parameters are propagated to all sub-parameters
        # This cannot be done in the init method, because the sub-parameters are not yet initialized.
        # This is not a noop, as it calls the __setattr__ method.
        # Do not remove!
        # See commonroad-io how to set the base parameters

    def __getitem__(self, item: str) -> Any:
        """
        Getter for base parameter value.

        :param: Item for which content should be returned.
        :return: Item value.
        """
        try:
            value = self.__getattribute__(item)
        except AttributeError as e:
            raise KeyError(f"{item} is not a parameter of {self.__class__.__name__}") from e
        return value

    def __setitem__(self, key: str, value: Any):
        """
        Setter for item.

        :param key: Name of item.
        :param value: Value of item.
        """
        try:
            self.__setattr__(key, value)
        except AttributeError as e:
            raise KeyError(f"{key} is not a parameter of {self.__class__.__name__}") from e


@dataclass
class PlanningConfiguration(BaseConfiguration):
    """Planning parameters for reactive planner."""

    # planner time step (in s)
    dt: float = 0.1
    # time_steps_computation * dt = horizon. e.g. 20 * 0.1 = 2s
    time_steps_computation: int = 60
    # replanning frequency (in time steps)
    replanning_frequency: int = 3
    # continuous collision checking
    continuous_collision_check: bool = False
    # time scaling factor for collision checking if planner time step and scenario time step deviate
    factor: int = 1
    # velocity threshold (in m/s) for switching to low velocity mode
    low_vel_mode_threshold: float = 4.0
    # kinematic constraints to check.
    # The list can contain these constraints: velocity, acceleration, kappa, kappa_dot,
    # yaw_rate (Exact naming important!!)
    constraints_to_check: List[str] = \
        field(default_factory=lambda: ["velocity", "acceleration", "kappa", "kappa_dot", "yaw_rate"])
    # lookahead in dt*standstill_lookahead seconds if current velocity <= 0.1 and after specified time too
    standstill_lookahead: int = 10
    # safety margin for dynamic obstacles
    safety_margin_dynamic_obstacles: float = 0.0

    def __post_init__(self):
        self.planning_horizon: float = self.dt * self.time_steps_computation


@dataclass
class SamplingConfiguration(BaseConfiguration):
    """Sampling parameters for reactive planner."""

    # choose sampling method
    # 1: sampling in fixed intervals (see above)
    # 2: adaptive corridor sampling (requires CommonRoad-Reach)
    sampling_method: int = 1

    # choose longitudinal driving mode
    # "velocity_keeping": samples around a desired lon. velocity; (default mode)
    #                     generates QUARTIC polynomials in longitudinal direction
    # "stopping": samples around a desired lon. position (stop point);
    #             generates QUINTIC polynomials in longitudinal direction,
    #             sets target lon. velocity to 0.0
    longitudinal_mode: str = "velocity_keeping"

    # number of sampling levels
    num_sampling_levels: int = 4

    # sampling in fixed intervals
    # minimum time sampling in [s] (t_max is given by planning horizon)
    t_min: float = 0.4
    # longitudinal velocity sampling interval in [m/s] (interval determined by setting desired velocity)
    # TODO improve setting of velocity sampling around a given desired velocity
    # ratio of vehicle a_max which is used for deceleration, i.e., computing v_min for sampling
    # (value in interval ]0, 1])
    max_deceleration_ratio: float = 0.125
    v_min: float = 0
    v_max: float = 0
    # longitudinal position sampling interval around a desired stop point in [m]
    # (interval determined by setting desired stop position)
    s_min: float = -1
    s_max: float = 1
    # lateral sampling interval around reference path in [m]
    d_min: float = -3
    d_max: float = 3

    # number of initial velocity samples (in first sampling level)
    vel_init_samples: int = 3
    # number of initial position samples (in first sampling level)
    pos_init_samples: int = 3

@dataclass
class DebugConfiguration(BaseConfiguration):
    """Parameters specifying debug-related information."""

    # save plots
    save_plots: bool = False
    # save config/logs
    save_config: bool = False
    # show plots
    show_plots: bool = False
    # show evaluation plots
    show_evaluation_plots: bool = True
    # plots file format
    plots_file_format: str = "png"
    # draw the reference path
    draw_ref_path: bool = True
    # draw the planning problem
    draw_planning_problem: bool = True
    # draw obstacles with vehicle icons
    draw_icons: bool = True
    # draw trajectory occupancies of other vehicles
    draw_occupancies_other: bool = False
    # draw sampled trajectory set
    draw_traj_set: bool = False
    # logging settings - Options: NOTSET, DEBUG, INFO, WARNING, ERROR, CRITICAL
    logging_level: str = "INFO"
    # use multiprocessing True/False
    multiproc: bool = True
    # number of workers for multiprocessing
    num_workers: int = 6
    # number of workers for multiprocessing in visualization
    num_workers_viz: int = 6
    max_queue_size: int = 50


@dataclass
class VehicleConfiguration(BaseConfiguration):
    """Class to store vehicle configurations"""

    # default vehicle type ID is 2 (BMW 320i parameters)
    id_type_vehicle: int = 2

    # get dimensions from given vehicle ID
    length: float = 4.508
    width: float = 1.61

    # distances front/rear axle to vehicle center
    wb_front_axle: float = 1.1561957064
    wb_rear_axle: float = 1.4227170936

    # get constraints from given vehicle ID
    a_max: float = 11.5
    v_switch: float = 7.319
    delta_min: float = -1.066
    delta_max: float = 1.066
    v_delta_min: float = -0.4
    v_delta_max: float = 0.4

    def __post_init__(self):
        self._update_computed_params()

    def _update_computed_params(self):
        """Update additional params which are computed from configurable parameters"""
        # wheelbase
        self.wheelbase: float = self.wb_front_axle + self.wb_rear_axle
        # max curvature
        self.kappa_max: float = np.tan(self.delta_max) / self.wheelbase

    def update_vehicle_config(self, yaml_config: DictConfig):
        """
        Overwrites the vehicle parameters which have not been passed explicitly via the YAML file
        with default values from the CommonRoad vehicle model (specified by id_type_vehicle)
        """
        # get vehicle parameters from CommonRoad vehicle models given cr_vehicle_id
        vehicle_parameters: VehicleParameters = \
            VehicleParameterMapping.from_vehicle_type(VehicleType(self.id_type_vehicle))

        # map param names to CR vehicle params
        name_to_cr_veh_param: Dict["str", float] = {
            "length": vehicle_parameters.l,
            "width": vehicle_parameters.w,
            "wb_front_axle": vehicle_parameters.a,
            "wb_rear_axle": vehicle_parameters.b,
            "a_max": vehicle_parameters.longitudinal.a_max,
            "v_switch": vehicle_parameters.longitudinal.v_switch,
            "delta_min": vehicle_parameters.steering.min,
            "delta_max": vehicle_parameters.steering.max,
            "v_delta_min": vehicle_parameters.steering.v_min,
            "v_delta_max": vehicle_parameters.steering.v_max,
        }

        # overwrite params
        for f in fields(self):
            if (f.name not in yaml_config.keys() and
                    name_to_cr_veh_param.get(f.name) is not None):
                setattr(self, f.name, name_to_cr_veh_param[f.name])

        # update computed values
        self._update_computed_params()


@dataclass
class GeneralConfiguration(BaseConfiguration):
    """General parameters for evaluations."""

    # paths are relative to the root directory
    path_scenarios: Optional[str] = Path(__file__).parents[2] / "example_scenarios"
    path_output: str = "output/"
    path_logs: str = "output/logs/"
    path_pickles: str = "output/pickles/"
    path_scenario: Optional[str] = None
    name_scenario: Optional[str] = None

    def set_path_scenario(self, scenario_name: str):
        """
        Setter for scenario path.

        :param scenario_name: Name of CommonRoad scenario.
        """
        self.path_scenario = os.path.join(self.path_scenarios, scenario_name)


@dataclass
class ReactivePlannerConfiguration(BaseConfiguration):
    """Configuration parameters for reactive planner."""

    vehicle: VehicleConfiguration = field(default_factory=VehicleConfiguration)
    planning: PlanningConfiguration = field(default_factory=PlanningConfiguration)
    sampling: SamplingConfiguration = field(default_factory=SamplingConfiguration)
    debug: DebugConfiguration = field(default_factory=DebugConfiguration)
    general: GeneralConfiguration = field(default_factory=GeneralConfiguration)

    def __post_init__(self):
        self.scenario: Optional[Scenario] = None
        self.planning_problem: Optional[PlanningProblem] = None
        self.planning_problem_set: Optional[PlanningProblemSet] = None

    @property
    def name_scenario(self) -> str:
        return self.general.name_scenario

    @classmethod
    def load(cls, file_path: Union[Path, str], scenario_name: Optional[str] = None, validate_types: bool = True) \
            -> 'ReactivePlannerConfiguration':
        """
        Loads parameters from a config yaml file and returns the Configuration class.

        :param file_path: Path to yaml file containing config parameters.
        :param scenario_name: Name of scenario which should be used. If provided, scenario and planning problem are
                              loaded from a CR scenario XML file.
        :param validate_types:  Boolean indicating whether loaded config should be validated against CARLA parameters.
        :return: Base parameter class.
        """
        file_path = Path(file_path)

        assert file_path.suffix == ".yaml", f"File type {file_path.suffix} is unsupported! Please use .yaml!"
        loaded_yaml = OmegaConf.load(file_path)
        if validate_types:
            OmegaConf.merge(OmegaConf.structured(ReactivePlannerConfiguration), loaded_yaml)
        params = _dict_to_params(OmegaConf.to_object(loaded_yaml), cls)
        # add path to scenario file to config
        if scenario_name:
            params.general.set_path_scenario(scenario_name)
        # update vehicle configuration params
        params.vehicle.update_vehicle_config(loaded_yaml["vehicle"])
        return params

    def update(self, scenario: Scenario = None, planning_problem: PlanningProblem = None,
               idx_planning_problem: Optional[int] = None):
        """
        Updates configuration based on the given attributes.
        Function used to construct initial configuration before planner initialization and update configuration during
        re-planning.

        :param scenario: (initial or updated) Scenario object
        :param planning_problem: (initial or updated) planning problem
        :param state_initial: initial state (can be different from planning problem initial state during re-planning)
        """
        # update scenario and planning problem with explicitly given ones
        self.scenario = scenario
        self.planning_problem = planning_problem

        # if both scenario and planning problem are not explicitly provided
        if scenario is None and planning_problem is None:
            try:
                self.scenario, self.planning_problem, self.planning_problem_set = \
                    load_scenario_and_planning_problem(self.general.path_scenario, idx_planning_problem)
            except FileNotFoundError:
                warnings.warn(f"<ReactivePlannerConfiguration.update()>: No scenario .xml file found at "
                              f"path_scenario = {self.general.path_scenario}")

        # Check that a scenario is set (planning problem can be set afterwards)
        assert self.scenario is not None, "<Configuration.update()>: no scenario has been specified"
