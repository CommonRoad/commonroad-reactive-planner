import numpy as np
import dataclasses
import inspect
import os.path
from dataclasses import dataclass, field
from typing import Union, Any, Optional, Dict, List
import pathlib
from omegaconf import OmegaConf

from commonroad.scenario.state import InitialState
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping
from commonroad.common.solution import VehicleType
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad_route_planner.route import Route
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
    return cls(**kwargs)


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

    @classmethod
    def load(cls, file_path: Union[pathlib.Path, str], scenario_name: str, validate_types: bool = True) \
            -> 'ReactivePlannerConfiguration':
        """
        Loads config file and creates parameter class.

        :param file_path: Path to yaml file containing config parameters.
        :param scenario_name: Name of scenario which should be used.
        :param validate_types:  Boolean indicating whether loaded config should be validated against CARLA parameters.
        :return: Base parameter class.
        """
        file_path = pathlib.Path(file_path)
        assert file_path.suffix == ".yaml", f"File type {file_path.suffix} is unsupported! Please use .yaml!"
        loaded_yaml = OmegaConf.load(file_path)
        if validate_types:
            OmegaConf.merge(OmegaConf.structured(ReactivePlannerConfiguration), loaded_yaml)
        params = _dict_to_params(OmegaConf.to_object(loaded_yaml), cls)
        params.general.set_path_scenario(scenario_name)
        return params


@dataclass
class PlanningConfiguration(BaseConfiguration):
    """Planning parameters for reactive planner."""

    # planner time step (in s)
    dt: float = 0.1
    # time_steps_computation * dt = horizon. e.g. 20 * 0.1 = 2s
    time_steps_computation: int = 60
    planning_horizon: float = dt * time_steps_computation
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

    def __post_init__(self):
        # global route and reference path is stored in planning config
        self.route: Optional[Route] = None
        self.reference_path: Optional[np.ndarray] = None


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
    v_min: float = 0
    v_max: float = 0
    # longitudinal position sampling interval around a desired stop point in [m]
    # (interval determined by setting desired stop position)
    s_min: float = -1
    s_max: float = 1
    # lateral sampling interval around reference path in [m]
    d_min: float = -3
    d_max: float = 3


@dataclass
class DebugConfiguration(BaseConfiguration):
    """Parameters specifying debug-related information."""

    # save plots
    save_plots: bool = False
    # save config/logs
    save_config: bool = False
    # show plots
    show_plots: bool = False
    # draw the reference path
    draw_ref_path: bool = True
    # draw the planning problem
    draw_planning_problem: bool = True
    # draw obstacles with vehicle icons
    draw_icons: bool = False
    # draw sampled trajectory set
    draw_traj_set: bool = False
    # logging settings - Options: NOTSET, DEBUG, INFO, WARNING, ERROR, CRITICAL
    logging_level: str = "INFO"
    # use multiprocessing True/False
    multiproc: bool = True
    # number of workers for multiprocessing
    num_workers: int = 6


@dataclass
class VehicleConfiguration(BaseConfiguration):
    """Class to store vehicle configurations"""

    id_type_vehicle: int = 2
    # get vehicle parameters from CommonRoad vehicle models given cr_vehicle_id
    vehicle_parameters: VehicleParameters = VehicleParameterMapping.from_vehicle_type(VehicleType(id_type_vehicle))

    # get dimensions from given vehicle ID
    length: float = vehicle_parameters.l
    width: float = vehicle_parameters.w

    # distances front/rear axle to vehicle center
    wb_front_axle: float = vehicle_parameters.a
    wb_rear_axle: float = vehicle_parameters.b

    # get constraints from given vehicle ID
    a_max: float = vehicle_parameters.longitudinal.a_max
    v_switch: float = vehicle_parameters.longitudinal.v_switch
    delta_min: float = vehicle_parameters.steering.min
    delta_max: float = vehicle_parameters.steering.max
    v_delta_min: float = vehicle_parameters.steering.v_min
    v_delta_max: float = vehicle_parameters.steering.v_max

    # wheelbase
    wheelbase: float = vehicle_parameters.a + vehicle_parameters.b


@dataclass
class GeneralConfiguration(BaseConfiguration):
    """General parameters for evaluations."""

    # paths are relative to the root directory
    path_scenarios: str = "example_scenarios/"
    path_output: str = "output/"
    path_logs: str = "output/logs/"
    path_pickles: str = "output/pickles/"
    path_offline_data: str = "output/offline_data/"
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

    def update(self, scenario: Scenario = None, planning_problem: PlanningProblem = None,
               state_initial: InitialState = None):
        """
        Updates configuration based on the given attributes.
        Function used to construct initial configuration before planner initialization and update configuration during
        re-planning.

        :param scenario: (initial or updated) Scenario object
        :param planning_problem: (initial or updated) planning problem
        :param state_initial: initial state (can be different from planning problem initial state during re-planning)
        """
        # update scenario and planning problem with explicitly given ones
        if scenario:
            self.scenario = scenario
        if planning_problem:
            self.planning_problem = planning_problem

        # if scenario and planning problem not explicitly given
        if scenario is None and planning_problem is None:
            if self.scenario is None or self.planning_problem is None:
                # read original scenario and pp from scenario file
                self.scenario, self.planning_problem, self.planning_problem_set = \
                    load_scenario_and_planning_problem(self.general.path_scenario)
            else:
                # keep previously stored scenario and planning problem
                pass
        else:
            raise RuntimeError("ReactiveParams::update: Scenario or Planning not None")

        # Check that scenario and planning problem are set
        assert self.scenario is not None, "<Configuration.update()>: no scenario has been specified"
        assert self.planning_problem is not None, "<Configuration.update()>: no planning problem has been specified"

        # update initial state for planning if explicitly given
        self.planning.state_initial = state_initial if state_initial else self.planning_problem.initial_state
