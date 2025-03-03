__author__ = "Gerald Würsching, Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "2025.1"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Alpha"

from typing import Optional

import numpy as np
import logging

from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.common.util import make_valid_orientation

from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner

from commonroad_clcs.clcs import CurvilinearCoordinateSystem
from commonroad_clcs.config import (
    CLCSParams,
    ProcessingOption,
    ResamplingOption
)


# get logger
logger = logging.getLogger("RP_LOGGER")


class CoordinateSystem(CurvilinearCoordinateSystem):
    """
    Wrapper class around the commonroad_clcs.clcs.CurvilinearCoordinateSystem class.
    Retains existing CoordinateSystem interface in the reactive planner.
    """

    def __init__(
            self,
            reference: np.ndarray,
            preprocess_reference: bool = True,
            clcs_params: Optional[CLCSParams] = None
    ):
        """
        Init of curvilinear coordinate system defined in commonroad_clcs.clcs.
        :params reference: reference path as numpy array
        :params preprocess_reference: bool indicating whether to preprocess the reference path
                - True: reference path is automatically pre-processed with the settings in clcs_params
                - False: it is advised to pre-process the reference path beforehand for usage with the CLCS
        :params clcs_params: CLCS parameters
                - if None: The default clcs parameters are taken (see commonroad_clcs.config for details)
        """

        super().__init__(
            reference_path=reference,
            params=clcs_params,
            preprocess_path=preprocess_reference
        )

        logger.info(f"Coordinate System: Pre-process reference path is set to {preprocess_reference}.")
        logger.info("Coordinate System initialized")

    @property
    def reference(self) -> np.ndarray:
        """
        Legacy property to keep existing interfaces.
        Returns reference path used by CCosy.
        """
        return self.ref_path



def create_initial_ref_path(
    lanelet_network: LaneletNetwork,
    planning_problem: PlanningProblem
) -> np.ndarray:
    """
    Create route and initial reference path
    """
    # run route planner
    route_planner = RoutePlanner(lanelet_network, planning_problem)
    routes = route_planner.plan_routes()
    # plan initial reference path
    ref_path_planner = ReferencePathPlanner(
        lanelet_network,
        planning_problem,
        routes
    )
    ref_path = ref_path_planner.plan_shortest_reference_path().reference_path

    logger.info("Route and initial reference path planned")

    return ref_path


def create_coordinate_system(
    ref_path: np.ndarray,
    clcs_params: Optional[CLCSParams] = None
) -> CoordinateSystem:
    """
    Create coordinate system from reference path.
    Reference path is preprocessed beforehand with the settings specified in clcs_params.
    If no clcs_params are given, a default processing setting is given.
    """
    # if no params provided, we use default settings
    if clcs_params is None:
        clcs_params = CLCSParams(processing_option=ProcessingOption.SPLINE_SMOOTHING)
        clcs_params.spline.degree_spline = 3
        clcs_params.spline.smoothing_factor = 2.0

        clcs_params.resampling.option = ResamplingOption.ADAPTIVE
        clcs_params.resampling.min_step = 0.5
        clcs_params.resampling.max_step = 1.0

    clcs = CoordinateSystem(
        reference=ref_path,
        preprocess_reference=True,
        clcs_params=clcs_params
    )

    return clcs


def interpolate_angle(x: float, x1: float, x2: float, y1: float, y2: float) -> float:
    delta = y2 - y1
    return make_valid_orientation(delta * (x - x1) / (x2 - x1) + y1)
