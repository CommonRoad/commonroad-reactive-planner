__author__ = "Gerald Würsching, Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.1"
__maintainer__ = "Gerald Würsching"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Alpha"

import numpy as np

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.geometry.util import compute_pathlength_from_polyline,compute_curvature_from_polyline, \
    compute_orientation_from_polyline, resample_polyline

from commonroad.common.util import make_valid_orientation


def interpolate_angle(x: float, x1: float, x2: float, y1: float, y2: float) -> float:
    """
    Interpolates an angle value between two angles according to the miminal value of the absolute difference
    :param x: value of other dimension to interpolate
    :param x1: lower bound of the other dimension
    :param x2: upper bound of the other dimension
    :param y1: lower bound of angle to interpolate
    :param y2: upper bound of angle to interpolate
    :return: interpolated angular value (in rad)
    """
    def absmin(x):
        return x[np.argmin(np.abs(x))]

    delta = y2 - y1
    # delta_2pi_minus = delta - 2 * np.pi
    # delta_2pi_plus = delta + 2 * np.pi
    # delta = absmin(np.array([delta, delta_2pi_minus, delta_2pi_plus]))

    return make_valid_orientation(delta * (x - x1) / (x2 - x1) + y1)


def extrapolate_ref_path(reference_path: np.ndarray, resample_step: float = 2.0) -> np.ndarray:
    """
    Function to extrapolate the end of the reference path in order to avoid CCosy errors and/or invalid trajectory
    samples when the reference path is shorter than the planning horizon.
    :param reference_path: original reference path
    :param resample_step: interval for resampling
    :return extrapolated reference path
    """
    p = np.poly1d(np.polyfit(reference_path[-2:, 0], reference_path[-2:, 1], 1))
    x = 2.3*reference_path[-1, 0] - reference_path[-2, 0]
    new_polyline = np.concatenate((reference_path, np.array([[x, p(x)]])), axis=0)
    return resample_polyline(new_polyline, step=resample_step)


class CoordinateSystem:

    def __init__(self, reference: np.ndarray):
        # initialize reference and CCosy
        self.reference = reference

        # initialize reference state vectors
        self._ref_pos = compute_pathlength_from_polyline(self.reference)
        self._ref_curv = compute_curvature_from_polyline(self.reference)
        self._ref_theta = compute_orientation_from_polyline(self.reference)
        self._ref_curv_d = np.gradient(self._ref_curv, self._ref_pos)

    @property
    def reference(self) -> np.ndarray:
        """returns reference path used by CCosy due to slight modifications within the CCosy module"""
        return np.asarray(self.ccosy.reference_path())

    @reference.setter
    def reference(self, reference):
        """set reference path and Curvilinear Coordinate System"""
        self._reference = reference
        self._ccosy = CurvilinearCoordinateSystem(reference)

    @property
    def ccosy(self) -> CurvilinearCoordinateSystem:
        """return Curvlinear Coordinate System"""
        return self._ccosy

    @property
    def ref_pos(self) -> np.ndarray:
        """position (s-coordinate) along reference path"""
        return self._ref_pos

    @property
    def ref_curv(self) -> np.ndarray:
        """curvature along reference path"""
        return self._ref_curv

    @property
    def ref_curv_d(self) -> np.ndarray:
        """curvature rate along reference path"""
        return self._ref_curv_d

    @property
    def ref_theta(self) -> np.ndarray:
        """orientation along reference path"""
        return self._ref_theta

    def convert_to_cartesian_coords(self, s: float, d: float) -> np.ndarray:
        """convert curvilinear (s,d) point to Cartesian (x,y) point"""
        try:
            cartesian_coords = self._ccosy.convert_to_cartesian_coords(s, d)
        except:
            cartesian_coords = None

        return cartesian_coords

    def convert_to_curvilinear_coords(self, x: float, y: float) -> np.ndarray:
        """convert Cartesian (x,y) point to curviinear (s,d) point"""
        return self._ccosy.convert_to_curvilinear_coords(x, y)
