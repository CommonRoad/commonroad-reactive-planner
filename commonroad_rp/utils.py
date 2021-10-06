__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.1"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Alpha"

import numpy as np

from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem

from commonroad.common.util import make_valid_orientation


def compute_orientation_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the orientations along a given polyline
    :param polyline: The polyline to check
    :return: The orientations along the polyline
    """
    assert isinstance(polyline, np.ndarray) and len(polyline) > 1 and polyline.ndim == 2 and len(
        polyline[0, :]) == 2, '<Math>: not a valid polyline. polyline = {}'.format(polyline)

    if (len(polyline) < 2):
        raise NameError('Cannot create orientation from polyline of length < 2')

    orientation = []
    for i in range(0, len(polyline) - 1):
        pt1 = polyline[i]
        pt2 = polyline[i + 1]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    for i in range(len(polyline) - 1, len(polyline)):
        pt1 = polyline[i - 1]
        pt2 = polyline[i]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    orientation = np.array(orientation) + 2 * np.pi * (np.array(orientation) < 0)

    return orientation


def compute_curvature_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the curvature of a given polyline
    :param polyline: The polyline for the curvature computation
    :return: The curvature of the polyline
    """
    assert isinstance(polyline, np.ndarray) and polyline.ndim == 2 and len(
        polyline[:, 0]) > 2, 'Polyline malformed for curvature computation p={}'.format(polyline)
    x_d = np.gradient(polyline[:, 0])
    x_dd = np.gradient(x_d)
    y_d = np.gradient(polyline[:, 1])
    y_dd = np.gradient(y_d)

    return (x_d * y_dd - x_dd * y_d) / ((x_d ** 2 + y_d ** 2) ** (3. / 2.))


def compute_pathlength_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """
    Computes the pathlength of a given polyline
    :param polyline: The polyline
    :return: The pathlength of the polyline
    """
    assert isinstance(polyline, np.ndarray) and polyline.ndim == 2 and len(
        polyline[:, 0]) > 2, 'Polyline malformed for pathlenth computation p={}'.format(polyline)
    distance = np.zeros((len(polyline),))
    for i in range(1, len(polyline)):
        distance[i] = distance[i - 1] + np.linalg.norm(polyline[i] - polyline[i - 1])

    return np.array(distance)


def extend_trajectory(s, d, s_dot, theta, v, a, duration, dT) -> tuple:
    """
    Extends a trajectory assuming constant motion
    :param s: Longitudinal position
    :param d: Lateral position
    :param s_dot: Longitudinal velocity
    :param theta: Orientation
    :param v: Velocity
    :param a: Acceleration
    :param duration: Duration of extension
    :param dT: Time step of extension
    :return: Tuple (s,d,theta,v,a)
    """
    # compute time array
    t = np.arange(0, duration + dT, dT)
    s_n = s + s_dot * t
    d_n = d + v * np.sin(theta) * t  # np.repeat(d,len(t))
    theta_n = np.repeat(theta, len(t))
    v_n = np.repeat(v, len(t))
    a_n = np.repeat(a, len(t))

    return (s_n, d_n, theta_n, v_n, a_n)


def interpolate_angle(x: float, x1: float, x2: float, y1: float, y2:float) -> float:
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
    delta_2pi_minus = delta - 2 * np.pi
    delta_2pi_plus = delta + 2 * np.pi
    delta = absmin(np.array([delta, delta_2pi_minus, delta_2pi_plus]))

    return make_valid_orientation(delta * (x - x1) / (x2 - x1) + y1)


class CoordinateSystem():

    def __init__(self, reference: np.ndarray):
        self._reference = reference
        self._ccosy = CurvilinearCoordinateSystem(reference)
        self._ref_pos = compute_pathlength_from_polyline(reference)
        self._ref_curv = compute_curvature_from_polyline(reference)
        self._ref_theta = compute_orientation_from_polyline(reference)
        self._ref_curv_d = np.gradient(self._ref_curv, self._ref_pos)

    def reference(self) -> np.ndarray:
        return self._reference

    def ccosy(self) -> CurvilinearCoordinateSystem:
        return self._ccosy

    def ref_pos(self) -> np.ndarray:
        return self._ref_pos

    def ref_curv(self) -> np.ndarray:
        return self._ref_curv

    def ref_curv_d(self) -> np.ndarray:
        return self._ref_curv_d

    def ref_theta(self) -> np.ndarray:
        return self._ref_theta

    def convert_to_cartesian_coords(self, s: float, d: float) -> np.ndarray:
        try:
            cartesian_coords = self._ccosy.convert_to_cartesian_coords(s, d)
        except:
            cartesian_coords = None

        return cartesian_coords

    def convert_to_curvilinear_coords(self, x: float, y: float) -> np.ndarray:
        return self._ccosy.convert_to_curvilinear_coords(x, y)
