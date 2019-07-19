from commonroad.common.validity import *
import numpy as np


def compute_orientation_from_polyline(polyline: npy.ndarray) -> npy.ndarray:
    """
    Computes the orientations along a given polyline
    :param polyline: The polyline to check
    :return: The orientations along the polyline
    """
    assert isinstance(polyline, npy.ndarray) and len(polyline) > 1 and polyline.ndim == 2 and len(
        polyline[0, :]) == 2, '<Math>: not a valid polyline. polyline = {}'.format(polyline)

    if len(polyline) < 2:
        raise NameError('Cannot create orientation from polyline of length < 2')

    orientation = [0]
    for i in range(1, len(polyline)):
        pt1 = polyline[i - 1]
        pt2 = polyline[i]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    return orientation


def compute_curvature_from_polyline(polyline: npy.ndarray) -> npy.ndarray:
    """
    Computes the curvature of a given polyline
    :param polyline: The polyline for the curvature computation
    :return: The curvature of the polyline
    """
    assert isinstance(polyline, npy.ndarray) and polyline.ndim == 2 and len(
        polyline[:, 0]) > 2, 'Polyline malformed for curvature computation p={}'.format(polyline)
    x_d = np.gradient(polyline[:, 0])
    x_dd = np.gradient(x_d)
    y_d = np.gradient(polyline[:, 1])
    y_dd = np.gradient(y_d)

    return (x_d * y_dd - x_dd * y_d) / ((x_d ** 2 + y_d ** 2) ** (3. / 2.))


def compute_pathlength_from_polyline(polyline: npy.ndarray) -> npy.ndarray:
    """
    Computes the pathlength of a given polyline
    :param polyline: The polyline
    :return: The pathlength of the polyline
    """
    assert isinstance(polyline, npy.ndarray) and polyline.ndim == 2 and len(
        polyline[:, 0]) > 2, 'Polyline malformed for pathlenth computation p={}'.format(polyline)
    distance = np.zeros((len(polyline),))
    for i in range(1, len(polyline)):
        distance[i] = distance[i - 1] + npy.linalg.norm(polyline[i] - polyline[i - 1])

    return npy.array(distance)


def chaikins_corner_cutting(polyline):
    new_polyline = list()
    new_polyline.append(polyline[0])
    for i in range(0, len(polyline)-1):
        new_polyline.append((3/4)*polyline[i] + (1/4)*polyline[i+1])
        new_polyline.append((1/4)*polyline[i] + (3/4)*polyline[i+1])
    new_polyline.append(polyline[-1])
    return new_polyline


def resample_polyline(polyline, step=2.0):
    new_polyline = [polyline[0]]
    current_position = 0 + step
    current_length = np.linalg.norm(polyline[0] - polyline[1])
    current_idx = 0
    while current_idx < len(polyline) - 1:
        if current_position >= current_length:
            current_position = current_position - current_length
            current_idx += 1
            if current_idx > len(polyline) - 2:
                break
            current_length = np.linalg.norm(polyline[current_idx + 1]
                                            - polyline[current_idx])
        else:
            rel = current_position/current_length
            new_polyline.append((1-rel) * polyline[current_idx] +
                                rel * polyline[current_idx + 1])
            current_position += step
    new_polyline.append(polyline[-1])
    return np.array(new_polyline)