import numpy as np

# "cimport" is used to import special compile-time information
# about the numpy module (this is stored in a file numpy.pxd which is
# currently part of the Cython distribution).
cimport numpy as np

# We now need to fix a datatype for our arrays. I've used the variable
# DTYPE for this, which is assigned to the usual NumPy runtime
# type info object.
DTYPE = np.float

# "ctypedef" assigns a corresponding compile-time type to DTYPE_t. For
# every type in the numpy module there's a corresponding compile-time
# type with a _t-suffix.
ctypedef np.float_t DTYPE_t


# "def" can type its arguments but not have a return type. The type of the
# arguments for a "def" function is checked at run-time when entering the
# function.
#
# The arrays f, g and h is typed as "np.ndarray" instances. The only effect
# this has is to a) insert checks that the function arguments really are
# NumPy arrays, and b) make some attribute access like f.shape[0] much
# more efficient. (In this example this doesn't matter though.)

def compute_curvature_from_polyline(np.ndarray[DTYPE_t, ndim=2] polyline):
    """
    Computes the curvature of a given polyline
    :param polyline: The polyline for the curvature computation
    :return: The curvature of the polyline
    """
    cdef np.ndarray[DTYPE_t, ndim=1] x_d = np.gradient(polyline[:, 0])
    cdef np.ndarray[DTYPE_t, ndim=1] x_dd = np.gradient(x_d)
    cdef np.ndarray[DTYPE_t, ndim=1] y_d = np.gradient(polyline[:, 1])
    cdef np.ndarray[DTYPE_t, ndim=1] y_dd = np.gradient(y_d)

    # compute curvature
    cdef np.ndarray[DTYPE_t, ndim=1] curvature = (x_d * y_dd - x_dd * y_d) / ((x_d ** 2 + y_d ** 2) ** (3. / 2.))

    return curvature


def compute_pathlength_from_polyline(np.ndarray[DTYPE_t, ndim=2] polyline):
    """
    Computes the pathlength of a given polyline
    :param polyline: The polyline
    :return: The pathlength of the polyline
    """

    cdef np.ndarray[DTYPE_t, ndim=1] distance = np.zeros((polyline.shape[0],))
    for i in range(1, len(polyline)):
        distance[i] = distance[i - 1] + np.linalg.norm(polyline[i] - polyline[i - 1])

    return distance

def compute_orientation_from_polyline(np.ndarray[DTYPE_t, ndim=2] polyline):
    cdef np.ndarray[DTYPE_t, ndim=1] orientation = np.zeros((polyline.shape[0],))
    cdef int i
    for i in range(1, polyline.shape[0]):
        pt1 = polyline[i-1]
        pt2 = polyline[i]
        tmp = pt2 - pt1
        orientation[i] = np.arctan2(tmp[1], tmp[0])

    for i in range(1, orientation.shape[0]):
        if orientation[i - 1] < orientation[i] - 2. * np.pi / 3.:
            orientation[i - 1] = np.pi - orientation[i - 1]

    # check orientations
    if orientation[-1] < orientation[-2] - 2. * np.pi / 3.:
        orientation[-1] = np.pi - orientation[-1]

    return orientation  # np.array(orientation)