from typing import Union
import numpy as np

# planning parameter
class PlanningParameter:
    k_long = None
    k_lat = None

    k_jerk_lon = None
    k_jerk_lat = None
    k_time = None
    k_distance = None

    prediction_horizon = None
    speed_limit = None

    t_step_size = None
    lat_step_size = None
    long_step_size = None


# vehicle parameter
class VehicleParameter:
    jerk_long_max = None
    jerk_lat_max = None

    acceleration_max = 8  # m/s²
    velocity_max = 150 / 3.6  # m/s

    curvature_max = 0.2

    width = 1.674  # vehicle width [m]
    length = 4.298  # vehicle length [m]

    # parameters for calculating steering angle

    length_front = length / 3
    length_rear = 2 * length / 3
    stiffness_front = 40000  # front tire stiffness [N/rad]
    stiffness_rear = 40000  # rear tire stiffens [N/rad]
    mass = 1500  # mass of vehicle [kg]

def parameter_velocity_reaching():
    params = PlanningParameter()

    params.k_long = .1
    params.k_lat = 5.0

    params.k_jerk_lat = 5.0
    params.k_jerk_lon = 5.0
    params.k_time = 10.0
    params.k_distance = 2.0

    params.prediction_horizon = 3.0  # s
    params.speed_limit = 130.0 / 3.6  # m/s

    params.t_step_size = .5  # s
    params.lat_step_size = 1
    params.long_step_size = 1

    return params


class CartesianSample:
    def __init__(self, x, y, theta, v, a, kappa, kappa_dot):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.a = a
        self.kappa = kappa
        self.kappa_dot = kappa_dot

class CurviLinearSample:
    def __init__(self, s, d, theta, dd = None, ddd = None, ss = None, sss = None):
        self.s = s
        self.d = d
        self.theta = theta
        # store time derivations
        self.dd = dd
        self.ddd = ddd
        self.ss = ss
        self.sss = sss

class TrajectorySample:
    def __init__(self, dt, trajectory_long, trajectory_lat, total_cost):
        self.dt = dt
        self.trajectory_long = trajectory_long
        self.trajectory_lat = trajectory_lat
        self.total_cost = total_cost
        self.cartesian = None
        self.curvilinear = None
        self.ext_cartesian = None
        self.ext_curvilinear = None

    def reevaluate_costs(self):
        desired_time = self.trajectory_long.desired_horizon
        desired_speed = self.trajectory_long._desired_velocity

        a_u = np.append(self.cartesian.a,self.ext_cartesian.a) if self.ext_cartesian is not None else self.cartesian.a
        v_u = np.append(self.cartesian.v, self.ext_cartesian.v) if self.ext_cartesian is not None else self.cartesian.v
        d_u = np.append(self.curvilinear.d, self.ext_curvilinear.d) if self.ext_cartesian is not None else self.curvilinear.d
        theta_u = np.append(self.curvilinear.theta,self.ext_curvilinear.theta) if self.ext_cartesian is not None else self.curvilinear.theta

        # acceleration costs
        costs = np.sum((1*a_u) ** 2)
        # velocity costs
        costs += np.sum((5*(v_u - desired_speed))**2)
        # distance costs
        costs += np.sum((0.15*d_u)**2) + (20*d_u[-1])**2
        # orientation costs
        costs += np.sum((0.1*np.abs(theta_u))**2) + (5*(np.abs(theta_u[-1])))**2

        #costs += (10*(self.trajectory_long.duration_s - desired_time)) ** 2

        self.total_cost = costs

        return costs


class TrajectoryBundle:
    def __init__(self, params: PlanningParameter):
        self.trajectory_bundle = []
        self.params = params

    def optimal_trajectory(self) -> Union[TrajectorySample, None]:
        """
        :return: trajectory in trajectory_bundle with minimal cost. None, if trajectory bundle is empty.
        """
        if not self.trajectory_bundle:
            return None
        return min(self.trajectory_bundle, key=lambda x: x.total_cost)

    def min_costs(self) -> float:
        return min([x.total_cost for x in self.trajectory_bundle])

    def max_costs(self) -> float:
        return max([x.total_cost for x in self.trajectory_bundle])

    def updated_optimal_trajectory(self) -> Union[TrajectorySample, None]:
        """
        :return: trajectory in trajectory_bundle with minimal cost. None, if trajectory bundle is empty.
        """
        if not self.trajectory_bundle:
            return None
        return min(self.trajectory_bundle, key=lambda x: x.reevaluate_costs())

    def optimal_trajectory_idx(self) -> Union[int, None]:
        """
        :return: index of trajectory in trajectory_bundle with minimal cost. None, if trajectory bundle is empty.
        """
        if not self.trajectory_bundle:
            return None
        return self.trajectory_bundle.index(min(self.trajectory_bundle, key=lambda x: x.total_cost))

    def add_trajectory(self, trajectory: TrajectorySample):
        self.trajectory_bundle.append(trajectory)

    def remove_trajectory(self, trajectory_idx: int):
        self.trajectory_bundle.pop(trajectory_idx)

    def empty(self) -> bool:
        return len(self.trajectory_bundle) == 0