from typing import Union
import numpy as np


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
        """ Calculates the cost for sampled trajectory
        :return: calculated cost
        """
        # desired_time = self.trajectory_long.desired_horizon
        desired_speed = self.trajectory_long._desired_velocity

        a_u = np.append(self.cartesian.a,self.ext_cartesian.a) if self.ext_cartesian is not None else self.cartesian.a
        v_u = np.append(self.cartesian.v, self.ext_cartesian.v) if self.ext_cartesian is not None else self.cartesian.v
        d_u = np.append(self.curvilinear.d, self.ext_curvilinear.d) if self.ext_cartesian is not None else self.curvilinear.d
        theta_u = np.append(self.curvilinear.theta,self.ext_curvilinear.theta) if self.ext_cartesian is not None else self.curvilinear.theta

        # acceleration costs
        costs = np.sum(a_u ** 2)
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
    def __init__(self):
        self.trajectory_bundle = []

    def min_costs(self) -> float:
        """ Computes minimal cost for all sampled trajectories in trajectory bundle
        :return: minimal cost. None, if trajectory bundle is empty.
        """
        return min([x.total_cost for x in self.trajectory_bundle])

    def max_costs(self) -> float:
        """ Computes max cost for all sampled trajectories in trajectory bundle
        :return: maximal cost. None, if trajectory bundle is empty.
        """
        return max([x.total_cost for x in self.trajectory_bundle])

    def add_trajectory(self, trajectory: TrajectorySample):
        """ Add trajectory to trajectory bundle list
        :param: trajectory: new trajectory to add
        """
        self.trajectory_bundle.append(trajectory)

    def empty(self) -> bool:
        """ Check if trajectory bundle list is empty
        :return: true if no trajectories are stored in trajectory bundle else false
        """
        return len(self.trajectory_bundle) == 0

    def updated_optimal_trajectory(self) -> Union[TrajectorySample, None]:
        """
        :return: trajectory in trajectory_bundle with minimal cost. None, if trajectory bundle is empty.
        """
        if not self.trajectory_bundle:
            return None
        return min(self.trajectory_bundle, key=lambda x: x.reevaluate_costs())

    #def optimal_trajectory(self) -> Union[TrajectorySample, None]:
    #    """
    #    :return: trajectory in trajectory_bundle with minimal cost. None, if trajectory bundle is empty.
    #    """
    #    if not self.trajectory_bundle:
    #        return None
    #    return min(self.trajectory_bundle, key=lambda x: x.total_cost)
    #
    #def remove_trajectory(self, trajectory_idx: int):
    #    """ Remove trajectory to trajectory bundle list
    #    :param: trajectory_idx: id of trajectory to remove
    #    """
    #    self.trajectory_bundle.pop(trajectory_idx)
#
    #def optimal_trajectory_idx(self) -> Union[int, None]:
    #    """
    #    :return: index of trajectory in trajectory_bundle with minimal cost. None, if trajectory bundle is empty.
    #    """
    #    if not self.trajectory_bundle:
    #        return None
    #    return self.trajectory_bundle.index(min(self.trajectory_bundle, key=lambda x: x.total_cost))
#