__author__ = "Christian Pek"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["BMW Group CAR@TUM, interACT"]
__version__ = "0.1"
__maintainer__ = "Christian Pek"
__email__ = "Christian.Pek@tum.de"
__status__ = "Alpha"

from abc import ABC, abstractmethod

import numpy as np

import commonroad_rp.trajectories

class CostFunction(ABC):

    def __init__(self):
        pass

    @abstractmethod
    def evaluate(self, trajectory: commonroad_rp.trajectories.TrajectorySample) -> float:
        pass


class DefaultCostFunction(CostFunction):

    def __init__(self, desired_speed):
        self.desired_speed = desired_speed

    def evaluate(self, trajectory: commonroad_rp.trajectories.TrajectorySample):
        # desired_time = self.trajectory_long.desired_horizon

        # acceleration costs
        costs = np.sum((1 * trajectory.cartesian.a) ** 2)
        # velocity costs
        costs += np.sum((5 * (trajectory.cartesian.v - self.desired_speed)) ** 2)
        # distance costs
        costs += np.sum((0.15 * trajectory.curvilinear.d) ** 2) + (20 * trajectory.curvilinear.d[-1]) ** 2
        # orientation costs
        costs += np.sum((0.1 * np.abs(trajectory.curvilinear.theta)) ** 2) + (
                    5 * (np.abs(trajectory.curvilinear.theta[-1]))) ** 2

        return costs


class DefaultCostFunctionFailSafe(CostFunction):

    def evaluate(self, trajectory: commonroad_rp.trajectories.TrajectorySample):
        # desired_time = self.trajectory_long.desired_horizon

        # acceleration costs
        costs = np.sum((1 * trajectory.cartesian.a) ** 2)
        # distance costs
        costs += np.sum((0.25 * trajectory.curvilinear.d) ** 2) + (20 * trajectory.curvilinear.d[-1]) ** 2
        # orientation costs
        costs += np.sum((0.25 * np.abs(trajectory.curvilinear.theta)) ** 2) + (
                    5 * (np.abs(trajectory.curvilinear.theta[-1]))) ** 2

        return costs





