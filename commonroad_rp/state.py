from dataclasses import dataclass
import numpy as np

from commonroad.scenario.state import KSState, FloatExactOrInterval, InitialState


@dataclass(eq=False)
class ReactivePlannerState(KSState):
    """
    State class used for output trajectory of reactive planner with the following additions to KSState:
    * Position is here defined w.r.t. rear-axle position (in KSState: position is defined w.r.t. vehicle center)
    * Extends KSState attributes by acceleration and yaw rate
    """
    def __repr__(self):
        return f"(time_step={self.time_step}, position={self.position},steering_angle={self.steering_angle}, " \
               f"velocity={self.velocity}, orientation={self.orientation}, acceleration={self.acceleration}, " \
               f"yaw_rate = {self.yaw_rate})"

    acceleration: FloatExactOrInterval = None
    yaw_rate: FloatExactOrInterval = None

    @classmethod
    def shift_state_to_center(
        cls,
        state: "ReactivePlannerState",
        wb_rear_axle: float
    ) -> "ReactivePlannerState":
        """
        Shifts position from rear-axle to vehicle center
        :param state: original state with position defined on the rear axle
        :param wb_rear_axle: distance between rear-axle and vehicle center
        :return new state, where the positions are shifted to vehicle center
        """
        # shift positions from rear axle to center
        orientation = state.orientation
        pos_x = state.position[0] + wb_rear_axle * np.cos(orientation)
        pos_y = state.position[1] + wb_rear_axle * np.sin(orientation)
        
        return ReactivePlannerState(
            time_step=state.time_step,
            position=np.array([pos_x, pos_y]),
            velocity=state.velocity,
            acceleration=state.acceleration,
            orientation=state.orientation,
            steering_angle=state.steering_angle,
            yaw_rate=state.yaw_rate
        )

    @classmethod
    def create_from_initial_state(
        cls,
        initial_state: InitialState,
        wheelbase: float,
        wb_rear_axle: float
    )-> "ReactivePlannerState":
        """
        Converts InitialState object to ReactivePlannerState object by:
        * adding initial acceleration (if not existing)
        * adding initial steering angle
        * removing initial slip angle
        * shifting position from center to rear axle
        :param initial_state: InitialState object
        :param wheelbase: wheelbase of the vehicle
        :param wb_rear_axle: distance between rear-axle and vehicle center
        """
        assert type(initial_state) == InitialState, "<ReactivePlannerState.create_from_initial_state()>: Input must be" \
                                                    "of type InitialState"
        # add acceleration (if not existing)
        if not hasattr(initial_state, 'acceleration'):
            initial_state.acceleration = 0.

        # remove slip angle
        if hasattr(initial_state, "slip_angle"):
            delattr(initial_state, "slip_angle")

        # shift initial position from center to rear axle
        orientation = initial_state.orientation
        initial_state_shifted = initial_state.translate_rotate(np.array([-wb_rear_axle * np.cos(orientation),
                                                                         -wb_rear_axle * np.sin(orientation)]), 0.0)

        # convert to ReactivePlannerState
        x0_planner = ReactivePlannerState()
        x0_planner = initial_state_shifted.convert_state_to_state(x0_planner)

        # add steering angle
        x0_planner.steering_angle = np.arctan2(wheelbase * x0_planner.yaw_rate, x0_planner.velocity)

        return x0_planner
