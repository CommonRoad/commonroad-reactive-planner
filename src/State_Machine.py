from transitions import Machine
from commonroad.scenario.obstacle import ObstacleType
from parameter import VehicleParameter, PlanningParameter
import numpy as np
from scipy import spatial
from route_planner import RoutePlanner

class CarHighlevelStates(object):

    states =['following', 'lane_change_left']
    transitions = [{'trigger': 'Car_ahead_too_slow',    'source': 'following',          'dest': 'lane_change_left',
                    'before': 'set_lanelet_ids', 'conditions': 'lanelet_ids_valid'},

                   {'trigger': 'on_new_centerline',     'source': 'lane_change_left',   'dest': 'following',
                    'before': 'reset_lanelet_ids'}]

    def __init__(self):

        self.old_lanelet = None
        self.new_lanelet = None

        self.machine = Machine(model=self, states=CarHighlevelStates.states,
                               transitions=CarHighlevelStates.transitions, initial='following')

    def set_lanelet_ids(self, old_lanelet = None, new_lanelet = None):
        self.old_lanelet = old_lanelet
        self.new_lanelet = new_lanelet

    def reset_lanelet_ids(self, old_lanelet = None, new_lanelet = None):
        self.old_lanelet = None
        self.new_lanelet = None

    def lanelet_ids_valid(self, old_lanelet = None, new_lanelet = None):
        if old_lanelet is not None and new_lanelet is not None:
            return True
        return False


class StateMachine:
    def __init__(self, scenario):

        self._scenario = scenario
        self._states = CarHighlevelStates()
        self._vehicleparameter = VehicleParameter()
        self._route_planner = RoutePlanner(scenario.scenario_path)


    def init_reference_path(self):
        '''
        Initializes reference path
        :return: Reference path to be set
        '''
        source_position = self._route_planner.planning_problem.initial_state.position
        reference_path = self._route_planner.set_reference_lane(0, source_position)
        return reference_path

    def check_if_velocity_is_too_slow(self, ego, obstacle_ahead):
        '''
        Checks if the velocity of the car ahead is too slow
        :param ego: Data of ego vehicle
        :param obstacle_ahead: Obstacle ahead of ego car
        :return:
        '''
        # Velocity differences between ego and the obstacle ahead
        vel_difference_overtaking = 0
        vel_difference = 0

        # Should only overtake obstacles that are significantly slower
        allowed_velocity_difference = ego._initial_state.velocity / 10

        # Static obstacle is ahead
        if not hasattr(obstacle_ahead[0].initial_state, 'velocity'):
            print("Static obstacle ahead!")
            vel_difference_overtaking = ego._initial_state.velocity - allowed_velocity_difference
            vel_difference = ego._initial_state.velocity

        # Dynamic obstacle is ahead
        elif obstacle_ahead[0].initial_state.velocity < ego._initial_state.velocity + allowed_velocity_difference:
            vel_difference_overtaking = obstacle_ahead[0].initial_state.velocity \
                                            - (ego._initial_state.velocity + allowed_velocity_difference)
            vel_difference = obstacle_ahead[0].initial_state.velocity - ego._initial_state.velocity

        return vel_difference_overtaking, vel_difference

    def check_current_state(self, scenario, ego, reference_path, near_obstacles, obstacle_ahead, k):
        '''
        Main function to check and eventually change the current state
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :param reference_path: Current reference path
        :param near_obstacles: Obstacles around car [obstacle, position, velocity, lanelet_id]
        :param obstacle_ahead: Obstacle ahead of ego car
        :param k: Time step of main function
        :return: Velocity ego car, new reference path
        '''
        print("Current State: ", self._states.state)

        ego_velocity = ego._initial_state.velocity
        ego_position = ego._initial_state.position

        if self._states.state == 'lane_change_left':

            if self.check_if_car_on_new_lanelet(scenario, ego):
                if self.check_if_car_on_new_centervertice(scenario, ego):
                    self._states.on_new_centerline()

            return ego_velocity, reference_path

        if self._states.state == 'following':

            velocity_has_to_be_changed = False

            if self.check_for_possible_overtaking_lanelet(scenario, ego):
                if self.check_lane_change_possible(scenario, ego, near_obstacles, obstacle_ahead, left=True):

                    old_lanelet = self.get_laneletid_of_egocar(scenario, ego)
                    new_lanelet = scenario.lanelet_network.find_lanelet_by_id(self.get_laneletid_of_egocar(scenario, ego)).adj_left

                    self._states.Car_ahead_too_slow(old_lanelet,new_lanelet)

                    reference_path = self._route_planner.set_reference_lane(-1, ego_position)

                else:
                    # No lanechange possible due to neighboring obstacles
                    velocity_has_to_be_changed = True

            else:
                # No lanechange possible due to missing lanelet on the left
                velocity_has_to_be_changed = True

            # Change velocity if necessary
            if velocity_has_to_be_changed:

                if not hasattr(obstacle_ahead[0].initial_state, 'velocity'):
                    ego_velocity = 0
                if ego_velocity != 0:
                    ego_velocity = obstacle_ahead[0].initial_state.velocity

                print("Changed velocity to: ", ego_velocity)

            return ego_velocity, reference_path

    # Check functions for 'lane_chenge_left' State
    def check_if_car_on_new_lanelet(self, scenario, ego):
        '''
        Checks if ego vehicle is on new lanelet
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :return: True if ego vehicle is on new lanelet
        '''
        ego_laneletid = self.get_laneletid_of_egocar(scenario, ego)

        if ego_laneletid == self._states.new_lanelet:
            return True

        return False

    def check_if_car_on_new_centervertice(self, scenario, ego):
        '''
        Checks if ego vehicle is on new center vertices
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :return: True if ego vehicle is on new center vertices
        '''
        new_lanelet = scenario.lanelet_network.find_lanelet_by_id(self._states.new_lanelet)
        ego_position = ego._initial_state.position

        distance, index = spatial.KDTree(new_lanelet.center_vertices).query(ego_position)

        if distance < 0.5:
            return True

        return False

    def check_for_possible_overtaking_lanelet(self, scenario, ego):
        '''
        Checks if ego vehicle has a possible overtaking lanelet
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :return:
        '''
        laneletid = self.get_laneletid_of_egocar(scenario, ego)

        if scenario.lanelet_network.find_lanelet_by_id(laneletid).adj_left_same_direction:
            print('Overtaking lanelet exists')
            return True

        return False

    def check_lane_change_possible(self, scenario, ego, near_obstacles, obstacle_ahead, left=True):
        '''
        Checks distance to obstacles on neighboring lane and decides if a lane change is possible
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :param near_obstacles: Obstacles around car [obstacle, position, velocity, lanelet_id]
        :param obstacle_ahead: Obstacle ahead of ego car
        :param left: Boolean if left or right lane gets checked
        :return: True if lane Change possible
        '''
        obstacles_on_neighboring_lane = self.get_obstacles_on_neighboring_lane(ego, scenario, near_obstacles, left)
        # Number is used to count down all obstacles that fulfill the checks (see below)
        number_neighbors = len(obstacles_on_neighboring_lane)

        ego_position = ego._initial_state.position
        ego_parameter = VehicleParameter()

        safety_margin = ego._initial_state.velocity / 10

        rotation_matrix = [[np.cos(ego._initial_state.orientation), -np.sin(ego._initial_state.orientation)],
                           [np.sin(ego._initial_state.orientation), np.cos(ego._initial_state.orientation)]]

        for obstacle in obstacles_on_neighboring_lane:

            obstacle_position = obstacle[1]
            transformed_obstacle_position = np.matmul(obstacle_position - ego_position, rotation_matrix)

            if transformed_obstacle_position[0] > 0:

                # Check distance to obstacle in front
                distance_to_obstacle = transformed_obstacle_position[0] - ego_parameter.length/2
                if obstacle[0].obstacle_shape.length/2 < distance_to_obstacle - safety_margin:
                    print("Enough space to neighboring car: ",
                          abs(distance_to_obstacle - safety_margin) - obstacle[0].obstacle_shape.length/2)

                    # Check Velocity of obstacle in front
                    if obstacle[0].initial_state.velocity > obstacle_ahead[0].initial_state.velocity:
                        print("Car on left lane is faster: ",
                              obstacle[0].initial_state.velocity - obstacle_ahead[0].initial_state.velocity)
                        # Fulfills conditions --> delete from number
                        number_neighbors -= 1

            # Obstacle is not in front
            else:
                distance_to_obstacle = abs(transformed_obstacle_position[0]) - ego_parameter.length / 2

                if obstacle[0].obstacle_shape.length/2 < distance_to_obstacle:
                    print("Enough space to neighboring following car: ",
                          distance_to_obstacle - obstacle[0].obstacle_shape.length/2)

                    number_neighbors -= 1

        # All obstacles on neighboring lane fulfill conditions
        if number_neighbors == 0:
            print("Lanechange possible!")
            return True
        # Some do not fulfill conditions
        else:
            print("No lanechange due to neighboring Obstacles!")
            return False

    # Get Obstacle functions
    def get_obstacles_around(self, scenario, ego, k):
        '''
        Computes obstacles around and in front of the ego vehicle
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :param k: Time step of main function
        :return: near_obstacles, obstacle_ahead
        '''
        near_obstacles = self.get_near_obstacles(ego, scenario, k)

        obstacle_ahead = self.get_obstacle_ahead(scenario, ego, near_obstacles)

        return near_obstacles, obstacle_ahead

    def get_near_obstacles(self, ego, scenario, k):
        '''
        Computes obstacles around the ego vehicle
        :param ego: Data of ego vehicle
        :param scenario: Current scenario
        :param k: Time step of main function
        :return: near_obstacles
        '''
        near_obstacles = []
        o_type = ObstacleType

        # Radius in which obstacles get recognized
        r = ego._initial_state.velocity *2 + self._vehicleparameter.length / 2

        for static_obstacle in scenario.static_obstacles:
            if static_obstacle.obstacle_type is not o_type.ROAD_BOUNDARY:

                position = static_obstacle.occupancy_at_time(k).shape.center
                distance = np.linalg.norm(position - ego._initial_state.position)

                # At least one part of obstacle lies inside the circle
                if distance < r + static_obstacle.obstacle_shape.length/2:
                    near_obstacles.append([static_obstacle, position, 0,
                                           self.get_laneletid_of_obstacle(scenario, static_obstacle, k)])

        for dynamic_obstacle in scenario.dynamic_obstacles:
            if dynamic_obstacle.occupancy_at_time(k) is not None and dynamic_obstacle.obstacle_type is not o_type.ROAD_BOUNDARY:

                position = dynamic_obstacle.occupancy_at_time(k).shape.center
                distance = np.linalg.norm(position - ego._initial_state.position)

                # At least one part of obstacle lies inside the circle
                if distance < r + dynamic_obstacle.obstacle_shape.length/2:
                    near_obstacles.append([dynamic_obstacle, position, dynamic_obstacle.initial_state.velocity,
                                           self.get_laneletid_of_obstacle(scenario, dynamic_obstacle, k)])

        return near_obstacles

    def get_obstacle_ahead(self, scenario, ego, near_obstacles):
        '''
        Computes in front of the ego vehicle
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :param near_obstacles: Obstacles around car [obstacle, position, velocity, lanelet_id]
        :return: obstacle_ahead
        '''
        ego_position = ego._initial_state.position
        ego_laneletid = self.get_laneletid_of_egocar(scenario, ego)

        nearest_obstacle = None

        # Matrix to rotate lanelet-network around orientation of ego vehicle
        rotation_matrix = [[np.cos(ego._initial_state.orientation), -np.sin(ego._initial_state.orientation)],
                           [np.sin(ego._initial_state.orientation), np.cos(ego._initial_state.orientation)]]


        for obstacle in near_obstacles:

            # Compare lanelet-IDs
            if ego_laneletid == obstacle[3]:

                # Center obstacles around ego_position and rotate coordinate system
                transformed_obstacle_position = np.matmul(obstacle[1] - ego_position, rotation_matrix)

                # Obstacle is in front
                if transformed_obstacle_position[0] > 0:
                    print("Ahead Obstacle: ", obstacle[1])
                    if nearest_obstacle is None:
                        nearest_obstacle = [obstacle, transformed_obstacle_position[0]]
                    # Get the obstacle that is closest to the ego
                    elif transformed_obstacle_position[0] < nearest_obstacle[1]:
                        nearest_obstacle = [obstacle, transformed_obstacle_position[0]]

        if nearest_obstacle is not None:
            return nearest_obstacle[0]

        return None

    def get_obstacles_on_neighboring_lane(self, ego, scenario, near_obstacles, left = True):
        '''
        Computes and returns obstacles on the neighboring lane
        :param ego: Data of ego vehicle
        :param scenario: Current scenario
        :param near_obstacles: Obstacles around car [obstacle, position, velocity, lanelet_id]
        :param left: Boolean if left or right lane gets checked
        :return: Obstacles on neighboring lane
        '''
        lanelet_ego = self.get_laneletid_of_egocar(scenario, ego)

        if left:
            neighboring_lanelet_id = scenario.lanelet_network.find_lanelet_by_id(lanelet_ego).adj_left
        else:
            neighboring_lanelet_id = scenario.lanelet_network.find_lanelet_by_id(lanelet_ego).adj_right

        obstacles_on_neighboring_lane = []

        for obstacle in near_obstacles:
            # Compare lanelet-IDs
            if obstacle[3] == neighboring_lanelet_id:
                obstacles_on_neighboring_lane.append(obstacle)

        return obstacles_on_neighboring_lane

    # Get lanelet-IDs
    def get_laneletid_of_egocar(self, scenario, ego):
        '''
        Gets lanelet id of ego vehicle
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :return: lanelet id of ego vehicle
        '''
        x, y = ego._initial_state.position[0], ego._initial_state.position[1]
        heading = ego._initial_state.orientation
        x_1, y_1 = np.array([x, y]) + np.array([1, 1 * np.tan(heading)])
        tmp = np.array([[x, y], [x_1, y_1]])

        return scenario.lanelet_network.find_lanelet_by_position(tmp)[0][0]

    def get_laneletid_of_obstacle(self, scenario, obstacle, k):
        '''
        Gets lanelet id of an obstacle
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :param k: Time step of main function
        :return: lanelet id of an obstacle
        '''
        position = obstacle.occupancy_at_time(k).shape.center
        x, y = position[0], position[1]
        heading = obstacle.occupancy_at_time(k).shape.orientation
        x_1, y_1 = np.array([x, y]) + np.array([1, 1 * np.tan(heading)])
        tmp = np.array([[x, y], [x_1, y_1]])

        if scenario.lanelet_network.find_lanelet_by_position(tmp)[0]:
            return scenario.lanelet_network.find_lanelet_by_position(tmp)[0][0]
        else:
            return scenario.lanelet_network.find_lanelet_by_position(tmp)[1][0]
