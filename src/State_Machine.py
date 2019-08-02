from transitions import Machine
from commonroad.scenario.obstacle import ObstacleType
from parameter import VehicleParameter, PlanningParameter
import numpy as np
from scipy import spatial
from route_planner import RoutePlanner

class CarHighlevelStates(object):

    # define states
    states =['following', 'lane_change_left']
    
    # define transitions
    transitions = [{'trigger': 'Car_ahead_too_slow',    'source': 'following',          'dest': 'lane_change_left',
                    'before': 'set_lanelet_ids', 'conditions': 'lanelet_ids_valid'},

                   {'trigger': 'on_new_centerline',     'source': 'lane_change_left',   'dest': 'following',
                    'before': 'reset_lanelet_ids'}]

    def __init__(self):

        # define values for old/new lanelet
        self.old_lanelet = None
        self.new_lanelet = None

        # define new machine
        self._machine = Machine(model=self, states=CarHighlevelStates.states,
                               transitions=CarHighlevelStates.transitions, initial='following')

    def set_lanelet_ids(self, old_lanelet = None, new_lanelet = None):
        """
        Set current lanelet and lane to the left/right
        :param old_lanelet: current lanelet of ego vehicle
        :param new_lanelet: lanelet ego wants to merge on 
        """
        self.old_lanelet = old_lanelet
        self.new_lanelet = new_lanelet

    def reset_lanelet_ids(self, old_lanelet = None, new_lanelet = None):
        """
        Reset old and new lanelet
        :param old_lanelet: current lanelet of ego vehicle
        :param new_lanelet: lanelet ego wants to merge on 
        """
        self.old_lanelet = None
        self.new_lanelet = None

    def lanelet_ids_valid(self, old_lanelet = None, new_lanelet = None):
        """
        Check if value of old/new lanelet is still valid
        :param old_lanelet: current lanelet of ego vehicle
        :param new_lanelet: lanelet ego wants to merge on
        """
        if old_lanelet is not None and new_lanelet is not None:
            return True
        return False


class StateMachine:
    def __init__(self, scenario):

        # create scenario object
        self._scenario = scenario

        # create states object
        self._states = CarHighlevelStates()

        # create vehicle parameter object
        self._vehicleparameter = VehicleParameter()

        # create rout planner object
        self._route_planner = RoutePlanner(scenario.scenario_path)

    def init_reference_path(self):
        """
        Initializes reference path
        :return: Reference path to be set
        """
        # get source position of initial state
        source_position = self._route_planner.planning_problem.initial_state.position

        # get initial reference path
        reference_path = self._route_planner.set_reference_lane(0, source_position)

        return reference_path

    def check_if_velocity_is_too_slow(self, ego, obstacle_ahead):
        """
        Checks if the velocity of the car ahead is too slow
        :param ego: Data of ego vehicle
        :param obstacle_ahead: Obstacle ahead of ego car
        :return:
        """
        # Velocity differences between ego and the obstacle ahead
        vel_difference_overtaking = 0
        vel_difference = 0

        # Should only overtake obstacles that are significantly slower
        allowed_velocity_difference = ego._initial_state.velocity / 10

        # Static obstacle is ahead
        if not hasattr(obstacle_ahead[0].initial_state, 'velocity'):
            print("Static obstacle ahead!")

            # compute velocity differences
            vel_difference_overtaking = ego._initial_state.velocity - allowed_velocity_difference
            vel_difference = ego._initial_state.velocity

        # Dynamic obstacle is ahead
        elif obstacle_ahead[0].initial_state.velocity < ego._initial_state.velocity + allowed_velocity_difference:

            # compute velocity differences
            vel_difference_overtaking = obstacle_ahead[0].initial_state.velocity \
                                            - (ego._initial_state.velocity + allowed_velocity_difference)
            vel_difference = obstacle_ahead[0].initial_state.velocity - ego._initial_state.velocity

        return vel_difference_overtaking, vel_difference

    def check_current_state(self, ego, reference_path, near_obstacles, obstacle_ahead, k):
        """
        Main function to check and eventually change the current state
        :param ego: Data of ego vehicle
        :param reference_path: Current reference path
        :param near_obstacles: Obstacles around car [obstacle, position, velocity, lanelet_id]
        :param obstacle_ahead: Obstacle ahead of ego car
        :param k: Time step of main function
        :return: Velocity ego car, new reference path
        """
        print("Current State: ", self._states.state)

        # Get ego velocity and position
        ego_velocity = ego._initial_state.velocity
        ego_position = ego._initial_state.position

        # State is "lane_change_left"
        if self._states.state == 'lane_change_left':

            # Check if ego car is on new lane
            if self.check_if_car_on_new_lanelet(self._scenario.scenario_set, ego):
                
                # Check if ego car is near new center line
                if self.check_if_car_on_new_centervertice(self._scenario.scenario_set, ego):
                    
                    # Change state _machine state to "following"
                    self._states.on_new_centerline()

            return ego_velocity, reference_path

        # State is "following"
        if self._states.state == 'following':

            # define if velocity has to be changed
            velocity_has_to_be_changed = False

            # Check if lanlet on the left exists
            if self.check_for_possible_overtaking_lanelet(self._scenario.scenario_set, ego):
                
                # Check if distance to obstacles on neighboring lane is big enough
                if self.check_lane_change_possible(self._scenario.scenario_set, ego, near_obstacles, obstacle_ahead, left=True):

                    # Set old and new lanlet for further checks
                    old_lanelet = self.get_laneletid_of_egocar(self._scenario.scenario_set, ego)
                    new_lanelet = self._scenario.scenario_set.lanelet_network.find_lanelet_by_id\
                        (self.get_laneletid_of_egocar(self._scenario.scenario_set, ego)).adj_left

                    # Set new state to "lane_change_left"
                    self._states.Car_ahead_too_slow(old_lanelet,new_lanelet)

                    # Set new reference path
                    reference_path = self._route_planner.set_reference_lane(-1, ego_position)

                else:
                    # No lanechange possible due to neighboring obstacles
                    velocity_has_to_be_changed = True

            else:
                # No lanechange possible due to missing lanelet on the left
                velocity_has_to_be_changed = True

            # Change velocity if necessary
            if velocity_has_to_be_changed:

                # If obstacle in front is static
                if not hasattr(obstacle_ahead[0].initial_state, 'velocity'):
                    ego_velocity = 0

                # If obstacle in front is dynamic
                if ego_velocity != 0:
                    ego_velocity = obstacle_ahead[0].initial_state.velocity

                print("Changed velocity to: ", ego_velocity)

            return ego_velocity, reference_path

    def check_if_car_on_new_lanelet(self, scenario, ego):
        """
        Checks if ego vehicle is on new lanelet
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :return: True if ego vehicle is on new lanelet
        """
        # get ego lanelet id
        ego_laneletid = self.get_laneletid_of_egocar(scenario, ego)

        # ego is on new lanelet
        if ego_laneletid == self._states.new_lanelet:
            return True

        return False

    def check_if_car_on_new_centervertice(self, scenario, ego):
        """
        Checks if ego vehicle is on new center vertices
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :return: True if ego vehicle is on new center vertices
        """
        # get get lanelet of new lanelet id
        new_lanelet = scenario.lanelet_network.find_lanelet_by_id(self._states.new_lanelet)
        
        # get ego position
        ego_position = ego._initial_state.position

        # compute distance to center vertice
        distance, index = spatial.KDTree(new_lanelet.center_vertices).query(ego_position)

        # ego is close enough to center vertice
        if distance < 0.5:
            return True

        return False

    def check_for_possible_overtaking_lanelet(self, scenario, ego):
        """
        Checks if ego vehicle has a possible overtaking lanelet
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :return:
        """
        # get ego lanelet id
        ego_laneletid = self.get_laneletid_of_egocar(scenario, ego)

        # check if overtaking lanelet exists
        if scenario.lanelet_network.find_lanelet_by_id(ego_laneletid).adj_left_same_direction:
            print('Overtaking lanelet exists')
            return True

        return False

    def check_lane_change_possible(self, scenario, ego, near_obstacles, obstacle_ahead, left=True):
        """
        Checks distance to obstacles on neighboring lane and decides if a lane change is possible
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :param near_obstacles: Obstacles around car [obstacle, position, velocity, lanelet_id]
        :param obstacle_ahead: Obstacle ahead of ego car
        :param left: Boolean if left or right lane gets checked
        :return: True if lane Change possible
        """
        # get obstacles on neighboring lane
        obstacles_on_neighboring_lane = self.get_obstacles_on_neighboring_lane(ego, scenario, near_obstacles, left)
        
        # Number is used to count down all obstacles that fulfill the checks (see below)
        number_neighbors = len(obstacles_on_neighboring_lane)

        # get ego position and parameter
        ego_position = ego._initial_state.position
        ego_parameter = self._vehicleparameter

        # set safety margin
        safety_margin = ego._initial_state.velocity / 10

        # set rotation matrix
        rotation_matrix = [[np.cos(ego._initial_state.orientation), -np.sin(ego._initial_state.orientation)],
                           [np.sin(ego._initial_state.orientation), np.cos(ego._initial_state.orientation)]]

        # go through obstacles on neighboring lane
        for obstacle in obstacles_on_neighboring_lane:

            # get obstacle position
            obstacle_position = obstacle[1]
            
            # transform obstacle position
            transformed_obstacle_position = np.matmul(obstacle_position - ego_position, rotation_matrix)

            # Obstacle is in front
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

    def get_obstacles_around(self, ego, k):
        """
        Computes obstacles around and in front of the ego vehicle
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :param k: Time step of main function
        :return: near_obstacles, obstacle_ahead
        """
        # compute near obstacles around ego vehicle
        near_obstacles = self.get_near_obstacles(ego, self._scenario.scenario_set, k)

        # compute obstacle in front of ego vehicle
        obstacle_ahead = self.get_obstacle_ahead(self._scenario.scenario_set, ego, near_obstacles)

        return near_obstacles, obstacle_ahead

    def get_near_obstacles(self, ego, scenario, k):
        """
        Computes obstacles around the ego vehicle
        :param ego: Data of ego vehicle
        :param scenario: Current scenario
        :param k: Time step of main function
        :return: near_obstacles
        """
        # define list of near obstacles
        near_obstacles = []
        
        # define obstacle type object
        o_type = ObstacleType

        # Radius in which obstacles get recognized
        r = ego._initial_state.velocity *2 + self._vehicleparameter.length / 2

        # go through each static obstacle
        for static_obstacle in scenario.static_obstacles:
            if static_obstacle.obstacle_type is not o_type.ROAD_BOUNDARY:

                # get current position of obstacle
                position = static_obstacle.occupancy_at_time(k).shape.center
                
                # compute distance to obstacle
                distance = np.linalg.norm(position - ego._initial_state.position)

                # At least one part of obstacle lies inside the circle
                if distance < r + static_obstacle.obstacle_shape.length/2:
                    near_obstacles.append([static_obstacle, position, 0,
                                           self.get_laneletid_of_obstacle(scenario, static_obstacle, k)])

        # go through each dynamic obstacle
        for dynamic_obstacle in scenario.dynamic_obstacles:
            if dynamic_obstacle.occupancy_at_time(k) is not None and dynamic_obstacle.obstacle_type is not o_type.ROAD_BOUNDARY:

                # get current position of obstacle
                position = dynamic_obstacle.occupancy_at_time(k).shape.center

                # compute distance to obstacle
                distance = np.linalg.norm(position - ego._initial_state.position)

                # At least one part of obstacle lies inside the circle
                if distance < r + dynamic_obstacle.obstacle_shape.length/2:
                    near_obstacles.append([dynamic_obstacle, position, dynamic_obstacle.initial_state.velocity,
                                           self.get_laneletid_of_obstacle(scenario, dynamic_obstacle, k)])

        return near_obstacles

    def get_obstacle_ahead(self, scenario, ego, near_obstacles):
        """
        Computes in front of the ego vehicle
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :param near_obstacles: Obstacles around car [obstacle, position, velocity, lanelet_id]
        :return: obstacle_ahead
        """
        # get ego position and lanelet id
        ego_position = ego._initial_state.position
        ego_laneletid = self.get_laneletid_of_egocar(scenario, ego)

        # set variable for nearest obstacle
        nearest_obstacle = None

        # Matrix to rotate lanelet-network around orientation of ego vehicle
        rotation_matrix = [[np.cos(ego._initial_state.orientation), -np.sin(ego._initial_state.orientation)],
                           [np.sin(ego._initial_state.orientation), np.cos(ego._initial_state.orientation)]]

        # go through each near obstacle
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

        # at least one obstacle in front exists
        if nearest_obstacle is not None:
            return nearest_obstacle[0]

        return None

    def get_obstacles_on_neighboring_lane(self, ego, scenario, near_obstacles, left = True):
        """
        Computes and returns obstacles on the neighboring lane
        :param ego: Data of ego vehicle
        :param scenario: Current scenario
        :param near_obstacles: Obstacles around car [obstacle, position, velocity, lanelet_id]
        :param left: Boolean if left or right lane gets checked
        :return: Obstacles on neighboring lane
        """
        # get ego lanelet id
        ego_laneletid = self.get_laneletid_of_egocar(scenario, ego)

        # change to the left is wanted
        if left:
            neighboring_lanelet_id = scenario.lanelet_network.find_lanelet_by_id(ego_laneletid).adj_left

        # change to the right is wanted
        else:
            neighboring_lanelet_id = scenario.lanelet_network.find_lanelet_by_id(ego_laneletid).adj_right

        # define list of neighboring obstacles
        obstacles_on_neighboring_lane = []

        # go through each near obstacle
        for obstacle in near_obstacles:
            
            # Compare lanelet-IDs
            if obstacle[3] == neighboring_lanelet_id:
                
                # obstacle lies on left/right lane
                obstacles_on_neighboring_lane.append(obstacle)

        return obstacles_on_neighboring_lane

    def get_laneletid_of_egocar(self, scenario, ego):
        """
        Gets lanelet id of ego vehicle
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :return: lanelet id of ego vehicle
        """
        # compute x,y-Axis value
        x, y = ego._initial_state.position[0], ego._initial_state.position[1]
        
        # get orientation
        heading = ego._initial_state.orientation
        x_1, y_1 = np.array([x, y]) + np.array([1, 1 * np.tan(heading)])
        tmp = np.array([[x, y], [x_1, y_1]])
        
        # return lanelet id based on current position
        return scenario.lanelet_network.find_lanelet_by_position(tmp)[0][0]

    def get_laneletid_of_obstacle(self, scenario, obstacle, k):
        """
        Gets lanelet id of an obstacle
        :param scenario: Current scenario
        :param ego: Data of ego vehicle
        :param k: Time step of main function
        :return: lanelet id of an obstacle
        """
        # compute x,y-Axis value
        position = obstacle.occupancy_at_time(k).shape.center
        x, y = position[0], position[1]
        
        # get current orientation
        heading = obstacle.occupancy_at_time(k).shape.orientation
        x_1, y_1 = np.array([x, y]) + np.array([1, 1 * np.tan(heading)])
        tmp = np.array([[x, y], [x_1, y_1]])

        # return lanelet id based on current position
        if scenario.lanelet_network.find_lanelet_by_position(tmp)[0]:
            return scenario.lanelet_network.find_lanelet_by_position(tmp)[0][0]
        else:
            return scenario.lanelet_network.find_lanelet_by_position(tmp)[1][0]
