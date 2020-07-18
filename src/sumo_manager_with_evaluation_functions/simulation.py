import os
from copy import deepcopy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.scenario.scenario import Scenario
from cr2sumo.interface.sumo_interface import SumoInterface
from cr2sumo.rpc.sumo_client import SumoRPCClient
from cr2sumo.visualization.video import create_video
from sumo_config.default import SumoCommonRoadConfig
from evaluation import evaluation

class simulation:
    scenario: Scenario              #for storing the complete scenario in the end
    conf: SumoCommonRoadConfig      #to keep possibility to rerun
    data_all_timesteps: []          #all the data from the simulation to save method calls

    def __init__(self, scenario_name: str):
        #startup to simulate the scenario (from simualte_scenario)
        scenario_folder: str = os.path.join(os.getcwd(), "scenarios", "cr_scenarios")
        print('Simulating {}'.format(scenario_name))
        conf = SumoCommonRoadConfig()
        conf.scenario_name = scenario_name
        self.conf = conf
        sumo_interface = SumoInterface()
        sumo_client: SumoRPCClient = sumo_interface.start_simulator()
        cr_file = os.path.join(scenario_folder, conf.scenario_name + '.xml')
        sumo_client.send_commonroad_scenario(conf.scenario_name, cr_file)
        sumo_client.initialize(conf)

        dyn_obstacle_data = []              #list for the dynamical object data
        ego_vehicles_data = []              #list for the ego vehicle data

        #for loop iterating through all time steps
        for t in range(conf.simulation_steps):
            ego_vehicles = sumo_client.ego_vehicles
            time_step = sumo_client.current_time_step
            commonroad_scenario: Scenario = sumo_client.commonroad_scenario_at_time_step(time_step)

            dyn_obsticles_timestep = []     #list to collect the data from all dynamic obstacles for time step t
            ego_vehicle_timestep = []       #list to collect the data from all ego vehicles for time step t
            for i in commonroad_scenario.dynamic_obstacles:
                # generating list from current timestep data
                dyn_obsticles_timestep.append([i.initial_state.__getattribute__("position"),
                                               i.initial_state.__getattribute__("acceleration"),
                                               i.initial_state.__getattribute__("velocity"),
                                               i.initial_state.__getattribute__("orientation")])

            dyn_obstacle_data.append(dyn_obsticles_timestep)            #add time step line to the main list

            # plan trajectories for all ego vehicles
            for id, ego_vehicle in ego_vehicles.items():
                current_state = ego_vehicle.current_state
                state = deepcopy(current_state)
                state.time_step = 1
                ego_trajectory = [state]
                ego_vehicle.set_planned_trajectory(ego_trajectory)
                #generating list from current timestep data

                ego_vehicle_timestep.append(
                    [current_state.__getattribute__("position"),
                     current_state.__getattribute__("acceleration"),
                     current_state.__getattribute__("velocity"),
                     current_state.__getattribute__("orientation")])

            ego_vehicles_data.append(ego_vehicle_timestep)              #add time step line to the main list
            sumo_client.simulate_step()
        self.scenario = sumo_client.commonroad_scenarios_all_time_steps()
        sumo_client.stop()
        sumo_interface.stop_simulator()
        self.data_all_timesteps = [ego_vehicles_data, dyn_obstacle_data]    #storing all the data in the object attribute

    def visualize(self):
        #to do, as resimualating is necessary at the moment
        sumo_interface = SumoInterface()
        sumo_client: SumoRPCClient = sumo_interface.start_simulator()
        cr_file = os.path.join(os.path.join(os.getcwd(), "scenarios", "cr_scenarios"), self.conf.scenario_name + '.xml')
        sumo_client.send_commonroad_scenario(self.conf.scenario_name, cr_file)
        sumo_client.initialize(self.conf)

        for t in range(self.conf.simulation_steps):
            ego_vehicles = sumo_client.ego_vehicles

            # plan trajectories for all ego vehicles
            for id, ego_vehicle in ego_vehicles.items():
                current_state = ego_vehicle.current_state
                state = deepcopy(current_state)
                state.time_step = 1
                ego_trajectory = [state]
                ego_vehicle.set_planned_trajectory(ego_trajectory)

            sumo_client.simulate_step()
        sumo_client.stop()
        # Create video and plot the simulation
        output_folder = "./"
        print("Creating video")
        create_video(sumo_client, self.conf.video_start, self.conf.video_end, output_folder)
        print("Video created")
        scenario = sumo_client.commonroad_scenarios_all_time_steps()
        plt.clf()
        draw_object(scenario)
        plt.autoscale()
        plt.axis('equal')
        print("Plotting scenario")
        plt.show()
        sumo_interface.stop_simulator()

s = simulation('a9')
#s.visualize()
print(evaluation.test_max_speed(s.data_all_timesteps))
