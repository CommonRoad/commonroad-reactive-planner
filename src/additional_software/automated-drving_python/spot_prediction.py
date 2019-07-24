import sys
sys.path.append("/home/sebastian/eclipse-workspace/spot")
from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from spot import *
from commonroad.scenario.obstacle import Obstacle, DynamicObstacle, StaticObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from commonroad.geometry.shape import Polygon
from typing import Union, List
from commonroad.geometry.shape import Rectangle, Circle, ShapeGroup
import os


class SpotPrediction():
	def __init__(self, t0: float, time_steps: int, dt: float, net: Union[None, cpp_roadNetwork] = None, cpp_obstacles: Union[None, cpp_dynamicObstacle] = None, pyobstacles: Union[None, DynamicObstacle] = None, elapsed: float = None):
		self.t0 = t0
		self.time_steps = time_steps
		self.dt = dt
		self.roadNetwork = None
		self.cpp_obstacles = []
		self.py_obstacles = []
		self.elapsed_time = None

	def createRoadNetwork(self, py_lanelets: List[Lanelet]):

		# convert python-lanelets to cpp-lanelets
		lanelets = []
		for item in py_lanelets:
			laneletObj = cpp_vehicularLanelet()
			laneletObj.setId(item.lanelet_id)
			if item.speed_limit == None:
				item.speed_limit = 999999.0
			laneletObj.setSpeedLimit(item.speed_limit)
			v = cpp_vertice()
			for vert in item.left_vertices:
				v.x = vert[0]
				v.y = vert[1]
				laneletObj.addLeftVertice(v)
			for vert in item.right_vertices:
				v.x = vert[0]
				v.y = vert[1]
				laneletObj.addRightVertice(v)
			for vert in item.center_vertices:
				v.x = vert[0]
				v.y = vert[1]
				laneletObj.addCenterVertice(v)

			lanelets.append(laneletObj)

		# assign adjacent lanelets
		i = 0
		for item in py_lanelets:
			for pred in item.predecessor:
				for l in lanelets:
					if l.getId() == pred:
						lanelets[i].addPredecessor(l)
						break
				
			for suc in item.successor:
				for l in lanelets:
					if l.getId() == suc:
						lanelets[i].addSuccessor(l)
						break
			if item.adj_left != None:
				if item.adj_left_same_direction:
					lanelets[i].setLeftAdjacent(item.adj_left, 'same')
				else:
					lanelets[i].setLeftAdjacent(item.adj_left, 'opposite')
			if item.adj_right != None:
				if item.adj_right_same_direction:
					lanelets[i].setRightAdjacent(item.adj_right, 'same')
				else:
					lanelets[i].setRightAdjacent(item.adj_right, 'opposite')
			i = i + 1

		self.roadNetwork = getRoadNetwork(lanelets)

		
	def setObstacles(self, scenario: Scenario):
		# convert python-obstacles to cpp-obstacles
		obstacles = []
		for key, item in scenario._dynamic_obstacles.items():
			if item.obstacle_type == ObstacleType.CAR:
				dynamicObj = cpp_vehicle()
				dynamicObj.setId(item.obstacle_id)
				dynamicObj.setPosition(item.prediction.trajectory.state_list[0].position[0], item.prediction.trajectory.state_list[0].position[1])
				dynamicObj.setOrientation(item.prediction.trajectory.state_list[0].orientation)
				dynamicObj.setVelocity(item.prediction.trajectory.state_list[0].velocity)
				if isinstance(item._obstacle_shape, Rectangle):
					rect = cpp_rectangle()
					rect.setLength(item._obstacle_shape.length)
					rect.setWidth(item._obstacle_shape.width)
				dynamicObj.setShape(rect)
				obstacles.append(dynamicObj)
			elif item.obstacle_type == ObstacleType.PEDESTRIAN:
				dynamicObj = cpp_pedestrian()
				dynamicObj.setId(item.obstacle_id)
				dynamicObj.setPosition(item.prediction.trajectory.state_list[0].position[0], item.prediction.trajectory.state_list[0].position[1])
				dynamicObj.setPedestrianOrientation([item.prediction.trajectory.state_list[0].orientation])
				dynamicObj.setPedestrianVelocity([item.prediction.trajectory.state_list[0].velocity])
				if isinstance(item._obstacle_shape, Circle):
					circ = cpp_circle()
					circ.setRadius(item._obstacle_shape.radius)
					circ.setCenter(item._obstacle_shape.center[0], item.obstacle_shape.center[1])
				dynamicObj.setShape(circ)
				obstacles.append(dynamicObj)
		self.cpp_obstacles = obstacles
		self.py_obstacles = scenario._dynamic_obstacles

	def calcOccupancies(self, num_threads: int, verbose: int = None):
		if len(self.cpp_obstacles) == 0:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")
			return
		elif self.roadNetwork == None:
			print("No lanes! Please create lanes with SpotPredicition.createRoadNetwork(scenario.lanelet_network.lanelets)")
			return
		# call spot with converted elements	
		returned_objects = call_spot(self.cpp_obstacles, self.roadNetwork, [self.t0, self.dt, self.time_steps*self.dt+self.t0], num_threads)
		self.elapsed_time = returned_objects[1]
		if verbose is not None:
			print("Calculated occupancies for dynamic obstacles:", " ".join(str(item.getId()) for item in returned_objects[0]))
		# assign occupancies to python obstacles
		for item in returned_objects[0]:
			obst = self.py_obstacles[item.getId()]
			occ = item.getOccupancy()
			occMatrix = occ.getOccMatrix()
			occupancy_list = []
			for i in range(self.time_steps):
				sh = ShapeGroup()
				occ = Occupancy(i, sh)
				occupancy_list.append(occ)
			for laneOcc in occMatrix:
				i = 0
				for obj in laneOcc:
					vertices = []
					for vert in obj.vertices:
						vertices.append([vert.x, vert.y])
					if len(vertices) < 3:
						i = i + 1
						continue
					shape_obj = Polygon(vertices)
					occupancy_list[i].shape.shapes.append(shape_obj)
					i = i + 1
			obst.prediction = SetBasedPrediction(0, occupancy_list.copy())

	def setSpeedLimitOfLanes(self, limit: float):
		if self.roadNetwork is not None:
			for laneObj in self.roadNetwork.lanes:
				speedLimit = []
				for i in range(len(laneObj.getSpeedLimit())):
					speedLimit.append(limit)
				laneObj.setSpeedLimit(speedLimit)
		else:
			print("No lanes! Please create lanes with SpotPredicition.createRoadNetwork(scenario.lanelet_network.lanelets)")

	def setSpeedLimitOfLanelets(self, limit: float):
		if self.roadNetwork is not None:
			for laneletObj in self.roadNetwork.lanelets:
				laneletObj.setSpeedLimit(limit)
		else:
			print("Lanelets not set!")

	
	def restrictSpeedLimitOfLanes(self, limit: float):
		if self.roadNetwork is not None:
			for laneObj in self.roadNetwork.lanes:
				speedLimit = []
				for i in range(len(laneObj.getSpeedLimit())):
					if laneObj.getSpeedLimit()[i] < limit:
						speedLimit.append(laneObj.getSpeedLimit()[i])
					else:
						speedLimit.append(limit)
				laneObj.setSpeedLimit(speedLimit)
		else:
			print("No lanes! Please create lanes with SpotPredicition.createRoadNetwork(scenario.lanelet_network.lanelets)")
	
	def setAmaxOfVehicles(self, a_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_vehicle):
					item.setAmax(a_max)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setVmaxOfVehicles(self, v_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_vehicle):
					item.setVmax(v_max)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")				


	def setSpeedingFactorOfVehicles(self, factor: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_vehicle):
					item.setSpeedingFactor(factor)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")		

	# constraint C3 (not allowing backward driving)
	def setConstraint3OfVehicles(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_vehicle):
					item.setConstraint3(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")	

	# changing and crossing lanes is forbidden unless allowed by traffic regulations
	def setConstraint5OfVehicles(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_vehicle):
					item.setConstraint5(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")	

	def setVsOfVehicles(self, v_s: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_vehicle):
					item.setVs(v_s)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")	

	def setbStopOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setbStop(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")	

	def setbCrossOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setbCross(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")


	def setdPerpOfPedestrians(self, val: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setdPerp(val)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setComputeOccRuleBasedOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setComputeOccRuleBased(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")


	def setComputeOccDynamicBasedOfPedestrians(self, boolean: bool):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setComputeOccDynamicBased(boolean)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setNumVertOfPedestrians(self, num: int): 
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setNumVert(num)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setAmaxOfPedestrians(self, a_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setAmax(a_max)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setAstopOfPedestrians(self, a_stop: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setAstop(a_stop)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setVmaxOfPedestrians(self, v_max: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setVmax(a_stop)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setLaneWidthOfPedestrians(self, width: float):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setLaneWidth(width)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")

	def setNumLanesOfPedestrians(self, num: int):
		if len(self.cpp_obstacles) != 0:
			for item in self.cpp_obstacles:
				if isinstance(item, cpp_pedestrian):
					item.setNumLanes(num)
		else:
			print("Obstacles not set. Please set obstacles with SpotPredicition.setObstacles(scenario).")


	
