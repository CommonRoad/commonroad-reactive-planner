#include "../src/main.h"
#include "../src/world/obstacle/obstacle.h"
#include "../src/world/obstacle/dynamicObstacle.h"
#include "../src/world/obstacle/staticObstacle.h"
#include "../src/world/obstacle/vehicle.h"
#include "../src/world/obstacle/pedestrian.h"
#include "../src/world/lanelets/lanelet.h"
#include "../src/world/lanelets/vehicularLanelet.h"
#include "../src/geometry/rectangle.h"
#include "../src/world/lane/lane.h"
#include "../src/geometry/circle.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

struct roadNetwork {
	std::vector <vehicularLanelet> lanelets;
	std::vector <lane> lanes;
};

void run_unittests()
{
	int result = main();
}

std::tuple<std::vector<obstacle*>, double> pass_to_spot(py::list py_obstacles, py::object py_roadNetwork, py::list py_timeInterval, int num_threads)
{
	std::vector <obstacle*> obstacles;
	for (auto obst : py_obstacles)
	{
		if(py::isinstance<dynamicObstacle>(obst))
		{
			if(py::isinstance<vehicle>(obst))
			{
				vehicle* car = obst.cast<vehicle*>();
          		obstacles.push_back(car);
				continue;
			}
			
			else if(py::isinstance<pedestrian>(obst))
			{
				pedestrian* ped = obst.cast<pedestrian*>();
          		obstacles.push_back(ped);
				continue;
			}
		}
    }
	roadNetwork* net;
	
	if(py::isinstance<roadNetwork>(py_roadNetwork))
	{
			net = py_roadNetwork.cast<roadNetwork*>();
	}

	timeStruct timeInterval;
	timeInterval.startingTime = py_timeInterval[0].cast<double>();
	timeInterval.timeStep = py_timeInterval[1].cast<double>();
	timeInterval.ending = py_timeInterval[2].cast<double>();
		
	double elapsed = run_spot(&net->lanelets, &net->lanes, &obstacles, timeInterval, num_threads);
	return std::make_tuple(obstacles, elapsed);
}

void createRoadNetwork(roadNetwork* net, py::list py_lanelets)
{
	for (auto laneletObj : py_lanelets)
	{			
		if(py::isinstance<lanelet>(laneletObj))
		{
			vehicularLanelet newLanelet  = laneletObj.cast<vehicularLanelet>();
           	(*net).lanelets.push_back(newLanelet);
			continue;
		}
	}
	
	createLanesFromLanelets(&net->lanes, &net->lanelets);
	
	// compute the shortest path for every lane
	#pragma omp parallel for schedule(guided)
	for (size_t i = 0; i < (*net).lanes.size(); i++)
	{
		(*net).lanes[i].calculateShortestPath();
	}
}


class PyShape : public shape {
public:
    /* Inherit the constructors */
    using shape::shape;
};

class PyObstacle : public obstacle {
public:
    /* Inherit the constructors */
    using obstacle::obstacle;

    /* Trampoline (need one for each virtual function) */
	void computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval) override {
		PYBIND11_OVERLOAD_PURE(
            void, /* Return type */
            obstacle,      /* Parent class */
            computeOccupancyCore,          /* Name of function in C++ (must match Python name) */
            lanelets, lanes, timeInterval /* Argument(s) */
        );
	}
    std::vector <std::vector <vertice> > M1_accelerationBasedOccupancy(timeStruct timeInterval) override {
        PYBIND11_OVERLOAD_PURE(
            std::vector <std::vector <vertice> >, /* Return type */
            obstacle,      /* Parent class */
            M1_accelerationBasedOccupancy,          /* Name of function in C++ (must match Python name) */
            timeInterval      /* Argument(s) */
        );
    }
};

class PyDynamicObstacle : public dynamicObstacle {
public:
    using dynamicObstacle::dynamicObstacle; // Inherit constructors
	
	/* Trampoline (need one for each virtual function) */
	void computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval) override {
		PYBIND11_OVERLOAD_PURE(
            void, /* Return type */
            obstacle,      /* Parent class */
            computeOccupancyCore,          /* Name of function in C++ (must match Python name) */
            lanelets, lanes, timeInterval /* Argument(s) */
        );
	}
};

/*
class PyLanelet : public lanelet {
public:
    using lanelet::lanelet; // Inherit constructors
};
*/

PYBIND11_MODULE(spot, m) {
	py::class_<timeStruct>(m, "cpp_timeStruct")
		.def(py::init<>())
	    .def_readwrite("startingTime", &timeStruct::startingTime)
    	.def_readwrite("timeStep", &timeStruct::timeStep)
		.def_readwrite("ending", &timeStruct::ending);
	
	py::class_<vertice>(m, "cpp_vertice")
		.def(py::init<>())
	    .def_readwrite("x", &vertice::x)
		.def_readwrite("y", &vertice::y);
	
	py::class_<roadNetwork>(m, "cpp_roadNetwork")
		.def(py::init<>())
	    .def_readwrite("lanelets", &roadNetwork::lanelets)
		.def_readwrite("lanes", &roadNetwork::lanes);
	
	py::class_<occTypes>(m, "cpp_occTypes")
		.def(py::init<>())
	    .def_readwrite("forLane", &occTypes::forLane)
		.def_readwrite("timeInterval", &occTypes::timeInterval)
    	.def_readwrite("vertices", &occTypes::vertices);
	
	py::class_<shape, PyShape, std::unique_ptr<shape, py::nodelete>> shape(m, "cpp_shape");
	shape
		.def(py::init<>());
	
	py::class_<rectangle, std::unique_ptr<rectangle, py::nodelete>>(m, "cpp_rectangle", shape)
		.def(py::init<>())
		.def("setLength", &rectangle::setLength)
		.def("setWidth", &rectangle::setWidth)
		.def("getLength", &rectangle::getLength)
		.def("getWidth", &rectangle::getWidth);
	
	py::class_<circle, std::unique_ptr<circle, py::nodelete>>(m, "cpp_circle", shape)
		.def(py::init<>())
		.def("setRadius", &circle::setRadius)
		.def("setCenter", &circle::setCenter)
		.def("getRadius", &circle::getRadius)
		.def("getCenter", &circle::getCenter);
	
	py::class_<occupancy, std::unique_ptr<occupancy, py::nodelete>>(m, "cpp_occupancy")
		.def(py::init<>())
		.def("getOccMatrix", &occupancy::getOccMatrix);
	
	py::class_<obstacle, PyObstacle, std::unique_ptr<obstacle, py::nodelete>> obstacle(m, "cpp_obstacle");
    obstacle
        .def(py::init<>())
        	.def("setId", &obstacle::setId)
		.def("setPosition", &obstacle::setPosition)
		.def("setOrientation", &obstacle::setOrientation)
		.def("addInLane", &obstacle::addInLane)
		.def("setShape", &obstacle::setShape)
		.def("getId", &obstacle::getId)
		.def("getXpos", &obstacle::getXpos)
		.def("getYpos", &obstacle::getYpos)
		.def("getOrientation", &obstacle::getOrientation)
		.def("getInLane", &obstacle::getInLane)
		.def("getShape", &obstacle::getShape)
		.def("getOccupancy", &obstacle::getOccupancy)
		.def("updateInLane", &obstacle::updateInLane);

	py::class_<staticObstacle, std::unique_ptr<staticObstacle, py::nodelete>>(m, "cpp_staticObstacle", obstacle)
		.def(py::init<>())
		.def("M1_accelerationBasedOccupancy", &staticObstacle::M1_accelerationBasedOccupancy)
		.def("computeOccupancyCore", &staticObstacle::computeOccupancyCore);
	
	py::class_<dynamicObstacle, PyDynamicObstacle, std::unique_ptr<dynamicObstacle, py::nodelete>> dynamicObstacle(m, "cpp_dynamicObstacle", obstacle);
    dynamicObstacle
		.def(py::init<>())
		.def("setVelocity", &dynamicObstacle::setVelocity)
		.def("setAcceleration", &dynamicObstacle::setAcceleration)
		.def("setVmax", &dynamicObstacle::setVmax)
		.def("setAmax", &dynamicObstacle::setAmax)
		.def("setSpeedingFactor", &dynamicObstacle::setSpeedingFactor)
		.def("getVelocity", &dynamicObstacle::getVelocity)
		.def("getAcceleration", &dynamicObstacle::getAcceleration)
		.def("getVmax", &dynamicObstacle::getVmax)
		.def("getAmax", &dynamicObstacle::getAmax)
		.def("M1_accelerationBasedOccupancy", &dynamicObstacle::M1_accelerationBasedOccupancy);
	
	py::class_<vehicle, std::unique_ptr<vehicle, py::nodelete>>(m, "cpp_vehicle", dynamicObstacle)
		.def(py::init<>())
		.def("setVs", &vehicle::setVs)
		.def("setPowerMax", &vehicle::setPowerMax)
		.def("getVs", &vehicle::getVs)
		.def("getPowerMax", &vehicle::getPowerMax)
		.def("manageConstraints", &vehicle::manageConstraints)
		.def("computeOccupancyCore", &vehicle::computeOccupancyCore)
		.def("M2_laneFollowingOccupancy", &vehicle::M2_laneFollowingOccupancy)
		.def("setConstraint3", &vehicle::setConstraint3)
		.def("setConstraint5", &vehicle::setConstraint5)
		.def("getConstraint3", &vehicle::getConstraint3)
		.def("getConstraint5", &vehicle::getConstraint5);
	
	py::class_<pedestrian, std::unique_ptr<pedestrian, py::nodelete>>(m, "cpp_pedestrian", dynamicObstacle)
		.def(py::init<>())
		.def("setNumVert", &pedestrian::setNumVert)
		.def("setComputeOccDynamicBased", &pedestrian::setComputeOccDynamicBased)
		.def("setComputeOccRuleBased", &pedestrian::setComputeOccRuleBased)
		.def("setPedestrianOrientation", &pedestrian::setPedestrianOrientation)
		.def("setPedestrianVelocity", &pedestrian::setPedestrianVelocity)
		.def("setbCross", &pedestrian::setbCross)
		.def("setbStop", &pedestrian::setbStop)
		.def("setComputeOccDynamicBased", &pedestrian::setComputeOccDynamicBased)
		.def("setComputeOccRuleBased", &pedestrian::setComputeOccRuleBased)
		.def("getComputeOccDynamicBased", &pedestrian::getComputeOccDynamicBased)
		.def("getComputeOccRuleBased", &pedestrian::getComputeOccRuleBased)
		.def("getdPerp", &pedestrian::getdPerp)
		.def("setdPerp", &pedestrian::setdPerp)
		.def("getNumVert", &pedestrian::getNumVert)
		.def("getAstop", &pedestrian::getAstop)
		.def("setAstop", &pedestrian::setAstop)
		.def("getbStop", &pedestrian::getbStop)
		.def("getbCross", &pedestrian::getbCross)
		.def("getNumLanes", &pedestrian::getNumLanes)
		.def("getLaneWidth", &pedestrian::getLaneWidth)
		.def("setNumLanes", &pedestrian::setNumLanes)
		.def("setLaneWidth", &pedestrian::setLaneWidth)
		.def("getComputeOccDynamicBased", &pedestrian::getComputeOccDynamicBased)
		.def("getComputeOccRuleBased", &pedestrian::getComputeOccRuleBased)
		.def("getPedestrianOrientation", &pedestrian::getPedestrianOrientation)
		.def("getPedestrianVelocity", &pedestrian::getPedestrianVelocity)
		.def("accOccPedestrian", &pedestrian::accOccPedestrian);
		
	py::class_<lanelet, std::unique_ptr<lanelet, py::nodelete>> lanelet(m, "cpp_lanelet");
	lanelet
		.def(py::init<>())
		.def("setId", &lanelet::setId)
		.def("addLeftVertice", &lanelet::addLeftVertice)
		.def("addRightVertice", &lanelet::addRightVertice)
		.def("addCenterVertice", &lanelet::addCenterVertice)
		.def("createCenterVertices", &lanelet::createCenterVertices)
		.def("getId", &lanelet::getId)
		.def("getLeftBorder", &lanelet::getLeftBorder)
		.def("getRightBorder", &lanelet::getRightBorder)
		.def("getCenter", &lanelet::getCenter);
	
	py::class_<vehicularLanelet, std::unique_ptr<vehicularLanelet, py::nodelete>>(m, "cpp_vehicularLanelet", lanelet)
		.def(py::init<>())
		.def("setSpeedLimit", &vehicularLanelet::setSpeedLimit)
		.def("addPredecessor", &vehicularLanelet::addPredecessor)
		.def("addSuccessor", &vehicularLanelet::addSuccessor)
		.def("setLeftAdjacent", &vehicularLanelet::setLeftAdjacent)
		.def("setRightAdjacent", &vehicularLanelet::setRightAdjacent)
		.def("getSpeedLimit", &vehicularLanelet::getSpeedLimit)
		.def("getPredecessors", &vehicularLanelet::getPredecessors)
		.def("getSuccessors", &vehicularLanelet::getSuccessors)
		.def("getAdjacentRight", &vehicularLanelet::getAdjacentRight)
		.def("getAdjacentLeft", &vehicularLanelet::getAdjacentLeft);
	
	py::class_<lane, std::shared_ptr<lane> /* <- holder type */>(m, "cpp_lane")
		.def(py::init<>())
		.def("setId", &lane::setId)
		.def("setLeftBorder", &lane::setLeftBorder)
		.def("setRightBorder", &lane::setRightBorder)
		.def("setLeftBorderVertices", &lane::setLeftBorderVertices)
		.def("addAssemblingLanelet", &lane::addAssemblingLanelet)
		.def("setAssemblingLanelet", &lane::setAssemblingLanelet)
		.def("setSpeedLimit", &lane::setSpeedLimit)
		//.def("setCenterVertices", &lane::setCenterVertices)
		.def("setAdjacentLeft", &lane::setAdjacentLeft)
		.def("setAdjacentRight", &lane::setAdjacentRight)
		.def("addCenterVertice", &lane::addCenterVertice)
		.def("addCenterVertices", &lane::addCenterVertices)
		.def("setLeftBorderDistances", &lane::setLeftBorderDistances)
		.def("setRightBorderDistances", &lane::setRightBorderDistances)
		.def("setLeftBorderCurvature", &lane::setLeftBorderCurvature)
		.def("setRightBorderCurvature", &lane::setRightBorderCurvature)
		.def("setShortestPath", &lane::setShortestPath)
		.def("getId", &lane::getId)
		.def("getLeftBorder", &lane::getLeftBorder)
		.def("getLeftBorderVertices", &lane::getLeftBorderVertices)
		.def("getLeftBorderDistances", &lane::getLeftBorderDistances)
		.def("getLeftBorderCurvature", &lane::getLeftBorderCurvature)
		.def("getRightBorderDistances", &lane::getRightBorderDistances)
		.def("getRightBorderCurvature", &lane::getRightBorderCurvature)
		.def("getRightBorder", &lane::getRightBorder)
		.def("getRightBorderVertices", &lane::getRightBorderVertices)
		.def("getAssLanelets", &lane::getAssLanelets)
		.def("getSpeedLimit", &lane::getSpeedLimit)
		.def("getMaxSpeedLimit", &lane::getMaxSpeedLimit)
		.def("getCenter", &lane::getCenter)
		.def("getAdjacentLeft", &lane::getAdjacentLeft)
		.def("getAdjacentLeftDir", &lane::getAdjacentLeftDir)
		.def("getAdjacentRight", &lane::getAdjacentRight)
		.def("getAdjacentRightDir", &lane::getAdjacentRightDir)
		.def("getCenterVertices", &lane::getCenterVertices)
		.def("getShortestPath", &lane::getShortestPath)
		.def("getOptVelProfile", &lane::getOptVelProfile)
		.def("calculateShortestPath", &lane::calculateShortestPath)
		.def("calculateOptimalVelocityProfile", &lane::calculateOptimalVelocityProfile);

	m.def("run_unittests", &run_unittests);
	
	m.def("getRoadNetwork", [](py::list py_lanelets)
	{
		roadNetwork* net = new roadNetwork;
		createRoadNetwork(net, py_lanelets);
		return net;
	}, py::return_value_policy::take_ownership);

	m.def("call_spot", [](py::list py_obstacles, py::object py_roadNetwork, py::list py_timeInterval, int num_threads)
	{
		return pass_to_spot(py_obstacles, py_roadNetwork, py_timeInterval, num_threads);
	}, py::return_value_policy::copy);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
