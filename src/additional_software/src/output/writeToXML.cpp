#include "../inputs/pugi_xml/pugixml.hpp"
#include "writeToXML.h"
#include <ctime>
#include <math.h>

template <typename T>
std::string ToString(T const & in_val)
{
    return std::to_string(in_val);
}

void writeToXML(std::vector <vehicularLanelet> lanelets, std::vector <obstacle*> obstacles, std::string xmlFile, float timeStepSize)
{

	// get date
	time_t t = time(0);   // get time now
	struct tm * now = localtime( & t );
	// construct string from given int data
	std::string date = std::to_string(now->tm_mday);
	date += "-";
	date += std::to_string(now->tm_mon + 1);
	date += "-";
	date += std::to_string(now->tm_year + 1900);

	// create new xml document
	pugi::xml_document doc;

	// add node
	pugi::xml_node commonRoad = doc.append_child("commonRoad");

	// add attributes to common road start node
	commonRoad.append_attribute("benchmarkID") = xmlFile.c_str();
	commonRoad.append_attribute("commonRoadVersion") ="2017a";
	commonRoad.append_attribute("date") = date.c_str();
	timeStepSize = floor( timeStepSize * 100.00 + 0.5 ) / 100.00; // round to two decimal places
	commonRoad.append_attribute("timeStepSize") = timeStepSize;

	// write lanelets
	for (size_t i = 0; i < lanelets.size(); i++)
	{
		// add lanelet node with id
		pugi::xml_node curLanelet = commonRoad.append_child("lanelet");
		curLanelet.append_attribute("id") = lanelets[i].getId();

		// add left bound vertices
		pugi::xml_node leftBound = curLanelet.append_child("leftBound");
		for (size_t j = 0; j < lanelets[i].getLeftBorder().size(); j++)
		{
			pugi::xml_node point = leftBound.append_child("point");
			vertice curVertice = lanelets[i].getLeftBorder()[j];
			pugi::xml_node x = point.append_child("x");
			x.append_child(pugi::node_pcdata).set_value(ToString(curVertice.x).c_str());
			pugi::xml_node y = point.append_child("y");
			y.append_child(pugi::node_pcdata).set_value(ToString(curVertice.y).c_str());
		}

		// add right bound vertices
		pugi::xml_node rightBound = curLanelet.append_child("rightBound");
		for (size_t j = 0; j < lanelets[i].getRightBorder().size(); j++)
		{
			pugi::xml_node point = rightBound.append_child("point");
			vertice curVertice = lanelets[i].getRightBorder()[j];
			pugi::xml_node x = point.append_child("x");
			x.append_child(pugi::node_pcdata).set_value(ToString(curVertice.x).c_str());
			pugi::xml_node y = point.append_child("y");
			y.append_child(pugi::node_pcdata).set_value(ToString(curVertice.y).c_str());
		}

		// add predecessors
		for (size_t j = 0; j < lanelets[i].getPredecessors().size(); j++)
		{
			pugi::xml_node predecessor = curLanelet.append_child("predecessor");
			predecessor.append_attribute("ref") = lanelets[i].getPredecessors()[j]->getId();
		}

		// add successors
		for (size_t j = 0; j < lanelets[i].getSuccessors().size(); j++)
		{
			pugi::xml_node succecessor = curLanelet.append_child("succecessor");
			succecessor.append_attribute("ref") = lanelets[i].getSuccessors()[j]->getId();
		}

		// add adjacent left lanelet
		if (lanelets[i].getAdjacentLeft().size() != 0)
		{
			pugi::xml_node adjacentLeft = curLanelet.append_child("adjacentLeft");
			adjacentLeft.append_attribute("drivingDir") = lanelets[i].getAdjacentLeftDir().c_str();
			adjacentLeft.append_attribute("ref") = lanelets[i].getAdjacentLeft()[0];
		}

		// add adjacent right lanelet
		if (lanelets[i].getAdjacentRight().size() != 0)
		{
			pugi::xml_node adjacentRight = curLanelet.append_child("adjacentRight");
			adjacentRight.append_attribute("drivingDir") = lanelets[i].getAdjacentRightDir().c_str();
			adjacentRight.append_attribute("ref") = lanelets[i].getAdjacentRight()[0];
		}

		// add speed limit
		pugi::xml_node speedLimit = curLanelet.append_child("speedLimit");
		speedLimit.append_child(pugi::node_pcdata).set_value(ToString(lanelets[i].getSpeedLimit()).c_str());
	}

	// write obstacles (static vs dynamic)
	for (size_t i = 0; i < obstacles.size(); i++)
	{
		// add lanelet node with id
		pugi::xml_node curObstacle = commonRoad.append_child("obstacle");
		curObstacle.append_attribute("id") = obstacles[i]->getId();
		if (dynamic_cast<staticObstacle*>(obstacles[i]))
		{
			pugi::xml_node role = curObstacle.append_child("role");
			role.append_child(pugi::node_pcdata).set_value("static");
			pugi::xml_node type = curObstacle.append_child("type");
			type.append_child(pugi::node_pcdata).set_value("parkedVehicle");
			pugi::xml_node occupancySet = curObstacle.append_child("occupancySet");
			pugi::xml_node occupancy = occupancySet.append_child("occupancy");
			pugi::xml_node shapeOfObstacle = occupancy.append_child("shape");
			pugi::xml_node poly = shapeOfObstacle.append_child("polygon");
			std::vector <std::vector <occTypes> >* occupancyMatrix = obstacles[i]->getOccupancy()->getOccMatrix();
			// add occupancy object vertices
			for (size_t m = 0; m < (*occupancyMatrix)[0][0].vertices.size(); m++)
			{
				pugi::xml_node point = poly.append_child("point");
				vertice tempVertice = (*occupancyMatrix)[0][0].vertices[m];
				pugi::xml_node x = point.append_child("x");
				x.append_child(pugi::node_pcdata).set_value(ToString(tempVertice.x).c_str());
				pugi::xml_node y = point.append_child("y");
				y.append_child(pugi::node_pcdata).set_value(ToString(tempVertice.y).c_str());
			}
			pugi::xml_node time = occupancy.append_child("time");
			pugi::xml_node exact = time.append_child("exact");
			exact.append_child(pugi::node_pcdata).set_value(ToString(0).c_str());
		}
		else if (dynamic_cast<dynamicObstacle*>(obstacles[i]))
		{
			pugi::xml_node role = curObstacle.append_child("role");
			role.append_child(pugi::node_pcdata).set_value("dynamic");
			if (dynamic_cast<vehicle*>(obstacles[i]))
			{
				pugi::xml_node type = curObstacle.append_child("type");
				type.append_child(pugi::node_pcdata).set_value("car");
			}
			else if(dynamic_cast<pedestrian*>(obstacles[i]))
			{
				pugi::xml_node type = curObstacle.append_child("type");
				type.append_child(pugi::node_pcdata).set_value("car");
			}
			pugi::xml_node occupancySet = curObstacle.append_child("occupancySet");
			pugi::xml_node occupancy = occupancySet.append_child("occupancy");
			pugi::xml_node shapeOfObstacle = occupancy.append_child("shape");
			std::vector <std::vector <occTypes> >* occupancyMatrix = obstacles[i]->getOccupancy()->getOccMatrix();
			// add occupancy objetcs
			for (size_t k = 0; k < (*occupancyMatrix).size(); k++)
			{
				for (size_t l = 0; l < (*occupancyMatrix)[k].size(); l++)
				{
					pugi::xml_node poly = shapeOfObstacle.append_child("polygon");
					for (size_t m = 0; m < (*occupancyMatrix)[k][l].vertices.size(); m++)
					{
						pugi::xml_node point = poly.append_child("point");
						vertice tempVertice = (*occupancyMatrix)[k][l].vertices[m];
						pugi::xml_node x = point.append_child("x");
						x.append_child(pugi::node_pcdata).set_value(ToString(tempVertice.x).c_str());
						pugi::xml_node y = point.append_child("y");
						y.append_child(pugi::node_pcdata).set_value(ToString(tempVertice.y).c_str());
					}
				}
			}
			pugi::xml_node time = occupancy.append_child("time");
			pugi::xml_node exact = time.append_child("exact");
			exact.append_child(pugi::node_pcdata).set_value(ToString(0).c_str());
		}
	}

	std::string output_name = "Results/Output_";
	std::string temp = xmlFile;
	size_t found = temp.find_last_of("/");
	output_name += temp.substr(found+1);
	//std::cout << output_name << std::endl;
	doc.save_file(output_name.c_str());
}
