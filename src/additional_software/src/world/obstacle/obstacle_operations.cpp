#include "obstacle_operations.h"

#include "../../inputs/pugi_xml/pugixml.hpp"
#include <string.h>
#include <stdexcept>
#include "../../geometry/rectangle.h"
#include "../../geometry/geometricOperations.h"
#include <cmath>

std::vector <obstacle*> createObstacleFromXML(std::string xmlFile, std::vector <obstacle*>* previousObstacles, double timeStamp, bool noInit, std::vector <lane>* lanes)
{
	//std::cout << "TimeStamp: " << timeStamp << std::endl;

	std::vector <obstacle*> obstacleList;
	pugi::xml_document doc;
	if (!doc.load_file(xmlFile.c_str())) throw std::runtime_error("Couldn't load XML-File");;

	pugi::xml_node commonRoad = doc.child("commonRoad");
	if ((strcmp(commonRoad.attribute("commonRoad").value(), "2017a")))
	{

	//size_t numObstacles = std::distance(commonRoad.children("obstacle").begin(), commonRoad.children("obstacle").end());
	size_t n = 0;
	for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements; roadElements = roadElements.next_sibling())
	{
	    if(!(strcmp(roadElements.name(), "obstacle")))
	    {
	    	if(!(strcmp(roadElements.first_child().text().as_string(), "dynamic")) &&
	    		!(strcmp(roadElements.first_child().next_sibling().text().as_string(), "car")))
	    	{
	    		vehicle* obst = new vehicle;
	    		bool timeStampAvailable = false;
	    		obst->setId(roadElements.first_attribute().as_int());
		    	for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling())
		    		{
		    			if(!(strcmp(child.name(), "shape")))
		    			{
		    				if (!(strcmp(child.first_child().name(), "rectangle")))
							{
		    					rectangle* rect = new rectangle(child.first_child().child("length").text().as_double(), child.first_child().child("width").text().as_double());
		    					obst->setShape(rect);
							}
		    				continue;
		    			}
		    			if(!(strcmp(child.name(), "trajectory")))
		    			{
		    			//std::cout << child.first_child().child("time").child("exact").text().as_double() << std::endl;
		    			for (pugi::xml_node states = child.first_child(); states; states = states.next_sibling())
		    			{
		    				//std::cout << states.child("time").child("exact").text().as_double() << std::endl;
		    				if (states.child("time").child("exact").text().as_double() == timeStamp)
		    				{
		    					obst->setPosition(states.child("position").child("point").child("x").text().as_double(),
		    									states.child("position").child("point").child("y").text().as_double());
		    					obst->setOrientation(states.child("orientation").child("exact").text().as_double());
		    					obst->setVelocity(states.child("velocity").child("exact").text().as_double());
		    					obst->setAcceleration(states.child("acceleration").child("exact").text().as_double());
		    					timeStampAvailable = true;
		    				}
		    			}

		    			continue;
		    		}
		    		}
		    	if (timeStampAvailable)
		    	{
		    		//obst->updateInLane(lanes); // set the lanes in which the obstacle is situated
		    	}
		    	if (noInit && timeStampAvailable)
		    	{
		    		obstacle* previous_obst = getObstacleById(previousObstacles, obst->getId());
		    		//manage constraint 5 (lane crossing)
		    		if (!(obst->getInLane().empty()) &&
		    				!(previous_obst->getInLane().empty()))
		    		{
		    			vertice previous_pos;
		    			previous_pos.x = previous_obst->getXpos();
		    			previous_pos.y = previous_obst->getYpos();
		    			double laneOrientation_previous = calcAngleOfVerticesAtPosition(previous_obst->getInLane()[0]->getCenterVertices(), previous_pos);
		    			vertice new_pos;
		    			new_pos.x = obst->getXpos();
		    			new_pos.y = obst->getYpos();
		    			double laneOrientation_new = calcAngleOfVerticesAtPosition(obst->getInLane()[0]->getCenterVertices(), new_pos);
		    			if (std::abs(laneOrientation_previous - laneOrientation_new) > M_PI/2)
		    			{
		    				// warning
		    				obst->setConstraint5(false);
		    			}
		    		}
		    	}
		    	if (timeStampAvailable)
		    	{
		    		obst->updateInLane(lanes);
		    		obstacleList.push_back(obst);
		    	}
		    	else
		    	{
		    		delete obst;
		    	}
	    	}
	    	else if(!(strcmp(roadElements.first_child().text().as_string(), "static")))
			{
	    		staticObstacle* obst = new staticObstacle;
	    		obst->setId(roadElements.first_attribute().as_int());
		    	for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling())
		    		{
		    		if(!(strcmp(child.name(), "shape")))
		    		{
		    			pugi::xml_node var = child.first_child();
		    		    obst->setOrientation(var.child("orientation").text().as_double());
		    		    obst->setPosition(var.child("center").child("x").text().as_double(),
		    		    				var.child("center").child("y").text().as_double());
	    				rectangle* rect = new rectangle(var.child("length").text().as_double(), var.child("width").text().as_double());
	    				obst->setShape(rect);
		    		}
		    		    continue;
		    		}
		    	obst->updateInLane(lanes);
		    	obstacleList.push_back(obst);
			}
	    	n++; // counter for previous obstacles
	    }
	}

	if (noInit)
	{
		while(!(previousObstacles->empty()))
			{
				obstacle* temp = previousObstacles->back();
				previousObstacles->pop_back();
				delete temp;
				temp = 0;
			}
	}
	}
	else
	{
		throw std::runtime_error("wrong CommonRoad version!");
	}
	return obstacleList;
}

obstacle* getObstacleById(std::vector <obstacle*>* obstacleList, size_t id)
{
	obstacle* temp = 0;
	for(size_t i = 0; i < (*obstacleList).size(); i++)
	{
		if ((*obstacleList)[i]->getId() == id)
		{
			temp = (*obstacleList)[i];
			break;
		}
	}
	return temp;
}
