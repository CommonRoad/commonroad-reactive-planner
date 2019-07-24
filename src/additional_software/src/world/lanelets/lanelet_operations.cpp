#include "lanelet_operations.h"

#include "../../inputs/pugi_xml/pugixml.hpp"
#include <string.h>
#include <stdexcept>

std::vector <vehicularLanelet> createLaneletFromXML(std::string xmlFile)
{
	std::vector <vehicularLanelet> lanelets;

	pugi::xml_document doc;
	if (!doc.load_file(xmlFile.c_str())) throw std::runtime_error("Couldn't load XML-File");

	pugi::xml_node commonRoad = doc.child("commonRoad");
	if ((strcmp(commonRoad.attribute("commonRoad").value(), "2017a")))
	{

	// get the number of lanelets
	size_t n = std::distance(commonRoad.children("lanelet").begin(), commonRoad.children("lanelet").end());


	/*
	 * all lanelets must be initilized first because they are referencing
	 * each other
	 */

	for (size_t i = 0; i < n; i++)
	{
		vehicularLanelet newLanelet;
		lanelets.push_back(newLanelet);
	}
	size_t arrayIndex = 0;
	// set id of the lanelets
	for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements; roadElements = roadElements.next_sibling())
	{
	    if(!(strcmp(roadElements.name(), "lanelet")))
	    {
	    	lanelets[arrayIndex].setId(roadElements.first_attribute().as_int());
	    	arrayIndex++;
	    }
	}

	// get the other values of the lanelets
	arrayIndex = 0;
	for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements; roadElements = roadElements.next_sibling())
	{
	    if(!(strcmp(roadElements.name(), "lanelet")))
	    {
	    	for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling())
	    		{
	    		// set left bound
	    		if(!(strcmp(child.name(), "leftBound")))
	    		{
	    		    for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling())
	    		    {
	    		    	if(!(strcmp(points.name(), "point")))
	    		    	{
	    		    		vertice newVertice;
	    		    		newVertice.x = points.child("x").text().as_double();
	    		    		newVertice.y = points.child("y").text().as_double();
	    		    		lanelets[arrayIndex].addLeftVertice(newVertice);
	    		    	}
	    		    }
	    		    continue;
	    		}
	    		// set right bound
	    		if(!(strcmp(child.name(), "rightBound")))
	    		{
	    			for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling())
	    			{
	    		    	if(!(strcmp(points.name(), "point")))
	    		    	{
	    		    		vertice newVertice;
	    		    		newVertice.x = points.child("x").text().as_double();
	    		    		newVertice.y = points.child("y").text().as_double();
	    		    		lanelets[arrayIndex].addRightVertice(newVertice);
	    		    	}
	    		    }
	    			continue;
	    		}
	    		// set successor lanelets
	    		if(!(strcmp(child.name(), "successor")))
	    		{
	    			size_t successorId = child.first_attribute().as_int();
	    			for (size_t i = 0; i < n; i++)
	    			{
	    				if(lanelets[i].getId() == successorId)
	    				{
	    					lanelets[arrayIndex].addSuccessor(&lanelets[i]);
	    					break;
	    				}
	    			}
	    			continue;
	    		}
	    		// set predecessor lanelets
	    		if(!(strcmp(child.name(), "predecessor")))
	    		{
	    			size_t predecessorId = child.first_attribute().as_int();
	    			for (size_t i = 0; i < n; i++)
	    			{
	    				if(lanelets[i].getId() == predecessorId)
	    				{
	    					lanelets[arrayIndex].addPredecessor(&lanelets[i]);
	    					break;
	    				}
	    			}
	    			continue;
	    		}
	    		// set left adjacent lanelets
	    		if(!(strcmp(child.name(), "adjacentLeft")))
	    		{
	    			size_t adjacentId = child.attribute("ref").as_int();
	    			std::string dir = child.attribute("drivingDir").as_string();
	    			// std::cout << "string_dir: " << dir << std::endl;
	    			for (size_t i = 0; i < n; i++)
	    			{
	    			    if(lanelets[i].getId() == adjacentId)
	    			    {
	    			    	lanelets[arrayIndex].setLeftAdjacent(lanelets[i].getId(), dir);
	    			    	break;
	    			    }
	    			}
	    			continue;
	    		}
	    		// set right adjacent lanelets
	    		if(!(strcmp(child.name(), "adjacentRight")))
	    		{
	    			size_t adjacentId = child.attribute("ref").as_int();
	    			std::string dir = child.attribute("drivingDir").as_string();
	    			for (size_t i = 0; i < n; i++)
	    			{
	    			    if(lanelets[i].getId() == adjacentId)
	    			    {
	    			    	lanelets[arrayIndex].setRightAdjacent(lanelets[i].getId(), dir);
	    			    	break;
	    			    }
	    			}
	    			continue;
	    		}
	    		// set speed limit
	    		if(!(strcmp(child.name(), "speedLimit")))
	    		{
	    			for (size_t i = 0; i < n; i++)
	    			{
	    			    lanelets[arrayIndex].setSpeedLimit(child.text().as_double());
	    			}
	    			continue;
	    		}
	    	}
	    	lanelets[arrayIndex].createCenterVertices();
	    	arrayIndex++;
	    }
	}
	}
	else
	{
		throw std::runtime_error("Wrong CommonRoad Version!");
	}
	return lanelets;
}

std::vector <vehicularLanelet*> noPredecessorLanelets(std::vector <vehicularLanelet>* lanelets)
{
	std::vector <vehicularLanelet*> noPredLanelets;
	for (size_t i = 0; i < (*lanelets).size(); i++) {
		if((*lanelets)[i].getPredecessors().size() == 0) // lanelet has no predecessor
		{
			noPredLanelets.push_back(&(*lanelets)[i]);
		}
	}
	return noPredLanelets;
}

std::vector <vehicularLanelet*> findAllSuccessorLanelets(vehicularLanelet* lanelets)
{
	std::vector <vehicularLanelet*> allSuccessorLanelets;
	for (size_t i = 0; i < lanelets->getSuccessors().size(); i++)
	{
		allSuccessorLanelets.push_back(lanelets->getSuccessors()[i]);
	}
	return allSuccessorLanelets;
}

/*
lanelet* specificLanelet(std::list <lanelet*> lanelets, size_t id)
{
	size_t index = 1;
	for (std::list <lanelet*>::const_iterator it = lanelets.begin(); it != lanelets.end(); ++it) {
		if(index == id)
		{
			return *it;
		}
		index++;
	}
	throw std::runtime_error("No Element with such id!");
}
*/
