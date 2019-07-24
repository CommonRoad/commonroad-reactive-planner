//============================================================================
// Name        : readXMLfile.cpp
// Author      : Sebastian
// Version     :
//============================================================================

#include "readXMLfile.h"

#include "pugi_xml/pugixml.hpp"
#include <string.h>



void readXMLfile(std::string xmlFile)
{
    pugi::xml_document doc;
    if (!doc.load_file(xmlFile.c_str())) return;


    pugi::xml_node commonRoad = doc.child("commonRoad");

    std::cout << commonRoad.name() << std::endl;

    if (!(strcmp(commonRoad.first_attribute().name(), "commonRoadVersion"))
    		&& !(strcmp(commonRoad.first_attribute().value(), "2017a")))
    		{
    	std::cout << "Right Version" << std::endl;
    		}
    else
    {
    	std::cout << "Wrong Version" << std::endl;
    }

    std::cout << commonRoad.attribute("benchmarkID").value() << std::endl;
    std::cout << commonRoad.attribute("date").value() << std::endl;
    std::cout << commonRoad.attribute("timeStepSize").value() << std::endl;

    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements; roadElements = roadElements.next_sibling())
    {
    	if(!(strcmp(roadElements.name(), "lanelet")))
    	{
    		std::cout << roadElements.name() << std::endl;
    		for (pugi::xml_node child = roadElements.first_child(); child; child = child.next_sibling())
    		{
    			if(!(strcmp(child.name(), "leftBound")))
    			{
    				for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling())
    				{
    					if(!(strcmp(points.name(), "point")))
    					{
    					    std::cout << "x:" << points.child("x").text().as_double() << std::endl;
    					    std::cout << "y:" << points.child("y").text().as_double() << std::endl;
    					}
    				}
    			}
    			if(!(strcmp(child.name(), "rightBound")))
    			{
    			 	for (pugi::xml_node points = child.first_child(); points; points = points.next_sibling())
    			    {
    			 		if(!(strcmp(points.name(), "point")))
    			 		{
    			 			std::cout << "x:" << points.child("x").text().as_double() << std::endl;
    			 			std::cout << "y:" << points.child("y").text().as_double() << std::endl;
    			 		}
    			 	}
    			}
    		}



    	}
    	if(!(strcmp(roadElements.name(), "obstacle")))
    	{
    		std::cout << roadElements.name() << std::endl;
    	}
    	if(!(strcmp(roadElements.name(), "planningProblem")))
    	{
    		std::cout << roadElements.name() << std::endl;
    	}
        }
}

float getTimeStep(std::string xmlFile)
{
	pugi::xml_document doc;
	if (!doc.load_file(xmlFile.c_str())) throw std::runtime_error("Couldn't load XML-File");;

	pugi::xml_node commonRoad = doc.child("commonRoad");
	float timeStep;
	if ((strcmp(commonRoad.attribute("commonRoad").value(), "2017a")))
	{
		timeStep = commonRoad.attribute("timeStepSize").as_float();
	}
	else
	{
		throw std::runtime_error("Wrong CommonRoad Version!");
	}
	return timeStep;
}

