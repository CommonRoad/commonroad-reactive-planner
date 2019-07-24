/*
 * functions on obstacles
 */

#ifndef HEADER_OBSTACLE_OPERATIONS
#define HEADER_OBSTACLE_OPERATIONS

#include "vehicle.h"
#include "staticObstacle.h"
#include <iostream>

//creates all obstacle objects from the XML input
std::vector <obstacle*> createObstacleFromXML(std::string xmlFile, std::vector <obstacle*>* previousObstacles, double timeStamp, bool noInit, std::vector <lane>* lanes);

// find obstacle by id
obstacle* getObstacleById(std::vector <obstacle*>*  obstacleList, size_t id);

#endif
