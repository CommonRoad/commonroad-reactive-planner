/*
 * pedestrianLanelet.cpp
 *
 *  Created on: Jun 25, 2018
 *      Author: sebastian
 */

#include "pedestrianLanelet.h"

/*
 * setter functions
 */
void pedestrianLanelet::setPriority(const size_t priority)
{
	prio = priority;
}

void pedestrianLanelet::setType(const std::string typeOfSideWalk)
{
	type = typeOfSideWalk;
}

void pedestrianLanelet::setSide(const std::string sideOfSideWalk)
{
	side = sideOfSideWalk;
}

void pedestrianLanelet::addConnectedLanelet(pedestrianLanelet* connectedLanelet)
{
	connectedLanelets.push_back(connectedLanelet);
}

/*
 * getter functions
 */
size_t pedestrianLanelet::getPriority() const
{
	return prio;
}

std::string pedestrianLanelet::getType() const
{
	return type;
}

std::string pedestrianLanelet::getSide() const
{
	return side;
}

std::vector<pedestrianLanelet*> pedestrianLanelet::getConnectedLanelet() const
{
	return connectedLanelets;
}
