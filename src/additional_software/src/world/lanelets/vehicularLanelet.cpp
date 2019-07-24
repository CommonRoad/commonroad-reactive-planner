/*
 * vehicularLanelet.cpp
 *
 *  Created on: Jun 25, 2018
 *      Author: sebastian
 */

#include "vehicularLanelet.h"

struct adjacent {
		lanelet* adj;
		std::string dir;
	};

void vehicularLanelet::setSpeedLimit(const double lim)
{
	speedLimit = lim;
}

void vehicularLanelet::addPredecessor(vehicularLanelet* pre)
{
	predecessorLanelets.push_back(pre);
}

void vehicularLanelet::addSuccessor(vehicularLanelet* suc)
{
	successorLanelets.push_back(suc);
}

void vehicularLanelet::setLeftAdjacent(size_t left, std::string dir)
{
	adjacentLeft.adj.push_back(left);
	adjacentLeft.dir = dir;
}

void vehicularLanelet::setRightAdjacent(size_t right, std::string dir)
{
	adjacentRight.adj.push_back(right);
	adjacentRight.dir = dir;
}

double vehicularLanelet::getSpeedLimit() const
{
	return speedLimit;
}

std::vector <vehicularLanelet*> vehicularLanelet::getPredecessors() const
{
	return predecessorLanelets;
}

std::vector <vehicularLanelet*> vehicularLanelet::getSuccessors() const
{
	return successorLanelets;
}

std::vector<size_t> vehicularLanelet::getAdjacentLeft() const
{
	return adjacentLeft.adj;
}

std::vector<size_t> vehicularLanelet::getAdjacentRight() const
{
	return adjacentRight.adj;
}

std::string vehicularLanelet::getAdjacentLeftDir() const
{
	return adjacentLeft.dir;
}

std::string vehicularLanelet::getAdjacentRightDir() const
{
	return adjacentRight.dir;
}


