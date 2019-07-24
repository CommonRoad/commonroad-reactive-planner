#include "obstacle.h"
#include "../lane/lane_operations.h"
#include "../../geometry/geometricOperations.h"
#include <cmath>

#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif

obstacle::obstacle()
{

}

obstacle::~obstacle()
{
	delete geoShape;
	geoShape = 0;
}

void obstacle::setId(const size_t num)
{
	id = num;
}

void obstacle::setPosition(const double x, const double y)
{
	xPosition = x;
	yPosition = y;
}

void obstacle::setOrientation(const double value)
{
	orientation = wrapToPi(value);
}

void obstacle::addInLane(lane* l)
{
	inLane.push_back(l);
}



void obstacle::updateInLane(std::vector <lane>* lanes)
{
	std::vector <lane*> lanesAtObjPosition;
	lanesAtObjPosition = findLaneByPosition(lanes, this->getXpos(), this->getYpos());
	double laneOrientation;
	vertice temp;
	temp.x = this->getXpos();
	temp.y = this->getYpos();
	//std::cout << lanesAtObjPosition.front()->getId() << std::endl;
	for (size_t i = 0; i < lanesAtObjPosition.size(); i++)
	{
		laneOrientation = calcAngleOfVerticesAtPosition(lanesAtObjPosition[i]->getCenterVertices(), temp);
		//std::cout << laneOrientation << std::endl;
		/*
		* assume that a traffic participant is only in a lane, if its
		* orientation differs less than +- pi/5 from the lane orientation
		*/
		if(std::abs(this->getOrientation() - laneOrientation) < M_PI/5 || std::abs(this->getOrientation() - laneOrientation) > 4*M_PI/5)
		{
			//std::cout << this->getId() << std::endl;
			this->addInLane(lanesAtObjPosition[i]);
		}
	}
}

void obstacle::setShape(shape* gShape)
{
	geoShape = gShape;
}

size_t obstacle::getId() const
{
	return id;
}

double obstacle::getXpos() const
{
	return xPosition;
}

double obstacle::getYpos() const
{
	return yPosition;
}

double obstacle::getOrientation() const
{
	return orientation;
}

std::vector <lane*> obstacle::getInLane() const
{
	return inLane;
}

shape* obstacle::getShape() const
{
	return geoShape;
}

occupancy* obstacle::getOccupancy() const
{
	return &occupy;
}

// virtual methods
void obstacle::computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval)
{

}

std::vector <std::vector <vertice> > obstacle::M1_accelerationBasedOccupancy(timeStruct timeInterval)
{
	std::vector <std::vector <vertice> > temp;
	return temp;
}
