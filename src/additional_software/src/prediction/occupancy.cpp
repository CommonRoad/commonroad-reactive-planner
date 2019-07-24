#include "occupancy.h"
/*
void occupancy::setForLane(std::vector <lane*> inLanes)
{
	forLane = inLanes;
}

void occupancy::setVertices(std::vector <vertice> verticeList)
{
	vertices = verticeList;
}
*/

std::vector <std::vector <occTypes> >* occupancy::getOccMatrix()
{
	return &occupancyMatrix;
}
