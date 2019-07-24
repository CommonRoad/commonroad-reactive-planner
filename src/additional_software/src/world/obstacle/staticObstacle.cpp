#include "staticObstacle.h"
#include "../../prediction/occupancyCalculation.h"
#include "../../geometry/geometricOperations.h"
#include "../../geometry/rectangle.h"
#include <math.h>

void staticObstacle::computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval)
{
	size_t timeLength = round((timeInterval.ending-timeInterval.startingTime)/timeInterval.timeStep);
	std::vector <std::vector <vertice> > verticesM1;
	//size_t timeLength = (timeInterval.ending-timeInterval.startingTime)/timeInterval.timeStep;

	verticesM1 = this->M1_accelerationBasedOccupancy(timeInterval);
	// initialize occupancy matrix
	std::vector<std::vector<occTypes> > occMatrix(1);
	occMatrix[0].resize(1);
	for (size_t k = 0; k < timeLength; k++)
	{
		for (size_t j = 0; j < this->getInLane().size(); j++)
		{
			occMatrix[0][k].forLane.push_back(this->getInLane()[j]);
			occMatrix[0][k].timeInterval = timeInterval;
			occMatrix[0][k].vertices = verticesM1.front();
		}
	}


	/*
	std::cout << "Print M1 vertices" << std::endl;
	for (size_t i = 0; i < verticesM1.front().size(); i++)
	{
		std::cout << verticesM1.front()[i].y << std::endl;
	}
	*/
	(*this->getOccupancy()->getOccMatrix()) = occMatrix;
}

std::vector <std::vector <vertice> > staticObstacle::M1_accelerationBasedOccupancy(timeStruct timeInterval)
{
	std::vector <vertice> p;
	std::vector <vertice> q;
	std::vector <std::vector <vertice> > occM1;
	//compute the occupancy for static and dynamic objects
	//std::cout << this->getShape() << std::endl;
	if (dynamic_cast<rectangle*>(this->getShape()))
	{
		rectangle* rect = dynamic_cast<rectangle*>(this->getShape());
		vertice q1;

		q1.x = 0;
		q1.y = 0;
		q.push_back(q1);
		// vertices p represent the occupancy with vehicle dimensions (Theorem 1)
		p = addObjectDimensions(q, rect->getLength(), rect->getWidth());
		/*
		 * rotate and translate the vertices of the occupancy set in local
		 * coordinates to the object's reference position and rotation
		 */
		vertice position;
		position.x = this->getXpos();
		position.y = this->getYpos();
		occM1.push_back(rotateAndTranslateVertices(p, position, this->getOrientation()));
	}
	return occM1;
}
