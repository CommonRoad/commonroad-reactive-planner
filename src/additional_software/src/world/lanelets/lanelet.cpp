#include "lanelet.h"
//#include <limits>
#include <boost/math/tools/precision.hpp>

struct adjacent {
		lanelet* adj;
		std::string dir;
	};

lanelet::lanelet()
{
	id = 0;
}

lanelet::~lanelet()
{
}

void lanelet::setId(const size_t num)
{
	id = num;
}

void lanelet::addLeftVertice(const vertice left)
{
	leftBorder.push_back(left);
}

void lanelet::addRightVertice(const vertice right)
{
	rightBorder.push_back(right);
}


void lanelet::addCenterVertice(const vertice center)
{
	centerVertices.push_back(center);
}

void lanelet::createCenterVertices()
{
	// initialise
	size_t numVertices = leftBorder.size();

	for (size_t i = 0; i < numVertices; i++)
	{
		/*
		 * calculate a center vertex as the arithmetic mean between the opposite
		 * vertex on the left and right border
		 * (calculate x and y values seperately in order to minimize error)
		 */
		vertice newVertice;
		newVertice.x = 0.5 * (leftBorder[i].x + rightBorder[i].x);
		newVertice.y = 0.5 * (leftBorder[i].y + rightBorder[i].y);
		addCenterVertice(newVertice);
	}
}

size_t lanelet::getId() const
{
	return id;
}

std::vector <vertice> lanelet::getLeftBorder() const
{
	return leftBorder;
}

std::vector <vertice> lanelet::getRightBorder() const
{
	return rightBorder;
}


std::vector <vertice> lanelet::getCenter() const
{
	return centerVertices;
}

