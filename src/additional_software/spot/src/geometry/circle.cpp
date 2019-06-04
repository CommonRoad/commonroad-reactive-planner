/*
 * circle.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: sebastian
 */

#include "circle.h"

circle::circle() : shape()
{
	radius = 0;
	center = vertice {.x = 0, .y = 0};
}

/* setter functions */

void circle::setRadius(const double rad)
{
	radius = rad;
}

void circle::setCenter(const double x, const double y)
{
	center = vertice {.x = x, .y = y};
}

/* getter functions */

double circle::getRadius() const
{
	return radius;
}

vertice circle::getCenter() const
{
	return center;
}
