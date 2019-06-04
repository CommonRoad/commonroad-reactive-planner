/*
 * minkowski.h
 *
 *  Created on: 19.01.2018
 *      Author: Sebastian
 */

#ifndef GEOMETRY_MINKOWSKI_H_
#define GEOMETRY_MINKOWSKI_H_

#include <iostream>
#include <boost/polygon/polygon.hpp>
typedef boost::polygon::point_data<int> point;
typedef boost::polygon::polygon_set_data<int> polygon_set;
typedef boost::polygon::polygon_with_holes_data<int> polygon;
typedef std::pair<point, point> edge;

void convolve_two_segments(std::vector<point>& figure, const edge& a, const edge& b);
bool test_minkowskiSum();
void convolve_two_polygon_sets(polygon_set& result, const polygon_set& a, const polygon_set& b);

#endif /* GEOMETRY_MINKOWSKI_H_ */
