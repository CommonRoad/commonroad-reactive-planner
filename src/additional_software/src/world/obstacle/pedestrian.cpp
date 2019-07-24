/*
 * pedestrian.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: sebastian
 */

#include "pedestrian.h"
#include "../../geometry/circle.h"
#include <cmath>
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "../../geometry/geometricOperations.h"
#include "../lane/lane_operations.h"
#include <fstream>
#include <math.h>

/*
* getter functions
*/
size_t pedestrian::getNumVert() const
{
	return num_vert;
}

bool pedestrian::getComputeOccDynamicBased() const
{
	return computeOccDynamicBased;
}

bool pedestrian::getComputeOccRuleBased() const
{
	return computeOccRuleBased;
}

std::vector <double> pedestrian::getPedestrianOrientation() const
{
	return pedestrian_orientation;
}

std::vector <double> pedestrian::getPedestrianVelocity() const
{
	return pedestrian_velocity;
}

vertice pedestrian::getPRoadBoundary() const
{
	return pRoadBoundary;
}

bool pedestrian::getbStop() const
{
	return bstop;
}

bool pedestrian::getbCross() const
{
	return bcross;
}

double pedestrian::getdPerp() const
{
	return dperp;
}

int pedestrian::getNumLanes() const
{
	return numLanes;
}

double pedestrian::getLaneWidth() const
{
	return laneWidth;
}

double pedestrian::getAstop() const
{
	return a_stop;
}

void pedestrian::manageConstraints()
{
	dynamicObstacle::manageConstraints();
}

/*
 * setter functions
 */
void pedestrian::setNumVert(const size_t num)
{
	num_vert = num;
}

void pedestrian::setComputeOccDynamicBased(const bool val)
{
	computeOccDynamicBased = val;
}

void pedestrian::setComputeOccRuleBased(const bool val)
{
	computeOccRuleBased = val;
}

void pedestrian::setPedestrianOrientation(const std::vector <double> orientation_values)
{
	pedestrian_orientation = orientation_values;
}

void pedestrian::setPedestrianVelocity(const std::vector <double> velocity_values)
{
	pedestrian_velocity = velocity_values;
}

void pedestrian::setPRoadBoundary(const vertice vert)
{
	pRoadBoundary = vert;
}

void pedestrian::setbStop(const bool val)
{
	bstop = val;
}

void pedestrian::setbCross(const bool val)
{
	bcross = val;
}

void pedestrian::setdPerp(const double perp)
{
	dperp = perp;
}

void pedestrian::setNumLanes(const int num)
{
	numLanes = num;
}

void pedestrian::setLaneWidth(const double width)
{
	laneWidth = width;
}

void pedestrian::setAstop(const double val)
{
	a_stop = val;
}

// functions
std::vector <vertice> pedestrian::accOccPedestrian(double t, double dt)
{
	std::vector <vertice> M1_vertices;

	circle* circle_obj = dynamic_cast<circle*>(this->getShape());

	circle circle_t1, circle_t2;
	std::vector <vertice> vertices_t1, vertices_t2;

	// input solutions plus dimensions
	double radius_input = 0.5 * this->getAmax() * std::pow((t + dt), 2.0)
		+ circle_obj->getRadius();

	// homogeneous solution is annulus sector
	double uncertain_orientation = getMax(this->getPedestrianOrientation()) - getMean(this->getPedestrianOrientation());

	using coordinate_type = double;
	using point_type = boost::geometry::model::d2::point_xy<coordinate_type>;
	using polygon_type = boost::geometry::model::polygon<point_type>;
	using boost::geometry::get;

	if (uncertain_orientation > std::pow(10.0, -9.0))
	{
		circle_t1.setRadius(getMin(this->getPedestrianVelocity())*t);
		circle_t1.setCenter(0, 0);
		vertices_t1 = encloseSectionByPolygon(&circle_t1, this->getNumVert(), -uncertain_orientation, uncertain_orientation);
		circle_t2.setRadius(getMax(this->getPedestrianVelocity())*(t+dt));
		circle_t2.setCenter(0, 0);
		vertices_t2 = encloseSectionByPolygon(&circle_t2, this->getNumVert(), -uncertain_orientation, uncertain_orientation);
		/*
		for (size_t z = 0; z < vertices_t2.size(); z++)
		{
			std::cout << vertices_t2[z].y << std::endl;
		}
		*/

	   // Declare strategies
	    boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(radius_input);
	    boost::geometry::strategy::buffer::join_round join_strategy;
	    boost::geometry::strategy::buffer::end_round end_strategy;
	    boost::geometry::strategy::buffer::point_circle point_strategy;
	    boost::geometry::strategy::buffer::side_straight side_strategy;

		 // Construct polygon
		polygon_type occ_t1t2;
		int m;
		for (m = 0; m < (int)vertices_t1.size(); m++)
		{
			boost::geometry::append(occ_t1t2, point_type {vertices_t1[m].x, vertices_t1[m].y});
		}
		for (m = vertices_t2.size()-1; m >= 0; m--)
		{
			boost::geometry::append(occ_t1t2, point_type {vertices_t2[m].x, vertices_t2[m].y});
		}
		boost::geometry::append(occ_t1t2, point_type {vertices_t1[0].x, vertices_t1[0].y});
		polygon_type hull;
		boost::geometry::convex_hull(occ_t1t2, hull);

	    // Declare output
	    std::deque<polygon_type> result;

		boost::geometry::buffer(hull.outer(), result, distance_strategy, side_strategy, join_strategy, end_strategy, point_strategy);
		vertice temp;
		polygon_type polygon_result;
		polygon_result = result[0];
		std::vector<point_type> const& points = polygon_result.outer();
		for (std::vector<point_type>::size_type i = 0; i < points.size(); ++i)
		{
			temp.x = double(get<0>(points[i]));
			temp.y = double(get<1>(points[i]));
			//std::cout << "x: " << temp.x << " y: " << temp.y << std::endl;
			M1_vertices.push_back(temp);
		}
		/*
		// plot result
		std::ofstream svg("output.svg");
		boost::geometry::svg_mapper<point_type> mapper(svg, 400, 400);
		mapper.add(hull.outer());
		mapper.add(polygon_result);
		mapper.map(hull.outer(), "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2");
		mapper.map(polygon_result, "fill-opacity:0.5;fill:rgb(204,153,0);stroke:rgb(202,153,0);stroke-width:2");
		*/

	}
	else // homogeneous solution is only a line (thus, we directly add radius_input)
	{

		circle_t1.setRadius(radius_input);
		circle_t1.setCenter(getMin(this->getPedestrianVelocity()) * t, 0);
		vertices_t1 = encloseByPolygon(&circle_t1, this->getNumVert());
		circle_t2.setRadius(radius_input);
		circle_t2.setCenter(getMax(this->getPedestrianVelocity()) * (t + dt), 0);
		vertices_t2 = encloseByPolygon(&circle_t2, this->getNumVert());

		// Construct polygon
		polygon_type occ;
		int m;
		for (m = 0; m < (int)vertices_t1.size(); m++)
		{
			boost::geometry::append(occ, point_type {vertices_t1[m].x, vertices_t1[m].y});
		}
		for (m = vertices_t2.size()-1; m >= 0; m--)
		{
			boost::geometry::append(occ, point_type {vertices_t2[m].x, vertices_t2[m].y});
		}
		boost::geometry::append(occ, point_type {vertices_t1[0].x, vertices_t1[0].y});
		polygon_type hull;
		boost::geometry::convex_hull(occ, hull);
		std::vector<point_type> const& points = hull.outer();
		for (std::vector<point_type>::size_type i = 0; i < points.size(); ++i)
		{
			vertice temp;
			temp.x = double(get<0>(points[i]));
			temp.y = double(get<1>(points[i]));
			M1_vertices.push_back(temp);
		}
	}

	return M1_vertices;
}

void pedestrian::computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval)
{
	// get time interval properties
	float ts = timeInterval.startingTime;
	float dt = timeInterval.timeStep;
	float tf = timeInterval.ending;
	std::vector <vertice> accOcc_tvmax;
	std::vector <std::vector <vertice> > accOcc, velOcc, dynOcc;
    size_t numTimeIntervals = round((tf-ts)/dt);


	using coordinate_type = double;
	using point_type = boost::geometry::model::d2::point_xy<coordinate_type>;
	using polygon_type = boost::geometry::model::polygon<point_type>;
	using boost::geometry::get;

	std::vector <vertice> ruleBased_Occ;



	if(this->getComputeOccDynamicBased())
	{
		// --- dynamic-based occupancy prediction for consecutive time intervals ---
	    double t_vmax = std::max(0.0, ((this->getVmax() - getMax(this->getPedestrianVelocity())) / this->getAmax()));
	    accOcc_tvmax = this->accOccPedestrian(t_vmax, 0);

	    // Declare strategies
	    boost::geometry::strategy::buffer::join_round join_strategy;
	    boost::geometry::strategy::buffer::end_round end_strategy;
	    boost::geometry::strategy::buffer::point_circle point_strategy;
	    boost::geometry::strategy::buffer::side_straight side_strategy;


	    // Declare output
	    std::deque<polygon_type> result;

	    for (double t = ts; t < tf; t += dt)
	    {

	    	accOcc.push_back(this->accOccPedestrian(t, dt));
	    	if ( t <= t_vmax)
	    	{
	    		velOcc.push_back(std::vector <vertice>());
	    	}
	    	else
	    	{
	    		boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(this->getVmax() * (t + dt - t_vmax));
				int numVertices = accOcc.back().size();
				polygon_type polygon;
				for (int m = 0; m < numVertices; m++)
				{
					boost::geometry::append(polygon, point_type {accOcc_tvmax[m].x, accOcc_tvmax[m].y});
				}
				// close polygon
				boost::geometry::append(polygon, point_type{accOcc_tvmax[0].x, accOcc_tvmax[0].y});
	    		 // Create the buffer of a multi polygon
	    		boost::geometry::buffer(polygon, result, distance_strategy, side_strategy, join_strategy, end_strategy, point_strategy);
	    		std::vector <vertice> buffered_polygon;
	    		vertice temp;
				polygon_type polygon_result;
				polygon_result = result[0];
				std::vector<point_type> const& points = polygon_result.outer();
				for (std::vector<point_type>::size_type i = 0; i < points.size(); ++i)
				{
					temp.x = double(get<0>(points[i]));
					temp.y = double(get<1>(points[i]));
					buffered_polygon.push_back(temp);
				}
				velOcc.push_back(buffered_polygon);
	    	}
	    }

	    // intersect occupancy of the two models for each time interval
	    std::vector <vertice> dynOcc_i;
	    for (size_t i = 0; i < numTimeIntervals; i++)
	    {

	    	 if ((ts + i*dt <= t_vmax) && accOcc[i].size() != 0)
	    	 {
	    		 dynOcc_i = accOcc[i];
	    	 }
	    	 else if (accOcc[i].size() != 0 && velOcc[i].size() != 0)
	    	 {
					polygon_type polygon1, polygon2;
					int m;
					for (m = 0; m < (int)accOcc[i].size(); m++)
					{
						boost::geometry::append(polygon1, point_type {accOcc[i][m].x, accOcc[i][m].y});
					}
					// close polygon
					boost::geometry::append(polygon1, point_type{accOcc[i][0].x, accOcc[i][0].y});
					for (m = 0; m < (int)velOcc[i].size(); m++)
					{
						boost::geometry::append(polygon2, point_type {velOcc[i][m].x, velOcc[i][m].y});
					}
					// close polygon
					boost::geometry::append(polygon2, point_type{velOcc[i][0].x, velOcc[i][0].y});


					std::deque<polygon_type> output;
					boost::geometry::intersection(polygon1, polygon2, output);
					polygon_type polygon_result;


					if (output.size() != 0)
					{
						polygon_result = output[0];
						std::vector<point_type> const& points = polygon_result.outer();
						for (std::vector<point_type>::size_type it = 0; it < points.size(); ++it)
						{
							vertice temp;
							temp.x = double(get<0>(points[it]));
							temp.y = double(get<1>(points[it]));
							//std::cout << "temp: " << temp.x << " " <<  temp.y << std::endl;
							dynOcc_i.push_back(temp);
						}
					}
					else
					{
						std::cout << "something went wrong" << std::endl;
					}

	    	 }
	    	 else
	    	 {
	    		 std::runtime_error("Error");
	    	 }
	    	 // translate and rotate vertices according to the pedestrian's position and orientation
	    	 dynOcc.push_back(rotateAndTranslateVertices(dynOcc_i, vertice{this->getXpos(), this->getYpos()}, getMean(this->getPedestrianOrientation())));
	    }
	}

	// Compute rule-based occupancy for consecutive time intervals
	if(this->getComputeOccRuleBased())
	{

		vertice closest_vert;
		vertice pLambda;
		double shortest_dist = 99999;
		double cur_dist;
		// compute closest point on lanelet boundary
		for(size_t j = 0; j < lanelets->size(); j++)
		{
			for(size_t idx = 0; idx < (*lanelets)[j].getLeftBorder().size(); idx++)
			{
				cur_dist = calcPseudoDistance(idx, (*lanelets)[j].getLeftBorder(), vertice{this->getXpos(), this->getYpos()}, &pLambda);
				if (cur_dist < shortest_dist)
				{
					shortest_dist = cur_dist;
					closest_vert = pLambda;
				}
				cur_dist = calcPseudoDistance(idx, (*lanelets)[j].getRightBorder(), vertice{this->getXpos(), this->getYpos()}, &pLambda);
				if (cur_dist < shortest_dist)
				{
					shortest_dist = cur_dist;
					closest_vert = pLambda;
				}
			}
		}
		this->setPRoadBoundary(closest_vert);

        // Ostop
		// if onRoad, use closest point on lanelet boundary; else, use position
		std::vector <vertice> vertices_stop;
		if(!this->getbStop())
		{
			vertice center_stop;
			if (std::sqrt(std::pow(this->getXpos() - this->getPRoadBoundary().x, 2.0) + std::pow(this->getYpos() - this->getPRoadBoundary().y, 2.0)) - dynamic_cast<circle*>(this->getShape())->getRadius() < 0)
			{
				center_stop = this->getPRoadBoundary();
			}
			else
			{
				center_stop = vertice{this->getXpos(), this->getYpos()};
			}
			double radius_stop = ((std::pow(getMax(this->getPedestrianVelocity()), 2)) / (this->getAstop() * 2))
					+ dynamic_cast<circle*>(this->getShape())->getRadius();
			circle circle_stop(radius_stop, center_stop);
			vertices_stop = encloseByPolygon(&circle_stop, this->getNumVert(), 0);
		}

		// Operp
		std::vector <vertice> vertices_perp;
		if(!this->getbCross())
		{
			double laneWidth = this->getLaneWidth();
			double crossedLanelets = this->getNumLanes();
			double lengthCrossing = laneWidth * crossedLanelets;

			vertice lengthVector{this->getPRoadBoundary().x- this->getXpos(), this->getPRoadBoundary().y - this->getYpos()};
			double norm = std::sqrt(lengthVector.x*lengthVector.x + lengthVector.y * lengthVector.y);
			lengthVector.x *= 1.0/norm;
			lengthVector.y *= 1.0/norm;
			vertice widthVector{-lengthVector.y, lengthVector.x};
			vertice temp;
			temp.x = widthVector.x * this->getdPerp()/2.0 + this->getPRoadBoundary().x;
			temp.y = widthVector.y * this->getdPerp()/2.0 + this->getPRoadBoundary().y;
			vertices_perp.push_back(temp);
			temp.x += lengthVector.x * lengthCrossing;
			temp.y += lengthVector.y * lengthCrossing;
			vertices_perp.push_back(temp);
			temp.x = - widthVector.x * this->getdPerp()/2.0 + lengthVector.x * lengthCrossing + this->getPRoadBoundary().x;
			temp.y = - widthVector.y * this->getdPerp()/2.0 + lengthVector.y * lengthCrossing + this->getPRoadBoundary().y;
			vertices_perp.push_back(temp);
			temp.x = - widthVector.x * this->getdPerp()/2.0 + this->getPRoadBoundary().x;
			temp.y = - widthVector.y * this->getdPerp()/2.0 + this->getPRoadBoundary().y;
			vertices_perp.push_back(temp);
		}



		// final Orule

		 // Construct
		polygon_type polygon1;
		polygon_type polygon2;
		for (size_t m = 0; m < vertices_stop.size(); m++)
		{
			boost::geometry::append(polygon1, point_type {vertices_stop[m].x, vertices_stop[m].y});
		}
		// close polygon
		if (vertices_stop.size() > 0)
		{
			boost::geometry::append(polygon1, point_type {vertices_stop[0].x, vertices_stop[0].y});
		}
		for (size_t m = 0; m < vertices_perp.size(); m++)
		{
			boost::geometry::append(polygon2, point_type {vertices_perp[m].x, vertices_perp[m].y});
		}
		// close polygon
		if(vertices_perp.size() > 0)
		{
			boost::geometry::append(polygon2, point_type {vertices_perp[0].x, vertices_perp[0].y});
		}

		std::deque<polygon_type> output;

		if(this->getbCross() == false && this->getbStop() == false)
		{
			boost::geometry::union_(polygon1, polygon2, output);

			std::vector<point_type> const& points = output[0].outer();
			for (std::vector<point_type>::size_type it = 0; it < points.size(); ++it)
			{
				vertice temp;
				temp.x = double(get<0>(points[it]));
				temp.y = double(get<1>(points[it]));
				ruleBased_Occ.push_back(temp);
			}
		}
		else if (this->getbCross() == false)
		{
			ruleBased_Occ = vertices_perp;
		}
		else
		{
			ruleBased_Occ = vertices_stop;
		}
	}

	std::vector<std::vector<occTypes> > occMatrix(1);
	occMatrix[0].resize(numTimeIntervals);
	for(size_t i = 0; i < numTimeIntervals; i++)
	{
		if (ruleBased_Occ.size() == 0)
		{
			/*
			polygon_type dynBased_Occ, rule_Occ;
			size_t m;
			for (m = 0; m < dynOcc[i].size(); m++)
			{
				boost::geometry::append(dynBased_Occ, point_type {dynOcc[i][m].x, dynOcc[i][m].y});
			}
			boost::geometry::append(dynBased_Occ, point_type {dynOcc[i][0].x, dynOcc[i][0].y});
			polygon_type hull;
			boost::geometry::convex_hull(dynBased_Occ, hull);

			std::vector<point_type> const& points = hull.outer();
			//std::cout << std::endl;
			for (std::vector<point_type>::size_type it = 0; it < points.size(); ++it)
			{
				vertice temp;
				temp.x = double(get<0>(points[it]));
				temp.y = double(get<1>(points[it]));
				//std::cout << "temp: "<< i << "   " << temp.x << " " << temp.y << std::endl;
				occMatrix[0][i].vertices.push_back(temp);
			}
			*/

			occMatrix[0][i].timeInterval = {ts + dt*i, dt, ts + dt*(i+1)};
			continue;
		}
		else if (this->getComputeOccDynamicBased() == false)
		{
			for (size_t it = 0; it < ruleBased_Occ.size(); ++it)
			{
				occMatrix[0][i].vertices.push_back(ruleBased_Occ[i]);
			}
			continue;
		}
		polygon_type dynBased_Occ, rule_Occ;
		size_t m;
		for (m = 0; m < dynOcc[i].size(); m++)
		{
			boost::geometry::append(dynBased_Occ, point_type {dynOcc[i][m].x, dynOcc[i][m].y});
		}
		boost::geometry::append(dynBased_Occ, point_type {dynOcc[i][0].x, dynOcc[i][0].y});
		polygon_type hull;
		boost::geometry::convex_hull(dynBased_Occ, hull);
		//std::cout << std::endl;
		for (m = 0; m < ruleBased_Occ.size(); m++)
		{
			boost::geometry::append(rule_Occ, point_type {ruleBased_Occ[m].x, ruleBased_Occ[m].y});
		}
		boost::geometry::append(rule_Occ, point_type {ruleBased_Occ[0].x, ruleBased_Occ[0].y});


		std::deque<polygon_type> output;
		boost::geometry::intersection(hull, rule_Occ, output);
		polygon_type polygon_result;
		if (output.size() != 0)
		{
			polygon_result = output[0];
			/*
			{
				// plot result
				std::ofstream svg("output" + std::to_string(i) +  ".svg");
				boost::geometry::svg_mapper<point_type> mapper(svg, 800, 800);
				//mapper.add(polygon1);
				mapper.add(polygon_result);
				//mapper.map(polygon1, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2");
				mapper.map(polygon_result, "fill-opacity:0.5;fill:rgb(204,153,0);stroke:rgb(202,153,0);stroke-width:2");
			}
			*/
			std::vector<point_type> const& points = polygon_result.outer();
			for (std::vector<point_type>::size_type it = 0; it < points.size(); ++it)
			{
				vertice temp;
				temp.x = double(get<0>(points[it]));
				temp.y = double(get<1>(points[it]));
				//std::cout << "temp: "<< i << "   " << temp.x << " " << temp.y << std::endl;
				occMatrix[0][i].vertices.push_back(temp);
			}
		}
		else
		{
			std::cout << "something went wrong" << std::endl;
		}
		occMatrix[0][i].timeInterval = {ts + dt*i, dt, ts + dt*(i+1)};
	}
	std::vector <std::vector <occTypes> >* occMatrixPointer = this->getOccupancy()->getOccMatrix();
	(*occMatrixPointer) = occMatrix;
}
