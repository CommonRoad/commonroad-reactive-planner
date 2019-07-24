#include "vehicle.h"
#include "../../prediction/occupancyCalculation.h"
#include <boost/math/tools/precision.hpp>
#include "../../geometry/geometricOperations.h"
#include "../lane/lane_operations.h"
#include "../../geometry/rectangle.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif

 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

double vehicle::getVs() const
{
	return v_s;
}

double vehicle::getPowerMax() const
{
	return power_max;
}

bool vehicle::getConstraint3() const
{
	return constraint3;
}

bool vehicle::getConstraint5() const
{
	return constraint5;
}

void vehicle::setPowerMax(const double power)
{
	power_max = power;
}

void vehicle::setVs(const double velo)
{
	v_s = velo;
}

void vehicle::setConstraint3(const bool c3)
{
	constraint3 = c3;
}

void vehicle::setConstraint5(const bool c5)
{
	constraint5 = c5;
}

bool vehicle::getCompute_Occ_M1() const
{
	return compute_occ_m1;
}
bool vehicle::getCompute_Occ_M2() const
{
	return compute_occ_m2;
}

void vehicle::setSpeedingFactor(const double factor)
{
	speedingFactor = factor;
}

double vehicle::getSpeedingFactor() const
{
	return speedingFactor;
}

void vehicle::computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval)
{
	this->manageConstraints();
	std::vector <std::vector <vertice> > verticesM1;
	std::vector <std::vector <vertice> > verticesM2;
	size_t timeLength = round((timeInterval.ending-timeInterval.startingTime)/timeInterval.timeStep);

	if (this->getCompute_Occ_M1())
	{
		verticesM1 = this->M1_accelerationBasedOccupancy(timeInterval);
	}
	std::vector <lane*> reachableLanes;
	/*
	 * find all Lane objects of the map which can be reached by the
	 * obstacle according to the adjacency graph and contraint5
	 * (only if ~constraint 5, the occupancy will be computed for all lanes)
	 */
	if (!(this->getInLane().empty()))
	{
		reachableLanes = findAllReachableLanes(lanes, this->getConstraint5(), this->getInLane());
	}
	else
	{
		// obstacle is not in a lane. thus, we can only consider occM1
		std::vector <std::vector <occTypes> >* occMatrixPointer = this->getOccupancy()->getOccMatrix();
		std::vector<std::vector<occTypes> > occMatrix(1);
		occMatrix[0].resize(timeLength);
		timeStruct tInterval;
		for (size_t k = 0; k < timeLength; k++)
		{
			tInterval.startingTime = timeInterval.startingTime + timeInterval.timeStep*k;
			tInterval.timeStep = timeInterval.timeStep;
			tInterval.ending = timeInterval.startingTime + (k+1)*timeInterval.timeStep;
			occMatrix[0][k].timeInterval = tInterval;
			occMatrix[0][k].vertices = verticesM1[k];
			(*occMatrixPointer) = occMatrix;
			return;
		}
	}

	// initialise the new occupancy matrix
	std::vector<std::vector<occTypes> > occMatrix(reachableLanes.size());
	for (size_t i = 0 ; i < reachableLanes.size() ; i++ )
	{
		occMatrix[i].resize(timeLength);
	}
	// compute occupancy for all reachable lanes
	for (size_t i = 0; i < reachableLanes.size(); i++)
	{
		lane reachableLanes_i = (*reachableLanes[i]);
		/*
		 * flip the current lane if driving direction is opposite
		 * (compare vehicle oriantation to lane orientation):
		 * calculate horizontal angle of the closest center vertices
		 */
/*
		vertice position;
		position.x = this->getXpos();
		position.y = this->getYpos();

		double laneOrientation = calcAngleOfVerticesAtPosition(reachableLanes_i.getCenterVertices(), position);
		std::cout << laneOrientation << " " << this->getOrientation() << std::endl;
		// error
		// flip lane if difference of orientation is larger than pi/2
		if(std::abs(this->getOrientation() - laneOrientation) > M_PI/2)
		{

			// warning
			std::cout << "lane flipped for obstacle " << this->getId() << " in lane " << reachableLanes_i.getId() << std::endl;
			//flipLane(&reachableLanes_i);
		}
*/
		/*
		 * M_2: compute the lane-following occupancy i       n the current lane
		 * for each time interval (already cut with lane borders)
		 * (maximum possible occupancy with maximum longitudinally
		 * acceleration and deceleration along the shortest path through the lane)
		 */

		if (this->getCompute_Occ_M2())
		{
			verticesM2 = this->M2_laneFollowingOccupancy(&reachableLanes_i, timeInterval);
		}
		// intersect the occupancy M1 and M2 for each time interval
		for (size_t k = 0; k < timeLength; k++)
		{
			//std::cout << "k: " << k << "size: " << verticesM2[k].size() << std::endl;
			std::vector<vertice> verticesOcc;
			if (this->getCompute_Occ_M1() && this->getCompute_Occ_M2()
					&& k < verticesM1.size() && k < verticesM2.size()
					&& verticesM1[k].size() != 0 && verticesM2[k].size() != 0)
			{


				// intersect the occupancy M_1 and M_2
				using coordinate_type = double;
				using point_type = boost::geometry::model::d2::point_xy<coordinate_type>;
				using polygon_type = boost::geometry::model::polygon<point_type>;
				using boost::geometry::get;

				// Construct
				polygon_type polygon1;
				polygon_type polygon2;
				for (size_t m = 0; m < verticesM1[k].size(); m++)
				{
					boost::geometry::append(polygon1, point_type {verticesM1[k][m].x, verticesM1[k][m].y});
				}
				// close polygon
				if (verticesM1[k].size() > 0)
				{
					boost::geometry::append(polygon1, point_type {verticesM1[k][0].x, verticesM1[k][0].y});
				}
				for (size_t m = 0; m < verticesM2[k].size(); m++)
				{
					boost::geometry::append(polygon2, point_type {verticesM2[k][m].x, verticesM2[k][m].y});
				}
				// close polygon
				if (verticesM2[k].size() > 0)
				{

					boost::geometry::append(polygon2, point_type {verticesM2[k][0].x, verticesM2[k][0].y});
				}

				std::deque<polygon_type> output;

				boost::geometry::intersection(polygon1, polygon2, output);
				polygon_type polygon_result;
				if (output.size() != 0)
				{
					polygon_result = output[0];
					std::vector<point_type> const& points = polygon_result.outer();
					for (std::vector<point_type>::size_type i = 0; i < points.size(); ++i)
					{
						vertice temp;
						temp.x = double(get<0>(points[i]));
						temp.y = double(get<1>(points[i]));
						verticesOcc.push_back(temp);
					}
				}
				else if (output.size() == 0)
				{
					//std::cout << "output empty for lane: " << reachableLanes_i.getId() << std::endl;
				}
			}
			else if (this->getCompute_Occ_M1() && k < verticesM1.size() && verticesM1[k].size() != 0)
			{
				// intersect the occupancy M_1 with the lane border
				using coordinate_type = double;
				using point_type = boost::geometry::model::d2::point_xy<coordinate_type>;
				using polygon_type = boost::geometry::model::polygon<point_type>;
				using boost::geometry::get;

				 // Construct
				polygon_type polygon1;
				polygon_type polygon2;

				for (size_t m = 0; m < reachableLanes[i]->getLeftBorderVertices().size(); m++)
				{
					boost::geometry::append(polygon1, point_type {reachableLanes[i]->getLeftBorderVertices()[m].x, reachableLanes[i]->getLeftBorderVertices()[m].y});
				}
				int rightSize = reachableLanes[i]->getRightBorderVertices().size() - 1;
				for (int m = rightSize; m >= 0; m--)
				{
					boost::geometry::append(polygon1, point_type {reachableLanes[i]->getRightBorderVertices()[m].x, reachableLanes[i]->getRightBorderVertices()[m].y});
				}
				// close polygon
				boost::geometry::append(polygon1, point_type {reachableLanes[i]->getLeftBorderVertices()[0].x, reachableLanes[i]->getLeftBorderVertices()[0].y});
				for (size_t m = 0; m < verticesM1[k].size(); m++)
				{
					boost::geometry::append(polygon2, point_type {verticesM1[k][m].x, verticesM1[k][m].y});
				}
				// close polygon
				if (verticesM1[k].size() > 0)
				{
					boost::geometry::append(polygon2, point_type {verticesM1[k][0].x, verticesM1[k][0].y});
				}

				//boost::geometry::correct(polygon1);
				//boost::geometry::correct(polygon2);
				std::deque<polygon_type> output;
				boost::geometry::intersection(polygon1, polygon2, output);
				polygon_type polygon_result;
				if (output.size() != 0)
				{
					polygon_result = output[0];
					std::vector<point_type> const& points = polygon_result.outer();
					for (std::vector<point_type>::size_type i = 0; i < points.size(); ++i)
					{
						vertice temp;
						temp.x = double(get<0>(points[i]));
						temp.y = double(get<1>(points[i]));
						verticesOcc.push_back(temp);
					}
				}
				else if (output.size() == 0)
				{
					std::cout << "output empty for lane: " << reachableLanes_i.getId() << std::endl;
				}
			}
			else if (this->getCompute_Occ_M2() && k < verticesM2.size() && verticesM2[k].size() > 0)
			{
				// occupancy M_2 is the complete occupancy
				std::cout << "M2 is complete occupancy." << std::endl;
				verticesOcc = verticesM2[k];
			}
			else
			{
				// the occupancy is empty
				std::cout << "empty" << std::endl;
			}
			occMatrix[i][k].forLane.push_back(reachableLanes[i]);
			timeStruct tInterval;
			tInterval.startingTime = timeInterval.startingTime + timeInterval.timeStep*k;
			tInterval.timeStep = timeInterval.timeStep;
			tInterval.ending = timeInterval.startingTime + (k+1)*timeInterval.timeStep;
			occMatrix[i][k].timeInterval = tInterval;
			occMatrix[i][k].vertices = verticesOcc;
		}
		std::vector <std::vector <occTypes> >* occMatrixPointer = this->getOccupancy()->getOccMatrix();
		(*occMatrixPointer) = occMatrix;
	}
}


void vehicle::manageConstraints()
{

	dynamicObstacle::manageConstraints();

	/*
	 * manage constraint C1 (maximum speed of obstacle):
	 * if speed of obstacle is higher than the speed limit * speeding
	 * factor, increase speeding factor
	 */
	if (!(this->getInLane().empty()))
	{
		if (this->getVelocity() > (this->getInLane().front()->getMaxSpeedLimit() *
			(1 + this->getSpeedingFactor())))
		{
			// warning
			double speedingFactor_currently = (this->getVelocity() / this->getInLane().front()->getMaxSpeedLimit()) - 1;
			this->setSpeedingFactor(speedingFactor_currently);
		}
	}
	else
	{
		// warning
	}

	/*
	 * manage constraint C2 (modeling maximum engine power),
	 * which is only applicable for vehicles
	 */

	/*
	 * if acceleration is higher than by formula (a = a_max * v_s/v)
	 * for speeds above v_s, set v_s to infinity
	 */
	if (this->getVelocity() < this->getVmax() &&
		this->getVelocity() > this->getVs() &&
				this->getAcceleration() > (this->getAmax() *
						(this->getVs() / this->getVelocity())))
		{
			// warning
			std::cout << "The acceleration of obstacle " << this->getId() << "(a= " << this->getAcceleration() <<
					" is higher than its modelled maximum acceleration (a = a_max * v_s/v " <<
					(this->getAmax() * (this->getVs() / this->getVelocity())) << " for the switching velocity (v_s = " <<
					this->getVs() << ")." << std::endl;
			this->setVs(9999);
		}

	/*
	 * manage constraint C3 (not allowing backward driving):
	 * if speed of obstacle is negative, backward driving must be considered
	 */
	if (this->getConstraint3() && this->getVelocity() < 0)
	{
		std::cout << "The speed of obstacle " << this->getId() <<  " (v = " <<  this->getVelocity() <<
				") is negative, although constraint 3 was true" << std::endl;
		this->setConstraint3(false);
	}

	/*
	// ego vehicle can go anywhere
	if (typeid(*dynamicObst) == typeid(egoVehicle))
	{
		dynamicObst.setCnstraint5(false);
	}
	*/
}

std::vector <std::vector <vertice> > vehicle::M2_laneFollowingOccupancy(lane* curLane, timeStruct timeInterval)
{
	std::vector <std::vector <vertice> > occM2;
	double laneSpeedLimit = -1;
	for (size_t i = 0; i < curLane->getSpeedLimit().size(); i++)
	{
		if (curLane->getSpeedLimit()[i] > laneSpeedLimit)
		{
			laneSpeedLimit = curLane->getSpeedLimit()[i];
		}
	}
	double v_max_M2 = min(this->getVmax(), laneSpeedLimit * (1 + this->getSpeedingFactor()));

	/*
	 * find the index of the lane vertice of the inner bound which has the minimal
	 * distance to the obstacle's postion, which must not be in the current lane
	 * starting coordinates of the inflection point segmentation
	 */
	size_t iPath, iBorder;
	vertice temp;
	temp.x = this->getXpos();
	temp.y = this->getYpos();

	std::vector <size_t> ids = findInnerVerticeOfPosition(temp, curLane->getShortestPath(), curLane->getLeftBorderVertices(), curLane->getRightBorderVertices());

	iPath = ids[0];
	iBorder = ids[1];

	// assign the inner bound at this vertice
	border innerBound;
	if (curLane->getShortestPath().side[iPath]) //i.e. left
	{
		innerBound = curLane->getLeftBorder();
	}
	else //i.e. right
	{
	    innerBound = curLane->getRightBorder();
	}

	/*
	 * calculate the distance between the vertice iBorder and the perpendicular
	 * projection of the obstacle's coordinates onto the inner bound, i.e. the
	 * path variable xi at the vertice iBorder (xi = 0 at objPosition)
	 * (instead of constructing the initial inner lane bound which is
	 * perdendicular to hStart)
	 * xiObstacle is positive, if projection is behind in driving direction
	 * xiObstacle is negative, if projection is in front in driving direction
	 */
	double xiObstacle = calcProjectedDistance(iBorder, innerBound, temp);

	//std::cout << "xiObstacle: " << xiObstacle << std::endl;
	double xiClosest_t, xiClosest_tdt, xiFurthest, xiClosest;
	rectangle* rect = dynamic_cast<rectangle*>(this->getShape());
	// compute the occupancy M2 for each time interval

	for (double t = timeInterval.startingTime; t < timeInterval.ending-0.0001; t = t + timeInterval.timeStep)
	{
		//std::cout << "t: " << t << std::endl;
	    // xi is the path variable (equation (7))
	    // compute the closest longitudinal reach, i.e. xi of hStart (Definition 7)
	    // (xi might be negative if backward driving is allowed)
	    xiClosest_t = closestLongitudinalReach(t, this->getVelocity(), this->getAmax(), this->getConstraint3());
	    xiClosest_tdt = closestLongitudinalReach(t+timeInterval.timeStep, this->getVelocity(), this->getAmax(), this->getConstraint3());
	    if (xiClosest_t < xiClosest_tdt)
	    {
	    	xiClosest = xiClosest_t;
	    }
	    else
	    {
	    	xiClosest = xiClosest_tdt;
	    }
	    //std::cout << "xiClosest: " << xiClosest << std::endl;
	    // compute the furthest longitudinal reach, i.e. xi of hfinal (Definition 7)
	  //xiFurthest = furthestLongitudinalReach(t+timeInterval.timeStep, this->getVelocity(), this->getVs(), v_max_M2, this->getAmax(), iPath, curLane);
	 // std::cout << "t: " << t <<"  new: " << xiFurthest << std::endl;
	    xiFurthest = furthestLongitudinalReach_old(t+timeInterval.timeStep, this->getVelocity(), this->getVs(), v_max_M2, this->getAmax());
	  // std::cout << "t: " << t <<"  old: " << xiFurthest << std::endl;
	    // compute the polygon vertices of the occupancy when following the
	    // shortest path through the current lane starting at xiClosest and
	    // ending at xiFurthest with subtracted and added half of the obstacle's
	    // length, respectively
	    occM2.push_back(inflectionPointSegmentation(iPath, xiObstacle,
	    		(xiClosest - (0.5 * rect->getLength())), (xiFurthest + (0.5 * rect->getLength())),
				curLane->getShortestPath(), curLane->getLeftBorder(), curLane->getRightBorder()));
	}
	return occM2;
}
