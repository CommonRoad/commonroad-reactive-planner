#include "lane.h"
#include "lane_operations.h"
#include "../../geometry/geometricOperations.h"
#include <math.h>


#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

bool lane::initialized;
size_t lane::laneIdCount;

lane::lane()
{
	// if static variable is not initialized yet
	if (!lane::initialized)
	{
		lane::laneIdCount = 0;
		lane::initialized = true;
	}
	lane::laneIdCount++;
	id = laneIdCount;
	adjacentLeftDir = "";
	adjacentRightDir = "";
}

lane::~lane()
{
}

void lane::setId(const size_t num)
{
	id = num;
}

void lane::setLeftBorder(const std::vector <vertice> vertices,
						const std::vector <double> distances,
						const std::vector <double> curvature)
{
	border newBorder;
	newBorder.vertices = vertices;
	newBorder.distances = distances;
	newBorder.curvature = curvature;
	leftBorder = newBorder;
}

void lane::setRightBorder(const std::vector <vertice> vertices,
						const std::vector <double> distances,
						const std::vector <double> curvature)
{
	border newBorder;
	newBorder.vertices = vertices;
	newBorder.distances = distances;
	newBorder.curvature = curvature;
	rightBorder = newBorder;
}

void lane::setLeftBorderVertices(std::vector <vertice> vertices)
{
	leftBorder.vertices = vertices;
}

void lane::setRightBorderVertices(std::vector <vertice> vertices)
{
	rightBorder.vertices = vertices;
}

void lane::setSpeedLimit(const std::vector <double> lim)
{
	speedLimit = lim;
}

void lane::addAssemblingLanelet(vehicularLanelet* ass)
{
	assemblingLanelets.push_back(ass);
}

void lane::setAssemblingLanelet(std::vector <vehicularLanelet*> ass)
{
	assemblingLanelets = ass;
}

void lane::setAdjacentLeft(lane* adjLane, const std::string drv)
{
	adjacentLeft.push_back(adjLane);
	adjacentLeftDir = drv;
}

void lane::setAdjacentRight(lane* adjLane, const std::string drv)
{
	adjacentRight.push_back(adjLane);
	adjacentRightDir = drv;
}

void lane::setShortestPath(shortestPath shortest)
{
	shPath.curvature = shortest.curvature;
	shPath.indexBorder = shortest.indexBorder;
	shPath.side = shortest.side;
	shPath.xi = shortest.xi;
}
/*
void lane::setSide(const std::string sd)
{
	side = sd;
}
*/
void lane::setCenterVertices(const std::vector <vertice> centerVer)
{
	centerVertices = centerVer;
}

void lane::addCenterVertice(const vertice centerVer)
{
	centerVertices.push_back(centerVer);
}

void lane::addCenterVertices(const std::vector <vertice> centerVer)
{
	for (size_t i = 0; i < centerVer.size(); i++)
	{
		centerVertices.push_back(centerVer[i]);
	}
}

void lane::setLeftBorderDistances(const std::vector <double> distances)
{
	leftBorder.distances = distances;
}

void lane::setRightBorderDistances(const std::vector <double> distances)
{
	rightBorder.distances = distances;
}

void lane::setLeftBorderCurvature(const std::vector <double> curvature)
{
	leftBorder.curvature = curvature;
}

void lane::setRightBorderCurvature(const std::vector <double> curvature)
{
	rightBorder.curvature = curvature;
}

void lane::copyLane(lane copiedLane)
{
	leftBorder = copiedLane.getLeftBorder();
	rightBorder = copiedLane.getRightBorder();
	assemblingLanelets = copiedLane.getAssLanelets();
	speedLimit = copiedLane.getSpeedLimit();
	centerVertices = copiedLane.getCenterVertices();
	adjacentLeft = copiedLane.getAdjacentLeft();
	adjacentLeftDir = copiedLane.getAdjacentLeftDir();
	adjacentRight = copiedLane.getAdjacentRight();
	adjacentRightDir = copiedLane.getAdjacentRightDir();
}

size_t lane::getId() const
{
	return id;
}

std::vector <double> lane::getOptVelProfile() const
{
	return optVelocityProfile;
}

border lane::getLeftBorder() const
{
	return leftBorder;
}

border lane::getRightBorder() const
{
	return rightBorder;
}

std::vector <vertice> lane::getLeftBorderVertices() const
{
	return leftBorder.vertices;
}

std::vector <vertice> lane::getRightBorderVertices() const
{
	return rightBorder.vertices;
}

std::vector <vehicularLanelet*> lane::getAssLanelets() const
{
	return assemblingLanelets;
}

std::vector <double> lane::getSpeedLimit() const
{
	return speedLimit;
}

double lane::getMaxSpeedLimit() const
{
	double max = 0;
	for (size_t i = 0; i < speedLimit.size(); i++)
	{
		if (speedLimit[i] > max)
		{
			max = speedLimit[i];
		}
	}
	return max;
}

center_struct lane::getCenter() const
{
	return center;
}

std::vector <lane*> lane::getAdjacentLeft()
{
	return adjacentLeft;
}

std::vector <lane*> lane::getAdjacentRight()
{
	return adjacentRight;
}

std::string lane::getAdjacentLeftDir() const
{
	return adjacentLeftDir;
}

std::string lane::getAdjacentRightDir() const
{
	return adjacentRightDir;
}

/*
std::string lane::getSide() const
{
	return side;
}
*/

std::vector <vertice> lane::getCenterVertices() const
{
	return centerVertices;
}

std::vector <double> lane::getLeftBorderDistances() const
{
	return leftBorder.distances;
}

std::vector <double> lane::getLeftBorderCurvature() const
{
	return leftBorder.curvature;
}

std::vector <double> lane::getRightBorderDistances() const
{
	return rightBorder.distances;
}

std::vector <double> lane::getRightBorderCurvature() const
{
	return rightBorder.curvature;
}

shortestPath lane::getShortestPath() const
{
	return shPath;
}

void lane::calculateShortestPath()
{
	// calcuate the point-wise distance of the points of the borders
	// (not the path length distance xi)
	this->setLeftBorderDistances(calcPathDistances(this->getLeftBorderVertices()));
	this->setRightBorderDistances(calcPathDistances(this->getRightBorderVertices()));

	// calculate the signed curvature for each border
	this->setLeftBorderCurvature(calcCurvature(this->getLeftBorderVertices()));
	this->setRightBorderCurvature(calcCurvature(this->getRightBorderVertices()));

	// find the shortest path through the lane network (Definition 8)
	this->setShortestPath(findShortestPath(this->getLeftBorder(), this->getRightBorder()));
}

void lane::calculateOptimalVelocityProfile()
{

	// iterators
	size_t i;
	size_t k;

	double a_x_max = 8; // maximal feasible acceleration in tangential direction
	double a_y_max = 3; // maximal feasible acceleration in normal direction

	shortestPath path = shPath;
	size_t path_length = path.curvature.size();
	std::vector <double> v;

	std::vector<double> radius;
	std::cout << "size: " << path.curvature.size() << std::endl;
	for (i = 0; i < path.curvature.size(); i++)
	{
		path.curvature[i] += std::pow(10, -12); // add small value to avoid zero-divison
		radius.push_back(1.0/std::abs(path.curvature[i]));
		v.push_back(10000.0);
	}

	std::vector<double> v_opt = v;

	std::vector<size_t> minima;
	std::vector<region> regions;

	// find minima and regions
	getLocalConstantRegions(&radius, &minima, &regions);

	size_t idx;
	std::vector<double> v_temp;
	double a_x;
	double s_diff;
	double t;
	double v_c;

	// calculate velocity for minima first
	for(i=0; i<minima.size(); i++)
	{
		idx = minima[i];
		v_temp = v;


		// determine v_crit and accelerations at minimum
		v_temp[idx] = sqrt(a_y_max*radius[idx]);

		// decelaration to the left of the minimum
		for (k = 1; k < idx; k++)
		{
			v_c = v_temp[idx-k+1]; // critical velocity of last vertex
			if(radius[idx-k] < radius[idx-k+1])
			{
				break; // break deceleration if radius gets smaller again
			}
			a_x = a_x_max * sqrt(1 - std::pow(std::pow(v_c, 2)/(radius[idx-k]*a_y_max), 2));
			s_diff = std::abs(shPath.xi[idx-k+1] - shPath.xi[idx-k]); // distance between current and last vertex
			t = v_c/a_x + sqrt(std::pow((v_c/a_x),2)+2*s_diff/a_x); // determine time to reconstruct velocity
			v_temp[idx-k] = v_c+a_x*t;
			// set acceleration in x direction
			//a_x_opt[idx-k] = min(a_x_opt[idx-k], a_x);
		}
		// deceleration to the right of the minimum
		for(k = 1; k <path_length-idx; k++)
		{
			v_c = v_temp[idx+k-1]; // critical velocity of last vertex
			if(radius[idx+k] < radius[idx+k-1])
			{
				break; // break deceleration if radius gets smaller again
			}
			a_x = a_x_max * sqrt(1 - std::pow(v_c,2)/std::pow((radius[idx+k]*a_y_max), 2));
			s_diff = std::abs(shPath.xi[idx+k-1]-shPath.xi[idx+k]);  // distance between current and last vertex
			t = v_c/a_x + sqrt(std::pow((v_c/a_x), 2)+2*s_diff/a_x); //calculate time for reconstructing v
			v_temp[idx+k] = v_c+a_x*t;
			// set acceleration in x_direction
			//a_x_opt[idx+k] = min(a_x_opt[idx+k], a_x);
		}

		// minimum velocity of optimal velocity curve
		for (k = 0; k < path_length; k++)
		{
			v_opt[k] = min(v_opt[k], v_temp[k]);
		}
	}

	region r;
	// calculate velocity of optimal velocity curve
	for(i = 0; i < regions.size(); i++)
	{
		r = regions[i];
		v_c = sqrt(a_y_max*radius[r.begin]);
		for (k = r.begin; k < r.end; k++)
		{
			v_opt[k] = min(v_opt[k], v_c);
			//a_x_opt[k] = min(a_x_opt[k], 0.0);
		}
	}

	optVelocityProfile = v_opt;
}
