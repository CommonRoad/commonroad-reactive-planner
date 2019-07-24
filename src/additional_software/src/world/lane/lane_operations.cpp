#include "lane_operations.h"
#include "../../geometry/geometricOperations.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>

#include <stdexcept>

void createLanesFromLanelets(std::vector <lane>* lanes, std::vector <vehicularLanelet>* vehicularLanelets)
{
	std::vector <vehicularLanelet*> firstLanelets = noPredecessorLanelets(vehicularLanelets);
	size_t size = firstLanelets.size();
	size_t m;

	for (size_t i = 1; i <= size; i++)
	{

		/* the size of laneStruct might have changed after the recursive function
	    *  combinevehicularLaneletAndSuccessors(). hence, set the new variable m to write
	    *  into the next empty row of the struct
	    */
		if (i != 1)
		{
			m = lanes->size() + 1;
		}
		else
		{
			m = i;
		}

		//set the vehicularLanelet properties of the ith first vehicularLanelet into the list
		lane laneObject;
		laneObject.setLeftBorderVertices(firstLanelets[i-1]->getLeftBorder());
		laneObject.setRightBorderVertices(firstLanelets[i-1]->getRightBorder());
		laneObject.addAssemblingLanelet(firstLanelets[i-1]);
		std::vector <double> speedLim(firstLanelets[i-1]->getLeftBorder().size(), firstLanelets[i-1]->getSpeedLimit());
		laneObject.setSpeedLimit(speedLim);
		laneObject.addCenterVertices(firstLanelets[i-1]->getCenter());

		lanes->push_back(laneObject);

		/*
		 * copy the first properties in the next field of the laneStruct, if
		 * there is a road fork
		 */
		for (size_t e = m+1; e <= m+firstLanelets[i-1]->getSuccessors().size()-1; e++)
		{
				lane aNewLane;
				aNewLane.copyLane((*lanes)[m-1]);
				lanes->push_back(aNewLane);
		}


		for (size_t k = 1; k <= firstLanelets[i-1]->getSuccessors().size(); k++)
		{

			if (((*lanes)[m+k-2].getAssLanelets().size()
					> (*lanes)[m-1].getAssLanelets().size()) ||
					((*lanes)[m+k-2].getAssLanelets().back()->getId() !=
					(*lanes)[m-1].getAssLanelets()[(*lanes)[m+k-2].getAssLanelets().size()-1]->getId()))
			{
				lane newLane;
				lane* specificLane = &(*lanes)[m-1];
				newLane.addAssemblingLanelet(specificLane->getAssLanelets().front());
				newLane.setLeftBorderVertices(specificLane->getAssLanelets().front()->getLeftBorder());
				newLane.setRightBorderVertices(specificLane->getAssLanelets().front()->getRightBorder());
				std::vector <double> lim(specificLane->getAssLanelets().front()->getLeftBorder().size(),
						specificLane->getAssLanelets().front()->getSpeedLimit());
				newLane.setSpeedLimit(lim);
				newLane.addCenterVertices(specificLane->getAssLanelets().front()->getCenter());
				lanes->push_back(newLane);

				combineLaneletAndSuccessors(lanes, firstLanelets[i-1]->getSuccessors()[k-1], lanes->size());
			}
			else
			{
				combineLaneletAndSuccessors(lanes, firstLanelets[i-1]->getSuccessors()[k-1], m+k-1);
			}
		}

	}
	size_t numLanes = lanes->size();
	/*
	 * add left and right adjacent lane for all lanes
	 * (if all vehicularLanelets are adjacent with the same lane,
	 * the lane is adjacent to the lane)
	 */
	for (size_t i = 0; i < numLanes; i++)
	{
		// adjacent left
		if ((*lanes)[i].getAssLanelets()[0]->getAdjacentLeft().size() != 0)
		{
			bool leftFlag;
			std::vector <lane*> leftLanes;
			// find the left lane
			if((*lanes)[i].getAssLanelets().front()->getAdjacentLeft().size() != 0)
			{
				leftLanes = findLaneByLanelet(lanes, (*lanes)[i].getAssLanelets().front()->getAdjacentLeft()[0]);
			}
			for (size_t j = 0; j < leftLanes.size(); j++)
			{
				// check whether all vehicularLanelets of this ith lane have only left adjacent
				// vehicularLanelets which belong to the left lane
				leftFlag = true;
				for (size_t k = 1; k < (*lanes)[i].getAssLanelets().size(); k++)
				{
					// check whether the vehicularLanelet has a left adjecent vehicularLanelet
					if ((*lanes)[i].getAssLanelets()[k]->getAdjacentLeft().size() == 0)
					{
						leftFlag = false;
						break;
					}
					else
					{
						std::vector <lane*> vectorLanes = findLaneByLanelet(lanes, (*lanes)[i].getAssLanelets()[k]->getAdjacentLeft()[0]);
						for (size_t l = 0; l < vectorLanes.size(); l++)
						{
							leftFlag = false;
							if (leftLanes[j] == vectorLanes[l])
							{
								leftFlag = true;
								break;
							}
						}
					}
				}
				if (leftFlag)
				{
					(*lanes)[i].setAdjacentLeft(leftLanes[j], (*lanes)[i].getAssLanelets().front()->getAdjacentLeftDir());
					break;
				}
			}
		}

		// adjacentRight
		if ((*lanes)[i].getAssLanelets()[0]->getAdjacentRight().size() != 0)
		{
			bool rightFlag;
			std::vector <lane*> rightLanes;
			// find the right lane
			if((*lanes)[i].getAssLanelets().front()->getAdjacentRight().size())
			{
				rightLanes = findLaneByLanelet(lanes, (*lanes)[i].getAssLanelets().front()->getAdjacentRight()[0]);
			}
			for (size_t j = 0; j < rightLanes.size(); j++)
			{
				// check whether all vehicularLanelets of this ith lane have only right adjacent
				// vehicularLanelets which belong to the right lane
				rightFlag = true;
				for (size_t k = 1; k < (*lanes)[i].getAssLanelets().size(); k++)
				{
					// check whether the vehicularLanelet has a right adjecent vehicularLanelet
					if ((*lanes)[i].getAssLanelets()[k]->getAdjacentRight().size() == 0)
					{
						rightFlag = false;
						break;
					}
					else
					{
						std::vector <lane*> vectorLanes = findLaneByLanelet(lanes, (*lanes)[i].getAssLanelets()[k]->getAdjacentRight()[0]);
						for (size_t l = 0; l < vectorLanes.size(); l++)
						{
							rightFlag = false;
							if (rightLanes[j] == vectorLanes[l])
							{
								rightFlag = true;
								break;
							}
						}
					}
				}
				if (rightFlag)
				{
					(*lanes)[i].setAdjacentRight(rightLanes[j], (*lanes)[i].getAssLanelets().front()->getAdjacentRightDir());
					break;
				}
			}
		}

	}
}

void combineLaneletAndSuccessors(std::vector <lane>* laneList, vehicularLanelet* curLanelet, const size_t k)
{

	if (curLanelet == 0) { return; }
	// check for cyclic adjacencies
	lane* specificLane = &(*laneList)[k-1];
	for (size_t i = 0; i < specificLane->getAssLanelets().size(); i++) {

		if(specificLane->getAssLanelets()[i]->getId() == curLanelet->getId())
		{
			std::cout << "The cyclic adjacency of vehicularLanelet has been cut after one cycle!" << std::endl;
			return;
		}
	}

	std::vector <vertice> temp = specificLane->getLeftBorderVertices();
	for (size_t i = 1; i < curLanelet->getLeftBorder().size(); i++)
	{
		temp.push_back(curLanelet->getLeftBorder()[i]);
	}
	specificLane->setLeftBorderVertices(temp);

	temp = specificLane->getRightBorderVertices();
	for (size_t i = 1; i < curLanelet->getRightBorder().size(); i++)
	{
		temp.push_back(curLanelet->getRightBorder()[i]);
	}
	//temp.insert(temp.end(), curvehicularLanelet->getRightBorder().begin(), curvehicularLanelet->getRightBorder().end());
	specificLane->setRightBorderVertices(temp);
	specificLane->addAssemblingLanelet(curLanelet);

	std::vector <double> tempSpeed = specificLane->getSpeedLimit();
	double compSpeed = tempSpeed.back();
	tempSpeed.pop_back();

	if (compSpeed > curLanelet->getSpeedLimit())
	{
		tempSpeed.push_back(compSpeed);
	}
	else
	{
		tempSpeed.push_back(curLanelet->getSpeedLimit());
	}

	std::vector <double> curVector(curLanelet->getLeftBorder().size()-1, curLanelet->getSpeedLimit());
	tempSpeed.insert(tempSpeed.end(), curVector.begin(), curVector.end());
	specificLane->setSpeedLimit(tempSpeed);
	for (size_t m = 1; m < curLanelet->getCenter().size(); m++)
	{
		specificLane->addCenterVertice(curLanelet->getCenter()[m]);
	}

	// for all successor vehicularLanelets, copy this lane in the next rows of the struct
	for (int  e = k+1; e <= (int)curLanelet->getSuccessors().size()-1+(int)k; e++)
	{
		if ((int)(*laneList).size() < e)
		{
			lane aNewLane;
			aNewLane.copyLane((*laneList)[k-1]);
			(*laneList).push_back(aNewLane);

		}
		else
		{
			(*laneList)[e-1].copyLane((*laneList)[k-1]);
		}

	}

	for (int n = 1; n <= (int)curLanelet->getSuccessors().size(); n++)
	{
		lane* specLane1 = &(*laneList)[k-1];
		lane* specLane2 = &(*laneList)[n+k-2];

		if ((specLane2->getAssLanelets().size()
			> specLane1->getAssLanelets().size())
			||
			(specLane2->getAssLanelets().back()->getId() !=
			specLane1->getAssLanelets()[specLane2->getAssLanelets().size()-1]->getId())
			)

		{

			lane aLane;
			lane* specificLane = &(*laneList)[k-1];
			aLane.addAssemblingLanelet(specificLane->getAssLanelets().front());
			aLane.setLeftBorderVertices(specificLane->getAssLanelets().front()->getLeftBorder());
			aLane.setRightBorderVertices(specificLane->getAssLanelets().front()->getRightBorder());
			std::vector <double> lim(specificLane->getAssLanelets().front()->getLeftBorder().size(),
						specificLane->getAssLanelets().front()->getSpeedLimit());
			aLane.setSpeedLimit(lim);
			aLane.addCenterVertices(specificLane->getAssLanelets().front()->getCenter());
			(*laneList).push_back(aLane);
			size_t p = 1;

			while((*laneList).back().getAssLanelets().back()->getId() != curLanelet->getId())
			{
				(*laneList).back().addAssemblingLanelet((*laneList)[k-1].getAssLanelets()[p]);
				std::vector <vertice> temp = (*laneList).back().getLeftBorderVertices();
				for (size_t i = 1; i < (*laneList)[k-1].getAssLanelets()[p]->getLeftBorder().size(); i++)
				{
					temp.push_back((*laneList)[k-1].getAssLanelets()[p]->getLeftBorder()[i]);
				}
				(*laneList).back().setLeftBorderVertices(temp);
				temp = (*laneList).back().getRightBorderVertices();
				for (size_t i = 1; i < (*laneList)[k-1].getAssLanelets()[p]->getRightBorder().size(); i++)
				{
					temp.push_back((*laneList)[k-1].getAssLanelets()[p]->getRightBorder()[i]);
				}
				(*laneList).back().setRightBorderVertices(temp);
				std::vector <double> tempSp = (*laneList).back().getSpeedLimit();
				double speed = tempSp.back();
				tempSp.pop_back();
				if (speed > (*laneList)[k-1].getAssLanelets()[p]->getSpeedLimit())
				{
					tempSp.push_back(speed);
				}
				else
				{
					tempSp.push_back((*laneList)[k-1].getAssLanelets()[p]->getSpeedLimit());
				}
				std::vector <double> currVector((*laneList)[k-1].getAssLanelets()[p]->getLeftBorder().size()-1, (*laneList)[k-1].getAssLanelets()[p]->getSpeedLimit());
				tempSp.insert(tempSp.end(), currVector.begin(), currVector.end());
				(*laneList).back().setSpeedLimit(tempSp);
				for (size_t m = 1; m < (*laneList)[k-1].getAssLanelets()[p]->getCenter().size(); m++)
				{
					(*laneList).back().addCenterVertice((*laneList)[k-1].getAssLanelets()[p]->getCenter()[m]);
				}
				p++;
			}
			combineLaneletAndSuccessors(laneList, curLanelet->getSuccessors()[n-1], (*laneList).size());
		}
		else
		{
			combineLaneletAndSuccessors(laneList, curLanelet->getSuccessors()[n-1], k+n-1);
		}
	}


}

std::vector <lane*> findLaneByLanelet(std::vector <lane>* laneList, size_t vehicularLaneletId)
{
	std::vector <lane*> newLanes;
	for (size_t i = 0; i < (*laneList).size(); i++)
	{
		for (size_t j = 0; j < (*laneList)[i].getAssLanelets().size(); j++)
		{
			if((*laneList)[i].getAssLanelets()[j]->getId() == vehicularLaneletId)
			{
				newLanes.push_back(&(*laneList)[i]);
			}
		}
	}
	return newLanes;
}

std::vector <lane*> findLaneByPosition(std::vector <lane>* lanes, double xPos, double yPos)
{

	using coordinate_type = double;
	using point_type = boost::geometry::model::d2::point_xy<coordinate_type>;
	using polygon_type = boost::geometry::model::polygon<point_type>;


	std::vector <lane*> cutedLanes;
	for (size_t i = 0; i < (*lanes).size(); i++)
	{
		 // Construct
		polygon_type polygon1;
		polygon_type polygon2;

		// needed to close the polygon
		vertice firstVertice = (*lanes)[i].getLeftBorderVertices().front();

		//flip vertices of right border
		std::vector <vertice> flipedVertices;
		for(size_t j = 1; j <= (*lanes)[i].getRightBorderVertices().size(); j++)
		{
			flipedVertices.push_back((*lanes)[i].getRightBorderVertices()[(*lanes)[i].getRightBorderVertices().size() - j]);
		}

		for (size_t k = 0; k < (*lanes)[i].getLeftBorderVertices().size(); k++)
		{
			boost::geometry::append(polygon1, point_type {(*lanes)[i].getLeftBorderVertices()[k].x, (*lanes)[i].getLeftBorderVertices()[k].y});
		}

		for (size_t k = 0; k < (*lanes)[i].getLeftBorderVertices().size(); k++)
		{
			boost::geometry::append(polygon1, point_type {flipedVertices[k].x, flipedVertices[k].y});
		}

		// close polygon
		boost::geometry::append(polygon1, point_type {firstVertice.x, firstVertice.y});

		boost::geometry::append(polygon2, point_type {xPos, yPos});

		if(boost::geometry::intersects(polygon1, polygon2))
		{
			cutedLanes.push_back(&((*lanes)[i]));
		}
		else
		{
			// nothing
		}
	}
	return cutedLanes;
}

void flipLane(lane* aLane)
{
	lane flippedLane;
	std::vector <vertice> flippedVertices;
	//flip vertices of right border
	for(size_t j = 1; j <= aLane->getRightBorderVertices().size(); j++)
	{
		flippedVertices.push_back(aLane->getRightBorderVertices()[aLane->getRightBorderVertices().size() - j]);
	}
	aLane->setRightBorderVertices(flippedVertices);
	//flip vertices of left border
	for(size_t j = 1; j <= aLane->getLeftBorderVertices().size(); j++)
	{
		flippedVertices.push_back(aLane->getLeftBorderVertices()[aLane->getLeftBorderVertices().size() - j]);
	}
	aLane->setLeftBorderVertices(flippedVertices);
	//flip center vertices
	for(size_t j = 1; j <= aLane->getCenterVertices().size(); j++)
	{
		flippedVertices.push_back(aLane->getCenterVertices()[aLane->getCenterVertices().size() - j]);
	}
	aLane->setCenterVertices(flippedVertices);
	// flip speed limits
	std::vector <double> flippedSpeedLimit;
	for(size_t j = 1; j <= aLane->getSpeedLimit().size(); j++)
	{
		flippedSpeedLimit.push_back(aLane->getSpeedLimit()[aLane->getSpeedLimit().size() - j]);
	}
	aLane->setSpeedLimit(flippedSpeedLimit);
	// flip vehicularLanelets
	std::vector <vehicularLanelet*> flippedvehicularLanelets;
	for(size_t j = 1; j <= aLane->getAssLanelets().size(); j++)
	{
		flippedvehicularLanelets.push_back(aLane->getAssLanelets()[aLane->getAssLanelets().size() - j]);
	}
	aLane->setAssemblingLanelet(flippedvehicularLanelets);
}

std::vector <lane*> findReachableLanes(std::vector <lane>* lanes, bool constraint5, std::vector <vehicularLanelet*> vehicularLanelets, size_t inLaneSize, std::string side)
{
	std::vector <lane*> reachableByLanelets;
	std::vector <lane*> leftLane, rightLane;

	// find all vehicularLanelets which are left or right adjacent to the vehicularLanelet(s)
	for (size_t i = 0; i < vehicularLanelets.size(); i++)
	{
		// search left
		if (((inLaneSize == 1 && side == "") || side == "left") && vehicularLanelets[i]->getAdjacentLeft().size() != 0 &&
				(vehicularLanelets[i]->getAdjacentLeftDir() == "same" ||
				(vehicularLanelets[i]->getAdjacentLeftDir() == "opposite" && !constraint5)))
		{
			if (vehicularLanelets[i]->getId() == vehicularLanelets[i]->getAdjacentLeft()[0])
			{
				std::cout << "left adjacent vehicularLanelet is also searchLanelet" << std::endl;
				// warning
			}
			std::vector <lane*> adjacentLeftLane = findLaneByLanelet(lanes, vehicularLanelets[i]->getAdjacentLeft()[0]);
			if (adjacentLeftLane.size() >= 2 && i == vehicularLanelets.size() - 1)
			{
				// try to detect and remove merged lanes
				lane* temp = adjacentLeftLane[0];
				adjacentLeftLane.clear();
				adjacentLeftLane.push_back(temp);
			}
			// recursively continue the search
			std::vector <lane*> furtherAdjacentLeftLanes;
			if (vehicularLanelets[i]->getAdjacentLeft().size() != 0 && vehicularLanelets[i]->getAdjacentLeftDir() == "same")
			{
				furtherAdjacentLeftLanes = findReachableLanes(lanes, constraint5, adjacentLeftLane[0]->getAssLanelets(), 1, "left");
			}
			// add only new adjacent lanes
			for (size_t x = 0; x < adjacentLeftLane.size(); x++)
			{

				bool inserted = false;
				if (leftLane.empty())
				{
					leftLane.push_back(adjacentLeftLane[x]);
				}
				else
				{
					for (size_t y = 0; y < leftLane.size(); y++)
					{
						if (leftLane[y]->getId() == adjacentLeftLane[x]->getId())
						{
							inserted = true;
						}
					}
					if (!inserted)
					{
						leftLane.push_back(adjacentLeftLane[x]);
					}
				}
			}
			for (size_t x = 0; x < furtherAdjacentLeftLanes.size(); x++)
			{
				bool inserted = false;
				for (size_t y = 0; y < leftLane.size(); y++)
				{
					if (leftLane[y]->getId() == furtherAdjacentLeftLanes[x]->getId())
					{
						inserted = true;
					}
					}
				if (!inserted)
				{
					leftLane.push_back(furtherAdjacentLeftLanes[x]);
				}
			}
		}
		// search right
		if (((inLaneSize == 1 && side == "") || side == "right") && vehicularLanelets[i]->getAdjacentRight().size() != 0 &&
				(vehicularLanelets[i]->getAdjacentRightDir() == "same" ||
				(vehicularLanelets[i]->getAdjacentRightDir() == "opposite" && !constraint5)))
		{
			if (vehicularLanelets[i]->getId() == vehicularLanelets[i]->getAdjacentRight()[0])
			{
					std::cout << "right adjacent vehicularLanelet is also searchLanet" << std::endl;
			}
			std::vector <lane*> adjacentRightLane = findLaneByLanelet(lanes, vehicularLanelets[i]->getAdjacentRight()[0]);
			if (adjacentRightLane.size() >= 2 && i == vehicularLanelets.size() - 1)
			{
				// try to detect and remove merged lanes
				lane* temp = adjacentRightLane[0];
				adjacentRightLane.clear();
				adjacentRightLane.push_back(temp);
			}
			// recursively continue the search
			std::vector <lane*> furtherAdjacentRightLanes;
			if (vehicularLanelets[i]->getAdjacentRight().size() != 0 && vehicularLanelets[i]->getAdjacentRightDir() == "same")
			{
				furtherAdjacentRightLanes = findReachableLanes(lanes, constraint5, adjacentRightLane[0]->getAssLanelets(), 1, "right");
			}
			// add only new adjacent lanes
			for (size_t x = 0; x < adjacentRightLane.size(); x++)
			{
				bool inserted = false;
				if (rightLane.empty())
				{
					rightLane.push_back(adjacentRightLane[x]);
				}
				else
				{
					for (size_t y = 0; y < rightLane.size(); y++)
					{
						if (rightLane[y]->getId() == adjacentRightLane[x]->getId())
						{
							inserted = true;
						}
					}
					if (!inserted)
					{
						rightLane.push_back(adjacentRightLane[x]);
					}
				}
			}
			for (size_t x = 0; x < furtherAdjacentRightLanes.size(); x++)
			{
				bool inserted = false;
				for (size_t y = 0; y < rightLane.size(); y++)
				{
					if (rightLane[y]->getId() == furtherAdjacentRightLanes[x]->getId())
					{
						inserted = true;
					}
					}
				if (!inserted)
				{
					rightLane.push_back(furtherAdjacentRightLanes[x]);
				}
			}
		}
	}
	reachableByLanelets = leftLane;
	for (size_t x = 0; x < rightLane.size(); x++)
	{
		bool inserted = false;
		for (size_t y = 0; y < reachableByLanelets.size(); y++)
		{
			if (reachableByLanelets[y]->getId() == rightLane[x]->getId())
			{
				inserted = true;
			}
			}
		if (!inserted)
		{
			reachableByLanelets.push_back(rightLane[x]);
		}
	}
	return reachableByLanelets;
}

shortestPath findShortestPath(border leftBorder, border rightBorder)
{
	// start at the first vertice of the lane borders
	size_t iFirst = 0;

	shortestPath result;
	border innerBound, outerBound;

	// find inner lane bound
	if (leftBorder.curvature[iFirst] > 0) // (Definition 8)
	{
		// left bound is the inner lane bound
		innerBound = leftBorder;
		innerBound.side = 1; // flag for left
		outerBound = rightBorder;
		outerBound.side = 0; // flag for right
	}
	else
	{
		// right bound is the inner lane bound
		innerBound = rightBorder;
		innerBound.side = 0; // flag for right
		outerBound = leftBorder;
		outerBound.side = 1; // flag for left
	}

	// shortest path starts at the first vertice of the inner bound
	result.xi.push_back(0);
	result.indexBorder.push_back(iFirst);
	result.side.push_back(innerBound.side);
	result.curvature.push_back(innerBound.curvature[iFirst]);
	// 2) recursively follow all inner bounds
	result = followBound(iFirst, result, innerBound, outerBound);

	return result;
}

shortestPath followBound(size_t iStart, shortestPath shortestPath, border innerBound, border outerBound)
{
	size_t i = iStart;
	size_t j;
	while (i < innerBound.distances.size()-1)
	{
		/*
		 * check if at the next vertice the inner bound should change,
		 * i.e. curvature < 0 --> right OR curvature > 0 --> left
		 */

		if ((((innerBound.side && innerBound.curvature[i+1] < 0) || (innerBound.side == 0 && innerBound.curvature[i+1] > 0)) && (std::abs(innerBound.curvature[i+1]) > std::abs(outerBound.curvature[i+1]))) ||
				(((outerBound.side && outerBound.curvature[i+1] > 0) || (outerBound.side == 0 && outerBound.curvature[i+1] < 0)) && (std::abs(outerBound.curvature[i+1]) > std::abs(innerBound.curvature[i+1]))))
				{
					vertice piminus1 = innerBound.vertices[getNextIndex(innerBound.vertices, i, "backward", innerBound.distances)];
					vertice piplus1 = innerBound.vertices[getNextIndex(innerBound.vertices, i, "forward", innerBound.distances)];

					//  construct tangent at vertice i of inner bound (h')
					vertice hPrime;
					hPrime.x = piplus1.x - piminus1.x;
					hPrime.y = piplus1.y - piminus1.y;

					// construct normal vector to tangent at vertice i of inner bound (g)
					vertice g;
					g.x = -hPrime.y;
					g.y = hPrime.x;

					//std::cout << "i: " << i << std::endl;
					//std::cout << innerBound.vertices[i].x << std::endl;
					// construct hCross = gamma + alpha*g
					vertice gamma = innerBound.vertices[i];
					vertice hCrossStart = gamma;
					vertice hCrossEnd;
					hCrossEnd.x = gamma.x + 1*g.x;
					hCrossEnd.y = gamma.y + 1*g.y;

					//std::cout << "hCrossStart.x: " << hCrossStart.x << std::endl;
					//std::cout << "hCrossStart.y: " << hCrossStart.y << std::endl;
					//std::cout << "iStart: " << iStart << std::endl;
					//std::cout << "hCrossEnd.x: " << hCrossEnd.x << std::endl;
					//std::cout << "hCrossEnd.y: " << hCrossEnd.y << std::endl;
					/*
					 * find point mu_j on outer bound, such that mu_j is the first
					 * vertice in front of hCross in driving direction
					 * (mu_j is an approximation of mu on the lane grid)
					 */
					vertice mu_j;
					for (size_t k = iStart; k < outerBound.vertices.size(); k++)
					{
						j = k;
						mu_j = outerBound.vertices[j];
						// due to the specific orientation of hCross, mu_j is always on
						// the right of hCross to be in front in driving direction
						if (!(isLeft(hCrossStart, hCrossEnd, mu_j)))
						{
							break;
						}
					}
				/*
				 * calculate the distance from mu to mu_j along outer bound
				 * (by intersecting hCross and hPrimeOuterBound)
				 * (vector equation: mu_j + beta*hPrimeOuterBound)
				 */
				//std::cout << "j: " << j << std::endl;
				vertice hPrimeOuterBound;
				if (j > 0)
				{
					vertice mu_jminus1 = outerBound.vertices[getNextIndex(outerBound.vertices, j, "backward", outerBound.distances)];
					double norm = std::pow((std::pow((mu_jminus1.x - mu_j.x), 2) + std::pow((mu_jminus1.y - mu_j.y), 2)), 0.5);
					hPrimeOuterBound.x = (mu_jminus1.x - mu_j.x) / norm;
					hPrimeOuterBound.y = (mu_jminus1.y - mu_j.y) / norm;
				}
				else
				{
					vertice mu_jplus1 = outerBound.vertices[getNextIndex(outerBound.vertices, j, "forward", outerBound.distances)];
					double norm = std::pow((std::pow((mu_jplus1.x - mu_j.x), 2) + std::pow((mu_jplus1.y - mu_j.y), 2)), 0.5);
					hPrimeOuterBound.x = (mu_jplus1.x - mu_j.x) / norm;
					hPrimeOuterBound.y = (mu_jplus1.y - mu_j.y) / norm;
				}
				double beta  = calcVectorIntersectionPoint(gamma, g, mu_j, hPrimeOuterBound);

				// update shortestPath:
				size_t iStartNew;
				if (j > i) // the new inner bound (j) is ahead of the old one (i)
				{
					//std::cout << "beta: " << beta << std::endl;
					// add distance beta to path variable xi
					shortestPath.xi.push_back(shortestPath.xi.back() + beta);
					// the current index is j
					iStartNew = j;
				}
				else // % j <= i
				{
					// the new inner bound (j) is behind of the old one (i), so it
					// must follow up, as we have already checked for inflection
					// points until i+1 --> follow up on the outer bound to i+1:
					// add distance beta (i.e. distance mu to mu_j) and the distances
					// from mu_j until (i+1) to path variable xi
					double sum = 0;
					for (size_t z = j + 1; z <= i + 1; z++)
					{
						sum += outerBound.distances[z];
					}
					//std::cout << "beta: " << beta << std::endl;
					//std::cout << "sum: " << sum << std::endl;
					shortestPath.xi.push_back(shortestPath.xi.back() + beta + sum);
					// the current index is i+1
					iStartNew = i+1;
				}
				// the current index of the border is the new starting index
				shortestPath.indexBorder.push_back(iStartNew);
				// after the inflection point, the outer bound is the new inner bound
				shortestPath.side.push_back(outerBound.side);
				// set the curvature
				shortestPath.curvature.push_back(outerBound.curvature[iStartNew]);

				// DEBUG: check if outer bound should really become the new inner bound
				if (shortestPath.side.back()) // -> left
				{
					size_t temp;
					if (outerBound.curvature[i+1] > 0)
					{
						temp = 1;
					}
					else
					{
						temp = -1;
					}
					if (temp < 0 && (std::abs(outerBound.curvature[i+1]) > std::abs(innerBound.curvature[i+1])))
					{
						// warning
					}
				}
				else // ~shortestPath.side(end) -> right
				{
					size_t temp;
					if (outerBound.curvature[i+1] > 0)
					{
						temp = 1;
					}
					else
					{
						temp = -1;
					}
					if (temp > 0 && (std::abs(outerBound.curvature[i+1]) > std::abs(innerBound.curvature[i+1])))
					{
						// warning
					}
				}
				/*
				 * 5) follow the new inner bound from vertice iStartNew with
				 * switched inner and outer lane bound(recursively call followBound() until the end of the inner bound is reached)
				 */
				shortestPath = followBound(iStartNew, shortestPath, outerBound, innerBound);
				// do not continue on the former inner bound, as an inflection point
				// has been reached and the new inner bound has been followed
				break;
			}
		else // between the vertices i and (i+1) is no inflection point
		{
			//take one step along the current inner bound but omit vertices
			// which are identical (which can be the case for inner bounds)
			j = getNextIndex(innerBound.vertices, i, "forward", innerBound.distances);

			double sum = 0;
			for (size_t a = i+1; a <= j; a++)
			{
				sum += innerBound.distances[a];
			}
			//adding the distance between the ith and jth vertice to path variable xi
			shortestPath.xi.push_back(shortestPath.xi.back() + sum);
			// the current index is j
			shortestPath.indexBorder.push_back(j);
			// the side is the inner bound
			shortestPath.side.push_back(innerBound.side);
			// set the curvature
			shortestPath.curvature.push_back(innerBound.curvature[j]);

			// DEBUG: check if inner bound is really inner bound
			if (shortestPath.side.back()) // -> left
			{
				if (innerBound.curvature[i+1] < 0 && std::abs(innerBound.curvature[i+1]) > std::abs(outerBound.curvature[i+1]))
				{
					// warning
				}
			}
			else // ~shortestPath.side(end) -> right
			{
				if (innerBound.curvature[i+1] > 0 && std::abs(innerBound.curvature[i+1]) > std::abs(outerBound.curvature[i+1]))
				{

				}
			}

		// continue following the bound at j
		i = j;
		}
	}
	return shortestPath;
}
