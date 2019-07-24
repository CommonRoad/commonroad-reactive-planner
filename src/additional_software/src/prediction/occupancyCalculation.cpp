#include "occupancyCalculation.h"

#include "../world/obstacle/staticObstacle.h"
#include "../world/obstacle/vehicle.h"
#include "../geometry/rectangle.h"
#include "../geometry/geometricOperations.h"
#include "../world/lane/lane_operations.h"

#include <boost/math/tools/precision.hpp>
#include <tgmath.h>

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

std::vector <lane*> findAllReachableLanes(std::vector <lane>* lanes, bool constraint5, std::vector <lane*> inLane)
{
	bool inserted;
	/*
	std::vector <lanelet*> inLaneLanelets;

	// get the lanelets from our lanes. Make sure that we do not have duplicates
	for (size_t i = 0; i < inLane.size(); i++)
	{
		if (i == 0)
		{
			for(size_t j = 0; j < inLane[i]->getAssLanelets().size(); j++)
			{
				inLaneLanelets.push_back(inLane[i]->getAssLanelets()[j]);
			}
		}
		for(size_t j = 0; j < inLane[i]->getAssLanelets().size(); j++)
		{
			inserted = false;
			for (size_t k = 0; k < inLaneLanelets.size(); k++)
			{
				if (inLane[i]->getAssLanelets()[j]->getId() == inLaneLanelets[k]->getId())
				{
					inserted = true;
				}
			}
			if (!inserted)
			{
				inLaneLanelets.push_back(inLane[i]->getAssLanelets()[j]);
			}
		}
	}
	*/
	//std::vector <lane*> temp = findReachableLanes(lanes, constraint5, inLaneLanelets);
	std::vector <lane*> temp = findReachableLanes(lanes, constraint5, inLane[0]->getAssLanelets(), inLane.size(), "");
	std::vector <lane*> reachableLanes;
	reachableLanes = inLane;
	/*
	 * return and remove double entries
	 * (the current obstacle's lane is reachable, but
	 * do not add lanes which are obstacle's lane(s))
	 */
	for (size_t i = 0; i < temp.size(); i++)
	{
		inserted = false;
		for (size_t j = 0; j < reachableLanes.size(); j++)
		{
				if(reachableLanes[j]->getId() == temp[i]->getId())
				{
					inserted = true;
				}
		}
		if(!inserted)
		{
			reachableLanes.push_back(temp[i]);
		}
	}
	return reachableLanes;
}

std::vector <vertice> accelerationOccupancyLocal(double vx, double a_max, double tk, double tkplus1)
{
	std::vector <vertice> q;

	// set velocity vector in local coordinates, where vx = velocity, vy = 0
	vertice v;
	v.x = vx;
	v.y = 0;
	// compute center coordinates [cx, cy]
	vertice c_tk, c_tkplus1;
	c_tk.x = v.x * tk;
	c_tk.y = v.y * tk;
	c_tkplus1.x = v.x * tkplus1;
	c_tkplus1.y = v.y * tkplus1;

	// compute radius
	double r_tk = 0.5 * std::abs(a_max) * std::pow(tk, 2);
	double r_tkplus1 = 0.5 * std::abs(a_max) * std::pow(tkplus1, 2);
	// calculate the boundary bx(tkplus1): time at d(bx(t))/dt = 0
	double tStar = sqrt(2.0/3.0) * (vx / std::abs(a_max));
	double bx_tkplus1;
	if (tkplus1 < tStar)
	{
		bx_tkplus1 = (vx * tkplus1) - (std::pow(a_max, 2) * std::pow(tkplus1, 3) / (2*vx));
	}
	else
	{
		bx_tkplus1 = (vx * tStar) - (std::pow(a_max, 2) * std::pow(tStar, 3) / (2*vx));
	}

	/*
	 * compute vertices of the convex hull (Lemma 1 with modification):
	 * (polygon to over-approximate the occupancy described by two circles at
	 * tk and tkplus1)
	 */

	// vertices around the first circle (q1 and q6)
	if ((c_tk.x - r_tk) > (c_tkplus1.x - r_tkplus1))
	{
		/*
		 * if the first circle is enclosed by the second one,
		 * construct q only around the second circle (which is at tkplus1)
		 */
		vertice q1, q3, q4, q6;
		q1.x = c_tkplus1.x - r_tkplus1;
		q1.y = c_tkplus1.y + r_tkplus1;
		q3.x = c_tkplus1.x + r_tkplus1;
		q3.y = c_tkplus1.y + r_tkplus1;
		q4.x = c_tkplus1.x + r_tkplus1;
		q4.y = c_tkplus1.y - r_tkplus1;
		q6.x = c_tkplus1.x - r_tkplus1;
		q6.y = c_tkplus1.y - r_tkplus1;

		// set the polygon q
		q.push_back(q1);
		q.push_back(q3);
		q.push_back(q4);
		q.push_back(q6);
	}
	else
	{
		//construct q around the first and second circle
		vertice q1, q2, q3, q4, q5, q6;
		q1.x = c_tk.x - r_tk;
		q1.y = c_tk.y + r_tk;
		q2.x = bx_tkplus1;
		q2.y = c_tkplus1.y + r_tkplus1;
		q3.x = c_tkplus1.x + r_tkplus1;
		q3.y = c_tkplus1.y + r_tkplus1;
		q4.x = c_tkplus1.x + r_tkplus1;
		q4.y = c_tkplus1.y - r_tkplus1;
		q5.x = bx_tkplus1;
		q5.y = c_tkplus1.y - r_tkplus1;
		q6.x = c_tk.x - r_tk;
		q6.y = c_tk.y - r_tk;

		// set the polygon q
		q.push_back(q1);
		q.push_back(q2);
		q.push_back(q3);
		q.push_back(q4);
		q.push_back(q5);
		q.push_back(q6);
	}

	return q;
}

/*
std::vector <vertice> M2_laneFollowingOccupancy(obstacle* obst, lane* curLane, timeStruct timeInterval)
{
	std::vector <vertice> occM2;

	return occM2;
}
*/

double closestLongitudinalReach(double t, double v0, double a_max, bool constraint3)
{

	// To find the closest reach, the vehicle performs full braking. Hence,
	// a = - a_max.

	double t1;
	// compute the relevant time
	if (constraint3)
	{
		// as constraint3 holds, the velocity must not be negative
		// v0/a_max refers to the time needed to reach v = 0
		t1 = min(v0/a_max, t);
	}
	else // constraint3 = false
	{
		// backward driving is allowed
		t1 = t;
	}
	// compute the traveled distance s = (1/2 * a * t^2) + (v0 * t) + (s0)
	double xi = (0.5 * - a_max * std::pow(t1, 2.0)) + (v0 * t1);
	return xi;
}

double furthestLongitudinalReach_old(double t, double v0, double v_s, double v_max, double a_max)
{
	double t1, t2, t3, v1, xi, xi1, xi2, xi3;
	// compute the relevant time intervals (t = t1 + t2 + t3)
	// time interval 1: accelerate from v0 to v_s with a = a_max
	if (v0 < v_s && v0 < v_max)
	{
		//(v_s - v0)/a_max refers to the time needed to reach v = v_s
		t1 = min( ((v_s - v0)/a_max), t );
	}
	else
	{
		t1 = 0;
	}
	v1 = a_max * t1 + v0;


	// time interval 2: accelerate from v_s to v_max with a = a_max * v_s/v
	if (v1 >= v_s && v1 < v_max) // (and if also t1 ~= 0, then v1 == v_s)
	{
		// v_max^2 - v1^2)/(2 * a_max * v_s) refers to the time needed to
		// reach v = v_max (Weiß, (5.9))
		t2 = min( ((std::pow(v_max, 2.0) - std::pow(v1, 2.0))/(2.0 * a_max * v_s)), (t - t1) );
	}
	else
	{
		t2 = 0;
	}

	// time interval 3: constant velocity (a = 0)
	// t3 is the remaining time
	t3 = t - t1 - t2;

	// compute the travelled distance in each time interval
	xi1 = (0.5 * a_max * std::pow(t1, 2)) + (v0 * t1);
	if (t2 > 0)
	{
		xi2 = (std::pow((2.0 * a_max * v_s * t2) + std::pow(v1, 2),1.5) - std::pow(v1, 3)) / (3.0 * a_max * v_s); // (Wei�, (5.8))
	}
	else
	{
		xi2 = 0;
	}
	xi3 = v_max * t3;

	// sum of travelled distances
	xi = xi1 + xi2 + xi3;
	return xi;
}


/*
 * ToDo: take into account the distance to the closest vertex on the shortest path. Currently the distance can be negative or positive.
 * For the future the closest vertex has to be in front of the car (positive distance). The fist step then is to drive to this vertex and then adjust
 * the velocity profile. It does not make sense to adjust the initial velocity of the car.
 * --> these changes have been removed, since velenis is not in use
 */
double furthestLongitudinalReach(double tf, double v0, double v_s, double v_max, double a_max, size_t index, lane* curLane)
{
	double xi; // furthest reach
	double a_x;
	double v_c;
	double t;

	size_t i;
	double s_diff;
	std::vector <double> velProf = curLane->getOptVelProfile(); // offline calculated velocity profile
	shortestPath shPath = curLane->getShortestPath();

	// ToDO: move this if-clause into an own function. No need to adjust the velocity profile for every iteration of the loop.
	if(v0 >= 0)
	{
		std::vector<double> v_initial;
		v_initial.resize(velProf.size());
		v_initial[index] = v0;
		velProf[index] = min(velProf[index], v_initial[index]);

		// adjust velocity profile by the initial velocity of the car. We start at the closest vertex.
		for(i = index; i < velProf.size(); i++)
		{
			v_c = v_initial[i-1];
			s_diff = std::abs(shPath.xi[i-1]-shPath.xi[i]); // distance to next vertex.

			// accelerate with a_x = a_max as long as v < v_s
			if(v_c < v_s && s_diff > 0)
			{
				a_x = a_max;
				double t1 = (v_s - v_c)/a_max; // time needed to reach v_s
				t = -v_c/a_x + sqrt(std::pow((v_c/a_x), 2)+2*s_diff/a_x); // time needed to reach next vertex
				if (t < t1)
				{
					s_diff = 0; // accelerate with maximum acceleration for the whole distance left
					v_c = v_c+a_x*t;
				}
				else
				{
					s_diff -= v_c*t1 + 0.5*a_x*t1*t1;
					v_c = v_s;
				}
			}
			// accelerate with a_x = a_max * (v_s/v) as long as v_s < v < v_max
			if (v_c >= v_s && v_c < v_max && s_diff > 0)
			{
				// v_max^2 - v1^2)/(2 * a_max * v_s) refers to the time needed to
				// reach v = v_max (Weiß, (5.9))
				double t2 = (std::pow(v_max, 2.0) - std::pow(v_c, 2.0))/(2.0 * a_max * v_s);
				// distance until v_max is reached
				double s_temp = (std::pow((2.0 * a_max * v_s * t2) + std::pow(v_c, 2),1.5) - std::pow(v_c, 3)) / (3.0 * a_max * v_s);
				if(s_temp > s_diff)
				{
					// time needed to reach next vertex
					t = (std::pow(cbrt(3*a_max*s_diff*v_s+std::pow(v_c,3 )), 2) - (v_c*v_c)) / (2*a_max*v_s);
					v_c = sqrt(t*2*a_max*v_s+v_c*v_c);
					s_diff = 0;
				}
				else
				{
					s_diff -= s_temp;
					v_c = v_max; // reached v_max
				}
			}
			// accelerate with a_x = 0 if v = v_max
			if(v_c <= v_max && s_diff > 0)
			{
				v_c = v_max;
			}
			v_initial[i] = min(v_c, v_max); // critical velocity higher than speed-limit/maximum power of car
			velProf[i] = min(velProf[i], v_initial[i]); // minimum of offline and online calculations at vertex
		}
	}



	double t_step;
	t = 0;
	xi = 0;

	/*
	 * Calculate furthest reach with respect to v_s and v_max. We have to keep track of the distance between to vertice and the
	 * final time for the interval
	 */
	while(index < velProf.size()-1)
	{
		double v = velProf[index];
		double a;
		t_step = 0;
		s_diff = std::abs(shPath.xi[index]-shPath.xi[index+1]); // distance to next vertex

	    if(v < v_s && s_diff > 0)
	    {
	    	a = a_max;
	    	// time needed to reach v_s
	    	double t_vs = (v_s-v)/a_max;
	    	// distance travelled until v_s is reached
	    	double s_temp = v*t_vs + 0.5*a*t_vs*t_vs;

	    	if (tf > t_vs+t && s_temp < s_diff)
	    	{
	    		s_diff -= s_temp;
	    		v = v_s;
	    		t += t_vs;
	    	}
	    	else
	    	{
	    		t_step = -v/a + sqrt(std::pow((v/a), 2)+2*s_diff/a);
	    		if (t_step+t >= tf) // reached final time?
	    		{
	    			xi += v*(tf-t) + 0.5*a*(tf-t)*(tf-t);
	    			break;
	    		}
	    		xi +=std::abs(shPath.xi[index]-shPath.xi[index+1]);
	    		t += t_step;
	    		index++;
	    		continue; // reached next vertex
	    	}
	    }
	    if(v >= v_s && v < v_max && s_diff > 0 && t < tf)
	    {
			// v_max^2 - v1^2)/(2 * a_max * v_s) refers to the time needed to
			// reach v = v_max (Weiß, (5.9))
	    	double t_vmax = (std::pow(v_max, 2.0) - std::pow(v, 2.0))/(2.0 * a_max * v_s);
			// distance until v_max is reached
	    	double s_temp = (std::pow((2.0 * a_max * v_s * t_vmax) + std::pow(v, 2),1.5) - std::pow(v, 3)) / (3.0 * a_max * v_s);
	    	if (t_vmax+t < tf && s_temp < s_diff)
	    	{
	    		s_diff -= s_temp;
	    		v = v_max;
	    		t += t_vmax;
	    	}
	    	else
	    	{
	    		t_step = (std::pow(cbrt(3*a_max*s_diff*v_s+std::pow(v,3)), 2) - (v*v)) / (2*a_max*v_s);
	    		if(t+t_step > tf) // reached final time?
	    		{
	    			xi += std::abs(shPath.xi[index]-shPath.xi[index+1]) - s_diff + (std::pow((2.0 * a_max * v_s * (tf-t)) + std::pow(v, 2),1.5) - std::pow(v, 3)) / (3.0 * a_max * v_s);
	    			break;
	    		}
	    		xi += std::abs(shPath.xi[index]-shPath.xi[index+1]);
	    		t += t_step;
	    		index++;
	    		continue; // reached next vertex
	    	}
	    }
	    if (s_diff > 0) // v = v_max
	    {
	    	t_step = s_diff/v_max;
	    	if (t_step+t >= tf) // reached final time?
	    	{
	    		xi += v_max*(tf-t) + std::abs(shPath.xi[index]-shPath.xi[index+1]) - s_diff;
	    		break;
	    	}
	    	t += t_step;
	    	s_diff -= t_step*v;
	    }

	    xi += std::abs(shPath.xi[index]-shPath.xi[index+1]); // add distance between two vertices's to furthest reach
		index++;


	}

	return xi;
}


std::vector <vertice> inflectionPointSegmentation(size_t iPath_Obstacle, double xiObstacle, double xiClosest,
		double xiFurthest, shortestPath shPath, border leftBorder, border rightBorder)
{
	std::vector <vertice> vertices;

	// transform the path variable according to the obstacle's position, i.e.
	// such that xi is zero at the obstacle's position
	std::vector <double> xi;
	for (size_t i = 0; i < shPath.xi.size(); i++)
	{
		//std::cout << "shPath.xi[i]: " << shPath.xi[i] << std::endl;
		xi.push_back(shPath.xi[i] - shPath.xi[iPath_Obstacle] + xiObstacle);
	}

	// compute the front and final bound, but make sure xiClosest and xiFurthest
	// are within the current lane:
	// (if they are not and EXTEND_LANES, the front and last bound will be
	// constructed by extending the lane border as a straight line, else
	// the occupancy will start at the beginning or end at the end of the lane,
	// repsectively)
	size_t iLeftBorder_frontBound, iRightBorder_frontBound;
	std::vector <vertice> frontBound, finalBound;

	// front bound
	if (xiClosest < xi[0])
	{
	    // closest longitudinal reach is behind the lane
	    // warning(['Lane is not long enough in backward driving direction to'...
	    //    ' represent the whole occupancy until the closest reach.'])
	   /*

	   if (extend_backward)
	   {
		   frontBound = constructBound(iPath_Obstacle, xiClosest, "front", xi, shPath, leftBorder, rightBorder, &iLeftBorder_frontBound, &iRightBorder_frontBound);
	   }
	   */
		  iLeftBorder_frontBound = 0;
		  iRightBorder_frontBound = 0;
	}
	else if (xiClosest > xi.back())
	{
	    // closest longitudinal reach is ahead of the lane
	    // warning(['Lane is not long enough in forward driving direction to'...
	    //    ' represent the whole occupancy until closest reach.'])
		/*
		if (extend_forward)
		{
			frontBound = constructBound(iPath_Obstacle, xiClosest, "front", xi, shPath, leftBorder, rightBorder, &iLeftBorder_frontBound, &iRightBorder_frontBound);
		}
		*/

	        // both bounds are ahead of the lane but will be cut, i.e.
	        // the occupancy is empty
	        //warning(['RETURN: Lane was not long enough in forward driving'...
	        //' direction.'])
	        return vertices;
	}
	else
	{
	    // regular computation of the front bound
	    frontBound = constructBound(iPath_Obstacle, xiClosest, "front", xi, shPath, leftBorder, rightBorder, &iLeftBorder_frontBound, &iRightBorder_frontBound);
	}
	size_t iLeftBorder_finalBound, iRightBorder_finalBound;
	// final bound
	if (xiFurthest < xi[0])
	{
	    // furthest longitudinal reach is behind the lane
	    //warning(['Lane is not long enough in backward driving direction to'...
	    //    ' represent the whole occupancy until the furthest reach.'])
	    /*
		if (extend_backward)
	    {
		    finalBound = constructBound(iPath_Obstacle, xiFurthest, "final", xi, shPath, leftBorder, rightBorder, &iLeftBorder_finalBound, &iRightBorder_finalBound);
	    }
	    */

	        // both bounds are behind the lane but will be cut, i.e.
	        // the occupancy is empty
	        //warning(['RETURN: Lane was not long enough in backward driving'...
	        //' direction.'])
	        return vertices;
	}
	else if (xiFurthest > xi.back())
	{
	    // furthest longitudinal reach is ahead of the lane
	    // warning(['Lane is not long enough in forward driving direction to'...
	    //    ' represent the whole occupancy until furthest reach.'])
		/*
	    if (extend_forward)
	    {
	    	finalBound = constructBound(iPath_Obstacle, xiFurthest, "final", xi, shPath, leftBorder, rightBorder, &iLeftBorder_finalBound, &iRightBorder_finalBound);
	    }
	    */
	        iLeftBorder_finalBound = leftBorder.vertices.size()-1;
	        iRightBorder_finalBound = iLeftBorder_finalBound;
	}
	else
	{
	    // regular computation of the final bound
	    finalBound = constructBound(iPath_Obstacle, xiFurthest, "final", xi, shPath, leftBorder, rightBorder, &iLeftBorder_finalBound, &iRightBorder_finalBound);
	}
	// extract the left and right lane border vertices which enclose the shortest path
	std::vector <vertice> verticesLeft;
	for (size_t i = iLeftBorder_frontBound; i <= iLeftBorder_finalBound; i++)
	{
		verticesLeft.push_back(leftBorder.vertices[i]);
	}
	std::vector <vertice> verticesRight;

	for (int i = iRightBorder_finalBound; i >= int(iRightBorder_frontBound); i--)
	{
		verticesRight.push_back(rightBorder.vertices[i]);
	}
	// save the vertices array of the computed occupancy (clockwise-ordered)
	size_t i;
	for (i = 0; i < frontBound.size(); i++)
	{
		//std::cout << "frontBound[i]:" << frontBound[i].x << std::endl;
		vertices.push_back(frontBound[i]);
	}
	for (i = 0; i < verticesLeft.size(); i++)
	{
		//std::cout << "verticesLeft[i]:" << verticesLeft[i].x << std::endl;
		vertices.push_back(verticesLeft[i]);
	}
	for (i = 0; i < finalBound.size(); i++)
	{
		//std::cout << "finalBound[i]:" << finalBound[i].x << std::endl;
		vertices.push_back(finalBound[i]);
	}
	for (i = 0; i < verticesRight.size(); i++)
	{
		//std::cout << "verticesRight[i]:" << verticesRight[i].x << std::endl;
		vertices.push_back(verticesRight[i]);
	}

	return vertices;
}

std::vector <vertice> constructBound(size_t iPath_Obstacle, double xiBound, std::string typeOfBound, std::vector <double> xi, shortestPath shPath, border leftBorder,
								border rightBorder, size_t* iLeftBorder_Bound, size_t* iRightBorder_Bound)
{
	// construct the bound of the occupancy, such that xiBound is enclosed
	// (over-approximation):
	// (note that xiBound might be negative if backward driving is allowed)
	size_t j;

	if (xiBound >= xi[iPath_Obstacle])
	    // follow the shortest path forwards until just before xiClosest is reached
	    for (j = iPath_Obstacle; j < (xi.size()-1); j++)
	    {
	        if (xi[j+1] > xiBound)
	        {
	            // the bound is between the vertex j and the next vertex (j+1)
	            break;
	        }
	        // else step forward to vertice (j+1)
	    }
	else // (xiBound - xiObstacle) < 0
	{
	    // check if iPath_Obstacle is not at the beginning of the bound
	    if (iPath_Obstacle > 0)
	    {
	        // follow the shortest path backwards until xiBound is passed
	        for (j = (iPath_Obstacle-1); j > 0; j--)
	        {
	            if (xi[j] < xiBound)
	            {
	                // the bound is between vertex j and previous vertex (j+1),
	                // which is closer to the obstacle
	                break;
	            }
	            // else step back to vertex (j-1)
	        }
	    }
	    else // iPath_Obstacle == 1
	    {
	        j = 0;
	    }
	}

	// the bound is between the vertices j and (j+1):
	// set indexes on inner bound
	size_t iPath_Bound_j = j;
	size_t iInnerBound_Bound_j = shPath.indexBorder[iPath_Bound_j];


	// assign the inner bound at vertice j
	border innerBound, outerBound;
	if (shPath.side[iPath_Bound_j]) // i.e. left
	{
	    innerBound = leftBorder;
	    outerBound = rightBorder;
	}
	else // i.e. right
	{
	    innerBound = rightBorder;
	    outerBound = leftBorder;
	}

	// find the next (non-identical) vertex on the inner bound
	size_t iInnerBound_Bound_jplus1 = getNextIndex(innerBound.vertices, iInnerBound_Bound_j, "forward", innerBound.distances);
	//std::cout << "iInnerBound_Bound_jplus1: " << iInnerBound_Bound_jplus1 << std::endl;

	//std::cout << "iInnerBound_Bound_jplus1: " << iInnerBound_Bound_jplus1 << std::endl;
	//  distance on inner bound between vertex j and xiBound
	double distanceBound = xiBound - xi[iPath_Bound_j];

	//std::cout << "distanceBound: " << distanceBound << std::endl;
	// calculate the coordinates of xiBound on the inner bound
	// (vector equation: pointInnerBound = vertex(Bound_j) + lamda * vector)
	vertice vectorInnerBound;
	vectorInnerBound.x = innerBound.vertices[iInnerBound_Bound_jplus1].x - innerBound.vertices[iInnerBound_Bound_j].x;
	vectorInnerBound.y = innerBound.vertices[iInnerBound_Bound_jplus1].y - innerBound.vertices[iInnerBound_Bound_j].y;
	double lamdaInnerBound;
	if (std::pow(std::pow(vectorInnerBound.x, 2.0) + std::pow(vectorInnerBound.y, 2.0), 0.5) != 0)
	{
	    lamdaInnerBound = distanceBound / (std::pow(std::pow(vectorInnerBound.x, 2.0) + std::pow(vectorInnerBound.y, 2.0), 0.5));
	}
	else
	{
	    lamdaInnerBound = 0;
	}
	//std::cout << "lamdaInnerBound: " << lamdaInnerBound << std::endl;
	vertice pXiBound;
	pXiBound.x = innerBound.vertices[iInnerBound_Bound_j].x + lamdaInnerBound * vectorInnerBound.x;
	pXiBound.y = innerBound.vertices[iInnerBound_Bound_j].y + lamdaInnerBound * vectorInnerBound.y;
	//std::cout << "pXiBound: " << pXiBound.x << std::endl;
	// construct tangent at pXiBound
	vertice t;
	t.x = innerBound.vertices[iInnerBound_Bound_jplus1].x - innerBound.vertices[iInnerBound_Bound_j].x;
	t.y = innerBound.vertices[iInnerBound_Bound_jplus1].y - innerBound.vertices[iInnerBound_Bound_j].y;

	// construct normal vector to tangent at pXiBound
	vertice n;
	n.x = -t.y;
	n.y = t.x;

	// find vertex on outer bound, such that mu_j is the first
	// vertice in front of n in driving direction
	vertice mu_j;
	vertice temp;
	size_t iOuterBound_Bound_jplus1;
	//std::cout << "iInnerBound_Bound_j: " << iInnerBound_Bound_j << std::endl;
	for (iOuterBound_Bound_jplus1 = iInnerBound_Bound_j; iOuterBound_Bound_jplus1 < outerBound.vertices.size(); iOuterBound_Bound_jplus1++)
	{
	    mu_j = outerBound.vertices[iOuterBound_Bound_jplus1];
	    // due to the specific orientation of n, mu_j is always on
	    // the right of n to be in front in driving direction
	    temp.x = pXiBound.x + n.x;
	    temp.y = pXiBound.y + n.y;
	    if (!(isLeft(pXiBound, temp, mu_j)))
	    {
	        break;
	    }
	}
	//std::cout << "iOuterBound_Bound_jplus1: " << iOuterBound_Bound_jplus1 << std::endl;
	size_t iOuterBound_Bound_j = getNextIndex(outerBound.vertices, iOuterBound_Bound_jplus1, "backward", outerBound.distances);
	//std::cout << "iOuterBound_Bound_j: " << iOuterBound_Bound_j << std::endl;

	// calculate projection of pXiBound on outer bound
	// (pXiOuterBound = pXiBound + alpha*n, where alpha is found by intersection
	// with outer bound)
	vertice temp2;
	temp2.x = outerBound.vertices[iOuterBound_Bound_jplus1].x - outerBound.vertices[iOuterBound_Bound_j].x;
	temp2.y = outerBound.vertices[iOuterBound_Bound_jplus1].y - outerBound.vertices[iOuterBound_Bound_j].y;
	double alpha = calcVectorIntersectionPointAlpha(pXiBound, n, outerBound.vertices[iOuterBound_Bound_j], temp2);
	//std::cout << "alpha: " << alpha << std::endl;
	//std::cout << "alpha: " << n.x << std::endl;
	vertice pXiOuterBound;
	pXiOuterBound.x = pXiBound.x + alpha * n.x;
	pXiOuterBound.y = pXiBound.y + alpha * n.y;
	//std::cout << "pXiOuterBound:" << pXiOuterBound.x << std::endl;
	// save the the points of the bound and the index of the lane border at
	// which the occupancy will start or end

	std::vector <vertice> bound;
	if (typeOfBound == "front")
	{
	        // the front bound is from the right to the left border at pBound
	        // and continues at index (j+1) on the border
	        if (shPath.side[iPath_Bound_j]) // i.e. left
			{
	            bound.push_back(pXiOuterBound);
	            bound.push_back(pXiBound);
	            *iLeftBorder_Bound = iInnerBound_Bound_jplus1;
	            *iRightBorder_Bound = iOuterBound_Bound_jplus1;
			}
	        else // i.e. right
	        {
	            bound.push_back(pXiBound);
	            bound.push_back(pXiOuterBound);
	            *iLeftBorder_Bound = iOuterBound_Bound_jplus1;
	            *iRightBorder_Bound = iInnerBound_Bound_jplus1;
	        }
	}
	else if (typeOfBound == "final")
	{
	        // the final bound is from the left to the right border at pBound
	        // and runs until index j on the border
	        if (shPath.side[iPath_Bound_j]) // i.e. left
	        {
	            bound.push_back(pXiBound);
	            bound.push_back(pXiOuterBound);
	            *iLeftBorder_Bound = iInnerBound_Bound_j;
	            *iRightBorder_Bound = iOuterBound_Bound_j;
	        }
	        else // i.e. right
	        {
	            bound.push_back(pXiOuterBound);
	            bound.push_back(pXiBound);
	            *iLeftBorder_Bound = iOuterBound_Bound_j;
	            *iRightBorder_Bound = iInnerBound_Bound_j;
	        }
	}
	//std::cout << "*iLeftBorder_Bound: " << *iLeftBorder_Bound << std::endl;
	//std::cout << "*iRightBorder_Bound: " << *iRightBorder_Bound << std::endl;

	/*
	// DEGUB:
	// check the actual xi of the bound, whether the bound over-approximates the
	// reach xiBound
	vertice pBoundLeft, pBoundRight;
	double xiBoundLeft = 0;
	double xiBoundRight = 0;
	iInnerBound_Bound_j = shPath.indexBorder[iPath_Bound_j];
	if (shPath.side[iPath_Bound_j]) // i.e. left
	{
	    pBoundLeft = pXiBound;
	    pBoundRight = pXiOuterBound;
	}
	else // i.e. right
	{
	    pBoundLeft = pXiOuterBound;
	    pBoundRight = pXiBound;
	}
	if (distanceBound == 1)
	{
	    xiBoundLeft = norm(pBoundLeft - leftBorder.vertices(:,iInnerBound_Bound_j)) + xi(iPath_Bound_j);
	    xiBoundRight = norm(pBoundRight - rightBorder.vertices(:,iInnerBound_Bound_j)) + xi(iPath_Bound_j);
	}

	if (typeOfBound == "front")
	{
	        // the computed front bound must not be further away than the
	        // closest reach
	        if ((xiBoundLeft - xiBound) > 10e-9 && (xiBoundRight - xiBound) > 10e-9)
	        {

	        }
	    case 'final'
	        % the computed final bound must not be closer than the furthest
	        % reach
	        if (xiBoundLeft - xiBound) < -10e-9  && (xiBoundRight - xiBound) < -10e-9
	            warning('Final bound is closer than xiFurthest, i.e. no over-approximation.')
	        end
	}
	*/

	return bound;
}


