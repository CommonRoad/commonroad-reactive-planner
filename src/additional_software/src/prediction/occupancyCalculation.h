/*
 * predictions for occupancy
 */

#include "../world/obstacle/obstacle.h"

#ifndef HEADER_OCCUPANCYCALCULATIONS
#define HEADER_OCCUPANCYCALCULATIONS

/*
 * predicts the occupancy of the obstacle on the
 * given map for the given time interval
 */
//void computeOccupancyCore(obstacle* obst, std::vector <lane>* lanes, timeStruct timeInterval);

// constraint management for the obstacles
//void manageConstraints(obstacle* obst);

/*
 * M_1: compute the acceleration-based occupancy for each time interval
 * (maximum possible occupancy with maximum longitudinally and laterally
 * acceleration, i.e. "occupancy towards the road boundaries")
 */
//std::vector <vertice> M1_accelerationBasedOccupancy(obstacle* obst, timeStruct timeInterval);


double closestLongitudinalReach(double t, double v0, double a_max, bool constraint3);

/*
 * find all Lane objects of the map which can be
 * reached by the obstacle according to the adjacency graph and contraint5
 */
std::vector <lane*> findAllReachableLanes(std::vector <lane>* lanes, bool constraint5, std::vector <lane*> inLane);

/*
 * compute the occupancy ploygon for one time step in
 * local coordinates (acceleration-based occupancy; abstraction M_1)
 * Note that constraint3 (i.e. driving backwards in a lane is not allowed)
 * is not considered, since this depends on the geometry of the lane
 */
std::vector <vertice> accelerationOccupancyLocal(double vx, double a_max, double tk, double tkplus1);

/*
 * compute the lane-following occupancy for each
 * time step, taking into account only the full acceleration and full deceleration
 * (shortest path through a lane; abstraction M_2)
 */
//std::vector <vertice> M2_laneFollowingOccupancy(obstacle* obst, lane* curLane, timeStruct timeInterval);

/*
 * compute the shortest possible travelled
 * distance in time t
 */
/*
 * compute the furthest possible travelled
 * along the path in time t
 */
double furthestLongitudinalReach_old(double t, double v0, double v_s, double v_max, double a_max);

/*
 * compute furthest reach with respect to curvature of the path
 */
double furthestLongitudinalReach(double t, double v0, double v_s, double v_max, double a_max, size_t index, lane* curLane);

//compute the segmentation through the lane along the shortest path for one time step
std::vector <vertice> inflectionPointSegmentation(size_t iPath_Obstacle, double xiObstacle, double xiClosest,
		double xiFurthest, shortestPath shPath, border leftBorder, border rightBorder);


/*
 * construct the bound of the inflection point segmentation
 * along the shortest path, such that the reachable xiBound is enclosed
 */
std::vector <vertice> constructBound(size_t iPath_Obstacle, double xiBound, std::string typeOfBound, std::vector <double> xi, shortestPath shPath,
						border leftBorder, border rightBorder, size_t* iLeftBorder_Bound, size_t* iRightBorder_Bound);

#endif
