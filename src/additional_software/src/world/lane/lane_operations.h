/*
 * functions on lanes
 */

#ifndef HEADER_LANE_OPERATIONS
#define HEADER_LANE_OPERATIONS

#include "lane.h"
#include <iostream>

//creates lane objects from lanelet input
void createLanesFromLanelets(std::vector <lane>* lanes, std::vector <vehicularLanelet>* lanelets);

// get specific lane from list
//lane* getSpecificLane(std::vector <lane*> lanes, size_t id);

// builds a struct with lane properties of the current lanelet and all longitudinally adjacent lanelets
void combineLaneletAndSuccessors(std::vector <lane>* laneList, vehicularLanelet* curLanelet, size_t k);

/*
 * search through all obj to find the lane(s) which
 * contain(s) the lanelet
 */
std::vector <lane*> findLaneByLanelet(std::vector <lane>* laneList, size_t laneletId);

/*
 * find lane by xy-position. needed for inLane of dynamic obstacles
 */
std::vector <lane*> findLaneByPosition(std::vector <lane>* lanes, double xPos, double yPos);

/*
 * search all Lane objects to return the lanes which
 * can be reached by the obstacle according to the adjacency graph and contraint5
 */
std::vector <lane*> findReachableLanes(std::vector <lane>* lanes, bool constraint5, std::vector <vehicularLanelet*> lanelets, size_t inLaneSize, std::string side = "");

/*
 * flip a lane including all its properties
 */
void flipLane(lane* aLane);

/*
 * find the shortest possible path through the lane (network)
 * by always following the inner bound (i.e. corresponding path, Definition 8)
 */
shortestPath findShortestPath(border leftBorder, border rightBorder);

/*
 * keep following the inner bound of the lane until the next
 * inflection point and compute the path variable xi of the shortest path
 */
shortestPath followBound(size_t iStart, shortestPath shortestPath, border innerBound, border outerBound);

#endif
