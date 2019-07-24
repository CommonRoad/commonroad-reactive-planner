/*
 * functions on lanelets
 */

#ifndef HEADER_LANELET_OPERATIONS
#define HEADER_LANELET_OPERATIONS

#include "vehicularLanelet.h"
#include <iostream>

//creates all lanelets objects from the XML input
std::vector <vehicularLanelet> createLaneletFromXML(std::string xmlFile);

// find the lanelets which have no predecessor
std::vector <vehicularLanelet*> noPredecessorLanelets(std::vector <vehicularLanelet>* lanelets);

// returns all successor lanelets of a lanelet
std::vector <vehicularLanelet*> findAllSuccessorLanelets(vehicularLanelet* lanelets);

// get specific lanelet
//lanelet* specificLanelet(std::list <lanelet*> lanelets, size_t id);


#endif
