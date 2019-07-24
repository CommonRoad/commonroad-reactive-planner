/*
 * test_reachableLanes.cpp
 *
 *  Created on: 21.03.2018
 *      Author: Sebastian
 */

#define TESTREACHABLELANES 1
#include "catch.hpp"
#include "../world/lane/lane_operations.h"
#include "../world/lanelets/lanelet_operations.h"
#include "../inputs/readXMLfile.h"
#include "../world/obstacle/obstacle_operations.h"
#include "../prediction/occupancyCalculation.h"

#if TESTREACHABLELANES==1

/*
 * the id of a lane is statically distributed, therefore other attributes should be used for the unittest.
 */
TEST_CASE( "Ger_Muc_1a_reachableLanes", "[reachableLanes]" )
{
	std::cout << "reachable lanes unittests executed" << std::endl;

	std::vector <vehicularLanelet> lanelets = createLaneletFromXML("unittest_data/GER_Muc_1a_lanelets.xml");
	std::vector <lane> lanes;
	createLanesFromLanelets(&lanes, &lanelets);
	std::vector <obstacle*> obstacles;
	obstacles = createObstacleFromXML("unittest_data/GER_Muc_1a_lanelets.xml", &obstacles, 0.0, false, &lanes);
	std::vector <lane*> reachableLanes;
	REQUIRE( obstacles.size() == 1);
	REQUIRE( obstacles.front()->getInLane().size() != 0);
	REQUIRE( obstacles.front()->getId() == 102 );
	//constraint5 == true
	reachableLanes = findAllReachableLanes(&lanes, true, obstacles.front()->getInLane());
	REQUIRE( reachableLanes.size() == 3);
	REQUIRE( reachableLanes[0]->getAssLanelets().size() == 4);
	REQUIRE( reachableLanes[0]->getCenterVertices().size() == 92);
	REQUIRE( reachableLanes[1]->getAssLanelets().size() == 6);
	REQUIRE( reachableLanes[1]->getCenterVertices().size() == 105);
	REQUIRE( reachableLanes[2]->getAssLanelets().size() == 4);
	REQUIRE( reachableLanes[2]->getCenterVertices().size() == 104);
	while(!(obstacles.empty()))
	{
		obstacle* temp = obstacles.back();
		obstacles.pop_back();
		delete temp;
		temp = 0;
	}
}


#endif

