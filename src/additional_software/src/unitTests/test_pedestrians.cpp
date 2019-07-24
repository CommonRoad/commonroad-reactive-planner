/*
 * test_pedestrians.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: sebastian
 */

#define TESTPEDESTRIANS 0
#include "catch.hpp"
#include "../world/obstacle/pedestrian.h"
#include "../geometry/circle.h"


#if TESTPEDESTRIANS==1

/*
 * the id of a lane is statically distributed, therefore other attributes should be used for the unittest.
 */
TEST_CASE( "Pedestrian", "[pedestrian]" )
{
	std::cout << "pedestrian unittests executed" << std::endl;

	pedestrian pedo_obj;
	pedo_obj.setId(123);
	pedo_obj.setPosition(2, 3);
	pedo_obj.setPedestrianOrientation({0.3, 0.4, 0.6});
	circle* circle_obj = new circle();
	circle_obj->setRadius(0.1);
	pedo_obj.setShape(circle_obj);
	pedo_obj.setPedestrianVelocity({1});
	pedo_obj.setAcceleration(0.2);

	std::vector <lane> lanes;
	std::vector <vehicularLanelet> lanelets;

	timeStruct timeInterval{0, 0.1, 1};

	pedo_obj.computeOccupancyCore(&lanelets, &lanes, timeInterval);



	REQUIRE(1);
}


#endif

