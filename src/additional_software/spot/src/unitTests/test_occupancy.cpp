/*
 * test_shortestPath.cpp
 *
 *  Created on: 18.05.2018
 *      Author: Sebastian
 */

#define TESTOCCUPANCY 1
#include "catch.hpp"
#include "../prediction/occupancyCalculation.h"
#include "../world/lane/lane_operations.h"
#include "../world/lanelets/lanelet_operations.h"
#include "../inputs/readXMLfile.h"
#include "../world/obstacle/obstacle_operations.h"
#include "../prediction/occupancyCalculation.h"

#if TESTOCCUPANCY==1


/*
TEST_CASE( "velocityProfile_Ffb-1b", "[velocityProfile_Ffb-1b]")
{
	double v_max = 50;
	size_t t = 0;
	size_t dt = 0.1;



	shortestPath shPath;

	shPath.curvature.push_back(0.000021879326678);
	shPath.curvature.push_back(-0.000407675709607);
	shPath.curvature.push_back(-0.000307285912070);
	shPath.curvature.push_back(-0.000117764195241);
	shPath.curvature.push_back(-0.000207466904522);
	shPath.curvature.push_back(-0.002758429914321);
	shPath.curvature.push_back(-0.565923940353998);
	shPath.curvature.push_back(0);
	shPath.curvature.push_back(0);
	shPath.curvature.push_back(-0.014717917295108);
	shPath.curvature.push_back(-4.060203617859635);
	shPath.curvature.push_back(-0.007140085289367);
	shPath.curvature.push_back(-0.017114277033397);
	shPath.curvature.push_back(-0.036825810821846);
	shPath.curvature.push_back(-0.478670502735990);
	shPath.curvature.push_back(-0.010436110679931);
	shPath.curvature.push_back(-0.019680891445459);
	shPath.curvature.push_back(-0.003550407714733);
	shPath.curvature.push_back(-0.001422272719613);
	shPath.curvature.push_back(-0.002342600744393);
	shPath.curvature.push_back(-0.001632954309647);
	shPath.curvature.push_back(-0.000437821139740);
	shPath.curvature.push_back(0.000417947083341);
	shPath.curvature.push_back(0.000475393790670);


	shPath.xi.push_back(0);
	shPath.xi.push_back(10.9002992349325);
	shPath.xi.push_back(26.9107827257581);
	shPath.xi.push_back(60.5693990660060);
	shPath.xi.push_back(87.3790120525823);
	shPath.xi.push_back(109.7522276216506);
	shPath.xi.push_back(112.9278936305375);
	shPath.xi.push_back(116.1730173770224);
	shPath.xi.push_back(118.6815582776258);
	shPath.xi.push_back(120.5488421687360);
	shPath.xi.push_back(120.6513901575023);
	shPath.xi.push_back(120.8894631295088);
	shPath.xi.push_back(122.4969869924732);
	shPath.xi.push_back(123.2730196850835);
	shPath.xi.push_back(124.6316213780016);
	shPath.xi.push_back(125.5280111648689);
	shPath.xi.push_back(133.6454600843263);
	shPath.xi.push_back(133.8655399107530);
	shPath.xi.push_back(157.2454423239368);
	shPath.xi.push_back(182.2225452370771);
	shPath.xi.push_back(201.2462298197651);
	shPath.xi.push_back(221.7395539761825);
	shPath.xi.push_back(245.2911778004494);
	shPath.xi.push_back(267.7474634670507);

	double initialVelocity = 20;

	std::vector <double> velProf = calculateOptimalVelocityProfile(shPath, initialVelocity, 5, v_max);

	size_t i;
	for(i = 0; i < velProf.size(); i++)
	{
		//std::cout << velProf[i] << std::endl;
	}
}

TEST_CASE( "velocityProfile", "[velocityProfile]" )
{
	std::cout << "occupancy unittests executed" << std::endl;



	size_t t = 0;
	size_t dt = 0.1;

	shortestPath shPath;

	shPath.curvature.push_back(-0.000000350554884);
	shPath.curvature.push_back(-0.000000030256674);
	shPath.curvature.push_back(-0.000000069403384);
	shPath.curvature.push_back(-0.000000069404407);
	shPath.curvature.push_back(-0.000000319867630);
	shPath.curvature.push_back(0.000000250464264);
	shPath.curvature.push_back(-0.000000000000000);
	shPath.curvature.push_back(0.000018217739476);
	shPath.curvature.push_back(0.000176732080459);
	shPath.curvature.push_back(0.002924137612541);
	shPath.curvature.push_back(4.243002125512458);
	shPath.curvature.push_back(0.169794902642633);
	shPath.curvature.push_back(0.287771029343633);
	shPath.curvature.push_back(0.519488643322571);
	shPath.curvature.push_back(0.426970321121299);
	shPath.curvature.push_back(0.307423730872581);
	shPath.curvature.push_back(0.238481572249910);
	shPath.curvature.push_back(0.294065600301080);
	shPath.curvature.push_back(0.293703901740401);
	shPath.curvature.push_back(0.293418041271674);
	shPath.curvature.push_back(0.183468362402088);
	shPath.curvature.push_back(2.872522733147267);
	shPath.curvature.push_back(0.000258264123726);
	shPath.curvature.push_back(0.000019861836224);
	shPath.curvature.push_back(-0.000000022143677);
	shPath.curvature.push_back(0.000000180025031);
	shPath.curvature.push_back(-0.000000223485011);
	shPath.curvature.push_back(-0.000000285006082);
	shPath.curvature.push_back(-0.000000066712962);
	shPath.curvature.push_back(-0.000000351325918);


	shPath.xi.push_back(0);
	shPath.xi.push_back(9.8026779086125);
	shPath.xi.push_back(19.6103837352309);
	shPath.xi.push_back(29.4180344860479);
	shPath.xi.push_back(39.2257487886987);

	shPath.xi.push_back(49.0333774547121);
	shPath.xi.push_back(58.8410282055290);
	shPath.xi.push_back(68.6486789563460);
	shPath.xi.push_back(78.4563297071629);
	shPath.xi.push_back(87.9118023384173);

	shPath.xi.push_back(88.2174440018076);
	shPath.xi.push_back(88.6426454246650);
	shPath.xi.push_back(89.0732074991677);
	shPath.xi.push_back(89.2787111001093);
	shPath.xi.push_back(89.5095009179090);
	shPath.xi.push_back(89.7352930789775);
	shPath.xi.push_back(90.1920375261249);
	shPath.xi.push_back(90.6489210780285);
	shPath.xi.push_back(91.1059046663553);
	shPath.xi.push_back(91.5631514706087);
	shPath.xi.push_back(92.0205326464347);
	shPath.xi.push_back(92.4780749942016);
	shPath.xi.push_back(95.4827136746422);
	shPath.xi.push_back(98.4879041527983);
	shPath.xi.push_back(111.3936907404050);
	shPath.xi.push_back(121.7788147035554);
	shPath.xi.push_back(132.1640350806105);
    shPath.xi.push_back(142.5492554576656);
	shPath.xi.push_back(152.9344354045465);
	shPath.xi.push_back(163.3246552058789);

	double initialVelocity = 13.8889;
	double v_max_M2 = 16.668;

	std::vector <double> velProf = calculateOptimalVelocityProfile(shPath, initialVelocity, v_max_M2, v_max_M2);


	size_t i;
	for(i = 0; i < velProf.size(); i++)
	{
		std::cout << velProf[i] << std::endl;
	}

}
*/


TEST_CASE( "clostestLongitudinalReach", "[clostestLongitudinalReach]" )
{

	double a_max = 8;
	double v0 = 20;
	double t = 4;

	// backward-driving not allowed (constrain3 = true)
	double result = closestLongitudinalReach(t, v0, a_max, true);
	REQUIRE( result == 25.0 );

	// backward-driving allowed (constrain3 = false)
	result = closestLongitudinalReach(t, v0, a_max, false);
	REQUIRE( result == 16.0 );

}

TEST_CASE( "M1_Occupancy", "[M1_Occupancy]")
{
	std::cout << "occupancy unittests executed" << std::endl;

	std::vector <vehicularLanelet> lanelets = createLaneletFromXML("unittest_data/scenario_minimalExample.xml");
	std::vector <lane> lanes;
	createLanesFromLanelets(&lanes, &lanelets);
	std::vector <obstacle*> obstacles;
	obstacles = createObstacleFromXML("unittest_data/scenario_minimalExample.xml", &obstacles, 0.0, false, &lanes);

	double epsilon = 0.000001;

	std::vector <std::vector <vertice> > verticesM1;
	std::vector <vertice> temp;
	vertice v;

	v.x = -12.1;
	v.y = 0.95;
	temp.push_back(v);
	v.x = -10.602133333333333;
	v.y = 0.990000000000000;
	temp.push_back(v);
	v.x = -6.359999999999999;
	v.y = 0.990000000000000;
	temp.push_back(v);
	v.x = -6.359999999999999;
	v.y = -0.990000000000000;
	temp.push_back(v);
	v.x = -10.602133333333333;
	v.y = -0.990000000000000;
	temp.push_back(v);
	v.x = -12.100000000000000;
	v.y = -0.950000000000000;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	v.x = -10.640000000000001;
	v.y = 0.990000000000000;
	temp.push_back(v);
	v.x = -9.117066666666666;
	v.y = 1.110000000000000;
	temp.push_back(v);
	v.x = -4.740000000000000;
	v.y = 1.110000000000000;
	temp.push_back(v);
	v.x = -4.740000000000000;
	v.y = -1.110000000000000;
	temp.push_back(v);
	v.x = -9.117066666666666;
	v.y = -1.110000000000000;
	temp.push_back(v);
	v.x = -10.640000000000001;
	v.y = -0.990000000000000;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	v.x = -9.260000000000000;
	v.y = 1.110000000000000;
	temp.push_back(v);
	v.x = -7.657599999999999;
	v.y = 1.310000000000000;
	temp.push_back(v);
	v.x = -3.039999999999999;
	v.y = 1.310000000000000;
	temp.push_back(v);
	v.x = -3.039999999999999;
	v.y = -1.310000000000000;
	temp.push_back(v);
	v.x = -7.657599999999999;
	v.y = -1.310000000000000;
	temp.push_back(v);
	v.x = -9.260000000000000;
	v.y = -1.110000000000000;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	v.x = -7.959999999999999;
	v.y = 1.310000000000000;
	temp.push_back(v);
	v.x = -6.236533333333334;
	v.y = 1.590000000000000;
	temp.push_back(v);
	v.x = -1.260000000000000;
	v.y = 1.590000000000000;
	temp.push_back(v);
	v.x = -1.260000000000000;
	v.y = -1.590000000000000;
	temp.push_back(v);
	v.x = -6.236533333333334;
	v.y = -1.590000000000000;
	temp.push_back(v);
	v.x = -7.959999999999999;
	v.y = -1.310000000000000;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	v.x = -6.740000000000000;
	v.y = 1.590000000000000;
	temp.push_back(v);
	v.x = -4.866666666666667;
	v.y = 1.950000000000000;
	temp.push_back(v);
	v.x = 0.600000000000000;
	v.y = 1.950000000000000;
	temp.push_back(v);
	v.x = 0.600000000000000;
	v.y = -1.950000000000000;
	temp.push_back(v);
	v.x = -4.866666666666667;
	v.y = -1.950000000000000;
	temp.push_back(v);
	v.x = -6.740000000000000;
	v.y = -1.590000000000000;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	v.x = -5.600000000000000;
	v.y = 1.950000000000000;
	temp.push_back(v);
	v.x = -3.560800000000000;
	v.y =  2.390000000000000;
	temp.push_back(v);
	v.x = 2.539999999999999;
	v.y = 2.390000000000000;
	temp.push_back(v);
	v.x = 2.539999999999999;
	v.y = -2.390000000000000;
	temp.push_back(v);
	v.x = -3.560800000000000;
	v.y = -2.390000000000000;
	temp.push_back(v);
	v.x = -5.600000000000000;
	v.y = -1.950000000000000;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	v.x = -4.539999999999999;
	v.y = 2.390000000000001;
	temp.push_back(v);
	v.x = -2.331733333333331;
	v.y = 2.910000000000000;
	temp.push_back(v);
	v.x = 4.560000000000002;
	v.y = 2.910000000000000;
	temp.push_back(v);
	v.x = 4.560000000000002;
	v.y = -2.910000000000000;
	temp.push_back(v);
	v.x = -2.331733333333331;
	v.y = -2.910000000000000;
	temp.push_back(v);
	v.x = -4.539999999999999;
	v.y = -2.390000000000001;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	v.x = -3.559999999999999;
	v.y = 2.910000000000000;
	temp.push_back(v);
	v.x = -1.192266666666667;
	v.y = 3.510000000000001;
	temp.push_back(v);
	v.x = 6.660000000000000;
	v.y = 3.510000000000001;
	temp.push_back(v);
	v.x = 6.660000000000000;
	v.y = -3.510000000000001;
	temp.push_back(v);
	v.x = -1.192266666666667;
	v.y = -3.510000000000001;
	temp.push_back(v);
	v.x = -3.559999999999999;
	v.y = -2.910000000000000;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	v.x = -2.660000000000000;
	v.y = 3.510000000000001;
	temp.push_back(v);
	v.x = -0.155199999999999;
	v.y = 4.190000000000000;
	temp.push_back(v);
	v.x = 8.840000000000003;
	v.y = 4.190000000000000;
	temp.push_back(v);
	v.x = 8.840000000000003;
	v.y = -4.190000000000000;
	temp.push_back(v);
	v.x = -0.155199999999999;
	v.y = -4.190000000000000;
	temp.push_back(v);
	v.x = -2.660000000000000;
	v.y = -3.510000000000001;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	v.x = -1.840000000000000;
	v.y = 4.190000000000000;
	temp.push_back(v);
	v.x = 0.766666666666667;
	v.y = 4.950000000000000;
	temp.push_back(v);
	v.x = 11.100000000000001;
	v.y = 4.950000000000000;
	temp.push_back(v);
	v.x = 11.100000000000001;
	v.y = -4.950000000000000;
	temp.push_back(v);
	v.x = 0.766666666666667;
	v.y = -4.950000000000000;
	temp.push_back(v);
	v.x = -1.840000000000000;
	v.y = -4.190000000000000;
	temp.push_back(v);
	verticesM1.push_back(temp);
	temp.clear();

	timeStruct timeInterval;
	timeInterval.startingTime = 0;
	timeInterval.timeStep = 0.1;
	timeInterval.ending = 1;

	std::vector <std::vector <vertice> > result = obstacles[1]->M1_accelerationBasedOccupancy(timeInterval);

	REQUIRE( result.size() == verticesM1.size() );

	for (size_t i = 0; i < verticesM1.size(); i++)
	{
		REQUIRE( result[i].size() == verticesM1[i].size() );
		for(size_t j = 0; j < verticesM1[i].size(); j++)
		{
			REQUIRE ( std::abs(verticesM1[i][j].x - result[i][j].x) < epsilon );
			REQUIRE ( std::abs(verticesM1[i][j].y - result[i][j].y) < epsilon );
		}
	}
}

#endif
