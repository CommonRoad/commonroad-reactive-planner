/*
 * test_shortestPath.cpp
 *
 *  Created on: 21.03.2018
 *      Author: Sebastian
 */

#define TESTSHORTESTPATH 1
#include "catch.hpp"
#include "../world/lane/lane_operations.h"
#include "../world/lanelets/lanelet_operations.h"
#include "../inputs/readXMLfile.h"
#include "../world/obstacle/obstacle_operations.h"
#include "../prediction/occupancyCalculation.h"

#if TESTSHORTESTPATH==1

TEST_CASE( "Ger_Muc_1a_shortestPath", "[shortestPath]" )
{
	std::cout << "shortest path unittests executed" << std::endl;
	double epsilon = 0.00000001;

	std::vector <vehicularLanelet> lanelets = createLaneletFromXML("unittest_data/GER_Muc_1a_lanelets.xml");
	std::vector <lane> lanes;
	createLanesFromLanelets(&lanes, &lanelets);

	size_t i;
	for (i = 0; i < lanes.size(); i++)
	{
		lanes[i].calculateShortestPath();
	}

	shortestPath shPath[lanes.size()];
	for (i = 0; i < lanes.size(); i++)
	{
		shPath[i] = lanes[i].getShortestPath();
	}

	// test side of shortestPath according to the lane
	REQUIRE( shPath[0].side.size() == 44);
	REQUIRE( shPath[0].side[0] == 1);
	REQUIRE( shPath[0].side[1] == 0);
	REQUIRE( shPath[0].side[2] == 0);
	REQUIRE( shPath[0].side[3] == 0);
	REQUIRE( shPath[0].side[4] == 0);
	REQUIRE( shPath[0].side[5] == 1);
	REQUIRE( shPath[0].side[6] == 1);
	REQUIRE( shPath[0].side[7] == 1);
	REQUIRE( shPath[0].side[8] == 1);
	REQUIRE( shPath[0].side[9] == 0);
	REQUIRE( shPath[0].side[10] == 0);
	REQUIRE( shPath[0].side[11] == 0);
	REQUIRE( shPath[0].side[12] == 0);
	REQUIRE( shPath[0].side[13] == 0);
	REQUIRE( shPath[0].side[14] == 0);
	REQUIRE( shPath[0].side[15] == 0);
	REQUIRE( shPath[0].side[16] == 0);
	REQUIRE( shPath[0].side[17] == 0);
	REQUIRE( shPath[0].side[18] == 0);
	REQUIRE( shPath[0].side[19] == 0);
	REQUIRE( shPath[0].side[20] == 0);
	REQUIRE( shPath[0].side[21] == 0);
	REQUIRE( shPath[0].side[22] == 0);
	REQUIRE( shPath[0].side[23] == 0);
	REQUIRE( shPath[0].side[24] == 0);
	REQUIRE( shPath[0].side[25] == 0);
	REQUIRE( shPath[0].side[26] == 0);
	REQUIRE( shPath[0].side[27] == 0);
	REQUIRE( shPath[0].side[28] == 0);
	REQUIRE( shPath[0].side[29] == 0);
	REQUIRE( shPath[0].side[30] == 0);
	REQUIRE( shPath[0].side[31] == 0);
	REQUIRE( shPath[0].side[32] == 0);
	REQUIRE( shPath[0].side[33] == 0);
	REQUIRE( shPath[0].side[34] == 0);
	REQUIRE( shPath[0].side[35] == 0);
	REQUIRE( shPath[0].side[36] == 0);
	REQUIRE( shPath[0].side[37] == 1);
	REQUIRE( shPath[0].side[38] == 1);
	REQUIRE( shPath[0].side[39] == 1);
	REQUIRE( shPath[0].side[40] == 1);
	REQUIRE( shPath[0].side[41] == 1);
	REQUIRE( shPath[0].side[42] == 1);
	REQUIRE( shPath[0].side[43] == 0);

	// test path variable(xi)
	REQUIRE( shPath[0].xi.size() == 44);
	REQUIRE( std::abs(shPath[0].xi[0] - 0) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[1] - 19.3699129557140) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[2] - 19.3947325044609) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[3] - 43.5521164499547) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[4] - 43.6029232408394) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[5] - 47.5816136355303) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[6] - 47.9046108491406) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[7] - 53.2077304084189) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[8] - 53.3770122444203) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[9] - 57.0731792250406) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[10] - 57.1626363097237) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[11] - 60.2549571679142) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[12] - 69.5120522333221) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[13] - 72.5690195573215) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[14] - 79.3935791182016) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[15] - 80.0838227346154) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[16] - 83.9851030078078) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[17] - 88.8130295824359) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[18] - 92.7442963630042) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[19] - 96.0261143774590) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[20] - 98.0629816387207) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[21] - 101.7401092582085) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[22] - 106.8520399873949) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[23] - 108.7780767679368) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[24] - 122.3276049907200) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[25] - 127.6006212907138) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[26] - 130.7594418696641) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[27] - 132.5962638779164) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[28] - 135.0048312031120) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[29] - 142.7137562727959) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[30] - 143.0715947838592) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[31] - 149.8917906204903) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[32] - 150.0761608187328) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[33] - 156.1737526589596) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[34] - 156.3429933455571) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[35] - 168.3508533755541) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[36] - 177.7244978775608) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[37] - 312.8546281379797) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[38] - 313.5971884250945) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[39] - 329.1273802581016) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[40] - 333.4210495245915) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[41] - 336.4012187899186) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[42] - 336.4846744667776) < epsilon);
	REQUIRE( std::abs(shPath[0].xi[43] - 343.1128880662582) < epsilon);

	// test curvature of shortest path
	REQUIRE( shPath[0].curvature.size() == 44);
	REQUIRE( std::abs(shPath[0].curvature[0] - 0.000000003162144) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[1] - -0.001111731711084) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[2] - -0.000584905604726) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[3] - -0.000678399566717) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[4] - -0.145267005489496) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[5] - 0.014415717811818) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[6] - 0.006445618983850) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[7] - -0.004404691264026) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[8] - -0.014928946689036) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[9] - -0.006771476554133) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[10] - -0.024181406771421) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[11] - -0.000543459528636) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[12] - -0.004944897256498) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[13] - -0.012785875295534) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[14] - -0.022209306934392) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[15] - -0.080664665075640) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[16] - -0.011831636345593) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[17] - -0.012676456477920) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[18] - -0.010123419772368) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[19] - -0.016679086794362) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[20] - -0.035473960736054) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[21] - -0.014656800111132) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[22] - -0.024998812779960) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[23] - -0.004886421793409) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[24] - -0.010211382084035) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[25] - -0.144425963577565) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[26] - -0.214087847122532) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[27] - -0.340754584780146) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[28] - -0.024445741767365) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[29] - -0.018999634962457) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[30] - -0.009053257824671) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[31] - -0.026069294030612) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[32] - -0.036937858938343) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[33] - -0.026717501619251) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[34] - -0.003459961447722) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[35] - 0.000037919557151) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[36] - 0.000072273969121) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[37] - 0.002917374334889) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[38] - 0.003602186037846) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[39] - 0.004036376097891) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[40] - 0.050238731149752) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[41] - 0.063996761488612) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[42] - 0.004503770785417) < epsilon);
	REQUIRE( std::abs(shPath[0].curvature[43] - -0.000035360849248) < epsilon);

	// test indexBorder of shortestPath
	REQUIRE( shPath[0].indexBorder.size() == 44);
	REQUIRE( shPath[0].indexBorder[0] == 0);
	REQUIRE( shPath[0].indexBorder[1] == 1);
	REQUIRE( shPath[0].indexBorder[2] == 2);
	REQUIRE( shPath[0].indexBorder[3] == 3);
	REQUIRE( shPath[0].indexBorder[4] == 4);
	REQUIRE( shPath[0].indexBorder[5] == 5);
	REQUIRE( shPath[0].indexBorder[6] == 6);
	REQUIRE( shPath[0].indexBorder[7] == 7);
	REQUIRE( shPath[0].indexBorder[8] == 8);
	REQUIRE( shPath[0].indexBorder[9] == 9);
	REQUIRE( shPath[0].indexBorder[10] == 10);
	REQUIRE( shPath[0].indexBorder[11] == 11);
	REQUIRE( shPath[0].indexBorder[12] == 12);
	REQUIRE( shPath[0].indexBorder[13] == 13);
	REQUIRE( shPath[0].indexBorder[14] == 14);
	REQUIRE( shPath[0].indexBorder[15] == 15);
	REQUIRE( shPath[0].indexBorder[16] == 16);
	REQUIRE( shPath[0].indexBorder[17] == 17);
	REQUIRE( shPath[0].indexBorder[18] == 18);
	REQUIRE( shPath[0].indexBorder[19] == 19);
	REQUIRE( shPath[0].indexBorder[20] == 20);
	REQUIRE( shPath[0].indexBorder[21] == 21);
	REQUIRE( shPath[0].indexBorder[22] == 22);
	REQUIRE( shPath[0].indexBorder[23] == 23);
	REQUIRE( shPath[0].indexBorder[24] == 24);
	REQUIRE( shPath[0].indexBorder[25] == 25);
	REQUIRE( shPath[0].indexBorder[26] == 26);
	REQUIRE( shPath[0].indexBorder[27] == 27);
	REQUIRE( shPath[0].indexBorder[28] == 28);
	REQUIRE( shPath[0].indexBorder[29] == 29);
	REQUIRE( shPath[0].indexBorder[30] == 30);
	REQUIRE( shPath[0].indexBorder[31] == 31);
	REQUIRE( shPath[0].indexBorder[32] == 32);
	REQUIRE( shPath[0].indexBorder[33] == 33);
	REQUIRE( shPath[0].indexBorder[34] == 34);
	REQUIRE( shPath[0].indexBorder[35] == 35);
	REQUIRE( shPath[0].indexBorder[36] == 36);
	REQUIRE( shPath[0].indexBorder[37] == 37);
	REQUIRE( shPath[0].indexBorder[38] == 38);
	REQUIRE( shPath[0].indexBorder[39] == 39);
	REQUIRE( shPath[0].indexBorder[40] == 40);
	REQUIRE( shPath[0].indexBorder[41] == 41);
	REQUIRE( shPath[0].indexBorder[42] == 42);
	REQUIRE( shPath[0].indexBorder[43] == 43);
}

#endif


