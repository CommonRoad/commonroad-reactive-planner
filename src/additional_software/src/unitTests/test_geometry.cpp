/*
 * test_geometry.cpp
 *
 *  Created on: 20.03.2018
 *      Author: Sebastian
 */
#define TESTGEOMETRY 1
#include "catch.hpp"
#include "../auxiliaryDefs/structs.h"
#include "../geometry/geometricOperations.h"
#include "../geometry/minkowski.h"

#if TESTGEOMETRY==1

double epsilon = 0.000001;



TEST_CASE( "gradient", "[gradient]" )
{
	std::cout << "geometry unittests executed" << std::endl;
	std::vector <double> vertices_x;
	std::vector <double> vertices_y;
	vertice temp;
	temp.x = 10;
	temp.y = 10;
	vertices_x.push_back(temp.x);
	vertices_y.push_back(temp.y);
	temp.x = 12;
	temp.y = 8;
	vertices_x.push_back(temp.x);
	vertices_y.push_back(temp.y);
	temp.x = 15.5;
	temp.y = 12;
	vertices_x.push_back(temp.x);
	vertices_y.push_back(temp.y);
	temp.x = 20;
	temp.y = 15;
	vertices_x.push_back(temp.x);
	vertices_y.push_back(temp.y);
	temp.x = 22;
	temp.y = 17.3;
	vertices_x.push_back(temp.x);
	vertices_y.push_back(temp.y);
	temp.x = 30;
	temp.y = 30;
	vertices_x.push_back(temp.x);
	vertices_y.push_back(temp.y);
	temp.x = 20;
	temp.y = 28;
	vertices_x.push_back(temp.x);
	vertices_y.push_back(temp.y);
	std::vector <double> result_x = gradient(vertices_x);
	std::vector <double> result_y = gradient(vertices_y);
	REQUIRE( result_x[0] == 2);
	REQUIRE( result_y[0] == -2);
	REQUIRE( result_x[1] == 2.75);
	REQUIRE( result_y[1] == 1);
	REQUIRE( result_x[2] == 4);
	REQUIRE( result_y[2] == 3.5);
	REQUIRE( result_x[3] == 3.25);
	REQUIRE( std::abs(result_y[3] - 2.65) < epsilon);
	REQUIRE( result_x[4] == 5);
	REQUIRE( result_y[4] == 7.5);
	REQUIRE( result_x[5] == -1);
	REQUIRE( result_y[5] == 5.35);
	REQUIRE( result_x[6] == -10);
	REQUIRE( result_y[6] == -2);
}

TEST_CASE( "calcPathDistances", "[calcPathDistances]" )
{
	std::vector <vertice> vertices;
	vertice temp;
	temp.x = 15;
	temp.y = 25;
	vertices.push_back(temp);
	temp.x = 20;
	temp.y = 28;
	vertices.push_back(temp);
	temp.x = 12;
	temp.y = 29;
	vertices.push_back(temp);
	temp.x = 10;
	temp.y = 30;
	vertices.push_back(temp);
	temp.x = 5;
	temp.y = 35;
	vertices.push_back(temp);
	std::vector <double> distances = calcPathDistances(vertices);
	REQUIRE( distances[0] == 0);
	REQUIRE( std::abs(distances[1] - 5.830951894) < epsilon);
	REQUIRE( std::abs(distances[2] - 8.062257748) < epsilon);
	REQUIRE( std::abs(distances[3] - 2.236067977) < epsilon);
	REQUIRE( std::abs(distances[4] - 7.071067811) < epsilon);
}

TEST_CASE( "findClosestVertexOfPosition", "[findClosestVertexOfPosition]" )
{
	std::vector <vertice> p;
	vertice temp;
	temp.x = 0;
	temp.y = 0;
	p.push_back(temp);
	temp.x = 1;
	temp.y = 1;
	p.push_back(temp);
	temp.x = 2;
	temp.y = 3;
	p.push_back(temp);
	temp.x = 3;
	temp.y = 0;
	p.push_back(temp);
	vertice pos;
	pos.x = 0;
	pos.y = 2;
	size_t index = findClosestVertexOfPosition(p, pos);
	REQUIRE (index == 1);
}

TEST_CASE( "calcAngleOfVerticesAtPosition", "[calcAngleOfVerticesAtPosition]")
{
	// this function uses the findClosestVertexOfPosition internally
	std::vector <vertice> p;
	vertice temp;
	temp.x = 0;
	temp.y = 0;
	p.push_back(temp);
	temp.x = 1;
	temp.y = 1;
	p.push_back(temp);
	temp.x = 2;
	temp.y = 3;
	p.push_back(temp);
	temp.x = 3;
	temp.y = 0;
	p.push_back(temp);
	vertice pos;
	pos.x = 0;
	pos.y = 2;
	double angle = calcAngleOfVerticesAtPosition( p, pos);
	// atan2 calculated with https://www.medcalc.org/manual/atan2_function.php
	REQUIRE( std::abs(angle - 1.107148717794) < epsilon);
}

TEST_CASE( "Minkowski Sum", "[minkowski_sum]" )
{
    REQUIRE( test_minkowskiSum() );
}


TEST_CASE( "getNextIndex", "[getNextIndex]" )
{
	std::vector <vertice> vertices;
	vertice temp;
	temp.x = -15,
	temp.y = -2;
	vertices.push_back(temp);
	temp.x = 0;
	temp.y = -2;
	vertices.push_back(temp);
	temp.x = 15;
	temp.y = -2;
	vertices.push_back(temp);
	std::string direction;
	direction = "forward";
	std::vector <double> distances;
	distances.push_back(0);
	distances.push_back(15);
	distances.push_back(15);
	size_t currentIndex = 0;
	size_t index = getNextIndex(vertices, currentIndex, direction, distances);
    REQUIRE( index == 1 );
}

TEST_CASE( "addObjectDimensions with one vertice", "[addObjectDimensions-OneVertice]" )
{
	std::vector <vertice> q;
	std::vector <vertice> p; // result
	vertice temp;
	temp.x = 1;
	temp.y = 1;
	q.push_back(temp);
	double length = 2;
	double width = 3;
	p = addObjectDimensions(q, length, width);
	bool rightResults = false;
	if (p[0].x == 0 && p[0].y == 2.5 && p[1].x == 2 && p[1].y == 2.5 &&
			p[2].x == 2 && p[2].y == -0.5 && p[3].x == 0 && p[3].y == -0.5)
	{
		rightResults = true;
	}
    REQUIRE( rightResults );
}

TEST_CASE( "addObjectDimensions with one vertice (Matlab-Results)", "[addObjectDimensions-OneVertice_Matlab]" )
{
	std::vector <vertice> q;
	std::vector <vertice> p; // result
	vertice temp;
	temp.x = 0;
	temp.y = 0;
	q.push_back(temp);
	double length = 5.4864;
	double width = 1.6459;
	p = addObjectDimensions(q, length, width);
	bool rightResults = false;
	if (p[0].x == -2.7432 && p[0].y == 0.82295 && p[1].x == 2.7432 && p[1].y == 0.82295 &&
			p[2].x == 2.7432 && p[2].y == -0.82295 && p[3].x == -2.7432 && p[3].y == -0.82295)
	{
		rightResults = true;
	}
    REQUIRE( rightResults );
}

TEST_CASE( "addObjectDimensions with arbitrary vertices ", "[addObjectDimensions-ArbitraryVertices]" )
{
	std::vector <vertice> q;
	std::vector <vertice> p; // result
	vertice temp;
	temp.x = 1;
	temp.y = 1;
	q.push_back(temp);
	temp.x = 1;
	temp.y = 2;
	q.push_back(temp);
	temp.x = 2;
	temp.y = 2;
	q.push_back(temp);
	temp.x = 2;
	temp.y = 1;
	q.push_back(temp);
	double length = 2;
	double width = 3;
	p = addObjectDimensions(q, length, width);
	REQUIRE( p[0].x == 0);
	REQUIRE( p[0].y == -0.5);
	REQUIRE( p[1].x == 0);
	REQUIRE( p[1].y == 3.5);
	REQUIRE( p[2].x == 3);
	REQUIRE( p[2].y == 3.5);
	REQUIRE( p[3].x == 3);
	REQUIRE( p[3].y == -0.5);
}

TEST_CASE( "addObjectDimensions with arbitrary vertices (Matlab-Results)", "[addObjectDimensions-ArbitraryVertices_Matlab]" )
{
	std::vector <vertice> q;
	std::vector <vertice> p; // result
	vertice temp;
	temp.x = 11.76174;
	temp.y = 12.960000000000004;
	q.push_back(temp);
	temp.x = 37.681740000000005;
	temp.y = 12.960000000000004;
	q.push_back(temp);
	temp.x = 37.681740000000005;
	temp.y = -12.960000000000004;
	q.push_back(temp);
	temp.x = 11.76174;
	temp.y = -12.960000000000004;
	q.push_back(temp);
	double length = 5.334;
	double width = 1.7983;
	p = addObjectDimensions(q, length, width);
	REQUIRE( 1 );
}

TEST_CASE( "addObjectDimensions with six vertices (Matlab-Results)", "[addObjectDimensions-SixVertices_Matlab]" )
{
	std::vector <vertice> q;
	std::vector <vertice> p; // result
	vertice temp;
	temp.x = 0;
	temp.y = 0;
	q.push_back(temp);
	temp.x = 1.992705435012611;
	temp.y = 0.04;
	q.push_back(temp);
	temp.x = 2.03431;
	temp.y = 0.04;
	q.push_back(temp);
	temp.x = 2.03431;
	temp.y = -0.04;
	q.push_back(temp);
	temp.x = 1.992705435012611;
	temp.y = -0.04;
	q.push_back(temp);
	temp.x = 0;
	temp.y = 0;
	q.push_back(temp);
	double length = 5.4864;
	double width = 1.6459;
	p = addObjectDimensions(q, length, width);
	REQUIRE( std::abs(p[0].x - -2.7432) < epsilon );
	REQUIRE( std::abs(p[0].y - 0.82295) < epsilon );
	REQUIRE( std::abs(p[1].x - -0.750494564987389) < epsilon );
	REQUIRE( std::abs(p[1].y - 0.86295) < epsilon );
	REQUIRE( std::abs(p[2].x - 4.777509999999999) < epsilon );
	REQUIRE( std::abs(p[2].y - 0.86295) < epsilon );
	REQUIRE( std::abs(p[3].x - 4.777509999999999) < epsilon );
	REQUIRE( std::abs(p[3].y - -0.86295) < epsilon );
	REQUIRE( std::abs(p[4].x - -0.750494564987389) < epsilon );
	REQUIRE( std::abs(p[4].y - -0.86295) < epsilon );
	REQUIRE( std::abs(p[5].x - -2.7432) < epsilon );
	REQUIRE( std::abs(p[5].y - -0.82295) < epsilon );
}

TEST_CASE( "calcAngleOfVerticesAtPosition (Matlab-Results)", "[calcAngleOfVerticesAtPosition_Matlab]" )
{
	vertice position;
	position.x = 132.7427;
	position.y = 2.6801;
	std::vector <vertice> vertices;
	vertice temp;
	temp.x = -22.6981;
	temp.y = 4.45635;
	vertices.push_back(temp);
	temp.x = -12.7554;
	temp.y = 4.3807;
	vertices.push_back(temp);
	temp.x = -12.339;
	temp.y = 4.37485;
	vertices.push_back(temp);
	temp.x = -12.29545;
	temp.y = 4.3743;
	vertices.push_back(temp);
	temp.x = -12.2521;
	temp.y = 4.37355;
	vertices.push_back(temp);
	temp.x = -12.23005;
	temp.y = 4.3731;
	vertices.push_back(temp);
	temp.x = -8.87555;
	temp.y = 4.30455;
	vertices.push_back(temp);
	temp.x = -8.46885;
	temp.y = 4.2942;
	vertices.push_back(temp);
	temp.x = -8.43605;
	temp.y = 4.29345;
	vertices.push_back(temp);
	temp.x = -8.40325;
	temp.y = 4.29255;
	vertices.push_back(temp);
	temp.x = -8.38675;
	temp.y = 4.29205;
	vertices.push_back(temp);
	temp.x = 1.34785;
	temp.y = 3.99795;
	vertices.push_back(temp);
	temp.x = 1.8357;
	temp.y = 3.9856;
	vertices.push_back(temp);
	temp.x = 5.1904;
	temp.y = 3.9167;
	vertices.push_back(temp);
	temp.x = 5.58585;
	temp.y = 3.90645;
	vertices.push_back(temp);
	temp.x = 5.622;
	temp.y = 3.9056;
	vertices.push_back(temp);
	temp.x = 5.65815;
	temp.y = 3.9045;
	vertices.push_back(temp);
	temp.x = 5.6763;
	temp.y = 3.904;
	vertices.push_back(temp);
	temp.x = 15.79925;
	temp.y = 3.58815;
	vertices.push_back(temp);
	temp.x = 16.2854;
	temp.y = 3.5757;
	vertices.push_back(temp);
	temp.x = 19.64015;
	temp.y = 3.5068;
	vertices.push_back(temp);
	temp.x = 20.15215;
	temp.y = 3.4974;
	vertices.push_back(temp);
	temp.x = 30.1927;
	temp.y = 3.3363;
	vertices.push_back(temp);
	temp.x = 30.66645;
	temp.y = 3.32765;
	vertices.push_back(temp);
	temp.x = 30.6969;
	temp.y = 3.3271;
	vertices.push_back(temp);
	temp.x = 34.0593;
	temp.y = 3.25805;
	vertices.push_back(temp);
	temp.x = 34.551;
	temp.y = 3.24765;
	vertices.push_back(temp);
	temp.x = 44.3891;
	temp.y = 3.0333;
	vertices.push_back(temp);
	temp.x = 44.8913;
	temp.y = 3.0227;
	vertices.push_back(temp);
	temp.x = 48.246;
	temp.y = 2.9538;
	vertices.push_back(temp);
	temp.x = 48.6242;
	temp.y = 2.94365;
	vertices.push_back(temp);
	temp.x = 48.6661;
	temp.y = 2.9426;
	vertices.push_back(temp);
	temp.x = 48.70795;
	temp.y = 2.94135;
	vertices.push_back(temp);
	temp.x = 48.72895;
	temp.y = 2.9407;
	vertices.push_back(temp);
	temp.x = 59.11485;
	temp.y = 2.59926;
	vertices.push_back(temp);
	temp.x = 59.59815;
	temp.y = 2.58648;
	vertices.push_back(temp);
	temp.x = 62.9529;
	temp.y = 2.51757;
	vertices.push_back(temp);
	temp.x = 63.4647;
	temp.y = 2.508185;
	vertices.push_back(temp);
	temp.x = 73.29565;
	temp.y = 2.349595;
	vertices.push_back(temp);
	temp.x = 73.77;
	temp.y = 2.340925;
	vertices.push_back(temp);
	temp.x = 73.79985;
	temp.y = 2.340345;
	vertices.push_back(temp);
	temp.x = 77.1621;
	temp.y = 2.271315;
	vertices.push_back(temp);
	temp.x = 77.55455;
	temp.y = 2.261125;
	vertices.push_back(temp);
	temp.x = 77.5917;
	temp.y = 2.260185;
	vertices.push_back(temp);
	temp.x = 77.6289;
	temp.y = 2.25915;
	vertices.push_back(temp);
	temp.x = 77.6476;
	temp.y = 2.258555;
	vertices.push_back(temp);
	temp.x = 87.46525;
	temp.y = 1.949325;
	vertices.push_back(temp);
	temp.x = 87.9509;
	temp.y = 1.936795;
	vertices.push_back(temp);
	temp.x = 91.3057;
	temp.y = 1.86789;
	vertices.push_back(temp);
	temp.x = 91.82415;
	temp.y = 1.859435;
	vertices.push_back(temp);
	temp.x = 101.70725;
	temp.y = 1.738596;
	vertices.push_back(temp);
	temp.x = 102.15515;
	temp.y = 1.731246;
	vertices.push_back(temp);
	temp.x = 102.1832;
	temp.y = 1.730803;
	vertices.push_back(temp);
	temp.x = 102.2112;
	temp.y = 1.7303155;
	vertices.push_back(temp);
	temp.x = 102.2253;
	temp.y = 1.7300185;
	vertices.push_back(temp);
	temp.x = 105.58035;
	temp.y = 1.6612105;
	vertices.push_back(temp);
	temp.x = 106.1032;
	temp.y = 1.6534385;
	vertices.push_back(temp);
	temp.x = 116.3801;
	temp.y = 1.55385;
	vertices.push_back(temp);
	temp.x = 116.81075;
	temp.y = 1.54733;
	vertices.push_back(temp);
	temp.x = 116.8473;
	temp.y = 1.54684;
	vertices.push_back(temp);
	temp.x = 116.88375;
	temp.y = 1.546215;
	vertices.push_back(temp);
	temp.x = 116.9022;
	temp.y = 1.545825;
	vertices.push_back(temp);
	temp.x = 120.2574;
	temp.y = 1.477165;
	vertices.push_back(temp);
	temp.x = 120.8477;
	temp.y = 1.48219;
	vertices.push_back(temp);
	temp.x = 130.83575;
	temp.y = 1.78439;
	vertices.push_back(temp);
	temp.x = 131.1293;
	temp.y = 1.7863745;
	vertices.push_back(temp);
	temp.x = 131.2075;
	temp.y = 1.786279;
	vertices.push_back(temp);
	temp.x = 131.32515;
	temp.y = 1.7860935;
	vertices.push_back(temp);
	temp.x = 131.41205;
	temp.y = 1.784397;
	vertices.push_back(temp);
	temp.x = 134.7764;
	temp.y = 1.720606;
	vertices.push_back(temp);
	temp.x = 135.3793;
	temp.y = 1.7276295;
	vertices.push_back(temp);
	temp.x = 145.1453;
	temp.y = 2.07346;
	vertices.push_back(temp);
	temp.x = 145.62583;
	temp.y = 2.08541;
	vertices.push_back(temp);
	temp.x = 145.69555;
	temp.y = 2.08736;
	vertices.push_back(temp);
	temp.x = 145.76505;
	temp.y = 2.08876;
	vertices.push_back(temp);
	temp.x = 145.8007;
	temp.y = 2.08927;
	vertices.push_back(temp);
	temp.x = 149.0393;
	temp.y = 2.13674;
	vertices.push_back(temp);
	temp.x = 149.70175;
	temp.y = 2.15441;
	vertices.push_back(temp);
	temp.x = 159.61225;
	temp.y = 2.5283;
	vertices.push_back(temp);
	temp.x = 160.0771;
	temp.y = 2.54044;
	vertices.push_back(temp);
	temp.x = 160.15445;
	temp.y = 2.542665;
	vertices.push_back(temp);
	temp.x = 160.23135;
	temp.y = 2.54427;
	vertices.push_back(temp);
	temp.x = 160.27095;
	temp.y = 2.54481;
	vertices.push_back(temp);
	temp.x = 163.5099;
	temp.y = 2.59248;
	vertices.push_back(temp);
	temp.x = 164.1813;
	temp.

	y = 2.612425;
	vertices.push_back(temp);
	temp.x = 164.1813;
	temp.y = 2.612425;
	vertices.push_back(temp);
	REQUIRE( std::abs(calcAngleOfVerticesAtPosition(vertices, position) - -0.0189585973667633) < 0.000001 );
}

TEST_CASE( "radiansToPi", "[radiansToPi]")
{
	double angle1 = -4.0;
	double wrapped_angle1 = wrapToPi(angle1);
	REQUIRE( std::abs(wrapped_angle1 - 2.28318530718) < epsilon);
	double angle2 = 5.0;
	double wrapped_angle2 = wrapToPi(angle2);
	REQUIRE( std::abs(wrapped_angle2 - -1.28318530718) < epsilon);
	double angle3 = 2.5;
	double wrapped_angle3 = wrapToPi(angle3);
	REQUIRE( std::abs(wrapped_angle3 - 2.5) < epsilon);
}

TEST_CASE( "minima-regions", "[minima-regions]")
{
	std::vector <double> radius;
	radius.push_back(1);
	radius.push_back(2);
	radius.push_back(1.5);
	radius.push_back(2);
	radius.push_back(2);
	radius.push_back(2);
	radius.push_back(0);
	radius.push_back(0);
	radius.push_back(1);
	radius.push_back(2);
	radius.push_back(3);
	radius.push_back(3);
	radius.push_back(3);
	radius.push_back(3);
	radius.push_back(2);
	radius.push_back(4);
	radius.push_back(1);
	radius.push_back(0);

	std::vector<size_t> minima;
	std::vector<region> regions;

	getLocalConstantRegions(&radius, &minima, &regions);

	REQUIRE( minima.size() == 4);
	REQUIRE( minima[0] == 0);
	REQUIRE( minima[1] == 2);
	REQUIRE( minima[2] == 14);
	REQUIRE( minima[3] == 17);

	REQUIRE( regions.size() == 3);
	REQUIRE( regions[0].begin == 3);
	REQUIRE( regions[0].end == 5);
	REQUIRE( regions[1].begin == 6);
	REQUIRE( regions[1].end == 7);
	REQUIRE( regions[2].begin == 10);
	REQUIRE( regions[2].end == 13);
}

TEST_CASE( "calcProjectedDistance", "[calcProjectedDistance]")
{
	vertice position;
	position.x = -9.6435;
	position.y = 65.3305;

	size_t i = 3;

	border bound;
	vertice temp;
	temp.x = -3.2361;
	temp.y = 94.080699999999993;
	bound.vertices.push_back(temp);
	temp.x = -5.8536;
	temp.y = 84.634399999999999;
	bound.vertices.push_back(temp);
	temp.x = -8.4725;
	temp.y = 75.183000000000007;
	bound.vertices.push_back(temp);
	temp.x = -11.0914;
	temp.y = 65.731700000000004;
	bound.vertices.push_back(temp);
	temp.x = -13.710400000000000;
	temp.y = 56.280299999999997;
	bound.vertices.push_back(temp);
	temp.x = -16.329300000000000;
	temp.y = 46.829000000000001;
	bound.vertices.push_back(temp);
	temp.x = -18.948300000000000;
	temp.y = 37.377600000000001;
	bound.vertices.push_back(temp);
	temp.x = -21.567299999999999;
	temp.y = 27.926200000000001;
	bound.vertices.push_back(temp);
	temp.x = -24.186199999999999;
	temp.y = 18.474799999999998;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.827200000000001;
	temp.y = 8.940899999999999;
	bound.vertices.push_back(temp);
	temp.x = -26.831000000000000;
	temp.y = 8.940000000000000;
	bound.vertices.push_back(temp);
	temp.x = -26.831000000000000;
	temp.y = 8.940000000000000;
	bound.vertices.push_back(temp);
	temp.x = -26.831000000000000;
	temp.y = 8.940000000000000;
	bound.vertices.push_back(temp);
	temp.x = -26.831000000000000;
	temp.y = 8.940000000000000;
	bound.vertices.push_back(temp);
	temp.x = -26.831000000000000;
	temp.y = 8.940000000000000;
	bound.vertices.push_back(temp);
	temp.x = -26.831000000000000;
	temp.y = 8.940000000000000;
	bound.vertices.push_back(temp);
	temp.x = -26.831000000000000;
	temp.x = 8.940000000000000;
	bound.vertices.push_back(temp);
	temp.x = -39.276499999999999;
	temp.y = 12.365900000000000;
	bound.vertices.push_back(temp);
	temp.x = -49.289200000000001;
	temp.y = 15.122100000000000;
	bound.vertices.push_back(temp);
	temp.x = -59.302000000000000;
	temp.y = 17.878299999999999;
	bound.vertices.push_back(temp);
	temp.x = -69.314800000000005;
	temp.y = 20.634499999999999;
	bound.vertices.push_back(temp);
	temp.x = -79.327500000000001;
	temp.y = 23.390799999999999;
	bound.vertices.push_back(temp);
	temp.x = -89.3402999999999997;
	temp.y = 26.146999999999998;
	bound.vertices.push_back(temp);

    bound.distances.push_back(0);
    bound.distances.push_back(9.802239026875435);
    bound.distances.push_back(9.807527678778166);
    bound.distances.push_back(9.807431309981226);
    bound.distances.push_back(9.807554382209672);
    bound.distances.push_back(9.807431309981219);
    bound.distances.push_back(9.807554382209664);
    bound.distances.push_back(9.807554382209664);
    bound.distances.push_back(9.807527678778177);
    bound.distances.push_back(9.892933347091750);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0.003905124837952);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(0);
    bound.distances.push_back(12.908418224554083);
    bound.distances.push_back(10.385123963150370);
    bound.distances.push_back(10.385220377055076);
    bound.distances.push_back(10.385220377055083);
    bound.distances.push_back(10.385150503483322);
    bound.distances.push_back(10.385220377055076);


	double distance = calcProjectedDistance(i, bound, position);

	REQUIRE( std::abs(distance - 0.000009797093831252431) < epsilon);
}

TEST_CASE( "isLeft", "[isLeft]")
{
	vertice lineStart;
	vertice lineEnd;
	lineStart.x = 1;
	lineStart.y = 1;
	lineEnd.x = 4;
	lineEnd.y =2;

	vertice pos;
	pos.x = 2;
	pos.y = 2;

	bool result = isLeft(lineStart, lineEnd, pos);

	REQUIRE( result == true );

	pos.x = 3;
	pos.y = 1;

	result = isLeft(lineStart, lineEnd, pos);

	REQUIRE( result == false );
}


TEST_CASE( "calcVectorIntersectionPoint", "[calcVectorIntersectionPoint]")
{
	vertice a, b, c , d;

	a.x = 16;
	a.y = 47;

	b.x = 19;
	b.y = -5;

	c.x = -16;
	c.y = 37;

	d.x = 0.25;
	d.y = 1;

	double beta = calcVectorIntersectionPoint(a, b, c, d);

	REQUIRE( std::abs(beta - 17.283950617283949) < epsilon );

	double alpha = calcVectorIntersectionPointAlpha(a, b, c, d);

	REQUIRE( std::abs(alpha - -1.45679012345679) < epsilon );
}

TEST_CASE( "findInnerVerticeOfPosition", "[findInnerVerticeOfPosition]")
{
	shortestPath shPath;
	shPath.side.push_back(0);
	shPath.side.push_back(0);
	shPath.side.push_back(0);
	shPath.indexBorder.push_back(0);
	shPath.indexBorder.push_back(1);
	shPath.indexBorder.push_back(2);

	vertice position;
	position.x = -10;
	position.y = 0;

	std::vector<vertice> leftBorderVertices;
	vertice temp;
	temp.x = -15;
	temp.y = 2;
	leftBorderVertices.push_back(temp);
	temp.x = 0;
	temp.y = 2;
	leftBorderVertices.push_back(temp);
	temp.x = 15;
	temp.y = 2;
	leftBorderVertices.push_back(temp);

	std::vector<vertice> rightBorderVertices;
	temp.x = -15;
	temp.y = -2;
	rightBorderVertices.push_back(temp);
	temp.x = 0;
	temp.y = -2;
	rightBorderVertices.push_back(temp);
	temp.x = 15;
	temp.y = -2;
	rightBorderVertices.push_back(temp);

	std::vector <size_t> ids = findInnerVerticeOfPosition(position, shPath, leftBorderVertices, rightBorderVertices);

	REQUIRE( ids[0] == 0);
	REQUIRE( ids[1] == 0);
}

TEST_CASE( "calcCurvature_new", "[calcCurvature_new]")
{
	std::vector <vertice> vertices;

	vertice temp;

	temp.x = 1;
	temp.y = 6;
	vertices.push_back(temp);

	temp.x = 9.54;
	temp.y = 9.54;
	vertices.push_back(temp);

	temp.x = 6;
	temp.y = 1;
	vertices.push_back(temp);

	std::vector <double> curvature = calcCurvature(vertices);

	for(size_t i = 0; i < curvature.size(); i++)
	{
		std::cout << "curv: " << curvature[i] << std::endl;
	}

}

#endif


