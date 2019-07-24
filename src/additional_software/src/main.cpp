/*
 * main.cpp
 *
 *  Created on: 02.01.2018>
 *      Author: Sebastian
 */

#define RUNSPOT 1
#define PLOT 1
#define CATCH_CONFIG_RUNNER
//#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "unitTests/catch.hpp"

#include "main.h"

#include <chrono>  // for high_resolution_clock

#include <omp.h>

#include <thread>

static Catch::Session session; // There must be exactly once instance


void draw(std::string& xmlInput)
{
	system(("python3 -O /home/sebastian/Documents/automated-driving/python/script.py " + xmlInput).c_str());
}

int test_spot(std::string xmlInput)
{
	// Record start time
	auto start = std::chrono::high_resolution_clock::now();


	std::string xmlFile = "Scenarios/Release_2017a/";
	xmlFile += xmlInput; // "scenario_minimalExample.xml"
	const float timeStepSize = getTimeStep(xmlFile);
	//std::cout << "Time Step Size: " << timeStepSize << std::endl;

	std::vector <vehicularLanelet> lanelets = createLaneletFromXML(xmlFile);
	std::cout << "Number of lanelets: " << lanelets.size() << std::endl;

	std::vector <lane> lanes;
	createLanesFromLanelets(&lanes, &lanelets);
	std::cout << "Number of lanes: " << lanes.size() << std::endl;

	// compute the shortest path for every lane
	//size_t max_threads = omp_get_max_threads();
	#pragma omp parallel for num_threads(2) schedule(guided)
	for (size_t i = 0; i < lanes.size(); i++)
	{
		lanes[i].calculateShortestPath();
	}

/*
	//#pragma omp parallel for num_threads(2) schedule(guided)
	for (size_t i = 0; i < lanes.size(); i++)
	{
		lanes[i].calculateOptimalVelocityProfile();
	}
*/

	pedestrian* pedo_obj = new pedestrian();
	pedo_obj->setId(123);
	pedo_obj->setPosition(30, -6);
	pedo_obj->setPedestrianOrientation({4.4});
	circle* circle_obj = new circle();
	circle_obj->setRadius(0.1);
	pedo_obj->setShape(circle_obj);
	pedo_obj->setPedestrianVelocity({1});
	pedo_obj->setAcceleration(0.2);



	std::vector <obstacle*> obstacles;


	bool noInit = false; // first timeStamp
	obstacles = createObstacleFromXML(xmlFile, &obstacles, 0.0, noInit, &lanes);
	obstacles.push_back(pedo_obj);
	noInit = true;
	std::cout << "Number of obstacles: " << obstacles.size() << std::endl;

	// time horizon in seconds for prediction of the occupancy
	timeStruct timeInterval;
	timeInterval.startingTime = 0;
	timeInterval.timeStep = 0.1;
	timeInterval.ending = 7;

	/*
	 * compute the occupancy for all obstacles in the map for the time interval
	 * parallelize the prediction of each obstacle
	 */
	//std::cout << "Maximum number available threads: " << max_threads << std::endl;
	#pragma omp parallel for schedule(guided)
	for(size_t i = 0; i < obstacles.size(); i++)
	{
		obstacles[i]->computeOccupancyCore(&lanelets, &lanes, timeInterval);
	}

	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	std::cout << "Elapsed time: " << elapsed.count() <<  " s\n\n";

	// ToDO: ego vehicle

	#if PLOT == 1

		// export data to xml file
		writeToXML(lanelets, obstacles, xmlFile, timeStepSize);

		// plot results with the automated-driving framework (python)
		std::thread t(&draw, std::ref(xmlInput));
		t.detach();

		//system("xdg-open /home/sebastian/eclipse-workspace/spot/test");
		std::this_thread::sleep_for(std::chrono::seconds(5));

	#endif

	while(!(obstacles.empty()))
	{
		obstacle* temp = obstacles.back();
		obstacles.pop_back();
		delete temp;
		temp = 0;
	}
	// elapsed time

	return 1;
}


double run_spot(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, std::vector <obstacle*>* obstacles, timeStruct timeInterval, int num_threads)
{
	// Record start time
	auto start = std::chrono::high_resolution_clock::now();

	std::cout << "Number of obstacles: " << obstacles->size() << std::endl;
	std::cout << "Number of lanelets: " << lanelets->size() << std::endl;
	std::cout << "Number of lanes: " << lanes->size() << std::endl;

	omp_set_num_threads(num_threads);

	//#pragma omp parallel for schedule(guided)
	for(size_t i = 0; i < obstacles->size(); i++)
	{
		(*obstacles)[i]->updateInLane(lanes);
	}

	/*
	 * compute the occupancy for all obstacles in the map for the time interval
	 * parallelize the prediction of each obstacle
	 */
	#pragma omp parallel for schedule(guided)
	for(size_t i = 0; i < (*obstacles).size(); i++)
	{
		(*obstacles)[i]->computeOccupancyCore(lanelets, lanes, timeInterval);
	}

	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	std::cout << "Elapsed time: " << elapsed.count() <<  " s\n\n";
	return elapsed.count();
}





#if RUNSPOT == 1

TEST_CASE( "Execute whole spot program", "[spot]" )
{
	//REQUIRE( run_spot("GER_Muc_1a.xml") );
    REQUIRE( test_spot("GER_Muc_3a.xml") );
    //REQUIRE( run_spot("NGSIM_US101_9.xml") );
}

#endif


int main()
{
    session.run();

    return 1;
}
