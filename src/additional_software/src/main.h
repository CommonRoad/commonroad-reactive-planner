#ifndef HEADER_MAIN
#define HEADER_MAIN

#include "inputs/readXMLfile.h"
#include "world/lanelets/lanelet_operations.h"
#include "world/obstacle/obstacle_operations.h"
#include "world/lane/lane_operations.h"
#include "world/obstacle/pedestrian.h"
#include <typeinfo>
#include "geometry/rectangle.h"
#include "geometry/geometricOperations.h"
#include "prediction/occupancyCalculation.h"
#include "inputs/pugi_xml/pugixml.hpp"
#include <fstream>
#include "output/writeToXML.h"
#include "geometry/minkowski.h"

int main();

double run_spot(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, std::vector <obstacle*>* obstacles, timeStruct timeInterval, int num_threads);

#endif
