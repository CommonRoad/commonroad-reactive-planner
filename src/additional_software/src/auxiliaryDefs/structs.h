#ifndef HEADER_STRUCTS
#define HEADER_STRUCTS

#include <iostream>
#include <vector>

struct vertice {
	double x;
	double y;
} ;

struct border {
	std::vector <vertice> vertices; // vertices of border
	std::vector <double> distances; // distance of each vertice to previous vertice
	std::vector <double> curvature; // curvature of the right border of each vertice
	size_t side;
} ;

struct center_struct {
	std::vector <vertice> vertices; // vertices of the center positions of the lane
	std::vector <double> distances; // path length at each center vertice
} ;

struct timeStruct {
	float startingTime;
	float timeStep;
	float ending;
};

typedef struct {
	size_t begin;
	size_t end;
} region;
#endif
