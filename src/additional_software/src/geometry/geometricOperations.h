/*
 * geometric operations
 */

#ifndef HEADER_GEOMETRICOPERATIONS
#define HEADER_GEOMETRICOPERATIONS

#include "../auxiliaryDefs/structs.h"
#include "../world/lane/lane.h"
#include "circle.h"

/*
 * add the dimensions of the object (length and width)
 * to the polygon vertices q in the object's coordinate frame
 */
std::vector <vertice> addObjectDimensions(std::vector <vertice> q, double length, double width);

/*
 * rotateAndTranslateVertices - rotate and translate the vertices from
 * the special relative coordinates to the reference position and orientation
 * (transfer local coordinates to global coordinates)
 */
std::vector <vertice> rotateAndTranslateVertices(std::vector <vertice> vertices, vertice refPosition, double refOrientation);

/*
 * finds the vertex of the given polyline which
 * has the minimal distance to the given position
 */
size_t findClosestVertexOfPosition(std::vector <vertice> v, vertice pos);

/*
 * calculates the angle (orientation) of
 * the line segment of the vertices which has the minimal distance to the
 * given position.
 * The angle is measured in the closed interval [-pi, pi] relative to the
 * global x-axis (counter-clockwise is positive).
 */
double calcAngleOfVerticesAtPosition(std::vector <vertice> v, vertice pos);

/*
 * find the vertice of the inner bound which has the
 * minimal distance to the object's position
 */
std::vector <size_t> findInnerVerticeOfPosition(vertice position, shortestPath shPath, std::vector <vertice> leftBorderVertices, std::vector <vertice> rightBorderVertices);

/*
 * project the position on the bound such that it is
 * perpendicular to the pseudo target and calculate the distance of the
 * projection point to vertice i
 */
double calcProjectedDistance(size_t i, border bound, vertice position);

/*
 * returns the next index in the specified direction while
 * omitting vertices with zero distance
 */
size_t getNextIndex(std::vector <vertice> vertices, size_t currentIndex, std::string direction, std::vector <double> distances);

/*
 * calculate the distance (L2-norm) between all adjacent
 * points (x,y) of the polyline
 */
std::vector <double> calcPathDistances(std::vector <vertice> vertices);

// calculate the well-known signed curvature of the polyline
std::vector <double> calcCurvature(std::vector <vertice> vertices);

// calculate the well-known signed curvature of the polyline
std::vector <double> calcCurvature_new(std::vector <vertice> vertices);

/*
 * returns boolean array whether the points are on the left of the
 * lines (function accepts single/multiple points and single/multiple lines)
 */
bool isLeft(vertice linesStart, vertice linesEnd, vertice points);

/*
 * calculate the parameters alpha and beta
 * such that the two lines intersect: a + alpha*b = c + beta*d
 * (lines are give in vector equation: r = OA + lamda*AB)
 */
double calcVectorIntersectionPoint(vertice a, vertice b, vertice c, vertice d);
double calcVectorIntersectionPointAlpha(vertice a, vertice b, vertice c, vertice d);

/*
 * calculates the central difference for interior data points.
 * The Algorithm used can be is equivalent to the gradient function from Matlab
 */
std::vector <double> gradient(std::vector <double> data);

// Wrap angle in radians to [(minus)pi, pi]
double wrapToPi(double rad);

// determines minmia and regions of constant values along a curve
void getLocalConstantRegions(std::vector <double>* radius, std::vector <size_t>* minima, std::vector <region>* regions);

// circumscribedPolygon of circle - function to compute circumscribing polygon with num_vertices corners
std::vector<vertice> encloseByPolygon(circle* geometry, double num_vertices, double orientation = 0);

/*
 * function to compute circumscribing polygon
 * portion with num_vertices corners, i.e. over-approximation of circle
 * sector specified by the start and end orientation
 */
std::vector<vertice> encloseSectionByPolygon(circle* geometry, double num_vertices, double start_orientation, double end_orientation);

// scales polygon vertices by the factor relative to a fixpoint
std::vector<vertice> scaleVerticesAroundFixpoint(std::vector<vertice> vertices, vertice fixpoint, double factor);

double calcPseudoDistance(size_t idx, std::vector <vertice> vertices, vertice pos, vertice* pLampda);

// minimum value of array
double getMin(std::vector <double> array);

// maximum value of array
double getMax(std::vector <double> array);

// mean value of array
double getMean(std::vector <double> array);

#endif
