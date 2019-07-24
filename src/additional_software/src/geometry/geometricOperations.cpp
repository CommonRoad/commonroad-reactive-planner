#include "geometricOperations.h"

#include <cmath>
#include <boost/math/tools/precision.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>

#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif

std::vector <vertice> addObjectDimensions(std::vector <vertice> q, double length, double width)
{
	std::vector <vertice> p;

	// check for special cases
	if (q.size() == 1) // exactly one vertice
	{
		// add the dimension around the point q
		vertice p1, p2, p3, p4;
		p1.x = q.front().x + (-0.5 * length);
		p1.y = q.front().y + (0.5 * width);
		p2.x = q.front().x + (0.5 * length);
		p2.y = q.front().y + (0.5 * width);
		p3.x = q.front().x + (0.5 * length);
		p3.y = q.front().y + (-0.5 * width);
		p4.x = q.front().x + (-0.5 * length);
		p4.y = q.front().y + (-0.5 * width);
		p.push_back(p1);
		p.push_back(p2);
		p.push_back(p3);
		p.push_back(p4);
	}
	else if (q.size() == 6) // exactly six vertices
	{
		// add the dimensions to all six vertices q (Theorem 1)
		vertice p1, p2, p3, p4, p5, p6;
		p1.x = q[0].x + (-0.5 * length);
		p1.y = q[0].y + (0.5 * width);
		p2.x = q[1].x + (-0.5 * length);
		p2.y = q[1].y + (0.5 * width);
		p3.x = q[2].x + (0.5 * length);
		p3.y = q[2].y + (0.5 * width);
		p4.x = q[3].x + (0.5 * length);
		p4.y = q[3].y + (-0.5 * width);
		p5.x = q[4].x + (-0.5 * length);
		p5.y = q[4].y + (-0.5 * width);
		p6.x = q[5].x + (-0.5 * length);
		p6.y = q[5].y + (-0.5 * width);
		p.push_back(p1);
		p.push_back(p2);
		p.push_back(p3);
		p.push_back(p4);
		p.push_back(p5);
		p.push_back(p6);
	}
	else if (q.size() != 0) // arbitrary polygon
	{
		// add the dimensions to all vertices q:
		// (left up, right up, left down, right down)
		std::vector <vertice> p_LU = q;
		std::vector <vertice> p_RU = q;
		std::vector <vertice> p_LD = q;
		std::vector <vertice> p_RD = q;
		std::vector <vertice> p_all;
		for (size_t i = 0; i < q.size(); i++)
		{
			p_LU[i].x -= 0.5 * length;
			p_LU[i].y += 0.5 * width;

			p_RU[i].x += 0.5 * length;
			p_RU[i].y += 0.5 * width;

			p_LD[i].x -= 0.5 * length;
			p_LD[i].y -= 0.5 * width;

			p_RD[i].x += 0.5 * length;
			p_RD[i].y -= 0.5 * width;
		}
		for (size_t i = 0; i < q.size(); i++)
		{
			p_all.push_back(p_LU[i]);
		}
		for (size_t i = 0; i < q.size(); i++)
		{
			p_all.push_back(p_RU[i]);
		}
		for (size_t i = 0; i < q.size(); i++)
		{
			p_all.push_back(p_LD[i]);
		}
		for (size_t i = 0; i < q.size(); i++)
		{
			p_all.push_back(p_RD[i]);
		}
		using coordinate_type = double;
		using point_type = boost::geometry::model::d2::point_xy<coordinate_type>;
		using polygon_type = boost::geometry::model::polygon<point_type>;
		using boost::geometry::get;

		 // Construct polygon
		polygon_type polygon;
		for (size_t m = 0; m < p_all.size(); m++)
		{
			boost::geometry::append(polygon, point_type {p_all[m].x, p_all[m].y});
		}
		boost::geometry::append(polygon, point_type {p_all[0].x, p_all[0].y}); // close polygon
		polygon_type hull;
		boost::geometry::convex_hull(polygon, hull);

		std::vector<point_type> const& points = hull.outer();
		for (std::vector<point_type>::size_type i = 0; i < points.size(); ++i)
		{
			vertice temp;
			temp.x = double(get<0>(points[i]));
			temp.y = double(get<1>(points[i]));
			p.push_back(temp);
		}
	}
	else
	{
		throw std::runtime_error("Input vector is not a 2D row of vertices.");
	}
	return p;
}

std::vector <vertice> rotateAndTranslateVertices(std::vector <vertice> vertices, vertice refPosition, double refOrientation)
{
	double cosinus = cos(refOrientation);
	double sinus = sin(refOrientation);
	std::vector <vertice> transVertices;
	// rotation
	for (size_t i = 0; i < vertices.size(); i++)
	{
		vertice temp;
		temp.x = cosinus * vertices[i].x - sinus * vertices[i].y;
		temp.y = sinus * vertices[i].x + cosinus * vertices[i].y;
		transVertices.push_back(temp);
	}

	// translation
	for (size_t i = 0; i < transVertices.size(); i++)
	{
		transVertices[i].x = transVertices[i].x + refPosition.x;
		transVertices[i].y = transVertices[i].y + refPosition.y;
	}

	return transVertices;
}

size_t findClosestVertexOfPosition(std::vector <vertice> v, vertice pos)
{
	size_t id = 0;
	double distance = boost::math::tools::max_value<double>();
	double temp;
	for (size_t i = 0; i < v.size(); i++)
	{
		temp = std::pow(std::pow(v[i].x - pos.x, 2) + std::pow(v[i].y - pos.y, 2), 0.5);
		if (temp < distance)
		{
			distance = temp;
			id = i;
		}
	}
	if (v.size() == 0)
	{
		throw std::runtime_error("Received vector is empty!");
	}
	return id;
}

double calcAngleOfVerticesAtPosition(std::vector <vertice> v, vertice pos)
{
	// std::cout << pos.x << std::endl;
	//find closest vertex
	size_t indexClosestVertex = findClosestVertexOfPosition(v, pos);
	//std::cout << indexClosestVertex << std::endl;
	//std::cout << v.size() << std::endl;
	if (indexClosestVertex == v.size()-1)
	{
		indexClosestVertex--;
	}

	double angle = std::atan2(v[indexClosestVertex+1].y - v[indexClosestVertex].y,
						v[indexClosestVertex+1].x - v[indexClosestVertex].x);
	return angle;
}

std::vector <size_t> findInnerVerticeOfPosition(vertice position, shortestPath shPath, std::vector <vertice> leftBorderVertices, std::vector <vertice> rightBorderVertices)
{

	size_t iBorder;
	std::vector <double> distances;
	std::vector <size_t> ids;

	// calculate distance between the inner bound and the object position for
	// all vertices (to find the global minima)
	for (size_t iPath = 0; iPath < shPath.side.size(); iPath++)
	{
		iBorder = shPath.indexBorder[iPath];
		if (shPath.side[iPath]) // i.e. left
		{
			distances.push_back(std::pow(std::pow(leftBorderVertices[iBorder].x - position.x, 2) + std::pow(leftBorderVertices[iBorder].y - position.y, 2), 0.5));
		}
	    else //i.e. right
	    {
	    	distances.push_back(std::pow(std::pow(rightBorderVertices[iBorder].x - position.x, 2) + std::pow(rightBorderVertices[iBorder].y - position.y, 2), 0.5));
	    }
	}

	// find the inner vertice of the minimal distance (in respect to the shortest
	//  path indexes)
	double minDistance = boost::math::tools::max_value<double>();
	size_t iPath_min = -1;
	for (size_t i = 0; i < distances.size(); i++)
	{
		if (distances[i] < minDistance)
		{
			minDistance = distances[i];
			iPath_min = i;
		}
	}

	//check for plausibility
	/*
	if (minDistance >= 99)
	    warning(['The object is not close enough to the lane. (Its position' ...
	    ' is more than 99 m away from the nearest lane vertex.'])
	    %error('Object position is not close enough to lane. Error in geometry.findVerticeOfPosition')
	end
	*/

	// the innner vertice of the minimal distance in respect to the border indexes
	size_t iBorder_min = shPath.indexBorder[iPath_min];
	ids.push_back(iPath_min);
	ids.push_back(iBorder_min);
	return ids;
}

size_t getNextIndex(std::vector <vertice> vertices, size_t currentIndex, std::string direction, std::vector <double> distances)
{
	size_t nextIndex = 0;

	// define epsilon for distance check
	double epsilon = 10e-6;

	// find next vertex in the specified direction
	if (direction == "backward")
	{
		if (currentIndex == 0)
		{
			nextIndex = currentIndex;
		}
		else
		{
			size_t k = currentIndex - 1;
			while (distances[k+1] <= epsilon && k > 0)
			{
				k = k - 1;
			}
			nextIndex = k;
		}
	}
	else if (direction == "forward")
	{
		if (currentIndex == vertices.size())
		{
			nextIndex = currentIndex;
		}
		else
		{
			size_t k = currentIndex + 1;
			while (distances[k] <= epsilon && k < vertices.size()-1)
			{
				k = k + 1;
			}
			nextIndex = k;
		}
	}
	else
	{
		// error
		throw std::runtime_error("Wrong Input!");
	}
	return nextIndex;
}

double calcProjectedDistance(size_t i, border bound, vertice position)
{

	// vertice_i of inner bound
	vertice vi = bound.vertices[i];

	/*
	 * find the previous and next vertices on the lane bound:
	 * (note that these vertices might not be the on the inner bound)
	 */
	if (i == 0 && i == bound.vertices.size() + 1)
	{
		// error
		throw std::runtime_error("Error!");
	}
	size_t iminus1 = getNextIndex(bound.vertices, i, "backward", bound.distances);
	vertice viminus1 = bound.vertices[iminus1];
	vertice viminus2 = bound.vertices[getNextIndex(bound.vertices, iminus1, "backward", bound.distances)];

	size_t iplus1 = getNextIndex(bound.vertices, i, "forward", bound.distances);
	vertice viplus1 = bound.vertices[iplus1];
	vertice viplus2 = bound.vertices[getNextIndex(bound.vertices, iplus1, "forward", bound.distances)];

	/*
	 * choose the segment in which the object's position is projected in:
	 * calculate the angle between the points (vi, p, vi-1) and (vi, p, vi+1)
	 */
	vertice a;
	a.x = vi.x - position.x;
	a.y = vi.y - position.y;
	vertice b;
	b.x = viminus1.x - position.x;
	b.y = viminus1.y - position.y;
	double beta_iminus1 = std::atan2(a.x*b.y - a.y*b.x, a.x*b.x + a.y*b.y);
	b.x = viplus1.x - position.x;
	b.y = viplus1.y - position.y;
	double beta_iplus1 = std::atan2(a.x*b.y - a.y*b.x, a.x*b.x + a.y*b.y);

	/*
	 * set the flag:
	 * if position is between vertices v(i-1) and v(i) -> flag = 1
	 * if position is between vertices v(i) and v(i+1) -> flag = 0
	 */
	double flag_segement = abs(beta_iminus1) > abs(beta_iplus1);

	/*
	 * construct the points (p) and tangents (t) of the base and the tip of the
	 * segment
	 */
	vertice pBase, tBase, pTip, tTip, ti;
	ti.x = viplus1.x - viminus1.x;
	ti.y = viplus1.y - viminus1.y;
	if (flag_segement) // v(i-1) -> v(i)
	{
		pBase = viminus1;
		tBase.x = vi.x - viminus2.x;
		tBase.y = vi.y - viminus2.y;
		pTip = vi;
		tTip = ti;
	}
	else // ~flag: v(i) -> v(i+1)
	{
		pBase = vi;
		tBase = ti;
		pTip = viplus1;
		tTip.x = viplus2.x - vi.x;
		tTip.y = viplus2.y - vi.y;
	}

	/*
	 * transform the coordinate system:
	 * translate by -pBase, such that pBase == 0
	 * rotate by theta, such that pTip(2) == 0
	 * (note that atan() does not work for all cases)
	 */
	double theta = std::atan2(-(pTip.y - pBase.y), (pTip.x - pBase.x));
	double l = std::pow(std::pow((pTip.x - pBase.x),2) + std::pow((pTip.y - pBase.y),2), 0.5);

	/*
	 * transform the tangent vectors into new coordinate system,
	 * i.e. rotate with theta
	 */
	vertice tBase_Rot, tTip_Rot;
	tBase_Rot.x = tBase.x * cos(theta) -  tBase.y * sin(theta);
	tBase_Rot.y = tBase.x * sin(theta) +  tBase.y * cos(theta);
	tTip_Rot.x = tTip.x * cos(theta) -  tTip.y * sin(theta);
	tTip_Rot.y = tTip.x * sin(theta) +  tTip.y * cos(theta);


	// transform the tangents such that t = [1; m], i.e. slopes m = ty / tx
	double mBase = tBase_Rot.y / tBase_Rot.x;
	double mTip = tTip_Rot.y / tTip_Rot.x;


	// transform the position of the object = [x, y] in new coordinate system:
	// translate
	double x_Trans = position.x - pBase.x;
	double y_Trans = position.y - pBase.y;
	// rotate
	double x = x_Trans * cos(theta) -  y_Trans * sin(theta);
	double y = x_Trans * sin(theta) +  y_Trans * cos(theta);

	// solve equations (1) - (4) for parameter lamda (equation (5))
	double lamda = (x + y * mBase) / (l - y * (mTip - mBase));
	double distance;
	// distance from pLamda to vi (distance is equal in both coordinate systems)
	if (flag_segement) // (vi == pTip)
	{
		// i.e. distance from pLamda to pTip
		distance = (1 - lamda) * l; // previous verified version
	}
	else // (vi == pBase)
	{
		//i.e. distance from pLamda to pBase
		distance = - lamda * l;
	}

	/*
	// projection vector nLamda (connecting pLamda perpendicular with position)
	double pLamda = lamda * pTip + (1-lamda) * pBase; // (equation (2))
	nLamda = position - pLamda;
	*/

	return distance;
}

std::vector <double> calcPathDistances(std::vector <vertice> vertices)
{
	std::vector <double> distances;
	distances.push_back(0);
	//std::cout << "1: " << vertices[1].x << std::endl;
	//std::cout << "0: " << vertices[0].x << std::endl;
	//std::cout << "d: " << std::sqrt(std::pow(vertices[1].x - vertices[0].x, 2.0) + std::pow(vertices[1].y - vertices[0].y, 2.0)) << std::endl;
	for (size_t i = 1; i < vertices.size(); i++)
	{
		// calculate the distance between the current and previous point
		distances.push_back(sqrt(std::pow(vertices[i].x - vertices[i-1].x, 2) +
				std::pow(vertices[i].y - vertices[i-1].y, 2)));
	}
	return distances;
}

std::vector <double> calcCurvature(std::vector <vertice> vertices)
{
	std::vector <double> xVector, yVector, dx, ddx, dy, ddy, curvature;
	//bool xNotZero = true;
	//bool yNotZero = true;

	// seperate x- and y-values
	for (size_t i = 0; i < vertices.size(); i++)
	{
		xVector.push_back(vertices[i].x);
		//std::cout << "temp: " << vertices[i].x << std::endl;
		yVector.push_back(vertices[i].y);
	}


	//calculate the gradients
	dx = gradient(xVector);
	ddx = gradient(dx);
	dy = gradient(yVector);
	ddy = gradient(dy);

	/*
	// check for mot zero values
	for (size_t i = 0; i < dx.size(); i++)
	{
		if (dx[i] == 0)
		{
			xNotZero = false;
		}
		if (dy[i] == 0)
		{
			yNotZero = false;
		}
	}
	*/

	// calculate the signed curvature

	for (size_t i = 0; i < dx.size(); i++)
	{
		if (dx[i] != 0 && dy[i] != 0)
		{
			double temp = (dx[i] * ddy[i] - ddx[i] * dy[i]) / (std::pow(std::pow(dx[i], 2) + std::pow(dy[i], 2), 1.5));
			curvature.push_back(temp);
		}
		else
		{
			curvature.push_back(0.0);
		}
	}
	return curvature;
}

std::vector <double> calcCurvature_new(std::vector <vertice> vertices)
{
	std::vector <double> curvature;

	if(vertices.size() == 0)
	{
		return curvature;
	}
	else if(vertices.size() == 1)
	{
		curvature.push_back(0);
		return curvature;
	}
	else if(vertices.size() == 2)
	{
		curvature.push_back(0);
		curvature.push_back(0);
		return curvature;
	}

	curvature.push_back(0);

	size_t i;
	double m1, m2;
	double x_c, y_c;
	double c;

	for(i = 1; i < vertices.size()-1; i++)
	{
		if ((vertices[i].x - vertices[i-1].x) != 0)
		{
			m1 = (vertices[i].y - vertices[i-1].y)/(vertices[i].x - vertices[i-1].x);
		}
		else
		{
			curvature.push_back(0);
			continue;
		}
		if ((vertices[i+1].x - vertices[i].x) != 0)
		{
			m2 = (vertices[i+1].y - vertices[i].y)/(vertices[i+1].x - vertices[i].x);
		}
		else
		{
			curvature.push_back(0);
			continue;
		}
		if(m2 == m1 || m1 == 0 || m2 == 0)
		{
			curvature.push_back(0);
			continue;
		}


		x_c = (m1*m2*(vertices[i-1].y-vertices[i+1].y)+m2*(vertices[i-1].x+vertices[i].x)-m1*(vertices[i].x+vertices[i+1].x))/(2*(m2-m1));
		y_c = -(1.0/m1)*(x_c-((vertices[i-1].x+vertices[i].x)/2))+(vertices[i-1].y+vertices[i].y)/2;

		c = 1.0/sqrt(std::pow(vertices[i].x-x_c, 2.0) + std::pow(vertices[i].y-y_c, 2.0));
		vertice temp;
		temp.x = x_c;
		temp.y = y_c;

		if (c>0.01)
		{
			std::cout << "vert1: " << vertices[i-1].x << "  " << vertices[i-1].y << std::endl;
			std::cout << "vert2: " << vertices[i].x << "  " << vertices[i].y << std::endl;
			std::cout << "vert3: " << vertices[i+1].x << "  " << vertices[i+1].y << std::endl;
			std::cout << "temp-x: " << temp.x << std::endl;
			std::cout << "temp-y: " << temp.y << std::endl;
			std::cout << "m1: " << m1 << " vert: " << vertices[i].x - vertices[i-1].x << std::endl;
			std::cout << "m2: " << m2 << std::endl;
			std::cout << "c: " << c << std::endl;
		}
		if(isLeft(vertices[i-1], vertices[i+1], vertices[i]))
		{
			c *= -1;
		}
		curvature.push_back(c);
	}

	curvature.push_back(0);


	return curvature;
}

bool isLeft(vertice linesStart, vertice linesEnd, vertice points)
{
	// isLeft if (x1 - x0)*(y2 - y0) - (x2 - x0)*(y1 - y0) > 0
	bool res = (linesEnd.x - linesStart.x) * (points.y - linesStart.y) >
		(points.x - linesStart.x) * (linesEnd.y - linesStart.y);
	return res;
}

double calcVectorIntersectionPoint(vertice a, vertice b, vertice c, vertice d)
{
	// catch division by zero
	if (b.x < std::pow(10, -20))
	{
	    b.x += std::pow(10, -10);
	}
	// analytical solution of vector equation system
	double beta = ( a.x * b.y - a.y * b.x - c.x * b.y + c.y * b.x ) / ( d.x * b.y - d.y * b.x );
	return beta;
}

double calcVectorIntersectionPointAlpha(vertice a, vertice b, vertice c, vertice d)
{
	// catch division by zero
	if (b.x < std::pow(10, -20))
	{
	    b.x += std::pow(10, -10);
	}
	// analytical solution of vector equation system
	double beta = ( a.x * b.y - a.y * b.x - c.x * b.y + c.y * b.x ) / ( d.x * b.y - d.y * b.x );
	//std::cout << "beta1: " << beta << std::endl;
	double alpha = (c.x + beta * d.x - a.x) / b.x;
	return alpha;
}

// implementation based on https://de.mathworks.com/help/matlab/ref/gradient.html
std::vector <double> gradient(std::vector <double> data)
{
	std::vector <double> result;
	double temp;
	for (size_t i = 0; i < data.size(); i++)
	{
		if(i == 0)
		{
			temp = data[1] - data[0];
		}
		else if (i == data.size() - 1)
		{
			temp = data[i] - data[i-1];
		}
		else
		{
			temp = 0.5 * (data[i+1] - data[i-1]);
		}
		result.push_back(temp);
	}
	return result;
}

double wrapToPi(double rad)
{
	// wraps it to the range [-pi, pi].
	if(rad > M_PI)
	{
		return rad - 2*M_PI;
	}
	else if(rad < -M_PI)
	{
		return rad + 2*M_PI;
	}
	return rad;
}

void getLocalConstantRegions(std::vector <double>* radius, std::vector <size_t>* minima, std::vector <region>* regions)
{
	double d_i, d_i_1;
	size_t i;
	region newRegion;
	std::vector <double> diff;

	// check begin of curve
	if ((*radius)[0] < (*radius)[1])
	{

		(*minima).push_back(0);
	}

	// calculate difference along curve
	for(i = 0; i < (*radius).size()-1;i++)
	{
		diff.push_back((*radius)[i+1] - (*radius)[i]);
	}

	i = 0;
	while(i < diff.size()-1)
	{
			d_i = diff[i];
			d_i_1 = diff[i+1];

			// if gradient direction changes => found minimum
			if(d_i < 0 && d_i_1 > 0 && d_i_1 != 0)
			{
				(*minima).push_back(i+1);
			}

			// if value == 0 then a region of constant values starts
			if(d_i == 0)
			{
				newRegion.begin = i;
				while(i < diff.size() && diff[i+1] == 0.0)
				{
					i++;
				}
				newRegion.end = i+1;
				(*regions).push_back(newRegion);
			}
			i++;
	}

	// check end of curve
	if((*radius)[radius->size()-2] > radius->back())
	{
		(*minima).push_back(radius->size()-1);
	}
}

std::vector<vertice> encloseByPolygon(circle* geometry, double num_vertices, double orientation)
{
	std::vector<vertice> vertices;

	// comupte circumradius of polygon determined by inradius and number of
	// corners of polygon
	double central_angle = 2.0*M_PI / num_vertices;
	double circumradius = geometry->getRadius() / (cos(central_angle/2.0));

	vertice temp;

	// generate polygon vertices clockwise
	for (int i = num_vertices-1; i >= 0; i--)
	{
		temp.x = circumradius * (cos(orientation + i*central_angle)) + geometry->getCenter().x;
		temp.y = circumradius * (sin(orientation + i*central_angle)) + geometry->getCenter().y;
		vertices.push_back(temp);
	}

	return vertices;
}

std::vector<vertice> encloseSectionByPolygon(circle* geometry, double num_vertices, double start_orientation, double end_orientation)
{

	std::vector<vertice> vertices;

	// compute angle of sector
	double sector_angle = end_orientation - start_orientation;

	// compute circumradius of polygon portion determined by in radius and number of
	// corner polygons
	double central_angle = sector_angle / (num_vertices-1);
	double circumradius = geometry->getRadius() / cos(central_angle/2.0);

	vertice temp;

	// generate polygon vertices
	// generate polygon vertices clockwise
	for (int i = num_vertices-1; i >= 0; i--)
	{
		temp.x = circumradius * (cos(start_orientation + i*central_angle)) + geometry->getCenter().x;
		temp.y = circumradius * (sin(start_orientation + i*central_angle)) + geometry->getCenter().y;
		vertices.push_back(temp);
	}

	return vertices;
}

std::vector<vertice> scaleVerticesAroundFixpoint(std::vector<vertice> vertices, vertice fixpoint, double factor)
{
	std::vector<vertice> scaledVertices;
	vertice temp;
	for (size_t i = 0; i < vertices.size(); i++)
	{
		temp.x = (vertices[i].x - fixpoint.x) * factor + fixpoint.x;
		temp.y = (vertices[i].y - fixpoint.y) * factor + fixpoint.y;
		scaledVertices.push_back(temp);
	}
	return scaledVertices;
}

double calcPseudoDistance(size_t idx, std::vector <vertice> vertices, vertice pos, vertice* pLampda)
{
	double dist;


	vertice vi = vertices[idx];

	int flag;
	vertice viminus1, viminus2;
	if ((int)idx-1 > 0)
	{
		flag = 1;
		viminus2 = vertices[idx-2];
		viminus1 = vertices[idx-1];
	}
	else if ((int)idx-1 == 0)
	{
		flag = 1;
		viminus2 = vertices[idx-1];
		viminus1 = viminus2;
	}
	else
	{
		flag = 0;
		viminus1 = vi;
	}

	vertice viplus1, viplus2;
	if ((int)idx + 1 < (int)vertices.size())
	{
		viplus2 = vertices[idx+2];
		viplus1 = vertices[idx+1];
	}
	else if ((int)idx+2 == (int)vertices.size())
	{
		viplus2 = vertices[idx+1];
		viplus1 = viplus2;
	}
	else if ((int)idx +1 == (int)vertices.size())
	{
		viplus2 = vertices[idx];
		viplus1 = viplus2;
		if (flag == 0)
		{
			std::cout << "vertices are too short" << std::endl;
		}
	}

	vertice ti{viplus1.x - viminus1.x, viplus1.y - viminus1.y};
	vertice pBase, tBase, pTip, tTip;
	if (flag == 1)
	{
		pBase = viminus1;
		tBase = vertice{vi.x - viminus2.x, vi.y - viminus2.y};
		pTip = vi;
		tTip = ti;
	}
	else
	{
		pBase = vi;
		tBase = ti;
		pTip = viplus1;
		tTip = vertice{viplus2.x - vi.x, viplus2.y - vi.y};
	}

	double theta = std::atan2(-(pTip.y - pBase.y), (pTip.x - pBase.x));


	double l = std::sqrt(std::pow((pTip.x - pBase.x), 2.0) + std::pow((pTip.y - pBase.y), 2.0));


	vertice tBase_Rot, tTip_Rot;
	tBase_Rot.x = tBase.x * cos(theta) - tBase.y * sin(theta);
	tBase_Rot.y = tBase.x * sin(theta) + tBase.y * cos(theta);
	tTip_Rot.x = tTip.x * cos(theta) - tTip.y * sin(theta);
	tTip_Rot.y = tTip.x * sin(theta) + tTip.y * cos(theta);


	double mBase = tBase_Rot.y / tBase_Rot.x;
	double mTip = tTip_Rot.y / tTip_Rot.x;

	double x_Trans = pos.x - pBase.x;
	double y_Trans = pos.y - pBase.y;

	double x = x_Trans * cos(theta) - y_Trans * sin(theta);
	double y = x_Trans * sin(theta) + y_Trans * cos(theta);

	double lambda = (x + y * mBase) / (l - y * (mTip - mBase));

	dist = std::sqrt(std::pow(lambda - x, 2.0) +  std::pow(0 - y, 2));


	std::vector <vertice> temp = rotateAndTranslateVertices({vertice{lambda, 0}}, pBase, -theta);

	(*pLampda) = temp[0];

	return dist;
}

double getMin(std::vector <double> array)
{
	double min = 0;
	if(array.size() > 0)
	{
		min = array[0];
	}
	else
	{
		std::runtime_error("Can not determine minimum of empty list!");
	}
	for (size_t i = 1; i < array.size(); i++)
	{
		if(array[i] < min)
		{
			min = array[i];
		}
	}
	return min;
}

double getMax(std::vector <double> array)
{
	double max = 0;
	if(array.size() > 0)
	{
		max = array[0];
	}
	else
	{
		std::runtime_error("Can not determine maximum of empty list!");
	}
	for (size_t i = 1; i < array.size(); i++)
	{
		if(array[i] > max)
		{
			max = array[i];
		}
	}
	return max;
}

double getMean(std::vector <double> array)
{
	double mean = 0;
	if(array.size() > 0)
	{
		for (size_t i = 0; i < array.size(); i++)
		{
			mean += array[i];
		}
	}
	else
	{
		std::runtime_error("Can not determine mean of empty list!");
	}
	return mean/(double)array.size();
}






