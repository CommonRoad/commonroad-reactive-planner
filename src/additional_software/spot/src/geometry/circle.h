/*
 * geometry class for circles
 */

#ifndef HEADER_CIRCLE
#define HEADER_CIRCLE

#include "shape.h"
#include "../auxiliaryDefs/structs.h"

class circle : public shape
{
public:

	/*
	 * constructor
	 */
	circle();
	circle(double rad)
	{
		radius = rad;
	}
	circle(double rad, vertice vert)
	{
		radius = rad;
		center = vert;
	}

	/*
	 * destructor
	 */
	~circle()
	{

	}

	/*
	 * setter functions
	 */
	void setRadius(const double rad);
	void setCenter(const double x, const double y);

	/*
	* getter functions
	*/
	double getRadius() const;
	vertice getCenter() const;

private:
	double radius;
	vertice center;
};

#endif
