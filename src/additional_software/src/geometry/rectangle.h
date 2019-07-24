/*
 * geometry class for rectangles
 */

#ifndef HEADER_RECTANGLE
#define HEADER_RECTANGLE

#include "shape.h"

class rectangle : public shape
{
public:

	/*
	 * constructor
	 */
	rectangle();
	rectangle(const double l, const double w);

	/*
	 * destructor
	 */
	~rectangle()
	{

	}

	/*
	 * setter functions
	 */
	void setLength(const double l);
	void setWidth(const double w);

	/*
	* getter functions
	*/
	double getLength() const;
	double getWidth() const;

private:
	double length;
	double width;
};

#endif
