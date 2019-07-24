#include "rectangle.h"

rectangle::rectangle() : shape()
{

}

rectangle::rectangle(const double l, const double w) : shape()
{
	length = l;
	width = w;
}

void rectangle::setLength(const double l)
{
	length = l;
}

void rectangle::setWidth(const double w)
{
	width = w;
}

double rectangle::getLength() const
{
	return length;
}

double rectangle::getWidth() const
{
	return width;
}

