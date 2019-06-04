#include "dynamicObstacle.h"
#include "../../geometry/geometricOperations.h"
#include "../../geometry/rectangle.h"
#include "../../prediction/occupancyCalculation.h"
#include <typeinfo>

 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })


void dynamicObstacle::setVelocity(const double velo)
{
	velocity = velo;
}

void dynamicObstacle::setAcceleration(const double acc)
{
	acceleration = acc;
}

void dynamicObstacle::setVmax(const double vmax)
{
	v_max = vmax;
}

void dynamicObstacle::setAmax(const double amax)
{
	a_max = amax;
}

void dynamicObstacle::setSpeedingFactor(const double speedFac)
{
	speedingFactor = speedFac;
}

double dynamicObstacle::getVelocity() const
{
	return velocity;
}

double dynamicObstacle::getAcceleration() const
{
	return acceleration;
}

double dynamicObstacle::getVmax() const
{
	return v_max;
}

double dynamicObstacle::getAmax() const
{
	return a_max;
}

double dynamicObstacle::getSpeedingFactor() const
{
	return speedingFactor;
}

/* virtual methods */

// constraints C1-C5 are only applicable for dynamic obstacles
void dynamicObstacle::manageConstraints()
{

	//if speed of obstacle is higher than its v_max, set v_max to actual speed
	if (this->getVelocity() > this->getVmax())
	{
		std::cout << "The speed of obstacle " << this->getId() << " (v = " << this->getVelocity() <<
				") is higher than its parameterized maximum speed (v_max = " << this->getVmax() << ")." << std::endl;
		this->setVmax(this->getVelocity());
	}

	/* manage constraint C4 (maximum absolute acceleration of dynamic obstacle)
	 * if acceleration of dynamic obstacle is higher than its a_max, set a_max
	 * to actual acceleration
	 */
	if (this->getAcceleration() > this->getAmax())
	{
		std::cout << "The acceleration of obstacle " << this->getId() << " (a = " << this->getAcceleration() << " is higher than its "
				<< "maximum allowed acceleration (a_max = " << this->getAmax() << ")." << std::endl;
		this->setAmax(this->getAcceleration());
	}

}

std::vector <std::vector <vertice> > dynamicObstacle::M1_accelerationBasedOccupancy(timeStruct timeInterval)
{
	std::vector <vertice> p;
	std::vector <vertice> q;
	std::vector <std::vector <vertice> > occM1;
	//compute the occupancy for static and dynamic objects
	if (timeInterval.startingTime == timeInterval.ending)
	{
		if (typeid(this->getShape()) == typeid(rectangle))
		{

			rectangle* rect = dynamic_cast<rectangle*>(this->getShape());
			vertice q1;
			q1.x = 0;
			q1.y = 0;
			q.push_back(q1);
			// vertices p represent the occupancy with vehicle dimensions (Theorem 1)
			p = addObjectDimensions(q, rect->getLength(), rect->getWidth());
			/*
			 * rotate and translate the vertices of the occupancy set in local
			 * coordinates to the object's reference position and rotation
			 */
			vertice position;
			position.x = this->getXpos();
			position.y = this->getYpos();
			occM1.push_back(rotateAndTranslateVertices(p, position, this->getOrientation()));
		}
	}
	else
	{
		for (double t = timeInterval.startingTime; t <= 1.000001*(timeInterval.ending - timeInterval.timeStep); t = t + timeInterval.timeStep)
		{
			/*
			 * calculate the polygon vertices q which represent the convex occupancy
			 * in local coordinates without vehicle dimensions:
			 * (special position and orientation) (Lemma 1)
			 */
			q = accelerationOccupancyLocal(this->getVelocity(), this->getAmax(), t, t+timeInterval.timeStep);

			rectangle* rect = dynamic_cast<rectangle*>(this->getShape());

			// vertices p represent the occupancy with vehicle dimensions (Theorem 1)
			p = addObjectDimensions(q, rect->getLength(), rect->getWidth());
			/*
			 * rotate and translate the vertices of the occupancy set in local
			 * coordinates to the object's reference position and rotatio
			 */

			vertice position;
			position.x = this->getXpos();
			position.y = this->getYpos();

			std::vector <vertice> vertices;
			vertices = rotateAndTranslateVertices(p, position, this->getOrientation());
			occM1.push_back(vertices);
			//k++;
		}
	}
	return occM1;
}
