/*
 * representation for dynamic obstacles.
 */

#ifndef HEADER_DYNAMICOBSTACLE
#define HEADER_DYNAMICOBSTACLE

#include "obstacle.h"

class dynamicObstacle : public obstacle
{
public:

	/*
	 * constructor
	 */
	dynamicObstacle() : obstacle()
	{
		velocity = 0;
		acceleration = 0;
		speedingFactor = 0.2;
	}

	/*
	 * destructor
	 */
	~dynamicObstacle()
	{

	}

	/*
	 * setter functions
	 */
	void setVelocity(const double velo);
	void setAcceleration(const double acc);
	void setVmax(const double vmax);
	void setAmax(const double amax);
	void setSpeedingFactor(const double speedFac);


	/*
	* getter functions
	*/
	double getVelocity() const;
	double getAcceleration() const;
	double getVmax() const;
	double getAmax() const;
	double getSpeedingFactor() const;


	virtual std::vector <std::vector <vertice> > M1_accelerationBasedOccupancy(timeStruct timeInterval);

	/*
	 * verify the restrictions of all constraints by
	 * checking whether their limitation has been violated. If the actual
	 * property is higher than the constraint, it gets increased.
	 * Note that you can modify these rules according to your specification.
	 */
	virtual void manageConstraints();

protected:
	/* defined in subclass */
	double v_max; // maximum velocity of the obstacle in m/s
	double a_max; // maximum absolute acceleration of the obstacle in m/s^2

private:
	double velocity; // scalar velocity of the obstacle in m/s
	double acceleration; // scalar acceleration of the obstacle in m/s^2

	double speedingFactor; // parameter, how many percent above the speed limit the obstacle's





};

#endif
