/*
 * representation for static obstacles.
 */

#ifndef HEADER_VEHICLE
#define HEADER_VEHICLE

#include "dynamicObstacle.h"

#include <limits>

class vehicle : public dynamicObstacle
{
public:

	/*
	 * constructor
	 */
	vehicle() : dynamicObstacle()
	{
		v_s = 5;
		v_max = 83.3333;
		a_max = 8;
		power_max = std::numeric_limits<double>::infinity();
		constraint3 = true;
		constraint5 = true;
		compute_occ_m1 = true;
		compute_occ_m2 = true;
		speedingFactor = 0.2;
	}

	/*
	 * setter functions
	 */
	void setVs(const double velo);
	void setPowerMax(const double power);
	void setConstraint3(const bool c3);
	void setConstraint5(const bool c5);
	void setSpeedingFactor(const double factor);

	/*
	* getter functions
	*/
	double getVs() const;
	double getPowerMax() const;
	bool getConstraint3() const;
	bool getConstraint5() const;
	bool getCompute_Occ_M1() const;
	bool getCompute_Occ_M2() const;
	double getSpeedingFactor() const;


	void manageConstraints();

	// virtual functions
	void computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval);

	std::vector <std::vector <vertice> > M2_laneFollowingOccupancy(lane* curLane, timeStruct timeInterval);

private:
	double v_s; // switching velocity in m/s (modeling limited engine power)
	double power_max; // maximum power for accelerating

	bool constraint3; // backward driving is not allowed by default
	bool constraint5; // changing and crossing lanes is forbidden unless allowed by traffic regulations

	bool compute_occ_m1;
	bool compute_occ_m2;

	double speedingFactor;

};

#endif
