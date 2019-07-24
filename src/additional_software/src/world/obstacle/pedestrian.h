/*
 * representation for pedestrian obstacles.
 */

#ifndef HEADER_PEDESTRIAN
#define HEADER_PEDESTRIAN

#include "dynamicObstacle.h"


class pedestrian : public dynamicObstacle
{
public:

	/*
	 * constructor
	 */
	pedestrian() : dynamicObstacle()
	{
		num_vert = 24;
		computeOccDynamicBased = true;
		computeOccRuleBased = true;
		bstop = true;
		bcross = true;
		a_max = 0.6;
		a_stop = 0.6;
		v_max = 2; // 7.2 km/h
		dperp = 2; // Pre-defined width of perpendicular crossing
		numLanes = 2;
		laneWidth = 4;
	}

	/*
	 * setter functions
	 */
	void setNumVert(const size_t num);
	void setComputeOccDynamicBased(const bool val);
	void setComputeOccRuleBased(const bool val);
	void setbStop(const bool val);
	void setbCross(const bool val);
	void setPedestrianOrientation(const std::vector <double> orientation_values);
	void setPedestrianVelocity(const std::vector <double> velocity_values);
	void setPRoadBoundary(const vertice vert);
	void setdPerp(const double perp);
	void setLaneWidth(const double width);
	void setNumLanes(const int num);
	void setAstop(const double val);

	/*
	* getter functions
	*/
	size_t getNumVert() const;
	bool getComputeOccDynamicBased() const;
	bool getComputeOccRuleBased() const;
	bool getbStop() const;
	bool getbCross() const;
	std::vector <double> getPedestrianOrientation() const;
	std::vector <double> getPedestrianVelocity() const;
	vertice getPRoadBoundary() const;
	double getdPerp() const;
	int getNumLanes() const;
	double getLaneWidth() const;
	double getAstop() const;


	// virtual functions
	void manageConstraints();
	void computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval);

	/*
	 * function to compute acceleration-constrained
	 * occupancy of pedestrians for the given time interval in the local
	 * coordinate system of the pedestrian (i.e. origin at its initial position
	 * and x-axis aligned with its velocity vector)
	 */
	std::vector <vertice> accOccPedestrian(double t, double dt);

private:
	/* constraints for occupancy prediction*/
	size_t num_vert; // number of vertices for circumscribing polygons
	bool computeOccDynamicBased;
	bool computeOccRuleBased;
	bool bstop;
	bool bcross;
	std::vector <double> pedestrian_orientation;
	std::vector <double> pedestrian_velocity;
	vertice pRoadBoundary;
	double dperp;
	double laneWidth;
	double a_stop;
	int numLanes;
};

#endif
