#ifndef HEADER_OBSTACLE
#define HEADER_OBSTACLE

#include "../lane/lane.h"
#include "../../geometry/shape.h"
#include "../../prediction/occupancy.h"

class obstacle
{
public:

	/*
	 * constructor
	 */
	obstacle();

	/*
	 *
	 */
	 virtual ~obstacle();

	/*
	 * setter functions
	 */
	void setId(const size_t num);
	void setPosition(const double x, const double y);
	void setOrientation(const double value);
	void addInLane(lane* l);
	void setShape(shape* gShape);

	/*
	* getter functions
	*/
	size_t getId() const;
	double getXpos() const;
	double getYpos() const;
	double getOrientation() const;
	std::vector <lane*> getInLane() const;
	shape* getShape() const;
	occupancy* getOccupancy() const;

	// finds all lanes in which the obstacle is positioned in
	void updateInLane(std::vector <lane>* lanes);

	/*
	 * predicts the occupancy of the obstacle on the
	 * given map for the given time interval
	 */
	virtual void computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval);

	/*
	 * M_1: compute the acceleration-based occupancy for each time interval
	 * (maximum possible occupancy with maximum longitudinally and laterally
	 * acceleration, i.e. "occupancy towards the road boundaries")
	 */
	virtual std::vector <std::vector <vertice> > M1_accelerationBasedOccupancy(timeStruct timeInterval);

private:
	size_t id; // unique id
	double xPosition; // x-coordinate of the obstacle
	double yPosition; // y-coordinate of the obstacle
	double orientation; // orientation of the obstacle
	shape* geoShape; // geometric shape of the object
	std::vector<lane*> inLane; //lane, in which the obstacle is located in
	mutable occupancy occupy; // occupancy
};



#endif
