/*
 * class to predict the occupancy of other obstacles
 */

#include "../world/lane/lane.h"
#include "../auxiliaryDefs/structs.h"

#ifndef HEADER_OCCUPANCY
#define HEADER_OCCUPANCY

struct occTypes
{
	std::vector <lane*> forLane;
	std::vector <vertice> vertices;
	timeStruct timeInterval;
};

class occupancy
{
public:
	/*
	 * constructor
	 */
	occupancy()
	{

	}

	/*
	 * destructor
	 */
	~occupancy()
	{
	}
	/*
	 * setter functions
	 */

	/*
	* getter functions
	*/
	std::vector <std::vector <occTypes> >* getOccMatrix();

	// general properties
	//bool COMPUTE_OCC_M1;
	//bool COMPUTE_OCC_M2;

	// obstacles property
	//double INITIAL_SPEEDING_FACTOR;

	// lane properties
	//bool EXTEND_LANES_BACKWARD;
	//bool EXTEND_LANES_FORWARD;


private:
	std::vector <std::vector <occTypes> > occupancyMatrix;
};




#endif
