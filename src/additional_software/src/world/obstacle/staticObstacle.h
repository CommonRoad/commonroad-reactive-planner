/*
 * representation for static obstacles.
 */

#ifndef HEADER_STATICOBSTACLE
#define HEADER_STATICOBSTACLE

#include "obstacle.h"


class staticObstacle : public obstacle
{
public:

	/*
	 * constructor
	 */
	staticObstacle() : obstacle()
	{

	}
	/*
	 * destructor
	 */
	~staticObstacle()
	{

	}

	/*
	 * setter functions
	 */


	/*
	* getter functions
	*/

	std::vector <std::vector <vertice> > M1_accelerationBasedOccupancy(timeStruct timeInterval);

	// virtual methods
	void computeOccupancyCore(std::vector <vehicularLanelet>* lanelets, std::vector <lane>* lanes, timeStruct timeInterval);


private:



};




#endif
