/*
 * representation of road lanelets
 */

#ifndef HEADER_VEHICULARLANELET
#define HEADER_VEHICULARLANELET


#include "lanelet.h"

class vehicularLanelet : public lanelet
{
public:

	struct adjacent {
		std::vector<size_t> adj;
		std::string dir;
	};

	/*
	 * constructor
	 */
	vehicularLanelet() : lanelet()
	{
		speedLimit = 99999;
		adjacentLeft.dir = "";
		adjacentRight.dir = "";
	}

	/*
	 * destructor
	 */
	~vehicularLanelet()
	{

	}

	/*
	 * setter functions
	 */
	void setSpeedLimit(const double lim);
	void addPredecessor(vehicularLanelet* pre);
	void addSuccessor(vehicularLanelet* suc);
	void setLeftAdjacent(size_t left, std::string dir);
	void setRightAdjacent(size_t right, std::string dir);

	/*
	* getter functions
	*/
	double getSpeedLimit() const;
	std::vector <vehicularLanelet*> getPredecessors() const;
	std::vector <vehicularLanelet*> getSuccessors() const;
	std::vector<size_t> getAdjacentRight() const;
	std::vector<size_t> getAdjacentLeft() const;
	std::string getAdjacentRightDir() const;
	std::string getAdjacentLeftDir() const;

private:
	double speedLimit; // official speed limit of the lanelet
	std::vector <vehicularLanelet*> predecessorLanelets; // previous lanelets
	std::vector <vehicularLanelet*> successorLanelets; // longitudinally adjacent lanelets
	adjacent adjacentLeft; // left adjacent lanelet with driving tag
	adjacent adjacentRight; // right adjacent lanelet with driving tag
};




#endif
