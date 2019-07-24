/*
 * representation of pedestrian lanelets
 */

#ifndef HEADER_PEDESTRIANLANELET
#define HEADER_PEDESTRIANLANELET


#include "lanelet.h"

class pedestrianLanelet : public lanelet
{
public:

	struct adjacent {
		std::vector<size_t> adj;
		std::string dir;
	};

	/*
	 * constructor
	 */
	pedestrianLanelet() : lanelet()
	{
		prio = 1;
		type = "";
		side = "";
	}

	/*
	 * destructor
	 */
	~pedestrianLanelet()
	{

	}

	/*
	 * setter functions
	 */
	void setPriority(const size_t priority);
	void setType(const std::string typeOfSideWalk);
	void setSide(const std::string sideOfSideWalk);
	void addConnectedLanelet(pedestrianLanelet* connectedLanelet);

	/*
	* getter functions
	*/
	size_t getPriority() const;
	std::string getType() const;
	std::string getSide() const;
	std::vector<pedestrianLanelet*> getConnectedLanelet() const;




private:
	std::string type; // 'crossing' pr 'sideWalk'
	size_t prio; // priority of crossing
	std::string side; // on which side of the lane the sidewalk is located
	std::vector<pedestrianLanelet*> connectedLanelets; // adjacent PedestrianLanelets
};




#endif
